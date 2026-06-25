#!/usr/bin/env python3
"""
Walker S2 关节测试脚本

继承 RobotController，提供指定关节的状态查询和位置控制命令行接口。
适用于单关节调试、限位验证、零点标定等场景。

支持身体关节（RobotCommand 通路）和手指关节（JointCommand 通路）。

【使用示例】

    # 查询指定身体关节当前状态
    python3 joint_test.py --joints R_elbow_yaw_joint L_shoulder_pitch_joint

    # 移动指定身体关节到目标角度
    python3 joint_test.py --move R_elbow_yaw_joint=0.5 --duration 2.0

    # 多关节同时移动
    python3 joint_test.py --move R_elbow_yaw_joint=0.5 L_shoulder_pitch_joint=-0.3

    # 相对当前位置偏移
    python3 joint_test.py --shift R_elbow_yaw_joint=+0.1

    # 持续监控指定关节（10Hz 刷新）
    python3 joint_test.py --monitor --joints head_pitch_joint head_yaw_joint

    # 交互模式（Python REPL）
    python3 joint_test.py --interactive --joints R_elbow_yaw_joint

    # ---- 手指关节 ----

    # 查询左手手指状态
    python3 joint_test.py --hand left --print

    # 移动左拇指
    python3 joint_test.py --hand left --hand-move thumb_swing=0.5

    # 右食指偏移
    python3 joint_test.py --hand right --hand-shift index_mcp=+0.2

    # 设置整手姿态（7个角度值，按 V4_HAND_*_JOINTS 顺序）
    python3 joint_test.py --hand left --hand-pose 0.5 0.3 0.1 0.8 0.8 0.8 0.8

    # 预设姿态：张开 / 握拳
    python3 joint_test.py --hand both --hand-open
    python3 joint_test.py --hand left --hand-close

    # 双手周期波形运动
    python3 joint_test.py --hand both --hand-wave

    # 监控手指关节
    python3 joint_test.py --monitor --joints left_thumb_swing left_index_mcp
"""

import argparse
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_control import (
    BODY_JOINT_LIMITS,
    BODY_JOINT_NAMES,
    DEFAULT_LOCK_JOINTS,
    GRIP_ACCELERATION_LIMIT,
    GRIP_FORCE_LIMIT,
    GRIP_POSITION_LIMIT,
    GRIP_VELOCITY_LIMIT,
    V4_HAND_CLOSE_POSE,
    V4_HAND_JOINT_LIMITS,
    V4_HAND_JOINT_MAP,
    V4_HAND_LEFT_JOINTS,
    V4_HAND_OPEN_POSE,
    V4_HAND_RIGHT_JOINTS,
    RobotController,
)


# 所有已知的手指关节全名集合（用于快速判断）
_ALL_HAND_JOINT_NAMES = set(V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS)


def _is_hand_joint(name):
    """判断关节名是否为手指关节"""
    return name in _ALL_HAND_JOINT_NAMES


def _infer_hand_side(name):
    """从关节全名推断手别。

    Returns:
        "left" / "right" / None
    """
    if name.startswith("left_"):
        return "left"
    if name.startswith("right_"):
        return "right"
    return None


class JointTestController(RobotController):
    """指定关节的状态查询与位置控制控制器。

    在 RobotController 基础上新增：
        1. get_joint_position(name)  — 查询单关节当前位置
        2. get_joints_positions(names) — 批量查询
        3. print_joint_states(names)  — 格式化打印指定关节状态
        4. move_joint(name, target_rad, duration) — 控制单关节到目标角度
        5. shift_joint(name, delta_rad, duration) — 控制单关节相对偏移
        6. monitor_joints(names, hz, duration) — 持续监控指定关节
        7. print_hand_states(sides)  — 打印手指关节状态
        8. move_hand_joint(side, name, target, duration) — 移动手指关节
        9. shift_hand_joint(side, name, delta, duration) — 手指关节偏移
       10. monitor_hand_joints(side, names, hz, duration) — 监控手指关节
    """

    def get_joint_position(self, joint_name):
        """获取指定身体关节的当前位置（rad）。

        Args:
            joint_name: 关节名
        Returns:
            float: 当前位置，None 表示无数据或关节名无效
        """
        try:
            idx = self.joint_index(joint_name)
        except ValueError:
            self.get_logger().error(f"Unknown joint: {joint_name}")
            return None
        pos = self.get_current_position()
        if pos is None:
            return None
        return float(pos[idx])

    def get_joints_positions(self, joint_names):
        """批量获取指定身体关节的当前位置。

        Args:
            joint_names: 关节名列表
        Returns:
            dict: {joint_name: position}，无数据的关节值为 None
        """
        pos = self.get_current_position()
        result = {}
        for name in joint_names:
            try:
                idx = self.joint_index(name)
                result[name] = float(pos[idx]) if pos is not None else None
            except ValueError:
                result[name] = None
        return result

    def print_joint_states(self, joint_names=None):
        """格式化打印指定身体关节的当前状态。

        Args:
            joint_names: 要打印的关节名列表，None 表示全部
        """
        names = joint_names or self.all_joints
        pos = self.get_current_position()
        if pos is None:
            print("No current position available")
            return

        print(f"\n{'关节名':<32s} {'位置(rad)':>10s} {'位置(°)':>9s} {'限位范围':>18s} {'状态':>8s}")
        print("-" * 82)

        for name in names:
            try:
                idx = self.joint_index(name)
            except ValueError:
                print(f"  {name:<32s} UNKNOWN")
                continue

            val = pos[idx]
            deg = np.degrees(val)

            locked = "LOCKED" if name in self.lock_joints else ""

            if name in BODY_JOINT_LIMITS:
                lo, hi = BODY_JOINT_LIMITS[name]
                range_str = f"[{lo:.2f}, {hi:.2f}]"
                if val < lo:
                    status = "⚠️BELOW"
                elif val > hi:
                    status = "⚠️ABOVE"
                else:
                    status = "OK"
            else:
                range_str = "N/A"
                status = ""

            print(
                f"  {name:<32s} {val:>+10.4f} {deg:>+9.2f} "
                f"{range_str:>18s} {status:>8s} {locked}"
            )

    def move_joint(self, joint_name, target_rad, duration_sec=2.0, wait=True):
        """控制单个身体关节移动到目标角度，其他关节保持当前位置。

        Args:
            joint_name: 关节名
            target_rad: 目标角度（rad）
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        return self.move_to_pose(
            {joint_name: target_rad},
            duration_sec=duration_sec,
            wait=wait,
            unlock_required_joints=True,
        )

    def shift_joint(self, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """控制单个身体关节相对当前位置偏移。

        Args:
            joint_name: 关节名
            delta_rad: 偏移量（rad），正=正向，负=负向
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        current = self.get_joint_position(joint_name)
        if current is None:
            self.get_logger().error(
                f"Cannot shift {joint_name}: no current position"
            )
            return False
        target = current + delta_rad

        # 显示限位信息
        if joint_name in BODY_JOINT_LIMITS:
            lo, hi = BODY_JOINT_LIMITS[joint_name]
            if target < lo or target > hi:
                clamped = max(lo, min(hi, target))
                self.get_logger().warning(
                    f"{joint_name}: target {target:.4f} rad exceeds limit "
                    f"[{lo}, {hi}], will be clamped to {clamped:.4f}"
                )

        self.get_logger().info(
            f"Shift {joint_name}: {current:.4f} → {target:.4f} rad "
            f"(Δ={delta_rad:+.4f} rad, {np.degrees(delta_rad):+.2f}°)"
        )

        return self.move_joint(joint_name, target, duration_sec=duration_sec, wait=wait)

    def monitor_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控指定身体关节的位置变化。

        Args:
            joint_names: 要监控的关节名列表
            hz: 刷新频率（Hz）
            duration_sec: 监控时长（秒），None 表示持续到 Ctrl+C
        """
        interval = 1.0 / hz
        start_time = time.time()

        # 表头
        header = f"{'time':>6s}"
        for name in joint_names:
            short = name.replace("_joint", "").replace("_", " ")
            header += f"  {short:>12s}"
        print(header)
        print("-" * len(header))

        try:
            while True:
                if duration_sec is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= duration_sec:
                        break

                pos = self.get_current_position()
                if pos is not None:
                    elapsed = time.time() - start_time
                    line = f"{elapsed:6.1f}"
                    for name in joint_names:
                        try:
                            idx = self.joint_index(name)
                            line += f"  {pos[idx]:>+12.4f}"
                        except ValueError:
                            line += f"  {'N/A':>12s}"
                    print(line, flush=True)

                time.sleep(interval)

        except KeyboardInterrupt:
            pass

        print("\n监控结束")

    # ---- 手指关节方法 ----

    def print_hand_states(self, sides=None):
        """格式化打印手指关节的当前状态。

        Args:
            sides: 要打印的手别列表，如 ["left", "right"]；None 表示双手
        """
        sides = sides or ["left", "right"]

        print(f"\n{'关节名':<24s} {'位置(rad)':>10s} {'位置(°)':>9s} {'限位范围':>18s} {'状态':>8s}")
        print("-" * 75)

        for side in sides:
            joint_names = V4_HAND_JOINT_MAP[side]
            pos = self.get_hand_position(side)

            for name in joint_names:
                short = name.removeprefix("left_").removeprefix("right_")

                if pos is not None:
                    idx = joint_names.index(name)
                    val = pos[idx]
                    deg = np.degrees(val)
                else:
                    val = None
                    deg = None

                if short in V4_HAND_JOINT_LIMITS:
                    lo, hi = V4_HAND_JOINT_LIMITS[short]
                    range_str = f"[{lo:.2f}, {hi:.2f}]"
                    if val is not None:
                        if val < lo:
                            status = "⚠️BELOW"
                        elif val > hi:
                            status = "⚠️ABOVE"
                        else:
                            status = "OK"
                    else:
                        status = "NO DATA"
                else:
                    range_str = "N/A"
                    status = ""

                val_str = f"{val:>+10.4f}" if val is not None else f"{'N/A':>10s}"
                deg_str = f"{deg:>+9.2f}" if deg is not None else f"{'N/A':>9s}"

                print(
                    f"  {name:<24s} {val_str} {deg_str} "
                    f"{range_str:>18s} {status:>8s}"
                )

    def move_hand_joint(self, side, joint_name, target_rad, duration_sec=2.0, wait=True):
        """移动单个手指关节到目标角度，其他手指关节保持当前位置。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
            target_rad: 目标角度（rad）
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        return self.move_hand(
            side, {joint_name: target_rad},
            duration_sec=duration_sec, wait=wait,
        )

    def shift_hand_joint(self, side, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """控制手指关节相对当前位置偏移。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
            delta_rad: 偏移量（rad）
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        return self.shift_hand(
            side, joint_name, delta_rad,
            duration_sec=duration_sec, wait=wait,
        )

    def print_grip_states(self, sides=None):
        """格式化打印夹爪当前状态。"""
        sides = sides or ["left", "right"]

        print(f"\n{'夹爪':<8s} {'init':>6s} {'state':>6s} {'error':>6s} {'homed':>6s} {'位置(m)':>10s} {'速度(m/s)':>10s} {'电流(A)':>10s}")
        print("-" * 78)

        for side in sides:
            state = self.get_grip_state(side)
            if state is None:
                print(f"  {side:<6s} {'NO DATA':>68s}")
                continue
            print(
                f"  {side:<6s} {state.init_state:>6d} {state.grip_state:>6d} "
                f"{state.error_code:>6d} {state.homed:>6d} "
                f"{state.pos:>+10.4f} {state.vel:>+10.4f} {state.cur:>+10.4f}"
            )

    def monitor_grips(self, sides=None, hz=10, duration_sec=None):
        """持续监控夹爪状态。"""
        sides = sides or ["left", "right"]
        interval = 1.0 / hz
        start_time = time.time()

        header = f"{'time':>6s}"
        for side in sides:
            header += f"  {side + '_pos':>10s} {side + '_vel':>10s} {side + '_state':>10s}"
        print(header)
        print("-" * len(header))

        try:
            while True:
                if duration_sec is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= duration_sec:
                        break

                elapsed = time.time() - start_time
                line = f"{elapsed:6.1f}"
                for side in sides:
                    state = self.get_grip_state(side)
                    if state is None:
                        line += f"  {'N/A':>10s} {'N/A':>10s} {'N/A':>10s}"
                    else:
                        line += f"  {state.pos:>+10.4f} {state.vel:>+10.4f} {state.grip_state:>10d}"
                print(line, flush=True)
                time.sleep(interval)

        except KeyboardInterrupt:
            pass

        print("\n监控结束")

    def monitor_hand_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控手指关节的位置变化。

        Args:
            joint_names: 手指关节全名列表（如 ["left_thumb_swing", "left_index_mcp"]）
            hz: 刷新频率（Hz）
            duration_sec: 监控时长（秒），None 表示持续到 Ctrl+C
        """
        # 按手别分组
        by_side = {}
        for name in joint_names:
            side = _infer_hand_side(name)
            if side is None:
                print(f"⚠️ 无法判断手别: {name}，跳过")
                continue
            by_side.setdefault(side, []).append(name)

        interval = 1.0 / hz
        start_time = time.time()

        # 表头
        header = f"{'time':>6s}"
        for name in joint_names:
            short = name.removeprefix("left_").removeprefix("right_")
            header += f"  {short:>12s}"
        print(header)
        print("-" * len(header))

        try:
            while True:
                if duration_sec is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= duration_sec:
                        break

                elapsed = time.time() - start_time
                line = f"{elapsed:6.1f}"

                for name in joint_names:
                    side = _infer_hand_side(name)
                    if side is None:
                        line += f"  {'N/A':>12s}"
                        continue
                    pos = self.get_hand_position(side)
                    if pos is not None:
                        try:
                            idx = V4_HAND_JOINT_MAP[side].index(name)
                            line += f"  {pos[idx]:>+12.4f}"
                        except ValueError:
                            line += f"  {'N/A':>12s}"
                    else:
                        line += f"  {'N/A':>12s}"

                print(line, flush=True)
                time.sleep(interval)

        except KeyboardInterrupt:
            pass

        print("\n监控结束")


# ============================================================================
# 命令行解析
# ============================================================================


def parse_move_arg(move_list):
    """解析 --move / --shift / --hand-move / --hand-shift 参数，格式：JointName=angle"""
    result = {}
    for item in move_list:
        if "=" not in item:
            print(f"✗ 格式错误: '{item}'，应为 JointName=angle（如 R_elbow_yaw_joint=0.5）")
            sys.exit(1)
        name, val_str = item.split("=", 1)
        name = name.strip()
        try:
            val = float(val_str.strip())
        except ValueError:
            print(f"✗ 数值错误: '{val_str}' 不是有效浮点数")
            sys.exit(1)
        result[name] = val
    return result


def resolve_hand_sides(hand_arg):
    """将 --hand 参数值转为手别列表。"""
    if hand_arg == "both":
        return ["left", "right"]
    if hand_arg in ("left", "right"):
        return [hand_arg]
    print(f"✗ 无效的 --hand 值: '{hand_arg}'，应为 left/right/both")
    sys.exit(1)


def resolve_grip_sides(grip_arg):
    """将 --grip 参数值转为夹爪侧列表。"""
    if grip_arg == "both":
        return ["left", "right"]
    if grip_arg in ("left", "right"):
        return [grip_arg]
    print(f"✗ 无效的 --grip 值: '{grip_arg}'，应为 left/right/both")
    sys.exit(1)


def resolve_hand_pose_arg(pose_list):
    """将 --hand-pose 参数（7个浮点数）转为角度列表。"""
    if len(pose_list) != 7:
        print(f"✗ --hand-pose 需要 7 个角度值（V4 手 = 7 关节），当前 {len(pose_list)} 个")
        sys.exit(1)
    try:
        return [float(v) for v in pose_list]
    except ValueError:
        print("✗ --hand-pose 的值必须为浮点数")
        sys.exit(1)


def build_hand_pose_dict_from_full(pose_values, side):
    """从 7 个角度值和手别构建 pose_dict。"""
    joint_names = V4_HAND_JOINT_MAP[side]
    return {name: val for name, val in zip(joint_names, pose_values)}


def main(args=None):
    # ---- 构建可用关节名列表（身体 + 手指） ----
    all_known_joints = list(BODY_JOINT_NAMES) + list(V4_HAND_LEFT_JOINTS) + list(V4_HAND_RIGHT_JOINTS)

    parser = argparse.ArgumentParser(
        description="Walker S2 关节测试脚本 — 身体关节与手指关节的状态查询与位置控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
使用示例：
  # 查询指定身体关节状态
  python3 joint_test.py --joints R_elbow_yaw_joint L_shoulder_pitch_joint

  # 移动指定身体关节到目标角度
  python3 joint_test.py --move R_elbow_yaw_joint=0.5

  # 多关节同时移动
  python3 joint_test.py --move R_elbow_yaw_joint=0.5 L_shoulder_pitch_joint=-0.3

  # 相对当前位置偏移
  python3 joint_test.py --shift R_elbow_yaw_joint=+0.1

  # 持续监控
  python3 joint_test.py --monitor --joints head_pitch_joint head_yaw_joint

  # 交互模式
  python3 joint_test.py --interactive --joints R_elbow_yaw_joint

  # ---- 手指关节 ----

  # 查询左手手指状态
  python3 joint_test.py --hand left --print

  # 移动左拇指
  python3 joint_test.py --hand left --hand-move thumb_swing=0.5

  # 右食指偏移
  python3 joint_test.py --hand right --hand-shift index_mcp=+0.2

  # 设置整手姿态（7个值）
  python3 joint_test.py --hand left --hand-pose 0.5 0.3 0.1 0.8 0.8 0.8 0.8

  # 预设姿态：张开 / 握拳
  python3 joint_test.py --hand both --hand-open
  python3 joint_test.py --hand left --hand-close

  # 双手周期波形
  python3 joint_test.py --hand both --hand-wave

  # 监控手指关节
  python3 joint_test.py --monitor --joints left_thumb_swing left_index_mcp

可用身体关节名：
  """ + "\n  ".join(BODY_JOINT_NAMES) + """

可用手指关节名（左手）：
  """ + "\n  ".join(V4_HAND_LEFT_JOINTS) + """

可用手指关节名（右手）：
  """ + "\n  ".join(V4_HAND_RIGHT_JOINTS),
    )

    # ---- 关节选择 ----
    parser.add_argument(
        "--joints", nargs="+", default=None,
        metavar="JOINT",
        help="指定关节名（空格分隔），用于 --print / --monitor；"
             "支持身体关节和手指关节全名；默认全部身体关节",
    )

    # ---- 手部选择 ----
    parser.add_argument(
        "--hand", default=None,
        choices=["left", "right", "both"],
        help="指定操作哪只手（配合 --hand-move / --hand-shift / --hand-pose 等使用）",
    )

    # ---- 夹爪选择 ----
    parser.add_argument(
        "--grip", default=None,
        choices=["left", "right", "both"],
        help="指定操作哪侧夹爪（配合 --grip-* 使用）",
    )

    # ---- 身体关节操作模式 ----
    parser.add_argument(
        "--print", action="store_true",
        help="打印指定关节的当前状态",
    )
    parser.add_argument(
        "--move", nargs="+", default=None,
        metavar="JOINT=ANGLE",
        help="移动身体关节到目标角度（rad），格式：JointName=angle",
    )
    parser.add_argument(
        "--shift", nargs="+", default=None,
        metavar="JOINT=DELTA",
        help="身体关节相对当前位置偏移（rad），格式：JointName=delta",
    )
    parser.add_argument(
        "--monitor", action="store_true",
        help="持续监控指定关节的位置变化",
    )
    parser.add_argument(
        "--interactive", action="store_true",
        help="交互模式：启动后保持运行，可在 Python REPL 中调用 API",
    )

    # ---- 手指关节操作模式 ----
    parser.add_argument(
        "--hand-move", nargs="+", default=None,
        metavar="JOINT=ANGLE",
        help="移动手指关节到目标角度（rad），格式：JointName=angle；"
             "JointName 可用短名（如 thumb_swing），需配合 --hand 指定手别",
    )
    parser.add_argument(
        "--hand-shift", nargs="+", default=None,
        metavar="JOINT=DELTA",
        help="手指关节相对偏移（rad），格式同 --hand-move",
    )
    parser.add_argument(
        "--hand-pose", nargs=7, default=None,
        metavar="ANGLE",
        help="设置整手姿态（7 个角度值 rad，按 V4_HAND_*_JOINTS 顺序）",
    )
    parser.add_argument(
        "--hand-open", action="store_true",
        help="手指张开（所有关节归零）",
    )
    parser.add_argument(
        "--hand-close", action="store_true",
        help="手指握拳（所有关节到限位上限）",
    )
    parser.add_argument(
        "--hand-wave", action="store_true",
        help="手部周期波形运动（调用 hand_periodic_motion）",
    )

    # ---- 夹爪操作模式 ----
    parser.add_argument(
        "--grip-print", action="store_true",
        help="打印夹爪状态（/ecat/{left,right}_grip/state）",
    )
    parser.add_argument(
        "--grip-move", type=float, default=None,
        metavar="POS",
        help="夹爪移动到目标位置，范围 [0, 0.05] m",
    )
    parser.add_argument(
        "--grip-force", type=float, default=41.0,
        help="夹爪目标力，范围 [41, 100] N，默认 41",
    )
    parser.add_argument(
        "--grip-vel", type=float, default=0.005,
        help="夹爪目标速度，范围 [0, 0.01] m/s，默认 0.005",
    )
    parser.add_argument(
        "--grip-acc", type=float, default=0.0,
        help="夹爪目标加速度，范围 [0, 3] m/s^2，默认 0；写入 GripCmd.cur 字段",
    )
    parser.add_argument(
        "--grip-mode", type=int, default=0,
        help="夹爪控制模式，默认 0；推压模式可用 10",
    )
    parser.add_argument(
        "--grip-repeat", type=float, default=0.5,
        help="夹爪命令连续发布时长（秒），默认 0.5；0 表示只发布一次",
    )
    parser.add_argument(
        "--grip-home", action="store_true",
        help="夹爪回零（homing=1）",
    )
    parser.add_argument(
        "--grip-stop", action="store_true",
        help="夹爪停止（stop=1）",
    )
    parser.add_argument(
        "--grip-monitor", action="store_true",
        help="持续监控夹爪状态",
    )

    # ---- 通用参数 ----
    parser.add_argument(
        "--duration", type=float, default=2.0,
        help="运动持续时间（秒），用于 --move / --shift / --hand-move 等，默认 2.0",
    )
    parser.add_argument(
        "--monitor-hz", type=float, default=10.0,
        help="监控刷新频率（Hz），用于 --monitor，默认 10",
    )
    parser.add_argument(
        "--monitor-time", type=float, default=None,
        help="监控时长（秒），用于 --monitor，默认持续到 Ctrl+C",
    )
    parser.add_argument(
        "--no-lock", action="store_true",
        help="不锁定任何身体关节（默认锁定 head/waist）",
    )
    parser.add_argument(
        "--no-safety", action="store_true",
        help="禁用安全速度检查",
    )
    parser.add_argument(
        "--no-limits", action="store_true",
        help="禁用关节限位裁剪",
    )

    cli_args, ros_args = parser.parse_known_args()

    # ---- 判断操作模式 ----
    has_body_action = any([
        cli_args.print,
        cli_args.move,
        cli_args.shift,
        cli_args.monitor,
        cli_args.interactive,
    ])
    has_hand_action = any([
        cli_args.hand_move,
        cli_args.hand_shift,
        cli_args.hand_pose,
        cli_args.hand_open,
        cli_args.hand_close,
        cli_args.hand_wave,
    ])
    has_grip_action = any([
        cli_args.grip_print,
        cli_args.grip_move is not None,
        cli_args.grip_home,
        cli_args.grip_stop,
        cli_args.grip_monitor,
    ])

    # 无操作模式时默认打印
    if not has_body_action and not has_hand_action and not has_grip_action:
        cli_args.print = True

    # ---- 手指操作需要 --hand 参数 ----
    if has_hand_action and cli_args.hand is None:
        # 尝试从关节名推断手别
        cli_args.hand = "both"

    hand_sides = resolve_hand_sides(cli_args.hand) if (has_hand_action or cli_args.hand) else []

    if has_grip_action and cli_args.grip is None:
        cli_args.grip = "both"
    grip_sides = resolve_grip_sides(cli_args.grip) if (has_grip_action or cli_args.grip) else []

    # ---- 解析关节名列表 ----
    # 将 --joints 分为身体关节和手指关节
    specified_joints = cli_args.joints or []
    body_joint_names = []
    hand_joint_names = []

    if specified_joints:
        for name in specified_joints:
            if name in BODY_JOINT_NAMES:
                body_joint_names.append(name)
            elif _is_hand_joint(name):
                hand_joint_names.append(name)
            else:
                print(f"✗ 未知关节名: '{name}'")
                print(f"  可用身体关节: {', '.join(BODY_JOINT_NAMES)}")
                print(f"  可用手指关节: {', '.join(V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS)}")
                sys.exit(1)
    else:
        body_joint_names = list(BODY_JOINT_NAMES)

    # ---- 初始化 ROS2 ----
    rclpy.init(args=ros_args)

    lock_joints = None if cli_args.no_lock else DEFAULT_LOCK_JOINTS
    controller = JointTestController(
        lock_joints=lock_joints,
        enable_safety_check=not cli_args.no_safety,
        enable_limit_check=not cli_args.no_limits,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # 等待身体状态（仅夹爪操作不强制等待身体状态）
        if not has_grip_action or has_body_action or has_hand_action:
            if not controller.wait_for_state(timeout=5.0):
                print("[FATAL] 未收到机器人状态，请检查：")
                print("  1. 运控是否启动 (rosa run t800_mc_server start_mc_client)")
                print("  2. SDK 控制器是否切换 (switch_controller config_mc_walker_s2_v1_sps)")
                print("  3. DDS 中间件是否为 CycloneDDS")
                return

        # ================================================================
        # 身体关节操作
        # ================================================================

        if cli_args.grip_print:
            controller.wait_for_grip_state(timeout=2.0)
            controller.print_grip_states(grip_sides)

        elif cli_args.grip_move is not None:
            print("\n=== 移动夹爪 ===")
            print(f"  位置范围: [{GRIP_POSITION_LIMIT[0]:.3f}, {GRIP_POSITION_LIMIT[1]:.3f}] m")
            print(f"  力矩范围: [{GRIP_FORCE_LIMIT[0]:.0f}, {GRIP_FORCE_LIMIT[1]:.0f}] N")
            print(f"  速度范围: [{GRIP_VELOCITY_LIMIT[0]:.3f}, {GRIP_VELOCITY_LIMIT[1]:.3f}] m/s")
            print(f"  加速度范围: [{GRIP_ACCELERATION_LIMIT[0]:.1f}, {GRIP_ACCELERATION_LIMIT[1]:.1f}] m/s^2")
            for side in grip_sides:
                print(
                    f"  {side} grip → pos={cli_args.grip_move:.4f}m "
                    f"force={cli_args.grip_force:.1f}N vel={cli_args.grip_vel:.4f}m/s "
                    f"acc={cli_args.grip_acc:.2f}m/s^2 mode={cli_args.grip_mode}"
                )
            input("\n按回车发送夹爪命令（Ctrl+C 取消）...")

            all_ok = True
            for side in grip_sides:
                ok = controller.send_grip_command(
                    side,
                    pos=cli_args.grip_move,
                    force=cli_args.grip_force,
                    vel=cli_args.grip_vel,
                    acc=cli_args.grip_acc,
                    mode=cli_args.grip_mode,
                    repeat_sec=cli_args.grip_repeat,
                )
                if not ok:
                    all_ok = False
            print("✓ 夹爪命令已发送" if all_ok else "✗ 部分夹爪命令发送失败")

        elif cli_args.grip_home:
            print("\n=== 夹爪回零 ===")
            for side in grip_sides:
                print(f"  {side} grip homing=1")
            input("\n按回车发送回零命令（Ctrl+C 取消）...")
            for side in grip_sides:
                controller.home_grip(side)
            print("✓ 回零命令已发送")

        elif cli_args.grip_stop:
            print("\n=== 停止夹爪 ===")
            for side in grip_sides:
                print(f"  {side} grip stop=1")
            input("\n按回车发送停止命令（Ctrl+C 取消）...")
            for side in grip_sides:
                controller.stop_grip(side)
            print("✓ 停止命令已发送")

        elif cli_args.grip_monitor:
            print(f"\n=== 监控夹爪 ({cli_args.monitor_hz}Hz) ===")
            if cli_args.monitor_time:
                print(f"  持续 {cli_args.monitor_time:.1f} 秒")
            else:
                print("  按 Ctrl+C 停止")
            controller.monitor_grips(
                grip_sides,
                hz=cli_args.monitor_hz,
                duration_sec=cli_args.monitor_time,
            )

        elif cli_args.print:
            # 打印身体关节
            if body_joint_names:
                controller.print_joint_states(body_joint_names)
            # 打印手指关节
            if hand_joint_names:
                # 收集涉及的手别
                sides_for_print = set()
                for name in hand_joint_names:
                    side = _infer_hand_side(name)
                    if side:
                        sides_for_print.add(side)
                controller.print_hand_states(sorted(sides_for_print))
            # 如果只指定了 --hand 而没有 --joints，也打印手部
            elif hand_sides and not specified_joints:
                controller.print_hand_states(hand_sides)

        elif cli_args.move:
            pose_dict = parse_move_arg(cli_args.move)
            # 验证关节名
            for name in pose_dict:
                if name not in BODY_JOINT_NAMES:
                    print(f"✗ 未知身体关节名: '{name}'")
                    sys.exit(1)

            print("\n=== 移动身体关节 ===")
            for name, angle in pose_dict.items():
                lo_hi = ""
                if name in BODY_JOINT_LIMITS:
                    lo, hi = BODY_JOINT_LIMITS[name]
                    lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
                print(f"  {name} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")

            input("\n按回车开始移动（Ctrl+C 取消）...")

            ok = controller.move_to_pose(
                pose_dict,
                duration_sec=cli_args.duration,
                wait=True,
                unlock_required_joints=True,
            )
            if ok:
                print("✓ 移动完成，当前位置：")
                controller.print_joint_states(list(pose_dict.keys()))
            else:
                print("✗ 移动失败")

        elif cli_args.shift:
            shift_dict = parse_move_arg(cli_args.shift)
            # 验证关节名
            for name in shift_dict:
                if name not in BODY_JOINT_NAMES:
                    print(f"✗ 未知身体关节名: '{name}'")
                    sys.exit(1)

            print("\n=== 身体关节偏移 ===")
            for name, delta in shift_dict.items():
                current = controller.get_joint_position(name)
                if current is not None:
                    target = current + delta
                    print(
                        f"  {name}: {current:+.4f} → {target:+.4f} rad "
                        f"(Δ={delta:+.4f} rad, {np.degrees(delta):+.2f}°)"
                    )
                else:
                    print(f"  {name}: 无法读取当前位置")

            input("\n按回车开始移动（Ctrl+C 取消）...")

            all_ok = True
            for name, delta in shift_dict.items():
                ok = controller.shift_joint(
                    name, delta, duration_sec=cli_args.duration, wait=True
                )
                if not ok:
                    print(f"✗ {name} 偏移失败")
                    all_ok = False

            if all_ok:
                print("✓ 偏移完成，当前位置：")
                controller.print_joint_states(list(shift_dict.keys()))

        elif cli_args.monitor:
            print(f"\n=== 监控关节 ({cli_args.monitor_hz}Hz) ===")
            if cli_args.monitor_time:
                print(f"  持续 {cli_args.monitor_time:.1f} 秒")
            else:
                print("  按 Ctrl+C 停止")

            # 分开处理身体关节和手指关节
            if body_joint_names:
                controller.monitor_joints(
                    body_joint_names,
                    hz=cli_args.monitor_hz,
                    duration_sec=cli_args.monitor_time,
                )
            elif hand_joint_names:
                controller.monitor_hand_joints(
                    hand_joint_names,
                    hz=cli_args.monitor_hz,
                    duration_sec=cli_args.monitor_time,
                )

        elif cli_args.interactive:
            controller.print_joint_states(body_joint_names or None)
            if hand_sides:
                controller.print_hand_states(hand_sides)
            print("\n节点运行中，可用的 API：")
            print("  # 身体关节")
            print("  controller.get_joint_position('R_elbow_yaw_joint')")
            print("  controller.get_joints_positions(['head_pitch_joint', 'head_yaw_joint'])")
            print("  controller.move_joint('R_elbow_yaw_joint', 0.5, duration_sec=2.0)")
            print("  controller.shift_joint('R_elbow_yaw_joint', +0.1, duration_sec=2.0)")
            print("  controller.print_joint_states(['R_elbow_yaw_joint'])")
            print("  controller.monitor_joints(['head_pitch_joint'], hz=10)")
            print("\n  # 手指关节")
            print("  controller.get_hand_position('left')")
            print("  controller.get_hand_joint_position('left', 'thumb_swing')")
            print("  controller.move_hand('left', {'thumb_swing': 0.5}, duration_sec=2.0)")
            print("  controller.move_hand_joint('left', 'thumb_swing', 0.5)")
            print("  controller.shift_hand('right', 'index_mcp', +0.2)")
            print("  controller.send_hand_position('left', [0.5, 0.3, 0.1, 0.8, 0.8, 0.8, 0.8])")
            print("  controller.print_hand_states(['left', 'right'])")
            print("  controller.monitor_hand_joints(['left_thumb_swing', 'left_index_mcp'], hz=10)")
            print("  controller.hand_periodic_motion(left_hand=True, right_hand=True)")
            print("\n  # 夹爪")
            print("  controller.get_grip_state('left')")
            print("  controller.print_grip_states(['left', 'right'])")
            print("  controller.send_grip_command('left', pos=0.02, force=50, vel=0.005, acc=1.0)")
            print("  controller.home_grip('right')")
            print("  controller.stop_grip('left')")
            print("  controller.monitor_grips(['left', 'right'], hz=10)")
            print("\n按 Ctrl+C 退出。")
            spin_thread.join()

        # ================================================================
        # 手指关节操作
        # ================================================================

        elif cli_args.hand_move:
            pose_dict = parse_move_arg(cli_args.hand_move)

            print("\n=== 移动手指关节 ===")
            for side in hand_sides:
                print(f"\n  [{side} 手]")
                for name_or_short, angle in pose_dict.items():
                    # 解析全名
                    short = name_or_short
                    if not name_or_short.startswith(side + "_"):
                        full = f"{side}_{name_or_short}"
                    else:
                        full = name_or_short
                        short = name_or_short.removeprefix(side + "_")

                    lo_hi = ""
                    if short in V4_HAND_JOINT_LIMITS:
                        lo, hi = V4_HAND_JOINT_LIMITS[short]
                        lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
                    print(f"    {full} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")

            input("\n按回车开始移动（Ctrl+C 取消）...")

            for side in hand_sides:
                ok = controller.move_hand(
                    side, pose_dict,
                    duration_sec=cli_args.duration, wait=True,
                )
                if ok:
                    print(f"✓ {side} 手移动完成")
                else:
                    print(f"✗ {side} 手移动失败")

        elif cli_args.hand_shift:
            shift_dict = parse_move_arg(cli_args.hand_shift)

            print("\n=== 手指关节偏移 ===")
            for side in hand_sides:
                print(f"\n  [{side} 手]")
                for name_or_short, delta in shift_dict.items():
                    current = controller.get_hand_joint_position(side, name_or_short)
                    if current is not None:
                        target = current + delta
                        print(
                            f"    {name_or_short}: {current:+.4f} → {target:+.4f} rad "
                            f"(Δ={delta:+.4f} rad, {np.degrees(delta):+.2f}°)"
                        )
                    else:
                        print(f"    {name_or_short}: 无法读取当前位置")

            input("\n按回车开始移动（Ctrl+C 取消）...")

            for side in hand_sides:
                for name_or_short, delta in shift_dict.items():
                    ok = controller.shift_hand(
                        side, name_or_short, delta,
                        duration_sec=cli_args.duration, wait=True,
                    )
                    if not ok:
                        print(f"✗ {side} 手 {name_or_short} 偏移失败")

            print("✓ 偏移完成")

        elif cli_args.hand_pose:
            angles = resolve_hand_pose_arg(cli_args.hand_pose)

            print("\n=== 设置整手姿态 ===")
            for side in hand_sides:
                joint_names = V4_HAND_JOINT_MAP[side]
                print(f"\n  [{side} 手]")
                for name, angle in zip(joint_names, angles):
                    short = name.removeprefix("left_").removeprefix("right_")
                    lo_hi = ""
                    if short in V4_HAND_JOINT_LIMITS:
                        lo, hi = V4_HAND_JOINT_LIMITS[short]
                        lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
                    print(f"    {name} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")

            input("\n按回车开始移动（Ctrl+C 取消）...")

            for side in hand_sides:
                pose_dict = build_hand_pose_dict_from_full(angles, side)
                ok = controller.move_hand(
                    side, pose_dict,
                    duration_sec=cli_args.duration, wait=True,
                )
                if ok:
                    print(f"✓ {side} 手姿态设置完成")
                else:
                    print(f"✗ {side} 手姿态设置失败")

        elif cli_args.hand_open:
            print("\n=== 手指张开 ===")
            for side in hand_sides:
                print(f"  {side} 手: 所有关节 → 0.0 rad")
            input("\n按回车开始移动（Ctrl+C 取消）...")

            for side in hand_sides:
                ok = controller.move_hand(
                    side, V4_HAND_OPEN_POSE,
                    duration_sec=cli_args.duration, wait=True,
                )
                if ok:
                    print(f"✓ {side} 手张开完成")
                else:
                    print(f"✗ {side} 手张开失败")

        elif cli_args.hand_close:
            print("\n=== 手指握拳 ===")
            for side in hand_sides:
                joint_names = V4_HAND_JOINT_MAP[side]
                print(f"  {side} 手:")
                for name in joint_names:
                    short = name.removeprefix("left_").removeprefix("right_")
                    if short in V4_HAND_JOINT_LIMITS:
                        _, hi = V4_HAND_JOINT_LIMITS[short]
                        print(f"    {name} → {hi:+.4f} rad ({np.degrees(hi):+.2f}°)")
            input("\n按回车开始移动（Ctrl+C 取消）...")

            for side in hand_sides:
                ok = controller.move_hand(
                    side, V4_HAND_CLOSE_POSE,
                    duration_sec=cli_args.duration, wait=True,
                )
                if ok:
                    print(f"✓ {side} 手握拳完成")
                else:
                    print(f"✗ {side} 手握拳失败")

        elif cli_args.hand_wave:
            print("\n=== 手部周期波形运动 ===")
            for side in hand_sides:
                print(f"  {side} 手")
            print("  按 Ctrl+C 停止")
            controller.hand_periodic_motion(
                left_hand="left" in hand_sides,
                right_hand="right" in hand_sides,
            )

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")

    finally:
        controller.stop()
        time.sleep(0.1)
        executor.remove_node(controller)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
