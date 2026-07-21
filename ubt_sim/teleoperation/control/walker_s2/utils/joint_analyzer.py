#!/usr/bin/env python3
"""
关节运动实时分析脚本

同时订阅 /mc/sdk/robot_command（指令）和 /mc/sdk/robot_state（实际），
逐帧对比指定关节的指令位置 vs 实际位置，实时输出跟踪误差、延迟、超调等指标。

【使用示例】

    # 分析 L_elbow_yaw_joint，同时发送 0.5 rad 阶跃指令
    python3 joint_analysis.py --joint L_elbow_yaw_joint --step 0.5

    # 分析 L_elbow_yaw_joint，发送 0.3 rad 振幅的正弦波指令
    python3 joint_analysis.py --joint L_elbow_yaw_joint --sine --amplitude 0.3 --period 4.0

    # 纯监听模式（不发送指令，只分析已有控制器的运动）
    python3 joint_analysis.py --joint L_elbow_yaw_joint --listen --duration 10.0

    # 记录到 CSV 文件用于离线分析
    python3 joint_analysis.py --joint L_elbow_yaw_joint --step 0.5 --csv /tmp/joint_data.csv
"""

import argparse
import csv
import os
import sys
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
except ImportError as exc:
    raise ImportError(
        "Walker S2 ROS2 SDK messages not found. Source ROS2 and the vendored Walker SDK messages first.\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash"
    ) from exc

# ---- 与 walker_s2_controller.py 一致的常量 ----
BODY_JOINT_NAMES = [
    "L_elbow_roll_joint",    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint","L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",  "L_wrist_pitch_joint",
    "L_wrist_roll_joint",    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",     "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",   "R_wrist_roll_joint",
    "head_pitch_joint",      "head_yaw_joint",
    "waist_yaw_joint",
]

BODY_JOINT_LIMITS = {
    "L_elbow_roll_joint":       (-2.6180, 0.0),
    "L_elbow_yaw_joint":        (-2.9147, 2.9147),
    "L_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "L_shoulder_roll_joint":    (-1.85,   0.0873),
    "L_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "L_wrist_pitch_joint":      (-1.5882, 1.5882),
    "L_wrist_roll_joint":       (-1.9897, 1.9897),
    "R_elbow_roll_joint":       (-2.6180, 0.0),
    "R_elbow_yaw_joint":        (-2.9147, 2.9147),
    "R_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "R_shoulder_roll_joint":    (-1.85,   0.0873),
    "R_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}


class JointAnalyzer(Node):
    """同时订阅 RobotCommand 和 RobotState，逐帧对比关节指令 vs 实际。"""

    def __init__(self, joint_name: str, *, control_hz: float = 200.0):
        super().__init__("joint_analyzer")

        if joint_name not in BODY_JOINT_NAMES:
            raise ValueError(f"Unknown joint: {joint_name}")
        self.joint_name = joint_name
        self.joint_idx = BODY_JOINT_NAMES.index(joint_name)
        self.control_hz = control_hz

        # 限位
        self.joint_limit = BODY_JOINT_LIMITS.get(joint_name)

        # 数据缓冲
        self._data_lock = threading.Lock()
        self._cmd_history: deque = deque(maxlen=10000)   # (t, cmd_pos)
        self._state_history: deque = deque(maxlen=10000)  # (t, actual_pos)
        self._raw_samples: list = []  # 用于 CSV 导出: [{t, cmd, actual}, ...]

        # 统计数据
        self._max_err = 0.0
        self._max_err_t = 0.0
        self._err_sum = 0.0
        self._err_count = 0
        self._start_time: Optional[float] = None

        # QoS
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 订阅 RobotState（实际位置）
        self.state_sub = self.create_subscription(
            RobotState, "/mc/sdk/robot_state", self._state_callback, qos_sub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # 订阅 RobotCommand（指令位置）
        self.cmd_sub = self.create_subscription(
            RobotCommand, "/mc/sdk/robot_command", self._cmd_callback, qos_sub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # 指令发布器（用于发送测试信号）
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(RobotCommand, "/mc/sdk/robot_command", qos_pub)

        # 200Hz 定时器（用于主动发送测试信号）
        self._signal_active = False
        self._signal_type = None      # "step", "sine"
        self._signal_start_t: float = 0.0
        self._signal_amplitude: float = 0.0
        self._signal_period: float = 0.0
        self._signal_offset: float = 0.0   # 阶跃目标值 / 正弦偏移
        self._signal_initial_pos: float = 0.0  # 记录 motion 开始时刻的实际位置（用于保持其他关节）

        self.control_timer = self.create_timer(
            1.0 / control_hz, self._control_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(f"JointAnalyzer ready: {joint_name}")

    # ---- 公开 API ----

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """等待首个 state + cmd 数据。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._data_lock:
                if len(self._state_history) > 0:
                    return True
            time.sleep(0.05)
        return False

    def send_step(self, target_rad: float):
        """发送阶跃指令：将目标关节直接设为目标值。"""
        self._signal_type = "step"
        self._signal_amplitude = 0.0
        self._signal_offset = float(target_rad)
        self._start_signal()

    def send_sine(self, amplitude: float, period_sec: float, offset: float = 0.0):
        """发送正弦波指令：position = offset + amplitude * sin(2π * t / period)。"""
        self._signal_type = "sine"
        self._signal_amplitude = float(amplitude)
        self._signal_period = float(period_sec)
        self._signal_offset = float(offset)
        self._start_signal()

    def stop_signal(self):
        """停止发送测试信号。"""
        self._signal_active = False
        self.get_logger().info("Test signal stopped")

    def _start_signal(self):
        """记录 signal 开始时刻和初始位置。"""
        with self._data_lock:
            if self._state_history:
                self._signal_initial_pos = self._state_history[-1][1]
            else:
                self._signal_initial_pos = 0.0
        self._signal_start_t = time.time()
        self._signal_active = True

    def get_stats(self) -> dict:
        """获取当前统计指标。"""
        with self._data_lock:
            return {
                "max_err_rad": self._max_err,
                "max_err_deg": np.degrees(self._max_err),
                "max_err_t": self._max_err_t,
                "mean_err_rad": self._err_sum / self._err_count if self._err_count > 0 else 0.0,
                "sample_count": self._err_count,
            }

    def get_raw_samples(self) -> list:
        """获取所有采样点 [{t, cmd, actual}, ...]。"""
        with self._data_lock:
            return list(self._raw_samples)

    def export_csv(self, path: str):
        """导出采样点到 CSV 文件。"""
        samples = self.get_raw_samples()
        if not samples:
            self.get_logger().warning("No samples to export")
            return
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t", "cmd", "actual", "err"])
            writer.writeheader()
            writer.writerows(samples)
        self.get_logger().info(f"Exported {len(samples)} samples to {path}")

    # ---- 内部回调 ----

    def _state_callback(self, msg: RobotState):
        """RobotState → 提取目标关节实际位置。"""
        t = time.time()
        if self._start_time is None:
            self._start_time = t

        joint_states = msg.joint_states
        name_to_idx = {name: idx for idx, name in enumerate(joint_states.name)}

        if self.joint_name not in name_to_idx:
            return

        actual = float(joint_states.position[name_to_idx[self.joint_name]])
        rel_t = t - self._start_time

        with self._data_lock:
            self._state_history.append((rel_t, actual))

    def _cmd_callback(self, msg: RobotCommand):
        """RobotCommand → 提取目标关节指令位置。"""
        t = time.time()
        if self._start_time is None:
            self._start_time = t

        # 从 joint_cmd 列表中找到目标关节
        cmd_val = None
        for jc in msg.joint_cmd:
            if jc.name == self.joint_name:
                cmd_val = float(jc.position)
                break

        if cmd_val is None:
            return

        rel_t = t - self._start_time

        with self._data_lock:
            self._cmd_history.append((rel_t, cmd_val))

    def _control_callback(self):
        """200Hz：发送测试信号 + 计算误差指标。"""
        t = time.time()

        # ---- 发送测试信号 ----
        if self._signal_active:
            elapsed = t - self._signal_start_t

            if self._signal_type == "step":
                target = self._signal_offset
            elif self._signal_type == "sine":
                omega = 2 * np.pi / self._signal_period
                target = self._signal_offset + self._signal_amplitude * np.sin(omega * elapsed)
            else:
                target = 0.0

            # 限位裁剪
            if self.joint_limit is not None:
                lo, hi = self.joint_limit
                target = max(lo, min(hi, target))

            self._publish_single_joint_cmd(target)

        # ---- 计算误差 ----
        self._compute_error(t)

    def _publish_single_joint_cmd(self, position: float):
        """发布只包含目标关节的 RobotCommand，同时写入 _cmd_history 供误差计算。

        注意：Bridge 订阅 RobotCommand 后消费并转发到 Isaac Sim，不会回显到 ROS2。
        因此主动模式下自行记录发出的指令，而不是依赖 _cmd_callback。
        """
        t = time.time()
        rel_t = t - self._start_time if self._start_time is not None else 0.0

        # 记录指令（bridge 不回显 RobotCommand，需自行记录）
        with self._data_lock:
            self._cmd_history.append((rel_t, float(position)))

        msg = RobotCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        jc = JointCmd()
        jc.name = self.joint_name
        jc.control_mode = JointCmd.MODE_POSITION
        jc.position = float(position)
        msg.joint_cmd.append(jc)

        self.cmd_pub.publish(msg)

    def _compute_error(self, t: float):
        """匹配最近的 cmd 和 state，计算跟踪误差。"""
        with self._data_lock:
            if not self._state_history or not self._cmd_history:
                return

            # 取最新的 state 和 cmd
            state_t, actual = self._state_history[-1]
            cmd_t, cmd = self._cmd_history[-1]

            # 只统计有 cmd 数据之后的误差
            if state_t < self._cmd_history[0][0]:
                return

            err = abs(cmd - actual)
            rel_t = state_t

            self._raw_samples.append({"t": round(rel_t, 4), "cmd": round(cmd, 6), "actual": round(actual, 6), "err": round(err, 6)})

            if err > self._max_err:
                self._max_err = err
                self._max_err_t = rel_t
            self._err_sum += err
            self._err_count += 1

    # ---- 实时输出 ----

    def print_status_line(self):
        """输出一行当前状态。"""
        with self._data_lock:
            if not self._state_history or not self._cmd_history:
                return

            state_t, actual = self._state_history[-1]
            cmd_t, cmd = self._cmd_history[-1]

            err = abs(cmd - actual)
            delay_ms = (state_t - cmd_t) * 1000

        limit_str = ""
        if self.joint_limit is not None:
            lo, hi = self.joint_limit
            if actual < lo or actual > hi:
                limit_str = " ⚠️LIMIT"

        mode = ""
        if self._signal_active:
            mode = f" [{self._signal_type}]"

        print(
            f"\r{' ' * 100}\r"  # 清行
            f"t={state_t:6.2f}s | "
            f"cmd={cmd:+8.4f} | actual={actual:+8.4f} | "
            f"err={err:.4f}rad ({np.degrees(err):.2f}°) | "
            f"delay={delay_ms:+.1f}ms"
            f"{limit_str}{mode}",
            end="", flush=True,
        )


# ============================================================================
# 命令行入口
# ============================================================================

def main(args=None):
    parser = argparse.ArgumentParser(
        description="关节运动实时分析 — 同时订阅 RobotCommand + RobotState，对比指令 vs 实际",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--joint", required=True, choices=BODY_JOINT_NAMES,
                        help="要分析的关节名")
    parser.add_argument("--step", type=float, default=None,
                        help="发送阶跃指令到目标角度（rad）")
    parser.add_argument("--sine", action="store_true",
                        help="发送正弦波指令")
    parser.add_argument("--amplitude", type=float, default=0.3,
                        help="正弦波振幅（rad），默认 0.3")
    parser.add_argument("--period", type=float, default=4.0,
                        help="正弦波周期（s），默认 4.0")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="正弦波偏置（rad），默认 0.0")
    parser.add_argument("--listen", action="store_true",
                        help="纯监听模式：不发送指令，只观察已有控制器的运动")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="运行时长（s），默认 10.0")
    parser.add_argument("--csv", type=str, default=None,
                        help="导出 CSV 文件路径")
    parser.add_argument("--hz", type=float, default=10.0,
                        help="终端刷新频率（Hz），默认 10")
    parser.add_argument("--quiet", action="store_true",
                        help="安静模式：只在结束时打印汇总")

    cli_args, ros_args = parser.parse_known_args(args)

    # 确定运行模式
    if cli_args.listen:
        mode = "listen"
    elif cli_args.sine:
        mode = "sine"
    elif cli_args.step is not None:
        mode = "step"
    else:
        mode = "listen"
        cli_args.listen = True

    rclpy.init(args=ros_args)

    analyzer = JointAnalyzer(cli_args.joint)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(analyzer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print(f"\n{'='*70}")
        print(f"  关节分析: {cli_args.joint}")
        print(f"  模式:     {mode}")
        print(f"  限位:     {BODY_JOINT_LIMITS.get(cli_args.joint, 'N/A')}")
        if not cli_args.quiet:
            print(f"  刷新:     {cli_args.hz} Hz")
        print(f"  时长:     {cli_args.duration} s")
        print(f"{'='*70}\n")

        if not analyzer.wait_for_data(timeout=5.0):
            print("[FATAL] 未收到 RobotState，请确认仿真已启动")
            return

        # 记录初始位置
        with analyzer._data_lock:
            init_pos = analyzer._state_history[-1][1] if analyzer._state_history else 0.0
        print(f"初始位置: {init_pos:+.4f} rad ({np.degrees(init_pos):+.2f}°)")

        # 等待 cmd 数据（listen 模式下 cmd 可能来自外部控制器）
        if mode == "listen":
            print("纯监听模式，等待 RobotCommand 数据...")
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with analyzer._data_lock:
                    if len(analyzer._cmd_history) > 0:
                        break
                time.sleep(0.1)
            with analyzer._data_lock:
                if len(analyzer._cmd_history) == 0:
                    print("[WARN] 3秒内未收到 RobotCommand，将只记录实际位置（无误差对比）")

        # 发送测试信号
        if mode == "step":
            target = cli_args.step
            print(f"\n发送阶跃指令: {init_pos:+.4f} → {target:+.4f} rad "
                  f"(Δ={target - init_pos:+.4f} rad, {np.degrees(target - init_pos):+.2f}°)")
            input("按回车发送指令（Ctrl+C 取消）...")
            analyzer.send_step(target)

        elif mode == "sine":
            print(f"\n发送正弦波: amplitude={cli_args.amplitude:.3f} rad, "
                  f"period={cli_args.period:.1f}s, offset={cli_args.offset:.3f} rad")
            input("按回车发送指令（Ctrl+C 取消）...")
            analyzer.send_sine(cli_args.amplitude, cli_args.period, cli_args.offset)

        # 主循环：实时打印状态
        print()
        interval = 1.0 / cli_args.hz
        start = time.time()

        try:
            while time.time() - start < cli_args.duration:
                if not cli_args.quiet:
                    analyzer.print_status_line()
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n\n用户中断")

        # 停止信号
        analyzer.stop_signal()
        time.sleep(0.1)

        # ---- 汇总分析 ----
        stats = analyzer.get_stats()
        print(f"\n{'='*70}")
        print(f"  分析汇总: {cli_args.joint}")
        print(f"{'='*70}")
        print(f"  采样点数:    {stats['sample_count']}")
        print(f"  最大跟踪误差: {stats['max_err_rad']:.4f} rad  ({stats['max_err_deg']:.2f}°)")
        print(f"  平均跟踪误差: {stats['mean_err_rad']:.4f} rad  ({np.degrees(stats['mean_err_rad']):.2f}°)")
        print(f"  最大误差时刻: t = {stats['max_err_t']:.2f} s")

        # 额外分析
        samples = analyzer.get_raw_samples()
        if len(samples) >= 2:
            errors = [s["err"] for s in samples]
            err_arr = np.array(errors)

            # 稳态误差（最后 20% 数据的平均误差）
            n_steady = max(1, len(err_arr) // 5)
            steady_err = float(np.mean(err_arr[-n_steady:]))
            print(f"  稳态误差:     {steady_err:.4f} rad  ({np.degrees(steady_err):.2f}°)")

            # 误差标准差
            err_std = float(np.std(err_arr))
            print(f"  误差标准差:   {err_std:.4f} rad  ({np.degrees(err_std):.2f}°)")

            # 峰峰值
            actuals = np.array([s["actual"] for s in samples])
            p2p = float(np.ptp(actuals))
            print(f"  实际位置峰峰值: {p2p:.4f} rad  ({np.degrees(p2p):.2f}°)")

            # 超调检测（阶跃模式下）
            if mode == "step" and len(samples) > 10:
                cmds = np.array([s["cmd"] for s in samples])
                target_val = cmds[-1]
                overshoot = float(np.max(actuals) - target_val) if target_val > init_pos else float(target_val - np.min(actuals))
                if overshoot > 0.005:
                    print(f"  ⚠️ 超调:       {overshoot:.4f} rad  ({np.degrees(overshoot):.2f}°)")

            # 上升时间（阶跃模式下，从 10% → 90%）
            if mode == "step" and len(samples) > 10:
                target_final = float(np.array([s["cmd"] for s in samples])[-1])
                step_size = target_final - float(actuals[0])
                if abs(step_size) > 0.01:
                    lo_thresh = actuals[0] + 0.1 * step_size
                    hi_thresh = actuals[0] + 0.9 * step_size
                    t_lo = t_hi = None
                    for s in samples:
                        if t_lo is None and (s["actual"] - actuals[0]) * step_size >= 0.1 * abs(step_size):
                            t_lo = s["t"]
                        if t_hi is None and (s["actual"] - actuals[0]) * step_size >= 0.9 * abs(step_size):
                            t_hi = s["t"]
                            break
                    if t_lo is not None and t_hi is not None:
                        rise_time = t_hi - t_lo
                        print(f"  上升时间(10→90%): {rise_time:.3f} s")

            # 延迟估计（cmd 和 state 时间戳差）
            if len(samples) >= 10:
                delays = []
                for i in range(len(samples)):
                    s = samples[i]
                    cmd_t = None
                    state_t = s["t"]
                    # 在 cmd_history 中找最近的 cmd
                    with analyzer._data_lock:
                        for ct, cv in analyzer._cmd_history:
                            if abs(ct - state_t) < 0.5:
                                cmd_t = ct
                                break
                # 简化：用 state 时间戳与对应 cmd 的差值
                print(f"  （延迟分析需离线处理 CSV 数据）")

        if cli_args.csv:
            analyzer.export_csv(cli_args.csv)

        print(f"\n{'='*70}\n")

    except KeyboardInterrupt:
        pass

    finally:
        analyzer.stop_signal()
        time.sleep(0.1)
        try:
            executor.remove_node(analyzer)
        except Exception:
            pass
        analyzer.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
