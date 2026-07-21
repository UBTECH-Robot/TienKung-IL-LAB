#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 键盘+鼠标末端执行器遥操作控制脚本。

通过 pynput 捕获键盘和鼠标输入，实时控制机械臂末端执行器的位姿。

EE 局部坐标系（sixforce_link 帧，相对夹爪）: x=夹爪下方, y=夹爪左方, z=夹爪前方

键位映射::

    W/S       末端局部坐标系前后移动 (z 轴, z+=夹爪前方)
    A/D       末端局部坐标系左右移动 (y 轴, y+=夹爪左方)
    Q/E       基座坐标系上下移动 (world Z)
    R/T       滚转 roll  (绕夹爪前方 z, R=左滚 T=右滚)
    鼠标 X    偏航 yaw  (绕夹爪下方 x)
    鼠标 Y    俯仰 pitch (绕夹爪左方 y)
    ↑↓←→      同鼠标（方向键替代，粗调）
    空格       夹爪开合切换（边沿触发）
    1          切换左右臂控制（边沿触发，切换后需按 K 同步）
    K          从机器人同步虚拟目标位姿
    ESC        退出

参考实现：
    LeRobot ``teleop_keyboard.py`` — pynput 事件队列 + 状态字典模式

运行前置条件（同 pick_part.py）::

    1. 仿真/真机端启动 ROS2 bridge
    2. 本容器内 source ROS2 + Walker SDK 环境
    3. python keyboard_ee_control.py [--side right] [--translation-step 0.005] ...
"""

import argparse
import logging
import os
import sys
import threading
import time
from queue import Queue

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

# ------------------------------------------------------------------
# pynput 可用性检查（参考 LeRobot keyboard teleoperator）
# ------------------------------------------------------------------
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        logging.warning("未检测到 DISPLAY 环境变量，pynput 可能无法捕获键盘/鼠标事件。")
        logging.warning("如果在 headless 容器内运行，请在宿主机上执行本脚本。")
    from pynput import keyboard  # noqa: E402
    from pynput import mouse     # noqa: E402
except Exception as e:
    logging.error(f"无法导入 pynput: {e}")
    logging.error("请安装: pip install pynput")
    sys.exit(1)

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)
from utils.controller import WalkerS2Controller

# ============================================================================
# RPY 工具函数（与 controller.py 中 _rpy_to_rotation_matrix 约定一致）
# ============================================================================


def _rpy_to_rotation_matrix(roll, pitch, yaw):
    """RPY (intrinsic XYZ) → 3×3 旋转矩阵。

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，与 URDF / Pinocchio 约定一致。
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def _rotation_matrix_to_rpy(R):
    """3×3 旋转矩阵 → RPY (intrinsic XYZ)，与 URDF / Pinocchio 约定一致。"""
    # pitch = atan2(-R20, sqrt(R00^2 + R10^2))
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # 万向节死锁处理
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return float(roll), float(pitch), float(yaw)


# ============================================================================
# 默认控制参数
# ============================================================================

DEFAULT_TRANSLATION_STEP = 0.003     # 平移步长 (m/tick)，每帧 3mm
DEFAULT_ROTATION_STEP = 0.015        # 键盘旋转步长 (rad/tick)，约 0.86°/tick
DEFAULT_MOUSE_SENSITIVITY = 0.0005   # 鼠标灵敏度 (rad/pixel)
DEFAULT_POLL_HZ = 200.0              # 键盘/鼠标采样频率
DEFAULT_SEND_HZ = 30.0               # 控制信号发送频率
DEFAULT_DURATION = 0.12              # 每帧轨迹时长 (s)
DEFAULT_MAX_DELTA_XYZ = 0.02         # 单次发送最大平移累积 (m)，20mm/send
DEFAULT_MAX_DELTA_RPY = 0.10         # 单次发送最大旋转累积 (rad)
DEFAULT_SYNC_INTERVAL = 1.0          # 全量同步实际 EE 位姿的间隔 (s)

# ============================================================================
# 键盘+鼠标 EE 控制器
# ============================================================================


class KeyboardEEController:
    """键盘+鼠标末端执行器遥操作控制器。

    采用 LeRobot ``KeyboardTeleop`` 的事件队列 + 状态字典模式：
    - pynput ``keyboard.Listener`` 捕获按键事件 → ``Queue`` → ``current_pressed`` dict
    - pynput ``mouse.Controller`` 在主循环中采样鼠标位置计算 delta
    - 所有 delta 合并为单次 ``move_arm_ee_delta_local(wait=False)`` 调用
    - 空格/'1' 使用边沿触发防抖

    Parameters
    ----------
    controller : WalkerS2Controller
        已初始化的控制器实例（需 enable_ik=True）。
    side : str
        初始控制手臂 ("left" / "right")。
    translation_step : float
        每次按键帧的平移步长 (m)。
    rotation_step : float
        每次按键帧的旋转步长 (rad)。
    mouse_sensitivity : float
        鼠标每像素对应的旋转弧度。
    control_hz : float
        控制循环频率。
    duration : float
        每帧下发轨迹的时长 (s)。
    """

    def __init__(
        self,
        controller: WalkerS2Controller,
        side: str = "right",
        translation_step: float = DEFAULT_TRANSLATION_STEP,
        rotation_step: float = DEFAULT_ROTATION_STEP,
        mouse_sensitivity: float = DEFAULT_MOUSE_SENSITIVITY,
        poll_hz: float = DEFAULT_POLL_HZ,
        send_hz: float = DEFAULT_SEND_HZ,
        duration: float = DEFAULT_DURATION,
        max_delta_xyz: float = DEFAULT_MAX_DELTA_XYZ,
        max_delta_rpy: float = DEFAULT_MAX_DELTA_RPY,
        sync_interval: float = DEFAULT_SYNC_INTERVAL,
    ):
        self._controller = controller
        self.side = side
        self.translation_step = translation_step
        self.rotation_step = rotation_step
        self.mouse_sensitivity = mouse_sensitivity
        self.poll_hz = poll_hz
        self.send_hz = send_hz
        self.duration = duration
        self.poll_period = 1.0 / poll_hz
        self.send_period = 1.0 / send_hz
        self.max_delta_xyz = max_delta_xyz
        self.max_delta_rpy = max_delta_rpy
        self.sync_interval = sync_interval

        # 键盘状态（参考 LeRobot KeyboardTeleop）
        self._key_event_queue: Queue = Queue()
        self.current_pressed: dict = {}

        # 鼠标状态
        self._mouse_ctrl = mouse.Controller()

        # 夹爪切换状态（边沿触发用）
        self._gripper_open: bool = True
        self._space_prev: bool = False

        # 手臂切换状态（边沿触发用）
        self._key1_prev: bool = False

        # 启停控制 / 同步（边沿触发，K 键）
        self._enabled: bool = True
        self._key_k_prev: bool = False

        # 虚拟目标位姿 (base frame [x,y,z,roll,pitch,yaw])
        # 所有 delta 在此变量上累积，启动时和按 K 时从机器人同步
        self._virtual_pose: np.ndarray | None = None

        # 帧间 delta 累积器（在 is_busy 期间累积，发送后清零）
        self._acc_delta_xyz = np.zeros(3)
        self._acc_delta_rpy = np.zeros(3)

        # 鼠标归位（None=不归位，tuple=归位中心点坐标）
        self._mouse_center: tuple | None = None

        # pynput 监听器
        self._key_listener: keyboard.Listener | None = None
        self._running: bool = False

    # ------------------------------------------------------------------
    # pynput 回调
    # ------------------------------------------------------------------

    def _on_key_press(self, key):
        """按键按下回调 → 推入事件队列。"""
        try:
            key_char = key.char
        except AttributeError:
            key_char = key
        self._key_event_queue.put((key_char, True))

    def _on_key_release(self, key):
        """按键释放回调 → 推入事件队列；ESC 直接退出。"""
        try:
            key_char = key.char
        except AttributeError:
            key_char = key
        self._key_event_queue.put((key_char, False))
        if key == keyboard.Key.esc:
            self._running = False

    # ------------------------------------------------------------------
    # 键盘状态管理（LeRobot 模式）
    # ------------------------------------------------------------------

    def _drain_key_events(self):
        """消费事件队列，更新 current_pressed 状态字典。"""
        while not self._key_event_queue.empty():
            key, is_pressed = self._key_event_queue.get_nowait()
            self.current_pressed[key] = is_pressed

    def _is_pressed(self, key) -> bool:
        """查询按键是否处于按下状态。"""
        return self.current_pressed.get(key, False)

    # ------------------------------------------------------------------
    # 主控制循环
    # ------------------------------------------------------------------

    def run(self):
        """启动键盘监听并进入主控制循环。

        双频架构：
        - 输入采样以 ``poll_hz`` 频率运行（默认 200Hz），低延迟捕获键鼠
        - 控制信号以 ``send_hz`` 频率发送（默认 30Hz），delta 在采样帧间累积

        每帧：
        1. 消费键盘事件队列
        2. 计算鼠标位移 delta
        3. 累积 delta_xyz / delta_rpy
        4. 鼠标归位（防止屏幕边缘）
        5. 到达发送周期时：钳制 → 应用累积 → 下发 IK 目标
        6. 边沿触发：空格/'1'/'K'
        7. 频率控制
        """
        self._key_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release,
        )
        try:
            self._key_listener.start()
        except Exception as e:
            print(f"\nERROR: 无法启动键盘监听器: {e}")
            print("pynput 需要 X11 display server。如果当前在 headless 容器内运行，")
            print("请在宿主机上执行本脚本，或启动容器时挂载 display:")
            print("  docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...")
            raise
        self._running = True

        self._print_banner()

        # 初始化虚拟位姿（从机器人读取当前末端姿态）
        print(f"读取 {self.side} 臂当前末端姿态...")
        ee_init = self._controller.get_ee_pose(self.side)
        if ee_init is None:
            print("ERROR: 无法获取当前末端姿态，请检查 IK 初始化和机器人状态")
            self._running = False
            return
        self._virtual_pose = np.array(ee_init, dtype=float)
        print(f"  初始位姿 (base frame): xyz={[round(v,4) for v in self._virtual_pose[:3]]} "
              f"rpy={[round(v,4) for v in self._virtual_pose[3:]]}")

        prev_mx, prev_my = self._mouse_ctrl.position
        self._mouse_center = (prev_mx, prev_my)  # 记录初始位置作为归位中心
        frame_count = 0
        last_send_time = time.perf_counter()
        last_sync_time = time.perf_counter()
        _first_key_event = False

        try:
            while self._running and rclpy.ok():
                loop_start = time.perf_counter()
                frame_count += 1

                # --- 1. 消费键盘事件 ---
                had_events = not self._key_event_queue.empty()
                self._drain_key_events()

                if had_events and not _first_key_event:
                    _first_key_event = True
                    print(f"[诊断] 已收到键盘事件，当前 {len(self.current_pressed)} 个按键状态")
                    active = [str(k) for k, v in self.current_pressed.items() if v]
                    if active:
                        print(f"[诊断] 已按下的键: {active}")

                if frame_count % 600 == 1 and frame_count > 1:
                    active_keys = [str(k) for k, v in self.current_pressed.items() if v]
                    print(f"[心跳] 帧 {frame_count}, 活跃按键: {active_keys or '无'}")

                # --- 2. 鼠标 delta ---
                cur_mx, cur_my = self._mouse_ctrl.position
                mouse_dx = cur_mx - prev_mx
                mouse_dy = cur_my - prev_my
                prev_mx, prev_my = cur_mx, cur_my

                # --- 3. 计算局部坐标系 delta ---
                # EE 局部坐标系 (sixforce_link 帧，相对夹爪): x=夹爪下方, y=夹爪左方, z=夹爪前方
                step = self.translation_step
                d_local_x = 0.0  # x=夹爪下方
                d_local_y = 0.0  # y=夹爪左方
                d_local_z = 0.0  # z=夹爪前方（夹爪指向）
                if self._is_pressed("w"):
                    d_local_z += step
                if self._is_pressed("s"):
                    d_local_z -= step
                if self._is_pressed("a"):
                    d_local_y += step
                if self._is_pressed("d"):
                    d_local_y -= step

                # 基座坐标系 Z (QE)
                dz_base = 0.0
                if self._is_pressed("q"):
                    dz_base += step
                if self._is_pressed("e"):
                    dz_base -= step

                # 旋转 delta（EE 局部坐标系，按实际物理轴命名）
                # dd_x: 绕 X=夹爪下方 = yaw   |  dd_y: 绕 Y=夹爪左方 = pitch
                # dd_z: 绕 Z=夹爪前方 = roll
                rot_step = self.rotation_step
                ms = self.mouse_sensitivity
                dd_x = 0.0   # yaw (绕 X)
                dd_y = 0.0   # pitch (绕 Y)
                dd_z = 0.0   # roll (绕 Z)
                # 鼠标 X → yaw (绕 X)
                dd_x += mouse_dx * ms
                # 鼠标 Y → pitch (绕 Y)
                dd_y += -mouse_dy * ms
                if self._is_pressed(keyboard.Key.up):
                    dd_y -= rot_step
                if self._is_pressed(keyboard.Key.down):
                    dd_y += rot_step
                if self._is_pressed(keyboard.Key.left):
                    dd_x -= rot_step
                if self._is_pressed(keyboard.Key.right):
                    dd_x += rot_step
                if self._is_pressed("r"):
                    dd_z -= rot_step
                if self._is_pressed("t"):
                    dd_z += rot_step

                delta_xyz_local = np.array([d_local_x, d_local_y, d_local_z])
                delta_rpy = np.array([dd_x, dd_y, dd_z])  # → Rx(dd_x) @ Ry(dd_y) @ Rz(dd_z)

                has_local = any(abs(v) > 1e-9 for v in delta_xyz_local) or any(abs(v) > 1e-9 for v in delta_rpy)
                has_base_z = abs(dz_base) > 1e-9

                # --- 4a. 鼠标归位（防止飞到屏幕边缘）---
                if self._mouse_center is not None:
                    cx, cy = self._mouse_center
                    self._mouse_ctrl.position = (cx, cy)
                    prev_mx, prev_my = cx, cy

                # --- 4b. 累积 delta 到虚拟位姿，钳制单次最大位移 ---
                if (has_local or has_base_z) and self._virtual_pose is not None:
                    vp = self._virtual_pose
                    R_vp = _rpy_to_rotation_matrix(vp[3], vp[4], vp[5])

                    # 平移: 局部 delta → base frame (旋转) + base-Z (直接加)
                    delta_base = R_vp @ delta_xyz_local
                    self._acc_delta_xyz += delta_base
                    self._acc_delta_xyz[2] += dz_base

                    # 旋转: 累积
                    self._acc_delta_rpy += delta_rpy

                # --- 5. 下发运动（send_hz 节拍 + 等上一条轨迹完成）---
                now = time.perf_counter()
                if (self._enabled and self._virtual_pose is not None
                        and now - last_send_time >= self.send_period
                        and not self._controller.is_busy):
                    # 钳制累积量，防止单次位移过大触发安全速度检查
                    acc_norm = float(np.linalg.norm(self._acc_delta_xyz))
                    if acc_norm > self.max_delta_xyz and acc_norm > 1e-9:
                        self._acc_delta_xyz *= self.max_delta_xyz / acc_norm
                    rpy_norm = float(np.linalg.norm(self._acc_delta_rpy))
                    if rpy_norm > self.max_delta_rpy and rpy_norm > 1e-9:
                        self._acc_delta_rpy *= self.max_delta_rpy / rpy_norm

                    # 应用累积 delta 到虚拟位姿
                    if acc_norm > 1e-9 or rpy_norm > 1e-9:
                        vp = self._virtual_pose
                        R_vp = _rpy_to_rotation_matrix(vp[3], vp[4], vp[5])
                        vp[0] += self._acc_delta_xyz[0]
                        vp[1] += self._acc_delta_xyz[1]
                        vp[2] += self._acc_delta_xyz[2]
                        if rpy_norm > 1e-9:
                            R_delta = _rpy_to_rotation_matrix(
                                self._acc_delta_rpy[0], self._acc_delta_rpy[1], self._acc_delta_rpy[2])
                            R_new = R_vp @ R_delta
                            vp[3], vp[4], vp[5] = _rotation_matrix_to_rpy(R_new)

                        self._controller.move_arm_ik(
                            self.side,
                            target_xyzrpy=vp.tolist(),
                            duration_sec=self.duration,
                            wait=False,
                            require_success=False,
                            max_iter=300,
                        )

                    # 发送后清零累积器并更新时间戳
                    self._acc_delta_xyz = np.zeros(3)
                    self._acc_delta_rpy = np.zeros(3)
                    last_send_time = now

                # --- 5a. 定期全量同步实际 EE 位姿（防漂移）---
                if now - last_sync_time >= self.sync_interval:
                    ee_actual = self._controller.get_ee_pose(self.side)
                    if ee_actual is not None:
                        self._virtual_pose = np.array(ee_actual, dtype=float)
                    last_sync_time = now
                    self._acc_delta_xyz = np.zeros(3)
                    self._acc_delta_rpy = np.zeros(3)

                # --- 6. 夹爪切换（空格，边沿触发）---
                space_pressed = self._is_pressed(keyboard.Key.space)
                if space_pressed and not self._space_prev:
                    if self._gripper_open:
                        self._controller.close_grip(self.side, wait=False)
                        self._gripper_open = False
                        print(f"[{self.side}] 夹爪闭合")
                    else:
                        self._controller.open_grip(self.side, wait=False)
                        self._gripper_open = True
                        print(f"[{self.side}] 夹爪张开")
                self._space_prev = space_pressed

                # --- 7. 手臂切换（'1'，边沿触发）---
                key1_pressed = self._is_pressed("1")
                if key1_pressed and not self._key1_prev:
                    old_side = self.side
                    self.side = "left" if self.side == "right" else "right"
                    print(f"切换手臂: {old_side} → {self.side}（虚拟位姿需按 K 同步）")
                    self.current_pressed["1"] = False
                    # 切换手臂后使虚拟位姿失效，等待 K 同步
                    self._virtual_pose = None
                self._key1_prev = key1_pressed

                # --- 8. K 键：启停切换 + 从机器人同步虚拟位姿 ---
                key_k_pressed = self._is_pressed("k")
                if key_k_pressed and not self._key_k_prev:
                    self._enabled = not self._enabled
                    status = "启用" if self._enabled else "暂停"
                    # 同步虚拟位姿
                    ee = self._controller.get_ee_pose(self.side)
                    if ee is not None:
                        self._virtual_pose = np.array(ee, dtype=float)
                        print(f"[K] 控制{status}，虚拟位姿已同步: "
                              f"xyz={[round(v,4) for v in self._virtual_pose[:3]]} "
                              f"rpy={[round(v,4) for v in self._virtual_pose[3:]]}")
                    else:
                        print(f"[K] 控制{status}（同步失败）")
                    self.current_pressed["k"] = False
                self._key_k_prev = key_k_pressed

                # --- 9. 频率控制（输入采样频率）---
                elapsed = time.perf_counter() - loop_start
                sleep_time = self.poll_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception:
            self._controller.get_logger().error(
                "KeyboardEEController loop crashed", exc_info=True
            )
            raise
        finally:
            if self._key_listener is not None:
                self._key_listener.stop()
            print(f"\n键盘+鼠标 EE 控制已停止 (共 {frame_count} 帧)")

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _print_banner(self):
        """打印控制台操作提示。"""
        deg = lambda rad: f"{np.degrees(rad):.1f}°"
        print(f"\n{'=' * 60}")
        print(f"  键盘+鼠标 EE 控制")
        print(f"{'=' * 60}")
        print(f"  手臂:       {self.side}（按 1 切换）")
        print(f"  平移步长:   {self.translation_step * 1000:.1f} mm/tick")
        print(f"  旋转步长:   {self.rotation_step:.4f} rad/tick  ({deg(self.rotation_step)})")
        print(f"  鼠标灵敏度: {self.mouse_sensitivity:.4f} rad/pixel")
        print(f"  输入采样:   {self.poll_hz} Hz")
        print(f"  信号发送:   {self.send_hz} Hz")
        print(f"  轨迹时长:   {self.duration:.3f} s")
        print(f"  最大位移:   {self.max_delta_xyz*1000:.0f} mm / {self.max_delta_rpy:.3f} rad")
        mouse_info = "归位" if self._mouse_center is not None else "自由"
        print(f"  鼠标模式:   {mouse_info}")
        safety = "禁用" if not self._controller.enable_safety_check else "启用"
        print(f"  安全检查:   {safety}")
        print(f"{'=' * 60}")
        print(f"  EE 局部坐标系（相对夹爪）: x=下方, y=左方, z=前方")
        print(f"  W/S:       夹爪前后 (z)")
        print(f"  A/D:       夹爪左右 (y)")
        print(f"  Q/E:       基座坐标系上下")
        print(f"  R/T:       滚转 roll (绕夹爪前方 z, R=左滚 T=右滚)")
        print(f"  鼠标 X:    偏航 yaw (绕夹爪下方 x)")
        print(f"  鼠标 Y:    俯仰 pitch (绕夹爪左方 y)")
        print(f"  ↑↓←→:      同鼠标（粗调，可叠加）")
        print(f"  空格:      夹爪开合切换")
        print(f"  1:         切换左右臂")
        print(f"  K:         启停切换 + 同步虚拟位姿")
        print(f"  ESC:       退出")
        print(f"{'=' * 60}\n")


# ============================================================================
# CLI 入口
# ============================================================================


def main():
    parser = argparse.ArgumentParser(
        description="Walker S2 键盘+鼠标末端执行器遥操作",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python keyboard_ee_control.py
  python keyboard_ee_control.py --side left
  python keyboard_ee_control.py --translation-step 0.005 --send-hz 50
  python keyboard_ee_control.py --poll-hz 100 --send-hz 20
        """,
    )
    parser.add_argument(
        "--side", default="right", choices=["left", "right"],
        help="初始控制手臂 (默认: right)",
    )
    parser.add_argument(
        "--translation-step", type=float, default=DEFAULT_TRANSLATION_STEP,
        help=f"平移步长，单位 m/tick (默认: {DEFAULT_TRANSLATION_STEP})",
    )
    parser.add_argument(
        "--rotation-step", type=float, default=DEFAULT_ROTATION_STEP,
        help=f"键盘旋转步长，单位 rad/tick (默认: {DEFAULT_ROTATION_STEP})",
    )
    parser.add_argument(
        "--mouse-sensitivity", type=float, default=DEFAULT_MOUSE_SENSITIVITY,
        help=f"鼠标灵敏度，单位 rad/pixel (默认: {DEFAULT_MOUSE_SENSITIVITY})",
    )
    parser.add_argument(
        "--poll-hz", type=float, default=DEFAULT_POLL_HZ,
        help=f"键盘/鼠标输入采样频率 (默认: {DEFAULT_POLL_HZ})",
    )
    parser.add_argument(
        "--send-hz", type=float, default=DEFAULT_SEND_HZ,
        help=f"控制信号发送频率 (默认: {DEFAULT_SEND_HZ})",
    )
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION,
        help=f"每帧轨迹时长，单位 s (默认: {DEFAULT_DURATION})",
    )
    parser.add_argument(
        "--max-delta-xyz", type=float, default=DEFAULT_MAX_DELTA_XYZ,
        help=f"单次发送最大平移 (m)，默认 {DEFAULT_MAX_DELTA_XYZ}",
    )
    parser.add_argument(
        "--max-delta-rpy", type=float, default=DEFAULT_MAX_DELTA_RPY,
        help=f"单次发送最大旋转 (rad)，默认 {DEFAULT_MAX_DELTA_RPY}",
    )
    parser.add_argument(
        "--sync-interval", type=float, default=DEFAULT_SYNC_INTERVAL,
        help=f"全量同步实际 EE 位姿的间隔，单位 s (默认: {DEFAULT_SYNC_INTERVAL})",
    )
    parser.add_argument(
        "--no-safety-check", action="store_true",
        help="禁用关节速度安全检查（遥操作场景下操作员直接监控，可放宽限制）",
    )
    parser.add_argument(
        "--quiet", "-q", action="store_true",
        help="静默模式，抑制控制器 INFO 日志（仅 WARN/ERROR）",
    )
    args = parser.parse_args()

    # --- ROS2 初始化（遵循 pick_part.py / carry_box.py 模式）---
    rclpy.init()
    controller = WalkerS2Controller(
        enable_ik=True,
        subscribe_images=False,
        enable_safety_check=not args.no_safety_check,
    )
    if args.quiet:
        controller.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if not controller.wait_for_state(timeout=5.0):
        print("ERROR: 等待机器人状态超时，请检查 bridge 连接")
        controller.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        return

    print("机器人状态已连接，启动键盘+鼠标控制...")

    kb_ctrl = KeyboardEEController(
        controller=controller,
        side=args.side,
        translation_step=args.translation_step,
        rotation_step=args.rotation_step,
        mouse_sensitivity=args.mouse_sensitivity,
        poll_hz=args.poll_hz,
        send_hz=args.send_hz,
        duration=args.duration,
        max_delta_xyz=args.max_delta_xyz,
        max_delta_rpy=args.max_delta_rpy,
        sync_interval=args.sync_interval,
    )

    try:
        kb_ctrl.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
