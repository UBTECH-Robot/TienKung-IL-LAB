#!/usr/bin/env python3
"""分步初始化 Walker S2 机器人到预备姿态（robot ready pose）。

调用 robot_control.py 中的 move_to_ready_pose(staged=True)，
按 4 个阶段安全地移动双臂到预备抓取位姿。

阶段：
  1a/3  肩 yaw + elbow yaw 预定位
  1b/3  肩 pitch + elbow roll + 腕 pitch
  2/3   肩 pitch 回到预备姿态
  3/3   执行完整 READY_POSE

Usage:
  python3 robotready
  python3 robotready --duration 25.0
  python3 robotready --no-wait
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

logger = logging.getLogger("robotready")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--duration", type=float, default=20.0,
        help="分步初始化总时长（秒），默认 20.0",
    )
    parser.add_argument(
        "--no-wait", action="store_true",
        help="不等待到位收敛（默认会检查并补偿未到位的关节）",
    )
    parser.add_argument(
        "--settle-tolerance", type=float, default=0.03,
        help="到位误差阈值（rad），默认 0.03",
    )
    parser.add_argument(
        "--no-safety", action="store_true",
        help="禁用安全速度检查（不推荐）",
    )
    parser.add_argument(
        "--no-limits", action="store_true",
        help="禁用关节限位裁剪（不推荐）",
    )
    parser.add_argument(
        "--hz", type=int, default=500,
        help="身体控制发布频率（Hz），默认 500",
    )
    parser.add_argument(
        "--pvt", action="store_true",
        help="启用 PVT 力位混合模式 (mode=7)：速度前馈 + 可调 Kp/Kd",
    )
    parser.add_argument(
        "--pvt-kp", type=float, default=None,
        help="PVT 位置增益 Kp（标量），默认保守值 50.0",
    )
    parser.add_argument(
        "--pvt-kd", type=float, default=None,
        help="PVT 速度增益 Kd（标量），默认保守值 2.0",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _print_state(controller, label: str = "当前关节状态") -> None:
    """打印所有身体关节位置。"""
    pos = controller.get_current_position()
    if pos is None:
        print("[WARN] 无关节状态数据")
        return

    names = controller.joint_names
    locked = controller.lock_joints
    print(f"\n{'='*64}")
    print(f"  {label}")
    print(f"{'='*64}")
    for i, name in enumerate(names):
        lock_flag = " [LOCKED]" if name in locked else ""
        print(f"  [{i:2d}] {name:30s} = {pos[i]:+8.4f} rad{lock_flag}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    # ── Path setup: add robot_control to sys.path ─────────────────────────
    _repo_root = Path(__file__).resolve().parents[4]
    _robot_control_path = str(_repo_root / "ubt_IL/walker/walker_sdk_ros2/robot_control")
    if _robot_control_path not in sys.path:
        sys.path.insert(0, _robot_control_path)

    from robot_control import RobotController, DEFAULT_LOCK_JOINTS

    # ── ROS2 init ──────────────────────────────────────────────────────────
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()

    if args.pvt:
        print("=" * 64)
        print("⚠️  PVT (mode=7) 力位混合模式启用：速度前馈 + 可调 Kp/Kd")
        pvt_kp_val = args.pvt_kp if args.pvt_kp is not None else 50.0
        pvt_kd_val = args.pvt_kd if args.pvt_kd is not None else 2.0
        print(f"    Kp={pvt_kp_val}, Kd={pvt_kd_val}")
        print("=" * 64)

    controller = RobotController(
        lock_joints=list(DEFAULT_LOCK_JOINTS),
        enable_safety_check=not args.no_safety,
        enable_limit_check=not args.no_limits,
        control_hz=args.hz,
        use_pvt=args.pvt,
        pvt_kp=args.pvt_kp,
        pvt_kd=args.pvt_kd,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    return_code = 1

    try:
        # ── Wait for robot state ──────────────────────────────────────────
        if not controller.wait_for_state(timeout=10.0):
            print("\n[FATAL] 未收到机器人状态，请检查：")
            print("  1. 运控是否启动 (rosa run t800_mc_server start_mc_client)")
            print("  2. SDK 控制器是否切换 (switch_controller config_mc_walker_s2_v1_sps)")
            print("  3. DDS 中间件是否为 CycloneDDS")
            return 1

        # ── Print initial state ───────────────────────────────────────────
        _print_state(controller, "初始关节状态")

        # ── Confirm ───────────────────────────────────────────────────────
        print(f"\n即将开始分步初始化（总时长 ~{args.duration:.1f}s）：")
        print("  1a/3  肩 yaw + elbow yaw 预定位")
        print("  1b/3  肩 pitch + elbow roll + 腕 pitch")
        print("  2/3   肩 pitch 回到预备姿态")
        print("  3/3   完整 READY_POSE")
        print()
        try:
            input("按回车开始（Ctrl+C 取消）...")
        except EOFError:
            print("[WARN] 非交互终端，直接开始")

        # ── Execute staged initialization ─────────────────────────────────
        logger.info(
            "开始分步初始化，总时长=%.1fs, settle_check=%s",
            args.duration,
            not args.no_wait,
        )

        success = controller.move_to_ready_pose(
            duration_sec=args.duration,
            staged=True,
            wait=True,
        )

        if success:
            # 等待最终到位收敛
            # move_to_ready_pose 内部已有 settle_check，这里再额外等待一下确保稳定
            time.sleep(0.5)
            _print_state(controller, "初始化完成 - 最终关节状态")
            print("\n✓ 机器人已到达预备姿态")
            return_code = 0
        else:
            print("\n✗ 分步初始化失败：部分阶段未完成到位")
            _print_state(controller, "失败时的关节状态")
            return_code = 1

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
        return_code = 1

    finally:
        controller.stop()
        time.sleep(0.1)
        executor.remove_node(controller)
        controller.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
