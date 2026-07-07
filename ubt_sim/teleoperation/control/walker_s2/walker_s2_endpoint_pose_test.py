#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 末端/TCP 位姿打印脚本。

Walker 控制脚本中的 IK EE 点按 TCP 使用。本脚本打印 IK/FK TCP 位姿，
以及仿真 world frame 中的 sixforce 与 finger link 位姿，方便对照调试。
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

try:
    from .walker_s2_controller import WalkerS2Controller
except ImportError:
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from walker_s2_controller import WalkerS2Controller


SIXFORCE_LINKS = {
    "left": "L_sixforce_link",
    "right": "R_sixforce_link",
}
FINGER_PREFIX = {
    "left": "L_finger",
    "right": "R_finger",
}

ROBOT_WORLD_POS = (0.7, -0.2, 0.9)
ROBOT_WORLD_ROT_WXYZ = (0.7071068, 0.0, 0.0, 0.7071068)
DEFAULT_TARGET_WORLD_POS = (1.00213, 0.50822, 1.13042)
IK_KWARGS = {
    "max_iter": 200,
    "pos_tol": 1e-2,
    "rot_tol": 5e-2,
    "rot_weight": 0.2,
    "rot_axis_weights": (0.2, 0.2, 1.0),
    "null_weight": 0.1,
    "unlock_waist": False,
}


def _fmt(values, precision=5):
    if values is None:
        return "None"
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def _link_pose_text(name, pose):
    if pose is None:
        return f"{name}: None"
    return f"{name}: pos={_fmt(pose.get('pos'))}, rot(wxyz)={_fmt(pose.get('rot'))}"


def _mean_xyz(poses):
    valid = [pose.get("pos") for pose in poses if pose and pose.get("pos") is not None]
    if not valid:
        return None
    n = float(len(valid))
    return [sum(float(pos[i]) for pos in valid) / n for i in range(3)]


def _quat_wxyz_to_matrix(q):
    w, x, y, z = [float(v) for v in q]
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm <= 0.0:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _world_xyz_to_base(xyz):
    rot = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
    delta = [float(xyz[i]) - ROBOT_WORLD_POS[i] for i in range(3)]
    return [sum(rot[row][col] * delta[row] for row in range(3)) for col in range(3)]


class WalkerS2EndpointPoseTest(WalkerS2Controller):
    """测试节点：打印当前 IK TCP 与仿真夹爪 link 位姿，可选移动 TCP。"""

    def print_endpoint_poses(self, side="right", timeout=5.0):
        if side not in SIXFORCE_LINKS:
            raise ValueError(f"Invalid side '{side}', expected left or right")

        ok = self.wait_for_state(timeout=timeout)
        if not ok:
            return False
        self.wait_for_grip_state(side, timeout=2.0)
        self.wait_for_finger_link_states(timeout=timeout)

        sixforce_link = SIXFORCE_LINKS[side]
        ee_pose_base = self.get_ee_pose(side)
        print("\n=== IK/FK TCP pose (URDF base frame) ===")
        print(f"{sixforce_link} / TCP: xyzrpy={_fmt(ee_pose_base)}")

        states = self.get_finger_link_states() or {}
        links = states.get("links") or {}
        print("\n=== Sim link poses (world frame) ===")
        print(_link_pose_text(sixforce_link, links.get(sixforce_link)))

        prefix = FINGER_PREFIX[side]
        finger_items = sorted(
            (name, pose)
            for name, pose in links.items()
            if name.startswith(prefix) and "link" in name.lower()
        )
        if not finger_items:
            print(f"No {side} finger link poses found in /sim/finger_link_states")
        else:
            print(f"\n--- {side} finger / gripper links ---")
            for name, pose in finger_items:
                print(_link_pose_text(name, pose))
            gripper_center = _mean_xyz([pose for _, pose in finger_items])
            print(f"\nObserved {side} gripper center: pos={_fmt(gripper_center)} (mean of finger link positions)")

        grip_state = self.get_grip_state(side)
        if grip_state is not None:
            print("\n=== ECAT two-finger grip state ===")
            print(grip_state)
        return True

    def move_ee_to_world_pos(self, side="right", world_pos=None, duration_sec=2.0):
        if side != "right":
            raise ValueError("move_ee_to_world_pos currently supports the right arm only")
        world_pos = DEFAULT_TARGET_WORLD_POS if world_pos is None else world_pos
        current = self.get_ee_pose(side)
        if current is None:
            self.get_logger().error("No current EE pose available")
            return False
        target_base_xyz = _world_xyz_to_base(world_pos)
        target_xyzrpy = [float(v) for v in target_base_xyz] + [float(v) for v in current[3:]]
        self.get_logger().info(
            f"Move IK TCP ({SIXFORCE_LINKS[side]}) to world pos={_fmt(world_pos)} "
            f"=> base target xyzrpy={_fmt(target_xyzrpy)}"
        )
        return self.move_arm_ik(
            side,
            target_xyzrpy,
            duration_sec=duration_sec,
            wait=True,
            require_success=True,
            **IK_KWARGS,
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Print Walker S2 arm endpoint and gripper link poses")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--move-target-world",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_TARGET_WORLD_POS,
        help="world-frame target position for the IK TCP before printing again",
    )
    parser.add_argument("--duration", type=float, default=2.0, help="movement duration in seconds")
    parser.add_argument("--no-move", action="store_true", help="only print current poses")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = WalkerS2EndpointPoseTest(
        node_name="walker_s2_endpoint_pose_test_node",
        enable_ik=True,
        subscribe_images=False,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        time.sleep(0.5)
        ok = node.print_endpoint_poses(side=args.side, timeout=args.timeout)
        if not ok:
            node.get_logger().error("Failed to read Walker S2 endpoint poses")
            sys.exit(1)
        if not args.no_move:
            if not node.move_ee_to_world_pos(
                side=args.side,
                world_pos=args.move_target_world,
                duration_sec=args.duration,
            ):
                node.get_logger().error("Failed to move EE to requested world position")
                sys.exit(1)
            time.sleep(0.5)
            print("\n=== After moving to requested world position ===")
            if not node.print_endpoint_poses(side=args.side, timeout=args.timeout):
                node.get_logger().error("Failed to read Walker S2 endpoint poses after move")
                sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
