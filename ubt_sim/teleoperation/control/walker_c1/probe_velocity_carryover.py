#!/usr/bin/env python3
"""Diagnostic: does residual joint velocity carry over between pick-place
episodes and explain the multi-episode degradation?

Runs 3 episodes back-to-back on the SAME sim process (no stack restart), and
before/after each episode prints the max |joint velocity| across the right
hand joints (the low-stiffness=10 fingers, most likely to still be jittering
after a forceful grasp).
"""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy

try:
    from .pick_place_replay import WalkerC1PickPlaceReplay, DEFAULT_TRAJECTORY
except ImportError:
    import os

    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from pick_place_replay import WalkerC1PickPlaceReplay, DEFAULT_TRAJECTORY

RIGHT_HAND_NAMES = (
    "R_thumb_cmp_joint", "R_thumb_mpp_joint", "R_thumb_ip_joint",
    "R_index_mpp_joint", "R_index_ip_joint",
    "R_middle_mpp_joint", "R_middle_ip_joint",
    "R_ring_mpp_joint", "R_ring_ip_joint",
    "R_little_mpp_joint", "R_little_ip_joint",
)
RIGHT_ARM_NAMES = (
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
)


WAIST_HEAD_NAMES = ("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
                    "head_yaw_joint", "head_pitch_joint")
LEG_NAMES = ("L_hip_pitch_joint", "L_hip_roll_joint", "L_hip_yaw_joint",
             "R_hip_pitch_joint", "R_hip_roll_joint", "R_hip_yaw_joint")


def report_velocity(node, label: str) -> None:
    node.spin_for(0.3)
    names = node.object_state.get("joint_names")
    vel = node.object_state.get("joint_vel_probe")
    # POSITION is the primary signal here: is anything drifting from HOME?
    waist_head_pos = {n: round(float(node.joint_pos.get(n, -9)), 4) for n in WAIST_HEAD_NAMES}
    leg_pos = {n: round(float(node.joint_pos.get(n, -9)), 4) for n in LEG_NAMES}
    right_arm_pos = {n: round(float(node.joint_pos.get(n, -9)), 4) for n in RIGHT_ARM_NAMES}
    node.get_logger().info(f"[{label}] waist/head POS (want ~0): {waist_head_pos}")
    node.get_logger().info(f"[{label}] leg POS (want home ~0/0.08): {leg_pos}")
    node.get_logger().info(f"[{label}] right_arm POS: {right_arm_pos}")
    if not names or not vel:
        node.get_logger().warn(f"[{label}] no velocity probe data yet")
        return
    vmap = dict(zip(names, vel))
    hand_v = {n: round(float(vmap.get(n, 0.0)), 4) for n in RIGHT_HAND_NAMES}
    arm_v = {n: round(float(vmap.get(n, 0.0)), 4) for n in RIGHT_ARM_NAMES}
    max_hand = max(abs(v) for v in hand_v.values())
    max_arm = max(abs(v) for v in arm_v.values())
    node.get_logger().info(f"[{label}] max|hand_vel|={max_hand:.4f} rad/s max|arm_vel|={max_arm:.4f} rad/s")


def main() -> int:
    rclpy.init()
    node = WalkerC1PickPlaceReplay()
    try:
        if not node.wait_for_state():
            node.get_logger().error("no robot state")
            return 1
        report_velocity(node, "startup")
        for ep in range(3):
            node.get_logger().info(f"=== episode {ep + 1}/3 ===")
            report_velocity(node, f"before-ep{ep + 1}")
            ok = node.run_episode(DEFAULT_TRAJECTORY)
            node.get_logger().info(f"episode {ep + 1} -> {'SUCCESS' if ok else 'FAILURE'}")
            report_velocity(node, f"immediately-after-ep{ep + 1}")
            # Watch decay over a few seconds of sim time (holding position).
            for t_wait in (1.0, 3.0, 5.0):
                node.wait_sim_steps(int(t_wait * 100), timeout=15.0)
                report_velocity(node, f"after-ep{ep + 1}+{t_wait:.0f}s-settle")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
