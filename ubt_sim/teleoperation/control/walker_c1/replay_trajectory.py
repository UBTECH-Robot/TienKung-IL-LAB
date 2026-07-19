#!/usr/bin/env python3
"""Replay a recorded Walker C1 trajectory (HDF5) through the ROS SDK topics.

Reads the ``action/*`` streams of a dataset trajectory (the same schema the
collectors write) and republishes them frame-by-frame at the recorded cadence
via /mc/sdk/robot_command and /mc/{left,right}_hand/command. Works against the
simulator (ROS_DOMAIN_ID=146) and, unchanged, the real robot (=0).

Also a decisive experiment: if replaying a known-successful in-process
trajectory reproduces the grasp through the ROS channel, the channel is
sound and any remaining gap lives in waypoint choreography.

Run:
  /usr/bin/python3 replay_trajectory.py /ubt_sim/dataset/walker_c1/<ts>/trajectory.hdf5
"""
from __future__ import annotations

import argparse
import sys
import time

import h5py
import numpy as np
import rclpy

try:
    from .robot_controller import WalkerC1RobotController
except ImportError:
    import os

    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from robot_controller import WalkerC1RobotController

DEFAULT_APPLE_W = (8.21, 5.90, 0.95)


class WalkerC1Replayer(WalkerC1RobotController):
    def __init__(self):
        super().__init__(node_name="walker_c1_replayer")

    def replay(self, h5_path: str, rate_scale: float = 1.0, apple_w=None) -> None:
        with h5py.File(h5_path, "r") as f:
            arm_r = f["action/arm_right_position_align/data"][:]
            arm_l = f["action/arm_left_position_align/data"][:]
            hand_r = f["action/end_effector_right_position_align/data"][:]
            hand_l = f["action/end_effector_left_position_align/data"][:]
            ts = f["observations/timestamp"][:]

        n = arm_r.shape[0]
        dts = np.diff(ts)
        dts = np.clip(dts, 0.005, 0.2) / rate_scale
        self.get_logger().info(
            f"replaying {n} frames from {h5_path} (mean dt {float(dts.mean())*1000:.0f} ms)"
        )

        if not self.wait_for_state():
            self.get_logger().error("no robot state; is the stack up?")
            return

        if apple_w is not None:
            self.set_object_world_pos(*apple_w)
            self.spin_for(1.5)

        from constants import RIGHT_ARM_JOINT_NAMES  # noqa: F401 (names below)

        left_names = [n.replace("R_", "L_") for n in
                      ("R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
                       "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint",
                       "R_wrist_roll_joint")]

        for i in range(n):
            body = dict(zip(
                ("R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
                 "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint",
                 "R_wrist_roll_joint"), [float(v) for v in arm_r[i]]))
            body.update(dict(zip(left_names, [float(v) for v in arm_l[i]])))
            self.publish_body_pose(body)
            self.move_hand("right", [float(v) for v in hand_r[i]], repeats=1)
            self.move_hand("left", [float(v) for v in hand_l[i]], repeats=1)
            if i % 50 == 0:
                ob = self.object_state.get("object_pos_w")
                if ob:
                    self.get_logger().info(f"frame {i}/{n} apple z(world) {ob[2]:.3f}")
            # Pace by SIM steps: the recording took one frame every 3
            # physics steps; feed it back at exactly that cadence.
            self.wait_sim_steps(3, timeout=5.0)

        self.spin_for(1.0)
        ob = self.object_state.get("object_pos_w")
        if ob:
            self.get_logger().info(f"final apple world pos: {np.round(ob, 3).tolist()}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a C1 trajectory over ROS")
    parser.add_argument("h5", help="Path to trajectory.hdf5")
    parser.add_argument("--rate", type=float, default=1.0, help="Playback speed scale")
    parser.add_argument("--apple", type=float, nargs=3, default=list(DEFAULT_APPLE_W),
                        help="World xyz to place the apple before replaying (match the recording!)")
    args = parser.parse_args()

    rclpy.init()
    node = WalkerC1Replayer()
    try:
        node.replay(args.h5, rate_scale=args.rate, apple_w=args.apple)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
