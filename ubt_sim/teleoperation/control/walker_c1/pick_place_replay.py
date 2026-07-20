#!/usr/bin/env python3
"""Fixed-position pick-place via teach-and-repeat (the primary mode).

Replays a known-successful demonstration trajectory (HDF5) through the ROS
SDK topics, paced by simulation steps, for the FIXED apple position it was
recorded at. This is classic industrial teach-and-repeat: deterministic and
verified end-to-end (grasp -> lift -> carry -> place, 4.8cm from the plate
center in the acceptance experiment).

The IK-choreographed pick_place_controller.py remains the bonus track for
randomized positions.

Run (episodes back-to-back with success stats):
  /usr/bin/python3 pick_place_replay.py --episodes 5
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy

try:
    from .replay_trajectory import WalkerC1Replayer
except ImportError:
    import os

    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from replay_trajectory import WalkerC1Replayer

# The taught demonstration and the apple spot it was recorded with.
DEFAULT_TRAJECTORY = "/ubt_sim/dataset/walker_c1/1784105817/trajectory.hdf5"
TAUGHT_APPLE_W = (8.207, 5.877, 0.95)
PLATE_CENTER_W = (8.374, 6.046)
SUCCESS_DIST = 0.12


class WalkerC1PickPlaceReplay(WalkerC1Replayer):
    def run_episode(self, h5_path: str) -> bool:
        if not self.wait_for_state():
            self.get_logger().error("no robot state; is the stack running?")
            return False

        # NOTE: scene resets are DISABLED by default. Empirically on this
        # Isaac stack (fabric + CPU pipeline) the SECOND and later env resets
        # progressively corrupt contact behavior: with resets the replay went
        # success, then deterministic failure ever after; without resets it
        # succeeded back-to-back. Episodes are made repeatable by teleporting
        # the apple + pre-posing the arm instead.
        if getattr(self, "use_scene_reset", False):
            self.reset_sim()
            self.wait_sim_steps(120, timeout=15.0)

        # Teach-and-repeat rule: the replay must start from the DEMO'S OWN
        # first frame (the recording begins at the home hold and ramps to
        # ready itself). Starting anywhere else makes frame 0 a violent snap
        # and the whole run diverges — verified: from-HOME replay succeeds,
        # from-READY replay fails 5/5.
        import h5py
        with h5py.File(h5_path, "r") as f:
            arm_r0 = [float(v) for v in f["action/arm_right_position_align/data"][0]]
            arm_l0 = [float(v) for v in f["action/arm_left_position_align/data"][0]]
            hand_r0 = [float(v) for v in f["action/end_effector_right_position_align/data"][0]]
            hand_l0 = [float(v) for v in f["action/end_effector_left_position_align/data"][0]]
        right_names = ("R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
                       "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint",
                       "R_wrist_roll_joint")
        pre = dict(zip(right_names, arm_r0))
        pre.update(dict(zip([n.replace("R_", "L_") for n in right_names], arm_l0)))
        # Ramp there smoothly over ~2 sim-seconds, then settle.
        cur = {n: float(self.joint_pos.get(n, 0.0)) for n in pre}
        for i in range(1, 41):
            t = i / 40.0
            self.publish_body_pose({n: (1 - t) * cur[n] + t * pre[n] for n in pre})
            self.wait_sim_steps(5, timeout=5.0)
        self.move_hand("right", hand_r0)
        self.move_hand("left", hand_l0)
        self.wait_sim_steps(50, timeout=10.0)
        self.replay(h5_path, apple_w=TAUGHT_APPLE_W)

        ob = self.object_state.get("object_pos_w")
        if not ob:
            self.get_logger().warn("no object state for the success check")
            return False
        dist = float(np.hypot(ob[0] - PLATE_CENTER_W[0], ob[1] - PLATE_CENTER_W[1]))
        ok = dist <= SUCCESS_DIST and ob[2] > 0.9
        self.get_logger().info(
            f"episode result: apple=({ob[0]:.3f},{ob[1]:.3f},{ob[2]:.3f}) "
            f"plate_dist={dist:.3f} (limit {SUCCESS_DIST}) -> {'SUCCESS' if ok else 'FAILURE'}"
        )
        return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="C1 fixed-position pick-place (teach-and-repeat)")
    parser.add_argument("--trajectory", default=DEFAULT_TRAJECTORY)
    parser.add_argument("--episodes", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = WalkerC1PickPlaceReplay()
    ok = 0
    try:
        for ep in range(args.episodes):
            node.get_logger().info(f"=== episode {ep + 1}/{args.episodes} ===")
            if node.run_episode(args.trajectory):
                ok += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done: {ok}/{args.episodes} success")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
