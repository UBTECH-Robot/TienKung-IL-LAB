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

        # Ready pose first (safety flow), then place the apple at the taught
        # spot and replay the demonstration.
        self.go_ready()
        self.spin_for(0.5)
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
