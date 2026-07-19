#!/usr/bin/env python3
"""Walker C1 pick-place task over ROS SDK topics (Tienkung-style).

Flow (user requirement: start from the reset.py ready pose, never touch the
table):

  reset sim -> go_ready (staged, = reset.py) -> command apple to a known spot
  -> palm-down approach ABOVE the apple -> descend to grasp height (palm z
  safety floor keeps fingers off the table) -> close -> lift -> carry over the
  plate -> lower -> open -> back to ready -> success check.

All arm waypoints are BASE-frame palm poses with an explicit palm-down
attitude solved by ikpy (position + orientation), so the hand approaches at a
controlled angle instead of drifting into the table.

Run (ROS side, sim stack up):
  source /opt/ros/humble/setup.bash
  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
  export ROS_DOMAIN_ID=146
  /usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/pick_place_controller.py
"""
from __future__ import annotations

import argparse
import sys
import time

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


# ── task geometry (world frame, from the parlor scene) ──
APPLE_SPAWN_W = (8.21, 5.90, 0.95)     # dropped slightly above the tabletop
PLATE_CENTER_W = (8.374, 6.046, 0.925)
APPLE_RADIUS = 0.022

# Palm axis for the grasp phases: the PROVEN attitude's palm normal (from the
# successful in-process trajectory) — tilted ~33 deg, NOT straight down. The
# tilted cage cradles the ball against the palm corner; a straight-down cage
# leaves the ball hanging over the open bottom gap and it ratchets out.
GRASP_PALM_AXIS = (0.4528, 0.2907, -0.8429)
SUCCESS_DIST = 0.12                     # Tienkung uses 0.12 m

# All grasp-phase waypoints lock the FULL palm attitude to the proven cage
# orientation (ready pose + wrist rolled -90deg, computed via FK at startup):
# a Z-axis-only constraint leaves the hand yaw free, so the finger cage lands
# at a different rotation every run and the close misses.

# Palm-origin offset relative to the APPLE CENTER at the proven grasp
# (measured from successful trajectory 1784105817): the palm sits BEHIND and
# BESIDE the apple — the fingers reach past it — not above it.
PALM_GRASP_OFFSET = (-0.075, -0.069, 0.042)

# Base-frame heights (base sits ~3mm above the tabletop):
HOVER_Z = 0.20          # approach altitude above the table
LIFT_Z = 0.18
CARRY_Z = 0.20
RELEASE_Z = 0.12


class WalkerC1PickPlace(WalkerC1RobotController):
    def __init__(self):
        super().__init__(node_name="walker_c1_pick_place")

    def world_to_base(self, pos_w) -> np.ndarray:
        root = self.object_state.get("robot_root_pose_w")
        if not root:
            raise RuntimeError("no robot_root_pose_w yet (is the sim bridge up?)")
        w, x, y, z = root[3:7]
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        return rot.T @ (np.array(pos_w, dtype=float) - np.array(root[:3]))

    def run_task(self, randomize: bool = False) -> bool:
        rng = np.random.default_rng()

        self.get_logger().info("waiting for robot state ...")
        if not self.wait_for_state():
            self.get_logger().error("no /mc/sdk/robot_state; is the stack running?")
            return False

        # 1. Ready pose first (user safety requirement — same as reset.py).
        self.get_logger().info("going to ready pose (staged) ...")
        self.go_ready()
        self.spin_for(1.0)

        # 2. Place the apple at a known spot (sim-only; on the real robot the
        #    apple is put at the agreed position by hand / perception).
        apple_w = list(APPLE_SPAWN_W)
        if randomize:
            apple_w[0] += float(rng.uniform(-0.03, 0.01))
            apple_w[1] += float(rng.uniform(-0.05, 0.01))
        self.get_logger().info(f"placing apple at world {np.round(apple_w,3).tolist()}")
        self.set_object_world_pos(*apple_w)
        self.spin_for(1.5)  # let it settle on the table

        apple_b = self.object_pos_in_base()
        if apple_b is None:
            self.get_logger().warn("no /sim/object_state; falling back to commanded position")
            apple_b = self.world_to_base(apple_w)
        self.get_logger().info(f"apple in base frame: {np.round(apple_b,3).tolist()}")

        # Palm waypoints = apple/plate position + the proven palm offset.
        gx = float(apple_b[0]) + PALM_GRASP_OFFSET[0]
        gy = float(apple_b[1]) + PALM_GRASP_OFFSET[1]
        grasp_z = float(apple_b[2]) + 0.030
        rot = self.grasp_attitude

        # 3+4. Grasp with verify-and-retry: a single attempt lands ~50-60%
        # (contact chaos); re-trying against the apple's current position
        # multiplies the episode success rate.
        held = False
        trace = []
        for attempt in range(3):
            # Pre-shape the fingers into the proven open cage ([0.2]x6) BEFORE
            #    approaching, then palm-down approach and vertical descend.
            self.move_hand("right", [0.2] * 6)
            self.get_logger().info("approach above apple ...")
            if not self.move_right_arm([gx, gy, HOVER_Z], rot, duration=2.5):
                return False
            self.get_logger().info("descend to grasp height ...")
            if not self.move_right_arm([gx, gy, grasp_z], palm_axis=GRASP_PALM_AXIS, duration=2.5):
                return False
            self.spin_for(0.5)

            # Closed-loop mouth-over-apple alignment (the mechanism that made the
            # in-process grasp reliable): measure the actual cage-mouth center and
            # nudge the palm until the apple sits in it.
            palm_xy = np.array([gx, gy], dtype=float)
            for _ in range(4):
                mouth = self.mouth_center_w()
                ob = self.object_state.get("object_pos_w")
                if mouth is None or ob is None:
                    self.get_logger().warn("no hand-link state; skipping mouth alignment")
                    break
                err = np.array(ob[:2]) - mouth[:2]
                self.get_logger().info(f"mouth->apple xy err: {np.round(err*1000,0).tolist()} mm")
                if float(np.linalg.norm(err)) < 0.008:
                    break
                palm_xy = palm_xy + err
                if not self.move_right_arm([palm_xy[0], palm_xy[1], grasp_z], palm_axis=GRASP_PALM_AXIS, duration=0.8):
                    break
            self.spin_for(0.3)
            # The aligned palm xy is the new anchor for every hold-phase waypoint;
            # lifting back to the PRE-alignment xy yanks the ball out of the cage.
            gx, gy = float(palm_xy[0]), float(palm_xy[1])

            # 4. Close, verify nothing exploded, lift.
            self.get_logger().info("closing hand (staged, soft-contact) ...")
            # Staged close: with stiffness-25 fingers a single deep command
            # slams the ball off the table (force = k * command deficit).
            # Small increments keep the deficit — and thus the contact force —
            # gentle all the way in, while the final depth still enjoys the
            # stiff hold.
            final_close = [0.7, 0.9, 0.95, 0.95, 0.95, 0.95]
            start_close = [0.2] * 6
            for step in range(1, 11):
                t = step / 10.0
                self.move_hand("right", [(1 - t) * a + t * b for a, b in zip(start_close, final_close)], repeats=2)
                self.spin_for(0.15)
            self.spin_for(0.8)
            hand_now = [round(float(self.hand_pos.get(n, -9)), 3) for n in
                        ("right_thumb_swing", "right_thumb_mcp", "right_index_mcp",
                         "right_middle_mcp", "right_ring_mcp", "right_little_mcp")]
            self.get_logger().info(f"hand achieved vs cmd [0.7,0.85,0.8,0.8,0.8,0.8]: {hand_now}")
            self.get_logger().info("lift ...")
            import threading
            trace.clear()
            tracing = [True]
            def _tracer():
                while tracing[0]:
                    ob = self.object_pos_in_base()
                    if ob is not None:
                        trace.append(round(float(ob[2]), 3))
                    time.sleep(0.2)
            th = threading.Thread(target=_tracer, daemon=True); th.start()
            # Two-stage lift with an IN-FLIGHT regrip: raise just enough to get
            # the apple off the table (deep-closing on the table squeezes the
            # apple out, but once airborne the same deep close only wraps it),
            # tighten, then continue up.
            self.move_right_arm([gx, gy, grasp_z + 0.025], palm_axis=GRASP_PALM_AXIS, duration=2.0, corrections=0)
            self.spin_for(0.3)
            self.move_right_arm([gx, gy, LIFT_Z], palm_axis=GRASP_PALM_AXIS, duration=3.5, corrections=0)
            self.spin_for(0.5)
            tracing[0] = False; th.join(timeout=1.0)
            self.get_logger().info(f"apple z trace during lift: {trace}")

            lifted = self.object_pos_in_base()
            if lifted is not None:
                held = float(lifted[2]) > float(apple_b[2]) + 0.05
                self.get_logger().info(
                    f"lift check: attempt={attempt + 1} apple z {apple_b[2]:.3f} -> "
                    f"{lifted[2]:.3f} ({'HELD' if held else 'NOT HELD'})"
                )
            if held:
                break
            ob_w = self.object_state.get("object_pos_w")
            if ob_w is None or ob_w[2] < 0.5:  # world z: fell off the table
                self.get_logger().warn("apple fell off the table; aborting attempts")
                break
            self.get_logger().info("grasp missed; reopening and retrying ...")
            self.move_hand("right", [0.2] * 6)
            self.spin_for(0.5)
            fresh = self.object_pos_in_base()
            if fresh is not None:
                apple_b = fresh
            gx = float(apple_b[0]) + PALM_GRASP_OFFSET[0]
            gy = float(apple_b[1]) + PALM_GRASP_OFFSET[1]
            grasp_z = float(apple_b[2]) + 0.030

        # Lock the cage attitude for the carry: capture the palm rotation the
        # arm naturally settled into after the lift (mode-Z free yaw rotates
        # the cage during long lateral moves and rolls the apple out).
        carry_rot = self.fk_palm()[:3, :3]

        # 5. Carry over the plate, lower, release.
        plate_b = self.world_to_base(PLATE_CENTER_W)
        px = float(plate_b[0]) + PALM_GRASP_OFFSET[0]
        py = float(plate_b[1]) + PALM_GRASP_OFFSET[1]
        self.get_logger().info("carry over plate ...")
        self.move_right_arm([px, py, CARRY_Z], rot_mat=carry_rot, duration=6.0, corrections=0)
        self.get_logger().info("lower ...")
        self.move_right_arm([px, py, RELEASE_Z], rot_mat=carry_rot, duration=3.0, corrections=0)
        self.get_logger().info("release ...")
        self.open_hand("right")
        self.spin_for(1.0)
        self.move_right_arm([px - 0.05, py - 0.05, CARRY_Z], rot, duration=1.5)

        # 6. Back to ready.
        self.get_logger().info("back to ready ...")
        self.go_ready()
        self.spin_for(1.0)

        # 7. Success check (sim: apple within SUCCESS_DIST of the plate center).
        final_b = self.object_pos_in_base()
        if final_b is None:
            self.get_logger().warn("no object state for the success check")
            return True
        dist = float(np.linalg.norm(np.array(final_b[:2]) - np.array(plate_b[:2])))
        ok = dist <= SUCCESS_DIST and final_b[2] > 0.0
        self.get_logger().info(
            f"final: apple-plate horizontal dist {dist:.3f} m (limit {SUCCESS_DIST}) -> "
            f"{'SUCCESS' if ok else 'FAILURE'}"
        )
        return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Walker C1 ROS pick-place task")
    parser.add_argument("--randomize", action="store_true", help="Randomize the apple spot")
    parser.add_argument("--episodes", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = WalkerC1PickPlace()
    ok_count = 0
    try:
        for ep in range(args.episodes):
            node.get_logger().info(f"=== episode {ep + 1}/{args.episodes} ===")
            if node.run_task(randomize=args.randomize):
                ok_count += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done: {ok_count}/{args.episodes} success")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
