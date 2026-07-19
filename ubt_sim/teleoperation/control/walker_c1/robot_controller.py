#!/usr/bin/env python3
"""Walker C1 / Astron ROS-side robot controller (Tienkung-style).

Talks ONLY the SDK topic surface, so the same code drives the simulator
(through the walker_c1 ROS2-ZMQ bridge, ROS_DOMAIN_ID=146) and, later, the
real robot (ROS_DOMAIN_ID=0):

  pub  /mc/sdk/robot_command      body joint position commands
  pub  /mc/{left,right}_hand/command   SDK 6-joint hand commands
  sub  /mc/sdk/robot_state        body joint feedback
  sub  /mc/{left,right}_hand/joint_states
  (sim only) pub /sim/cmd_reset, /sim/cmd_set_object_pose
  (sim only) sub /sim/object_state

IK/FK: ikpy over the trimmed right_arm.urdf (base_link -> R_palm_link chain,
waist joints locked at zero). Position AND orientation are solved together —
the palm attitude at every waypoint is explicit, which is what keeps the
fingers off the table.

Safety: move_right_arm() clamps the target so the palm never goes below
MIN_PALM_Z_BASE (table height + finger clearance, in base frame).

Dependencies: rclpy, ikpy, numpy (ROS-side Python 3.10).
"""
from __future__ import annotations

import json
import os
import time
import warnings
from typing import Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    from ikpy.chain import Chain

try:
    from .constants import (
        RIGHT_ARM_JOINT_NAMES,
        TASK_RESET_ARM_CLEAR_POSE,
        TASK_RESET_BODY_POSE,
        TASK_RESET_ELBOW_CLEAR_POSE,
        TASK_RESET_LEFT_HAND_POSE,
        TASK_RESET_RIGHT_HAND_POSE,
    )
except ImportError:
    from constants import (
        RIGHT_ARM_JOINT_NAMES,
        TASK_RESET_ARM_CLEAR_POSE,
        TASK_RESET_BODY_POSE,
        TASK_RESET_ELBOW_CLEAR_POSE,
        TASK_RESET_LEFT_HAND_POSE,
        TASK_RESET_RIGHT_HAND_POSE,
    )

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RIGHT_ARM_URDF = os.path.join(_THIS_DIR, "right_arm.urdf")

RIGHT_HAND_SDK_NAMES = [
    "right_thumb_swing", "right_thumb_mcp", "right_index_mcp",
    "right_middle_mcp", "right_ring_mcp", "right_little_mcp",
]
LEFT_HAND_SDK_NAMES = [
    "left_thumb_swing", "left_thumb_mcp", "left_index_mcp",
    "left_middle_mcp", "left_ring_mcp", "left_little_mcp",
]

# Table top is at world z=0.897 and the robot base sits at world z=0.90, so
# the tabletop is roughly z~0 in base frame. Fingers extend up to ~10cm below
# the palm at palm-down attitude; keep the palm above table + clearance.
MIN_PALM_Z_BASE = 0.050

# Palm-down = palm z-axis (the palm normal, +y-ish at the ready pose) pointing
# straight down in base frame.
PALM_DOWN_AXIS = (0.0, 0.0, -1.0)


def rpy_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class WalkerC1RobotController(Node):
    """High-level motion primitives over the C1 SDK topics."""

    def __init__(self, node_name: str = "walker_c1_robot_controller",
                 urdf_path: str = DEFAULT_RIGHT_ARM_URDF):
        super().__init__(node_name)

        self.body_pub = self.create_publisher(RobotCommand, "/mc/sdk/robot_command", 10)
        self.left_hand_pub = self.create_publisher(JointCommand, "/mc/left_hand/command", 10)
        self.right_hand_pub = self.create_publisher(JointCommand, "/mc/right_hand/command", 10)
        self.reset_pub = self.create_publisher(Bool, "/sim/cmd_reset", 1)
        self.object_pose_pub = self.create_publisher(Point, "/sim/cmd_set_object_pose", 1)

        self.joint_pos: dict[str, float] = {}
        self.hand_pos: dict[str, float] = {}
        self.object_state: dict = {}
        self.create_subscription(RobotState, "/mc/sdk/robot_state", self._state_cb, 10)
        self.create_subscription(String, "/sim/object_state", self._object_cb, 10)
        self.create_subscription(JointState, "/mc/right_hand/joint_states",
                                 lambda m: self.hand_pos.update(zip(m.name, m.position)), 10)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.chain = Chain.from_urdf_file(urdf_path, base_elements=["base_link"])
        self.link_names = [link.name for link in self.chain.links]
        self.arm_link_idx = [self.link_names.index(name) for name in RIGHT_ARM_JOINT_NAMES]
        # Only the 7 arm joints are active for the IK; waist stays locked.
        mask = [False] * len(self.chain.links)
        for idx in self.arm_link_idx:
            mask[idx] = True
        self.chain.active_links_mask = np.array(mask)
        self._arm_bounds = [self.chain.links[i].bounds for i in self.arm_link_idx]
        self._last_cmd_arm: Optional[list[float]] = None

        # The proven grasp attitude, extracted from a SUCCESSFUL in-process
        # trajectory (dataset 1784105817, FK of the arm angles one frame
        # before the hand closed). Palm normal ~(0.45, 0.29, -0.84): down
        # with a slight forward tilt. Do not re-derive this analytically —
        # "ready + wrist roll" gives a different (wrong) attitude because the
        # arm keeps reorienting after the roll.
        self.grasp_attitude = np.array([
            [0.8080, -0.3770, 0.4528],
            [-0.5335, -0.7943, 0.2907],
            [0.2501, -0.4764, -0.8429],
        ])

    # ── state ──
    def _state_cb(self, msg: RobotState) -> None:
        self.joint_pos.update(zip(msg.joint_states.name, msg.joint_states.position))

    def _object_cb(self, msg: String) -> None:
        try:
            self.object_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.joint_pos for name in RIGHT_ARM_JOINT_NAMES):
                return True
        return False

    def current_arm(self) -> list[float]:
        return [float(self.joint_pos.get(name, 0.0)) for name in RIGHT_ARM_JOINT_NAMES]

    # ── kinematics ──
    def _q_vector(self, arm: Sequence[float]) -> np.ndarray:
        q = np.zeros(len(self.chain.links))
        for idx, val in zip(self.arm_link_idx, arm):
            q[idx] = val
        return q

    def fk_palm(self, arm: Optional[Sequence[float]] = None) -> np.ndarray:
        """4x4 palm pose in BASE frame for the given (or current) arm angles."""
        return self.chain.forward_kinematics(self._q_vector(arm or self.current_arm()))

    def _clamp_to_bounds(self, arm: Sequence[float]) -> list[float]:
        out = []
        for val, bounds in zip(arm, self._arm_bounds):
            lo, hi = bounds if isinstance(bounds, tuple) else (None, None)
            if lo is not None:
                val = max(float(lo) + 1e-4, val)
            if hi is not None:
                val = min(float(hi) - 1e-4, val)
            out.append(float(val))
        return out

    def solve_ik(self, pos_base: Sequence[float], rot_mat: Optional[np.ndarray] = None,
                 palm_axis: Optional[Sequence[float]] = None,
                 seed_arm: Optional[Sequence[float]] = None) -> Optional[list[float]]:
        """Solve IK. Seeding: last COMMANDED arm (chained solutions converge
        much better than measured feedback, which carries tracking error and
        can sit epsilon outside the URDF bounds ikpy enforces)."""
        target = np.array(pos_base, dtype=float)
        if target[2] < MIN_PALM_Z_BASE:
            self.get_logger().warn(
                f"IK target z={target[2]:.3f} below safety floor {MIN_PALM_Z_BASE}; clamping."
            )
            target[2] = MIN_PALM_Z_BASE
        base_seed = seed_arm or self._last_cmd_arm or self.current_arm()
        seed = self._q_vector(self._clamp_to_bounds(base_seed))
        kwargs = {}
        if rot_mat is not None:
            kwargs = {"target_orientation": np.asarray(rot_mat), "orientation_mode": "all"}
        elif palm_axis is not None:
            kwargs = {"target_orientation": list(palm_axis), "orientation_mode": "Z"}
        try:
            q = self.chain.inverse_kinematics(target, initial_position=seed, **kwargs)
        except Exception as exc:
            self.get_logger().error(f"IK failed: {exc}")
            return None
        arm = [float(q[idx]) for idx in self.arm_link_idx]
        reached = self.chain.forward_kinematics(q)[:3, 3]
        err = float(np.linalg.norm(reached - target))
        if err > 0.02:
            self.get_logger().warn(f"IK position residual {err:.3f} m for target {np.round(target,3)}")
        return arm

    # ── command primitives ──
    def _publish_arm(self, arm: Sequence[float]) -> None:
        msg = RobotCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, pos in zip(RIGHT_ARM_JOINT_NAMES, arm):
            cmd = JointCmd()
            cmd.name = name
            cmd.control_mode = JointCmd.MODE_POSITION
            cmd.position = float(pos)
            msg.joint_cmd.append(cmd)
        self.body_pub.publish(msg)

    def publish_body_pose(self, pose: dict[str, float]) -> None:
        msg = RobotCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, pos in pose.items():
            cmd = JointCmd()
            cmd.name = name
            cmd.control_mode = JointCmd.MODE_POSITION
            cmd.position = float(pos)
            msg.joint_cmd.append(cmd)
        self.body_pub.publish(msg)

    def move_right_arm_joints(self, target_arm: Sequence[float], duration: float = 2.0,
                              hz: float = 20.0) -> None:
        """Ramp the right arm from its current angles to target over duration."""
        start = self._last_cmd_arm or self.current_arm()
        steps = max(int(duration * hz), 1)
        for i in range(steps):
            t = (i + 1) / steps
            arm = [(1 - t) * a + t * b for a, b in zip(start, target_arm)]
            self._publish_arm(arm)
            self.spin_for(1.0 / hz)
        self._last_cmd_arm = list(target_arm)

    def move_right_arm(self, pos_base: Sequence[float], rot_mat: Optional[np.ndarray] = None,
                       palm_axis: Optional[Sequence[float]] = None, duration: float = 2.0,
                       corrections: int = 3, tol: float = 0.008) -> bool:
        """IK to a base-frame palm pose, ramp there, then close the loop:
        measure the FK error (gravity sag / tracking lag) and re-command a
        virtually offset target until the palm is within tol. This is the ROS
        equivalent of the in-process lesson 'integrate the correction on the
        COMMAND' — open-loop position IK alone leaves ~2cm of sag."""
        target = np.array(pos_base, dtype=float)
        arm = self.solve_ik(target, rot_mat=rot_mat, palm_axis=palm_axis)
        if arm is None:
            return False
        self.move_right_arm_joints(arm, duration=duration)

        virtual = target.copy()
        for _ in range(max(corrections, 0)):
            self.spin_for(0.3)
            reached = self.fk_palm()[:3, 3]
            err = target - reached
            if float(np.linalg.norm(err)) < tol:
                break
            virtual = virtual + err
            arm = self.solve_ik(virtual, rot_mat=rot_mat, palm_axis=palm_axis)
            if arm is None:
                break
            self.move_right_arm_joints(arm, duration=0.6)

        reached = self.fk_palm()[:3, 3]
        self.get_logger().info(
            f"palm target {np.round(target,3).tolist()} -> "
            f"reached {np.round(reached,3).tolist()} "
            f"(err {float(np.linalg.norm(target-reached))*1000:.0f} mm)"
        )
        return True

    def move_hand(self, side: str, sdk_positions: Sequence[float], repeats: int = 5) -> None:
        names = RIGHT_HAND_SDK_NAMES if side == "right" else LEFT_HAND_SDK_NAMES
        pub = self.right_hand_pub if side == "right" else self.left_hand_pub
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.names = list(names)
        msg.position = [float(v) for v in sdk_positions]
        msg.mode = [2] * len(names)
        for _ in range(repeats):
            pub.publish(msg)
            self.spin_for(0.05)

    def open_hand(self, side: str) -> None:
        self.move_hand(side, [0.0] * 6)

    def close_hand(self, side: str, grip: float = 0.8) -> None:
        self.move_hand(side, [grip] * 6)

    # ── task-level helpers ──
    def go_ready(self, stage_duration: float = 2.0, hz: float = 20.0) -> None:
        """Staged move to the grasp-ready pose (same semantics as reset.py)."""
        self.open_hand("left")
        self.open_hand("right")
        for pose in (TASK_RESET_ELBOW_CLEAR_POSE, TASK_RESET_ARM_CLEAR_POSE, TASK_RESET_BODY_POSE):
            merged = dict(TASK_RESET_BODY_POSE)
            merged.update(pose)
            end = time.time() + stage_duration
            while time.time() < end:
                self.publish_body_pose(merged)
                self.spin_for(1.0 / hz)
        self.move_hand("left", TASK_RESET_LEFT_HAND_POSE)
        self.move_hand("right", TASK_RESET_RIGHT_HAND_POSE)
        self._last_cmd_arm = [TASK_RESET_BODY_POSE[n] for n in RIGHT_ARM_JOINT_NAMES]

    def reset_sim(self) -> None:
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)

    def set_object_world_pos(self, x: float, y: float, z: float) -> None:
        self.object_pose_pub.publish(Point(x=float(x), y=float(y), z=float(z)))

    def mouth_center_w(self) -> Optional[np.ndarray]:
        """Live cage-mouth center (world), same formula as the proven
        in-process servo: midpoint of the four fingertip links' mean and the
        thumb tip. Sim-only (uses /sim/object_state link poses)."""
        links = self.object_state.get("right_hand_links_w") or {}
        fingers = [links.get(f"R_{f}_ip_link") for f in ("index", "middle", "ring", "little")]
        thumb = links.get("R_thumb_ip_link")
        if any(v is None for v in fingers) or thumb is None:
            return None
        wall = np.mean(np.array(fingers, dtype=float), axis=0)
        return 0.5 * (wall + np.array(thumb, dtype=float))

    def object_pos_in_base(self) -> Optional[np.ndarray]:
        """Object position in base frame (sim only, from /sim/object_state)."""
        obj = self.object_state.get("object_pos_w")
        root = self.object_state.get("robot_root_pose_w")
        if not obj or not root:
            return None
        pos = np.array(obj) - np.array(root[:3])
        w, x, y, z = root[3:7]
        # rotate world offset into base frame (inverse of root quaternion)
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        return rot.T @ pos
