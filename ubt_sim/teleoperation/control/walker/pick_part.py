#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 简化零件抓取脚本。

IK 控制目标已经是夹爪 TCP 点，不再从 force sensor 额外外推 finger/TCP
虚拟抓取点偏移。轨迹点由人工指定为零件 world pose 的若干 TCP 位置偏移，TCP 姿态
由当前姿态或命令行指定的 base-frame RPY 控制。
"""

import argparse
import json
import os
import sys
import threading
import time
from copy import deepcopy

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 支持直接运行和包导入两种方式
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from walker_s2_controller import BODY_JOINT_LIMITS, READY_POSE, WalkerS2Controller


DEFAULT_PART_STATES_TOPIC = "/sim/part_states"
DEFAULT_RANDOMIZE_PARTS_TOPIC = "/sim/cmd_randomize_parts"
DEFAULT_PART_NAME = "part_a_red"
DEFAULT_PART_SEQUENCE = ("part_a_ori", "part_a_red", "part_b_blue", "part_b_ori")
DEFAULT_APPROACH_OFFSET_WORLD = (0.0, 0.0, 0.12)
DEFAULT_DESCEND_OFFSET_WORLD = (0.0, 0.0, 0.035)
DEFAULT_LIFT_OFFSET_WORLD = (0.0, 0.0, 0.14)
DEFAULT_ROT_WEIGHT = 0.10
DEFAULT_POSITION_TOLERANCE = 0.01
DEFAULT_JOINT_LIMIT_MARGIN = 0.0
DEFAULT_REQUIRE_IK_OK = True
DEFAULT_AUTO_GRASP = True
DEFAULT_UNCONSTRAIN_ROT_Z = True
DEFAULT_UNLOCK_WAIST = True
DEFAULT_GRASP_RADIUS = 0.0
DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG = 10.0
DEFAULT_GRASP_LIFT_HEIGHT = 0.15
DEFAULT_GRASP_PREGRASP_HEIGHT = 0.10
DEFAULT_GRASP_TARGET_OFFSET_WORLD = (0.0, 0.0, 0.02)
DEFAULT_GRASP_DESCEND_AFTER_TARGET_WORLD = (0.0, 0.0, -0.01)
DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT = 32
DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT = 5
DEFAULT_GRASP_MAX_ATTEMPTS = 3
DEFAULT_GRASP_SUCCESS_CHECK = True
DEFAULT_GRASP_SUCCESS_MIN_LIFT_DELTA = 0.04
DEFAULT_GRASP_SUCCESS_MAX_PART_TO_EE_DIST = 0.18
DEFAULT_GRASP_SUCCESS_PART_STATE_TIMEOUT = 0.3
DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT = 0.2
DEFAULT_GRIPPER_FORWARD_AXIS_EE = (-1.0, 0.0, 0.0)

# walker_s2_part_sorting.yaml 中机器人 init_state.rot = [0.7071068, 0, 0, 0.7071068]。
# Isaac Lab 四元数顺序为 wxyz，表示机器人 base 相对 world 绕 world z 轴 +90°。
DEFAULT_BASE_IN_WORLD_POS = (0.7, -0.2, 0.9)
DEFAULT_BASE_TO_WORLD_QUAT_WXYZ = (0.7071068, 0.0, 0.0, 0.7071068)
DEFAULT_WORLD_TO_BASE_QUAT_WXYZ = (0.7071068, 0.0, 0.0, -0.7071068)
DEFAULT_RIGHT_BOX_WORLD_POS1 = (1.2, 0.3 - 0.06, 1.05)
DEFAULT_RIGHT_BOX_WORLD_POS2 = (1.2, 0.3 + 0.07, 1.05)
DEFAULT_RIGHT_BOX_WORLD_POS = DEFAULT_RIGHT_BOX_WORLD_POS1
DEFAULT_PLACE_APPROACH_HEIGHT = 0.18
DEFAULT_PLACE_RELEASE_HEIGHT = 0.10
DEFAULT_PLACE_LIFT_HEIGHT = 0.20
DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD = (-0.25, 0.0, 0.05)


class PartStateMonitor(Node):
    """缓存 /sim/part_states 的最新零件状态。"""

    def __init__(self, topic=DEFAULT_PART_STATES_TOPIC):
        super().__init__("walker_s2_part_state_monitor")
        self._part_states = None
        self._seq = 0
        self._lock = threading.Lock()
        self._received = threading.Event()
        self._updated = threading.Condition(self._lock)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            String,
            topic,
            self._part_state_callback,
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _part_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid part_states JSON: {exc}")
            return
        with self._updated:
            self._part_states = data
            self._seq += 1
            self._received.set()
            self._updated.notify_all()

    def wait_for_part_states(self, timeout=5.0):
        ok = self._received.wait(timeout=timeout)
        if not ok:
            self.get_logger().warning(f"Timeout waiting for part_states ({timeout:.1f}s)")
        return ok

    def get_part_states(self):
        with self._lock:
            return deepcopy(self._part_states)

    def get_update_seq(self):
        with self._lock:
            return self._seq

    def wait_for_new_part_states(self, last_seq, timeout=0.3):
        with self._updated:
            ok = self._updated.wait_for(lambda: self._seq > int(last_seq), timeout=timeout)
            if not ok:
                self.get_logger().warning(f"Timeout waiting for new part_states ({timeout:.2f}s)")
            return ok

    def get_part_pose(self, part_name):
        states = self.get_part_states()
        if not states:
            return None
        return (states.get("parts") or {}).get(part_name)


def _format_vec(values):
    return "[" + ", ".join(f"{float(v):+.4f}" for v in values) + "]"


def _quat_wxyz_to_matrix(quat_wxyz):
    q = np.asarray(quat_wxyz, dtype=float)
    if q.shape != (4,):
        raise ValueError(f"Quaternion must have 4 values [w,x,y,z], got shape {q.shape}")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-9:
        raise ValueError("Quaternion norm is zero")
    w, x, y, z = q / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def _matrix_to_rpy(rot):
    rot = np.asarray(rot, dtype=float)
    pitch = float(np.arctan2(-rot[2, 0], np.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])))
    if abs(np.cos(pitch)) > 1e-9:
        roll = float(np.arctan2(rot[2, 1], rot[2, 2]))
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-rot[0, 1], rot[1, 1]))
    return np.asarray([roll, pitch, yaw], dtype=float)


def _rpy_to_matrix(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def _base_x_rotation(roll):
    cr, sr = np.cos(float(roll)), np.sin(float(roll))
    return np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)


def _base_y_rotation(pitch):
    cp, sp = np.cos(float(pitch)), np.sin(float(pitch))
    return np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)


def transform_world_delta_to_base(delta_world, world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ):
    """把 world frame 位移向量旋转到 Walker URDF base frame。"""
    rot_world_to_base = _quat_wxyz_to_matrix(world_to_base_quat_wxyz)
    return rot_world_to_base @ np.asarray(delta_world, dtype=float)


def transform_world_point_to_base(
    point_world,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    """把 world frame 点坐标转换到 Walker URDF base frame。"""
    delta_world = np.asarray(point_world, dtype=float) - np.asarray(base_in_world_pos, dtype=float)
    return transform_world_delta_to_base(delta_world, world_to_base_quat_wxyz)


def transform_base_delta_to_world(delta_base, world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ):
    """把 Walker URDF base frame 位移向量旋转到 world frame。"""
    rot_world_to_base = _quat_wxyz_to_matrix(world_to_base_quat_wxyz)
    return rot_world_to_base.T @ np.asarray(delta_base, dtype=float)


def transform_base_point_to_world(
    point_base,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    """把 Walker URDF base frame 点坐标转换到 world frame。"""
    return np.asarray(base_in_world_pos, dtype=float) + transform_base_delta_to_world(point_base, world_to_base_quat_wxyz)


def _pose_axes_from_rpy(rpy):
    rot = _rpy_to_matrix(rpy)
    return rot[:, 0], rot[:, 1], rot[:, 2]


def _tcp_debug_pose(
    tcp_base,
    tcp_rpy_base,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    """返回 TCP 在 base/world 下的位置与方向，用于检查抓取几何。"""
    tcp_base = np.asarray(tcp_base, dtype=float)
    tcp_rpy_base = np.asarray(tcp_rpy_base, dtype=float)
    rot_base = _rpy_to_matrix(tcp_rpy_base)
    forward_axis_ee = _normalize(gripper_forward_axis_ee)
    tcp_forward_base = rot_base @ forward_axis_ee
    tcp_x_base, tcp_y_base, tcp_z_base = _pose_axes_from_rpy(tcp_rpy_base)
    return {
        "tcp_base": tcp_base,
        "tcp_world": transform_base_point_to_world(tcp_base, base_in_world_pos, world_to_base_quat_wxyz),
        "tcp_rpy_base": tcp_rpy_base,
        "tcp_x_base": tcp_x_base,
        "tcp_y_base": tcp_y_base,
        "tcp_z_base": tcp_z_base,
        "tcp_x_world": transform_base_delta_to_world(tcp_x_base, world_to_base_quat_wxyz),
        "tcp_y_world": transform_base_delta_to_world(tcp_y_base, world_to_base_quat_wxyz),
        "tcp_z_world": transform_base_delta_to_world(tcp_z_base, world_to_base_quat_wxyz),
        "tcp_forward_base": tcp_forward_base,
        "tcp_forward_world": transform_base_delta_to_world(tcp_forward_base, world_to_base_quat_wxyz),
    }


def _log_grasp_debug(controller, label, debug_pose, prefix=""):
    controller.get_logger().info(
        f"{prefix}{label}: tcp_world={_format_vec(debug_pose['tcp_world'])}, "
        f"tcp_base={_format_vec(debug_pose['tcp_base'])}, "
        f"tcp_rpy_base={_format_vec(debug_pose['tcp_rpy_base'])}"
    )
    controller.get_logger().info(
        f"{prefix}{label}: tcp_axes_world x={_format_vec(debug_pose['tcp_x_world'])}, "
        f"y={_format_vec(debug_pose['tcp_y_world'])}, z={_format_vec(debug_pose['tcp_z_world'])}"
    )
    controller.get_logger().info(
        f"{prefix}{label}: tcp_axes_base x={_format_vec(debug_pose['tcp_x_base'])}, "
        f"y={_format_vec(debug_pose['tcp_y_base'])}, z={_format_vec(debug_pose['tcp_z_base'])}"
    )
    controller.get_logger().info(
        f"{prefix}{label}: tcp_forward_world={_format_vec(debug_pose['tcp_forward_world'])}, "
        f"tcp_forward_base={_format_vec(debug_pose['tcp_forward_base'])}"
    )


def _log_actual_ee_debug(
    controller,
    side,
    label,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    radius=DEFAULT_GRASP_RADIUS,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    _ = radius  # 兼容旧参数；当前 IK 末端已经是 TCP，不再额外外推。
    ee_pose = controller.get_ee_pose(side)
    if ee_pose is None:
        controller.get_logger().warning(f"actual {label}: no {side} EE pose available")
        return
    ee_pose = np.asarray(ee_pose, dtype=float)
    debug_pose = _tcp_debug_pose(
        ee_pose[:3],
        ee_pose[3:],
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )
    _log_grasp_debug(controller, label, debug_pose, prefix="actual ")


def _limit_violations(joint_targets, margin=0.0):
    violations = []
    for name, value in (joint_targets or {}).items():
        if name not in BODY_JOINT_LIMITS:
            continue
        lo, hi = BODY_JOINT_LIMITS[name]
        value = float(value)
        if value < lo + margin or value > hi - margin:
            violations.append((name, value, lo, hi))
    return violations


def _normalize(vec, eps=1e-9):
    vec = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm <= eps:
        raise ValueError(f"Cannot normalize near-zero vector: {vec}")
    return vec / norm


def _ik_pos_err(diagnostics, side, ik_ok=False):
    side_diag = (diagnostics or {}).get(side) or {}
    pos_err = side_diag.get("pos_err")
    if pos_err is not None:
        return float(pos_err)
    return 0.0 if ik_ok else float("inf")


def _ik_rot_err(diagnostics, side, ik_ok=False):
    side_diag = (diagnostics or {}).get(side) or {}
    rot_err = side_diag.get("rot_err")
    if rot_err is not None:
        return float(rot_err)
    return 0.0 if ik_ok else float("inf")


def _joint_limit_proximity_penalty(joint_targets):
    max_penalty = 0.0
    for name, value in (joint_targets or {}).items():
        if name not in BODY_JOINT_LIMITS:
            continue
        lo, hi = BODY_JOINT_LIMITS[name]
        half_range = 0.5 * (float(hi) - float(lo))
        if half_range <= 1e-9:
            continue
        center = 0.5 * (float(lo) + float(hi))
        max_penalty = max(max_penalty, abs(float(value) - center) / half_range)
    return max_penalty


def _target_rpy_from_args(
    current_ee_pose,
    ee_rpy_deg=None,
    ee_rpy_delta_deg=None,
    tilt_base_x_deg=None,
    tilt_base_y_deg=None,
):
    if ee_rpy_deg is not None:
        return np.deg2rad(np.asarray(ee_rpy_deg, dtype=float))

    current_rpy = np.asarray(current_ee_pose[3:], dtype=float)
    rot = _rpy_to_matrix(current_rpy)
    if tilt_base_x_deg is not None:
        rot = _base_x_rotation(np.deg2rad(float(tilt_base_x_deg))) @ rot
    if tilt_base_y_deg is not None:
        rot = _base_y_rotation(np.deg2rad(float(tilt_base_y_deg))) @ rot
    if ee_rpy_delta_deg is not None:
        rot = _rpy_to_matrix(np.deg2rad(np.asarray(ee_rpy_delta_deg, dtype=float))) @ rot
    return _matrix_to_rpy(rot)


def _basis_from_forward(forward, up_hint):
    forward = _normalize(forward)
    up_hint = _normalize(up_hint)
    side = np.cross(up_hint, forward)
    if np.linalg.norm(side) <= 1e-6:
        fallback = np.asarray([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(fallback, forward))) > 0.9:
            fallback = np.asarray([0.0, 1.0, 0.0], dtype=float)
        side = np.cross(fallback, forward)
    side = _normalize(side)
    up = _normalize(np.cross(forward, side))
    return np.column_stack([forward, side, up])


def _rpy_for_gripper_forward_world(
    forward_world,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    forward_base = transform_world_delta_to_base(forward_world, world_to_base_quat_wxyz)
    z_down_hint_base = transform_world_delta_to_base((0.0, 0.0, -1.0), world_to_base_quat_wxyz)
    target_basis = _basis_from_forward(forward_base, z_down_hint_base)
    source_basis = _basis_from_forward(gripper_forward_axis_ee, (0.0, 0.0, 1.0))
    return _matrix_to_rpy(target_basis @ source_basis.T)


def _sample_grasp_directions_world(
    side,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    azimuth_count=DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT,
    elevation_count=DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT,
    min_table_angle_deg=DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG,
):
    min_elevation = np.deg2rad(float(min_table_angle_deg))
    max_elevation = 0.5 * np.pi
    elevations = np.linspace(min_elevation, max_elevation, max(1, int(elevation_count)))
    azimuths = np.linspace(0.0, 2.0 * np.pi, max(1, int(azimuth_count)), endpoint=False)
    directions = []
    for elevation in elevations:
        z = float(np.sin(elevation))
        xy = float(np.cos(elevation))
        for azimuth in azimuths:
            direction = np.asarray([xy * np.cos(azimuth), xy * np.sin(azimuth), z], dtype=float)
            direction_base = transform_world_delta_to_base(direction, world_to_base_quat_wxyz)
            if side == "right" and direction_base[1] >= 0.0:
                continue
            if side == "left" and direction_base[1] <= 0.0:
                continue
            directions.append(direction)
    return directions


def _score_grasp_candidate(candidate, current_ee_pose=None):
    stages = candidate["stages"]
    max_pos_err = max(stage["pos_err"] for stage in stages)
    max_rot_err = max(stage["rot_err"] for stage in stages)
    joint_penalty = max(_joint_limit_proximity_penalty(stage["joint_targets"]) for stage in stages)
    motion_penalty = 0.0
    if current_ee_pose is not None:
        motion_penalty = float(np.linalg.norm(candidate["ee_base"] - np.asarray(current_ee_pose[:3], dtype=float)))
    elevation_penalty = float(candidate["direction_world"][2])
    robot_side_reward = float(candidate.get("robot_side_alignment", 0.0))
    tcp_z_down_penalty = 1.0 - float(candidate.get("tcp_z_world_down_alignment", 0.0))
    xy_rot_balance_penalty = float(candidate.get("xy_rot_balance_penalty", 0.0))
    return (
        100.0 * max_pos_err
        + max_rot_err
        + 0.05 * joint_penalty
        + 0.2 * motion_penalty
        + 0.4 * elevation_penalty
        + 2.0 * tcp_z_down_penalty
        + 0.2 * xy_rot_balance_penalty
        - 0.4 * robot_side_reward
    )


def _solve_ee_target(
    controller,
    side,
    label,
    target_pose,
    rot_weight,
    unlock_waist,
    joint_limit_margin,
    seed_joint_targets=None,
    unconstrain_rot_z=False,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
):
    seed_names = list(seed_joint_targets.keys()) if seed_joint_targets is not None else None
    seed_positions = list(seed_joint_targets.values()) if seed_joint_targets is not None else None
    joint_targets, ik_ok, diagnostics = controller.solve_arm_ik(
        side,
        target_pose,
        rot_weight=rot_weight,
        rot_axis_weights=(1.0, 1.0, 0.0) if unconstrain_rot_z else None,
        unlock_waist=unlock_waist,
        joint_names=seed_names,
        joint_positions=seed_positions,
    )
    pos_err = _ik_pos_err(diagnostics, side, ik_ok=ik_ok)
    rot_err = _ik_rot_err(diagnostics, side, ik_ok=ik_ok)
    violations = _limit_violations(joint_targets, margin=joint_limit_margin)
    position_tolerance = float(position_tolerance)
    return {
        "label": label,
        "pose": np.asarray(target_pose, dtype=float),
        "joint_targets": joint_targets,
        "ik_ok": bool(ik_ok),
        "diagnostics": diagnostics,
        "pos_err": pos_err,
        "rot_err": rot_err,
        "position_ok": pos_err <= position_tolerance,
        "position_tolerance": position_tolerance,
        "violations": violations,
    }


def _solve_right_ee_target(
    controller,
    label,
    target_pose,
    rot_weight,
    unlock_waist,
    joint_limit_margin,
    seed_joint_targets=None,
    unconstrain_rot_z=False,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
):
    return _solve_ee_target(
        controller,
        "right",
        label,
        target_pose,
        rot_weight,
        unlock_waist,
        joint_limit_margin,
        seed_joint_targets=seed_joint_targets,
        unconstrain_rot_z=unconstrain_rot_z,
        position_tolerance=position_tolerance,
    )


def _require_stage_labels(stage_map, labels, logger, context):
    missing = [label for label in labels if label not in stage_map]
    if missing:
        logger.error(f"Missing {context} stages: {missing}; available={sorted(stage_map.keys())}")
        return False
    return True


def _default_place_box_world_pos_for_part(part_name):
    """A 类零件放入箱内位置 1，B 类零件放入箱内位置 2。"""
    name = str(part_name).lower()
    if name.startswith("part_b") or "_b_" in name:
        return DEFAULT_RIGHT_BOX_WORLD_POS2
    return DEFAULT_RIGHT_BOX_WORLD_POS1


def solve_place_waypoints(
    controller,
    side,
    box_world_pos,
    ee_rpy_base,
    rot_weight,
    unlock_waist,
    joint_limit_margin,
    seed_joint_targets=None,
    unconstrain_rot_z=False,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    exit_left_offset_world=DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD,
    place_approach_height=DEFAULT_PLACE_APPROACH_HEIGHT,
    place_release_height=DEFAULT_PLACE_RELEASE_HEIGHT,
    place_lift_height=DEFAULT_PLACE_LIFT_HEIGHT,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
):
    """求解搬运到箱子上方、松爪后抬升并按偏移离开箱体的 waypoint。"""
    box_world_pos = np.asarray(box_world_pos, dtype=float)
    lift_world = box_world_pos + np.asarray([0.0, 0.0, float(place_lift_height)], dtype=float)
    waypoints = (
        ("place_approach", box_world_pos + np.asarray([0.0, 0.0, float(place_approach_height)], dtype=float)),
        ("place_release", box_world_pos + np.asarray([0.0, 0.0, float(place_release_height)], dtype=float)),
        ("place_lift", lift_world),
        ("place_exit_left", lift_world + np.asarray(exit_left_offset_world, dtype=float)),
    )
    stages = []
    seed = seed_joint_targets
    for label, target_world in waypoints:
        target_base = transform_world_point_to_base(target_world, base_in_world_pos, world_to_base_quat_wxyz)
        target_pose = np.concatenate([target_base, ee_rpy_base])
        stage = _solve_ee_target(
            controller,
            side,
            label,
            target_pose,
            rot_weight=rot_weight,
            unlock_waist=unlock_waist,
            joint_limit_margin=joint_limit_margin,
            seed_joint_targets=seed,
            unconstrain_rot_z=unconstrain_rot_z,
            position_tolerance=position_tolerance,
        )
        stage["target_world"] = target_world
        stage["target_base"] = target_base
        stage["debug_pose"] = _tcp_debug_pose(
            target_base,
            ee_rpy_base,
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )
        stages.append(stage)
        seed = stage["joint_targets"]
    return stages


def choose_suitable_grasp_pose(
    controller,
    part_pose,
    side="right",
    current_ee_pose=None,
    radius=DEFAULT_GRASP_RADIUS,
    lift_height=DEFAULT_GRASP_LIFT_HEIGHT,
    pregrasp_height=DEFAULT_GRASP_PREGRASP_HEIGHT,
    target_offset_world=DEFAULT_GRASP_TARGET_OFFSET_WORLD,
    descend_after_target_world=DEFAULT_GRASP_DESCEND_AFTER_TARGET_WORLD,
    min_table_angle_deg=DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG,
    azimuth_count=DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT,
    elevation_count=DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT,
    rot_weight=DEFAULT_ROT_WEIGHT,
    unconstrain_rot_z=False,
    unlock_waist=False,
    joint_limit_margin=DEFAULT_JOINT_LIMIT_MARGIN,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
    require_ik_ok=DEFAULT_REQUIRE_IK_OK,
):
    """采样 TCP 朝向，返回最佳 IK 可达抓取序列。"""
    _ = radius  # 兼容旧参数；当前 IK 末端已经是 TCP，不再额外外推。
    if side not in ("left", "right"):
        raise ValueError(f"Invalid arm side: {side}")

    part_pos = np.asarray(part_pose["pos"], dtype=float)
    target_pos = part_pos + np.asarray(target_offset_world, dtype=float)
    directions = _sample_grasp_directions_world(
        side,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        azimuth_count=azimuth_count,
        elevation_count=elevation_count,
        min_table_angle_deg=min_table_angle_deg,
    )
    robot_side_world = np.asarray(base_in_world_pos, dtype=float) - target_pos
    robot_side_world[2] = 0.0
    try:
        robot_side_world = _normalize(robot_side_world)
    except ValueError:
        robot_side_world = np.asarray([1.0, 0.0, 0.0], dtype=float)

    def direction_priority(direction_world):
        horizontal = np.asarray(direction_world, dtype=float).copy()
        horizontal[2] = 0.0
        try:
            robot_side_alignment = float(np.dot(_normalize(horizontal), robot_side_world))
        except ValueError:
            robot_side_alignment = 0.0
        elevation = float(direction_world[2])
        return (elevation, -robot_side_alignment)

    directions = sorted(directions, key=direction_priority)
    candidates = []
    fail_stats = {
        label: {
            "no_solution": 0,
            "ik_not_ok": 0,
            "pos": 0,
            "limit": 0,
            "best_pos_err": float("inf"),
            "best_rot_err": float("inf"),
        }
        for label in ("pregrasp", "grasp", "descend_after_grasp", "lift")
    }

    for direction_world in directions:
        direction_world = np.asarray(direction_world, dtype=float)
        grasp_world = target_pos
        descend_world = grasp_world + np.asarray(descend_after_target_world, dtype=float)
        pregrasp_world = grasp_world + np.asarray([0.0, 0.0, float(pregrasp_height)], dtype=float)
        lift_world = descend_world + np.asarray([0.0, 0.0, float(lift_height)], dtype=float)
        forward_world = -direction_world
        direction_base = transform_world_delta_to_base(direction_world, world_to_base_quat_wxyz)
        forward_base = transform_world_delta_to_base(forward_world, world_to_base_quat_wxyz)
        try:
            target_rpy = _rpy_for_gripper_forward_world(
                forward_world,
                gripper_forward_axis_ee=gripper_forward_axis_ee,
                world_to_base_quat_wxyz=world_to_base_quat_wxyz,
            )
        except ValueError:
            continue

        waypoints = (
            ("pregrasp", pregrasp_world),
            ("grasp", grasp_world),
            ("descend_after_grasp", descend_world),
            ("lift", lift_world),
        )
        stages = []
        seed = None
        valid = True
        for label, target_world in waypoints:
            target_base = transform_world_point_to_base(target_world, base_in_world_pos, world_to_base_quat_wxyz)
            target_pose = np.concatenate([target_base, target_rpy])
            stage = _solve_ee_target(
                controller,
                side,
                label,
                target_pose,
                rot_weight=rot_weight,
                unlock_waist=unlock_waist,
                joint_limit_margin=joint_limit_margin,
                seed_joint_targets=seed,
                unconstrain_rot_z=unconstrain_rot_z,
                position_tolerance=position_tolerance,
            )
            stage["target_world"] = target_world
            stage["target_base"] = target_base
            stage["debug_pose"] = _tcp_debug_pose(
                target_base,
                target_rpy,
                gripper_forward_axis_ee=gripper_forward_axis_ee,
                base_in_world_pos=base_in_world_pos,
                world_to_base_quat_wxyz=world_to_base_quat_wxyz,
            )
            stages.append(stage)

            stats = fail_stats[label]
            stats["best_pos_err"] = min(stats["best_pos_err"], stage["pos_err"])
            stats["best_rot_err"] = min(stats["best_rot_err"], stage["rot_err"])
            if stage["joint_targets"] is None:
                stats["no_solution"] += 1
                valid = False
                break
            if require_ik_ok and not stage["ik_ok"]:
                stats["ik_not_ok"] += 1
                valid = False
                break
            if not stage["position_ok"]:
                stats["pos"] += 1
                valid = False
                break
            if stage["violations"]:
                stats["limit"] += 1
                valid = False
                break
            seed = stage["joint_targets"]

        if not valid or len(stages) != 4:
            continue

        horizontal = np.asarray(direction_world, dtype=float).copy()
        horizontal[2] = 0.0
        try:
            robot_side_alignment = float(np.dot(_normalize(horizontal), robot_side_world))
        except ValueError:
            robot_side_alignment = 0.0
        tcp_x_world, tcp_y_world, tcp_z_world = (
            stages[0]["debug_pose"]["tcp_x_world"],
            stages[0]["debug_pose"]["tcp_y_world"],
            stages[0]["debug_pose"]["tcp_z_world"],
        )
        tcp_z_world_down_alignment = max(0.0, float(np.dot(_normalize(tcp_z_world), np.asarray([0.0, 0.0, -1.0], dtype=float))))
        xy_rot_balance_penalty = abs(float(tcp_x_world[2])) + abs(float(tcp_y_world[2]))
        candidate = {
            "side": side,
            "stages": stages,
            "direction_world": direction_world,
            "direction_base": direction_base,
            "forward_world": forward_world,
            "forward_base": forward_base,
            "robot_side_alignment": robot_side_alignment,
            "tcp_z_world_down_alignment": tcp_z_world_down_alignment,
            "xy_rot_balance_penalty": xy_rot_balance_penalty,
            "target_pos_world": target_pos,
            "pregrasp_world": pregrasp_world,
            "ee_world": grasp_world,
            "descend_world": descend_world,
            "lift_world": lift_world,
            "pregrasp_base": stages[0]["target_base"],
            "ee_base": stages[1]["target_base"],
            "descend_base": stages[2]["target_base"],
            "lift_base": stages[3]["target_base"],
            "ee_rpy_base": target_rpy,
            "sample_count": len(directions),
        }
        candidate["score"] = _score_grasp_candidate(candidate, current_ee_pose=current_ee_pose)
        candidates.append(candidate)

    if not candidates:
        controller.get_logger().error(
            "No auto grasp candidate passed filters. "
            f"samples={len(directions)}, target_pos_world={_format_vec(target_pos)}, "
            f"fail_stats={fail_stats}"
        )
        return None

    best = min(candidates, key=lambda item: item["score"])
    best["valid_count"] = len(candidates)
    return best


def _get_part_pose_or_log(part_monitor, part_name, logger):
    part_pose = part_monitor.get_part_pose(part_name)
    if part_pose is None:
        states = part_monitor.get_part_states() or {}
        available = sorted((states.get("parts") or {}).keys())
        logger.error(f"Part '{part_name}' not found. Available parts: {available}")
    return part_pose


def _current_ee_world_pos(
    controller,
    side,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
):
    ee_pose = controller.get_ee_pose(side)
    if ee_pose is None:
        return None
    ee_pose = np.asarray(ee_pose, dtype=float)
    return transform_base_point_to_world(ee_pose[:3], base_in_world_pos, world_to_base_quat_wxyz)


def _finger_grasp_reference_world(controller, side, timeout=DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT):
    if float(timeout) > 0.0 and hasattr(controller, "wait_for_finger_link_states"):
        controller.wait_for_finger_link_states(timeout=float(timeout))
    if not hasattr(controller, "get_finger_link_pose"):
        return None, "none"

    prefix = "R" if side == "right" else "L"
    candidates = [f"{prefix}_finger1_link", f"{prefix}_finger2_link"]
    points = []
    for link_name in candidates:
        pose = controller.get_finger_link_pose(link_name)
        if pose and pose.get("pos") is not None:
            points.append(np.asarray(pose["pos"], dtype=float))
    if len(points) >= 2:
        return sum(points) / float(len(points)), "finger_midpoint"
    if len(points) == 1:
        return points[0], "finger_single"

    fallback = controller.get_finger_link_pose(f"{prefix}_sixforce_link")
    if fallback and fallback.get("pos") is not None:
        return np.asarray(fallback["pos"], dtype=float), "finger_single"
    return None, "none"


def _check_grasp_success_after_lift(
    controller,
    part_monitor,
    part_name,
    part_pose_before_lift,
    side,
    min_lift_delta=DEFAULT_GRASP_SUCCESS_MIN_LIFT_DELTA,
    max_part_to_ee_dist=DEFAULT_GRASP_SUCCESS_MAX_PART_TO_EE_DIST,
    part_state_timeout=DEFAULT_GRASP_SUCCESS_PART_STATE_TIMEOUT,
    finger_timeout=DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    last_part_state_seq=None,
):
    logger = controller.get_logger()
    if last_part_state_seq is not None and float(part_state_timeout) > 0.0:
        part_monitor.wait_for_new_part_states(last_part_state_seq, timeout=float(part_state_timeout))
    after_pose = part_monitor.get_part_pose(part_name)
    if after_pose is None:
        logger.error(f"Grasp success check failed: part '{part_name}' not found after lift")
        return {
            "success": False,
            "part_pose_after": None,
            "lift_delta": float("nan"),
            "part_to_ref_dist": None,
            "ref_type": "none",
            "ref_world": None,
        }

    before_pos = np.asarray(part_pose_before_lift["pos"], dtype=float)
    after_pos = np.asarray(after_pose["pos"], dtype=float)
    lift_delta = float(after_pos[2] - before_pos[2])

    ref_world, ref_type = _finger_grasp_reference_world(controller, side, timeout=finger_timeout)
    if ref_world is None:
        ref_world = _current_ee_world_pos(controller, side, base_in_world_pos, world_to_base_quat_wxyz)
        ref_type = "ee" if ref_world is not None else "none"

    part_to_ref_dist = None
    if ref_world is not None:
        ref_world = np.asarray(ref_world, dtype=float)
        part_to_ref_dist = float(np.linalg.norm(after_pos - ref_world))

    lift_ok = lift_delta >= float(min_lift_delta)
    near_ok = part_to_ref_dist is not None and part_to_ref_dist <= float(max_part_to_ee_dist)
    success = bool(lift_ok and near_ok)
    logger.info(
        f"Grasp success check: before={_format_vec(before_pos)}, after={_format_vec(after_pos)}, "
        f"lift_delta={lift_delta:.4f}m/{float(min_lift_delta):.4f}m, "
        f"ref_type={ref_type}, ref_world={_format_vec(ref_world) if ref_world is not None else 'None'}, "
        f"part_to_ref={part_to_ref_dist if part_to_ref_dist is not None else float('nan'):.4f}m/"
        f"{float(max_part_to_ee_dist):.4f}m, success={success}"
    )
    if not success:
        reasons = []
        if not lift_ok:
            reasons.append("lift_delta_too_small")
        if not near_ok:
            reasons.append("part_not_near_gripper")
        logger.warning(f"Grasp success check failed: {reasons}")
    return {
        "success": success,
        "part_pose_after": after_pose,
        "lift_delta": lift_delta,
        "part_to_ref_dist": part_to_ref_dist,
        "ref_type": ref_type,
        "ref_world": ref_world,
    }


def _plan_grasp_stages(
    controller,
    part_pose,
    side,
    auto_grasp,
    approach_offset_world,
    descend_offset_world,
    lift_offset_world,
    ee_rpy_deg,
    ee_rpy_delta_deg,
    tilt_base_x_deg,
    tilt_base_y_deg,
    rot_weight,
    unconstrain_rot_z,
    unlock_waist,
    joint_limit_margin,
    world_to_base_quat_wxyz,
    base_in_world_pos,
    grasp_radius,
    grasp_lift_height,
    grasp_pregrasp_height,
    grasp_azimuth_count,
    grasp_elevation_count,
    gripper_forward_axis_ee,
    grasp_target_offset_world,
    grasp_descend_after_target_world,
    grasp_min_table_angle_deg,
    position_tolerance,
    require_ik_ok,
):
    current_ee_pose = controller.get_ee_pose(side)
    if current_ee_pose is None:
        controller.get_logger().error(f"No {side} EE pose available from IK")
        return None
    current_ee_pose = np.asarray(current_ee_pose, dtype=float)
    part_pos = np.asarray(part_pose["pos"], dtype=float)
    part_base = transform_world_point_to_base(part_pos, base_in_world_pos, world_to_base_quat_wxyz)
    stages = []
    candidate = None

    if auto_grasp:
        candidate = choose_suitable_grasp_pose(
            controller,
            part_pose,
            side=side,
            current_ee_pose=current_ee_pose,
            radius=grasp_radius,
            lift_height=grasp_lift_height,
            pregrasp_height=grasp_pregrasp_height,
            target_offset_world=grasp_target_offset_world,
            descend_after_target_world=grasp_descend_after_target_world,
            min_table_angle_deg=grasp_min_table_angle_deg,
            azimuth_count=grasp_azimuth_count,
            elevation_count=grasp_elevation_count,
            rot_weight=rot_weight,
            unconstrain_rot_z=unconstrain_rot_z,
            unlock_waist=unlock_waist,
            joint_limit_margin=joint_limit_margin,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
            base_in_world_pos=base_in_world_pos,
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            position_tolerance=position_tolerance,
            require_ik_ok=require_ik_ok,
        )
        if candidate is None:
            controller.get_logger().error("No IK-valid auto grasp candidate found")
            return None
        stages = candidate["stages"]
    else:
        target_rpy = _target_rpy_from_args(
            current_ee_pose,
            ee_rpy_deg,
            ee_rpy_delta_deg,
            tilt_base_x_deg,
            tilt_base_y_deg,
        )
        offsets = {
            "approach": np.asarray(approach_offset_world, dtype=float),
            "descend": np.asarray(descend_offset_world, dtype=float),
            "lift": np.asarray(lift_offset_world, dtype=float),
        }
        seed = None
        controller.get_logger().info(f"Target EE rpy base: {_format_vec(target_rpy)}")
        for label, offset in offsets.items():
            target_world = part_pos + offset
            target_base = transform_world_point_to_base(target_world, base_in_world_pos, world_to_base_quat_wxyz)
            target_pose = np.concatenate([target_base, target_rpy])
            result = _solve_ee_target(
                controller,
                side,
                label,
                target_pose,
                rot_weight=rot_weight,
                unlock_waist=unlock_waist,
                joint_limit_margin=joint_limit_margin,
                seed_joint_targets=seed,
                unconstrain_rot_z=unconstrain_rot_z,
                position_tolerance=position_tolerance,
            )
            result["target_world"] = target_world
            result["target_base"] = target_base
            result["debug_pose"] = _tcp_debug_pose(
                target_base,
                target_rpy,
                gripper_forward_axis_ee=gripper_forward_axis_ee,
                base_in_world_pos=base_in_world_pos,
                world_to_base_quat_wxyz=world_to_base_quat_wxyz,
            )
            stages.append(result)
            seed = result["joint_targets"]

    return {
        "part_pose": part_pose,
        "part_pos": part_pos,
        "part_base": part_base,
        "current_ee_pose": current_ee_pose,
        "stages": stages,
        "candidate": candidate,
    }


def _log_grasp_plan(
    controller,
    plan,
    part_name,
    side,
    rot_weight,
    unlock_waist,
    base_in_world_pos,
    world_to_base_quat_wxyz,
    gripper_forward_axis_ee,
    grasp_radius,
    attempt_index=None,
    max_attempts=None,
):
    prefix = f"Attempt {attempt_index}/{max_attempts}: " if attempt_index is not None else ""
    controller.get_logger().info(f"{prefix}Target part: {part_name}")
    controller.get_logger().info(f"{prefix}Part world pos: {_format_vec(plan['part_pos'])}")
    controller.get_logger().info(f"{prefix}Part base pos: {_format_vec(plan['part_base'])}")
    controller.get_logger().info(
        f"{prefix}Base world pos: {_format_vec(base_in_world_pos)}, "
        f"world_to_base_quat_wxyz={_format_vec(world_to_base_quat_wxyz)}"
    )
    controller.get_logger().info(f"{prefix}Current {side} EE base pose: {_format_vec(plan['current_ee_pose'])}")
    _log_actual_ee_debug(
        controller,
        side,
        f"attempt_{attempt_index}_start" if attempt_index is not None else "start",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )
    controller.get_logger().info(f"{prefix}Rot weight: {float(rot_weight):.3f}, unlock_waist={unlock_waist}")

    candidate = plan.get("candidate")
    if candidate is None:
        return
    controller.get_logger().info(
        f"{prefix}Auto grasp samples={candidate['sample_count']}, valid={candidate['valid_count']}, "
        f"score={candidate['score']:.6f}, target_pos_world={_format_vec(candidate['target_pos_world'])}, "
        f"target_pos_base={_format_vec(transform_world_point_to_base(candidate['target_pos_world'], base_in_world_pos, world_to_base_quat_wxyz))}"
    )
    controller.get_logger().info(
        f"{prefix}Auto grasp direction_world={_format_vec(candidate['direction_world'])}, "
        f"direction_base={_format_vec(candidate['direction_base'])}, "
        f"forward_world={_format_vec(candidate['forward_world'])}, "
        f"forward_base={_format_vec(candidate['forward_base'])}"
    )
    controller.get_logger().info(
        f"{prefix}Auto grasp pregrasp_world={_format_vec(candidate['pregrasp_world'])}, "
        f"pregrasp_base={_format_vec(candidate['pregrasp_base'])}, "
        f"ee_world={_format_vec(candidate['ee_world'])}, ee_base={_format_vec(candidate['ee_base'])}, "
        f"descend_world={_format_vec(candidate['descend_world'])}, descend_base={_format_vec(candidate['descend_base'])}, "
        f"lift_world={_format_vec(candidate['lift_world'])}, lift_base={_format_vec(candidate['lift_base'])}, "
        f"ee_rpy_base={_format_vec(candidate['ee_rpy_base'])}, "
        f"tcp_z_down={candidate['tcp_z_world_down_alignment']:.4f}, xy_rot_balance={candidate['xy_rot_balance_penalty']:.4f}"
    )


def _validate_stages_for_execution(controller, stages, require_ik_ok, dry_run=False):
    ok_to_execute = True
    for stage in stages:
        controller.get_logger().info(
            f"{stage['label']}: sample_world={_format_vec(stage['target_world'])}, "
            f"sample_base={_format_vec(stage['target_base'])}, pos_err={stage['pos_err']:.6f}m, "
            f"rot_err={stage['rot_err']:.6f}rad, ik_ok={stage['ik_ok']}, violations={stage['violations']}"
        )
        if "debug_pose" in stage:
            _log_grasp_debug(controller, stage["label"], stage["debug_pose"], prefix="target ")
        if stage["joint_targets"] is None:
            controller.get_logger().error(f"{stage['label']} IK failed: {stage['diagnostics']}")
            ok_to_execute = False
        if require_ik_ok and not stage["ik_ok"]:
            controller.get_logger().error(f"{stage['label']} IK did not report success: {stage['diagnostics']}")
            ok_to_execute = False
        if not stage["position_ok"]:
            controller.get_logger().error(
                f"{stage['label']} position error too large: "
                f"{stage['pos_err']:.6f}m > {stage['position_tolerance']:.6f}m"
            )
            ok_to_execute = False
        if stage["joint_targets"] is not None:
            joint_text = ", ".join(
                f"{name}={float(value):+.4f}"
                for name, value in stage["joint_targets"].items()
            )
            controller.get_logger().info(f"{stage['label']} joint_targets: {joint_text}")
        if stage["violations"]:
            for name, value, lo, hi in stage["violations"]:
                controller.get_logger().error(f"{stage['label']} would exceed {name}: {value:.4f} not in [{lo}, {hi}]")
            ok_to_execute = False

    if dry_run:
        for stage in stages:
            print(
                f"{stage['label']}: pose={stage['pose'].tolist()} "
                f"pos_err={stage['pos_err']:.6f} rot_err={stage['rot_err']:.6f} "
                f"violations={stage['violations']} diagnostics={stage['diagnostics']}"
            )
            print(f"{stage['label']}_joint_targets={stage['joint_targets']}")
    return ok_to_execute


def _stage_map_or_log(controller, stages, required_labels, context):
    stage_map = {stage["label"]: stage for stage in stages}
    if len(stage_map) != len(stages):
        labels = [stage["label"] for stage in stages]
        controller.get_logger().error(f"Duplicate stage labels found: {labels}")
        return None
    if not _require_stage_labels(stage_map, required_labels, controller.get_logger(), context):
        return None
    return stage_map


def _execute_grasp_once(
    controller,
    part_monitor,
    part_name,
    part_pose_before_lift,
    stage_map,
    side,
    auto_grasp,
    duration_per_step,
    gripper_duration,
    timeout,
    no_close_grip,
    stop_after_open,
    grasp_success_check,
    grasp_success_min_lift_delta,
    grasp_success_max_part_to_ee_dist,
    grasp_success_part_state_timeout,
    grasp_success_finger_timeout,
    base_in_world_pos,
    world_to_base_quat_wxyz,
    gripper_forward_axis_ee,
    grasp_radius,
):
    if auto_grasp:
        if not controller.move_to_pose(stage_map["pregrasp"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
            controller.get_logger().error("Pregrasp trajectory failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
        _log_actual_ee_debug(
            controller,
            side,
            "after_pregrasp_move",
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            radius=grasp_radius,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )

        controller.get_logger().info(f"Open {side} gripper")
        if not controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration)):
            controller.get_logger().error(f"Open {side} gripper failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}

        if stop_after_open:
            controller.get_logger().info("Stop after approach/open because --stop-after-open was requested")
            return {"ok": True, "grasp_success": None, "lift_stage": None, "check": None}

        if not controller.move_to_pose(stage_map["grasp"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
            controller.get_logger().error("Grasp trajectory failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
        _log_actual_ee_debug(
            controller,
            side,
            "after_grasp_move",
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            radius=grasp_radius,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )

        if not controller.move_to_pose(stage_map["descend_after_grasp"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
            controller.get_logger().error("Descend-after-grasp trajectory failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
        _log_actual_ee_debug(
            controller,
            side,
            "after_descend_after_grasp_move",
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            radius=grasp_radius,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )
    else:
        if not controller.move_to_pose(stage_map["approach"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
            controller.get_logger().error("Approach trajectory failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
        _log_actual_ee_debug(
            controller,
            side,
            "after_approach_move",
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            radius=grasp_radius,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )

        controller.get_logger().info(f"Open {side} gripper")
        if not controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration)):
            controller.get_logger().error(f"Open {side} gripper failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}

        if stop_after_open:
            controller.get_logger().info("Stop after approach/open because --stop-after-open was requested")
            return {"ok": True, "grasp_success": None, "lift_stage": None, "check": None}

        if not controller.move_to_pose(stage_map["descend"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
            controller.get_logger().error("Descend trajectory failed")
            return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
        _log_actual_ee_debug(
            controller,
            side,
            "after_descend_move",
            gripper_forward_axis_ee=gripper_forward_axis_ee,
            radius=grasp_radius,
            base_in_world_pos=base_in_world_pos,
            world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        )

    if no_close_grip:
        controller.get_logger().info("Skip close/lift because --no-close-grip was requested")
        return {"ok": True, "grasp_success": None, "lift_stage": None, "check": None}

    controller.get_logger().info(f"Close {side} gripper")
    if not controller.close_grip(side, wait=True, timeout=max(timeout, gripper_duration)):
        controller.get_logger().error(f"Close {side} gripper failed")
        return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}

    if grasp_success_check and hasattr(part_monitor, "get_update_seq"):
        part_state_seq_before_lift = part_monitor.get_update_seq()
    else:
        part_state_seq_before_lift = None

    if not controller.move_to_pose(stage_map["lift"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
        controller.get_logger().error("Lift trajectory failed")
        return {"ok": False, "grasp_success": None, "lift_stage": None, "check": None}
    _log_actual_ee_debug(
        controller,
        side,
        "after_lift_move",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )

    if not grasp_success_check:
        return {"ok": True, "grasp_success": True, "lift_stage": stage_map["lift"], "check": None}

    check = _check_grasp_success_after_lift(
        controller,
        part_monitor,
        part_name,
        part_pose_before_lift,
        side,
        min_lift_delta=grasp_success_min_lift_delta,
        max_part_to_ee_dist=grasp_success_max_part_to_ee_dist,
        part_state_timeout=grasp_success_part_state_timeout,
        finger_timeout=grasp_success_finger_timeout,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        last_part_state_seq=part_state_seq_before_lift,
    )
    return {"ok": True, "grasp_success": bool(check["success"]), "lift_stage": stage_map["lift"], "check": check}


def _execute_place(
    controller,
    side,
    lift_stage,
    place_box_world_pos,
    place_exit_left_offset_world,
    place_approach_height,
    place_release_height,
    place_lift_height,
    duration_per_step,
    gripper_duration,
    timeout,
    rot_weight,
    unlock_waist,
    joint_limit_margin,
    unconstrain_rot_z,
    world_to_base_quat_wxyz,
    base_in_world_pos,
    gripper_forward_axis_ee,
    grasp_radius,
    position_tolerance,
    require_ik_ok,
):
    ee_rpy_base = np.asarray(lift_stage["pose"][3:], dtype=float)
    place_stages = solve_place_waypoints(
        controller,
        side,
        place_box_world_pos,
        ee_rpy_base,
        rot_weight=rot_weight,
        unlock_waist=unlock_waist,
        joint_limit_margin=joint_limit_margin,
        seed_joint_targets=lift_stage.get("joint_targets"),
        unconstrain_rot_z=unconstrain_rot_z,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
        base_in_world_pos=base_in_world_pos,
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        exit_left_offset_world=place_exit_left_offset_world,
        place_approach_height=place_approach_height,
        place_release_height=place_release_height,
        place_lift_height=place_lift_height,
        position_tolerance=position_tolerance,
    )
    controller.get_logger().info(f"Place box world pos: {_format_vec(place_box_world_pos)}")
    controller.get_logger().info(f"Place exit-left offset world: {_format_vec(place_exit_left_offset_world)}")
    if not _validate_stages_for_execution(controller, place_stages, require_ik_ok, dry_run=False):
        return False
    stage_map = _stage_map_or_log(
        controller,
        place_stages,
        ("place_approach", "place_release", "place_lift", "place_exit_left"),
        "place",
    )
    if stage_map is None:
        return False

    if not controller.move_to_pose(stage_map["place_approach"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
        controller.get_logger().error("Place approach trajectory failed")
        return False
    _log_actual_ee_debug(
        controller,
        side,
        "after_place_approach_move",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )

    if not controller.move_to_pose(stage_map["place_release"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
        controller.get_logger().error("Place release trajectory failed")
        return False
    _log_actual_ee_debug(
        controller,
        side,
        "after_place_release_move",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )

    controller.get_logger().info(f"Open {side} gripper to release part")
    if not controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration)):
        controller.get_logger().error(f"Open {side} gripper failed")
        return False

    if not controller.move_to_pose(stage_map["place_lift"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
        controller.get_logger().error("Place lift trajectory failed")
        return False
    _log_actual_ee_debug(
        controller,
        side,
        "after_place_lift_move",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )

    if not controller.move_to_pose(stage_map["place_exit_left"]["joint_targets"], duration_sec=duration_per_step, wait=True, unlock_required_joints=True):
        controller.get_logger().error("Place exit-left trajectory failed")
        return False
    _log_actual_ee_debug(
        controller,
        side,
        "after_place_exit_left_move",
        gripper_forward_axis_ee=gripper_forward_axis_ee,
        radius=grasp_radius,
        base_in_world_pos=base_in_world_pos,
        world_to_base_quat_wxyz=world_to_base_quat_wxyz,
    )
    return True


def move_ee_by_waypoints(
    controller,
    part_monitor,
    part_name=DEFAULT_PART_NAME,
    approach_offset_world=DEFAULT_APPROACH_OFFSET_WORLD,
    descend_offset_world=DEFAULT_DESCEND_OFFSET_WORLD,
    lift_offset_world=DEFAULT_LIFT_OFFSET_WORLD,
    ee_rpy_deg=None,
    ee_rpy_delta_deg=None,
    tilt_base_x_deg=None,
    tilt_base_y_deg=None,
    duration_per_step=2.0,
    gripper_duration=1.0,
    rot_weight=DEFAULT_ROT_WEIGHT,
    unconstrain_rot_z=False,
    timeout=5.0,
    dry_run=False,
    world_to_base_quat_wxyz=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    base_in_world_pos=DEFAULT_BASE_IN_WORLD_POS,
    unlock_waist=False,
    joint_limit_margin=DEFAULT_JOINT_LIMIT_MARGIN,
    no_close_grip=False,
    stop_after_open=False,
    auto_grasp=False,
    side="right",
    grasp_radius=DEFAULT_GRASP_RADIUS,
    grasp_min_table_angle_deg=DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG,
    grasp_lift_height=DEFAULT_GRASP_LIFT_HEIGHT,
    grasp_pregrasp_height=DEFAULT_GRASP_PREGRASP_HEIGHT,
    grasp_azimuth_count=DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT,
    grasp_elevation_count=DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT,
    gripper_forward_axis_ee=DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    grasp_target_offset_world=DEFAULT_GRASP_TARGET_OFFSET_WORLD,
    grasp_descend_after_target_world=DEFAULT_GRASP_DESCEND_AFTER_TARGET_WORLD,
    place_after_grasp=True,
    place_box_world_pos=None,
    place_exit_left_offset_world=DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD,
    place_approach_height=DEFAULT_PLACE_APPROACH_HEIGHT,
    place_release_height=DEFAULT_PLACE_RELEASE_HEIGHT,
    place_lift_height=DEFAULT_PLACE_LIFT_HEIGHT,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
    require_ik_ok=DEFAULT_REQUIRE_IK_OK,
    grasp_max_attempts=DEFAULT_GRASP_MAX_ATTEMPTS,
    grasp_success_check=DEFAULT_GRASP_SUCCESS_CHECK,
    grasp_success_min_lift_delta=DEFAULT_GRASP_SUCCESS_MIN_LIFT_DELTA,
    grasp_success_max_part_to_ee_dist=DEFAULT_GRASP_SUCCESS_MAX_PART_TO_EE_DIST,
    grasp_success_part_state_timeout=DEFAULT_GRASP_SUCCESS_PART_STATE_TIMEOUT,
    grasp_success_finger_timeout=DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT,
):
    """直接以 TCP 为 EE 控制目标，按 waypoint 执行抓取/放置，支持左右手与自动/手工路径。"""
    if float(position_tolerance) <= 0.0:
        controller.get_logger().error(f"position_tolerance must be positive, got {position_tolerance}")
        return False
    grasp_max_attempts = int(grasp_max_attempts)
    if grasp_max_attempts < 1:
        controller.get_logger().error(f"grasp_max_attempts must be >= 1, got {grasp_max_attempts}")
        return False
    if float(grasp_success_min_lift_delta) < 0.0:
        controller.get_logger().error(
            f"grasp_success_min_lift_delta must be non-negative, got {grasp_success_min_lift_delta}"
        )
        return False
    if float(grasp_success_max_part_to_ee_dist) <= 0.0:
        controller.get_logger().error(
            f"grasp_success_max_part_to_ee_dist must be positive, got {grasp_success_max_part_to_ee_dist}"
        )
        return False
    if float(grasp_success_finger_timeout) < 0.0:
        controller.get_logger().error(
            f"grasp_success_finger_timeout must be non-negative, got {grasp_success_finger_timeout}"
        )
        return False
    if float(place_approach_height) < float(place_release_height):
        controller.get_logger().warning(
            f"place_approach_height {place_approach_height:.3f} < place_release_height {place_release_height:.3f}"
        )
    if float(place_lift_height) < float(place_release_height):
        controller.get_logger().warning(
            f"place_lift_height {place_lift_height:.3f} < place_release_height {place_release_height:.3f}"
        )
    if place_box_world_pos is None:
        place_box_world_pos = _default_place_box_world_pos_for_part(part_name)

    if not controller.wait_for_state(timeout=timeout):
        return False
    if not part_monitor.wait_for_part_states(timeout=timeout):
        return False
    if not controller.initialize_ik():
        return False

    def plan_from_latest_part_pose(attempt_index=None):
        part_pose = _get_part_pose_or_log(part_monitor, part_name, controller.get_logger())
        if part_pose is None:
            return None
        plan = _plan_grasp_stages(
            controller,
            part_pose,
            side,
            auto_grasp,
            approach_offset_world,
            descend_offset_world,
            lift_offset_world,
            ee_rpy_deg,
            ee_rpy_delta_deg,
            tilt_base_x_deg,
            tilt_base_y_deg,
            rot_weight,
            unconstrain_rot_z,
            unlock_waist,
            joint_limit_margin,
            world_to_base_quat_wxyz,
            base_in_world_pos,
            grasp_radius,
            grasp_lift_height,
            grasp_pregrasp_height,
            grasp_azimuth_count,
            grasp_elevation_count,
            gripper_forward_axis_ee,
            grasp_target_offset_world,
            grasp_descend_after_target_world,
            grasp_min_table_angle_deg,
            position_tolerance,
            require_ik_ok,
        )
        if plan is None:
            if attempt_index is not None:
                controller.get_logger().error(f"Attempt {attempt_index}/{grasp_max_attempts}: grasp planning failed")
            return None
        _log_grasp_plan(
            controller,
            plan,
            part_name,
            side,
            rot_weight,
            unlock_waist,
            base_in_world_pos,
            world_to_base_quat_wxyz,
            gripper_forward_axis_ee,
            grasp_radius,
            attempt_index=attempt_index,
            max_attempts=grasp_max_attempts if attempt_index is not None else None,
        )
        return plan

    if dry_run:
        plan = plan_from_latest_part_pose()
        if plan is None:
            return False
        stages = list(plan["stages"])
        if place_after_grasp and not stop_after_open and not no_close_grip:
            lift_stage = next((stage for stage in stages if stage["label"] == "lift"), None)
            if lift_stage is None:
                controller.get_logger().error("Dry run cannot plan place: lift stage is missing")
                return False
            ee_rpy_base = np.asarray(lift_stage["pose"][3:], dtype=float)
            place_stages = solve_place_waypoints(
                controller,
                side,
                place_box_world_pos,
                ee_rpy_base,
                rot_weight=rot_weight,
                unlock_waist=unlock_waist,
                joint_limit_margin=joint_limit_margin,
                seed_joint_targets=lift_stage.get("joint_targets"),
                unconstrain_rot_z=unconstrain_rot_z,
                world_to_base_quat_wxyz=world_to_base_quat_wxyz,
                base_in_world_pos=base_in_world_pos,
                gripper_forward_axis_ee=gripper_forward_axis_ee,
                exit_left_offset_world=place_exit_left_offset_world,
                place_approach_height=place_approach_height,
                place_release_height=place_release_height,
                place_lift_height=place_lift_height,
                position_tolerance=position_tolerance,
            )
            stages.extend(place_stages)
            controller.get_logger().info(f"Place box world pos: {_format_vec(place_box_world_pos)}")
            controller.get_logger().info(f"Place exit-left offset world: {_format_vec(place_exit_left_offset_world)}")
        return _validate_stages_for_execution(controller, stages, require_ik_ok, dry_run=True)

    for attempt in range(1, grasp_max_attempts + 1):
        plan = plan_from_latest_part_pose(attempt_index=attempt)
        if plan is None:
            if attempt < grasp_max_attempts:
                controller.get_logger().warning(f"Retry grasp planning with latest part pose ({attempt + 1}/{grasp_max_attempts})")
                continue
            return False

        stages = plan["stages"]
        if not _validate_stages_for_execution(controller, stages, require_ik_ok, dry_run=False):
            if attempt < grasp_max_attempts:
                controller.get_logger().warning(f"Retry after invalid grasp plan ({attempt + 1}/{grasp_max_attempts})")
                continue
            return False

        required_labels = ("pregrasp", "grasp", "descend_after_grasp", "lift") if auto_grasp else ("approach", "descend", "lift")
        stage_map = _stage_map_or_log(controller, stages, required_labels, "auto grasp" if auto_grasp else "manual grasp")
        if stage_map is None:
            return False

        result = _execute_grasp_once(
            controller,
            part_monitor,
            part_name,
            plan["part_pose"],
            stage_map,
            side,
            auto_grasp,
            duration_per_step,
            gripper_duration,
            timeout,
            no_close_grip,
            stop_after_open,
            grasp_success_check,
            grasp_success_min_lift_delta,
            grasp_success_max_part_to_ee_dist,
            grasp_success_part_state_timeout,
            grasp_success_finger_timeout,
            base_in_world_pos,
            world_to_base_quat_wxyz,
            gripper_forward_axis_ee,
            grasp_radius,
        )
        if not result["ok"]:
            return False
        if result["grasp_success"] is None:
            return True
        if result["grasp_success"]:
            controller.get_logger().info(f"Grasp succeeded on attempt {attempt}/{grasp_max_attempts}")
            if place_after_grasp:
                if not _execute_place(
                    controller,
                    side,
                    result["lift_stage"],
                    place_box_world_pos,
                    place_exit_left_offset_world,
                    place_approach_height,
                    place_release_height,
                    place_lift_height,
                    duration_per_step,
                    gripper_duration,
                    timeout,
                    rot_weight,
                    unlock_waist,
                    joint_limit_margin,
                    unconstrain_rot_z,
                    world_to_base_quat_wxyz,
                    base_in_world_pos,
                    gripper_forward_axis_ee,
                    grasp_radius,
                    position_tolerance,
                    require_ik_ok,
                ):
                    return False

            controller.get_logger().info("Move back to READY_POSE after grasp/place sequence")
            if not controller.move_to_pose(READY_POSE, duration_sec=3.0, wait=True, unlock_required_joints=True):
                controller.get_logger().error("Move back to READY_POSE failed")
                return False

            controller.get_logger().info("EE grasp/place sequence completed" if place_after_grasp else "EE grasp sequence completed")
            return True

        if attempt < grasp_max_attempts:
            controller.get_logger().warning(
                f"Grasp failed on attempt {attempt}/{grasp_max_attempts}; open gripper and retry with latest part pose"
            )
            controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration))
            continue

        controller.get_logger().error(f"Grasp failed after {grasp_max_attempts} attempt(s); not executing place")
        controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration))
        return False

    return False


def randomize_part_positions(
    controller,
    part_monitor,
    part_names=DEFAULT_PART_SEQUENCE,
    topic=DEFAULT_RANDOMIZE_PARTS_TOPIC,
    timeout=5.0,
    settle_time=0.5,
    seed=None,
):
    """通过 ROS2 bridge 请求仿真随机化零件位置，并等待 part_states 更新。"""
    part_names = tuple(part_names)
    if not part_names:
        controller.get_logger().error("part_names must not be empty")
        return False

    last_seq = part_monitor.get_update_seq() if hasattr(part_monitor, "get_update_seq") else None
    pub = controller.create_publisher(String, topic, 1)
    time.sleep(0.2)

    if seed is None:
        seed = time.time_ns() & 0xFFFFFFFF
    payload = {"parts": list(part_names), "seed": int(seed)}
    msg = String()
    msg.data = json.dumps(payload)
    controller.get_logger().info(f"Randomize part positions via {topic}: {msg.data}")
    pub.publish(msg)

    if last_seq is not None and float(timeout) > 0.0:
        if not part_monitor.wait_for_new_part_states(last_seq, timeout=float(timeout)):
            controller.get_logger().error("No part_states update received after randomize request")
            return False
    if float(settle_time) > 0.0:
        time.sleep(float(settle_time))
    return True


def move_parts_by_waypoints(
    controller,
    part_monitor,
    part_names=DEFAULT_PART_SEQUENCE,
    randomize_before=True,
    randomize_topic=DEFAULT_RANDOMIZE_PARTS_TOPIC,
    randomize_timeout=None,
    randomize_settle_time=0.5,
    randomize_seed=None,
    **kwargs,
):
    """按顺序对多个零件执行同一套抓取/放置流程。"""
    fallback_part_name = kwargs.pop("part_name", DEFAULT_PART_NAME)
    if part_names is None:
        part_names = (fallback_part_name,)
    else:
        part_names = tuple(part_names)
    if not part_names:
        controller.get_logger().error("part_names must not be empty")
        return False

    if randomize_before:
        timeout = kwargs.get("timeout", 5.0) if randomize_timeout is None else randomize_timeout
        if not randomize_part_positions(
            controller,
            part_monitor,
            part_names=part_names,
            topic=randomize_topic,
            timeout=timeout,
            settle_time=randomize_settle_time,
            seed=randomize_seed,
        ):
            return False

    for index, part_name in enumerate(part_names, start=1):
        controller.get_logger().info(f"Start part {index}/{len(part_names)}: {part_name}")
        ok = move_ee_by_waypoints(
            controller,
            part_monitor,
            part_name=part_name,
            **kwargs,
        )
        if not ok:
            controller.get_logger().error(f"Part {index}/{len(part_names)} failed: {part_name}")
            return False
        controller.get_logger().info(f"Part {index}/{len(part_names)} completed: {part_name}")

    controller.get_logger().info(f"Completed {len(part_names)} part(s): {list(part_names)}")
    return True


def move_right_ee_by_manual_waypoints(*args, **kwargs):
    """Backward-compatible wrapper for move_ee_by_waypoints."""
    return move_ee_by_waypoints(*args, **kwargs)


def parse_args():
    parser = argparse.ArgumentParser(description="Move Walker S2 EE through grasp/place waypoints")
    parser.add_argument("--part", default=DEFAULT_PART_NAME, help="零件名，例如 part_a_ori / part_a_red / part_b_blue / part_b_ori")
    parser.add_argument("--all-parts", action="store_true", help="按默认顺序依次抓取四个零件")
    parser.add_argument("--parts", nargs="+", default=None, help="自定义多零件抓取顺序；指定后会依次执行每个零件")
    parser.add_argument("--randomize-parts", action=argparse.BooleanOptionalAction, default=True, help="多零件抓取前先通过 bridge 随机化零件位置")
    parser.add_argument("--randomize-parts-topic", default=DEFAULT_RANDOMIZE_PARTS_TOPIC, help="零件随机化命令 topic")
    parser.add_argument("--randomize-seed", type=int, default=None, help="零件随机化 seed；不指定则每次自动生成随机 seed")
    parser.add_argument("--randomize-settle-time", type=float, default=0.5, help="随机化后等待物体状态稳定的时间，单位 s")
    parser.add_argument("--side", choices=("left", "right"), default="right", help="选择左手或右手抓取")
    parser.add_argument("--auto-grasp", action=argparse.BooleanOptionalAction, default=DEFAULT_AUTO_GRASP, help="在零件 world 坐标周围球面采样，自动选择 IK 可达抓取姿态")
    parser.add_argument("--approach-offset", type=float, nargs=3, default=DEFAULT_APPROACH_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="approach EE 目标相对零件 world 坐标的偏移，单位 m")
    parser.add_argument("--descend-offset", type=float, nargs=3, default=DEFAULT_DESCEND_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="descend EE 目标相对零件 world 坐标的偏移，单位 m")
    parser.add_argument("--lift-offset", type=float, nargs=3, default=DEFAULT_LIFT_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="lift EE 目标相对零件 world 坐标的偏移，单位 m")
    parser.add_argument("--ee-rpy-deg", type=float, nargs=3, default=None, metavar=("R", "P", "Y"), help="手工指定 EE base-frame RPY，单位度；不填则使用当前姿态/增量")
    parser.add_argument("--ee-rpy-delta-deg", type=float, nargs=3, default=None, metavar=("R", "P", "Y"), help="当前 EE 姿态左乘的 base-frame RPY 增量，单位度")
    parser.add_argument("--tilt-base-x-deg", type=float, default=None, help="当前 EE 姿态左乘 base x 轴倾斜角，单位度")
    parser.add_argument("--tilt-base-y-deg", type=float, default=None, help="当前 EE 姿态左乘 base y 轴倾斜角，单位度")
    parser.add_argument("--rot-weight", type=float, default=DEFAULT_ROT_WEIGHT, help="IK 姿态误差权重；0 表示位置优先")
    parser.add_argument("--unconstrain-rot-z", action=argparse.BooleanOptionalAction, default=DEFAULT_UNCONSTRAIN_ROT_Z, help="IK 姿态只约束 base-frame x/y 旋转误差，不约束 z 轴旋转")
    parser.add_argument("--joint-limit-margin-deg", type=float, default=np.rad2deg(DEFAULT_JOINT_LIMIT_MARGIN), help="关节限位安全裕量，单位度")
    parser.add_argument("--position-tolerance", type=float, default=DEFAULT_POSITION_TOLERANCE, help="IK 位置误差容忍阈值，单位 m")
    parser.add_argument("--require-ik-ok", action=argparse.BooleanOptionalAction, default=DEFAULT_REQUIRE_IK_OK, help="要求 IK solver 返回 success；--no-require-ik-ok 可恢复仅按位置误差/限位判定的调试策略")
    parser.add_argument("--duration", type=float, default=2.0, help="每段关节轨迹执行时间，单位 s")
    parser.add_argument("--gripper-duration", type=float, default=1.0, help="等待夹爪打开/闭合的超时时间，单位 s")
    parser.add_argument("--unlock-waist", action=argparse.BooleanOptionalAction, default=DEFAULT_UNLOCK_WAIST, help="IK 求解时允许 waist_yaw_joint 参与所选手臂求解，并在下发时临时解锁腰部")
    parser.add_argument("--stop-after-open", action="store_true", help="只执行 approach/pregrasp 和打开所选夹爪，然后结束")
    parser.add_argument("--no-close-grip", action="store_true", help="只执行 approach/open/descend，不闭合夹爪和抬起")
    parser.add_argument("--timeout", type=float, default=5.0, help="等待 ROS 状态 topic 的超时时间，单位 s")
    parser.add_argument("--grasp-radius", type=float, default=DEFAULT_GRASP_RADIUS, help="兼容旧参数；当前 IK 末端已经是 TCP，不再按该半径外推")
    parser.add_argument("--grasp-max-attempts", type=int, default=DEFAULT_GRASP_MAX_ATTEMPTS, help="抓取总尝试次数；1 表示不重试")
    parser.add_argument("--grasp-success-check", action=argparse.BooleanOptionalAction, default=DEFAULT_GRASP_SUCCESS_CHECK, help="close/lift 后检查零件是否被抓起；失败时重新读取零件位置并重试")
    parser.add_argument("--grasp-success-min-lift-delta", type=float, default=DEFAULT_GRASP_SUCCESS_MIN_LIFT_DELTA, help="判断抓取成功所需的零件 world z 最小抬升量，单位 m")
    parser.add_argument("--grasp-success-max-part-to-ee-dist", type=float, default=DEFAULT_GRASP_SUCCESS_MAX_PART_TO_EE_DIST, help="lift 后零件到夹爪参考点的最大允许距离，单位 m")
    parser.add_argument("--grasp-success-part-state-timeout", type=float, default=DEFAULT_GRASP_SUCCESS_PART_STATE_TIMEOUT, help="lift 后等待下一帧 /sim/part_states 的时间，单位 s；0 表示不等待")
    parser.add_argument("--grasp-success-finger-timeout", type=float, default=DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT, help="可选等待 /sim/finger_link_states 的时间，单位 s；0 表示使用 EE/TCP")
    parser.add_argument("--grasp-min-table-angle-deg", type=float, default=DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG, help="采样方向相对桌面的最小仰角，单位度")
    parser.add_argument("--grasp-lift-height", type=float, default=DEFAULT_GRASP_LIFT_HEIGHT, help="抓取后 world z 方向抬升高度，单位 m")
    parser.add_argument("--grasp-pregrasp-height", type=float, default=DEFAULT_GRASP_PREGRASP_HEIGHT, help="auto-grasp 中抓取前零件上方路径点高度，单位 m")
    parser.add_argument("--grasp-target-offset", type=float, nargs=3, default=DEFAULT_GRASP_TARGET_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="球面采样目标点相对零件 world 坐标的偏移")
    parser.add_argument("--grasp-azimuth-count", type=int, default=DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT, help="球面方位角采样数量")
    parser.add_argument("--grasp-elevation-count", type=int, default=DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT, help="球面仰角采样数量")
    parser.add_argument("--gripper-forward-axis-ee", type=float, nargs=3, default=DEFAULT_GRIPPER_FORWARD_AXIS_EE, metavar=("X", "Y", "Z"), help="EE 局部坐标中从 force sensor 指向夹具/TCP 的轴")
    parser.add_argument("--place", action=argparse.BooleanOptionalAction, default=True, help="抓取成功并抬起后，移动到放置箱子上方并松开夹爪")
    parser.add_argument("--place-box-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="放置箱子 world frame 位置；不指定时 A 类零件放位置 1，B 类零件放位置 2")
    parser.add_argument("--place-approach-height", type=float, default=DEFAULT_PLACE_APPROACH_HEIGHT, help="place_approach 相对箱子 world z 的高度，单位 m")
    parser.add_argument("--place-release-height", type=float, default=DEFAULT_PLACE_RELEASE_HEIGHT, help="place_release 相对箱子 world z 的高度，单位 m")
    parser.add_argument("--place-lift-height", type=float, default=DEFAULT_PLACE_LIFT_HEIGHT, help="松爪后 place_lift 相对箱子 world z 的高度，单位 m")
    parser.add_argument("--place-exit-left-offset", type=float, nargs=3, default=DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="松爪后先抬升，再按该 world 偏移离开箱体范围")
    parser.add_argument(
        "--base-pos",
        type=float,
        nargs=3,
        default=DEFAULT_BASE_IN_WORLD_POS,
        metavar=("X", "Y", "Z"),
        help="URDF base 原点在 world frame 下的位置；默认来自 walker_s2_part_sorting.yaml",
    )
    parser.add_argument(
        "--world-to-base-quat",
        type=float,
        nargs=4,
        default=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
        metavar=("W", "X", "Y", "Z"),
        help="world frame 到 URDF base frame 的旋转四元数 wxyz；默认对应当前场景机器人绕 world z +90°",
    )
    parser.add_argument("--dry-run", action="store_true", help="只检查 IK 和关节限位，不下发控制")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    controller = WalkerS2Controller(enable_ik=True, subscribe_images=False)
    part_monitor = PartStateMonitor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    executor.add_node(part_monitor)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    try:
        part_names = DEFAULT_PART_SEQUENCE if args.all_parts else args.parts
        move_kwargs = dict(
            approach_offset_world=args.approach_offset,
            descend_offset_world=args.descend_offset,
            lift_offset_world=args.lift_offset,
            ee_rpy_deg=args.ee_rpy_deg,
            ee_rpy_delta_deg=args.ee_rpy_delta_deg,
            tilt_base_x_deg=args.tilt_base_x_deg,
            tilt_base_y_deg=args.tilt_base_y_deg,
            duration_per_step=args.duration,
            gripper_duration=args.gripper_duration,
            rot_weight=args.rot_weight,
            unconstrain_rot_z=args.unconstrain_rot_z,
            unlock_waist=args.unlock_waist,
            joint_limit_margin=np.deg2rad(args.joint_limit_margin_deg),
            position_tolerance=args.position_tolerance,
            require_ik_ok=args.require_ik_ok,
            no_close_grip=args.no_close_grip,
            stop_after_open=args.stop_after_open,
            timeout=args.timeout,
            dry_run=args.dry_run,
            world_to_base_quat_wxyz=args.world_to_base_quat,
            base_in_world_pos=args.base_pos,
            auto_grasp=args.auto_grasp,
            side=args.side,
            grasp_radius=args.grasp_radius,
            grasp_max_attempts=args.grasp_max_attempts,
            grasp_success_check=args.grasp_success_check,
            grasp_success_min_lift_delta=args.grasp_success_min_lift_delta,
            grasp_success_max_part_to_ee_dist=args.grasp_success_max_part_to_ee_dist,
            grasp_success_part_state_timeout=args.grasp_success_part_state_timeout,
            grasp_success_finger_timeout=args.grasp_success_finger_timeout,
            grasp_min_table_angle_deg=args.grasp_min_table_angle_deg,
            grasp_lift_height=args.grasp_lift_height,
            grasp_pregrasp_height=args.grasp_pregrasp_height,
            grasp_target_offset_world=args.grasp_target_offset,
            grasp_azimuth_count=args.grasp_azimuth_count,
            grasp_elevation_count=args.grasp_elevation_count,
            gripper_forward_axis_ee=args.gripper_forward_axis_ee,
            place_after_grasp=args.place,
            place_box_world_pos=args.place_box_pos,
            place_exit_left_offset_world=args.place_exit_left_offset,
            place_approach_height=args.place_approach_height,
            place_release_height=args.place_release_height,
            place_lift_height=args.place_lift_height,
        )
        if part_names:
            ok = move_parts_by_waypoints(
                controller,
                part_monitor,
                part_names=part_names,
                randomize_before=args.randomize_parts,
                randomize_topic=args.randomize_parts_topic,
                randomize_timeout=args.timeout,
                randomize_settle_time=args.randomize_settle_time,
                randomize_seed=args.randomize_seed,
                **move_kwargs,
            )
        else:
            ok = move_ee_by_waypoints(
                controller,
                part_monitor,
                part_name=args.part,
                **move_kwargs,
            )
        if not ok:
            raise SystemExit(1)
    except KeyboardInterrupt:
        controller.get_logger().warning("Interrupted by user")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        part_monitor.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
