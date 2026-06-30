#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 零件抓取任务 + HDF5 数据采集脚本。

复用 pick_part.py 的抓取/放置逻辑，通过 before_execute_callback 在第一条
实际抓取轨迹开始前启动录制，避免把 randomize、等待状态、IK 规划等静止前缀写入数据集。
"""

import argparse
import os
import sys
import threading

import cv2
import h5py
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# 支持直接运行和包导入两种方式
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from pick_part import (  # noqa: E402
    DEFAULT_APPROACH_OFFSET_WORLD,
    DEFAULT_AUTO_GRASP,
    DEFAULT_BASE_IN_WORLD_POS,
    DEFAULT_DESCEND_OFFSET_WORLD,
    DEFAULT_GRASP_LIFT_HEIGHT,
    DEFAULT_GRASP_MAX_ATTEMPTS,
    DEFAULT_GRASP_MIN_TABLE_ANGLE_DEG,
    DEFAULT_GRASP_PREGRASP_HEIGHT,
    DEFAULT_GRASP_RADIUS,
    DEFAULT_GRASP_SAMPLE_AZIMUTH_COUNT,
    DEFAULT_GRASP_SAMPLE_ELEVATION_COUNT,
    DEFAULT_GRASP_SUCCESS_CHECK,
    DEFAULT_GRASP_SUCCESS_FINGER_TIMEOUT,
    DEFAULT_GRASP_SUCCESS_MAX_PART_TO_EE_DIST,
    DEFAULT_GRASP_SUCCESS_MIN_LIFT_DELTA,
    DEFAULT_GRASP_SUCCESS_PART_STATE_TIMEOUT,
    DEFAULT_GRASP_TARGET_OFFSET_WORLD,
    DEFAULT_GRIPPER_FORWARD_AXIS_EE,
    DEFAULT_JOINT_LIMIT_MARGIN,
    DEFAULT_LIFT_OFFSET_WORLD,
    DEFAULT_PART_NAME,
    DEFAULT_PART_SEQUENCE,
    DEFAULT_PLACE_APPROACH_HEIGHT,
    DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD,
    DEFAULT_PLACE_LIFT_HEIGHT,
    DEFAULT_PLACE_RELEASE_HEIGHT,
    DEFAULT_PLACE_ROT_WEIGHT,
    DEFAULT_POSITION_TOLERANCE,
    DEFAULT_RANDOMIZE_PARTS_TOPIC,
    DEFAULT_REQUIRE_IK_OK,
    DEFAULT_RESET_SCENE_SETTLE_TIME,
    DEFAULT_ROBOT_INIT_DURATION,
    DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT,
    DEFAULT_ROBOT_INIT_TOLERANCE,
    DEFAULT_ROT_WEIGHT,
    DEFAULT_UNCONSTRAIN_ROT_Z,
    DEFAULT_UNLOCK_WAIST,
    DEFAULT_WORLD_TO_BASE_QUAT_WXYZ,
    PartStateMonitor,
    initialize_robot_pose,
    move_ee_by_waypoints,
    randomize_part_positions,
    reset_scene,
)
from walker_s2_camera import Camera  # noqa: E402
from walker_s2_controller import (  # noqa: E402
    BODY_JOINT_NAMES,
    DEFAULT_COMMAND_TOPIC,
    DEFAULT_IMAGE_DEPTH_TOPIC,
    DEFAULT_IMAGE_RGB_TOPIC,
    DEFAULT_LEFT_GRIP_COMMAND_TOPIC,
    DEFAULT_LEFT_GRIP_STATE_TOPIC,
    DEFAULT_LEFT_HAND_COMMAND_TOPIC,
    DEFAULT_LEFT_HAND_STATE_TOPIC,
    DEFAULT_RIGHT_GRIP_COMMAND_TOPIC,
    DEFAULT_RIGHT_GRIP_STATE_TOPIC,
    DEFAULT_RIGHT_HAND_COMMAND_TOPIC,
    DEFAULT_RIGHT_HAND_STATE_TOPIC,
    DEFAULT_STATE_TOPIC,
    GripCmd,
    GripStatus,
    JointCommand,
    RobotCommand,
    RobotState,
    V4_HAND_JOINT_MAP,
    WalkerS2Controller,
)


SAVE_HZ = 30.0
PLACEHOLDER_IMG_SHAPE = (360, 640, 3)
PLACEHOLDER_DEPTH_SHAPE = (360, 640)


class WalkerDataRecorder(Node):
    """Walker S2 抓取任务 HDF5 数据录制节点。"""

    _BUFFER_KEYS = (
        "joint_position", "joint_velocity", "joint_effort",
        "hand_right_position", "hand_right_velocity", "hand_right_effort",
        "hand_left_position", "hand_left_velocity", "hand_left_effort",
        "grip_right_position", "grip_right_velocity", "grip_right_current",
        "grip_left_position", "grip_left_velocity", "grip_left_current",
        "action_joint_position", "action_hand_right_position", "action_hand_left_position",
        "action_grip_right_position", "action_grip_left_position",
        "img", "depth", "timestamp",
    )

    def __init__(
        self,
        rgb_camera,
        depth_camera,
        save_hz=SAVE_HZ,
        node_name="walker_pick_part_save_data_recorder",
    ):
        super().__init__(node_name)
        self.rgb_camera = rgb_camera
        self.depth_camera = depth_camera
        self.save_hz = float(save_hz)
        self.data_buffer = {k: [] for k in self._BUFFER_KEYS}
        self.is_saving = False
        self.dropped_frames = 0

        self._lock = threading.Lock()
        self._joint_position = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        self._joint_velocity = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        self._joint_effort = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        self._hand_position = {
            "left": np.zeros(len(V4_HAND_JOINT_MAP["left"]), dtype=float),
            "right": np.zeros(len(V4_HAND_JOINT_MAP["right"]), dtype=float),
        }
        self._hand_velocity = {
            "left": np.zeros(len(V4_HAND_JOINT_MAP["left"]), dtype=float),
            "right": np.zeros(len(V4_HAND_JOINT_MAP["right"]), dtype=float),
        }
        self._hand_effort = {
            "left": np.zeros(len(V4_HAND_JOINT_MAP["left"]), dtype=float),
            "right": np.zeros(len(V4_HAND_JOINT_MAP["right"]), dtype=float),
        }
        self._grip_state = {
            "left": {"pos": 0.0, "vel": 0.0, "cur": 0.0},
            "right": {"pos": 0.0, "vel": 0.0, "cur": 0.0},
        }
        self._action_joint_position = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        self._action_hand_position = {
            "left": np.zeros(len(V4_HAND_JOINT_MAP["left"]), dtype=float),
            "right": np.zeros(len(V4_HAND_JOINT_MAP["right"]), dtype=float),
        }
        self._action_grip_position = {"left": 0.0, "right": 0.0}
        self._received_joint_state = False
        self._received_action_joint = False
        self._received_action_hand = {"left": False, "right": False}
        self._received_action_grip = {"left": False, "right": False}

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            RobotState, DEFAULT_STATE_TOPIC, self._robot_state_cb, qos_state,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            JointState, DEFAULT_LEFT_HAND_STATE_TOPIC,
            lambda msg: self._hand_state_cb("left", msg),
            qos_state, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            JointState, DEFAULT_RIGHT_HAND_STATE_TOPIC,
            lambda msg: self._hand_state_cb("right", msg),
            qos_state, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            GripStatus, DEFAULT_LEFT_GRIP_STATE_TOPIC,
            lambda msg: self._grip_state_cb("left", msg),
            qos_state, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            GripStatus, DEFAULT_RIGHT_GRIP_STATE_TOPIC,
            lambda msg: self._grip_state_cb("right", msg),
            qos_state, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            RobotCommand, DEFAULT_COMMAND_TOPIC, self._robot_command_cb, qos_cmd,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            JointCommand, DEFAULT_LEFT_HAND_COMMAND_TOPIC,
            lambda msg: self._hand_command_cb("left", msg),
            qos_cmd, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            JointCommand, DEFAULT_RIGHT_HAND_COMMAND_TOPIC,
            lambda msg: self._hand_command_cb("right", msg),
            qos_cmd, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            GripCmd, DEFAULT_LEFT_GRIP_COMMAND_TOPIC,
            lambda msg: self._grip_command_cb("left", msg),
            qos_cmd, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            GripCmd, DEFAULT_RIGHT_GRIP_COMMAND_TOPIC,
            lambda msg: self._grip_command_cb("right", msg),
            qos_cmd, callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.save_interval = 1.0 / self.save_hz
        self.save_timer = self.create_timer(
            self.save_interval, self._timer_save_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _robot_state_cb(self, msg: RobotState):
        joint_states = msg.joint_states
        names = list(joint_states.name)
        name_to_idx = {name: idx for idx, name in enumerate(names)}
        position = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        velocity = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        effort = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        for i, name in enumerate(BODY_JOINT_NAMES):
            src = name_to_idx.get(name)
            if src is None:
                continue
            if src < len(joint_states.position):
                position[i] = float(joint_states.position[src])
            if src < len(joint_states.velocity):
                velocity[i] = float(joint_states.velocity[src])
            if src < len(joint_states.effort):
                effort[i] = float(joint_states.effort[src])
        with self._lock:
            self._joint_position = position
            self._joint_velocity = velocity
            self._joint_effort = effort
            self._received_joint_state = True
            if not self._received_action_joint:
                self._action_joint_position = position.copy()

    def _hand_state_cb(self, side, msg: JointState):
        position, velocity, effort = self._ordered_joint_state(
            msg.name, msg.position, msg.velocity, msg.effort, V4_HAND_JOINT_MAP[side]
        )
        with self._lock:
            self._hand_position[side] = position
            self._hand_velocity[side] = velocity
            self._hand_effort[side] = effort
            if not self._received_action_hand[side]:
                self._action_hand_position[side] = position.copy()

    def _grip_state_cb(self, side, msg: GripStatus):
        with self._lock:
            self._grip_state[side] = {
                "init_state": int(msg.init_state),
                "grip_state": int(msg.grip_state),
                "error_code": int(msg.error_code),
                "homed": int(msg.homed),
                "pos": float(msg.pos),
                "vel": float(msg.vel),
                "cur": float(msg.cur),
            }
            if not self._received_action_grip[side]:
                self._action_grip_position[side] = float(msg.pos)

    def _robot_command_cb(self, msg: RobotCommand):
        with self._lock:
            action = self._action_joint_position.copy() if self._received_action_joint else self._joint_position.copy()
            name_to_idx = {name: idx for idx, name in enumerate(BODY_JOINT_NAMES)}
            for joint_cmd in msg.joint_cmd:
                idx = name_to_idx.get(joint_cmd.name)
                if idx is not None:
                    action[idx] = float(joint_cmd.position)
            self._action_joint_position = action
            self._received_action_joint = True

    def _hand_command_cb(self, side, msg: JointCommand):
        names = getattr(msg, "names", [])
        positions = getattr(msg, "position", [])
        action, _, _ = self._ordered_joint_state(names, positions, [], [], V4_HAND_JOINT_MAP[side])
        with self._lock:
            self._action_hand_position[side] = action
            self._received_action_hand[side] = True

    def _grip_command_cb(self, side, msg: GripCmd):
        with self._lock:
            self._action_grip_position[side] = float(msg.pos)
            self._received_action_grip[side] = True

    @staticmethod
    def _ordered_joint_state(names, positions, velocities, efforts, joint_order):
        name_to_idx = {name: idx for idx, name in enumerate(names)}
        position = np.zeros(len(joint_order), dtype=float)
        velocity = np.zeros(len(joint_order), dtype=float)
        effort = np.zeros(len(joint_order), dtype=float)
        for i, name in enumerate(joint_order):
            src = name_to_idx.get(name)
            if src is None:
                continue
            if src < len(positions):
                position[i] = float(positions[src])
            if src < len(velocities):
                velocity[i] = float(velocities[src])
            if src < len(efforts):
                effort[i] = float(efforts[src])
        return position, velocity, effort

    def start_save_data(self):
        """开始录制数据。"""
        for key in self._BUFFER_KEYS:
            self.data_buffer[key].clear()
        self.dropped_frames = 0
        self.is_saving = True
        self.get_logger().info(f"Started recording Walker data at {self.save_hz:.0f}Hz")

    def stop_save_data(self):
        """停止录制。"""
        self.is_saving = False

    def record_snapshot(self):
        """记录一帧数据快照。"""
        if not self.is_saving:
            return
        try:
            with self._lock:
                snapshot = {
                    "joint_position": self._joint_position.copy(),
                    "joint_velocity": self._joint_velocity.copy(),
                    "joint_effort": self._joint_effort.copy(),
                    "hand_right_position": self._hand_position["right"].copy(),
                    "hand_right_velocity": self._hand_velocity["right"].copy(),
                    "hand_right_effort": self._hand_effort["right"].copy(),
                    "hand_left_position": self._hand_position["left"].copy(),
                    "hand_left_velocity": self._hand_velocity["left"].copy(),
                    "hand_left_effort": self._hand_effort["left"].copy(),
                    "grip_right_position": float(self._grip_state["right"].get("pos", 0.0)),
                    "grip_right_velocity": float(self._grip_state["right"].get("vel", 0.0)),
                    "grip_right_current": float(self._grip_state["right"].get("cur", 0.0)),
                    "grip_left_position": float(self._grip_state["left"].get("pos", 0.0)),
                    "grip_left_velocity": float(self._grip_state["left"].get("vel", 0.0)),
                    "grip_left_current": float(self._grip_state["left"].get("cur", 0.0)),
                    "action_joint_position": self._action_joint_position.copy(),
                    "action_hand_right_position": self._action_hand_position["right"].copy(),
                    "action_hand_left_position": self._action_hand_position["left"].copy(),
                    "action_grip_right_position": float(self._action_grip_position["right"]),
                    "action_grip_left_position": float(self._action_grip_position["left"]),
                    "timestamp": self.get_clock().now().nanoseconds / 1e9,
                }
            snapshot["img"] = self._get_rgb_image()
            snapshot["depth"] = self._get_depth_image()
        except Exception as e:
            self.dropped_frames += 1
            self.get_logger().error(f"Error recording snapshot (dropped={self.dropped_frames}): {e}")
            return

        for key, value in snapshot.items():
            self.data_buffer[key].append(value)

    def _get_rgb_image(self):
        img = self.rgb_camera.get_latest_image()
        if img is None:
            return np.zeros(PLACEHOLDER_IMG_SHAPE, dtype=np.uint8)
        info = self.rgb_camera.get_image_info() or {}
        encoding = info.get("encoding", "rgb8")
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        elif encoding in ("bgr8", "yuv422"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return np.asarray(img, dtype=np.uint8).copy()

    def _get_depth_image(self):
        depth = self.depth_camera.get_latest_image()
        if depth is None:
            return np.zeros(PLACEHOLDER_DEPTH_SHAPE, dtype=np.uint16)
        depth = np.asarray(depth)
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            depth = np.nan_to_num(depth, nan=0.0, posinf=65.535, neginf=0.0)
            depth = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        elif depth.dtype != np.uint16:
            depth = np.clip(depth, 0, 65535).astype(np.uint16)
        return depth.copy()

    def save_data(self):
        """保存数据到 HDF5 文件。"""
        self.stop_save_data()
        length = len(self.data_buffer["joint_position"])
        if length == 0:
            self.get_logger().warning("No frames recorded, skip saving.")
            return None

        lens = {key: len(self.data_buffer[key]) for key in self._BUFFER_KEYS}
        if len(set(lens.values())) != 1:
            self.get_logger().error(f"Buffer length mismatch, abort save: {lens}")
            return None

        ts = self.get_clock().now().seconds_nanoseconds()[0]
        dataset_root = os.path.join(self._find_ubt_sim_dir(), "dataset")
        dir_name = os.path.join(dataset_root, f"{ts}")
        new_dir = not os.path.exists(dir_name)
        os.makedirs(dir_name, exist_ok=True)
        if new_dir:
            try:
                os.chmod(dir_name, 0o777)
            except PermissionError:
                pass
        filename = os.path.join(dir_name, "trajectory.hdf5")
        self.get_logger().info(f"Saving {length} frames to {filename}...")

        with h5py.File(filename, "w") as f:
            f.create_dataset("observation/joint_state/position/data", data=np.array(self.data_buffer["joint_position"]))
            f.create_dataset("observation/joint_state/velocity/data", data=np.array(self.data_buffer["joint_velocity"]))
            f.create_dataset("observation/joint_state/effort/data", data=np.array(self.data_buffer["joint_effort"]))
            f.create_dataset("observation/hand_right_position/data", data=np.array(self.data_buffer["hand_right_position"]))
            f.create_dataset("observation/hand_right_velocity/data", data=np.array(self.data_buffer["hand_right_velocity"]))
            f.create_dataset("observation/hand_right_effort/data", data=np.array(self.data_buffer["hand_right_effort"]))
            f.create_dataset("observation/hand_left_position/data", data=np.array(self.data_buffer["hand_left_position"]))
            f.create_dataset("observation/hand_left_velocity/data", data=np.array(self.data_buffer["hand_left_velocity"]))
            f.create_dataset("observation/hand_left_effort/data", data=np.array(self.data_buffer["hand_left_effort"]))
            f.create_dataset("observation/grip_right_position/data", data=np.array(self.data_buffer["grip_right_position"]))
            f.create_dataset("observation/grip_right_velocity/data", data=np.array(self.data_buffer["grip_right_velocity"]))
            f.create_dataset("observation/grip_right_current/data", data=np.array(self.data_buffer["grip_right_current"]))
            f.create_dataset("observation/grip_left_position/data", data=np.array(self.data_buffer["grip_left_position"]))
            f.create_dataset("observation/grip_left_velocity/data", data=np.array(self.data_buffer["grip_left_velocity"]))
            f.create_dataset("observation/grip_left_current/data", data=np.array(self.data_buffer["grip_left_current"]))
            f.create_dataset("action/joint_state/position/data", data=np.array(self.data_buffer["action_joint_position"]))
            f.create_dataset("action/hand_right_position/data", data=np.array(self.data_buffer["action_hand_right_position"]))
            f.create_dataset("action/hand_left_position/data", data=np.array(self.data_buffer["action_hand_left_position"]))
            f.create_dataset("action/grip_right_position/data", data=np.array(self.data_buffer["action_grip_right_position"]))
            f.create_dataset("action/grip_left_position/data", data=np.array(self.data_buffer["action_grip_left_position"]))
            f.create_dataset("observation/timestamp/data", data=np.array(self.data_buffer["timestamp"]))

            dt = h5py.special_dtype(vlen=np.dtype("uint8"))
            img_ds = f.create_dataset("camera_observations/color_images/camera_head", (length,), dtype=dt)
            for i, img_rgb in enumerate(self.data_buffer["img"]):
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                success, encoded_img = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if success:
                    img_ds[i] = encoded_img.flatten()
                else:
                    self.get_logger().error(f"Failed to encode image {i}")

            depth_ds = f.create_dataset("camera_observations/depth_images/camera_head", (length,), dtype=dt)
            for i, depth_mm in enumerate(self.data_buffer["depth"]):
                success, encoded_depth = cv2.imencode(".png", depth_mm)
                if success:
                    depth_ds[i] = encoded_depth.flatten()
                else:
                    self.get_logger().error(f"Failed to encode depth {i}")

        try:
            os.chmod(filename, 0o666)
        except PermissionError:
            pass
        self.get_logger().info(f"Data saved: {length} frames, dropped={self.dropped_frames}.")
        return filename

    @staticmethod
    def _find_ubt_sim_dir():
        project_dir = os.path.dirname(os.path.abspath(__file__))
        while os.path.basename(project_dir) != "ubt_sim":
            parent = os.path.dirname(project_dir)
            if parent == project_dir:
                return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            project_dir = parent
        return project_dir

    def _timer_save_callback(self):
        self.record_snapshot()


def build_parser():
    parser = argparse.ArgumentParser(description="Walker S2 零件抓取 + HDF5 数据采集")
    parser.add_argument("--part", default=DEFAULT_PART_NAME, help="零件名，例如 part_a_ori / part_a_red / part_b_blue / part_b_ori")
    parser.add_argument("--all-parts", action="store_true", help="按默认顺序依次抓取四个零件")
    parser.add_argument("--parts", nargs="+", default=None, help="自定义多零件抓取顺序；指定后会依次执行每个零件")
    parser.add_argument("--randomize-parts", action=argparse.BooleanOptionalAction, default=True, help="多零件抓取前先通过 bridge 随机化零件位置")
    parser.add_argument("--randomize-parts-topic", default=DEFAULT_RANDOMIZE_PARTS_TOPIC, help="零件随机化命令 topic")
    parser.add_argument("--randomize-seed", type=int, default=None, help="零件随机化 seed；不指定则每次自动生成随机 seed")
    parser.add_argument("--randomize-settle-time", type=float, default=0.5, help="随机化后等待物体状态稳定的时间，单位 s")
    parser.add_argument("--reset-scene", action="store_true", help="抓取前发布 /sim/cmd_reset 重置仿真场景；录制不会包含 reset 阶段")
    parser.add_argument("--reset-scene-settle-time", type=float, default=DEFAULT_RESET_SCENE_SETTLE_TIME, help="场景 reset 后等待状态稳定的时间，单位 s")
    parser.add_argument("--robot-init", action="store_true", help="抓取前调用 controller.py --init 等价流程，将机器人移动到 READY_POSE；录制不会包含 init 阶段")
    parser.add_argument("--robot-init-duration", type=float, default=DEFAULT_ROBOT_INIT_DURATION, help="机器人 READY_POSE 初始化轨迹时长，单位 s")
    parser.add_argument("--robot-init-settle-timeout", type=float, default=DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT, help="机器人 READY_POSE 初始化后等待关节收敛的超时时间，单位 s")
    parser.add_argument("--robot-init-tolerance", type=float, default=DEFAULT_ROBOT_INIT_TOLERANCE, help="机器人 READY_POSE 初始化到位判定阈值，单位 rad")
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
    parser.add_argument("--place-rot-weight", type=float, default=DEFAULT_PLACE_ROT_WEIGHT, help="放置阶段 IK 姿态误差权重；默认低于抓取阶段以减少姿态约束")
    parser.add_argument("--place-exit-left-offset", type=float, nargs=3, default=DEFAULT_PLACE_EXIT_LEFT_OFFSET_WORLD, metavar=("X", "Y", "Z"), help="松爪后先抬升，再按该 world 偏移离开箱体范围")
    parser.add_argument("--base-pos", type=float, nargs=3, default=DEFAULT_BASE_IN_WORLD_POS, metavar=("X", "Y", "Z"), help="URDF base 原点在 world frame 下的位置；默认来自 walker_s2_part_sorting.yaml")
    parser.add_argument("--world-to-base-quat", type=float, nargs=4, default=DEFAULT_WORLD_TO_BASE_QUAT_WXYZ, metavar=("W", "X", "Y", "Z"), help="world frame 到 URDF base frame 的旋转四元数 wxyz")
    parser.add_argument("--dry-run", action="store_true", help="只检查 IK 和关节限位，不下发控制")
    parser.add_argument("--save-hz", type=float, default=SAVE_HZ, help="数据录制频率 Hz")
    parser.add_argument("--save-only-success", action=argparse.BooleanOptionalAction, default=True, help="仅任务成功时保存 HDF5")
    return parser


def build_move_kwargs(args):
    return dict(
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
        place_rot_weight=args.place_rot_weight,
    )


def run_save_task(controller, part_monitor, recorder, args, move_kwargs):
    """执行抓取任务，并在第一条实际动作前开始录制。"""
    part_names = DEFAULT_PART_SEQUENCE if args.all_parts else args.parts
    if part_names is None:
        part_names = (args.part,)
    else:
        part_names = tuple(part_names)
    if not part_names:
        controller.get_logger().error("part_names must not be empty")
        return False

    timeout = args.timeout
    if not controller.wait_for_state(timeout=timeout):
        return False
    if not part_monitor.wait_for_part_states(timeout=timeout):
        return False

    if not args.dry_run:
        recorder.rgb_camera.wait_for_image(timeout=timeout)
        recorder.depth_camera.wait_for_image(timeout=timeout)

    if args.reset_scene:
        if not reset_scene(
            controller,
            part_monitor,
            timeout=timeout,
            settle_time=args.reset_scene_settle_time,
        ):
            return False

    if args.robot_init:
        if not initialize_robot_pose(
            controller,
            duration_sec=args.robot_init_duration,
            settle_timeout=args.robot_init_settle_timeout,
            tolerance=args.robot_init_tolerance,
            timeout=timeout,
        ):
            return False

    if args.randomize_parts:
        if not randomize_part_positions(
            controller,
            part_monitor,
            part_names=part_names,
            topic=args.randomize_parts_topic,
            timeout=timeout,
            settle_time=args.randomize_settle_time,
            seed=args.randomize_seed,
        ):
            return False

    started = {"value": False}

    def start_record_once():
        if started["value"]:
            return
        recorder.start_save_data()
        started["value"] = True

    ok = False
    try:
        for index, part_name in enumerate(part_names, start=1):
            controller.get_logger().info(f"Start part {index}/{len(part_names)}: {part_name}")
            ok = move_ee_by_waypoints(
                controller,
                part_monitor,
                part_name=part_name,
                before_execute_callback=start_record_once,
                **move_kwargs,
            )
            if not ok:
                controller.get_logger().error(f"Part {index}/{len(part_names)} failed: {part_name}")
                return False
            controller.get_logger().info(f"Part {index}/{len(part_names)} completed: {part_name}")
        controller.get_logger().info(f"Completed {len(part_names)} part(s): {list(part_names)}")
        return True
    finally:
        if started["value"]:
            recorder.stop_save_data()
            if ok or not args.save_only_success:
                recorder.save_data()
            else:
                recorder.get_logger().warning("Task failed. Recorded data will NOT be saved.")


def main():
    args = build_parser().parse_args()
    rclpy.init()

    controller = WalkerS2Controller(enable_ik=True, subscribe_images=False)
    part_monitor = PartStateMonitor()
    rgb_camera = Camera(topic=DEFAULT_IMAGE_RGB_TOPIC, node_name="walker_save_rgb_camera")
    depth_camera = Camera(topic=DEFAULT_IMAGE_DEPTH_TOPIC, node_name="walker_save_depth_camera")
    recorder = WalkerDataRecorder(rgb_camera, depth_camera, save_hz=args.save_hz)

    executor = MultiThreadedExecutor(num_threads=5)
    for node in (controller, part_monitor, rgb_camera, depth_camera, recorder):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        ok = run_save_task(controller, part_monitor, recorder, args, build_move_kwargs(args))
        if not ok:
            raise SystemExit(1)
    except KeyboardInterrupt:
        controller.get_logger().warning("Interrupted by user")
    finally:
        try:
            recorder.save_timer.cancel()
        except Exception:
            pass
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        recorder.destroy_node()
        depth_camera.destroy_node()
        rgb_camera.destroy_node()
        part_monitor.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
