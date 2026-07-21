#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 抓取任务 HDF5 数据录制节点（从 pick_part_save_data 抽出）。"""

import json
import os
import threading

import cv2
import h5py
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from .controller import (  # noqa: E402
    BODY_JOINT_NAMES,
    DEFAULT_COMMAND_TOPIC,
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
)

SAVE_HZ = 30.0
PLACEHOLDER_IMG_SHAPE = None   # 从第一帧实际分辨率推断
PLACEHOLDER_DEPTH_SHAPE = None
DEPTH_SENTINEL = 65535         # uint16 最大值，表示无效深度


class WalkerS2DataRecorder(Node):
    """Walker S2 抓取任务 HDF5 数据录制节点。"""

    # 非相机依赖的 buffer key（相机 key 在 __init__ 中根据 cameras dict 动态生成）
    _BASE_BUFFER_KEYS = (
        "joint_position", "joint_velocity", "joint_effort",
        "hand_right_position", "hand_right_velocity", "hand_right_effort",
        "hand_left_position", "hand_left_velocity", "hand_left_effort",
        "grip_right_position", "grip_right_velocity", "grip_right_current",
        "grip_right_state", "grip_right_error_code", "grip_right_homed",
        "grip_left_position", "grip_left_velocity", "grip_left_current",
        "grip_left_state", "grip_left_error_code", "grip_left_homed",
        "action_joint_position", "action_hand_right_position", "action_hand_left_position",
        "action_grip_right_position", "action_grip_left_position",
        "depth", "timestamp",
    )

    def __init__(
        self,
        cameras,
        depth_camera,
        save_hz=SAVE_HZ,
        node_name="walker_s2_pick_part_save_data_recorder",
    ):
        super().__init__(node_name)
        self.cameras = cameras              # {"stereo_left": Camera(...), ...}
        self.depth_camera = depth_camera
        self.save_hz = float(save_hz)

        # 动态 buffer keys：基础 keys + 每路相机的 img_<name>
        cam_img_keys = tuple(f"img_{name}" for name in cameras)
        self._BUFFER_KEYS = self._BASE_BUFFER_KEYS + cam_img_keys
        self.data_buffer = {k: [] for k in self._BUFFER_KEYS}
        self.is_saving = False
        self.dropped_frames = 0

        # 帧去重：每个相机的最近时间戳 + 上一帧缓存
        self._last_frame_ts = {name: -1.0 for name in cameras}
        self._last_img = {}        # cam_name → 最近一次有效帧（复用去重帧）
        self._last_depth_ts = -1.0

        # 图像分辨率从第一帧推断
        self._img_shape = None
        self._depth_shape = None

        # Episode 元数据（由外部在 start_save_data 前设置）
        self._episode_meta = {}

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
            "left": {"pos": 0.0, "vel": 0.0, "cur": 0.0,
                      "grip_state": 0, "error_code": 0, "homed": 0},
            "right": {"pos": 0.0, "vel": 0.0, "cur": 0.0,
                       "grip_state": 0, "error_code": 0, "homed": 0},
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

    def set_episode_metadata(self, **kwargs):
        """设置当前 episode 的元数据（start_save_data 前调用）。"""
        self._episode_meta = dict(kwargs)

    def start_save_data(self):
        """开始录制数据。"""
        for key in self._BUFFER_KEYS:
            self.data_buffer[key].clear()
        self.dropped_frames = 0
        # 重置帧去重时间戳 + 上一帧缓存
        for name in self.cameras:
            self._last_frame_ts[name] = -1.0
        self._last_img.clear()
        self._last_depth_ts = -1.0
        self._img_shape = None
        self._depth_shape = None
        self.is_saving = True
        self.get_logger().info(f"Started recording Walker S2 data at {self.save_hz:.0f}Hz")

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
                    "grip_right_state": int(self._grip_state["right"].get("grip_state", 0)),
                    "grip_right_error_code": int(self._grip_state["right"].get("error_code", 0)),
                    "grip_right_homed": int(self._grip_state["right"].get("homed", 0)),
                    "grip_left_position": float(self._grip_state["left"].get("pos", 0.0)),
                    "grip_left_velocity": float(self._grip_state["left"].get("vel", 0.0)),
                    "grip_left_current": float(self._grip_state["left"].get("cur", 0.0)),
                    "grip_left_state": int(self._grip_state["left"].get("grip_state", 0)),
                    "grip_left_error_code": int(self._grip_state["left"].get("error_code", 0)),
                    "grip_left_homed": int(self._grip_state["left"].get("homed", 0)),
                    "action_joint_position": self._action_joint_position.copy(),
                    "action_hand_right_position": self._action_hand_position["right"].copy(),
                    "action_hand_left_position": self._action_hand_position["left"].copy(),
                    "action_grip_right_position": float(self._action_grip_position["right"]),
                    "action_grip_left_position": float(self._action_grip_position["left"]),
                    "timestamp": self.get_clock().now().nanoseconds / 1e9,
                }

            # 多路相机图像采集（含帧去重）
            for cam_name, cam in self.cameras.items():
                snapshot[f"img_{cam_name}"] = self._get_rgb_image(cam, cam_name)

            snapshot["depth"] = self._get_depth_image()
        except Exception as e:
            self.dropped_frames += 1
            self.get_logger().error(f"Error recording snapshot (dropped={self.dropped_frames}): {e}")
            return

        for key, value in snapshot.items():
            self.data_buffer[key].append(value)

    def _get_rgb_image(self, cam, cam_name):
        """获取单路 RGB 图像，含帧去重和时间戳校验。

        无新帧时复用上一帧（而非返回 None），避免低帧率相机出现黑帧。
        """
        img = cam.get_latest_image()
        if img is None:
            # 无相机数据 → 用上一帧或黑色占位
            cached = self._last_img.get(cam_name)
            if cached is not None:
                return cached
            if self._img_shape is None:
                return None
            return np.zeros(self._img_shape, dtype=np.uint8)

        info = cam.get_image_info() or {}
        ts = info.get("timestamp", 0.0)

        # 帧去重：时间戳未前进 → 复用上一帧
        if ts <= self._last_frame_ts.get(cam_name, -1.0):
            cached = self._last_img.get(cam_name)
            if cached is not None:
                return cached
            if self._img_shape is None:
                return None
            return np.zeros(self._img_shape, dtype=np.uint8)

        self._last_frame_ts[cam_name] = ts

        # 从第一帧推断分辨率
        if self._img_shape is None and img is not None:
            self._img_shape = img.shape[:2] + (3,) if img.ndim == 3 else img.shape + (3,)

        encoding = info.get("encoding", "rgb8")
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        elif encoding in ("bgr8", "yuv422"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        result = np.asarray(img, dtype=np.uint8).copy()
        self._last_img[cam_name] = result  # 缓存最新有效帧
        return result

    def _get_depth_image(self):
        """获取深度图像，无数据时使用哨兵值 65535。"""
        depth = self.depth_camera.get_latest_image()
        if depth is None:
            if self._depth_shape is None:
                return None
            return np.full(self._depth_shape, DEPTH_SENTINEL, dtype=np.uint16)

        info = self.depth_camera.get_image_info() or {}
        ts = info.get("timestamp", 0.0)
        if ts <= self._last_depth_ts:
            return None
        self._last_depth_ts = ts

        depth = np.asarray(depth)
        if self._depth_shape is None:
            self._depth_shape = depth.shape

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

        meta = self._episode_meta
        part_name = meta.get("part_name", "unknown")
        ts = self.get_clock().now().seconds_nanoseconds()[0]

        dataset_root = os.path.join(self._find_ubt_sim_dir(), "dataset", "walker_s2")
        base_dir_name = os.path.join(dataset_root, f"{ts}")
        os.makedirs(base_dir_name, mode=0o777, exist_ok=True)

        # 如果只有单个 part，直接放 ts/ 下；多 part 放子目录
        episode_count = meta.get("episode_count", 1)
        if episode_count <= 1:
            dir_name = base_dir_name
        else:
            idx = meta.get("episode_index", 1)
            dir_name = os.path.join(base_dir_name, f"episode_{idx:03d}_{part_name}")

        os.makedirs(dir_name, mode=0o777, exist_ok=True)
        for path in [dataset_root, base_dir_name, dir_name]:
            try:
                os.chmod(path, 0o777)
            except (PermissionError, OSError):
                pass
        filename = os.path.join(dir_name, "trajectory.hdf5")
        self.get_logger().info(f"Saving {length} frames to {filename}...")

        with h5py.File(filename, "w") as f:
            # ---- Observation ----
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
            f.create_dataset("observation/grip_right_state/data", data=np.array(self.data_buffer["grip_right_state"]))
            f.create_dataset("observation/grip_right_error_code/data", data=np.array(self.data_buffer["grip_right_error_code"]))
            f.create_dataset("observation/grip_right_homed/data", data=np.array(self.data_buffer["grip_right_homed"]))
            f.create_dataset("observation/grip_left_position/data", data=np.array(self.data_buffer["grip_left_position"]))
            f.create_dataset("observation/grip_left_velocity/data", data=np.array(self.data_buffer["grip_left_velocity"]))
            f.create_dataset("observation/grip_left_current/data", data=np.array(self.data_buffer["grip_left_current"]))
            f.create_dataset("observation/grip_left_state/data", data=np.array(self.data_buffer["grip_left_state"]))
            f.create_dataset("observation/grip_left_error_code/data", data=np.array(self.data_buffer["grip_left_error_code"]))
            f.create_dataset("observation/grip_left_homed/data", data=np.array(self.data_buffer["grip_left_homed"]))
            f.create_dataset("action/joint_state/position/data", data=np.array(self.data_buffer["action_joint_position"]))
            f.create_dataset("action/hand_right_position/data", data=np.array(self.data_buffer["action_hand_right_position"]))
            f.create_dataset("action/hand_left_position/data", data=np.array(self.data_buffer["action_hand_left_position"]))
            f.create_dataset("action/grip_right_position/data", data=np.array(self.data_buffer["action_grip_right_position"]))
            f.create_dataset("action/grip_left_position/data", data=np.array(self.data_buffer["action_grip_left_position"]))
            f.create_dataset("observation/timestamp/data", data=np.array(self.data_buffer["timestamp"]))

            # ---- Multi-camera color images (JPEG) ----
            dt = h5py.special_dtype(vlen=np.dtype("uint8"))
            for cam_name in self.cameras:
                key = f"img_{cam_name}"
                ds = f.create_dataset(f"camera_observations/color_images/{cam_name}", (length,), dtype=dt)
                for i, img_rgb in enumerate(self.data_buffer[key]):
                    if img_rgb is None:
                        self.get_logger().warning(f"Frame {i} missing for camera {cam_name}, using placeholder")
                        img_rgb = np.zeros(self._img_shape or (480, 640, 3), dtype=np.uint8)
                    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                    success, encoded_img = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    if success:
                        ds[i] = encoded_img.flatten()
                    else:
                        self.get_logger().error(f"Failed to encode {cam_name} image {i}")

            # ---- Depth images (PNG) ----
            depth_ds = f.create_dataset("camera_observations/depth_images/camera_head", (length,), dtype=dt)
            for i, depth_mm in enumerate(self.data_buffer["depth"]):
                if depth_mm is None:
                    depth_mm = np.full(self._depth_shape or (480, 640), DEPTH_SENTINEL, dtype=np.uint16)
                success, encoded_depth = cv2.imencode(".png", depth_mm)
                if success:
                    depth_ds[i] = encoded_depth.flatten()
                else:
                    self.get_logger().error(f"Failed to encode depth {i}")

            # ---- Episode metadata as HDF5 root attributes ----
            f.attrs["robot_type"] = "walker_s2"
            f.attrs["task"] = "part_sorting"
            f.attrs["part_name"] = str(part_name)
            f.attrs["side"] = str(meta.get("side", "right"))
            f.attrs["fps"] = float(self.save_hz)
            f.attrs["auto_grasp"] = str(meta.get("auto_grasp", False))
            f.attrs["success"] = str(meta.get("success", False))
            f.attrs["camera_names"] = json.dumps(list(self.cameras.keys()))
            f.attrs["joint_names"] = json.dumps(BODY_JOINT_NAMES)
            f.attrs["created_at"] = self.get_clock().now().nanoseconds / 1e9

        try:
            os.chmod(filename, 0o666)
        except (PermissionError, OSError):
            self.get_logger().warning(f"Cannot chmod {filename}, file may be owned by root")
        self.get_logger().info(f"Data saved: {length} frames, dropped={self.dropped_frames}.")
        return filename

    @staticmethod
    def _find_ubt_sim_dir():
        # 优先使用环境变量（容器内 / 部署环境可显式指定）
        env = os.environ.get("UBT_SIM_DATASET_DIR")
        if env and os.path.isdir(env):
            return env
        # 回退：从当前文件向上查找名为 "ubt_sim" 的目录
        project_dir = os.path.dirname(os.path.abspath(__file__))
        while os.path.basename(project_dir) != "ubt_sim":
            parent = os.path.dirname(project_dir)
            if parent == project_dir:
                return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            project_dir = parent
        return project_dir

    def _timer_save_callback(self):
        self.record_snapshot()
