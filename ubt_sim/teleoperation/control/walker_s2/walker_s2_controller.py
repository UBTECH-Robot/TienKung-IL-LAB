#!/usr/bin/env python3
"""
Walker S2 机器人直接控制脚本

从 executor_node_sdk.py 提取，移除 VLA 推理依赖（不订阅 Gr00tMotionChunk），
保留核心的安全检查、线性插值、关节锁定、200Hz 发布逻辑，
提供 Python API 直接控制真机。

控制方法：方法 2（SDK 控制器，RobotCommand/JointCmd，MODE_POSITION=2）
话题：/mc/sdk/robot_command（发布），/mc/sdk/robot_state（订阅）

【运行前置条件】

1. 必须先在运控容器中启动运控并切换到 SDK 控制器：

    docker exec -it walker-motion.manipulation_robot_app-1 bash
    source /opt/walker/setup.bash
    rosa run t800_mc_server start_mc_client
    rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps

2. 确保机器人处于安全位置（先用遥控器移到安全位置再启动控制器）

3. 在控制容器中执行此脚本前需要 source 环境：

    source /home/ubt/additional/scripts/setup.sh

【使用示例】

    # 命令行：
    python3 robot_control.py --print-state            # 仅打印当前关节位置
    python3 robot_control.py --demo                   # 运行安全演示（小幅运动）

    # Python API：
    from robot_control import RobotController
    import rclpy, threading
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    controller = RobotController(
        lock_joints=['head_pitch_joint', 'head_yaw_joint', 'waist_yaw_joint'],
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    threading.Thread(target=executor.spin, daemon=True).start()

    controller.wait_for_state(timeout=5.0)
    pos = controller.get_current_position()
    target = pos.copy()
    target[controller.joint_index('R_elbow_yaw_joint')] += 0.1
    controller.move_to_position(target, duration_sec=2.0)

    rclpy.shutdown()
"""

import argparse
import json
import os
import threading
import time
from collections import deque
from typing import Optional

import yaml

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from ecat_task_msgs.msg import GripCmd, GripStatus
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
except ImportError as exc:
    raise ImportError(
        "Walker S2 ROS2 SDK messages not found. Source ROS2 and the vendored Walker SDK messages first, e.g.\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash\n"
        "Then run this control script with /usr/bin/python3, not Isaac Sim Python."
    ) from exc

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool, String

# ============================================================================
# 常量
# ============================================================================

DEFAULT_COMMAND_TOPIC = "/mc/sdk/robot_command"
DEFAULT_STATE_TOPIC = "/mc/sdk/robot_state"
DEFAULT_LEFT_HAND_COMMAND_TOPIC = "/mc/left_hand/command"
DEFAULT_RIGHT_HAND_COMMAND_TOPIC = "/mc/right_hand/command"
DEFAULT_LEFT_HAND_STATE_TOPIC = "/mc/left_hand/joint_states"
DEFAULT_RIGHT_HAND_STATE_TOPIC = "/mc/right_hand/joint_states"
DEFAULT_LEFT_GRIP_COMMAND_TOPIC = "/ecat/left_grip/cmd"
DEFAULT_RIGHT_GRIP_COMMAND_TOPIC = "/ecat/right_grip/cmd"
DEFAULT_LEFT_GRIP_STATE_TOPIC = "/ecat/left_grip/state"
DEFAULT_RIGHT_GRIP_STATE_TOPIC = "/ecat/right_grip/state"
DEFAULT_RESET_TOPIC = "/sim/cmd_reset"
DEFAULT_FINGER_LINK_STATES_TOPIC = "/sim/finger_link_states"
DEFAULT_IMAGE_RGB_TOPIC = "/sensor/camera/stereo/color/raw"
DEFAULT_IMAGE_DEPTH_TOPIC = "/sensor/camera/stereo/depth/raw"
DEFAULT_CONTROL_HZ = 200
DEFAULT_MAX_JOINT_SPEED = 6.28  # rad/s，安全速度上限
DEFAULT_LOCK_JOINTS = ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"]

# ============================================================================
# 关节定义（原 utars_clamp_and_place_large_bio_box_in_test_field.yaml 中的
# actions.joints 段，硬编码以消除对配置文件的依赖）
# ============================================================================

BODY_JOINT_NAMES = [
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
]

# 关节限位（rad），来源：Walker S2 硬件规格书
# 键 = 关节名（与 BODY_JOINT_NAMES 一致），值 = (lower, upper)
BODY_JOINT_LIMITS = {
    "L_elbow_roll_joint":       (-2.6180, 0.0),
    "L_elbow_yaw_joint":        (-2.9147, 2.9147),
    "L_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "L_shoulder_roll_joint":    (-1.85,   0.0873),
    "L_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "L_wrist_pitch_joint":      (-1.5882, 1.5882),
    "L_wrist_roll_joint":       (-1.9897, 1.9897),
    "R_elbow_roll_joint":       (-2.6180, 0.0),
    "R_elbow_yaw_joint":        (-2.9147, 2.9147),
    "R_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "R_shoulder_roll_joint":    (-1.85,   0.0873),
    "R_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}

# V4 手部关节限位（rad），左右手相同
# 键 = 短名（去掉 left_/right_ 前缀），查找时 removeprefix 即可
V4_HAND_JOINT_LIMITS = {
    "thumb_swing":  (0.0, 2.11),
    "thumb_mcp":    (0.0, 1.85),
    "thumb_pip":    (0.0, 1.09),
    "index_mcp":    (0.0, 1.71),
    "middle_mcp":   (0.0, 1.71),
    "ring_mcp":     (0.0, 1.71),
    "little_mcp":   (0.0, 1.71),
}


# ============================================================================
# 头部周期运动测试参数
# 参考：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/example/src/walker_s2/
#       low_level/pub_head_command.cpp
#
# 原 SDK demo：500Hz 发布，position = sin(time_cnt) * 0.5，time_cnt += 0.002
# 对应连续函数：position = sin(2π * t / T) * amplitude
# 其中：振幅 0.5 rad，时间步 0.002s（500Hz），周期 T = 2π ≈ 6.28s
# ============================================================================

HEAD_TEST_AMPLITUDE = 0.5    # 振幅（弧度），约 28.6°
HEAD_TEST_PERIOD = 2 * np.pi  # 周期（秒），约 6.28s
HEAD_TEST_DEFAULT_CYCLES = 2  # 默认运动周期数

# ============================================================================
# V4 手部周期运动测试参数
# 参考：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/example/src/walker_s2/
#       low_level/pub_hand_v4_command.cpp
#
# V4 手 = 单手 7 关节（含 thumb_pip，区别于 V3 手的 6 关节）
# 原 SDK demo：500Hz 发布，position = sin(time_cnt + i * 0.2) * 0.6
#               每个关节相位差 0.2 rad，mode=5（手部控制器自定义模式）
#
# 注意：
#   - 手部走独立通路：JointCommand 消息 + /mc/{left,right}_hand/command 话题
#   - 不需要 switch_controller config_mc_walker_s2_v1_sps（手部控制器始终监听）
#   - 与身体关节完全独立，不在 YAML config 中
# ============================================================================

V4_HAND_LEFT_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_thumb_pip",      # V4 独有，V3 没有此关节
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

V4_HAND_RIGHT_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_thumb_pip",     # V4 独有
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

V4_HAND_TEST_AMPLITUDE = 0.6        # 振幅（rad），与 SDK demo 一致
V4_HAND_TEST_PERIOD = 2 * np.pi     # 周期（s），与 SDK demo 一致（time_cnt += 0.002 @500Hz）
V4_HAND_TEST_PHASE_DIFF = 0.2       # 关节间相位差（rad），与 SDK demo 一致
V4_HAND_TEST_DEFAULT_CYCLES = 2     # 默认循环数
V4_HAND_TEST_HZ = 200               # 手部测试发布频率
V4_HAND_LEFT_TOPIC = DEFAULT_LEFT_HAND_COMMAND_TOPIC
V4_HAND_RIGHT_TOPIC = DEFAULT_RIGHT_HAND_COMMAND_TOPIC
V4_HAND_LEFT_STATE_TOPIC = DEFAULT_LEFT_HAND_STATE_TOPIC
V4_HAND_RIGHT_STATE_TOPIC = DEFAULT_RIGHT_HAND_STATE_TOPIC

# 手部关节查找表：side → (joint_names_list, publisher_topic)
V4_HAND_JOINT_MAP = {
    "left": V4_HAND_LEFT_JOINTS,
    "right": V4_HAND_RIGHT_JOINTS,
}

# 手部预设姿态（用于 --hand-open / --hand-close）
V4_HAND_OPEN_POSE = {name: 0.0 for name in V4_HAND_JOINT_LIMITS}
V4_HAND_CLOSE_POSE = {name: hi for name, (_, hi) in V4_HAND_JOINT_LIMITS.items()}
GRIP_OPENING_MIN_M = 0.0
GRIP_OPENING_MAX_M = 0.05
GRIP_DEFAULT_VEL = 0.05
GRIP_DEFAULT_FORCE = 20.0
HOME_POSE = {name: 0.0 for name in BODY_JOINT_NAMES}
LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
]

# ============================================================================
# 预备姿态（双臂抬起预备抓取的站立位姿）
# ============================================================================

READY_POSE = {
    "L_elbow_roll_joint":       -1.700,
    "L_elbow_yaw_joint":        1.500,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.700,
    "R_elbow_yaw_joint":        -1.500,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# 初始化分段 1a：
READY_STAGE_1_PITCH_ROLL_POSE = {

    "L_shoulder_yaw_joint": -1.5600,
    "R_shoulder_yaw_joint": 1.5600,
    "L_elbow_yaw_joint": 1.5000,
    "R_elbow_yaw_joint": -1.5000,
}

# 初始化分段 1b:
READY_STAGE_1_ELBOW_YAW_POSE = {
    "L_shoulder_pitch_joint":   -2.000,
    "R_shoulder_pitch_joint":   2.000,
    "L_wrist_pitch_joint": 0.8000,
    "R_wrist_pitch_joint": -0.8000,
    "L_elbow_roll_joint":        -2.6000,
    "R_elbow_roll_joint":        -2.6000,
}

# 初始化分段 2：肩 pitch 回到最终预备姿态，再执行完整 READY_POSE
READY_STAGE_2_POSE = {
    "L_shoulder_pitch_joint": READY_POSE["L_shoulder_pitch_joint"],
    "R_shoulder_pitch_joint": READY_POSE["R_shoulder_pitch_joint"],
}

# ============================================================================
# 主控制器
# ============================================================================


class WalkerS2Controller(Node):
    """Walker S2 SDK 控制器节点

    职责：
        1. 订阅 /mc/sdk/robot_state 维护最新关节位置
        2. 提供 move_to_position / execute_trajectory 等 API
        3. 200Hz 定时器发布 RobotCommand 到 /mc/sdk/robot_command
        4. 安全检查：最大关节速度
        5. 关节锁定：发布时跳过指定关节
        6. 关节限位：超限时自动裁剪到限位边界
    """

    def __init__(
        self,
        node_name: str = "walker_s2_controller",
        command_topic: Optional[str] = None,
        state_topic: Optional[str] = None,
        config_path: Optional[str] = None,
        control_hz: float = DEFAULT_CONTROL_HZ,
        lock_joints=None,
        max_joint_speed: float = DEFAULT_MAX_JOINT_SPEED,
        enable_safety_check: bool = True,
        enable_limit_check: bool = True,
        subscribe_images: bool = True,
        enable_ik: bool = False,
        ik_urdf_path: Optional[str] = None,
        ik_auto_initialize: bool = True,
    ):
        super().__init__(node_name)

        self._config = self._load_config(config_path)
        command_topic = command_topic or self._get_topic("sub", "command", DEFAULT_COMMAND_TOPIC)
        state_topic = state_topic or self._get_topic("pub", "state", DEFAULT_STATE_TOPIC)
        left_hand_topic = self._get_topic("sub", "left_hand_command", DEFAULT_LEFT_HAND_COMMAND_TOPIC)
        right_hand_topic = self._get_topic("sub", "right_hand_command", DEFAULT_RIGHT_HAND_COMMAND_TOPIC)
        left_hand_state_topic = self._get_topic("pub", "left_hand_state", DEFAULT_LEFT_HAND_STATE_TOPIC)
        right_hand_state_topic = self._get_topic("pub", "right_hand_state", DEFAULT_RIGHT_HAND_STATE_TOPIC)
        left_grip_topic = self._get_topic("sub", "left_grip_command", DEFAULT_LEFT_GRIP_COMMAND_TOPIC)
        right_grip_topic = self._get_topic("sub", "right_grip_command", DEFAULT_RIGHT_GRIP_COMMAND_TOPIC)
        left_grip_state_topic = self._get_topic("pub", "left_grip_state", DEFAULT_LEFT_GRIP_STATE_TOPIC)
        right_grip_state_topic = self._get_topic("pub", "right_grip_state", DEFAULT_RIGHT_GRIP_STATE_TOPIC)
        reset_topic = self._get_topic("sub", "reset", DEFAULT_RESET_TOPIC)
        finger_link_states_topic = self._get_topic("pub", "finger_link_states", DEFAULT_FINGER_LINK_STATES_TOPIC)
        image_rgb_topic = self._get_topic("pub", "image_rgb", DEFAULT_IMAGE_RGB_TOPIC)
        image_depth_topic = self._get_topic("pub", "image_depth", DEFAULT_IMAGE_DEPTH_TOPIC)

        # 关节配置（硬编码，原来自 YAML 文件）
        self.all_joints = list(BODY_JOINT_NAMES)
        self.n_joints = len(self.all_joints)

        # 控制参数
        self.control_hz = control_hz
        self.timer_period = 1.0 / control_hz
        self.max_joint_speed = max_joint_speed
        self.enable_safety_check = enable_safety_check
        self.enable_limit_check = enable_limit_check

        # 锁定关节
        self.lock_joints = set(DEFAULT_LOCK_JOINTS if lock_joints is None else lock_joints)

        # 状态缓冲
        self.robot_states_buffer = deque(maxlen=1)
        self.robot_states_buffer_lock = threading.Lock()

        # 轨迹状态
        self.trajectory_lock = threading.Lock()
        self.current_trajectory = np.empty((0, self.n_joints), dtype=float)
        self.current_index = 0
        self.is_publishing = False
        self.safety_violation = False
        self.current_publish_joints = None
        self.publish_changed_epsilon = 1e-6

        # QoS
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅/发布
        self.state_sub = self.create_subscription(
            RobotState, state_topic, self._state_callback, qos_sub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.command_pub = self.create_publisher(
            RobotCommand, command_topic, qos_pub
        )

        # 手部发布者（V4 手专用，走 JointCommand 通路，与身体控制独立）
        self.left_hand_pub = self.create_publisher(
            JointCommand, left_hand_topic, qos_pub
        )
        self.right_hand_pub = self.create_publisher(
            JointCommand, right_hand_topic, qos_pub
        )
        self._hand_pubs = {"left": self.left_hand_pub, "right": self.right_hand_pub}

        # 二指夹爪发布者（ECAT GripCmd 通路，仿真中控制 PGC 两指夹爪）
        self.left_grip_pub = self.create_publisher(
            GripCmd, left_grip_topic, qos_pub
        )
        self.right_grip_pub = self.create_publisher(
            GripCmd, right_grip_topic, qos_pub
        )
        self._grip_pubs = {"left": self.left_grip_pub, "right": self.right_grip_pub}

        # 手部状态订阅（/mc/{left,right}_hand/joint_states → sensor_msgs/JointState）
        self._hand_states = {}       # {"left": np.array(7), "right": np.array(7)}
        self._hand_state_lock = threading.Lock()
        self._hand_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
        self._grip_states = {}       # {"left": {pos, vel, cur, ...}, "right": {...}}
        self._grip_state_lock = threading.Lock()
        self._grip_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
        self._finger_link_states = None
        self._finger_link_states_lock = threading.Lock()
        self._finger_link_states_received = threading.Event()

        self.ik_solver = None
        self._ik_initialized = False
        self._ik_lock = threading.Lock()
        self._ik_urdf_path = ik_urdf_path

        self.left_hand_state_sub = self.create_subscription(
            JointState, left_hand_state_topic,
            lambda msg: self._hand_state_callback("left", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.right_hand_state_sub = self.create_subscription(
            JointState, right_hand_state_topic,
            lambda msg: self._hand_state_callback("right", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.left_grip_state_sub = self.create_subscription(
            GripStatus, left_grip_state_topic,
            lambda msg: self._grip_state_callback("left", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.right_grip_state_sub = self.create_subscription(
            GripStatus, right_grip_state_topic,
            lambda msg: self._grip_state_callback("right", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.finger_link_states_sub = self.create_subscription(
            String, finger_link_states_topic, self._finger_link_state_callback,
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.reset_pub = self.create_publisher(Bool, reset_topic, 1)
        self.latest_img = None
        self.latest_depth = None
        if subscribe_images:
            self.image_sub = self.create_subscription(
                Image, image_rgb_topic, self._image_callback, qos_sub,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.depth_sub = self.create_subscription(
                Image, image_depth_topic, self._depth_callback, qos_sub,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        # 200Hz 控制定时器
        self.control_timer = self.create_timer(
            self.timer_period, self._control_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        if enable_ik and ik_auto_initialize:
            if not self.initialize_ik():
                self.get_logger().warning("Walker S2 IK auto-initialization failed; joint-space control remains available")

        self.get_logger().info(
            f"WalkerS2Controller initialized: {self.n_joints} joints, "
            f"{control_hz}Hz, locked={sorted(self.lock_joints)}, "
            f"limit_check={self.enable_limit_check}"
        )

    @staticmethod
    def _load_config(config_path: Optional[str] = None) -> dict:
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, os.pardir, "bridges", "walker_s2_bridge_config.yaml"
            )
        try:
            with open(config_path) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _get_topic(self, section: str, key: str, default: str) -> str:
        # section/key names follow walker_s2_bridge_config.yaml from the bridge perspective.
        try:
            return self._config["topics"][section][key]["topic"]
        except (KeyError, TypeError):
            return default

    # ========================================================================
    # 公开 API
    # ========================================================================

    def wait_for_state(self, timeout=5.0):
        """阻塞等待第一个机器人状态消息。

        Args:
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        start = time.time()
        while time.time() - start < timeout:
            with self.robot_states_buffer_lock:
                if len(self.robot_states_buffer) > 0:
                    self.get_logger().info(
                        f"Robot state received (took {time.time() - start:.2f}s)"
                    )
                    return True
            time.sleep(0.05)
        self.get_logger().error(f"Timeout waiting for robot state ({timeout}s)")
        return False

    def get_current_position(self):
        """获取最新关节位置（numpy 数组，n_joints 维），None 表示无数据"""
        with self.robot_states_buffer_lock:
            if len(self.robot_states_buffer) > 0:
                return self.robot_states_buffer[-1].copy()
        return None

    @staticmethod
    def _default_ik_urdf_path():
        return os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, os.pardir, os.pardir,
                "assets", "robots", "walker_s2", "s2.urdf",
            )
        )

    @staticmethod
    def _xyzrpy_dict(xyzrpy):
        values = [float(v) for v in xyzrpy]
        return dict(zip(["x", "y", "z", "roll", "pitch", "yaw"], values))

    def _current_arm_values(self, joint_names):
        current = self.get_current_position()
        if current is not None:
            pos_map = dict(zip(self.all_joints, current))
            return [float(pos_map.get(name, READY_POSE.get(name, 0.0))) for name in joint_names]
        return [float(READY_POSE.get(name, 0.0)) for name in joint_names]

    def initialize_ik(self, urdf_path=None, left_neutral=None, right_neutral=None, save_current_as_initial=True):
        """初始化 Walker S2 双臂 IK。

        IK 目标格式为 [x, y, z, roll, pitch, yaw]，单位 m/rad，坐标系为
        Walker S2 URDF 机器人基座坐标系，不是 /sim/finger_link_states 的 world 坐标系。
        """
        resolved_urdf = (
            urdf_path
            or self._ik_urdf_path
            or os.environ.get("WALKER_S2_IK_URDF")
            or self._default_ik_urdf_path()
        )
        resolved_urdf = os.path.abspath(resolved_urdf)
        if not os.path.exists(resolved_urdf):
            self.get_logger().error(f"Walker S2 IK URDF not found: {resolved_urdf}")
            return False

        try:
            try:
                from .walker_s2_ik import WalkerS2IK
            except ImportError:
                from walker_s2_ik import WalkerS2IK
            solver = WalkerS2IK(
                resolved_urdf,
                joint_limits=BODY_JOINT_LIMITS,
                joint_limit_margin=0.0,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize Walker S2 IK: {exc}")
            return False

        with self._ik_lock:
            self.ik_solver = solver
            self._ik_urdf_path = resolved_urdf
            self._ik_initialized = True
            self._sync_ik_from_current_state_locked()
            left = left_neutral if left_neutral is not None else self._current_arm_values(LEFT_ARM_JOINTS)
            right = right_neutral if right_neutral is not None else self._current_arm_values(RIGHT_ARM_JOINTS)
            self.ik_solver.set_neutral_config(left, right)
            if save_current_as_initial:
                self.ik_solver.save_initial_q()
        self.get_logger().info(
            f"Walker S2 IK initialized with URDF: {resolved_urdf}; "
            f"joint_limits_constrained={solver._joint_limit_override_count}"
        )
        return True

    def reset_ik(self, save_current_as_initial=True):
        if self.ik_solver is None:
            return self.initialize_ik(save_current_as_initial=save_current_as_initial)
        with self._ik_lock:
            self.ik_solver.reset_runtime_state()
            self._sync_ik_from_current_state_locked()
            if save_current_as_initial:
                self.ik_solver.save_initial_q()
        return True

    def _sync_ik_from_current_state_locked(self):
        if self.ik_solver is None:
            return False
        current = self.get_current_position()
        if current is None:
            return False
        self.ik_solver.sync_joint_positions(self.all_joints, current.tolist())
        return True

    def _sync_ik_from_current_state(self):
        with self._ik_lock:
            return self._sync_ik_from_current_state_locked()

    def _ensure_ik_initialized(self):
        if self.ik_solver is not None and self._ik_initialized:
            return True
        return self.initialize_ik()

    def get_ee_pose(self, side, as_dict=False):
        """获取当前单臂末端 [x,y,z,roll,pitch,yaw]（URDF base frame）。"""
        if side not in ("left", "right"):
            raise ValueError(f"Invalid arm side: {side}")
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            pose = self.ik_solver.get_ee_pose(side)
        return self._xyzrpy_dict(pose) if as_dict else pose

    def get_ee_poses(self, as_dict=False):
        """获取左右臂末端 [x,y,z,roll,pitch,yaw]（URDF base frame）。"""
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            poses = self.ik_solver.get_both_ee_poses()
        if as_dict:
            return {side: self._xyzrpy_dict(pose) for side, pose in poses.items()}
        return poses

    def wait_until_position(self, target_position, timeout=5.0, tolerance=0.05, ignored_joints=None):
        """等待实际关节位置收敛到目标附近。

        execute_trajectory(wait=True) 只表示轨迹点发布完毕；仿真/真机实际关节
        还需要继续收敛。此方法基于 RobotState 检查实际位置误差。
        """
        target = np.array(target_position, dtype=float)
        if target.shape != (self.n_joints,):
            self.get_logger().error(f"Target shape {target.shape} != ({self.n_joints},)")
            return False, []

        ignored = set(ignored_joints or [])
        check_indices = [i for i, name in enumerate(self.all_joints) if name not in ignored]
        deadline = time.time() + timeout
        last_pos = None

        while time.time() < deadline:
            pos = self.get_current_position()
            if pos is not None:
                last_pos = pos
                err = np.abs(pos - target)
                if check_indices and float(np.max(err[check_indices])) <= tolerance:
                    return True, []
            time.sleep(0.05)

        if last_pos is None:
            return False, [(name, None, float(target[i]), None) for i, name in enumerate(self.all_joints)]

        err = np.abs(last_pos - target)
        misses = [
            (self.all_joints[i], float(last_pos[i]), float(target[i]), float(err[i]))
            for i in check_indices
            if err[i] > tolerance
        ]
        misses.sort(key=lambda item: item[3], reverse=True)
        return False, misses

    def joint_index(self, joint_name):
        """获取关节名对应的索引"""
        if joint_name not in self.all_joints:
            raise ValueError(f"Unknown joint: {joint_name}")
        return self.all_joints.index(joint_name)

    @property
    def joint_names(self):
        """所有关节名列表（只读）"""
        return list(self.all_joints)

    # ---- 手部关节 API ----

    def hand_joint_names(self, side):
        """获取指定手的关节名列表。

        Args:
            side: "left" 或 "right"
        Returns:
            list[str]: 关节名列表
        Raises:
            ValueError: side 不是 "left" 或 "right"
        """
        if side not in V4_HAND_JOINT_MAP:
            raise ValueError(f"Invalid side '{side}', expected 'left' or 'right'")
        return list(V4_HAND_JOINT_MAP[side])

    def hand_joint_index(self, side, joint_name):
        """获取手部关节名在指定手中的索引。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名（如 "left_thumb_swing"）或短名（如 "thumb_swing"）
        Returns:
            int: 索引
        Raises:
            ValueError: 关节名无效
        """
        names = V4_HAND_JOINT_MAP[side]
        # 支持全名和短名两种写法
        full_name = f"{side}_{joint_name}" if not joint_name.startswith(side + "_") else joint_name
        if full_name in names:
            return names.index(full_name)
        # 也试试直接匹配
        if joint_name in names:
            return names.index(joint_name)
        raise ValueError(f"Unknown hand joint: {joint_name} (side={side})")

    def wait_for_hand_state(self, side=None, timeout=5.0):
        """阻塞等待手部状态消息。

        Args:
            side: "left"、"right" 或 None（等待双手）
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            remaining = max(0.0, deadline - time.time())
            if not self._hand_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} hand state ({remaining:.1f}s)")
                return False
        return True

    def get_hand_position(self, side):
        """获取指定手的手指关节当前位置。

        Args:
            side: "left" 或 "right"
        Returns:
            numpy.ndarray: 7 维位置数组，None 表示无数据
        """
        with self._hand_state_lock:
            if side in self._hand_states:
                return self._hand_states[side].copy()
        return None

    def get_hand_joint_position(self, side, joint_name):
        """获取指定手的单个手指关节当前位置。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
        Returns:
            float: 当前位置（rad），None 表示无数据或关节名无效
        """
        try:
            idx = self.hand_joint_index(side, joint_name)
        except ValueError:
            self.get_logger().error(f"Unknown hand joint: {joint_name} (side={side})")
            return None
        pos = self.get_hand_position(side)
        if pos is None:
            return None
        return float(pos[idx])

    # ---- 二指夹爪 API（/ecat/{left,right}_grip）----

    def wait_for_grip_state(self, side=None, timeout=5.0):
        """阻塞等待二指夹爪状态消息。"""
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            if s not in self._grip_state_received:
                raise ValueError(f"Invalid side '{s}', expected 'left' or 'right'")
            remaining = max(0.0, deadline - time.time())
            if not self._grip_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} grip state ({remaining:.1f}s)")
                return False
        return True

    def get_grip_state(self, side):
        """获取二指夹爪状态字典，None 表示无数据。"""
        with self._grip_state_lock:
            if side in self._grip_states:
                return dict(self._grip_states[side])
        return None

    def get_grip_position(self, side):
        """获取二指夹爪开口（m），None 表示无数据。"""
        state = self.get_grip_state(side)
        if state is None:
            return None
        return float(state["pos"])

    def send_grip_command(
        self,
        side,
        pos,
        vel=GRIP_DEFAULT_VEL,
        force=GRIP_DEFAULT_FORCE,
        cur=0.0,
        mode=0,
        stop=0,
        reset=0,
        homing=0,
    ):
        """单次发送二指夹爪 ECAT GripCmd。

        Args:
            side: "left" 或 "right"
            pos: 目标开口，单位 m；仿真夹爪范围 [0.0, 0.05]
            vel: 目标速度，单位 m/s
            force: 目标夹持力，单位 N
            cur: 目标电流/占位字段
            mode/stop/reset/homing: ECAT GripCmd 控制字段
        """
        if side not in self._grip_pubs:
            self.get_logger().error(f"Invalid grip side '{side}'")
            return False

        target_pos = float(pos)
        if self.enable_limit_check:
            clamped = max(GRIP_OPENING_MIN_M, min(GRIP_OPENING_MAX_M, target_pos))
            if clamped != target_pos:
                self.get_logger().warning(
                    f"CLAMPED {side} grip pos: {target_pos:.4f} → {clamped:.4f} m"
                )
            target_pos = clamped

        msg = GripCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.init = 1
        msg.mode = int(mode)
        msg.stop = int(stop)
        msg.reset = int(reset)
        msg.homing = int(homing)
        msg.pos = target_pos
        msg.vel = float(vel)
        msg.force = float(force)
        msg.cur = float(cur)
        self._grip_pubs[side].publish(msg)
        return True

    def open_grip(self, side, wait=False, timeout=2.0):
        """张开二指夹爪。"""
        ok = self.send_grip_command(side, GRIP_OPENING_MAX_M)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def close_grip(self, side, wait=False, timeout=2.0):
        """闭合二指夹爪。"""
        ok = self.send_grip_command(side, GRIP_OPENING_MIN_M)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def move_grip(self, side, pos, wait=False, timeout=2.0):
        """移动二指夹爪到指定开口（m）。"""
        ok = self.send_grip_command(side, pos)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def move_hand(self, side, pose_dict, duration_sec=2.0, wait=True):
        """按"关节名→角度"字典移动手指关节（线性插值 + 200Hz 发布）。

        未指定的关节保持当前位置。走 JointCommand 通路（mode=5），
        与身体 RobotCommand 通路完全独立。

        Args:
            side: "left" 或 "right"
            pose_dict: dict，键=关节名（全名或短名），值=目标弧度
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        if side not in V4_HAND_JOINT_MAP:
            self.get_logger().error(f"Invalid side '{side}'")
            return False

        joint_names = V4_HAND_JOINT_MAP[side]
        publisher = self._hand_pubs[side]

        # 获取当前位置
        current = self.get_hand_position(side)
        if current is None:
            self.get_logger().warning(
                f"No hand state for {side} hand, assuming zero position"
            )
            current = np.zeros(len(joint_names))

        # 构建目标位置
        target = current.copy()
        for name_or_short, angle in pose_dict.items():
            try:
                idx = self.hand_joint_index(side, name_or_short)
            except ValueError:
                self.get_logger().error(f"Unknown hand joint: {name_or_short}")
                return False
            target[idx] = float(angle)

        # 限位裁剪
        if self.enable_limit_check:
            target_list, violations = self._clamp_hand_position(joint_names, target.tolist())
            target = np.array(target_list)
            for name, val, lo, hi in violations:
                self.get_logger().warning(f"CLAMPED hand {name}: {val:.4f} → [{lo}, {hi}]")

        # 线性插值生成轨迹
        n_pts = max(2, int(duration_sec * V4_HAND_TEST_HZ))
        trajectory = np.column_stack([
            np.linspace(current[j], target[j], n_pts)
            for j in range(len(joint_names))
        ])

        if wait:
            self._execute_hand_trajectory(publisher, joint_names, trajectory)
        else:
            # 非阻塞：在后台线程执行
            t = threading.Thread(
                target=self._execute_hand_trajectory,
                args=(publisher, joint_names, trajectory),
                daemon=True,
            )
            t.start()

        return True

    def shift_hand(self, side, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """控制手指关节相对当前位置偏移。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
            delta_rad: 偏移量（rad）
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        current = self.get_hand_joint_position(side, joint_name)
        if current is None:
            self.get_logger().error(
                f"Cannot shift {joint_name}: no current position"
            )
            return False
        target = current + delta_rad

        # 限位提示
        short = joint_name.removeprefix("left_").removeprefix("right_")
        if short in V4_HAND_JOINT_LIMITS:
            lo, hi = V4_HAND_JOINT_LIMITS[short]
            if target < lo or target > hi:
                clamped = max(lo, min(hi, target))
                self.get_logger().warning(
                    f"{joint_name}: target {target:.4f} rad exceeds limit "
                    f"[{lo}, {hi}], will be clamped to {clamped:.4f}"
                )

        self.get_logger().info(
            f"Shift hand {joint_name}: {current:.4f} → {target:.4f} rad "
            f"(Δ={delta_rad:+.4f} rad, {np.degrees(delta_rad):+.2f}°)"
        )

        return self.move_hand(side, {joint_name: target}, duration_sec=duration_sec, wait=wait)

    def send_hand_position(self, side, positions):
        """单次发送手指关节目标位置。

        Args:
            side: "left" 或 "right"
            positions: 位置列表或数组，长度 = 7
        """
        if side not in V4_HAND_JOINT_MAP:
            self.get_logger().error(f"Invalid side '{side}'")
            return
        joint_names = V4_HAND_JOINT_MAP[side]
        publisher = self._hand_pubs[side]
        pos_list = [float(p) for p in positions]
        if self.enable_limit_check:
            pos_list, _ = self._clamp_hand_position(joint_names, pos_list)
        self._publish_hand_cmd(publisher, joint_names, pos_list)

    def _execute_hand_trajectory(self, publisher, joint_names, trajectory):
        """按轨迹逐点发布手部 JointCommand（阻塞执行）。

        Args:
            publisher: ROS2 publisher（JointCommand）
            joint_names: 关节名列表
            trajectory: numpy 数组 (N, n_hand_joints)
        """
        period = 1.0 / V4_HAND_TEST_HZ
        n_pts = trajectory.shape[0]
        start_time = time.time()

        for k in range(n_pts):
            t = time.time() - start_time
            if t >= n_pts * period:
                break

            positions = trajectory[k, :].tolist()
            if self.enable_limit_check:
                positions, _ = self._clamp_hand_position(joint_names, positions)
            self._publish_hand_cmd(publisher, joint_names, positions)

            # 频率控制
            elapsed = time.time() - start_time
            next_t = (k + 1) * period
            sleep_t = next_t - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        self.get_logger().info("Hand trajectory execution completed")

    def _hand_state_callback(self, side, msg: JointState):
        """手部关节状态回调，缓存最新位置。"""
        joint_names = V4_HAND_JOINT_MAP[side]
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        positions = np.zeros(len(joint_names), dtype=float)

        for i, name in enumerate(joint_names):
            if name in name_to_idx:
                positions[i] = msg.position[name_to_idx[name]]

        with self._hand_state_lock:
            self._hand_states[side] = positions
        self._hand_state_received[side].set()

    def _grip_state_callback(self, side, msg: GripStatus):
        """二指夹爪状态回调，缓存最新 GripStatus。"""
        state = {
            "init_state": int(msg.init_state),
            "grip_state": int(msg.grip_state),
            "error_code": int(msg.error_code),
            "homed": int(msg.homed),
            "pos": float(msg.pos),
            "vel": float(msg.vel),
            "cur": float(msg.cur),
        }
        with self._grip_state_lock:
            self._grip_states[side] = state
        self._grip_state_received[side].set()

    def _finger_link_state_callback(self, msg: String):
        """缓存仿真发布的 finger_link world pose JSON。"""
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid finger_link_states JSON: {exc}")
            return
        with self._finger_link_states_lock:
            self._finger_link_states = state
        self._finger_link_states_received.set()

    def wait_for_finger_link_states(self, timeout=5.0):
        """等待 /sim/finger_link_states；该状态为 world frame，仅用于验证/debug。"""
        ok = self._finger_link_states_received.wait(timeout=timeout)
        if not ok:
            self.get_logger().warning(f"Timeout waiting for finger_link_states ({timeout:.1f}s)")
        return ok

    def get_finger_link_states(self):
        """获取最新 finger_link world pose JSON，None 表示无数据。"""
        with self._finger_link_states_lock:
            if self._finger_link_states is not None:
                return json.loads(json.dumps(self._finger_link_states))
        return None

    def get_finger_link_pose(self, link_name: str):
        """获取指定 finger link 的 world pose 字典，None 表示无数据或 link 不存在。"""
        states = self.get_finger_link_states()
        if not states:
            return None
        return (states.get("links") or {}).get(link_name)

    def solve_arm_ik(self, side, target_xyzrpy, sync_state=True, **ik_kwargs):
        """只求解单臂 IK，不下发控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，返回的关节目标会包含 waist_yaw_joint。
        """
        if side not in ("left", "right"):
            raise ValueError(f"Invalid arm side: {side}")
        if not self._ensure_ik_initialized():
            return None, False, {"error": "ik_not_initialized"}
        with self._ik_lock:
            if sync_state:
                self._sync_ik_from_current_state_locked()
            if side == "left":
                result = self.ik_solver.solve_dual_arm(left_target_xyzrpy=target_xyzrpy, **ik_kwargs)
                joints = result.get("left_joint_positions")
                ok = bool(result.get("left_success", False))
                names = result.get("left_joint_names", LEFT_ARM_JOINTS)
            else:
                result = self.ik_solver.solve_dual_arm(right_target_xyzrpy=target_xyzrpy, **ik_kwargs)
                joints = result.get("right_joint_positions")
                ok = bool(result.get("right_success", False))
                names = result.get("right_joint_names", RIGHT_ARM_JOINTS)
        diagnostics = result.get("diagnostics", {})
        return dict(zip(names, [float(v) for v in joints])) if joints is not None else None, ok, diagnostics

    def solve_dual_arm_ik(self, left_target_xyzrpy=None, right_target_xyzrpy=None, sync_state=True, **ik_kwargs):
        """只求解双臂 IK，不下发控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，仅支持单臂目标，返回的关节列表会包含 waist_yaw_joint。
        """
        if left_target_xyzrpy is None and right_target_xyzrpy is None:
            return {}
        if not self._ensure_ik_initialized():
            return {"error": "ik_not_initialized"}
        with self._ik_lock:
            if sync_state:
                self._sync_ik_from_current_state_locked()
            result = self.ik_solver.solve_dual_arm(
                left_target_xyzrpy=left_target_xyzrpy,
                right_target_xyzrpy=right_target_xyzrpy,
                **ik_kwargs,
            )
        return result

    def move_arm_ik(self, side, target_xyzrpy, duration_sec=1.5, wait=True, require_success=True, **ik_kwargs):
        """单臂 Cartesian IK 控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，会在下发 waist_yaw_joint 目标时临时解锁腰部。
        """
        joint_targets, ok, diagnostics = self.solve_arm_ik(side, target_xyzrpy, **ik_kwargs)
        if joint_targets is None:
            return False
        if require_success and not ok:
            self.get_logger().warning(f"{side} arm IK did not converge: {diagnostics}")
            return False
        return self.move_to_pose(joint_targets, duration_sec=duration_sec, wait=wait, unlock_required_joints=True)

    def move_arm_ee_delta(
        self,
        side,
        delta_xyz=(0.0, 0.0, 0.0),
        delta_rpy=(0.0, 0.0, 0.0),
        duration_sec=1.5,
        wait=True,
        require_success=True,
        **ik_kwargs,
    ):
        """控制单侧末端相对当前位置位移。

        delta_xyz / delta_rpy 均定义在 Walker S2 URDF base frame 下，单位 m/rad。
        例如 delta_xyz=(0, 0, 0.02) 表示末端在 base frame 的 z 方向移动 2cm。
        """
        current = self.get_ee_pose(side)
        if current is None:
            return False
        delta = np.concatenate([
            np.asarray(delta_xyz, dtype=float),
            np.asarray(delta_rpy, dtype=float),
        ])
        if delta.shape != (6,):
            self.get_logger().error(f"EE delta must have 3 xyz + 3 rpy values, got shape {delta.shape}")
            return False
        target = np.asarray(current, dtype=float) + delta
        self.get_logger().info(
            f"Move {side} EE delta xyz={delta[:3].tolist()} rpy={delta[3:].tolist()} "
            f"target={target.tolist()}"
        )
        return self.move_arm_ik(
            side,
            target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            **ik_kwargs,
        )

    def move_left_ee_delta(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta("left", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_right_ee_delta(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta("right", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_dual_arm_ik(
        self,
        left_target_xyzrpy=None,
        right_target_xyzrpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        **ik_kwargs,
    ):
        """双臂 Cartesian IK 控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，仅支持单臂目标。
        """
        result = self.solve_dual_arm_ik(left_target_xyzrpy, right_target_xyzrpy, **ik_kwargs)
        if not result or "error" in result:
            return False

        pose_dict = {}
        waist_target = None
        if left_target_xyzrpy is not None:
            ok = bool(result.get("left_success", False))
            if require_success and not ok:
                self.get_logger().warning(f"left arm IK did not converge: {result.get('diagnostics', {})}")
                return False
            left_targets = dict(zip(result["left_joint_names"], [float(v) for v in result["left_joint_positions"]]))
            waist_target = left_targets.get("waist_yaw_joint")
            pose_dict.update(left_targets)
        if right_target_xyzrpy is not None:
            ok = bool(result.get("right_success", False))
            if require_success and not ok:
                self.get_logger().warning(f"right arm IK did not converge: {result.get('diagnostics', {})}")
                return False
            right_targets = dict(zip(result["right_joint_names"], [float(v) for v in result["right_joint_positions"]]))
            right_waist_target = right_targets.get("waist_yaw_joint")
            if waist_target is not None and right_waist_target is not None and abs(waist_target - right_waist_target) > 1e-6:
                self.get_logger().error("Conflicting waist_yaw_joint targets from left/right IK results")
                return False
            pose_dict.update(right_targets)

        return self.move_to_pose(pose_dict, duration_sec=duration_sec, wait=wait, unlock_required_joints=True)

    def move_dual_ee_delta(
        self,
        left_delta_xyz=None,
        right_delta_xyz=None,
        left_delta_rpy=None,
        right_delta_rpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        **ik_kwargs,
    ):
        """控制双臂末端相对当前位置位移。

        delta 均定义在 Walker S2 URDF base frame 下，单位 m/rad。传 None 表示该侧不动。
        """
        poses = self.get_ee_poses()
        if poses is None:
            return False

        left_target = None
        right_target = None
        if left_delta_xyz is not None or left_delta_rpy is not None:
            left_target = np.asarray(poses["left"], dtype=float).copy()
            left_target += np.concatenate([
                np.asarray(left_delta_xyz if left_delta_xyz is not None else (0.0, 0.0, 0.0), dtype=float),
                np.asarray(left_delta_rpy if left_delta_rpy is not None else (0.0, 0.0, 0.0), dtype=float),
            ])
        if right_delta_xyz is not None or right_delta_rpy is not None:
            right_target = np.asarray(poses["right"], dtype=float).copy()
            right_target += np.concatenate([
                np.asarray(right_delta_xyz if right_delta_xyz is not None else (0.0, 0.0, 0.0), dtype=float),
                np.asarray(right_delta_rpy if right_delta_rpy is not None else (0.0, 0.0, 0.0), dtype=float),
            ])
        if left_target is None and right_target is None:
            self.get_logger().warning("move_dual_ee_delta called with no left/right delta")
            return False

        return self.move_dual_arm_ik(
            left_target_xyzrpy=left_target,
            right_target_xyzrpy=right_target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            **ik_kwargs,
        )

    control_dual_arm_ik = move_dual_arm_ik

    def move_to_position(self, target_position, duration_sec=3.0, wait=True, publish_changed_only=False):
        """平滑移动到目标位置（从当前位置线性插值）。

        Args:
            target_position: 目标关节位置，长度 n_joints 的列表或 numpy 数组
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
            publish_changed_only: True 时仅发布本次轨迹中实际变化的关节
        Returns:
            bool: True=成功（已开始/完成），False=失败
        """
        target = np.array(target_position, dtype=float)
        if target.shape != (self.n_joints,):
            self.get_logger().error(
                f"Target shape {target.shape} != ({self.n_joints},)"
            )
            return False

        # 限位裁剪
        if self.enable_limit_check:
            target, violations = self._clamp_position(target)
            if violations:
                for name, val, lo, hi in violations:
                    self.get_logger().warning(
                        f"CLAMPED {name}: {val:.4f} → [{lo}, {hi}]"
                    )

        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False

        # 起点+终点 → 逐关节线性插值
        n_pts = max(2, int(duration_sec * self.control_hz))
        t_orig = np.linspace(0.0, 1.0, 2)
        t_new = np.linspace(0.0, 1.0, n_pts)
        trajectory = np.column_stack([
            np.interp(t_new, t_orig, [current[j], target[j]])
            for j in range(self.n_joints)
        ])

        return self.execute_trajectory(
            trajectory,
            wait=wait,
            publish_changed_only=publish_changed_only,
        )

    def execute_trajectory(self, trajectory, wait=True, publish_changed_only=False):
        """执行预定义轨迹。

        Args:
            trajectory: numpy 数组 (N, n_joints)，每行一个时间步的关节位置
                        点间距按 1/control_hz 秒（200Hz → 5ms/点）
            wait: 是否阻塞等待完成
            publish_changed_only: True 时仅发布轨迹中实际变化的关节
        Returns:
            bool: True=成功，False=失败（维度错误/安全违规）
        """
        trajectory = np.array(trajectory, dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != self.n_joints:
            self.get_logger().error(
                f"Trajectory shape {trajectory.shape} != (N, {self.n_joints})"
            )
            return False

        publish_joints = None
        if publish_changed_only:
            publish_joints = self._infer_changed_joints(trajectory)

        # 限位裁剪
        if self.enable_limit_check:
            all_violations = []
            for i in range(len(trajectory)):
                trajectory[i], viols = self._clamp_position(trajectory[i])
                all_violations.extend(viols)
            if all_violations:
                seen = set()
                for name, val, lo, hi in all_violations:
                    if name not in seen:
                        seen.add(name)
                        self.get_logger().warning(
                            f"CLAMPED {name} in trajectory to [{lo}, {hi}]"
                        )

        # 安全检查：最大关节速度
        if self.enable_safety_check and len(trajectory) >= 2:
            max_speeds = np.max(
                np.abs(np.diff(trajectory, axis=0)) / self.timer_period, axis=0
            )
            unsafe = []
            for i, name in enumerate(self.all_joints):
                if name in self.lock_joints:
                    continue
                if publish_joints is not None and name not in publish_joints:
                    continue
                if max_speeds[i] > self.max_joint_speed:
                    unsafe.append((name, max_speeds[i]))
            if unsafe:
                self.get_logger().error(
                    f"SAFETY VIOLATION: {len(unsafe)} joints exceed "
                    f"{self.max_joint_speed} rad/s"
                )
                for name, speed in unsafe:
                    self.get_logger().error(f"  {name}: {speed:.3f} rad/s")
                self.safety_violation = True
                return False

        # 写入轨迹
        with self.trajectory_lock:
            self.current_trajectory = trajectory.copy()
            self.current_publish_joints = publish_joints
            self.current_index = 0
            self.is_publishing = True
            self.safety_violation = False

        if publish_joints is None:
            publish_desc = "all unlocked joints"
        else:
            publish_desc = ", ".join(sorted(publish_joints)) or "none"
        self.get_logger().info(
            f"Executing trajectory: {len(trajectory)} points, "
            f"~{len(trajectory) / self.control_hz:.2f}s, publish={publish_desc}"
        )

        # 阻塞等待
        if wait:
            while True:
                with self.trajectory_lock:
                    if not self.is_publishing:
                        break
                time.sleep(0.01)

        return True

    def _infer_changed_joints(self, trajectory, epsilon=None):
        """根据轨迹列是否变化推断本次实际需要发布的关节。"""
        eps = self.publish_changed_epsilon if epsilon is None else float(epsilon)
        changed = set()
        for i, name in enumerate(self.all_joints):
            col = trajectory[:, i]
            if float(np.max(np.abs(col - col[0]))) > eps:
                changed.add(name)
        return changed

    def stop(self):
        """立即停止发布指令（机器人保持在最后一个发送的位置）"""
        with self.trajectory_lock:
            self.is_publishing = False
            self.current_index = self.current_trajectory.shape[0]
            self.current_publish_joints = None
        self.get_logger().info("Stop requested")

    def set_lock_joints(self, joint_names):
        """动态设置锁定关节列表"""
        self.lock_joints = set(joint_names or [])
        self.get_logger().info(f"Lock joints updated: {sorted(self.lock_joints)}")

    @property
    def is_busy(self):
        """是否正在执行轨迹"""
        with self.trajectory_lock:
            return self.is_publishing

    def _clamp_position(self, position):
        """裁剪关节位置到限位范围。

        Args:
            position: numpy 数组 (n_joints,)
        Returns:
            (clamped, violations) 元组：
                clamped: 裁剪后的数组
                violations: [(joint_name, requested, lower, upper), ...] 被裁剪的关节列表
        """
        clamped = position.copy()
        violations = []
        for i, name in enumerate(self.all_joints):
            if name not in BODY_JOINT_LIMITS:
                continue
            lo, hi = BODY_JOINT_LIMITS[name]
            val = clamped[i]
            if val < lo:
                clamped[i] = lo
                violations.append((name, val, lo, hi))
            elif val > hi:
                clamped[i] = hi
                violations.append((name, val, lo, hi))
        return clamped, violations

    def _clamp_hand_position(self, joint_names, positions):
        """裁剪手部关节位置到限位范围。

        Args:
            joint_names: 关节名列表（如 ["left_thumb_swing", ...]）
            positions: 对应位置列表
        Returns:
            (clamped_positions, violations) 元组
        """
        clamped = list(positions)
        violations = []
        for i, name in enumerate(joint_names):
            short = name.removeprefix("left_").removeprefix("right_")
            if short not in V4_HAND_JOINT_LIMITS:
                continue
            lo, hi = V4_HAND_JOINT_LIMITS[short]
            val = clamped[i]
            if val < lo:
                clamped[i] = lo
                violations.append((name, val, lo, hi))
            elif val > hi:
                clamped[i] = hi
                violations.append((name, val, lo, hi))
        return clamped, violations

    def move_to_pose(self, pose_dict, duration_sec=1.5, wait=True,
                     unlock_required_joints=True, publish_changed_only=True,
                     settle_check=True, settle_timeout=None,
                     settle_tolerance=0.03, max_settle_retries=2,
                     ignored_joints=None):
        """按"关节名→角度"字典移动机器人。未指定的关节保持当前位置。

        相比 move_to_position（传整个 17 维向量），这个 API 更方便：
        只关心你要改的几个关节，其余自动从当前位置读取。

        Args:
            pose_dict: dict，键=关节名，值=目标弧度
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
            unlock_required_joints: 若目标关节在 lock_joints 中，是否临时解锁
                                    wait=True 时执行完自动恢复锁定；
                                    wait=False 时不恢复（无法感知完成时刻）
        Returns:
            bool: True=成功，False=失败
        """
        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False

        # 校验关节名 + 检测需要解锁的关节
        target = current.copy()
        joints_needing_unlock = []
        for joint_name, angle in pose_dict.items():
            if joint_name not in self.all_joints:
                self.get_logger().error(f"Unknown joint: {joint_name}")
                return False
            idx = self.all_joints.index(joint_name)
            target[idx] = float(angle)
            if joint_name in self.lock_joints:
                joints_needing_unlock.append(joint_name)

        # 临时解锁（保存原锁定状态以便恢复）
        original_lock = None
        if joints_needing_unlock:
            if unlock_required_joints:
                original_lock = self.lock_joints.copy()
                self.get_logger().warning(
                    f"Temporarily unlocking joints: {joints_needing_unlock}"
                )
                self.set_lock_joints(list(self.lock_joints - set(joints_needing_unlock)))
            else:
                self.get_logger().warning(
                    f"Joints {joints_needing_unlock} are locked; their target "
                    f"values will be silently dropped"
                )

        target_joint_names = list(pose_dict.keys())

        def log_target_joint_errors(prefix):
            pos = self.get_current_position()
            if pos is None:
                return
            errors = []
            for name in target_joint_names:
                idx = self.all_joints.index(name)
                actual = float(pos[idx])
                desired = float(target[idx])
                errors.append((name, actual, desired, abs(actual - desired)))
            errors.sort(key=lambda item: item[3], reverse=True)
            error_text = ", ".join(
                f"{name}: actual={actual:+.4f}, target={desired:+.4f}, err={err:.4f}"
                for name, actual, desired, err in errors
            )
            self.get_logger().info(f"{prefix}: {error_text}")

        result = self.move_to_position(
            target,
            duration_sec=duration_sec,
            wait=wait,
            publish_changed_only=publish_changed_only,
        )

        settled = True
        # execute_trajectory(wait=True) 只表示轨迹点发布完毕；之后用闭环补偿重发目标，避免多关节动作停在中间状态。
        if result and wait and settle_check:
            check_timeout = settle_timeout
            if check_timeout is None:
                check_timeout = max(2.0, min(float(duration_sec), 3.0))
            ignored = set(ignored_joints or [])
            ignored.update(name for name in self.all_joints if name not in target_joint_names)
            if not unlock_required_joints:
                ignored.update(joints_needing_unlock)
            settled = False
            for attempt in range(max_settle_retries + 1):
                arrived, misses = self.wait_until_position(
                    target,
                    timeout=check_timeout,
                    tolerance=settle_tolerance,
                    ignored_joints=ignored,
                )
                log_target_joint_errors(f"Settle check {attempt + 1}/{max_settle_retries + 1}")
                if arrived:
                    settled = True
                    break
                if attempt >= max_settle_retries:
                    self.get_logger().warning(
                        f"Position did not settle before relock: {misses[:5]}"
                    )
                    break
                self.get_logger().warning(
                    f"Position did not settle, corrective retry {attempt + 1}/{max_settle_retries}: {misses[:5]}"
                )
                correction_duration = max(1.0, min(float(duration_sec) * 0.5, 2.0))
                result = self.move_to_position(
                    target,
                    duration_sec=correction_duration,
                    wait=True,
                    publish_changed_only=publish_changed_only,
                )
                if not result:
                    settled = False
                    break

        # 自动恢复锁定（仅 wait=True 时可安全恢复）
        if original_lock is not None and wait:
            self.set_lock_joints(list(original_lock))

        return bool(result and settled)

    def move_to_ready_pose(self, duration_sec=15.0, wait=True, staged=True):
        """分段移动到预备姿态（先 shoulder pitch / elbow roll，再 elbow yaw，最后 READY_POSE）。

        适用于实验开始前的初始化——将机器人从任意位置安全地移到统一的起始位姿。
        会临时解锁所有被锁定的关节（head/waist 等），以便完整执行预备姿态。

        Args:
            duration_sec: 分段运动总时长（秒），默认 10.0s
            wait: 保留 API 兼容；初始化分段为保证安全顺序始终阻塞执行
        Returns:
            bool: True=成功，False=失败
        """
        if not staged:
            return self.move_to_pose(
                READY_POSE,
                duration_sec=duration_sec,
                wait=wait,
                unlock_required_joints=True,
            )

        if not wait:
            self.get_logger().warning(
                "move_to_ready_pose(wait=False) requested, but staged init runs synchronously for safety"
            )

        duration_sec = float(duration_sec)
        if duration_sec < 1.5:
            self.get_logger().warning(
                f"Ready pose duration {duration_sec:.2f}s is too short for staged motion; using 1.50s"
            )
            duration_sec = 1.5

        pitch_roll_duration = duration_sec * 0.35
        elbow_yaw_duration = duration_sec * 0.35
        other_duration = duration_sec * 0.2
        reset_duration = duration_sec * 0.1

        stages = [
            ("1a/3 肩 pitch + elbow roll", READY_STAGE_1_PITCH_ROLL_POSE, pitch_roll_duration),
            ("1b/3 elbow yaw", READY_STAGE_1_ELBOW_YAW_POSE, elbow_yaw_duration),
            ("2/3 肩 pitch 回到预备姿态", READY_STAGE_2_POSE, other_duration),
            ("3/3 执行完整 READY_POSE", READY_POSE, reset_duration),
        ]

        for label, pose, stage_duration in stages:
            self.get_logger().info(
                f"Ready pose stage {label}: {stage_duration:.2f}s"
            )
            if not self.move_to_pose(
                pose,
                duration_sec=stage_duration,
                wait=True,
                unlock_required_joints=True,
            ):
                self.get_logger().error(f"Ready pose stage failed: {label}")
                return False

        return True

    def ready_position_vector(self):
        """返回 READY_POSE 对应的 17 维目标向量。"""
        return np.array([READY_POSE[name] for name in self.all_joints], dtype=float)

    def home(self, duration_sec=3.0, wait=True, unlock_required_joints=False):
        """移动到仿真 home 位姿（17 个 SDK body joints 全 0）。"""
        return self.move_to_pose(
            HOME_POSE,
            duration_sec=duration_sec,
            wait=wait,
            unlock_required_joints=unlock_required_joints,
        )

    def move_arm_joints(self, side, joints, duration_sec=1.5, wait=True):
        """按 7 维关节角移动单侧手臂；不做 Cartesian IK。"""
        if side not in ("left", "right"):
            self.get_logger().error(f"Invalid arm side: {side}")
            return False
        joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        if len(joints) != len(joint_names):
            self.get_logger().error(f"{side} arm expects {len(joint_names)} joints, got {len(joints)}")
            return False
        return self.move_to_pose(
            dict(zip(joint_names, [float(v) for v in joints])),
            duration_sec=duration_sec,
            wait=wait,
            unlock_required_joints=True,
        )

    def move_left_arm_joints(self, joints, duration_sec=1.5, wait=True):
        return self.move_arm_joints("left", joints, duration_sec=duration_sec, wait=wait)

    def move_right_arm_joints(self, joints, duration_sec=1.5, wait=True):
        return self.move_arm_joints("right", joints, duration_sec=duration_sec, wait=wait)

    move_left_arm = move_left_arm_joints
    move_right_arm = move_right_arm_joints

    def open_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_OPEN_POSE, duration_sec=duration_sec, wait=wait)

    def close_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_CLOSE_POSE, duration_sec=duration_sec, wait=wait)

    open_two_finger_grip = open_grip
    close_two_finger_grip = close_grip
    move_two_finger_grip = move_grip

    def reset_sim(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self.get_logger().info("Sent simulation reset command")

    def _image_callback(self, msg: Image):
        try:
            self.latest_img = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()
        except Exception as exc:
            self.get_logger().warning(f"Failed to cache RGB image: {exc}")

    def _depth_callback(self, msg: Image):
        try:
            self.latest_depth = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()
        except Exception as exc:
            self.get_logger().warning(f"Failed to cache depth image: {exc}")

    def head_periodic_motion(
        self,
        amplitude=HEAD_TEST_AMPLITUDE,
        period_sec=HEAD_TEST_PERIOD,
        cycles=HEAD_TEST_DEFAULT_CYCLES,
        move_yaw=True,
        move_pitch=True,
        return_to_zero=True,
        wait=True,
    ):
        """头部周期 sin 运动测试（参考 SDK demo pub_head_command.cpp）。

        生成轨迹：position = sin(2π * t / period) * amplitude，
        以本控制器频率（200Hz）采样并通过 execute_trajectory 发布。

        ⚠️ 副作用：
            - 自动解锁 head_pitch_joint / head_yaw_joint
            - 其他关节（含 waist）保持当前位置不变
            - 完成后 head 关节保持解锁状态，如需重新锁定调用 set_lock_joints()

        Args:
            amplitude: 振幅（弧度），默认 0.5（与 SDK demo 一致，约 28.6°）
            period_sec: 单个周期时长（秒），默认 2π≈6.28（与 SDK demo 一致）
            cycles: 运动周期数，默认 2 个完整周期
            move_yaw: 是否运动 head_yaw_joint
            move_pitch: 是否运动 head_pitch_joint
            return_to_zero: 完成最后一个周期后是否额外用 1/4 周期回到 0
                            （防止突然停在非零位置抖动）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功
        """
        if not (move_yaw or move_pitch):
            self.get_logger().error("Must move at least one of yaw/pitch")
            return False

        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False

        # 校验头部关节存在
        head_joints = []
        if move_pitch:
            if "head_pitch_joint" not in self.all_joints:
                self.get_logger().error("head_pitch_joint not in config")
                return False
            head_joints.append("head_pitch_joint")
        if move_yaw:
            if "head_yaw_joint" not in self.all_joints:
                self.get_logger().error("head_yaw_joint not in config")
                return False
            head_joints.append("head_yaw_joint")

        # 自动解锁头部关节
        locked_head = [j for j in head_joints if j in self.lock_joints]
        if locked_head:
            self.get_logger().warning(
                f"Unlocking head joints for periodic motion: {locked_head}"
            )
            self.set_lock_joints(list(self.lock_joints - set(locked_head)))

        # 生成 sin 轨迹
        total_duration = period_sec * cycles
        if return_to_zero:
            total_duration += period_sec / 4

        n_pts = max(2, int(total_duration * self.control_hz))
        t = np.linspace(0.0, total_duration, n_pts)

        # 初始化轨迹为"当前位置保持不变"，再覆盖头部维度
        trajectory = np.tile(current, (n_pts, 1))

        # 计算 sin 波形
        omega = 2 * np.pi / period_sec
        sin_wave = amplitude * np.sin(omega * t)

        if return_to_zero:
            # 超过 cycles 整周期后线性衰减到 0
            full_cycles_duration = period_sec * cycles
            ramp_mask = t > full_cycles_duration
            if ramp_mask.any():
                ramp_t = t[ramp_mask] - full_cycles_duration
                ramp_factor = 1.0 - ramp_t / (period_sec / 4)
                ramp_factor = np.clip(ramp_factor, 0.0, 1.0)
                sin_wave[ramp_mask] = sin_wave[ramp_mask] * ramp_factor

        for joint_name in head_joints:
            idx = self.all_joints.index(joint_name)
            trajectory[:, idx] = sin_wave

        self.get_logger().info(
            f"Head periodic motion: amplitude={amplitude:.3f} rad, "
            f"period={period_sec:.3f}s, cycles={cycles}, "
            f"total={total_duration:.2f}s, points={n_pts}, "
            f"joints={head_joints}"
        )

        return self.execute_trajectory(trajectory, wait=wait)

    def hand_periodic_motion(
        self,
        amplitude=V4_HAND_TEST_AMPLITUDE,
        period_sec=V4_HAND_TEST_PERIOD,
        cycles=V4_HAND_TEST_DEFAULT_CYCLES,
        phase_diff=V4_HAND_TEST_PHASE_DIFF,
        left_hand=True,
        right_hand=True,
        publish_hz=V4_HAND_TEST_HZ,
        return_to_zero=True,
    ):
        """V4 手部周期 sin 运动测试（参考 SDK demo pub_hand_v4_command.cpp）。

        与身体控制走独立通路：
            - 消息：JointCommand（不是 RobotCommand）
            - 话题：/mc/{left,right}_hand/command
            - 模式：mode=5（手部控制器自定义，非 JointCommand 标准枚举）
            - 控制器：手部控制器始终监听，**不需要 switch_controller**

        与 SDK demo 完全一致的运动模式：
            position[i] = sin(2π * t / period + i * phase_diff) * amplitude
            7 个手指关节按 phase_diff（默认 0.2 rad）依次错相，
            产生类似"波浪"的依次张合效果。

        V4 手 vs V3 手：V4 有 7 关节（含 thumb_pip），V3 只有 6 关节。
        本方法只适用于 V4 手。

        ⚠️ 与身体控制方法的差异：
            - 阻塞执行（无 wait 参数，本方法本身就是串行循环）
            - 直接以 publish_hz 循环发布，不进入轨迹队列
            - 完成后**不影响**身体控制（身体可继续 execute_trajectory）

        Args:
            amplitude: 振幅（rad），默认 0.6（与 SDK demo 一致）
            period_sec: 单周期时长（s），默认 2π
            cycles: 循环数，默认 2
            phase_diff: 相邻关节间相位差（rad），默认 0.2
            left_hand: 是否运动左手
            right_hand: 是否运动右手
            publish_hz: 发布频率（Hz），默认 200
            return_to_zero: 完成后是否平滑回到 0 位（避免手指停在张开状态）
        Returns:
            bool: True=完成
        """
        if not (left_hand or right_hand):
            self.get_logger().error("Must enable at least one of left/right hand")
            return False

        parts = []
        if left_hand:
            parts.append("left")
        if right_hand:
            parts.append("right")

        total_duration = period_sec * cycles
        if return_to_zero:
            total_duration += period_sec / 4

        n_pts = max(2, int(total_duration * publish_hz))
        full_cycles_duration = period_sec * cycles

        self.get_logger().info(
            f"V4 hand periodic motion: amplitude={amplitude:.3f} rad, "
            f"period={period_sec:.3f}s, cycles={cycles}, phase_diff={phase_diff:.3f}, "
            f"total={total_duration:.2f}s, hands={parts}, "
            f"publish_hz={publish_hz}, return_to_zero={return_to_zero}"
        )

        omega = 2 * np.pi / period_sec
        period_time = 1.0 / publish_hz
        start_time = time.time()
        hand_limit_warned = False

        try:
            for k in range(n_pts):
                # 实时计算时刻 t（与起始时间对齐，不依赖采样均匀性）
                t = time.time() - start_time
                if t >= total_duration:
                    break

                # 计算 ramp（最后 1/4 周期衰减到 0）
                if return_to_zero and t > full_cycles_duration:
                    ramp_t = t - full_cycles_duration
                    ramp_factor = max(0.0, 1.0 - ramp_t / (period_sec / 4))
                else:
                    ramp_factor = 1.0

                # 为每个手指计算 sin 值（每个关节相位差 i * phase_diff）
                base_phase = omega * t
                positions = [
                    amplitude * ramp_factor * np.sin(base_phase + i * phase_diff)
                    for i in range(len(V4_HAND_LEFT_JOINTS))   # 7 个关节
                ]

                # 限位裁剪
                if self.enable_limit_check:
                    positions, hand_violations = self._clamp_hand_position(
                        V4_HAND_LEFT_JOINTS, positions
                    )
                    # 只在首次超限时打 warning（避免 200Hz 洪水日志）
                    if hand_violations and not hand_limit_warned:
                        for name, val, lo, hi in hand_violations:
                            self.get_logger().warning(
                                f"CLAMPED hand {name}: {val:.4f} → [{lo}, {hi}]"
                            )
                        hand_limit_warned = True

                if left_hand:
                    self._publish_hand_cmd(
                        self.left_hand_pub, V4_HAND_LEFT_JOINTS, positions
                    )
                if right_hand:
                    self._publish_hand_cmd(
                        self.right_hand_pub, V4_HAND_RIGHT_JOINTS, positions
                    )

                # 频率控制：固定时间步 sleep（粗略）
                elapsed = time.time() - start_time
                next_t = (k + 1) * period_time
                sleep_t = next_t - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            self.get_logger().warning("Hand motion interrupted, sending zero command")
            self._send_hand_zero(left_hand, right_hand)
            return False

        # 兜底：完成后再发一次 0 位（保证手指完全放松）
        if return_to_zero:
            self._send_hand_zero(left_hand, right_hand)

        self.get_logger().info("V4 hand periodic motion completed")
        return True

    def _send_hand_zero(self, left_hand, right_hand):
        """向双手发送零位指令（确保手指完全放松）"""
        zeros = [0.0] * len(V4_HAND_LEFT_JOINTS)
        if left_hand:
            self._publish_hand_cmd(self.left_hand_pub, V4_HAND_LEFT_JOINTS, zeros)
        if right_hand:
            self._publish_hand_cmd(self.right_hand_pub, V4_HAND_RIGHT_JOINTS, zeros)

    def _publish_hand_cmd(self, publisher, joint_names, positions):
        """构造并发布手部 JointCommand（手部走独立通路，与身体不同）"""
        cmd = JointCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""
        cmd.names = list(joint_names)
        cmd.position = [float(p) for p in positions]
        # 注意：JointCommand 使用并行数组 + mode[]，mode=5 是手部控制器自定义值
        # 不是 JointCommand.POSITION_MODE=1，也不是 JointCmd.MODE_POSITION=2
        cmd.mode = [5] * len(joint_names)  # mode=5: 手部控制器自定义模式
        # 其他字段（velocity/torque/acceleration/kp/kd）使用空数组即可（demo 也未设置）
        publisher.publish(cmd)

    # ========================================================================
    # 内部回调
    # ========================================================================

    def _state_callback(self, msg: RobotState):
        """从 RobotState 提取 n_joints 维位置向量（按 config 中的关节顺序）"""
        joint_states = msg.joint_states
        name_to_idx = {name: idx for idx, name in enumerate(joint_states.name)}

        positions = np.zeros(self.n_joints, dtype=float)
        for i, joint_name in enumerate(self.all_joints):
            if joint_name not in name_to_idx:
                self.get_logger().error(
                    f"Joint '{joint_name}' not in RobotState"
                )
                return
            positions[i] = joint_states.position[name_to_idx[joint_name]]

        with self.robot_states_buffer_lock:
            self.robot_states_buffer.append(positions)

    def _control_callback(self):
        """200Hz 定时回调：取轨迹点 → 构造 RobotCommand → 发布"""
        if self.safety_violation:
            return

        with self.trajectory_lock:
            if not self.is_publishing:
                return
            if self.current_index >= self.current_trajectory.shape[0]:
                self.is_publishing = False
                self.current_publish_joints = None
                self.get_logger().info("Trajectory execution completed")
                return

            point = self.current_trajectory[self.current_index, :]
            publish_joints = self.current_publish_joints
            if publish_joints is not None:
                publish_joints = set(publish_joints)
            self.current_index += 1

        # 构造并发布 RobotCommand
        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""

        for idx, name in enumerate(self.all_joints):
            if name in self.lock_joints:
                continue
            if publish_joints is not None and name not in publish_joints:
                continue
            jc = JointCmd()
            jc.name = name
            jc.control_mode = JointCmd.MODE_POSITION
            jc.position = float(point[idx])
            cmd.joint_cmd.append(jc)

        if cmd.joint_cmd:
            self.command_pub.publish(cmd)


RobotController = WalkerS2Controller

# ============================================================================
# 命令行入口
# ============================================================================


def cmd_print_state(controller):
    """打印当前关节状态"""
    pos = controller.get_current_position()
    if pos is None:
        print("No current position available")
        return
    print(f"\n当前关节位置 ({controller.n_joints} 维):")
    for i, name in enumerate(controller.all_joints):
        locked = " [LOCKED]" if name in controller.lock_joints else ""
        limit_flag = ""
        if name in BODY_JOINT_LIMITS:
            lo, hi = BODY_JOINT_LIMITS[name]
            if pos[i] < lo:
                limit_flag = f"  ⚠️ BELOW LIMIT ({lo:.4f})"
            elif pos[i] > hi:
                limit_flag = f"  ⚠️ EXCEEDS LIMIT ({hi:.4f})"
            else:
                limit_flag = f"  [{lo:.2f}, {hi:.2f}]"
        print(f"  [{i:2d}] {name:30s} = {pos[i]:+.4f} rad{locked}{limit_flag}")


def cmd_print_grip_state(controller, sides=None):
    """打印二指夹爪状态。"""
    sides = sides or ["left", "right"]
    controller.wait_for_grip_state(timeout=1.0)
    print("\n二指夹爪状态:")
    print(f"{'side':<8s} {'pos(m)':>8s} {'vel':>8s} {'cur':>8s} {'init':>6s} {'state':>6s} {'homed':>6s} {'err':>6s}")
    print("-" * 68)
    for side in sides:
        state = controller.get_grip_state(side)
        if state is None:
            print(f"{side:<8s} {'N/A':>8s}")
            continue
        print(
            f"{side:<8s} {state['pos']:>8.4f} {state['vel']:>8.4f} {state['cur']:>8.4f} "
            f"{state['init_state']:>6d} {state['grip_state']:>6d} "
            f"{state['homed']:>6d} {state['error_code']:>6d}"
        )


def cmd_print_ee(controller, sides=None):
    """打印末端笛卡尔位姿(URDF base frame,xyz + rpy)。

    依赖 IK solver;构造控制器时需传 enable_ik=True(或运行时 initialize_ik())。
    """
    sides = sides or ["left", "right"]
    print("\n末端笛卡尔位姿 (URDF base frame):")
    print(
        f"{'side':<6s} {'x(m)':>10s} {'y(m)':>10s} {'z(m)':>10s} "
        f"{'roll(rad)':>11s} {'pitch(rad)':>11s} {'yaw(rad)':>11s} "
        f"{'roll(°)':>9s} {'pitch(°)':>9s} {'yaw(°)':>9s}"
    )
    print("-" * 106)
    any_ok = False
    for side in sides:
        pose = controller.get_ee_pose(side, as_dict=True)
        if pose is None:
            print(f"{side:<6s}  N/A (IK 未初始化或未收到机器人状态)")
            continue
        any_ok = True
        print(
            f"{side:<6s} "
            f"{pose['x']:>+10.4f} {pose['y']:>+10.4f} {pose['z']:>+10.4f} "
            f"{pose['roll']:>+11.4f} {pose['pitch']:>+11.4f} {pose['yaw']:>+11.4f} "
            f"{np.degrees(pose['roll']):>+9.1f} {np.degrees(pose['pitch']):>+9.1f} {np.degrees(pose['yaw']):>+9.1f}"
        )
    if not any_ok:
        print(
            "✗ 末端位姿获取失败。请确认:1) 已收到机器人状态  "
            "2) URDF 存在(默认 ubt_sim/assets/robots/walker_s2/s2.urdf,或设 WALKER_S2_IK_URDF)"
        )


def cmd_demo(controller):
    """安全演示：先移动到默认起始位姿，再在右臂 elbow_yaw 上做 ±0.05 rad 的小幅运动"""
    pos = controller.get_current_position()
    if pos is None:
        print("No current position available, abort demo")
        return

    print("\n=== 安全演示：右臂 elbow_yaw 小幅运动 ===")

    # 步骤 0：移动到预备姿态
    print(f"\n步骤 0: 移动到预备姿态")
    if not controller.move_to_ready_pose(duration_sec=3.0, wait=True):
        print("步骤 0 失败：无法到达起始位姿")
        return

    # 重新读取起始位姿（move_to_pose 完成后的实际位置）
    start_pos = controller.get_current_position()
    if start_pos is None:
        print("起始位姿读取失败，abort demo")
        return

    try:
        joint_name = "R_elbow_yaw_joint"
        joint_idx = controller.joint_index(joint_name)
    except ValueError:
        print(f"Joint not found, abort demo")
        return

    delta = 0.05  # 0.05 rad ≈ 3°，安全幅度
    duration = 2.0

    # 正向小幅运动
    target1 = start_pos.copy()
    target1[joint_idx] += delta
    print(f"\n步骤 1: {joint_name} += {delta} rad，{duration}s")
    if not controller.move_to_position(target1, duration_sec=duration):
        print("步骤 1 失败")
        return

    time.sleep(0.5)

    # 回到起始位姿
    print(f"\n步骤 2: 回到起始位姿，{duration}s")
    if not controller.move_to_position(start_pos, duration_sec=duration):
        print("步骤 2 失败")
        return

    print("\n演示完成")


def cmd_head_test(controller, amplitude, period_sec, cycles, yaw_only, pitch_only):
    """头部周期 sin 运动测试（参考 SDK demo pub_head_command.cpp）"""
    move_yaw = not pitch_only
    move_pitch = not yaw_only
    if not move_yaw and not move_pitch:
        print("✗ --yaw-only 和 --pitch-only 不能同时指定")
        return

    parts = []
    if move_pitch:
        parts.append("head_pitch")
    if move_yaw:
        parts.append("head_yaw")

    print("\n=== 头部周期 sin 运动测试 ===")
    print(f"参考脚本：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/.../pub_head_command.cpp")
    print(f"运动关节：{', '.join(parts)}")
    print(f"振幅：     ±{amplitude:.3f} rad (≈ ±{np.degrees(amplitude):.1f}°)")
    print(f"周期：     {period_sec:.3f} s")
    print(f"循环次数： {cycles}")
    print(f"总时长：   {period_sec * cycles + period_sec / 4:.2f} s (含 1/4 周期归零)")
    print()
    print("⚠️ 注意：会临时解锁 head 关节")
    print("⚠️ 注意：其他关节（含 waist、双臂）保持当前位置不变")

    input("\n按回车开始测试（Ctrl+C 取消）...")

    if not controller.head_periodic_motion(
        amplitude=amplitude,
        period_sec=period_sec,
        cycles=cycles,
        move_yaw=move_yaw,
        move_pitch=move_pitch,
        return_to_zero=True,
        wait=True,
    ):
        print("✗ 测试失败")
        return
    print("✓ 头部测试完成（已回到 0 位）")


def cmd_hand_test(controller, amplitude, period_sec, cycles, phase_diff,
                  left_only, right_only):
    """V4 手部周期 sin 运动测试（参考 SDK demo pub_hand_v4_command.cpp）"""
    left_hand = not right_only
    right_hand = not left_only
    if not (left_hand or right_hand):
        print("✗ --left-only 和 --right-only 不能同时指定")
        return

    parts = []
    if left_hand:
        parts.append("左手")
    if right_hand:
        parts.append("右手")

    print("\n=== V4 手部周期 sin 运动测试 ===")
    print(f"参考脚本：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/.../pub_hand_v4_command.cpp")
    print(f"运动手部：{', '.join(parts)}")
    print(f"关节数：   7 个/手（thumb_swing/mcp/pip + index/middle/ring/little_mcp）")
    print(f"振幅：     ±{amplitude:.3f} rad (≈ ±{np.degrees(amplitude):.1f}°)")
    print(f"周期：     {period_sec:.3f} s")
    print(f"相位差：   {phase_diff:.3f} rad（相邻手指相位差，产生波浪效果）")
    print(f"循环次数： {cycles}")
    print(f"总时长：   {period_sec * cycles + period_sec / 4:.2f} s (含归零段)")
    print()
    print("⚠️ 注意：手部走独立通路（JointCommand → /mc/{side}_hand/command）")
    print("⚠️ 注意：不需要 switch_controller（手部控制器始终监听）")
    print("⚠️ 注意：仅对 V4 手（7 关节，含 thumb_pip）有效，V3 手会因 thumb_pip 未知而报错")
    print("⚠️ 注意：身体（双臂/头/腰）保持当前位置不变")

    input("\n按回车开始测试（Ctrl+C 取消）...")

    if not controller.hand_periodic_motion(
        amplitude=amplitude,
        period_sec=period_sec,
        cycles=cycles,
        phase_diff=phase_diff,
        left_hand=left_hand,
        right_hand=right_hand,
        return_to_zero=True,
    ):
        print("✗ 测试失败或被中断")
        return
    print("✓ V4 手部测试完成（已回到 0 位）")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Walker S2 机器人直接控制脚本（SDK 控制器，模式B）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
注意事项：
  - 运行前必须先 switch_controller config_mc_walker_s2_v1_sps
  - 启动前用遥控器将机器人移到安全位置
  - 默认锁定 head_pitch/head_yaw/waist_yaw（不会发送这些关节的指令）
""",
    )
    parser.add_argument(
        "--no-lock", action="store_true",
        help="不锁定任何关节（默认锁定 head/waist）",
    )
    parser.add_argument(
        "--no-safety", action="store_true",
        help="禁用安全速度检查",
    )
    parser.add_argument(
        "--no-limits", action="store_true",
        help="禁用关节限位裁剪",
    )
    parser.add_argument(
        "--print-state", action="store_true",
        help="仅打印当前关节状态后退出",
    )
    parser.add_argument(
        "--print-ee", action="store_true",
        help="打印末端笛卡尔位姿(URDF base frame,xyz + rpy,需启用 IK)",
    )
    parser.add_argument(
        "--ee-side", choices=["left", "right", "both"], default="both",
        help="末端位姿打印侧,默认 both",
    )
    parser.add_argument(
        "--init", action="store_true",
        help="分段移动到预备姿态（先 shoulder pitch / elbow roll，再 elbow yaw，最后 READY_POSE）",
    )
    parser.add_argument(
        "--init-duration", type=float, default=10.0,
        help="预备姿态运动时长（秒），默认 10.0",
    )
    parser.add_argument(
        "--init-settle-timeout", type=float, default=10.0,
        help="轨迹发布完后等待实际关节收敛的超时时间（秒），默认 10.0",
    )
    parser.add_argument(
        "--init-tolerance", type=float, default=0.08,
        help="判定预备姿态到位的最大关节误差（rad），默认 0.08",
    )
    parser.add_argument(
        "--move-joint", action="store_true",
        help="移动单个身体关节到指定角度，用于诊断关节控制",
    )
    parser.add_argument(
        "--joint", choices=BODY_JOINT_NAMES, default=None,
        help="--move-joint 使用的关节名",
    )
    parser.add_argument(
        "--pos", type=float, default=None,
        help="--move-joint 使用的目标角度（rad）",
    )
    parser.add_argument(
        "--duration", type=float, default=3.0,
        help="--move-joint 运动时长（秒），默认 3.0",
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.03,
        help="--move-joint 到位判定阈值（rad），默认 0.03",
    )
    parser.add_argument(
        "--settle-timeout", type=float, default=3.0,
        help="--move-joint 额外等待实际关节收敛的超时时间（秒），默认 3.0",
    )
    parser.add_argument(
        "--demo", action="store_true",
        help="运行安全演示：右臂 elbow_yaw ±0.05 rad",
    )
    parser.add_argument(
        "--head-test", action="store_true",
        help="头部周期 sin 运动测试（参考 SDK pub_head_command.cpp）",
    )
    parser.add_argument(
        "--head-amplitude", type=float, default=HEAD_TEST_AMPLITUDE,
        help=f"头部测试振幅（rad），默认 {HEAD_TEST_AMPLITUDE} (与 SDK demo 一致)",
    )
    parser.add_argument(
        "--head-period", type=float, default=HEAD_TEST_PERIOD,
        help=f"头部测试周期（s），默认 {HEAD_TEST_PERIOD:.3f} (与 SDK demo 一致)",
    )
    parser.add_argument(
        "--head-cycles", type=int, default=HEAD_TEST_DEFAULT_CYCLES,
        help=f"头部测试循环次数，默认 {HEAD_TEST_DEFAULT_CYCLES}",
    )
    parser.add_argument(
        "--yaw-only", action="store_true",
        help="头部测试：仅运动 head_yaw_joint",
    )
    parser.add_argument(
        "--pitch-only", action="store_true",
        help="头部测试：仅运动 head_pitch_joint",
    )
    parser.add_argument(
        "--hand-test", action="store_true",
        help="V4 手部周期 sin 运动测试（参考 SDK pub_hand_v4_command.cpp）",
    )
    parser.add_argument(
        "--hand-amplitude", type=float, default=V4_HAND_TEST_AMPLITUDE,
        help=f"手部测试振幅（rad），默认 {V4_HAND_TEST_AMPLITUDE} (与 SDK demo 一致)",
    )
    parser.add_argument(
        "--hand-period", type=float, default=V4_HAND_TEST_PERIOD,
        help=f"手部测试周期（s），默认 {V4_HAND_TEST_PERIOD:.3f} (与 SDK demo 一致)",
    )
    parser.add_argument(
        "--hand-cycles", type=int, default=V4_HAND_TEST_DEFAULT_CYCLES,
        help=f"手部测试循环次数，默认 {V4_HAND_TEST_DEFAULT_CYCLES}",
    )
    parser.add_argument(
        "--hand-phase-diff", type=float, default=V4_HAND_TEST_PHASE_DIFF,
        help=f"手部相邻关节相位差（rad），默认 {V4_HAND_TEST_PHASE_DIFF}（产生波浪效果）",
    )
    parser.add_argument(
        "--left-only", action="store_true",
        help="手部测试：仅运动左手",
    )
    parser.add_argument(
        "--right-only", action="store_true",
        help="手部测试：仅运动右手",
    )
    parser.add_argument(
        "--grip-side", choices=["left", "right", "both"], default="both",
        help="二指夹爪操作侧，默认 both",
    )
    parser.add_argument(
        "--grip-state", action="store_true",
        help="打印二指夹爪状态（/ecat/{side}_grip/state）",
    )
    parser.add_argument(
        "--grip-open", action="store_true",
        help=f"张开二指夹爪到 {GRIP_OPENING_MAX_M:.3f} m",
    )
    parser.add_argument(
        "--grip-close", action="store_true",
        help="闭合二指夹爪到 0.000 m",
    )
    parser.add_argument(
        "--grip-pos", type=float, default=None,
        help=f"移动二指夹爪到指定开口（m，范围 {GRIP_OPENING_MIN_M:.3f}~{GRIP_OPENING_MAX_M:.3f}）",
    )
    parser.add_argument(
        "--grip-vel", type=float, default=GRIP_DEFAULT_VEL,
        help=f"二指夹爪命令速度（m/s），默认 {GRIP_DEFAULT_VEL}",
    )
    parser.add_argument(
        "--grip-force", type=float, default=GRIP_DEFAULT_FORCE,
        help=f"二指夹爪命令夹持力（N），默认 {GRIP_DEFAULT_FORCE}",
    )
    parser.add_argument(
        "--interactive", action="store_true",
        help="保持节点运行，等待外部 Python 调用（适合 IPython/REPL）",
    )
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    lock_joints = None if cli_args.no_lock else DEFAULT_LOCK_JOINTS
    controller = RobotController(
        lock_joints=lock_joints,
        enable_safety_check=not cli_args.no_safety,
        enable_limit_check=not cli_args.no_limits,
        enable_ik=cli_args.print_ee,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            print("[FATAL] 未收到机器人状态，请检查：")
            print("  1. 运控是否启动 (rosa run t800_mc_server start_mc_client)")
            print("  2. SDK 控制器是否切换 (switch_controller config_mc_walker_s2_v1_sps)")
            print("  3. DDS 中间件是否为 CycloneDDS")
            return

        if cli_args.print_state:
            cmd_print_state(controller)
        elif cli_args.print_ee:
            sides = ["left", "right"] if cli_args.ee_side == "both" else [cli_args.ee_side]
            cmd_print_ee(controller, sides)
        elif cli_args.move_joint:
            if cli_args.joint is None or cli_args.pos is None:
                print("[ERROR] --move-joint requires --joint and --pos")
                return
            cmd_print_state(controller)
            print(
                f"\n=== 单关节测试: {cli_args.joint} -> {cli_args.pos:+.4f} rad "
                f"({cli_args.duration:.1f}s) ==="
            )
            ok = controller.move_to_pose(
                {cli_args.joint: cli_args.pos},
                duration_sec=cli_args.duration,
                wait=True,
                unlock_required_joints=True,
            )
            current = controller.get_current_position()
            if current is not None:
                idx = controller.joint_index(cli_args.joint)
                actual = float(current[idx])
                err = abs(actual - float(cli_args.pos))
                print(f"{cli_args.joint}: actual={actual:+.4f}, target={cli_args.pos:+.4f}, err={err:.4f}")
            if ok:
                print("✓ 单关节命令完成并收敛")
            else:
                print("✗ 单关节命令未收敛")
            cmd_print_state(controller)
        elif cli_args.init:
            cmd_print_state(controller)
            print(f"\n=== 分段移动到预备姿态（{cli_args.init_duration:.1f}s）===")
            print("步骤：1a) shoulder pitch + elbow roll  1b) elbow yaw  2) shoulder pitch 回预备姿态  3) READY_POSE")
            input("按回车开始（Ctrl+C 取消）...")
            if controller.move_to_ready_pose(duration_sec=cli_args.init_duration):
                print("✓ 预备姿态轨迹发布完成，等待实际关节收敛...")
                reached, misses = controller.wait_until_position(
                    controller.ready_position_vector(),
                    timeout=cli_args.init_settle_timeout,
                    tolerance=cli_args.init_tolerance,
                )
                if reached:
                    print(f"✓ 预备姿态已到位（误差 ≤ {cli_args.init_tolerance:.3f} rad）")
                else:
                    print(
                        f"⚠️ 预备姿态未完全到位（超时 {cli_args.init_settle_timeout:.1f}s，"
                        f"阈值 {cli_args.init_tolerance:.3f} rad）"
                    )
                    for name, actual, target, err in misses[:8]:
                        if actual is None:
                            print(f"  {name}: no state, target={target:+.4f}")
                        else:
                            print(f"  {name}: actual={actual:+.4f}, target={target:+.4f}, err={err:.4f}")
                cmd_print_state(controller)
            else:
                print("✗ 预备姿态失败")
        elif cli_args.demo:
            cmd_print_state(controller)
            input("\n按回车开始演示（Ctrl+C 取消）...")
            cmd_demo(controller)
        elif cli_args.head_test:
            cmd_print_state(controller)
            cmd_head_test(
                controller,
                amplitude=cli_args.head_amplitude,
                period_sec=cli_args.head_period,
                cycles=cli_args.head_cycles,
                yaw_only=cli_args.yaw_only,
                pitch_only=cli_args.pitch_only,
            )
        elif cli_args.hand_test:
            # 手部测试不读取关节状态（手部状态不在 RobotState 中），跳过 print_state
            cmd_hand_test(
                controller,
                amplitude=cli_args.hand_amplitude,
                period_sec=cli_args.hand_period,
                cycles=cli_args.hand_cycles,
                phase_diff=cli_args.hand_phase_diff,
                left_only=cli_args.left_only,
                right_only=cli_args.right_only,
            )
        elif cli_args.grip_state or cli_args.grip_open or cli_args.grip_close or cli_args.grip_pos is not None:
            sides = ["left", "right"] if cli_args.grip_side == "both" else [cli_args.grip_side]
            if cli_args.grip_state and not (cli_args.grip_open or cli_args.grip_close or cli_args.grip_pos is not None):
                cmd_print_grip_state(controller, sides)
            else:
                if cli_args.grip_open:
                    target = GRIP_OPENING_MAX_M
                elif cli_args.grip_close:
                    target = GRIP_OPENING_MIN_M
                else:
                    target = cli_args.grip_pos
                print(f"\n=== 二指夹爪控制: sides={sides}, pos={target:.4f} m ===")
                for side in sides:
                    if controller.send_grip_command(
                        side,
                        target,
                        vel=cli_args.grip_vel,
                        force=cli_args.grip_force,
                    ):
                        print(f"✓ sent {side} grip command: pos={target:.4f} m")
                    else:
                        print(f"✗ failed to send {side} grip command")
                time.sleep(0.5)
                cmd_print_grip_state(controller, sides)
        elif cli_args.interactive:
            cmd_print_state(controller)
            print("\n节点运行中，按 Ctrl+C 退出。")
            spin_thread.join()
        else:
            cmd_print_state(controller)
            print("\n用法: --print-state | --print-ee | --init | --move-joint --joint JOINT --pos POS | --demo | --head-test | --hand-test | --grip-state | --grip-open | --grip-close | --grip-pos POS | --interactive")

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")

    finally:
        controller.stop()
        time.sleep(0.1)
        try:
            executor.shutdown(timeout_sec=1.0)
        except TypeError:
            executor.shutdown()
        spin_thread.join(timeout=2.0)
        if spin_thread.is_alive():
            controller.get_logger().warning("Executor spin thread did not exit before shutdown")
        try:
            executor.remove_node(controller)
        except Exception:
            pass
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
