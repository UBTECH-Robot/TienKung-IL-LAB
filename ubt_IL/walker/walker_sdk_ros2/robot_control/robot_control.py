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
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ecat_task_msgs.msg import GripCmd, GripStatus
from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
from sensor_msgs.msg import JointState

# ============================================================================
# 常量
# ============================================================================

DEFAULT_COMMAND_TOPIC = "/mc/sdk/robot_command"
DEFAULT_STATE_TOPIC = "/mc/sdk/robot_state"
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
V4_HAND_LEFT_TOPIC = "/mc/left_hand/command"
V4_HAND_RIGHT_TOPIC = "/mc/right_hand/command"
V4_HAND_LEFT_STATE_TOPIC = "/mc/left_hand/joint_states"
V4_HAND_RIGHT_STATE_TOPIC = "/mc/right_hand/joint_states"

GRIP_LEFT_CMD_TOPIC = "/ecat/left_grip/cmd"
GRIP_RIGHT_CMD_TOPIC = "/ecat/right_grip/cmd"
GRIP_LEFT_STATE_TOPIC = "/ecat/left_grip/state"
GRIP_RIGHT_STATE_TOPIC = "/ecat/right_grip/state"
GRIP_POSITION_LIMIT = (0.0, 0.05)     # m
GRIP_FORCE_LIMIT = (41.0, 100.0)      # N
GRIP_VELOCITY_LIMIT = (0.0, 0.01)     # m/s
GRIP_ACCELERATION_LIMIT = (0.0, 3.0)  # m/s^2，复用 GripCmd.cur 字段

LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
]

# 手部关节查找表：side → (joint_names_list, publisher_topic)
V4_HAND_JOINT_MAP = {
    "left": V4_HAND_LEFT_JOINTS,
    "right": V4_HAND_RIGHT_JOINTS,
}

# 手部预设姿态（用于 --hand-open / --hand-close）
V4_HAND_OPEN_POSE = {name: 0.0 for name in V4_HAND_JOINT_LIMITS}
V4_HAND_CLOSE_POSE = {name: hi for name, (_, hi) in V4_HAND_JOINT_LIMITS.items()}

# ============================================================================
# 预备姿态（双臂抬起预备抓取的站立位姿）
# ============================================================================

READY_POSE = {
    "L_elbow_roll_joint":       -1.700,
    # "L_elbow_yaw_joint":        2.8800,
    "L_elbow_yaw_joint":        1.5000,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.700,
    # "R_elbow_yaw_joint":        -2.8800,
    "R_elbow_yaw_joint":        -1.5000,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# 初始化分段 1a：直接复制仿真侧 walker_s2_controller.py 的 init 流程
READY_STAGE_1_PITCH_ROLL_POSE = {
    "L_shoulder_yaw_joint": -1.5600,
    "R_shoulder_yaw_joint": 1.5600,
    "L_elbow_yaw_joint": 1.5000,
    "R_elbow_yaw_joint": -1.5000,
}

# 初始化分段 1b：抬肩/收肘/调整腕 pitch
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


class RobotController(Node):
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
        node_name="robot_control_node",
        command_topic=None,
        state_topic=None,
        config_path=None,
        control_hz=DEFAULT_CONTROL_HZ,
        lock_joints=None,
        max_joint_speed=DEFAULT_MAX_JOINT_SPEED,
        enable_safety_check=True,
        enable_limit_check=True,
    ):
        super().__init__(node_name)

        self._config = self._load_config(config_path)
        command_topic = command_topic or self._get_topic("sub", "command", DEFAULT_COMMAND_TOPIC)
        state_topic = state_topic or self._get_topic("pub", "state", DEFAULT_STATE_TOPIC)
        left_hand_topic = self._get_topic("sub", "left_hand_command", V4_HAND_LEFT_TOPIC)
        right_hand_topic = self._get_topic("sub", "right_hand_command", V4_HAND_RIGHT_TOPIC)
        left_hand_state_topic = self._get_topic("pub", "left_hand_state", V4_HAND_LEFT_STATE_TOPIC)
        right_hand_state_topic = self._get_topic("pub", "right_hand_state", V4_HAND_RIGHT_STATE_TOPIC)
        left_grip_topic = self._get_topic("sub", "left_grip_command", GRIP_LEFT_CMD_TOPIC)
        right_grip_topic = self._get_topic("sub", "right_grip_command", GRIP_RIGHT_CMD_TOPIC)
        left_grip_state_topic = self._get_topic("pub", "left_grip_state", GRIP_LEFT_STATE_TOPIC)
        right_grip_state_topic = self._get_topic("pub", "right_grip_state", GRIP_RIGHT_STATE_TOPIC)

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
        self.lock_joints = set(lock_joints or [])

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

        # 手部状态订阅（/mc/{left,right}_hand/joint_states → sensor_msgs/JointState）
        self._hand_states = {}       # {"left": np.array(7), "right": np.array(7)}
        self._hand_state_lock = threading.Lock()
        self._hand_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
        self.left_hand_state_sub = self.create_subscription(
            JointState, left_hand_state_topic,
            lambda msg: self._hand_state_callback("left", msg),
            10, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.right_hand_state_sub = self.create_subscription(
            JointState, right_hand_state_topic,
            lambda msg: self._hand_state_callback("right", msg),
            10, callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # 夹爪发布者/状态订阅（大寰 PGC-140-50 / 电缸）
        self.left_grip_pub = self.create_publisher(
            GripCmd, left_grip_topic, qos_pub
        )
        self.right_grip_pub = self.create_publisher(
            GripCmd, right_grip_topic, qos_pub
        )
        self._grip_pubs = {"left": self.left_grip_pub, "right": self.right_grip_pub}
        self._grip_states = {}       # {"left": GripStatus, "right": GripStatus}
        self._grip_state_lock = threading.Lock()
        self._grip_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
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

        # 200Hz 控制定时器
        self.control_timer = self.create_timer(
            self.timer_period, self._control_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"RobotController initialized: {self.n_joints} joints, "
            f"{control_hz}Hz, locked={sorted(self.lock_joints)}, "
            f"limit_check={self.enable_limit_check}"
        )

    @staticmethod
    def _load_config(config_path=None):
        """读取本容器内的可选 topic 配置；无配置时保持真机默认 topic。"""
        if not config_path:
            return {}
        try:
            import yaml
        except ImportError:
            # PyYAML 只在显式使用 config_path 时需要，避免给真机默认运行增加依赖。
            return {}
        try:
            with open(config_path) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _get_topic(self, section, key, default):
        """按 bridge 配置格式取 topic；section/key 是从 bridge 视角命名。"""
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

    # ---- 夹爪 API ----

    def wait_for_grip_state(self, side=None, timeout=5.0):
        """阻塞等待夹爪状态消息。

        Args:
            side: "left"、"right" 或 None（等待双侧）
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            if s not in self._grip_state_received:
                self.get_logger().error(f"Invalid grip side '{s}'")
                return False
            remaining = max(0.0, deadline - time.time())
            if not self._grip_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} grip state ({remaining:.1f}s)")
                return False
        return True

    def get_grip_state(self, side):
        """获取指定侧夹爪最新状态，None 表示无数据。"""
        if side not in ("left", "right"):
            self.get_logger().error(f"Invalid grip side '{side}'")
            return None
        with self._grip_state_lock:
            return self._grip_states.get(side)

    def send_grip_command(
        self,
        side,
        pos,
        force=41.0,
        vel=0.005,
        acc=0.0,
        mode=0,
        init=1,
        stop=0,
        reset=0,
        homing=0,
        repeat_sec=0.5,
        repeat_hz=20.0,
    ):
        """发送夹爪控制命令。

        Args:
            side: "left" 或 "right"
            pos: 目标位置，范围 [0, 0.05] m
            force: 目标力，范围 [41, 100] N
            vel: 目标速度，范围 [0, 0.01] m/s
            acc: 目标加速度，范围 [0, 3] m/s^2（写入 GripCmd.cur 字段）
            mode: 0=位置/速度/力控制，10=推压模式
            init/stop/reset/homing: GripCmd 控制标志
            repeat_sec: 连续发布时长（秒），默认 0.5；0 表示只发布一次
            repeat_hz: 连续发布频率（Hz）
        Returns:
            bool: True=已发布，False=参数错误
        """
        if side not in self._grip_pubs:
            self.get_logger().error(f"Invalid grip side '{side}', expected 'left' or 'right'")
            return False

        pos = self._clamp_scalar("grip pos", float(pos), GRIP_POSITION_LIMIT)
        force = self._clamp_scalar("grip force", float(force), GRIP_FORCE_LIMIT)
        vel = self._clamp_scalar("grip vel", float(vel), GRIP_VELOCITY_LIMIT)
        acc = self._clamp_scalar("grip acc", float(acc), GRIP_ACCELERATION_LIMIT)

        msg = GripCmd()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.init = int(init)
        msg.mode = int(mode)
        msg.stop = int(stop)
        msg.reset = int(reset)
        msg.homing = int(homing)
        msg.pos = pos
        msg.vel = vel
        msg.force = force
        msg.cur = acc

        if repeat_sec and repeat_sec > 0:
            interval = 1.0 / repeat_hz
            n_pub = max(1, int(repeat_sec * repeat_hz))
            for _ in range(n_pub):
                self._grip_pubs[side].publish(msg)
                time.sleep(interval)
        else:
            self._grip_pubs[side].publish(msg)

        self.get_logger().info(
            f"Grip command {side}: pos={pos:.4f}m force={force:.1f}N "
            f"vel={vel:.4f}m/s acc={acc:.2f}m/s^2 mode={mode} "
            f"init={init} stop={stop} reset={reset} homing={homing}"
        )
        return True

    def home_grip(self, side):
        """发送夹爪回零命令。"""
        return self.send_grip_command(
            side, pos=0.0, force=41.0, vel=0.005, acc=0.0, homing=1, repeat_sec=1.0
        )

    def stop_grip(self, side):
        """发送夹爪停止命令。"""
        state = self.get_grip_state(side)
        pos = state.pos if state is not None else 0.0
        return self.send_grip_command(
            side, pos=pos, force=41.0, vel=0.0, acc=0.0, stop=1, repeat_sec=0.2
        )

    def open_grip(self, side, wait=False, timeout=2.0):
        """张开二指夹爪（真机安全参数）。"""
        ok = self.send_grip_command(
            side,
            pos=GRIP_POSITION_LIMIT[1],
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def close_grip(self, side, wait=False, timeout=2.0):
        """闭合二指夹爪（真机安全参数）。"""
        ok = self.send_grip_command(
            side,
            pos=GRIP_POSITION_LIMIT[0],
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def move_grip(self, side, pos, wait=False, timeout=2.0):
        """移动二指夹爪到指定开口（m）。"""
        ok = self.send_grip_command(
            side,
            pos=pos,
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    open_two_finger_grip = open_grip
    close_two_finger_grip = close_grip
    move_two_finger_grip = move_grip

    def _grip_state_callback(self, side, msg: GripStatus):
        """夹爪状态回调，缓存最新状态。"""
        with self._grip_state_lock:
            self._grip_states[side] = msg
        self._grip_state_received[side].set()

    def _clamp_scalar(self, name, value, limits):
        """裁剪标量到限位范围。"""
        if not self.enable_limit_check:
            return value
        lo, hi = limits
        if value < lo:
            self.get_logger().warning(f"CLAMPED {name}: {value:.4f} → {lo:.4f}")
            return lo
        if value > hi:
            self.get_logger().warning(f"CLAMPED {name}: {value:.4f} → {hi:.4f}")
            return hi
        return value

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

    def wait_until_position(self, target_position, timeout=5.0, tolerance=0.05, ignored_joints=None):
        """等待实际关节位置收敛到目标附近。

        execute_trajectory(wait=True) 只表示轨迹点发布完毕；真机实际关节
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
            publish_changed_only: True 时仅发布 pose_dict 涉及变化的关节
            settle_check: wait=True 后是否检查实际关节到位
            settle_timeout: 单次到位检查超时；None 时按 duration_sec 自动推导
            settle_tolerance: 到位误差阈值（rad）
            max_settle_retries: 未到位时补偿重发次数
            ignored_joints: 到位检查时忽略的关节名列表
        Returns:
            bool: True=成功且到位（若启用检查），False=失败
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

    def move_to_ready_pose(self, duration_sec=None, wait=True, staged=False):
        """移动到预备姿态（双臂自然下垂的站立位姿）。

        staged=True 时按真机侧安全阶段依次执行，默认 10s；staged=False 保留旧版直达行为，默认 3s。
        """
        if duration_sec is None:
            duration_sec = 10.0 if staged else 3.0

        if not staged:
            return self.move_to_pose(
                READY_POSE,
                duration_sec=duration_sec,
                wait=wait,
                unlock_required_joints=True,
            )

        if not wait:
            self.get_logger().warning(
                "move_to_ready_pose(wait=False, staged=True) requested, but staged init runs synchronously for safety"
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


WalkerS2Controller = RobotController

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
        "--init", action="store_true",
        help="移动到预备姿态（双臂自然下垂站立）",
    )
    parser.add_argument(
        "--init-duration", type=float, default=None,
        help="预备姿态运动时长（秒）；直达默认 3.0，分段默认 10.0",
    )
    parser.add_argument(
        "--staged-init", action="store_true",
        help="分段移动到预备姿态，适合双臂抬起/大幅 ready pose（建议 --init-duration 10~15）",
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
        elif cli_args.init:
            cmd_print_state(controller)
            init_mode = "分段" if cli_args.staged_init else "直达"
            init_duration = cli_args.init_duration
            if init_duration is None:
                init_duration = 10.0 if cli_args.staged_init else 3.0
            print(f"\n=== 移动到预备姿态（{init_mode}，{init_duration:.1f}s）===")
            input("按回车开始（Ctrl+C 取消）...")
            if controller.move_to_ready_pose(
                duration_sec=init_duration,
                staged=cli_args.staged_init,
            ):
                print("✓ 预备姿态完成")
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
        elif cli_args.interactive:
            cmd_print_state(controller)
            print("\n节点运行中，按 Ctrl+C 退出。")
            spin_thread.join()
        else:
            cmd_print_state(controller)
            print("\n用法: --print-state | --init | --demo | --head-test | --hand-test | --interactive")

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")

    finally:
        controller.stop()
        time.sleep(0.1)
        executor.remove_node(controller)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
