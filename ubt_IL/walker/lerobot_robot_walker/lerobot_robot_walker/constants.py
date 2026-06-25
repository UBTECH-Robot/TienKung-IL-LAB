"""Walker S2 机器人硬件常量（walker 插件侧）。

数据来源：walker/walker_sdk_ros2/robot_control/robot_control.py
此模块与 ros2_walker_bridge.py 独立维护——两边运行在不同 Python 环境
（3.12 vs 3.10），不交叉导入，仅通过 ZMQ + JSON 配置通信。
"""

# ── 身体关节名（17: 7左臂 + 7右臂 + head_pitch + head_yaw + waist_yaw）──
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

# ── 身体关节限位（rad），来源：Walker S2 硬件规格书 ──
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
    "R_shoulder_yaw_joint":     (-2.8972, 2.9147),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}

# ── V4 手部关节名（每手 7 关节，含 thumb_pip）──
V4_HAND_LEFT_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_thumb_pip",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

V4_HAND_RIGHT_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_thumb_pip",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

# ── V4 手部关节限位（rad），短名（去掉 left_/right_ 前缀）──
V4_HAND_JOINT_LIMITS = {
    "thumb_swing":  (0.0, 2.11),
    "thumb_mcp":    (0.0, 1.85),
    "thumb_pip":    (0.0, 1.09),
    "index_mcp":    (0.0, 1.71),
    "middle_mcp":   (0.0, 1.71),
    "ring_mcp":     (0.0, 1.71),
    "little_mcp":   (0.0, 1.71),
}

# ── 归位位姿（17-dim body，按 BODY_JOINT_NAMES 顺序）──
READY_POSE = {
    "L_elbow_roll_joint":       -1.5600,
    "L_elbow_yaw_joint":        2.8800,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.5600,
    "R_elbow_yaw_joint":        -2.8800,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# 扁平化 home_position，按 BODY_JOINT_NAMES 顺序
HOME_POSITION = [READY_POSE[name] for name in BODY_JOINT_NAMES]

# 手部张开位置（V4: 全 0 = 手指自然伸直）
HAND_OPEN_POSITION = [0.0] * 7

# ── 默认锁定关节（不发送控制指令，保持当前位置）──
DEFAULT_LOCK_JOINTS = ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"]

# ── ROS2 话题名 ──
TOPIC_BODY_CMD = "/mc/sdk/robot_command"
TOPIC_BODY_STATE = "/mc/sdk/robot_state"
TOPIC_LEFT_HAND_CMD = "/mc/left_hand/command"
TOPIC_RIGHT_HAND_CMD = "/mc/right_hand/command"
TOPIC_LEFT_HAND_STATE = "/mc/left_hand/joint_states"
TOPIC_RIGHT_HAND_STATE = "/mc/right_hand/joint_states"

# ── 大寰 PGC-140-50 夹爪话题（ecat_task_msgs/{GripCmd,GripStatus}）──
TOPIC_LEFT_GRIP_CMD = "/ecat/left_grip/cmd"
TOPIC_RIGHT_GRIP_CMD = "/ecat/right_grip/cmd"
TOPIC_LEFT_GRIP_STATE = "/ecat/left_grip/state"
TOPIC_RIGHT_GRIP_STATE = "/ecat/right_grip/state"

# ── 相机话题（默认）──
TOPIC_CAMERA_STEREO = "/sensor/camera/stereo/color/raw"
TOPIC_CAMERA_WRIST_LEFT = "/sensor/camera/wrist_left/color/raw"
TOPIC_CAMERA_WRIST_RIGHT = "/sensor/camera/wrist_right/color/raw"
