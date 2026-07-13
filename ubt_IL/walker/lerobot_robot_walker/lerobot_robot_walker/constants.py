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


# =====================================================================
# 关节 DOF 映射配置（支持任意自由度 / 任意关节顺序的模型训练与部署）
# =====================================================================
#
# 设计要点（两套映射机制解耦，与天工 DOF 架构一致）：
#   - policy ↔ 数据集：按位映射。枚举成员顺序 == 数据集 action/state 顺序。
#     故枚举成员可任意重排以匹配数据集顺序。
#   - policy ↔ 硬件：按名散射。6 个硬件分组是【固定物理 motor/手指/夹爪顺序】
#     （bridge 按位寻址，严禁重排），与枚举顺序无关。
#     send_action 按名从 action 字典取值、按物理序拼 bridge list。
# 二者解耦后，枚举可随意重排、可取任意子集，均能正确部署。

from enum import IntEnum

# ── 固定硬件分组（物理顺序，bridge 按位寻址，严禁重排）──
LEFT_ARM_JOINTS = [
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint", "L_shoulder_yaw_joint", "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint", "R_shoulder_yaw_joint", "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]
HEAD_JOINTS = ["head_pitch_joint", "head_yaw_joint"]
WAIST_JOINTS = ["waist_yaw_joint"]

# canonical 全集（物理序），供派生填充迭代用
CANONICAL_JOINT_NAMES = (
    LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + HEAD_JOINTS + WAIST_JOINTS
    + V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS
    + ["left_grip", "right_grip"]
)  # 33: 17 body + 14 V4 hand + 2 PGC gripper


# ── DOF 枚举：定义 all_joints（policy 维度与顺序 = 数据集顺序）──
# 成员名须取自 CANONICAL_JOINT_NAMES；成员顺序须与数据集 action/state 顺序一致（可重排）。
# 不含 .pos 后缀，由 joint_names_with_pos() 统一追加。

class WalkerS219DofJointIndex(IntEnum):
    """19-DOF：全 17 body + 左右 PGC 1-DOF 夹爪（顺序与 walker_s2_gripper_19d.json 一致）"""

    # Left arm (0-6)
    L_elbow_roll_joint = 0
    L_elbow_yaw_joint = 1
    L_shoulder_pitch_joint = 2
    L_shoulder_roll_joint = 3
    L_shoulder_yaw_joint = 4
    L_wrist_pitch_joint = 5
    L_wrist_roll_joint = 6
    # Right arm (7-13)
    R_elbow_roll_joint = 7
    R_elbow_yaw_joint = 8
    R_shoulder_pitch_joint = 9
    R_shoulder_roll_joint = 10
    R_shoulder_yaw_joint = 11
    R_wrist_pitch_joint = 12
    R_wrist_roll_joint = 13
    # Head (14-15)
    head_pitch_joint = 14
    head_yaw_joint = 15
    # Waist (16)
    waist_yaw_joint = 16
    # PGC grippers (17-18)
    left_grip = 17
    right_grip = 18


class WalkerS210DofJointIndex(IntEnum):
    """10-DOF：仅右臂 7 + 头 2 + 右 PGC 夹爪 1（顺序与 walker_s2_real_10d_1RGBD.json 一致）"""

    # Right arm (0-6)
    R_elbow_roll_joint = 0
    R_elbow_yaw_joint = 1
    R_shoulder_pitch_joint = 2
    R_shoulder_roll_joint = 3
    R_shoulder_yaw_joint = 4
    R_wrist_pitch_joint = 5
    R_wrist_roll_joint = 6
    # Head (7-8)
    head_pitch_joint = 7
    head_yaw_joint = 8
    # Right PGC gripper (9)
    right_grip = 9


class WalkerS231DofJointIndex(IntEnum):
    """31-DOF：全 17 body + 左右 V4 7-DOF 手（顺序与 walker_s2_v4_hand_31d.json 一致）"""

    # Left arm (0-6)
    L_elbow_roll_joint = 0
    L_elbow_yaw_joint = 1
    L_shoulder_pitch_joint = 2
    L_shoulder_roll_joint = 3
    L_shoulder_yaw_joint = 4
    L_wrist_pitch_joint = 5
    L_wrist_roll_joint = 6
    # Right arm (7-13)
    R_elbow_roll_joint = 7
    R_elbow_yaw_joint = 8
    R_shoulder_pitch_joint = 9
    R_shoulder_roll_joint = 10
    R_shoulder_yaw_joint = 11
    R_wrist_pitch_joint = 12
    R_wrist_roll_joint = 13
    # Head (14-15)
    head_pitch_joint = 14
    head_yaw_joint = 15
    # Waist (16)
    waist_yaw_joint = 16
    # Left V4 hand (17-23)
    left_thumb_swing = 17
    left_thumb_mcp = 18
    left_thumb_pip = 19
    left_index_mcp = 20
    left_middle_mcp = 21
    left_ring_mcp = 22
    left_little_mcp = 23
    # Right V4 hand (24-30)
    right_thumb_swing = 24
    right_thumb_mcp = 25
    right_thumb_pip = 26
    right_index_mcp = 27
    right_middle_mcp = 28
    right_ring_mcp = 29
    right_little_mcp = 30


# DOF 名 -> 枚举类 注册表（新增 DOF：定义 IntEnum + 在此注册即可）
JOINT_INDEX_ENUMS: dict[str, type] = {
    "walker_s2_19d": WalkerS219DofJointIndex,
    "walker_s2_10d": WalkerS210DofJointIndex,
    "walker_s2_31d": WalkerS231DofJointIndex,
}


# ── 非激活关节的静态填充 ──
# 任何非激活关节（不在所选 DOF 枚举中）的默认静态值：
#   车身关节取 READY_POSE 对应位，V4 手取 0.0（伸直），夹爪取 0.0（闭合安全位）。
DEFAULT_INACTIVE_FILL: dict[str, float] = {
    **READY_POSE,
    **{j: 0.0 for j in V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS},
    "left_grip": 0.0,
    "right_grip": 0.0,
}

# 可选：per-DOF 非激活关节填充覆盖（裸关节名 -> 值）。空时全用默认。
# 例：让 10-DOF 的非激活左手张到全开而非默认 0.0
#   {"walker_s2_10d": {"left_grip": 0.03, "left_thumb_swing": 0.5, ...}}
INACTIVE_FILL_OVERRIDES: dict[str, dict[str, float]] = {}


def inactive_fill_for(dof_name: str, enum_cls: type) -> dict[str, float]:
    """选中 DOF 枚举未包含的关节 -> .pos 键的静态填充值（按名，与枚举顺序无关）。

    先取 DEFAULT_INACTIVE_FILL，再用 INACTIVE_FILL_OVERRIDES[dof_name] 覆盖。
    返回的 dict 键已补 ``.pos`` 后缀，可直接用于 robot 的 action 字典。
    """
    active = {m.name for m in enum_cls}
    fill = {
        j: DEFAULT_INACTIVE_FILL.get(j, 0.0)
        for j in CANONICAL_JOINT_NAMES
        if j not in active
    }
    fill.update(INACTIVE_FILL_OVERRIDES.get(dof_name, {}))
    return {f"{j}.pos": v for j, v in fill.items()}


def joint_names_with_pos(enum_cls: type) -> list[str]:
    """从 DOF 枚举派生 all_joints（成员名 + .pos 后缀，顺序 = 枚举顺序 = 数据集顺序）。"""
    return [f"{m.name}.pos" for m in enum_cls]
