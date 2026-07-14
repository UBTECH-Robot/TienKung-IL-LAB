"""Walker C1 / Astron teleoperation constants.

These constants intentionally live on the ROS/teleoperation side and do not
import Isaac-side modules.
"""

BODY_JOINT_NAMES = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

LEFT_HAND_JOINT_NAMES = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

RIGHT_HAND_JOINT_NAMES = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

# Task reset pose for tabletop manipulation. The arm shape is adapted from the
# Tiankung pick-place reset pose by matching joint semantics to C1 joint names.
TASK_RESET_BODY_POSE = {
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.35,
    "L_shoulder_pitch_joint": -0.152,
    "L_shoulder_roll_joint": 0.30,
    "L_shoulder_yaw_joint": 0.135,
    "L_elbow_pitch_joint": -1.155,
    "L_elbow_yaw_joint": 0.124,
    "L_wrist_pitch_joint": -0.361,
    "L_wrist_roll_joint": -0.006,
    "R_shoulder_pitch_joint": -0.291,
    "R_shoulder_roll_joint": -0.30,
    "R_shoulder_yaw_joint": -0.136,
    "R_elbow_pitch_joint": -1.155,
    "R_elbow_yaw_joint": -0.124,
    "R_wrist_pitch_joint": -0.361,
    "R_wrist_roll_joint": 0.194,
}

# C1/Astron SDK hand commands use 0 rad as the open neutral pose.
TASK_RESET_LEFT_HAND_POSE = [0.0] * len(LEFT_HAND_JOINT_NAMES)
TASK_RESET_RIGHT_HAND_POSE = [0.0] * len(RIGHT_HAND_JOINT_NAMES)

TASK_RESET_ARM_CLEAR_POSE = {
    "L_shoulder_roll_joint": 1.5,
    "R_shoulder_roll_joint": -1.5,
}

TASK_RESET_ELBOW_CLEAR_POSE = {
    "L_elbow_pitch_joint": -1.5,
    "R_elbow_pitch_joint": -1.5,
}

DEFAULT_TASK_RESET_HZ = 100.0
DEFAULT_TASK_RESET_DURATION = 3.0
DEFAULT_TASK_RESET_CLEAR_DURATION = 1.0
