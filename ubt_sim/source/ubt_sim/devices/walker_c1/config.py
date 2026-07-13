from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from ubt_sim.utils.constant import ASSETS_ROOT

"""Configuration for the Walker C1 / Astron robot asset."""
ROBOTS_ROOT = Path(ASSETS_ROOT) / "robots"
WALKER_C1_USD_PATH = ROBOTS_ROOT / "walker_c1" / "walker_c1.usd"
WALKER_C1_URDF_PATH = ROBOTS_ROOT / "walker_c1" / "walker_astron_v2_hand_v3_no_sixforce_mesh.urdf"

WALKER_C1_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]

WALKER_C1_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

WALKER_C1_LEFT_HAND_JOINTS = [
    "L_thumb_cmp_joint",
    "L_thumb_mpp_joint",
    "L_thumb_ip_joint",
    "L_index_mpp_joint",
    "L_index_ip_joint",
    "L_middle_mpp_joint",
    "L_middle_ip_joint",
    "L_ring_mpp_joint",
    "L_ring_ip_joint",
    "L_little_mpp_joint",
    "L_little_ip_joint",
]

WALKER_C1_RIGHT_HAND_JOINTS = [
    "R_thumb_cmp_joint",
    "R_thumb_mpp_joint",
    "R_thumb_ip_joint",
    "R_index_mpp_joint",
    "R_index_ip_joint",
    "R_middle_mpp_joint",
    "R_middle_ip_joint",
    "R_ring_mpp_joint",
    "R_ring_ip_joint",
    "R_little_mpp_joint",
    "R_little_ip_joint",
]

WALKER_C1_HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]

WALKER_C1_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]

WALKER_C1_LEFT_LEG_JOINTS = [
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]

WALKER_C1_RIGHT_LEG_JOINTS = [
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

WALKER_C1_ARM_JOINTS = WALKER_C1_LEFT_ARM_JOINTS + WALKER_C1_RIGHT_ARM_JOINTS
WALKER_C1_HAND_JOINTS = WALKER_C1_LEFT_HAND_JOINTS + WALKER_C1_RIGHT_HAND_JOINTS
WALKER_C1_LEG_JOINTS = WALKER_C1_LEFT_LEG_JOINTS + WALKER_C1_RIGHT_LEG_JOINTS

# First control scope for tabletop manipulation. Legs are present in the robot
# config but are held at home pose until a full-body policy is introduced.
WALKER_C1_UPPER_BODY_JOINTS = (
    WALKER_C1_LEFT_ARM_JOINTS
    + WALKER_C1_RIGHT_ARM_JOINTS
    + WALKER_C1_LEFT_HAND_JOINTS
    + WALKER_C1_RIGHT_HAND_JOINTS
    + WALKER_C1_HEAD_JOINTS
    + WALKER_C1_WAIST_JOINTS
)

WALKER_C1_HOME_POSE = {
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.0,
    "L_shoulder_pitch_joint": 0.0,
    "L_shoulder_roll_joint": 0.0,
    "L_shoulder_yaw_joint": 0.0,
    "L_elbow_pitch_joint": 0.0,
    "L_elbow_yaw_joint": 0.0,
    "L_wrist_pitch_joint": 0.0,
    "L_wrist_roll_joint": 0.0,
    "R_shoulder_pitch_joint": 0.0,
    "R_shoulder_roll_joint": 0.0,
    "R_shoulder_yaw_joint": 0.0,
    "R_elbow_pitch_joint": 0.0,
    "R_elbow_yaw_joint": 0.0,
    "R_wrist_pitch_joint": 0.0,
    "R_wrist_roll_joint": 0.0,
    "L_hip_pitch_joint": 0.0,
    "L_hip_roll_joint": 0.0,
    "L_hip_yaw_joint": 0.0,
    "L_knee_pitch_joint": 0.08,
    "L_ankle_pitch_joint": 0.0,
    "L_ankle_roll_joint": 0.0,
    "R_hip_pitch_joint": 0.0,
    "R_hip_roll_joint": 0.0,
    "R_hip_yaw_joint": 0.0,
    "R_knee_pitch_joint": 0.08,
    "R_ankle_pitch_joint": 0.0,
    "R_ankle_roll_joint": 0.0,
    "L_thumb_cmp_joint": 0.0,
    "L_thumb_mpp_joint": 0.0,
    "L_thumb_ip_joint": 0.0,
    "L_index_mpp_joint": 0.0,
    "L_index_ip_joint": 0.0,
    "L_middle_mpp_joint": 0.0,
    "L_middle_ip_joint": 0.0,
    "L_ring_mpp_joint": 0.0,
    "L_ring_ip_joint": 0.0,
    "L_little_mpp_joint": 0.0,
    "L_little_ip_joint": 0.0,
    "R_thumb_cmp_joint": 0.0,
    "R_thumb_mpp_joint": 0.0,
    "R_thumb_ip_joint": 0.0,
    "R_index_mpp_joint": 0.0,
    "R_index_ip_joint": 0.0,
    "R_middle_mpp_joint": 0.0,
    "R_middle_ip_joint": 0.0,
    "R_ring_mpp_joint": 0.0,
    "R_ring_ip_joint": 0.0,
    "R_little_mpp_joint": 0.0,
    "R_little_ip_joint": 0.0,
}

# Actuator gains aligned to the proven Walker S2 reference (same Isaac Lab stack,
# same fixed-root upper-body setup, gravity enabled). The original C1 values
# (arm/head 80, waist 120, leg 200) were untuned placeholders too weak to hold
# HOME_POSE against gravity, which collapsed the legs (pigeon-toed feet) and let
# the head droop at load. Hand gains are intentionally left as-is (separate work).
WALKER_C1_ARM_STIFFNESS = {
    name: (500 if ("shoulder_roll" in name or "elbow_pitch" in name) else 600)
    for name in WALKER_C1_ARM_JOINTS
}
WALKER_C1_ARM_DAMPING = {name: 40 for name in WALKER_C1_ARM_JOINTS}
WALKER_C1_HAND_STIFFNESS = {name: 200 for name in WALKER_C1_HAND_JOINTS}
WALKER_C1_HAND_DAMPING = {name: 20 for name in WALKER_C1_HAND_JOINTS}
WALKER_C1_HEAD_STIFFNESS = {name: 600 for name in WALKER_C1_HEAD_JOINTS}
WALKER_C1_HEAD_DAMPING = {name: 60 for name in WALKER_C1_HEAD_JOINTS}
WALKER_C1_WAIST_STIFFNESS = {name: 600 for name in WALKER_C1_WAIST_JOINTS}
WALKER_C1_WAIST_DAMPING = {name: 60 for name in WALKER_C1_WAIST_JOINTS}
WALKER_C1_LEG_STIFFNESS = {
    name: (
        1500 if ("hip_pitch" in name or "knee" in name)
        else 1600 if "ankle" in name
        else 1100  # hip_roll, hip_yaw
    )
    for name in WALKER_C1_LEG_JOINTS
}
WALKER_C1_LEG_DAMPING = {
    name: (
        65 if ("hip_pitch" in name or "knee" in name)
        else 70 if "ankle" in name
        else 55  # hip_roll, hip_yaw
    )
    for name in WALKER_C1_LEG_JOINTS
}

WALKER_C1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(WALKER_C1_USD_PATH.resolve()),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=WALKER_C1_HOME_POSE,
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 60,
                ".*shoulder_roll.*": 60,
                ".*shoulder_yaw.*": 25,
                ".*elbow_.*": 25,
                ".*wrist_.*": 6,
            },
            velocity_limit_sim=14,
            stiffness={name: WALKER_C1_ARM_STIFFNESS[name] for name in WALKER_C1_LEFT_ARM_JOINTS},
            damping={name: WALKER_C1_ARM_DAMPING[name] for name in WALKER_C1_LEFT_ARM_JOINTS},
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 60,
                ".*shoulder_roll.*": 60,
                ".*shoulder_yaw.*": 25,
                ".*elbow_.*": 25,
                ".*wrist_.*": 6,
            },
            velocity_limit_sim=14,
            stiffness={name: WALKER_C1_ARM_STIFFNESS[name] for name in WALKER_C1_RIGHT_ARM_JOINTS},
            damping={name: WALKER_C1_ARM_DAMPING[name] for name in WALKER_C1_RIGHT_ARM_JOINTS},
        ),
        "left_hand": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_HAND_JOINTS,
            effort_limit_sim=50,
            velocity_limit_sim=10,
            stiffness={name: WALKER_C1_HAND_STIFFNESS[name] for name in WALKER_C1_LEFT_HAND_JOINTS},
            damping={name: WALKER_C1_HAND_DAMPING[name] for name in WALKER_C1_LEFT_HAND_JOINTS},
        ),
        "right_hand": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_HAND_JOINTS,
            effort_limit_sim=50,
            velocity_limit_sim=10,
            stiffness={name: WALKER_C1_HAND_STIFFNESS[name] for name in WALKER_C1_RIGHT_HAND_JOINTS},
            damping={name: WALKER_C1_HAND_DAMPING[name] for name in WALKER_C1_RIGHT_HAND_JOINTS},
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_HEAD_JOINTS,
            effort_limit_sim=5,
            velocity_limit_sim=5.24,
            stiffness=WALKER_C1_HEAD_STIFFNESS,
            damping=WALKER_C1_HEAD_DAMPING,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_WAIST_JOINTS,
            effort_limit_sim={
                "waist_yaw_joint": 63,
                "waist_pitch_joint": 165,
                "waist_roll_joint": 110,
            },
            velocity_limit_sim=10.47,
            stiffness=WALKER_C1_WAIST_STIFFNESS,
            damping=WALKER_C1_WAIST_DAMPING,
        ),
        "left_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_pitch.*": 210,
                ".*hip_roll.*": 165,
                ".*hip_yaw.*": 110,
                ".*knee_pitch.*": 210,
                ".*ankle_.*": 60,
            },
            velocity_limit_sim=12.57,
            stiffness={name: WALKER_C1_LEG_STIFFNESS[name] for name in WALKER_C1_LEFT_LEG_JOINTS},
            damping={name: WALKER_C1_LEG_DAMPING[name] for name in WALKER_C1_LEFT_LEG_JOINTS},
        ),
        "right_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_pitch.*": 210,
                ".*hip_roll.*": 165,
                ".*hip_yaw.*": 110,
                ".*knee_pitch.*": 210,
                ".*ankle_.*": 60,
            },
            velocity_limit_sim=12.57,
            stiffness={name: WALKER_C1_LEG_STIFFNESS[name] for name in WALKER_C1_RIGHT_LEG_JOINTS},
            damping={name: WALKER_C1_LEG_DAMPING[name] for name in WALKER_C1_RIGHT_LEG_JOINTS},
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
