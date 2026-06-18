from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from ubt_sim.utils.constant import ASSETS_ROOT

"""Configuration for the Walker S2 robot asset."""
ROBOTS_ROOT = Path(ASSETS_ROOT) / "robots"
WALKER_S2_USD_PATH = ROBOTS_ROOT / "walker_s2" / "s2_v1.usd"

WALKER_S2_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]

WALKER_S2_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

# Finger joints are driven by separate gripper assets in the baseline model and are
# not part of the loaded robot articulation. Keep these empty for Phase 1
# load-only validation; gripper integration belongs to a later control phase.
WALKER_S2_LEFT_HAND_JOINTS = []
WALKER_S2_RIGHT_HAND_JOINTS = []

WALKER_S2_HEAD_JOINTS = [
    "head_pitch_joint",
    "head_yaw_joint",
]

WALKER_S2_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
]

WALKER_S2_LEFT_LEG_JOINTS = [
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_hip_pitch_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]

WALKER_S2_RIGHT_LEG_JOINTS = [
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_hip_pitch_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

WALKER_S2_HOME_POSE = {
    "L_shoulder_pitch_joint": 0.09322471888572098,
    "L_shoulder_roll_joint": -0.5933223843430208,
    "L_shoulder_yaw_joint": -1.595878574835185,
    "L_elbow_roll_joint": -1.8963565338596158,
    "L_elbow_yaw_joint": 1.4000461262831179,
    "L_wrist_pitch_joint": -0.00048740902645395785,
    "L_wrist_roll_joint": 0.0998718010009366,
    "R_shoulder_pitch_joint": -0.09321727661087699,
    "R_shoulder_roll_joint": -0.5933455607833843,
    "R_shoulder_yaw_joint": 1.595869459316937,
    "R_elbow_roll_joint": -1.8963607249359917,
    "R_elbow_yaw_joint": -1.4000874256427638,
    "R_wrist_pitch_joint": 0.00048144049606466176,
    "R_wrist_roll_joint": 0.09985407619802703,
    "head_pitch_joint": 0.0,
    "head_yaw_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "L_hip_roll_joint": 0.0,
    "L_hip_yaw_joint": 0.0,
    "L_hip_pitch_joint": 0.0,
    "L_knee_pitch_joint": 0.0,
    "L_ankle_pitch_joint": 0.0,
    "L_ankle_roll_joint": 0.0,
    "R_hip_roll_joint": 0.0,
    "R_hip_yaw_joint": 0.0,
    "R_hip_pitch_joint": 0.0,
    "R_knee_pitch_joint": 0.0,
    "R_ankle_pitch_joint": 0.0,
    "R_ankle_roll_joint": 0.0,
}

WALKER_S2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(WALKER_S2_USD_PATH.resolve()),
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
        joint_pos=WALKER_S2_HOME_POSE,
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_LEFT_ARM_JOINTS,
            effort_limit_sim=80,
            velocity_limit_sim=4,
            stiffness=80,
            damping=20,
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_RIGHT_ARM_JOINTS,
            effort_limit_sim=80,
            velocity_limit_sim=4,
            stiffness=80,
            damping=20,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_HEAD_JOINTS,
            effort_limit_sim=5,
            velocity_limit_sim=4,
            stiffness=40,
            damping=8,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_WAIST_JOINTS,
            effort_limit_sim=265,
            velocity_limit_sim=4,
            stiffness=120,
            damping=20,
        ),
        "left_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_LEFT_LEG_JOINTS,
            effort_limit_sim=250,
            velocity_limit_sim=10,
            stiffness=120,
            damping=20,
        ),
        "right_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_RIGHT_LEG_JOINTS,
            effort_limit_sim=250,
            velocity_limit_sim=10,
            stiffness=120,
            damping=20,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
