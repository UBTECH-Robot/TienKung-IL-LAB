import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import EventTermCfg, ObservationGroupCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import (
    WALKER_C1_CFG,
    WALKER_C1_HEAD_JOINTS,
    WALKER_C1_HOME_POSE,
    WALKER_C1_LEFT_ARM_JOINTS,
    WALKER_C1_LEFT_HAND_JOINTS,
    WALKER_C1_LEFT_LEG_JOINTS,
    WALKER_C1_RIGHT_ARM_JOINTS,
    WALKER_C1_RIGHT_HAND_JOINTS,
    WALKER_C1_RIGHT_LEG_JOINTS,
    WALKER_C1_WAIST_JOINTS,
)
from ubt_sim.env.digital_twin_env_cfg import ManagerBasedRLDigitalTwinEnvCfg
from ubt_sim.utils.config_loader import load_config, resolve_asset_path

_TASK_CFG = load_config("walker_c1/parlor.yaml")
_SCENE_USD_PATH = resolve_asset_path(_TASK_CFG["scene"]["usd_path"])
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_HEAD_RGB_CAMERA_CFG = _TASK_CFG["cameras"]["head_rgb"]
_HEAD_RGB_RESOLUTION = _HEAD_RGB_CAMERA_CFG.get("resolution", [640, 480])
_HEAD_RGB_OFFSET = _HEAD_RGB_CAMERA_CFG.get("offset", {})
_HEAD_RGB_INTRINSICS = _HEAD_RGB_CAMERA_CFG.get("intrinsics", {})

PARLOR_SCENE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
)

# Pick-place task props (M2). The parlor scene furniture is visual-only (no
# collision, no RigidBodyAPI), so physics comes from invisible primitive
# colliders aligned to the visuals:
#   - the scene table /World/table spans x [8.144, 8.744], y [5.483, 6.683]
#     with its top at z = 0.897 -> invisible slab collider under the tabletop;
#   - the scene plate /World/plate sits at (8.374, 6.046), rim top z = 0.931
#     -> invisible disk collider as the place-target surface;
#   - the graspable apple is our own rigid red sphere resting on the tabletop
#     (the scene's decorative apple is deactivated in scene_v2_c1.usda).
_TABLE_TOP_Z = 0.897
_TABLE_POS = (8.444, 6.083, _TABLE_TOP_Z - 0.03)  # slab top flush with visual top
_TABLE_SIZE = (0.60, 1.20, 0.06)
_PLATE_POS = (8.374, 6.046, 0.90)                 # disk top at z = 0.925 (in-dish)
_PLATE_RADIUS = 0.085
_PLATE_HEIGHT = 0.05
# r=0.022 (plum-sized apple): the C1 hand's cage aperture is ~5-6cm, so the
# object diameter sets the alignment margin. 5.4cm left only mm of slack —
# grasp outcomes flipped between spawns 6mm apart; 4.4cm gives ~±0.8cm,
# matching what the closed-loop mouth servo can reliably deliver.
_GRASP_OBJECT_RADIUS = 0.022
_GRASP_OBJECT_MASS = 0.08
# Apple start: near the table front edge, in front of the right hand
# (ready-pose palm sits at world ~(8.04, 5.89, 0.85+)).
_GRASP_OBJECT_INIT_POS = (8.21, 5.90, _TABLE_TOP_Z + _GRASP_OBJECT_RADIUS + 0.002)


@configclass
class WalkerC1ParlorSceneCfg(InteractiveSceneCfg):
    scene = PARLOR_SCENE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    robot = WALKER_C1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(_ROBOT_INIT_STATE["pos"]),
            rot=tuple(_ROBOT_INIT_STATE["rot"]),
            joint_pos=WALKER_C1_HOME_POSE,
        ),
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TableTopCollider",
        spawn=sim_utils.CuboidCfg(
            size=_TABLE_SIZE,
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_POS),
    )

    plate = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlateCollider",
        spawn=sim_utils.CylinderCfg(
            radius=_PLATE_RADIUS,
            height=_PLATE_HEIGHT,
            axis="Z",
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2, dynamic_friction=1.2
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=_PLATE_POS),
    )

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        spawn=sim_utils.SphereCfg(
            radius=_GRASP_OBJECT_RADIUS,
            # NOTE: do NOT add high angular_damping to stop post-release rolling:
            # a ball that cannot roll gets squeeze-ejected by the closing fingers
            # (tested angular_damping=2.0 -> apple shot off the table on every
            # descend). Rolling is instead controlled by releasing the apple
            # ~1cm above the plate surface.
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=_GRASP_OBJECT_MASS),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.75, 0.12, 0.10)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2, dynamic_friction=1.2
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=_GRASP_OBJECT_INIT_POS),
    )

    camera = TiledCameraCfg(
        prim_path=_HEAD_RGB_CAMERA_CFG["prim_path"],
        offset=TiledCameraCfg.OffsetCfg(
            pos=tuple(_HEAD_RGB_OFFSET.get("pos", [0.0, 0.0, 0.0])),
            rot=tuple(_HEAD_RGB_OFFSET.get("rot", [1.0, 0.0, 0.0, 0.0])),
            convention=_HEAD_RGB_OFFSET.get("convention", "ros"),
        ),
        update_period=0.0333,
        height=int(_HEAD_RGB_RESOLUTION[1]),
        width=int(_HEAD_RGB_RESOLUTION[0]),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=float(_HEAD_RGB_INTRINSICS.get("focal_length", 24.0)),
            focus_distance=float(_HEAD_RGB_INTRINSICS.get("focus_distance", 400.0)),
            horizontal_aperture=float(_HEAD_RGB_INTRINSICS.get("horizontal_aperture", 44.76)),
            vertical_aperture=float(_HEAD_RGB_INTRINSICS.get("vertical_aperture", 24.99)),
            clipping_range=tuple(_HEAD_RGB_INTRINSICS.get("clipping_range", [0.1, 1.0e5])),
        ),
    )


@configclass
class ActionsCfg:
    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_ARM_JOINTS,
        scale=1.0,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_ARM_JOINTS,
        scale=1.0,
    )
    left_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_HEAD_JOINTS,
        scale=1.0,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_WAIST_JOINTS,
        scale=1.0,
    )
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_LEG_JOINTS,
        scale=1.0,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_LEG_JOINTS,
        scale=1.0,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        joint_pos = mdp.ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # Without this, env.reset() does NOT restore scene entity states: the
    # apple stays wherever the previous episode left it (and the robot keeps
    # its joint state). Masked for a long time by --randomize (which writes
    # the apple pose explicitly) and by episodes ending at the ready pose.
    reset_scene = EventTermCfg(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class RewardsCfg:
    pass


@configclass
class TerminationsCfg:
    pass


@configclass
class WalkerC1ParlorEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    scene: WalkerC1ParlorSceneCfg = WalkerC1ParlorSceneCfg(num_envs=1, env_spacing=5.0)
    viewer = ViewerCfg(eye=(5.5, 6.0, 2.4), lookat=(7.9, 6.08, 1.2))

    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    episode_length_s = float(_TASK_CFG["simulation"].get("episode_length_s", 1000.0))
    dynamic_reset_gripper_effort_limit = False

    def __post_init__(self):
        super().__post_init__()
        self.sim.use_fabric = True
        self.sim.enable_scene_query_support = False
        self.sim.disable_contact_processing = True
        self.sim.dt = float(_TASK_CFG["simulation"].get("dt", 0.01))
        self.decimation = int(_TASK_CFG["simulation"].get("decimation", 1))
        self.sim.render_interval = int(_TASK_CFG["simulation"].get("render_interval", 3))

    def use_teleop_device(self, teleop_device: str) -> None:
        self.task_type = teleop_device

    def preprocess_device_action(self, action, teleop_device):
        if isinstance(action, dict) and "walker_c1" in action:
            return action["walker_c1"]
        return action
