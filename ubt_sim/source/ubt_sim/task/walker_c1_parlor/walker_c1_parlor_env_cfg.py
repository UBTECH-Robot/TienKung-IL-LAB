import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import ObservationGroupCfg, SceneEntityCfg
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
    pass


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
