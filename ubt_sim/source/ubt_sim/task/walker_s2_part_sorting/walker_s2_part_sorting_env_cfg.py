import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_s2.config import (
    WALKER_S2_CFG,
    WALKER_S2_HOME_POSE,
    WALKER_S2_LEFT_ARM_JOINTS,
    WALKER_S2_RIGHT_ARM_JOINTS,
    WALKER_S2_LEFT_HAND_JOINTS,
    WALKER_S2_RIGHT_HAND_JOINTS,
    WALKER_S2_HEAD_JOINTS,
    WALKER_S2_WAIST_JOINTS,
    WALKER_S2_LEFT_LEG_JOINTS,
    WALKER_S2_RIGHT_LEG_JOINTS,
)
from ubt_sim.env.digital_twin_env_cfg import ManagerBasedRLDigitalTwinEnvCfg
from ubt_sim.utils.config_loader import load_config, resolve_asset_path

# Load scene config from YAML
_TASK_CFG = load_config("walker_s2_part_sorting.yaml")
_SCENE_USD_PATH = resolve_asset_path(_TASK_CFG["scene"]["usd_path"])
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_OBJECTS_CFG = _TASK_CFG["objects"]
_HEAD_RGB_CAMERA_CFG = _TASK_CFG["cameras"]["head_rgb"]
_HEAD_RGB_RESOLUTION = _HEAD_RGB_CAMERA_CFG.get("resolution", [640, 480])


def _usd_asset_cfg(asset_cfg: dict) -> AssetBaseCfg:
    return AssetBaseCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=resolve_asset_path(asset_cfg["usd_path"]),
            scale=tuple(asset_cfg.get("scale", (1.0, 1.0, 1.0))),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(asset_cfg.get("pos", (0.0, 0.0, 0.0))),
            rot=tuple(asset_cfg.get("rot", (1.0, 0.0, 0.0, 0.0))),
        ),
    )


_PART_CFGS = {part_cfg["name"]: part_cfg for part_cfg in _OBJECTS_CFG["parts"]}


@configclass
class WalkerS2PartSortingSceneCfg(InteractiveSceneCfg):
    scene = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Scene",
        spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
    )

    table = _usd_asset_cfg(_OBJECTS_CFG["table"]).replace(prim_path="{ENV_REGEX_NS}/Table")
    box = _usd_asset_cfg(_OBJECTS_CFG["box"]).replace(prim_path="{ENV_REGEX_NS}/Box")
    part_a_ori = _usd_asset_cfg(_PART_CFGS["part_a_ori"]).replace(prim_path="{ENV_REGEX_NS}/PartA_Ori")
    part_a_red = _usd_asset_cfg(_PART_CFGS["part_a_red"]).replace(prim_path="{ENV_REGEX_NS}/PartA_Red")
    part_b_blue = _usd_asset_cfg(_PART_CFGS["part_b_blue"]).replace(prim_path="{ENV_REGEX_NS}/PartB_Blue")
    part_b_ori = _usd_asset_cfg(_PART_CFGS["part_b_ori"]).replace(prim_path="{ENV_REGEX_NS}/PartB_Ori")

    robot = WALKER_S2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(_ROBOT_INIT_STATE["pos"]),
            rot=tuple(_ROBOT_INIT_STATE["rot"]),
            joint_pos=WALKER_S2_HOME_POSE,
        ),
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

    camera = TiledCameraCfg(
        prim_path=_HEAD_RGB_CAMERA_CFG["prim_path"],
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="ros",
        ),
        update_period=0.0333,
        height=int(_HEAD_RGB_RESOLUTION[1]),
        width=int(_HEAD_RGB_RESOLUTION[0]),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=44.76,
            vertical_aperture=24.99,
            clipping_range=(0.1, 1.0e5),
        ),
    )


@configclass
class ActionsCfg:
    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_ARM_JOINTS,
        scale=1.0,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_ARM_JOINTS,
        scale=1.0,
    )
    left_gripper_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_gripper_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_HEAD_JOINTS,
        scale=1.0,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_WAIST_JOINTS,
        scale=1.0,
    )
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_LEG_JOINTS,
        scale=1.0,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_LEG_JOINTS,
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
class WalkerS2PartSortingEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    scene: WalkerS2PartSortingSceneCfg = WalkerS2PartSortingSceneCfg(num_envs=1, env_spacing=5.0)
    viewer = ViewerCfg(eye=(2.8, 2.0, 2.0), lookat=(0.8, 0.25, 1.0))

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
        if isinstance(action, dict) and "walker_s2" in action:
            return action["walker_s2"]
        return action
