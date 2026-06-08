from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig


def _make_joints(prefix: str, n: int) -> list[str]:
    """Generate joint feature names: prefix_j1.pos, prefix_j2.pos, ..."""
    return [f"{prefix}_j{i}.pos" for i in range(1, n + 1)]


@RobotConfig.register_subclass("tienkung")
@dataclass
class TienKungRobotConfig(RobotConfig):
    # ZMQ configuration (LeRobot ↔ Bridge2 internal communication)
    zmq_host: str = "127.0.0.1"
    zmq_cmd_port: int = 5559       # LeRobot PUB → Bridge2 SUB
    zmq_status_port: int = 5560    # Bridge2 PUB → LeRobot SUB
    bridge_enabled: bool = True     # Auto-start Bridge2 subprocess
    bridge_script: str = "/opt/ros2_deploy_bridge.py"  # Path to Bridge2 script

    # ROS2 topics (real robot defaults)
    ros_namespace: str = ""
    cmd_namespace: str = ""

    # Hand type: "inspire" or "brainco"
    hand_type: str = "inspire"

    # Joint group definitions (used for ZMQ message grouping)
    left_arm_joints: list[str] = field(default_factory=lambda: _make_joints("left_arm", 7))
    right_arm_joints: list[str] = field(default_factory=lambda: _make_joints("right_arm", 7))
    left_hand_joints: list[str] = field(default_factory=lambda: _make_joints("left_hand", 6))
    right_hand_joints: list[str] = field(default_factory=lambda: _make_joints("right_hand", 6))

    # Full joint ordering (determines model action/observation tensor dimension mapping).
    # Must be a permutation of the union of the 4 joint groups above.
    # 根据顺序排列的关节配置修改
    all_joints: list[str] = field(default_factory=lambda: (
        _make_joints("left_arm", 7) + _make_joints("right_arm", 7)
        + _make_joints("left_hand", 6) + _make_joints("right_hand", 6)
    ))

    # Safety
    max_relative_target: float | None = None
    disable_torque_on_disconnect: bool = True

    # Home position (14-dim: left arm 7 + right arm 7)
    home_position: list[float] = field(
        default_factory=lambda: [
            -0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006,
            -0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194,
        ]
    )

    # Cameras (keyed by name matching policy's expected image key, e.g. "camera_head")
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        # Validate all_joints is a permutation of the union of 4 joint groups
        group_set = set(self.left_arm_joints + self.right_arm_joints
                        + self.left_hand_joints + self.right_hand_joints)
        all_set = set(self.all_joints)
        if group_set != all_set:
            raise ValueError(
                f"all_joints must be a permutation of the union of 4 joint groups. "
                f"Missing: {group_set - all_set}, Extra: {all_set - group_set}"
            )
        if len(self.all_joints) != len(all_set):
            raise ValueError(
                f"all_joints has duplicates: {len(self.all_joints)} items vs {len(all_set)} unique"
            )
