#!/usr/bin/env python3
"""Probe Walker C1 hand actuator runtime settings and direct joint response.

Run inside the Isaac Sim container:

    /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source"))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Probe Walker C1 hand runtime actuator behavior.")
parser.add_argument("--steps", type=int, default=600, help="Number of simulation steps.")
parser.add_argument("--target", type=float, default=0.8, help="Right hand joint position target in radians.")
parser.add_argument("--hand-effort", type=float, default=None, help="Temporarily override both hand effort limits.")
parser.add_argument("--hand-stiffness", type=float, default=None, help="Temporarily override both hand stiffness values.")
parser.add_argument("--hand-damping", type=float, default=None, help="Temporarily override both hand damping values.")
parser.add_argument("--ip-ratio", type=float, default=1.0, help="Target ratio for non-thumb IP joints.")
parser.add_argument("--thumb-ip-ratio", type=float, default=1.0, help="Target ratio for the thumb IP joint.")
parser.add_argument(
    "--teleport-target",
    action="store_true",
    help="Write the right hand joint state directly to target before stepping.",
)
parser.add_argument("--disable-gravity", action="store_true", help="Temporarily disable gravity for the articulation.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import WALKER_C1_CFG, WALKER_C1_RIGHT_HAND_JOINTS

for _actuator_name in ["left_hand", "right_hand"]:
    _actuator_cfg = WALKER_C1_CFG.actuators[_actuator_name]
    if args_cli.hand_effort is not None:
        _actuator_cfg.effort_limit_sim = float(args_cli.hand_effort)
    if args_cli.hand_stiffness is not None:
        _actuator_cfg.stiffness = float(args_cli.hand_stiffness)
    if args_cli.hand_damping is not None:
        _actuator_cfg.damping = float(args_cli.hand_damping)
if args_cli.disable_gravity:
    WALKER_C1_CFG.spawn.rigid_props.disable_gravity = True


@configclass
class WalkerC1ProbeSceneCfg(InteractiveSceneCfg):
    robot = WALKER_C1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


def _compact(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu()
        if value.numel() > 16:
            return {
                "shape": list(value.shape),
                "min": float(value.min().item()),
                "max": float(value.max().item()),
                "first": value.flatten()[:8].tolist(),
            }
        return value.tolist()
    if isinstance(value, (list, tuple)):
        if len(value) > 16:
            return {"len": len(value), "first": list(value[:8])}
        return list(value)
    if isinstance(value, dict):
        return {str(k): _compact(v) for k, v in value.items()}
    return value


def _print_actuator_info(robot) -> None:
    print("[INFO] runtime actuators:")
    for name, actuator in robot.actuators.items():
        if "hand" not in name:
            continue
        print(f"  actuator={name} class={actuator.__class__.__name__}")
        for attr in [
            "joint_indices",
            "joint_names",
            "stiffness",
            "damping",
            "effort_limit",
            "effort_limit_sim",
            "velocity_limit",
            "velocity_limit_sim",
            "computed_effort",
            "applied_effort",
        ]:
            if hasattr(actuator, attr):
                print(f"    {attr}: {_compact(getattr(actuator, attr))}")


def _print_joint_physics_info(robot, joint_ids: list[int], joint_names: list[str]) -> None:
    print(f"[INFO] num_fixed_tendons={getattr(robot, 'num_fixed_tendons', '-')}")
    if getattr(robot, "fixed_tendon_names", None):
        print(f"[INFO] fixed_tendon_names={robot.fixed_tendon_names}")
    fields = [
        "joint_armature",
        "joint_friction_coeff",
        "joint_dynamic_friction_coeff",
        "joint_viscous_friction_coeff",
        "joint_effort_limits",
        "joint_vel_limits",
    ]
    print("[INFO] right hand joint physics:")
    for name, joint_id in zip(joint_names, joint_ids):
        parts = [f"{name}"]
        for field in fields:
            value = getattr(robot.data, field, None)
            if value is not None:
                parts.append(f"{field}={_compact(value[0, joint_id])}")
        print("  " + " ".join(parts))


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    scene = InteractiveScene(WalkerC1ProbeSceneCfg(num_envs=1, env_spacing=3.0))

    sim.reset()
    scene.reset()
    robot = scene["robot"]
    joint_names = list(robot.data.joint_names)
    right_hand_ids = [joint_names.index(name) for name in WALKER_C1_RIGHT_HAND_JOINTS]

    print(f"[INFO] device={args_cli.device} steps={args_cli.steps} target={args_cli.target}")
    _print_actuator_info(robot)
    print("[INFO] right hand ids:")
    for joint_id, name in zip(right_hand_ids, WALKER_C1_RIGHT_HAND_JOINTS):
        print(f"  {joint_id:02d} {name}")
    _print_joint_physics_info(robot, right_hand_ids, WALKER_C1_RIGHT_HAND_JOINTS)

    target = robot.data.default_joint_pos.clone()
    target_map = {
        "R_thumb_cmp_joint": float(args_cli.target),
        "R_thumb_mpp_joint": float(args_cli.target),
        "R_thumb_ip_joint": float(args_cli.target) * float(args_cli.thumb_ip_ratio),
        "R_index_mpp_joint": float(args_cli.target),
        "R_index_ip_joint": float(args_cli.target) * float(args_cli.ip_ratio),
        "R_middle_mpp_joint": float(args_cli.target),
        "R_middle_ip_joint": float(args_cli.target) * float(args_cli.ip_ratio),
        "R_ring_mpp_joint": float(args_cli.target),
        "R_ring_ip_joint": float(args_cli.target) * float(args_cli.ip_ratio),
        "R_little_mpp_joint": float(args_cli.target),
        "R_little_ip_joint": float(args_cli.target) * float(args_cli.ip_ratio),
    }
    for joint_id, name in zip(right_hand_ids, WALKER_C1_RIGHT_HAND_JOINTS):
        target[:, joint_id] = target_map[name]
    if args_cli.teleport_target:
        zero_vel = robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(target, zero_vel)
        scene.update(sim.get_physics_dt())
        print("[INFO] wrote right hand joint state directly to target before stepping.")

    for _ in range(max(args_cli.steps, 1)):
        robot.set_joint_position_target(target)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())

    pos = robot.data.joint_pos[0, right_hand_ids].detach().cpu().tolist()
    vel = robot.data.joint_vel[0, right_hand_ids].detach().cpu().tolist()
    soft_limits = robot.data.soft_joint_pos_limits[0, right_hand_ids].detach().cpu().tolist()
    pos_map = dict(zip(WALKER_C1_RIGHT_HAND_JOINTS, pos))
    vel_map = dict(zip(WALKER_C1_RIGHT_HAND_JOINTS, vel))
    limits_map = dict(zip(WALKER_C1_RIGHT_HAND_JOINTS, soft_limits))
    _print_actuator_info(robot)
    print("[RESULT] right hand final positions:")
    for name in WALKER_C1_RIGHT_HAND_JOINTS:
        lower, upper = limits_map[name]
        print(
            f"  {name}: target={target_map[name]:.6f} pos={pos_map[name]:.6f} "
            f"vel={vel_map[name]:.6f} limit=[{lower:.6f}, {upper:.6f}]"
        )

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
