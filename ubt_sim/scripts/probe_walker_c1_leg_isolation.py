#!/usr/bin/env python3
"""Isolation test for the Walker C1 left-leg asymmetry.

Spawns WALKER_C1_CFG ALONE (no parlor scene, no ground plane), drives every joint
to HOME_POSE, settles N steps, and dumps the left/right leg joints side by side.

Logic:
  - If the legs settle symmetric here -> the asymmetry seen in the parlor task is
    caused by the robot colliding with the parlor scene furniture.
  - If the left leg is STILL twisted here -> the cause is intrinsic to the robot /
    reset, not the scene.

Run:
  /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_leg_isolation.py \
    --headless --device cpu --steps 300
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source"))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Walker C1 leg asymmetry isolation probe.")
parser.add_argument("--steps", type=int, default=300, help="Number of simulation steps to settle.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import (
    WALKER_C1_CFG,
    WALKER_C1_LEFT_LEG_JOINTS,
    WALKER_C1_RIGHT_LEG_JOINTS,
)


@configclass
class WalkerC1LegProbeSceneCfg(InteractiveSceneCfg):
    robot = WALKER_C1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


# left leg joint  ->  its mirror on the right leg (same kinematic role)
_LR_PAIRS = [
    (l, l.replace("L_", "R_", 1)) for l in WALKER_C1_LEFT_LEG_JOINTS
]


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    scene = InteractiveScene(WalkerC1LegProbeSceneCfg(num_envs=1, env_spacing=3.0))

    sim.reset()
    scene.reset()
    robot = scene["robot"]

    # Drive every joint to its HOME_POSE default and settle.
    target = robot.data.default_joint_pos.clone()
    for _ in range(max(args_cli.steps, 1)):
        robot.set_joint_position_target(target)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())

    joint_names = list(robot.data.joint_names)
    pos = robot.data.joint_pos[0].detach().cpu().tolist()
    pos_map = dict(zip(joint_names, pos))

    print("\n============ WALKER C1 LEG ISOLATION (no scene) ============")
    print(f"steps = {args_cli.steps}   device = {args_cli.device}")
    print("(robot spawned alone: no parlor furniture, no ground plane)")
    print("-----------------------------------------------------------")
    print(f"{'joint role':<16}{'LEFT':>10}{'RIGHT':>10}{'|L|-|R|':>10}")
    print("-----------------------------------------------------------")
    max_asym = 0.0
    max_asym_role = None
    for l_name, r_name in _LR_PAIRS:
        role = l_name.replace("L_", "").replace("_joint", "")
        lv = pos_map.get(l_name, 0.0)
        rv = pos_map.get(r_name, 0.0)
        asym = abs(lv) - abs(rv)
        if abs(asym) > abs(max_asym):
            max_asym = asym
            max_asym_role = role
        print(f"{role:<16}{lv:>10.4f}{rv:>10.4f}{asym:>10.4f}")
    print("-----------------------------------------------------------")
    print(f"largest |L|-|R| asymmetry = {max_asym:.4f} at {max_asym_role}")
    print("===========================================================\n")

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
