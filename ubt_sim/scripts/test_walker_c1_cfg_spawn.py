#!/usr/bin/env python3
"""Smoke test Walker C1 ArticulationCfg spawning through Isaac Lab.

Run inside the Isaac Sim container:

    /isaac-sim/python.sh -u /ubt_sim/scripts/test_walker_c1_cfg_spawn.py --headless
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source"))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn Walker C1 from WALKER_C1_CFG and print articulation info.")
parser.add_argument("--steps", type=int, default=10, help="Number of simulation steps after reset.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import WALKER_C1_CFG, WALKER_C1_HOME_POSE, WALKER_C1_USD_PATH


@configclass
class WalkerC1SpawnSceneCfg(InteractiveSceneCfg):
    robot = WALKER_C1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


def main() -> None:
    print(f"[INFO] Walker C1 USD: {WALKER_C1_USD_PATH}")
    print(f"[INFO] home_pose_count={len(WALKER_C1_HOME_POSE)}")

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 1.0])

    scene_cfg = WalkerC1SpawnSceneCfg(num_envs=1, env_spacing=3.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    scene.reset()
    print("[INFO] Simulation and scene reset complete.")

    robot = scene["robot"]
    joint_names = list(robot.data.joint_names)
    print(f"[OK] robot_spawned=True")
    print(f"[INFO] joint_count={len(joint_names)}")
    print("[INFO] first_joints=")
    for name in joint_names[:20]:
        print(f"  {name}")
    if len(joint_names) > 20:
        print(f"  ... ({len(joint_names) - 20} more)")

    expected = set(WALKER_C1_HOME_POSE)
    missing = sorted(expected - set(joint_names))
    extra = sorted(set(joint_names) - expected)
    print(f"[INFO] home_pose_missing_from_spawn={missing}")
    print(f"[INFO] spawn_extra_joints={extra}")
    if missing:
        raise RuntimeError(f"Spawned articulation missing configured joints: {missing}")

    for _ in range(max(args_cli.steps, 1)):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())

    print("[OK] spawn_smoke_test_passed")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
