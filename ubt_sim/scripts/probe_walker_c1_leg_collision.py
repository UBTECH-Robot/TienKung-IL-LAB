#!/usr/bin/env python3
"""Localize the Walker C1 left-leg collision in the parlor task.

Builds the real parlor task env, settles the robot through the controller action
path, then dumps the WORLD positions of the left/right leg links side by side so
we can see where the left leg is pushed and how far it deviates from the (clean,
symmetric) right leg. Also reports the scene's world bounding box for context.

Run:
  /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_leg_collision.py \
    --headless --device cpu --enable_cameras --steps 300
"""

import argparse
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Localize Walker C1 left-leg collision.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerC1-Parlor-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=300)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args_cli = parser.parse_args()

import sys

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")
sys.argv.append("--/physics/suppressReadback=false")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

import ubt_sim  # noqa: F401  (importing the package registers the UBTSim-* gym tasks)


def _get_robot(env):
    articulations = env.scene.articulations
    if "robot" in articulations:
        return articulations["robot"]
    _, art = next(iter(articulations.items()))
    return art


def _scene_bbox(env):
    """World-space AABB of the parlor scene prim, for context."""
    try:
        from pxr import Usd, UsdGeom

        stage = env.sim.stage
        prim = stage.GetPrimAtPath("/World/envs/env_0/Scene")
        if not prim or not prim.IsValid():
            return None
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return (list(rng.GetMin()), list(rng.GetMax()))
    except Exception as exc:  # noqa: BLE001
        return f"(bbox unavailable: {exc})"


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()
    robot = _get_robot(env)

    # Settle through the real controller path (no external command).
    from ubt_sim.devices.walker_c1.action_process import reset_hold_targets, to_controller_data

    reset_hold_targets()
    for _ in range(args_cli.steps):
        action = to_controller_data({}, env)
        env.step(action)

    body_names = list(robot.data.body_names)
    body_pos = robot.data.body_pos_w[0].detach().cpu().tolist()
    pos_by_body = dict(zip(body_names, body_pos))
    root = robot.data.root_pos_w[0].detach().cpu().tolist()

    # left leg links paired with their right-side mirror
    leg_keys = ["hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll"]

    def find(side_prefix, key):
        for name in body_names:
            if name.startswith(side_prefix) and key in name:
                return name
        return None

    print("\n============ WALKER C1 LEFT-LEG COLLISION LOCALIZE ============")
    print(f"task={args_cli.task}  steps={args_cli.steps}")
    print(f"root_pos_w (x,y,z) = [{root[0]:.3f}, {root[1]:.3f}, {root[2]:.3f}]")
    print(f"scene world bbox   = {_scene_bbox(env)}")
    print("--------------------------------------------------------------")
    print(f"{'link role':<14}{'LEFT (x,y,z)':>26}{'RIGHT (x,y,z)':>26}{'dZ':>8}")
    print("--------------------------------------------------------------")
    for key in leg_keys:
        ln = find("L_", key)
        rn = find("R_", key)
        if ln is None or rn is None:
            continue
        lp = pos_by_body[ln]
        rp = pos_by_body[rn]
        dz = lp[2] - rp[2]
        print(
            f"{key:<14}"
            f"({lp[0]:>6.3f},{lp[1]:>6.3f},{lp[2]:>6.3f})"
            f"     ({rp[0]:>6.3f},{rp[1]:>6.3f},{rp[2]:>6.3f})"
            f"{dz:>8.3f}"
        )
    print("--------------------------------------------------------------")
    print("Right leg is the clean/symmetric reference; large L/R gaps = where the")
    print("left leg is pushed. Compare foot (ankle_roll) height to the table top.")
    print("==============================================================\n")

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
