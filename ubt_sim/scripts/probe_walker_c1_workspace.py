# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Read-only probe: where is Walker C1's right hand in world space at the
pre-grasp ready pose? Used to place a graspable object within reach for M2.

Builds the parlor env, drives the robot to the ready pose through the real
controller action path, lets it settle, then prints the world positions of the
robot root and the right-arm/hand links. No recording, no object.

Run:
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/probe_walker_c1_workspace.py --headless --device cpu
"""

import argparse
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Probe Walker C1 right-hand workspace at ready pose.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerC1-Parlor-v0")
parser.add_argument("--settle_steps", type=int, default=150)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args_cli = parser.parse_args()

import sys

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.devices.walker_c1.action_process import reset_hold_targets, to_controller_data

# Ready pose (mirrors collect_walker_c1_pick_place.py READY_*).
READY_CMD = {
    "waist": [0.0, 0.0, 0.0],
    "head": [0.0, 0.35],
    "left_arm": [-0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006],
    "right_arm": [-0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194],
    "left_hand": [0.0] * 6,
    "right_hand": [0.0] * 6,
}

_INTEREST = ("R_shoulder", "R_elbow", "R_wrist", "R_palm", "R_index", "R_thumb")


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()
    reset_hold_targets()

    for _ in range(args_cli.settle_steps):
        env.step(to_controller_data(READY_CMD, env))

    robot = env.scene["robot"]
    root = robot.data.root_pos_w[0].detach().cpu().tolist()
    body_names = list(robot.data.body_names)
    body_pos = robot.data.body_pos_w[0].detach().cpu().tolist()

    print("\n================ WALKER C1 WORKSPACE PROBE (ready pose) ================")
    print(f"root_pos_w (x,y,z) = [{root[0]:.4f}, {root[1]:.4f}, {root[2]:.4f}]")
    print("-----------------------------------------------------------------------")
    print(f"{'link':<26}{'x':>10}{'y':>10}{'z':>10}")
    print("-----------------------------------------------------------------------")
    for name, pos in zip(body_names, body_pos):
        if any(k in name for k in _INTEREST):
            print(f"{name:<26}{pos[0]:>10.4f}{pos[1]:>10.4f}{pos[2]:>10.4f}")
    print("-----------------------------------------------------------------------")

    if "object" in env.scene.keys():
        obj = env.scene["object"]
        opos = obj.data.root_pos_w[0].detach().cpu().tolist()
        print(f"{'OBJECT (settled)':<26}{opos[0]:>10.4f}{opos[1]:>10.4f}{opos[2]:>10.4f}")
        rpalm = dict(zip(body_names, body_pos)).get("R_palm_link")
        if rpalm is not None:
            dx, dy, dz = opos[0] - rpalm[0], opos[1] - rpalm[1], opos[2] - rpalm[2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            print(f"  object - R_palm offset = ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  dist={dist:.3f} m")
    print("=======================================================================\n")

    os._exit(0)


if __name__ == "__main__":
    main()
