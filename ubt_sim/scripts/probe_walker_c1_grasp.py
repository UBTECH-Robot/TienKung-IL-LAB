# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Empirical grasp test: can Walker C1's right hand physically hold a ball?

The parlor task now has a static collision table with a sphere resting on it,
placed in the right-hand grasp zone at the ready pose. This script:
  1. settles the robot at the ready pose (hand around the ball),
  2. closes the right hand,
  3. lifts the arm,
and reports whether the ball rose WITH the hand (grasp held) or stayed on the
table (grasp failed). This settles the "can C1 friction-grip an object under
gravity, given the known finger-droop issue" question with data, not guesswork.

Run:
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/probe_walker_c1_grasp.py --headless --device cpu --enable_cameras
"""

import argparse
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Walker C1 physical grasp test.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerC1-Parlor-v0")
parser.add_argument("--grip", type=float, default=0.9, help="Right-hand close target (rad).")
parser.add_argument("--lift_shoulder", type=float, default=-0.9, help="R_shoulder_pitch lift target.")
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

READY = {
    "waist": [0.0, 0.0, 0.0],
    "head": [0.0, 0.35],
    "left_arm": [-0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006],
    "right_arm": [-0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194],
    "left_hand": [0.0] * 6,
    "right_hand": [0.0] * 6,
}


def _lerp(a, b, t):
    return [(1.0 - t) * x + t * y for x, y in zip(a, b)]


def _palm_z(env):
    robot = env.scene["robot"]
    names = list(robot.data.body_names)
    pos = robot.data.body_pos_w[0].detach().cpu().tolist()
    return dict(zip(names, pos)).get("R_palm_link")


def _ball_pos(env):
    return env.scene["object"].data.root_pos_w[0].detach().cpu().tolist()


def _run(env, cmd, steps):
    for _ in range(steps):
        env.step(to_controller_data(cmd, env))


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()
    reset_hold_targets()

    cmd = {k: list(v) for k, v in READY.items()}

    # Phase 0: settle at ready (hand around ball, ball settles on table).
    _run(env, cmd, 120)
    palm0 = _palm_z(env)
    ball0 = _ball_pos(env)
    print(f"[settle]  palm={[round(v,3) for v in palm0]}  ball={[round(v,3) for v in ball0]}")

    # Phase 1: close the right hand around the ball.
    start = list(cmd["right_hand"])
    close = [args_cli.grip] * 6
    for i in range(60):
        cmd["right_hand"] = _lerp(start, close, (i + 1) / 60.0)
        env.step(to_controller_data(cmd, env))
    palm1 = _palm_z(env)
    ball1 = _ball_pos(env)
    print(f"[closed]  palm={[round(v,3) for v in palm1]}  ball={[round(v,3) for v in ball1]}")

    # Phase 2: lift the arm (raise shoulder pitch), keep hand closed.
    start_arm = list(cmd["right_arm"])
    lift_arm = list(cmd["right_arm"])
    lift_arm[0] = args_cli.lift_shoulder  # R_shoulder_pitch
    for i in range(100):
        cmd["right_arm"] = _lerp(start_arm, lift_arm, (i + 1) / 100.0)
        env.step(to_controller_data(cmd, env))
    _run(env, cmd, 40)  # settle at top
    palm2 = _palm_z(env)
    ball2 = _ball_pos(env)
    print(f"[lifted]  palm={[round(v,3) for v in palm2]}  ball={[round(v,3) for v in ball2]}")

    # Verdict: did the ball rise with the palm during the lift?
    palm_rise = palm2[2] - palm1[2]
    ball_rise = ball2[2] - ball1[2]
    print("\n================ C1 GRASP TEST VERDICT ================")
    print(f"palm z rise during lift = {palm_rise:+.3f} m")
    print(f"ball z rise during lift = {ball_rise:+.3f} m")
    print(f"ball final z            = {ball2[2]:.3f}  (table top ~0.78, started ~{ball0[2]:.3f})")
    if palm_rise < 0.03:
        print("INCONCLUSIVE: the palm did not actually rise; adjust --lift_shoulder.")
    elif ball_rise > 0.6 * palm_rise:
        print(f"GRASP HELD: ball tracked the hand ({ball_rise:.3f} vs palm {palm_rise:.3f}).")
    else:
        print(f"GRASP FAILED: ball stayed/dropped ({ball_rise:.3f} vs palm {palm_rise:.3f}).")
    print("======================================================\n")

    os._exit(0)


if __name__ == "__main__":
    main()
