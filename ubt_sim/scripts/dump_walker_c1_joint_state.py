# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Read-only dump of Walker C1 joint state after env.reset() (load-only style).

Builds the same parlor task env as sim_runner.py, resets it once, and prints the
actual per-joint positions + root pose. No physics stepping, no ZMQ, no control.
Use to compare articulation state across git commits objectively (no eyeballing).

Run:
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/dump_walker_c1_joint_state.py --headless --device cpu
"""

import argparse
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Dump Walker C1 joint state after reset.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerC1-Parlor-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--steps",
    type=int,
    default=0,
    help="After reset, drive HOME_POSE joint targets and step physics N times, then dump.",
)
parser.add_argument(
    "--use-controller",
    dest="use_controller",
    action="store_true",
    help=(
        "Step through the REAL controller action path (to_controller_data + env.step, "
        "with no external command) instead of hand-setting HOME_POSE targets. "
        "Faithfully reproduces controller-mode startup behavior."
    ),
)
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

from ubt_sim.devices.walker_c1.config import WALKER_C1_HOME_POSE


def _get_robot(env):
    """Return the robot articulation regardless of its scene key."""
    articulations = env.scene.articulations
    if "robot" in articulations:
        return articulations["robot"]
    # Fall back to the first articulation in the scene.
    name, art = next(iter(articulations.items()))
    print(f"[INFO] robot articulation key = '{name}'")
    return art


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()

    robot = _get_robot(env)

    if args_cli.steps > 0 and args_cli.use_controller:
        # Faithful controller-mode proxy: feed an empty command through the real
        # action pipeline (to_controller_data builds the action from the startup
        # hold targets) and step the env exactly like sim_runner does.
        from ubt_sim.devices.walker_c1.action_process import reset_hold_targets, to_controller_data

        reset_hold_targets()
        for _ in range(args_cli.steps):
            action = to_controller_data({}, env)
            env.step(action)
        print(f"[INFO] Stepped {args_cli.steps} env.step() calls through the real controller path.")
    elif args_cli.steps > 0:
        import torch

        joint_names_now = list(robot.data.joint_names)
        home = torch.tensor(
            [[WALKER_C1_HOME_POSE.get(n, 0.0) for n in joint_names_now]],
            device=robot.device,
            dtype=robot.data.joint_pos.dtype,
        )
        for _ in range(args_cli.steps):
            robot.set_joint_position_target(home)
            robot.write_data_to_sim()
            env.sim.step(render=False)
            robot.update(env.sim.get_physics_dt())
        print(f"[INFO] Stepped {args_cli.steps} physics steps while driving HOME_POSE targets.")

    joint_names = list(robot.data.joint_names)
    joint_pos = robot.data.joint_pos[0].detach().cpu().tolist()
    root_pos = robot.data.root_pos_w[0].detach().cpu().tolist()
    root_quat = robot.data.root_quat_w[0].detach().cpu().tolist()

    print("\n================ WALKER C1 JOINT STATE DUMP ================")
    print(f"task = {args_cli.task}")
    print(f"device = {args_cli.device}")
    print(f"root_pos_w  (x,y,z)    = {root_pos}")
    print(f"root_quat_w (w,x,y,z)  = {root_quat}")
    print(f"num_joints = {len(joint_names)}")
    print("-----------------------------------------------------------")
    print(f"{'joint_name':<26}{'actual':>10}{'home_pose':>12}{'diff':>10}")
    print("-----------------------------------------------------------")

    pos_by_name = dict(zip(joint_names, joint_pos))
    # Print in a stable, human-readable order: sorted by name.
    max_diff = 0.0
    max_diff_joint = None
    for name in sorted(pos_by_name.keys()):
        actual = pos_by_name[name]
        home = WALKER_C1_HOME_POSE.get(name)
        if home is None:
            home_str = "  (n/a)"
            diff_str = "   n/a"
        else:
            diff = actual - home
            home_str = f"{home:>12.4f}"
            diff_str = f"{diff:>10.4f}"
            if abs(diff) > abs(max_diff):
                max_diff = diff
                max_diff_joint = name
            print(f"{name:<26}{actual:>10.4f}{home_str}{diff_str}")
            continue
        print(f"{name:<26}{actual:>10.4f}{home_str}{diff_str}")

    print("-----------------------------------------------------------")
    print(f"max |actual-home| = {abs(max_diff):.4f} at {max_diff_joint}")
    print("===========================================================\n")

    # Isaac/AppLauncher can hang on teardown in minimal scripts; exit hard.
    os._exit(0)


if __name__ == "__main__":
    main()
