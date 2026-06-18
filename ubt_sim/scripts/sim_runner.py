# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run ubt_sim teleoperation environments."""

import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import signal

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="UBT Sim teleoperation environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument("--perf_stats", action="store_true", help="Print performance statistics.")
parser.add_argument("--load_only", action="store_true", help="Load and render the environment without teleop control.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

import sys

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import time

import gymnasium as gym
import torch
from pxr import Gf, Sdf, UsdGeom, UsdShade
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.utils.loop_utils import KeyboardResetController, PerfMonitor, RateLimiter


_HEAD_MATERIAL_PROFILES = {
    "stable": ((0.62, 0.62, 0.60), 0.58, 0.0, 1.0),
    "paint_matte": ((0.58, 0.58, 0.56), 0.58, 0.0, 1.0),
    "paint_finish": ((0.68, 0.68, 0.65), 0.46, 0.0, 1.0),
    "steel_blued": ((0.12, 0.13, 0.14), 0.38, 0.12, 1.0),
    "glass": ((0.0, 0.0, 0.0), 0.18, 0.0, 1.0),
}


_HEAD_SUBSET_TO_PROFILE = {
    "Paint_Matte": "paint_matte",
    "Paint_Matte_Finish": "paint_finish",
    "Steel_Blued": "steel_blued",
    "Tinted_Glass_R02": "glass",
}


def _define_head_material(stage, material_path, profile_name):
    color, roughness, metallic, opacity = _HEAD_MATERIAL_PROFILES[profile_name]
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path.AppendPath("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _head_material_mode() -> str:
    mode = os.environ.get("UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE", "all").lower()
    if mode in {"stable", "all"}:
        return mode
    if mode in _HEAD_MATERIAL_PROFILES:
        return mode
    print(f"[WARN] Unknown Walker S2 head material mode '{mode}', falling back to stable.")
    return "stable"


def _fix_walker_s2_head_material(stage) -> None:
    """Repair Walker S2 head material bindings after the robot USD is instantiated."""
    fixed = False
    mode = _head_material_mode()
    robot_prims = [prim for prim in stage.Traverse() if prim.GetName() == "Robot"]
    for robot_prim in robot_prims:
        robot_path = robot_prim.GetPath()
        materials = {
            name: _define_head_material(stage, robot_path.AppendPath(f"Looks/Head_{name}"), name)
            for name in _HEAD_MATERIAL_PROFILES
        }

        head_meshes = [
            prim
            for prim in stage.Traverse()
            if prim.GetName() == "head_pitch_01" and str(prim.GetPath()).startswith(str(robot_path))
        ]
        for head_mesh in head_meshes:
            UsdGeom.Imageable(head_mesh).MakeVisible()
            UsdShade.MaterialBindingAPI(head_mesh).Bind(materials["stable"])
            for child in head_mesh.GetChildren():
                if child.GetTypeName() != "GeomSubset":
                    continue
                profile_name = _HEAD_SUBSET_TO_PROFILE.get(child.GetName(), "stable") if mode == "all" else mode
                UsdShade.MaterialBindingAPI(child).Bind(materials[profile_name])
            fixed = True
            print(f"[INFO] Repaired Walker S2 head material ({mode}): {head_mesh.GetPath()}")
    if not fixed:
        print("[WARN] Walker S2 head mesh head_pitch_01 was not found under any Robot prim.")


def _apply_scene_repairs(env) -> None:
    if "WalkerS2" not in (args_cli.task or ""):
        return
    _fix_walker_s2_head_material(env.sim.stage)


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    if not args_cli.load_only:
        env_cfg.use_teleop_device("tiangong_pro")
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    keyboard_reset = KeyboardResetController()
    rate_limiter = RateLimiter(args_cli.step_hz)
    perf_monitor = None if args_cli.load_only else (PerfMonitor() if args_cli.perf_stats else None)

    if args_cli.load_only:
        print("[INFO] Load-only mode: teleop controller and action preprocessing are disabled.")
        teleop_interface = None
    else:
        from ubt_sim.devices import TiangongProController

        teleop_interface = TiangongProController(env)
        teleop_interface.display_controls()

    env.reset()
    _apply_scene_repairs(env)
    if teleop_interface is not None:
        teleop_interface.reset()
    if args_cli.load_only:
        print("[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.")
    rate_limiter.update_from_env(env)
    print(f"[INFO] RateLimiter sleep_duration={rate_limiter.sleep_duration:.6f}s")

    interrupted = False

    def signal_handler(signum, frame):
        nonlocal interrupted
        interrupted = True
        print("\n[INFO] Ctrl+C detected. Cleaning up...")

    original_sigint_handler = signal.signal(signal.SIGINT, signal_handler)

    try:
        while simulation_app.is_running() and not interrupted:
            with torch.inference_mode():
                if args_cli.load_only:
                    if keyboard_reset.reset_requested:
                        print("[INFO] Resetting environment...")
                        env.sim.reset()
                        env.reset()
                        keyboard_reset.reset_requested = False
                    simulation_app.update()
                    rate_limiter.sleep(env)
                    continue

                if keyboard_reset.reset_requested or teleop_interface.reset_requested:
                    print("[INFO] Resetting environment...")
                    env.sim.reset()
                    env.reset()
                    teleop_interface.reset()
                    keyboard_reset.reset_requested = False

                if perf_monitor is not None:
                    t_0 = time.perf_counter()
                    actions = teleop_interface.advance()
                    t_1 = time.perf_counter()
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)
                    t_2 = time.perf_counter()
                else:
                    actions = teleop_interface.advance()
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)

                if actions is None:
                    env.render()
                else:
                    env.step(actions)

                if perf_monitor is not None:
                    t_3 = time.perf_counter()
                    perf_monitor.record(
                        (t_1 - t_0) * 1000,
                        (t_2 - t_1) * 1000,
                        (t_3 - t_2) * 1000,
                    )
                    perf_monitor.maybe_print()

                rate_limiter.sleep(env)

            if interrupted:
                break
    except Exception as e:
        import traceback

        print(f"\n[ERROR] {e}\n")
        traceback.print_exc()
    finally:
        signal.signal(signal.SIGINT, original_sigint_handler)
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
