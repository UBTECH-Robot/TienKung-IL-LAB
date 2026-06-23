# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run Walker S2 simulation environments."""

import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import signal

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Walker S2 simulation environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerS2-PartSorting-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument("--perf_stats", action="store_true", help="Print performance statistics.")
parser.add_argument("--load_only", action="store_true", help="Load and render the environment without ROS control.")
parser.add_argument("--zmq_cmd_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_CMD_PORT", 5655)))
parser.add_argument("--zmq_status_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_STATUS_PORT", 5656)))
parser.add_argument("--zmq_image_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_IMAGE_PORT", 5657)))
parser.add_argument("--zmq_jpeg_image_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_JPEG_IMAGE_PORT", 5658)))
parser.add_argument(
    "--physics_device",
    type=str,
    default=os.environ.get("UBT_SIM_WALKER_S2_PHYSICS_DEVICE", "cpu"),
    help=(
        "Device for Isaac Lab physics tensors. Defaults to CPU while AppLauncher/rendering stays on cuda:0. "
        "This avoids Isaac Sim 5.0 / Isaac Lab 2.2 Walker S2 articulation startup spam: "
        "getVelocities expected device 0, received device -1."
    ),
)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device=os.environ.get("UBT_SIM_WALKER_S2_DEVICE", "cuda:0"))
args_cli = parser.parse_args()

import sys

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")
# Force PhysX tensor readback compatibility. Isaac Sim 5.0 / Isaac Lab 2.2 can
# create the articulation velocity buffer on CPU when GPU dynamics suppresses
# readback, which triggers getVelocities device mismatch spam on startup.
sys.argv.append("--/physics/suppressReadback=false")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import time

import gymnasium as gym
import torch
from pxr import Gf, Sdf, UsdGeom, UsdShade
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.devices.walker_s2 import WalkerS2Controller
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


def _apply_part_randomization_if_requested(env, teleop_interface) -> None:
    request = teleop_interface.pop_part_randomization_request()
    if request is None:
        return

    randomizer = getattr(env.cfg, "randomize_part_positions", None)
    if randomizer is None:
        print("[WARN] Current task does not support part randomization.")
        return

    try:
        result = randomizer(env, request)
        print(f"[INFO] Randomized Walker S2 part positions: {result}")
    except Exception as exc:
        print(f"[WARN] Failed to randomize Walker S2 part positions: {exc}")


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


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.physics_device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("walker_s2")
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    print(f"[INFO] Walker S2 render/app device args_cli.device={args_cli.device}")
    print(f"[INFO] Walker S2 physics device args_cli.physics_device={args_cli.physics_device}")
    print(f"[INFO] Walker S2 physics env.device={env.device}")
    print(f"[INFO] Walker S2 physics env.cfg.sim.device={env.cfg.sim.device}")

    keyboard_reset = KeyboardResetController()
    rate_limiter = RateLimiter(args_cli.step_hz)
    perf_monitor = None if args_cli.load_only else (PerfMonitor() if args_cli.perf_stats else None)

    if args_cli.load_only:
        print("[INFO] Walker S2 load-only mode: ROS control and action preprocessing are disabled.")
        teleop_interface = None
    else:
        teleop_interface = WalkerS2Controller(
            env,
            cmd_port=args_cli.zmq_cmd_port,
            status_port=args_cli.zmq_status_port,
            image_port=args_cli.zmq_image_port,
            jpeg_image_port=args_cli.zmq_jpeg_image_port,
        )
        teleop_interface.display_controls()

    env.reset()
    _fix_walker_s2_head_material(env.sim.stage)
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
                        _fix_walker_s2_head_material(env.sim.stage)
                        keyboard_reset.reset_requested = False
                    simulation_app.update()
                    rate_limiter.sleep(env)
                    continue

                if keyboard_reset.reset_requested or teleop_interface.reset_requested:
                    print("[INFO] Resetting environment...")
                    env.sim.reset()
                    env.reset()
                    _fix_walker_s2_head_material(env.sim.stage)
                    teleop_interface.reset()
                    keyboard_reset.reset_requested = False

                if perf_monitor is not None:
                    t_0 = time.perf_counter()
                    actions = teleop_interface.advance()
                    t_1 = time.perf_counter()
                    _apply_part_randomization_if_requested(env, teleop_interface)
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)
                    t_2 = time.perf_counter()
                else:
                    actions = teleop_interface.advance()
                    _apply_part_randomization_if_requested(env, teleop_interface)
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
