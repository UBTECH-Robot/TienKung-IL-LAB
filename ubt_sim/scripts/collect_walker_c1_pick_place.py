# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Walker C1 scripted-motion data collection (M1: pipeline bring-up).

In-process Isaac Lab collector: builds the parlor task env, drives the robot
through a deterministic scripted right-arm waypoint sequence (no external ZMQ,
no ROS), and records per-frame observations + actions + head camera into an
HDF5 file using the same schema the Tienkung pick-place collector writes
(``puppet/*`` + ``action/*`` + ``camera_observations/color_images/camera_head``).
That HDF5 is then convertible to a LeRobot dataset with
``ubt_IL/scripts/convert/convert_to_lerobot.py`` (see configs/Walker_C1_26_1RGB.json).

M1 goal is to prove the driving -> recording -> conversion pipe end to end;
it does NOT yet grasp a real object (that is M2). The scripted motion is a
right-arm reach/close/lift/return sweep from the pre-grasp ready pose.

Run:
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/collect_walker_c1_pick_place.py \
    --headless --device cpu --enable_cameras --episodes 1
"""

import argparse
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Walker C1 scripted pick-place data collection.")
parser.add_argument("--task", type=str, default="UBTSim-WalkerC1-Parlor-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--episodes", type=int, default=1, help="Number of trajectories to record.")
parser.add_argument(
    "--record_every",
    type=int,
    default=3,
    help="Record one frame every N env.step() calls (camera refreshes ~30Hz).",
)
parser.add_argument(
    "--phase_steps",
    type=int,
    default=45,
    help="env.step() count per scripted motion phase.",
)
parser.add_argument(
    "--out_root",
    type=str,
    default="/ubt_sim/dataset/walker_c1",
    help="Directory that receives one <timestamp>/trajectory.hdf5 per episode.",
)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args_cli = parser.parse_args()

import sys

# Cameras are required for head RGB recording.
if not getattr(args_cli, "enable_cameras", False):
    args_cli.enable_cameras = True

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import cv2
import gymnasium as gym
import h5py
import numpy as np
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.devices.walker_c1.action_process import (
    reset_hold_targets,
    to_controller_data,
    to_ros_data,
)
from ubt_sim.devices.walker_c1.config import (
    WALKER_C1_LEFT_ARM_JOINTS,
    WALKER_C1_RIGHT_ARM_JOINTS,
)

# ── Pre-grasp ready pose (sim-side copy) ──
# Values mirror teleoperation/control/walker_c1/constants.py::TASK_RESET_BODY_POSE.
# Kept as a local copy because the Isaac (Py3.11) and ROS/teleop (Py3.10) sides
# must not cross-import (see ubt_sim/CLAUDE.md). Tune here and in that file together.
READY_WAIST = [0.0, 0.0, 0.0]
READY_HEAD = [0.0, 0.35]
READY_LEFT_ARM = [-0.152, 0.30, 0.135, -1.155, 0.124, -0.361, -0.006]
READY_RIGHT_ARM = [-0.291, -0.30, -0.136, -1.155, -0.124, -0.361, 0.194]
READY_LEFT_HAND = [0.0] * 6
READY_RIGHT_HAND = [0.0] * 6

# HDF5 buffer keys; order fixed for length-consistency checks.
_BUFFER_KEYS = (
    "arm_right", "hand_right", "arm_left", "hand_left",
    "action_arm_right", "action_arm_left", "action_hand_right", "action_hand_left",
    "img", "timestamp",
)

# Camera resolution comes from config/walker_c1/parlor.yaml (width, height) = 640x480.
PLACEHOLDER_IMG_SHAPE = (480, 640, 3)


def _lerp(a, b, t):
    return [(1.0 - t) * x + t * y for x, y in zip(a, b)]


def _build_command(cmd_state):
    """Compose the to_controller_data command dict from the current group state."""
    return {
        "waist": list(cmd_state["waist"]),
        "head": list(cmd_state["head"]),
        "left_arm": list(cmd_state["left_arm"]),
        "right_arm": list(cmd_state["right_arm"]),
        "left_hand": list(cmd_state["left_hand"]),
        "right_hand": list(cmd_state["right_hand"]),
    }


def _record_frame(env, cmd_state, buffers):
    """Append one obs/action/camera snapshot to the buffers."""
    status = to_ros_data(env)
    pos_map = dict(zip(status["joint_names"], status["joint_pos"]))

    obs_arm_right = [pos_map.get(j, 0.0) for j in WALKER_C1_RIGHT_ARM_JOINTS]
    obs_arm_left = [pos_map.get(j, 0.0) for j in WALKER_C1_LEFT_ARM_JOINTS]
    obs_hand_right = list(status["right_hand_sdk_pos"])
    obs_hand_left = list(status["left_hand_sdk_pos"])

    # Camera head RGB (fall back to zeros if not yet rendered).
    img = None
    if "camera" in env.scene.keys():
        try:
            out = env.scene["camera"].data.output
            rgb_tensor = out.get("rgb") if out is not None else None
            if rgb_tensor is not None and rgb_tensor.shape[0] > 0:
                img = rgb_tensor[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
        except Exception:
            img = None
    if img is None:
        img = np.zeros(PLACEHOLDER_IMG_SHAPE, dtype=np.uint8)

    snapshot = {
        "arm_right": obs_arm_right,
        "arm_left": obs_arm_left,
        "hand_right": obs_hand_right,
        "hand_left": obs_hand_left,
        "action_arm_right": list(cmd_state["right_arm"]),
        "action_arm_left": list(cmd_state["left_arm"]),
        "action_hand_right": list(cmd_state["right_hand"]),
        "action_hand_left": list(cmd_state["left_hand"]),
        "img": img,
        "timestamp": time.time(),
    }
    for k, v in snapshot.items():
        buffers[k].append(v)


def _run_phase(env, cmd_state, target_state, buffers, phase_steps, record_every):
    """Linearly interpolate the changed groups to target over phase_steps env.steps."""
    start = {k: list(v) for k, v in cmd_state.items()}
    for step in range(phase_steps):
        t = (step + 1) / float(phase_steps)
        for group in cmd_state:
            if group in target_state:
                cmd_state[group] = _lerp(start[group], target_state[group], t)
        action = to_controller_data(_build_command(cmd_state), env)
        env.step(action)
        if step % record_every == 0:
            _record_frame(env, cmd_state, buffers)


def _save_hdf5(buffers, out_root):
    length = len(buffers["arm_right"])
    if length == 0:
        print("[WARN] No frames recorded, skip saving.")
        return None

    lens = {k: len(buffers[k]) for k in _BUFFER_KEYS}
    if len(set(lens.values())) != 1:
        print(f"[ERROR] Buffer length mismatch, abort save: {lens}")
        return None

    ts = int(time.time())
    dir_name = os.path.join(out_root, str(ts))
    os.makedirs(dir_name, exist_ok=True)
    try:
        os.chmod(dir_name, 0o777)
    except PermissionError:
        pass
    filename = os.path.join(dir_name, "trajectory.hdf5")
    print(f"[INFO] Saving {length} frames to {filename} ...")

    with h5py.File(filename, "w") as f:
        f.create_dataset("puppet/arm_right_position_align/data", data=np.asarray(buffers["arm_right"], dtype=np.float32))
        f.create_dataset("puppet/end_effector_right_position_align/data", data=np.asarray(buffers["hand_right"], dtype=np.float32))
        f.create_dataset("puppet/arm_left_position_align/data", data=np.asarray(buffers["arm_left"], dtype=np.float32))
        f.create_dataset("puppet/end_effector_left_position_align/data", data=np.asarray(buffers["hand_left"], dtype=np.float32))
        f.create_dataset("action/arm_right_position_align/data", data=np.asarray(buffers["action_arm_right"], dtype=np.float32))
        f.create_dataset("action/arm_left_position_align/data", data=np.asarray(buffers["action_arm_left"], dtype=np.float32))
        f.create_dataset("action/end_effector_right_position_align/data", data=np.asarray(buffers["action_hand_right"], dtype=np.float32))
        f.create_dataset("action/end_effector_left_position_align/data", data=np.asarray(buffers["action_hand_left"], dtype=np.float32))
        f.create_dataset("observations/timestamp", data=np.asarray(buffers["timestamp"], dtype=np.float64))

        dt = h5py.special_dtype(vlen=np.dtype("uint8"))
        img_ds = f.create_dataset("camera_observations/color_images/camera_head", (length,), dtype=dt)
        for i, img_rgb in enumerate(buffers["img"]):
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            ok, enc = cv2.imencode(".jpg", img_bgr)
            if ok:
                img_ds[i] = enc.flatten()
            else:
                print(f"[ERROR] Failed to encode image {i}")

    try:
        os.chmod(filename, 0o666)
    except PermissionError:
        pass
    print(f"[INFO] Data saved: {length} frames.")
    return filename


def _collect_one_episode(env, phase_steps, record_every):
    env.reset()
    reset_hold_targets()

    cmd_state = {
        "waist": list(READY_WAIST),
        "head": list(READY_HEAD),
        "left_arm": list(READY_LEFT_ARM),
        "right_arm": list(READY_RIGHT_ARM),
        "left_hand": list(READY_LEFT_HAND),
        "right_hand": list(READY_RIGHT_HAND),
    }
    buffers = {k: [] for k in _BUFFER_KEYS}

    # Phase 0: settle into the pre-grasp ready pose.
    _run_phase(env, cmd_state, {k: list(v) for k, v in cmd_state.items()}, buffers, phase_steps, record_every)
    # Phase 1: reach down/forward with the right arm (deterministic sweep, no IK yet).
    reach = list(READY_RIGHT_ARM)
    reach[0] += 0.35   # shoulder_pitch forward
    reach[3] += 0.30   # elbow_pitch extend
    _run_phase(env, cmd_state, {"right_arm": reach}, buffers, phase_steps, record_every)
    # Phase 2: close the right hand.
    _run_phase(env, cmd_state, {"right_hand": [0.8] * 6}, buffers, phase_steps, record_every)
    # Phase 3: lift back up.
    _run_phase(env, cmd_state, {"right_arm": list(READY_RIGHT_ARM)}, buffers, phase_steps, record_every)
    # Phase 4: open the right hand (return to ready).
    _run_phase(env, cmd_state, {"right_hand": [0.0] * 6}, buffers, phase_steps, record_every)

    return buffers


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    saved = []
    for ep in range(args_cli.episodes):
        print(f"[INFO] === Episode {ep + 1}/{args_cli.episodes} ===")
        buffers = _collect_one_episode(env, args_cli.phase_steps, args_cli.record_every)
        path = _save_hdf5(buffers, args_cli.out_root)
        if path:
            saved.append(path)

    print(f"[INFO] Collected {len(saved)} trajectory file(s):")
    for p in saved:
        print(f"  {p}")

    # Isaac/AppLauncher can hang on teardown in minimal scripts; exit hard.
    os._exit(0)


if __name__ == "__main__":
    main()
