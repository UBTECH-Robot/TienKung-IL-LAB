# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Walker C1 scripted pick-place data collection (M2: real physics grasp).

In-process Isaac Lab collector: builds the parlor task env and runs a full
pick-place episode with real physics (no attach tricks, gravity on):
hover over the apple -> descend -> close hand -> lift -> carry over the plate
-> release -> return to the ready pose. The right arm is driven by per-step
damped-least-squares IK that servos the hand grasp center (thumb/index/middle
midpoint) to Cartesian targets; everything goes through the real controller
action path (``to_controller_data`` -> env.step).

Every frame is recorded (obs + action + head camera) into an HDF5 with the
same schema the Tienkung pick-place collector writes (``puppet/*`` +
``action/*`` + ``camera_observations/color_images/camera_head``), convertible
to LeRobot with ``ubt_IL/scripts/convert/convert_to_lerobot.py``
(configs/Walker_C1_26_1RGB.json). An episode is saved ONLY if it succeeds:
the apple ends up inside the plate radius at plate height (--save_on_failure
overrides, for debugging).

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
    "--out_root",
    type=str,
    default="/ubt_sim/dataset/walker_c1",
    help="Directory that receives one <timestamp>/trajectory.hdf5 per episode.",
)
parser.add_argument(
    "--save_on_failure",
    action="store_true",
    help="Also save episodes whose success check failed (debugging).",
)
parser.add_argument(
    "--randomize",
    action="store_true",
    help="Randomize the apple start position on the tabletop each episode.",
)
parser.add_argument(
    "--debug_watch",
    action="store_true",
    help="Print apple + grasp-center positions every 10 steps (diagnosis).",
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
from ubt_sim.task.walker_c1_parlor.walker_c1_parlor_env_cfg import (
    _GRASP_OBJECT_INIT_POS,
    _GRASP_OBJECT_RADIUS,
    _PLATE_HEIGHT,
    _PLATE_POS,
    _PLATE_RADIUS,
)

# ── Pre-grasp ready pose (sim-side copy) ──
# Values mirror teleoperation/control/walker_c1/constants.py::TASK_RESET_BODY_POSE.
# Kept as a local copy because the Isaac (Py3.11) and ROS/teleop (Py3.10) sides
# must not cross-import (see ubt_sim/CLAUDE.md). Tune here and in that file together.
READY_WAIST = [0.0, 0.0, 0.0]
READY_HEAD = [0.0, 0.50]
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


_WATCH = {"env": None, "ctx": None, "on": False}


def _watch(env, label, step):
    if not _WATCH["on"]:
        return
    if step % 5 != 0:
        return
    apple_t = env.scene["object"].data.root_pos_w[0]
    vel = env.scene["object"].data.root_lin_vel_w[0].detach().cpu().tolist()
    speed = sum(v * v for v in vel) ** 0.5
    apple = apple_t.detach().cpu().tolist()
    robot = _WATCH["ctx"]["robot"]
    dists = (robot.data.body_pos_w[0] - apple_t.unsqueeze(0)).norm(dim=-1)
    near_i = int(dists.argmin())
    near_name = robot.data.body_names[near_i]
    print(
        f"[watch:{label}:{step:03d}] apple=({apple[0]:.3f},{apple[1]:.3f},{apple[2]:.3f}) "
        f"speed={speed:.3f} nearest={near_name} d={float(dists[near_i]):.3f}"
    )


def _run_phase(env, cmd_state, target_state, buffers, phase_steps, record_every, label="lerp"):
    """Linearly interpolate the changed groups to target over phase_steps env.steps."""
    start = {k: list(v) for k, v in cmd_state.items()}
    for step in range(phase_steps):
        t = (step + 1) / float(phase_steps)
        for group in cmd_state:
            if group in target_state:
                cmd_state[group] = _lerp(start[group], target_state[group], t)
        action = to_controller_data(_build_command(cmd_state), env)
        env.step(action)
        _watch(env, label, step)
        if step % record_every == 0:
            _record_frame(env, cmd_state, buffers)


# ── Cartesian IK reach (right arm) ──
# The servo point is the hand "grasp center": midpoint of the thumb/index/middle
# links, i.e. the pocket the apple should sit in when the hand closes.
_GRASP_CENTER_LINKS = ("R_thumb_mpp_link", "R_index_ip_link", "R_middle_ip_link")
_IK_DAMPING = 0.1        # DLS lambda (m)
_IK_MAX_DQ = 0.010       # rad per env.step (100 Hz -> max 1.0 rad/s per joint)
_IK_MAX_LEAD = 0.20      # rad: max command lead over measured (anti-windup)
_IK_DONE_TOL = 0.012     # m: early-exit tolerance for an IK phase


def _make_ik_ctx(env):
    robot = env.scene["robot"]
    arm_ids, _ = robot.find_joints(list(WALKER_C1_RIGHT_ARM_JOINTS), preserve_order=True)
    palm_ids, _ = robot.find_bodies(["R_palm_link"])
    body_names = list(robot.data.body_names)
    grasp_ids = [body_names.index(name) for name in _GRASP_CENTER_LINKS]
    return {"robot": robot, "arm_ids": list(arm_ids), "palm_id": int(palm_ids[0]), "grasp_ids": grasp_ids}


def _grasp_center(ctx):
    return ctx["robot"].data.body_pos_w[0, ctx["grasp_ids"]].mean(dim=0)


def _mouth_center(ctx):
    """Live center of the palm-down mouth: midpoint between the finger-tip
    wall and the thumb tip, computed from actual link poses each step."""
    body_names = ctx["robot"].data.body_names
    pos = ctx["robot"].data.body_pos_w[0]
    fingers = [body_names.index(n) for n in
               ("R_index_ip_link", "R_middle_ip_link", "R_ring_ip_link", "R_little_ip_link")]
    thumb = body_names.index("R_thumb_ip_link")
    return 0.5 * (pos[fingers].mean(dim=0) + pos[thumb])


def _object_pos(env):
    return env.scene["object"].data.root_pos_w[0].detach().cpu().tolist()


def _print_hand_map(env, ctx):
    """Dump right-hand link positions relative to the grasp center (diagnosis)."""
    robot = ctx["robot"]
    gc = _grasp_center(ctx).cpu()
    apple = env.scene["object"].data.root_pos_w[0].cpu()
    print(f"[handmap] gc=({gc[0]:.3f},{gc[1]:.3f},{gc[2]:.3f}) apple_rel_gc="
          f"({apple[0]-gc[0]:+.3f},{apple[1]-gc[1]:+.3f},{apple[2]-gc[2]:+.3f})")
    for i, name in enumerate(robot.data.body_names):
        if name.startswith("R_") and ("palm" in name or "thumb" in name or "index" in name
                                      or "middle" in name or "ring" in name or "little" in name):
            p = robot.data.body_pos_w[0, i].cpu()
            print(f"[handmap]   {name:<22} rel_gc=({p[0]-gc[0]:+.3f},{p[1]-gc[1]:+.3f},{p[2]-gc[2]:+.3f})")


def _ik_arm_step(ctx, cmd_state, target_pos, max_dq=_IK_MAX_DQ, joint_subset=None, null_ref=None):
    """One damped-least-squares position IK step.

    The correction is integrated on the COMMAND (cmd_state["right_arm"]), not on
    the measured joint angles: with a position controller that sags under
    gravity, command = measured + dq never builds up enough lead to close the
    error. Anti-windup: the command may lead the measured position by at most
    _IK_MAX_LEAD per joint, and never leaves the soft joint limits.
    """
    robot = ctx["robot"]
    err = torch.as_tensor(target_pos, dtype=torch.float32) - _grasp_center(ctx).cpu()
    jac_full = robot.root_physx_view.get_jacobians()
    if jac_full.shape[1] == robot.num_bodies:  # floating base: root cols first
        jac = jac_full[0, ctx["palm_id"], 0:3, 6:]
    else:  # fixed base: jacobian rows exclude the root link
        jac = jac_full[0, ctx["palm_id"] - 1, 0:3, :]
    j = jac[:, ctx["arm_ids"]].cpu()  # 3 x 7
    if joint_subset is not None:
        # Freeze the non-subset joints (e.g. wrist during carry): zero their
        # jacobian columns so the IK solves position with the subset only and
        # their commands stay at the grasp-time values.
        mask = torch.zeros(j.shape[1])
        mask[list(joint_subset)] = 1.0
        j = j * mask.unsqueeze(0)
    q_now = torch.as_tensor(cmd_state["right_arm"], dtype=torch.float32)
    jjt = j @ j.T + (_IK_DAMPING**2) * torch.eye(3)
    dq = j.T @ torch.linalg.solve(jjt, err)
    if null_ref:
        # Null-space bias: pull the listed joints toward reference values
        # without disturbing the position task — keeps the grasp-time hand
        # orientation repeatable while retaining the joints for reach.
        jpinv = j.T @ torch.linalg.inv(jjt)
        n_proj = torch.eye(j.shape[1]) - jpinv @ j
        dq_null = torch.zeros(j.shape[1])
        for idx, ref in null_ref.items():
            dq_null[idx] = 0.1 * (float(ref) - float(q_now[idx]))
        dq = dq + n_proj @ dq_null
    dq = torch.clamp(dq, -max_dq, max_dq)
    q_cmd = q_now + dq
    q_meas = robot.data.joint_pos[0, ctx["arm_ids"]].cpu()
    q_cmd = torch.clamp(q_cmd, q_meas - _IK_MAX_LEAD, q_meas + _IK_MAX_LEAD)
    limits = robot.data.soft_joint_pos_limits[0, ctx["arm_ids"]].cpu()
    q_cmd = torch.clamp(q_cmd, limits[:, 0], limits[:, 1])
    return q_cmd.tolist(), float(err.norm())


def _run_ik_phase(env, ctx, cmd_state, target_pos, steps, buffers, record_every, label,
                  max_dq=_IK_MAX_DQ, joint_subset=None, servo_mouth_xy=False, null_ref=None,
                  done_tol=_IK_DONE_TOL):
    """Servo the grasp center to a world target while recording frames.

    With servo_mouth_xy, the xy error is measured at the live MOUTH center
    (finger wall / thumb midpoint) instead of the grasp center, closing the
    loop on the actual pocket-over-apple alignment; z still tracks the grasp
    center (the mouth center dives as the fingers close, so its z is not a
    stable height reference).
    """
    dist = float("inf")
    for step in range(steps):
        if servo_mouth_xy:
            gc = _grasp_center(ctx).cpu()
            mouth = _mouth_center(ctx).cpu()
            adj_target = [
                float(target_pos[0] + gc[0] - mouth[0]),
                float(target_pos[1] + gc[1] - mouth[1]),
                float(target_pos[2]),
            ]
            cmd_state["right_arm"], dist = _ik_arm_step(ctx, cmd_state, adj_target, max_dq, joint_subset, null_ref)
        else:
            cmd_state["right_arm"], dist = _ik_arm_step(ctx, cmd_state, target_pos, max_dq, joint_subset, null_ref)
        env.step(to_controller_data(_build_command(cmd_state), env))
        _watch(env, label, step)
        if step % record_every == 0:
            _record_frame(env, cmd_state, buffers)
        if dist < done_tol:
            break
    reached = _grasp_center(ctx).cpu().tolist()
    print(
        f"[phase:{label}] grasp_center=({reached[0]:.3f},{reached[1]:.3f},{reached[2]:.3f}) "
        f"target=({target_pos[0]:.3f},{target_pos[1]:.3f},{target_pos[2]:.3f}) err={dist:.3f} m"
    )
    return dist


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


def _randomize_object_start(env, rng):
    """Move the apple to a random reachable spot on the tabletop before starting."""
    obj = env.scene["object"]
    origin = env.scene.env_origins[0].detach().cpu().tolist()
    # Range centered on the reachable zone of the palm-down frozen-wrist arm:
    # +x (away from the robot) hits its reach boundary ~2cm past the default
    # spawn (descend stalls short), so the far side stays tight.
    x = _GRASP_OBJECT_INIT_POS[0] + rng.uniform(-0.03, 0.01)
    y = _GRASP_OBJECT_INIT_POS[1] + rng.uniform(-0.05, 0.01)
    pose = obj.data.default_root_state[:, :7].clone()
    pose[0, 0] = origin[0] + x
    pose[0, 1] = origin[1] + y
    pose[0, 2] = origin[2] + _GRASP_OBJECT_INIT_POS[2]
    obj.write_root_pose_to_sim(pose)
    obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=obj.data.default_root_state.device))


def _collect_one_episode(env, record_every, rng=None):
    env.reset()
    reset_hold_targets()
    if rng is not None:
        _randomize_object_start(env, rng)

    ctx = _make_ik_ctx(env)
    _WATCH["env"], _WATCH["ctx"], _WATCH["on"] = env, ctx, bool(args_cli.debug_watch)
    plate_top_z = _PLATE_POS[2] + _PLATE_HEIGHT / 2.0

    # Start commands at the (all-zero upper body) HOME pose the reset hold
    # targets anchor to, and RAMP to ready: commanding READY in one step makes
    # the arm snap over the table and sometimes swat the apple during settle.
    cmd_state = {
        "waist": [0.0] * len(READY_WAIST),
        "head": [0.0] * len(READY_HEAD),
        "left_arm": [0.0] * len(READY_LEFT_ARM),
        "right_arm": [0.0] * len(READY_RIGHT_ARM),
        "left_hand": list(READY_LEFT_HAND),
        "right_hand": list(READY_RIGHT_HAND),
    }
    ready_state = {
        "waist": list(READY_WAIST),
        "head": list(READY_HEAD),
        "left_arm": list(READY_LEFT_ARM),
        "right_arm": list(READY_RIGHT_ARM),
    }
    buffers = {k: [] for k in _BUFFER_KEYS}

    # Phase 0: ramp into the pre-grasp ready pose, then settle.
    _run_phase(env, cmd_state, ready_state, buffers, 100, record_every)
    _run_phase(env, cmd_state, {}, buffers, 40, record_every)
    apple0 = _object_pos(env)
    print(f"[phase:settle] apple=({apple0[0]:.3f},{apple0[1]:.3f},{apple0[2]:.3f})")

    # Phase 1: pre-shape the fingers into a half-open cup FIRST, then hover in
    # above the apple. The approach must not happen with an open hand: position
    # IK leaves orientation free, the wrist pitches during the approach, and an
    # extended fingertip sweeps through apple height and flicks it away
    # (verified: apple launched at 0.68 m/s while R_index_ip was the nearest
    # link at fingertip-contact distance). Curled fingers pull the tips up and
    # out of the approach path.
    # Palm-down pre-shape: ALL fingers slightly open — tucked fingers curl
    # under the palm and occupy the mouth cavity (the tucked little knuckle
    # was the lowest link, striking the apple first). Open fingers form the
    # cage fence around the apple; soft gains + closed-loop xy make fingertip
    # grazes harmless nudges.
    _run_phase(env, cmd_state, {"right_hand": [0.2] * 6}, buffers, 30, record_every, label="preshape")
    # Roll the wrist -90deg so the hand mouth faces DOWN (it naturally faces
    # sideways/+y, which cannot retain a ball through arm motion — measured
    # over many runs).
    roll_arm = list(cmd_state["right_arm"])
    roll_arm[6] = READY_RIGHT_ARM[6] - 1.57
    _run_phase(env, cmd_state, {"right_arm": roll_arm}, buffers, 60, record_every, label="roll")
    # Wrist (pitch/roll) stays frozen so the palm-down attitude holds.
    # elbow_yaw stays IN the IK for reach (shoulder3+elbow_pitch alone stalled
    # ~5cm short on randomized spawns), but is null-space-biased back to its
    # post-roll value so the grasp-time hand orientation stays repeatable
    # (free yaw drift rotated the cage and the close missed, also 0/3).
    hold_joints = (0, 1, 2, 3, 4)
    yaw_ref = {4: roll_arm[4]}

    # Grasp with verify-and-retry: a single blind cage attempt lands ~25%
    # (cm-level sensitivity on a ball); verifying the lift and re-trying
    # against the CURRENT apple position multiplies the episode success rate.
    # Approach path per attempt: align xy HIGH (22cm, fingertips cannot clip
    # the apple), drop vertically to hover, then descend to apple+0.04 (with
    # the ~1cm early-exit tolerance the gc settles at the verified
    # cage-around-equator height apple+0.05; deeper pins the fingertips
    # against the tabletop, higher closes above the apple). xy servos the
    # LIVE mouth center onto the apple (servo_mouth_xy), z tracks the gc.
    held = False
    for attempt in range(3):
        apple_now = _object_pos(env)
        if apple_now[2] < apple0[2] - 0.05:
            print(f"[check:grasp] apple fell off the table, aborting attempts")
            break
        gx, gy = apple_now[0], apple_now[1]
        approach = [gx, gy, apple_now[2] + 0.22]
        _run_ik_phase(env, ctx, cmd_state, approach, 200, buffers, record_every, "approach",
                      joint_subset=hold_joints, servo_mouth_xy=True, null_ref=yaw_ref)
        hover = [gx, gy, apple_now[2] + 0.12]
        _run_ik_phase(env, ctx, cmd_state, hover, 140, buffers, record_every, "hover",
                      max_dq=0.006, joint_subset=hold_joints, servo_mouth_xy=True, null_ref=yaw_ref)
        beside = [gx, gy, apple_now[2] + 0.04]
        _run_ik_phase(env, ctx, cmd_state, beside, 400, buffers, record_every, "descend",
                      max_dq=0.006, joint_subset=hold_joints, servo_mouth_xy=True, null_ref=yaw_ref,
                      done_tol=0.008)
        apple_d = _object_pos(env)
        print(f"[check:descend] apple=({apple_d[0]:.3f},{apple_d[1]:.3f},{apple_d[2]:.3f})")

        # Close decisively (staggering gives the apple time to escape), let
        # the squeeze settle, then verify with a lift.
        _run_phase(env, cmd_state, {"right_hand": [0.7, 0.9, 0.95, 0.95, 0.95, 0.95]}, buffers, 60, record_every, label="close")
        _run_phase(env, cmd_state, {}, buffers, 40, record_every, label="squeeze")
        lift = [gx, gy, apple_now[2] + 0.15]
        _run_ik_phase(env, ctx, cmd_state, lift, 200, buffers, record_every, "lift",
                      max_dq=0.005, joint_subset=hold_joints, null_ref=yaw_ref)
        apple_lifted = _object_pos(env)
        held = apple_lifted[2] > apple0[2] + 0.06
        print(f"[check:lift] attempt={attempt + 1} apple z {apple0[2]:.3f} -> "
              f"{apple_lifted[2]:.3f} ({'HELD' if held else 'NOT HELD'})")
        if held:
            break
        # Reopen to the pre-shape and try again at the apple's new position.
        _run_phase(env, cmd_state, {"right_hand": [0.2] * 6}, buffers, 30, record_every, label="reopen")

    release_z = plate_top_z + _GRASP_OBJECT_RADIUS + 0.01
    if held:
        # Carry the apple over the plate, lower to just above the plate
        # surface (dropping from height bounces it off), and release.
        carry = [_PLATE_POS[0], _PLATE_POS[1], release_z + 0.09]
        _run_ik_phase(env, ctx, cmd_state, carry, 240, buffers, record_every, "carry",
                      max_dq=0.006, joint_subset=hold_joints, null_ref=yaw_ref)
        lower = [_PLATE_POS[0], _PLATE_POS[1], release_z]
        _run_ik_phase(env, ctx, cmd_state, lower, 120, buffers, record_every, "lower",
                      max_dq=0.005, joint_subset=hold_joints, null_ref=yaw_ref)
        _run_phase(env, cmd_state, {"right_hand": [0.0] * 6}, buffers, 50, record_every)

    # Retreat up and return the arm to the ready pose, then settle.
    retreat = [_PLATE_POS[0] - 0.06, _PLATE_POS[1] - 0.06, release_z + 0.15]
    _run_ik_phase(env, ctx, cmd_state, retreat, 60, buffers, record_every, "retreat")
    _run_phase(env, cmd_state, {"right_hand": [0.0] * 6}, buffers, 20, record_every)
    _run_phase(env, cmd_state, {"right_arm": list(READY_RIGHT_ARM)}, buffers, 120, record_every)
    _run_phase(env, cmd_state, {}, buffers, 40, record_every)

    # Success: apple rests inside the plate disk at plate height.
    apple_end = _object_pos(env)
    horiz = ((apple_end[0] - _PLATE_POS[0]) ** 2 + (apple_end[1] - _PLATE_POS[1]) ** 2) ** 0.5
    z_ok = plate_top_z - 0.01 <= apple_end[2] <= plate_top_z + 2.5 * _GRASP_OBJECT_RADIUS
    success = horiz <= _PLATE_RADIUS and z_ok
    print(
        f"[check:place] apple=({apple_end[0]:.3f},{apple_end[1]:.3f},{apple_end[2]:.3f}) "
        f"plate_center_dist={horiz:.3f} (limit {_PLATE_RADIUS}) z_ok={z_ok} -> "
        f"{'SUCCESS' if success else 'FAILURE'}"
    )
    return buffers, success


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device("walker_c1")
    env_cfg.seed = int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # In GUI mode, open a second viewport showing the recorded head-camera
    # view (what actually goes into the dataset). No-op when headless.
    if not getattr(args_cli, "headless", False):
        try:
            from omni.kit.viewport.utility import create_viewport_window

            cam_path = "/World/envs/env_0/Robot/head_pitch_link/Camera_RGB"
            vp_window = create_viewport_window(
                "HeadCam (dataset view)", width=480, height=360, position_x=60, position_y=60
            )
            if hasattr(vp_window.viewport_api, "set_active_camera"):
                vp_window.viewport_api.set_active_camera(cam_path)
            else:
                vp_window.viewport_api.camera_path = cam_path
            print(f"[INFO] Head-camera viewport opened at {cam_path}")
        except Exception as exc:
            print(f"[WARN] Could not open the head-camera viewport: {exc}")

    import random

    rng = random.Random() if args_cli.randomize else None
    saved, failed = [], 0
    for ep in range(args_cli.episodes):
        print(f"[INFO] === Episode {ep + 1}/{args_cli.episodes} ===")
        buffers, success = _collect_one_episode(env, args_cli.record_every, rng)
        if success or args_cli.save_on_failure:
            path = _save_hdf5(buffers, args_cli.out_root)
            if path:
                saved.append(path)
        if not success:
            failed += 1
            print("[WARN] Episode failed the place check" + ("" if args_cli.save_on_failure else "; not saved."))

    print(f"[INFO] Collected {len(saved)} trajectory file(s) ({failed} failed episode(s)):")
    for p in saved:
        print(f"  {p}")

    # Isaac/AppLauncher can hang on teardown in minimal scripts; exit hard.
    os._exit(0)


if __name__ == "__main__":
    main()
