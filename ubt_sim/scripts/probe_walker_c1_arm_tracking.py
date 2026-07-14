#!/usr/bin/env python3
"""Diagnose Walker C1 right-arm tracking under gravity (isolation).

Spawns WALKER_C1_CFG ALONE (no parlor scene, no table), commands the right arm
to a series of target poses, settles, and reports per-joint COMMANDED vs
ACHIEVED angle + applied torque. This localizes why the grasp test could not
reach/lift the object:
  - large per-joint error  -> actuators too weak / gravity droop (stiffness/effort)
  - small per-joint error  -> joints DO track; the hand just isn't where expected
    (a kinematics/placement issue, not a tracking one)

Run:
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/probe_walker_c1_arm_tracking.py --headless --device cpu --steps 120
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source"))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Walker C1 right-arm tracking probe.")
parser.add_argument("--steps", type=int, default=120, help="Settle steps per pose.")
parser.add_argument("--no_gravity", action="store_true", help="Disable gravity to isolate actuation from gravity load.")
parser.add_argument("--force_drive", action="store_true", help="Override all joint drives from acceleration to force type before sim start (fix test).")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import WALKER_C1_CFG, WALKER_C1_RIGHT_ARM_JOINTS


@configclass
class WalkerC1ArmProbeSceneCfg(InteractiveSceneCfg):
    robot = WALKER_C1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


# right arm order: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch,
#                   elbow_yaw, wrist_pitch, wrist_roll]
_TEST_POSES = {
    "ready":     [-0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194],
    "reach_fwd": [-0.500,  0.000,  0.000, -0.500,  0.000,  0.000, 0.000],
    "lift_up":   [-1.300,  0.000,  0.000, -1.000,  0.000,  0.000, 0.000],
    "arm_down":  [ 0.300,  0.000,  0.000, -1.400,  0.000,  0.000, 0.000],
}


def main() -> None:
    import torch

    grav = (0.0, 0.0, 0.0) if args_cli.no_gravity else (0.0, 0.0, -9.81)
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device, gravity=grav)
    sim = sim_utils.SimulationContext(sim_cfg)
    scene = InteractiveScene(WalkerC1ArmProbeSceneCfg(num_envs=1, env_spacing=3.0))

    if args_cli.force_drive:
        import omni.usd
        from pxr import Usd, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        n = 0
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if "/Robot/" not in str(prim.GetPath()):
                continue
            drive = UsdPhysics.DriveAPI(prim, "angular")
            t = drive.GetTypeAttr() if drive else None
            if t and t.Get() == "acceleration":
                t.Set("force")
                n += 1
        print(f"[FIX] overrode {n} joint drives: acceleration -> force")

    sim.reset()
    scene.reset()
    robot = scene["robot"]

    joint_names = list(robot.data.joint_names)
    body_names = list(robot.data.body_names)
    ra_idx = [joint_names.index(j) for j in WALKER_C1_RIGHT_ARM_JOINTS]
    palm_idx = body_names.index("R_palm_link") if "R_palm_link" in body_names else None

    print("\n================ WALKER C1 ARM TRACKING (isolation) ================")
    print(f"steps/pose = {args_cli.steps}   device = {args_cli.device}")

    # Sim-enforced joint limits (what the loaded USD/articulation actually clamps to).
    try:
        limits = robot.data.joint_pos_limits[0].detach().cpu().tolist()  # [num_joints, 2]
        print("--- sim-enforced right-arm joint limits (lower, upper) ---")
        for k, j in enumerate(WALKER_C1_RIGHT_ARM_JOINTS):
            lo, hi = limits[ra_idx[k]]
            print(f"  {j:<22} [{lo:+.3f}, {hi:+.3f}]")
    except Exception as e:
        print(f"[WARN] could not read joint_pos_limits: {e}")

    # What stiffness / damping / effort did the sim ACTUALLY apply (vs config)?
    try:
        js = robot.data.joint_stiffness[0].detach().cpu().tolist()
        jd = robot.data.joint_damping[0].detach().cpu().tolist()
        try:
            je = robot.data.joint_effort_limits[0].detach().cpu().tolist()
        except Exception:
            je = robot.actuators["right_arm"].effort_limit
            je = je[0].detach().cpu().tolist() if hasattr(je, "detach") else None
        print("--- sim-applied right-arm actuator params (stiffness / damping / effort) ---")
        for k, j in enumerate(WALKER_C1_RIGHT_ARM_JOINTS):
            idx = ra_idx[k]
            ev = je[idx] if isinstance(je, list) else "?"
            print(f"  {j:<22} k={js[idx]:>8.1f}  d={jd[idx]:>7.1f}  eff={ev}")
    except Exception as e:
        print(f"[WARN] could not read actuator params: {e}")

    # Right-arm link masses (a huge distal mass would explain gravity-dominated sag
    # that 80 N.m can't hold).
    try:
        masses = robot.root_physx_view.get_masses()[0].detach().cpu().tolist()
        print("--- right-arm link masses (kg) ---")
        total = 0.0
        for k, name in enumerate(body_names):
            if name.startswith("R_") and any(
                s in name for s in ("shoulder", "elbow", "wrist", "palm", "thumb", "index", "middle", "ring", "little")
            ):
                print(f"  {name:<24} {masses[k]:.4f}")
                total += masses[k]
        print(f"  -> right arm+hand total = {total:.3f} kg")
        all_masses = robot.root_physx_view.get_masses()[0].detach().cpu().tolist()
        print(f"  -> FULL articulation total mass = {sum(all_masses):.3f} kg over {len(all_masses)} bodies")
    except Exception as e:
        print(f"[WARN] could not read masses: {e}")

    # USD unit scale (a wrong metersPerUnit/kilogramsPerUnit inflates gravity torque).
    try:
        import omni.usd
        from pxr import UsdGeom, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
        print(f"[USD UNITS] metersPerUnit={mpu}  kilogramsPerUnit={kpu}")
    except Exception as e:
        print(f"[WARN] could not read USD units: {e}")

    # Per-link COM offset (body frame). A COM placed meters from the joint would
    # produce a huge gravity torque despite normal mass.
    try:
        coms = robot.root_physx_view.get_coms()[0].detach().cpu().tolist()  # [bodies, 7] pos+quat
        print("--- right-arm link COM offset (bodyframe x,y,z) + |offset| ---")
        for k, name in enumerate(body_names):
            if name.startswith("R_") and any(s in name for s in ("elbow", "wrist", "palm", "shoulder")):
                c = coms[k]
                mag = (c[0] ** 2 + c[1] ** 2 + c[2] ** 2) ** 0.5
                print(f"  {name:<24} ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})  |{mag:.3f}| m")
    except Exception as e:
        print(f"[WARN] could not read COMs: {e}")

    # Link inertias (diagonal). Inflated inertia -> sluggish joints despite low mass.
    try:
        inert = robot.root_physx_view.get_inertias()[0].detach().cpu().tolist()  # [bodies, 9]
        print("--- right-arm link inertia diag (Ixx, Iyy, Izz) ---")
        for k, name in enumerate(body_names):
            if name.startswith("R_") and any(s in name for s in ("elbow", "wrist", "palm", "shoulder")):
                row = inert[k]
                print(f"  {name:<24} ({row[0]:.5f}, {row[4]:.5f}, {row[8]:.5f})")
    except Exception as e:
        print(f"[WARN] could not read inertias: {e}")

    # ACTUAL USD PhysX drive params on the elbow joint (maxForce is what physically
    # caps the torque; Isaac Lab's effort_limit_sim is supposed to write it here).
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            name = prim.GetName()
            if name in ("R_elbow_pitch_joint", "R_shoulder_pitch_joint") and "/Robot/" in str(prim.GetPath()):
                drive = UsdPhysics.DriveAPI(prim, "angular")
                if drive:
                    mf = drive.GetMaxForceAttr().Get()
                    ds = drive.GetStiffnessAttr().Get()
                    dd = drive.GetDampingAttr().Get()
                    dt = drive.GetTypeAttr().Get()
                    print(f"[USD DRIVE] {name}: TYPE={dt}  maxForce={mf}  stiffness={ds}  damping={dd}")
    except Exception as e:
        print(f"[WARN] could not read USD drive: {e}")

    for pose_name, ra_target in _TEST_POSES.items():
        target = robot.data.default_joint_pos.clone()
        for k, idx in enumerate(ra_idx):
            target[0, idx] = ra_target[k]
        for _ in range(max(args_cli.steps, 1)):
            robot.set_joint_position_target(target)
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())

        achieved = robot.data.joint_pos[0].detach().cpu().tolist()
        try:
            eff = robot.data.applied_torque[0].detach().cpu().tolist()
        except Exception:
            eff = [float("nan")] * len(joint_names)

        print(f"\n--- pose '{pose_name}' ---")
        print(f"{'joint':<22}{'cmd':>9}{'actual':>9}{'err':>9}{'torque':>9}")
        max_err = 0.0
        max_err_joint = None
        for k, j in enumerate(WALKER_C1_RIGHT_ARM_JOINTS):
            idx = ra_idx[k]
            cmd = ra_target[k]
            act = achieved[idx]
            err = act - cmd
            if abs(err) > abs(max_err):
                max_err, max_err_joint = err, j
            print(f"{j:<22}{cmd:>9.3f}{act:>9.3f}{err:>9.3f}{eff[idx]:>9.2f}")
        if palm_idx is not None:
            palm = robot.data.body_pos_w[0, palm_idx].detach().cpu().tolist()
            print(f"  R_palm_link world = [{palm[0]:.3f}, {palm[1]:.3f}, {palm[2]:.3f}]")
        print(f"  max |err| = {abs(max_err):.3f} rad at {max_err_joint}")

    print("===================================================================\n")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
