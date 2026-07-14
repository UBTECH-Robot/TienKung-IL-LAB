#!/usr/bin/env python3
"""Bake a force-drive version of the Walker C1 robot USD.

Root cause (verified 2026-07-14): C1's USD joint drives are TYPE=acceleration.
In an acceleration drive PhysX scales the drive by the joint's (tiny) inertia,
so the effective torque stiffness = stiffness * inertia is near zero and the
upper body cannot hold poses under gravity (arm sag + finger droop). Converting
the drives to force type fixes it (probe_walker_c1_arm_tracking.py --force_drive
brought all arm errors from 0.4-1.1 rad down to ~0.02 rad).

This script ONLY changes each angular/linear drive's `type` attribute from
"acceleration" to "force". It does NOT touch stiffness / damping / maxForce
(Isaac Lab's actuator config overrides gains at runtime anyway). The source USD
is left untouched; the result is written to a NEW file so we can A/B and revert
by pointing WALKER_C1_USD_PATH back.

Run (no simulation needed; AppLauncher only to get the USD/pxr libs):
  docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
    /ubt_sim/scripts/bake_walker_c1_force_drive_usd.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Bake force-drive Walker C1 USD.")
parser.add_argument("--src", type=str, default="/ubt_sim/assets/robots/walker_c1/walker_c1.usd")
parser.add_argument("--dst", type=str, default="/ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd")
parser.add_argument("--dry_run", action="store_true", help="Only report what would change; do not write.")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics


def main():
    print(f"[INFO] opening source USD: {args_cli.src}")
    stage = Usd.Stage.Open(args_cli.src)
    if stage is None:
        raise RuntimeError(f"could not open {args_cli.src}")

    changed = 0
    scanned = 0
    for prim in stage.Traverse():
        for token in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI(prim, token)
            type_attr = drive.GetTypeAttr() if drive else None
            if not type_attr:
                continue
            scanned += 1
            cur = type_attr.Get()
            if cur == "acceleration":
                if not args_cli.dry_run:
                    type_attr.Set("force")
                changed += 1
                if changed <= 8:
                    print(f"  [{token}] {prim.GetPath()}: acceleration -> force")

    print(f"[INFO] drives scanned={scanned}  converted(acceleration->force)={changed}")

    if args_cli.dry_run:
        print("[INFO] dry-run: no file written.")
    else:
        stage.Export(args_cli.dst)
        try:
            os.chmod(args_cli.dst, 0o664)
        except PermissionError:
            pass
        print(f"[OK] wrote force-drive USD: {args_cli.dst}")

    os._exit(0)


if __name__ == "__main__":
    main()
