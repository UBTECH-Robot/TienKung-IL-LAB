#!/usr/bin/env python3
"""Inspect Walker C1 USD joint physics metadata.

Run inside the Isaac Sim container:

    /isaac-sim/python.sh -u /ubt_sim/scripts/inspect_walker_c1_usd.py
"""

from __future__ import annotations

import argparse
from pathlib import Path


RIGHT_HAND_JOINTS = [
    "R_thumb_cmp_joint",
    "R_thumb_mpp_joint",
    "R_thumb_ip_joint",
    "R_index_mpp_joint",
    "R_index_ip_joint",
    "R_middle_mpp_joint",
    "R_middle_ip_joint",
    "R_ring_mpp_joint",
    "R_ring_ip_joint",
    "R_little_mpp_joint",
    "R_little_ip_joint",
]

LEFT_HAND_JOINTS = [
    "L_thumb_cmp_joint",
    "L_thumb_mpp_joint",
    "L_thumb_ip_joint",
    "L_index_mpp_joint",
    "L_index_ip_joint",
    "L_middle_mpp_joint",
    "L_middle_ip_joint",
    "L_ring_mpp_joint",
    "L_ring_ip_joint",
    "L_little_mpp_joint",
    "L_little_ip_joint",
]

JOINTS_BY_GROUP = {
    "right_hand": RIGHT_HAND_JOINTS,
    "left_hand": LEFT_HAND_JOINTS,
}


def _value(attr):
    if not attr or not attr.HasAuthoredValueOpinion():
        return None
    return attr.Get()


def _rel_targets(rel):
    if not rel:
        return []
    return [str(path) for path in rel.GetTargets()]


def _find_prim_by_name(stage: Usd.Stage, name: str) -> Usd.Prim | None:
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    return None


def _api_names(prim: Usd.Prim) -> list[str]:
    return [str(name) for name in prim.GetAppliedSchemas()]


def _drive_info(prim: Usd.Prim) -> dict[str, object]:
    info = {}
    for drive_name in ["angular", "linear", "rotX", "rotY", "rotZ", "transX", "transY", "transZ"]:
        drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
        if not drive:
            continue
        data = {
            "target_position": _value(drive.GetTargetPositionAttr()),
            "target_velocity": _value(drive.GetTargetVelocityAttr()),
            "stiffness": _value(drive.GetStiffnessAttr()),
            "damping": _value(drive.GetDampingAttr()),
            "max_force": _value(drive.GetMaxForceAttr()),
            "type": _value(drive.GetTypeAttr()),
        }
        info[drive_name] = data
    return info


def _joint_info(prim: Usd.Prim) -> dict[str, object]:
    revolute = UsdPhysics.RevoluteJoint(prim)
    joint = UsdPhysics.Joint(prim)
    return {
        "path": str(prim.GetPath()),
        "type": prim.GetTypeName(),
        "apis": _api_names(prim),
        "axis": _value(revolute.GetAxisAttr()) if revolute else None,
        "lower_limit": _value(revolute.GetLowerLimitAttr()) if revolute else None,
        "upper_limit": _value(revolute.GetUpperLimitAttr()) if revolute else None,
        "body0": _rel_targets(joint.GetBody0Rel()) if joint else [],
        "body1": _rel_targets(joint.GetBody1Rel()) if joint else [],
        "drives": _drive_info(prim),
    }


def _mass_info(stage: Usd.Stage, path: str) -> dict[str, object]:
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return {}
    mass_api = UsdPhysics.MassAPI(prim)
    return {
        "path": path,
        "type": prim.GetTypeName(),
        "apis": _api_names(prim),
        "mass": _value(mass_api.GetMassAttr()),
        "center_of_mass": _value(mass_api.GetCenterOfMassAttr()),
        "diagonal_inertia": _value(mass_api.GetDiagonalInertiaAttr()),
        "principal_axes": _value(mass_api.GetPrincipalAxesAttr()),
    }


def _collision_prims_for_paths(stage: Usd.Stage, paths: list[str]) -> list[dict[str, object]]:
    seen = set()
    collisions = []
    for path in paths:
        if not path:
            continue
        root = stage.GetPrimAtPath(path)
        if not root:
            continue
        for prim in Usd.PrimRange(root):
            apis = _api_names(prim)
            is_collision = "PhysicsCollisionAPI" in apis or "PhysicsMeshCollisionAPI" in apis
            if not is_collision:
                continue
            prim_path = str(prim.GetPath())
            if prim_path in seen:
                continue
            seen.add(prim_path)
            collisions.append(
                {
                    "path": prim_path,
                    "type": prim.GetTypeName(),
                    "apis": apis,
                    "collision_enabled": _value(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()),
                }
            )
    return collisions


def _print_joint_report(stage: Usd.Stage, names: list[str]) -> None:
    for name in names:
        prim = _find_prim_by_name(stage, name)
        if prim is None:
            print(f"\n[MISSING] joint {name}")
            continue

        info = _joint_info(prim)
        print(f"\n[JOINT] {name}")
        print(f"  path: {info['path']}")
        print(f"  type: {info['type']}")
        print(f"  apis: {', '.join(info['apis']) if info['apis'] else '-'}")
        print(f"  axis: {info['axis']}")
        print(f"  limit: lower={info['lower_limit']} upper={info['upper_limit']}")
        print(f"  body0: {info['body0']}")
        print(f"  body1: {info['body1']}")
        for body_path in info["body0"] + info["body1"]:
            mass = _mass_info(stage, body_path)
            if mass:
                print(
                    "  body_mass: {path} mass={mass} com={center_of_mass} "
                    "diag_inertia={diagonal_inertia} apis={apis}".format(
                        path=mass["path"],
                        mass=mass["mass"],
                        center_of_mass=mass["center_of_mass"],
                        diagonal_inertia=mass["diagonal_inertia"],
                        apis=",".join(mass["apis"]),
                    )
                )

        drives = info["drives"]
        if not drives:
            print("  drives: -")
        for drive_name, drive in drives.items():
            print(
                "  drive:{name}: target={target_position} vel={target_velocity} "
                "stiffness={stiffness} damping={damping} max_force={max_force} type={type}".format(
                    name=drive_name,
                    **drive,
                )
            )

        collisions = _collision_prims_for_paths(stage, info["body0"] + info["body1"])
        print(f"  collision_prims_near_bodies: {len(collisions)}")
        for collision in collisions[:10]:
            print(
                "    - {path} type={type} enabled={enabled} apis={apis}".format(
                    path=collision["path"],
                    type=collision["type"],
                    enabled=collision["collision_enabled"],
                    apis=",".join(collision["apis"]),
                )
            )
        if len(collisions) > 10:
            print(f"    ... {len(collisions) - 10} more")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect Walker C1 hand joint metadata in USD.")
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path("/ubt_sim/assets/robots/walker_c1/walker_c1.usd"),
        help="Walker C1 USD path inside the container.",
    )
    parser.add_argument(
        "--group",
        choices=sorted(JOINTS_BY_GROUP),
        default="right_hand",
        help="Joint group to inspect.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    global Usd, UsdPhysics
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(args.usd))
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.usd}")

    print(f"[INFO] USD: {args.usd}")
    print(f"[INFO] defaultPrim: {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else '-'}")
    _print_joint_report(stage, JOINTS_BY_GROUP[args.group])
    app.close()


if __name__ == "__main__":
    main()
