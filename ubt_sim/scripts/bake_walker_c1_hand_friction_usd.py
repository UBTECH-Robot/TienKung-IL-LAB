#!/usr/bin/env python3
"""Bake a high-friction physics material onto the Walker C1 finger/palm
collision prims.

Why: grip normal force is plentiful (4 N.m effort -> tens of N at the
fingertips) yet the apple still slides out of the closed cage during
lift/carry. The finger collision prims were exported from URDF with NO
physics material, i.e. PhysX default friction (~0.5 combined multiply with
the apple's 1.2 gives ~0.6 effective). This bakes static/dynamic friction
1.5 onto every hand collision prim.

Non-destructive: reads walker_c1_force_drive.usd, writes
walker_c1_force_drive_grip.usd. Point WALKER_C1_USD_PATH back to revert.

Run (pure pxr, no Isaac boot — fast):
  docker exec walker-c1-ubt-sim bash -c "U=/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311; \
    PYTHONPATH=\$U LD_LIBRARY_PATH=\$U/bin /isaac-sim/python.sh \
    /ubt_sim/scripts/bake_walker_c1_hand_friction_usd.py"
"""
from __future__ import annotations

import shutil

from pxr import Usd, UsdPhysics, UsdShade

SRC = "/ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd"
DST = "/ubt_sim/assets/robots/walker_c1/walker_c1_force_drive_grip.usd"

HAND_LINK_KEYS = (
    "_thumb_cmp_link", "_thumb_mpp_link", "_thumb_ip_link",
    "_index_mpp_link", "_index_ip_link",
    "_middle_mpp_link", "_middle_ip_link",
    "_ring_mpp_link", "_ring_ip_link",
    "_little_mpp_link", "_little_ip_link",
    "_palm_link",
)

STATIC_FRICTION = 1.5
DYNAMIC_FRICTION = 1.5


def main() -> None:
    shutil.copyfile(SRC, DST)
    stage = Usd.Stage.Open(DST)

    # One shared high-friction physics material.
    mat_path = "/PhysicsMaterials/HandGripMaterial"
    mat = UsdShade.Material.Define(stage, mat_path)
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys_mat.CreateStaticFrictionAttr().Set(STATIC_FRICTION)
    phys_mat.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION)
    phys_mat.CreateRestitutionAttr().Set(0.0)

    bound = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not any(key in path for key in HAND_LINK_KEYS):
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(mat, materialPurpose="physics")
        bound += 1
        print(f"[BIND] {path}")

    stage.GetRootLayer().Save()
    print(f"[OK] bound {bound} hand collision prims with friction "
          f"{STATIC_FRICTION}/{DYNAMIC_FRICTION} -> {DST}")


if __name__ == "__main__":
    main()
