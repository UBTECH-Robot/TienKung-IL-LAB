#!/usr/bin/env python3
"""Minimal C1 URDF import smoke test for Isaac Sim.

Run this inside an Isaac Sim container with:

    /isaac-sim/python.sh /ubt_sim/scripts/import_walker_c1_urdf_test.py --headless

The script only checks whether Isaac Sim can import the C1 URDF as an
articulation. It does not hook the robot into ubt_sim task/control code.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import C1 URDF into Isaac Sim and print basic articulation info.")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("/ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3_no_sixforce_mesh.urdf"),
        help="Path to the C1 URDF inside the Isaac container.",
    )
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without a GUI.")
    parser.add_argument("--merge-fixed-joints", action="store_true", help="Merge fixed joints during URDF import.")
    parser.add_argument("--floating-base", action="store_true", help="Do not fix the robot base.")
    parser.add_argument("--steps", type=int, default=5, help="Number of simulation app updates after import.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = SimulationApp({"headless": args.headless})

    import omni.kit.commands
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdLux, UsdPhysics

    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"C1 URDF not found: {urdf_path}")

    print(f"[INFO] Importing URDF: {urdf_path}")

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")

    import_config.merge_fixed_joints = args.merge_fixed_joints
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = not args.floating_base
    import_config.distance_scale = 1.0
    import_config.make_default_prim = True
    import_config.create_physics_scene = True

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError("URDFParseAndImportFile failed")

    print(f"[INFO] Imported articulation prim: {prim_path}")

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath("/physicsScene").IsValid():
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
        physx_scene_api.CreateEnableCCDAttr(True)
        physx_scene_api.CreateEnableStabilizationAttr(True)
    else:
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 5.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5))
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    light.CreateIntensityAttr(500)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(max(args.steps, 1)):
        app.update()

    articulation = Articulation(prim_path)
    articulation.initialize()

    if not articulation.is_physics_handle_valid():
        raise RuntimeError(f"Imported prim is not a valid articulation: {prim_path}")

    dof_names = list(articulation.dof_names)
    print(f"[OK] Articulation is valid: {prim_path}")
    print(f"[INFO] dof_count={len(dof_names)}")
    print("[INFO] first_dofs=")
    for name in dof_names[:20]:
        print(f"  {name}")
    if len(dof_names) > 20:
        print(f"  ... ({len(dof_names) - 20} more)")

    timeline.stop()
    app.close()


if __name__ == "__main__":
    main()
