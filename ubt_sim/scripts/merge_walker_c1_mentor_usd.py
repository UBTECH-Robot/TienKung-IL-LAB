#!/usr/bin/env python3
"""Merge the proven dexterous hands into the mentor C1 sensor USD.

The mentor asset contains the same 31 body joints and 40 body rigid links as
the current C1 asset, plus calibrated cameras and IMUs, but omits both hands.
This script preserves the mentor body and sensor opinions and copies only the
missing hand links/joints plus the proven hand friction material.

Run inside the Isaac Sim container with the bundled pxr libraries. The helper
script ``start_c1_mentor_sensor_sim.sh`` does this automatically when needed.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdPhysics


ASSET_ROOT = Path("/ubt_sim/assets/robots/walker_c1")
MENTOR_DIR = ASSET_ROOT / "Collected_walker_c1_v1_sensorKpkd" / "Collected_walker_astron_v1_sensorKpkd"
DEFAULT_MENTOR_USD = MENTOR_DIR / "walker_astron_v1_sensorKpkd.usd"
DEFAULT_HAND_USD = ASSET_ROOT / "walker_c1_force_drive_grip.usd"
DEFAULT_OUTPUT_USD = MENTOR_DIR / "walker_astron_v1_sensorKpkd_hands.usd"

LEFT_FISHEYE_CAMERA = (
    "/walker_astron_v1/head_pitch_link/head_fisheye_left/"
    "head_fisheye_left_Camera"
)
GRIP_MATERIAL_SOURCE = Sdf.Path("/PhysicsMaterials/HandGripMaterial")
GRIP_MATERIAL_PARENT = Sdf.Path("/walker_astron_v1/PhysicsMaterials")
GRIP_MATERIAL_TARGET = GRIP_MATERIAL_PARENT.AppendChild("HandGripMaterial")
HAND_PROTOTYPE_ROOT = Sdf.Path("/C1HandPrototypes")
HAND_LOOKS_ROOT = Sdf.Path("/walker_astron_v1/Looks")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mentor-usd", type=Path, default=DEFAULT_MENTOR_USD)
    parser.add_argument("--hand-usd", type=Path, default=DEFAULT_HAND_USD)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_USD)
    return parser.parse_args()


def _joint_prims(stage: Usd.Stage) -> dict[str, Usd.Prim]:
    return {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.Joint)
    }


def _rigid_body_prims(stage: Usd.Stage) -> dict[str, Usd.Prim]:
    return {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }


def _copy_prim(source: Sdf.Layer, target: Sdf.Layer, path: Sdf.Path) -> None:
    if not source.GetPrimAtPath(path):
        raise RuntimeError(f"source prim is missing: {path}")
    if not Sdf.CopySpec(source, path, target, path):
        raise RuntimeError(f"failed to copy prim: {path}")


def _source_references(
    stage: Usd.Stage, layer: Sdf.Layer, root: Usd.Prim
) -> list[tuple[Sdf.Path, Sdf.Reference]]:
    references: list[tuple[Sdf.Path, Sdf.Reference]] = []
    for prim in Usd.PrimRange(root):
        spec = layer.GetPrimAtPath(prim.GetPath())
        if not spec or not spec.HasInfo("references"):
            continue
        reference_list = spec.GetInfo("references")
        for reference in reference_list.GetAppliedItems():
            if reference.assetPath:
                raise RuntimeError(
                    f"unexpected external hand prototype reference at {prim.GetPath()}: "
                    f"{reference.assetPath}"
                )
            references.append((prim.GetPath(), reference))
    return references


def _remap_hand_prototypes(
    hand_stage: Usd.Stage,
    hand_layer: Sdf.Layer,
    target_layer: Sdf.Layer,
    body_prims: list[Usd.Prim],
) -> dict[Sdf.Path, Sdf.Path]:
    references = [
        item
        for body_prim in body_prims
        for item in _source_references(hand_stage, hand_layer, body_prim)
    ]
    source_paths = sorted({reference.primPath for _, reference in references})
    prototype_root = Sdf.CreatePrimInLayer(target_layer, HAND_PROTOTYPE_ROOT)
    prototype_root.specifier = Sdf.SpecifierDef
    prototype_root.typeName = "Scope"

    path_map = {
        source_path: HAND_PROTOTYPE_ROOT.AppendChild(source_path.name)
        for source_path in source_paths
    }
    for source_path, target_path in path_map.items():
        if not Sdf.CopySpec(hand_layer, source_path, target_layer, target_path):
            raise RuntimeError(
                f"failed to copy hand prototype {source_path} -> {target_path}"
            )

    for prim_path, reference in references:
        target_spec = target_layer.GetPrimAtPath(prim_path)
        if not target_spec:
            raise RuntimeError(f"copied hand prim spec is missing: {prim_path}")
        remapped = Sdf.ReferenceListOp()
        remapped.explicitItems = [
            Sdf.Reference(
                reference.assetPath,
                path_map[reference.primPath],
                reference.layerOffset,
                reference.customData,
            )
        ]
        target_spec.SetInfo("references", remapped)
    return path_map


def _remap_hand_visual_materials(
    hand_stage: Usd.Stage,
    hand_layer: Sdf.Layer,
    target_layer: Sdf.Layer,
    prototype_path_map: dict[Sdf.Path, Sdf.Path],
) -> dict[Sdf.Path, Sdf.Path]:
    bindings: list[tuple[Sdf.Path, list[Sdf.Path]]] = []
    for source_prototype, target_prototype in prototype_path_map.items():
        source_prim = hand_stage.GetPrimAtPath(source_prototype)
        if not source_prim:
            raise RuntimeError(f"source hand prototype is missing: {source_prototype}")
        for prim in Usd.PrimRange.AllPrims(source_prim):
            relationship = prim.GetRelationship("material:binding")
            targets = relationship.GetTargets() if relationship else []
            if not targets:
                continue
            relative_path = prim.GetPath().MakeRelativePath(source_prototype)
            bindings.append((target_prototype.AppendPath(relative_path), targets))

    source_materials = sorted({target for _, targets in bindings for target in targets})
    if not source_materials:
        raise RuntimeError("no hand visual material bindings were found in source prototypes")
    material_path_map = {
        source_path: HAND_LOOKS_ROOT.AppendChild(f"C1Hand_{source_path.name}")
        for source_path in source_materials
    }
    for source_path, target_path in material_path_map.items():
        if not hand_layer.GetPrimAtPath(source_path):
            raise RuntimeError(f"source hand visual material is missing: {source_path}")
        if not Sdf.CopySpec(hand_layer, source_path, target_layer, target_path):
            raise RuntimeError(
                f"failed to copy hand visual material {source_path} -> {target_path}"
            )

    for target_prim_path, source_targets in bindings:
        binding_path = target_prim_path.AppendProperty("material:binding")
        binding = target_layer.GetPropertyAtPath(binding_path)
        if not binding:
            raise RuntimeError(f"copied hand material binding is missing: {binding_path}")
        binding.targetPathList.explicitItems = [
            material_path_map[source_target] for source_target in source_targets
        ]
    return material_path_map


def _bind_hand_collision_material(
    stage: Usd.Stage, body_paths: list[Sdf.Path]
) -> int:
    material_path = GRIP_MATERIAL_TARGET
    if not stage.GetPrimAtPath(material_path):
        raise RuntimeError("HandGripMaterial is missing before prototype binding")
    layer = stage.GetRootLayer()
    bound = 0
    for body_path in body_paths:
        collision_path = body_path.AppendChild("collisions")
        collision_prim = stage.GetPrimAtPath(collision_path)
        spec = layer.GetPrimAtPath(collision_path)
        if not collision_prim or not spec:
            raise RuntimeError(f"hand collision instance is missing: {collision_path}")
        schemas = Sdf.TokenListOp()
        existing_schemas = (
            spec.GetInfo("apiSchemas").GetAppliedItems()
            if spec.HasInfo("apiSchemas")
            else []
        )
        schemas.explicitItems = sorted(set(existing_schemas) | {"MaterialBindingAPI"})
        spec.SetInfo("apiSchemas", schemas)
        binding = Sdf.RelationshipSpec(
            spec, "material:binding:physics", custom=False
        )
        binding.targetPathList.explicitItems = [material_path]
        bound += 1
    return bound


def _validate(stage: Usd.Stage) -> None:
    joints = _joint_prims(stage)
    rigid_bodies = _rigid_body_prims(stage)
    revolute_count = sum(
        prim.IsA(UsdPhysics.RevoluteJoint) for prim in joints.values()
    )
    fixed_count = sum(
        prim.IsA(UsdPhysics.FixedJoint) for prim in joints.values()
    )
    required = {
        "L_palm_link",
        "R_palm_link",
        "L_thumb_cmp_joint",
        "R_thumb_cmp_joint",
        "L_little_ip_joint",
        "R_little_ip_joint",
    }
    available = set(joints) | set(rigid_bodies)
    missing = sorted(required - available)
    if missing:
        raise RuntimeError(f"merged USD is missing hand prims: {missing}")
    if (revolute_count, fixed_count, len(rigid_bodies)) != (53, 17, 70):
        raise RuntimeError(
            "unexpected merged structure: "
            f"revolute={revolute_count}, fixed={fixed_count}, "
            f"rigid_bodies={len(rigid_bodies)}"
        )
    for side in ("L", "R"):
        palm_path = Sdf.Path(f"/walker_astron_v1/{side}_palm_link")
        visuals = stage.GetPrimAtPath(palm_path.AppendChild("visuals"))
        collisions = stage.GetPrimAtPath(palm_path.AppendChild("collisions"))
        visual_prototype = visuals.GetPrototype() if visuals else Usd.Prim()
        collision_prototype = collisions.GetPrototype() if collisions else Usd.Prim()
        has_mesh = bool(visual_prototype) and any(
            prim.GetTypeName() == "Mesh" for prim in Usd.PrimRange(visual_prototype)
        )
        has_collision = bool(collision_prototype) and any(
            prim.HasAPI(UsdPhysics.CollisionAPI)
            for prim in Usd.PrimRange(collision_prototype)
        )
        collision_material_bound = bool(
            collisions.GetRelationship("material:binding:physics").GetTargets()
        ) if collisions else False
        visual_stack_paths = {
            str(spec.path) for spec in visuals.GetPrimStack()
        } if visuals else set()
        remapped_visual = any(
            path.startswith(str(HAND_PROTOTYPE_ROOT)) for path in visual_stack_paths
        )
        if not has_mesh or not has_collision or not collision_material_bound or not remapped_visual:
            raise RuntimeError(
                f"{side} palm instance is incomplete: "
                f"visual_mesh={has_mesh}, collision={has_collision}, "
                f"physics_material={collision_material_bound}, "
                f"remapped_visual={remapped_visual}"
            )
    if not stage.GetPrimAtPath(GRIP_MATERIAL_TARGET):
        raise RuntimeError("merged USD is missing HandGripMaterial")
    prototype_root = stage.GetPrimAtPath(HAND_PROTOTYPE_ROOT)
    visual_material_targets = {
        target
        for prototype in prototype_root.GetAllChildren()
        for prim in Usd.PrimRange.AllPrims(prototype)
        for target in prim.GetRelationship("material:binding").GetTargets()
    }
    missing_visual_materials = sorted(
        str(path) for path in visual_material_targets if not stage.GetPrimAtPath(path)
    )
    non_hand_materials = sorted(
        str(path)
        for path in visual_material_targets
        if not path.name.startswith("C1Hand_")
    )
    if not visual_material_targets or missing_visual_materials or non_hand_materials:
        raise RuntimeError(
            "hand visual material remap is incomplete: "
            f"missing={missing_visual_materials}, non_hand={non_hand_materials}"
        )


def main() -> None:
    args = _parse_args()
    mentor_usd = args.mentor_usd.expanduser().resolve()
    hand_usd = args.hand_usd.expanduser().resolve()
    output_usd = args.output.expanduser().resolve()
    for path in (mentor_usd, hand_usd):
        if not path.is_file():
            raise FileNotFoundError(path)

    output_usd.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(mentor_usd, output_usd)

    mentor_stage = Usd.Stage.Open(str(output_usd))
    hand_stage = Usd.Stage.Open(str(hand_usd))
    if mentor_stage is None or hand_stage is None:
        raise RuntimeError("failed to open source or destination USD")

    mentor_joints = _joint_prims(mentor_stage)
    hand_joints = _joint_prims(hand_stage)
    mentor_bodies = _rigid_body_prims(mentor_stage)
    hand_bodies = _rigid_body_prims(hand_stage)
    target_layer = mentor_stage.GetRootLayer()
    hand_layer = hand_stage.GetRootLayer()

    body_names = sorted(set(hand_bodies) - set(mentor_bodies))
    joint_names = sorted(set(hand_joints) - set(mentor_joints))
    copied_body_prims = [hand_bodies[name] for name in body_names]
    for body_prim in copied_body_prims:
        _copy_prim(hand_layer, target_layer, body_prim.GetPath())
    for name in joint_names:
        _copy_prim(hand_layer, target_layer, hand_joints[name].GetPath())

    # The hand link's visuals/collisions are instanceable prims that reference
    # flattened prototypes at the layer root. Both assets use the same numeric
    # prototype names for unrelated meshes, so copying them in place corrupts
    # the mentor body. Copy only hand-referenced prototypes under a unique root
    # and rewrite every copied hand instance reference.
    prototype_path_map = _remap_hand_prototypes(
        hand_stage, hand_layer, target_layer, copied_body_prims
    )
    visual_material_path_map = _remap_hand_visual_materials(
        hand_stage, hand_layer, target_layer, prototype_path_map
    )
    material_parent = Sdf.CreatePrimInLayer(target_layer, GRIP_MATERIAL_PARENT)
    material_parent.specifier = Sdf.SpecifierDef
    material_parent.typeName = "Scope"
    if not Sdf.CopySpec(
        hand_layer,
        GRIP_MATERIAL_SOURCE,
        target_layer,
        GRIP_MATERIAL_TARGET,
    ):
        raise RuntimeError(
            f"failed to copy grip material {GRIP_MATERIAL_SOURCE} -> "
            f"{GRIP_MATERIAL_TARGET}"
        )

    # The collected main layer overrides the left-fisheye sub-USD with an
    # accidental 0.646 m local translation. The sub-USD and right camera both
    # author zero here; clearing it restores the symmetric +/-71.4 mm mount.
    left_fisheye = mentor_stage.GetPrimAtPath(LEFT_FISHEYE_CAMERA)
    if not left_fisheye:
        raise RuntimeError(f"left fisheye camera is missing: {LEFT_FISHEYE_CAMERA}")
    left_fisheye.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

    target_layer.Save()
    mentor_stage.Reload()
    bound_collision_count = _bind_hand_collision_material(
        mentor_stage, [prim.GetPath() for prim in copied_body_prims]
    )
    mentor_stage.GetRootLayer().Save()
    merged_stage = Usd.Stage.Open(str(output_usd))
    if merged_stage is None:
        raise RuntimeError(f"failed to reopen merged USD: {output_usd}")
    _validate(merged_stage)

    print(f"[OK] merged mentor body/sensors with current dexterous hands: {output_usd}")
    print(f"[OK] copied {len(body_names)} rigid bodies and {len(joint_names)} joints")
    print(
        f"[OK] remapped {len(prototype_path_map)} hand instance prototypes under "
        f"{HAND_PROTOTYPE_ROOT}"
    )
    print(f"[OK] copied and remapped {len(visual_material_path_map)} hand visual materials")
    print(f"[OK] bound {bound_collision_count} hand collision instances to HandGripMaterial")
    print("[OK] structure: 53 revolute joints, 17 fixed joints, 70 rigid bodies")
    print("[OK] corrected the collected left-fisheye local translation to zero")


if __name__ == "__main__":
    main()
