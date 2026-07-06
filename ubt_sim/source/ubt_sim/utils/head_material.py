"""Walker S2 head material repair utilities.

Fixes material bindings on Walker S2 head mesh after the robot USD is instantiated. The
head geometry is split into GeomSubsets that need explicit material assignments per subset.
"""

import os

from pxr import Gf, Sdf, UsdGeom, UsdShade

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


def head_material_mode() -> str:
    """Return the active head material mode from the environment.

    Controlled via ``UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE``. Valid values:
    ``"all"`` (each GeomSubset gets its own material), ``"stable"`` (uniform grey),
    or one of the named profiles: ``"paint_matte"``, ``"paint_finish"``,
    ``"steel_blued"``, ``"glass"``.
    """
    mode = os.environ.get("UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE", "all").lower()
    if mode in {"stable", "all"}:
        return mode
    if mode in _HEAD_MATERIAL_PROFILES:
        return mode
    print(f"[WARN] Unknown Walker S2 head material mode '{mode}', falling back to stable.")
    return "stable"


def fix_walker_s2_head_material(stage) -> None:
    """Repair Walker S2 head material bindings after the robot USD is instantiated.

    Call once after ``env.reset()`` to ensure the head mesh and its GeomSubsets
    have valid material bindings.
    """
    fixed = False
    mode = head_material_mode()
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
