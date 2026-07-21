#!/usr/bin/env python3
"""绕指定轴旋转 USD 中关节（FixedJoint / RevoluteJoint 等）的安装角度。

通过在 joint prim 的 ``physics:localRot1`` 属性上叠加 quaternion 实现。
仅影响该关节及其所有下游 link（子 link、子关节…），即整个运动学链。

用法（在 Docker 容器内运行）：:

    /isaac-sim/python.sh scripts/rotate_usd_joint.py <usd_path> <joint_path> [options]

示例：:

    # 绕 X 轴逆时针旋转 90°
    /isaac-sim/python.sh scripts/rotate_usd_joint.py \\
        assets/robots/walker_s2/s2_v1.usd \\
        /s2_v1/joints/L_sixforce_joint \\
        --axis x --angle 90

    # 绕 Y 轴顺时针旋转 45°，并备份
    /isaac-sim/python.sh scripts/rotate_usd_joint.py \\
        assets/robots/walker_s2/s2_v1.usd \\
        /s2_v1/joints/R_wrist_roll_joint \\
        --axis y --angle -45 --backup

环境依赖：
    - Isaac Sim 容器内运行（USD 库路径自动探测）
    - 也可在主机上运行，需手动设置 USD_ROOT / LD_LIBRARY_PATH / PYTHONPATH
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path


# ============================================================================
# USD 库加载
# ============================================================================

def _find_usd_root() -> str:
    """自动探测 Isaac Sim 容器中的 USD 库路径。"""
    candidates = list(
        Path("/isaac-sim/extscache").glob("omni.usd.libs-*/pxr/Usd/__init__.py")
    )
    if candidates:
        return str(candidates[0].parent.parent.parent)
    # 尝试常见位置
    for guess in [
        "/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311",
    ]:
        if (Path(guess) / "pxr/Usd/__init__.py").exists():
            return guess
    raise RuntimeError("找不到 USD 库，请在 Isaac Sim 容器中运行此脚本")


def _setup_usd_env() -> None:
    usd_root = os.environ.get("USD_ROOT", "")
    if not usd_root:
        usd_root = _find_usd_root()
    bin_dir = Path(usd_root) / "bin"
    if str(bin_dir) not in os.environ.get("LD_LIBRARY_PATH", ""):
        os.environ["LD_LIBRARY_PATH"] = (
            f"{bin_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        )
    sys.path.insert(0, usd_root)


_setup_usd_env()
from pxr import Gf, Usd  # noqa: E402


# ============================================================================
# 核心逻辑
# ============================================================================

AXIS_VECTORS = {
    "x": Gf.Vec3d(1.0, 0.0, 0.0),
    "y": Gf.Vec3d(0.0, 1.0, 0.0),
    "z": Gf.Vec3d(0.0, 0.0, 1.0),
}


def rotate_quat(quat: Gf.Quatf, axis: str, angle_deg: float) -> Gf.Quatf:
    """在给定四元数上叠加绕指定轴的旋转（右乘）。

    Args:
        quat: 原始四元数（USD 格式：x, y, z, w）。
        axis: 旋转轴 ``"x"`` / ``"y"`` / ``"z"``。
        angle_deg: 旋转角度（度），正 → 逆时针，负 → 顺时针。

    Returns:
        ``quat * R_axis(angle_deg)``
    """
    axis_vec = AXIS_VECTORS[axis.lower()]
    rad = math.radians(angle_deg)
    half = rad / 2.0
    sin_half = math.sin(half)
    cos_half = math.cos(half)
    rot = Gf.Quatf(
        float(axis_vec[0] * sin_half),
        float(axis_vec[1] * sin_half),
        float(axis_vec[2] * sin_half),
        float(cos_half),
    )
    return quat * rot


def rotate_joint(
    usd_path: str,
    joint_prim_path: str,
    axis: str,
    angle_deg: float,
    *,
    backup: bool = False,
    dry_run: bool = False,
    attr_name: str = "physics:localRot1",
) -> None:
    """旋转 USD 文件中指定关节的安装角度。

    Args:
        usd_path: USD 文件路径。
        joint_prim_path: joint prim 在 stage 中的完整路径，
            例如 ``/s2_v1/joints/L_sixforce_joint``。
        axis: 旋转轴 ``"x"`` / ``"y"`` / ``"z"``。
        angle_deg: 旋转角度（度）。
        backup: 是否在修改前备份原文件。
        dry_run: 仅计算新值，不实际写入。
        attr_name: 要修改的属性名，默认 ``physics:localRot1``。
    """
    # 备份
    if backup and not dry_run:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        src = Path(usd_path)
        bak = src.with_suffix(f"{src.suffix}.bak.{ts}")
        shutil.copy2(usd_path, bak)
        print(f"[backup] {bak.name}")

    stage = Usd.Stage.Open(usd_path)
    prim = stage.GetPrimAtPath(joint_prim_path)
    if not prim:
        raise ValueError(f"Prim 不存在: {joint_prim_path}")

    attr = prim.GetAttribute(attr_name)
    old_val = attr.Get()
    if old_val is None:
        raise ValueError(
            f"属性 {attr_name} 在 {joint_prim_path} 上不存在或未设置"
        )

    # USD 返回 Gf.Quatf 或 tuple，统一处理
    if isinstance(old_val, Gf.Quatf):
        old_quat = old_val
    else:
        old_quat = Gf.Quatf(
            float(old_val[0]),
            float(old_val[1]),
            float(old_val[2]),
            float(old_val[3]),
        )
    new_quat = rotate_quat(old_quat, axis, angle_deg).GetNormalized()

    def _fmt(q: Gf.Quatf) -> str:
        return f"(w={q.real:.6g}, x={q.imaginary[0]:.6g}, y={q.imaginary[1]:.6g}, z={q.imaginary[2]:.6g})"

    direction = "逆时针" if angle_deg > 0 else "顺时针"
    print(f"[{joint_prim_path}]")
    print(f"  {attr_name}:")
    print(f"    old: {_fmt(old_quat)}")
    print(f"    new: {_fmt(new_quat)}")
    print(f"  axis={axis.upper()}  angle={angle_deg:+.1f}° ({direction})")

    if dry_run:
        print("  [dry-run] 未写入")
    else:
        attr.Set(new_quat)
        stage.Save()
        print("  [saved]")


# ============================================================================
# CLI
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="旋转 USD 关节安装角度（在 localRot1 上叠加 quaternion）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s s2_v1.usd /s2_v1/joints/L_sixforce_joint --axis x --angle 90
  %(prog)s s2_v1.usd /s2_v1/joints/R_sixforce_joint --axis x --angle -90 --backup
  %(prog)s s2_v1.usd /s2_v1/joints/L_sixforce_joint --axis x --angle 90 --dry-run
        """,
    )
    parser.add_argument("usd_path", help="USD 文件路径")
    parser.add_argument("joint_prim_path", help="joint prim 路径，如 /s2_v1/joints/L_sixforce_joint")
    parser.add_argument("--axis", required=True, choices=["x", "y", "z"],
                        help="旋转轴")
    parser.add_argument("--angle", required=True, type=float,
                        help="旋转角度（度），正=逆时针，负=顺时针")
    parser.add_argument("--attr", default="physics:localRot1",
                        help="要修改的属性名 (default: physics:localRot1)")
    parser.add_argument("--backup", action="store_true",
                        help="修改前备份原文件")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅计算不写入")

    args = parser.parse_args()

    rotate_joint(
        usd_path=args.usd_path,
        joint_prim_path=args.joint_prim_path,
        axis=args.axis,
        angle_deg=args.angle,
        backup=args.backup,
        dry_run=args.dry_run,
        attr_name=args.attr,
    )


if __name__ == "__main__":
    main()
