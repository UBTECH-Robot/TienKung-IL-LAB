#!/usr/bin/env python3
"""将旧版 stats.json 升级到 LeRobot v3 聚合所需格式。

背景：早期转换的真机数据集 stats.json 是旧格式——
  - 缺少 count 字段（aggregate_feature_stats 需要）
  - image/video 统计量是扁平 (3,) 而非 (3,1,1)
导致 lerobot-edit-dataset merge 在 aggregate_stats 校验时抛出
"Shape of quantile 'mean' must be (3,1,1)" 并缺少 count。

本脚本对每个数据集做就地修补（原地备份 stats.json -> stats.json.bak）：
  1. 给每个 feature 补 count = [total_frames]（若缺失）
  2. 把 image feature 的 mean/std/min/max 从 (3,) 重塑为 (3,1,1)
已经是新格式的数据集（如 sim_pick_place）会被自动跳过（幂等）。

用法：
  python3 fix_stats_format.py [DATASET_ROOT] [DATASET_NAME ...]
  # 默认扫描 DATASET_ROOT 下所有数据集；DATASET_ROOT 默认 ./ubt_IL/dataset
  # 例：python3 fix_stats_format.py ubt_IL/dataset Pick_up_the_apple_1 Pick_up_the_apple_2
  # 例（预览不改盘）：python3 fix_stats_format.py --dry-run
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np


def is_image_key(key: str) -> bool:
    # 与 compute_stats._validate_stat_value 的判定一致："image" in feature_key
    return "image" in key


def as_array(v):
    return np.array(v)


def needs_patch(stats: dict) -> bool:
    """是否需要修补：任一 feature 缺 count，或任一 image 统计量非 (3,1,1)。"""
    for key, fstats in stats.items():
        if "count" not in fstats:
            return True
        if is_image_key(key):
            for stat in ("mean", "std", "min", "max"):
                if stat in fstats and as_array(fstats[stat]).shape != (3, 1, 1):
                    return True
    return False


def patch_stats(stats: dict, total_frames: int) -> dict:
    patched = {}
    for key, fstats in stats.items():
        new = dict(fstats)
        # 1. 补 count（形状 (1,)）
        if "count" not in new:
            new["count"] = [int(total_frames)]
        # 2. image 统计量 (3,) -> (3,1,1)
        if is_image_key(key):
            for stat in ("mean", "std", "min", "max"):
                if stat in new:
                    arr = as_array(new[stat])
                    if arr.shape == (3, 1, 1):
                        new[stat] = arr.tolist()
                    elif arr.shape == (3,):
                        new[stat] = arr.reshape(3, 1, 1).tolist()
                    else:
                        raise ValueError(
                            f"{key}.{stat} 形状 {arr.shape} 既非 (3,) 也非 (3,1,1)，需人工检查"
                        )
        patched[key] = new
    return patched


def process_dataset(ds_dir: Path, dry_run: bool) -> str:
    stats_path = ds_dir / "meta" / "stats.json"
    info_path = ds_dir / "meta" / "info.json"
    if not stats_path.is_file() or not info_path.is_file():
        return "skip   (no meta/stats.json or info.json)"
    stats = json.loads(stats_path.read_text())
    info = json.loads(info_path.read_text())
    total_frames = info.get("total_frames")
    if not isinstance(total_frames, int) or total_frames <= 0:
        return f"skip   (invalid total_frames={total_frames!r})"

    if not needs_patch(stats):
        return "ok     (already new format, untouched)"

    patched = patch_stats(stats, total_frames)

    # 校验修补结果满足聚合要求
    for key, fstats in patched.items():
        cnt = as_array(fstats["count"])
        if cnt.shape != (1,):
            raise ValueError(f"{key}.count shape {cnt.shape} != (1,)")
        if is_image_key(key):
            for stat in ("mean", "std", "min", "max"):
                shp = as_array(fstats[stat]).shape
                if shp != (3, 1, 1):
                    raise ValueError(f"{key}.{stat} shape {shp} != (3,1,1) after patch")

    if dry_run:
        return "would-patch"

    bak = stats_path.with_suffix(".json.bak")
    if not bak.exists():
        shutil.copy2(stats_path, bak)
    stats_path.write_text(json.dumps(patched, indent=4))
    return f"patched (backup: {bak.name}, total_frames={total_frames})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", nargs="?", default="ubt_IL/dataset", help="数据集根目录")
    ap.add_argument("names", nargs="*", help="只处理指定数据集名（默认根目录下全部）")
    ap.add_argument("--dry-run", action="store_true", help="只预览，不写盘")
    args = ap.parse_args()

    root = Path(args.root)
    if args.names:
        ds_dirs = [root / n for n in args.names]
    else:
        ds_dirs = [p for p in sorted(root.iterdir()) if p.is_dir()]

    print(f"root={root}  dry_run={args.dry_run}")
    print(f"{'dataset':<28} status")
    print("-" * 60)
    n_patch = 0
    for ds_dir in ds_dirs:
        if not ds_dir.is_dir():
            print(f"{ds_dir.name:<28} skip   (not a dir)")
            continue
        try:
            status = process_dataset(ds_dir, args.dry_run)
        except Exception as e:
            status = f"ERROR  ({e})"
        if "patched" in status or "would-patch" in status:
            n_patch += 1
        print(f"{ds_dir.name:<28} {status}")
    print("-" * 60)
    print(f"{'would patch' if args.dry_run else 'patched'}: {n_patch}")


if __name__ == "__main__":
    main()
