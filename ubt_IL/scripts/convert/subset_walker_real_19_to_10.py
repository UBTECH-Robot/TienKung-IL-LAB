# -*- coding: utf-8 -*-
"""Subset the Walker S2 real 19D LeRobot v3 dataset to the 10 moving DOFs.

The 19D dataset has 9 constant dims (left arm 7 + waist 1 + left_grip 1) that
break QUANTILES normalization (q99-q01 ~= 0 -> normalized targets explode).
This script keeps only the moving dims:

    keep  = [7..13] (R arm 7) + [14,15] (head 2) + [18] (right_grip)  -> 10D
    drop  = [0..6]  (L arm 7)  + [16]    (waist 1) + [17] (left_grip) -> 9D

Because per-dim stats are independent, slicing preserves correct q01/q99 etc.,
so no stats recompute is needed. Videos are reused as-is via symlink.

Run inside the lerobot container venv (has pyarrow/pandas):
    /lerobot/.venv/bin/python /ubt_IL/scripts/convert/subset_walker_real_19_to_10.py \
        --src /ubt_IL/dataset/Walker_S2_real_19_1RGBD \
        --dst /ubt_IL/dataset/Walker_S2_real_10d_1RGBD
"""

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

# Indices into the 19D action_order (see walker_s2_gripper_19d.json / info.json)
KEEP_DIMS = [7, 8, 9, 10, 11, 12, 13, 14, 15, 18]
NEW_DIM = len(KEEP_DIMS)  # 10

NEW_NAMES = [
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "right_grip",
]

OLD_DIM = 19
SLICE_FEATURES = ("action", "observation.state")


def subset_parquet(src_path: Path, dst_path: Path) -> None:
    """Rewrite a data parquet with action/state sliced to KEEP_DIMS.

    Preserves the `huggingface` KV schema metadata (with action/state length
    updated to NEW_DIM) so HF `datasets` loads the int64 index columns as scalar
    Values instead of trying to cast them to List(int64, length=1) from info.json.
    Drops the `pandas` metadata that `from_pandas` would add.
    """
    import hashlib

    table = pq.read_table(src_path)
    schema = table.schema

    # Update the huggingface KV metadata: action / observation.state length -> NEW_DIM.
    kv = dict(schema.metadata or {})
    hf = json.loads(kv[b"huggingface"]) if b"huggingface" in kv else {"info": {"features": {}}}
    for key in SLICE_FEATURES:
        if key in hf.get("info", {}).get("features", {}):
            hf["info"]["features"][key]["length"] = NEW_DIM
    # Fresh fingerprint so HF datasets cache can't collide with the 19D dataset.
    fp = hashlib.md5((src_path.name + str(NEW_DIM)).encode()).hexdigest()[:16]
    hf["fingerprint"] = fp
    new_kv = {b"huggingface": json.dumps(hf).encode()}

    new_arrays, new_fields = [], []
    for i, field in enumerate(schema):
        if field.name in SLICE_FEATURES:
            orig = table.column(i).combine_chunks()  # FixedSizeListArray[19]
            n = len(orig)
            flat = orig.values.to_numpy().reshape(n, OLD_DIM)[:, KEEP_DIMS].reshape(-1)
            arr = pa.FixedSizeListArray.from_arrays(pa.array(flat, type=pa.float32()), NEW_DIM)
            new_arrays.append(arr)
            new_fields.append(pa.field(field.name, pa.list_(pa.float32(), NEW_DIM)))
        else:
            new_arrays.append(table.column(i))
            new_fields.append(field)
    new_schema = pa.schema(new_fields, metadata=new_kv)
    new_table = pa.Table.from_arrays(new_arrays, schema=new_schema)

    dst_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(new_table, dst_path)


def subset_stats(src_stats: dict) -> dict:
    """Slice per-dim stat arrays for action / observation.state to KEEP_DIMS."""
    out = json.loads(json.dumps(src_stats))  # deep copy
    for key in SLICE_FEATURES:
        if key not in out:
            continue
        for stat_name, value in out[key].items():
            if isinstance(value, list) and len(value) == OLD_DIM:
                out[key][stat_name] = [value[i] for i in KEEP_DIMS]
    return out


def subset_info(src_info: dict) -> dict:
    """Update info.json: action/state shape -> [10] and names -> NEW_NAMES."""
    out = json.loads(json.dumps(src_info))  # deep copy
    feats = out.get("features", {})
    for key in SLICE_FEATURES:
        f = feats.get(key)
        if f is None:
            continue
        if f.get("shape") != [OLD_DIM]:
            raise ValueError(f"info.json feature {key} shape {f.get('shape')} != [{OLD_DIM}]")
        f["shape"] = [NEW_DIM]
        f["names"] = list(NEW_NAMES)
    return out


def slice_episodes_parquet_stats(eps_parquet: Path) -> None:
    """Slice per-episode stats/action/* and stats/observation.state/* to KEEP_DIMS.

    These columns are list<double> (one 19-element array per episode). LeRobot
    drops stats/ columns on load, so this is cosmetic, but keeps metadata
    consistent with the 10D data.
    """
    table = pq.read_table(eps_parquet)
    orig_metadata = table.schema.metadata  # 19D eps has only ARROW:schema (no pandas/hf)
    df = table.to_pandas()
    stats_cols = [c for c in df.columns
                  if c.startswith("stats/action/") or c.startswith("stats/observation.state/")]
    for col in stats_cols:
        df[col] = df[col].apply(lambda v: np.asarray(v)[KEEP_DIMS] if len(v) == OLD_DIM else v)
    # from_pandas adds a `pandas` KV entry; replace with the original metadata so the
    # episodes parquet keeps the same KV format as the 19D source (ARROW:schema only).
    new_table = pa.Table.from_pandas(df, schema=table.schema, preserve_index=False)
    new_table = new_table.replace_schema_metadata(orig_metadata)
    pq.write_table(new_table, eps_parquet)


def main() -> None:
    ap = argparse.ArgumentParser(description="Subset Walker S2 real 19D -> 10D LeRobot v3 dataset")
    ap.add_argument("--src", required=True, help="Source 19D dataset root")
    ap.add_argument("--dst", required=True, help="Destination 10D dataset root")
    ap.add_argument("--overwrite", action="store_true", help="Replace dst if it exists")
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    if dst.exists():
        if not args.overwrite:
            raise SystemExit(f"Destination exists: {dst}. Pass --overwrite to replace.")
        shutil.rmtree(dst)

    # 1. data/ : rewrite parquets with sliced action/state
    for parq in sorted(src.glob("data/chunk-*/file-*.parquet")):
        rel = parq.relative_to(src / "data")
        subset_parquet(parq, dst / "data" / rel)
        print(f"  data/{rel}: sliced -> {NEW_DIM}D")

    # 2. videos/ : reuse unchanged via symlink
    src_videos = src / "videos"
    if src_videos.exists():
        os.symlink(src_videos.resolve(), dst / "videos")
        print(f"  videos -> symlinked to {src_videos}")

    # 3. images/ : reuse unchanged via symlink (may be empty for video datasets)
    src_images = src / "images"
    if src_images.exists():
        os.symlink(src_images.resolve(), dst / "images")

    # 4. meta/ : copy episodes tree + tasks (dim-independent), rewrite stats.json + info.json
    dst_meta = dst / "meta"
    dst_meta.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src / "meta" / "episodes", dst_meta / "episodes")
    shutil.copy2(src / "meta" / "tasks.parquet", dst_meta / "tasks.parquet")
    for eps_parquet in sorted((dst_meta / "episodes").rglob("*.parquet")):
        slice_episodes_parquet_stats(eps_parquet)
        print(f"  meta/episodes/{eps_parquet.name}: per-episode stats sliced -> {NEW_DIM}D")

    src_stats = json.load(open(src / "meta" / "stats.json"))
    json.dump(subset_stats(src_stats), open(dst_meta / "stats.json", "w"), indent=4)
    src_info = json.load(open(src / "meta" / "info.json"))
    json.dump(subset_info(src_info), open(dst_meta / "info.json", "w"), indent=4)
    print(f"  meta/stats.json + info.json: action/state -> [{NEW_DIM}]")

    # 5. verify
    df = pd.read_parquet(next((dst / "data").rglob("*.parquet")))
    a = np.stack(df["action"].to_numpy())
    s = np.stack(df["observation.state"].to_numpy())
    print(f"\nverify: action {a.shape}, state {s.shape}, frames {len(df)}")
    print(f"action[0] = {a[0]}")
    info = json.load(open(dst_meta / "info.json"))
    print(f"info action shape = {info['features']['action']['shape']}, names = {info['features']['action']['names']}")
    print(f"\nDone: {dst}")


if __name__ == "__main__":
    main()
