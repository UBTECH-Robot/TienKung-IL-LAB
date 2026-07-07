# -*- coding: utf-8 -*-
"""Convert Walker S2 real HDF5 recordings to LeRobot v3.0.

This converter targets the aligned Walker real-data layout:

    <episode>/hdf5/metadata_aligned.hdf5
    <episode>/hdf5/camera_data/.../*_aligned.mp4

It produces a 19D dataset compatible with the Walker S2 gripper 19D order:
17 body/head/waist joints + left/right 1D PGC grippers.
"""

import argparse
import json
import logging
import shutil
from fractions import Fraction
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from tqdm import tqdm

from lerobot.datasets import LeRobotDataset
from lerobot.datasets.io_utils import write_info
from lerobot.datasets.utils import DatasetInfo
import lerobot.datasets.compute_stats as _compute_stats

# lerobot 0.5.x validate_feature_numpy_array compares numpy .shape (tuple)
# against feature spec shape (list) directly with !=, which always fails.
# Patch to normalize both sides to tuple before comparison, same as convert_to_lerobot.py.
try:
    from lerobot.datasets import feature_utils as _fu

    _orig_validate = _fu.validate_feature_numpy_array

    def _patched_validate(name, expected_dtype, expected_shape, value):
        if isinstance(value, np.ndarray) and isinstance(expected_shape, list):
            expected_shape = tuple(expected_shape)
        return _orig_validate(name, expected_dtype, expected_shape, value)

    _fu.validate_feature_numpy_array = _patched_validate
except Exception:
    pass


def _patch_dataset_info_fraction_json() -> None:
    """Let DatasetInfo write Fraction fps as float while keeping runtime fps rational."""
    if getattr(DatasetInfo.to_dict, "_walker_fraction_patch", False):
        return
    original_to_dict = DatasetInfo.to_dict

    def patched_to_dict(self):
        data = original_to_dict(self)
        if isinstance(data.get("fps"), Fraction):
            data["fps"] = float(data["fps"])
        return data

    patched_to_dict._walker_fraction_patch = True
    DatasetInfo.to_dict = patched_to_dict


def _patch_compute_stats_object_dtype() -> None:
    """Cast object numeric arrays to float before LeRobot running stats.

    Fraction timestamps make the writer's timestamp column an object array, while
    the stats implementation expects numeric ndarrays. Casting keeps timestamps
    serializable and avoids changing LeRobot internals globally outside this run.
    """
    if getattr(_compute_stats.get_feature_stats, "_walker_object_patch", False):
        return
    original_get_feature_stats = _compute_stats.get_feature_stats

    def patched_get_feature_stats(array, axis, keepdims, quantile_list=None):
        if isinstance(array, np.ndarray) and array.dtype == object:
            array = array.astype(np.float64)
        return original_get_feature_stats(array, axis, keepdims, quantile_list)

    patched_get_feature_stats._walker_object_patch = True
    _compute_stats.get_feature_stats = patched_get_feature_stats


_patch_dataset_info_fraction_json()
_patch_compute_stats_object_dtype()


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "configs" / "Walker_S2_real_19_1RGBD.json"
DEFAULT_NUMERIC_FEATURES = {"episode_index", "frame_index", "index", "task_index"}


def str2bool(v: str) -> bool:
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "1"):
        return True
    if v.lower() in ("no", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError(f"Boolean value expected, got '{v}'")


def load_walker_real_config(config_path: str | Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load Walker real conversion config and split feature schema from mapping."""
    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    mapping = config.pop("walker_real_mapping", None)
    if mapping is None:
        raise ValueError(f"Config '{config_path}' is missing 'walker_real_mapping'.")

    features = config
    expected_names = mapping["body_joint_order"] + mapping.get("gripper_order", ["left_grip", "right_grip"])
    expected_dim = len(expected_names)

    for key in ("observation.state", "action"):
        feature = features.get(key)
        if feature is None:
            raise ValueError(f"Config missing feature '{key}'.")
        if feature.get("shape") != [expected_dim]:
            raise ValueError(f"Feature '{key}' must have shape [{expected_dim}], got {feature.get('shape')}")
        names = feature.get("names")
        if names is not None and names != expected_names:
            raise ValueError(f"Feature '{key}' names do not match walker_real_mapping order.")

    if "observation.images.camera_head" not in features:
        raise ValueError("Config missing feature 'observation.images.camera_head'.")

    logging.info("Loaded %d LeRobot feature specs from %s", len(features), config_path)
    return features, mapping


def discover_episodes(src_root: str | Path, metadata_rel_path: str) -> list[Path]:
    """Return episode directories containing metadata_rel_path.

    src_root can be either one episode directory or a dataset root containing
    multiple episode directories.
    """
    src_root = Path(src_root)
    if (src_root / metadata_rel_path).is_file():
        return [src_root]

    episodes = [p for p in sorted(src_root.iterdir()) if p.is_dir() and (p / metadata_rel_path).is_file()]
    return episodes


def decode_hdf5_object(value: Any) -> str:
    """Decode a scalar HDF5 object/string cell to str."""
    if isinstance(value, np.ndarray):
        if value.shape:
            value = value.reshape(-1)[0]
        else:
            value = value.item()
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def _flatten_object_dataset(dataset: h5py.Dataset) -> list[Any]:
    return list(np.asarray(dataset[()]).reshape(-1))


def load_json_list(file: h5py.File, hdf5_key: str) -> list[dict[str, Any]]:
    """Load a Walker json_list dataset as a list of dictionaries."""
    rows = []
    for cell in _flatten_object_dataset(file[hdf5_key]):
        rows.append(json.loads(decode_hdf5_object(cell)))
    return rows


def load_timestamps(file: h5py.File, hdf5_key: str) -> np.ndarray:
    return np.asarray(file[hdf5_key][()]).reshape(-1).astype(np.uint64)


def extract_body_positions(joint_json_list: list[dict[str, Any]], body_joint_order: list[str]) -> np.ndarray:
    """Extract selected body/head/waist joint positions by joint name."""
    frames = []
    for idx, msg in enumerate(joint_json_list):
        names = msg.get("name")
        positions = msg.get("position")
        if names is None or positions is None:
            raise ValueError(f"mc_joint_states frame {idx} missing 'name' or 'position'.")
        name_to_position = dict(zip(names, positions, strict=False))
        missing = [name for name in body_joint_order if name not in name_to_position]
        if missing:
            raise ValueError(f"mc_joint_states frame {idx} missing joints: {missing}")
        frames.append([name_to_position[name] for name in body_joint_order])
    return np.asarray(frames, dtype=np.float32)


def extract_gripper_pos(grip_json_list: list[dict[str, Any]], label: str) -> np.ndarray:
    """Extract GripStatus/GripCmd pos as shape (T, 1)."""
    values = []
    for idx, msg in enumerate(grip_json_list):
        if "pos" not in msg:
            raise ValueError(f"{label} frame {idx} missing 'pos'.")
        values.append(float(msg["pos"]))
    return np.asarray(values, dtype=np.float32)[:, None]


def resolve_camera_mp4(hdf5_path: Path, file: h5py.File, value_list_key: str) -> Path:
    """Resolve the aligned MP4 path referenced by the camera value_list."""
    values = [decode_hdf5_object(cell) for cell in _flatten_object_dataset(file[value_list_key])]
    candidates = [v for v in values if v]
    if not candidates:
        raise ValueError(f"Camera value_list '{value_list_key}' is empty.")

    # For MP4-backed image topics all rows normally contain the same relative path.
    rel_path = candidates[0]
    mp4_path = hdf5_path.parent / rel_path
    if not mp4_path.is_file():
        raise FileNotFoundError(f"Camera MP4 not found: {mp4_path}")
    return mp4_path


def _resize_rgb(frame_rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize an RGB frame with PIL to avoid cv2/numpy ABI issues."""
    if frame_rgb.shape[1] == width and frame_rgb.shape[0] == height:
        return frame_rgb
    from PIL import Image

    image = Image.fromarray(frame_rgb)
    resampling = getattr(Image, "Resampling", Image).BILINEAR
    return np.asarray(image.resize((width, height), resampling))


def read_aligned_video(mp4_path: Path, image_size: list[int], expected_frames: int) -> np.ndarray:
    """Read aligned MP4 as RGB uint8 frames resized to (H, W, 3)."""
    width, height = int(image_size[0]), int(image_size[1])
    import imageio.v3 as iio

    frames = []
    for frame_rgb in iio.imiter(mp4_path):
        if frame_rgb.ndim == 2:
            frame_rgb = np.stack([frame_rgb, frame_rgb, frame_rgb], axis=-1)
        elif frame_rgb.shape[-1] == 4:
            frame_rgb = frame_rgb[..., :3]
        frame_rgb = _resize_rgb(frame_rgb.astype(np.uint8), width, height)
        frames.append(frame_rgb)

    if len(frames) != expected_frames:
        raise ValueError(f"Video frame count mismatch for {mp4_path}: expected {expected_frames}, got {len(frames)}")
    return np.stack(frames).astype(np.uint8)


def estimate_fps(timestamps_ns: np.ndarray) -> float:
    """Estimate FPS from aligned uint64 nanosecond timestamps."""
    if len(timestamps_ns) < 2:
        raise ValueError("Need at least two timestamps to estimate FPS.")
    diffs = np.diff(timestamps_ns.astype(np.int64))
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        raise ValueError("No positive timestamp intervals; cannot estimate FPS.")
    return float(1e9 / np.median(diffs))


def parse_fps(fps_arg: str, timestamps_ns: np.ndarray | None = None) -> Fraction:
    if fps_arg == "auto":
        if timestamps_ns is None:
            raise ValueError("--fps auto requires timestamps.")
        fps = estimate_fps(timestamps_ns)
    else:
        fps = float(fps_arg)
    if fps <= 0:
        raise ValueError(f"FPS must be positive, got {fps}")
    return Fraction(fps).limit_denominator(1000)


def ensure_frame_counts(fields: dict[str, np.ndarray]) -> int:
    counts = {name: len(value) for name, value in fields.items()}
    unique_counts = set(counts.values())
    if len(unique_counts) != 1:
        raise ValueError(f"Frame count mismatch: {counts}")
    return unique_counts.pop()


def initialize_dataset(
    repo_id: str,
    tgt_path: str | Path,
    fps: Fraction,
    robot_type: str,
    features: dict[str, Any],
    overwrite: bool,
    image_writer_processes: int,
    image_writer_threads: int,
) -> LeRobotDataset:
    dataset_path = Path(tgt_path) / repo_id
    if dataset_path.exists():
        if not overwrite:
            raise FileExistsError(f"Dataset already exists: {dataset_path}. Pass --overwrite to replace it.")
        shutil.rmtree(dataset_path)
        logging.warning("Removed existing dataset: %s", dataset_path)

    logging.info("Creating LeRobot dataset at %s (fps=%s, robot_type=%s)", dataset_path, fps, robot_type)
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=str(dataset_path),
        fps=fps,
        robot_type=robot_type,
        features=features,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
    )
    return dataset


def process_episode(
    episode_dir: Path,
    dataset: LeRobotDataset,
    task_name: str,
    mapping: dict[str, Any],
) -> bool:
    hdf5_path = episode_dir / mapping["metadata_rel_path"]
    try:
        with h5py.File(hdf5_path, "r") as file:
            joint_json = load_json_list(file, mapping["joint_state_json"])
            left_grip_state_json = load_json_list(file, mapping["left_grip_state_json"])
            right_grip_state_json = load_json_list(file, mapping["right_grip_state_json"])
            left_grip_cmd_json = load_json_list(file, mapping["left_grip_cmd_json"])
            right_grip_cmd_json = load_json_list(file, mapping["right_grip_cmd_json"])
            timestamps = load_timestamps(file, mapping["joint_state_timestamp"])

            body = extract_body_positions(joint_json, mapping["body_joint_order"])
            left_grip_state = extract_gripper_pos(left_grip_state_json, "left_grip_state")
            right_grip_state = extract_gripper_pos(right_grip_state_json, "right_grip_state")
            left_grip_cmd = extract_gripper_pos(left_grip_cmd_json, "left_grip_cmd")
            right_grip_cmd = extract_gripper_pos(right_grip_cmd_json, "right_grip_cmd")

            camera_mp4 = resolve_camera_mp4(hdf5_path, file, mapping["camera_head_value_list"])
            expected_frames = ensure_frame_counts(
                {
                    "joint": body,
                    "left_grip_state": left_grip_state,
                    "right_grip_state": right_grip_state,
                    "left_grip_cmd": left_grip_cmd,
                    "right_grip_cmd": right_grip_cmd,
                    "timestamps": timestamps,
                }
            )
            images = read_aligned_video(camera_mp4, mapping["image_size"], expected_frames)

        # Assemble state/action from body + only the grippers listed in gripper_order
        # (defaults to both, preserving the original 19D behavior).
        grips = mapping.get("gripper_order", ["left_grip", "right_grip"])
        state_grips, action_grips = [], []
        for g in grips:
            if g == "left_grip":
                state_grips.append(left_grip_state)
                action_grips.append(left_grip_cmd)
            elif g == "right_grip":
                state_grips.append(right_grip_state)
                action_grips.append(right_grip_cmd)
            else:
                raise ValueError(f"Unknown gripper in gripper_order: {g}")
        expected_dim = len(mapping["body_joint_order"]) + len(grips)
        observation_state = np.concatenate([body] + state_grips, axis=1).astype(np.float32)
        action = np.concatenate([body] + action_grips, axis=1).astype(np.float32)
        num_frames = ensure_frame_counts(
            {
                "observation.state": observation_state,
                "action": action,
                "observation.images.camera_head": images,
            }
        )
        if observation_state.shape[1] != expected_dim or action.shape[1] != expected_dim:
            raise ValueError(f"Expected {expected_dim}D state/action, got {observation_state.shape}, {action.shape}")

        episode_fps = estimate_fps(timestamps)
        logging.info(
            "Episode %s: frames=%d, estimated_fps=%.6f, camera=%s",
            episode_dir.name,
            num_frames,
            episode_fps,
            camera_mp4,
        )
        logging.info(
            "Action mapping: body action uses observed joint positions; gripper action uses GripCmd.pos."
        )

        for i in tqdm(range(num_frames), desc=f"Processing {episode_dir.name}"):
            dataset.add_frame(
                {
                    "observation.state": observation_state[i],
                    "action": action[i],
                    "observation.images.camera_head": images[i],
                    "task": task_name,
                }
            )
        return True
    except Exception as exc:
        logging.error("Skipped episode %s: %s", episode_dir, exc)
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="Walker S2 real HDF5 -> LeRobot v3.0 converter")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG), help="Walker real conversion config JSON")
    parser.add_argument("--repo_id", type=str, required=True, help="Dataset repository ID / output dataset name")
    parser.add_argument("--src_root", type=str, required=True, help="Episode directory or dataset root containing episodes")
    parser.add_argument("--tgt_path", type=str, required=True, help="Target output parent directory")
    parser.add_argument("--task_name", type=str, default="walker_s2_real", help="Task name stored in the dataset")
    parser.add_argument("--fps", type=str, default="auto", help="Dataset FPS, or 'auto' to estimate from aligned timestamps")
    parser.add_argument("--robot_type", type=str, default="walker_s2", help="Robot type stored in metadata")
    parser.add_argument("--save_one", type=str2bool, default=False, help="Convert only the first successful episode")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing output dataset if present")
    parser.add_argument("--image_writer_processes", type=int, default=4, help="Number of image writer processes")
    parser.add_argument("--image_writer_threads", type=int, default=4, help="Number of image writer threads")
    args = parser.parse_args()

    features, mapping = load_walker_real_config(args.config)
    episodes = discover_episodes(args.src_root, mapping["metadata_rel_path"])
    if not episodes:
        raise FileNotFoundError(
            f"No episodes found under {args.src_root} with {mapping['metadata_rel_path']}"
        )

    first_hdf5 = episodes[0] / mapping["metadata_rel_path"]
    with h5py.File(first_hdf5, "r") as file:
        first_timestamps = load_timestamps(file, mapping["joint_state_timestamp"])
    fps = parse_fps(args.fps, first_timestamps)

    dataset = initialize_dataset(
        repo_id=args.repo_id,
        tgt_path=args.tgt_path,
        fps=fps,
        robot_type=args.robot_type,
        features=features,
        overwrite=args.overwrite,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )

    success_count = 0
    logging.info("Found %d candidate episode(s).", len(episodes))
    try:
        for episode_dir in episodes:
            if process_episode(episode_dir, dataset, args.task_name, mapping):
                dataset.save_episode()
                success_count += 1
                logging.info("Saved episode: %s (%d/%d)", episode_dir.name, success_count, len(episodes))
                if args.save_one:
                    break
            else:
                dataset.clear_episode_buffer()
    finally:
        dataset.finalize()

    dataset.meta.info.fps = float(fps)
    write_info(dataset.meta.info, dataset.meta.root)
    logging.info("Conversion complete: %d/%d episode(s) saved.", success_count, len(episodes))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    main()
