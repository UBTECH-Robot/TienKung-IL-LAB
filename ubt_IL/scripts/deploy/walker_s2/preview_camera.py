#!/usr/bin/env python3
"""Preview Walker S2 camera frames from ROS2 image topics.

Two modes:
  --topic    Single camera preview (legacy mode)
  --robot    Multi-camera preview from robot config (reads camera_topics)

Usage:
  python3 preview_camera.py --robot walker_s2_31d
  python3 preview_camera.py --robot walker_s2_10d
  python3 preview_camera.py --topic /sensor/camera/stereo/color/raw
  python3 preview_camera.py --topic /sensor/camera/stereo/color/raw --width 640 --height 480
  python3 preview_camera.py --robot walker_s2_10d --once --save-frame /tmp/tiled.jpg
"""

from __future__ import annotations

import argparse
import logging
import threading
import time
from pathlib import Path

import numpy as np

logger = logging.getLogger("preview_walker_camera")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--robot", default=None,
        choices=["walker_s2_31d", "walker_s2_19d", "walker_s2_10d"],
        help="Robot model name — preview all cameras from its camera_topics",
    )
    source.add_argument(
        "--topic", default=None,
        help="Single ROS2 camera topic (default if --robot not given: /sensor/camera/stereo/color/raw)",
    )
    parser.add_argument(
        "--msg-type", default="Image2m",
        choices=["Image8k", "Image512k", "Image1m", "Image2m", "Image4m", "Image8m",
                 "sensor_msgs/Image"],
        help="Image message type (default: Image2m / shm_msgs.msg.Image2m)",
    )
    parser.add_argument("--width", type=int, default=0,
                        help="Per-camera cell width; 0 keeps native size")
    parser.add_argument("--height", type=int, default=0,
                        help="Per-camera cell height; 0 keeps native size")
    parser.add_argument("--window", default="Walker camera", help="OpenCV preview window title")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Wait timeout for first frame in seconds")
    parser.add_argument("--print-fps", action="store_true", help="Periodically print display FPS")
    parser.add_argument("--save-frame", default=None,
                        help="Optional path to save the latest frame (tiled in multi-camera mode)")
    parser.add_argument("--once", action="store_true",
                        help="Receive one frame from each camera, optionally save, then exit")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Camera registry (populated by _setup_cameras)
# ---------------------------------------------------------------------------

class CameraEntry:
    """A named camera node + its metadata."""
    __slots__ = ("name", "node", "topic")

    def __init__(self, name: str, node, topic: str):
        self.name = name
        self.node = node
        self.topic = topic


# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------

def _resolve_msg_type(name: str):
    """Resolve a --msg-type string to a ROS2 message class."""
    if name == "sensor_msgs/Image":
        from sensor_msgs.msg import Image
        return Image
    import shm_msgs.msg
    return getattr(shm_msgs.msg, name)


def _resolve_camera_topics(args: argparse.Namespace) -> dict[str, str]:
    """Return {camera_name: ros2_topic} from --robot or --topic."""
    if args.robot:
        import importlib.util
        _constants_path = "/ubt_IL/walker/lerobot_robot_walker/lerobot_robot_walker/constants.py"
        spec = importlib.util.spec_from_file_location("_walker_constants", _constants_path)
        _mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_mod)
        cfg = _mod.ROBOT_MODELS[args.robot]
        return dict(cfg.camera_topics)

    # Single-topic mode
    topic = args.topic or "/sensor/camera/stereo/color/raw"
    return {"camera": topic}


def _setup_cameras(topics: dict[str, str], msg_type) -> list[CameraEntry]:
    """Create one Camera node per topic.  Must be called after rclpy.init()."""
    import sys
    _walker_sdk_path = "/ubt_IL/walker/walker_sdk_ros2/robot_control"
    if _walker_sdk_path not in sys.path:
        sys.path.insert(0, _walker_sdk_path)
    from camera import Camera

    entries: list[CameraEntry] = []
    for name, topic in topics.items():
        node = Camera(topic=topic, msg_type=msg_type, node_name=f"cam_{name}")
        entries.append(CameraEntry(name=name, node=node, topic=topic))
    return entries


# ---------------------------------------------------------------------------
# Tiling
# ---------------------------------------------------------------------------

def _tile_grid(n: int) -> tuple[int, int]:
    """Return (cols, rows) for n cameras — prefer 3-column layout for 5 cams."""
    if n <= 2:
        return (n, 1)
    if n == 4:
        return (2, 2)
    # 3 or 5 → single row of 3, or 3×2
    return (3, (n + 2) // 3)


def _build_tile(
    frames: list[tuple[str, np.ndarray | None]],
    cell_w: int,
    cell_h: int,
    cv2,
) -> np.ndarray:
    """Tile frames into a single BGR canvas.  Missing frames → black placeholder."""
    n = len(frames)
    cols, rows = _tile_grid(n)
    canvas = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for idx, (name, frame) in enumerate(frames):
        r, c = divmod(idx, cols)
        y0, x0 = r * cell_h, c * cell_w

        if frame is not None:
            h, w = frame.shape[:2]
            # Scale to fit cell, preserving aspect ratio
            scale = min(cell_w / w, cell_h / h)
            fh, fw = int(h * scale), int(w * scale)
            if fh != h or fw != w:
                display = cv2.resize(frame, (fw, fh))
            else:
                display = frame
            # Center in cell
            dy, dx = (cell_h - fh) // 2, (cell_w - fw) // 2
            canvas[y0 + dy:y0 + dy + fh, x0 + dx:x0 + dx + fw] = display

        # Label
        cv2.putText(canvas, name, (x0 + 5, y0 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return canvas


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _save_frame(path: str, frame: np.ndarray, cv2) -> None:
    output = Path(path)
    ok = cv2.imwrite(str(output), frame)
    if not ok:
        raise RuntimeError(f"failed to write frame to {output}")
    logger.info("Saved frame to %s", output)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    try:
        import cv2
    except ImportError as exc:
        logger.error("Missing dependency: %s. Install opencv-python-headless.", exc)
        return 1

    # ── Resolve topics & message type ────────────────────────────────────
    camera_topics = _resolve_camera_topics(args)
    MsgType = _resolve_msg_type(args.msg_type)

    # ── ROS2 init ────────────────────────────────────────────────────────
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    entries = _setup_cameras(camera_topics, MsgType)

    executor = MultiThreadedExecutor(num_threads=max(2, len(entries) + 1))
    for e in entries:
        executor.add_node(e.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── Wait for all cameras ─────────────────────────────────────────────
    logger.info("Waiting for %d camera(s) (timeout=%.1fs) ...", len(entries), args.timeout)
    all_ready = True
    for e in entries:
        if not e.node.wait_for_image(timeout=args.timeout):
            logger.error("Timeout: no image on '%s' (%s)", e.topic, e.name)
            all_ready = False
    if not all_ready:
        logger.error("Not all cameras are publishing. Check topics and RMW_IMPLEMENTATION.")

    # Print info for ready cameras
    for e in entries:
        if e.node.is_available():
            info = e.node.get_image_info()
            if info:
                logger.info("  %-20s → %dx%d %s", e.name,
                            info["width"], info["height"], info["encoding"])

    if not args.once and any(e.node.is_available() for e in entries):
        cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
        logger.info("Press 'q' or Esc in the preview window to exit.")

    # ── Determine cell size ──────────────────────────────────────────────
    cell_w = args.width
    cell_h = args.height
    if cell_w == 0 or cell_h == 0:
        # Use first available camera's native resolution
        for e in entries:
            info = e.node.get_image_info()
            if info:
                cell_w = cell_w or info["width"]
                cell_h = cell_h or info["height"]
                break
        cell_w = cell_w or 640
        cell_h = cell_h or 480

    # ── Main loop ────────────────────────────────────────────────────────
    last_tile: np.ndarray | None = None
    last_fps_log = time.perf_counter()
    frames = 0
    return_code = 0

    try:
        while rclpy.ok():
            # Collect latest frame from each camera
            frame_list: list[tuple[str, np.ndarray | None]] = []
            for e in entries:
                img = e.node.get_latest_image(encoding="bgr8")
                frame_list.append((e.name, img))

            if all(f is None for _, f in frame_list):
                time.sleep(0.01)
                continue

            # In single-camera mode, just use the raw frame
            if len(entries) == 1:
                display_frame = frame_list[0][1]
                if display_frame is None:
                    time.sleep(0.01)
                    continue
                if cell_w > 0 and cell_h > 0:
                    display_frame = cv2.resize(display_frame, (cell_w, cell_h))
                last_tile = display_frame
            else:
                display_frame = _build_tile(frame_list, cell_w, cell_h, cv2)
                last_tile = display_frame

            frames += 1

            # --once
            if args.once:
                if args.save_frame and last_tile is not None:
                    _save_frame(args.save_frame, last_tile, cv2)
                else:
                    logger.info("Received one frame from %d camera(s)", len(entries))
                return 0

            # Show
            try:
                cv2.imshow(args.window, display_frame)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as exc:
                logger.error(
                    "OpenCV preview failed: %s. "
                    "If running headless, use --once --save-frame /tmp/frame.jpg.",
                    exc,
                )
                return_code = 1
                break
            if key in (ord("q"), 27):
                break

            # FPS
            if args.print_fps:
                now = time.perf_counter()
                elapsed = now - last_fps_log
                if elapsed >= 5.0:
                    logger.info("Preview FPS: %.1f", frames / elapsed)
                    frames = 0
                    last_fps_log = now

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        if args.save_frame and last_tile is not None and not args.once:
            _save_frame(args.save_frame, last_tile, cv2)
        cv2.destroyAllWindows()
        for e in entries:
            executor.remove_node(e.node)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        for e in entries:
            e.node.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
