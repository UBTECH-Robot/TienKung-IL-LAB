#!/usr/bin/env python3
"""Preview Walker S2 camera frames from the Bridge2 ZMQ image stream."""

from __future__ import annotations

import argparse
import base64
import json
import logging
import time
from pathlib import Path

logger = logging.getLogger("preview_walker_camera")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Bridge2 ZMQ image host")
    parser.add_argument("--port", type=int, default=5563, help="Bridge2 ZMQ image port")
    parser.add_argument("--camera", default="camera_head", help="Camera key in the ZMQ images map")
    parser.add_argument("--width", type=int, default=0, help="Preview resize width; 0 keeps native size")
    parser.add_argument("--height", type=int, default=0, help="Preview resize height; 0 keeps native size")
    parser.add_argument("--window", default="Walker camera", help="OpenCV preview window title")
    parser.add_argument("--timeout-ms", type=int, default=5000, help="Warn when no frames arrive for this long")
    parser.add_argument("--print-fps", action="store_true", help="Periodically print receive/display FPS")
    parser.add_argument("--save-frame", default=None, help="Optional path to save the latest received frame")
    parser.add_argument("--once", action="store_true", help="Receive one frame, optionally save it, then exit")
    return parser.parse_args()


def _decode_frame(message: str, camera_name: str, cv2, np) -> tuple[object | None, list[str]]:
    data = json.loads(message)
    images = data.get("images", {})
    if not isinstance(images, dict):
        return None, []

    available = sorted(str(name) for name in images)
    jpeg_b64 = images.get(camera_name)
    if jpeg_b64 is None:
        return None, available

    jpeg_bytes = base64.b64decode(jpeg_b64)
    np_img = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
    return frame, available


def _resize_for_display(frame: object, width: int, height: int, cv2) -> object:
    if width > 0 and height > 0:
        return cv2.resize(frame, (width, height))
    return frame


def _save_frame(path: str, frame: object, cv2) -> None:
    output = Path(path)
    ok = cv2.imwrite(str(output), frame)
    if not ok:
        raise RuntimeError(f"failed to write frame to {output}")
    logger.info("Saved frame to %s", output)


def main() -> int:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    if (args.width > 0) != (args.height > 0):
        logger.error("--width and --height must be set together, or both left as 0")
        return 2

    try:
        import cv2
        import numpy as np
        import zmq
    except ImportError as exc:
        logger.error("Missing dependency: %s. Run this script in the LeRobot venv/container.", exc)
        return 1

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.connect(f"tcp://{args.host}:{args.port}")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    logger.info("Listening for Walker camera '%s' at tcp://%s:%d", args.camera, args.host, args.port)
    logger.info("Press 'q' or Esc in the preview window to exit.")

    last_warning = 0.0
    last_fps_log = time.perf_counter()
    last_frame: object | None = None
    frames = 0

    try:
        while True:
            events = dict(poller.poll(args.timeout_ms))
            if socket not in events:
                logger.warning(
                    "No camera frames received for %.1fs. Is Walker Bridge2 running with camera_topics configured?",
                    args.timeout_ms / 1000.0,
                )
                continue

            try:
                message = socket.recv_string(flags=zmq.NOBLOCK)
                frame, available = _decode_frame(message, args.camera, cv2, np)
            except zmq.Again:
                continue
            except (json.JSONDecodeError, ValueError, TypeError) as exc:
                now = time.perf_counter()
                if now - last_warning > 1.0:
                    logger.warning("Failed to parse camera message: %s", exc)
                    last_warning = now
                continue

            if frame is None:
                now = time.perf_counter()
                if now - last_warning > 1.0:
                    logger.warning("Camera '%s' not found in message. Available cameras: %s", args.camera, available)
                    last_warning = now
                continue

            last_frame = frame
            frames += 1

            if args.save_frame and args.once:
                _save_frame(args.save_frame, frame, cv2)
                return 0

            if args.once:
                logger.info("Received one frame from camera '%s' with shape %s", args.camera, frame.shape)
                return 0

            display_frame = _resize_for_display(frame, args.width, args.height, cv2)
            try:
                cv2.imshow(args.window, display_frame)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as exc:
                logger.error(
                    "OpenCV preview failed: %s. If running headless, use --once --save-frame /tmp/walker_camera.jpg.",
                    exc,
                )
                return 1
            if key in (ord("q"), 27):
                break

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
        if args.save_frame and last_frame is not None and not args.once:
            _save_frame(args.save_frame, last_frame, cv2)
        cv2.destroyAllWindows()
        poller.unregister(socket)
        socket.close()
        context.term()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
