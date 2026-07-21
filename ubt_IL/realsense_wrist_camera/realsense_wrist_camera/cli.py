"""CLI entry point for realsense-wrist-camera.

Usage:
    # Auto-discover + start (one command):
    realsense-wrist-camera --discover

    # From config file:
    realsense-wrist-camera --config /path/to/cameras.json

    # Single camera test:
    realsense-wrist-camera --serial <SN> --topic /test/camera
"""

import argparse
import json
import logging
import sys
from pathlib import Path

from ._common import DEFAULT_WRIST_TOPICS, topic_to_frame_id

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] [%(name)s] %(message)s",
)
logger = logging.getLogger("realsense_wrist_camera.cli")

def main():
    parser = argparse.ArgumentParser(
        description="Intel RealSense D405 wrist camera ROS2 publisher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  # Auto-discover cameras and start (one command):\n"
            "  realsense-wrist-camera --discover\n\n"
            "  # From config file:\n"
            "  realsense-wrist-camera --config /path/to/cameras.json\n\n"
            "  # Single camera test:\n"
            "  realsense-wrist-camera --serial 241322100110 \\\n"
            "      --topic /sensor/camera/wrist_left/color/raw \\\n"
            "      --msg-type shm_msgs/Image1m\n"
        ),
    )

    # Auto-discover mode
    parser.add_argument(
        "--discover", action="store_true",
        help="Auto-discover connected RealSense cameras and start",
    )

    # JSON config mode
    parser.add_argument(
        "--config",
        type=str,
        help="Path to JSON config file",
    )

    # Single-camera mode
    parser.add_argument("--serial", type=str,
                        help="Camera serial number (single mode)")
    parser.add_argument("--topic", type=str, default="/test/camera",
                        help="ROS2 topic (single mode, default: /test/camera)")
    parser.add_argument("--frame-id", type=str, default="realsense_camera",
                        help="Frame ID in message header")
    parser.add_argument("--msg-type", type=str, default="shm_msgs/Image1m",
                        choices=["sensor_msgs/Image", "shm_msgs/Image1m",
                                 "shm_msgs/Image2m", "shm_msgs/Image4m"],
                        help="ROS2 image message type (default: shm_msgs/Image1m)")
    parser.add_argument("--width", type=int, default=640,
                        help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480,
                        help="Frame height (default: 480)")
    parser.add_argument("--fps", type=int, default=15,
                        help="Frame rate (default: 15)")

    args = parser.parse_args()

    # -- Build camera configs
    cameras = _load_camera_configs(args)

    if not cameras:
        print("ERROR: No cameras configured.", file=sys.stderr)
        print("Use --discover, --config <file>, or --serial <SN>.",
              file=sys.stderr)
        sys.exit(1)

    # -- Initialize ROS2
    try:
        rclpy_init()
    except Exception as e:
        logger.error("Failed to initialize ROS2: %s", e)
        logger.error(
            "Ensure ROS2 is installed and sourced "
            "(e.g., source /opt/ros/humble/setup.bash)"
        )
        sys.exit(1)

    # -- Create node and start
    node = None
    try:
        from realsense_wrist_camera.node import RealSenseWristCameraNode

        node = RealSenseWristCameraNode(cameras=cameras)
        node.start()

        logger.info(
            "RealSense wrist camera service running. Press Ctrl+C to stop."
        )
        node.spin_forever()

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error("Fatal error: %s", e)
        sys.exit(1)
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy_shutdown()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_camera_configs(args: argparse.Namespace) -> list[dict]:
    """Load camera configs from --discover, --config, or --serial."""

    # --discover: auto-detect cameras
    if args.discover:
        return _discover_cameras(args)

    # --config: JSON file
    if args.config:
        return _load_from_config(args)

    # --serial: single camera
    if args.serial:
        return [_single_camera_config(args)]

    return []


def _discover_cameras(args: argparse.Namespace) -> list[dict]:
    """Auto-discover RealSense cameras and build configs."""
    from realsense_wrist_camera.driver import RealSenseD405Driver

    devices = RealSenseD405Driver.discover()
    if not devices:
        raise RuntimeError(
            "No RealSense cameras detected. "
            "Check USB connection and run 'find-realsense-cameras' for details."
        )

    cameras = []
    for i, dev in enumerate(devices):
        topic = DEFAULT_WRIST_TOPICS[i] if i < len(DEFAULT_WRIST_TOPICS) else f"/camera/realsense_{i}"
        frame_id = topic_to_frame_id(topic, i)
        cameras.append({
            "serial": dev["serial"],
            "topic": topic,
            "msg_type": args.msg_type,
            "frame_id": frame_id,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
        })
        logger.info(
            "Auto-detected: %s (SN=%s) → %s",
            dev["name"], dev["serial"], topic,
        )

    return cameras


def _load_from_config(args: argparse.Namespace) -> list[dict]:
    """Load cameras from JSON config file."""
    path = Path(args.config)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    cameras = data.get("cameras", [])
    if not isinstance(cameras, list) or not cameras:
        raise ValueError(f"Config file {path} must contain a non-empty 'cameras' list.")

    for i, cam in enumerate(cameras):
        if "serial" not in cam:
            raise ValueError(f"Camera entry {i} missing 'serial' field")
        cam.setdefault("topic", "/test/camera")
        cam.setdefault("msg_type", "shm_msgs/Image1m")
        cam.setdefault("frame_id", f"realsense_camera_{i}")
        cam.setdefault("width", 640)
        cam.setdefault("height", 480)
        cam.setdefault("fps", 15)

    return cameras


def _single_camera_config(args: argparse.Namespace) -> dict:
    """Build single-camera config from CLI args."""
    return {
        "serial": args.serial,
        "topic": args.topic,
        "msg_type": args.msg_type,
        "frame_id": args.frame_id,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
    }


# ---------------------------------------------------------------------------
# ROS2 lifecycle wrappers
# ---------------------------------------------------------------------------

_ros_initialized = False


def rclpy_init():
    """Initialize rclpy (idempotent)."""
    global _ros_initialized
    if not _ros_initialized:
        import rclpy
        rclpy.init()
        _ros_initialized = True


def rclpy_shutdown():
    """Shutdown rclpy if initialized."""
    global _ros_initialized
    if _ros_initialized:
        import rclpy
        rclpy.shutdown()
        _ros_initialized = False


if __name__ == "__main__":
    main()
