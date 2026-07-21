"""Shared constants and utilities used by CLI and config tools."""

# Default ROS2 topics for Walker S2 wrist cameras (index 0 = left, 1 = right)
DEFAULT_WRIST_TOPICS = [
    "/sensor/camera/wrist_left/color/raw",
    "/sensor/camera/wrist_right/color/raw",
]


def topic_to_frame_id(topic: str, index: int = 0) -> str:
    """Extract a reasonable frame_id from a ROS2 topic path.

    /sensor/camera/wrist_left/color/raw -> wrist_left
    /test/camera                        -> camera_0
    """
    parts = topic.strip("/").split("/")
    if len(parts) >= 3 and parts[0] in ("sensor", "camera"):
        return parts[2]
    return f"camera_{index}"
