#!/bin/bash
# realsense_wrist_camera — self-contained install script
#
# Installs pyrealsense2 (if not already available) and the package itself
# into system Python 3.10. ROS2 packages (rclpy, sensor_msgs, shm_msgs)
# must already be available in the environment.
#
# Usage:
#   bash install.sh

set -e

echo "[realsense_wrist_camera] Installing..."

# 0. Check Python
PYTHON="${REALSENSE_PYTHON:-/usr/bin/python3}"
if ! "$PYTHON" --version >/dev/null 2>&1; then
    echo "[realsense_wrist_camera] ERROR: Python not found at $PYTHON"
    exit 1
fi
echo "[realsense_wrist_camera] Using Python: $("$PYTHON" --version)"

# 1. Detect a working pip index.
#    Jetson base images set PIP_INDEX_URL env to a local redirect
#    (jetson.webredirect.org) that may be unreachable in some containers.
#    Check both the env var (takes precedence) and pip config.
PIP_INDEX_ARGS=()
JETSON_REDIRECT=0
if [ -n "${PIP_INDEX_URL:-}" ] && echo "${PIP_INDEX_URL}" | grep -q "jetson.webredirect"; then
    JETSON_REDIRECT=1
elif "$PYTHON" -m pip config get global.index-url 2>/dev/null | grep -q "jetson.webredirect"; then
    JETSON_REDIRECT=1
fi

if [ "$JETSON_REDIRECT" = "1" ]; then
    # Use Tsinghua mirror (fast in China, consistent with project Dockerfiles).
    PIP_INDEX_ARGS=(-i "https://pypi.tuna.tsinghua.edu.cn/simple")
    echo "[realsense_wrist_camera] Overriding unreachable Jetson pip redirect -> tsinghua mirror"
fi

# 2. Determine pip install flags.
#    Non-root users need --user. If pip auto-detects user mode it's fine, but
#    being explicit avoids surprises. Root can install system-wide.
PIP_INSTALL_FLAGS=("${PIP_INDEX_ARGS[@]}")
if [ "$(id -u)" -ne 0 ]; then
    PIP_INSTALL_FLAGS+=(--user)
    echo "[realsense_wrist_camera] Installing as user (--user)."
    # Ensure ~/.local/bin is in PATH for CLI entry points
    export PATH="$HOME/.local/bin:$PATH"
fi

# 3. Install pyrealsense2
if "$PYTHON" -c "import pyrealsense2" 2>/dev/null; then
    echo "[realsense_wrist_camera] pyrealsense2 already installed."
else
    echo "[realsense_wrist_camera] Installing pyrealsense2..."
    "$PYTHON" -m pip install "${PIP_INSTALL_FLAGS[@]}" pyrealsense2 || {
        echo "[realsense_wrist_camera] ERROR: pyrealsense2 install failed."
        echo "[realsense_wrist_camera] Try manually:"
        echo "  pip install -i https://pypi.org/simple pyrealsense2"
        exit 1
    }
fi

# 4. Install the package (editable mode)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
echo "[realsense_wrist_camera] Installing package from $PKG_DIR..."
"$PYTHON" -m pip install "${PIP_INSTALL_FLAGS[@]}" -e "$PKG_DIR" || {
    echo "[realsense_wrist_camera] ERROR: Package install failed."
    exit 1
}

# 5. Verify CLI availability
echo ""
echo "[realsense_wrist_camera] Installation complete."
echo ""
echo "  One-command start: bash /ubt_IL/realsense_wrist_camera/scripts/start.sh"
echo ""
echo "  Device discovery:  find-realsense-cameras"
echo "  Start service:     realsense-wrist-camera --config <config.json>"
echo "  Quick test:        realsense-wrist-camera --serial <SN> --topic /test/camera"
echo ""
echo "  Example config:    /ubt_IL/realsense_wrist_camera/configs/wrist_cameras.example.json"
