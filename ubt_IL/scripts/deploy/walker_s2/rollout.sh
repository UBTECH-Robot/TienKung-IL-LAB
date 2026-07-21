#!/bin/bash
# Walker S2 部署（rollout）脚本
# 在 ubt_IL/lerobot 容器内运行。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/configs"

POLICY_PATH="${POLICY_PATH:-}"
# ROBOT_MODEL 统一入口：一个变量选定关节 DOF + 末端执行器 + 相机配置。
# 可用值见 walker constants.py 的 ROBOT_MODELS 注册表。
ROBOT_MODEL="${ROBOT_MODEL:-walker_s2_31d}"
# JOINT_CONFIG 默认等于 ROBOT_MODEL；显式设置可部署 DOF 子集策略（如 10d 模型用 31d 硬件）。
JOINT_CONFIG="${JOINT_CONFIG:-$ROBOT_MODEL}"
# ROBOT_CONFIG 可选：指向自定义 JSON 覆盖文件（不使用 ROBOT_MODELS 默认相机参数时）。
ROBOT_CONFIG="${ROBOT_CONFIG:-}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-0}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-15}"
DURATION="${DURATION:-30}"
TASK="${TASK:-walker s2 rollout}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-1}"
PREVIEW_CAMERA_HOST="${PREVIEW_CAMERA_HOST:-127.0.0.1}"
PREVIEW_CAMERA_PORT="${PREVIEW_CAMERA_PORT:-5563}"
PREVIEW_CAMERA_NAME="${PREVIEW_CAMERA_NAME:-camera_head}"
PREVIEW_CAMERA_WIDTH="${PREVIEW_CAMERA_WIDTH:-0}"
PREVIEW_CAMERA_HEIGHT="${PREVIEW_CAMERA_HEIGHT:-0}"
PREVIEW_CAMERA_TIMEOUT_MS="${PREVIEW_CAMERA_TIMEOUT_MS:-5000}"
PREVIEW_CAMERA_PRINT_FPS="${PREVIEW_CAMERA_PRINT_FPS:-1}"
PREVIEW_CAMERA_WINDOW="${PREVIEW_CAMERA_WINDOW:-Walker camera}"

if [ -z "$POLICY_PATH" ]; then
    echo "[ERROR] POLICY_PATH is required."
    echo "[INFO] Example:"
    echo "       ROBOT_MODEL=walker_s2_19d POLICY_PATH=/ubt_IL/model/<walker_policy>/checkpoints/last/pretrained_model bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh"
    exit 1
fi

if [ -n "$ROBOT_CONFIG" ] && [ ! -f "$ROBOT_CONFIG" ]; then
    echo "[ERROR] ROBOT_CONFIG not found: $ROBOT_CONFIG"
    echo "[INFO] Set ROBOT_MODEL to one of the registered models, or unset ROBOT_CONFIG to use ROBOT_MODELS defaults."
    exit 1
fi

# -----------------------------------------------------------------------
# Preflight: always validate that the policy's action dim / names match
# the robot config, whether sourced from ROBOT_CONFIG JSON or ROBOT_MODELS
# registry.  This catches mismatches before any hardware is touched.
# -----------------------------------------------------------------------
if [ -f "$POLICY_PATH/config.json" ]; then
    if [ -n "$ROBOT_CONFIG" ]; then
        echo "[INFO] Preflight: validating policy against ROBOT_CONFIG=$ROBOT_CONFIG"
        /lerobot/.venv/bin/python - "$ROBOT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json, sys
from pathlib import Path

rc = Path(sys.argv[1]);  pc = Path(sys.argv[2])
allow_dim_only = sys.argv[3] == "1"

with rc.open("r", encoding="utf-8") as f:
    robot = json.load(f)
with pc.open("r", encoding="utf-8") as f:
    policy = json.load(f)

action_order = robot.get("action_order")
if not isinstance(action_order, list) or not action_order:
    raise SystemExit(f"[ERROR] {rc} must contain non-empty action_order")
if any(not isinstance(n, str) or not n for n in action_order):
    raise SystemExit("[ERROR] action_order entries must be non-empty strings")
expected_features = [f"{n}.pos" for n in action_order]
expected_dim = len(expected_features)

shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] Could not find policy output action shape in config.json")
action_dim = int(shape[0])
if action_dim != expected_dim:
    raise SystemExit(
        f"[ERROR] Action dim mismatch: robot config expects {expected_dim}, policy has {action_dim}\n"
        f"        robot_config={rc}\n        policy_config={pc}"
    )

names = None
for root in (policy, policy.get("policy", {})):
    candidate = root.get("action_feature_names")
    if candidate:
        names = list(candidate)
        break
    output_action = root.get("output_features", {}).get("action", {})
    candidate = output_action.get("names")
    if isinstance(candidate, list):
        names = list(candidate)
        break

if names is not None:
    if names != expected_features:
        raise SystemExit(
            "[ERROR] Policy action names/order do not match robot config.\n"
            f"        expected={expected_features}\n        policy={names}"
        )
elif not allow_dim_only:
    raise SystemExit(
        "[ERROR] Policy config has no action names; refusing dim-only deployment.\n"
        "        Set ALLOW_DIM_ONLY_POLICY=1 only if you verified the policy order matches robot action_order."
    )

print(f"[INFO] Robot config source : {rc}")
print(f"[INFO] Policy action dim   : {action_dim}")
print(f"[INFO] Expected action dim : {expected_dim}")
print(f"[INFO] Action feature names: {expected_features}")
if names is None:
    print("[WARN] ALLOW_DIM_ONLY_POLICY=1: policy action names are unavailable; using robot config order by dimension only.")
PY
    else
        echo "[INFO] Preflight: validating policy against ROBOT_MODEL=$JOINT_CONFIG"
        /lerobot/.venv/bin/python - "$JOINT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json, sys

joint_config   = sys.argv[1]
pc             = sys.argv[2]
allow_dim_only = sys.argv[3] == "1"

from lerobot_robot_walker.constants import ROBOT_MODELS, joint_names_with_pos

if joint_config not in ROBOT_MODELS:
    raise SystemExit(
        f"[ERROR] {joint_config!r} not in ROBOT_MODELS registry. "
        f"Available: {list(ROBOT_MODELS)}"
    )
spec = ROBOT_MODELS[joint_config]
expected_features = joint_names_with_pos(spec["joint_order"])
expected_dim = len(expected_features)

with open(pc, "r", encoding="utf-8") as f:
    policy = json.load(f)

shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] Could not find policy output action shape in config.json")
action_dim = int(shape[0])
if action_dim != expected_dim:
    raise SystemExit(
        f"[ERROR] Action dim mismatch: ROBOT_MODELS[{joint_config}] expects {expected_dim}, policy has {action_dim}\n"
        f"        policy_config={pc}"
    )

names = None
for root in (policy, policy.get("policy", {})):
    candidate = root.get("action_feature_names")
    if candidate:
        names = list(candidate)
        break
    output_action = root.get("output_features", {}).get("action", {})
    candidate = output_action.get("names")
    if isinstance(candidate, list):
        names = list(candidate)
        break

if names is not None:
    if names != expected_features:
        raise SystemExit(
            "[ERROR] Policy action names/order do not match robot config.\n"
            f"        expected={expected_features}\n        policy={names}"
        )
elif not allow_dim_only:
    raise SystemExit(
        "[ERROR] Policy config has no action names; refusing dim-only deployment.\n"
        "        Set ALLOW_DIM_ONLY_POLICY=1 only if you verified the policy order matches ROBOT_MODELS[{joint_config}]."
    )

print(f"[INFO] Robot config source : ROBOT_MODELS[{joint_config}]")
print(f"[INFO] Policy action dim   : {action_dim}")
print(f"[INFO] Expected action dim : {expected_dim}")
print(f"[INFO] Action feature names: {expected_features}")
if names is None:
    print("[WARN] ALLOW_DIM_ONLY_POLICY=1: policy action names are unavailable; using robot config order by dimension only.")
PY
    fi
else
    echo "[WARN] Policy config.json not found at $POLICY_PATH/config.json; skipping dimension preflight."
fi

if [ -z "$ROBOT_CONFIG" ]; then
    echo "[INFO] ROBOT_MODEL=$ROBOT_MODEL JOINT_CONFIG=$JOINT_CONFIG"
fi

cd /ubt_IL/lerobot

PREVIEW_PID=""
cleanup_preview() {
    if [ -n "$PREVIEW_PID" ]; then
        kill "$PREVIEW_PID" 2>/dev/null || true
        wait "$PREVIEW_PID" 2>/dev/null || true
    fi
}

if [ "$PREVIEW_CAMERA" = "1" ]; then
    PREVIEW_CMD=(
        /lerobot/.venv/bin/python "$SCRIPT_DIR/preview_camera.py"
        --host "$PREVIEW_CAMERA_HOST"
        --port "$PREVIEW_CAMERA_PORT"
        --camera "$PREVIEW_CAMERA_NAME"
        --width "$PREVIEW_CAMERA_WIDTH"
        --height "$PREVIEW_CAMERA_HEIGHT"
        --timeout-ms "$PREVIEW_CAMERA_TIMEOUT_MS"
        --window "$PREVIEW_CAMERA_WINDOW"
    )
    if [ "$PREVIEW_CAMERA_PRINT_FPS" = "1" ]; then
        PREVIEW_CMD+=(--print-fps)
    fi
    echo "[INFO] Starting Walker camera preview: ${PREVIEW_CAMERA_NAME} at ${PREVIEW_CAMERA_HOST}:${PREVIEW_CAMERA_PORT}"
    "${PREVIEW_CMD[@]}" &
    PREVIEW_PID=$!
    trap cleanup_preview EXIT INT TERM
fi

ROBOT_CONFIG_ARG=()
if [ -n "$ROBOT_CONFIG" ]; then
    ROBOT_CONFIG_ARG=(--robot.robot_config_path="$ROBOT_CONFIG")
fi

/lerobot/.venv/bin/lerobot-rollout \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    --robot.type=walker \
    "${ROBOT_CONFIG_ARG[@]}" \
    --robot.joint_config="$JOINT_CONFIG" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION"
