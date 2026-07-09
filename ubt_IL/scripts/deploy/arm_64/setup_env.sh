#!/bin/bash
# env_vla 环境构建脚本（去 Docker，conda + 本地编译 cp312 wheel）
# 在 Jetson AGX Orin (JetPack 6 / glibc 2.35 / CUDA 12.6) 上构建 LeRobot 推理环境。
# 决策与依据见同目录 README.md。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # -> .../ubt_IL
ARM64_DIR="$SCRIPT_DIR"

CONDA_BASE="${CONDA_BASE:-$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")}"
ENV_NAME="${ENV_NAME:-env_vla}"
PIP_MIRROR="${PIP_MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"

# 本地编译的 cp312 wheel（位于本目录）
TORCH_WHL="$ARM64_DIR/torch-2.7.1a0+gite2d141d-cp312-cp312-linux_aarch64.whl"
TV_WHL="$ARM64_DIR/torchvision-0.22.1+59a3e1f-cp312-cp312-linux_aarch64.whl"

echo "[setup] PROJECT_ROOT = $PROJECT_ROOT"
echo "[setup] ENV_NAME     = $ENV_NAME"
echo "[setup] CONDA_BASE   = $CONDA_BASE"

# 校验 wheel 存在
for w in "$TORCH_WHL" "$TV_WHL"; do
    [ -f "$w" ] || { echo "[setup] ERROR: 缺少 wheel: $w"; exit 1; }
done

# 1. conda 初始化
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"

# 2. 创建 env（幂等）
if conda env list | grep -q "^${ENV_NAME} "; then
    echo "[setup] env '${ENV_NAME}' 已存在，跳过创建（如需重建先 conda env remove -n ${ENV_NAME}）"
else
    echo "[setup] 创建 conda env '${ENV_NAME}' (python=3.12)..."
    conda create -n "$ENV_NAME" python=3.12 -y
fi

conda activate "$ENV_NAME"
echo "[setup] python: $(which python) | $(python --version)"

PIPCMD="pip install --no-cache-dir"
IDX="-i $PIP_MIRROR --extra-index-url https://pypi.org/simple"

# 3. 装本地 torch/torchvision wheel（--no-deps，避免拉 nvidia-* / 破坏 numpy<2）
echo "[setup] 安装本地 cp312 torch/torchvision wheel..."
$PIPCMD --no-deps "$TORCH_WHL" "$TV_WHL"

# 4. torch 运行时小依赖
echo "[setup] 安装 torch 运行时依赖..."
$PIPCMD $IDX filelock "typing-extensions>=4.8.0" sympy networkx jinja2 fsspec setuptools

# 5. numpy <2（本地 torch 按 numpy 1.x 编译，2.x 会崩溃）
echo "[setup] 安装 numpy 1.26.4 (<2)..."
$PIPCMD $IDX "numpy==1.26.4"

# 6. LeRobot 运行时依赖（含 dataset extra：av/deepdiff/rerun-sdk；rollout 导入链需要）
echo "[setup] 安装 LeRobot 运行时依赖..."
$PIPCMD $IDX \
    "draccus==0.10.0" "gymnasium>=1.1.1" einops safetensors \
    "huggingface-hub>=1.0" termcolor tqdm "packaging>=24.2" requests \
    Pillow datasets diffusers h5py pandas pyarrow pyzmq "opencv-python<4.14" \
    "av>=15.0.0,<16.0.0" "deepdiff>=7.0.1,<9.0.0" "rerun-sdk>=0.24.0,<0.27.0" \
    "cmake>=3.29.0.1"

# 6.5 强制 numpy<2（上一步的依赖可能把 numpy 升到 2.x；本地 torch 按 1.x 编译，
#     2.x 会触发 "compiled using NumPy 1.x" 崩溃。容器 entrypoint 同款做法）
echo "[setup] 强制 numpy==1.26.4 (<2)..."
$PIPCMD $IDX "numpy==1.26.4"

# 7. editable 安装 LeRobot + tienkung 插件（--no-deps 保 numpy<2；3.12 原生免补丁）
echo "[setup] editable 安装 LeRobot..."
$PIPCMD --no-deps -e "$PROJECT_ROOT/lerobot"
echo "[setup] editable 安装 tienkung 插件..."
$PIPCMD --no-deps -e "$PROJECT_ROOT/tienkung/lerobot_robot_tienkung"

# 8. 验证
echo "[setup] 验证导入..."
python - <<'EOF'
import torch, numpy
print("torch:", torch.__version__, "| numpy:", numpy.__version__)
print("cuda available:", torch.cuda.is_available())
import lerobot
from lerobot_robot_tienkung import TienKungRobotConfig
print("lerobot + tienkung plugin: OK")
EOF

echo ""
echo "[setup] 完成。激活环境: conda activate $ENV_NAME"
echo "[setup] 部署: bash $ARM64_DIR/rollout_host.sh"
