#!/usr/bin/env bash
# =====================================================================
# Ubuntu Server 一键安装依赖 —— 推力保持架翻边止口检测 (inspector_pure.py)
# =====================================================================
# 做三件事:
#   1) apt 装系统级前置: python3 + venv + OpenCV 运行库(headless server 上 import cv2
#      也要 libGL/glib, 否则报 libGL.so.1 找不到) + git-lfs(datasets 是 LFS 素材)
#   2) 依赖安装: 优先 uv 按 uv.lock 复现锁定环境(numpy/opencv 版本与 Windows 完全一致);
#      装不上 uv 时退回 python3 venv + pip 按 requirements.txt(opencv 版本仍精确 pin)
#   3) 结果统一落在项目根 venv/ , 之后用 ./run_inspector.sh 启动
#
# 用法:
#   bash install_ubuntu.sh            # 核心 + http + plc 全装
#   bash install_ubuntu.sh --core     # 只装核心(numpy/opencv/pyserial)
#   SKIP_APT=1 bash install_ubuntu.sh # 跳过 apt(无 sudo / 系统库已就绪)
# =====================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

CORE_ONLY=0
[ "${1:-}" = "--core" ] && CORE_ONLY=1

log(){ printf '[INFO] %s\n' "$*"; }
die(){ printf '[ERROR] %s\n' "$*" >&2; exit 1; }

# 1) 系统前置 ----------------------------------------------------------------
if [ "${SKIP_APT:-0}" != "1" ]; then
  SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
  log "apt 安装系统前置(python3 / venv / pip / OpenCV 运行库 / git-lfs / curl)..."
  $SUDO apt-get update -y
  # libgl1 + libglib2.0-0: opencv-python 的 cv2.so 在无 GUI 的 server 上也需要这两个动态库
  #   (24.04 起 libgl1-mesa-glx 已并入 libgl1)。若 import cv2 仍报缺别的 .so, 按提示 apt 补装。
  $SUDO apt-get install -y --no-install-recommends \
      python3 python3-venv python3-pip curl ca-certificates git-lfs \
      libgl1 libglib2.0-0
else
  log "SKIP_APT=1, 跳过系统包安装"
fi

# 2) 依赖: 优先 uv -----------------------------------------------------------
# 环境目录统一叫 venv/(无点), 与 run_inspector.sh 和现场 Ubuntu 一致。
# uv 默认建 .venv, 用此环境变量改到 venv, 保证 uv / pip 两条路径都落同一个目录。
export UV_PROJECT_ENVIRONMENT="$ROOT/venv"
UV="$(command -v uv || true)"
if [ -z "$UV" ]; then
  log "未检测到 uv, 尝试安装(Astral 官方脚本, 只装到用户目录, 不动系统 python)..."
  if curl -LsSf https://astral.sh/uv/install.sh | sh; then
    export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
    UV="$(command -v uv || true)"
  fi
fi

if [ -n "$UV" ] && [ -f pyproject.toml ]; then
  log "用 uv sync 复现锁定环境(读 uv.lock)..."
  if [ "$CORE_ONLY" = "1" ]; then "$UV" sync
  else "$UV" sync --extra http --extra plc; fi
else
  log "无 uv, 退回 venv + pip(opencv 版本仍按 requirements.txt 精确 pin)..."
  [ -d venv ] || python3 -m venv venv
  ./venv/bin/python -m pip install --upgrade pip
  if [ "$CORE_ONLY" = "1" ]; then
    ./venv/bin/python -m pip install "numpy>=2.4.6" "opencv-python==4.14.0.94" "pyserial>=3.5"
  else
    ./venv/bin/python -m pip install -r requirements.txt
  fi
fi

[ -x "$ROOT/venv/bin/python" ] || die "安装后仍找不到 venv/bin/python, 请看上方日志"

# 3) 冒烟自检 ----------------------------------------------------------------
log "自检 import numpy / cv2 / serial ..."
./venv/bin/python - <<'PY'
import numpy, cv2, serial
print("[OK] numpy %s | opencv %s | pyserial %s"
      % (numpy.__version__, cv2.__version__, serial.__version__))
PY

cat <<EOF

[完成] 依赖已装到 venv/  (解释器: $ROOT/venv/bin/python)
启动:  ./run_inspector.sh              # 默认相机模式(产线)
       ./run_inspector.sh --mode local # 离线跑本地样本目录做验证

Ubuntu 上还需注意(脚本不代改, 属现场配置):
  * datasets 图片素材是 Git LFS: 首次克隆后跑  git lfs install && git lfs pull  才有真图。
  * UNO 继电器串口: uno_relay.py 的 COM_PORT 默认 "COM7"(Windows), Linux 无效,
    改成 "AUTO"(自动找 CH340)或 "/dev/ttyUSB0"。
  * 串口权限: sudo usermod -aG dialout \$USER  然后重新登录。
  * 相机直连网卡需 169.254 段地址: sudo ip addr add 169.254.44.200/16 dev <网卡名>。
EOF
