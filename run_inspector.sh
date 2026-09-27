#!/usr/bin/env bash
# =====================================================================
# 一键启动 inspector_pure.py (Ubuntu Server)
# =====================================================================
# 默认相机模式(产线); 显式传 --mode 或其它参数则原样透传, 可覆盖默认。
#
# 用法:
#   ./run_inspector.sh                       # 相机模式, IP 自动识别(环境变量->同网段探测->内置默认)
#   ./run_inspector.sh --mode local          # 离线遍历本地样本目录
#   ./run_inspector.sh --ip 169.254.44.201   # 手动指定相机 IP
#   ./run_inspector.sh --mode camera --trigger external   # 显式产线配置
#   CAMERA_IP=169.254.44.201 ./run_inspector.sh           # 用环境变量固定相机 IP
#
# 依赖未装时先跑:  bash install_ubuntu.sh
# =====================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# venv 解释器: 现场 Ubuntu 与 install_ubuntu.sh 统一用 venv/(无点), 优先它;
# 万一是老现场手建的 .venv/(有点)则回退。两个都没有才报错。
if [ -x "$ROOT/venv/bin/python" ]; then
  PY="$ROOT/venv/bin/python"
elif [ -x "$ROOT/.venv/bin/python" ]; then
  PY="$ROOT/.venv/bin/python"
else
  echo "[ERROR] 没找到 venv/bin/python(也没有 .venv/bin/python), 先执行: bash install_ubuntu.sh" >&2
  exit 1
fi

# 中文日志防乱码(Linux 默认多为 UTF-8, 显式设置以防 C/POSIX locale 编码报错)
export PYTHONUTF8=1
export PYTHONIOENCODING=UTF-8

# 以「脚本方式」而非 -m 运行: 让 src/flange_inspect 进入 sys.path[0], 保证 inspector 内
# `from uno_relay import ...` 这类同目录导入可解析(-m 从仓库根跑会解析失败)。
SCRIPT="src/flange_inspect/inspector_pure.py"

# 未显式给 --mode 时默认相机模式(产线)。
HAS_MODE=0
for a in "$@"; do case "$a" in --mode|--mode=*) HAS_MODE=1;; esac; done

if [ "$HAS_MODE" = "1" ]; then
  exec "$PY" "$SCRIPT" "$@"
else
  exec "$PY" "$SCRIPT" --mode camera "$@"
fi


