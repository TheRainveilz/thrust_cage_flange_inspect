#!/usr/bin/env bash
# =====================================================================
# 一键启动 main_pipeline.py (Ubuntu Server) —— 产线唯一的入口
# =====================================================================
# 主程序独占 UNO 串口并看护两个检测站(正反面翻边止口 / 兜孔缺粒)，两个检测站各跑一个
# 独立子进程，判定结果经 localhost 上报主程序，由主程序吹对应的引脚(D8 / D9)。
# 详见 src/flange_inspect/main_pipeline.py 顶部注释。
#
# ★ 产线上 systemd 只跑这一个脚本；**不要再单独跑 run_inspector.sh** ——
#   那样两个进程会抢同一块 UNO 的串口，第二个必然打不开(或写交错吹错件)。
#   run_inspector.sh 只在单站调试(不接执行器)时用。
#
# 用法:
#   ./run_pipeline.sh                          # 产线: 两站全跑, 相机模式(各站自己带 --mode camera)
#   ./run_pipeline.sh --dry-uno                # 不连串口, 只打印该吹哪个引脚(无硬件验链路)
#   ./run_pipeline.sh --only front             # 只跑正反面站(调试单站)
#   ./run_pipeline.sh --only front --mode local --dir datasets/flange --exit-when-done
#                                              # 离线喂样本: 未知参数原样透传给检测子进程
#   ./run_pipeline.sh --status-interval 30     # 每 30s 打一行监督摘要
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

# 中文日志防乱码(Linux 默认多为 UTF-8, 显式设置以防 C/POSIX locale 编码报错)。
# 主程序 spawn 子进程时会自己再注入一遍, 这里设置是为了主程序自身的日志。
export PYTHONUTF8=1
export PYTHONIOENCODING=UTF-8

# 以「脚本方式」而非 -m 运行: 让 src/flange_inspect 进入 sys.path[0], 保证
# `import uno_relay` / `import inspector_pure` / `import station_link` 这类同目录导入
# 可解析(-m 从仓库根跑会解析失败)。主程序 spawn 子进程时同样用脚本路径。
SCRIPT="src/flange_inspect/main_pipeline.py"

# 参数全部原样透传给主程序: 它用 parse_known_args, 认得的自己吃掉(如 --only/--dry-uno),
# 认不得的原样转给各检测子进程(如 --mode local --dir ...)。所以这里不做任何默认值注入。
exec "$PY" "$SCRIPT" "$@"
