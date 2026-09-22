# -*- coding: utf-8 -*-
"""一键安装本项目的 Python 运行依赖。

优先使用 uv(仓库自带 pyproject.toml + uv.lock，环境最可复现)；
检测不到 uv 时自动退回 pip + requirements.txt。跨平台，Windows/Linux/macOS 均可。

用法:
    python install.py            # 装核心 + 全部可选依赖(http/plc)
    python install.py --core     # 只装核心运行依赖，跳过 http/plc 可选
    python install.py --no-uv    # 强制走 pip，不用 uv

Windows 无命令行经验者可直接双击 install.bat。

注意: bcorner 是随仓库附带的已编译 C++ 扩展(.pyd)，不经 pip 安装；
本脚本只负责纯 Python 依赖。inspector_pure.py 不需要 bcorner。
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys

MIN_PY = (3, 11)
ROOT = os.path.dirname(os.path.abspath(__file__))


def info(msg: str) -> None:
    print("[INFO] " + msg)


def die(msg: str) -> None:
    print("[ERROR] " + msg)
    sys.exit(1)


def run(cmd: list[str]) -> int:
    print("[RUN ] " + " ".join(cmd))
    return subprocess.call(cmd, cwd=ROOT)


def main() -> int:
    ap = argparse.ArgumentParser(description="一键安装本项目 Python 依赖")
    ap.add_argument("--core", action="store_true",
                    help="只装核心运行依赖，跳过 http/plc 可选依赖")
    ap.add_argument("--no-uv", action="store_true",
                    help="强制走 pip，不使用 uv")
    args = ap.parse_args()

    # 1) 校验 Python 版本
    if sys.version_info < MIN_PY:
        die("需要 Python %d.%d 及以上，当前为 %s。请升级后重试。"
            % (MIN_PY[0], MIN_PY[1], platform.python_version()))
    info("Python %s  (%s)" % (platform.python_version(), sys.executable))

    # 2) 优先 uv 路径(读 pyproject.toml + uv.lock，环境最可复现)
    uv = None if args.no_uv else shutil.which("uv")
    has_pyproject = os.path.exists(os.path.join(ROOT, "pyproject.toml"))
    if uv and has_pyproject:
        info("检测到 uv，使用 uv sync 安装(可复现锁定环境)")
        cmd = [uv, "sync"]
        if not args.core:
            cmd += ["--extra", "http", "--extra", "plc"]
        rc = run(cmd)
        if rc != 0:
            die("uv sync 失败(退出码 %d)。可加 --no-uv 改用 pip 重试。" % rc)
        info("依赖安装完成。运行示例: uv run python -m src.flange_inspect.inspector_pure")
        return 0

    # 3) pip 退路
    req = "requirements.txt"
    req_path = os.path.join(ROOT, req)
    if not os.path.exists(req_path):
        die("找不到 %s，无法用 pip 安装。" % req)
    info("使用 pip 安装 %s" % req)

    rc = run([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    if rc != 0:
        info("pip 自升级失败(退出码 %d)，继续尝试安装依赖……" % rc)

    if args.core:
        # 只装核心三件套(与 requirements.txt 的核心段同源)
        core = ["numpy>=2.4.6", "opencv-python==4.14.0.94", "pyserial>=3.5"]
        rc = run([sys.executable, "-m", "pip", "install", *core])
    else:
        rc = run([sys.executable, "-m", "pip", "install", "-r", req_path])
    if rc != 0:
        die("pip 安装失败(退出码 %d)。请检查网络或换用国内镜像源后重试，例如:\n"
            "    python install.py  之前先设:  pip config set global.index-url "
            "https://pypi.tuna.tsinghua.edu.cn/simple" % rc)

    info("依赖安装完成。运行示例: python -m src.flange_inspect.inspector_pure")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
