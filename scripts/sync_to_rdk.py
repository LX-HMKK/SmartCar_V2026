#!/usr/bin/env python3
"""sync_to_rdk.py - 本机 <-> RDK X5 工作空间同步（rsync over ssh 封装）。

子命令：
  push         本机 src/ + config/ -> RDK /root/ros2_ws/{src,config}/（--delete 镜像）
  pull         RDK /root/ros2_ws/src/ -> 本机 src/（反向，慎用）
  init-vendor  一次性回传官方 origincar 与第三方 obstacle_detector_2 到本机
  setup        scp source_env.sh 到 RDK ~/

环境：本机需 rsync（choco install rsync）+ 免密 ssh key 到 root@192.168.128.10。
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

# ---- 配置 ----
HOST = "root@192.168.128.10"
REMOTE_WS = "/root/ros2_ws"
REMOTE_VENDOR_ORIGINCAR = "/userdata/dev_ws/src/origincar"
REMOTE_VENDOR_OBSTACLE = "/root/ros2_ws/src/obstacle_detector_2"

REPO_ROOT = Path(__file__).resolve().parent.parent
LOCAL_SRC = REPO_ROOT / "src"
LOCAL_CONFIG = REPO_ROOT / "config"
LOCAL_VENDOR_ORIGINCAR = LOCAL_SRC / "origincar"
LOCAL_THIRD_PARTY = LOCAL_SRC / "third_party"
LOCAL_OBSTACLE = LOCAL_THIRD_PARTY / "obstacle_detector_2"
LOCAL_SOURCE_ENV = REPO_ROOT / "scripts" / "source_env.sh"

EXCLUDES = [
    "**/.git/", "build/", "install/", "log/",
    "__pycache__/", "*.pyc", ".vscode/", ".idea/",
    "*.bak", "*.orig",
]
VENDOR_EXCLUDES = ["**/.git", "build/", "install/", "log/"]


def build_excludes(excludes=None):
    """返回 rsync --exclude 参数列表。"""
    if excludes is None:
        excludes = EXCLUDES
    args = []
    for ex in excludes:
        args.extend(["--exclude", ex])
    return args


def build_rsync_args(src, dst, delete=True, dry_run=False, excludes=None):
    """构建 rsync 参数（不含 'rsync' 前缀）。"""
    args = ["-avz", "--itemize-changes", "-e", "ssh"]
    if delete:
        args.append("--delete")
    if dry_run:
        args.append("--dry-run")
    args += build_excludes(excludes)
    args += [str(src), str(dst)]
    return args


def check_push_safety(path=None):
    """push 前安全检查：src/origincar 必须存在，防止空同步清空 RDK。"""
    if path is None:
        path = LOCAL_VENDOR_ORIGINCAR
    return path.is_dir()


def ensure_rsync_available():
    """检查 rsync 可用，不可用则报错退出。"""
    if shutil.which("rsync") is None:
        print("错误：未找到 rsync。Windows 请运行 choco install rsync（需管理员），"
              "或使用 WSL 的 rsync。", file=sys.stderr)
        sys.exit(2)


def push_targets():
    """push 的 (src, dst) 列表。"""
    return [
        (str(LOCAL_SRC) + "/", f"{HOST}:{REMOTE_WS}/src/"),
        (str(LOCAL_CONFIG) + "/", f"{HOST}:{REMOTE_WS}/config/"),
    ]


def pull_targets():
    """pull 的 (src, dst) 列表。"""
    return [(f"{HOST}:{REMOTE_WS}/src/", str(LOCAL_SRC) + "/")]


def init_vendor_targets():
    """init-vendor 的 (src, dst) 列表。"""
    return [
        (f"{HOST}:{REMOTE_VENDOR_ORIGINCAR}/", str(LOCAL_VENDOR_ORIGINCAR) + "/"),
        (f"{HOST}:{REMOTE_VENDOR_OBSTACLE}/", str(LOCAL_OBSTACLE) + "/"),
    ]


def build_parser():
    parser = argparse.ArgumentParser(
        description="本机 <-> RDK X5 工作空间同步（rsync 封装）")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("push", help="本机 src/+config/ -> RDK（--delete 镜像）")
    p.add_argument("--dry-run", action="store_true", help="预览不实际传输")
    p.set_defaults(func="push")

    p = sub.add_parser("pull", help="RDK src/ -> 本机（反向，慎用）")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--no-delete", action="store_true", help="不删除本机多余文件")
    p.set_defaults(func="pull")

    p = sub.add_parser("init-vendor", help="一次性回传官方包与第三方")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func="init_vendor")

    p = sub.add_parser("setup", help="scp source_env.sh 到 RDK ~/")
    p.set_defaults(func="setup")

    return parser
