#!/usr/bin/env python3
"""sync_to_rdk.py - 本机 <-> RDK X5 工作空间同步（rsync over ssh 封装）。

子命令：
  push         本机 src/config 和 RDK 运行脚本 -> RDK（源码树 --delete 镜像）
  pull         RDK /home/sunrise/ros2_ws/src/ -> 本机 src/（反向，慎用）
  pull-waypoints  仅回传 RDK 上现场微调后的语义航点 YAML
  init-vendor  一次性回传官方 origincar 到本机
  setup        scp source_env.sh 到 RDK 工作空间 scripts/

环境：
  - 免密 ssh key 到 RDK；默认 root@192.168.128.10。DHCP 地址可通过
    SMARTCAR_RDK_HOST 覆盖（接受 <ip> 或 root@<ip>）。
  - 本机需安装 rsync。
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# ---- 配置 ----
DEFAULT_HOST = "root@192.168.128.10"


def resolve_rdk_host():
    """Return the configured RDK SSH target, defaulting to the wired address."""
    configured = os.environ.get("SMARTCAR_RDK_HOST", "").strip()
    if not configured:
        return DEFAULT_HOST
    return configured if "@" in configured else f"root@{configured}"


HOST = resolve_rdk_host()
REMOTE_WS = "/home/sunrise/ros2_ws"
REMOTE_VENDOR_ORIGINCAR = "/userdata/dev_ws/src/origincar"

REPO_ROOT = Path(__file__).resolve().parent.parent
LOCAL_SRC = REPO_ROOT / "src"
LOCAL_CONFIG = REPO_ROOT / "config"
LOCAL_VENDOR_ORIGINCAR = LOCAL_SRC / "origincar"
LOCAL_SOURCE_ENV = REPO_ROOT / "scripts" / "source_env.sh"
RUNTIME_SCRIPT_TARGETS = (
    (REPO_ROOT / "scripts" / "source_env.sh",
     f"{REMOTE_WS}/scripts/source_env.sh"),
    (REPO_ROOT / "scripts" / "nav_prepare.sh",
     f"{REMOTE_WS}/scripts/nav_prepare.sh"),
    (REPO_ROOT / "scripts" / "nav_test.sh",
     f"{REMOTE_WS}/scripts/nav_test.sh"),
    (REPO_ROOT / "scripts" / "competition_mode.sh",
     f"{REMOTE_WS}/scripts/competition_mode.sh"),
    (REPO_ROOT / "scripts" / "competition_launch.py",
     f"{REMOTE_WS}/scripts/competition_launch.py"),
    (REPO_ROOT / "scripts" / "media_test.sh",
     f"{REMOTE_WS}/scripts/media_test.sh"),
    (REPO_ROOT / "scripts" / "nav_status.py",
     f"{REMOTE_WS}/scripts/nav_status.py"),
    (REPO_ROOT / "scripts" / "ros_cleanup.sh",
     f"{REMOTE_WS}/scripts/ros_cleanup.sh"),
    (REPO_ROOT / "scripts" / "nav_deploy.sh",
     f"{REMOTE_WS}/scripts/nav_deploy.sh"),
    (REPO_ROOT / "scripts" / "sync_to_rdk.py",
     f"{REMOTE_WS}/scripts/sync_to_rdk.py"),
)
WAYPOINTS_RELATIVE_PATH = Path(
    "src/smartcar_nav2/config/waypoints/default_waypoints.yaml")
LOCAL_WAYPOINTS = REPO_ROOT / WAYPOINTS_RELATIVE_PATH
REMOTE_WAYPOINTS = f"{REMOTE_WS}/{WAYPOINTS_RELATIVE_PATH.as_posix()}"

EXCLUDES = [
    "**/.git/", "build/", "install/", "log/",
    "__pycache__/", "**/.pytest_cache/", "*.pyc", ".vscode/", ".idea/",
    "*.bak", "*.orig",
    # This ignored RDK-local file contains the Volcengine credential. It must
    # neither be copied from this workspace nor removed by push --delete.
    "volcengine_ark.local.yaml",
]
VENDOR_EXCLUDES = [
    "**/.git", "build/", "install/", "log/",
    "__pycache__/", "*.pyc",
    "*.deb",  # 预编译二进制包，不纳入源码 VCS
    # ydlidar_ros2_driver 经 find_package(ydlidar_sdk) 链接系统已装库，不需 src 内 SDK 源码；
    # 该 C SDK 约 20M、上游可得，不纳入 VCS（spec §7 已记录此偏离）
    "YDLidar-SDK-master/",
]

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


def is_remote_path(p):
    """判断是否为 rsync 远程路径（host:path 或 user@host:path）。

    冒号前为单字母（盘符 D:）视为本地；多字符且无路径分隔符视为远程主机。
    """
    if ":" not in p:
        return False
    head = p.split(":", 1)[0]
    if len(head) == 1 and head.isalpha():
        return False  # Windows 盘符
    if head == "":
        return False
    return "/" not in head and "\\" not in head


def build_exec_command(args):
    """Construct the native rsync command line."""
    return ["rsync"] + list(args)


def check_push_safety(path=None):
    """push 前安全检查：被 --delete 镜像的关键子树必须存在且非空，防止空/部分同步清空 RDK。

    传 path 时仅校验该路径（单元测试用）；不传则校验全部被镜像子树。
    """
    if path is not None:
        return path.is_dir() and any(path.iterdir())
    required = [LOCAL_VENDOR_ORIGINCAR, LOCAL_CONFIG]
    missing = [str(p) for p in required if not (p.is_dir() and any(p.iterdir()))]
    missing.extend(
        str(path) for path, _ in RUNTIME_SCRIPT_TARGETS if not path.is_file())
    if missing:
        print(f"错误：关键子树缺失或为空：{missing}，拒绝 push --delete（防清空 RDK）。"
              "请确认本地工作区完整后重试。", file=sys.stderr)
        return False
    return True


def ensure_local_dest_parent(dst):
    """若 dst 为本地路径，确保其父目录存在（rsync 不创建中间父目录）。"""
    if is_remote_path(dst):
        return
    Path(dst).parent.mkdir(parents=True, exist_ok=True)


def ensure_rsync_available():
    """Exit with a clear error when native rsync is unavailable."""
    if shutil.which("rsync") is None:
        print("错误：未找到 rsync。", file=sys.stderr)
        sys.exit(2)


def push_targets():
    """push 的源码目录和运行时脚本白名单。"""
    targets = [
        (str(LOCAL_SRC) + "/", f"{HOST}:{REMOTE_WS}/src/"),
        (str(LOCAL_CONFIG) + "/", f"{HOST}:{REMOTE_WS}/config/"),
    ]
    targets.extend(
        (str(path), f"{HOST}:{remote_path}")
        for path, remote_path in RUNTIME_SCRIPT_TARGETS
    )
    return targets


def pull_targets():
    """pull 的 (src, dst) 列表。"""
    return [(f"{HOST}:{REMOTE_WS}/src/", str(LOCAL_SRC) + "/")]


def init_vendor_targets():
    """init-vendor 的 (src, dst) 列表。"""
    return [(f"{HOST}:{REMOTE_VENDOR_ORIGINCAR}/",
             str(LOCAL_VENDOR_ORIGINCAR) + "/")]


def build_parser():
    parser = argparse.ArgumentParser(
        description="本机 <-> RDK X5 工作空间同步（rsync 封装）")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser(
        "push", help="本机 src/config 和运行脚本 -> RDK（源码树 --delete 镜像）")
    p.add_argument("--dry-run", action="store_true", help="预览不实际传输")
    p.set_defaults(func="push")

    p = sub.add_parser("pull", help="RDK src/ -> 本机（反向，慎用）")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--delete", action="store_true", help="删除本机不在 RDK 上的文件（默认不删，本机为权威源）")
    p.set_defaults(func="pull")

    p = sub.add_parser(
        "pull-waypoints",
        help="仅回传 RDK 上现场微调后的 default_waypoints.yaml",
    )
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func="pull_waypoints")

    p = sub.add_parser("init-vendor", help="一次性回传官方 origincar")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func="init_vendor")

    p = sub.add_parser("setup", help="scp source_env.sh 到 RDK 工作空间 scripts/")
    p.set_defaults(func="setup")

    return parser


def run_rsync(src, dst, delete=True, dry_run=False, excludes=None):
    """Execute rsync over SSH from the local Ubuntu host."""
    if not dry_run:
        ensure_local_dest_parent(dst)
    args = build_rsync_args(src, dst, delete=delete, dry_run=dry_run, excludes=excludes)
    cmd = build_exec_command(args)
    print("$ " + " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"rsync 失败（退出码 {result.returncode}）", file=sys.stderr)
        sys.exit(result.returncode)


def cmd_push(args):
    ensure_rsync_available()
    if not check_push_safety():
        sys.exit(1)
    for src, dst in push_targets():
        run_rsync(src, dst, delete=True, dry_run=args.dry_run)


def cmd_pull(args):
    ensure_rsync_available()
    delete = args.delete
    for src, dst in pull_targets():
        run_rsync(src, dst, delete=delete, dry_run=args.dry_run)


def cmd_pull_waypoints(args):
    ensure_rsync_available()
    run_rsync(
        f"{HOST}:{REMOTE_WAYPOINTS}",
        str(LOCAL_WAYPOINTS),
        delete=False,
        dry_run=args.dry_run,
        excludes=[],
    )


def cmd_init_vendor(args):
    ensure_rsync_available()
    for src, dst in init_vendor_targets():
        run_rsync(src, dst, delete=False, dry_run=args.dry_run,
                  excludes=VENDOR_EXCLUDES)
    print("回传完成。请检查后 git add src/ && git commit。")


def cmd_setup(args):
    if not LOCAL_SOURCE_ENV.is_file():
        print(f"错误：{LOCAL_SOURCE_ENV} 不存在", file=sys.stderr)
        sys.exit(1)
    cmd = ["scp", str(LOCAL_SOURCE_ENV), f"{HOST}:{REMOTE_WS}/scripts/source_env.sh"]
    print("$ " + " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        sys.exit(result.returncode)
    print(f"已部署到 {HOST}:{REMOTE_WS}/scripts/source_env.sh，RDK 上 source {REMOTE_WS}/scripts/source_env.sh 即可。")


_DISPATCH = {
    "push": cmd_push,
    "pull": cmd_pull,
    "pull_waypoints": cmd_pull_waypoints,
    "init_vendor": cmd_init_vendor,
    "setup": cmd_setup,
}


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    _DISPATCH[args.func](args)


if __name__ == "__main__":
    main()
