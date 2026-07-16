#!/usr/bin/env python3
"""sync_to_rdk.py - 本机 <-> RDK X5 工作空间同步（rsync over ssh 封装）。

子命令：
  push         本机 src/ + config/ -> RDK /root/ros2_ws/{src,config}/（--delete 镜像）
  pull         RDK /root/ros2_ws/src/ -> 本机 src/（反向，慎用）
  init-vendor  一次性回传官方 origincar 与第三方 obstacle_detector_2 到本机
  setup        scp source_env.sh 到 RDK ~/

环境：
  - 免密 ssh key 到 root@192.168.128.10。
  - Linux/macOS：需本地 rsync。
  - Windows：choco 的 rsync 无法处理盘符路径（D: 被当作 host:path），故经 WSL 运行
    rsync，需已安装 WSL（默认 Ubuntu-22.04，可用 WSL_DISTRO 环境变量覆盖），
    且 WSL 内已配置到 RDK 的免密 ssh key。
"""
import argparse
import os
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
VENDOR_EXCLUDES = [
    "**/.git", "build/", "install/", "log/",
    "__pycache__/", "*.pyc",
    "*.deb",  # 预编译二进制包，不纳入源码 VCS
    "YDLidar-SDK-master/",  # 非 ROS 的 C SDK，RDK 已装、驱动不依赖，不纳入 VCS
]

WSL_DISTRO_DEFAULT = "Ubuntu-22.04"


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


def to_wsl_path(p):
    """将本地 Windows 路径转为 WSL /mnt/<drive>/ 形式。

    远程路径与相对路径原样返回；D:\\a\\b 与 D:/a/b 均转为 /mnt/d/a/b。
    """
    if is_remote_path(p):
        return p
    if len(p) >= 2 and p[0].isalpha() and p[1] == ":":
        drive = p[0].lower()
        rest = p[2:].replace("\\", "/")
        return f"/mnt/{drive}{rest}"
    return p


def build_exec_command(args, platform_name=None):
    """构造实际执行的命令列表。

    Windows 经 WSL 调用 rsync（本地路径转 /mnt/<drive>/）；其他平台直接 rsync。
    """
    platform_name = platform_name if platform_name is not None else sys.platform
    if platform_name.startswith(("win", "cygwin", "msys")):
        distro = os.environ.get("WSL_DISTRO", WSL_DISTRO_DEFAULT)
        translated = [to_wsl_path(a) for a in args]
        return ["wsl", "-d", distro, "--", "rsync"] + translated
    return ["rsync"] + list(args)


def check_push_safety(path=None):
    """push 前安全检查：src/origincar 必须存在，防止空同步清空 RDK。"""
    if path is None:
        path = LOCAL_VENDOR_ORIGINCAR
    return path.is_dir()


def ensure_local_dest_parent(dst):
    """若 dst 为本地路径，确保其父目录存在（rsync 不创建中间父目录）。"""
    if is_remote_path(dst):
        return
    Path(dst).parent.mkdir(parents=True, exist_ok=True)


def ensure_rsync_available():
    """检查同步工具可用，不可用则报错退出。Windows 检查 wsl，其他平台检查 rsync。"""
    if sys.platform.startswith(("win", "cygwin", "msys")):
        if shutil.which("wsl") is None:
            print("错误：未找到 wsl。Windows 上 rsync 经 WSL 运行，请启用 WSL。",
                  file=sys.stderr)
            sys.exit(2)
        return
    if shutil.which("rsync") is None:
        print("错误：未找到 rsync。", file=sys.stderr)
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
    p.add_argument("--delete", action="store_true", help="删除本机不在 RDK 上的文件（默认不删，本机为权威源）")
    p.set_defaults(func="pull")

    p = sub.add_parser("init-vendor", help="一次性回传官方包与第三方")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func="init_vendor")

    p = sub.add_parser("setup", help="scp source_env.sh 到 RDK ~/")
    p.set_defaults(func="setup")

    return parser


def run_rsync(src, dst, delete=True, dry_run=False, excludes=None):
    """执行 rsync over ssh（Windows 下经 WSL）。"""
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
        print("错误：本机 src/origincar 不存在，拒绝 push --delete（防清空 RDK）。"
              "请先运行 init-vendor。", file=sys.stderr)
        sys.exit(1)
    for src, dst in push_targets():
        run_rsync(src, dst, delete=True, dry_run=args.dry_run)


def cmd_pull(args):
    ensure_rsync_available()
    delete = args.delete
    for src, dst in pull_targets():
        run_rsync(src, dst, delete=delete, dry_run=args.dry_run)


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
    cmd = ["scp", str(LOCAL_SOURCE_ENV), f"{HOST}:~/source_env.sh"]
    print("$ " + " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        sys.exit(result.returncode)
    print(f"已部署到 {HOST}:~/source_env.sh，RDK 上 source ~/source_env.sh 即可。")


_DISPATCH = {
    "push": cmd_push,
    "pull": cmd_pull,
    "init_vendor": cmd_init_vendor,
    "setup": cmd_setup,
}


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    _DISPATCH[args.func](args)


if __name__ == "__main__":
    main()
