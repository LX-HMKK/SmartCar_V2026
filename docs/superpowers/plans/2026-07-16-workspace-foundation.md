# 工作空间基础设施 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立本机 <-> RDK X5 单一工作空间与 rsync 同步基础设施，回传官方包入 VCS，并通过回归验证复现已知的 TF/odom/IMU/scan 行为。

**Architecture:** 本机 `src/` 为权威源（vendor 的官方 `origincar` 包 + `third_party` + 后续 `smartcar_*`），通过 rsync 镜像到 RDK `/root/ros2_ws/src/`（外加 `config/` -> `/root/ros2_ws/config/`）；RDK 只 source `tros humble` + `/root/ros2_ws`。`scripts/sync_to_rdk.py` 封装 rsync（push/pull/init-vendor/setup），`scripts/source_env.sh` 封装 source 链。

**Tech Stack:** Python 3（stdlib：argparse/subprocess/shutil）、rsync over ssh、bash、colcon、ROS2 Humble（tros）。

参考 spec：`docs/superpowers/specs/2026-07-16-workspace-foundation-design.md`

---

## Task 1: 仓库脚手架（.gitignore + 占位 + source_env.sh）

**Files:**
- Create: `.gitignore`
- Create: `config/.gitkeep`
- Create: `src/.gitkeep`
- Create: `scripts/source_env.sh`

- [ ] **Step 1: 创建 `.gitignore`**

```gitignore
# colcon 构建产物
build/
install/
log/

# Python
__pycache__/
*.pyc

# 嵌套 git（vendor 包的 .git）
**/.git/

# 编辑器/IDE
.vscode/
.idea/

# 本地工具目录
.remember/

# 备份
*.bak
*.orig
```

- [ ] **Step 2: 创建占位文件**

```bash
mkdir -p config src scripts
touch config/.gitkeep src/.gitkeep
```

- [ ] **Step 3: 创建 `scripts/source_env.sh`**

```bash
#!/usr/bin/env bash
# RDK X5 单一工作空间环境入口
# 部署：python scripts/sync_to_rdk.py setup  （scp 到 RDK ~/source_env.sh）
source /opt/tros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
```

- [ ] **Step 4: 语法检查 source_env.sh**

Run: `bash -n scripts/source_env.sh`
Expected: 无输出，退出码 0

- [ ] **Step 5: 验证 .gitignore 生效**

Run: `git status --short`
Expected: 看到 `.gitignore`、`config/.gitkeep`、`src/.gitkeep`、`scripts/source_env.sh` 待追踪；**不应**看到 `.remember/`

- [ ] **Step 6: 提交**

```bash
git add .gitignore config/.gitkeep src/.gitkeep scripts/source_env.sh
git commit -m "feat(workspace): 新增 .gitignore、占位目录与 source_env.sh"
```

---

## Task 2: sync_to_rdk.py 纯逻辑（TDD）

**Files:**
- Create: `tests/test_sync_to_rdk.py`
- Create: `scripts/sync_to_rdk.py`

- [ ] **Step 1: 写失败测试（纯逻辑部分）**

创建 `tests/test_sync_to_rdk.py`：

```python
"""sync_to_rdk.py 纯逻辑单元测试（不触发实际 rsync/网络）。"""
import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest import mock

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "sync_to_rdk.py"
spec = importlib.util.spec_from_file_location("sync_to_rdk", SCRIPT)
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


class TestExcludes(unittest.TestCase):
    def test_excludes_contains_key_patterns(self):
        for pat in ["**/.git/", "build/", "install/", "log/",
                    "__pycache__/", "*.pyc"]:
            self.assertIn(pat, sync.EXCLUDES)

    def test_build_excludes_produces_pairs(self):
        self.assertEqual(
            sync.build_excludes(["build/", "*.pyc"]),
            ["--exclude", "build/", "--exclude", "*.pyc"],
        )


class TestBuildRsyncArgs(unittest.TestCase):
    def test_push_args_include_delete_ssh_and_paths(self):
        args = sync.build_rsync_args("src/", "root@host:/ws/src/", delete=True)
        self.assertIn("--delete", args)
        self.assertIn("-e", args)
        self.assertIn("ssh", args)
        self.assertIn("src/", args)
        self.assertIn("root@host:/ws/src/", args)

    def test_no_delete_omits_flag(self):
        args = sync.build_rsync_args("a/", "b/", delete=False)
        self.assertNotIn("--delete", args)

    def test_dry_run_flag(self):
        args = sync.build_rsync_args("a/", "b/", dry_run=True)
        self.assertIn("--dry-run", args)


class TestPushSafety(unittest.TestCase):
    def test_returns_false_when_missing(self):
        self.assertFalse(sync.check_push_safety(Path("/nonexistent/xyz_abc")))

    def test_returns_true_when_exists(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertTrue(sync.check_push_safety(Path(d)))


class TestRsyncAvailable(unittest.TestCase):
    def test_ensure_exits_when_missing(self):
        with mock.patch("shutil.which", return_value=None):
            with self.assertRaises(SystemExit):
                sync.ensure_rsync_available()

    def test_ensure_ok_when_present(self):
        with mock.patch("shutil.which", return_value="/usr/bin/rsync"):
            sync.ensure_rsync_available()


class TestTargets(unittest.TestCase):
    def test_push_targets_src_and_config(self):
        dsts = [t[1] for t in sync.push_targets()]
        self.assertTrue(any(d.endswith("/src/") for d in dsts))
        self.assertTrue(any(d.endswith("/config/") for d in dsts))
        self.assertTrue(all(d.startswith(sync.HOST) for d in dsts))

    def test_init_vendor_two_pulls(self):
        t = sync.init_vendor_targets()
        self.assertEqual(len(t), 2)
        self.assertTrue(any(sync.REMOTE_VENDOR_ORIGINCAR in s for s, _ in t))
        self.assertTrue(any(sync.REMOTE_VENDOR_OBSTACLE in s for s, _ in t))
        self.assertTrue(any("origincar" in d for _, d in t))
        self.assertTrue(any("obstacle_detector_2" in d for _, d in t))


class TestParser(unittest.TestCase):
    def test_push_subcommand(self):
        args = sync.build_parser().parse_args(["push"])
        self.assertEqual(args.command, "push")
        self.assertFalse(args.dry_run)

    def test_push_dry_run(self):
        self.assertTrue(sync.build_parser().parse_args(["push", "--dry-run"]).dry_run)

    def test_no_subcommand_errors(self):
        with self.assertRaises(SystemExit):
            sync.build_parser().parse_args([])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python -m unittest tests.test_sync_to_rdk -v`
Expected: FAIL / Error（`sync_to_rdk.py` 不存在，`spec_from_file_location` 报错或属性缺失）

- [ ] **Step 3: 实现纯逻辑（创建 `scripts/sync_to_rdk.py`）**

```python
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
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python -m unittest tests.test_sync_to_rdk -v`
Expected: 全部 PASS（7 个测试方法）

- [ ] **Step 5: 提交**

```bash
git add scripts/sync_to_rdk.py tests/test_sync_to_rdk.py
git commit -m "feat(sync): 新增 sync_to_rdk.py 纯逻辑与单元测试"
```

---

## Task 3: sync_to_rdk.py 命令层与 main（TDD）

**Files:**
- Modify: `tests/test_sync_to_rdk.py`（追加 TestCmdPush）
- Modify: `scripts/sync_to_rdk.py`（追加 run_rsync / cmd_* / main）

- [ ] **Step 1: 追加失败测试**

在 `tests/test_sync_to_rdk.py` 的 `if __name__ == "__main__":` 之前追加：

```python
class TestCmdPush(unittest.TestCase):
    def test_push_blocks_without_origincar(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch.object(sync, "check_push_safety", return_value=False), \
             mock.patch("subprocess.run") as run:
            with self.assertRaises(SystemExit):
                sync.main(["push"])
            run.assert_not_called()

    def test_push_runs_rsync_when_safe(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch.object(sync, "check_push_safety", return_value=True), \
             mock.patch("subprocess.run") as run:
            sync.main(["push", "--dry-run"])
            self.assertGreaterEqual(run.call_count, 2)  # src + config


class TestCmdSetup(unittest.TestCase):
    def test_setup_missing_env_errors(self):
        with mock.patch.object(sync, "LOCAL_SOURCE_ENV", Path("/nonexistent.sh")):
            with self.assertRaises(SystemExit):
                sync.main(["setup"])

    def test_setup_calls_scp(self):
        with tempfile.NamedTemporaryFile(suffix=".sh") as f:
            with mock.patch.object(sync, "LOCAL_SOURCE_ENV", Path(f.name)), \
                 mock.patch("subprocess.run") as run:
                sync.main(["setup"])
                self.assertTrue(run.called)
                argv = run.call_args[0][0]
                self.assertEqual(argv[0], "scp")
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python -m unittest tests.test_sync_to_rdk.TestCmdPush tests.test_sync_to_rdk.TestCmdSetup -v`
Expected: FAIL（`sync.main` / `run_rsync` / `cmd_push` 等不存在）

- [ ] **Step 3: 实现命令层**

在 `scripts/sync_to_rdk.py` 末尾（`build_parser` 之后）追加：

```python
def run_rsync(src, dst, delete=True, dry_run=False, excludes=None):
    """执行 rsync over ssh。"""
    args = ["rsync"] + build_rsync_args(
        src, dst, delete=delete, dry_run=dry_run, excludes=excludes)
    print("$ " + " ".join(args))
    result = subprocess.run(args)
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
    delete = not args.no_delete
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
```

- [ ] **Step 4: 运行全部测试确认通过**

Run: `python -m unittest tests.test_sync_to_rdk -v`
Expected: 全部 PASS（含新增 TestCmdPush/TestCmdSetup）

- [ ] **Step 5: 提交**

```bash
git add scripts/sync_to_rdk.py tests/test_sync_to_rdk.py
git commit -m "feat(sync): 新增 rsync 命令层与 main 调度"
```

---

## Task 4: 安装 rsync（Windows 前置）

**Files:** 无（环境变更，不提交）

> 注意：`choco install` 需管理员权限。若 choco 的 rsync 与 OpenSSH 冲突，回退方案：在 WSL（Ubuntu-18.04）中 `sudo apt install rsync`，并用 `wsl rsync` 调用（需路径翻译 /mnt/d/...）。

- [ ] **Step 1: 安装 rsync（管理员 PowerShell）**

用户在**管理员** PowerShell 中运行：
```powershell
choco install rsync -y
```

- [ ] **Step 2: 验证本机 rsync 可用**

Run（Git Bash）: `rsync --version`
Expected: 显示 rsync 版本（如 3.x），退出码 0

- [ ] **Step 3: 验证 rsync over ssh 能连 RDK**

Run: `rsync -e ssh --dry-run root@192.168.128.10:/userdata/dev_ws/src/origincar/ /tmp/origincar-dryrun/`
Expected: 列出远端文件（dry-run），无 ssh 错误。若报 ssh/cygwin 冲突，改用 WSL 方案。

---

## Task 5: init-vendor 回传官方包入 VCS

**Files:**
- Create: `src/origincar/**`（vendor，从 RDK 拉取）
- Create: `src/third_party/obstacle_detector_2/**`（vendor，从 RDK 拉取）
- Delete: `src/.gitkeep`（被实际内容替代）

- [ ] **Step 1: 运行 init-vendor**

Run: `python scripts/sync_to_rdk.py init-vendor`
Expected: rsync 拉取 `/userdata/dev_ws/src/origincar/` -> `src/origincar/`，`/root/ros2_ws/src/obstacle_detector_2/` -> `src/third_party/obstacle_detector_2/`；无 .git 目录

- [ ] **Step 2: 验证关键文件到位**

Run: `ls src/origincar/origincar_base/launch/origincar_bringup.launch.py src/third_party/obstacle_detector_2/package.xml`
Expected: 两个文件均存在

- [ ] **Step 3: 验证无嵌套 .git**

Run: `find src -name .git -type d`
Expected: 无输出（vendor 化已去除 .git）

- [ ] **Step 4: 验证修复点在 launch 中**

Run: `grep -n "static_transform_publisher" src/origincar/origincar_base/launch/origincar_bringup.launch.py | head -3`
Expected: 命中 `--frame-id`/`--child-frame-id` 等 Humble 命名参数（确认修复已随 vendor 进入 VCS）

- [ ] **Step 5: 移除 src/.gitkeep 并提交**

```bash
git rm src/.gitkeep
git add src/origincar src/third_party
git commit -m "feat(workspace): vendor 回传官方 origincar 包与 obstacle_detector_2"
```

> 注意：此提交文件量大（官方包 + SDK）。确认 `.gitignore` 已排除 `build/ install/ log/ **/.git/`，避免误入库。

---

## Task 6: setup + push 部署与镜像

**Files:** 无（RDK 状态变更，不提交）

- [ ] **Step 1: 部署 source_env.sh 到 RDK**

Run: `python scripts/sync_to_rdk.py setup`
Expected: scp 成功，提示已部署到 `~/source_env.sh`

- [ ] **Step 2: push dry-run 预览**

Run: `python scripts/sync_to_rdk.py push --dry-run`
Expected: 列出将传输的文件（首次为全量），含 `src/origincar/`、`src/third_party/`，无报错

- [ ] **Step 3: push 实际镜像**

Run: `python scripts/sync_to_rdk.py push`
Expected: rsync 完成，`/root/ros2_ws/src/` 与 `/root/ros2_ws/config/` 镜像本机

- [ ] **Step 4: 核对 RDK 镜像**

Run: `ssh root@192.168.128.10 'ls /root/ros2_ws/src/'`
Expected: 看到 `origincar`、`third_party`（旧 `obstacle_detector_2` 已被 --delete 清除）

Run: `ssh root@192.168.128.10 'ls /root/ros2_ws/config/'`
Expected: 看到 `.gitkeep`（或后续共享配置）

- [ ] **Step 5: idempotency 检查**

Run: `python scripts/sync_to_rdk.py push --dry-run`
Expected: 几乎无文件变更（`--itemize-changes` 输出为空或极少）

---

## Task 7: 单一工作空间回归验证

**Files:** 无（RDK 构建 + 运行验证）

> 构建 RDK 上全部官方包 + SDK 可能 10-20 分钟。建议后台执行。

- [ ] **Step 1: 清理并构建**

Run（后台，长时）:
```bash
ssh root@192.168.128.10 'source ~/source_env.sh && cd /root/ros2_ws && rm -rf build install log && colcon build --symlink-install' 2>&1 | tee /tmp/rdk-build.log
```
Expected: 构建完成，`Summary: X packages finished`，无 Failed。若失败，查 `/tmp/rdk-build.log`。

- [ ] **Step 2: 重新 source 加载新构建**

Run: `ssh root@192.168.128.10 'source ~/source_env.sh && ros2 pkg list | grep -E "origincar|obstacle_detector"'`
Expected: 列出 `origincar_base`、`origincar_bringup`、`obstacle_detector` 等

- [ ] **Step 3: 后台启动 bringup**

Run:
```bash
ssh root@192.168.128.10 'nohup bash -c "source ~/source_env.sh && ros2 launch origincar_base origincar_bringup.launch.py" > /tmp/bringup.log 2>&1 &'
```
等待 5 秒让节点起来。

- [ ] **Step 4: 核对话题频率**

Run: `ssh root@192.168.128.10 'source ~/source_env.sh && timeout 5 ros2 topic hz /odom_combined'`
Expected: ~20 Hz

Run: `ssh root@192.168.128.10 'source ~/source_env.sh && timeout 5 ros2 topic hz /imu/data'`
Expected: ~20 Hz

Run: `ssh root@192.168.128.10 'source ~/source_env.sh && timeout 5 ros2 topic hz /scan'`
Expected: ~10 Hz

- [ ] **Step 5: 核对 TF 树**

Run: `ssh root@192.168.128.10 'source ~/source_env.sh && timeout 3 ros2 run tf2_ros tf2_echo odom_combined base_link'`
Expected: 输出平移/旋转，无 "could not find transform" 报错

- [ ] **Step 6: 停止 bringup**

Run: `ssh root@192.168.128.10 'pkill -f origincar_bringup; pkill -f ros2'`
Expected: 进程已终止

- [ ] **Step 7: 记录回归结果**

将话题频率与 TF 结果记入后续部署文档（或简短附在 commit message）。若任一不达标，回退 `source /userdata/dev_ws/install/setup.bash` 排查，并在 spec/plan 中记录差异。

---

## Task 8: README 最小文档

**Files:**
- Modify: `README.md`（当前为空）

- [ ] **Step 1: 写入最小 README**

```markdown
# 智慧医疗赛智能汽车

第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛。硬件：OriginCar + RDK X5 8G + ROS2 Humble。

## 工作空间

- 本机 `src/` 为权威源：`origincar/`（官方 vendor）、`third_party/`、`smartcar_*`（自研）。
- RDK 上单一工作空间 `/root/ros2_ws`，source 链见 `scripts/source_env.sh`。
- 同步：`python scripts/sync_to_rdk.py push`（详见 `scripts/sync_to_rdk.py --help`）。

## 文档

- 技术方案：`docs/智慧医疗挑战赛_技术方案_本地部署版.md`
- RDK 环境：`docs/deployment/rdk-environment-setup.md`
- 设计/计划：`docs/superpowers/specs/`、`docs/superpowers/plans/`
```

- [ ] **Step 2: 提交**

```bash
git add README.md
git commit -m "docs(readme): 补充工作空间与同步说明"
```

---

## Self-Review（已在撰写后内联修复）

**Spec 覆盖**：
- §4 目录结构 + .gitignore → Task 1
- §5 构建/source 策略 + source_env.sh → Task 1（source_env.sh）、Task 7（构建/source 验证）
- §6 同步脚本 push/pull/init-vendor/setup + 安全护栏 + 排除 → Task 2/3（含 check_push_safety、ensure_rsync_available、EXCLUDES、VENDOR_EXCLUDES）
- §7 vendor 回传 → Task 5
- §8 验证流程 → Task 6（push）+ Task 7（构建+话题+TF）
- §9 错误处理与测试 → Task 2/3 单元测试；Task 4 rsync 可用性；Task 7 端到端
- config/ 同步缺口（spec §6 注）→ Task 2 push_targets 含 config/，Task 6 Step 4 核对

**占位符扫描**：无 TBD/TODO；每步含具体命令或代码。

**类型一致**：`build_rsync_args`、`check_push_safety`、`push_targets`、`init_vendor_targets`、`build_parser`、`main` 在 Task 2/3 与测试中签名一致；`_DISPATCH` 的 key 与 `set_defaults(func=...)` 一致（push/pull/init_vendor/setup）。

**已知风险**：
- Task 4 choco rsync 与 OpenSSH 可能冲突 → 已给 WSL 回退。
- Task 5 提交量大 → 已强调 .gitignore 排除构建产物。
- Task 7 构建耗时 → 已建议后台 + tee 日志。
