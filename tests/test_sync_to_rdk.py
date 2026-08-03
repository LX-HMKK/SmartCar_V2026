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


class TestBuildExecCommand(unittest.TestCase):
    def test_uses_native_rsync(self):
        cmd = sync.build_exec_command(["-avz", "src/", "dst/"])
        self.assertEqual(cmd[0], "rsync")
        self.assertNotIn("wsl", cmd)


class TestPushSafety(unittest.TestCase):
    def test_returns_false_when_missing(self):
        self.assertFalse(sync.check_push_safety(Path("/nonexistent/xyz_abc")))

    def test_returns_true_when_nonempty(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, "marker").touch()
            self.assertTrue(sync.check_push_safety(Path(d)))

    def test_returns_false_for_empty_dir(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertFalse(sync.check_push_safety(Path(d)))

    def test_default_path_checks_all_subtrees(self):
        with tempfile.TemporaryDirectory() as ok1, \
                tempfile.TemporaryDirectory() as empty:
            Path(ok1, "pkg.xml").touch()
            # config 为空 -> 整体拒绝
            with mock.patch.object(sync, "LOCAL_VENDOR_ORIGINCAR", Path(ok1)), \
                    mock.patch.object(sync, "LOCAL_CONFIG", Path(empty)):
                self.assertFalse(sync.check_push_safety())
            # 两个关键树均非空 -> 通过
            Path(empty, "cfg.yaml").touch()
            self.assertTrue(sync.check_push_safety())


class TestEnsureLocalDestParent(unittest.TestCase):
    def test_creates_local_parent(self):
        with tempfile.TemporaryDirectory() as d:
            dst = str(Path(d) / "newparent" / "pkg") + "/"
            sync.ensure_local_dest_parent(dst)
            self.assertTrue((Path(d) / "newparent").is_dir())

    def test_remote_noop(self):
        # 远程路径不应触碰本地文件系统，无异常即通过
        sync.ensure_local_dest_parent("root@192.168.128.10:/root/ros2_ws/src/")


class TestRsyncAvailable(unittest.TestCase):
    def test_ensure_exits_when_missing(self):
        with mock.patch("shutil.which", return_value=None):
            with self.assertRaises(SystemExit):
                sync.ensure_rsync_available()

    def test_ensure_ok_when_present(self):
        with mock.patch("shutil.which", return_value="/usr/bin/rsync"):
            sync.ensure_rsync_available()


class TestTargets(unittest.TestCase):
    def test_rdk_host_environment_accepts_ip_or_ssh_target(self):
        with mock.patch.dict("os.environ", {"SMARTCAR_RDK_HOST": "10.0.0.2"}):
            self.assertEqual(sync.resolve_rdk_host(), "root@10.0.0.2")
        with mock.patch.dict(
                "os.environ", {"SMARTCAR_RDK_HOST": "operator@10.0.0.2"}):
            self.assertEqual(sync.resolve_rdk_host(), "operator@10.0.0.2")

    def test_push_targets_src_and_config(self):
        dsts = [t[1] for t in sync.push_targets()]
        self.assertTrue(any(d.endswith("/src/") for d in dsts))
        self.assertTrue(any(d.endswith("/config/") for d in dsts))
        self.assertTrue(all(d.startswith(sync.HOST) for d in dsts))

    def test_init_vendor_pulls_origincar_only(self):
        t = sync.init_vendor_targets()
        self.assertEqual(len(t), 1)
        source, destination = t[0]
        self.assertIn(sync.REMOTE_VENDOR_ORIGINCAR, source)
        self.assertIn("origincar", destination)

    def test_waypoint_pull_is_scoped_to_one_yaml(self):
        self.assertTrue(sync.REMOTE_WAYPOINTS.endswith("/default_waypoints.yaml"))
        self.assertEqual(
            sync.LOCAL_WAYPOINTS.name,
            "default_waypoints.yaml",
        )


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

    def test_pull_waypoints_subcommand(self):
        args = sync.build_parser().parse_args(["pull-waypoints", "--dry-run"])
        self.assertEqual(args.command, "pull-waypoints")
        self.assertTrue(args.dry_run)


class TestCmdPush(unittest.TestCase):
    def test_push_blocks_without_origincar(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch.object(sync, "check_push_safety", return_value=False), \
             mock.patch("subprocess.run") as run:
            with self.assertRaises(SystemExit):
                sync.main(["push"])
            run.assert_not_called()

    def test_push_runs_rsync_when_safe(self):
        ok = mock.Mock(returncode=0)
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch.object(sync, "check_push_safety", return_value=True), \
             mock.patch("subprocess.run", return_value=ok) as run:
            sync.main(["push", "--dry-run"])
            self.assertGreaterEqual(run.call_count, 2)  # src + config


class TestCmdSetup(unittest.TestCase):
    def test_setup_missing_env_errors(self):
        with mock.patch.object(sync, "LOCAL_SOURCE_ENV", Path("/nonexistent.sh")):
            with self.assertRaises(SystemExit):
                sync.main(["setup"])

    def test_setup_calls_scp(self):
        with tempfile.NamedTemporaryFile(suffix=".sh") as f:
            ok = mock.Mock(returncode=0)
            with mock.patch.object(sync, "LOCAL_SOURCE_ENV", Path(f.name)), \
                 mock.patch("subprocess.run", return_value=ok) as run:
                sync.main(["setup"])
                self.assertTrue(run.called)
                argv = run.call_args[0][0]
                self.assertEqual(argv[0], "scp")


class TestCmdPull(unittest.TestCase):
    def test_pull_default_no_delete(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch("subprocess.run", return_value=mock.Mock(returncode=0)) as run:
            sync.main(["pull"])
            argv = run.call_args[0][0]
            self.assertNotIn("--delete", argv)

    def test_pull_delete_opt_in(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch("subprocess.run", return_value=mock.Mock(returncode=0)) as run:
            sync.main(["pull", "--delete"])
            argv = run.call_args[0][0]
            self.assertIn("--delete", argv)

    def test_pull_waypoints_only_syncs_the_waypoint_file(self):
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch("subprocess.run", return_value=mock.Mock(returncode=0)) as run:
            sync.main(["pull-waypoints", "--dry-run"])
            argv = run.call_args[0][0]
            self.assertIn(sync.REMOTE_WAYPOINTS, " ".join(argv))
            self.assertIn("default_waypoints.yaml", " ".join(argv))
            self.assertNotIn("--delete", argv)


class TestCmdInitVendor(unittest.TestCase):
    def test_init_vendor_runs_one_rsync_with_vendor_excludes(self):
        ok = mock.Mock(returncode=0)
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch("subprocess.run", return_value=ok) as run:
            sync.main(["init-vendor"])
            self.assertEqual(run.call_count, 1)  # origincar
            for call in run.call_args_list:
                argv = call[0][0]
                self.assertIn("--exclude", argv)
                self.assertTrue(
                    any("YDLidar-SDK-master" in a or "*.deb" in a for a in argv),
                    "VENDOR_EXCLUDES 未传给 rsync")

    def test_init_vendor_exits_on_rsync_failure(self):
        fail = mock.Mock(returncode=1)
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch("subprocess.run", return_value=fail):
            with self.assertRaises(SystemExit):
                sync.main(["init-vendor"])


class TestRsyncFailure(unittest.TestCase):
    def test_push_exits_on_rsync_failure(self):
        fail = mock.Mock(returncode=23)
        with mock.patch.object(sync, "ensure_rsync_available"), \
             mock.patch.object(sync, "check_push_safety", return_value=True), \
             mock.patch("subprocess.run", return_value=fail):
            with self.assertRaises(SystemExit) as cm:
                sync.main(["push"])
            self.assertEqual(cm.exception.code, 23)


if __name__ == "__main__":
    unittest.main()
