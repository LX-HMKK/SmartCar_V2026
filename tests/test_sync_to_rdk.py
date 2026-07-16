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


if __name__ == "__main__":
    unittest.main()
