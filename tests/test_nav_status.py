"""Pure contracts for the aggregated navigation startup status."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "nav_status.py"
spec = importlib.util.spec_from_file_location("nav_status", SCRIPT)
nav_status = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = nav_status
spec.loader.exec_module(nav_status)


class NavStatusContracts(unittest.TestCase):
    def test_uses_a_short_poll_interval_for_startup_discovery(self):
        self.assertEqual(nav_status.POLL_INTERVAL_SEC, 0.2)
        self.assertEqual(nav_status.PREARM_RPC_TIMEOUT_SEC, 12.0)

    def ready_depth_snapshot(self):
        return nav_status.StartupSnapshot(
            lifecycle={name: nav_status.ACTIVE_STATE_ID
                       for name in nav_status.LIFECYCLE_NODES},
            safety="emergency_stop",
            direction_guard="stopped",
            task="IDLE",
            task_services={"start": True, "reset": True},
            vision_services={"qr": True, "vlm": True},
            vision_required=True,
            qr_reader_ready=True,
            qr_reader_required=True,
            depth_status="depth_points_active",
            depth_points=2,
            depth_scan=2,
            depth_frame="depth_camera_link_1",
            scan_frame="base_footprint",
            local_costmap=1,
            global_costmap=1,
            costmap_sources={"local": "depth_scan", "global": "depth_scan"},
            costmap_topics={
                "local": "/smartcar/depth/scan",
                "global": "/smartcar/depth/scan",
            },
            capture_age_sec=0.08,
            depth_points_received_at=0.0,
            depth_scan_received_at=0.0,
            safety_requirements=nav_status.SafetyRequirements(
                require_odom=False,
                odom_timeout_sec=0.35,
                require_raw_odom=False,
                raw_odom_timeout_sec=0.25,
                require_depth_points=True,
                depth_points_timeout_sec=1.0,
                minimum_voltage=0.0,
                voltage_timeout_sec=1.0,
            ),
        )

    def test_depth_snapshot_requires_all_live_inputs(self):
        snapshot = self.ready_depth_snapshot()

        self.assertEqual(nav_status.missing_ready_items(snapshot, True), [])
        snapshot.scan_frame = "depth_camera_link_1"
        self.assertIn(
            "depth_scan_frame", nav_status.missing_ready_items(snapshot, True))

    def test_depth_snapshot_rejects_stale_or_unconsumed_inputs(self):
        snapshot = self.ready_depth_snapshot()
        snapshot.depth_scan_received_at = -1.1
        snapshot.costmap_sources["global"] = "scan"
        snapshot.capture_age_sec = 0.30

        missing = nav_status.missing_ready_items(snapshot, True)
        self.assertIn("depth_scan_fresh", missing)
        self.assertIn("global_costmap_source", missing)
        self.assertIn("depth_capture_age", missing)

    def test_snapshot_rejects_faulted_guard_or_missing_task_service(self):
        snapshot = self.ready_depth_snapshot()
        snapshot.direction_guard = "fault_candidate_timeout"
        snapshot.task_services["reset"] = False

        missing = nav_status.missing_ready_items(snapshot, True)
        self.assertIn("direction_guard=stopped", missing)
        self.assertIn("task_service=reset", missing)

    def test_competition_snapshot_requires_qr_vlm_and_preloaded_reader(self):
        snapshot = self.ready_depth_snapshot()
        self.assertEqual(
            nav_status.missing_ready_items(
                snapshot,
                True,
                require_vision=True,
                require_qr_reader=True,
            ),
            [],
        )

        snapshot.vision_services["vlm"] = False
        snapshot.qr_reader_ready = False
        missing = nav_status.missing_ready_items(
            snapshot,
            True,
            require_vision=True,
            require_qr_reader=True,
        )
        self.assertIn("vision_service=vlm", missing)
        self.assertIn("qr_reader", missing)

    def test_preloaded_reader_flag_requires_vision_service_flag(self):
        arguments = type("Args", (), {
            "timeout": 1.0,
            "launch_pid": 0,
            "rviz_pid": 0,
            "vision_services": False,
            "preloaded_qr_reader": True,
        })()
        with self.assertRaisesRegex(ValueError, "vision-services"):
            nav_status.run(arguments)

    def test_summary_exposes_all_startup_surfaces(self):
        snapshot = nav_status.StartupSnapshot(
            safety="emergency_stop", task="IDLE", local_costmap=1,
            global_costmap=1, depth_status="depth_points_active",
            depth_points=2, depth_scan=2, scan_frame="base_footprint",
            capture_age_sec=0.08,
        )

        rendered = nav_status.summary(snapshot)
        for token in ("nav2=waiting[controller_server=missing",
                      "safety=emergency_stop", "task=IDLE",
                      "costmaps=1/1", "points=2", "scan=2",
                      "capture_age=0.080s"):
            self.assertIn(token, rendered)

    def test_summary_names_each_inactive_lifecycle_node_and_state(self):
        snapshot = nav_status.StartupSnapshot(
            lifecycle={
                "controller_server": 2,
                "planner_server": 0,
                "smoother_server": 15,
                "behavior_server": nav_status.ACTIVE_STATE_ID,
            },
        )

        waiting = nav_status.inactive_lifecycle_nodes(snapshot)
        self.assertIn("controller_server=inactive", waiting)
        self.assertIn("planner_server=unknown", waiting)
        self.assertIn("smoother_server=errorprocessing", waiting)
        self.assertIn("bt_navigator=missing", waiting)
        self.assertIn("velocity_smoother=missing", waiting)
        self.assertIn("nav2=waiting[controller_server=inactive",
                      nav_status.summary(snapshot))

    def test_timed_out_rpc_is_cancelled_discarded_and_can_be_retried(self):
        class Future:
            def __init__(self):
                self.cancelled = 0

            def done(self):
                return False

            def cancel(self):
                self.cancelled += 1

        future = Future()
        pending = nav_status.PendingRpc(future=future, started_at=10.0)
        calls = {"velocity_smoother": pending}

        self.assertIsNone(nav_status.discard_timed_out_rpc(
            calls, "velocity_smoother",
            10.0 + nav_status.RPC_TIMEOUT_SEC - 0.001,
        ))
        self.assertIn("velocity_smoother", calls)
        expired = nav_status.discard_timed_out_rpc(
            calls, "velocity_smoother",
            10.0 + nav_status.RPC_TIMEOUT_SEC,
        )

        self.assertIs(expired, pending)
        self.assertNotIn("velocity_smoother", calls)
        self.assertEqual(future.cancelled, 1)

    def test_late_rpc_callback_cannot_replace_a_newer_request(self):
        old_future = object()
        new_future = object()
        old = nav_status.PendingRpc(future=old_future, started_at=1.0)
        replacement = nav_status.PendingRpc(future=new_future, started_at=2.0)
        calls = {"velocity_smoother": replacement}

        self.assertIsNone(nav_status.take_current_rpc(
            calls, "velocity_smoother", old_future))
        self.assertIs(calls["velocity_smoother"], replacement)
        self.assertIs(
            nav_status.take_current_rpc(
                calls, "velocity_smoother", new_future),
            replacement,
        )
        self.assertNotIn("velocity_smoother", calls)

    def test_timeline_log_is_private_append_only_and_single_line_safe(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "runtime" / "startup.timeline.log"
            timeline = nav_status.StartupTimelineLogger(str(log_path))
            timeline.emit("first\nsecond")
            timeline.emit("final")

            content = log_path.read_text(encoding="utf-8")
            self.assertIn("first\\nsecond", content)
            self.assertIn("final", content)
            self.assertEqual(len(content.splitlines()), 2)
            self.assertEqual(log_path.stat().st_mode & 0o077, 0)

    def test_timeline_log_option_is_optional(self):
        self.assertIsNone(nav_status.parse_args([]).timeline_log)
        self.assertEqual(
            nav_status.parse_args(
                ["--timeline-log", "/tmp/nav-startup.timeline.log"]
            ).timeline_log,
            "/tmp/nav-startup.timeline.log",
        )

    def test_prearm_option_is_opt_in(self):
        self.assertFalse(nav_status.parse_args([]).prearm)
        self.assertTrue(nav_status.parse_args(["--prearm"]).prearm)

    def test_service_response_requires_explicit_success(self):
        accepted = type("Response", (), {"success": True})()
        rejected = type("Response", (), {"success": False})()

        self.assertTrue(nav_status.service_response_succeeded(accepted))
        self.assertFalse(nav_status.service_response_succeeded(rejected))
        self.assertFalse(nav_status.service_response_succeeded(object()))


if __name__ == "__main__":
    unittest.main()
