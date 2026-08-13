"""Pure contracts for the aggregated navigation startup status."""
import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "nav_status.py"
spec = importlib.util.spec_from_file_location("nav_status", SCRIPT)
nav_status = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = nav_status
spec.loader.exec_module(nav_status)


class NavStatusContracts(unittest.TestCase):
    def ready_depth_snapshot(self):
        return nav_status.StartupSnapshot(
            lifecycle={name: nav_status.ACTIVE_STATE_ID
                       for name in nav_status.LIFECYCLE_NODES},
            safety="emergency_stop",
            direction_guard="stopped",
            task="IDLE",
            task_services={"start": True, "reset": True},
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
                require_scan=False,
                scan_timeout_sec=0.35,
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

    def test_summary_exposes_all_startup_surfaces(self):
        snapshot = nav_status.StartupSnapshot(
            safety="emergency_stop", task="IDLE", local_costmap=1,
            global_costmap=1, depth_status="depth_points_active",
            depth_points=2, depth_scan=2, scan_frame="base_footprint",
            capture_age_sec=0.08,
        )

        rendered = nav_status.summary(snapshot)
        for token in ("nav2=0/6", "safety=emergency_stop", "task=IDLE",
                      "costmaps=1/1", "points=2", "scan=2",
                      "capture_age=0.080s"):
            self.assertIn(token, rendered)


if __name__ == "__main__":
    unittest.main()
