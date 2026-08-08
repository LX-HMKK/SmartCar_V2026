"""Unit contracts for the Gazebo-only PointCloud2 obstacle fixture."""
import importlib.util
from pathlib import Path
import struct
import unittest

from sensor_msgs.msg import LaserScan


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = (
    ROOT / "src" / "smartcar_sim" / "scripts"
    / "sim_depth_pointcloud_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("sim_depth_pointcloud_adapter", ADAPTER)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class SimDepthPointCloudAdapterTests(unittest.TestCase):
    def test_adapter_has_a_direct_script_entrypoint(self):
        source = ADAPTER.read_text(encoding="utf-8")
        self.assertIn('if __name__ == "__main__":', source)
        self.assertIn("    main()", source)

    def test_adapter_keeps_only_finite_obstacle_returns(self):
        scan = LaserScan()
        scan.header.frame_id = "origincar/laser_link/lidar_sensor"
        scan.header.stamp.sec = 42
        scan.range_min = 0.10
        scan.range_max = 4.0
        scan.angle_min = 0.0
        scan.angle_increment = 1.5707963267948966
        scan.ranges = [0.10, 0.25, 1.0, float("inf"), 3.5]

        cloud = MODULE.scan_to_depth_point_cloud(
            scan, min_range_m=0.25, max_range_m=3.5)

        self.assertEqual(cloud.header.frame_id, scan.header.frame_id)
        self.assertEqual(cloud.header.stamp.sec, 42)
        self.assertEqual(cloud.width, 1)
        self.assertEqual(cloud.height, 1)
        self.assertEqual(cloud.point_step, MODULE.XYZ_POINT_STEP)
        self.assertEqual(cloud.row_step, MODULE.XYZ_POINT_STEP)
        self.assertTrue(cloud.is_dense)
        x, y, z = struct.unpack("<fff", bytes(cloud.data))
        self.assertAlmostEqual(x, -1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.0)

    def test_adapter_can_materialize_depth_height_in_base_frame(self):
        scan = LaserScan()
        scan.header.frame_id = "laser_link"
        scan.range_min = 0.10
        scan.range_max = 4.0
        scan.angle_min = 0.0
        scan.angle_increment = 1.0
        scan.ranges = [1.0]

        cloud = MODULE.scan_to_depth_point_cloud(
            scan,
            min_range_m=0.25,
            max_range_m=3.5,
            output_frame="base_footprint",
            sensor_origin_x_m=0.0341,
            sensor_origin_y_m=-0.02,
            point_height_m=0.15,
        )

        self.assertEqual(cloud.header.frame_id, "base_footprint")
        x, y, z = struct.unpack("<fff", bytes(cloud.data))
        self.assertAlmostEqual(x, 1.0341, places=6)
        self.assertAlmostEqual(y, -0.02, places=6)
        self.assertAlmostEqual(z, 0.15, places=6)

    def test_adapter_rejects_invalid_range_limits(self):
        with self.assertRaisesRegex(ValueError, "range limits"):
            MODULE.scan_to_depth_point_cloud(
                LaserScan(), min_range_m=3.5, max_range_m=0.25)

    def test_adapter_handles_ros_shutdown_without_a_nonzero_exit_path(self):
        source = ADAPTER.read_text(encoding="utf-8")
        self.assertIn("except (KeyboardInterrupt, ExternalShutdownException)", source)
        self.assertIn("if rclpy.ok():", source)


if __name__ == "__main__":
    unittest.main()
