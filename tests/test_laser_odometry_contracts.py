"""Contracts for optional scan-to-scan laser odometry integration."""
from pathlib import Path
import py_compile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
RF2O = ROOT / "src" / "third_party" / "rf2o_laser_odometry"
BRINGUP = ROOT / "src" / "smartcar_bringup"


class LaserOdometryContractTests(unittest.TestCase):
    def test_upstream_provenance_and_license_are_retained(self):
        upstream = (RF2O / "UPSTREAM.md").read_text(encoding="utf-8")
        self.assertIn("313bb4c4123bcc0cc2e042f278312b19a3c46f31", upstream)
        self.assertTrue((RF2O / "LICENSE").is_file())

    def test_node_defaults_never_compete_with_ekf_tf(self):
        header = (RF2O / "include" / "rf2o_laser_odometry" /
                  "CLaserOdometry2DNode.h").read_text(encoding="utf-8")
        source = (RF2O / "src" / "CLaserOdometry2DNode.cpp").read_text(
            encoding="utf-8")
        self.assertIn('"odom_topic", "/odom_laser"', header)
        self.assertIn('"odom_frame_id", "odom_combined"', header)
        self.assertIn('"base_frame_id", "base_footprint"', header)
        self.assertIn('"publish_tf", false', header)
        self.assertIn("publish_tf must be false", header)
        self.assertNotIn("TransformBroadcaster", header)
        self.assertNotIn("sendTransform", source)
        self.assertIn("pose_covariance_diagonal", source)
        self.assertIn("twist_covariance_diagonal", source)

        upstream_launch = (
            RF2O / "launch" / "rf2o_laser_odometry.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"publish_tf": False', upstream_launch)
        self.assertNotIn('"publish_tf": True', upstream_launch)

    def test_reset_and_invalid_scan_guards_exist(self):
        source = (RF2O / "src" / "CLaserOdometry2DNode.cpp").read_text(
            encoding="utf-8")
        self.assertIn("resetCallBack", source)
        self.assertIn("current_scan_time <= rf2o_ref.last_odom_time", source)
        self.assertIn("ranges.size() != rf2o_ref.width", source)
        self.assertIn("kMinimumScanPoints", source)
        self.assertIn("getPose().matrix().allFinite()", source)
        self.assertIn("reset(false)", source)

        core = (RF2O / "src" / "CLaserOdometry2D.cpp").read_text(
            encoding="utf-8")
        self.assertIn("laser_oldpose_ = laser_pose_", core)
        self.assertIn("robot_oldpose_ = robot_initial_pose", core)
        self.assertIn("lin_speed = 0.0", core)
        self.assertIn("ang_speed = 0.0", core)

    def test_per_scan_telemetry_is_not_logged_at_info(self):
        core = (RF2O / "src" / "CLaserOdometry2D.cpp").read_text(
            encoding="utf-8"
        )
        for message in ("execution time (ms)", "LASERodom", "BASEodom"):
            with self.subTest(message=message):
                self.assertIn(f'RCLCPP_DEBUG(get_logger(), "[rf2o] {message}', core)
                self.assertNotIn(f'RCLCPP_INFO(get_logger(), "[rf2o] {message}', core)

    def test_latest_scan_is_received_before_each_processing_cycle(self):
        node = (RF2O / "src" / "CLaserOdometry2DNode.cpp").read_text(
            encoding="utf-8"
        )
        main = node[node.index("int main("):]

        self.assertLess(
            main.index("rclcpp::spin_some(myLaserOdomNode)"),
            main.index("myLaserOdomNode->process()"),
        )
        self.assertNotIn(
            "rf2o_ref.range_wf(i) = new_scan->ranges[i]",
            node,
        )

    def test_bringup_config_is_conservative(self):
        config = yaml.safe_load(
            (BRINGUP / "config" / "laser_odometry.yaml").read_text(
                encoding="utf-8"))
        params = config["rf2o_laser_odometry"]["ros__parameters"]
        self.assertEqual(params["laser_scan_topic"], "/scan")
        self.assertEqual(params["odom_topic"], "/odom_laser")
        self.assertIs(params["publish_tf"], False)
        self.assertEqual(len(params["pose_covariance_diagonal"]), 6)
        self.assertEqual(len(params["twist_covariance_diagonal"]), 6)
        self.assertTrue(all(value > 0 for value in
                            params["pose_covariance_diagonal"]))
        self.assertTrue(all(value > 0 for value in
                            params["twist_covariance_diagonal"]))

    def test_ros_dependencies_cover_the_compiled_node(self):
        package = ET.parse(RF2O / "package.xml").getroot()
        dependencies = {
            element.text for element in package.findall("depend")
        }
        self.assertTrue({
            "eigen",
            "geometry_msgs",
            "nav_msgs",
            "rclcpp",
            "sensor_msgs",
            "std_srvs",
            "tf2",
            "tf2_geometry_msgs",
            "tf2_ros",
        }.issubset(dependencies))

        cmake = (RF2O / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("find_package(Eigen3 REQUIRED)", cmake)
        self.assertIn("target_link_libraries(${EXECUTABLE_NAME} Eigen3::Eigen)",
                      cmake)
        self.assertIn('"std_srvs"', cmake)
        self.assertNotIn("find_package(Boost", cmake)

    def test_launch_files_are_syntactically_valid(self):
        py_compile.compile(
            str(RF2O / "launch" / "rf2o_laser_odometry.launch.py"),
            doraise=True,
        )
        py_compile.compile(
            str(BRINGUP / "launch" / "laser_odometry.launch.py"),
            doraise=True,
        )


if __name__ == "__main__":
    unittest.main()
