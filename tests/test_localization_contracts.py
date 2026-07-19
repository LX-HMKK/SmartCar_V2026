import math
from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
BASE_PACKAGE = ROOT / "src" / "origincar" / "origincar_base"
EKF_FILE = BASE_PACKAGE / "config" / "ekf.yaml"
BASE_LAUNCH_FILE = BASE_PACKAGE / "launch" / "base_serial.launch.py"
BASE_HEADER_FILE = BASE_PACKAGE / "include" / "origincar_base" / "origincar_base.h"
CALIBRATION_HEADER_FILE = (
    BASE_PACKAGE / "include" / "origincar_base" / "sensor_calibration.hpp"
)
SERIAL_FRAME_HEADER_FILE = (
    BASE_PACKAGE / "include" / "origincar_base" / "serial_frame.hpp"
)
BASE_SOURCE_FILE = BASE_PACKAGE / "src" / "origincar_base.cpp"


EXPECTED_ODOM_CONFIG = [
    False, False, False,
    False, False, False,
    True, True, False,
    False, False, False,
    False, False, False,
]
EXPECTED_IMU_CONFIG = [
    False, False, False,
    False, False, False,
    False, False, False,
    False, False, True,
    False, False, False,
]
EXPECTED_LASER_ODOM_CONFIG = [
    True, True, False,
    False, False, True,
    False, False, False,
    False, False, False,
    False, False, False,
]
CALIBRATION_PARAMETERS = (
    "longitudinal_velocity_scale",
    "lateral_velocity_scale",
    "yaw_velocity_scale",
    "gyro_z_scale",
    "gyro_z_bias",
    "steering_command_scale",
    "steering_command_offset_rad",
    "max_calibrated_steering_command_rad",
)
COVARIANCE_PARAMETERS = (
    "odom_pose_covariance_diagonal",
    "odom_twist_covariance_diagonal",
    "imu_angular_velocity_covariance_diagonal",
    "imu_linear_acceleration_covariance_diagonal",
)
INTEGRATION_PARAMETERS = ("max_integration_dt_sec",)


def ekf_parameters():
    config = yaml.safe_load(EKF_FILE.read_text(encoding="utf-8"))
    return config["ekf_filter_node"]["ros__parameters"]


class EkfContractTests(unittest.TestCase):
    def test_frames_timeout_and_tf_owner_are_exact(self):
        params = ekf_parameters()
        self.assertEqual(params["sensor_timeout"], 0.25)
        self.assertIs(params["two_d_mode"], True)
        self.assertIs(params["publish_tf"], True)
        self.assertEqual(params["odom_frame"], "odom_combined")
        self.assertEqual(params["world_frame"], "odom_combined")
        self.assertEqual(params["base_link_frame"], "base_footprint")

        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertNotIn("sendTransform", source)

    def test_wheel_imu_and_optional_laser_measurements_are_fused(self):
        params = ekf_parameters()
        self.assertEqual(params["odom0"], "/odom")
        self.assertEqual(params["odom0_config"], EXPECTED_ODOM_CONFIG)
        self.assertIs(params["odom0_differential"], False)
        self.assertIs(params["odom0_relative"], False)

        self.assertEqual(params["imu0"], "/imu/data_raw")
        self.assertEqual(params["imu0_config"], EXPECTED_IMU_CONFIG)
        self.assertIs(params["imu0_differential"], False)
        self.assertIs(params["imu0_relative"], False)

        self.assertEqual(params["odom1"], "/odom_laser")
        self.assertEqual(
            params["odom1_config"], EXPECTED_LASER_ODOM_CONFIG)
        self.assertIs(params["odom1_differential"], True)
        self.assertIs(params["odom1_relative"], False)
        self.assertGreater(float(params["odom1_pose_rejection_threshold"]), 0.0)

    def test_only_declared_sensor_sources_are_configured(self):
        params = ekf_parameters()
        source_keys = {
            key for key in params
            if re.fullmatch(r"(?:odom|imu|pose|twist)\d+", key)
        }
        self.assertEqual(source_keys, {"odom0", "odom1", "imu0"})

    def test_initial_covariance_is_finite_and_not_overconfident(self):
        covariance = ekf_parameters()["initial_estimate_covariance"]
        self.assertEqual(len(covariance), 15 * 15)
        self.assertTrue(all(math.isfinite(float(value)) for value in covariance))
        for index in range(15):
            self.assertGreaterEqual(float(covariance[index * 15 + index]), 1e-3)


class BaseDriverContractTests(unittest.TestCase):
    def test_raw_odometry_frame_is_separate_from_ekf_output(self):
        launch_source = BASE_LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertIn("'odom_frame_id': 'odom'", launch_source)
        self.assertNotIn("'odom_frame_id': 'odom_combined'", launch_source)

    def test_all_calibration_launch_arguments_are_exposed(self):
        launch_source = BASE_LAUNCH_FILE.read_text(encoding="utf-8")
        for parameter in CALIBRATION_PARAMETERS + INTEGRATION_PARAMETERS:
            with self.subTest(parameter=parameter):
                self.assertIn(f"'{parameter}'", launch_source)
        self.assertIn(
            "DeclareLaunchArgument('max_integration_dt_sec', "
            "default_value='0.25')",
            launch_source,
        )

    def test_calibration_and_covariance_parameters_exist(self):
        self.assertTrue(CALIBRATION_HEADER_FILE.is_file())
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        for parameter in (
            CALIBRATION_PARAMETERS
            + COVARIANCE_PARAMETERS
            + INTEGRATION_PARAMETERS
        ):
            with self.subTest(parameter=parameter):
                self.assertIn(f'"{parameter}"', source)

    def test_internal_pose_and_velocity_storage_preserves_double_precision(self):
        header = BASE_HEADER_FILE.read_text(encoding="utf-8")
        vel_pos_struct = re.search(
            r"typedef struct __Vel_Pos_Data_(.*?)Vel_Pos_Data;",
            header,
            re.DOTALL,
        )
        self.assertIsNotNone(vel_pos_struct)
        self.assertEqual(vel_pos_struct.group(1).count("double"), 3)
        self.assertNotIn("float", vel_pos_struct.group(1))

    def test_valid_frame_is_calibrated_once_and_reused(self):
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertEqual(source.count("calibrate_sensor_sample("), 1)
        self.assertIn("constrained_lateral_velocity(", source)
        self.assertIn("integrate_planar(", source)
        self.assertRegex(source, r"Publish_ImuSensor\(sensor_time\)")
        self.assertRegex(source, r"Publish_Odom\(sensor_time\)")

    def test_integration_has_no_hidden_scale_and_skips_first_frame(self):
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertNotRegex(
            source,
            r"Robot_Pos\.X\s*\+=\s*1\.03\s*\*",
        )
        self.assertNotRegex(
            source,
            r"Robot_Pos\.Y\s*\+=\s*1\.125\s*\*",
        )
        self.assertIn("integration_clock_.update(", source)

    def test_wrapped_frames_do_not_bypass_checksum(self):
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertTrue(SERIAL_FRAME_HEADER_FILE.is_file())
        self.assertIn("sensor_frame_parser_.pop_frame(", source)
        self.assertNotIn("Header_Pos", source)
        self.assertNotIn("Tail_Pos", source)
        self.assertNotRegex(
            source,
            r"Check_Sum\(22\s*,\s*READ_DATA_CHECK\)\s*\|\|",
        )

    def test_raw_imu_orientation_is_explicitly_unavailable(self):
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertRegex(source, r"orientation\.w\s*=\s*1\.0")
        self.assertRegex(
            source,
            r"orientation_covariance\[0\]\s*=\s*-1\.0",
        )

    def test_shutdown_uses_existing_serial_and_read_failures_are_fail_closed(self):
        source = BASE_SOURCE_FILE.read_text(encoding="utf-8")
        self.assertNotIn("sigintHandler", source)
        self.assertNotIn('setPort("/dev/ttyACM0")', source)
        self.assertIn("origincar_base::~origincar_base()", source)
        self.assertIn("Send_Stop_Command();", source)
        self.assertIn("catch (const serial::IOException & error)", source)
        self.assertIn("Unable to read sensor data", source)


if __name__ == "__main__":
    unittest.main()
