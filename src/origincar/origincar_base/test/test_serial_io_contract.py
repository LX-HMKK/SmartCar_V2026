from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_SOURCE = PACKAGE_ROOT / "src" / "origincar_base.cpp"
BASE_LAUNCH = PACKAGE_ROOT / "launch" / "base_serial.launch.py"
ACKERMANN_ADAPTER = (
    PACKAGE_ROOT / "scripts" / "cmd_vel_to_ackermann_drive.py"
)


class SerialIoContractTests(unittest.TestCase):
    def test_sensor_reads_are_available_driven_and_bounded(self):
        source = BASE_SOURCE.read_text(encoding="utf-8")

        self.assertIn("Stm32_Serial.available()", source)
        self.assertIn("kSerialReadBudgetPerCycle", source)
        self.assertIn("bounded_serial_read_size(", source)
        self.assertIn("serial_read_timeout_ms != 0", source)
        self.assertIn('"serial_read_timeout_ms", 0', source)
        self.assertNotIn(
            "Stm32_Serial.read(\n        read_buffer.data(), "
            "read_buffer.size())",
            source,
        )

        launch_source = BASE_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "DeclareLaunchArgument('serial_read_timeout_ms', "
            "default_value='0')",
            launch_source,
        )
        self.assertIn(
            "DeclareLaunchArgument('serial_write_timeout_ms', "
            "default_value='20')",
            launch_source,
        )

    def test_backlog_is_coalesced_before_one_latest_frame_is_published(self):
        source = BASE_SOURCE.read_text(encoding="utf-8")

        self.assertIn("pop_latest_frame(", source)
        self.assertIn("latest_sensor_frame_selector_.offer(", source)
        self.assertIn("choose_pending_frame_action(", source)
        self.assertIn("PendingFrameAction::kDefer", source)
        self.assertIn("PendingFrameAction::kDiscardBacklogStale", source)
        self.assertIn("discard_backlog_stale()", source)
        self.assertIn("latest_sensor_frame_selector_.take_latest(", source)
        self.assertLess(
            source.index("choose_pending_frame_action("),
            source.index("latest_sensor_frame_selector_.take_latest("),
        )
        self.assertIn("kMaxPendingSensorFrameAgeSec", source)
        self.assertLess(
            source.index("expire_if_older_than("),
            source.index("latest_sensor_frame_selector_.take_latest("),
        )
        self.assertIn(
            "bool origincar_base::Get_Sensor_Data(rclcpp::Time & sensor_time)",
            source,
        )
        self.assertIn(
            "pending_sensor_frame_time_ = rclcpp::Node::now();",
            source,
        )
        self.assertIn("sensor_time = pending_sensor_frame_time_;", source)
        serial_tick = source[
            source.index("void origincar_base::on_serial_tick()"):
            source.index("origincar_base::origincar_base()")
        ]
        self.assertNotIn(
            "const rclcpp::Time sensor_time = rclcpp::Node::now();",
            serial_tick,
        )
        self.assertIn("Get_Sensor_Data(sensor_time)", serial_tick)

    def test_serial_diagnostics_cover_backlog_loss_and_timing(self):
        source = BASE_SOURCE.read_text(encoding="utf-8")

        for field in (
            "frame_interval_ms",
            "max_frame_interval_ms",
            "dropped=",
            "expired=",
            "backlog_stale=",
            "bad=",
            "coalescing_events=",
            "backlog_high_watermark=",
            "short_reads=",
            "short_writes=",
        ):
            with self.subTest(field=field):
                self.assertIn(field, source)

    def test_command_write_failures_latch_and_attempt_recovery(self):
        source = BASE_SOURCE.read_text(encoding="utf-8")
        write_command = source[
            source.index("void origincar_base::Write_Command()"):
            source.index("void origincar_base::Prepare_Stop_Command()")
        ]

        self.assertIn("written_size != expected_size", write_command)
        self.assertIn("Handle_Serial_Write_Failure", write_command)
        for exception in (
            "serial::IOException",
            "serial::SerialException",
            "serial::PortNotOpenedException",
        ):
            with self.subTest(exception=exception):
                self.assertIn(exception, write_command)

        read_path = source[
            source.index(
                "bool origincar_base::Get_Sensor_Data("
                "rclcpp::Time & sensor_time)"
            ):
            source.index("void origincar_base::on_serial_tick()")
        ]
        for exception in (
            "serial::IOException",
            "serial::SerialException",
            "serial::PortNotOpenedException",
        ):
            with self.subTest(read_exception=exception):
                self.assertIn(exception, read_path)

        failure_path = source[
            source.index("void origincar_base::Handle_Serial_Write_Failure"):
            source.index("void origincar_base::Handle_Serial_Read_Failure")
        ]
        self.assertIn("Prepare_Stop_Command();", failure_path)
        self.assertIn("make_fail_closed_recovery_stream", failure_path)
        self.assertIn("serial_failure_latched_ = true;", failure_path)
        self.assertIn("kSerialRecoveryWriteAttempts = 2U", source)
        self.assertIn("attempt <= kSerialRecoveryWriteAttempts", failure_path)
        self.assertIn("STM32 stop state is unverified", failure_path)
        self.assertNotIn("failed closed", failure_path.lower())
        self.assertIn("rclcpp::shutdown();", failure_path)

    def test_command_adapter_uses_keep_last_one(self):
        source = ACKERMANN_ADAPTER.read_text(encoding="utf-8")

        self.assertIn("HistoryPolicy.KEEP_LAST", source)
        self.assertRegex(source, r"depth\s*=\s*1")
        self.assertNotRegex(source, r"QoSProfile\(depth\s*=\s*10\)")


if __name__ == "__main__":
    unittest.main()
