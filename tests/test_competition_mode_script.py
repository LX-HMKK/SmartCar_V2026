"""Static contracts for the supervised competition launcher."""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "competition_mode.sh"
CLEANUP = ROOT / "scripts" / "ros_cleanup.sh"


class CompetitionModeScriptTests(unittest.TestCase):
    def setUp(self):
        self.source = SCRIPT.read_text(encoding="utf-8")

    def function_body(self, name, next_name):
        start = self.source.index(f"{name}() {{")
        end = self.source.index(f"{next_name}() {{", start)
        return self.source[start:end]

    def test_exposes_a_two_phase_manual_start_interface(self):
        self.assertIn("competition_mode.sh prepare [--lazy-qr]", self.source)
        self.assertIn("competition_mode.sh arm", self.source)
        self.assertIn("competition_mode.sh start --confirm", self.source)
        self.assertIn("competition_mode.sh status", self.source)
        self.assertIn("competition_mode.sh stop", self.source)
        self.assertIn('if [[ $# -ne 1 || "$1" != "--confirm" ]]', self.source)
        self.assertIn('case "${1:-}" in', self.source)

    def test_prepare_keeps_motion_stopped_and_uses_one_aurora_stack(self):
        self.assertIn("camera_driver:=aurora", self.source)
        self.assertIn("use_camera:=true use_vision:=true", self.source)
        self.assertIn("use_depth_camera:=true", self.source)
        self.assertIn("localization_profile:=wheel_imu", self.source)
        self.assertIn(
            'VISION_CONFIG="$WORKSPACE/src/smartcar_vision/config/'
            'vision_volcengine.yaml"',
            self.source,
        )
        self.assertIn('vision_config_file:="$VISION_CONFIG"', self.source)
        self.assertIn("autostart_mission:=false", self.source)
        self.assertIn("safety_emergency_stop_on_start:=true", self.source)
        self.assertIn("supervised_competition_mode:=true", self.source)
        self.assertIn("continue_after_qr_failure:=true", self.source)
        self.assertIn("c_zone_direction:=counterclockwise", self.source)
        self.assertIn("preload_qr_reader:=", self.source)
        self.assertIn(
            'QR_READER_MODE_FILE="$STATE_DIR/qr_reader_preloaded"',
            self.source,
        )
        self.assertIn("read_preloaded_qr_reader_mode", self.source)
        self.assertIn("assert_no_competing_ros_stack", self.source)
        self.assertIn("ros2 node list", self.source)
        self.assertIn("--vision-services", self.source)
        self.assertIn("--preloaded-qr-reader", self.source)
        self.assertIn('VLM_CREDENTIALS_FILE="$WORKSPACE/config/', self.source)
        self.assertIn("require_vlm_credentials", self.source)
        self.assertIn("loginctl show-seat seat0", self.source)
        self.assertIn("getent passwd", self.source)
        self.assertIn("use_visualization:=false", self.source)
        self.assertNotIn("rviz2", self.source)
        self.assertNotIn("imshow", self.source)
        self.assertNotIn("rgb_imshow", self.source)
        self.assertNotIn("qr_test.launch.py", self.source)
        self.assertNotIn("vlm_test.launch.py", self.source)

    def test_display_uses_the_active_seat_xauthority_before_fallbacks(self):
        display = self.function_body("configure_display", "pid_is_running")
        active_session = display.index(
            "loginctl show-seat seat0 -p ActiveSession --value")
        active_user = display.index(
            'loginctl show-session "$desktop_session" -p Name --value')
        active_home = display.index('getent passwd "$desktop_user"')
        active_authority = display.index('export XAUTHORITY="$desktop_home/.Xauthority"')
        lightdm_fallback = display.index("/var/run/lightdm/root/:0")
        root_fallback = display.index("/root/.Xauthority")

        self.assertIn('if [[ "$DISPLAY" == :0 ]]', display)
        self.assertLess(active_session, active_user)
        self.assertLess(active_user, active_home)
        self.assertLess(active_home, active_authority)
        self.assertLess(active_authority, lightdm_fallback)
        self.assertLess(lightdm_fallback, root_fallback)

    def test_prepare_prearms_before_start_releases_the_stop(self):
        prearm = self.function_body("arm_prepared_stack", "hold_after_start_failure")
        start = self.function_body("start", "status")
        self.assertIn("verify_stack_health", prearm)
        self.assertIn("reset_task_origin", prearm)
        self.assertIn("write_armed_launch_identity", prearm)
        self.assertLess(
            prearm.index("verify_stack_health"),
            prearm.index("set_software_emergency_stop true"),
        )
        self.assertLess(
            prearm.index("set_software_emergency_stop true"),
            prearm.index("reset_task_origin"),
        )
        self.assertLess(
            prearm.index("reset_task_origin"),
            prearm.index("write_armed_launch_identity"),
        )
        self.assertIn("stack_is_armed", start)
        self.assertNotIn("verify_stack_health", start)
        self.assertNotIn("reset_task_origin", start)
        release = start.index("set_software_emergency_stop false")
        trigger = start.index("/smartcar/task/start")
        self.assertLess(release, trigger)
        self.assertIn("hold_after_start_failure", self.source)
        self.assertIn("set_software_emergency_stop true", self.source)
        self.assertIn("handle_start_interruption", self.source)
        self.assertIn("trap 'handle_start_interruption INT' INT", self.source)
        self.assertIn("trap 'handle_start_interruption TERM' TERM", self.source)
        self.assertIn("trap 'handle_start_interruption HUP' HUP", self.source)
        self.assertIn("/smartcar/task/stop", self.source)
        arm_interrupt = start.index("trap 'handle_start_interruption INT' INT")
        clear_interrupt = start.index("trap - INT TERM HUP")
        self.assertLess(arm_interrupt, start.index("set_software_emergency_stop false"))
        self.assertLess(start.index("/smartcar/task/start"), clear_interrupt)

        failure_hold = self.function_body(
            "hold_after_start_failure", "handle_start_interruption")
        interruption_hold = self.function_body("handle_start_interruption", "prepare")
        self.assertLess(
            failure_hold.index("set_software_emergency_stop true"),
            failure_hold.index("request_task_stop"),
        )
        self.assertLess(
            interruption_hold.index("set_software_emergency_stop true"),
            interruption_hold.index("request_task_stop"),
        )

    def test_prepare_rejects_a_stale_stack_and_stop_always_relatches(self):
        stale_check = self.function_body(
            "assert_no_competing_ros_stack", "set_software_emergency_stop")
        self.assertIn("timeout 5s ros2 node list", stale_check)
        self.assertIn("refuse to prepare over an unknown stack", stale_check)
        for node in (
            "/direction_guard",
            "/lifecycle_manager_navigation",
            "/depth_pointcloud_relay",
            "/depth_pointcloud_to_laserscan",
            "/vision_node",
            "/barcode_reader",
        ):
            with self.subTest(node=node):
                self.assertIn(node, stale_check)

        prepare_body = self.function_body("prepare", "arm")
        prepare = prepare_body.index("assert_no_competing_ros_stack")
        launch = prepare_body.index("nohup ros2 launch")
        self.assertLess(prepare, launch)
        self.assertIn(
            "arm_prepared_stack \"$launch_pid\" \"$preload_qr\" 75",
            prepare_body,
        )
        stop = self.source.index("stop() {")
        cleanup = self.source.index('bash "$WORKSPACE/scripts/ros_cleanup.sh"')
        self.assertLess(stop, cleanup)
        self.assertIn("set_software_emergency_stop true || true", self.source[stop:cleanup])

    def test_only_starts_the_unified_competition_output_ui(self):
        self.assertIn("competition_output_display", self.source)
        self.assertIn('UI_PID_FILE="$STATE_DIR/output_ui.pid"', self.source)
        self.assertIn("-p fullscreen:=true", self.source)
        self.assertIn("-p remote_start_enabled:=true", self.source)
        self.assertIn(
            '-p remote_start_command:="$WORKSPACE/scripts/competition_mode.sh"',
            self.source,
        )
        self.assertNotIn("vlm_display", self.source)

    def test_cleanup_tracks_competition_stack_and_ui(self):
        source = CLEANUP.read_text(encoding="utf-8")
        self.assertIn("COMPETITION_STATE_DIR=/tmp/smartcar_competition", source)
        self.assertIn('$COMPETITION_STATE_DIR/launch.pid', source)
        self.assertIn('$COMPETITION_STATE_DIR/output_ui.pid', source)
        self.assertIn(
            'rm -f "$COMPETITION_STATE_DIR/qr_reader_preloaded"',
            source,
        )
        self.assertIn(
            'rm -f "$COMPETITION_STATE_DIR/armed_launch.pid"',
            source,
        )


if __name__ == "__main__":
    unittest.main()
