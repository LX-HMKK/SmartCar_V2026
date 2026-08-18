"""Semantic waypoint mission orchestration node."""

import math
import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from smartcar_task.c_zone_direction import (
    CLOCKWISE,
    COUNTERCLOCKWISE,
    apply_c_zone_direction,
    normalize_c_zone_direction,
)
from smartcar_task.mission import Mission, MissionConfig
from smartcar_task.planning_segments import (
    PlanningSegmentError,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
    select_segment_prefix,
)
from smartcar_task.ros_adapters import (
    RosDirectionGuard,
    RosLocalization,
    RosNavigator,
    RosOutput,
    RosVision,
    SystemClock,
)
from smartcar_task.route_geometry import RouteGeometryError, materialize_free_yaws
from smartcar_task.waypoints import load_waypoint_document


SUPERVISED_P_TO_A_SEGMENT_ID = "p_to_qr"
SUPERVISED_P_TO_C1_SEGMENT_ID = "qr_to_vlm"
SUPERVISED_PREFIX_TASKS = {
    SUPERVISED_P_TO_A_SEGMENT_ID: ("nav",),
    SUPERVISED_P_TO_C1_SEGMENT_ID: (
        "nav", "via", "via", "via", "via", "nav",
    ),
}


def _positive_finite(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _nav2_behavior_tree_path(filename):
    """Resolve a production behavior tree from the installed Nav2 package."""
    return str(
        Path(get_package_share_directory("smartcar_nav2"))
        / "config"
        / "behavior_trees"
        / filename
    )


def _materialize_navigation_variant(
    authored_waypoints,
    planning_segments,
    selected_segments,
):
    """Build one immutable Nav2 input variant from authored constraints."""
    ordered_waypoints = materialize_mission_route(
        authored_waypoints,
        planning_segments,
    )
    materialized_waypoints = materialize_free_yaws(ordered_waypoints)
    materialized_by_id = {
        waypoint.id: waypoint for waypoint in materialized_waypoints
    }
    authored_navigation_segments = materialize_navigation_segments(
        authored_waypoints,
        planning_segments,
    )
    navigation_segments = tuple(
        tuple(materialized_by_id[waypoint.id] for waypoint in segment)
        for segment in authored_navigation_segments[:len(selected_segments)]
    )
    return materialized_waypoints, navigation_segments


class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")
        self._declare_parameters()

        self._load_route()
        self._configure_motion_gates()
        self._build_runtime()
        self._register_services()
        self._configure_autostart()
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} semantic waypoints in "
            f"{len(self._navigation_segments)} planning segments")

    def _declare_parameters(self):
        """Declare the task node's single, explicit runtime contract."""
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("c_zone_direction", "counterclockwise")
        self.declare_parameter("supervised_competition_mode", False)
        self.declare_parameter("steering_calibrated", False)
        self.declare_parameter("emergency_stop_ready", False)
        self.declare_parameter("operator_approved", False)
        self.declare_parameter("autostart_mission", False)
        self.declare_parameter("navigation_test_end_segment_id", "")
        self.declare_parameter("supervised_p_to_a_only", False)
        self.declare_parameter("supervised_p_to_c1_only", False)
        self.declare_parameter("supervised_full_route", False)
        self.declare_parameter("qr_reader_preloaded", False)
        self.declare_parameter("server_wait_timeout_sec", 30.0)
        self.declare_parameter("navigation_timeout_sec", 120.0)
        self.declare_parameter("goal_response_timeout_sec", 2.0)
        self.declare_parameter("cancel_timeout_sec", 3.0)
        self.declare_parameter("stop_timeout_sec", 5.0)
        behavior_trees = {
            "precise_behavior_tree":
            "navigate_to_pose_precise_w_replanning_and_recovery.xml",
            "transit_behavior_tree":
            "navigate_to_pose_transit_w_replanning_and_recovery.xml",
            "through_poses_behavior_tree":
            "navigate_through_poses_w_replanning_and_recovery.xml",
            "transit_through_poses_behavior_tree":
            "navigate_through_poses_transit_w_replanning_and_recovery.xml",
            "precise_through_poses_behavior_tree":
            "navigate_through_poses_precise_w_replanning_and_recovery.xml",
            "return_through_poses_behavior_tree":
            "navigate_through_poses_return_w_replanning_and_recovery.xml",
        }
        for name, filename in behavior_trees.items():
            self.declare_parameter(name, _nav2_behavior_tree_path(filename))
        scalar_defaults = {
            "direction_service_timeout_sec": 0.20,
            "direction_prepare_timeout_sec": 1.0,
            "direction_prepare_retry_period_sec": 0.02,
            "direction_renew_period_sec": 0.10,
            "direction_stop_timeout_sec": 2.0,
            "direction_stop_dwell_sec": 0.25,
            "direction_stop_linear_tolerance": 0.01,
            "direction_stop_angular_tolerance": 0.05,
            "direction_odom_stale_timeout_sec": 0.25,
            "qr_settle_sec": 2.0,
            "qr_timeout_sec": 3.0,
            "qr_retries": 1,
            "qr_retry_delay_sec": 0.25,
            "vlm_timeout_sec": 30.0,
            "reset_timeout_sec": 5.0,
            "origin_position_tolerance": 0.20,
            "origin_yaw_tolerance": 0.20,
            "qr_reader_startup_sec": 2.0,
        }
        for name, default in scalar_defaults.items():
            self.declare_parameter(name, default)
        self.declare_parameter("barcode_reader_image_topic", "/image")

    def _load_route(self):
        waypoints_file = str(
            self.get_parameter("waypoints_file").value).strip()
        if not waypoints_file:
            raise ValueError("waypoints_file must be provided")
        self._supervised_competition_mode = bool(
            self.get_parameter("supervised_competition_mode").value)
        try:
            waypoint_document, source_waypoints = load_waypoint_document(
                waypoints_file)
            selected_c_zone_direction = normalize_c_zone_direction(
                self.get_parameter("c_zone_direction").value)
            if (
                self._supervised_competition_mode
                and selected_c_zone_direction != COUNTERCLOCKWISE
            ):
                raise ValueError(
                    "supervised competition mode requires the authored "
                    "counterclockwise baseline")
            authored_waypoints = apply_c_zone_direction(
                source_waypoints, selected_c_zone_direction)
            planning_segments = load_planning_segments(
                waypoint_document, authored_waypoints)
            selected_segments = select_segment_prefix(
                planning_segments,
                self.get_parameter("navigation_test_end_segment_id").value,
            )
            self._waypoints, self._navigation_segments = (
                _materialize_navigation_variant(
                    authored_waypoints,
                    planning_segments,
                    selected_segments,
                )
            )
            self._competition_navigation_variants = None
            if self._supervised_competition_mode:
                self._load_competition_variants(
                    waypoint_document,
                    source_waypoints,
                    planning_segments,
                    selected_segments,
                )
            self._validate_navigation_test_modes(
                planning_segments, selected_segments)
        except (PlanningSegmentError, RouteGeometryError, ValueError) as error:
            raise ValueError(f"invalid mission route: {error}") from error

    def _load_competition_variants(
        self,
        waypoint_document,
        source_waypoints,
        planning_segments,
        selected_segments,
    ):
        if selected_segments != planning_segments:
            raise ValueError(
                "supervised competition mode requires the full route")
        endpoint_tasks = tuple(
            segment[-1].task for segment in self._navigation_segments)
        if endpoint_tasks != ("qr", "vlm", "return"):
            raise ValueError(
                "supervised competition mode requires the semantic QR, "
                "VLM, and return route")
        clockwise_waypoints = apply_c_zone_direction(
            source_waypoints, CLOCKWISE)
        clockwise_planning_segments = load_planning_segments(
            waypoint_document, clockwise_waypoints)
        if clockwise_planning_segments != planning_segments:
            raise ValueError("clockwise C-zone variant changes planning segments")
        _, clockwise_navigation_segments = _materialize_navigation_variant(
            clockwise_waypoints,
            clockwise_planning_segments,
            selected_segments,
        )
        self._competition_navigation_variants = {
            COUNTERCLOCKWISE: self._navigation_segments,
            CLOCKWISE: clockwise_navigation_segments,
        }

    def _validate_navigation_test_modes(self, planning_segments, selected_segments):
        supervised_prefixes = tuple(
            segment_id
            for parameter_name, segment_id in (
                ("supervised_p_to_a_only", SUPERVISED_P_TO_A_SEGMENT_ID),
                ("supervised_p_to_c1_only", SUPERVISED_P_TO_C1_SEGMENT_ID),
            )
            if bool(self.get_parameter(parameter_name).value)
        )
        if len(supervised_prefixes) > 1:
            raise ValueError("only one supervised navigation prefix may be enabled")
        supervised_full_route = bool(
            self.get_parameter("supervised_full_route").value)
        if supervised_full_route and supervised_prefixes:
            raise ValueError(
                "supervised full route and prefix cannot be combined")
        self._supervised_navigation_test = bool(
            supervised_prefixes) or supervised_full_route
        if self._supervised_competition_mode and self._supervised_navigation_test:
            raise ValueError(
                "supervised competition and navigation test modes conflict")
        selected_segment_ids = tuple(
            segment.id for segment in selected_segments)
        selected_tasks = tuple(
            waypoint.task
            for segment in self._navigation_segments
            for waypoint in segment)
        if supervised_prefixes:
            expected_segment_id = supervised_prefixes[0]
            expected_ids = tuple(
                segment.id for segment in planning_segments[:len(selected_segments)])
            if (
                selected_segment_ids != expected_ids
                or selected_segment_ids[-1] != expected_segment_id
                or selected_tasks != SUPERVISED_PREFIX_TASKS[expected_segment_id]
            ):
                raise ValueError(
                    "supervised navigation test requires the fixed "
                    "pure-navigation route prefix")
        if supervised_full_route:
            route_segment_ids = tuple(
                segment.id for segment in planning_segments)
            if (
                selected_segment_ids != route_segment_ids
                or any(
                    task not in {"start", "nav", "via", "return"}
                    for task in selected_tasks)
            ):
                raise ValueError(
                    "supervised full route requires the complete "
                    "pure-navigation route")

    def _configure_motion_gates(self):
        self._motion_gates = {
            name: bool(self.get_parameter(name).value)
            for name in (
                "steering_calibrated",
                "emergency_stop_ready",
                "operator_approved",
            )
        }
        self._stop_timeout_sec = _positive_finite(
            "stop_timeout_sec",
            self.get_parameter("stop_timeout_sec").value)

    def _build_runtime(self):
        self._io_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._output = RosOutput(self)
        direction_service_timeout = _positive_finite(
            "direction_service_timeout_sec",
            self.get_parameter("direction_service_timeout_sec").value)
        direction_renew_period = _positive_finite(
            "direction_renew_period_sec",
            self.get_parameter("direction_renew_period_sec").value)
        self._direction_guard = RosDirectionGuard(
            self,
            self._io_group,
            direction_service_timeout,
            self.get_parameter("direction_stop_timeout_sec").value,
            self.get_parameter("direction_stop_dwell_sec").value,
            self.get_parameter("direction_stop_linear_tolerance").value,
            self.get_parameter("direction_stop_angular_tolerance").value,
            self.get_parameter("direction_odom_stale_timeout_sec").value,
        )
        self._navigator = RosNavigator(
            self,
            self._io_group,
            self._direction_guard,
            self.get_parameter("precise_behavior_tree").value,
            self.get_parameter("transit_behavior_tree").value,
            self.get_parameter("navigation_timeout_sec").value,
            self.get_parameter("goal_response_timeout_sec").value,
            self.get_parameter("cancel_timeout_sec").value,
            direction_renew_period,
            self.get_parameter("direction_prepare_timeout_sec").value,
            self.get_parameter("direction_prepare_retry_period_sec").value,
            self.get_parameter("through_poses_behavior_tree").value,
            self.get_parameter("transit_through_poses_behavior_tree").value,
            self.get_parameter("precise_through_poses_behavior_tree").value,
            self.get_parameter("return_through_poses_behavior_tree").value,
        )
        self._navigator.prewarm_action_clients()
        self._vision = RosVision(self, self._io_group)
        self._localization = RosLocalization(
            self,
            self._navigator,
            self._io_group,
            self.get_parameter("reset_timeout_sec").value,
            self.get_parameter("origin_position_tolerance").value,
            self.get_parameter("origin_yaw_tolerance").value,
        )
        self._mission = Mission(
            navigator=self._navigator,
            vision=self._vision,
            localization=self._localization,
            clock=SystemClock(self),
            output=self._output,
            config=MissionConfig(
                server_wait_timeout_sec=self.get_parameter(
                    "server_wait_timeout_sec").value,
                qr_settle_sec=self.get_parameter("qr_settle_sec").value,
                qr_timeout_sec=self.get_parameter("qr_timeout_sec").value,
                qr_retries=self.get_parameter("qr_retries").value,
                qr_retry_delay_sec=self.get_parameter(
                    "qr_retry_delay_sec").value,
                vlm_timeout_sec=self.get_parameter("vlm_timeout_sec").value,
            ),
        )
        self._worker_lock = threading.RLock()
        self._worker = None

    def _register_services(self):
        for name, callback in (
            ("/smartcar/task/start", self._on_start),
            ("/smartcar/task/stop", self._on_stop),
            ("/smartcar/task/reset", self._on_reset),
        ):
            self.create_service(
                Trigger,
                name,
                callback,
                callback_group=self._service_group,
            )

    def _configure_autostart(self):
        self._autostart_timer = None
        self._autostart_retries = 0
        self._autostart_max_retries = 60
        if bool(self.get_parameter("autostart_mission").value):
            self._autostart_timer = self.create_timer(
                3.0,
                self._on_autostart,
                callback_group=self._service_group,
            )

    def _start_worker(self):
        with self._worker_lock:
            missing_gates = [
                name for name, ready in self._motion_gates.items()
                if not ready
            ]
            if missing_gates:
                return (
                    False,
                    "motion gates not satisfied: " + ",".join(missing_gates),
                )
            if self._worker is not None and self._worker.is_alive():
                return False, "mission worker already running"
            generation = self._mission.reserve_start()
            if generation is None:
                return False, f"mission state is {self._mission.state.value}"
            self._worker = threading.Thread(
                target=self._run_mission,
                args=(generation,),
                name="smartcar-mission",
                daemon=True,
            )
            self._worker.start()
        return True, "mission started"

    def _run_mission(self, generation):
        result = self._mission.run_reserved(
            generation,
            self._waypoints,
            self._navigation_segments,
            c_zone_navigation_variants=self._competition_navigation_variants,
        )
        if result.success:
            self.get_logger().info(result.status)
        else:
            self.get_logger().warning(result.status)

    def _on_start(self, _request, response):
        response.success, response.message = self._start_worker()
        return response

    def _on_stop(self, _request, response):
        accepted = self._mission.request_stop()
        if not accepted:
            response.success = False
            response.message = "no mission is running"
            return response
        with self._worker_lock:
            worker = self._worker
        if worker is not None:
            worker.join(timeout=self._stop_timeout_sec)
        terminal = (
            (worker is None or not worker.is_alive())
            and not self._navigator.is_active()
        )
        response.success = terminal
        response.message = (
            "mission stopped"
            if terminal
            else "stop requested; terminal confirmation pending"
        )
        return response

    def _on_reset(self, _request, response):
        with self._worker_lock:
            worker = self._worker
        if worker is not None and worker.is_alive():
            response.success = False
            response.message = "mission worker has not stopped"
            return response
        result = self._mission.reset()
        response.success = result.success
        response.message = result.status
        return response

    def _on_autostart(self):
        self._autostart_retries += 1
        success, message = self._start_worker()
        if success:
            if self._autostart_timer is not None:
                self.destroy_timer(self._autostart_timer)
                self._autostart_timer = None
            self.get_logger().info("Mission autostarted")
            return
        if self._autostart_retries >= self._autostart_max_retries:
            if self._autostart_timer is not None:
                self.destroy_timer(self._autostart_timer)
                self._autostart_timer = None
            self.get_logger().error(
                f"Mission autostart failed after "
                f"{self._autostart_retries} attempts: {message}")
        else:
            self.get_logger().warn(
                f"Mission autostart attempt {self._autostart_retries}/"
                f"{self._autostart_max_retries}: {message}")

    def stop_for_shutdown(self):
        accepted = self._mission.request_stop()
        if not accepted and self._navigator.is_active():
            self._navigator.cancel()
        with self._worker_lock:
            worker = self._worker
        if worker is not None:
            worker.join(timeout=self._stop_timeout_sec)
        self._vision.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_for_shutdown()
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
