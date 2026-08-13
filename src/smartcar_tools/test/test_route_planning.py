"""Focused contracts for the shared route-planning configuration."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
GEOMETRY_FILE = PACKAGE_ROOT / "config" / "routes" / "field_geometry.yaml"
CONFIG_FILE = PACKAGE_ROOT / "config" / "routes" / "route_planning.yaml"
ROUTE_PLANNING_SYNC = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_sim"
    / "scripts"
    / "sync_route_planning.py"
)
EDITOR_LAUNCH = PACKAGE_ROOT / "launch" / "waypoint_editor.launch.py"
SIM_TUNE = REPOSITORY_ROOT / "src" / "smartcar_sim" / "scripts" / "sim_tune.sh"
NAV2_PARAMS = REPOSITORY_ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
NAVIGATION_LAUNCH = REPOSITORY_ROOT / "src" / "smartcar_nav2" / "launch" / "navigation_launch.py"
SIMULATION_LAUNCH = REPOSITORY_ROOT / "src" / "smartcar_sim" / "launch" / "sim.launch.py"
SIMULATION_OVERLAY = (
    REPOSITORY_ROOT / "src" / "smartcar_sim" / "config" / "nav2_simulation.yaml"
)
SAFETY_CONFIG = (
    REPOSITORY_ROOT / "src" / "smartcar_safety" / "config" / "safety.yaml"
)

sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "smartcar_task"))

from smartcar_tools.field_reference import load_field_reference  # noqa: E402
from smartcar_tools.route_planning import (  # noqa: E402
    RoutePlanningConfigError,
    load_route_planning_config,
)
from smartcar_tools.route_preflight import LatticePreflightPlanner, Pose2D  # noqa: E402


def load_route_planning_sync():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "test_sync_route_planning", ROUTE_PLANNING_SYNC
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sync_route_planning.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SharedRoutePlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.reference = load_field_reference(GEOMETRY_FILE)

    def test_defaults_preserve_motion_and_realtime_costmap_constraints(self) -> None:
        config = load_route_planning_config(CONFIG_FILE)

        self.assertEqual(config.minimum_turning_radius_m, 0.23)
        self.assertEqual(config.simulation_minimum_turning_radius_m, 0.22)
        safety = yaml.safe_load(SAFETY_CONFIG.read_text(encoding="utf-8"))[
            "safety_node"]["ros__parameters"]
        physical_limit = safety["wheelbase"] / math.tan(
            safety["max_steering_angle"])
        nav2 = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))[
            "planner_server"]["ros__parameters"]["GridBased"]
        self.assertGreaterEqual(config.minimum_turning_radius_m, physical_limit)
        self.assertEqual(nav2["minimum_turning_radius"],
                         config.minimum_turning_radius_m)
        footprint = config.runtime_footprint
        self.assertEqual(footprint.length_m, 0.27)
        self.assertEqual(footprint.width_m, 0.13)
        self.assertEqual(footprint.center_x_from_base_footprint_m, 0.0841)
        self.assertEqual(footprint.body_half_length_m, 0.135)
        self.assertAlmostEqual(footprint.front_extent_m, 0.2191)
        self.assertAlmostEqual(footprint.rear_extent_m, 0.0509)
        self.assertAlmostEqual(footprint.half_length_m, 0.2191)
        self.assertEqual(footprint.half_width_m, 0.065)
        self.assertEqual(footprint.padding_m, 0.03)
        self.assertAlmostEqual(footprint.padded_half_length_m, 0.2491)
        self.assertAlmostEqual(footprint.padded_half_width_m, 0.095)
        self.assertEqual(
            config.simulation_costmap.inflation_radius_m, 0.30
        )

    def test_simulation_radius_is_required_and_positive(self) -> None:
        document = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))

        for invalid in (None, 0.0, -0.01):
            with self.subTest(invalid=invalid):
                altered = dict(document)
                if invalid is None:
                    altered.pop("simulation_minimum_turning_radius_m")
                else:
                    altered["simulation_minimum_turning_radius_m"] = invalid
                with tempfile.TemporaryDirectory() as temporary:
                    altered_path = Path(temporary) / "route_planning.yaml"
                    altered_path.write_text(
                        yaml.safe_dump(altered, sort_keys=False), encoding="utf-8"
                    )
                    with self.assertRaisesRegex(
                        RoutePlanningConfigError,
                        "simulation_minimum_turning_radius_m",
                    ):
                        load_route_planning_config(altered_path)

    def test_runtime_navigation_uses_realtime_costmaps_without_a_prior_map(self) -> None:
        parameters = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        for costmap_name in ("local_costmap", "global_costmap"):
            costmap = parameters[costmap_name][costmap_name]["ros__parameters"]
            self.assertEqual(
                costmap["plugins"], ["obstacle_layer", "inflation_layer"]
            )
            self.assertTrue(costmap["obstacle_layer"]["enabled"])
            self.assertTrue(costmap["inflation_layer"]["enabled"])
            self.assertNotIn("filters", costmap)
            self.assertNotIn("keepout_filter", costmap)

        for source in (NAVIGATION_LAUNCH, SIMULATION_LAUNCH):
            content = source.read_text(encoding="utf-8")
            self.assertNotIn("keepout_filter", content)
            self.assertNotIn("field_map", content)
            self.assertNotIn("nav2_map_server", content)

    def test_footprint_dimensions_reject_legacy_half_extent_fields(self) -> None:
        document = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
        document["runtime_footprint"] = {
            "half_length_m": 0.27,
            "half_width_m": 0.13,
            "center_x_from_base_footprint_m": 0.0841,
            "padding_m": 0.03,
        }

        with tempfile.TemporaryDirectory() as temporary:
            altered_path = Path(temporary) / "route_planning.yaml"
            altered_path.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            with self.assertRaisesRegex(
                RoutePlanningConfigError, "runtime_footprint"
            ):
                load_route_planning_config(altered_path)

    def test_simulation_costmap_radius_is_required_and_positive(self) -> None:
        document = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))

        for invalid in (None, 0.0, -0.01):
            with self.subTest(invalid=invalid):
                altered = yaml.safe_load(yaml.safe_dump(document))
                if invalid is None:
                    altered["simulation_costmap"].pop("inflation_radius_m")
                else:
                    altered["simulation_costmap"]["inflation_radius_m"] = invalid
                with tempfile.TemporaryDirectory() as temporary:
                    altered_path = Path(temporary) / "route_planning.yaml"
                    altered_path.write_text(
                        yaml.safe_dump(altered, sort_keys=False), encoding="utf-8"
                    )
                    with self.assertRaisesRegex(
                        RoutePlanningConfigError,
                        "inflation_radius_m",
                    ):
                        load_route_planning_config(altered_path)

    def test_editor_and_tune_entrypoints_accept_the_same_config_file(self) -> None:
        editor_launch = EDITOR_LAUNCH.read_text(encoding="utf-8")
        sim_tune = SIM_TUNE.read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("route_planning_file")', editor_launch)
        self.assertIn("SMARTCAR_ROUTE_PLANNING_CONFIG", editor_launch)
        self.assertIn("route_planning.yaml", sim_tune)
        self.assertIn("--route-planning-config", sim_tune)
        self.assertIn("sync_route_planning.py", sim_tune)
        self.assertIn("--simulation-overlay", sim_tune)
        self.assertNotIn("generate_field_map.py", sim_tune)
        self.assertNotIn("--nav2-params", sim_tune)

    def test_simulation_only_constraints_sync_into_overlay_without_mutating_base_nav2(self) -> None:
        synchronizer = load_route_planning_sync()
        planning = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
        planning["minimum_turning_radius_m"] = 0.63
        planning["simulation_minimum_turning_radius_m"] = 0.21
        planning["runtime_footprint"] = {
            "length_m": 0.58,
            "width_m": 0.28,
            "center_x_from_base_footprint_m": 0.11,
            "padding_m": 0.04,
        }
        planning["simulation_costmap"]["inflation_radius_m"] = 0.17

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            planning_file = root / "route_planning.yaml"
            nav2_params = root / "nav2_params.yaml"
            simulation_overlay = root / "nav2_simulation.yaml"
            planning_file.write_text(
                yaml.safe_dump(planning, sort_keys=False), encoding="utf-8"
            )
            nav2_params.write_text(NAV2_PARAMS.read_text(encoding="utf-8"), encoding="utf-8")
            simulation_overlay.write_text(
                SIMULATION_OVERLAY.read_text(encoding="utf-8"), encoding="utf-8"
            )

            self.assertTrue(
                synchronizer.synchronize(planning_file, simulation_overlay)
            )
            self.assertFalse(
                synchronizer.synchronize(
                    planning_file, simulation_overlay, check=True
                )
            )
            self.assertEqual(
                nav2_params.read_text(encoding="utf-8"),
                NAV2_PARAMS.read_text(encoding="utf-8"),
            )
            overlay = yaml.safe_load(simulation_overlay.read_text(encoding="utf-8"))

        self.assertEqual(
            overlay["planner_server"]["ros__parameters"]["GridBased"]
            ["minimum_turning_radius"],
            0.21,
        )
        self.assertEqual(
            overlay["smoother_server"]["ros__parameters"]
            ["constrained_smoother"]["minimum_turning_radius"],
            0.21,
        )
        controller = overlay["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        self.assertEqual(
            follow_path["regulated_linear_scaling_min_radius"], 0.21
        )
        self.assertEqual(controller["controller_plugins"], ["FollowPath"])
        self.assertNotIn("ForwardAvoidance", controller)
        for name in ("local_costmap", "global_costmap"):
            parameters = overlay[name][name]["ros__parameters"]
            self.assertEqual(
                parameters["footprint"],
                "[[0.4, 0.14], [0.4, -0.14], [-0.4, -0.14], [-0.4, 0.14]]",
            )
            self.assertEqual(parameters["footprint_padding"], 0.04)
            self.assertNotIn("filters", parameters)
            self.assertNotIn("keepout_filter", parameters)
            self.assertEqual(
                overlay[name][name]["ros__parameters"]["inflation_layer"]
                ["inflation_radius"],
                0.17,
            )

    def test_preflight_does_not_reject_b_zone_from_a_prior_map(self) -> None:
        config = load_route_planning_config(CONFIG_FILE)
        planner = LatticePreflightPlanner(self.reference, config)
        self.assertTrue(planner._is_free(1.0, 2.0, 0.0))
        self.assertTrue(planner._is_free(3.0, 2.0, 0.0))
        self.assertTrue(planner._is_free(0.0, 0.0, 0.0))
        self.assertFalse(planner._is_free(-0.6, 0.0, 0.0))

    def test_terminal_route_is_tangent_continuous_not_a_straight_line_patch(self) -> None:
        planner = LatticePreflightPlanner(
            self.reference,
            load_route_planning_config(CONFIG_FILE),
        )
        start = Pose2D(0.0, 0.0, 0.0)
        goal = Pose2D(1.2, 0.25, 0.4)

        planned = planner.plan(start, goal)
        self.assertIsNotNone(planned)
        points, expanded = planned
        self.assertEqual(points[0], type(points[0])(start.x, start.y))
        self.assertEqual(points[-1], type(points[-1])(goal.x, goal.y))
        self.assertGreater(expanded, 0)
        self.assertTrue(all(planner._is_free(point.x, point.y) for point in points))

        final_bearing = math.atan2(
            points[-1].y - points[-2].y,
            points[-1].x - points[-2].x,
        )
        final_error = math.atan2(
            math.sin(final_bearing - goal.yaw),
            math.cos(final_bearing - goal.yaw),
        )
        self.assertLess(abs(final_error), 0.08)


if __name__ == "__main__":
    unittest.main()
