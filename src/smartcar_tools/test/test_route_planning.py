"""Focused contracts for the shared route-planning configuration."""

from __future__ import annotations

import importlib.util
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
FIELD_MAP_GENERATOR = (
    REPOSITORY_ROOT
    / "src"
    / "smartcar_sim"
    / "scripts"
    / "generate_field_map.py"
)
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
KEEPOUT_OVERLAY = (
    REPOSITORY_ROOT / "src" / "smartcar_sim" / "config" / "nav2_keepout_filter.yaml"
)

sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "smartcar_task"))

from smartcar_tools.field_keepouts import (  # noqa: E402
    central_c_keepout,
    keepout_mask_bounds,
)
from smartcar_tools.field_reference import load_field_reference  # noqa: E402
from smartcar_tools.route_planning import load_route_planning_config  # noqa: E402
from smartcar_tools.route_preflight import LatticePreflightPlanner, Pose2D  # noqa: E402


def load_field_map_generator():
    spec = importlib.util.spec_from_file_location(
        "test_generate_field_map", FIELD_MAP_GENERATOR
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load generate_field_map.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_route_planning_sync():
    spec = importlib.util.spec_from_file_location(
        "test_sync_route_planning", ROUTE_PLANNING_SYNC
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load sync_route_planning.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def pgm_value_at(pgm: bytes, descriptor: str, x: float, y: float) -> int:
    magic, dimensions, maximum, pixels = pgm.split(b"\n", 3)
    if magic != b"P5" or maximum != b"255":
        raise ValueError("unexpected PGM header")
    width, height = (int(value) for value in dimensions.split())
    document = yaml.safe_load(descriptor)
    resolution = document["resolution"]
    origin_x, origin_y, _ = document["origin"]
    col = int((x - origin_x) // resolution)
    row_from_bottom = int((y - origin_y) // resolution)
    row = height - 1 - row_from_bottom
    return pixels[row * width + col]


class SharedRoutePlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.reference = load_field_reference(GEOMETRY_FILE)

    def test_defaults_preserve_current_motion_and_c_zone_constraints(self) -> None:
        config = load_route_planning_config(CONFIG_FILE)
        core = central_c_keepout(self.reference, config)

        self.assertEqual(config.minimum_turning_radius_m, 0.55)
        footprint = config.runtime_footprint
        self.assertEqual(footprint.half_length_m, 0.27)
        self.assertEqual(footprint.half_width_m, 0.13)
        self.assertEqual(footprint.padding_m, 0.03)
        self.assertAlmostEqual(footprint.padded_half_length_m, 0.30)
        self.assertAlmostEqual(footprint.padded_half_width_m, 0.16)
        self.assertEqual(
            config.simulation_keepout.costmap_inflation_radius_m, 0.20
        )
        self.assertEqual(config.c_zone_keepout.horizontal_inset_m, 0.80)
        self.assertEqual(config.c_zone_keepout.vertical_inset_m, 0.15)
        self.assertEqual((core.x_min, core.x_max, core.y_min, core.y_max), (
            1.3,
            2.7,
            3.15,
            3.5,
        ))

    def test_c_zone_tuning_changes_editor_keepout_and_simulation_pgm_together(self) -> None:
        generator = load_field_map_generator()
        document = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
        document["c_zone_keepout"]["horizontal_inset_m"] = 1.0

        with tempfile.TemporaryDirectory() as temporary:
            altered_path = Path(temporary) / "route_planning.yaml"
            altered_path.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            altered = load_route_planning_config(altered_path)
            altered_core = central_c_keepout(self.reference, altered)
            default_pgm, default_descriptor = generator.render(
                GEOMETRY_FILE, CONFIG_FILE
            )
            altered_pgm, altered_descriptor = generator.render(
                GEOMETRY_FILE, altered_path
            )

        self.assertEqual((altered_core.x_min, altered_core.x_max), (1.5, 2.5))
        # This cell lies inside the default C core but in the lane released by
        # the narrower tuned core. The editor and generated PGM must agree.
        self.assertEqual(
            pgm_value_at(default_pgm, default_descriptor, 1.35, 3.3), 0
        )
        self.assertEqual(
            pgm_value_at(altered_pgm, altered_descriptor, 1.35, 3.3), 254
        )

    def test_editor_and_tune_entrypoints_accept_the_same_config_file(self) -> None:
        editor_launch = EDITOR_LAUNCH.read_text(encoding="utf-8")
        sim_tune = SIM_TUNE.read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("route_planning_file")', editor_launch)
        self.assertIn("SMARTCAR_ROUTE_PLANNING_CONFIG", editor_launch)
        self.assertIn("route_planning.yaml", sim_tune)
        self.assertIn("--route-planning-config", sim_tune)
        self.assertIn("sync_route_planning.py", sim_tune)

    def test_shared_radius_footprint_and_cost_inflation_sync_into_simulation_nav2(self) -> None:
        synchronizer = load_route_planning_sync()
        planning = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
        planning["minimum_turning_radius_m"] = 0.63
        planning["runtime_footprint"] = {
            "half_length_m": 0.29,
            "half_width_m": 0.14,
            "padding_m": 0.04,
        }
        planning["simulation_keepout"]["costmap_inflation_radius_m"] = 0.17

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            planning_file = root / "route_planning.yaml"
            nav2_params = root / "nav2_params.yaml"
            keepout_overlay = root / "nav2_keepout_filter.yaml"
            planning_file.write_text(
                yaml.safe_dump(planning, sort_keys=False), encoding="utf-8"
            )
            nav2_params.write_text(NAV2_PARAMS.read_text(encoding="utf-8"), encoding="utf-8")
            keepout_overlay.write_text(
                KEEPOUT_OVERLAY.read_text(encoding="utf-8"), encoding="utf-8"
            )

            self.assertTrue(
                synchronizer.synchronize(planning_file, nav2_params, keepout_overlay)
            )
            self.assertFalse(
                synchronizer.synchronize(
                    planning_file, nav2_params, keepout_overlay, check=True
                )
            )
            nav2 = yaml.safe_load(nav2_params.read_text(encoding="utf-8"))
            overlay = yaml.safe_load(keepout_overlay.read_text(encoding="utf-8"))

        self.assertEqual(
            nav2["planner_server"]["ros__parameters"]["GridBased"]
            ["minimum_turning_radius"],
            0.63,
        )
        self.assertEqual(
            nav2["controller_server"]["ros__parameters"]["ReverseHandoff"]
            ["AckermannConstraints"]["min_turning_r"],
            0.63,
        )
        for name in ("local_costmap", "global_costmap"):
            parameters = nav2[name][name]["ros__parameters"]
            self.assertEqual(
                parameters["footprint"],
                "[[0.29, 0.14], [0.29, -0.14], [-0.29, -0.14], [-0.29, 0.14]]",
            )
            self.assertEqual(parameters["footprint_padding"], 0.04)
            self.assertEqual(
                overlay[name][name]["ros__parameters"]["inflation_layer"]
                ["inflation_radius"],
                0.17,
            )

    def test_preflight_uses_padded_oriented_footprint_and_mask_cells(self) -> None:
        config = load_route_planning_config(CONFIG_FILE)
        planner = LatticePreflightPlanner(self.reference, config)
        c_core = keepout_mask_bounds(self.reference, config)[2]

        # The final occupied C-core PGM row ends at 3.55 rather than the raw
        # 3.50 m edge, which is the actual KeepoutFilter collision boundary.
        for actual, expected in zip(
            (c_core.x_min, c_core.x_max, c_core.y_min, c_core.y_max),
            (1.3, 2.7, 3.15, 3.55),
        ):
            self.assertAlmostEqual(actual, expected)
        # A tangent vehicle clears the last C-core mask row, while the
        # diagonal entry that previously appeared in RViz intersects it.
        self.assertTrue(planner._is_free(1.25, 3.72, 0.0))
        self.assertFalse(planner._is_free(1.25, 3.72, 0.757))

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
