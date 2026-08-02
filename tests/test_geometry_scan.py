"""Regression coverage for offline Dubins and B-opening candidate scans."""

import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src" / "smartcar_sim" / "scripts" / "geometry_scan.py"
TOOLS_ROOT = ROOT / "src" / "smartcar_tools"


def load_script_module():
    sys.path.insert(0, str(TOOLS_ROOT))
    spec = importlib.util.spec_from_file_location("geometry_scan_for_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


class GeometryScanTests(unittest.TestCase):
    def test_standard_dubins_sampling_finishes_at_the_requested_pose(self):
        module = load_script_module()
        start = module.Pose(0.0, 0.0, 0.0)
        end = module.Pose(1.0, 1.0, math.pi / 2.0)

        result = module.compute_dubins(start, end, r=0.55)

        self.assertIsNotNone(result)
        final_x, final_y = result.path_points[-1]
        self.assertAlmostEqual(final_x, end.x, places=6)
        self.assertAlmostEqual(final_y, end.y, places=6)

    def test_default_dubins_radius_uses_simulation_constraint(self):
        module = load_script_module()
        planning = yaml.safe_load(
            module.DEFAULT_ROUTE_PLANNING_CONFIG.read_text(encoding="utf-8")
        )
        planning["minimum_turning_radius_m"] = 0.63
        planning["simulation_minimum_turning_radius_m"] = 0.22

        with tempfile.TemporaryDirectory() as temporary:
            planning_file = Path(temporary) / "route_planning.yaml"
            planning_file.write_text(
                yaml.safe_dump(planning, sort_keys=False), encoding="utf-8"
            )
            context = module.load_geometry_context(planning_file)

        previous_context = module._DEFAULT_CONTEXT
        module._DEFAULT_CONTEXT = context
        try:
            start = module.Pose(0.0, 0.0, 0.0)
            end = module.Pose(1.0, 1.0, math.pi / 2.0)
            implicit = module.compute_dubins(start, end)
        finally:
            module._DEFAULT_CONTEXT = previous_context

        simulated = module.compute_dubins(start, end, r=0.22)
        real_preflight = module.compute_dubins(start, end, r=0.63)
        self.assertIsNotNone(implicit)
        self.assertIsNotNone(simulated)
        self.assertIsNotNone(real_preflight)
        assert implicit is not None
        assert simulated is not None
        assert real_preflight is not None
        self.assertAlmostEqual(implicit.path_length, simulated.path_length)
        self.assertNotAlmostEqual(implicit.path_length, real_preflight.path_length)

    def test_b_scan_rejects_obsolete_dense_guides_at_simulation_radius(self):
        """The retired B guides must fail closed instead of inventing a loop."""
        module = load_script_module()
        context = module.load_geometry_context()
        start = module.Pose(
            3.127294927294929,
            0.9765623265623269,
            math.radians(17.342050622241263) + math.pi,
        )

        rejected = module.scan_b_segment_candidates(
            start,
            gate_positions=(module.Pose(1.80, 2.50, 0.0),),
            enter_positions=(module.Pose(1.60, 2.65, 0.0),),
            collision_context=context,
        )
        obsolete_guides = module.scan_b_segment_candidates(
            start,
            gate_positions=(module.Pose(1.80, 2.85, 0.0),),
            enter_positions=(module.Pose(1.10, 2.85, 0.0),),
            collision_context=context,
        )

        self.assertEqual(rejected, [])
        self.assertEqual(obsolete_guides, [])

    def test_compact_c_route_has_direct_short_safe_legs(self):
        """C guides are removed only when each replacement leg is feasible."""
        module = load_script_module()
        context = module.load_geometry_context()
        headings = [
            -math.pi + index * 2.0 * math.pi / 24.0
            for index in range(24)
        ]

        def shortest_safe_leg(start_xy, end_xy):
            candidates = []
            for start_yaw in headings:
                for end_yaw in headings:
                    segment = module.compute_dubins(
                        module.Pose(*start_xy, start_yaw),
                        module.Pose(*end_xy, end_yaw),
                        r=context.config.simulation_minimum_turning_radius_m,
                    )
                    if segment is None:
                        continue
                    segment = module._classify_segment(
                        segment, "compact", context=context
                    )
                    if segment.feasible and not segment.is_loop:
                        candidates.append(segment)
            self.assertTrue(candidates)
            return min(candidates, key=lambda segment: segment.path_length)

        c_entry = shortest_safe_leg((1.10, 2.85), (0.9979495132, 3.7958472007))
        c_exit = shortest_safe_leg((0.9979495132, 3.7958472007), (2.20, 2.35))

        self.assertLess(c_entry.path_length, 1.25)
        self.assertLess(c_exit.path_length, 2.50)


if __name__ == "__main__":
    unittest.main()
