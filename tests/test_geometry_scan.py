"""Regression coverage for offline Dubins and B-opening candidate scans."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest


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

    def test_b_scan_rejects_corner_grazing_points_and_returns_clear_chain(self):
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
        candidates = module.scan_b_segment_candidates(
            start,
            gate_positions=(module.Pose(1.80, 2.85, 0.0),),
            enter_positions=(module.Pose(1.10, 2.85, 0.0),),
            collision_context=context,
        )

        self.assertEqual(rejected, [])
        self.assertTrue(candidates)
        candidate = candidates[0]
        self.assertEqual(candidate.gate_heading_rank, 0)
        self.assertGreaterEqual(
            candidate.min_west_corner_clearance_m,
            module.DEFAULT_B_MIN_CORNER_CLEARANCE_M,
        )
        self.assertTrue(candidate.start_to_gate.feasible)
        self.assertTrue(candidate.gate_to_enter.feasible)
        self.assertTrue(module.check_corridor_passable(
            candidate.start_to_gate.path_points, context
        ))
        candidate_json = module.b_candidate_json(candidate)
        self.assertTrue(
            set(candidate_json) >= {"b_corridor_gate", "b_corridor_enter"}
        )
        self.assertNotEqual(
            (
                candidate_json["b_corridor_gate"]["x"],
                candidate_json["b_corridor_gate"]["y"],
            ),
            (1.80, 2.50),
        )


if __name__ == "__main__":
    unittest.main()
