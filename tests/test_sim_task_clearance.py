"""Keep physical simulation obstacles clear of protected task poses."""

from __future__ import annotations

import math
from pathlib import Path
import unittest
from xml.etree import ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "src" / "smartcar_sim" / "worlds" / "track.world"
NAV_ONLY = ROOT / "src" / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml"
DEFAULT = ROOT / "src" / "smartcar_nav2" / "config" / "waypoints" / "default_waypoints.yaml"
NAV2_PARAMS = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
KEEPOUT_OVERLAY = ROOT / "src" / "smartcar_sim" / "config" / "nav2_keepout_filter.yaml"
ROUTE_PLANNING = ROOT / "src" / "smartcar_tools" / "config" / "routes" / "route_planning.yaml"
FIELD_GEOMETRY = ROOT / "src" / "smartcar_tools" / "config" / "routes" / "field_geometry.yaml"

PROTECTED_IDS = ("p_start", "a_task_observe", "c_corner_1", "p_finish")


def _pose_values(text: str) -> tuple[float, float, float, float, float, float]:
    values = tuple(float(value) for value in text.split())
    if len(values) != 6:
        raise ValueError(f"SDF pose must contain six values: {text!r}")
    return values


def _yaw_from_quaternion(orientation: dict[str, float]) -> float:
    return math.atan2(
        2.0 * (orientation["w"] * orientation["z"]),
        1.0 - 2.0 * orientation["z"] * orientation["z"],
    )


def _corners(
    center_x: float,
    center_y: float,
    yaw: float,
    half_x: float,
    half_y: float,
) -> list[tuple[float, float]]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            center_x + sign_x * half_x * cosine - sign_y * half_y * sine,
            center_y + sign_x * half_x * sine + sign_y * half_y * cosine,
        )
        for sign_x, sign_y in ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0))
    ]


def _minimum_sat_separation(
    first: list[tuple[float, float]], second: list[tuple[float, float]]
) -> float:
    """Return the smallest SAT overlap; a negative value is a true clearance."""
    overlaps: list[float] = []
    for polygon in (first, second):
        for current, following in zip(polygon, polygon[1:] + polygon[:1]):
            edge_x = following[0] - current[0]
            edge_y = following[1] - current[1]
            length = math.hypot(edge_x, edge_y)
            if length <= 0.0:
                raise ValueError("collision polygon has a zero-length edge")
            axis_x, axis_y = -edge_y / length, edge_x / length
            first_projection = [axis_x * x + axis_y * y for x, y in first]
            second_projection = [axis_x * x + axis_y * y for x, y in second]
            overlaps.append(
                min(max(first_projection), max(second_projection))
                - max(min(first_projection), min(second_projection))
            )
    return min(overlaps)


def _load_waypoints(path: Path) -> dict[str, dict]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {waypoint["id"]: waypoint for waypoint in document["waypoints"]}


def _load_a_zone_cones() -> dict[str, dict[str, float]]:
    root = ET.parse(WORLD).getroot()
    cones: dict[str, dict[str, float]] = {}
    for model in root.findall(".//model"):
        name = model.attrib.get("name", "")
        if not name.startswith("cone_a"):
            continue
        if model.findtext("static") != "true":
            raise ValueError(f"{name} must remain static")
        pose_text = model.findtext("pose")
        size_text = model.findtext("./link/collision/geometry/box/size")
        if pose_text is None or size_text is None:
            raise ValueError(f"{name} must retain a box collision geometry")
        x, y, z, roll, pitch, yaw = _pose_values(pose_text)
        size_x, size_y, size_z = (float(value) for value in size_text.split())
        if abs(roll) > 1.0e-9 or abs(pitch) > 1.0e-9:
            raise ValueError(f"{name} must remain upright")
        cones[name] = {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "half_x": size_x * 0.5,
            "half_y": size_y * 0.5,
            "half_z": size_z * 0.5,
        }
    return cones


class SimulationTaskClearanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.nav_only = _load_waypoints(NAV_ONLY)
        self.default = _load_waypoints(DEFAULT)
        self.cones = _load_a_zone_cones()
        self.route_planning = yaml.safe_load(ROUTE_PLANNING.read_text(encoding="utf-8"))
        self.nav2_params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        self.keepout_overlay = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        self.field_geometry = yaml.safe_load(FIELD_GEOMETRY.read_text(encoding="utf-8"))

    def test_protected_task_poses_match_between_runtime_routes(self) -> None:
        self.assertTrue(set(PROTECTED_IDS).issubset(self.nav_only))
        self.assertTrue(set(PROTECTED_IDS).issubset(self.default))
        self.assertEqual(self.default["a_task_observe"]["task"], "qr")
        self.assertEqual(self.default["c_corner_1"]["task"], "vlm")
        for waypoint_id in PROTECTED_IDS:
            with self.subTest(waypoint=waypoint_id):
                self.assertEqual(
                    self.nav_only[waypoint_id]["pose"],
                    self.default[waypoint_id]["pose"],
                )

    def test_a_zone_cones_remain_physical_and_clear_all_protected_bodies(self) -> None:
        self.assertEqual(set(self.cones), {f"cone_a{index}" for index in range(1, 7)})
        footprint = self.route_planning["runtime_footprint"]
        half_length = float(footprint["half_length_m"]) + float(footprint["padding_m"])
        half_width = float(footprint["half_width_m"]) + float(footprint["padding_m"])

        for waypoint_id in PROTECTED_IDS:
            pose = self.nav_only[waypoint_id]["pose"]
            position = pose["position"]
            vehicle = _corners(
                float(position["x"]),
                float(position["y"]),
                _yaw_from_quaternion(pose["orientation"]),
                half_length,
                half_width,
            )
            for cone_id, cone in self.cones.items():
                with self.subTest(waypoint=waypoint_id, cone=cone_id):
                    obstacle = _corners(
                        cone["x"], cone["y"], cone["yaw"], cone["half_x"], cone["half_y"]
                    )
                    self.assertLess(
                        _minimum_sat_separation(vehicle, obstacle),
                        -1.0e-6,
                        "padded protected vehicle footprint overlaps a physical A-zone obstacle",
                    )

    def test_a6_stays_inside_a_zone_and_in_startup_scan_range(self) -> None:
        cone = self.cones["cone_a6"]
        geometry = self.field_geometry["geometry"]
        p_origin_x = float(geometry["p_origin_x_from_west_m"])
        p_origin_y = float(geometry["p_origin_y_from_south_m"])
        a_zone_min_x = -p_origin_x
        a_zone_max_x = float(geometry["field_width_m"]) - p_origin_x
        a_zone_min_y = -p_origin_y
        a_zone_max_y = float(geometry["zone_a_height_m"]) - p_origin_y
        for x, y in _corners(
            cone["x"], cone["y"], cone["yaw"], cone["half_x"], cone["half_y"]
        ):
            self.assertGreaterEqual(x, a_zone_min_x)
            self.assertLessEqual(x, a_zone_max_x)
            self.assertGreaterEqual(y, a_zone_min_y)
            self.assertLessEqual(y, a_zone_max_y)

        base_scan = self.nav2_params["local_costmap"]["local_costmap"]["ros__parameters"][
            "obstacle_layer"
        ]["scan"]
        height_scan = self.keepout_overlay["local_costmap"]["local_costmap"]["ros__parameters"][
            "obstacle_layer"
        ]["scan"]
        startup = self.nav_only["p_start"]["pose"]["position"]
        self.assertLessEqual(
            math.hypot(cone["x"] - float(startup["x"]), cone["y"] - float(startup["y"])),
            float(base_scan["obstacle_max_range"]),
        )
        self.assertGreaterEqual(
            cone["z"] + cone["half_z"], float(height_scan["min_obstacle_height"])
        )
        self.assertLessEqual(
            cone["z"] - cone["half_z"], float(height_scan["max_obstacle_height"])
        )


if __name__ == "__main__":
    unittest.main()
