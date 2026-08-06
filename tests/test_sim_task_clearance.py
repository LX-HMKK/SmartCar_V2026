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
SHARED_ROUTE_IDS = ("p_start",)


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


def _point_to_oriented_rectangle_distance(
    point_x: float,
    point_y: float,
    center_x: float,
    center_y: float,
    yaw: float,
    half_x: float,
    half_y: float,
) -> float:
    """Return signed distance from a point to an oriented rectangle."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    delta_x = point_x - center_x
    delta_y = point_y - center_y
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    excess_x = abs(local_x) - half_x
    excess_y = abs(local_y) - half_y
    if excess_x <= 0.0 and excess_y <= 0.0:
        return -min(-excess_x, -excess_y)
    return math.hypot(max(excess_x, 0.0), max(excess_y, 0.0))


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
        collision_radius_text = model.findtext(
            "./link/collision/geometry/cylinder/radius")
        collision_length_text = model.findtext(
            "./link/collision/geometry/cylinder/length")
        visual_radius_text = model.findtext("./link/visual/geometry/cylinder/radius")
        visual_length_text = model.findtext("./link/visual/geometry/cylinder/length")
        if (
            pose_text is None
            or collision_radius_text is None
            or collision_length_text is None
            or visual_radius_text is None
            or visual_length_text is None
        ):
            raise ValueError(f"{name} must retain matching cylindrical geometry")
        x, y, z, roll, pitch, yaw = _pose_values(pose_text)
        collision_radius = float(collision_radius_text)
        collision_length = float(collision_length_text)
        visual_radius = float(visual_radius_text)
        visual_length = float(visual_length_text)
        if abs(roll) > 1.0e-9 or abs(pitch) > 1.0e-9:
            raise ValueError(f"{name} must remain upright")
        if collision_radius <= 0.0 or collision_length <= 0.0:
            raise ValueError(f"{name} must have positive collision dimensions")
        if abs(collision_radius - visual_radius) > 1.0e-9 or abs(
            collision_length - visual_length
        ) > 1.0e-9:
            raise ValueError(f"{name} visual and collision geometry must match")
        cones[name] = {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "radius": collision_radius,
            "half_z": collision_length * 0.5,
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

    def test_shared_start_pose_matches_between_runtime_routes(self) -> None:
        self.assertTrue(set(SHARED_ROUTE_IDS).issubset(self.nav_only))
        self.assertTrue(set(SHARED_ROUTE_IDS).issubset(self.default))
        self.assertEqual(self.default["a_task_observe"]["task"], "qr")
        self.assertEqual(self.default["c_corner_1"]["task"], "vlm")
        for waypoint_id in SHARED_ROUTE_IDS:
            with self.subTest(waypoint=waypoint_id):
                self.assertEqual(
                    self.nav_only[waypoint_id]["pose"],
                    self.default[waypoint_id]["pose"],
                )
        self.assertNotEqual(
            self.nav_only["a_task_observe"]["pose"],
            self.default["a_task_observe"]["pose"],
        )

    def test_a_zone_cones_remain_physical_and_clear_all_protected_bodies(self) -> None:
        self.assertEqual(set(self.cones), {f"cone_a{index}" for index in range(1, 7)})
        footprint = self.route_planning["runtime_footprint"]
        half_length = (
            float(footprint["center_x_from_base_footprint_m"])
            + 0.5 * float(footprint["length_m"])
            + float(footprint["padding_m"])
        )
        half_width = 0.5 * float(footprint["width_m"]) + float(footprint["padding_m"])

        for route_name, route in (
            ("nav_only", self.nav_only),
            ("default", self.default),
        ):
            for waypoint_id in PROTECTED_IDS:
                pose = route[waypoint_id]["pose"]
                position = pose["position"]
                vehicle_x = float(position["x"])
                vehicle_y = float(position["y"])
                vehicle_yaw = _yaw_from_quaternion(pose["orientation"])
                for cone_id, cone in self.cones.items():
                    with self.subTest(
                        route=route_name, waypoint=waypoint_id, cone=cone_id
                    ):
                        self.assertAlmostEqual(cone["radius"], 0.05)
                        clearance = _point_to_oriented_rectangle_distance(
                            cone["x"],
                            cone["y"],
                            vehicle_x,
                            vehicle_y,
                            vehicle_yaw,
                            half_length,
                            half_width,
                        )
                        self.assertGreater(
                            clearance,
                            cone["radius"] + 1.0e-6,
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
        self.assertGreaterEqual(cone["x"] - cone["radius"], a_zone_min_x)
        self.assertLessEqual(cone["x"] + cone["radius"], a_zone_max_x)
        self.assertGreaterEqual(cone["y"] - cone["radius"], a_zone_min_y)
        self.assertLessEqual(cone["y"] + cone["radius"], a_zone_max_y)

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
