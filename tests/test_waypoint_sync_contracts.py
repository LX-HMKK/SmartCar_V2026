"""Keep deployed semantic route geometry identical to the verified sim route."""
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
WAYPOINT_DIRECTORY = ROOT / "src" / "smartcar_nav2" / "config" / "waypoints"
SIM_ROUTE = WAYPOINT_DIRECTORY / "nav_only.yaml"
DEPLOYMENT_ROUTE = WAYPOINT_DIRECTORY / "default_waypoints.yaml"


def _waypoints_by_id(document):
    return {waypoint["id"]: waypoint for waypoint in document["waypoints"]}


class WaypointSyncContracts(unittest.TestCase):
    def test_deployment_route_uses_the_verified_simulation_geometry(self):
        simulation = yaml.safe_load(SIM_ROUTE.read_text(encoding="utf-8"))
        deployment = yaml.safe_load(DEPLOYMENT_ROUTE.read_text(encoding="utf-8"))

        self.assertEqual(
            deployment["planning_segments"], simulation["planning_segments"]
        )
        simulation_points = _waypoints_by_id(simulation)
        deployment_points = _waypoints_by_id(deployment)
        self.assertEqual(set(deployment_points), set(simulation_points))
        for waypoint_id, simulation_point in simulation_points.items():
            with self.subTest(waypoint=waypoint_id):
                deployment_point = deployment_points[waypoint_id]
                self.assertEqual(
                    deployment_point["frame_id"], simulation_point["frame_id"]
                )
                self.assertEqual(
                    deployment_point["pose"], simulation_point["pose"]
                )
                self.assertEqual(
                    deployment_point["direction"],
                    simulation_point["direction"],
                )
                self.assertEqual(
                    deployment_point["goal_profile"],
                    simulation_point["goal_profile"],
                )

    def test_deployment_route_only_replaces_simulation_media_tasks(self):
        simulation = yaml.safe_load(SIM_ROUTE.read_text(encoding="utf-8"))
        deployment = yaml.safe_load(DEPLOYMENT_ROUTE.read_text(encoding="utf-8"))
        simulation_tasks = {
            waypoint["id"]: waypoint["task"]
            for waypoint in simulation["waypoints"]
        }
        deployment_tasks = {
            waypoint["id"]: waypoint["task"]
            for waypoint in deployment["waypoints"]
        }

        self.assertEqual(simulation_tasks["a_task_observe"], "nav")
        self.assertEqual(deployment_tasks["a_task_observe"], "qr")
        self.assertEqual(simulation_tasks["c_corner_1"], "nav")
        self.assertEqual(deployment_tasks["c_corner_1"], "vlm")
        for waypoint_id in set(simulation_tasks) - {
            "a_task_observe", "c_corner_1"
        }:
            with self.subTest(waypoint=waypoint_id):
                self.assertEqual(
                    deployment_tasks[waypoint_id], simulation_tasks[waypoint_id]
                )


if __name__ == "__main__":
    unittest.main()
