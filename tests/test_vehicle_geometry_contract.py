"""Static geometry contracts for real, Gazebo, and display-only models."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SDF = ROOT / "src/smartcar_sim/models/origincar/model.sdf"
MODEL_CONFIG = ROOT / "src/smartcar_sim/models/origincar/model.config"
SIM_NAV = ROOT / "src/smartcar_sim/config/nav2_simulation.yaml"
XACRO = ROOT / "src/origincar/origincar_description/urdf/origincar.xacro"
URDF = ROOT / "src/origincar/origincar_description/urdf/origincar.urdf"
BRINGUP_COORD = ROOT / "src/smartcar_bringup/config/bringup_coord.yaml"

WHEELBASE_M = 0.144
TRACK_M = 0.162
BASE_LINK_X_M = 0.0841
SIM_RADIUS_M = 0.22


def pose_xyz(root: ET.Element, link_name: str) -> tuple[float, float, float]:
    pose = root.find(f"model/link[@name='{link_name}']/pose")
    assert pose is not None
    return tuple(float(value) for value in pose.text.split()[:3])


class VehicleGeometryContractTests(unittest.TestCase):
    def test_gazebo_model_matches_confirmed_rear_axle_geometry(self):
        root = ET.parse(SDF).getroot()
        self.assertEqual(pose_xyz(root, "down_left_Link"), (0.0, 0.081, 0.03))
        self.assertEqual(pose_xyz(root, "down_right_Link"), (0.0, -0.081, 0.03))
        self.assertEqual(pose_xyz(root, "up_left_Link"), (WHEELBASE_M, 0.081, 0.03))
        self.assertEqual(pose_xyz(root, "up_right_Link"), (WHEELBASE_M, -0.081, 0.03))

        plugin = root.find("model/plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        self.assertIsNotNone(plugin)
        self.assertAlmostEqual(float(plugin.findtext("wheel_base")), WHEELBASE_M)
        self.assertAlmostEqual(float(plugin.findtext("kingpin_width")), TRACK_M)
        self.assertAlmostEqual(float(plugin.findtext("wheel_separation")), TRACK_M)
        self.assertAlmostEqual(
            float(plugin.findtext("steering_limit")),
            math.asin(WHEELBASE_M / SIM_RADIUS_M),
            places=3,
        )

    def test_display_model_uses_same_wheelbase_and_track(self):
        xacro = XACRO.read_text(encoding="utf-8")
        self.assertIn('name="Track" value="0.162"', xacro)
        self.assertIn('name="WheelBase" value="0.144"', xacro)
        self.assertIn('<box size="0.276 0.164 0.08"/>', xacro)

        root = ET.parse(URDF).getroot()
        origins = {
            joint.attrib["name"]: tuple(
                float(value) for value in joint.find("origin").attrib["xyz"].split()
            )
            for joint in root.findall("joint")
            if joint.attrib["name"] in {
                "down_left_joint", "down_right_joint",
                "up_left_joint", "up_right_joint",
            }
        }
        self.assertEqual(origins["down_left_joint"], (-0.072, 0.081, 0.03))
        self.assertEqual(origins["down_right_joint"], (-0.072, -0.081, 0.03))
        self.assertEqual(origins["up_left_joint"], (0.072, 0.081, 0.03))
        self.assertEqual(origins["up_right_joint"], (0.072, -0.081, 0.03))

    def test_camera_and_simulation_metadata_match_the_wheelbase(self):
        config = yaml.safe_load(BRINGUP_COORD.read_text(encoding="utf-8"))
        base = config["extrinsics"]["base_to_link"]["xyz"][0]
        depth = config["extrinsics"]["link_to_depth_camera"]["xyz"][0]
        self.assertAlmostEqual(base, BASE_LINK_X_M)
        self.assertAlmostEqual(base + depth, WHEELBASE_M)
        self.assertAlmostEqual(config["calibration"]["wheelbase"], WHEELBASE_M)

        model_config = MODEL_CONFIG.read_text(encoding="utf-8")
        self.assertIn("wheelbase 0.144m, track 0.162m", model_config)
        self.assertIn("0.144 m wheelbase", SIM_NAV.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
