from pathlib import Path
import sys
import unittest

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_common.qos import LATEST_SENSOR_QOS, STATUS_QOS  # noqa: E402


class SharedQosTests(unittest.TestCase):
    def test_latest_sensor_profile_discards_obsolete_samples(self):
        self.assertEqual(LATEST_SENSOR_QOS.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(LATEST_SENSOR_QOS.depth, 1)
        self.assertEqual(
            LATEST_SENSOR_QOS.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(
            LATEST_SENSOR_QOS.durability, DurabilityPolicy.VOLATILE)

    def test_status_profile_is_latched_and_reliable(self):
        self.assertEqual(STATUS_QOS.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(STATUS_QOS.depth, 1)
        self.assertEqual(STATUS_QOS.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(STATUS_QOS.durability, DurabilityPolicy.TRANSIENT_LOCAL)


if __name__ == "__main__":
    unittest.main()
