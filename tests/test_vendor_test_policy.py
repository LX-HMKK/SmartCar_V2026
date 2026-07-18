"""Contracts for inherited vendor-only lint registration."""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
VENDOR_CMAKE_FILES = (
    ROOT / "src" / "origincar" / "origincar_bringup" / "CMakeLists.txt",
    ROOT / "src" / "origincar" / "origincar_description" / "CMakeLists.txt",
    ROOT / "src" / "origincar" / "ydlidar_ros2_driver" / "CMakeLists.txt",
)


class VendorTestPolicyTests(unittest.TestCase):
    def test_legacy_vendor_lint_is_explicitly_opt_in(self):
        for path in VENDOR_CMAKE_FILES:
            with self.subTest(package=path.parent.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn("SMARTCAR_ENABLE_VENDOR_LINT", source)
                self.assertIn(
                    "if(BUILD_TESTING AND SMARTCAR_ENABLE_VENDOR_LINT)",
                    source,
                )
                option = source.split(
                    "option(\n  SMARTCAR_ENABLE_VENDOR_LINT", 1
                )[1].split(")", 1)[0]
                self.assertIn("\n  OFF\n", option)
                self.assertIn(
                    "ament_lint_auto_find_test_dependencies()", source)


if __name__ == "__main__":
    unittest.main()
