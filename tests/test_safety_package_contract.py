"""Packaging contract required for colcon to execute smartcar_safety tests."""
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SETUP_PY = REPOSITORY_ROOT / "src" / "smartcar_safety" / "setup.py"


class SafetyPackageContractTests(unittest.TestCase):
    def test_setup_registers_pytest_test_requirement(self):
        source = SETUP_PY.read_text(encoding="utf-8")
        self.assertIn('tests_require=["pytest"]', source)


if __name__ == "__main__":
    unittest.main()
