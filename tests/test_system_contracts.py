"""Expose the package-owned system contracts to the root local suite."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


TEST_FILE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "smartcar_bringup"
    / "test"
    / "test_system_contracts.py"
)
SPEC = spec_from_file_location("smartcar_system_contracts", TEST_FILE)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

SystemContractTests = MODULE.SystemContractTests
