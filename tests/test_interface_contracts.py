"""Contract tests for shared SmartCar vision service definitions."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SERVICE_DIRECTORY = REPOSITORY_ROOT / "src" / "smartcar_interfaces" / "srv"


def service_fields(service_name: str) -> tuple[list[str], list[str]]:
    """Return request and response field declarations without comments."""
    request_fields: list[str] = []
    response_fields: list[str] = []
    current_fields = request_fields

    for line in (SERVICE_DIRECTORY / service_name).read_text(encoding="utf-8").splitlines():
        field = line.split("#", maxsplit=1)[0].strip()
        if not field:
            continue
        if field == "---":
            current_fields = response_fields
            continue
        current_fields.append(field)

    return request_fields, response_fields


class VisionServiceInterfaceContractTests(unittest.TestCase):
    def test_read_qr_service_fields_match_contract(self) -> None:
        self.assertEqual(
            service_fields("ReadQr.srv"),
            (
                [
                    "builtin_interfaces/Time not_before",
                    "float32 timeout_sec",
                ],
                [
                    "bool success",
                    "string content",
                    "string status",
                ],
            ),
        )

    def test_describe_scene_service_fields_match_contract(self) -> None:
        self.assertEqual(
            service_fields("DescribeScene.srv"),
            (
                [
                    "builtin_interfaces/Time not_before",
                    "float32 timeout_sec",
                    "string prompt",
                ],
                [
                    "bool success",
                    "bool fallback_used",
                    "string description",
                    "string status",
                ],
            ),
        )


if __name__ == "__main__":
    unittest.main()
