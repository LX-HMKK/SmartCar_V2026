"""Contract tests for shared SmartCar service definitions."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SERVICE_DIRECTORY = REPOSITORY_ROOT / "src" / "smartcar_interfaces" / "srv"


def service_fields(service_name: str) -> tuple[list[str], list[str]]:
    """Return request and response field declarations without comments."""
    request_fields: list[str] = []
    response_fields: list[str] = []
    current_fields = request_fields

    source = (SERVICE_DIRECTORY / service_name).read_text(encoding="utf-8")
    for line in source.splitlines():
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
                ],
                [
                    "bool success",
                    "string description",
                    "string status",
                ],
            ),
        )

    def test_prepare_motion_binds_direction_generation_and_action(
            self) -> None:
        self.assertEqual(
            service_fields("PrepareMotion.srv"),
            (
                [
                    "uint8 FORWARD=1",
                    "uint8 REVERSE=2",
                    "uint8 FORWARD_RECOVERY=3",
                    "uint8 direction",
                    "uint64 generation",
                    "unique_identifier_msgs/UUID action_uuid",
                ],
                [
                    "bool success",
                    "string status",
                    "uint64 boot_epoch",
                    "uint64 lease_id",
                ],
            ),
        )

    def test_motion_lease_operations_require_complete_identity(self) -> None:
        identity = [
            "uint64 boot_epoch",
            "uint64 lease_id",
            "uint64 generation",
            "unique_identifier_msgs/UUID action_uuid",
        ]
        response = ["bool success", "string status"]
        for service_name in (
            "ActivateMotion.srv",
            "RenewMotion.srv",
            "StopMotion.srv",
        ):
            with self.subTest(service=service_name):
                self.assertEqual(
                    service_fields(service_name),
                    (identity, response),
                )

    def test_static_steering_hold_has_no_speed_field(
            self) -> None:
        self.assertEqual(
            service_fields("HoldSteeringCalibration.srv"),
            (
                [
                    "float64 steering_angle",
                    "float64 duration_sec",
                ],
                [
                    "bool success",
                    "string status",
                ],
            ),
        )


if __name__ == "__main__":
    unittest.main()
