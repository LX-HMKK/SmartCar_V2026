"""Offline tests for the Volcengine Ark vision CLI adapter."""
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest import mock
from urllib import error as urlerror


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.volcengine_vlm_cli import (  # noqa: E402
    _RejectRedirect,
    VolcengineVlmError,
    _load_api_key,
    extract_description,
    main,
    request_description,
)


class FakeResponse:
    def __init__(self, payload, status=200):
        self.status = status
        self._body = json.dumps(payload, ensure_ascii=False).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self, limit=-1):
        return self._body if limit < 0 else self._body[:limit]


class VolcengineVlmCliTests(unittest.TestCase):
    def test_redirects_are_rejected_before_authorization_can_move_hosts(self):
        response = FakeResponse({
            "output_text": "人物站立",
        })
        with tempfile.TemporaryDirectory() as directory, mock.patch(
            "smartcar_vision.volcengine_vlm_cli.urlrequest.build_opener"
        ) as build_opener:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"jpeg")
            build_opener.return_value.open.return_value = response
            description = request_description(
                image_path=image_path,
                prompt="描述",
                model="model-id",
                api_key="secret-key",
            )

        self.assertEqual(description, "人物站立")
        handler = build_opener.call_args.args[0]
        self.assertIsInstance(handler, _RejectRedirect)
        self.assertIsNone(handler.redirect_request())

    def test_request_uses_ark_schema_and_returns_plain_description(self):
        captured = {}

        def opener(request, timeout):
            captured["request"] = request
            captured["timeout"] = timeout
            return FakeResponse({
                "output_text": " 穿白衣的人正在挥手。 ",
            })

        with tempfile.TemporaryDirectory() as directory:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"jpeg")
            result = request_description(
                image_path=image_path,
                prompt="描述人物",
                model="model-id",
                api_key="secret-key",
                timeout_sec=3.5,
                opener=opener,
            )

        self.assertEqual(result, "穿白衣的人正在挥手。")
        self.assertEqual(captured["timeout"], 3.5)
        request = captured["request"]
        self.assertEqual(
            request.full_url,
            "https://ark.cn-beijing.volces.com/api/v3/responses",
        )
        self.assertEqual(
            request.get_header("Authorization"), "Bearer secret-key")
        payload = json.loads(request.data.decode("utf-8"))
        self.assertEqual(payload["model"], "model-id")
        self.assertEqual(
            payload["input"][0]["content"][1]["text"], "描述人物")
        self.assertEqual(
            payload["input"][0]["content"][0]["type"], "input_image")
        image_url = payload["input"][0]["content"][0]["image_url"]
        self.assertTrue(image_url.startswith("data:image/jpeg;base64,"))
        self.assertEqual(payload["max_output_tokens"], 256)
        self.assertNotIn("secret-key", request.data.decode("utf-8"))

    def test_typed_text_content_is_supported(self):
        response = {
            "output": [{
                "content": [
                    {"type": "output_text", "text": "人物"},
                    {"type": "text", "text": "正在行走"},
                    {"type": "image", "text": "ignored"},
                ],
            }],
        }
        self.assertEqual(extract_description(response), "人物正在行走")

    def test_missing_or_empty_content_is_rejected(self):
        for response in ({}, {"output": []}, {"output_text": "  "}):
            with self.subTest(response=response):
                with self.assertRaises(VolcengineVlmError):
                    extract_description(response)

    def test_main_reads_local_credentials_and_writes_description(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        captured = {}

        def opener(request, timeout):
            captured["authorization"] = request.get_header("Authorization")
            return FakeResponse({
                "output_text": "人物站立",
            })

        with tempfile.TemporaryDirectory() as directory, mock.patch(
            "smartcar_vision.volcengine_vlm_cli._credentials_file"
        ) as credentials_file:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"jpeg")
            credential_path = Path(directory) / "credentials.yaml"
            credential_path.write_text(
                "ark:\n  api_key: local-key\n", encoding="utf-8")
            credentials_file.return_value = credential_path
            code = main(
                [
                    "--image", str(image_path),
                    "--prompt", "描述",
                    "--model", "model-id",
                ],
                stdout=stdout,
                stderr=stderr,
                opener=opener,
            )

        self.assertEqual(code, 0)
        self.assertEqual(stdout.getvalue(), "人物站立\n")
        self.assertEqual(stderr.getvalue(), "")
        self.assertEqual(captured["authorization"], "Bearer local-key")

    def test_main_fails_fast_without_local_credentials(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch(
            "smartcar_vision.volcengine_vlm_cli._credentials_file",
            return_value=None,
        ):
            code = main(
                ["--image", "unused.jpg", "--prompt", "描述"],
                stdout=stdout,
                stderr=stderr,
                opener=lambda *_args, **_kwargs: self.fail("must not call API"),
            )
        self.assertEqual(code, 2)
        self.assertEqual(stdout.getvalue(), "")
        self.assertEqual(stderr.getvalue(), "credentials_file_missing\n")

    def test_empty_local_api_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch(
            "smartcar_vision.volcengine_vlm_cli._credentials_file"
        ) as credentials_file:
            credential_path = Path(directory) / "credentials.yaml"
            credential_path.write_text("ark:\n  api_key: ''\n", encoding="utf-8")
            credentials_file.return_value = credential_path
            with self.assertRaisesRegex(VolcengineVlmError, "^missing_api_key$"):
                _load_api_key()

    def test_http_timeout_has_stable_status(self):
        def opener(_request, timeout):
            del timeout
            raise urlerror.URLError(socket.timeout())

        with tempfile.TemporaryDirectory() as directory:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"jpeg")
            with self.assertRaisesRegex(
                VolcengineVlmError, "^http_timeout$"
            ):
                request_description(
                    image_path=image_path,
                    prompt="描述",
                    model="model-id",
                    api_key="secret-key",
                    opener=opener,
                )

    def test_insecure_or_credential_bearing_base_url_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"jpeg")
            for base_url in (
                "http://ark.example/api/v3",
                "https://user:password@ark.example/api/v3",
                "https://ark.example/api/v3?key=secret",
            ):
                with self.subTest(base_url=base_url):
                    with self.assertRaisesRegex(
                        VolcengineVlmError, "^invalid_base_url$"
                    ):
                        request_description(
                            image_path=image_path,
                            prompt="描述",
                            model="model-id",
                            api_key="secret-key",
                            base_url=base_url,
                            opener=lambda *_args, **_kwargs: self.fail(
                                "must reject before API call"),
                        )


if __name__ == "__main__":
    unittest.main()
