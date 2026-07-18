import base64
import json
from pathlib import Path
import sys
import unittest
from urllib import error as urllib_error


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_speech.volcengine_tts import TtsError, VolcengineV1TtsClient


class FakeResponse:
    def __init__(self, body, status=200):
        self._body = body
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self, limit=-1):
        return self._body if limit < 0 else self._body[:limit]


class RecordingOpener:
    def __init__(self, response=None, error=None):
        self.response = response
        self.error = error
        self.calls = []

    def __call__(self, request, timeout):
        self.calls.append((request, timeout))
        if self.error is not None:
            raise self.error
        return self.response


def api_response(**overrides):
    value = {
        "code": 3000,
        "data": base64.b64encode(b"fake-mp3").decode("ascii"),
        "addition": {"duration": "1234"},
    }
    value.update(overrides)
    return json.dumps(value).encode("utf-8")


class VolcengineV1TtsClientTests(unittest.TestCase):
    def make_client(self, opener, **overrides):
        options = {
            "app_id": "app-id",
            "access_token": "secret-token",
            "opener": opener,
            "uuid_factory": lambda: "request-1",
        }
        options.update(overrides)
        return VolcengineV1TtsClient(**options)

    def test_builds_v1_request_and_decodes_audio(self):
        opener = RecordingOpener(FakeResponse(api_response()))
        client = self.make_client(opener)

        result = client.synthesize("  人物描述  ")

        self.assertEqual(result.audio, b"fake-mp3")
        self.assertEqual(result.encoding, "mp3")
        self.assertEqual(result.request_id, "request-1")
        self.assertEqual(result.duration_ms, 1234)
        request, timeout = opener.calls[0]
        self.assertEqual(timeout, 10.0)
        self.assertEqual(request.full_url, "https://openspeech.bytedance.com/api/v1/tts")
        self.assertEqual(request.get_method(), "POST")
        self.assertEqual(request.get_header("Authorization"), "Bearer;secret-token")
        payload = json.loads(request.data.decode("utf-8"))
        self.assertEqual(payload["app"]["appid"], "app-id")
        self.assertEqual(payload["app"]["token"], "access_token")
        self.assertEqual(payload["app"]["cluster"], "volcano_tts")
        self.assertEqual(payload["request"]["reqid"], "request-1")
        self.assertEqual(payload["request"]["text"], "人物描述")
        self.assertEqual(payload["request"]["operation"], "query")

    def test_rejects_non_https_endpoint_before_any_request(self):
        opener = RecordingOpener(FakeResponse(api_response()))
        with self.assertRaisesRegex(ValueError, "HTTPS"):
            self.make_client(opener, api_url="http://example.test/tts")
        self.assertEqual(opener.calls, [])

    def test_api_error_and_invalid_audio_are_bounded_failures(self):
        opener = RecordingOpener(FakeResponse(api_response(code=5501)))
        with self.assertRaisesRegex(TtsError, "api_code:5501"):
            self.make_client(opener).synthesize("test")

        opener = RecordingOpener(FakeResponse(api_response(data="not base64!")))
        with self.assertRaisesRegex(TtsError, "invalid_audio_base64"):
            self.make_client(opener).synthesize("test")

    def test_response_size_and_transport_are_bounded(self):
        opener = RecordingOpener(FakeResponse(b"12345"))
        with self.assertRaisesRegex(TtsError, "response_too_large"):
            self.make_client(opener, max_response_bytes=4).synthesize("test")

        opener = RecordingOpener(
            error=urllib_error.URLError("offline"))
        with self.assertRaisesRegex(TtsError, "transport:URLError"):
            self.make_client(opener).synthesize("test")

    def test_text_bytes_speed_and_encoding_match_v1_limits(self):
        opener = RecordingOpener(FakeResponse(api_response()))
        with self.assertRaisesRegex(TtsError, "text_too_long"):
            self.make_client(opener).synthesize("人" * 342)
        self.assertEqual(opener.calls, [])

        for speed in (0.1, 3.1, float("nan")):
            with self.subTest(speed=speed):
                with self.assertRaisesRegex(ValueError, "0.2 and 3.0"):
                    self.make_client(opener, speed_ratio=speed)
        with self.assertRaisesRegex(ValueError, "mp3, wav, or ogg_opus"):
            self.make_client(opener, encoding="pcm")


if __name__ == "__main__":
    unittest.main()
