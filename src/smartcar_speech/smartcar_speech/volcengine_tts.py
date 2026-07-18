"""ROS-independent client for Volcengine's V1 non-streaming TTS API."""
import base64
import binascii
from dataclasses import dataclass
import json
import math
from urllib import error as urllib_error
from urllib import parse as urllib_parse
from urllib import request as urllib_request
import uuid


class TtsError(RuntimeError):
    """A bounded, credential-safe TTS request failure."""

    def __init__(self, public_status):
        self.public_status = str(public_status)
        super().__init__(self.public_status)


class _RejectRedirect(urllib_request.HTTPRedirectHandler):
    """Keep the bearer token on the configured HTTPS origin."""

    def redirect_request(self, *_args, **_kwargs):
        return None


@dataclass(frozen=True)
class SynthesisResult:
    audio: bytes
    encoding: str
    request_id: str
    duration_ms: int = None


class VolcengineV1TtsClient:
    """Synthesize text through the documented V1 JSON/base64 contract."""

    _ENCODINGS = frozenset(("mp3", "wav", "ogg_opus"))
    _MAX_TEXT_BYTES = 1024

    def __init__(
        self,
        app_id,
        access_token,
        api_url="https://openspeech.bytedance.com/api/v1/tts",
        cluster="volcano_tts",
        voice_type="zh_male_M392_conversation_wvae_bigtts",
        user_id="smartcar_rdk_x5",
        encoding="mp3",
        speed_ratio=1.0,
        timeout_sec=10.0,
        max_response_bytes=8 * 1024 * 1024,
        opener=None,
        uuid_factory=None,
    ):
        self._app_id = self._required("app_id", app_id)
        self._access_token = self._required("access_token", access_token)
        self._cluster = self._required("cluster", cluster)
        self._voice_type = self._required("voice_type", voice_type)
        self._user_id = self._required("user_id", user_id)
        self._api_url = self._validate_url(api_url)
        self._encoding = str(encoding).strip().lower()
        if self._encoding not in self._ENCODINGS:
            raise ValueError("encoding must be mp3, wav, or ogg_opus")
        self._speed_ratio = float(speed_ratio)
        if (
            not math.isfinite(self._speed_ratio)
            or not 0.2 <= self._speed_ratio <= 3.0
        ):
            raise ValueError("speed_ratio must be between 0.2 and 3.0")
        self._timeout_sec = float(timeout_sec)
        if not math.isfinite(self._timeout_sec) or self._timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and positive")
        self._max_response_bytes = int(max_response_bytes)
        if self._max_response_bytes <= 0:
            raise ValueError("max_response_bytes must be positive")
        self._opener = opener or urllib_request.build_opener(
            _RejectRedirect()).open
        self._uuid_factory = uuid_factory or uuid.uuid4

    @staticmethod
    def _required(name, value):
        result = str(value).strip()
        if not result:
            raise ValueError(f"{name} must be nonempty")
        return result

    @staticmethod
    def _validate_url(value):
        result = str(value).strip()
        parsed = urllib_parse.urlsplit(result)
        if parsed.scheme != "https" or not parsed.netloc:
            raise ValueError("api_url must be an absolute HTTPS URL")
        if parsed.username is not None or parsed.password is not None:
            raise ValueError("api_url must not contain credentials")
        return result

    def synthesize(self, text):
        normalized = str(text).strip()
        if not normalized:
            raise ValueError("text must be nonempty")
        try:
            text_size = len(normalized.encode("utf-8"))
        except UnicodeEncodeError as error:
            raise TtsError("invalid_text_encoding") from error
        if text_size > self._MAX_TEXT_BYTES:
            raise TtsError("text_too_long")
        request_id = str(self._uuid_factory())
        payload = {
            "app": {
                "appid": self._app_id,
                "token": "access_token",
                "cluster": self._cluster,
            },
            "user": {"uid": self._user_id},
            "audio": {
                "voice_type": self._voice_type,
                "encoding": self._encoding,
                "speed_ratio": self._speed_ratio,
            },
            "request": {
                "reqid": request_id,
                "text": normalized,
                "operation": "query",
            },
        }
        body = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        request = urllib_request.Request(
            self._api_url,
            data=body,
            headers={
                "Authorization": "Bearer;" + self._access_token,
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )

        try:
            with self._opener(request, timeout=self._timeout_sec) as response:
                status = getattr(response, "status", None)
                if status is None:
                    status = response.getcode()
                raw_response = response.read(self._max_response_bytes + 1)
        except urllib_error.HTTPError as error:
            raise TtsError(f"http_status:{error.code}") from error
        except (urllib_error.URLError, TimeoutError, OSError) as error:
            raise TtsError(f"transport:{type(error).__name__}") from error

        if status != 200:
            raise TtsError(f"http_status:{status}")
        if len(raw_response) > self._max_response_bytes:
            raise TtsError("response_too_large")
        try:
            response_data = json.loads(raw_response.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise TtsError("invalid_json_response") from error
        if not isinstance(response_data, dict):
            raise TtsError("invalid_json_response")

        api_code = response_data.get("code")
        if str(api_code) != "3000":
            raise TtsError(f"api_code:{api_code}")
        audio_base64 = response_data.get("data")
        if not isinstance(audio_base64, str) or not audio_base64:
            raise TtsError("missing_audio_data")
        try:
            audio = base64.b64decode(audio_base64, validate=True)
        except (ValueError, binascii.Error) as error:
            raise TtsError("invalid_audio_base64") from error
        if not audio:
            raise TtsError("empty_audio_data")

        duration_ms = None
        addition = response_data.get("addition")
        if isinstance(addition, dict) and addition.get("duration") is not None:
            try:
                duration_ms = int(addition["duration"])
            except (TypeError, ValueError, OverflowError):
                duration_ms = None
        return SynthesisResult(
            audio=audio,
            encoding=self._encoding,
            request_id=request_id,
            duration_ms=duration_ms,
        )
