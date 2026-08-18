"""CLI adapter for Volcengine Ark's OpenAI-compatible Responses API."""
import argparse
import base64
import json
import math
import os
from pathlib import Path
import socket
import sys
from urllib import error as urlerror
from urllib import parse as urlparse
from urllib import request as urlrequest

import yaml


DEFAULT_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
DEFAULT_TIMEOUT_SEC = 30.0
LOCAL_CREDENTIALS_RELATIVE_PATH = (
    Path("config") / "volcengine_ark.local.yaml"
)
MAX_IMAGE_BYTES = 5 * 1024 * 1024
MAX_RESPONSE_BYTES = 1024 * 1024


class VolcengineVlmError(RuntimeError):
    """Expected request or response failure with a stable status string."""


class _RejectRedirect(urlrequest.HTTPRedirectHandler):
    """Keep the bearer credential on the configured HTTPS origin."""

    def redirect_request(self, *_args, **_kwargs):
        return None


def _open_without_redirect(request, timeout):
    opener = urlrequest.build_opener(_RejectRedirect())
    return opener.open(request, timeout=timeout)


def _positive_timeout(value):
    try:
        timeout = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise VolcengineVlmError("invalid_timeout") from error
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise VolcengineVlmError("invalid_timeout")
    return timeout


def _responses_url(base_url):
    value = str(base_url).strip().rstrip("/")
    parsed = urlparse.urlsplit(value)
    if (
        parsed.scheme != "https"
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        raise VolcengineVlmError("invalid_base_url")
    if parsed.path.endswith("/responses"):
        return value
    return value + "/responses"


def build_payload(image_bytes, prompt, model):
    """Build an Ark Responses request without exposing credentials."""
    if not isinstance(image_bytes, bytes) or not image_bytes:
        raise VolcengineVlmError("image_empty")
    if len(image_bytes) > MAX_IMAGE_BYTES:
        raise VolcengineVlmError("image_too_large")
    prompt = str(prompt).strip()
    model = str(model).strip()
    if not prompt:
        raise VolcengineVlmError("prompt_empty")
    if not model:
        raise VolcengineVlmError("model_empty")
    encoded = base64.b64encode(image_bytes).decode("ascii")
    return {
        "model": model,
        "input": [{
            "role": "user",
            "content": [
                {
                    "type": "input_image",
                    "image_url": "data:image/jpeg;base64," + encoded,
                },
                {"type": "input_text", "text": prompt},
            ],
        }],
    }


def extract_description(response):
    """Extract text from an Ark Responses result."""
    description = ""
    if isinstance(response, dict):
        status = str(response.get("status", "")).strip()
        if status and status != "completed":
            details = response.get("incomplete_details")
            reason = details.get("reason") if isinstance(details, dict) else ""
            suffix = f":{reason}" if reason else ""
            raise VolcengineVlmError(f"response_{status}{suffix}")
        output_text = response.get("output_text")
        if isinstance(output_text, str):
            description = output_text.strip()
        if not description:
            output = response.get("output")
            if isinstance(output, list):
                description = "".join(
                    str(content.get("text", ""))
                    for item in output
                    if isinstance(item, dict)
                    for content in item.get("content", ())
                    if isinstance(content, dict)
                    and content.get("type") in ("output_text", "text")
                ).strip()
    if not description:
        raise VolcengineVlmError("response_empty_content")
    return description


def _credentials_file():
    """Find the ignored root-config credential YAML from source or colcon installs."""
    candidates = [Path.cwd() / LOCAL_CREDENTIALS_RELATIVE_PATH]
    module_path = Path(__file__).resolve()
    candidates.extend(
        parent / LOCAL_CREDENTIALS_RELATIVE_PATH
        for parent in (module_path.parent, *module_path.parents)
    )
    return next((path for path in candidates if path.is_file()), None)


def _load_api_key():
    api_key = os.environ.get("ARK_API_KEY", "").strip()
    if api_key:
        return api_key

    credentials_file = _credentials_file()
    if credentials_file is None:
        raise VolcengineVlmError("credentials_file_missing")
    try:
        document = yaml.safe_load(
            credentials_file.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise VolcengineVlmError("credentials_file_invalid") from error
    if not isinstance(document, dict):
        raise VolcengineVlmError("credentials_file_invalid")
    ark = document.get("ark")
    api_key = ark.get("api_key") if isinstance(ark, dict) else None
    if not isinstance(api_key, str):
        raise VolcengineVlmError("credentials_file_invalid")
    api_key = api_key.strip()
    if not api_key:
        raise VolcengineVlmError("missing_api_key")
    return api_key


def _load_vlm_config(path):
    """Read the canonical model and prompt from the vision parameter YAML."""
    try:
        document = yaml.safe_load(
            Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise VolcengineVlmError("vision_config_invalid") from error
    if not isinstance(document, dict):
        raise VolcengineVlmError("vision_config_invalid")
    node = document.get("vision_node")
    parameters = node.get("ros__parameters") if isinstance(node, dict) else None
    if not isinstance(parameters, dict):
        raise VolcengineVlmError("vision_config_invalid")
    model = str(parameters.get("vlm_model", "")).strip()
    prompt = str(parameters.get("default_prompt", "")).strip()
    if not model or not prompt:
        raise VolcengineVlmError("vision_config_invalid")
    return model, prompt


def request_description(
    image_path,
    prompt,
    model,
    api_key,
    base_url=DEFAULT_BASE_URL,
    timeout_sec=DEFAULT_TIMEOUT_SEC,
    opener=None,
):
    """Read a local image and send it to Ark."""
    try:
        image_bytes = Path(image_path).read_bytes()
    except OSError as error:
        raise VolcengineVlmError(
            f"image_read_error:{type(error).__name__}") from error
    return request_description_bytes(
        image_bytes=image_bytes,
        prompt=prompt,
        model=model,
        api_key=api_key,
        base_url=base_url,
        timeout_sec=timeout_sec,
        opener=opener,
    )


def request_description_bytes(
    image_bytes,
    prompt,
    model,
    api_key,
    base_url=DEFAULT_BASE_URL,
    timeout_sec=DEFAULT_TIMEOUT_SEC,
    opener=None,
):
    """Send one in-memory JPEG to Ark and return only its final text."""
    api_key = str(api_key).strip()
    if not api_key:
        raise VolcengineVlmError("missing_api_key")
    timeout = _positive_timeout(timeout_sec)
    payload = build_payload(image_bytes, prompt, model)
    encoded_payload = json.dumps(
        payload, ensure_ascii=False, separators=(",", ":")
    ).encode("utf-8")
    request = urlrequest.Request(
        _responses_url(base_url),
        data=encoded_payload,
        headers={
            "Authorization": "Bearer " + api_key,
            "Content-Type": "application/json",
        },
        method="POST",
    )

    request_opener = _open_without_redirect if opener is None else opener
    try:
        with request_opener(request, timeout=timeout) as response:
            status = getattr(response, "status", None)
            if status is None:
                status = response.getcode()
            body = response.read(MAX_RESPONSE_BYTES + 1)
    except urlerror.HTTPError as error:
        raise VolcengineVlmError(f"http_error:{error.code}") from error
    except (TimeoutError, socket.timeout) as error:
        raise VolcengineVlmError("http_timeout") from error
    except urlerror.URLError as error:
        reason = getattr(error, "reason", None)
        status = (
            "http_timeout"
            if isinstance(reason, (TimeoutError, socket.timeout))
            else "http_transport_error"
        )
        raise VolcengineVlmError(status) from error
    except OSError as error:
        raise VolcengineVlmError("http_transport_error") from error

    if status is not None and not 200 <= int(status) < 300:
        raise VolcengineVlmError(f"http_error:{status}")
    if len(body) > MAX_RESPONSE_BYTES:
        raise VolcengineVlmError("response_too_large")
    try:
        decoded = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise VolcengineVlmError("response_invalid_json") from error
    return extract_description(decoded)


def _parser():
    parser = argparse.ArgumentParser(
        description="Describe a JPEG with Volcengine Ark")
    parser.add_argument("--image", required=True)
    parser.add_argument("--config-file", required=True)
    parser.add_argument("--base-url")
    parser.add_argument(
        "--timeout-sec", type=float, default=DEFAULT_TIMEOUT_SEC)
    return parser


def main(argv=None, stdout=None, stderr=None, opener=None):
    """Run the CLI; injectable streams and opener keep tests offline."""
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    arguments = _parser().parse_args(argv)
    try:
        api_key = _load_api_key()
    except VolcengineVlmError as error:
        print(str(error), file=stderr)
        return 2

    try:
        model, prompt = _load_vlm_config(arguments.config_file)
    except VolcengineVlmError as error:
        print(str(error), file=stderr)
        return 2
    base_url = arguments.base_url or DEFAULT_BASE_URL
    try:
        description = request_description(
            image_path=arguments.image,
            prompt=prompt,
            model=model,
            api_key=api_key,
            base_url=base_url,
            timeout_sec=arguments.timeout_sec,
            opener=opener,
        )
    except VolcengineVlmError as error:
        print(str(error), file=stderr)
        return 1
    print(description, file=stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
