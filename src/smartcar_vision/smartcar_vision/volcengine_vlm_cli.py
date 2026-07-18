"""CLI adapter for Volcengine Ark's OpenAI-compatible vision API."""
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


DEFAULT_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
DEFAULT_MODEL = "doubao-1-5-vision-pro-32k-250115"
DEFAULT_API_KEY_ENV = "ARK_API_KEY"
FALLBACK_API_KEY_ENV = "DOUBAO_KEY"
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


def _chat_completions_url(base_url):
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
    if parsed.path.endswith("/chat/completions"):
        return value
    return value + "/chat/completions"


def build_payload(image_bytes, prompt, model, max_tokens):
    """Build the Ark chat-completions request without exposing credentials."""
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
    try:
        token_limit = int(max_tokens)
    except (TypeError, ValueError, OverflowError) as error:
        raise VolcengineVlmError("invalid_max_tokens") from error
    if not 1 <= token_limit <= 2048:
        raise VolcengineVlmError("invalid_max_tokens")

    encoded = base64.b64encode(image_bytes).decode("ascii")
    return {
        "model": model,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": "data:image/jpeg;base64," + encoded,
                    },
                },
            ],
        }],
        "max_tokens": token_limit,
        "temperature": 0.1,
    }


def extract_description(response):
    """Extract either string or typed text content from an Ark response."""
    try:
        content = response["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as error:
        raise VolcengineVlmError("response_missing_content") from error

    if isinstance(content, str):
        description = content.strip()
    elif isinstance(content, list):
        description = "".join(
            str(item.get("text", ""))
            for item in content
            if isinstance(item, dict)
            and item.get("type") in ("text", "output_text")
        ).strip()
    else:
        description = ""
    if not description:
        raise VolcengineVlmError("response_empty_content")
    return description


def request_description(
    image_path,
    prompt,
    model,
    api_key,
    base_url=DEFAULT_BASE_URL,
    timeout_sec=7.0,
    max_tokens=256,
    opener=None,
):
    """Send one bounded Ark request and return only the description text."""
    api_key = str(api_key).strip()
    if not api_key:
        raise VolcengineVlmError("missing_api_key")
    timeout = _positive_timeout(timeout_sec)
    try:
        image_bytes = Path(image_path).read_bytes()
    except OSError as error:
        raise VolcengineVlmError(
            f"image_read_error:{type(error).__name__}") from error
    payload = build_payload(image_bytes, prompt, model, max_tokens)
    encoded_payload = json.dumps(
        payload, ensure_ascii=False, separators=(",", ":")
    ).encode("utf-8")
    request = urlrequest.Request(
        _chat_completions_url(base_url),
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
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--model")
    parser.add_argument("--base-url")
    parser.add_argument("--api-key-env", default=DEFAULT_API_KEY_ENV)
    parser.add_argument("--timeout-sec", type=float, default=7.0)
    parser.add_argument("--max-tokens", type=int, default=256)
    return parser


def main(argv=None, environ=None, stdout=None, stderr=None, opener=None):
    """Run the CLI; injectable streams and opener keep tests offline."""
    environ = os.environ if environ is None else environ
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    arguments = _parser().parse_args(argv)
    key_name = str(arguments.api_key_env).strip()
    api_key = str(environ.get(key_name, "")).strip()
    if not api_key and key_name == DEFAULT_API_KEY_ENV:
        api_key = str(environ.get(FALLBACK_API_KEY_ENV, "")).strip()
    if not api_key:
        print(f"missing_api_key:{key_name}", file=stderr)
        return 2

    model = arguments.model or environ.get("VOLC_ARK_MODEL", DEFAULT_MODEL)
    base_url = arguments.base_url or environ.get(
        "VOLC_ARK_BASE_URL", DEFAULT_BASE_URL)
    try:
        description = request_description(
            image_path=arguments.image,
            prompt=arguments.prompt,
            model=model,
            api_key=api_key,
            base_url=base_url,
            timeout_sec=arguments.timeout_sec,
            max_tokens=arguments.max_tokens,
            opener=opener,
        )
    except VolcengineVlmError as error:
        print(str(error), file=stderr)
        return 1
    print(description, file=stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
