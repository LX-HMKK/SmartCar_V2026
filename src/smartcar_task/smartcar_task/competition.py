"""Pure QR helpers used by the competition mission and display."""

from smartcar_task.c_zone_direction import CLOCKWISE, COUNTERCLOCKWISE


QR_PARITY_ODD = "odd"
QR_PARITY_EVEN = "even"
QR_PARITY_UNRECOGNIZED = "unrecognized"
QR_UNRECOGNIZED_TEXT = "未识别"
C_ZONE_DIRECTION_COUNTERCLOCKWISE_TEXT = "逆时针"
C_ZONE_DIRECTION_CLOCKWISE_TEXT = "顺时针"


def classify_qr_parity(content):
    """Classify one QR payload without guessing when it is ambiguous."""
    value = str(content).strip()
    has_odd = "奇数" in value
    has_even = "偶数" in value
    if has_odd and has_even:
        # A race route must not select a C-zone branch from ambiguous text.
        return QR_PARITY_UNRECOGNIZED
    if has_odd:
        return QR_PARITY_ODD
    if has_even:
        return QR_PARITY_EVEN
    if value.isdecimal():
        return QR_PARITY_ODD if int(value) % 2 else QR_PARITY_EVEN
    return QR_PARITY_UNRECOGNIZED


def qr_parity_text(content):
    """Return the compact text shown on the competition display."""
    parity = classify_qr_parity(content)
    if parity == QR_PARITY_ODD:
        return "奇数"
    if parity == QR_PARITY_EVEN:
        return "偶数"
    return QR_UNRECOGNIZED_TEXT


def c_zone_direction_for_qr(content):
    """Select the authorized C-zone variant from one raw QR payload.

    The authored counterclockwise route is the deterministic fallback for an
    unreadable QR result, so semantic degradation still completes the route.
    """
    if classify_qr_parity(content) == QR_PARITY_EVEN:
        return CLOCKWISE
    return COUNTERCLOCKWISE


def c_zone_direction_text(direction):
    """Return the compact direction label used by the competition UI."""
    if direction == CLOCKWISE:
        return C_ZONE_DIRECTION_CLOCKWISE_TEXT
    if direction == COUNTERCLOCKWISE:
        return C_ZONE_DIRECTION_COUNTERCLOCKWISE_TEXT
    raise ValueError(f"unknown C-zone direction {direction!r}")
