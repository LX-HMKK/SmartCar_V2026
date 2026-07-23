"""Independent field reference and diagnostic tooling for SmartCar."""

from smartcar_tools.field_reference import (
    Bounds2D,
    FieldReference,
    FieldReferenceError,
    FieldGeometry,
    Point2D,
    ReferenceLabel,
    build_field_reference,
    load_field_geometry,
    load_field_reference,
    validate_field_geometry,
)

__all__ = [
    "Bounds2D",
    "FieldReference",
    "FieldReferenceError",
    "FieldGeometry",
    "Point2D",
    "ReferenceLabel",
    "build_field_reference",
    "load_field_geometry",
    "load_field_reference",
    "validate_field_geometry",
]
