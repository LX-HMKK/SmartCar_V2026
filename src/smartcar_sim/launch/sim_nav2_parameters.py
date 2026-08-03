"""Materialize the local Gazebo Nav2 parameter file before lifecycle startup."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

import yaml


def _load_mapping(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read Nav2 parameter file {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"Nav2 parameter file must be a YAML mapping: {path}")
    return document


def _deep_merge(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    """Merge Nav2 parameter mappings without mutating either document."""
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        current = merged.get(key)
        if isinstance(current, Mapping) and isinstance(value, Mapping):
            merged[key] = _deep_merge(current, value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def write_merged_nav2_parameters(
    base_params: Path,
    overlay: Path,
    destination: Path,
) -> Path:
    """Write the exact base-plus-local-overlay configuration atomically."""
    merged = _deep_merge(_load_mapping(base_params), _load_mapping(overlay))
    rendered = yaml.safe_dump(merged, allow_unicode=False, sort_keys=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=destination.parent,
        prefix=f".{destination.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(rendered)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination
