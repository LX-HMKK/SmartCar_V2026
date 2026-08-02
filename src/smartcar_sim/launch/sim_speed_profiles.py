"""Constrained Nav2 speed profiles for the local Gazebo launch only.

The real vehicle keeps its 0.15 m/s Nav2 configuration in ``smartcar_nav2``.
This module is imported only by ``sim.launch.py`` and produces the temporary
simulation-only configuration files used for an explicit speed trial.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class SimSpeedProfile:
    """One allowed local-Gazebo linear-speed trial."""

    name: str
    linear_speed_mps: float
    overlay_filename: str | None


SIM_SPEED_PROFILES = {
    "baseline": SimSpeedProfile("baseline", 0.15, None),
    "0.20": SimSpeedProfile("0.20", 0.20, "nav2_speed_020.yaml"),
    "0.25": SimSpeedProfile("0.25", 0.25, "nav2_speed_025.yaml"),
}

# This module is imported only by the local Gazebo launcher. It stays in
# lockstep with nav2_keepout_filter.yaml and the simulated Ackermann SDF.
SIMULATION_MINIMUM_TURNING_RADIUS_M = 0.22


def resolve_sim_speed_profile(value: object, config_dir: Path) -> SimSpeedProfile:
    """Resolve one exact supported profile and require its installed overlay."""
    profile_name = str(value).strip()
    profile = SIM_SPEED_PROFILES.get(profile_name)
    if profile is None:
        supported = ", ".join(SIM_SPEED_PROFILES)
        raise ValueError(
            f"sim_speed_profile must be one of: {supported}; got {value!r}"
        )
    if profile.overlay_filename is not None:
        overlay_path = config_dir / profile.overlay_filename
        if not overlay_path.is_file():
            raise ValueError(
                f"sim speed profile overlay is missing: {overlay_path}"
            )
    return profile


def speed_overlay_path(profile: SimSpeedProfile, config_dir: Path) -> Path | None:
    """Return the small speed-only overlay for a non-baseline trial."""
    if profile.overlay_filename is None:
        return None
    return config_dir / profile.overlay_filename


def _load_mapping(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read Nav2 overlay {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"Nav2 overlay must be a YAML mapping: {path}")
    return document


def _deep_merge(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    """Merge Nav2 parameter mappings without mutating either source document."""
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        current = merged.get(key)
        if isinstance(current, Mapping) and isinstance(value, Mapping):
            merged[key] = _deep_merge(current, value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _speed_parameters(
    document: Mapping[str, Any],
) -> tuple[float, float, list[Any], list[Any]]:
    """Extract the only parameters a speed profile may override."""
    try:
        controller = document["controller_server"]["ros__parameters"]
        smoother = document["velocity_smoother"]["ros__parameters"]
        forward_avoidance = controller["ForwardAvoidance"]
        forward_handoff = controller["ForwardHandoff"]
        desired = float(forward_avoidance["desired_linear_vel"])
        handoff_desired = float(forward_handoff["desired_linear_vel"])
        maximum = smoother["max_velocity"]
        minimum = smoother["min_velocity"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "sim speed overlay must set ForwardAvoidance, ForwardHandoff, and "
            "velocity_smoother max_velocity/min_velocity"
        ) from error
    if not isinstance(maximum, list) or not isinstance(minimum, list):
        raise ValueError("sim speed overlay velocity limits must be lists")
    return desired, handoff_desired, maximum, minimum


def validate_speed_overlay(profile: SimSpeedProfile, overlay_path: Path) -> None:
    """Reject malformed or scope-expanding speed overlays before Nav2 starts."""
    if profile.overlay_filename is None:
        raise ValueError("baseline profile has no speed overlay")
    document = _load_mapping(overlay_path)
    if set(document) != {"controller_server", "velocity_smoother"}:
        raise ValueError(
            "sim speed overlay may only target controller_server and "
            "velocity_smoother"
        )
    controller = document.get("controller_server")
    smoother = document.get("velocity_smoother")
    if not isinstance(controller, Mapping) or not isinstance(smoother, Mapping):
        raise ValueError("sim speed overlay node entries must be mappings")
    controller_parameters = controller.get("ros__parameters")
    smoother_parameters = smoother.get("ros__parameters")
    if not isinstance(controller_parameters, Mapping) or not isinstance(
        smoother_parameters, Mapping
    ):
        raise ValueError("sim speed overlay ros__parameters entries must be mappings")
    if set(controller_parameters) != {"ForwardAvoidance", "ForwardHandoff"} or set(smoother_parameters) != {
        "max_velocity",
        "min_velocity",
    }:
        raise ValueError(
            "sim speed overlay attempts to change parameters outside its speed cap"
        )
    forward_avoidance = controller_parameters.get("ForwardAvoidance")
    forward_handoff = controller_parameters.get("ForwardHandoff")
    if not isinstance(forward_avoidance, Mapping) or set(forward_avoidance) != {
        "desired_linear_vel", "forward_max_angular_velocity"
    }:
        raise ValueError(
            "sim speed overlay may only change "
            "ForwardAvoidance.desired_linear_vel/forward_max_angular_velocity"
        )
    if not isinstance(forward_handoff, Mapping) or set(forward_handoff) != {
        "desired_linear_vel", "forward_max_angular_velocity"
    }:
        raise ValueError(
            "sim speed overlay may only change "
            "ForwardHandoff.desired_linear_vel/forward_max_angular_velocity"
        )

    desired, handoff_desired, maximum, minimum = _speed_parameters(document)
    expected = profile.linear_speed_mps
    if (
        len(maximum) != 3
        or len(minimum) != 3
        or desired != expected
        or handoff_desired != expected
        or abs(
            float(forward_avoidance["forward_max_angular_velocity"])
            - expected / SIMULATION_MINIMUM_TURNING_RADIUS_M
        ) > 1.0e-9
        or abs(
            float(forward_handoff["forward_max_angular_velocity"])
            - expected / SIMULATION_MINIMUM_TURNING_RADIUS_M
        ) > 1.0e-9
        or any(
            abs(float(actual) - desired) > 1.0e-9
            for actual, desired in zip(
                maximum,
                (expected, 0.0,
                 expected / SIMULATION_MINIMUM_TURNING_RADIUS_M),
            )
        )
        or any(
            abs(float(actual) - desired) > 1.0e-9
            for actual, desired in zip(
                minimum,
                (-expected, 0.0,
                 -expected / SIMULATION_MINIMUM_TURNING_RADIUS_M),
            )
        )
    ):
        raise ValueError(
            f"sim speed overlay {overlay_path} does not match the "
            f"{profile.name} m/s profile"
        )


def write_merged_nav2_overlay(
    keepout_overlay: Path,
    speed_overlay: Path,
    destination: Path,
) -> Path:
    """Write a temporary keepout+speed overlay atomically for one simulator run."""
    speed_document = _load_mapping(speed_overlay)
    merged = _deep_merge(_load_mapping(keepout_overlay), speed_document)
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


def write_merged_nav2_parameters(
    base_params: Path,
    overlay: Path,
    destination: Path,
) -> Path:
    """Materialize the exact base-plus-overlay configuration for local Nav2.

    Nav2 controller plugins are instantiated during lifecycle configuration.
    In this Humble stack, a later ``--params-file`` can add nested values but
    does not reliably replace an already supplied controller ``plugin`` type.
    Materializing one merged file before the process starts gives plugin
    selection and all nested settings one unambiguous source of truth.
    """
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
