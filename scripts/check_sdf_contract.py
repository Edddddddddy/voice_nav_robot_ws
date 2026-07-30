#!/usr/bin/env python3
"""Validate the current native-DiffDrive teaching SDF structurally."""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class SdfContractError(ValueError):
    """The generated SDF does not match the current teaching checkpoint."""


EXPECTED_PLUGIN_TEXT = {
    "left_joint": "left_wheel_joint",
    "right_joint": "right_wheel_joint",
    "frame_id": "odom",
    "child_frame_id": "base_footprint",
}

EXPECTED_PLUGIN_NUMBERS = {
    "wheel_separation": 0.4,
    "wheel_radius": 0.035,
    "odom_publish_frequency": 50.0,
    "min_linear_velocity": -0.20,
    "max_linear_velocity": 0.40,
    "min_angular_velocity": -1.20,
    "max_angular_velocity": 1.20,
    "min_linear_acceleration": -0.50,
    "max_linear_acceleration": 0.50,
    "min_angular_acceleration": -1.50,
    "max_angular_acceleration": 1.50,
}


def required_text(parent: element_tree.Element, field: str) -> str:
    value = parent.findtext(field)
    if value is None or not value.strip():
        raise SdfContractError(f"DiffDrive plugin is missing {field}")
    return value.strip()


def required_number(parent: element_tree.Element, field: str) -> float:
    value = required_text(parent, field)
    try:
        return float(value)
    except ValueError as error:
        raise SdfContractError(
            f"DiffDrive {field} must be numeric; found {value!r}"
        ) from error


def validate_sdf(sdf_path: Path) -> None:
    try:
        root = element_tree.parse(sdf_path).getroot()
    except (OSError, element_tree.ParseError) as error:
        raise SdfContractError(f"cannot parse {sdf_path}: {error}") from error

    models = root.findall("./model")
    matching_models = [
        model for model in models if model.get("name") == "voice_nav_robot"
    ]
    if len(matching_models) != 1:
        raise SdfContractError(
            "expected exactly one model named voice_nav_robot"
        )
    model = matching_models[0]

    plugins = [
        plugin
        for plugin in model.findall("./plugin")
        if plugin.get("filename") == "gz-sim-diff-drive-system"
    ]
    if len(plugins) != 1:
        raise SdfContractError(
            "expected exactly one model DiffDrive plugin with filename "
            "gz-sim-diff-drive-system"
        )
    plugin = plugins[0]

    for field, expected in EXPECTED_PLUGIN_TEXT.items():
        actual = required_text(plugin, field)
        if actual != expected:
            raise SdfContractError(
                f"{field} must be {expected!r}; found {actual!r}"
            )

    for field, expected in EXPECTED_PLUGIN_NUMBERS.items():
        actual = required_number(plugin, field)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            raise SdfContractError(
                f"{field} must be {expected:g}; found {actual:g}"
            )

    caster_collisions = [
        collision
        for collision in model.findall(".//collision")
        if "caster_link" in (collision.get("name") or "")
    ]
    if len(caster_collisions) != 1:
        raise SdfContractError(
            "expected exactly one caster collision with caster_link in its name"
        )
    caster_collision = caster_collisions[0]
    for field in ("mu", "mu2"):
        value = caster_collision.findtext(f"./surface/friction/ode/{field}")
        try:
            actual = float(value) if value is not None else math.nan
        except ValueError:
            actual = math.nan
        if not math.isclose(actual, 0.001, rel_tol=0.0, abs_tol=1e-12):
            raise SdfContractError(
                f"caster collision {field} must be 0.001"
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf", type=Path, help="generated SDF file")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        validate_sdf(arguments.sdf)
    except SdfContractError as error:
        print(f"SDF contract failed: {error}", file=sys.stderr)
        return 1
    print("SDF contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
