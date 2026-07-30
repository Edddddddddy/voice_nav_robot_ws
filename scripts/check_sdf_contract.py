#!/usr/bin/env python3
"""Validate the generated product SDF structurally."""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class SdfContractError(ValueError):
    """The generated SDF does not match the current teaching checkpoint."""


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

    native_plugins = [
        plugin
        for plugin in model.findall("./plugin")
        if plugin.get("filename") == "gz-sim-diff-drive-system"
        or plugin.get("name") == "gz::sim::systems::DiffDrive"
    ]
    if native_plugins:
        raise SdfContractError(
            "native Gazebo DiffDrive plugin must not remain in product SDF"
        )

    control_plugins = [
        plugin
        for plugin in model.findall("./plugin")
        if plugin.get("name")
        == "gz_ros2_control::GazeboSimROS2ControlPlugin"
    ]
    if len(control_plugins) != 1:
        raise SdfContractError(
            "expected exactly one GazeboSimROS2ControlPlugin in product SDF"
        )
    control_plugin = control_plugins[0]
    if control_plugin.get("filename") != "libgz_ros2_control-system.so":
        raise SdfContractError(
            "GazeboSimROS2ControlPlugin filename must be "
            "libgz_ros2_control-system.so"
        )
    parameters = control_plugin.findtext("parameters")
    if parameters is None or not parameters.strip():
        raise SdfContractError(
            "GazeboSimROS2ControlPlugin must receive a controller YAML path"
        )
    hold_joints = control_plugin.findtext("hold_joints")
    if hold_joints is None or hold_joints.strip().lower() != "true":
        raise SdfContractError(
            "GazeboSimROS2ControlPlugin must hold unclaimed joints"
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
