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


def require_single(
    parent: element_tree.Element,
    path: str,
    context: str,
) -> element_tree.Element:
    """Return the only matching element or reject ambiguous structure."""
    matches = parent.findall(path)
    if len(matches) != 1:
        raise SdfContractError(
            f"{context} must contain exactly one {path}"
        )
    return matches[0]


def require_text(
    parent: element_tree.Element,
    path: str,
    context: str,
) -> str:
    """Return non-empty text from one structurally unique element."""
    element = require_single(parent, path, context)
    value = (element.text or "").strip()
    if not value:
        raise SdfContractError(
            f"{context} {path} must contain text"
        )
    return value


def require_number(
    parent: element_tree.Element,
    path: str,
    expected: float,
    context: str,
) -> None:
    """Require one finite numeric element equal to the contract value."""
    raw_value = require_text(parent, path, context)
    try:
        actual = float(raw_value)
    except ValueError as error:
        raise SdfContractError(
            f"{context} {path} must be a finite number"
        ) from error
    if not math.isfinite(actual):
        raise SdfContractError(
            f"{context} {path} must be a finite number"
        )
    if not math.isclose(
        actual,
        expected,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise SdfContractError(
            f"{context} {path} must be {expected}; found {actual}"
        )


def validate_lidar(model: element_tree.Element) -> None:
    """Validate the actual post-Xacro, post-URDF-conversion sensor graph."""
    sensors = model.findall(".//sensor")
    if len(sensors) != 1:
        raise SdfContractError(
            "generated product SDF must contain exactly one sensor"
        )
    sensor = sensors[0]
    sensor_bindings = [
        (link, candidate)
        for link in model.findall(".//link")
        for candidate in link.findall("./sensor")
    ]
    if (
        len(sensor_bindings) != 1
        or sensor_bindings[0][1] is not sensor
    ):
        raise SdfContractError(
            "generated product sensor must be a direct child of one link"
        )

    if sensor.get("type") != "gpu_lidar":
        raise SdfContractError(
            "generated product LiDAR type must be gpu_lidar"
        )
    if require_text(sensor, "./topic", "generated product LiDAR") != "/scan":
        raise SdfContractError(
            "generated product LiDAR topic must be /scan"
        )
    if (
        require_text(
            sensor,
            "./gz_frame_id",
            "generated product LiDAR",
        )
        != "laser_link"
    ):
        raise SdfContractError(
            "generated product LiDAR gz_frame_id must be laser_link"
        )
    require_number(
        sensor,
        "./update_rate",
        10.0,
        "generated product LiDAR update_rate",
    )

    lidar = require_single(
        sensor,
        "./lidar",
        "generated product LiDAR",
    )
    scan = require_single(
        lidar,
        "./scan",
        "generated product LiDAR",
    )
    horizontal = require_single(
        scan,
        "./horizontal",
        "generated product LiDAR",
    )
    for field, expected in (
        ("samples", 360.0),
        ("resolution", 1.0),
        ("min_angle", -math.pi),
        ("max_angle", math.pi),
    ):
        require_number(
            horizontal,
            f"./{field}",
            expected,
            f"generated product LiDAR horizontal {field}",
        )

    vertical = require_single(
        scan,
        "./vertical",
        "generated product LiDAR",
    )
    for field, expected in (
        ("samples", 1.0),
        ("resolution", 1.0),
        ("min_angle", 0.0),
        ("max_angle", 0.0),
    ):
        require_number(
            vertical,
            f"./{field}",
            expected,
            f"generated product LiDAR vertical {field}",
        )

    lidar_range = require_single(
        lidar,
        "./range",
        "generated product LiDAR",
    )
    for field, expected in (
        ("min", 0.05),
        ("max", 8.0),
        ("resolution", 0.01),
    ):
        require_number(
            lidar_range,
            f"./{field}",
            expected,
            f"generated product LiDAR range {field}",
        )
    if sensor.findall(".//noise"):
        raise SdfContractError(
            "generated product LiDAR must not contain noise"
        )


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

    validate_lidar(model)

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
