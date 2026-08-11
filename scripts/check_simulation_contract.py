#!/usr/bin/env python3
"""Validate the simulation world, LiDAR, bridge, and TF-owner contract."""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
import xml.etree.ElementTree as element_tree
from dataclasses import dataclass
from pathlib import Path


class SimulationContractError(ValueError):
    """A simulation artifact violates the stable simulation contract."""


@dataclass(frozen=True)
class PackageShare:
    """A statically resolved FindPackageShare substitution."""

    package: str


@dataclass(frozen=True)
class PackagePath:
    """A statically resolved path beneath a ROS package share."""

    package: str
    parts: tuple[str, ...]


@dataclass(frozen=True)
class Executable:
    """A statically resolved FindExecutable substitution."""

    name: str


class Unknown:
    """Marker for launch expressions outside the supported static subset."""


UNKNOWN = Unknown()

EXPECTED_WORLD_PATH = PackagePath(
    "voice_nav_sim",
    ("worlds", "voice_nav_test_world.sdf"),
)
EXPECTED_BRIDGE_PATH = PackagePath(
    "voice_nav_sim",
    ("config", "bridge.yaml"),
)
EXPECTED_WORLD_SYSTEMS = {
    "gz::sim::systems::Physics": "gz-sim-physics-system",
    "gz::sim::systems::UserCommands": "gz-sim-user-commands-system",
    "gz::sim::systems::SceneBroadcaster":
        "gz-sim-scene-broadcaster-system",
    "gz::sim::systems::Sensors": "gz-sim-sensors-system",
}
EXPECTED_RUNTIME_DEPENDENCIES = {
    "controller_manager",
    "gz_tools_vendor",
    "launch",
    "launch_ros",
    "robot_state_publisher",
    "ros_gz_bridge",
    "ros_gz_sim",
    "rosgraph_msgs",
    "ruby",
    "sensor_msgs",
    "xacro",
}
EXPECTED_BRIDGES = {
    "/clock": {
        "ros_topic_name": "/clock",
        "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock",
        "gz_type_name": "gz.msgs.Clock",
        "direction": "GZ_TO_ROS",
        "qos_profile": "CLOCK",
    },
    "/scan": {
        "ros_topic_name": "/scan",
        "gz_topic_name": "/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": "1",
        "publisher_queue": "1",
        "qos_profile": "SENSOR_DATA",
    },
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise SimulationContractError(
            f"cannot read {path}: {error}"
        ) from error


def parse_xml(path: Path) -> element_tree.Element:
    try:
        return element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise SimulationContractError(
            f"cannot parse {path}: {error}"
        ) from error


def local_name(tag: str) -> str:
    return tag.rsplit("}", maxsplit=1)[-1]


def direct_children(
    parent: element_tree.Element,
    name: str,
) -> list[element_tree.Element]:
    return [
        child for child in parent
        if local_name(child.tag) == name
    ]


def descendants(
    parent: element_tree.Element,
    name: str,
) -> list[element_tree.Element]:
    return [
        element for element in parent.iter()
        if local_name(element.tag) == name
    ]


def required_child(
    parent: element_tree.Element,
    name: str,
    context: str,
) -> element_tree.Element:
    children = direct_children(parent, name)
    if len(children) != 1:
        raise SimulationContractError(
            f"{context} must contain exactly one {name}"
        )
    return children[0]


def required_text(
    parent: element_tree.Element,
    name: str,
    context: str,
) -> str:
    child = required_child(parent, name, context)
    value = (child.text or "").strip()
    if not value:
        raise SimulationContractError(
            f"{context} {name} must be non-empty"
        )
    return value


def parse_numbers(
    text: str,
    count: int,
    context: str,
) -> tuple[float, ...]:
    fields = text.split()
    if len(fields) != count:
        raise SimulationContractError(
            f"{context} must contain {count} finite numbers"
        )
    try:
        values = tuple(float(field) for field in fields)
    except ValueError as error:
        raise SimulationContractError(
            f"{context} must contain {count} finite numbers"
        ) from error
    if not all(math.isfinite(value) for value in values):
        raise SimulationContractError(
            f"{context} must contain {count} finite numbers"
        )
    return values


def numbers_equal(
    actual: tuple[float, ...],
    expected: tuple[float, ...],
    *,
    tolerance: float = 1e-9,
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(
            actual_value,
            expected_value,
            rel_tol=0.0,
            abs_tol=tolerance,
        )
        for actual_value, expected_value in zip(actual, expected)
    )


def validate_world(path: Path) -> None:
    root = parse_xml(path)
    if local_name(root.tag) != "sdf":
        raise SimulationContractError(
            "simulation world root must be sdf"
        )
    worlds = direct_children(root, "world")
    if len(worlds) != 1:
        raise SimulationContractError(
            "simulation world file must contain exactly one world"
        )
    world = worlds[0]
    if world.get("name") != "voice_nav_test_world":
        raise SimulationContractError(
            "simulation world name must be voice_nav_test_world"
        )

    physics_profiles = direct_children(world, "physics")
    if len(physics_profiles) != 1:
        raise SimulationContractError(
            "simulation world must define exactly one physics profile"
        )
    physics = physics_profiles[0]
    if physics.get("name") != "default_physics":
        raise SimulationContractError(
            "simulation physics name must be default_physics"
        )
    if physics.get("type") != "ode":
        raise SimulationContractError(
            "simulation physics type must be ode"
        )
    max_step_size = required_text(
        physics,
        "max_step_size",
        "simulation physics",
    )
    if max_step_size != "0.001":
        raise SimulationContractError(
            "simulation physics max_step_size must be 0.001"
        )
    real_time_update_rate = required_text(
        physics,
        "real_time_update_rate",
        "simulation physics",
    )
    if real_time_update_rate != "500":
        raise SimulationContractError(
            "simulation physics real_time_update_rate must be 500"
        )

    uri_nodes = list(descendants(world, "uri"))
    if uri_nodes:
        uri_node = uri_nodes[0]
        uri = (uri_node.text or "").strip()
        raise SimulationContractError(
            "simulation world must not reference external URI; "
            "it must not reference resource URI at all: "
            f"{uri or '<empty>'}"
        )

    plugins = direct_children(world, "plugin")
    plugins_by_name: dict[str, list[element_tree.Element]] = {}
    for plugin in plugins:
        plugins_by_name.setdefault(plugin.get("name", ""), []).append(plugin)
    for plugin_name, expected_filename in EXPECTED_WORLD_SYSTEMS.items():
        matches = plugins_by_name.get(plugin_name, [])
        if len(matches) != 1:
            raise SimulationContractError(
                "simulation world must load exactly one "
                f"{plugin_name} system"
            )
        filename = matches[0].get("filename")
        if filename != expected_filename:
            raise SimulationContractError(
                f"{plugin_name} filename must be {expected_filename}; "
                f"found {filename or '<missing>'}"
            )

    sensors_plugin = plugins_by_name[
        "gz::sim::systems::Sensors"
    ][0]
    render_engine = required_text(
        sensors_plugin,
        "render_engine",
        "Gazebo Sensors system",
    )
    if render_engine != "ogre2":
        raise SimulationContractError(
            "Gazebo Sensors system render_engine must be ogre2"
        )

    ground_collisions = []
    for model in direct_children(world, "model"):
        static = required_text(
            model,
            "static",
            f"world model {model.get('name', '<unnamed>')}",
        )
        if static.lower() != "true":
            continue
        for collision in descendants(model, "collision"):
            geometries = direct_children(collision, "geometry")
            if len(geometries) != 1:
                continue
            planes = direct_children(geometries[0], "plane")
            if len(planes) != 1:
                continue
            normal = parse_numbers(
                required_text(
                    planes[0],
                    "normal",
                    "ground collision plane",
                ),
                3,
                "ground collision normal",
            )
            size = parse_numbers(
                required_text(
                    planes[0],
                    "size",
                    "ground collision plane",
                ),
                2,
                "ground collision size",
            )
            if (
                numbers_equal(normal, (0.0, 0.0, 1.0))
                and all(value > 0.0 for value in size)
            ):
                ground_collisions.append(collision)
    if len(ground_collisions) != 1:
        raise SimulationContractError(
            "simulation world must contain an inline static ground plane "
            "collision"
        )

    obstacle_matches = []
    for model in direct_children(world, "model"):
        static = [
            value.lower()
            for node in direct_children(model, "static")
            if (value := (node.text or "").strip())
        ]
        if static != ["true"]:
            continue
        pose_nodes = direct_children(model, "pose")
        if len(pose_nodes) != 1:
            continue
        pose = parse_numbers(
            (pose_nodes[0].text or "").strip(),
            6,
            f"world model {model.get('name', '<unnamed>')} pose",
        )
        if not numbers_equal(pose, (2.0, 0.0, 0.5, 0.0, 0.0, 0.0)):
            continue
        collision_sizes = []
        for collision in descendants(model, "collision"):
            geometry = direct_children(collision, "geometry")
            if len(geometry) != 1:
                continue
            boxes = direct_children(geometry[0], "box")
            if len(boxes) != 1:
                continue
            size_text = required_text(
                boxes[0],
                "size",
                "test obstacle collision box",
            )
            collision_sizes.append(
                parse_numbers(
                    size_text,
                    3,
                    "test obstacle collision box size",
                )
            )
        if any(
            numbers_equal(size, (0.5, 1.0, 1.0))
            for size in collision_sizes
        ):
            obstacle_matches.append(model)
    if len(obstacle_matches) != 1:
        raise SimulationContractError(
            "simulation world must contain exactly one static collision box "
            "centered at (2.0, 0, 0.5) with size (0.5, 1.0, 1.0)"
        )


def numeric_child(
    parent: element_tree.Element,
    name: str,
    context: str,
) -> float:
    raw_value = required_text(parent, name, context)
    try:
        value = float(raw_value)
    except ValueError as error:
        raise SimulationContractError(
            f"{context} {name} must be a finite number"
        ) from error
    if not math.isfinite(value):
        raise SimulationContractError(
            f"{context} {name} must be a finite number"
        )
    return value


def require_number(
    parent: element_tree.Element,
    name: str,
    expected: float,
    context: str,
) -> None:
    value = numeric_child(parent, name, context)
    if not math.isclose(
        value,
        expected,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise SimulationContractError(
            f"{context} {name} must be {expected}; found {value}"
        )


def validate_robot_description(path: Path) -> None:
    root = parse_xml(path)
    laser_links = [
        link for link in direct_children(root, "link")
        if link.get("name") == "laser_link"
    ]
    if len(laser_links) != 1:
        raise SimulationContractError(
            "robot root must directly define exactly one laser_link"
        )

    gazebo_blocks = direct_children(root, "gazebo")
    sensor_bindings = [
        (gazebo, sensor)
        for gazebo in gazebo_blocks
        for sensor in direct_children(gazebo, "sensor")
    ]
    if len(sensor_bindings) != 1:
        raise SimulationContractError(
            "robot root must directly define exactly one Gazebo sensor "
            "for the simulation contract"
        )

    laser_gazebo_blocks = [
        gazebo for gazebo in gazebo_blocks
        if gazebo.get("reference") == "laser_link"
    ]
    if len(laser_gazebo_blocks) != 1:
        raise SimulationContractError(
            "robot root must directly attach Gazebo configuration; "
            "laser_link must own exactly one Gazebo LiDAR sensor"
        )
    sensor_owner, sensor = sensor_bindings[0]
    if sensor_owner is not laser_gazebo_blocks[0]:
        raise SimulationContractError(
            "laser_link must own exactly one Gazebo LiDAR sensor"
        )
    if sensor.get("type") != "gpu_lidar":
        raise SimulationContractError(
            "laser_link sensor type must be gpu_lidar"
        )
    topic = required_text(sensor, "topic", "laser_link LiDAR")
    if topic != "/scan":
        raise SimulationContractError(
            "laser_link LiDAR topic must be /scan"
        )
    frame_id = required_text(
        sensor,
        "gz_frame_id",
        "laser_link LiDAR",
    )
    if frame_id != "laser_link":
        raise SimulationContractError(
            "laser_link LiDAR gz_frame_id must be laser_link"
        )
    require_number(sensor, "update_rate", 10.0, "laser_link LiDAR")
    sensor_poses = direct_children(sensor, "pose")
    if len(sensor_poses) > 1:
        raise SimulationContractError(
            "laser_link LiDAR must contain at most one pose"
        )
    if sensor_poses:
        sensor_pose = parse_numbers(
            (sensor_poses[0].text or "").strip(),
            6,
            "laser_link LiDAR pose",
        )
        if not numbers_equal(
            sensor_pose,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ):
            raise SimulationContractError(
                "laser_link LiDAR pose must be link-local zero"
            )
    if descendants(sensor, "noise"):
        raise SimulationContractError(
            "laser_link LiDAR must not configure synthetic noise"
        )

    lidar = required_child(sensor, "lidar", "laser_link LiDAR")
    scan = required_child(lidar, "scan", "laser_link LiDAR geometry")
    horizontal = required_child(
        scan,
        "horizontal",
        "laser_link LiDAR scan",
    )
    require_number(horizontal, "samples", 360.0, "horizontal scan")
    require_number(horizontal, "resolution", 1.0, "horizontal scan")
    require_number(horizontal, "min_angle", -math.pi, "horizontal scan")
    require_number(horizontal, "max_angle", math.pi, "horizontal scan")

    vertical = required_child(
        scan,
        "vertical",
        "laser_link LiDAR scan",
    )
    require_number(vertical, "samples", 1.0, "vertical scan")
    require_number(vertical, "resolution", 1.0, "vertical scan")
    require_number(vertical, "min_angle", 0.0, "vertical scan")
    require_number(vertical, "max_angle", 0.0, "vertical scan")

    lidar_range = required_child(
        lidar,
        "range",
        "laser_link LiDAR geometry",
    )
    require_number(lidar_range, "min", 0.05, "LiDAR range")
    require_number(lidar_range, "max", 8.0, "LiDAR range")
    require_number(lidar_range, "resolution", 0.01, "LiDAR range")


def strip_yaml_comment(line: str) -> str:
    quote: str | None = None
    escaped = False
    for index, character in enumerate(line):
        if escaped:
            escaped = False
            continue
        if character == "\\" and quote == '"':
            escaped = True
            continue
        if character in {"'", '"'}:
            if quote is None:
                quote = character
            elif quote == character:
                quote = None
            continue
        if character == "#" and quote is None:
            return line[:index]
    return line


def parse_yaml_scalar(value: str, line_number: int) -> str:
    scalar = value.strip()
    if not scalar:
        raise SimulationContractError(
            f"bridge YAML scalar is empty at line {line_number}"
        )
    if scalar[0] in {"'", '"'}:
        try:
            decoded = ast.literal_eval(scalar)
        except (SyntaxError, ValueError) as error:
            raise SimulationContractError(
                f"invalid quoted bridge YAML scalar at line {line_number}"
            ) from error
        if not isinstance(decoded, str):
            raise SimulationContractError(
                f"bridge YAML values must be strings at line {line_number}"
            )
        return decoded
    if any(character in scalar for character in "{}[]&*"):
        raise SimulationContractError(
            "bridge YAML must be a plain sequence of scalar mappings; "
            f"unsupported syntax at line {line_number}"
        )
    return scalar


def parse_yaml_mapping_entry(
    text: str,
    line_number: int,
) -> tuple[str, str]:
    match = re.fullmatch(
        r"([A-Za-z_][A-Za-z0-9_]*):(?:[ \t]+)(.+)",
        text,
    )
    if match is None:
        raise SimulationContractError(
            "bridge YAML must be a sequence of scalar mappings; "
            f"invalid entry at line {line_number}"
        )
    return match.group(1), parse_yaml_scalar(
        match.group(2),
        line_number,
    )


def parse_bridge_yaml(path: Path) -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    current: dict[str, str] | None = None
    for line_number, raw_line in enumerate(
        read_text(path).splitlines(),
        start=1,
    ):
        if "\t" in raw_line[: len(raw_line) - len(raw_line.lstrip())]:
            raise SimulationContractError(
                f"bridge YAML indentation must not use tabs at line "
                f"{line_number}"
            )
        line = strip_yaml_comment(raw_line).rstrip()
        if not line.strip() or line.strip() in {"---", "..."}:
            continue
        if line.startswith("- "):
            current = {}
            records.append(current)
            entry = line[2:]
        elif line.startswith("  ") and not line.startswith("   "):
            if current is None:
                raise SimulationContractError(
                    f"bridge YAML mapping has no sequence item at line "
                    f"{line_number}"
                )
            entry = line[2:]
        else:
            raise SimulationContractError(
                "bridge YAML must be a top-level sequence with two-space "
                f"mapping indentation at line {line_number}"
            )
        key, value = parse_yaml_mapping_entry(entry, line_number)
        if key in current:
            raise SimulationContractError(
                f"duplicate bridge YAML key {key} at line {line_number}"
            )
        current[key] = value
    if not records:
        raise SimulationContractError(
            "bridge YAML must contain /clock and /scan records"
        )
    return records


def validate_bridge(path: Path) -> None:
    records = parse_bridge_yaml(path)
    topics = [record.get("ros_topic_name", "") for record in records]
    if len(topics) != len(set(topics)):
        raise SimulationContractError(
            "bridge ros_topic_name values must be unique"
        )
    if set(topics) != set(EXPECTED_BRIDGES) or len(records) != 2:
        unexpected = sorted(
            topic or "<missing>" for topic in topics
            if topic not in EXPECTED_BRIDGES
        )
        detail = (
            f"; forbidden topic(s): {', '.join(unexpected)}"
            if unexpected
            else ""
        )
        raise SimulationContractError(
            "bridge allowlist must contain only /clock and /scan"
            + detail
        )
    records_by_topic = {
        record["ros_topic_name"]: record for record in records
    }
    for topic, expected in EXPECTED_BRIDGES.items():
        actual = records_by_topic[topic]
        missing = sorted(set(expected) - set(actual))
        extra = sorted(set(actual) - set(expected))
        if missing or extra:
            differences = []
            if missing:
                differences.append("missing " + ", ".join(missing))
            if extra:
                differences.append("unexpected " + ", ".join(extra))
            raise SimulationContractError(
                f"{topic} bridge fields are invalid: "
                + "; ".join(differences)
            )
        for field, expected_value in expected.items():
            actual_value = actual[field]
            if actual_value != expected_value:
                raise SimulationContractError(
                    f"{topic} bridge {field} must be {expected_value}; "
                    f"found {actual_value}"
                )


def validate_package(path: Path) -> None:
    root = parse_xml(path)
    if local_name(root.tag) != "package":
        raise SimulationContractError(
            "voice_nav_sim package manifest root must be package"
        )
    package_names = direct_children(root, "name")
    if (
        len(package_names) != 1
        or (package_names[0].text or "").strip() != "voice_nav_sim"
    ):
        raise SimulationContractError(
            "simulation package name must be voice_nav_sim"
        )
    runtime_dependencies = {
        (element.text or "").strip()
        for element in root
        if local_name(element.tag) in {"depend", "exec_depend"}
    }
    missing = sorted(
        EXPECTED_RUNTIME_DEPENDENCIES - runtime_dependencies
    )
    if missing:
        raise SimulationContractError(
            "voice_nav_sim is missing direct runtime dependencies: "
            + ", ".join(missing)
        )


def validate_cmake(path: Path) -> None:
    cmake = "\n".join(
        line.split("#", maxsplit=1)[0]
        for line in read_text(path).splitlines()
    )
    installed_directories: set[str] = set()
    for match in re.finditer(
        r"\binstall\s*\((?P<body>.*?)\)",
        cmake,
        flags=re.DOTALL,
    ):
        body = match.group("body")
        directory_match = re.match(
            r"\s*DIRECTORY\b(?P<directories>.*?)"
            r"\bDESTINATION\s+share/\$\{PROJECT_NAME\}",
            body,
            flags=re.DOTALL,
        )
        if directory_match is None:
            continue
        installed_directories.update(
            re.findall(
                r"(?<![\w./-])(config|launch|urdf|worlds)"
                r"(?![\w./-])",
                directory_match.group("directories"),
            )
        )
    required_directories = {"config", "launch", "urdf", "worlds"}
    missing = sorted(required_directories - installed_directories)
    if missing:
        raise SimulationContractError(
            "CMake install is missing directories: "
            + ", ".join(missing)
        )


def call_name(call: ast.Call) -> str:
    function = call.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ""


def keyword_value(call: ast.Call, name: str) -> ast.expr | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def launch_function_scope(
    tree: ast.Module,
) -> tuple[dict[str, ast.expr], ast.expr]:
    functions = [
        statement
        for statement in tree.body
        if isinstance(statement, ast.FunctionDef)
        and statement.name == "generate_launch_description"
    ]
    if len(functions) != 1:
        raise SimulationContractError(
            "launch source must define exactly one "
            "generate_launch_description function"
        )
    function = functions[0]
    assignments: dict[str, ast.expr] = {}
    return_values = []
    for statement in function.body:
        if isinstance(statement, ast.Assign):
            for target in statement.targets:
                if isinstance(target, ast.Name):
                    assignments[target.id] = statement.value
        elif (
            isinstance(statement, ast.AnnAssign)
            and isinstance(statement.target, ast.Name)
            and statement.value is not None
        ):
            assignments[statement.target.id] = statement.value
        elif isinstance(statement, ast.Return):
            if statement.value is None:
                raise SimulationContractError(
                    "generate_launch_description must return "
                    "LaunchDescription"
                )
            return_values.append(statement.value)
    if len(return_values) != 1:
        raise SimulationContractError(
            "generate_launch_description must contain exactly one direct "
            "return statement"
        )
    return assignments, return_values[0]


def collect_reachable_calls(
    root: ast.expr,
    assignments: dict[str, ast.expr],
) -> list[ast.Call]:
    calls: list[ast.Call] = []
    visited_nodes: set[int] = set()
    resolving_names: set[str] = set()

    def visit(node: ast.AST) -> None:
        if isinstance(node, ast.Name):
            if (
                node.id in resolving_names
                or node.id not in assignments
            ):
                return
            resolving_names.add(node.id)
            visit(assignments[node.id])
            resolving_names.remove(node.id)
            return
        node_identity = id(node)
        if node_identity in visited_nodes:
            return
        visited_nodes.add(node_identity)
        if isinstance(node, ast.Call):
            calls.append(node)
            for argument in node.args:
                visit(argument)
            for keyword in node.keywords:
                visit(keyword.value)
            return
        for _field_name, value in ast.iter_fields(node):
            if isinstance(value, ast.AST):
                visit(value)
            elif isinstance(value, list):
                for item in value:
                    if isinstance(item, ast.AST):
                        visit(item)

    visit(root)
    return calls


def static_slice(node: ast.slice) -> slice | int | Unknown:
    if isinstance(node, ast.Constant) and isinstance(node.value, int):
        return node.value
    if isinstance(node, ast.Slice):
        values: list[int | None] = []
        for part in (node.lower, node.upper, node.step):
            if part is None:
                values.append(None)
            elif (
                isinstance(part, ast.Constant)
                and isinstance(part.value, int)
            ):
                values.append(part.value)
            else:
                return UNKNOWN
        return slice(*values)
    return UNKNOWN


def evaluate_static(
    node: ast.expr | None,
    assignments: dict[str, ast.expr],
    resolving: frozenset[str] = frozenset(),
) -> object:
    if node is None:
        return UNKNOWN
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        if node.id in resolving or node.id not in assignments:
            return UNKNOWN
        return evaluate_static(
            assignments[node.id],
            assignments,
            resolving | {node.id},
        )
    if isinstance(node, (ast.List, ast.Tuple)):
        return [
            evaluate_static(item, assignments, resolving)
            for item in node.elts
        ]
    if isinstance(node, ast.Dict):
        result: dict[object, object] = {}
        for key_node, value_node in zip(node.keys, node.values):
            key = evaluate_static(key_node, assignments, resolving)
            value = evaluate_static(value_node, assignments, resolving)
            if isinstance(key, Unknown):
                return UNKNOWN
            result[key] = value
        return result
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = evaluate_static(node.left, assignments, resolving)
        right = evaluate_static(node.right, assignments, resolving)
        if isinstance(left, list) and isinstance(right, list):
            return left + right
        return UNKNOWN
    if isinstance(node, ast.Subscript):
        value = evaluate_static(node.value, assignments, resolving)
        index = static_slice(node.slice)
        if isinstance(value, list) and not isinstance(index, Unknown):
            try:
                return value[index]
            except IndexError:
                return UNKNOWN
        return UNKNOWN
    if isinstance(node, ast.Call):
        name = call_name(node)
        if name == "FindPackageShare" and len(node.args) == 1:
            package = evaluate_static(
                node.args[0],
                assignments,
                resolving,
            )
            if isinstance(package, str):
                return PackageShare(package)
        if name == "FindExecutable":
            executable_node = (
                keyword_value(node, "name")
                if keyword_value(node, "name") is not None
                else (node.args[0] if node.args else None)
            )
            executable = evaluate_static(
                executable_node,
                assignments,
                resolving,
            )
            if isinstance(executable, str):
                return Executable(executable)
        if name == "PathJoinSubstitution" and len(node.args) == 1:
            parts = evaluate_static(
                node.args[0],
                assignments,
                resolving,
            )
            if (
                isinstance(parts, list)
                and parts
                and isinstance(parts[0], PackageShare)
                and all(isinstance(part, str) for part in parts[1:])
            ):
                return PackagePath(
                    parts[0].package,
                    tuple(parts[1:]),
                )
        return UNKNOWN
    return UNKNOWN


def node_string(
    call: ast.Call,
    name: str,
    assignments: dict[str, ast.expr],
) -> str | None:
    value = evaluate_static(keyword_value(call, name), assignments)
    return value if isinstance(value, str) else None


def static_list(
    call: ast.Call,
    name: str,
    assignments: dict[str, ast.expr],
) -> list[object] | None:
    value = evaluate_static(keyword_value(call, name), assignments)
    return value if isinstance(value, list) else None


def require_world_launch(
    calls: list[ast.Call],
    assignments: dict[str, ast.expr],
) -> None:
    gazebo_commands: list[list[object]] = []
    for call in calls:
        if call_name(call) != "ExecuteProcess":
            continue
        command = static_list(call, "cmd", assignments)
        if command is None:
            continue
        if Executable("gz") in command and "sim" in command:
            gazebo_commands.append(command)
    if not gazebo_commands:
        raise SimulationContractError(
            "simulation launch must own Gazebo through a static "
            "ExecuteProcess command"
        )
    for command in gazebo_commands:
        if "empty.sdf" in command:
            raise SimulationContractError(
                "simulation launch must load the packaged non-empty test "
                "world; found built-in empty.sdf"
            )
        if EXPECTED_WORLD_PATH not in command:
            raise SimulationContractError(
                "simulation launch must load "
                "voice_nav_sim/worlds/voice_nav_test_world.sdf through "
                "FindPackageShare"
            )
    headless_commands = [
        command for command in gazebo_commands if "-s" in command
    ]
    if not headless_commands:
        raise SimulationContractError(
            "simulation launch must define a headless Gazebo server command"
        )
    if any(
        "--headless-rendering" not in command
        for command in headless_commands
    ):
        raise SimulationContractError(
            "headless Gazebo server command must include "
            "--headless-rendering"
        )


def require_bridge_launch(
    nodes: list[ast.Call],
    assignments: dict[str, ast.expr],
) -> None:
    bridges = [
        node for node in nodes
        if node_string(node, "package", assignments) == "ros_gz_bridge"
        and node_string(node, "executable", assignments)
        == "parameter_bridge"
    ]
    if len(bridges) != 1:
        raise SimulationContractError(
            "simulation launch must start exactly one ros_gz_bridge "
            "parameter_bridge"
        )
    bridge = bridges[0]
    arguments = static_list(bridge, "arguments", assignments)
    if arguments:
        raise SimulationContractError(
            "parameter_bridge topics must come only from config/bridge.yaml"
        )
    parameters = static_list(bridge, "parameters", assignments)
    if parameters is None:
        raise SimulationContractError(
            "parameter_bridge must load config/bridge.yaml through its "
            "config_file parameter"
        )
    expected_parameters = [
        {"config_file": EXPECTED_BRIDGE_PATH}
    ]
    if (
        len(parameters) == 1
        and isinstance(parameters[0], dict)
        and set(parameters[0]) == {"config_file"}
        and parameters[0]["config_file"] != EXPECTED_BRIDGE_PATH
    ):
        raise SimulationContractError(
            "parameter_bridge config_file must resolve to "
            "voice_nav_sim/config/bridge.yaml through FindPackageShare"
        )
    if parameters != expected_parameters:
        raise SimulationContractError(
            "parameter_bridge parameters must be exactly one config_file "
            "mapping with no overrides"
        )


def require_spawn_world(
    nodes: list[ast.Call],
    assignments: dict[str, ast.expr],
) -> None:
    spawners = [
        node for node in nodes
        if node_string(node, "package", assignments) == "ros_gz_sim"
        and node_string(node, "executable", assignments) == "create"
    ]
    if len(spawners) != 1:
        raise SimulationContractError(
            "simulation launch must start exactly one ros_gz_sim create node"
        )
    arguments = static_list(spawners[0], "arguments", assignments)
    if arguments is None:
        raise SimulationContractError(
            "robot spawn arguments must be a static list"
        )
    world_values = [
        arguments[index + 1]
        for index, argument in enumerate(arguments[:-1])
        if argument == "--world"
    ]
    if world_values != ["voice_nav_test_world"]:
        raise SimulationContractError(
            "robot spawn --world must be voice_nav_test_world"
        )


def require_direct_odom_remap(
    nodes: list[ast.Call],
    assignments: dict[str, ast.expr],
) -> None:
    drive_spawners = []
    for node in nodes:
        if (
            node_string(node, "package", assignments)
            != "controller_manager"
            or node_string(node, "executable", assignments) != "spawner"
        ):
            continue
        arguments = static_list(node, "arguments", assignments)
        if arguments is not None and "diff_drive_controller" in arguments:
            drive_spawners.append((node, arguments))
    if len(drive_spawners) != 1:
        raise SimulationContractError(
            "simulation launch must start exactly one "
            "diff_drive_controller spawner"
        )
    _node, arguments = drive_spawners[0]
    controller_ros_arguments = [
        arguments[index + 1]
        for index, argument in enumerate(arguments[:-1])
        if argument == "--controller-ros-args"
    ]
    required = ["--ros-args --remap ~/odom:=/odom"]
    if controller_ros_arguments != required:
        raise SimulationContractError(
            "diff_drive_controller must directly remap ~/odom to /odom "
            "through one --controller-ros-args value"
        )


def reject_relays_and_extra_tf_publishers(
    nodes: list[ast.Call],
    assignments: dict[str, ast.expr],
) -> None:
    relays = []
    tf_publishers = []
    robot_state_publishers = []
    for node in nodes:
        package = node_string(node, "package", assignments)
        executable = node_string(node, "executable", assignments)
        if package == "topic_tools" and executable == "relay":
            relays.append((package, executable))
        if package == "robot_state_publisher":
            robot_state_publishers.append(node)
        if (
            package == "tf2_ros"
            and executable in {
                "static_transform_publisher",
                "static_transform_broadcaster",
                "transform_publisher",
            }
        ):
            tf_publishers.append((package, executable))
    if relays:
        raise SimulationContractError(
            "simulation launch must not add an odometry topic relay"
        )
    if len(robot_state_publishers) != 1:
        raise SimulationContractError(
            "simulation launch must contain exactly one "
            "robot_state_publisher"
        )
    if tf_publishers:
        raise SimulationContractError(
            "simulation launch must not add another TF publisher"
        )


def validate_launch(path: Path) -> None:
    source = read_text(path)
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise SimulationContractError(
            f"cannot parse launch source {path}: {error}"
        ) from error
    assignments, return_expression = launch_function_scope(tree)
    calls = collect_reachable_calls(return_expression, assignments)
    descriptions = [
        call for call in calls
        if call_name(call) == "LaunchDescription"
    ]
    if len(descriptions) != 1:
        raise SimulationContractError(
            "generate_launch_description must return exactly one reachable "
            "LaunchDescription"
        )
    launch_nodes = [
        call for call in calls if call_name(call) == "Node"
    ]

    # Keep the world check first: it is the foundational simulation contract.
    require_world_launch(calls, assignments)
    require_bridge_launch(launch_nodes, assignments)
    require_spawn_world(launch_nodes, assignments)
    require_direct_odom_remap(launch_nodes, assignments)
    reject_relays_and_extra_tf_publishers(
        launch_nodes,
        assignments,
    )


def validate_contract(
    launch: Path,
    world: Path,
    robot_description: Path,
    bridge: Path,
    package: Path,
    cmake: Path,
) -> None:
    validate_launch(launch)
    validate_world(world)
    validate_robot_description(robot_description)
    validate_bridge(bridge)
    validate_package(package)
    validate_cmake(cmake)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--robot-description", required=True, type=Path)
    parser.add_argument("--bridge", required=True, type=Path)
    parser.add_argument("--package", required=True, type=Path)
    parser.add_argument("--cmake", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        validate_contract(
            arguments.launch,
            arguments.world,
            arguments.robot_description,
            arguments.bridge,
            arguments.package,
            arguments.cmake,
        )
    except SimulationContractError as error:
        print(
            f"Simulation contract failed: {error}",
            file=sys.stderr,
        )
        return 1
    print("Simulation contract passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
