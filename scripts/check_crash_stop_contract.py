#!/usr/bin/env python3
"""Validate the VN-0011A crash-stop evidence seams without starting ROS."""

from __future__ import annotations

import argparse
import ast
import re
import runpy
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


class CrashStopContractError(ValueError):
    """A checked artifact violates the VN-0011A crash-stop contract."""


ARTIFACTS = {
    "write_journal": (
        "src/voice_nav_sim/test_support/hardware_write_ledger_writer.hpp"
    ),
    "adapter_header": (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter.hpp"
    ),
    "adapter_source": (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter.cpp"
    ),
    "adapter_plugin": (
        "src/voice_nav_sim/test_support/"
        "journaled_gazebo_sim_system_adapter_plugins.xml"
    ),
    "adapter_test": (
        "src/voice_nav_sim/test/"
        "journaled_gazebo_sim_system_adapter_test.cpp"
    ),
    "evidence_policy": (
        "src/voice_nav_sim/test_support/crash_stop_policy.py"
    ),
    "robot_transformer": (
        "src/voice_nav_sim/test_support/crash_robot_description.py"
    ),
    "product_xacro": (
        "src/voice_nav_sim/urdf/voice_nav_robot.urdf.xacro"
    ),
    "product_launch": (
        "src/voice_nav_bringup/launch/product_sim.launch.py"
    ),
    "simulation_launch": (
        "src/voice_nav_sim/launch/simulation.launch.py"
    ),
    "product_gate_yaml": (
        "src/voice_nav_bringup/config/motion_gate.yaml"
    ),
    "sim_cmake": "src/voice_nav_sim/CMakeLists.txt",
    "sim_package": "src/voice_nav_sim/package.xml",
}

PUBLIC_HARDWARE_BASE = (
    "gz_ros2_control::GazeboSimSystemInterface"
)
CONCRETE_HARDWARE_CLASS = "gz_ros2_control::GazeboSimSystem"
UPSTREAM_PLUGIN_NAME = "gz_ros2_control/GazeboSimSystem"
TEST_PLUGIN_NAME = (
    "voice_nav_sim/JournaledGazeboSimSystemAdapter"
)
TEST_PLUGIN_TYPE = (
    "voice_nav_sim::JournaledGazeboSimSystemAdapter"
)
ADAPTER_TARGET = (
    "voice_nav_sim_journaled_gazebo_sim_system_adapter"
)
ADAPTER_TEST_TARGET = (
    "journaled_gazebo_sim_system_adapter_test"
)

POLICY_VALUES: dict[str, object] = {
    "PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER": False,
    "GATE_REQUIRE_UNIQUE_FINAL_MARKER": True,
    "GATE_FINAL_MARKER_MAX_COMMITS": 1,
    "GATE_ACK_DEADLINE_OUTPUT_PERIODS": 1,
    "JOURNAL_INSTRUMENTATION_ALLOCATION_FREE": True,
    "UPSTREAM_GAZEBO_WRITE_ALLOCATION_FREE": False,
}

PRODUCT_SEAM_MARKERS = {
    "journaledgazebosimsystemadapter",
    "journaledgazebosystem",
    "journaled_gazebo",
    "hardware_journal",
    "journal_shared_memory",
    "journal_nonce",
    "shm_open",
    "test_gate_event_journal_name",
    "test_gate_event_journal_descriptor",
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise CrashStopContractError(
            f"cannot read {path}: {error}"
        ) from error


def required_artifacts(root: Path) -> dict[str, Path]:
    paths = {
        name: root / relative_path
        for name, relative_path in ARTIFACTS.items()
    }
    for name in ARTIFACTS:
        path = paths[name]
        if not path.is_file():
            relative = path.relative_to(root).as_posix()
            raise CrashStopContractError(
                "missing VN-0011A crash-stop artifact: " + relative
            )
    return paths


def validate_adapter(
    header_path: Path,
    source_path: Path,
    write_journal_path: Path,
) -> None:
    header = read_text(header_path)
    source = read_text(source_path)
    write_journal = read_text(write_journal_path)
    combined = header + "\n" + source

    direct_concrete_base = re.search(
        r":\s*public\s+gz_ros2_control::GazeboSimSystem\b(?!Interface)",
        combined,
    )
    if direct_concrete_base is not None:
        raise CrashStopContractError(
            "test hardware Adapter must not directly subclass the concrete "
            "GazeboSimSystem PImpl class"
        )

    public_base_pattern = re.compile(
        r"class\s+JournaledGazeboSimSystemAdapter\s+final\s*:\s*"
        r"public\s+"
        r"gz_ros2_control::GazeboSimSystemInterface\b"
    )
    if public_base_pattern.search(header) is None:
        raise CrashStopContractError(
            "JournaledGazeboSimSystemAdapter must inherit the public "
            "GazeboSimSystemInterface extension seam"
        )

    loader_type = re.compile(
        r"pluginlib::ClassLoader\s*<\s*"
        r"gz_ros2_control::GazeboSimSystemInterface\s*>"
    )
    if loader_type.search(header) is None:
        raise CrashStopContractError(
            "test hardware Adapter must own a pluginlib loader for the "
            "public GazeboSimSystemInterface"
        )
    loader_position = header.find("upstream_loader_")
    instance_match = re.search(r"\bupstream_\s*;", header)
    instance_position = (
        -1 if instance_match is None else instance_match.start()
    )
    if (
        loader_position < 0
        or instance_position < 0
        or loader_position >= instance_position
    ):
        raise CrashStopContractError(
            "Adapter member lifetime must destroy the upstream instance "
            "before its pluginlib loader"
        )

    create_upstream = re.compile(
        r"createSharedInstance\s*\(\s*[\"']"
        + re.escape(UPSTREAM_PLUGIN_NAME)
        + r"[\"']\s*\)"
    )
    if create_upstream.search(source) is None:
        raise CrashStopContractError(
            "test hardware Adapter must pluginlib-load the pinned upstream "
            "gz_ros2_control/GazeboSimSystem"
        )

    attachment_interface = re.search(
        r"class\s+HardwareWriteJournalAttachment\b.*?"
        r"\battach\s*\(",
        header,
        flags=re.DOTALL,
    )
    attachment_member = re.search(
        r"std::shared_ptr\s*<\s*HardwareWriteJournalAttachment\s*>\s*"
        r"write_journal_attachment_\s*;",
        header,
    )
    if attachment_interface is None or attachment_member is None:
        raise CrashStopContractError(
            "Adapter must own the runtime journal attachment seam"
        )
    required_default_attachment = (
        "PosixHardwareWriteJournalAttachment",
        "AttachedHardwareWriteLedger",
        "HardwareWriteLedgerDiscoveryConfig",
        "std::make_shared<PosixHardwareWriteJournalAttachment>()",
    )
    if any(fragment not in source for fragment in required_default_attachment):
        raise CrashStopContractError(
            "default Adapter must construct the nonce-authenticated POSIX "
            "journal attachment"
        )

    required_delegations = {
        "upstream_->initSim(": "initSim",
        "upstream_->on_init(": "on_init",
        "upstream_->on_configure(": "on_configure",
        "upstream_->on_activate(": "on_activate",
        "upstream_->on_deactivate(": "on_deactivate",
        "upstream_->export_state_interfaces(": "export_state_interfaces",
        "upstream_->export_command_interfaces(": (
            "export_command_interfaces"
        ),
        "upstream_->prepare_command_mode_switch(": (
            "prepare_command_mode_switch"
        ),
        "upstream_->perform_command_mode_switch(": (
            "perform_command_mode_switch"
        ),
        "upstream_->read(": "read",
        "upstream_->write(": "write",
    }
    missing_delegations = sorted(
        name
        for fragment, name in required_delegations.items()
        if fragment not in source
    )
    if missing_delegations:
        raise CrashStopContractError(
            "test hardware Adapter source must expose forwarding-call "
            "topology for every lifecycle, interface, mode-switch, and I/O "
            "method overridden by the pinned upstream implementation; "
            "compile-time fake-upstream tests own argument/result parity; "
            "missing: "
            + ", ".join(missing_delegations)
        )

    journal_match = re.search(
        r"class\s+HardwareWriteJournal\b.*?\{(?P<body>.*?)\};",
        write_journal,
        flags=re.DOTALL,
    )
    if journal_match is None:
        raise CrashStopContractError(
            "hardware write journal must define HardwareWriteJournal"
        )
    journal_body = journal_match.group("body")
    if any(
        re.search(rf"\b{method}\s*\(", journal_body) is None
        for method in ("begin_write", "finish_write")
    ):
        raise CrashStopContractError(
            "HardwareWriteJournal must define begin_write and finish_write"
        )

    ticket_match = re.search(
        r"struct\s+HardwareWriteTicket\s*\{(?P<body>.*?)\};",
        write_journal,
        flags=re.DOTALL,
    )
    if ticket_match is None:
        raise CrashStopContractError(
            "hardware write journal must define HardwareWriteTicket"
        )
    required_ticket_fields = {
        "write_seq",
        "sim_stamp_ns",
        "bank_index",
        "bank_epoch",
        "included",
    }
    ticket_body = ticket_match.group("body")
    missing_ticket_fields = sorted(
        field
        for field in required_ticket_fields
        if re.search(rf"\b{re.escape(field)}\b", ticket_body) is None
    )
    if missing_ticket_fields:
        raise CrashStopContractError(
            "Hardware write ticket is missing Writer-owned facts: "
            + ", ".join(missing_ticket_fields)
        )

    observation_match = re.search(
        r"struct\s+HardwareWriteWheelObservation\s*"
        r"\{(?P<body>.*?)\};",
        write_journal,
        flags=re.DOTALL,
    )
    if observation_match is None:
        raise CrashStopContractError(
            "hardware write journal must define wheel observation facts"
        )
    required_observation_fields = {
        "status",
        "left_command_bits",
        "right_command_bits",
    }
    observation_body = observation_match.group("body")
    missing_observation_fields = sorted(
        field
        for field in required_observation_fields
        if re.search(rf"\b{re.escape(field)}\b", observation_body) is None
    )
    if missing_observation_fields:
        raise CrashStopContractError(
            "HardwareWriteWheelObservation is missing facts: "
            + ", ".join(missing_observation_fields)
        )

    journal_member = re.search(
        r"std::shared_ptr\s*<\s*HardwareWriteJournal\s*>\s*"
        r"write_journal_\s*;",
        header,
    )
    if journal_member is None:
        raise CrashStopContractError(
            "Adapter must depend on the HardwareWriteJournal interface"
        )
    forbidden_adapter_ownership = sorted(
        token
        for token in (
            "generation_",
            "next_write_seq_",
            "HardwareWriteRecord",
            "HardwareWriteSink",
        )
        if re.search(rf"\b{re.escape(token)}\b", combined) is not None
    )
    if forbidden_adapter_ownership:
        if any(
            token in {"HardwareWriteRecord", "HardwareWriteSink"}
            for token in forbidden_adapter_ownership
        ):
            raise CrashStopContractError(
                "Adapter must not use legacy sink or record ownership"
            )
        raise CrashStopContractError(
            "Adapter must not own Writer sequence or generation state"
        )

    begin_write = source.find("write_journal_->begin_write(")
    delegated_write = source.find("upstream_->write(")
    finish_write = source.find("write_journal_->finish_write(")
    if begin_write < 0 or delegated_write < 0 or begin_write >= delegated_write:
        raise CrashStopContractError(
            "Adapter must begin before the delegated upstream write"
        )
    if finish_write < 0 or finish_write <= delegated_write:
        raise CrashStopContractError(
            "Adapter must finish after the delegated upstream write"
        )


def validate_plugin_description(path: Path) -> None:
    try:
        root = element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise CrashStopContractError(
            f"cannot parse {path}: {error}"
        ) from error
    classes = list(root.findall(".//class"))
    if len(classes) != 1:
        raise CrashStopContractError(
            "test hardware plugin description must contain exactly one class"
        )
    plugin_class = classes[0]
    if plugin_class.get("name") != TEST_PLUGIN_NAME:
        raise CrashStopContractError(
            "test hardware plugin must use the owned Adapter plugin name"
        )
    if plugin_class.get("type") != TEST_PLUGIN_TYPE:
        raise CrashStopContractError(
            "test hardware plugin XML must name the exact owned Adapter type"
        )
    if plugin_class.get("base_class_type") != PUBLIC_HARDWARE_BASE:
        raise CrashStopContractError(
            "test hardware plugin XML must name "
            "GazeboSimSystemInterface as its public base class"
        )


def literal_assignments(path: Path) -> dict[str, object]:
    source = read_text(path)
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise CrashStopContractError(
            f"cannot parse {path}: {error}"
        ) from error

    assignments: dict[str, object] = {}
    duplicates: set[str] = set()
    for statement in tree.body:
        if isinstance(statement, ast.Assign):
            targets = statement.targets
            value_node = statement.value
        elif isinstance(statement, ast.AnnAssign):
            targets = (statement.target,)
            value_node = statement.value
        else:
            continue
        if value_node is None:
            continue
        try:
            value = ast.literal_eval(value_node)
        except (ValueError, TypeError):
            continue
        for target in targets:
            if not isinstance(target, ast.Name):
                continue
            if target.id in assignments:
                duplicates.add(target.id)
            assignments[target.id] = value
    if duplicates:
        raise CrashStopContractError(
            "crash-stop evidence policy contains duplicate declarations: "
            + ", ".join(sorted(duplicates))
        )
    return assignments


def validate_evidence_policy(path: Path) -> None:
    assignments = literal_assignments(path)
    for name, expected in POLICY_VALUES.items():
        if name not in assignments:
            raise CrashStopContractError(
                f"crash-stop evidence policy is missing {name}"
            )
        actual = assignments[name]
        if actual == expected and type(actual) is type(expected):
            continue
        if name == "PRODUCER_REQUIRE_UNIQUE_FINAL_MARKER":
            raise CrashStopContractError(
                "ordinary authority/candidate producer arming barriers must "
                "not require a unique final marker"
            )
        if name == "GATE_REQUIRE_UNIQUE_FINAL_MARKER":
            raise CrashStopContractError(
                "MotionGate-death arming must require a marker new to the "
                "generation"
            )
        if name == "GATE_FINAL_MARKER_MAX_COMMITS":
            raise CrashStopContractError(
                "MotionGate-death final marker must have exactly one "
                "COMMITTED Gate output"
            )
        if name == "GATE_ACK_DEADLINE_OUTPUT_PERIODS":
            raise CrashStopContractError(
                "MotionGate-death controller ACK must arrive before the next "
                "Gate output period"
            )
        raise CrashStopContractError(
            "allocation-free claim must be scoped to added preallocated "
            "journal instrumentation and must exclude upstream "
            "GazeboSimSystem::write()"
        )

    source = read_text(path)
    required_enforcement = {
        (
            "marker_commit_count != "
            "GATE_FINAL_MARKER_MAX_COMMITS"
        ): "exactly-once final marker check",
        (
            "ack_output_seq >= next_output_seq"
        ): "ACK-before-repeat ordering check",
    }
    missing = sorted(
        description
        for fragment, description in required_enforcement.items()
        if fragment not in source
    )
    if missing:
        raise CrashStopContractError(
            "crash-stop evidence policy does not enforce: "
            + ", ".join(missing)
        )


def validate_product_seam_absent(path: Path, label: str) -> None:
    source = read_text(path).lower()
    leaked = sorted(
        marker for marker in PRODUCT_SEAM_MARKERS if marker in source
    )
    if leaked:
        raise CrashStopContractError(
            f"{label} must not expose the default-off crash journal or test "
            "hardware Adapter seam: "
            + ", ".join(leaked)
        )


def validate_product_xacro(path: Path) -> None:
    validate_product_seam_absent(path, "product Xacro")
    try:
        root = element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise CrashStopContractError(
            f"cannot parse product Xacro {path}: {error}"
        ) from error
    hardware_plugins = list(
        root.findall(".//ros2_control/hardware/plugin")
    )
    if (
        len(hardware_plugins) != 1
        or (hardware_plugins[0].text or "").strip()
        != UPSTREAM_PLUGIN_NAME
    ):
        raise CrashStopContractError(
            "product Xacro must select exactly one unchanged upstream "
            "gz_ros2_control/GazeboSimSystem hardware plugin"
        )


def element_signature(element: element_tree.Element) -> tuple[object, ...]:
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(element_signature(child) for child in element),
    )


def transformer_fixture() -> str:
    return (
        '<robot name="contract_fixture">'
        '<link name="base_link" />'
        '<ros2_control name="GazeboSimSystem" type="system">'
        '<hardware><plugin>'
        + UPSTREAM_PLUGIN_NAME
        + '</plugin></hardware>'
        '<joint name="left_wheel_joint">'
        '<command_interface name="velocity" />'
        '</joint></ros2_control>'
        '<gazebo reference="base_link"><gravity>true</gravity></gazebo>'
        '</robot>'
    )


def call_transformer(
    transform: object,
    product_urdf: str,
) -> str:
    if not callable(transform):
        raise CrashStopContractError(
            "crash robot-description transformer must export callable "
            "transform_product_urdf"
        )
    try:
        transformed = transform(
            product_urdf,
            "/voice_nav_hardware_0011223344556677",
            "0123456789abcdef0123456789abcdef",
        )
    except Exception as error:
        raise CrashStopContractError(
            "crash robot-description transformer rejected the valid "
            f"contract fixture: {error}"
        ) from error
    if not isinstance(transformed, str):
        raise CrashStopContractError(
            "transform_product_urdf must return serialized XML text"
        )
    return transformed


def validate_robot_transformer(path: Path) -> None:
    source = read_text(path)
    if re.search(
        r"(?i)(?<![a-z0-9])(?:[a-z]:[\\/]|/home/|/mnt/[a-z]/)",
        source,
    ):
        raise CrashStopContractError(
            "crash robot-description transformer must not contain a "
            "machine-specific absolute path"
        )
    try:
        namespace = runpy.run_path(str(path))
    except Exception as error:
        raise CrashStopContractError(
            f"cannot load crash robot-description transformer {path}: "
            f"{error}"
        ) from error
    transform = namespace.get("transform_product_urdf")
    fixture = transformer_fixture()
    transformed_text = call_transformer(transform, fixture)
    try:
        original = element_tree.fromstring(fixture)
        transformed = element_tree.fromstring(transformed_text)
    except element_tree.ParseError as error:
        raise CrashStopContractError(
            "crash robot-description transformer returned invalid XML: "
            f"{error}"
        ) from error

    hardware_nodes = list(
        transformed.findall(".//ros2_control/hardware")
    )
    hardware_plugins = list(
        transformed.findall(".//ros2_control/hardware/plugin")
    )
    if len(hardware_nodes) != 1 or len(hardware_plugins) != 1:
        raise CrashStopContractError(
            "transformed robot must contain exactly one hardware block and "
            "one hardware plugin"
        )
    if (hardware_plugins[0].text or "").strip() != TEST_PLUGIN_NAME:
        raise CrashStopContractError(
            "crash robot-description transformer must select the owned test "
            "Adapter"
        )
    hardware = hardware_nodes[0]
    expected_parameters = {
        "journal_name": "/voice_nav_hardware_0011223344556677",
        "journal_nonce": "0123456789abcdef0123456789abcdef",
    }
    journal_parameters = [
        child
        for child in list(hardware)
        if child.tag == "param"
        and child.get("name") in expected_parameters
    ]
    actual_parameters = {
        parameter.get("name"): (parameter.text or "").strip()
        for parameter in journal_parameters
    }
    if (
        len(journal_parameters) != len(expected_parameters)
        or actual_parameters != expected_parameters
    ):
        raise CrashStopContractError(
            "transformed hardware must contain exactly the validated journal "
            "name and nonce parameters"
        )

    for parameter in journal_parameters:
        hardware.remove(parameter)
    hardware_plugins[0].text = UPSTREAM_PLUGIN_NAME
    if element_signature(transformed) != element_signature(original):
        raise CrashStopContractError(
            "crash robot-description transformer changed XML outside the "
            "single hardware plugin replacement and two journal parameters"
        )

    duplicate = fixture.replace(
        "</hardware>",
        "<plugin>" + UPSTREAM_PLUGIN_NAME + "</plugin></hardware>",
        1,
    )
    missing = fixture.replace(
        "<plugin>" + UPSTREAM_PLUGIN_NAME + "</plugin>",
        "",
        1,
    )
    for invalid_fixture in (duplicate, missing):
        try:
            assert callable(transform)
            transform(
                invalid_fixture,
                "/voice_nav_hardware_0011223344556677",
                "0123456789abcdef0123456789abcdef",
            )
        except Exception:
            continue
        raise CrashStopContractError(
            "crash robot-description transformer must reject a missing or "
            "duplicate upstream hardware plugin"
        )


def validate_adapter_behavior_test(path: Path) -> None:
    source = read_text(path)
    required_tests = {
        "LoadsExportedAdapterAndItsPinnedUpstream",
        "ForwardsInitSimArgumentsAndResult",
        "ForwardsOnInitArgumentAndResult",
        "ForwardsOnConfigureArgumentAndResult",
        "ForwardsExportedInterfaceCollections",
        "ForwardsActivationTransitions",
        "ForwardsCommandModeSwitches",
        "ForwardsReadAndWriteCycles",
        "ObservesActualWheelCommandsAfterDelegatedWrite",
        "ReportsMissingEntityAfterFailedReinitialization",
        "ReportsMissingWheelCommandComponent",
        "ReportsRemovedWheelEntity",
        "ReportsEmptyWheelCommandComponent",
        "FinishesJournalCycleWhenDelegatedWriteThrows",
        "AttachesJournalIdentityBeforeFirstWrite",
        "RejectsIncompleteJournalIdentityWithoutAttaching",
        "RejectsJournalAttachmentFailure",
    }
    missing = sorted(
        test_name
        for test_name in required_tests
        if test_name not in source
    )
    if missing:
        raise CrashStopContractError(
            "Adapter compile/behavior smoke test is missing: "
            + ", ".join(missing)
        )


def validate_sim_cmake(path: Path) -> None:
    source = read_text(path)
    required_find_packages = (
        "gz_sim_vendor",
        "gz-sim8",
        "gz_ros2_control",
        "hardware_interface",
        "pluginlib",
        "rclcpp",
        "rclcpp_lifecycle",
    )
    positions: dict[str, int] = {}
    for package in required_find_packages:
        match = re.search(
            r"find_package\(\s*"
            + re.escape(package)
            + r"\s+REQUIRED\s*\)",
            source,
        )
        if match is None:
            raise CrashStopContractError(
                "voice_nav_sim CMake must directly discover " + package
            )
        positions[package] = match.start()
    if not (
        positions["gz_sim_vendor"] < positions["gz_ros2_control"]
        and positions["gz-sim8"] < positions["gz_ros2_control"]
    ):
        raise CrashStopContractError(
            "voice_nav_sim must discover gz_sim_vendor and gz-sim8 before "
            "consuming gz_ros2_control exported targets"
        )

    required_patterns = {
        (
            r"pluginlib_export_plugin_description_file\s*\(\s*"
            r"gz_ros2_control\s+test_support/"
            r"journaled_gazebo_sim_system_adapter_plugins\.xml\s*\)"
        ): "export the Adapter plugin description",
        (
            r"add_library\s*\(\s*" + re.escape(ADAPTER_TARGET) + r"\b"
        ): "build the Adapter library",
        (
            r"target_link_libraries\s*\(\s*"
            + re.escape(ADAPTER_TARGET)
            + r"\b[^)]*\bgz-sim8::gz-sim8\b[^)]*\)"
        ): "link the Adapter to the direct gz-sim8 target",
        (
            r"install\s*\(\s*TARGETS\s+"
            + re.escape(ADAPTER_TARGET)
            + r"\b"
        ): "install the Adapter library",
        (
            r"ament_add_gtest\s*\(\s*"
            + re.escape(ADAPTER_TEST_TARGET)
            + r"\b"
        ): "register the Adapter compile/behavior smoke test",
    }
    missing = sorted(
        description
        for pattern, description in required_patterns.items()
        if re.search(pattern, source, flags=re.DOTALL) is None
    )
    if missing:
        raise CrashStopContractError(
            "voice_nav_sim CMake does not fully wire the test Adapter: "
            + ", ".join(missing)
        )
    adapter_dependencies = re.search(
        r"ament_target_dependencies\s*\(\s*"
        + re.escape(ADAPTER_TARGET)
        + r"\b(?P<body>[^)]*)\)",
        source,
        flags=re.DOTALL,
    )
    required_adapter_dependencies = {
        "gz_ros2_control",
        "hardware_interface",
        "pluginlib",
        "rclcpp",
        "rclcpp_lifecycle",
    }
    if adapter_dependencies is None:
        missing_adapter_dependencies = required_adapter_dependencies
    else:
        dependency_body = adapter_dependencies.group("body")
        missing_adapter_dependencies = {
            dependency
            for dependency in required_adapter_dependencies
            if re.search(
                r"\b" + re.escape(dependency) + r"\b",
                dependency_body,
            ) is None
        }
    if missing_adapter_dependencies:
        raise CrashStopContractError(
            "Adapter target is missing direct ament dependencies: "
            + ", ".join(sorted(missing_adapter_dependencies))
        )
    test_link = re.search(
        r"target_link_libraries\(\s*"
        + re.escape(ADAPTER_TEST_TARGET)
        + r"\s+(?:(?:PRIVATE|PUBLIC|INTERFACE)\s+)?"
        + re.escape(ADAPTER_TARGET)
        + r"\b",
        source,
    )
    if test_link is None:
        raise CrashStopContractError(
            "voice_nav_sim CMake must link the Adapter behavior test to the "
            "Adapter library"
        )
    posix_ledger_target = "voice_nav_sim_hardware_write_ledger_posix"
    posix_definition = re.search(
        r"add_library\(\s*" + re.escape(posix_ledger_target) + r"\b",
        source,
    )
    adapter_link_blocks = re.finditer(
        r"target_link_libraries\(\s*"
        + re.escape(ADAPTER_TARGET)
        + r"\b(?P<body>.*?)\)",
        source,
        flags=re.DOTALL,
    )
    adapter_links_posix = any(
        re.search(rf"\b{re.escape(posix_ledger_target)}\b", match.group("body"))
        is not None
        for match in adapter_link_blocks
    )
    if posix_definition is None or not adapter_links_posix:
        raise CrashStopContractError(
            "voice_nav_sim Adapter target must link the POSIX hardware ledger"
        )


def validate_sim_package(path: Path) -> None:
    try:
        root = element_tree.fromstring(read_text(path))
    except element_tree.ParseError as error:
        raise CrashStopContractError(
            f"cannot parse voice_nav_sim package.xml {path}: {error}"
        ) from error
    direct_test_dependencies = {
        (dependency.text or "").strip()
        for dependency_tag in ("depend", "test_depend")
        for dependency in root.findall(dependency_tag)
    }
    required = {
        "gz_ros2_control",
        "gz_sim_vendor",
        "hardware_interface",
        "pal_statistics_msgs",
        "pluginlib",
        "rclcpp",
        "rclcpp_lifecycle",
    }
    missing = sorted(required - direct_test_dependencies)
    if missing:
        raise CrashStopContractError(
            "voice_nav_sim package.xml is missing direct test dependencies: "
            + ", ".join(missing)
        )


def validate_contract(root: Path) -> None:
    paths = required_artifacts(root)
    validate_adapter(
        paths["adapter_header"],
        paths["adapter_source"],
        paths["write_journal"],
    )
    validate_plugin_description(paths["adapter_plugin"])
    validate_adapter_behavior_test(paths["adapter_test"])
    validate_sim_cmake(paths["sim_cmake"])
    validate_sim_package(paths["sim_package"])
    validate_evidence_policy(paths["evidence_policy"])
    validate_robot_transformer(paths["robot_transformer"])
    validate_product_xacro(paths["product_xacro"])
    validate_product_seam_absent(paths["product_launch"], "product launch")
    validate_product_seam_absent(
        paths["simulation_launch"],
        "product simulation launch",
    )
    validate_product_seam_absent(
        paths["product_gate_yaml"],
        "product MotionGate YAML",
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    root = arguments.root.resolve()
    try:
        validate_contract(root)
    except CrashStopContractError as error:
        print(f"Crash-stop contract failed: {error}", file=sys.stderr)
        return 1
    print("Crash-stop contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
