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
    "runtime_adapter_test": (
        "src/voice_nav_sim/test/"
        "test_journaled_gazebo_hardware_write.py"
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
RUNTIME_ADAPTER_TEST_FILE = (
    "test/test_journaled_gazebo_hardware_write.py"
)
RUNTIME_ADAPTER_CTEST = (
    "test_test_journaled_gazebo_hardware_write.py"
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

    def cmake_tokens(block: str) -> list[str]:
        raw_tokens = re.findall(
            r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[^\s]+',
            block,
        )
        return [
            token[1:-1]
            if len(token) >= 2 and token[0] == token[-1] and token[0] in "\"'"
            else token
            for token in raw_tokens
        ]
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

    cmake_without_comments = "\n".join(
        line.split("#", 1)[0] for line in source.splitlines()
    )
    command_shadow = re.search(
        r"\b(?:macro|function)\s*\(\s*"
        r"(?:add_launch_test|set_tests_properties|set_property)\b",
        cmake_without_comments,
        flags=re.IGNORECASE,
    )
    build_testing_rebind = re.search(
        r"\b(?:set|unset|option)\s*\(\s*BUILD_TESTING\b"
        r"|\bset_property\s*\(\s*CACHE\s+BUILD_TESTING\b",
        cmake_without_comments,
        flags=re.IGNORECASE,
    )

    def condition_stack_at(position: int) -> list[tuple[str, ...]]:
        stack: list[tuple[str, ...]] = []
        control_commands = re.finditer(
            r"\b(?P<command>if|elseif|else|endif)\s*"
            r"\((?P<body>.*?)\)",
            cmake_without_comments,
            flags=re.DOTALL | re.IGNORECASE,
        )
        for match in control_commands:
            if match.start() >= position:
                break
            command = match.group("command").lower()
            if command == "if":
                stack.append(tuple(cmake_tokens(match.group("body"))))
            elif command == "elseif" and stack:
                stack[-1] = (
                    "__ELSEIF__",
                    *cmake_tokens(match.group("body")),
                )
            elif command == "else" and stack:
                stack[-1] = ("__ELSE__",)
            elif command == "endif" and stack:
                stack.pop()
        return stack

    testing_condition = [("BUILD_TESTING",)]
    runtime_registrations = list(re.finditer(
        r"add_launch_test\s*\(\s*['\"]?"
        + re.escape(RUNTIME_ADAPTER_TEST_FILE)
        + r"['\"]?(?=\s)(?P<body>.*?)\)",
        cmake_without_comments,
        flags=re.DOTALL,
    ))

    def parse_launch_arguments(
        tokens: list[str],
    ) -> tuple[dict[str, list[str]], dict[str, list[str]], list[str]]:
        """Mirror cmake_parse_arguments for add_launch_test's keywords."""
        one_value_keywords = {"TARGET", "TIMEOUT", "PYTHON_EXECUTABLE"}
        multi_value_keywords = {"ARGS", "LABELS"}
        recognized = one_value_keywords | multi_value_keywords
        one_values = {keyword: [] for keyword in one_value_keywords}
        multi_values = {keyword: [] for keyword in multi_value_keywords}
        unparsed: list[str] = []
        active_multi: str | None = None
        index = 0
        while index < len(tokens):
            token = tokens[index]
            if token in one_value_keywords:
                active_multi = None
                if index + 1 >= len(tokens) or tokens[index + 1] in recognized:
                    one_values[token].append("")
                    index += 1
                    continue
                one_values[token].append(tokens[index + 1])
                index += 2
                continue
            if token in multi_value_keywords:
                active_multi = token
                index += 1
                continue
            if active_multi is not None:
                multi_values[active_multi].append(token)
            else:
                unparsed.append(token)
            index += 1
        return one_values, multi_values, unparsed

    runtime_is_isolated = False
    if (
        len(runtime_registrations) == 1
        and command_shadow is None
        and build_testing_rebind is None
    ):
        registration_tokens = cmake_tokens(
            runtime_registrations[0].group("body")
        )
        one_values, multi_values, unparsed = parse_launch_arguments(
            registration_tokens
        )
        targets = one_values["TARGET"]
        target_is_owned = (
            not targets
            or (len(targets) == 1 and targets[0] == RUNTIME_ADAPTER_CTEST)
        )
        runner_is_exact = unparsed == [
            "RUNNER",
            "${ament_cmake_ros_DIR}/run_test_isolated.py",
        ]
        registration_options_are_owned = (
            one_values["TIMEOUT"] == ["180"]
            and not one_values["PYTHON_EXECUTABLE"]
            and not multi_values["ARGS"]
            and not multi_values["LABELS"]
        )
        runtime_is_isolated = (
            target_is_owned
            and runner_is_exact
            and registration_options_are_owned
            and condition_stack_at(runtime_registrations[0].start())
            == testing_condition
        )

    if not runtime_is_isolated:
        raise CrashStopContractError(
            "voice_nav_sim CMake must register the isolated runtime Adapter "
            "test with its owned CTest target"
        )

    serial_assignments: list[tuple[str, int]] = []
    property_blocks = re.finditer(
        r"\b(?P<command>set_tests_properties|set_property)\s*"
        r"\((?P<body>.*?)\)",
        cmake_without_comments,
        flags=re.DOTALL | re.IGNORECASE,
    )
    for match in property_blocks:
        command = match.group("command").lower()
        tokens = cmake_tokens(match.group("body"))
        if command == "set_tests_properties":
            if "PROPERTIES" not in tokens:
                continue
            properties_index = tokens.index("PROPERTIES")
            targets = tokens[:properties_index]
            property_tokens = tokens[properties_index + 1:]
            dynamic_property_name = any(
                "${" in property_tokens[index]
                or "$<" in property_tokens[index]
                for index in range(0, len(property_tokens), 2)
            )
            run_serial_values = [
                property_tokens[index + 1]
                for index in range(0, len(property_tokens) - 1, 2)
                if property_tokens[index] == "RUN_SERIAL"
            ]
            ambiguous_target = any(
                "${" in target or "$<" in target
                for target in targets
            )
            if (
                dynamic_property_name
                and (
                    RUNTIME_ADAPTER_CTEST in targets
                    or ambiguous_target
                )
            ):
                serial_assignments.append(("", match.start()))
                continue
            if run_serial_values and ambiguous_target:
                serial_assignments.append(("", match.start()))
                continue
            if RUNTIME_ADAPTER_CTEST not in targets:
                continue
            serial_assignments.extend(
                (value, match.start())
                for value in run_serial_values
            )
            continue

        if not tokens or tokens[0] != "TEST" or "PROPERTY" not in tokens:
            continue
        property_index = tokens.index("PROPERTY")
        targets_and_options = tokens[1:property_index]
        targets = [
            token
            for token in targets_and_options
            if token not in {"APPEND", "APPEND_STRING"}
        ]
        if property_index + 1 >= len(tokens):
            continue
        property_name = tokens[property_index + 1]
        ambiguous_target = any(
            "${" in target or "$<" in target
            for target in targets
        )
        if (
            ("${" in property_name or "$<" in property_name)
            and (
                RUNTIME_ADAPTER_CTEST in targets
                or ambiguous_target
            )
        ):
            serial_assignments.append(("", match.start()))
            continue
        if property_name != "RUN_SERIAL":
            continue
        if ambiguous_target:
            serial_assignments.append(("", match.start()))
            continue
        if RUNTIME_ADAPTER_CTEST not in targets:
            continue
        values = tokens[property_index + 2:]
        if (
            len(values) != 1
            or any(
                option in targets_and_options
                for option in ("APPEND", "APPEND_STRING")
            )
        ):
            serial_assignments.append(("", match.start()))
        else:
            serial_assignments.append((values[0], match.start()))

    cmake_truthy = {"1", "ON", "YES", "TRUE", "Y"}
    runtime_is_serial = (
        len(serial_assignments) == 1
        and serial_assignments[0][0].upper() in cmake_truthy
        and condition_stack_at(serial_assignments[0][1])
        == testing_condition
    )
    if not runtime_is_serial:
        raise CrashStopContractError(
            "voice_nav_sim CMake must serialize its Gazebo process with one "
            "unambiguous RUN_SERIAL assignment"
        )


def validate_runtime_adapter_test(path: Path) -> None:
    source = read_text(path)
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise CrashStopContractError(
            f"cannot parse runtime Adapter test {path}: {error}"
        ) from error

    def final_call_name(node: ast.AST) -> str | None:
        if not isinstance(node, ast.Call):
            return None
        if isinstance(node.func, ast.Name):
            return node.func.id
        if isinstance(node.func, ast.Attribute):
            return node.func.attr
        return None

    def receiver_call(
        node: ast.AST,
        receiver: str,
        method: str,
    ) -> bool:
        return (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == method
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == receiver
        )

    def ast_equal(left: ast.AST | None, right: ast.AST | None) -> bool:
        if left is None or right is None:
            return False
        return ast.dump(left, include_attributes=False) == ast.dump(
            right,
            include_attributes=False,
        )

    def expression(source_text: str) -> ast.expr:
        return ast.parse(source_text, mode="eval").body

    def keyword_value(call: ast.AST | None, name: str) -> ast.expr | None:
        if not isinstance(call, ast.Call):
            return None
        matches = [
            keyword.value
            for keyword in call.keywords
            if keyword.arg == name
        ]
        return matches[0] if len(matches) == 1 else None

    def call_uses_name(
        call: ast.AST | None,
        position: int,
        name: str,
    ) -> bool:
        return (
            isinstance(call, ast.Call)
            and len(call.args) > position
            and isinstance(call.args[position], ast.Name)
            and call.args[position].id == name
        )

    def dotted_name(node: ast.AST) -> str:
        parts: list[str] = []
        current = node
        if isinstance(current, ast.Call):
            current = current.func
        while isinstance(current, ast.Attribute):
            parts.append(current.attr)
            current = current.value
        if isinstance(current, ast.Name):
            parts.append(current.id)
        return ".".join(reversed(parts))

    functions = {
        node.name: node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    runtime_method_name = (
        "test_real_gazebo_writer_records_nonzero_wheel_commands"
    )
    runtime_test = functions.get(runtime_method_name)
    if runtime_test is None:
        raise CrashStopContractError(
            "runtime Adapter test must expose the owned real-Gazebo behavior"
        )

    runtime_class = next(
        (
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ClassDef) and runtime_test in node.body
        ),
        None,
    )
    unittest_is_imported = any(
        isinstance(statement, ast.Import)
        and any(
            alias.name == "unittest" and alias.asname is None
            for alias in statement.names
        )
        for statement in tree.body
    )
    runtime_is_collected = (
        runtime_class is not None
        and runtime_class in tree.body
        and runtime_class.name == "JournaledGazeboHardwareWriteTest"
        and runtime_test in runtime_class.body
        and len(
            [
                statement
                for statement in runtime_class.body
                if isinstance(
                    statement,
                    (ast.FunctionDef, ast.AsyncFunctionDef),
                )
                and statement.name == runtime_method_name
            ]
        )
        == 1
        and len(runtime_class.bases) == 1
        and ast_equal(
            runtime_class.bases[0],
            expression("unittest.TestCase"),
        )
        and unittest_is_imported
        and [argument.arg for argument in runtime_test.args.args]
        == ["self", "proc_info", "gazebo_action", "ledger_owner"]
        and not runtime_test.args.posonlyargs
        and not runtime_test.args.kwonlyargs
        and runtime_test.args.vararg is None
        and runtime_test.args.kwarg is None
        and not runtime_test.args.defaults
        and not runtime_test.args.kw_defaults
    )
    if not runtime_is_collected:
        raise CrashStopContractError(
            "runtime Adapter evidence test must remain executable as a "
            "top-level unittest.TestCase method"
        )

    protected_assertion_methods = {
        "assertEqual",
        "assertGreater",
        "assertTrue",
    }

    def assignment_targets(node: ast.AST) -> list[ast.AST]:
        if isinstance(node, ast.Assign):
            return list(node.targets)
        if isinstance(node, (ast.AnnAssign, ast.AugAssign, ast.NamedExpr)):
            return [node.target]
        if isinstance(node, ast.Delete):
            return list(node.targets)
        return []

    def target_root_name(node: ast.AST) -> str | None:
        current = node
        while isinstance(current, (ast.Attribute, ast.Subscript)):
            current = current.value
        return current.id if isinstance(current, ast.Name) else None

    proof_primitives = {
        "abs",
        "all",
        "any",
        "math",
        "struct",
        "tuple",
        "zip",
    }
    proof_primitive_is_rebound = any(
        target_root_name(target) in proof_primitives | {"double_from_bits"}
        for node in ast.walk(tree)
        for target in assignment_targets(node)
    )
    proof_primitive_is_redefined = any(
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        and node.name in proof_primitives
        for node in ast.walk(tree)
    )
    bound_import_names = [
        alias.asname or alias.name.split(".")[-1]
        for statement in tree.body
        if isinstance(statement, (ast.Import, ast.ImportFrom))
        for alias in statement.names
    ]
    imports_are_owned = (
        bound_import_names.count("math") == 1
        and bound_import_names.count("struct") == 1
        and not any(
            name in {"abs", "all", "any", "tuple", "zip"}
            for name in bound_import_names
        )
    )
    decoder_functions = [
        statement
        for statement in tree.body
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef))
        and statement.name == "double_from_bits"
    ]
    decoder_returns = [] if len(decoder_functions) != 1 else [
        statement
        for statement in decoder_functions[0].body
        if isinstance(statement, ast.Return)
    ]
    decoder_is_exact = (
        len(decoder_functions) == 1
        and isinstance(decoder_functions[0], ast.FunctionDef)
        and [
            argument.arg
            for argument in decoder_functions[0].args.args
        ]
        == ["bits"]
        and not decoder_functions[0].args.posonlyargs
        and not decoder_functions[0].args.kwonlyargs
        and decoder_functions[0].args.vararg is None
        and decoder_functions[0].args.kwarg is None
        and len(decoder_returns) == 1
        and ast_equal(
            decoder_returns[0].value,
            expression(
                "struct.unpack('<d', struct.pack('<Q', bits))[0]"
            ),
        )
    )
    if (
        proof_primitive_is_rebound
        or proof_primitive_is_redefined
        or not imports_are_owned
        or not decoder_is_exact
    ):
        raise CrashStopContractError(
            "runtime Adapter evidence must preserve its proof primitives and "
            "exact wheel-bit decoder"
        )

    fixture_binding_is_rebound = any(
        target_root_name(target)
        in {"self", "proc_info", "gazebo_action", "ledger_owner"}
        and not any(
            isinstance(child, ast.Attribute)
            and child.attr in protected_assertion_methods
            for child in ast.walk(target)
        )
        for node in ast.walk(runtime_test)
        for target in assignment_targets(node)
    )
    if fixture_binding_is_rebound:
        raise CrashStopContractError(
            "runtime Adapter evidence test must remain executable with its "
            "injected fixture bindings"
        )

    runtime_binding_is_rebound = any(
        target_root_name(target) == "JournaledGazeboHardwareWriteTest"
        for node in ast.walk(tree)
        for target in assignment_targets(node)
    )
    runtime_binding_is_dynamically_rebound = any(
        isinstance(node, ast.Call)
        and final_call_name(node) in {"setattr", "delattr", "object"}
        and any(
            isinstance(child, ast.Name)
            and child.id == "JournaledGazeboHardwareWriteTest"
            for child in ast.walk(node)
        )
        and any(
            isinstance(child, ast.Constant)
            and child.value == runtime_method_name
            for child in ast.walk(node)
        )
        for node in ast.walk(tree)
    )
    if runtime_binding_is_rebound or runtime_binding_is_dynamically_rebound:
        raise CrashStopContractError(
            "runtime Adapter evidence test must remain executable without "
            "post-definition replacement"
        )

    assertion_method_is_rebound = any(
        any(
            isinstance(child, ast.Attribute)
            and child.attr in protected_assertion_methods
            for child in ast.walk(target)
        )
        for node in ast.walk(tree)
        for target in assignment_targets(node)
    )
    unittest_binding_is_rebound = any(
        any(
            (
                isinstance(child, ast.Name)
                and child.id == "unittest"
            )
            or (
                isinstance(child, ast.Attribute)
                and dotted_name(child).startswith("unittest.")
            )
            for child in ast.walk(target)
        )
        for node in ast.walk(tree)
        for target in assignment_targets(node)
    )
    class_overrides_assertion = any(
        isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef))
        and statement.name in protected_assertion_methods
        for statement in runtime_class.body
    )
    dynamic_assertion_replacement = any(
        isinstance(node, ast.Call)
        and final_call_name(node) in {"setattr", "delattr", "patch", "object"}
        and any(
            isinstance(child, ast.Constant)
            and child.value in protected_assertion_methods
            for child in ast.walk(node)
        )
        for node in ast.walk(tree)
    )
    if (
        assertion_method_is_rebound
        or unittest_binding_is_rebound
        or class_overrides_assertion
        or dynamic_assertion_replacement
    ):
        raise CrashStopContractError(
            "runtime Adapter evidence must use unmodified unittest assertions"
        )

    forbidden_markers = {
        "expectedfailure",
        "importorskip",
        "skip",
        "skipif",
        "skiptest",
        "skipunless",
        "xfail",
    }
    decorators = list(runtime_test.decorator_list)
    if runtime_class is not None:
        decorators.extend(runtime_class.decorator_list)
    decorator_disables_test = any(
        any(
            part.lower() in forbidden_markers
            for part in dotted_name(decorator).split(".")
        )
        for decorator in decorators
    )
    call_disables_test = any(
        final_call_name(node).lower() in forbidden_markers
        for node in ast.walk(runtime_test)
        if isinstance(node, ast.Call) and final_call_name(node) is not None
    )
    module_disables_test = any(
        isinstance(node, (ast.Assign, ast.AnnAssign))
        and any(
            (
                isinstance(target, ast.Name)
                and target.id
                in {"pytestmark", "__test__", "__unittest_skip__"}
            )
            or (
                isinstance(target, ast.Attribute)
                and target.attr in {"__test__", "__unittest_skip__"}
            )
            for target in (
                node.targets
                if isinstance(node, ast.Assign)
                else [node.target]
            )
        )
        for node in ast.walk(tree)
    )
    early_termination = any(
        isinstance(node, (ast.Return, ast.Raise))
        for node in ast.walk(runtime_test)
    )
    if (
        decorator_disables_test
        or call_disables_test
        or module_disables_test
        or early_termination
    ):
        raise CrashStopContractError(
            "runtime Adapter evidence test must remain executable and cannot "
            "return, raise, skip, xfail, or use expected-failure markers"
        )

    assignments_by_name: dict[str, list[tuple[int, ast.expr]]] = {}
    assertion_calls: list[tuple[int, ast.Call]] = []
    foreign_assertion_receiver = any(
        isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Call)
        and isinstance(statement.value.func, ast.Attribute)
        and statement.value.func.attr in protected_assertion_methods
        and not (
            isinstance(statement.value.func.value, ast.Name)
            and statement.value.func.value.id == "self"
        )
        for statement in runtime_test.body
    )
    if foreign_assertion_receiver:
        raise CrashStopContractError(
            "runtime Adapter evidence must use unmodified unittest assertions "
            "through the collected self receiver"
        )
    for index, statement in enumerate(runtime_test.body):
        if (
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name)
        ):
            assignments_by_name.setdefault(
                statement.targets[0].id,
                [],
            ).append((index, statement.value))
        if (
            isinstance(statement, ast.Expr)
            and isinstance(statement.value, ast.Call)
            and isinstance(statement.value.func, ast.Attribute)
            and statement.value.func.attr in protected_assertion_methods
            and isinstance(statement.value.func.value, ast.Name)
            and statement.value.func.value.id == "self"
        ):
            assertion_calls.append((index, statement.value))

    def unique_assignment(name: str) -> tuple[int, ast.expr] | None:
        assignments = assignments_by_name.get(name, [])
        return assignments[0] if len(assignments) == 1 else None

    def has_binary_assert(
        method: str,
        left: ast.AST,
        right: ast.AST,
        *,
        commutative: bool = False,
    ) -> bool:
        for _, call in assertion_calls:
            if final_call_name(call) != method or len(call.args) < 2:
                continue
            direct = ast_equal(call.args[0], left) and ast_equal(
                call.args[1],
                right,
            )
            reverse = commutative and ast_equal(
                call.args[0],
                right,
            ) and ast_equal(call.args[1], left)
            if direct or reverse:
                return True
        return False

    def true_assertion(
        predicate,
    ) -> tuple[int, ast.Call] | None:
        return next(
            (
                (index, call)
                for index, call in assertion_calls
                if final_call_name(call) == "assertTrue"
                and call.args
                and predicate(call.args[0])
            ),
            None,
        )

    launched_assignment = unique_assignment("launched_pid")
    launched_pid = (
        None if launched_assignment is None else launched_assignment[1]
    )
    pid_capture = (
        isinstance(launched_pid, ast.Subscript)
        and isinstance(launched_pid.value, ast.Attribute)
        and launched_pid.value.attr == "process_details"
        and isinstance(launched_pid.value.value, ast.Name)
        and launched_pid.value.value.id == "gazebo_action"
        and isinstance(launched_pid.slice, ast.Constant)
        and launched_pid.slice.value == "pid"
    )

    def asserts_exact_writer(call: ast.Call) -> bool:
        if final_call_name(call) != "assertEqual" or len(call.args) < 2:
            return False
        for candidate, expected in (
            (call.args[0], call.args[1]),
            (call.args[1], call.args[0]),
        ):
            if (
                receiver_call(candidate, "ledger_owner", "wait_for_writer")
                and len(candidate.args) == 1
                and call_uses_name(candidate, 0, "launched_pid")
                and ast_equal(expected, expression("launched_pid"))
            ):
                return True
        return False

    writer_assertion = next(
        (
            (index, call)
            for index, call in assertion_calls
            if asserts_exact_writer(call)
        ),
        None,
    )
    if not pid_capture or writer_assertion is None:
        raise CrashStopContractError(
            "runtime Adapter test must bind the ledger to the exact Gazebo "
            "Writer PID"
        )

    required_assignment_names = (
        "arm_ticket",
        "arm_response",
        "seal_ticket",
        "seal_response",
        "snapshot",
        "segments",
        "wheel_commands",
    )
    required_assignments = {
        name: unique_assignment(name)
        for name in required_assignment_names
    }
    if any(
        assignment is None
        for assignment in required_assignments.values()
    ):
        raise CrashStopContractError(
            "runtime Adapter test must prove ARM, SEAL, immutable snapshot, "
            "and ACK through the Parent ledger"
        )

    arm_index, arm_call = required_assignments["arm_ticket"]
    arm_response_index, arm_response_call = required_assignments[
        "arm_response"
    ]
    seal_index, seal_call = required_assignments["seal_ticket"]
    seal_response_index, seal_response_call = required_assignments[
        "seal_response"
    ]
    snapshot_index, snapshot_call = required_assignments["snapshot"]
    arm_interval = keyword_value(arm_call, "interval_id")
    seal_interval = keyword_value(seal_call, "interval_id")
    snapshot_interval = keyword_value(snapshot_call, "interval_id")

    ack_assertion = true_assertion(
        lambda node: (
            receiver_call(node, "ledger_owner", "acknowledge")
            and len(node.args) == 1
            and call_uses_name(node, 0, "snapshot")
        )
    )
    ordered_operations = (
        launched_assignment is not None
        and ack_assertion is not None
        and (
            launched_assignment[0]
            < writer_assertion[0]
            < arm_index
            < arm_response_index
            < seal_index
            < seal_response_index
            < snapshot_index
            < ack_assertion[0]
        )
    )
    interval_is_complete = (
        receiver_call(arm_call, "ledger_owner", "post_arm")
        and isinstance(arm_call, ast.Call)
        and not arm_call.args
        and arm_interval is not None
        and keyword_value(arm_call, "segment_budget") is not None
        and keyword_value(arm_call, "invocation_budget") is not None
        and ast_equal(
            keyword_value(arm_call, "require_zero_commands"),
            expression("False"),
        )
        and receiver_call(
            arm_response_call,
            "ledger_owner",
            "wait_response",
        )
        and isinstance(arm_response_call, ast.Call)
        and len(arm_response_call.args) == 1
        and call_uses_name(arm_response_call, 0, "arm_ticket")
        and receiver_call(seal_call, "ledger_owner", "post_seal")
        and isinstance(seal_call, ast.Call)
        and not seal_call.args
        and ast_equal(seal_interval, arm_interval)
        and ast_equal(
            keyword_value(seal_call, "bank_index"),
            expression("arm_response[10]"),
        )
        and ast_equal(
            keyword_value(seal_call, "bank_epoch"),
            expression("arm_response[11]"),
        )
        and ast_equal(
            keyword_value(seal_call, "not_before_sim_stamp_ns"),
            expression("0"),
        )
        and ast_equal(
            keyword_value(seal_call, "require_exact_stamp"),
            expression("False"),
        )
        and receiver_call(
            seal_response_call,
            "ledger_owner",
            "wait_response",
        )
        and isinstance(seal_response_call, ast.Call)
        and len(seal_response_call.args) == 1
        and call_uses_name(seal_response_call, 0, "seal_ticket")
        and receiver_call(
            snapshot_call,
            "ledger_owner",
            "read_sealed_interval",
        )
        and isinstance(snapshot_call, ast.Call)
        and not snapshot_call.args
        and ast_equal(snapshot_interval, arm_interval)
        and ast_equal(
            keyword_value(snapshot_call, "bank_index"),
            expression("seal_response[10]"),
        )
        and ast_equal(
            keyword_value(snapshot_call, "bank_epoch"),
            expression("seal_response[11]"),
        )
        and ast_equal(
            keyword_value(snapshot_call, "seal_fence_write_seq"),
            expression("seal_response[12]"),
        )
        and has_binary_assert(
            "assertEqual",
            expression("arm_response[9]"),
            expression("CONTROL_RESPONSE_OK"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression("seal_response[9]"),
            expression("CONTROL_RESPONSE_OK"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression("seal_response[10]"),
            expression("arm_response[10]"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression("seal_response[11]"),
            expression("arm_response[11]"),
            commutative=True,
        )
        and has_binary_assert(
            "assertGreater",
            expression("seal_response[12]"),
            expression("arm_response[12]"),
        )
        and ordered_operations
    )
    if not interval_is_complete:
        raise CrashStopContractError(
            "runtime Adapter test must prove ARM, SEAL, immutable snapshot, "
            "and ACK through the Parent ledger"
        )

    _, segments_call = required_assignments["segments"]
    _, wheel_commands_call = required_assignments["wheel_commands"]

    def exact_generator_quantifier(
        node: ast.AST,
        quantifier: str,
        element: ast.AST,
        iterable_name: str | None = None,
    ) -> bool:
        if (
            not isinstance(node, ast.Call)
            or final_call_name(node) != quantifier
            or len(node.args) != 1
            or not isinstance(node.args[0], ast.GeneratorExp)
        ):
            return False
        generator = node.args[0]
        if len(generator.generators) != 1:
            return False
        if iterable_name is not None and not ast_equal(
            generator.generators[0].iter,
            expression(iterable_name),
        ):
            return False
        return ast_equal(generator.elt, element)

    def positive_abs_comparison(node: ast.AST, variable: str) -> bool:
        if (
            not isinstance(node, ast.Compare)
            or len(node.ops) != 1
            or not isinstance(node.ops[0], ast.Gt)
            or len(node.comparators) != 1
            or not isinstance(node.left, ast.Call)
            or final_call_name(node.left) != "abs"
            or len(node.left.args) != 1
            or not ast_equal(node.left.args[0], expression(variable))
            or not isinstance(node.comparators[0], ast.Constant)
            or isinstance(node.comparators[0].value, bool)
            or not isinstance(node.comparators[0].value, (int, float))
        ):
            return False
        return node.comparators[0].value > 0

    def positive_wheel_pair(node: ast.AST) -> bool:
        if not isinstance(node, ast.BoolOp) or not isinstance(node.op, ast.And):
            return False
        return (
            len(node.values) == 2
            and any(
                positive_abs_comparison(value, "left")
                for value in node.values
            )
            and any(
                positive_abs_comparison(value, "right")
                for value in node.values
            )
        )

    segments_generator = (
        segments_call.args[0]
        if isinstance(segments_call, ast.Call)
        and final_call_name(segments_call) == "tuple"
        and len(segments_call.args) == 1
        and not segments_call.keywords
        and isinstance(segments_call.args[0], ast.GeneratorExp)
        else None
    )
    segments_from_snapshot = (
        isinstance(segments_generator, ast.GeneratorExp)
        and ast_equal(segments_generator.elt, expression("segment"))
        and len(segments_generator.generators) == 2
        and isinstance(segments_generator.generators[0].target, ast.Name)
        and segments_generator.generators[0].target.id == "page"
        and ast_equal(
            segments_generator.generators[0].iter,
            expression("snapshot.pages"),
        )
        and not segments_generator.generators[0].ifs
        and not segments_generator.generators[0].is_async
        and isinstance(segments_generator.generators[1].target, ast.Name)
        and segments_generator.generators[1].target.id == "segment"
        and ast_equal(
            segments_generator.generators[1].iter,
            expression("page.segments"),
        )
        and not segments_generator.generators[1].ifs
        and not segments_generator.generators[1].is_async
    )
    wheel_commands_generator = (
        wheel_commands_call.args[0]
        if isinstance(wheel_commands_call, ast.Call)
        and final_call_name(wheel_commands_call) == "tuple"
        and len(wheel_commands_call.args) == 1
        and not wheel_commands_call.keywords
        and isinstance(wheel_commands_call.args[0], ast.GeneratorExp)
        else None
    )
    wheel_commands_from_segments = (
        isinstance(wheel_commands_generator, ast.GeneratorExp)
        and ast_equal(
            wheel_commands_generator.elt,
            expression(
                "(double_from_bits(segment[6]), "
                "double_from_bits(segment[7]))"
            ),
        )
        and len(wheel_commands_generator.generators) == 1
        and isinstance(
            wheel_commands_generator.generators[0].target,
            ast.Name,
        )
        and wheel_commands_generator.generators[0].target.id == "segment"
        and ast_equal(
            wheel_commands_generator.generators[0].iter,
            expression("segments"),
        )
        and not wheel_commands_generator.generators[0].ifs
        and not wheel_commands_generator.generators[0].is_async
    )
    command_indexes = set()
    for call in ast.walk(wheel_commands_call):
        if final_call_name(call) != "double_from_bits" or len(call.args) != 1:
            continue
        argument = call.args[0]
        if (
            isinstance(argument, ast.Subscript)
            and isinstance(argument.value, ast.Name)
            and argument.value.id == "segment"
            and isinstance(argument.slice, ast.Constant)
            and isinstance(argument.slice.value, int)
        ):
            command_indexes.add(argument.slice.value)

    contiguous_assertion = true_assertion(
        lambda node: exact_generator_quantifier(
            node,
            "all",
            expression("following[1] == previous[2] + 1"),
        )
        and isinstance(node.args[0].generators[0].iter, ast.Call)
        and final_call_name(node.args[0].generators[0].iter) == "zip"
        and len(node.args[0].generators[0].iter.args) == 2
        and ast_equal(
            node.args[0].generators[0].iter.args[0],
            expression("segments"),
        )
        and ast_equal(
            node.args[0].generators[0].iter.args[1],
            expression("segments[1:]"),
        )
    )
    upstream_ok_assertion = true_assertion(
        lambda node: exact_generator_quantifier(
            node,
            "all",
            expression("segment[5] == 0"),
            "segments",
        )
    )
    finite_assertion = true_assertion(
        lambda node: exact_generator_quantifier(
            node,
            "all",
            expression(
                "math.isfinite(left) and math.isfinite(right)"
            ),
            "wheel_commands",
        )
    )
    nonzero_assertion = true_assertion(
        lambda node: (
            isinstance(node, ast.Call)
            and final_call_name(node) == "any"
            and len(node.args) == 1
            and isinstance(node.args[0], ast.GeneratorExp)
            and len(node.args[0].generators) == 1
            and ast_equal(
                node.args[0].generators[0].iter,
                expression("wheel_commands"),
            )
            and positive_wheel_pair(node.args[0].elt)
        )
    )
    clean_evidence = (
        has_binary_assert(
            "assertEqual",
            expression("snapshot.terminal_state"),
            expression("BANK_STATE_SEALED_OK"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression("snapshot.oracle_faults"),
            expression("0"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression(
                "ledger_owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD)"
            ),
            expression("0"),
            commutative=True,
        )
        and true_assertion(
            lambda node: ast_equal(node, expression("segments"))
        )
        is not None
        and has_binary_assert(
            "assertEqual",
            expression("segments[0][1]"),
            expression("arm_response[12] + 1"),
            commutative=True,
        )
        and has_binary_assert(
            "assertEqual",
            expression("segments[-1][2]"),
            expression("seal_response[12]"),
            commutative=True,
        )
        and contiguous_assertion is not None
        and upstream_ok_assertion is not None
        and finite_assertion is not None
        and nonzero_assertion is not None
        and segments_from_snapshot
        and wheel_commands_from_segments
        and {6, 7} <= command_indexes
    )
    if not clean_evidence:
        raise CrashStopContractError(
            "runtime Adapter test must require a fault-free terminal ledger "
            "with VALID upstream-OK non-zero wheel evidence"
        )

    expansion = functions.get("expanded_journaled_robot_description")
    generate = functions.get("generate_test_description")
    expansion_assignments = [] if expansion is None else [
        statement
        for statement in expansion.body
        if isinstance(statement, ast.Assign)
        and len(statement.targets) == 1
        and isinstance(statement.targets[0], ast.Name)
        and statement.targets[0].id == "product_urdf"
    ]
    expansion_returns = [] if expansion is None else [
        statement
        for statement in expansion.body
        if isinstance(statement, ast.Return)
    ]
    product_urdf_value = (
        expansion_assignments[0].value
        if len(expansion_assignments) == 1
        else None
    )
    transform_return = (
        expansion_returns[0].value
        if len(expansion_returns) == 1
        else None
    )
    product_calls = [] if product_urdf_value is None else [
        node
        for node in ast.walk(product_urdf_value)
        if isinstance(node, ast.Call)
    ]
    transformer_owns_identity = (
        isinstance(transform_return, ast.Call)
        and final_call_name(transform_return) == "transform_product_urdf"
        and len(transform_return.args) == 3
        and ast_equal(transform_return.args[0], expression("product_urdf"))
        and ast_equal(
            transform_return.args[1],
            expression("LEDGER_OWNER.name"),
        )
        and ast_equal(
            transform_return.args[2],
            expression("LEDGER_OWNER.nonce"),
        )
    )

    generate_assignments: dict[str, list[ast.expr]] = {}
    generate_returns: list[ast.Return] = []
    if generate is not None:
        for statement in generate.body:
            if (
                isinstance(statement, ast.Assign)
                and len(statement.targets) == 1
                and isinstance(statement.targets[0], ast.Name)
            ):
                generate_assignments.setdefault(
                    statement.targets[0].id,
                    [],
                ).append(statement.value)
            if isinstance(statement, ast.Return):
                generate_returns.append(statement)

    description_assignments = generate_assignments.get(
        "robot_description",
        [],
    )
    publisher_assignments = generate_assignments.get(
        "robot_state_publisher",
        [],
    )
    robot_description_value = (
        description_assignments[0]
        if len(description_assignments) == 1
        else None
    )
    robot_state_publisher_value = (
        publisher_assignments[0]
        if len(publisher_assignments) == 1
        else None
    )
    publisher_parameters = keyword_value(
        robot_state_publisher_value,
        "parameters",
    )
    description_parameter_values: list[ast.AST] = []
    if publisher_parameters is not None:
        for mapping in ast.walk(publisher_parameters):
            if not isinstance(mapping, ast.Dict):
                continue
            for key, value in zip(mapping.keys, mapping.values):
                if (
                    isinstance(key, ast.Constant)
                    and key.value == "robot_description"
                ):
                    description_parameter_values.append(value)

    publisher_reaches_launch = any(
        final_call_name(call) == "LaunchDescription"
        and call.args
        and isinstance(call.args[0], (ast.List, ast.Tuple))
        and any(
            ast_equal(element, expression("robot_state_publisher"))
            for element in call.args[0].elts
        )
        for return_statement in generate_returns
        for call in ast.walk(return_statement)
        if isinstance(call, ast.Call)
    )
    generation_reaches_expansion = (
        isinstance(robot_description_value, ast.Call)
        and final_call_name(robot_description_value)
        == "expanded_journaled_robot_description"
        and isinstance(robot_state_publisher_value, ast.Call)
        and final_call_name(robot_state_publisher_value) == "Node"
        and ast_equal(
            keyword_value(robot_state_publisher_value, "package"),
            expression("'robot_state_publisher'"),
        )
        and ast_equal(
            keyword_value(robot_state_publisher_value, "executable"),
            expression("'robot_state_publisher'"),
        )
        and len(description_parameter_values) == 1
        and ast_equal(
            description_parameter_values[0],
            expression("robot_description"),
        )
        and publisher_reaches_launch
    )

    canonical_transform_is_owned = (
        any(
            isinstance(call.func, ast.Attribute)
            and call.func.attr == "process_file"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "xacro"
            for call in product_calls
        )
        and expansion is not None
        and any(
            isinstance(node, ast.Constant)
            and node.value == "voice_nav_robot.urdf.xacro"
            for node in ast.walk(expansion)
        )
        and transformer_owns_identity
        and generation_reaches_expansion
    )
    if not canonical_transform_is_owned:
        raise CrashStopContractError(
            "runtime Adapter test must expand the canonical product Xacro "
            "and inject only its owned name-and-nonce hardware transform"
        )

    partition_is_unique = any(
        isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "TEST_PARTITION"
            for target in statement.targets
        )
        and isinstance(statement.value, ast.Call)
        and final_call_name(statement.value) == "claim_unique_test_partition"
        for statement in tree.body
    )
    setup = functions.get("setUp")
    cleanup_calls = [] if setup is None else [
        call
        for call in ast.walk(setup)
        if isinstance(call, ast.Call) and final_call_name(call) == "addCleanup"
    ]
    cleanup_is_scoped = any(
        call.args
        and isinstance(call.args[0], ast.Attribute)
        and call.args[0].attr == "structured_stop_gazebo"
        and any(
            keyword.arg == "expected_partition"
            and isinstance(keyword.value, ast.Name)
            and keyword.value.id == "TEST_PARTITION"
            for keyword in call.keywords
        )
        for call in cleanup_calls
    )
    broad_process_sweep = any(
        isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "os"
        and call.func.attr == "kill"
        or final_call_name(call) in {"pkill", "killall", "pgrep"}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
    )
    if not partition_is_unique or not cleanup_is_scoped or broad_process_sweep:
        raise CrashStopContractError(
            "runtime Adapter test must use a partition-scoped structured "
            "Gazebo stop without broad process discovery or signals"
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
    validate_runtime_adapter_test(paths["runtime_adapter_test"])
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
