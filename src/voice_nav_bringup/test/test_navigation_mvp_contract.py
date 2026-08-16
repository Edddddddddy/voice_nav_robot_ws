# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Source contract for the one-place Navigation MVP composition."""

import importlib.util
from collections import deque
from pathlib import Path
import sys
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import JointState
import yaml


_TEST_DIRECTORY = str(Path(__file__).resolve().parent)
if _TEST_DIRECTORY not in sys.path:
    sys.path.insert(0, _TEST_DIRECTORY)

import crash_stop_support as support


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_launch_module():
    launch_path = _package_root() / 'launch' / 'navigation_mvp.launch.py'
    specification = importlib.util.spec_from_file_location(
        'navigation_mvp_launch', launch_path
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_navigation_mvp_contract_is_one_place_and_single_nav2_writer():
    package_root = _package_root()
    launch_source = (
        package_root / 'launch' / 'navigation_mvp.launch.py'
    ).read_text(encoding='utf-8')
    runtime = yaml.safe_load(
        (package_root / 'config' / 'mission_navigation.yaml').read_text(
            encoding='utf-8'
        )
    )
    nav2 = yaml.safe_load(
        (package_root / 'config' / 'nav2_navigation_mvp.yaml').read_text(
            encoding='utf-8'
        )
    )
    map_yaml = yaml.safe_load(
        (package_root / 'config' / 'voice_nav_study_map.yaml').read_text(
            encoding='utf-8'
        )
    )

    assert "SetRemap('/cmd_vel', '/voice_nav/nav2_cmd_vel')" in launch_source
    assert "'node_names': ['map_server', 'amcl']" in launch_source
    runtime_parameters = runtime['mission_runtime_node']['ros__parameters']
    assert runtime_parameters['operating_mode'] == 'navigation'
    assert runtime_parameters['named_place_ids'] == ['study']
    assert nav2['amcl']['ros__parameters']['base_frame_id'] == 'base_footprint'
    assert map_yaml['image'] == 'voice_nav_study_map.pgm'
    assert (package_root / 'config' / map_yaml['image']).is_file()

    description = _load_launch_module().generate_launch_description()
    assert description.entities


def test_navigation_observation_selects_last_nonzero_without_sleep():
    def command(linear_x: float) -> TwistStamped:
        message = TwistStamped()
        message.twist.linear.x = linear_x
        return message

    samples = (
        (90, command(0.4)),
        (101, command(0.2)),
        (102, command(0.0)),
        (103, command(0.3)),
    )

    selected = support.last_nonzero_command_after(samples, 100)

    assert selected is not None
    assert selected[0] == 103
    assert selected[1] is samples[-1][1]


def _stamped_odom(stamp_ns: int) -> Odometry:
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    return message


def _stamped_joint(stamp_ns: int) -> JointState:
    message = JointState()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    message.name = ['left_wheel_joint', 'right_wheel_joint']
    message.velocity = [0.0, 0.0]
    return message


def test_navigation_stationarity_rejects_stopped_joint_stream():
    probe = object.__new__(support.CrashStopProbe)
    probe.lock = threading.Lock()
    probe.odometry = deque(
        (
            (stamp_ns, _stamped_odom(stamp_ns))
            for stamp_ns in (1_000_000, 101_000_000, 201_000_000)
        )
    )
    probe.joint_states = deque(
        [(1_000_000, _stamped_joint(1_000_000))]
    )

    with pytest.raises(AssertionError, match='stationarity watchdog'):
        probe.wait_stationary(0, 0, timeout=0.01)


def test_navigation_stationarity_returns_joint_endpoint_evidence():
    probe = object.__new__(support.CrashStopProbe)
    probe.lock = threading.Lock()
    samples = (1_000_000, 101_000_000, 201_000_000)
    probe.odometry = deque(
        (stamp_ns, _stamped_odom(stamp_ns)) for stamp_ns in samples
    )
    probe.joint_states = deque(
        (stamp_ns + 1_000_000, _stamped_joint(stamp_ns))
        for stamp_ns in samples
    )

    evidence = probe.wait_stationary(0, 0, timeout=0.01)

    assert evidence['hold_ns'] >= support.STATIONARY_HOLD_NS
    assert evidence['odom_receipt_ns'] > 0
    assert evidence['odom_stamp_ns'] > 0
    assert evidence['joint_receipt_ns'] > 0
    assert evidence['joint_stamp_ns'] > 0
    assert abs(evidence['joint_left_velocity']) <= support.ZERO_WHEEL_TOLERANCE
    assert abs(evidence['joint_right_velocity']) <= support.ZERO_WHEEL_TOLERANCE
