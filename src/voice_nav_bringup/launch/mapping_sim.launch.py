# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Fixed-world Mapping Mode launch composition."""

import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


class MappingPreflightError(RuntimeError):
    """A required Mapping launch executable is unavailable."""


def _require_taskset(taskset_locator=None):
    locator = shutil.which if taskset_locator is None else taskset_locator
    executable = locator('taskset')
    if not executable:
        raise MappingPreflightError('Mapping Mode requires taskset from util-linux before launch')
    return executable


def _load_mode_lock_support():
    from importlib.util import module_from_spec, spec_from_file_location
    from pathlib import Path

    module_path = Path(__file__).resolve().parents[1] / 'launch_support' / 'mode_lock.py'
    specification = spec_from_file_location('voice_nav_mapping_mode_lock', module_path)
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load Mapping mode-lock support')
    module = module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _request_mode_shutdown(_context, *, release_gate):
    release_gate.request_shutdown()
    return []


def _observe_slam_process_exit(_context, *, release_gate):
    release_gate.observe_slam_process_exit()
    return []


def _observe_tf_owner_disappearance(_context, *, release_gate):
    release_gate.observe_tf_owner_disappearance()
    return []


def _start_mapping(_context, *, acquire_mode_lock, taskset_locator=None):
    """Admit Mapping only after its deployment preflight succeeds."""
    _require_taskset(taskset_locator)
    owner = acquire_mode_lock(mode='mapping')
    try:
        product = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('voice_nav_bringup'),
                        'launch',
                        'product_sim.launch.py',
                    ]
                )
            ),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'shutdown_on_gazebo_exit': 'true',
                'world_name': 'voice_nav_house_world',
                'laser_update_rate': '20',
            }.items(),
        )
        slam = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            prefix='taskset --cpu-list 0',
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare('voice_nav_bringup'),
                        'config',
                        'slam_toolbox_mapping.yaml',
                    ]
                ),
                {'use_lifecycle_manager': False, 'use_sim_time': True},
            ],
        )
        release_gate = _load_mode_lock_support().ModeLockShutdownGate(owner)
        configure_slam = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
        activate_slam = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            )
        )
        request_shutdown = RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(
                        function=_request_mode_shutdown,
                        kwargs={'release_gate': release_gate},
                    )
                ]
            )
        )
        release_after_slam_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=slam,
                on_exit=[
                    OpaqueFunction(
                        function=_observe_slam_process_exit,
                        kwargs={'release_gate': release_gate},
                    ),
                    OpaqueFunction(
                        function=_observe_tf_owner_disappearance,
                        kwargs={'release_gate': release_gate},
                    ),
                ],
            )
        )
        return [
            request_shutdown,
            release_after_slam_exit,
            product,
            slam,
            configure_slam,
            activate_slam,
        ]
    except BaseException:
        owner.close()
        raise


def build_mapping_launch(*, acquire_mode_lock, taskset_locator=None):
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description='Run Mapping Mode headless when true.',
                choices=['true', 'false'],
            ),
            OpaqueFunction(
                function=_start_mapping,
                kwargs={
                    'acquire_mode_lock': acquire_mode_lock,
                    'taskset_locator': taskset_locator,
                },
            ),
        ]
    )


def generate_launch_description():
    support = _load_mode_lock_support()
    return build_mapping_launch(acquire_mode_lock=support.acquire_mode_lock)
