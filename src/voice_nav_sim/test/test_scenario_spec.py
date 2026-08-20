"""Behavioral contract for the closed simulation scenario interface."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


def _load_spec_module():
    source_root = Path(__file__).resolve().parents[1] / 'python'
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    from voice_nav_sim import _scenario_spec

    return _scenario_spec


def test_motion_resolves_immutable_simulation_combination():
    spec = _load_spec_module().resolve_scenario('motion', 'headless')

    assert spec.mode == 'motion'
    assert spec.display == 'headless'
    assert spec.world_name == 'voice_nav_test_world'
    assert spec.laser_update_rate == '10'
    assert spec.controller_owner == '/diff_drive_controller'
    assert spec.map_odom_owner is None
    with pytest.raises(AttributeError):
        spec.world_name = 'other_world'


def test_resolver_exposes_only_closed_scenario_and_display_names():
    module = _load_spec_module()

    assert module.scenario_names() == ('motion', 'mapping', 'navigation')
    assert module.display_names() == ('headless', 'gui')
    with pytest.raises(module.ScenarioSpecError) as error:
        module.resolve_scenario('roundtrip', 'headless')
    assert error.value.code == 'invalid_scenario'


def test_invalid_display_fails_before_a_spec_can_be_created():
    module = _load_spec_module()

    with pytest.raises(module.ScenarioSpecError) as error:
        module.resolve_scenario('motion', 'browser')
    assert error.value.code == 'invalid_display'


def test_mapping_and_navigation_choose_distinct_single_map_odom_owners():
    module = _load_spec_module()

    mapping = module.resolve_scenario('mapping', 'headless')
    navigation = module.resolve_scenario('navigation', 'gui')

    assert (
        mapping.world_name
        == navigation.world_name
        == 'voice_nav_house_world'
    )
    assert mapping.laser_update_rate == navigation.laser_update_rate == '20'
    assert mapping.map_odom_owner == '/slam_toolbox'
    assert navigation.map_odom_owner == '/amcl'
    assert mapping.controller_owner == navigation.controller_owner
    assert mapping.map_odom_owner != navigation.map_odom_owner


def _package_root() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('voice_nav_sim'))
    except Exception:
        return Path(__file__).resolve().parents[1]


def _load_launch():
    launch_path = _package_root() / 'launch' / 'simulation.launch.py'
    specification = importlib.util.spec_from_file_location(
        'voice_nav_sim_installed_simulation_launch', launch_path
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _resolved_actions(mode: str, display: str = 'headless'):
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update({
        'scenario': mode,
        'headless': 'true' if display == 'headless' else 'false',
        'shutdown_on_gazebo_exit': 'true',
    })
    return _load_launch()._build_simulation_actions(context)


def _nodes(actions):
    from launch import LaunchContext
    from launch.actions import RegisterEventHandler
    from launch_ros.actions import Node

    resolved = list(actions)
    for action in actions:
        if isinstance(action, RegisterEventHandler):
            resolved.extend(
                action.event_handler.handle(
                    SimpleNamespace(returncode=0), LaunchContext()
                )
            )
    return [action for action in resolved if isinstance(action, Node)]


def test_each_installed_scenario_resolves_one_controller_and_bridge():
    module = _load_spec_module()
    for mode in module.scenario_names():
        spec = module.resolve_scenario(mode)
        nodes = _nodes(_resolved_actions(mode))
        by_executable = {
            (
                getattr(node, '_Node__package', None),
                getattr(node, '_Node__node_executable', None),
            ): node
            for node in nodes
        }
        assert ('ros_gz_bridge', 'parameter_bridge') in by_executable
        controller_nodes = [
            node
            for node in nodes
            if getattr(node, '_Node__package', None)
            == 'controller_manager'
            and getattr(node, '_Node__node_executable', None) == 'spawner'
            and spec.controller_owner.removeprefix('/') in str(
                getattr(node, '_Node__arguments', ())
            )
        ]
        assert len(controller_nodes) == 1


def test_resolved_spawn_uses_spec_world_before_robot_creation():
    module = _load_spec_module()
    for mode in module.scenario_names():
        spec = module.resolve_scenario(mode)
        spawn = next(
            node
            for node in _nodes(_resolved_actions(mode))
            if getattr(node, '_Node__package', None) == 'ros_gz_sim'
        )
        assert spec.world_name in str(
            getattr(spawn, '_Node__arguments', ())
        )


def test_invalid_scenario_fails_before_spawn_actions_are_returned():
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update({
        'scenario': 'roundtrip',
        'headless': 'true',
        'shutdown_on_gazebo_exit': 'true',
    })
    with pytest.raises(_load_spec_module().ScenarioSpecError):
        _load_launch()._build_simulation_actions(context)
