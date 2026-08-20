"""
The closed, immutable simulation composition selected by a mode.

Launch files and installed entrypoints may ask this module for a spec, but
they cannot provide arbitrary world, controller, bridge, or TF-owner values.
The concrete table is deliberately kept private; the resolver is the only
selection seam used by product composition.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Final


_SCENARIO_NAMES: Final[tuple[str, ...]] = (
    'motion',
    'mapping',
    'navigation',
)
_DISPLAY_NAMES: Final[tuple[str, ...]] = ('headless', 'gui')


class ScenarioSpecError(ValueError):
    """A structured rejection before any simulation process is spawned."""

    def __init__(
        self,
        code: str,
        field: str,
        value: object,
        allowed: tuple[str, ...],
    ) -> None:
        self.code = code
        self.field = field
        self.value = value
        self.allowed = allowed
        allowed_text = ','.join(allowed)
        super().__init__(
            f'{code}: {field}={value!r}; allowed={allowed_text}'
        )


@dataclass(frozen=True, slots=True)
class ScenarioSpec:
    """
    One closed simulation composition.

    Instances are created only by :func:`resolve_scenario`; frozen slots keep
    the resolved assets and ownership contract immutable after validation.
    """

    mode: str
    display: str
    world_name: str
    laser_update_rate: str
    controller_owner: str
    map_odom_owner: str | None


@dataclass(frozen=True, slots=True)
class _ScenarioDefinition:
    world_name: str
    laser_update_rate: str
    controller_owner: str
    map_odom_owner: str | None


_DEFINITIONS: Final = MappingProxyType({
    'motion': _ScenarioDefinition(
        world_name='voice_nav_test_world',
        laser_update_rate='10',
        controller_owner='/diff_drive_controller',
        map_odom_owner=None,
    ),
    'mapping': _ScenarioDefinition(
        world_name='voice_nav_house_world',
        laser_update_rate='20',
        controller_owner='/diff_drive_controller',
        map_odom_owner='/slam_toolbox',
    ),
    'navigation': _ScenarioDefinition(
        world_name='voice_nav_house_world',
        laser_update_rate='20',
        controller_owner='/diff_drive_controller',
        map_odom_owner='/amcl',
    ),
})


def scenario_names() -> tuple[str, ...]:
    """Return the only scenarios accepted by the installed interface."""
    return _SCENARIO_NAMES


def display_names() -> tuple[str, ...]:
    """Return the only display selections accepted by the interface."""
    return _DISPLAY_NAMES


def display_from_headless(value: str) -> str:
    """Translate the launch boolean into the closed display name."""
    if value == 'true':
        return 'headless'
    if value == 'false':
        return 'gui'
    raise ScenarioSpecError(
        'invalid_display',
        'headless',
        value,
        ('true', 'false'),
    )


def resolve_scenario(mode: str, display: str = 'headless') -> ScenarioSpec:
    """Resolve one immutable spec or reject it before process creation."""
    if mode not in _DEFINITIONS:
        raise ScenarioSpecError(
            'invalid_scenario', 'scenario', mode, _SCENARIO_NAMES
        )
    if display not in _DISPLAY_NAMES:
        raise ScenarioSpecError(
            'invalid_display', 'display', display, _DISPLAY_NAMES
        )
    definition = _DEFINITIONS[mode]
    return ScenarioSpec(
        mode=mode,
        display=display,
        world_name=definition.world_name,
        laser_update_rate=definition.laser_update_rate,
        controller_owner=definition.controller_owner,
        map_odom_owner=definition.map_odom_owner,
    )


__all__ = [
    'ScenarioSpecError',
    'display_from_headless',
    'display_names',
    'resolve_scenario',
    'scenario_names',
]
