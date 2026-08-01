# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bounded Gazebo ground-truth pose snapshots for launch tests."""

from collections.abc import Callable, Mapping
import json
import math
import os
import shutil
import subprocess


QUERY_TIMEOUT_SECONDS = 10.0
QUERY_ATTEMPTS = 2


def _number(
    values: Mapping[str, object],
    key: str,
    *,
    default: float = 0.0,
) -> float:
    try:
        return float(values.get(key, default))
    except (TypeError, ValueError) as error:
        raise AssertionError(
            f'Gazebo pose field {key!r} is not numeric'
        ) from error


def _parse_model_pose(
    output: str,
    model_name: str,
) -> tuple[float, float, float, float, float, float]:
    try:
        payload = json.loads(output)
    except json.JSONDecodeError as error:
        raise AssertionError(
            f'cannot parse Gazebo pose JSON: {error}: {output}'
        ) from error
    if not isinstance(payload, dict) or not isinstance(
        payload.get('pose'),
        list,
    ):
        raise AssertionError('Gazebo pose snapshot has no pose list')

    poses = [
        pose
        for pose in payload['pose']
        if isinstance(pose, dict) and pose.get('name') == model_name
    ]
    if len(poses) != 1:
        raise AssertionError(
            f'expected one {model_name!r} pose in Gazebo snapshot, '
            f'found {len(poses)}'
        )

    position = poses[0].get('position', {})
    orientation = poses[0].get('orientation', {})
    if not isinstance(position, dict) or not isinstance(
        orientation,
        dict,
    ):
        raise AssertionError('Gazebo position and orientation must be maps')

    x = _number(orientation, 'x')
    y = _number(orientation, 'y')
    z = _number(orientation, 'z')
    w = _number(orientation, 'w')
    quaternion_norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(quaternion_norm) or quaternion_norm < 0.5:
        raise AssertionError('Gazebo pose quaternion is not finite and valid')

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    result = (
        _number(position, 'x'),
        _number(position, 'y'),
        _number(position, 'z'),
        roll,
        pitch,
        yaw,
    )
    if not all(math.isfinite(value) for value in result):
        raise AssertionError('Gazebo pose fields must be finite')
    return result


def read_model_pose(
    topic: str,
    model_name: str,
    *,
    expected_partition: str,
    environment: Mapping[str, str] | None = None,
    executable_lookup: Callable[[str], str | None] = shutil.which,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> tuple[float, float, float, float, float, float]:
    """Return one model pose from a bounded, retryable topic snapshot."""
    active_environment = dict(
        os.environ if environment is None else environment
    )
    actual_partition = active_environment.get('GZ_PARTITION', '')
    if not expected_partition or actual_partition != expected_partition:
        raise AssertionError(
            'Refusing Gazebo pose query outside the isolated test '
            f'partition: expected={expected_partition!r}, '
            f'actual={actual_partition!r}'
        )
    gz = executable_lookup('gz')
    if gz is None:
        raise AssertionError('gz executable is unavailable')
    arguments = [
        gz,
        'topic',
        '--echo',
        '--topic',
        topic,
        '--num',
        '1',
        '--json-output',
    ]

    last_timeout = None
    for _attempt in range(QUERY_ATTEMPTS):
        try:
            completed = runner(
                arguments,
                check=False,
                capture_output=True,
                text=True,
                timeout=QUERY_TIMEOUT_SECONDS,
                shell=False,
                env=active_environment,
            )
        except subprocess.TimeoutExpired as error:
            last_timeout = error
            continue
        break
    else:
        raise AssertionError(
            'Gazebo pose query timed out after '
            f'{QUERY_ATTEMPTS} attempts of '
            f'{QUERY_TIMEOUT_SECONDS:.1f} seconds'
        ) from last_timeout

    if completed.returncode != 0:
        raise AssertionError(
            'Gazebo pose query failed: '
            f'code={completed.returncode}, '
            f'stdout={completed.stdout!r}, '
            f'stderr={completed.stderr!r}'
        )
    return _parse_model_pose(completed.stdout, model_name)
