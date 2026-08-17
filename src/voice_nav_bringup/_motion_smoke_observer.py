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

"""Thin runtime observer reusing the canonical typed session ledger."""

from __future__ import annotations

from pathlib import Path
import runpy
import threading


def _canonical_namespace() -> dict[str, object]:
    installed = Path(__file__).with_name('_canonical_session_driver.py')
    if installed.is_file():
        return runpy.run_path(str(installed))
    source = Path(__file__).parent / 'test' / 'canonical_session_driver.py'
    return runpy.run_path(str(source))


class MotionSmokeObserver:
    """Observe only existing typed ROS endpoints; never submit a goal."""

    def __init__(self, node) -> None:
        canonical = _canonical_namespace()
        self._ledger = canonical['TypedObservationLedger']()
        self._voice_turn = canonical['VoiceTurn']
        self._lock = threading.RLock()
        self._subscriptions = [
            node.create_subscription(
                canonical['MissionState'], '/mission/state',
                self._on_mission_state, canonical['_state_qos'](),
            ),
            node.create_subscription(
                canonical['TwistStamped'], '/diff_drive_controller/cmd_vel',
                self._on_controller_command, canonical['_reliable_qos'](),
            ),
            node.create_subscription(
                canonical['Odometry'], '/odom',
                self._on_odometry, canonical['qos_profile_sensor_data'],
            ),
            node.create_subscription(
                canonical['VoiceTurn'], '/voice/turn',
                self._on_voice_turn, canonical['_reliable_qos'](),
            ),
            node.create_subscription(
                canonical['GoalStatusArray'], '/mission/execute/_action/status',
                self._on_mission_status, canonical['_reliable_qos'](),
            ),
            node.create_subscription(
                canonical['GoalStatusArray'], '/voice/speak/_action/status',
                self._on_speak_status, canonical['_reliable_qos'](),
            ),
        ]

    def _on_mission_state(self, message) -> None:
        with self._lock:
            self._ledger.on_mission_state(message)

    def _on_controller_command(self, message) -> None:
        with self._lock:
            self._ledger.on_controller_command(message)

    def _on_odometry(self, message) -> None:
        with self._lock:
            self._ledger.on_odometry(message)

    def _on_voice_turn(self, message) -> None:
        with self._lock:
            self._ledger.on_voice_turn(message)

    def _on_mission_status(self, message) -> None:
        with self._lock:
            self._ledger.on_mission_status(message)

    def _on_speak_status(self, message) -> None:
        with self._lock:
            self._ledger.on_speak_status(message)

    @staticmethod
    def _stationary_hold_ms(events, motion_ns: int | None) -> int:
        if motion_ns is None:
            return 0
        stationary_since: int | None = None
        last_ns: int | None = None
        for received_ns, _message, stationary in events:
            if received_ns < motion_ns:
                continue
            last_ns = received_ns
            if stationary:
                if stationary_since is None:
                    stationary_since = received_ns
            else:
                stationary_since = None
        if stationary_since is None or last_ns is None:
            return 0
        return int((last_ns - stationary_since) // 1_000_000)

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            latest_safety_sample = self._ledger.latest_safety_sample
            if latest_safety_sample is None:
                return {'observer_error': 'latest_safety_sample_unavailable'}
            turns = self._ledger.voice_turn_events
            commands = self._ledger.command_events
            mission_states = self._ledger.mission_status_events
            odometry = self._ledger.odometry_events
            nonzero_times = [
                received_ns
                for received_ns, _message, is_zero in commands
                if not is_zero
            ]
            speak_goal_ids = {
                goal_id for _received_ns, goal_id in self._ledger.speak_success_events
            }
            return {
                'transcript': turns[0][1].text if turns else '',
                'voice_turn_count': len(turns),
                'command_count': sum(
                    message.kind == self._voice_turn.COMMAND
                    for _received_ns, message in turns
                ),
                'stop_turn_count': sum(
                    message.kind == self._voice_turn.STOP
                    for _received_ns, message in turns
                ),
                'mission_count': len(self._ledger.mission_goal_ids),
                'mission_success_count': len(self._ledger.successful_mission_ids),
                'mission_status_event_count': len(mission_states),
                # The fixed Mandarin command is admitted by AgentCore's
                # deterministic parser; no LLM response path is submitted.
                'llm_calls': 0,
                'speak_status_completed_count': len(speak_goal_ids),
                'controller_nonzero': self._ledger.controller_nonzero_observed,
                'final_zero': self._ledger.latest_controller_zero,
                'final_gate_inhibited': latest_safety_sample.mission_gate_inhibited,
                'final_stationary': self._ledger.latest_odom_stationary,
                'stationary_ms': self._stationary_hold_ms(
                    odometry, max(nonzero_times) if nonzero_times else None,
                ),
            }

    def close(self) -> None:
        """Release observer subscriptions before the ROS context is closed."""
        for subscription in self._subscriptions:
            try:
                subscription.destroy()
            except AttributeError:
                pass
        self._subscriptions.clear()
