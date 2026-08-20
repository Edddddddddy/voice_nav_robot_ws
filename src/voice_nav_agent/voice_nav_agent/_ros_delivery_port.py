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

"""ROS Action/Service transport for :mod:`_agent_delivery_session`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional

from action_msgs.srv import CancelGoal

from voice_nav_interfaces.action import ExecuteMission, Speak
from voice_nav_interfaces.msg import MissionStep as MissionStepMessage
from voice_nav_interfaces.srv import StopMission

from ._agent_delivery_session import (
    MissionTerminal,
    SpeakRequest,
    StopRequest,
)


RESPONSE_DEADLINE_SECONDS = 1.0
SPEAK_DISCOVERY_RECHECK_SECONDS = 0.05


@dataclass(slots=True)
class _MissionIo:
    identity: object
    terminal: Callable[[MissionTerminal], None]
    goal_handle: Any = None
    send_timer: Any = None
    cancel_timer: Any = None
    cancel_requested: bool = False
    cancel_started: bool = False
    cancel_result: Optional[Callable[[bool], None]] = None


@dataclass(slots=True)
class _SpeakIo:
    identity: object
    request: SpeakRequest
    done: Callable[[], None]
    deadline: float = 0.0
    goal_handle: Any = None
    timer: Any = None
    send_started: bool = False
    cancel_started: bool = False


@dataclass(slots=True)
class _StopIo:
    identity: object
    done: Callable[[bool], None]
    timer: Any = None


class RosDeliveryPort:
    """Translate transport-neutral operations to bounded ROS I/O."""

    def __init__(
        self,
        *,
        mission_client: Any,
        stop_client: Any,
        speak_client: Any,
        timer_factory: Callable[[float, Callable[[], None]], Any],
        steady_time: Callable[[], float],
        invoke: Callable[..., Any],
        logger: Any,
    ) -> None:
        self._mission_client = mission_client
        self._stop_client = stop_client
        self._speak_client = speak_client
        self._timer_factory = timer_factory
        self._steady_time = steady_time
        self._invoke = invoke
        self._logger = logger
        self._missions: dict[object, _MissionIo] = {}
        self._speeches: dict[object, _SpeakIo] = {}
        self._stops: dict[object, _StopIo] = {}
        self._closed = False

    def submit_mission(
        self,
        identity: object,
        mission: object,
        callback: Callable[[MissionTerminal], None],
    ) -> bool:
        if self._closed:
            return False
        state = _MissionIo(identity, callback)
        self._missions[identity] = state
        try:
            future = self._mission_client.send_goal_async(
                self._mission_goal(mission)
            )
        except Exception as error:
            self._missions.pop(identity, None)
            self._logger.warning(
                f'Mission Goal transport failed before submission: {error}'
            )
            return False
        state.send_timer = self._timer_factory(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._mission_send_timeout(state),
        )
        future.add_done_callback(
            lambda completed: self._invoke(
                self._mission_goal_response, state, completed
            )
        )
        return True

    def cancel_mission(
        self, identity: object, callback: Callable[[bool], None]
    ) -> bool:
        state = self._missions.get(identity)
        if self._closed or state is None:
            return False
        state.cancel_requested = True
        state.cancel_result = callback
        if state.goal_handle is not None:
            self._request_mission_cancel(state)
        return True

    def submit_stop(
        self,
        identity: object,
        request: StopRequest,
        callback: Callable[[bool], None],
    ) -> bool:
        if self._closed or not _service_ready(self._stop_client):
            return False
        state = _StopIo(identity, callback)
        self._stops[identity] = state
        message = StopMission.Request()
        message.request_id = request.request_id
        message.source_instance_id = request.source_instance_id
        message.source_seq = request.source_seq
        message.reason = request.reason
        try:
            future = self._stop_client.call_async(message)
        except Exception as error:
            self._stops.pop(identity, None)
            self._logger.warning(f'STOP service call failed: {error}')
            return False
        state.timer = self._timer_factory(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._stop_timeout(state),
        )
        future.add_done_callback(
            lambda completed: self._invoke(
                self._stop_response, state, completed
            )
        )
        return True

    def submit_speak(
        self,
        identity: object,
        request: SpeakRequest,
        callback: Callable[[], None],
    ) -> bool:
        if self._closed:
            return False
        state = _SpeakIo(identity, request, callback)
        self._speeches[identity] = state
        if _action_server_ready(self._speak_client):
            self._send_speak(state)
        else:
            state.deadline = self._steady_time() + RESPONSE_DEADLINE_SECONDS
            self._schedule_speak_recheck(state)
        return True

    def retire(self, identity: object) -> None:
        mission = self._missions.pop(identity, None)
        if mission is not None:
            self._cancel_timer(mission.send_timer)
            self._cancel_timer(mission.cancel_timer)
            if mission.goal_handle is not None:
                self._best_effort_cancel(mission.goal_handle)
        speech = self._speeches.pop(identity, None)
        if speech is not None:
            self._cancel_timer(speech.timer)
            if speech.goal_handle is not None:
                self._best_effort_cancel_speak(speech)
        stop = self._stops.pop(identity, None)
        if stop is not None:
            self._cancel_timer(stop.timer)

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        identities = tuple(
            list(self._missions) + list(self._speeches) + list(self._stops)
        )
        for identity in identities:
            self.retire(identity)

    @staticmethod
    def _mission_goal(mission: Any) -> ExecuteMission.Goal:
        goal = ExecuteMission.Goal()
        goal.source_instance_id = mission.token.source_instance_id
        goal.source_seq = mission.token.source_seq
        goal.runtime_instance_id = mission.token.runtime_instance_id
        goal.admission_epoch = mission.token.admission_epoch
        for step in mission.steps:
            message = MissionStepMessage()
            message.kind = int(step.kind)
            message.distance_m = float(step.distance_m)
            message.angle_rad = float(step.angle_rad)
            message.target_id = step.target_id
            goal.steps.append(message)
        return goal

    def _mission_send_timeout(self, state: _MissionIo) -> None:
        if self._missions.pop(state.identity, None) is not state:
            return
        state.send_timer = None
        state.terminal(MissionTerminal.SUBMIT_TIMEOUT)

    def _mission_goal_response(self, state: _MissionIo, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._logger.warning(f'Mission Goal response failed: {error}')
            self._finish_mission(state, MissionTerminal.SUBMIT_FAILED)
            return
        if not getattr(handle, 'accepted', False):
            self._finish_mission(state, MissionTerminal.SUBMIT_FAILED)
            return
        if self._missions.get(state.identity) is not state:
            self._best_effort_cancel(handle)
            return
        self._cancel_timer(state.send_timer)
        state.send_timer = None
        state.goal_handle = handle
        if state.cancel_requested:
            self._request_mission_cancel(state)
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._invoke(
                    self._mission_result, state, completed
                )
            )
        except Exception as error:
            self._logger.warning(
                f'Mission result callback setup failed: {error}'
            )
            self._finish_mission(state, MissionTerminal.FAILED)

    def _request_mission_cancel(self, state: _MissionIo) -> None:
        if (
            self._missions.get(state.identity) is not state
            or state.goal_handle is None
            or state.cancel_started
        ):
            return
        state.cancel_started = True
        try:
            future = state.goal_handle.cancel_goal_async()
        except Exception as error:
            self._logger.warning(f'Mission cancel transport failed: {error}')
            self._finish_cancel(state, False)
            return
        state.cancel_timer = self._timer_factory(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._finish_cancel(state, False),
        )
        future.add_done_callback(
            lambda completed: self._invoke(
                self._mission_cancel_response, state, completed
            )
        )

    def _mission_cancel_response(self, state: _MissionIo, future: Any) -> None:
        if self._missions.get(state.identity) is not state:
            return
        try:
            confirmed = (
                int(future.result().return_code)
                == CancelGoal.Response.ERROR_NONE
            )
        except Exception:
            confirmed = False
        self._finish_cancel(state, confirmed)

    def _finish_cancel(self, state: _MissionIo, confirmed: bool) -> None:
        if self._missions.get(state.identity) is not state:
            return
        self._cancel_timer(state.cancel_timer)
        state.cancel_timer = None
        callback = state.cancel_result
        state.cancel_result = None
        if callback is not None:
            callback(confirmed)

    def _mission_result(self, state: _MissionIo, future: Any) -> None:
        try:
            code = int(future.result().result.code)
        except Exception as error:
            self._logger.warning(f'Mission result was malformed: {error}')
            code = ExecuteMission.Result.INTERNAL_ERROR
        terminal = {
            ExecuteMission.Result.SUCCEEDED: MissionTerminal.SUCCEEDED,
            ExecuteMission.Result.CANCELED: MissionTerminal.CANCELED,
            ExecuteMission.Result.STOPPED: MissionTerminal.STOPPED,
            ExecuteMission.Result.SAFETY_FAULT: MissionTerminal.SAFETY_FAULT,
        }.get(code, MissionTerminal.FAILED)
        self._finish_mission(state, terminal)

    def _finish_mission(
        self, state: _MissionIo, terminal: MissionTerminal
    ) -> None:
        if self._missions.pop(state.identity, None) is not state:
            return
        self._cancel_timer(state.send_timer)
        self._cancel_timer(state.cancel_timer)
        state.terminal(terminal)

    def _stop_timeout(self, state: _StopIo) -> None:
        if self._stops.pop(state.identity, None) is not state:
            return
        state.timer = None
        state.done(False)

    def _stop_response(self, state: _StopIo, future: Any) -> None:
        if self._stops.pop(state.identity, None) is not state:
            return
        self._cancel_timer(state.timer)
        try:
            response = future.result()
            confirmed = bool(response.motion_inhibited) and int(response.code) in (
                StopMission.Response.APPLIED,
                StopMission.Response.DUPLICATE,
            )
        except Exception:
            confirmed = False
        state.done(confirmed)

    def _schedule_speak_recheck(self, state: _SpeakIo) -> None:
        if self._speeches.get(state.identity) is not state:
            return
        remaining = state.deadline - self._steady_time()
        if remaining <= 0.0:
            self._finish_speak(state)
            return
        state.timer = self._timer_factory(
            min(SPEAK_DISCOVERY_RECHECK_SECONDS, remaining),
            lambda: self._speak_recheck(state),
        )

    def _speak_recheck(self, state: _SpeakIo) -> None:
        if self._speeches.get(state.identity) is not state:
            return
        state.timer = None
        if _action_server_ready(self._speak_client):
            self._send_speak(state)
        else:
            self._schedule_speak_recheck(state)

    def _send_speak(self, state: _SpeakIo) -> None:
        if (
            self._speeches.get(state.identity) is not state
            or state.send_started
        ):
            return
        state.send_started = True
        request = state.request
        goal = Speak.Goal()
        goal.source_instance_id = request.source_instance_id
        goal.source_seq = request.source_seq
        goal.session_id = request.session_id
        goal.turn_id = request.turn_id
        goal.priority = request.priority
        goal.text = request.text
        goal.allow_barge_in = True
        try:
            future = self._speak_client.send_goal_async(goal)
        except Exception as error:
            self._logger.warning(f'Speak Goal transport failed: {error}')
            self._finish_speak(state)
            return
        state.timer = self._timer_factory(
            RESPONSE_DEADLINE_SECONDS,
            lambda: self._finish_speak(state),
        )
        future.add_done_callback(
            lambda completed: self._invoke(
                self._speak_goal_response, state, completed
            )
        )

    def _speak_goal_response(self, state: _SpeakIo, future: Any) -> None:
        try:
            handle = future.result()
        except Exception:
            self._finish_speak(state)
            return
        if not getattr(handle, 'accepted', False):
            self._finish_speak(state)
            return
        if self._speeches.get(state.identity) is not state:
            self._best_effort_cancel(handle)
            return
        self._cancel_timer(state.timer)
        state.timer = None
        state.goal_handle = handle
        try:
            result = handle.get_result_async()
            result.add_done_callback(
                lambda completed: self._invoke(
                    self._speak_result, state, completed
                )
            )
        except Exception:
            self._finish_speak(state)

    def _speak_result(self, state: _SpeakIo, _future: Any) -> None:
        self._finish_speak(state)

    def _finish_speak(self, state: _SpeakIo) -> None:
        if self._speeches.pop(state.identity, None) is not state:
            return
        self._cancel_timer(state.timer)
        state.done()

    @staticmethod
    def _best_effort_cancel(handle: Any) -> None:
        try:
            handle.cancel_goal_async()
        except Exception:
            pass

    def _best_effort_cancel_speak(self, state: _SpeakIo) -> None:
        if state.cancel_started or state.goal_handle is None:
            return
        state.cancel_started = True
        self._best_effort_cancel(state.goal_handle)

    @staticmethod
    def _cancel_timer(timer: Any) -> None:
        if timer is None:
            return
        try:
            timer.cancel()
        except (AttributeError, RuntimeError):
            pass


def _action_server_ready(client: Any) -> bool:
    try:
        return bool(client.server_is_ready())
    except (AttributeError, RuntimeError):
        return False


def _service_ready(client: Any) -> bool:
    try:
        return bool(client.service_is_ready())
    except (AttributeError, RuntimeError):
        return False
