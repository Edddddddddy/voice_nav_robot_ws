"""Offline ROS-message adaptation checks for the rapid web console."""

import threading

from voice_nav_agent.rapid_web_console_node import RapidWebConsole

from voice_nav_interfaces.msg import VoiceTurn


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Future:
    def add_done_callback(self, callback):
        self.callback = callback


class _StopClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return _Future()


def _adapter(stop_ready=True):
    node = object.__new__(RapidWebConsole)
    node.instance_id = 'web-source'
    node.sequence = 0
    node.lock = threading.Lock()
    node.turns = _Publisher()
    node.stop = _StopClient(stop_ready)
    node.state_data = {'last_event': ''}
    return node


def test_web_command_uses_monotonic_voice_identity():
    """Web commands enter the existing Agent seam as ordered VoiceTurns."""
    node = _adapter()
    first = node.submit_command('前进 0.5 米')
    second = node.submit_command('去 kitchen')
    assert first['accepted'] and second['accepted']
    assert [message.voice_seq for message in node.turns.messages] == [1, 2]
    assert all(
        message.kind == VoiceTurn.COMMAND for message in node.turns.messages
    )
    assert node.turns.messages[1].text == '去 kitchen'


def test_web_stop_reuses_exact_identity_for_agent_retry():
    """Direct StopMission and the STOP VoiceTurn share one fingerprint."""
    node = _adapter()
    result = node.request_stop()
    assert result['direct_stop'] is True
    assert len(node.stop.requests) == 1
    assert len(node.turns.messages) == 1
    request = node.stop.requests[0]
    turn = node.turns.messages[0]
    assert turn.kind == VoiceTurn.STOP
    assert request.request_id == turn.turn_id
    assert request.source_instance_id == turn.voice_instance_id
    assert request.source_seq == turn.voice_seq
    assert request.reason == 'web_stop'


def test_web_stop_still_publishes_when_runtime_is_late():
    """Agent retry remains available before the direct service is ready."""
    node = _adapter(stop_ready=False)
    result = node.request_stop()
    assert result['direct_stop'] is False
    assert node.stop.requests == []
    assert node.turns.messages[0].kind == VoiceTurn.STOP
