"""Small checks for the transcript-level rapid wake gate."""

from voice_nav_agent.rapid_commands import WakeGate


def test_wake_gate_accepts_spaced_and_common_vosk_forms():
    """Common Vosk segmentation does not hide the rapid wake phrase."""
    gate = WakeGate()
    assert gate.accept(
        '\u5c0f \u667a\uff0c\u53bb\u53a8\u623f', 1.0
    ) == '\u53bb\u53a8\u623f'
    assert gate.accept('\u5c0f\u5fd7', 2.0) is None
    assert gate.accept('\u524d\u8fdb\u4e00\u7c73', 3.0) == '\u524d\u8fdb\u4e00\u7c73'


def test_wake_gate_still_expires():
    """An old wake phrase cannot authorize a later transcript."""
    gate = WakeGate(timeout_s=1.0)
    assert gate.accept('\u6653\u667a', 1.0) is None
    assert gate.accept('\u53bb\u53a8\u623f', 2.1) is None
