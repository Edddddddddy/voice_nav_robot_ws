from builtin_interfaces.msg import Duration
from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import VoiceTurn


def test_voice_turn_exposes_bounded_ordered_contract():
    turn = VoiceTurn()

    assert isinstance(VoiceTurn.COMMAND, int)
    assert VoiceTurn.COMMAND == 1
    assert isinstance(VoiceTurn.STOP, int)
    assert VoiceTurn.STOP == 2
    assert list(turn.get_fields_and_field_types().items()) == [
        ('voice_instance_id', 'string<36>'),
        ('voice_seq', 'uint64'),
        ('session_id', 'string<36>'),
        ('turn_id', 'string<36>'),
        ('kind', 'uint8'),
        ('text', 'string<512>'),
        ('confidence', 'float'),
        ('during_playback', 'boolean'),
    ]


def test_speak_exposes_bounded_goal_result_feedback_contract():
    goal = Speak.Goal()
    result = Speak.Result()
    feedback = Speak.Feedback()

    assert isinstance(Speak.Goal.NORMAL, int)
    assert Speak.Goal.NORMAL == 1
    assert isinstance(Speak.Goal.URGENT, int)
    assert Speak.Goal.URGENT == 2
    assert isinstance(Speak.Result.COMPLETED, int)
    assert Speak.Result.COMPLETED == 0
    assert isinstance(Speak.Result.CANCELED, int)
    assert Speak.Result.CANCELED == 1
    assert isinstance(Speak.Result.BARGED_IN, int)
    assert Speak.Result.BARGED_IN == 2
    assert isinstance(Speak.Result.FAILED, int)
    assert Speak.Result.FAILED == 10

    goal.source_instance_id = 'voice-instance'
    goal.source_seq = 8
    goal.session_id = 'session'
    goal.turn_id = 'turn'
    goal.priority = Speak.Goal.NORMAL
    goal.text = '正在前往目标'
    goal.allow_barge_in = True
    result.code = Speak.Result.COMPLETED
    result.detail = 'completed'
    feedback.played = Duration(sec=3, nanosec=500000000)

    assert list(goal.get_fields_and_field_types().items()) == [
        ('source_instance_id', 'string<36>'),
        ('source_seq', 'uint64'),
        ('session_id', 'string<36>'),
        ('turn_id', 'string<36>'),
        ('priority', 'uint8'),
        ('text', 'string<512>'),
        ('allow_barge_in', 'boolean'),
    ]
    assert list(result.get_fields_and_field_types().items()) == [
        ('code', 'uint16'),
        ('detail', 'string<160>'),
    ]
    assert list(feedback.get_fields_and_field_types().items()) == [
        ('played', 'builtin_interfaces/Duration'),
    ]
    assert isinstance(feedback.played, Duration)
