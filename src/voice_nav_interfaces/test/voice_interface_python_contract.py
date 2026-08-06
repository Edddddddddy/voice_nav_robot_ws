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
