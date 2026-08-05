from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState
from voice_nav_interfaces.msg import MissionStep
from voice_nav_interfaces.srv import StopMission


def test_mission_step_exposes_bounded_v1_contract():
    step = MissionStep()

    assert MissionStep.MOVE_DISTANCE == 1
    assert MissionStep.ROTATE_ANGLE == 2
    assert MissionStep.NAVIGATE_TO == 3
    assert MissionStep.SAVE_MAP == 4
    assert list(step.get_fields_and_field_types()) == [
        'kind',
        'distance_m',
        'angle_rad',
        'target_id',
    ]
    assert step.get_fields_and_field_types()['target_id'] == 'string<64>'


def test_execute_mission_exposes_bounded_fenced_contract():
    goal = ExecuteMission.Goal()
    result = ExecuteMission.Result()
    feedback = ExecuteMission.Feedback()

    assert ExecuteMission.Result.SUCCEEDED == 0
    assert ExecuteMission.Result.INVALID_PLAN == 10
    assert ExecuteMission.Result.BUSY == 11
    assert ExecuteMission.Result.MODE_MISMATCH == 12
    assert ExecuteMission.Result.UNKNOWN_TARGET == 13
    assert ExecuteMission.Result.STALE_REQUEST == 14
    assert ExecuteMission.Result.UNSUPPORTED_STEP == 15
    assert ExecuteMission.Result.DEPENDENCY_UNAVAILABLE == 20
    assert ExecuteMission.Result.EXECUTION_FAILED == 21
    assert ExecuteMission.Result.TIMEOUT == 22
    assert ExecuteMission.Result.CANCELED == 30
    assert ExecuteMission.Result.STOPPED == 31
    assert ExecuteMission.Result.SAFETY_FAULT == 32
    assert ExecuteMission.Result.INTERNAL_ERROR == 99
    assert ExecuteMission.Feedback.VALIDATING == 1
    assert ExecuteMission.Feedback.EXECUTING == 2
    assert ExecuteMission.Feedback.SAFE_STOPPING == 3

    assert list(goal.get_fields_and_field_types()) == [
        'source_instance_id',
        'source_seq',
        'runtime_instance_id',
        'admission_epoch',
        'steps',
    ]
    assert goal.get_fields_and_field_types() == {
        'source_instance_id': 'string<36>',
        'source_seq': 'uint64',
        'runtime_instance_id': 'string<36>',
        'admission_epoch': 'uint64',
        'steps': 'sequence<voice_nav_interfaces/MissionStep, 3>',
    }
    assert result.get_fields_and_field_types() == {
        'code': 'uint16',
        'failed_step': 'int32',
        'detail': 'string<160>',
    }
    assert feedback.get_fields_and_field_types() == {
        'phase': 'uint8',
        'step_index': 'uint32',
        'progress': 'float',
    }


def test_state_and_stop_expose_bounded_runtime_contract():
    state = MissionState()
    request = StopMission.Request()
    response = StopMission.Response()

    assert MissionState.MAPPING == 1
    assert MissionState.NAVIGATION == 2
    assert MissionState.UNAVAILABLE == 0
    assert MissionState.AVAILABLE == 1
    assert MissionState.BUSY == 2
    assert MissionState.FAULTED == 3
    assert MissionState.GATE_INHIBITED == 0
    assert MissionState.GATE_ARMED == 1
    assert MissionState.GATE_FAULTED == 2
    assert StopMission.Response.APPLIED == 0
    assert StopMission.Response.DUPLICATE == 1
    assert StopMission.Response.SAFETY_FAULT == 2

    assert state.get_fields_and_field_types() == {
        'runtime_instance_id': 'string<36>',
        'admission_epoch': 'uint64',
        'operating_mode': 'uint8',
        'availability': 'uint8',
        'gate_state': 'uint8',
        'active_step': 'uint32',
        'supported_step_mask': 'uint32',
        'max_steps': 'uint8',
        'named_place_ids': 'sequence<string<64>, 32>',
    }
    assert request.get_fields_and_field_types() == {
        'request_id': 'string<36>',
        'source_instance_id': 'string<36>',
        'source_seq': 'uint64',
        'reason': 'string<160>',
    }
    assert response.get_fields_and_field_types() == {
        'code': 'uint16',
        'runtime_instance_id': 'string<36>',
        'admission_epoch': 'uint64',
        'motion_inhibited': 'boolean',
        'detail': 'string<160>',
    }
