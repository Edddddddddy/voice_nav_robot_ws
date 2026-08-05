#include <gtest/gtest.h>

#include <voice_nav_interfaces/action/execute_mission.hpp>
#include <voice_nav_interfaces/msg/mission_state.hpp>
#include <voice_nav_interfaces/msg/mission_step.hpp>
#include <voice_nav_interfaces/srv/stop_mission.hpp>

namespace
{

using ExecuteMission = voice_nav_interfaces::action::ExecuteMission;
using MissionState = voice_nav_interfaces::msg::MissionState;
using MissionStep = voice_nav_interfaces::msg::MissionStep;
using StopMission = voice_nav_interfaces::srv::StopMission;

static_assert(MissionStep::MOVE_DISTANCE == 1);
static_assert(MissionStep::ROTATE_ANGLE == 2);
static_assert(MissionStep::NAVIGATE_TO == 3);
static_assert(MissionStep::SAVE_MAP == 4);
static_assert(ExecuteMission::Result::SUCCEEDED == 0);
static_assert(ExecuteMission::Result::STALE_REQUEST == 14);
static_assert(ExecuteMission::Result::UNSUPPORTED_STEP == 15);
static_assert(ExecuteMission::Result::SAFETY_FAULT == 32);
static_assert(StopMission::Response::APPLIED == 0);
static_assert(StopMission::Response::DUPLICATE == 1);
static_assert(StopMission::Response::SAFETY_FAULT == 2);

TEST(MissionInterfaceCppContract, ConstructsAllGeneratedV1Types)
{
  MissionStep step;
  step.kind = MissionStep::NAVIGATE_TO;
  step.target_id = "kitchen";

  ExecuteMission::Goal goal;
  goal.source_instance_id = "agent-instance";
  goal.source_seq = 7;
  goal.runtime_instance_id = "runtime-instance";
  goal.admission_epoch = 11;
  goal.steps.push_back(step);

  ExecuteMission::Result result;
  result.code = ExecuteMission::Result::SUCCEEDED;
  result.failed_step = -1;
  result.detail = "completed";

  ExecuteMission::Feedback feedback;
  feedback.phase = ExecuteMission::Feedback::EXECUTING;
  feedback.step_index = 0;
  feedback.progress = 0.5F;

  StopMission::Request request;
  request.request_id = "stop-request";
  request.source_instance_id = "agent-instance";
  request.source_seq = 8;
  request.reason = "operator requested stop";

  StopMission::Response response;
  response.code = StopMission::Response::APPLIED;
  response.runtime_instance_id = "runtime-instance";
  response.admission_epoch = 12;
  response.motion_inhibited = true;
  response.detail = "motion inhibited";

  MissionState state;
  state.runtime_instance_id = "runtime-instance";
  state.admission_epoch = 12;
  state.operating_mode = MissionState::NAVIGATION;
  state.availability = MissionState::AVAILABLE;
  state.gate_state = MissionState::GATE_INHIBITED;
  state.active_step = UINT32_MAX;
  state.supported_step_mask = 0x03;
  state.max_steps = 3;
  state.named_place_ids.push_back("kitchen");

  EXPECT_EQ(goal.steps.size(), 1U);
  EXPECT_EQ(goal.steps.front().target_id, "kitchen");
  EXPECT_EQ(result.failed_step, -1);
  EXPECT_TRUE(response.motion_inhibited);
  EXPECT_EQ(state.active_step, UINT32_MAX);
}

}  // namespace
