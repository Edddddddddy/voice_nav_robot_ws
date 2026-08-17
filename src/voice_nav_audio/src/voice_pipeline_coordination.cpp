// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "voice_pipeline_coordination.hpp"

namespace voice_nav_audio
{

VoicePipelineCoordination::VoicePipelineCoordination(
  SpeechOutputControl & output, StopMissionPort & stop_port,
  VoiceTurnBoundary * const turn_boundary) noexcept
: output_(output), stop_port_(stop_port), turn_boundary_(turn_boundary)
{
}

bool VoicePipelineCoordination::on_wake_accepted() noexcept
{
  return output_.admit_ordinary_wake();
}

void VoicePipelineCoordination::before_turn_published(VoiceTurnPublication & turn) noexcept
{
  if (turn_boundary_ != nullptr) {
    // The boundary owns capture/playback ordering. It must complete before
    // this publication reaches the ROS graph.
    turn_boundary_->on_voice_turn_published();
  }
  if (turn.kind != VoiceTurnKind::kStop || turn.turn_id == last_stop_turn_id_) {
    return;
  }
  turn.during_playback = output_.interrupt_for_stop();
  last_stop_turn_id_ = turn.turn_id;
  stop_port_.request(
    StopMissionRequest{turn.turn_id, turn.voice_instance_id, turn.voice_seq, "voice_stop"}, *this);
}

void VoicePipelineCoordination::on_response(const StopMissionResponse & response) noexcept
{
  last_response_ = response;
  have_response_ = true;
}

}  // namespace voice_nav_audio
