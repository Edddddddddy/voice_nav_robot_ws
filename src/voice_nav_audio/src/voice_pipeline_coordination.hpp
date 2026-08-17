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

#ifndef VOICE_NAV_AUDIO__VOICE_PIPELINE_COORDINATION_HPP_
#define VOICE_NAV_AUDIO__VOICE_PIPELINE_COORDINATION_HPP_

#include <cstdint>
#include <string>

#include "speech_input_core.hpp"
#include "speech_output_core.hpp"

namespace voice_nav_audio
{

struct StopMissionRequest
{
  std::string request_id{};
  std::string source_instance_id{};
  std::uint64_t source_seq{0U};
  std::string reason{};
};

enum class StopMissionCode : std::uint16_t
{
  Applied = 0U,
  Duplicate = 1U,
  SafetyFault = 2U,
  TransportFailure = 3U,
  Timeout = 4U,
};

struct StopMissionResponse
{
  StopMissionCode code{StopMissionCode::TransportFailure};
  bool motion_inhibited{false};
};

class StopMissionResponseSink
{
public:
  virtual ~StopMissionResponseSink() = default;

  virtual void on_response(const StopMissionResponse & response) noexcept = 0;
};

// Package-private typed seam for the existing /mission/stop service. The
// implementation is asynchronous; request() never waits or retries.
class StopMissionPort
{
public:
  virtual ~StopMissionPort() = default;

  virtual void request(
    const StopMissionRequest & request,
    StopMissionResponseSink & response_sink) noexcept = 0;
};

class VoiceTurnBoundary
{
public:
  virtual ~VoiceTurnBoundary() = default;

  virtual void on_voice_turn_published() noexcept = 0;
};

// Deep package-private coordinator: wake admission, PlaybackScope fencing,
// and the direct typed STOP request share one bounded Voice identity path.
class VoicePipelineCoordination final : public SpeechInputCoordination,
  private StopMissionResponseSink
{
public:
  VoicePipelineCoordination(
    SpeechOutputControl & output, StopMissionPort & stop_port,
    VoiceTurnBoundary * turn_boundary = nullptr) noexcept;

  [[nodiscard]] bool on_wake_accepted() noexcept override;
  void before_turn_published(VoiceTurnPublication & turn) noexcept override;

private:
  void on_response(const StopMissionResponse & response) noexcept override;

  SpeechOutputControl & output_;
  StopMissionPort & stop_port_;
  VoiceTurnBoundary * turn_boundary_{nullptr};
  StopMissionResponse last_response_{};
  bool have_response_{false};
  std::string last_stop_turn_id_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__VOICE_PIPELINE_COORDINATION_HPP_
