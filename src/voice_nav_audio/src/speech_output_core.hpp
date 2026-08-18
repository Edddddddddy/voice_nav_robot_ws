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

#ifndef VOICE_NAV_AUDIO__SPEECH_OUTPUT_CORE_HPP_
#define VOICE_NAV_AUDIO__SPEECH_OUTPUT_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "voice_nav_audio/audio_engine.hpp"

namespace voice_nav_audio
{

enum class SpeechPriority : std::uint8_t
{
  Normal = 1U,
  Urgent = 2U
};

enum class SpeechResultCode : std::uint16_t
{
  Completed = 0U,
  Canceled = 1U,
  BargedIn = 2U,
  Failed = 10U
};

struct SpeechGoal
{
  std::string source_instance_id{};
  std::uint64_t source_seq{0U};
  std::string session_id{};
  std::string turn_id{};
  SpeechPriority priority{SpeechPriority::Normal};
  std::string text{};
  bool allow_barge_in{false};
};

struct TtsRequest
{
  std::uint64_t scope_id{0U};
  std::string text{};
};

class TtsSink
{
public:
  virtual ~TtsSink() = default;
  [[nodiscard]] virtual bool on_pcm(
    std::uint64_t scope_id, std::uint32_t sample_rate_hz, std::uint32_t channels,
    const Sample * samples, std::size_t sample_count) noexcept = 0;
  virtual void on_complete(std::uint64_t scope_id) noexcept = 0;
  virtual void on_failed(std::uint64_t scope_id, const std::string & detail) noexcept = 0;
};

class TtsAdapter
{
public:
  virtual ~TtsAdapter() = default;
  virtual void start(const TtsRequest & request, TtsSink & sink) noexcept = 0;
  virtual void cancel(std::uint64_t scope_id) noexcept = 0;
};

struct SpeechResult
{
  std::uint64_t scope_id{0U};
  SpeechResultCode code{SpeechResultCode::Failed};
  std::string detail{};
  std::uint64_t played_samples{0U};
};

class SpeechOutputObserver
{
public:
  virtual ~SpeechOutputObserver() = default;
  virtual void on_played(std::uint64_t scope_id, std::uint64_t samples) noexcept = 0;
  virtual void on_result(const SpeechResult & result) noexcept = 0;
};

// Package-private PlaybackScope control seam used by the VoicePipeline
// coordinator.  It exposes only wake admission and STOP fencing.
class SpeechOutputControl
{
public:
  virtual ~SpeechOutputControl() = default;

  [[nodiscard]] virtual bool admit_ordinary_wake() noexcept = 0;
  [[nodiscard]] virtual bool interrupt_for_stop() noexcept = 0;
};

struct SpeechAdmission
{
  std::uint64_t scope_id{0U};
  bool start_synthesis{false};
  bool waits_for_generation{false};
  SpeechResult immediate_result{};
  bool has_immediate_result{false};
};

// Package-private output module.  The ROS layer only maps Speak values to this
// stable contract; provider callbacks and raw PCM cannot escape this class.
class SpeechOutputCore final : public SpeechOutputControl, private TtsSink
{
public:
  SpeechOutputCore(AudioEngine & engine, TtsAdapter & tts, SpeechOutputObserver & observer);

  [[nodiscard]] SpeechAdmission start(const SpeechGoal & goal) noexcept;
  [[nodiscard]] bool begin_synthesis(std::uint64_t scope_id) noexcept;
  [[nodiscard]] bool cancel(std::uint64_t scope_id) noexcept;
  [[nodiscard]] bool interrupt_for_barge_in() noexcept;
  [[nodiscard]] bool interrupt_for_stop() noexcept override;
  [[nodiscard]] bool admit_ordinary_wake() noexcept override;
  [[nodiscard]] bool advance() noexcept;
  [[nodiscard]] std::uint64_t ready_scope_id() const noexcept;

private:
  struct Scope
  {
    std::uint64_t id{0U};
    SpeechPriority priority{SpeechPriority::Normal};
    std::string text{};
    std::uint64_t audio_generation{0U};
    std::uint64_t wait_generation{0U};
    std::uint64_t enqueued_samples{0U};
    std::uint64_t played_samples{0U};
    bool allow_barge_in{false};
    bool waiting_for_generation{false};
    bool synthesis_started{false};
    bool synthesis_completed{false};
    std::uint8_t synthesis_restart_count{0U};
  };

  [[nodiscard]] bool on_pcm(
    std::uint64_t scope_id, std::uint32_t sample_rate_hz, std::uint32_t channels,
    const Sample * samples, std::size_t sample_count) noexcept override;
  void on_complete(std::uint64_t scope_id) noexcept override;
  void on_failed(std::uint64_t scope_id, const std::string & detail) noexcept override;

  [[nodiscard]] bool valid(const SpeechGoal & goal) const noexcept;
  void retire(SpeechResultCode code, const std::string & detail) noexcept;
  void request_fence() noexcept;

  AudioEngine & engine_;
  TtsAdapter & tts_;
  SpeechOutputObserver & observer_;
  Scope active_{};
  std::uint64_t next_scope_id_{1U};
  std::uint64_t pending_fence_generation_{0U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SPEECH_OUTPUT_CORE_HPP_
