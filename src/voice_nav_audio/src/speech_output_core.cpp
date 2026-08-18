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

#include "speech_output_core.hpp"

#include <array>
#include <limits>

namespace voice_nav_audio
{
namespace
{

constexpr std::uint32_t kChaowenSampleRateHz = 22050U;
constexpr std::uint32_t kMono = 1U;
constexpr std::size_t kMaximumTtsChunkSamples = 220U;
constexpr std::size_t kFadeSamples = AudioEngine::kFrameSamples;
constexpr std::size_t kMaximumGenerationDetail = 192U;

std::string generation_change_detail(const AudioMetrics & metrics)
{
  std::string detail = "AudioEngine generation changed while restarting; reasons=";
  detail += "external=" + std::to_string(metrics.external_discontinuity_fences);
  detail += ",stream=" + std::to_string(metrics.stream_lifecycle_fences);
  detail += ",output=" + std::to_string(metrics.active_output_xrun_fences);
  detail += ",frame=" + std::to_string(metrics.frame_size_fences);
  detail += ",reference=" + std::to_string(metrics.reference_overflow_fences);
  detail += ",callback=" + std::to_string(metrics.callback_exception_fences);
  detail += ",playback=" + std::to_string(metrics.speech_playback_fence_requests);
  detail += ",ring=" + std::to_string(metrics.playback_ring_overflow_fences);
  detail += ",write=" + std::to_string(metrics.playback_write_overflow_fences);
  if (detail.size() > kMaximumGenerationDetail) {
    detail.resize(kMaximumGenerationDetail);
  }
  return detail;
}

}  // namespace

SpeechOutputCore::SpeechOutputCore(
  AudioEngine & engine, TtsAdapter & tts, SpeechOutputObserver & observer)
: engine_(engine), tts_(tts), observer_(observer)
{
}

SpeechAdmission SpeechOutputCore::start(const SpeechGoal & goal) noexcept
{
  SpeechAdmission admission{};
  admission.scope_id = next_scope_id_++;
  if (admission.scope_id == 0U) {
    admission.scope_id = next_scope_id_++;
  }
  if (!valid(goal)) {
    admission.has_immediate_result = true;
    admission.immediate_result = SpeechResult{
      admission.scope_id, SpeechResultCode::Failed, "invalid Speak goal", 0U};
    return admission;
  }

  if (active_.id != 0U) {
    if (goal.priority != SpeechPriority::Urgent || active_.priority != SpeechPriority::Normal) {
      admission.has_immediate_result = true;
      admission.immediate_result = SpeechResult{
        admission.scope_id, SpeechResultCode::Failed, "another PlaybackScope is active", 0U};
      return admission;
    }
    retire(SpeechResultCode::Canceled, "preempted by URGENT Speak goal");
    request_fence();
    active_ = Scope{};
    active_.id = admission.scope_id;
    active_.priority = goal.priority;
    active_.text = goal.text;
    active_.allow_barge_in = goal.allow_barge_in;
    active_.wait_generation = pending_fence_generation_;
    active_.waiting_for_generation = true;
    admission.waits_for_generation = true;
    return admission;
  }

  active_ = Scope{};
  active_.id = admission.scope_id;
  active_.priority = goal.priority;
  active_.text = goal.text;
  active_.allow_barge_in = goal.allow_barge_in;
  const auto generation = engine_.playback_generation();
  if (generation < pending_fence_generation_) {
    active_.wait_generation = pending_fence_generation_;
    active_.waiting_for_generation = true;
    admission.waits_for_generation = true;
  } else {
    pending_fence_generation_ = 0U;
    active_.audio_generation = generation;
    admission.start_synthesis = true;
  }
  return admission;
}

bool SpeechOutputCore::begin_synthesis(const std::uint64_t scope_id) noexcept
{
  if (active_.id != scope_id || active_.waiting_for_generation || active_.synthesis_started) {
    return false;
  }
  active_.synthesis_started = true;
  tts_.start(TtsRequest{scope_id, active_.text}, static_cast<TtsSink &>(*this));
  return true;
}

bool SpeechOutputCore::cancel(const std::uint64_t scope_id) noexcept
{
  if (active_.id != scope_id) {
    return false;
  }
  tts_.cancel(scope_id);
  retire(SpeechResultCode::Canceled, "Speak goal canceled");
  request_fence();
  return true;
}

bool SpeechOutputCore::interrupt_for_barge_in() noexcept
{
  if (active_.id == 0U || !active_.allow_barge_in) {
    return false;
  }
  tts_.cancel(active_.id);
  retire(SpeechResultCode::BargedIn, "interrupted by ordinary wake");
  request_fence();
  return true;
}

bool SpeechOutputCore::admit_ordinary_wake() noexcept
{
  return active_.id == 0U || interrupt_for_barge_in();
}

bool SpeechOutputCore::interrupt_for_stop() noexcept
{
  if (active_.id == 0U) {
    return false;
  }
  tts_.cancel(active_.id);
  retire(SpeechResultCode::BargedIn, "interrupted by voice STOP");
  request_fence();
  return true;
}

bool SpeechOutputCore::advance() noexcept
{
  PlaybackWrite write{};
  while (engine_.try_pop_playback_write(write)) {
    if (active_.id == write.scope_id && !active_.waiting_for_generation &&
      write.generation == active_.audio_generation)
    {
      active_.played_samples += write.sample_count;
      observer_.on_played(active_.id, active_.played_samples);
    }
  }

  if (active_.id == 0U) {
    return false;
  }
  const auto generation = engine_.playback_generation();
  if (active_.waiting_for_generation) {
    if (active_.synthesis_restart_count != 0U && generation > active_.wait_generation) {
      retire(SpeechResultCode::Failed, generation_change_detail(engine_.metrics()));
      return false;
    }
    if (generation < active_.wait_generation) {
      return false;
    }
    active_.waiting_for_generation = false;
    active_.audio_generation = generation;
    if (generation >= pending_fence_generation_) {
      pending_fence_generation_ = 0U;
    }
    return true;
  }
  if (generation != active_.audio_generation) {
    if (active_.synthesis_completed && active_.enqueued_samples > 0U &&
      active_.played_samples == 0U && active_.synthesis_restart_count == 0U)
    {
      active_.enqueued_samples = 0U;
      active_.synthesis_started = false;
      active_.synthesis_completed = false;
      active_.synthesis_restart_count = 1U;
      request_fence();
      active_.waiting_for_generation = true;
      active_.wait_generation = pending_fence_generation_;
      return false;
    }
    retire(SpeechResultCode::Failed, "AudioEngine generation changed");
    return false;
  }
  if (active_.synthesis_completed && active_.enqueued_samples > 0U &&
    active_.played_samples >= active_.enqueued_samples)
  {
    retire(SpeechResultCode::Completed, "completed");
  }
  return false;
}

std::uint64_t SpeechOutputCore::ready_scope_id() const noexcept
{
  return active_.id != 0U && !active_.waiting_for_generation &&
         !active_.synthesis_started ? active_.id : 0U;
}

bool SpeechOutputCore::on_pcm(
  const std::uint64_t scope_id, const std::uint32_t sample_rate_hz,
  const std::uint32_t channels, const Sample * const samples,
  const std::size_t sample_count) noexcept
{
  if (active_.id != scope_id || active_.waiting_for_generation || !active_.synthesis_started) {
    return false;
  }

  if (samples == nullptr || sample_rate_hz != kChaowenSampleRateHz || channels != kMono ||
    sample_count == 0U || sample_count > kMaximumTtsChunkSamples)
  {
    return false;
  }

  const auto generation = engine_.playback_generation();
  if (generation != active_.audio_generation) {
    if (active_.enqueued_samples != 0U || active_.synthesis_restart_count != 0U) {
      return false;
    }
    active_.audio_generation = generation;
  }

  std::array<Sample, AudioEngine::kFrameSamples> converted{};
  const auto converted_count = (sample_count * AudioEngine::kSampleRate) / kChaowenSampleRateHz;
  if (converted_count == 0U || converted_count > converted.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < converted_count; ++index) {
    const auto source_index = (index * kChaowenSampleRateHz) / AudioEngine::kSampleRate;
    converted[index] = samples[source_index];
  }
  if (!engine_.enqueue_playback(converted.data(), converted_count, scope_id)) {
    retire(SpeechResultCode::Failed, "AudioEngine playback ring rejected PCM");
    return false;
  }
  active_.enqueued_samples += converted_count;
  return true;
}

void SpeechOutputCore::on_complete(const std::uint64_t scope_id) noexcept
{
  if (active_.id == scope_id && active_.synthesis_started && !active_.waiting_for_generation) {
    active_.synthesis_completed = true;
  }
}

void SpeechOutputCore::on_failed(
  const std::uint64_t scope_id, const std::string & detail) noexcept
{
  if (active_.id == scope_id) {
    retire(SpeechResultCode::Failed, detail.empty() ? "TTS failed" : detail);
    request_fence();
  }
}

bool SpeechOutputCore::valid(const SpeechGoal & goal) const noexcept
{
  return !goal.source_instance_id.empty() && goal.source_instance_id.size() <= 36U &&
         goal.source_seq != 0U && !goal.session_id.empty() && goal.session_id.size() <= 36U &&
         !goal.turn_id.empty() && goal.turn_id.size() <= 36U && !goal.text.empty() &&
         goal.text.size() <= 512U && (goal.priority == SpeechPriority::Normal ||
         goal.priority == SpeechPriority::Urgent);
}

void SpeechOutputCore::retire(
  const SpeechResultCode code, const std::string & detail) noexcept
{
  if (active_.id == 0U) {
    return;
  }
  engine_.mark_playback_inactive();
  observer_.on_result(SpeechResult{
    active_.id, code, detail, active_.played_samples});
  active_ = Scope{};
}

void SpeechOutputCore::request_fence() noexcept
{
  const auto requested_generation = engine_.playback_generation() + 1U;
  if (requested_generation > pending_fence_generation_) {
    pending_fence_generation_ = requested_generation;
  }
  engine_.request_fade_to_silence(kFadeSamples);
  engine_.request_playback_fence(PlaybackFenceReason::kSpeechScope);
}

}  // namespace voice_nav_audio
