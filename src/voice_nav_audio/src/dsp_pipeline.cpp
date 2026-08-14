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

#include "dsp_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace voice_nav_audio
{
namespace
{

// This normalized seven-tap binomial FIR has a zero at Nyquist before the
// fixed 3:1 decimation.  The six-sample history is intentionally retained
// across 10 ms frames and is cleared by every fail-closed reset.
constexpr std::array<std::int64_t, 7U> kResampleTaps{{1, 6, 15, 20, 15, 6, 1}};
constexpr std::int64_t kResampleTapSum = 64;

Sample saturate_sample(const std::int64_t value) noexcept
{
  return static_cast<Sample>(std::max(
      static_cast<std::int64_t>(std::numeric_limits<Sample>::min()),
      std::min(value, static_cast<std::int64_t>(std::numeric_limits<Sample>::max()))));
}

}  // namespace

DspPipeline::DspPipeline(
  DspAdapter & adapter,
  const DspConfiguration configuration) noexcept
: adapter_(adapter),
  configuration_valid_(
    configuration.render_sample_rate_hz == AudioEngine::kSampleRate &&
    configuration.capture_sample_rate_hz == AudioEngine::kSampleRate &&
    configuration.output_sample_rate_hz == 16000U &&
    configuration.channels == AudioEngine::kChannels &&
    configuration.frame_samples == AudioEngine::kFrameSamples)
{
}

DspResult DspPipeline::process(const DspInput & input) noexcept
{
  if (!configuration_valid_) {
    return reject(DspStatus::kInvalidConfiguration);
  }
  if (has_generation_ && input.generation < generation_) {
    return rejected(DspStatus::kRejected);
  }
  if (!has_generation_ || input.generation > generation_) {
    if (has_generation_) {
      reset_processing_state();
    }
    has_generation_ = true;
    generation_ = input.generation;
    generation_quarantined_ = false;
    expected_sequence_ = input.sequence;
  }
  if (!std::isfinite(input.delay_ms) || input.delay_ms < 40.0 || input.delay_ms > 250.0) {
    return reject(DspStatus::kInvalidDelay, true);
  }
  if (generation_quarantined_) {
    return rejected(DspStatus::kRejected);
  }
  if (input.discontinuity != DspDiscontinuity::kNone) {
    return reject(DspStatus::kDiscontinuity, true);
  }
  if (input.sequence != expected_sequence_) {
    return reject(DspStatus::kReorderedFrame, true);
  }

  DspFrame cleaned = input.capture;
  if (!adapter_.process_render(input.final_render_reference) ||
    !adapter_.set_stream_delay_ms(static_cast<int>(std::lround(input.delay_ms))) ||
    !adapter_.process_capture(cleaned))
  {
    return reject(DspStatus::kAdapterFailure, true);
  }

  DspResult result{};
  result.status = DspStatus::kCleaned;
  if (!resample_initialized_) {
    resample_history_.fill(cleaned.samples.front());
    resample_initialized_ = true;
  }
  std::size_t output_index = 0U;
  for (const Sample sample : cleaned.samples) {
    std::int64_t filtered = static_cast<std::int64_t>(sample) * kResampleTaps[0U];
    for (std::size_t index = 0U; index < resample_history_.size(); ++index) {
      filtered += static_cast<std::int64_t>(resample_history_[index]) * kResampleTaps[index + 1U];
    }
    for (std::size_t index = resample_history_.size() - 1U; index > 0U; --index) {
      resample_history_[index] = resample_history_[index - 1U];
    }
    resample_history_.front() = sample;
    if (resample_phase_ == 2U) {
      const auto rounded = filtered >= 0 ?
        (filtered + kResampleTapSum / 2) / kResampleTapSum :
        (filtered - kResampleTapSum / 2) / kResampleTapSum;
      result.cleaned[output_index++] = saturate_sample(rounded);
    }
    resample_phase_ = (resample_phase_ + 1U) % 3U;
  }
  expected_sequence_ = input.sequence + 1U;
  return result;
}

DspResult DspPipeline::reject(const DspStatus status, const bool quarantine_generation) noexcept
{
  reset_processing_state();
  generation_quarantined_ = generation_quarantined_ || (quarantine_generation && has_generation_);
  return rejected(status);
}

DspResult DspPipeline::rejected(const DspStatus status) noexcept
{
  DspResult result{};
  result.status = status;
  return result;
}

void DspPipeline::reset_processing_state() noexcept
{
  adapter_.reset();
  expected_sequence_ = 0U;
  resample_phase_ = 0U;
  resample_history_.fill(0);
  resample_initialized_ = false;
}

}  // namespace voice_nav_audio
