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

#ifndef VOICE_NAV_AUDIO__DSP_PIPELINE_HPP_
#define VOICE_NAV_AUDIO__DSP_PIPELINE_HPP_

#include <array>
#include <cstdint>

#include "voice_nav_audio/audio_engine.hpp"

namespace voice_nav_audio
{

struct DspFrame
{
  std::array<Sample, AudioEngine::kFrameSamples> samples{};
};

enum class DspDiscontinuity
{
  kNone,
  kXrun,
  kRingOverflow,
  kUnrecoverable,
};

struct DspInput
{
  std::uint64_t generation{0U};
  std::uint64_t sequence{0U};
  double delay_ms{0.0};
  DspDiscontinuity discontinuity{DspDiscontinuity::kNone};
  DspFrame final_render_reference{};
  DspFrame capture{};
};

struct DspConfiguration
{
  std::uint32_t render_sample_rate_hz{48000U};
  std::uint32_t capture_sample_rate_hz{48000U};
  std::uint32_t output_sample_rate_hz{16000U};
  std::uint32_t channels{1U};
  std::uint32_t frame_samples{480U};
};

enum class DspStatus
{
  kCleaned,
  kInvalidConfiguration,
  kInvalidDelay,
  kDiscontinuity,
  kReorderedFrame,
  kAdapterFailure,
  kRejected,
};

struct DspResult
{
  DspStatus status{DspStatus::kRejected};
  std::array<Sample, 160U> cleaned{};
};

class DspAdapter
{
public:
  virtual ~DspAdapter() = default;

  virtual bool process_render(const DspFrame & frame) noexcept = 0;
  virtual bool set_stream_delay_ms(int milliseconds) noexcept = 0;
  virtual bool process_capture(DspFrame & frame) noexcept = 0;
  virtual void reset() noexcept = 0;
};

// Package-private deep module.  This is the sole seam for a production WebRTC
// Adapter and the deterministic fake used by offline fixtures.
class DspPipeline final
{
public:
  explicit DspPipeline(
    DspAdapter & adapter,
    DspConfiguration configuration = DspConfiguration{}) noexcept;

  [[nodiscard]] DspResult process(const DspInput & input) noexcept;

private:
  [[nodiscard]] DspResult reject(DspStatus status, bool quarantine_generation = false) noexcept;
  [[nodiscard]] static DspResult rejected(DspStatus status) noexcept;
  void reset_processing_state() noexcept;

  DspAdapter & adapter_;
  bool configuration_valid_{false};
  bool has_generation_{false};
  std::uint64_t generation_{0U};
  bool generation_quarantined_{false};
  std::uint64_t expected_sequence_{0U};
  std::size_t resample_phase_{0U};
  std::array<Sample, 6U> resample_history_{};
  bool resample_initialized_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__DSP_PIPELINE_HPP_
