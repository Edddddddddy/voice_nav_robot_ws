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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "dsp_pipeline.hpp"
#include "webrtc_apm_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

constexpr std::size_t kFixtureFrames = 500U;
constexpr std::size_t kConvergenceFrames = 200U;
constexpr double kErleThresholdDb = 6.0;

struct FarFixtureSpec
{
  const char * name;
  int delay_ms;
  int drift_ppm;
};

struct FarFixtureResult
{
  FarFixtureSpec spec;
  double erle_median_db{0.0};
  bool accepted{false};
};

enum class ReferenceMode
{
  kFinalRender,
  kMutedFault,
};

struct PresenceFixtureResult
{
  double input_energy{0.0};
  double output_energy{0.0};
  bool preserved{false};
};

struct ResetFixtureResult
{
  bool generation_change{false};
  bool xrun{false};
  bool ring_overflow{false};
  bool unrecoverable{false};
};

class IdentityAdapter final : public DspAdapter
{
public:
  bool process_render(const DspFrame &) noexcept override {return true;}
  bool set_stream_delay_ms(const int) noexcept override {return true;}
  bool process_capture(DspFrame &) noexcept override {return true;}
  void reset() noexcept override {}
};

Sample saturate(const double value)
{
  return static_cast<Sample>(std::clamp(
      std::lround(value),
      static_cast<long>(std::numeric_limits<Sample>::min()),
      static_cast<long>(std::numeric_limits<Sample>::max())));
}

std::uint64_t fixture_prng_word(std::uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

Sample far_end_integer_sample(const std::uint64_t sample_index)
{
  constexpr std::uint64_t kFixtureSeed = 0x55d5eeda4d55f001ULL;
  const auto word = fixture_prng_word(kFixtureSeed + sample_index);
  const auto centered = static_cast<std::int64_t>(word & 0xffffULL) - 32768LL;
  return saturate(10000.0 * static_cast<double>(centered) / 32768.0);
}

Sample far_end_sample(const double sample_index)
{
  if (sample_index < 0.0) {
    return 0;
  }
  const auto lower_index = static_cast<std::uint64_t>(std::floor(sample_index));
  const auto fractional = sample_index - static_cast<double>(lower_index);
  const auto lower = static_cast<double>(far_end_integer_sample(lower_index));
  const auto upper = static_cast<double>(far_end_integer_sample(lower_index + 1U));
  return saturate(lower + fractional * (upper - lower));
}

Sample near_end_sample(const double sample_index)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  const auto seconds = sample_index / static_cast<double>(AudioEngine::kSampleRate);
  return saturate(4500.0 * std::sin(kTwoPi * 660.0 * seconds));
}

double energy(const DspFrame & frame)
{
  double total = 0.0;
  for (const auto sample : frame.samples) {
    const auto normalized = static_cast<double>(sample);
    total += normalized * normalized;
  }
  return total / static_cast<double>(frame.samples.size());
}

double energy(const DspResult & result)
{
  double total = 0.0;
  for (const auto sample : result.cleaned) {
    const auto normalized = static_cast<double>(sample);
    total += normalized * normalized;
  }
  return total / static_cast<double>(result.cleaned.size());
}

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

template<typename SubjectAdapter>
FarFixtureResult run_far_end_only(
  const FarFixtureSpec spec, const int reported_delay_ms,
  const ReferenceMode reference_mode = ReferenceMode::kFinalRender)
{
  SubjectAdapter adapter;
  DspPipeline pipeline(adapter);
  IdentityAdapter pre_aec_adapter;
  DspPipeline pre_aec_pipeline(pre_aec_adapter);
  std::vector<double> erle_values;
  erle_values.reserve(kFixtureFrames - kConvergenceFrames);
  constexpr double kEchoGain = 0.70;
  const auto delay_samples = static_cast<double>(spec.delay_ms) * 48.0;

  for (std::size_t frame_index = 0U; frame_index < kFixtureFrames; ++frame_index) {
    DspInput input{};
    input.generation = 100U + static_cast<std::uint64_t>(spec.delay_ms);
    input.sequence = frame_index + 1U;
    input.delay_ms = static_cast<double>(reported_delay_ms);
    for (std::size_t sample_index = 0U; sample_index < AudioEngine::kFrameSamples; ++sample_index) {
      const auto absolute_sample = static_cast<double>(
        frame_index * AudioEngine::kFrameSamples + sample_index);
      input.final_render_reference.samples[sample_index] =
        reference_mode == ReferenceMode::kFinalRender ? far_end_sample(absolute_sample) : 0;
      const auto echo_sample = absolute_sample - delay_samples +
        absolute_sample * static_cast<double>(spec.drift_ppm) / 1000000.0;
      input.capture.samples[sample_index] = saturate(kEchoGain * far_end_sample(echo_sample));
    }

    const auto pre_aec_result = pre_aec_pipeline.process(input);
    const auto result = pipeline.process(input);
    if (pre_aec_result.status != DspStatus::kCleaned || result.status != DspStatus::kCleaned) {
      return FarFixtureResult{spec, 0.0, false};
    }
    if (frame_index >= kConvergenceFrames) {
      erle_values.push_back(
        10.0 * std::log10((energy(pre_aec_result) + 1.0) / (energy(result) + 1.0)));
    }
  }

  const auto result = median(erle_values);
  return FarFixtureResult{spec, result, result >= kErleThresholdDb};
}

PresenceFixtureResult run_near_or_double_talk(const bool double_talk)
{
  WebRtcApmAdapter adapter;
  DspPipeline pipeline(adapter);
  double input_energy = 0.0;
  double output_energy = 0.0;
  for (std::size_t frame_index = 0U; frame_index < kFixtureFrames; ++frame_index) {
    DspInput input{};
    input.generation = double_talk ? 401U : 400U;
    input.sequence = frame_index + 1U;
    input.delay_ms = 100.0;
    for (std::size_t sample_index = 0U; sample_index < AudioEngine::kFrameSamples; ++sample_index) {
      const auto absolute_sample = static_cast<double>(
        frame_index * AudioEngine::kFrameSamples + sample_index);
      input.final_render_reference.samples[sample_index] =
        double_talk ? far_end_sample(absolute_sample) : 0;
      const auto near = near_end_sample(absolute_sample);
      const auto echo = double_talk ? 0.70 * far_end_sample(absolute_sample - 4800.0) : 0.0;
      input.capture.samples[sample_index] = saturate(static_cast<double>(near) + echo);
    }
    const auto result = pipeline.process(input);
    if (result.status != DspStatus::kCleaned) {
      return PresenceFixtureResult{};
    }
    if (frame_index >= kConvergenceFrames) {
      input_energy += energy(input.capture);
      output_energy += energy(result);
    }
  }
  return PresenceFixtureResult{
    input_energy,
    output_energy,
    output_energy >= input_energy * 0.15,
  };
}

bool is_silent_frame_with_status(const DspResult & result, const DspStatus expected_status)
{
  return result.status == expected_status &&
         std::all_of(
    result.cleaned.begin(), result.cleaned.end(),
    [](const Sample sample) {return sample == 0;});
}

bool generation_reset()
{
  WebRtcApmAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput old_input{};
  old_input.generation = 500U;
  old_input.sequence = 1U;
  old_input.delay_ms = 100.0;
  old_input.final_render_reference.samples.fill(3000);
  old_input.capture.samples.fill(3000);
  if (pipeline.process(old_input).status != DspStatus::kCleaned) {
    return false;
  }

  DspInput new_input{};
  new_input.generation = 501U;
  new_input.sequence = 1U;
  new_input.delay_ms = 100.0;
  return is_silent_frame_with_status(pipeline.process(new_input), DspStatus::kCleaned);
}

bool discontinuity_reset(const DspDiscontinuity discontinuity)
{
  WebRtcApmAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput old_input{};
  old_input.generation = 600U;
  old_input.sequence = 1U;
  old_input.delay_ms = 100.0;
  old_input.final_render_reference.samples.fill(3000);
  old_input.capture.samples.fill(3000);
  if (pipeline.process(old_input).status != DspStatus::kCleaned) {
    return false;
  }

  old_input.sequence = 2U;
  old_input.discontinuity = discontinuity;
  if (!is_silent_frame_with_status(pipeline.process(old_input), DspStatus::kDiscontinuity)) {
    return false;
  }
  old_input.sequence = 3U;
  old_input.discontinuity = DspDiscontinuity::kNone;
  old_input.final_render_reference.samples.fill(0);
  old_input.capture.samples.fill(0);
  if (!is_silent_frame_with_status(pipeline.process(old_input), DspStatus::kRejected)) {
    return false;
  }

  old_input.generation = 601U;
  old_input.sequence = 1U;
  return is_silent_frame_with_status(pipeline.process(old_input), DspStatus::kCleaned);
}

ResetFixtureResult generation_and_discontinuity_reset()
{
  return ResetFixtureResult{
    generation_reset(),
    discontinuity_reset(DspDiscontinuity::kXrun),
    discontinuity_reset(DspDiscontinuity::kRingOverflow),
    discontinuity_reset(DspDiscontinuity::kUnrecoverable),
  };
}

bool write_report(
  const std::string & path,
  const std::array<FarFixtureResult, 5U> & far_results,
  const FarFixtureResult reference_fault_result,
  const FarFixtureResult identity_result,
  const PresenceFixtureResult near_result,
  const PresenceFixtureResult double_talk_result,
  const ResetFixtureResult reset_result)
{
  std::ofstream report(path, std::ios::trunc);
  if (!report.is_open()) {
    return false;
  }
  report << std::fixed << std::setprecision(3);
  report << "{\n";
  report << "  \"fixture\": \"deterministic-splitmix64-echo-v1\",\n";
  report << "  \"source\": {\"kind\": \"fixed-seed-wideband-prng\", "
    "\"seed\": \"0x55d5eeda4d55f001\", \"fractional_delay\": \"linear\"},\n";
  report << "  \"sample_rate_hz\": 48000,\n";
  report << "  \"frame_samples\": 480,\n";
  report << "  \"output_sample_rate_hz\": 16000,\n";
  report << "  \"frames\": 500,\n";
  report << "  \"excluded_convergence_frames\": 200,\n";
  report << "  \"far_end_only\": [\n";
  for (std::size_t index = 0U; index < far_results.size(); ++index) {
    const auto & result = far_results[index];
    report << "    {\"name\": \"" << result.spec.name << "\", \"delay_ms\": " <<
      result.spec.delay_ms << ", \"drift_ppm\": " << result.spec.drift_ppm <<
      ", \"erle_median_db\": " << result.erle_median_db <<
      ", \"threshold_db\": 6.000, \"accepted\": " <<
      (result.accepted ? "true" : "false") << "}";
    report << (index + 1U == far_results.size() ? "\n" : ",\n");
  }
  report << "  ],\n";
  report << "  \"reference_fault\": {\"actual_delay_ms\": " <<
    reference_fault_result.spec.delay_ms <<
    ", \"fault\": \"muted-final-render-reference\", \"erle_median_db\": " <<
    reference_fault_result.erle_median_db << ", \"rejected\": " <<
    (!reference_fault_result.accepted ? "true" : "false") << "},\n";
  report << "  \"identity_no_aec\": {\"erle_median_db\": " <<
    identity_result.erle_median_db << ", \"threshold_db\": 6.000, \"below_threshold\": " <<
    (!identity_result.accepted ? "true" : "false") << "},\n";
  report << "  \"near_end_only\": {\"erle_median_db\": null, \"input_energy\": " <<
    near_result.input_energy << ", \"output_energy\": " << near_result.output_energy <<
    ", \"preserved\": " << (near_result.preserved ? "true" : "false") << "},\n";
  report << "  \"double_talk\": {\"erle_median_db\": null, \"input_energy\": " <<
    double_talk_result.input_energy << ", \"output_energy\": " <<
    double_talk_result.output_energy <<
    ", \"preserved\": " << (double_talk_result.preserved ? "true" : "false") << "},\n";
  report << "  \"reset\": {\"generation_change\": " <<
    (reset_result.generation_change ? "true" : "false") <<
    ", \"xrun\": " << (reset_result.xrun ? "true" : "false") <<
    ", \"ring_overflow\": " <<
    (reset_result.ring_overflow ? "true" : "false") <<
    ", \"unrecoverable\": " << (reset_result.unrecoverable ? "true" : "false") << "}\n";
  report << "}\n";
  report.flush();
  return report.good();
}

}  // namespace
}  // namespace voice_nav_audio

int main(const int argc, char ** argv)
{
  if (argc != 2) {
    return EXIT_FAILURE;
  }

  const std::array<voice_nav_audio::FarFixtureSpec, 5U> specifications{{
    {"far-40ms", 40, 0},
    {"far-100ms", 100, 0},
    {"far-250ms", 250, 0},
    {"far-100ms-drift-minus-100ppm", 100, -100},
    {"far-100ms-drift-plus-100ppm", 100, 100},
  }};
  std::array<voice_nav_audio::FarFixtureResult, 5U> far_results{};
  bool far_accepted = true;
  for (std::size_t index = 0U; index < specifications.size(); ++index) {
    far_results[index] = voice_nav_audio::run_far_end_only<voice_nav_audio::WebRtcApmAdapter>(
      specifications[index], specifications[index].delay_ms);
    far_accepted = far_accepted && far_results[index].accepted;
  }
  const voice_nav_audio::FarFixtureSpec reference_fault_spec{"far-100ms-reference-fault", 100, 0};
  const auto reference_fault_result = voice_nav_audio::run_far_end_only<voice_nav_audio::WebRtcApmAdapter>(
    reference_fault_spec, 100, voice_nav_audio::ReferenceMode::kMutedFault);
  const auto reference_fault_rejected = !reference_fault_result.accepted;
  const auto identity_result =
    voice_nav_audio::run_far_end_only<voice_nav_audio::IdentityAdapter>(specifications[1], 100);
  const auto identity_below_threshold = !identity_result.accepted;
  const auto near_result = voice_nav_audio::run_near_or_double_talk(false);
  const auto double_talk_result = voice_nav_audio::run_near_or_double_talk(true);
  const auto reset_result = voice_nav_audio::generation_and_discontinuity_reset();
  const auto report_written = voice_nav_audio::write_report(
    argv[1], far_results, reference_fault_result, identity_result, near_result, double_talk_result,
    reset_result);

  const auto passed = far_accepted && near_result.preserved &&
    double_talk_result.preserved && reset_result.generation_change &&
    reset_result.xrun && reset_result.ring_overflow && reset_result.unrecoverable &&
    reference_fault_rejected &&
    identity_below_threshold && report_written;
  if (!report_written) {
    std::cerr << "failed to write dsp WebRTC fixture report: " << argv[1] << std::endl;
  }
  std::cout << "dsp WebRTC fixture report: " << argv[1] << std::endl;
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
