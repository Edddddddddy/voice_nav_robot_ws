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
#include <cstdlib>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "dsp_pipeline.hpp"

namespace voice_nav_audio
{
namespace
{

class FakeDspAdapter final : public DspAdapter
{
public:
  enum class Call
  {
    kRender,
    kDelay,
    kCapture,
    kReset,
  };

  bool process_render(const DspFrame & frame) noexcept override
  {
    calls.push_back(Call::kRender);
    render = frame;
    return render_succeeds;
  }

  bool set_stream_delay_ms(const int milliseconds) noexcept override
  {
    calls.push_back(Call::kDelay);
    delay_ms = milliseconds;
    return delay_succeeds;
  }

  bool process_capture(DspFrame & frame) noexcept override
  {
    calls.push_back(Call::kCapture);
    capture = frame;
    return capture_succeeds;
  }

  void reset() noexcept override
  {
    calls.push_back(Call::kReset);
  }

  std::vector<Call> calls{};
  DspFrame render{};
  DspFrame capture{};
  int delay_ms{0};
  bool render_succeeds{true};
  bool delay_succeeds{true};
  bool capture_succeeds{true};
};

TEST(DspPipelineTest, SendsReferenceThenDelayThenCaptureAndProducesOneSixteenKilohertzFrame)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.generation = 7U;
  input.sequence = 1U;
  input.delay_ms = 40.0;
  input.final_render_reference.samples.fill(100);
  input.capture.samples.fill(900);

  const auto result = pipeline.process(input);

  ASSERT_EQ(result.status, DspStatus::kCleaned);
  EXPECT_EQ(result.cleaned.size(), 160U);
  EXPECT_TRUE(std::all_of(result.cleaned.begin(), result.cleaned.end(), [](const Sample sample) {
      return sample == 900;
    }));
  EXPECT_EQ(adapter.render.samples, input.final_render_reference.samples);
  EXPECT_EQ(adapter.capture.samples, input.capture.samples);
  EXPECT_EQ(adapter.delay_ms, 40);
  EXPECT_EQ(
    adapter.calls,
    (std::vector<FakeDspAdapter::Call>{
        FakeDspAdapter::Call::kRender,
        FakeDspAdapter::Call::kDelay,
        FakeDspAdapter::Call::kCapture}));
}

TEST(DspPipelineTest, RejectsOutOfRangeAndNonFiniteDelayWithoutPassingAudioToAdapter)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.generation = 7U;
  input.sequence = 1U;
  input.final_render_reference.samples.fill(100);
  input.capture.samples.fill(900);

  input.delay_ms = 40.0;
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
  adapter.calls.clear();

  input.sequence = 2U;
  input.delay_ms = 250.0;
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
  adapter.calls.clear();

  const std::array<double, 5U> invalid_delays{{
    39.0,
    251.0,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  }};
  for (const auto invalid_delay : invalid_delays) {
    adapter.calls.clear();
    input.delay_ms = invalid_delay;
    const auto result = pipeline.process(input);
    EXPECT_EQ(result.status, DspStatus::kInvalidDelay);
    EXPECT_TRUE(std::all_of(result.cleaned.begin(), result.cleaned.end(), [](const Sample sample) {
        return sample == 0;
      }));
    EXPECT_EQ(adapter.calls, (std::vector<FakeDspAdapter::Call>{FakeDspAdapter::Call::kReset}));
  }

  adapter.calls.clear();
  input.delay_ms = 100.0;
  const auto rejected_same_generation = pipeline.process(input);
  EXPECT_EQ(rejected_same_generation.status, DspStatus::kRejected);
  EXPECT_TRUE(std::all_of(
    rejected_same_generation.cleaned.begin(), rejected_same_generation.cleaned.end(),
      [](const Sample sample) {return sample == 0;}));
  EXPECT_TRUE(adapter.calls.empty());

  input.generation = 8U;
  input.sequence = 1U;
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
}

TEST(DspPipelineTest, RejectsAnInvalidAudioConfigurationFailClosed)
{
  DspConfiguration invalid_render_rate{};
  invalid_render_rate.render_sample_rate_hz = 44100U;
  DspConfiguration invalid_capture_rate{};
  invalid_capture_rate.capture_sample_rate_hz = 44100U;
  DspConfiguration invalid_output_rate{};
  invalid_output_rate.output_sample_rate_hz = 8000U;
  DspConfiguration invalid_channels{};
  invalid_channels.channels = 2U;
  DspConfiguration invalid_frame_samples{};
  invalid_frame_samples.frame_samples = 960U;
  const std::array<DspConfiguration, 5U> invalid_configurations{{
    invalid_render_rate,
    invalid_capture_rate,
    invalid_output_rate,
    invalid_channels,
    invalid_frame_samples,
  }};

  for (const auto & invalid : invalid_configurations) {
    FakeDspAdapter adapter;
    DspPipeline pipeline(adapter, invalid);
    DspInput input{};
    input.generation = 7U;
    input.sequence = 1U;
    input.delay_ms = 40.0;

    const auto result = pipeline.process(input);
    EXPECT_EQ(result.status, DspStatus::kInvalidConfiguration);
    EXPECT_TRUE(std::all_of(result.cleaned.begin(), result.cleaned.end(), [](const Sample sample) {
        return sample == 0;
      }));
    EXPECT_EQ(adapter.calls, (std::vector<FakeDspAdapter::Call>{FakeDspAdapter::Call::kReset}));
  }
}

TEST(DspPipelineTest, AdapterStageFailuresQuarantineTheirGenerationWithoutCallingLaterStages)
{
  struct StageFailure
  {
    FakeDspAdapter::Call failed_stage;
    std::vector<FakeDspAdapter::Call> expected_calls;
  };
  const std::array<StageFailure, 3U> failures{{
    {FakeDspAdapter::Call::kRender,
      {FakeDspAdapter::Call::kRender, FakeDspAdapter::Call::kReset}},
    {FakeDspAdapter::Call::kDelay,
      {FakeDspAdapter::Call::kRender, FakeDspAdapter::Call::kDelay, FakeDspAdapter::Call::kReset}},
    {FakeDspAdapter::Call::kCapture,
      {FakeDspAdapter::Call::kRender, FakeDspAdapter::Call::kDelay,
        FakeDspAdapter::Call::kCapture, FakeDspAdapter::Call::kReset}},
  }};

  for (const auto & failure : failures) {
    FakeDspAdapter adapter;
    if (failure.failed_stage == FakeDspAdapter::Call::kRender) {
      adapter.render_succeeds = false;
    } else if (failure.failed_stage == FakeDspAdapter::Call::kDelay) {
      adapter.delay_succeeds = false;
    } else {
      adapter.capture_succeeds = false;
    }
    DspPipeline pipeline(adapter);
    DspInput input{};
    input.generation = 30U;
    input.sequence = 1U;
    input.delay_ms = 100.0;
    input.final_render_reference.samples.fill(100);
    input.capture.samples.fill(500);

    const auto result = pipeline.process(input);

    EXPECT_EQ(result.status, DspStatus::kAdapterFailure);
    EXPECT_TRUE(std::all_of(result.cleaned.begin(), result.cleaned.end(), [](const Sample sample) {
        return sample == 0;
      }));
    EXPECT_EQ(adapter.calls, failure.expected_calls);

    adapter.render_succeeds = true;
    adapter.delay_succeeds = true;
    adapter.capture_succeeds = true;
    adapter.calls.clear();
    input.sequence = 2U;
    const auto same_generation = pipeline.process(input);
    EXPECT_EQ(same_generation.status, DspStatus::kRejected);
    EXPECT_TRUE(std::all_of(
      same_generation.cleaned.begin(), same_generation.cleaned.end(), [](const Sample sample) {
          return sample == 0;
      }));
    EXPECT_TRUE(adapter.calls.empty());

    input.generation = 31U;
    input.sequence = 1U;
    EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
  }
}

TEST(DspPipelineTest, ResetsProcessingStateForStrictlyNewerGenerationWithoutLeakingOldSamples)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.generation = 9U;
  input.sequence = 1U;
  input.delay_ms = 100.0;
  input.final_render_reference.samples.fill(100);
  input.capture.samples.fill(300);
  ASSERT_EQ(pipeline.process(input).status, DspStatus::kCleaned);

  adapter.calls.clear();
  input.generation = 10U;
  input.sequence = 1U;
  input.capture.samples.fill(700);
  const auto after_generation_change = pipeline.process(input);
  ASSERT_EQ(after_generation_change.status, DspStatus::kCleaned);
  EXPECT_TRUE(std::all_of(
    after_generation_change.cleaned.begin(), after_generation_change.cleaned.end(),
      [](const Sample sample) {return sample == 700;}));
  EXPECT_EQ(
    adapter.calls,
    (std::vector<FakeDspAdapter::Call>{
        FakeDspAdapter::Call::kReset,
        FakeDspAdapter::Call::kRender,
        FakeDspAdapter::Call::kDelay,
        FakeDspAdapter::Call::kCapture}));
}

TEST(DspPipelineTest, RejectsLateOlderGenerationWithoutCorruptingNewerSequenceFence)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.delay_ms = 100.0;
  input.final_render_reference.samples.fill(100);

  input.generation = 9U;
  input.sequence = 1U;
  input.capture.samples.fill(300);
  ASSERT_EQ(pipeline.process(input).status, DspStatus::kCleaned);

  input.generation = 10U;
  input.sequence = 1U;
  input.capture.samples.fill(700);
  ASSERT_EQ(pipeline.process(input).status, DspStatus::kCleaned);

  adapter.calls.clear();
  input.generation = 9U;
  input.sequence = 1U;
  input.capture.samples.fill(-600);
  const auto late = pipeline.process(input);
  EXPECT_EQ(late.status, DspStatus::kRejected);
  EXPECT_TRUE(std::all_of(late.cleaned.begin(), late.cleaned.end(), [](const Sample sample) {
      return sample == 0;
    }));
  EXPECT_TRUE(adapter.calls.empty());

  input.generation = 10U;
  input.sequence = 2U;
  input.capture.samples.fill(800);
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
}

TEST(DspPipelineTest, QuarantinesReorderedGenerationUntilAStrictlyNewerGenerationArrives)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.generation = 20U;
  input.sequence = 1U;
  input.delay_ms = 100.0;
  input.final_render_reference.samples.fill(100);
  input.capture.samples.fill(500);
  ASSERT_EQ(pipeline.process(input).status, DspStatus::kCleaned);

  adapter.calls.clear();
  input.sequence = 3U;
  const auto reordered = pipeline.process(input);
  EXPECT_EQ(reordered.status, DspStatus::kReorderedFrame);
  EXPECT_TRUE(std::all_of(reordered.cleaned.begin(), reordered.cleaned.end(),
    [](const Sample sample) {
      return sample == 0;
    }));
  EXPECT_EQ(adapter.calls, (std::vector<FakeDspAdapter::Call>{FakeDspAdapter::Call::kReset}));

  for (const auto sequence : std::array<std::uint64_t, 2U>{{2U, 4U}}) {
    adapter.calls.clear();
    input.sequence = sequence;
    const auto rejected = pipeline.process(input);
    EXPECT_EQ(rejected.status, DspStatus::kRejected);
    EXPECT_TRUE(std::all_of(rejected.cleaned.begin(), rejected.cleaned.end(),
      [](const Sample sample) {
        return sample == 0;
      }));
    EXPECT_TRUE(adapter.calls.empty());
  }

  input.generation = 21U;
  input.sequence = 1U;
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
}

TEST(DspPipelineTest, LowPassResamplingCarriesItsTailAcrossFramesAndClearsItOnReset)
{
  FakeDspAdapter adapter;
  DspPipeline pipeline(adapter);
  DspInput input{};
  input.generation = 11U;
  input.sequence = 1U;
  input.delay_ms = 100.0;
  input.capture.samples.fill(0);
  input.capture.samples.back() = 3000;

  const auto first = pipeline.process(input);
  ASSERT_EQ(first.status, DspStatus::kCleaned);
  EXPECT_EQ(first.cleaned.back(), 47);

  input.sequence = 2U;
  input.capture.samples.fill(0);
  const auto second = pipeline.process(input);
  ASSERT_EQ(second.status, DspStatus::kCleaned);
  EXPECT_EQ(second.cleaned[0], 938);
  EXPECT_EQ(second.cleaned[1], 47);

  input.sequence = 3U;
  input.discontinuity = DspDiscontinuity::kXrun;
  EXPECT_EQ(pipeline.process(input).status, DspStatus::kDiscontinuity);

  input.sequence = 4U;
  input.discontinuity = DspDiscontinuity::kNone;
  const auto rejected_same_generation = pipeline.process(input);
  EXPECT_EQ(rejected_same_generation.status, DspStatus::kRejected);
  EXPECT_TRUE(std::all_of(
    rejected_same_generation.cleaned.begin(), rejected_same_generation.cleaned.end(),
      [](const Sample sample) {return sample == 0;}));

  input.generation = 12U;
  input.sequence = 1U;
  const auto after_reset = pipeline.process(input);
  ASSERT_EQ(after_reset.status, DspStatus::kCleaned);
  EXPECT_TRUE(std::all_of(
    after_reset.cleaned.begin(), after_reset.cleaned.end(),
      [](const Sample sample) {return sample == 0;}));

  FakeDspAdapter high_frequency_adapter;
  DspPipeline high_frequency_pipeline(high_frequency_adapter);
  DspInput high_frequency{};
  high_frequency.generation = 12U;
  high_frequency.sequence = 1U;
  high_frequency.delay_ms = 100.0;
  for (std::size_t index = 0U; index < high_frequency.capture.samples.size(); ++index) {
    high_frequency.capture.samples[index] = index % 2U == 0U ? 3000 : -3000;
  }
  const auto low_passed = high_frequency_pipeline.process(high_frequency);
  ASSERT_EQ(low_passed.status, DspStatus::kCleaned);
  EXPECT_TRUE(std::all_of(
    low_passed.cleaned.begin() + 3, low_passed.cleaned.end(),
      [](const Sample sample) {return std::abs(static_cast<int>(sample)) <= 100;}));
}

TEST(DspPipelineTest, EveryDiscontinuityAxisQuarantinesItsGenerationUntilANewerGeneration)
{
  const std::array<DspDiscontinuity, 3U> discontinuities{{
    DspDiscontinuity::kXrun,
    DspDiscontinuity::kRingOverflow,
    DspDiscontinuity::kUnrecoverable,
  }};
  for (const auto discontinuity : discontinuities) {
    FakeDspAdapter adapter;
    DspPipeline pipeline(adapter);
    DspInput input{};
    input.generation = 20U;
    input.sequence = 1U;
    input.delay_ms = 100.0;
    input.final_render_reference.samples.fill(500);
    input.capture.samples.fill(500);
    ASSERT_EQ(pipeline.process(input).status, DspStatus::kCleaned);

    adapter.calls.clear();
    input.sequence = 2U;
    input.discontinuity = discontinuity;
    const auto discontinuous = pipeline.process(input);
    EXPECT_EQ(discontinuous.status, DspStatus::kDiscontinuity);
    EXPECT_TRUE(std::all_of(
      discontinuous.cleaned.begin(), discontinuous.cleaned.end(), [](const Sample sample) {
          return sample == 0;
      }));
    EXPECT_EQ(adapter.calls, (std::vector<FakeDspAdapter::Call>{FakeDspAdapter::Call::kReset}));

    input.discontinuity = DspDiscontinuity::kNone;
    for (const auto sequence : std::array<std::uint64_t, 2U>{{3U, 4U}}) {
      adapter.calls.clear();
      input.sequence = sequence;
      const auto rejected = pipeline.process(input);
      EXPECT_EQ(rejected.status, DspStatus::kRejected);
      EXPECT_TRUE(std::all_of(rejected.cleaned.begin(), rejected.cleaned.end(),
        [](const Sample sample) {
          return sample == 0;
        }));
      EXPECT_TRUE(adapter.calls.empty());
    }

    input.generation = 21U;
    input.sequence = 1U;
    EXPECT_EQ(pipeline.process(input).status, DspStatus::kCleaned);
  }
}

}  // namespace
}  // namespace voice_nav_audio
