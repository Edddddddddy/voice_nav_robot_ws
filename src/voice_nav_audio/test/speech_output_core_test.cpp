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
#include <vector>

#include "gtest/gtest.h"
#include "speech_output_core.hpp"

namespace voice_nav_audio
{
namespace
{

class ManualTts final : public TtsAdapter
{
public:
  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    request_ = request;
    sink_ = &sink;
    ++starts;
  }

  void cancel(const std::uint64_t scope_id) noexcept override
  {
    cancelled.push_back(scope_id);
  }

  [[nodiscard]] bool emit(
    const std::array<Sample, 147U> & samples) const noexcept
  {
    return sink_ != nullptr && sink_->on_pcm(request_.scope_id, 22050U, 1U,
      samples.data(), samples.size());
  }

  [[nodiscard]] bool emit_with_format(
    const std::uint32_t sample_rate_hz, const std::uint32_t channels,
    const Sample * const samples, const std::size_t sample_count) const noexcept
  {
    return sink_ != nullptr && sink_->on_pcm(
      request_.scope_id, sample_rate_hz, channels, samples, sample_count);
  }

  void complete() const noexcept
  {
    if (sink_ != nullptr) {
      sink_->on_complete(request_.scope_id);
    }
  }

  void fail(const std::string & detail) const noexcept
  {
    if (sink_ != nullptr) {
      sink_->on_failed(request_.scope_id, detail);
    }
  }

  TtsRequest request_{};
  TtsSink * sink_{nullptr};
  std::size_t starts{0U};
  std::vector<std::uint64_t> cancelled{};
};

class CollectingObserver final : public SpeechOutputObserver
{
public:
  void on_played(const std::uint64_t scope_id, const std::uint64_t samples) noexcept override
  {
    played_scope_ids.push_back(scope_id);
    played_samples.push_back(samples);
  }

  void on_result(const SpeechResult & result) noexcept override
  {
    results.push_back(result);
  }

  std::vector<std::uint64_t> played_scope_ids{};
  std::vector<std::uint64_t> played_samples{};
  std::vector<SpeechResult> results{};
};

SpeechGoal normal_goal()
{
  return SpeechGoal{"voice-instance", 7U, "session", "turn", SpeechPriority::Normal,
    "你好", false};
}

TEST(SpeechOutputCoreTest, CompletesOneNormalGoalOnlyAfterAudioEngineWritesPcm)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));
  ASSERT_EQ(tts.starts, 1U);

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();
  (void)core.advance();

  EXPECT_TRUE(observer.played_samples.empty());
  EXPECT_TRUE(observer.results.empty());

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();
  (void)core.advance();

  ASSERT_EQ(observer.played_samples.size(), 1U);
  EXPECT_EQ(observer.played_scope_ids.front(), admission.scope_id);
  EXPECT_EQ(observer.played_samples.front(), 320U);
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
  EXPECT_EQ(observer.results.front().played_samples, 320U);
}

TEST(SpeechOutputCoreTest, CompletesNewScopeAfterPlaybackOnlyFence)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);

  const auto capture_generation = engine.generation();
  engine.request_playback_fence();
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  ASSERT_EQ(engine.generation(), capture_generation);
  ASSERT_GT(engine.playback_generation(), capture_generation);

  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
  (void)core.advance();

  ASSERT_EQ(observer.played_samples.size(), 1U);
  EXPECT_EQ(observer.played_samples.front(), 320U);
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
}

TEST(SpeechOutputCoreTest, RejectsPcmAfterGenerationFenceOnceAudioWasEnqueued)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));

  engine.request_playback_fence();
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});

  EXPECT_FALSE(tts.emit(source));
  EXPECT_TRUE(observer.results.empty());
}

TEST(SpeechOutputCoreTest, RejectsWrongFormatAfterGenerationChangeThenAcceptsCurrentPcm)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{false, true});

  EXPECT_FALSE(tts.emit_with_format(16000U, 1U, source.data(), source.size()));
  EXPECT_FALSE(tts.emit_with_format(22050U, 2U, source.data(), source.size()));
  EXPECT_TRUE(observer.results.empty());
  EXPECT_TRUE(tts.emit(source));
  tts.complete();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();

  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
}

TEST(SpeechOutputCoreTest, BuffersOneBoundedTtsBurstBeforeTheFirstAudioCallback)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  constexpr std::size_t packet_count = 300U;
  for (std::size_t packet = 0U; packet < packet_count; ++packet) {
    ASSERT_TRUE(tts.emit(source)) << "packet=" << packet;
  }
  tts.complete();

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  for (std::size_t packet = 0U; packet < packet_count; ++packet) {
    engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
    AudioFrame capture{};
    AudioFrame reference{};
    (void)engine.try_pop_capture(capture);
    (void)engine.try_pop_reference(reference);
    (void)core.advance();
  }

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
  EXPECT_EQ(observer.results.front().played_samples, packet_count * 320U);
}

TEST(SpeechOutputCoreTest, RejectsRealPlaybackOverflowFailClosed)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  for (std::size_t packet = 0U; packet < AudioEngine::kPlaybackRingCapacity; ++packet) {
    const auto accepted = tts.emit(source);
    if (packet + 1U == AudioEngine::kPlaybackRingCapacity) {
      EXPECT_FALSE(accepted);
    } else {
      ASSERT_TRUE(accepted) << "packet=" << packet;
    }
  }

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Failed);
  EXPECT_EQ(observer.results.front().detail, "AudioEngine playback ring rejected PCM");
}

TEST(SpeechOutputCoreTest, RestartsCompletedUnplayedScopeOnceAfterGenerationChange)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.mark_discontinuity();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();
  EXPECT_TRUE(observer.results.empty());

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  ASSERT_TRUE(core.advance());
  ASSERT_EQ(core.ready_scope_id(), admission.scope_id);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));
  ASSERT_EQ(tts.starts, 2U);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();
  const auto retry_generation = engine.playback_generation();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{false, true});
  EXPECT_EQ(engine.playback_generation(), retry_generation);

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
  (void)core.advance();

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
  EXPECT_EQ(observer.results.front().played_samples, 320U);
}

TEST(SpeechOutputCoreTest, FailsClosedWhenGenerationChangesDuringTheSingleRetry)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.mark_discontinuity();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  ASSERT_TRUE(core.advance());
  ASSERT_EQ(core.ready_scope_id(), admission.scope_id);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));
  ASSERT_EQ(tts.starts, 2U);

  engine.mark_discontinuity();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_FALSE(tts.emit(source));
  tts.complete();
  (void)core.advance();

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Failed);
  EXPECT_EQ(observer.results.front().detail, "AudioEngine generation changed");
}

TEST(SpeechOutputCoreTest, ReportsBoundedGenerationReasonsWhenRetryFenceChanges)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));
  tts.complete();

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.mark_discontinuity();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();

  ASSERT_TRUE(core.advance() == false);
  ASSERT_TRUE(core.ready_scope_id() == 0U);

  engine.process_callback(nullptr, output.data(), output.size() - 1U, CallbackStatus{});
  (void)core.advance();

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Failed);
  EXPECT_NE(observer.results.front().detail.find("frame=1"), std::string::npos);
  EXPECT_NE(observer.results.front().detail.find("playback=1"), std::string::npos);
  EXPECT_LE(observer.results.front().detail.size(), 192U);
}

TEST(SpeechOutputCoreTest, FailsClosedWhenGenerationChangesAfterPlaybackStarted)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> source{};
  source.fill(1200);
  ASSERT_TRUE(tts.emit(source));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();
  ASSERT_EQ(observer.played_samples.back(), 320U);

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{false, true});
  tts.complete();
  (void)core.advance();

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Failed);
  EXPECT_EQ(observer.results.front().played_samples, 320U);
}

TEST(SpeechOutputCoreTest, RejectsConcurrentNormalAndLetsUrgentPreemptOnlyNormal)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto normal = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(normal.scope_id));

  const auto rejected = core.start(normal_goal());
  ASSERT_TRUE(rejected.has_immediate_result);
  EXPECT_EQ(rejected.immediate_result.code, SpeechResultCode::Failed);

  auto urgent_goal = normal_goal();
  urgent_goal.priority = SpeechPriority::Urgent;
  urgent_goal.text = "请立刻注意";
  const auto urgent = core.start(urgent_goal);
  ASSERT_TRUE(urgent.waits_for_generation);
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().scope_id, normal.scope_id);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Canceled);

  std::array<Sample, 147U> old_pcm{};
  old_pcm.fill(900);
  EXPECT_FALSE(tts.emit(old_pcm));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  EXPECT_TRUE(core.advance());
  ASSERT_TRUE(core.begin_synthesis(urgent.scope_id));
  EXPECT_EQ(tts.request_.text, urgent_goal.text);

  std::array<Sample, 147U> urgent_pcm{};
  urgent_pcm.fill(1200);
  ASSERT_TRUE(tts.emit(urgent_pcm));
  tts.complete();
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));

  (void)core.advance();
  ASSERT_EQ(observer.played_samples.size(), 1U);
  EXPECT_EQ(observer.played_scope_ids.front(), urgent.scope_id);
  EXPECT_EQ(observer.played_samples.front(), 320U);
  ASSERT_EQ(observer.results.size(), 2U);
  EXPECT_EQ(observer.results.back().scope_id, urgent.scope_id);
  EXPECT_EQ(observer.results.back().code, SpeechResultCode::Completed);
}

TEST(SpeechOutputCoreTest, CancelFencesLatePcmAndReturnsOneCanceledResult)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> pcm{};
  pcm.fill(800);
  ASSERT_TRUE(tts.emit(pcm));
  ASSERT_TRUE(core.cancel(admission.scope_id));
  EXPECT_FALSE(tts.emit(pcm));
  tts.complete();

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  (void)core.advance();
  (void)core.advance();
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Canceled);
  EXPECT_TRUE(observer.played_samples.empty());
}

TEST(SpeechOutputCoreTest, CancelLetsReplacementScopeWriteNonSilentPcmAndCompleteOnce)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto canceled = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(canceled.scope_id));

  std::array<Sample, 147U> old_pcm{};
  old_pcm.fill(800);
  ASSERT_TRUE(tts.emit(old_pcm));
  ASSERT_TRUE(core.cancel(canceled.scope_id));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));

  const auto replacement = core.start(normal_goal());
  ASSERT_TRUE(replacement.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(replacement.scope_id));
  std::array<Sample, 147U> replacement_pcm{};
  replacement_pcm.fill(1200);
  ASSERT_TRUE(tts.emit(replacement_pcm));
  tts.complete();

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::any_of(output.begin(), output.end(), [](const Sample sample) {
      return sample != 0;
    }));
  (void)core.advance();

  ASSERT_EQ(observer.played_samples.size(), 1U);
  EXPECT_EQ(observer.played_scope_ids.front(), replacement.scope_id);
  ASSERT_EQ(observer.results.size(), 2U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Canceled);
  EXPECT_EQ(observer.results.back().scope_id, replacement.scope_id);
  EXPECT_EQ(observer.results.back().code, SpeechResultCode::Completed);
}

TEST(SpeechOutputCoreTest, BargeInWaitsForItsPlaybackFenceBeforeStartingReplacementTts)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  auto barged_goal = normal_goal();
  barged_goal.allow_barge_in = true;
  const auto barged = core.start(barged_goal);
  ASSERT_TRUE(core.begin_synthesis(barged.scope_id));

  std::array<Sample, 147U> pcm{};
  pcm.fill(800);
  ASSERT_TRUE(tts.emit(pcm));
  ASSERT_TRUE(core.interrupt_for_barge_in());

  const auto replacement = core.start(normal_goal());
  EXPECT_FALSE(replacement.start_synthesis);
  EXPECT_TRUE(replacement.waits_for_generation);
  EXPECT_EQ(core.ready_scope_id(), 0U);

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(core.advance());
  EXPECT_EQ(core.ready_scope_id(), replacement.scope_id);
  ASSERT_TRUE(core.begin_synthesis(replacement.scope_id));
  ASSERT_TRUE(tts.emit(pcm));
  tts.complete();

  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  (void)core.advance();
  ASSERT_EQ(observer.results.size(), 2U);
  EXPECT_EQ(observer.results.front().scope_id, barged.scope_id);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::BargedIn);
  EXPECT_EQ(observer.results.back().scope_id, replacement.scope_id);
  EXPECT_EQ(observer.results.back().code, SpeechResultCode::Completed);
}

TEST(SpeechOutputCoreTest, FullyFadedPacketDoesNotAdvancePlayedOrComplete)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, 147U> pcm{};
  pcm.fill(1);
  pcm.front() = 0;
  ASSERT_TRUE(tts.emit(pcm));
  tts.complete();
  engine.request_fade_to_silence(4U);

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(output.begin(), output.end(), [](const Sample sample) {
      return sample == 0;
    }));
  (void)core.advance();

  EXPECT_TRUE(observer.played_samples.empty());
  EXPECT_TRUE(observer.results.empty());
}

TEST(SpeechOutputCoreTest, TtsFailureReturnsOneFailedResult)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  const auto admission = core.start(normal_goal());
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));
  tts.fail("deterministic fake failure");
  tts.fail("late duplicate");

  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Failed);
  EXPECT_EQ(observer.results.front().detail, "deterministic fake failure");
}

TEST(SpeechOutputCoreTest, InvalidGoalProducesStructuredFailureWithoutStartingTts)
{
  AudioEngine engine;
  ManualTts tts;
  CollectingObserver observer;
  SpeechOutputCore core(engine, tts, observer);
  auto invalid = normal_goal();
  invalid.text.clear();

  const auto rejected = core.start(invalid);
  ASSERT_TRUE(rejected.has_immediate_result);
  EXPECT_EQ(rejected.immediate_result.code, SpeechResultCode::Failed);
  EXPECT_EQ(rejected.immediate_result.detail, "invalid Speak goal");
  EXPECT_EQ(tts.starts, 0U);
  EXPECT_TRUE(observer.results.empty());

  const auto following = core.start(normal_goal());
  EXPECT_TRUE(following.start_synthesis);
}

}  // namespace
}  // namespace voice_nav_audio
