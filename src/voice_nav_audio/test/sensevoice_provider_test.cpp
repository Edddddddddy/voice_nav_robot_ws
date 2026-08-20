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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "sensevoice_provider.hpp"

namespace voice_nav_audio
{
namespace
{

using namespace std::chrono_literals;

CleanedAudioFrame frame(const std::uint64_t sequence)
{
  CleanedAudioFrame input{};
  input.audio_generation = 1U;
  input.audio_seq = sequence;
  input.samples.fill(100);
  return input;
}

CleanedAudioFrame frame(const std::uint64_t generation, const std::uint64_t sequence)
{
  CleanedAudioFrame input = frame(sequence);
  input.audio_generation = generation;
  return input;
}

class RecordingEventSink final : public SpeechEventSink
{
public:
  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events.push_back(event);
    }
    condition_.notify_all();
  }

  bool wait_for_kind(const SpeechEventKind kind)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, kind]() {
      for (const auto & event : events) {
        if (event.kind == kind) {
          return true;
        }
      }
      return false;
    });
  }

  bool wait_for_kind_count(const SpeechEventKind kind, const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, kind, expected]() {
      std::size_t actual = 0U;
      for (const auto & event : events) {
        actual += event.kind == kind ? 1U : 0U;
      }
      return actual >= expected;
    });
  }

  std::size_t count(const SpeechEventKind kind) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t result = 0U;
    for (const auto & event : events) {
      result += event.kind == kind ? 1U : 0U;
    }
    return result;
  }

  std::size_t total_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events.size();
  }

  std::vector<SpeechRecognitionEvent> events{};

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

class ScriptedSileroVad final : public SileroVadAdapter
{
public:
  explicit ScriptedSileroVad(const std::size_t endpoint_after)
  : endpoint_after_(endpoint_after)
  {
    flush_result = SileroVadFlushResult{
      SileroVadFlushStatus::kUnique, endpoint_after_ * CleanedAudioFrame::kSamples};
  }

  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++calls;
    ++turn_calls_;
    condition_.notify_all();
    if (turn_calls_ >= endpoint_after_) {
      return SileroVadResult{
        SileroVadDecision::kEndpoint, turn_calls_ * CleanedAudioFrame::kSamples};
    }
    return SileroVadResult{SileroVadDecision::kSpeech, 0U};
  }

  SileroVadFlushResult finish_input() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++finish_calls;
    condition_.notify_all();
    return flush_result;
  }

  void reset() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++reset_calls;
    turn_calls_ = 0U;
  }

  bool wait_for_calls(const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, expected]() {return calls >= expected;});
  }

  std::size_t call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

  std::size_t finish_call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_calls;
  }

  std::size_t calls{0U};
  std::size_t finish_calls{0U};
  std::size_t reset_calls{0U};
  SileroVadFlushResult flush_result{
    SileroVadFlushStatus::kUnique, 0U};

private:
  const std::size_t endpoint_after_;
  std::size_t turn_calls_{0U};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

class FixedEndpointSileroVad final : public SileroVadAdapter
{
public:
  explicit FixedEndpointSileroVad(const std::size_t endpoint_sample_exclusive)
  : endpoint_sample_exclusive_(endpoint_sample_exclusive)
  {
  }

  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    ++calls;
    return SileroVadResult{SileroVadDecision::kEndpoint, endpoint_sample_exclusive_};
  }

  SileroVadFlushResult finish_input() noexcept override
  {
    return SileroVadFlushResult{SileroVadFlushStatus::kUnique, endpoint_sample_exclusive_};
  }

  void reset() noexcept override
  {
  }

  std::size_t calls{0U};

private:
  const std::size_t endpoint_sample_exclusive_;
};

class RecordingSenseVoice final : public SenseVoiceAsrAdapter
{
public:
  explicit RecordingSenseVoice(
    std::string labeled_text = "<|zh|><|NEUTRAL|><|Speech|><|woitn|>开放时间早上9点至下午5点。")
  : labeled_text_(std::move(labeled_text))
  {
  }

  bool infer(
    const Sample * samples, const std::size_t sample_count,
    std::string & labeled_text) noexcept override
  {
    bool succeeds = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto call_index = calls;
      ++calls;
      if (call_index < inference_results.size()) {
        succeeds = inference_results[call_index];
      }
      inference_thread = std::this_thread::get_id();
      inferred_samples.assign(samples, samples + sample_count);
      if (succeeds) {
        labeled_text = labeled_text_;
      }
    }
    condition_.notify_all();
    return succeeds;
  }

  bool wait_for_call()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return calls != 0U;});
  }

  std::size_t call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

  std::thread::id inference_thread_id() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return inference_thread;
  }

  std::vector<Sample> inferred_samples_copy() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return inferred_samples;
  }

  std::size_t calls{0U};
  std::thread::id inference_thread{};
  std::vector<Sample> inferred_samples{};
  std::vector<bool> inference_results{};

private:
  const std::string labeled_text_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

class ScriptedKeywordSpotter final : public KeywordSpotterAdapter
{
public:
  explicit ScriptedKeywordSpotter(const bool hit_each_utterance)
  : hit_each_utterance_(hit_each_utterance)
  {
  }

  bool process(const CleanedAudioFrame &) noexcept override
  {
    ++calls;
    if (!hit_each_utterance_ || latched_) {
      return false;
    }
    latched_ = true;
    return true;
  }

  void reset() noexcept override
  {
    latched_ = false;
    ++reset_calls;
  }

  std::size_t calls{0U};
  std::size_t reset_calls{0U};

private:
  const bool hit_each_utterance_;
  bool latched_{false};
};

class LongIdleThenEndpointVad final : public SileroVadAdapter
{
public:
  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    ++turn_calls_;
    const auto endpoint_after = turn_index_ == 0U ? 1U : kIdleFrames + 1U;
    return turn_calls_ == endpoint_after ?
           SileroVadResult{
             SileroVadDecision::kEndpoint,
             turn_calls_ * CleanedAudioFrame::kSamples} :
           SileroVadResult{SileroVadDecision::kSilence, 0U};
  }

  SileroVadFlushResult finish_input() noexcept override
  {
    return {};
  }

  void reset() noexcept override
  {
    ++turn_index_;
    turn_calls_ = 0U;
  }

  static constexpr std::size_t kIdleFrames =
    SenseVoiceProviderConfig::kFramesPerSecond * 5U;

private:
  std::size_t turn_index_{0U};
  std::size_t turn_calls_{0U};
};

class AgingKeywordSpotter final : public KeywordSpotterAdapter
{
public:
  bool process(const CleanedAudioFrame &) noexcept override
  {
    ++calls_;
    ++frames_since_reset_;
    if (calls_ == 1U) {
      return true;
    }
    return calls_ == LongIdleThenEndpointVad::kIdleFrames + 2U &&
           frames_since_reset_ == 1U;
  }

  void reset() noexcept override
  {
    frames_since_reset_ = 0U;
    ++reset_calls;
  }

  std::size_t reset_calls{0U};

private:
  std::size_t calls_{0U};
  std::size_t frames_since_reset_{0U};
};

std::unique_ptr<KeywordSpotterAdapter> wake_every_utterance()
{
  return std::make_unique<ScriptedKeywordSpotter>(true);
}

class BlockingSenseVoice final : public SenseVoiceAsrAdapter
{
public:
  bool infer(
    const Sample *, const std::size_t, std::string & labeled_text) noexcept override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
    labeled_text = "<|zh|><|NEUTRAL|><|Speech|><|woitn|>开放时间早上9点至下午5点。";
    finished_ = true;
    condition_.notify_all();
    return true;
  }

  bool wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return entered_;});
  }

  bool wait_until_finished()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return finished_;});
  }

  void release() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_{false};
  bool released_{false};
  bool finished_{false};
};

class PrefixSilenceThenEndpointBarrierVad final : public SileroVadAdapter
{
public:
  explicit PrefixSilenceThenEndpointBarrierVad(const std::size_t prefix_silence_frames)
  : prefix_silence_frames_(prefix_silence_frames)
  {
  }

  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++calls;
    if (calls == prefix_silence_frames_ + 1U) {
      endpoint_entered_ = true;
      condition_.notify_all();
      condition_.wait(lock, [this]() {return endpoint_released_;});
    }
    return calls <= prefix_silence_frames_ ?
           SileroVadResult{SileroVadDecision::kSilence, 0U} :
           SileroVadResult{
             SileroVadDecision::kEndpoint, calls * CleanedAudioFrame::kSamples};
  }

  void reset() noexcept override
  {
  }

  bool wait_until_endpoint_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return endpoint_entered_;});
  }

  void release_endpoint() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      endpoint_released_ = true;
    }
    condition_.notify_all();
  }

  std::size_t call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

private:
  const std::size_t prefix_silence_frames_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t calls{0U};
  bool endpoint_entered_{false};
  bool endpoint_released_{false};
};

class SilenceSileroVad final : public SileroVadAdapter
{
public:
  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++calls;
    condition_.notify_all();
    return SileroVadResult{SileroVadDecision::kSilence, 0U};
  }

  void reset() noexcept override
  {
  }

  bool wait_for_calls(const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, expected]() {return calls >= expected;});
  }

  std::size_t call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t calls{0U};
};

class BlockingSileroVad final : public SileroVadAdapter
{
public:
  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++calls;
    blocked_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
    return SileroVadResult{SileroVadDecision::kSpeech, 0U};
  }

  void reset() noexcept override
  {
  }

  bool wait_until_blocked()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return blocked_;});
  }

  bool wait_for_calls(const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, expected]() {return calls >= expected;});
  }

  void release() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

  std::size_t call_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool blocked_{false};
  bool released_{false};
  std::size_t calls{0U};
};

struct ShutdownBarrierVadState
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  bool process_entered{false};
  bool process_active{false};
  bool process_released{false};
  bool process_exited{false};
  bool reset_during_process{false};
  bool destroyed_during_process{false};
  std::size_t reset_calls{0U};
};

class ShutdownBarrierVad final : public SileroVadAdapter
{
public:
  explicit ShutdownBarrierVad(std::shared_ptr<ShutdownBarrierVadState> state)
  : state_(std::move(state))
  {
  }

  ~ShutdownBarrierVad() override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->destroyed_during_process = state_->process_active;
    state_->condition.notify_all();
  }

  SileroVadResult process(const CleanedAudioFrame &) noexcept override
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->process_entered = true;
    state_->process_active = true;
    state_->condition.notify_all();
    state_->condition.wait(lock, [this]() {return state_->process_released;});
    state_->process_active = false;
    state_->process_exited = true;
    state_->condition.notify_all();
    return SileroVadResult{SileroVadDecision::kSilence, 0U};
  }

  void reset() noexcept override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    ++state_->reset_calls;
    state_->reset_during_process = state_->reset_during_process || state_->process_active;
    state_->condition.notify_all();
  }

  bool wait_until_process_entered()
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    return state_->condition.wait_for(lock, 2s, [this]() {return state_->process_entered;});
  }

  void release_process() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->process_released = true;
    }
    state_->condition.notify_all();
  }

private:
  std::shared_ptr<ShutdownBarrierVadState> state_;
};

struct ShutdownAwareAsrState
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  bool shutdown_called{false};
  std::size_t inference_calls{0U};
  std::size_t late_inference_calls{0U};
  std::vector<std::string> lifecycle_events{};
};

class ShutdownAwareAsr final : public SenseVoiceAsrAdapter
{
public:
  explicit ShutdownAwareAsr(std::shared_ptr<ShutdownAwareAsrState> state)
  : state_(std::move(state))
  {
  }

  bool infer(
    const Sample *, const std::size_t, std::string & labeled_text) noexcept override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->lifecycle_events.emplace_back("infer");
    ++state_->inference_calls;
    if (state_->shutdown_called) {
      ++state_->late_inference_calls;
    }
    labeled_text = "<|zh|><|NEUTRAL|><|Speech|><|woitn|>不应发布。";
    state_->condition.notify_all();
    return true;
  }

  void shutdown() noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->lifecycle_events.emplace_back("shutdown");
      state_->shutdown_called = true;
    }
    state_->condition.notify_all();
  }

  bool wait_until_shutdown()
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    return state_->condition.wait_for(lock, 2s, [this]() {return state_->shutdown_called;});
  }

private:
  std::shared_ptr<ShutdownAwareAsrState> state_;
};

class WakeBarrierSink final : public SpeechEventSink
{
public:
  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    events.push_back(event.kind);
    if (event.kind == SpeechEventKind::kWakeAccepted) {
      wake_entered_ = true;
      condition_.notify_all();
      condition_.wait(lock, [this]() {return wake_released_;});
    }
  }

  bool wait_until_wake_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return wake_entered_;});
  }

  void release_wake() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wake_released_ = true;
    }
    condition_.notify_all();
  }

  std::size_t count(const SpeechEventKind kind) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t result = 0U;
    for (const auto event_kind : events) {
      result += event_kind == kind ? 1U : 0U;
    }
    return result;
  }

  std::vector<SpeechEventKind> events{};

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool wake_entered_{false};
  bool wake_released_{false};
};

class ScopeWiringSink final : public SpeechEventSink
{
public:
  explicit ScopeWiringSink(SenseVoiceProvider & provider)
  : provider_(provider)
  {
  }

  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override
  {
    if (event.kind == SpeechEventKind::kWakeAccepted) {
      TurnScopeIdentity scope{};
      scope.id = 1U;
      scope.audio_generation = event.audio_generation;
      scope.session_id = "test-session";
      scope.turn_id = "test-turn";
      provider_.on_turn_scope_opened(scope);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events.push_back(event);
    }
    condition_.notify_all();
  }

  bool wait_for_kind(const SpeechEventKind kind)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, kind]() {
      for (const auto & event : events) {
        if (event.kind == kind) {
          return true;
        }
      }
      return false;
    });
  }

  bool wait_for_kind_count(const SpeechEventKind kind, const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, kind, expected]() {
      std::size_t actual = 0U;
      for (const auto & event : events) {
        actual += event.kind == kind ? 1U : 0U;
      }
      return actual >= expected;
    });
  }

  std::size_t count(const SpeechEventKind kind) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t result = 0U;
    for (const auto & event : events) {
      result += event.kind == kind ? 1U : 0U;
    }
    return result;
  }

  std::vector<SpeechRecognitionEvent> events{};

private:
  SenseVoiceProvider & provider_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

class CyclingScopeSink final : public SpeechEventSink
{
public:
  explicit CyclingScopeSink(SenseVoiceProvider & provider)
  : provider_(provider)
  {
  }

  void on_speech_event(const SpeechRecognitionEvent & event) noexcept override
  {
    if (event.kind == SpeechEventKind::kWakeAccepted) {
      active_scope_.id = ++next_scope_id_;
      active_scope_.audio_generation = event.audio_generation;
      active_scope_.session_id = "idle-reset-session";
      active_scope_.turn_id = "idle-reset-turn-" + std::to_string(next_scope_id_);
      provider_.on_turn_scope_opened(active_scope_);
    } else if (event.kind == SpeechEventKind::kEndpointFinal && active_scope_.id != 0U) {
      provider_.on_turn_scope_retired(active_scope_);
      active_scope_ = {};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events_.push_back(event.kind);
    }
    condition_.notify_all();
  }

  bool wait_for_kind_count(const SpeechEventKind kind, const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, kind, expected]() {
      return static_cast<std::size_t>(std::count(events_.cbegin(), events_.cend(), kind)) >=
             expected;
    });
  }

private:
  SenseVoiceProvider & provider_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<SpeechEventKind> events_{};
  TurnScopeIdentity active_scope_{};
  std::uint64_t next_scope_id_{0U};
};

class CollectingVoiceTurnSink final : public VoiceTurnSink
{
public:
  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      turns.push_back(turn);
    }
    condition_.notify_all();
  }

  bool wait_for_count(const std::size_t expected)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, expected]() {
               return turns.size() >= expected;
    });
  }

  std::vector<VoiceTurnPublication> turns{};

private:
  std::mutex mutex_;
  std::condition_variable condition_;
};

TEST(SenseVoiceProviderTest, ContinuousProviderReusesVadAndAsrAcrossTwoTurns)
{
  auto vad = std::make_unique<ScriptedSileroVad>(3U);
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  ScopeWiringSink sink(provider);

  const auto callback_thread = std::this_thread::get_id();
  provider.process_frame(frame(1U), sink);
  provider.process_frame(frame(2U), sink);
  provider.process_frame(frame(3U), sink);

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  ASSERT_TRUE(asr_probe->wait_for_call());
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 1U);
  EXPECT_NE(asr_probe->inference_thread_id(), callback_thread);
  EXPECT_EQ(vad_probe->call_count(), 3U);
  EXPECT_EQ(vad_probe->reset_calls, 1U);
  ASSERT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 1U);
  EXPECT_EQ(sink.events.back().final_text, "开放时间早上9点至下午5点。");

  provider.process_frame(frame(4U), sink);
  provider.process_frame(frame(5U), sink);
  provider.process_frame(frame(6U), sink);
  ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kEndpointFinal, 2U));
  ASSERT_TRUE(asr_probe->wait_for_call());
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 2U);
  EXPECT_EQ(vad_probe->call_count(), 6U);
  EXPECT_EQ(vad_probe->reset_calls, 2U);
  EXPECT_EQ(asr_probe->call_count(), 2U);
}

TEST(SenseVoiceProviderTest, LongIdleSilenceRearmsKeywordStreamForTheNextTurn)
{
  auto keyword_spotter = std::make_unique<AgingKeywordSpotter>();
  auto * const keyword_probe = keyword_spotter.get();
  SenseVoiceProvider provider(
    std::make_unique<LongIdleThenEndpointVad>(),
    std::make_unique<RecordingSenseVoice>("小智开始建图。"),
    std::move(keyword_spotter));
  CyclingScopeSink sink(provider);

  provider.process_frame(frame(1U), sink);
  ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kEndpointFinal, 1U));

  for (std::uint64_t sequence = 2U;
    sequence <= LongIdleThenEndpointVad::kIdleFrames + 2U; ++sequence)
  {
    provider.process_frame(frame(sequence), sink);
  }

  ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kWakeAccepted, 2U));
  ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kEndpointFinal, 2U));
  EXPECT_GE(keyword_probe->reset_calls, 3U);
}

TEST(SenseVoiceProviderTest, OrdinarySpeechWithoutAcousticWakePublishesNoTurn)
{
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  auto asr = std::make_unique<RecordingSenseVoice>("向前走0.5米。");
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(
    std::move(vad), std::move(asr), std::make_unique<ScriptedKeywordSpotter>(false));
  RecordingEventSink sink;

  provider.process_frame(frame(1U), sink);

  ASSERT_TRUE(asr_probe->wait_for_call());
  EXPECT_EQ(sink.count(SpeechEventKind::kWakeAccepted), 0U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
}

TEST(SenseVoiceProviderTest, SpokenWakePrefixRecoversAnAcousticKeywordMiss)
{
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  SenseVoiceProvider provider(
    std::move(vad),
    std::make_unique<RecordingSenseVoice>("小智停止并保存地图。"),
    std::make_unique<ScriptedKeywordSpotter>(false));
  ScopeWiringSink sink(provider);

  provider.process_frame(frame(1U), sink);

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  EXPECT_EQ(sink.count(SpeechEventKind::kWakeAccepted), 1U);
  ASSERT_EQ(sink.events.back().kind, SpeechEventKind::kEndpointFinal);
  EXPECT_EQ(sink.events.back().final_text, "小智停止并保存地图。");
  EXPECT_EQ(sink.events.back().scope.id, 1U);
}

TEST(SenseVoiceProviderTest, PrivilegedStopRemainsAvailableWithoutAcousticWake)
{
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  auto asr = std::make_unique<RecordingSenseVoice>("紧急停止");
  SenseVoiceProvider provider(
    std::move(vad), std::move(asr), std::make_unique<ScriptedKeywordSpotter>(false));
  RecordingEventSink sink;

  provider.process_frame(frame(1U), sink);

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  EXPECT_EQ(sink.count(SpeechEventKind::kWakeAccepted), 0U);
  ASSERT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 1U);
  EXPECT_EQ(sink.events.back().voice_turn_kind, VoiceTurnKind::kStop);
  EXPECT_EQ(sink.events.back().final_text, "紧急停止");
  EXPECT_EQ(sink.events.back().scope.id, 0U);
}

TEST(SenseVoiceProviderTest, ReArmsAfterAsrInferenceFailureForTheNextTurn)
{
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  auto asr = std::make_unique<RecordingSenseVoice>();
  asr->inference_results = {false, true};
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  ScopeWiringSink sink(provider);

  provider.process_frame(frame(1U), sink);
  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
  provider.process_frame(frame(2U), sink);

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  EXPECT_EQ(sink.count(SpeechEventKind::kFailure), 1U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 1U);
  EXPECT_EQ(asr_probe->call_count(), 2U);
}

TEST(SenseVoiceProviderTest, EndpointInferenceIncludesPreSpeechAndTrailingFrames)
{
  constexpr std::size_t kPrefixSilenceFrames = 2U;
  constexpr std::size_t kEndpointFrames = 1U;
  auto vad = std::make_unique<PrefixSilenceThenEndpointBarrierVad>(kPrefixSilenceFrames);
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  ScopeWiringSink sink(provider);

  auto prefix_one = frame(1U);
  prefix_one.samples.fill(111);
  auto prefix_two = frame(2U);
  prefix_two.samples.fill(222);
  auto endpoint = frame(3U);
  endpoint.samples.fill(333);
  provider.process_frame(prefix_one, sink);
  provider.process_frame(prefix_two, sink);
  provider.process_frame(endpoint, sink);

  ASSERT_TRUE(vad_probe->wait_until_endpoint_entered());
  auto trailing_one = frame(4U);
  trailing_one.samples.fill(444);
  auto trailing_two = frame(5U);
  trailing_two.samples.fill(555);
  provider.process_frame(trailing_one, sink);
  provider.process_frame(trailing_two, sink);
  EXPECT_EQ(vad_probe->call_count(), kPrefixSilenceFrames + kEndpointFrames);
  vad_probe->release_endpoint();

  provider.finish_input();

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  ASSERT_TRUE(asr_probe->wait_for_call());
  const auto samples = asr_probe->inferred_samples_copy();
  ASSERT_EQ(
    samples.size(), (kPrefixSilenceFrames + kEndpointFrames) * CleanedAudioFrame::kSamples);
  EXPECT_EQ(samples[0U], 111);
  EXPECT_EQ(samples[CleanedAudioFrame::kSamples], 222);
  EXPECT_EQ(samples[2U * CleanedAudioFrame::kSamples], 333);
  EXPECT_EQ(vad_probe->call_count(), kPrefixSilenceFrames + kEndpointFrames);
}

TEST(SenseVoiceProviderTest, InvalidEndpointSampleFailsClosedWithoutInference)
{
  const std::vector<std::size_t> invalid_endpoints{
    0U, CleanedAudioFrame::kSamples + 1U};

  for (const auto endpoint_sample_exclusive : invalid_endpoints) {
    auto vad = std::make_unique<FixedEndpointSileroVad>(endpoint_sample_exclusive);
    auto asr = std::make_unique<RecordingSenseVoice>();
    auto * const asr_probe = asr.get();
    SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
    RecordingEventSink sink;

    provider.process_frame(frame(1U), sink);

    ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
    provider.process_frame(frame(2U), sink);
    ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kFailure, 2U));
    EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
    EXPECT_EQ(asr_probe->call_count(), 0U);
  }
}

TEST(SenseVoiceProviderTest, FlushKeepsValidTailAndExcludesPadding)
{
  auto vad = std::make_unique<ScriptedSileroVad>(100U);
  auto * const vad_probe = vad.get();
  vad->flush_result = SileroVadFlushResult{
    SileroVadFlushStatus::kUnique, 192U};
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  ScopeWiringSink sink(provider);

  auto first = frame(1U);
  first.samples.fill(111);
  auto last = frame(2U);
  last.valid_samples = 32U;
  last.samples.fill(999);
  std::fill_n(last.samples.begin(), last.valid_samples, static_cast<Sample>(222));
  provider.process_frame(first, sink);
  provider.process_frame(last, sink);
  EXPECT_EQ(asr_probe->call_count(), 0U);

  provider.finish_input();
  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  ASSERT_TRUE(asr_probe->wait_for_call());
  const auto samples = asr_probe->inferred_samples_copy();
  ASSERT_EQ(samples.size(), 192U);
  EXPECT_EQ(samples.front(), 111);
  EXPECT_EQ(samples[160U], 222);
  EXPECT_EQ(vad_probe->finish_call_count(), 1U);
  EXPECT_EQ(asr_probe->call_count(), 1U);
}

TEST(SenseVoiceProviderTest, EmptyMultipleZeroAndOutOfRangeFlushFailClosed)
{
  const std::vector<SileroVadFlushResult> invalid_flushes{
    {SileroVadFlushStatus::kEmpty, 0U},
    {SileroVadFlushStatus::kMultiple, 0U},
    {SileroVadFlushStatus::kUnique, 0U},
    {SileroVadFlushStatus::kUnique, CleanedAudioFrame::kSamples + 1U},
  };

  for (const auto flush : invalid_flushes) {
    auto vad = std::make_unique<ScriptedSileroVad>(100U);
    vad->flush_result = flush;
    auto asr = std::make_unique<RecordingSenseVoice>();
    auto * const asr_probe = asr.get();
    SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
    RecordingEventSink sink;
    provider.process_frame(frame(1U), sink);
    provider.finish_input();

    ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
    provider.process_frame(frame(2U), sink);
    provider.finish_input();
    ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kFailure, 2U));
    EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
    EXPECT_EQ(asr_probe->call_count(), 0U);
  }
}

TEST(SenseVoiceProviderTest, DuplicateFinishInputFailsClosedWithoutSecondInference)
{
  auto vad = std::make_unique<ScriptedSileroVad>(100U);
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  RecordingEventSink sink;
  provider.process_frame(frame(1U), sink);
  provider.finish_input();
  provider.finish_input();

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
  EXPECT_EQ(asr_probe->call_count(), 0U);
  EXPECT_EQ(vad_probe->finish_call_count(), 0U);
}

TEST(SenseVoiceProviderTest, RejectsUnknownOrMalformedSenseVoiceTagCombinations)
{
  const std::vector<std::string> invalid_labels{
    "<|zh|><|Speech|><|woitn|>开放时间早上9点至下午5点。",
    "<|zh|><|NEUTRAL|><|Speech|>开放时间早上9点至下午5点。",
    "<|zh|><|NEUTRAL|><|Speech|><|woitn|><|zh|>开放时间早上9点至下午5点。",
    "<|zh|><|NEUTRAL|><|Speech|><|woitn|><|en|>开放时间早上9点至下午5点。",
    "<|zh|><|Speech|><|NEUTRAL|><|woitn|>开放时间早上9点至下午5点。",
    "<|zh|><|NEUTRAL|><|Speech|><|unknown|>开放时间早上9点至下午5点。",
  };

  for (const auto & invalid_label : invalid_labels) {
    auto vad = std::make_unique<ScriptedSileroVad>(1U);
    auto asr = std::make_unique<RecordingSenseVoice>(invalid_label);
    SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
    ScopeWiringSink sink(provider);
    provider.process_frame(frame(1U), sink);
    provider.finish_input();

    ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
    provider.process_frame(frame(2U), sink);
    ASSERT_TRUE(sink.wait_for_kind_count(SpeechEventKind::kFailure, 2U));
    EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
  }
}

TEST(SenseVoiceProviderTest, PlainUpstreamSenseVoiceTextRemainsAccepted)
{
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  auto asr = std::make_unique<RecordingSenseVoice>("开放时间早上9点至下午5点。");
  SenseVoiceProvider provider(std::move(vad), std::move(asr), wake_every_utterance());
  ScopeWiringSink sink(provider);
  provider.process_frame(frame(1U), sink);
  provider.finish_input();

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kEndpointFinal));
  EXPECT_EQ(sink.events.back().final_text, "开放时间早上9点至下午5点。");
}

TEST(SenseVoiceProviderTest, FifteenSecondSilenceBudgetTimesOutAtExactFrame)
{
  constexpr std::size_t kFramesPerSecond = 100U;
  constexpr std::size_t kMaximumUtteranceSeconds = 15U;
  constexpr std::size_t kMaximumUtteranceFrames =
    kFramesPerSecond * kMaximumUtteranceSeconds;
  auto vad = std::make_unique<SilenceSileroVad>();
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(
    std::move(vad), std::move(asr), std::make_unique<ScriptedKeywordSpotter>(false),
    SenseVoiceProviderConfig{kMaximumUtteranceFrames});
  RecordingEventSink sink;

  for (std::uint64_t sequence = 1U; sequence <= kMaximumUtteranceFrames; ++sequence) {
    provider.process_frame(frame(sequence), sink);
  }

  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kTimeout));
  EXPECT_TRUE(vad_probe->wait_for_calls(kMaximumUtteranceFrames));
  provider.process_frame(frame(kMaximumUtteranceFrames + 1U), sink);
  EXPECT_TRUE(vad_probe->wait_for_calls(kMaximumUtteranceFrames + 1U));
  EXPECT_EQ(vad_probe->call_count(), kMaximumUtteranceFrames + 1U);
  EXPECT_EQ(asr_probe->call_count(), 0U);
  EXPECT_EQ(sink.count(SpeechEventKind::kWakeAccepted), 0U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
}

TEST(SenseVoiceProviderTest, QueueOverflowQuarantinesOnWorkerWithOneFailure)
{
  constexpr std::size_t kQueueCapacityFrames = 3U;
  auto vad = std::make_unique<BlockingSileroVad>();
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<RecordingSenseVoice>();
  auto * const asr_probe = asr.get();
  SenseVoiceProvider provider(
    std::move(vad), std::move(asr), wake_every_utterance(),
    SenseVoiceProviderConfig{kQueueCapacityFrames});
  RecordingEventSink sink;

  provider.process_frame(frame(1U), sink);
  const bool worker_blocked = vad_probe->wait_until_blocked();
  provider.process_frame(frame(2U), sink);
  provider.process_frame(frame(3U), sink);
  provider.process_frame(frame(4U), sink);
  provider.process_frame(frame(5U), sink);

  EXPECT_EQ(sink.count(SpeechEventKind::kFailure), 0U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
  vad_probe->release();

  ASSERT_TRUE(worker_blocked);
  ASSERT_TRUE(sink.wait_for_kind(SpeechEventKind::kFailure));
  provider.process_frame(frame(6U), sink);
  ASSERT_TRUE(vad_probe->wait_for_calls(2U));
  EXPECT_EQ(vad_probe->call_count(), 2U);
  EXPECT_EQ(asr_probe->call_count(), 0U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
  EXPECT_EQ(
    sink.count(SpeechEventKind::kFailure) + sink.count(SpeechEventKind::kTimeout), 1U);
}

TEST(SenseVoiceProviderTest, ShutdownJoinsWorkerBeforeVadResetAndAsrShutdown)
{
  auto vad_state = std::make_shared<ShutdownBarrierVadState>();
  auto vad = std::make_unique<ShutdownBarrierVad>(vad_state);
  auto * const vad_probe = vad.get();
  auto asr_state = std::make_shared<ShutdownAwareAsrState>();
  auto asr = std::make_unique<ShutdownAwareAsr>(asr_state);
  auto provider = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::move(asr), std::make_unique<ScriptedKeywordSpotter>(false));
  RecordingEventSink sink;

  provider->process_frame(frame(1U), sink);
  ASSERT_TRUE(vad_probe->wait_until_process_entered());

  std::atomic<bool> shutdown_returned{false};
  std::thread shutdown_thread([&provider, &shutdown_returned]() {
      provider->shutdown();
      shutdown_returned.store(true, std::memory_order_release);
    });
  {
    std::lock_guard<std::mutex> lock(asr_state->mutex);
    EXPECT_FALSE(asr_state->shutdown_called);
  }
  EXPECT_FALSE(shutdown_returned.load(std::memory_order_acquire));

  {
    std::lock_guard<std::mutex> lock(vad_state->mutex);
    EXPECT_FALSE(vad_state->reset_during_process);
  }
  {
    std::lock_guard<std::mutex> lock(asr_state->mutex);
    EXPECT_EQ(asr_state->late_inference_calls, 0U);
  }

  // Release the worker before the post-join VAD reset and ASR shutdown.
  vad_probe->release_process();
  shutdown_thread.join();
  EXPECT_TRUE(shutdown_returned.load(std::memory_order_acquire));
  provider.reset();

  {
    std::lock_guard<std::mutex> lock(vad_state->mutex);
    EXPECT_TRUE(vad_state->process_exited);
    EXPECT_FALSE(vad_state->reset_during_process);
    EXPECT_FALSE(vad_state->destroyed_during_process);
    EXPECT_EQ(vad_state->reset_calls, 1U);
  }
  {
    std::lock_guard<std::mutex> lock(asr_state->mutex);
    EXPECT_EQ(asr_state->inference_calls, 0U);
    EXPECT_EQ(asr_state->late_inference_calls, 0U);
  }
  EXPECT_EQ(sink.total_count(), 0U);
}

TEST(SenseVoiceProviderTest, ShutdownFencesBlockedWakeBeforeAsrShutdown)
{
  WakeBarrierSink sink;
  auto vad = std::make_unique<ScriptedSileroVad>(1U);
  auto asr_state = std::make_shared<ShutdownAwareAsrState>();
  auto asr = std::make_unique<ShutdownAwareAsr>(asr_state);
  auto provider = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::move(asr), wake_every_utterance());

  provider->process_frame(frame(1U), sink);
  ASSERT_TRUE(sink.wait_until_wake_entered());

  std::atomic<bool> shutdown_returned{false};
  std::thread shutdown_thread([&provider, &shutdown_returned]() {
      provider->shutdown();
      shutdown_returned.store(true, std::memory_order_release);
    });
  {
    std::lock_guard<std::mutex> lock(asr_state->mutex);
    EXPECT_FALSE(asr_state->shutdown_called);
  }
  sink.release_wake();
  shutdown_thread.join();

  EXPECT_TRUE(shutdown_returned.load(std::memory_order_acquire));
  {
    std::lock_guard<std::mutex> lock(asr_state->mutex);
    EXPECT_TRUE(asr_state->shutdown_called);
    EXPECT_EQ(asr_state->lifecycle_events, (std::vector<std::string>{"shutdown"}));
    EXPECT_EQ(asr_state->inference_calls, 0U);
    EXPECT_EQ(asr_state->late_inference_calls, 0U);
  }
  EXPECT_EQ(sink.count(SpeechEventKind::kWakeAccepted), 1U);
  EXPECT_EQ(sink.count(SpeechEventKind::kEndpointFinal), 0U);
}

TEST(SenseVoiceProviderTest, CoreFenceBlocksGapAndReorderFramesFromProviderAdapters)
{
  auto verify_quarantine = [](const CleanedAudioFrame & first,
    const CleanedAudioFrame & invalid_one, const CleanedAudioFrame & invalid_two,
    const std::size_t expected_adapter_calls) {
      auto vad = std::make_unique<ScriptedSileroVad>(100U);
      auto * const vad_probe = vad.get();
      auto asr = std::make_unique<RecordingSenseVoice>();
      auto provider = std::make_unique<SenseVoiceProvider>(
        std::move(vad), std::move(asr), wake_every_utterance());
      CollectingVoiceTurnSink turn_sink;
      SpeechInputCore core(*provider, turn_sink);

      core.accept_cleaned_frame(first);
      ASSERT_TRUE(vad_probe->wait_for_calls(1U));
      core.accept_cleaned_frame(invalid_one);
      core.accept_cleaned_frame(invalid_two);
      if (expected_adapter_calls > 1U) {
        ASSERT_TRUE(vad_probe->wait_for_calls(expected_adapter_calls));
      }

      provider->shutdown();
      EXPECT_EQ(vad_probe->call_count(), expected_adapter_calls);
      EXPECT_TRUE(turn_sink.turns.empty());
      provider.reset();
    };

  verify_quarantine(frame(1U), frame(1U, 3U), frame(1U, 2U), 1U);
  verify_quarantine(frame(1U), frame(2U, 3U), frame(2U, 2U), 1U);
  // A stale older-generation frame has no side effect; the next contiguous
  // frame in the current generation remains valid and reaches the adapters.
  verify_quarantine(frame(1U), frame(0U, 2U), frame(1U, 2U), 2U);
}

TEST(SenseVoiceProviderTest, CoreDropsLateProviderFinalAfterContinuityRetiresScope)
{
  auto vad = std::make_unique<ScriptedSileroVad>(2U);
  auto * const vad_probe = vad.get();
  auto asr = std::make_unique<BlockingSenseVoice>();
  auto * const asr_probe = asr.get();
  auto provider = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::move(asr), wake_every_utterance());
  CollectingVoiceTurnSink turn_sink;
  SpeechInputCore core(*provider, turn_sink);

  core.accept_cleaned_frame(frame(1U));
  core.accept_cleaned_frame(frame(2U));
  ASSERT_TRUE(vad_probe->wait_for_calls(2U));
  ASSERT_TRUE(asr_probe->wait_until_entered());

  core.accept_cleaned_frame(frame(1U, 4U));
  asr_probe->release();
  ASSERT_TRUE(asr_probe->wait_until_finished());
  EXPECT_TRUE(turn_sink.turns.empty());
  provider.reset();
}

TEST(SenseVoiceProviderTest, CorePublishesOneFinalFromSerialProviderDelivery)
{
  auto vad = std::make_unique<ScriptedSileroVad>(2U);
  auto asr = std::make_unique<RecordingSenseVoice>("开放时间早上9点至下午5点。");
  auto provider = std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::move(asr), wake_every_utterance());
  CollectingVoiceTurnSink turn_sink;
  SpeechInputCore core(*provider, turn_sink);

  core.accept_cleaned_frame(frame(1U));
  core.accept_cleaned_frame(frame(2U));
  core.finish_input();

  ASSERT_TRUE(turn_sink.wait_for_count(1U));
  EXPECT_EQ(turn_sink.turns.size(), 1U);
  EXPECT_EQ(turn_sink.turns.front().text, "开放时间早上9点至下午5点。");
  provider.reset();
}

}  // namespace
}  // namespace voice_nav_audio
