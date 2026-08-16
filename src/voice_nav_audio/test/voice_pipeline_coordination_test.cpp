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
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "speech_input_core.hpp"
#include "speech_output_core.hpp"
#include "voice_pipeline_coordination.hpp"

namespace voice_nav_audio
{
namespace
{

enum class FakeRecognitionMode
{
  OrdinaryWake,
  PrivilegedStop,
  ScopedStop,
};

class FakeRecognizer final : public SpeechRecognizerAdapter
{
public:
  explicit FakeRecognizer(
    const FakeRecognitionMode mode = FakeRecognitionMode::OrdinaryWake,
    std::string final_text = {},
    const bool stale_stop_generation = false,
    const bool duplicate_stop_event = false)
  : mode_(mode), final_text_(std::move(final_text)),
    stale_stop_generation_(stale_stop_generation), duplicate_stop_event_(duplicate_stop_event)
  {
  }

  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U && mode_ == FakeRecognitionMode::OrdinaryWake) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 1U && mode_ == FakeRecognitionMode::PrivilegedStop) {
      auto event_frame = frame;
      if (stale_stop_generation_) {
        event_frame.audio_generation = frame.audio_generation - 1U;
      }
      const auto event = SpeechRecognitionEvent::endpoint_final(
        event_frame, TurnScopeIdentity{}, final_text_, 1.0F, VoiceTurnKind::kStop);
      sink.on_speech_event(event);
      if (duplicate_stop_event_) {
        sink.on_speech_event(event);
      }
    } else if (frame.audio_seq == 1U && mode_ == FakeRecognitionMode::ScopedStop) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U && mode_ == FakeRecognitionMode::ScopedStop) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
        frame, active_scope_, final_text_, 1.0F, VoiceTurnKind::kStop));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    active_scope_ = scope;
    opened_scopes.push_back(scope);
  }

  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override
  {
    retired_scopes.push_back(scope);
    active_scope_ = TurnScopeIdentity{};
  }

  std::vector<TurnScopeIdentity> opened_scopes{};
  std::vector<TurnScopeIdentity> retired_scopes{};

private:
  FakeRecognitionMode mode_{FakeRecognitionMode::OrdinaryWake};
  std::string final_text_{};
  bool stale_stop_generation_{false};
  bool duplicate_stop_event_{false};
  TurnScopeIdentity active_scope_{};
};

class FakeTts final : public TtsAdapter
{
public:
  explicit FakeTts(std::vector<std::string> * const events = nullptr)
  : events_(events)
  {
  }

  void start(const TtsRequest & request, TtsSink & sink) noexcept override
  {
    request_ = request;
    sink_ = &sink;
    ++start_count;
  }

  void cancel(const std::uint64_t scope_id) noexcept override
  {
    if (events_ != nullptr) {
      events_->emplace_back("playback_fence");
    }
    cancelled_scope_ids.push_back(scope_id);
  }

  bool emit_pcm() noexcept
  {
    std::array<Sample, 147U> pcm{};
    pcm.fill(1000);
    return sink_ != nullptr && sink_->on_pcm(
      request_.scope_id, 22050U, 1U, pcm.data(), pcm.size());
  }

  void complete() noexcept
  {
    if (sink_ != nullptr) {
      sink_->on_complete(request_.scope_id);
    }
  }

  TtsRequest request_{};
  TtsSink * sink_{nullptr};
  std::size_t start_count{0U};
  std::vector<std::uint64_t> cancelled_scope_ids{};

private:
  std::vector<std::string> * events_{nullptr};
};

class FakeStopMissionPort final : public StopMissionPort
{
public:
  explicit FakeStopMissionPort(std::vector<std::string> * const events = nullptr)
  : events_(events)
  {
  }

  void request(
    const StopMissionRequest & request,
    StopMissionResponseSink & response_sink) noexcept override
  {
    requests.push_back(request);
    response_sink_ = &response_sink;
    if (events_ != nullptr) {
      events_->emplace_back("stop_request");
    }
  }

  void respond(const StopMissionResponse & response) noexcept
  {
    if (response_sink_ != nullptr) {
      response_sink_->on_response(response);
    }
  }

  std::vector<StopMissionRequest> requests{};

private:
  StopMissionResponseSink * response_sink_{nullptr};
  std::vector<std::string> * events_{nullptr};
};

class CollectingSpeechObserver final : public SpeechOutputObserver
{
public:
  explicit CollectingSpeechObserver(std::vector<std::string> * const events = nullptr)
  : events_(events)
  {
  }

  void on_played(const std::uint64_t, const std::uint64_t) noexcept override {}

  void on_result(const SpeechResult & result) noexcept override
  {
    results.push_back(result);
    if (events_ != nullptr && result.code == SpeechResultCode::BargedIn) {
      events_->emplace_back("barged_in");
    }
  }

  std::vector<SpeechResult> results{};

private:
  std::vector<std::string> * events_{nullptr};
};

class CollectingTurnSink final : public VoiceTurnSink
{
public:
  explicit CollectingTurnSink(std::vector<std::string> * const events = nullptr)
  : events_(events)
  {
  }

  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    turns.push_back(turn);
    if (events_ != nullptr) {
      events_->emplace_back("turn_publish");
    }
  }

  std::vector<VoiceTurnPublication> turns{};

private:
  std::vector<std::string> * events_{nullptr};
};

CleanedAudioFrame frame(const std::uint64_t sequence, const std::uint64_t generation = 1U)
{
  CleanedAudioFrame result{};
  result.audio_generation = generation;
  result.audio_seq = sequence;
  return result;
}

TEST(
  VoicePipelineCoordinationTest,
  ActiveAllowBargeInOrdinaryWakeBargesExactlyOnceWithoutStopMission)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  const auto admission = output.start(SpeechGoal{
      "voice-instance", 1U, "session", "speak-turn", SpeechPriority::Normal,
      "正在播报", true});
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(output.begin_synthesis(admission.scope_id));
  ASSERT_TRUE(tts.emit_pcm());

  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer;
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));

  ASSERT_EQ(speech_observer.results.size(), 1U);
  EXPECT_EQ(speech_observer.results.front().scope_id, admission.scope_id);
  EXPECT_EQ(speech_observer.results.front().code, SpeechResultCode::BargedIn);
  EXPECT_TRUE(stop_port.requests.empty());
  EXPECT_EQ(events, (std::vector<std::string>{"playback_fence", "barged_in"}));
  ASSERT_EQ(recognizer.opened_scopes.size(), 1U);
  EXPECT_TRUE(recognizer.retired_scopes.empty());

  tts.complete();
  EXPECT_EQ(speech_observer.results.size(), 1U);

  std::array<Sample, AudioEngine::kFrameSamples> rendered{};
  engine.process_callback(nullptr, rendered.data(), rendered.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(rendered.cbegin(), rendered.cend(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(
  VoicePipelineCoordinationTest,
  OrdinaryWakeDoesNotInterruptOrOpenScopeWhenBargeInIsDisabled)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  const auto admission = output.start(SpeechGoal{
      "voice-instance", 1U, "session", "speak-turn", SpeechPriority::Normal,
      "正在播报", false});
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(output.begin_synthesis(admission.scope_id));
  ASSERT_TRUE(tts.emit_pcm());

  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer;
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));

  EXPECT_TRUE(stop_port.requests.empty());
  EXPECT_TRUE(turn_sink.turns.empty());
  EXPECT_TRUE(speech_observer.results.empty());
  EXPECT_TRUE(tts.cancelled_scope_ids.empty());
  EXPECT_TRUE(recognizer.opened_scopes.empty());
  EXPECT_TRUE(recognizer.retired_scopes.empty());
  EXPECT_TRUE(events.empty());
}

void assert_stop_case(const std::string & text, const bool allow_barge_in)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  const auto admission = output.start(SpeechGoal{
      "voice-instance", 1U, "session", "speak-turn", SpeechPriority::Normal,
      "正在播报", allow_barge_in});
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(output.begin_synthesis(admission.scope_id));
  ASSERT_TRUE(tts.emit_pcm());

  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer(FakeRecognitionMode::PrivilegedStop, text);
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));

  ASSERT_EQ(stop_port.requests.size(), 1U);
  ASSERT_EQ(turn_sink.turns.size(), 1U);
  const auto & turn = turn_sink.turns.front();
  const auto & request = stop_port.requests.front();
  EXPECT_EQ(turn.kind, VoiceTurnKind::kStop);
  EXPECT_EQ(turn.text, text);
  EXPECT_TRUE(turn.during_playback);
  EXPECT_EQ(request.request_id, turn.turn_id);
  EXPECT_EQ(request.source_instance_id, turn.voice_instance_id);
  EXPECT_EQ(request.source_seq, turn.voice_seq);
  EXPECT_EQ(request.reason, "voice_stop");
  ASSERT_EQ(tts.cancelled_scope_ids.size(), 1U);
  EXPECT_EQ(tts.cancelled_scope_ids.front(), admission.scope_id);
  ASSERT_EQ(speech_observer.results.size(), 1U);
  EXPECT_EQ(speech_observer.results.front().code, SpeechResultCode::BargedIn);
  EXPECT_EQ(events, (std::vector<std::string>{
      "playback_fence", "barged_in", "stop_request", "turn_publish"}));

  stop_port.respond(StopMissionResponse{StopMissionCode::Applied, true});
  stop_port.respond(StopMissionResponse{StopMissionCode::Duplicate, true});
  stop_port.respond(StopMissionResponse{StopMissionCode::SafetyFault, false});
  stop_port.respond(StopMissionResponse{StopMissionCode::TransportFailure, false});
  stop_port.respond(StopMissionResponse{StopMissionCode::Timeout, false});
  tts.complete();
  EXPECT_EQ(stop_port.requests.size(), 1U);
  EXPECT_EQ(turn_sink.turns.size(), 1U);
  EXPECT_EQ(speech_observer.results.size(), 1U);
  EXPECT_EQ(events, (std::vector<std::string>{
      "playback_fence", "barged_in", "stop_request", "turn_publish"}));

  std::array<Sample, AudioEngine::kFrameSamples> rendered{};
  engine.process_callback(nullptr, rendered.data(), rendered.size(), CallbackStatus{});
  EXPECT_TRUE(std::all_of(rendered.cbegin(), rendered.cend(), [](const Sample sample) {
      return sample == 0;
    }));
}

TEST(
  VoicePipelineCoordinationTest,
  DuplicatePrivilegedStopCallbackKeepsOneBoundedIdentityAndOneRequest)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  const auto admission = output.start(SpeechGoal{
      "voice-instance", 1U, "session", "speak-turn", SpeechPriority::Normal,
      "正在播报", false});
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(output.begin_synthesis(admission.scope_id));
  ASSERT_TRUE(tts.emit_pcm());

  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer(
    FakeRecognitionMode::PrivilegedStop, "小智停止", false, true);
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));

  ASSERT_EQ(stop_port.requests.size(), 1U);
  ASSERT_EQ(turn_sink.turns.size(), 1U);
  ASSERT_EQ(speech_observer.results.size(), 1U);
  EXPECT_EQ(stop_port.requests.front().request_id, turn_sink.turns.front().turn_id);
  EXPECT_EQ(events, (std::vector<std::string>{
      "playback_fence", "barged_in", "stop_request", "turn_publish"}));
}

TEST(
  VoicePipelineCoordinationTest,
  ApprovedStopTextsFenceAndRequestExactlyOnceForEitherBargeInPolicy)
{
  for (const std::string & text : {std::string{"小智停止"}, std::string{"紧急停止"}}) {
    for (const bool allow_barge_in : {false, true}) {
      SCOPED_TRACE(text + (allow_barge_in ? " allow" : " deny"));
      assert_stop_case(text, allow_barge_in);
    }
  }
}

TEST(
  VoicePipelineCoordinationTest,
  InvalidStopTextWithScopedFinalFailsClosedWithoutSideEffects)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer(FakeRecognitionMode::ScopedStop, "停止");
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));
  input.accept_cleaned_frame(frame(2U));

  EXPECT_EQ(recognizer.opened_scopes.size(), 1U);
  EXPECT_EQ(recognizer.retired_scopes.size(), 1U);
  EXPECT_TRUE(stop_port.requests.empty());
  EXPECT_TRUE(turn_sink.turns.empty());
  EXPECT_TRUE(speech_observer.results.empty());
  EXPECT_TRUE(tts.cancelled_scope_ids.empty());
  EXPECT_TRUE(events.empty());
}

TEST(
  VoicePipelineCoordinationTest,
  StalePrivilegedStopGenerationHasNoSideEffects)
{
  AudioEngine engine;
  std::vector<std::string> events;
  FakeTts tts(&events);
  CollectingSpeechObserver speech_observer(&events);
  SpeechOutputCore output(engine, tts, speech_observer);
  const auto admission = output.start(SpeechGoal{
      "voice-instance", 1U, "session", "speak-turn", SpeechPriority::Normal,
      "正在播报", true});
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(output.begin_synthesis(admission.scope_id));
  ASSERT_TRUE(tts.emit_pcm());

  FakeStopMissionPort stop_port(&events);
  CollectingTurnSink turn_sink(&events);
  VoicePipelineCoordination coordination(output, stop_port);
  FakeRecognizer recognizer(
    FakeRecognitionMode::PrivilegedStop, "小智停止", true);
  SpeechInputCore input(recognizer, turn_sink, default_voice_identity_generator(), &coordination);

  input.accept_cleaned_frame(frame(1U));

  EXPECT_TRUE(stop_port.requests.empty());
  EXPECT_TRUE(turn_sink.turns.empty());
  EXPECT_TRUE(speech_observer.results.empty());
  EXPECT_TRUE(tts.cancelled_scope_ids.empty());
  EXPECT_TRUE(events.empty());
}

}  // namespace
}  // namespace voice_nav_audio
