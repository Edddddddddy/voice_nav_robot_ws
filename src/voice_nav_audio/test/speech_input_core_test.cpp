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

#include <array>
#include <cstdio>
#include <memory>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "speech_input_core.hpp"

namespace voice_nav_audio
{
namespace
{

class CollectingVoiceTurnSink final : public VoiceTurnSink
{
public:
  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    turns.push_back(turn);
  }

  std::vector<VoiceTurnPublication> turns{};
};

class HappyPathRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame & frame,
    SpeechEventSink & sink) noexcept override
  {
    if (frame.audio_seq == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(frame));
    } else if (frame.audio_seq == 2U) {
      sink.on_speech_event(SpeechRecognitionEvent::activity(frame, active_scope_));
    } else if (frame.audio_seq == 3U) {
      sink.on_speech_event(
        SpeechRecognitionEvent::endpoint_final(frame, active_scope_, "前进一米", 0.75F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    active_scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override
  {
    if (scope.id == active_scope_.id) {
      active_scope_ = TurnScopeIdentity{};
    }
  }

private:
  TurnScopeIdentity active_scope_{};
};

class ManualRecognizer final : public SpeechRecognizerAdapter
{
public:
  void process_frame(
    const CleanedAudioFrame &,
    SpeechEventSink & sink) noexcept override
  {
    ++processed_frame_count;
    sink_ = &sink;
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    active_scope = scope;
    opened_scopes.push_back(scope);
  }

  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override
  {
    retired_scopes.push_back(scope);
    if (active_scope.id == scope.id) {
      active_scope = TurnScopeIdentity{};
    }
  }

  void emit(const SpeechRecognitionEvent & event) const noexcept
  {
    if (sink_ != nullptr) {
      sink_->on_speech_event(event);
    }
  }

  SpeechEventSink * sink_{nullptr};
  std::size_t processed_frame_count{0U};
  TurnScopeIdentity active_scope{};
  std::vector<TurnScopeIdentity> opened_scopes{};
  std::vector<TurnScopeIdentity> retired_scopes{};
};

class FailingVoiceIdentityGenerator final : public VoiceIdentityGenerator
{
public:
  bool generate(std::array<std::uint8_t, 16U> &) noexcept override
  {
    return false;
  }
};

class FixedVoiceIdentityGenerator final : public VoiceIdentityGenerator
{
public:
  explicit FixedVoiceIdentityGenerator(const std::array<std::uint8_t, 16U> bytes)
  : bytes_(bytes)
  {
  }

  bool generate(std::array<std::uint8_t, 16U> & bytes) noexcept override
  {
    bytes = bytes_;
    return true;
  }

private:
  std::array<std::uint8_t, 16U> bytes_{};
};

CleanedAudioFrame frame(const std::uint64_t generation, const std::uint64_t sequence)
{
  CleanedAudioFrame input{};
  input.audio_generation = generation;
  input.audio_seq = sequence;
  input.samples.fill(100);
  return input;
}

std::vector<std::string> probe_voice_identity()
{
  std::array<char, 128U> buffer{};
  std::string output{};
  const std::string command = std::string("\"") + SPEECH_INPUT_IDENTITY_PROBE + "\"";
  FILE * const process = popen(command.c_str(), "r");
  if (process == nullptr) {
    return {};
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), process) != nullptr) {
    output += buffer.data();
  }
  if (pclose(process) != 0) {
    return {};
  }

  std::vector<std::string> identities{};
  std::istringstream lines(output);
  for (std::string line; std::getline(lines, line); ) {
    identities.push_back(line);
  }
  return identities;
}

TEST(SpeechInputCoreTest, DoesNotReuseVoiceIdentityAcrossRealProcessRestarts)
{
  const auto first = probe_voice_identity();
  const auto second = probe_voice_identity();

  ASSERT_EQ(first.size(), 3U);
  ASSERT_EQ(second.size(), 3U);
  EXPECT_NE(first[0], second[0]);
  EXPECT_NE(first[1], second[1]);
  EXPECT_NE(first[2], second[2]);
  for (const auto & identity : first) {
    EXPECT_FALSE(identity.empty());
    EXPECT_LE(identity.size(), 36U);
  }
  for (const auto & identity : second) {
    EXPECT_FALSE(identity.empty());
    EXPECT_LE(identity.size(), 36U);
  }
}

TEST(SpeechInputCoreTest, FailsClosedWhenVoiceIdentityGenerationFails)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  FailingVoiceIdentityGenerator identity_generator;
  SpeechInputCore core(recognizer, sink, identity_generator);

  core.accept_cleaned_frame(frame(7U, 1U));

  EXPECT_EQ(recognizer.processed_frame_count, 0U);
  EXPECT_TRUE(sink.turns.empty());
  EXPECT_TRUE(recognizer.opened_scopes.empty());
}

TEST(SpeechInputCoreTest, ContinuesAudioSequenceAcrossGenerations)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto first = frame(7U, 41U);
  core.accept_cleaned_frame(first);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
  const auto first_scope = recognizer.active_scope;
  const auto next_generation = frame(8U, 42U);
  core.accept_cleaned_frame(next_generation);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(next_generation));

  EXPECT_EQ(recognizer.processed_frame_count, 2U);
  EXPECT_NE(recognizer.active_scope.id, 0U);
  EXPECT_NE(recognizer.active_scope.id, first_scope.id);
  EXPECT_EQ(recognizer.active_scope.audio_generation, 8U);
}

TEST(SpeechInputCoreTest, QuarantinesMalformedAndOldGenerationsWithoutRewindingSequence)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto first = frame(7U, 41U);
  core.accept_cleaned_frame(first);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
  const auto retired_scope = recognizer.active_scope;

  auto malformed = frame(8U, 42U);
  malformed.sample_rate_hz = 8000U;
  core.accept_cleaned_frame(malformed);
  const auto same_generation_valid = frame(8U, 43U);
  core.accept_cleaned_frame(same_generation_valid);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(same_generation_valid));
  EXPECT_EQ(recognizer.processed_frame_count, 1U);
  EXPECT_EQ(recognizer.active_scope.id, 0U);

  const auto recovered = frame(9U, 43U);
  core.accept_cleaned_frame(recovered);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(recovered));
  const auto recovered_scope = recognizer.active_scope;
  ASSERT_NE(recovered_scope.id, 0U);
  EXPECT_NE(recovered_scope.id, retired_scope.id);

  auto old_generation_malformed = frame(8U, 44U);
  old_generation_malformed.channels = 2U;
  core.accept_cleaned_frame(old_generation_malformed);
  EXPECT_EQ(recognizer.active_scope.id, recovered_scope.id);
  EXPECT_EQ(recognizer.processed_frame_count, 2U);

  const auto final_frame = frame(9U, 44U);
  core.accept_cleaned_frame(final_frame);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(final_frame, recovered_scope, "恢复后结果", 0.5F));
  ASSERT_EQ(sink.turns.size(), 1U);
  EXPECT_EQ(sink.turns.front().text, "恢复后结果");
}

TEST(SpeechInputCoreTest, RecoversFromSequenceRollbackGapAndReorderWithTheNextGlobalValue)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto first = frame(7U, 41U);
  core.accept_cleaned_frame(first);
  const auto continuous = frame(8U, 42U);
  core.accept_cleaned_frame(continuous);
  const auto rollback = frame(9U, 1U);
  core.accept_cleaned_frame(rollback);
  core.accept_cleaned_frame(frame(9U, 43U));
  EXPECT_EQ(recognizer.processed_frame_count, 2U);

  const auto after_rollback = frame(10U, 43U);
  core.accept_cleaned_frame(after_rollback);
  const auto gap = frame(10U, 45U);
  core.accept_cleaned_frame(gap);
  EXPECT_EQ(recognizer.processed_frame_count, 3U);

  const auto after_gap = frame(11U, 46U);
  core.accept_cleaned_frame(after_gap);
  core.accept_cleaned_frame(after_gap);
  EXPECT_EQ(recognizer.processed_frame_count, 4U);

  const auto after_reorder = frame(12U, 47U);
  core.accept_cleaned_frame(after_reorder);
  EXPECT_EQ(recognizer.processed_frame_count, 5U);
  EXPECT_TRUE(sink.turns.empty());
}

TEST(SpeechInputCoreTest, FailsClosedAfterTheGlobalSequenceReachesUint64Max)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto maximum = frame(7U, UINT64_MAX);
  core.accept_cleaned_frame(maximum);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(maximum));
  const auto scope = recognizer.active_scope;
  core.accept_cleaned_frame(frame(8U, 1U));
  core.accept_cleaned_frame(frame(9U, 2U));

  EXPECT_EQ(recognizer.processed_frame_count, 1U);
  EXPECT_EQ(recognizer.active_scope.id, 0U);
  EXPECT_EQ(recognizer.retired_scopes.size(), 1U);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(maximum, scope, "不得溢出", 0.5F));
  EXPECT_TRUE(sink.turns.empty());
}

TEST(SpeechInputCoreTest, UsesInjectedIdentityGeneratorForStableBoundedTurnIdentity)
{
  const std::array<std::uint8_t, 16U> bytes{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U,
    0x08U, 0x09U, 0x0aU, 0x0bU, 0x0cU, 0x0dU, 0x0eU, 0x0fU};
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  FixedVoiceIdentityGenerator identity_generator(bytes);
  SpeechInputCore core(recognizer, sink, identity_generator);
  const auto first = frame(7U, 1U);
  core.accept_cleaned_frame(first);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
  const auto scope = recognizer.active_scope;
  const auto final_frame = frame(7U, 2U);
  core.accept_cleaned_frame(final_frame);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(final_frame, scope, "确定身份", 0.5F));

  ASSERT_EQ(sink.turns.size(), 1U);
  EXPECT_EQ(sink.turns.front().voice_instance_id, "000102030405060708090a0b0c0d0e0f");
  EXPECT_EQ(sink.turns.front().session_id, "000102030405060708090a0b0c0d0e0f");
  EXPECT_EQ(sink.turns.front().turn_id, "t00010203040506070000000000000001");
  EXPECT_EQ(sink.turns.front().voice_instance_id.size(), 32U);
  EXPECT_EQ(sink.turns.front().session_id.size(), 32U);
  EXPECT_EQ(sink.turns.front().turn_id.size(), 33U);
}

TEST(SpeechInputCoreTest, PublishesExactlyOneCommandForAnAcceptedFinalTurn)
{
  HappyPathRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  core.accept_cleaned_frame(frame(7U, 1U));
  core.accept_cleaned_frame(frame(7U, 2U));
  core.accept_cleaned_frame(frame(7U, 3U));

  ASSERT_EQ(sink.turns.size(), 1U);
  const auto & turn = sink.turns.front();
  EXPECT_FALSE(turn.voice_instance_id.empty());
  EXPECT_EQ(turn.voice_seq, 1U);
  EXPECT_FALSE(turn.session_id.empty());
  EXPECT_FALSE(turn.turn_id.empty());
  EXPECT_EQ(turn.kind, VoiceTurnKind::kCommand);
  EXPECT_EQ(turn.text, "前进一米");
  EXPECT_FLOAT_EQ(turn.confidence, 0.75F);
  EXPECT_FALSE(turn.during_playback);
}

TEST(SpeechInputCoreTest, PublishesScriptedStopWithTheCompletedVoiceTurnIdentity)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto wake = frame(7U, 1U);
  core.accept_cleaned_frame(wake);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(wake));
  const auto scope = recognizer.active_scope;
  const auto final = frame(7U, 2U);
  core.accept_cleaned_frame(final);
  recognizer.emit(SpeechRecognitionEvent::endpoint_final(
      final, scope, "停止", 1.0F, VoiceTurnKind::kStop));

  ASSERT_EQ(sink.turns.size(), 1U);
  const auto & turn = sink.turns.front();
  EXPECT_EQ(turn.kind, VoiceTurnKind::kStop);
  EXPECT_EQ(turn.voice_seq, 1U);
  EXPECT_EQ(turn.session_id, scope.session_id);
  EXPECT_EQ(turn.turn_id, scope.turn_id);
  EXPECT_EQ(turn.text, "停止");
}

TEST(SpeechInputCoreTest, RejectsEveryInvalidFinalAndTerminalEventWithoutAPublication)
{
  struct InvalidFinal
  {
    std::string text;
    float confidence;
  };
  const std::vector<InvalidFinal> invalid_finals{
    {"", 0.5F},
    {std::string("\xc3\x28", 2U), 0.5F},
    {std::string(513U, 'a'), 0.5F},
    {"有效", std::numeric_limits<float>::quiet_NaN()},
    {"有效", std::numeric_limits<float>::infinity()},
    {"有效", -0.01F},
    {"有效", 1.01F},
  };
  for (const auto & invalid : invalid_finals) {
    ManualRecognizer recognizer;
    CollectingVoiceTurnSink sink;
    SpeechInputCore core(recognizer, sink);
    const auto first = frame(7U, 1U);
    core.accept_cleaned_frame(first);
    recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
    const auto second = frame(7U, 2U);
    core.accept_cleaned_frame(second);
    recognizer.emit(
      SpeechRecognitionEvent::endpoint_final(second, recognizer.active_scope, invalid.text,
        invalid.confidence));

    EXPECT_TRUE(sink.turns.empty());
    EXPECT_EQ(recognizer.retired_scopes.size(), 1U);
  }

  for (const auto terminal_kind : {SpeechEventKind::kTimeout, SpeechEventKind::kFailure}) {
    ManualRecognizer recognizer;
    CollectingVoiceTurnSink sink;
    SpeechInputCore core(recognizer, sink);
    const auto first = frame(8U, 1U);
    core.accept_cleaned_frame(first);
    recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
    const auto second = frame(8U, 2U);
    core.accept_cleaned_frame(second);
    SpeechRecognitionEvent terminal{};
    terminal.kind = terminal_kind;
    terminal.audio_generation = second.audio_generation;
    terminal.audio_seq = second.audio_seq;
    terminal.scope = recognizer.active_scope;
    recognizer.emit(terminal);
    recognizer.emit(
      SpeechRecognitionEvent::endpoint_final(second, terminal.scope, "不得发布", 0.5F));

    EXPECT_TRUE(sink.turns.empty());
    EXPECT_EQ(recognizer.retired_scopes.size(), 1U);
  }

  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);
  const auto only_frame = frame(9U, 1U);
  core.accept_cleaned_frame(only_frame);
  SpeechRecognitionEvent wake_miss{};
  wake_miss.kind = SpeechEventKind::kWakeMiss;
  wake_miss.audio_generation = only_frame.audio_generation;
  wake_miss.audio_seq = only_frame.audio_seq;
  recognizer.emit(wake_miss);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(only_frame, TurnScopeIdentity{}, "不得发布", 0.5F));
  EXPECT_TRUE(sink.turns.empty());
}

TEST(SpeechInputCoreTest, RetiresScopesForLatestWakeAndEveryContinuityFence)
{
  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);

  const auto first = frame(7U, 1U);
  core.accept_cleaned_frame(first);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
  const auto retired_by_new_wake = recognizer.active_scope;
  const auto second = frame(7U, 2U);
  core.accept_cleaned_frame(second);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(second));
  const auto current_scope = recognizer.active_scope;
  EXPECT_NE(retired_by_new_wake.id, current_scope.id);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(second, retired_by_new_wake, "旧 wake", 0.5F));
  EXPECT_TRUE(sink.turns.empty());

  const auto next_generation = frame(8U, 3U);
  core.accept_cleaned_frame(next_generation);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(second, current_scope, "旧 generation", 0.5F));
  EXPECT_TRUE(sink.turns.empty());
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(next_generation));
  const auto gap_scope = recognizer.active_scope;
  const auto before_gap = frame(8U, 4U);
  core.accept_cleaned_frame(before_gap);
  core.accept_cleaned_frame(frame(8U, 6U));
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(before_gap, gap_scope, "gap", 0.5F));
  EXPECT_TRUE(sink.turns.empty());

  const auto reordered_generation = frame(9U, 7U);
  core.accept_cleaned_frame(reordered_generation);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(reordered_generation));
  const auto reorder_scope = recognizer.active_scope;
  const auto before_reorder = frame(9U, 8U);
  core.accept_cleaned_frame(before_reorder);
  core.accept_cleaned_frame(before_reorder);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(before_reorder, reorder_scope, "reordered", 0.5F));
  EXPECT_TRUE(sink.turns.empty());

  const auto final_generation = frame(10U, 9U);
  core.accept_cleaned_frame(final_generation);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(final_generation));
  const auto final_scope = recognizer.active_scope;
  const auto final_frame = frame(10U, 10U);
  core.accept_cleaned_frame(final_frame);
  const auto final_event =
    SpeechRecognitionEvent::endpoint_final(final_frame, final_scope, "当前结果", 0.5F);
  recognizer.emit(final_event);
  recognizer.emit(final_event);

  ASSERT_EQ(sink.turns.size(), 1U);
  EXPECT_EQ(sink.turns.front().text, "当前结果");
  EXPECT_EQ(recognizer.opened_scopes.size(), 5U);
}

TEST(SpeechInputCoreTest, RejectsMalformedFramesUntilAStrictlyNewerGeneration)
{
  for (const bool non_mono : {false, true}) {
    ManualRecognizer recognizer;
    CollectingVoiceTurnSink sink;
    SpeechInputCore core(recognizer, sink);
    const auto first = frame(12U, 1U);
    core.accept_cleaned_frame(first);
    recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
    const auto old_scope = recognizer.active_scope;

    auto malformed = frame(12U, 2U);
    if (non_mono) {
      malformed.channels = 2U;
    } else {
      malformed.sample_rate_hz = 8000U;
    }
    core.accept_cleaned_frame(malformed);
    recognizer.emit(
      SpeechRecognitionEvent::endpoint_final(first, old_scope, "错误 frame", 0.5F));
    EXPECT_TRUE(sink.turns.empty());
    EXPECT_EQ(recognizer.retired_scopes.size(), 1U);

    const auto blocked_same_generation = frame(12U, 3U);
    core.accept_cleaned_frame(blocked_same_generation);
    recognizer.emit(SpeechRecognitionEvent::wake_accepted(blocked_same_generation));
    EXPECT_EQ(recognizer.opened_scopes.size(), 1U);

    const auto recovery_frame = frame(13U, 3U);
    core.accept_cleaned_frame(recovery_frame);
    recognizer.emit(SpeechRecognitionEvent::wake_accepted(recovery_frame));
    const auto recovered_scope = recognizer.active_scope;
    EXPECT_NE(recovered_scope.id, old_scope.id);

    recognizer.emit(SpeechRecognitionEvent::wake_accepted(first));
    recognizer.emit(
      SpeechRecognitionEvent::endpoint_final(first, old_scope, "旧结果", 0.5F));
    auto stale_malformed = frame(12U, 4U);
    stale_malformed.channels = 2U;
    core.accept_cleaned_frame(stale_malformed);
    EXPECT_EQ(recognizer.active_scope.id, recovered_scope.id);
    EXPECT_TRUE(sink.turns.empty());

    const auto final_frame = frame(13U, 4U);
    core.accept_cleaned_frame(final_frame);
    recognizer.emit(
      SpeechRecognitionEvent::endpoint_final(final_frame, recovered_scope, "恢复后结果", 0.5F));
    ASSERT_EQ(sink.turns.size(), 1U);
    EXPECT_EQ(sink.turns.front().text, "恢复后结果");
  }

  ManualRecognizer recognizer;
  CollectingVoiceTurnSink sink;
  SpeechInputCore core(recognizer, sink);
  const auto current_frame = frame(7U, 1U);
  core.accept_cleaned_frame(current_frame);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(current_frame));

  auto higher_malformed = frame(8U, 2U);
  higher_malformed.sample_rate_hz = 8000U;
  core.accept_cleaned_frame(higher_malformed);
  const auto same_generation_valid = frame(8U, 3U);
  core.accept_cleaned_frame(same_generation_valid);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(same_generation_valid));
  EXPECT_EQ(recognizer.opened_scopes.size(), 1U);
  EXPECT_TRUE(sink.turns.empty());

  const auto strictly_newer = frame(9U, 3U);
  core.accept_cleaned_frame(strictly_newer);
  recognizer.emit(SpeechRecognitionEvent::wake_accepted(strictly_newer));
  const auto recovered_scope = recognizer.active_scope;
  const auto recovered_final = frame(9U, 4U);
  core.accept_cleaned_frame(recovered_final);
  recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(recovered_final, recovered_scope, "严格更高 generation",
      0.5F));
  ASSERT_EQ(sink.turns.size(), 1U);
  EXPECT_EQ(sink.turns.front().text, "严格更高 generation");
}

TEST(SpeechInputCoreTest, CreatesDistinctBoundedVoiceLifetimesAndStrictSequences)
{
  ManualRecognizer first_recognizer;
  CollectingVoiceTurnSink first_sink;
  SpeechInputCore first_core(first_recognizer, first_sink);
  const auto first_frame = frame(11U, 1U);
  first_core.accept_cleaned_frame(first_frame);
  first_recognizer.emit(SpeechRecognitionEvent::wake_accepted(first_frame));
  const auto first_scope = first_recognizer.active_scope;
  const auto first_final_frame = frame(11U, 2U);
  first_core.accept_cleaned_frame(first_final_frame);
  first_recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(first_final_frame, first_scope, "第一句", 0.5F));

  const auto second_frame = frame(11U, 3U);
  first_core.accept_cleaned_frame(second_frame);
  first_recognizer.emit(SpeechRecognitionEvent::wake_accepted(second_frame));
  const auto second_scope = first_recognizer.active_scope;
  const auto second_final_frame = frame(11U, 4U);
  first_core.accept_cleaned_frame(second_final_frame);
  const auto old_result =
    SpeechRecognitionEvent::endpoint_final(second_final_frame, second_scope, "第二句", 0.5F);
  first_recognizer.emit(old_result);

  ASSERT_EQ(first_sink.turns.size(), 2U);
  EXPECT_EQ(first_sink.turns[0].voice_seq, 1U);
  EXPECT_EQ(first_sink.turns[1].voice_seq, 2U);
  EXPECT_EQ(first_sink.turns[0].voice_instance_id, first_sink.turns[1].voice_instance_id);
  EXPECT_EQ(first_sink.turns[0].session_id, first_sink.turns[1].session_id);
  EXPECT_NE(first_sink.turns[0].turn_id, first_sink.turns[1].turn_id);

  ManualRecognizer new_recognizer;
  CollectingVoiceTurnSink new_sink;
  SpeechInputCore new_core(new_recognizer, new_sink);
  const auto new_frame = frame(11U, 1U);
  new_core.accept_cleaned_frame(new_frame);
  new_recognizer.emit(SpeechRecognitionEvent::wake_accepted(new_frame));
  new_core.accept_cleaned_frame(frame(11U, 2U));
  new_core.accept_cleaned_frame(frame(11U, 3U));
  new_core.accept_cleaned_frame(frame(11U, 4U));
  new_recognizer.emit(old_result);
  const auto colliding_old_instance_result =
    SpeechRecognitionEvent::endpoint_final(frame(11U, 4U), first_scope, "旧实例", 0.5F);
  new_recognizer.emit(colliding_old_instance_result);
  EXPECT_TRUE(new_sink.turns.empty());
  new_recognizer.emit(
    SpeechRecognitionEvent::endpoint_final(
      frame(11U, 4U), new_recognizer.active_scope, "新 lifetime", 0.5F));

  ASSERT_EQ(new_sink.turns.size(), 1U);
  EXPECT_EQ(new_sink.turns.front().voice_seq, 1U);
  EXPECT_NE(new_sink.turns.front().voice_instance_id, first_sink.turns.front().voice_instance_id);
  EXPECT_LE(new_sink.turns.front().voice_instance_id.size(), 36U);
  EXPECT_LE(new_sink.turns.front().session_id.size(), 36U);
  EXPECT_LE(new_sink.turns.front().turn_id.size(), 36U);
}

}  // namespace
}  // namespace voice_nav_audio
