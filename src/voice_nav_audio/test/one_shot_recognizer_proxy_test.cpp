// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "one_shot_recognizer_proxy.hpp"
#include "speech_input_core.hpp"

namespace voice_nav_audio
{
namespace
{

class CollectingSink final : public VoiceTurnSink
{
public:
  void publish(const VoiceTurnPublication & turn) noexcept override
  {
    turns.push_back(turn);
  }

  std::vector<VoiceTurnPublication> turns{};
};

class FixedIdentityGenerator final : public VoiceIdentityGenerator
{
public:
  bool generate(std::array<std::uint8_t, 16U> & bytes) noexcept override
  {
    bytes = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
      8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U};
    return true;
  }
};

CleanedAudioFrame frame(const std::uint64_t generation, const std::uint64_t sequence)
{
  CleanedAudioFrame value{};
  value.audio_generation = generation;
  value.audio_seq = sequence;
  value.samples.fill(100);
  return value;
}

struct ChildTrace
{
  SpeechEventSink * sink{nullptr};
  TurnScopeIdentity scope{};
  TurnScopeIdentity retired_scope{};
  std::size_t frame_count{0U};
  std::size_t shutdown_count{0U};
};

class FakeOneShotChild final : public SpeechRecognizerAdapter
{
public:
  explicit FakeOneShotChild(std::shared_ptr<ChildTrace> trace)
  : trace_(std::move(trace))
  {
  }

  void shutdown() noexcept override
  {
    ++trace_->shutdown_count;
  }

  void process_frame(
    const CleanedAudioFrame & input, SpeechEventSink & sink) noexcept override
  {
    trace_->sink = &sink;
    ++trace_->frame_count;
    if (trace_->frame_count == 1U) {
      sink.on_speech_event(SpeechRecognitionEvent::wake_accepted(input));
    } else if (trace_->frame_count == 2U) {
      sink.on_speech_event(SpeechRecognitionEvent::endpoint_final(
          input, trace_->scope, "前进半米", 1.0F));
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override
  {
    trace_->scope = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override
  {
    trace_->retired_scope = scope;
    trace_->scope = TurnScopeIdentity{};
  }

private:
  std::shared_ptr<ChildTrace> trace_;
};

class FakeFactory final : public OneShotRecognizerFactory
{
public:
  std::unique_ptr<SpeechRecognizerAdapter> create_armed() override
  {
    auto trace = std::make_shared<ChildTrace>();
    traces.push_back(trace);
    return std::make_unique<FakeOneShotChild>(std::move(trace));
  }

  std::vector<std::shared_ptr<ChildTrace>> traces{};
};

class ThrowingFactory final : public OneShotRecognizerFactory
{
public:
  std::unique_ptr<SpeechRecognizerAdapter> create_armed() override
  {
    throw std::runtime_error("fake recognizer construction failed");
  }
};

}  // namespace

TEST(OneShotRecognizerProxyTest, KeepsVoiceIdentityAndSequenceAcrossRetiredChildren)
{
  FakeFactory factory;
  OneShotRecognizerProxy proxy(factory);
  CollectingSink sink;
  FixedIdentityGenerator identity;
  SpeechInputCore core(proxy, sink, identity);

  ASSERT_TRUE(proxy.start_turn());
  core.accept_cleaned_frame(frame(1U, 1U));
  core.accept_cleaned_frame(frame(1U, 2U));
  ASSERT_EQ(sink.turns.size(), 1U);

  const auto retired_scope = factory.traces.front()->retired_scope;
  ASSERT_TRUE(proxy.start_turn());
  ASSERT_EQ(factory.traces.size(), 2U);
  EXPECT_EQ(factory.traces.front()->shutdown_count, 1U);

  factory.traces.front()->sink->on_speech_event(
    SpeechRecognitionEvent::endpoint_final(
      frame(2U, 3U), retired_scope, "旧 child", 1.0F));
  EXPECT_EQ(sink.turns.size(), 1U);

  core.accept_cleaned_frame(frame(2U, 3U));
  core.accept_cleaned_frame(frame(2U, 4U));

  ASSERT_EQ(sink.turns.size(), 2U);
  EXPECT_EQ(sink.turns[0].voice_instance_id, sink.turns[1].voice_instance_id);
  EXPECT_EQ(sink.turns[0].session_id, sink.turns[1].session_id);
  EXPECT_EQ(sink.turns[0].voice_seq, 1U);
  EXPECT_EQ(sink.turns[1].voice_seq, 2U);
  EXPECT_NE(sink.turns[0].turn_id, sink.turns[1].turn_id);
}

TEST(OneShotRecognizerProxyTest, FactoryConstructionFailureFailsClosed)
{
  ThrowingFactory factory;
  OneShotRecognizerProxy proxy(factory);

  EXPECT_FALSE(proxy.start_turn());
}

}  // namespace voice_nav_audio
