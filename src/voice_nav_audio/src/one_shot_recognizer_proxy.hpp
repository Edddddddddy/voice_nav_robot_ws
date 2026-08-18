// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_AUDIO__ONE_SHOT_RECOGNIZER_PROXY_HPP_
#define VOICE_NAV_AUDIO__ONE_SHOT_RECOGNIZER_PROXY_HPP_

#include <memory>

#include "speech_input_core.hpp"

namespace voice_nav_audio
{

// 每个回合返回一个已 arm 的 one-shot recognizer；代理本身跨回合存活。
class OneShotRecognizerFactory
{
public:
  virtual ~OneShotRecognizerFactory() = default;

  [[nodiscard]] virtual std::unique_ptr<SpeechRecognizerAdapter> create_armed() = 0;
};

// 稳定的 SpeechInputCore recognizer owner。旧 child 只在下一轮开始时关闭并释放。
class OneShotRecognizerProxy final : public SpeechRecognizerAdapter
{
public:
  explicit OneShotRecognizerProxy(OneShotRecognizerFactory & factory) noexcept;
  ~OneShotRecognizerProxy() override;

  OneShotRecognizerProxy(const OneShotRecognizerProxy &) = delete;
  OneShotRecognizerProxy & operator=(const OneShotRecognizerProxy &) = delete;

  [[nodiscard]] bool start_turn() noexcept;

  void shutdown() noexcept override;
  void finish_input() noexcept override;
  void process_frame(
    const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept override;
  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept override;
  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept override;

private:
  void retire_child() noexcept;

  OneShotRecognizerFactory & factory_;
  std::unique_ptr<SpeechRecognizerAdapter> child_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__ONE_SHOT_RECOGNIZER_PROXY_HPP_
