// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_
#define VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "dsp_pipeline.hpp"
#include "one_shot_recognizer_proxy.hpp"
#include "rclcpp/executor.hpp"
#include "voice_pipeline.hpp"

namespace voice_nav_audio
{

enum class ContinuousVadPumpResult
{
  kCapturing,
  kFailed,
};

// Package-private long-lived VAD owner. Capture stays open across turns;
// OneShotRecognizerProxy replaces only its child at the next control pump.
class ContinuousVadSession final
{
public:
  static constexpr std::size_t kReadinessWarmupFrames{3U};

  ContinuousVadSession(
    OneShotRecognizerFactory & recognizer_factory,
    DspAdapter & dsp_adapter,
    std::unique_ptr<TtsAdapter> tts,
    FullDuplexAudioDevice * device = nullptr,
    SpeechOutputTraceSink * trace = nullptr,
    StopMissionPort * stop_port = nullptr);
  ~ContinuousVadSession();

  ContinuousVadSession(const ContinuousVadSession &) = delete;
  ContinuousVadSession & operator=(const ContinuousVadSession &) = delete;

  [[nodiscard]] ContinuousVadPumpResult pump() noexcept;
  void stop() noexcept;
  void add_to_executor(rclcpp::Executor & executor);
  void remove_from_executor(rclcpp::Executor & executor);

private:
  std::unique_ptr<OneShotRecognizerProxy> recognizer_{};
  OneShotRecognizerProxy * recognizer_view_{nullptr};
  std::unique_ptr<DspPipeline> dsp_{};
  std::unique_ptr<VoicePipeline> pipeline_{};
  bool input_publisher_active_{false};
  std::size_t readiness_warmup_frames_{0U};
  std::uint64_t next_audio_seq_{1U};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__CONTINUOUS_VAD_SESSION_HPP_
