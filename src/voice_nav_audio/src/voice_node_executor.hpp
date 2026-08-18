// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_AUDIO__VOICE_NODE_EXECUTOR_HPP_
#define VOICE_NAV_AUDIO__VOICE_NODE_EXECUTOR_HPP_

#include <atomic>
#include <chrono>
#include <thread>

#include "rclcpp/executor.hpp"

namespace voice_nav_audio
{

// Package-private owner for the installed voice_node executor thread. It is
// deliberately small so every exception path has the same stop-and-join
// guarantee.
class VoiceNodeExecutorGuard final
{
public:
  explicit VoiceNodeExecutorGuard(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {
      while (!stop_requested_.load(std::memory_order_acquire)) {
        executor_.spin_some(std::chrono::milliseconds(10));
      }
    })
  {
  }

  ~VoiceNodeExecutorGuard()
  {
    stop();
  }

  VoiceNodeExecutorGuard(const VoiceNodeExecutorGuard &) = delete;
  VoiceNodeExecutorGuard & operator=(const VoiceNodeExecutorGuard &) = delete;

  void stop() noexcept
  {
    if (!thread_.joinable()) {
      return;
    }
    stop_requested_.store(true, std::memory_order_release);
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & executor_;
  std::atomic<bool> stop_requested_{false};
  std::thread thread_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__VOICE_NODE_EXECUTOR_HPP_
