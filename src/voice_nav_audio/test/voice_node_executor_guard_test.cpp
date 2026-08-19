// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "voice_node_executor.hpp"

namespace voice_nav_audio
{
namespace
{

class RclcppContextGuard final
{
public:
  RclcppContextGuard()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppContextGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST(VoiceNodeExecutorGuardTest, JoinsOnReadinessAndPumpExceptionWithoutSleep)
{
  RclcppContextGuard context;
  const char * const failure_stages[] = {
    "agent readiness timeout", "continuous VAD pump failure",
  };
  for (const auto * const stage : failure_stages) {
    SCOPED_TRACE(stage);
    rclcpp::executors::SingleThreadedExecutor executor;
    EXPECT_THROW(
      {
        VoiceNodeExecutorGuard guard(executor);
        throw std::runtime_error(stage);
      },
      std::runtime_error);
    // Reaching the next iteration means the guard reclaimed its joinable
    // thread during stack unwinding; no sleep or retry is involved.
    EXPECT_NO_THROW(executor.spin_some(std::chrono::milliseconds(0)));
  }
}

TEST(VoiceNodeExecutorGuardTest, StopIsIdempotentBeforeDestruction)
{
  RclcppContextGuard context;
  rclcpp::executors::SingleThreadedExecutor executor;
  {
    VoiceNodeExecutorGuard guard(executor);
    guard.stop();
    guard.stop();
  }
}

}  // namespace
}  // namespace voice_nav_audio
