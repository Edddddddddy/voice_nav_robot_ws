// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

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

TEST(VoiceNodeExecutorGuardTest, JoinsOnEachPumpFailureExceptionPath)
{
  RclcppContextGuard context;
  const char * const failure_stages[] = {
    "re-arm factory", "DSP", "publisher activation",
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
