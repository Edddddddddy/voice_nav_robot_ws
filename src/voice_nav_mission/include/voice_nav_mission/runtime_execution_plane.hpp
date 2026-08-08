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

#ifndef VOICE_NAV_MISSION__RUNTIME_EXECUTION_PLANE_HPP_
#define VOICE_NAV_MISSION__RUNTIME_EXECUTION_PLANE_HPP_

#include <memory>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/runtime_completion_mailbox.hpp"

namespace voice_nav_mission
{

// Package-private production Module shared by MissionRuntimeNode and its
// deterministic seam tests.  It owns the Core-to-Node completion handoff so
// an Adapter thread can publish only immutable records; Core and Goal terminal
// delivery remain on the Node runtime worker.
class RuntimeExecutionPlane final
{
public:
  RuntimeExecutionPlane(
    RuntimeConfig config,
    std::shared_ptr<SteadyClockPort> clock,
    std::shared_ptr<MotionAuthorityPort> authority,
    std::shared_ptr<RelativeMotionPort> relative_motion,
    RuntimeCore::StateCallback state_callback,
    RuntimeCore::FeedbackCallback feedback_callback,
    RuntimeCore::ResultCallback result_callback,
    RuntimeCore::ChildFeedbackDispatcher child_feedback_dispatcher,
    RuntimeCore::AdmissionFenceCheck admission_fence_check,
    NodeCompletionMailbox::TokenEnqueue token_enqueue,
    NodeCompletionMailbox::EmergencyRequest emergency_request)
  : completion_mailbox_(
      std::move(token_enqueue), std::move(emergency_request)),
    core_(std::make_unique<RuntimeCore>(
      std::move(config),
      std::move(clock),
      std::move(authority),
      std::move(relative_motion),
      std::move(state_callback),
      std::move(feedback_callback),
      std::move(result_callback),
      std::move(child_feedback_dispatcher),
      RuntimeCore::ChildResultDispatcher{},
      std::move(admission_fence_check),
        [this](const MotionToken & token, RuntimeCore::ChildResultDelivery delivery) {
          return completion_mailbox_.register_delivery(token, std::move(delivery));
      },
        [this](const MotionToken & token) {
          completion_mailbox_.discard(token);
      }))
  {
  }

  ~RuntimeExecutionPlane()
  {
    shutdown();
  }

  RuntimeExecutionPlane(const RuntimeExecutionPlane &) = delete;
  RuntimeExecutionPlane & operator=(const RuntimeExecutionPlane &) = delete;

  [[nodiscard]] RuntimeCore * core() noexcept
  {
    return core_.get();
  }

  [[nodiscard]] const RuntimeCore * core() const noexcept
  {
    return core_.get();
  }

  [[nodiscard]] NodeCompletionMailbox & completion_mailbox() noexcept
  {
    return completion_mailbox_;
  }

  void shutdown() noexcept
  {
    core_.reset();
    completion_mailbox_.stop();
  }

private:
  NodeCompletionMailbox completion_mailbox_;
  std::unique_ptr<RuntimeCore> core_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EXECUTION_PLANE_HPP_
