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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/runtime_completion_mailbox.hpp"

namespace voice_nav_mission
{

// Package-private production Module shared by MissionRuntimeNode and its
// deterministic seam tests.  It owns the Core-to-Node completion mailbox so
// Adapter threads publish only immutable records; Core and Goal terminal
// delivery remain on the Node runtime worker.
class RuntimeExecutionPlane final
{
public:
  // Package-private production seam: tests may decorate the real Core
  // delivery callback without replacing the Node registry or relay.
  using ChildResultDeliveryDecorator = std::function<
    RuntimeCore::ChildResultDelivery(RuntimeCore::ChildResultDelivery)>;

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
    NodeCompletionMailbox::EmergencyRequest emergency_request,
    ChildResultDeliveryDecorator delivery_decorator = {},
    std::shared_ptr<NavigationPort> navigation = {},
    std::shared_ptr<MapStorePort> map_store = {},
    NodeCompletionMailbox::TerminalEnqueue terminal_enqueue = {},
    RuntimeCore::ChildResultDispatcher child_result_dispatcher = {})
  : emergency_request_(std::move(emergency_request)),
    completion_mailbox_(
      std::move(token_enqueue),
      [this](std::string detail) {
        if (emergency_request_) {
          emergency_request_(std::move(detail));
        }
      },
      std::move(terminal_enqueue)),
    completion_reaper_(completion_mailbox_),
    core_(std::make_unique<RuntimeCore>(
      std::move(config),
      std::move(clock),
      std::move(authority),
      std::move(relative_motion),
      std::move(state_callback),
      std::move(feedback_callback),
      std::move(result_callback),
      std::move(child_feedback_dispatcher),
      std::move(child_result_dispatcher),
      std::move(admission_fence_check),
        [this, decorator = std::move(delivery_decorator)](
          const MotionToken & token, RuntimeCore::ChildResultDelivery delivery) mutable {
          if (decorator) {
            delivery = decorator(std::move(delivery));
          }
          return completion_mailbox_.register_delivery(token, std::move(delivery));
      },
        [this](const MotionToken & token) {
          completion_mailbox_.discard(token);
        },
      std::move(navigation),
      std::move(map_store)))
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

  [[nodiscard]] std::recursive_mutex & core_serial_mutex() noexcept
  {
    return core_serial_mutex_;
  }

  [[nodiscard]] std::thread::id completion_reaper_thread_id() const noexcept
  {
    return completion_reaper_.thread_id();
  }

  void shutdown() noexcept
  {
    if (shutdown_complete_) {
      return;
    }
    completion_mailbox_.close();
    completion_reaper_.stop();
    core_.reset();
    completion_mailbox_.stop();
    shutdown_complete_ = true;
  }

private:
  NodeCompletionMailbox::EmergencyRequest emergency_request_;
  std::recursive_mutex core_serial_mutex_;
  NodeCompletionMailbox completion_mailbox_;
  NodeCompletionReaper completion_reaper_;
  std::unique_ptr<RuntimeCore> core_;
  bool shutdown_complete_{false};
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RUNTIME_EXECUTION_PLANE_HPP_
