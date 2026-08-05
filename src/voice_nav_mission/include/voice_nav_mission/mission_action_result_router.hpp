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

#ifndef VOICE_NAV_MISSION__MISSION_ACTION_RESULT_ROUTER_HPP_
#define VOICE_NAV_MISSION__MISSION_ACTION_RESULT_ROUTER_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

enum class OuterActionStatus : std::uint8_t
{
  Succeeded = 0,
  Canceled = 1,
  Aborted = 2,
};

struct ActionResultDelivery
{
  OuterActionStatus status{OuterActionStatus::Aborted};
  MissionResult result;
};

// This router is a package-private seam between Runtime Core callbacks and
// the ROS Action GoalHandle. It makes the provisional admission window
// explicit so a synchronous child result cannot be lost or delivered twice.
class MissionActionResultRouter final
{
public:
  using DeliveryCallback = std::function<void(
        std::uint64_t, const ActionResultDelivery &)>;

  void begin_admission()
  {
    if (pending_admission_) {
      throw std::logic_error("Mission Action result router is already occupied");
    }
    pending_admission_ = true;
    early_result_.reset();
  }

  void reject_admission()
  {
    pending_admission_ = false;
    early_result_.reset();
  }

  void commit(
    std::uint64_t mission_id,
    DeliveryCallback delivery)
  {
    if (!pending_admission_ || active_.has_value() || !delivery) {
      throw std::logic_error("Mission Action result commit is out of order");
    }
    pending_admission_ = false;
    active_ = Active{mission_id, std::move(delivery)};
    if (early_result_.has_value() && early_result_->first == mission_id) {
      const auto result = std::move(early_result_->second);
      early_result_.reset();
      deliver(mission_id, result);
    }
  }

  void finish(std::uint64_t mission_id, const MissionResult & result)
  {
    if (active_.has_value() && active_->mission_id == mission_id) {
      auto delivery = std::move(active_->delivery);
      active_.reset();
      delivery(mission_id, make_delivery(result));
      return;
    }
    if (pending_admission_ && !early_result_.has_value()) {
      early_result_ = std::make_pair(mission_id, result);
    }
  }

private:
  struct Active
  {
    std::uint64_t mission_id{0U};
    DeliveryCallback delivery;
  };

  [[nodiscard]] static ActionResultDelivery make_delivery(
    const MissionResult & result)
  {
    const auto status = result.code == MissionResultCode::Succeeded ?
      OuterActionStatus::Succeeded :
      (result.code == MissionResultCode::Canceled ?
      OuterActionStatus::Canceled : OuterActionStatus::Aborted);
    return ActionResultDelivery{status, result};
  }

  void deliver(std::uint64_t mission_id, const MissionResult & result)
  {
    auto delivery = std::move(active_->delivery);
    active_.reset();
    delivery(mission_id, make_delivery(result));
  }

  bool pending_admission_{false};
  std::optional<std::pair<std::uint64_t, MissionResult>> early_result_;
  std::optional<Active> active_;
};

// Package-private ROS Adapter seam. It mirrors MissionRuntimeNode::on_accepted
// while keeping GoalHandle-specific calls injectable for deterministic tests.
// Runtime Core may synchronously finish a mission from inside admit() (for
// example when child start throws), so the router's provisional window must
// surround the entire admission callback.
class MissionActionAdapterBoundary final
{
public:
  using AdmitCallback = std::function<AdmissionResult(const MissionGoal &)>;
  using RegisterCallback = std::function<void(std::uint64_t)>;
  using DeliveryCallback = MissionActionResultRouter::DeliveryCallback;
  using RejectionCallback = std::function<void(const MissionResult &)>;

  void on_accepted(
    const MissionGoal & goal,
    AdmitCallback admit,
    RegisterCallback register_goal,
    DeliveryCallback deliver,
    RejectionCallback reject)
  {
    if (!admit || !register_goal || !deliver || !reject) {
      throw std::invalid_argument("Mission Action Adapter callbacks are incomplete");
    }
    router_.begin_admission();
    AdmissionResult admission;
    try {
      admission = admit(goal);
    } catch (const std::exception & error) {
      router_.reject_admission();
      reject(MissionResult{
          MissionResultCode::InternalError,
          -1,
          std::string{"Mission admission threw: "} + error.what()});
      return;
    } catch (...) {
      router_.reject_admission();
      reject(MissionResult{
          MissionResultCode::InternalError,
          -1,
          "Mission admission threw an unknown exception"});
      return;
    }
    if (!admission.accepted) {
      router_.reject_admission();
      reject(admission.result);
      return;
    }
    register_goal(admission.mission_id);
    router_.commit(admission.mission_id, std::move(deliver));
  }

  void finish(std::uint64_t mission_id, const MissionResult & result)
  {
    router_.finish(mission_id, result);
  }

private:
  MissionActionResultRouter router_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MISSION_ACTION_RESULT_ROUTER_HPP_
