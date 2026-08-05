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

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MISSION_ACTION_RESULT_ROUTER_HPP_
