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

#ifndef VOICE_NAV_MISSION__MOTION_SOURCE_FRESHNESS_HPP_
#define VOICE_NAV_MISSION__MOTION_SOURCE_FRESHNESS_HPP_

#include <chrono>

namespace voice_nav_mission
{

// Deep ROS-free freshness Module.  The caller supplies receipt times, so
// behavior tests can advance a manual steady clock without sleeping.  A
// source is fresh through the inclusive deadline; an unobserved or backward
// time sample is never fresh.
class SteadySourceFreshness final
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit SteadySourceFreshness(std::chrono::milliseconds timeout);

  void observe(TimePoint receipt) noexcept;

  [[nodiscard]] bool observed() const noexcept;
  [[nodiscard]] bool fresh_at(TimePoint now) const noexcept;

private:
  std::chrono::milliseconds timeout_;
  TimePoint last_receipt_{};
  bool observed_{false};
};

// Collision Monitor's source_timeout contract is a ROS-time measurement age,
// independent from steady callback liveness.  Negative age is fail-closed;
// exactly the configured age remains usable and a greater age is stale.
[[nodiscard]] bool raw_stamp_age_is_fresh(
  std::chrono::nanoseconds age,
  std::chrono::milliseconds timeout) noexcept;

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MOTION_SOURCE_FRESHNESS_HPP_
