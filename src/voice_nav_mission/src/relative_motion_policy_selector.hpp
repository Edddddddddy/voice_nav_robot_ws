// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_MISSION__RELATIVE_MOTION_POLICY_SELECTOR_HPP_
#define VOICE_NAV_MISSION__RELATIVE_MOTION_POLICY_SELECTOR_HPP_

#include "voice_nav_mission/relative_motion_controller.hpp"

namespace voice_nav_mission
{
namespace detail
{

[[nodiscard]] inline RelativeMotionPolicy relative_motion_policy_for(
  const OperatingMode operating_mode) noexcept
{
  RelativeMotionPolicy policy;
  if (operating_mode == OperatingMode::Mapping) {
    policy.move_tolerance_m = 0.02;
    policy.rotate_tolerance_rad = 0.04;
  }
  return policy;
}

}  // namespace detail
}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RELATIVE_MOTION_POLICY_SELECTOR_HPP_
