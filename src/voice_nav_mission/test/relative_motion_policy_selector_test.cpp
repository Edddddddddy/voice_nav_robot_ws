// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include "relative_motion_policy_selector.hpp"

namespace voice_nav_mission
{
namespace
{

TEST(RelativeMotionPolicySelector, MappingKeepsFrozenG7ConvergenceTolerances)
{
  const auto policy = detail::relative_motion_policy_for(OperatingMode::Mapping);

  EXPECT_DOUBLE_EQ(policy.move_tolerance_m, 0.02);
  EXPECT_DOUBLE_EQ(policy.rotate_tolerance_rad, 0.04);
}

TEST(RelativeMotionPolicySelector, NavigationKeepsGlobalProductDefaults)
{
  const auto policy = detail::relative_motion_policy_for(OperatingMode::Navigation);

  EXPECT_DOUBLE_EQ(policy.move_tolerance_m, 0.05);
  EXPECT_DOUBLE_EQ(policy.rotate_tolerance_rad, 0.08);
}

}  // namespace
}  // namespace voice_nav_mission
