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

#include "voice_nav_mission/motion_source_freshness.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

TEST(MotionSourceFreshness, RawScanCallbackStoppedAt201MsStopsRenew)
{
  SteadySourceFreshness freshness(200ms);
  const auto t0 = SteadySourceFreshness::TimePoint{};

  freshness.observe(t0);

  EXPECT_TRUE(freshness.fresh_at(t0 + 199ms));
  EXPECT_TRUE(freshness.fresh_at(t0 + 200ms));
  EXPECT_FALSE(freshness.fresh_at(t0 + 201ms));
}

TEST(MotionSourceFreshness, RawScanStampAge299MsIsUsable)
{
  EXPECT_TRUE(raw_stamp_age_is_fresh(299ms, 300ms));
}

TEST(MotionSourceFreshness, RawScanStampAge300MsKeepsInclusiveBoundary)
{
  EXPECT_TRUE(raw_stamp_age_is_fresh(300ms, 300ms));
}

TEST(MotionSourceFreshness, RawScanStampAge301MsFailsClosed)
{
  EXPECT_FALSE(raw_stamp_age_is_fresh(301ms, 300ms));
}

TEST(MotionSourceFreshness, UnobservedAndBackwardSteadyTimeAreNotFresh)
{
  SteadySourceFreshness freshness(200ms);
  const auto t0 = SteadySourceFreshness::TimePoint{};

  EXPECT_FALSE(freshness.fresh_at(t0));
  freshness.observe(t0 + 10ms);
  EXPECT_FALSE(freshness.fresh_at(t0));
}

}  // namespace
}  // namespace voice_nav_mission
