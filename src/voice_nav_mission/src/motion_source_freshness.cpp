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

#include <stdexcept>

namespace voice_nav_mission
{

SteadySourceFreshness::SteadySourceFreshness(const std::chrono::milliseconds timeout)
: timeout_(timeout)
{
  if (timeout_.count() <= 0) {
    throw std::invalid_argument("steady source freshness timeout must be positive");
  }
}

void SteadySourceFreshness::observe(const TimePoint receipt) noexcept
{
  last_receipt_ = receipt;
  observed_ = true;
}

bool SteadySourceFreshness::observed() const noexcept
{
  return observed_;
}

bool SteadySourceFreshness::fresh_at(const TimePoint now) const noexcept
{
  return observed_ && now >= last_receipt_ &&
         now - last_receipt_ <= timeout_;
}

bool raw_stamp_age_is_fresh(
  const std::chrono::nanoseconds age,
  const std::chrono::milliseconds timeout) noexcept
{
  return timeout.count() > 0 && age >= std::chrono::nanoseconds::zero() &&
         age <= timeout;
}

}  // namespace voice_nav_mission
