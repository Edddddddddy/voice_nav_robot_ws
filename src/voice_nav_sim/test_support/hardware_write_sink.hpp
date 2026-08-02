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

#ifndef VOICE_NAV_SIM__HARDWARE_WRITE_SINK_HPP_
#define VOICE_NAV_SIM__HARDWARE_WRITE_SINK_HPP_

#include <cstdint>

namespace voice_nav_sim
{

struct HardwareWriteRecord
{
  std::uint64_t generation;
  std::uint64_t write_seq;
  std::int64_t sim_stamp_ns;
  std::uint8_t delegated_result;
  std::uint64_t left_command_bits;
  std::uint64_t right_command_bits;
};

class HardwareWriteSink
{
public:
  virtual ~HardwareWriteSink() = default;
  virtual bool append(const HardwareWriteRecord & record) noexcept = 0;
};

}  // namespace voice_nav_sim

#endif  // VOICE_NAV_SIM__HARDWARE_WRITE_SINK_HPP_
