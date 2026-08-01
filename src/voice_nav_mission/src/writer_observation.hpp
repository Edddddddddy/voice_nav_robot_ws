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

#ifndef WRITER_OBSERVATION_HPP_
#define WRITER_OBSERVATION_HPP_

#include <rmw/types.h>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "voice_nav_mission/motion_gate_core.hpp"

namespace voice_nav_mission
{

struct WriterEndpointObservation
{
  std::string topic_type;
  std::string node_name;
  std::string node_namespace;
  rmw_endpoint_type_t endpoint_type{RMW_ENDPOINT_INVALID};
  rmw_qos_profile_t qos{};
  WriterGid writer_gid{};
};

struct WriterObservationPolicy
{
  std::string expected_topic_type;
  std::string expected_writer_fqn;
};

class WriterObservationSession
{
public:
  explicit WriterObservationSession(WriterObservationPolicy policy);

  [[nodiscard]] OpenBinding observe(
    const std::vector<WriterEndpointObservation> & endpoints,
    std::chrono::milliseconds elapsed);

  void reset() noexcept;

private:
  WriterObservationPolicy policy_;
  std::optional<WriterGid> pinned_writer_gid_;
  bool identity_confirmed_{false};
  bool terminal_mismatch_{false};
  std::string terminal_detail_;
};

}  // namespace voice_nav_mission

#endif  // WRITER_OBSERVATION_HPP_
