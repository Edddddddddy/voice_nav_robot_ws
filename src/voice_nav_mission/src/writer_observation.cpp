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

#include "writer_observation.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace voice_nav_mission
{
namespace
{

constexpr std::size_t kMaximumDetailLength = 160U;

bool gid_is_zero(const WriterGid & gid)
{
  return std::all_of(
    gid.cbegin(), gid.cend(),
    [](std::uint8_t value) {return value == 0U;});
}

std::string bounded_detail(std::string detail)
{
  if (detail.size() > kMaximumDetailLength) {
    detail.resize(kMaximumDetailLength);
  }
  return detail;
}

std::string gid_text(const WriterGid & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

std::string normalized_namespace(std::string node_namespace)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/";
  }
  if (node_namespace.front() != '/') {
    node_namespace.insert(node_namespace.begin(), '/');
  }
  if (node_namespace.back() == '/') {
    node_namespace.pop_back();
  }
  return node_namespace;
}

std::string endpoint_fqn(const WriterEndpointObservation & endpoint)
{
  const auto node_namespace =
    normalized_namespace(endpoint.node_namespace);
  return node_namespace == "/" ?
         "/" + endpoint.node_name :
         node_namespace + "/" + endpoint.node_name;
}

std::string fqn_namespace(const std::string & fqn)
{
  const auto separator = fqn.rfind('/');
  return separator == 0U ? "/" : fqn.substr(0U, separator);
}

std::string observed_identity(
  const WriterEndpointObservation & endpoint)
{
  if (!endpoint.node_name.empty()) {
    return endpoint_fqn(endpoint);
  }
  const auto node_namespace =
    normalized_namespace(endpoint.node_namespace);
  return node_namespace == "/" ?
         "/<unresolved>" :
         node_namespace + "/<unresolved>";
}

std::string observation_summary(
  const WriterEndpointObservation & endpoint,
  std::chrono::milliseconds elapsed)
{
  return
    "n=1 k=" + std::to_string(static_cast<int>(endpoint.endpoint_type)) +
    " id=" + observed_identity(endpoint) +
    " q=" + std::to_string(static_cast<int>(endpoint.qos.history)) +
    "/" + std::to_string(endpoint.qos.depth) +
    "/" + std::to_string(static_cast<int>(endpoint.qos.reliability)) +
    "/" + std::to_string(static_cast<int>(endpoint.qos.durability)) +
    " g=" + gid_text(endpoint.writer_gid) +
    " ms=" + std::to_string(elapsed.count()) +
    " t=" + endpoint.topic_type;
}

bool candidate_qos_is_compatible(const rmw_qos_profile_t & qos)
{
  const bool history_compatible =
    qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ||
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN;
  const bool depth_compatible =
    qos.depth == 1U ||
    (
    qos.depth == 0U &&
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN);
  return
    history_compatible &&
    depth_compatible &&
    qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
    qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE;
}

OpenBinding mismatch(std::string detail)
{
  return {
    false,
    Reason::WriterMismatch,
    {},
    bounded_detail(std::move(detail))};
}

}  // namespace

WriterObservationSession::WriterObservationSession(
  WriterObservationPolicy policy)
: policy_(std::move(policy))
{
  if (
    policy_.expected_topic_type.empty() ||
    policy_.expected_writer_fqn.empty() ||
    policy_.expected_writer_fqn.front() != '/' ||
    policy_.expected_writer_fqn.back() == '/')
  {
    throw std::invalid_argument(
            "writer observation policy requires a type and absolute FQN");
  }
}

OpenBinding WriterObservationSession::observe(
  const std::vector<WriterEndpointObservation> & endpoints,
  std::chrono::milliseconds elapsed)
{
  if (terminal_mismatch_) {
    return mismatch(
      "candidate writer observation is terminal: " + terminal_detail_);
  }

  const auto reject_mismatch = [this](std::string detail) {
      detail = bounded_detail(std::move(detail));
      if (pinned_writer_gid_) {
        terminal_mismatch_ = true;
        terminal_detail_ = detail;
      }
      return mismatch(std::move(detail));
    };

  if (endpoints.empty()) {
    if (pinned_writer_gid_) {
      return reject_mismatch(
        "pinned candidate writer disappeared after " +
        std::to_string(elapsed.count()) + "ms");
    }
    return {
      false,
      Reason::WriterUnavailable,
      {},
      "candidate topic has no writer"};
  }
  if (endpoints.size() != 1U) {
    if (pinned_writer_gid_) {
      return reject_mismatch(
        "writer ambiguous; n=" + std::to_string(endpoints.size()) +
        " pin=" + gid_text(*pinned_writer_gid_) +
        " ms=" + std::to_string(elapsed.count()));
    }
    return {
      false,
      Reason::WriterAmbiguous,
      {},
      bounded_detail(
        "candidate topic has " + std::to_string(endpoints.size()) +
        " writers after " + std::to_string(elapsed.count()) + "ms")};
  }

  const auto & endpoint = endpoints.front();
  const auto summary = observation_summary(endpoint, elapsed);
  if (endpoint.endpoint_type != RMW_ENDPOINT_PUBLISHER) {
    return reject_mismatch("endpoint kind mismatch; " + summary);
  }
  if (endpoint.topic_type != policy_.expected_topic_type) {
    return reject_mismatch("writer type mismatch; " + summary);
  }
  if (!candidate_qos_is_compatible(endpoint.qos)) {
    return reject_mismatch("writer QoS mismatch; " + summary);
  }
  if (gid_is_zero(endpoint.writer_gid)) {
    return reject_mismatch("writer GID is all-zero; " + summary);
  }
  if (
    pinned_writer_gid_.has_value() &&
    *pinned_writer_gid_ != endpoint.writer_gid)
  {
    return reject_mismatch(
      "writer replaced pin=" + gid_text(*pinned_writer_gid_) + "; " +
      summary);
  }

  if (endpoint.node_name.empty()) {
    const auto observed_namespace =
      normalized_namespace(endpoint.node_namespace);
    const auto expected_namespace =
      fqn_namespace(policy_.expected_writer_fqn);
    if (observed_namespace != expected_namespace) {
      return reject_mismatch(
        "partial namespace mismatch; " + summary);
    }
    if (identity_confirmed_) {
      return {
        true,
        Reason::None,
        endpoint.writer_gid,
        bounded_detail(
          "confirmed identity retained; " + summary)};
    }
    if (!pinned_writer_gid_) {
      pinned_writer_gid_ = endpoint.writer_gid;
    }
    return {
      false,
      Reason::WriterMetadataPending,
      endpoint.writer_gid,
      bounded_detail(
        "identity unresolved; " + summary)};
  }

  const auto observed_fqn = endpoint_fqn(endpoint);
  if (observed_fqn != policy_.expected_writer_fqn) {
    return reject_mismatch("writer FQN mismatch; " + summary);
  }
  if (!pinned_writer_gid_) {
    pinned_writer_gid_ = endpoint.writer_gid;
  }
  identity_confirmed_ = true;
  return {
    true,
    Reason::None,
    endpoint.writer_gid,
    bounded_detail("writer ready; " + summary)};
}

void WriterObservationSession::reset() noexcept
{
  pinned_writer_gid_.reset();
  identity_confirmed_ = false;
  terminal_mismatch_ = false;
  terminal_detail_.clear();
}

}  // namespace voice_nav_mission
