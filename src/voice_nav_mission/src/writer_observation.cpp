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
#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace voice_nav_mission
{
namespace
{

constexpr std::size_t kMaximumDetailLength = 160U;
constexpr std::size_t kDigestSuffixLength = 9U;
constexpr std::size_t kSummaryFixedLength = 55U;
constexpr std::size_t kSummaryValueCount = 5U;
constexpr std::size_t kMinimumSummaryLength =
  kSummaryFixedLength + kSummaryValueCount * kDigestSuffixLength;
constexpr char kUnknownNodeName[] = "_NODE_NAME_UNKNOWN_";
constexpr char kUnknownNodeNamespace[] = "_NODE_NAMESPACE_UNKNOWN_";

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

std::string digest_text(const std::string & value)
{
  std::uint32_t digest = 2166136261U;
  for (const auto character : value) {
    digest ^= static_cast<std::uint8_t>(character);
    digest *= 16777619U;
  }

  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(8) << digest;
  return stream.str();
}

std::string compact_field(
  const std::string & value,
  std::size_t maximum_length)
{
  if (value.size() <= maximum_length) {
    return value;
  }

  const auto suffix = "~" + digest_text(value);
  if (maximum_length <= suffix.size()) {
    return suffix.substr(suffix.size() - maximum_length);
  }
  return value.substr(0U, maximum_length - suffix.size()) + suffix;
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

bool node_name_is_unresolved(const std::string & node_name)
{
  return node_name.empty() || node_name == kUnknownNodeName;
}

bool node_namespace_is_unresolved(const std::string & node_namespace)
{
  return node_namespace == kUnknownNodeNamespace;
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

std::string fqn_name(const std::string & fqn)
{
  return fqn.substr(fqn.rfind('/') + 1U);
}

std::string observed_identity(
  const WriterEndpointObservation & endpoint)
{
  const bool name_unresolved =
    node_name_is_unresolved(endpoint.node_name);
  const bool namespace_unresolved =
    node_namespace_is_unresolved(endpoint.node_namespace);
  if (!name_unresolved && !namespace_unresolved) {
    return endpoint_fqn(endpoint);
  }
  const auto node_namespace = namespace_unresolved ?
    std::string{"<unresolved-namespace>"} :
  normalized_namespace(endpoint.node_namespace);
  const auto node_name = name_unresolved ?
    std::string{"<unresolved>"} : endpoint.node_name;
  return node_namespace == "/" ?
         "/" + node_name :
         node_namespace + "/" + node_name;
}

std::string observation_summary(
  const WriterEndpointObservation & endpoint,
  std::chrono::milliseconds elapsed,
  std::size_t maximum_length)
{
  if (maximum_length < kMinimumSummaryLength) {
    throw std::logic_error("writer diagnostic summary budget is too small");
  }

  std::array<std::string, kSummaryValueCount> values{
    std::to_string(static_cast<int>(endpoint.endpoint_type)),
    observed_identity(endpoint),
    std::to_string(static_cast<int>(endpoint.qos.history)) +
    "/" + std::to_string(endpoint.qos.depth) +
    "/" + std::to_string(static_cast<int>(endpoint.qos.reliability)) +
    "/" + std::to_string(static_cast<int>(endpoint.qos.durability)),
    std::to_string(elapsed.count()),
    endpoint.topic_type};
  std::array<std::size_t, kSummaryValueCount> budgets{};
  std::size_t used = kSummaryFixedLength;
  for (std::size_t index = 0U; index < values.size(); ++index) {
    budgets[index] = std::min(values[index].size(), kDigestSuffixLength);
    used += budgets[index];
  }

  auto remaining = maximum_length - used;
  bool grew = true;
  while (remaining > 0U && grew) {
    grew = false;
    for (std::size_t index = 0U; index < values.size(); ++index) {
      if (remaining == 0U) {
        break;
      }
      if (budgets[index] < values[index].size()) {
        ++budgets[index];
        --remaining;
        grew = true;
      }
    }
  }

  const auto summary =
    "n=1 k=" + compact_field(values[0], budgets[0]) +
    " id=" + compact_field(values[1], budgets[1]) +
    " q=" + compact_field(values[2], budgets[2]) +
    " g=" + gid_text(endpoint.writer_gid) +
    " ms=" + compact_field(values[3], budgets[3]) +
    " t=" + compact_field(values[4], budgets[4]);
  if (summary.size() > maximum_length) {
    throw std::logic_error("writer diagnostic summary exceeded its budget");
  }
  return summary;
}

std::string observation_detail(
  std::string reason,
  const WriterEndpointObservation & endpoint,
  std::chrono::milliseconds elapsed)
{
  constexpr std::size_t separator_length = 2U;
  constexpr std::size_t maximum_reason_length =
    kMaximumDetailLength - separator_length - kMinimumSummaryLength;
  reason = compact_field(reason, maximum_reason_length);
  const auto summary = observation_summary(
    endpoint, elapsed,
    kMaximumDetailLength - reason.size() - separator_length);
  return reason + "; " + summary;
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
  if (terminal_mismatch_ && !endpoints.empty()) {
    return mismatch(terminal_detail_);
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
    return {
      false,
      Reason::WriterUnavailable,
      {},
      "candidate topic has no writer; n=0 k=none id=none q=none "
      "g=none ms=" + std::to_string(elapsed.count()) + " t=none"};
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
  const auto summary = [&endpoint, elapsed](std::string reason) {
      return observation_detail(std::move(reason), endpoint, elapsed);
    };
  if (endpoint.endpoint_type != RMW_ENDPOINT_PUBLISHER) {
    return reject_mismatch(summary("endpoint kind mismatch"));
  }
  if (endpoint.topic_type != policy_.expected_topic_type) {
    return reject_mismatch(summary("writer type mismatch"));
  }
  if (!candidate_qos_is_compatible(endpoint.qos)) {
    return reject_mismatch(summary("writer QoS mismatch"));
  }
  if (gid_is_zero(endpoint.writer_gid)) {
    return reject_mismatch(summary("writer GID is all-zero"));
  }
  if (
    pinned_writer_gid_.has_value() &&
    *pinned_writer_gid_ != endpoint.writer_gid)
  {
    return reject_mismatch(
      summary("writer replaced pin=" + gid_text(*pinned_writer_gid_)));
  }

  const bool name_unresolved =
    node_name_is_unresolved(endpoint.node_name);
  const bool namespace_unresolved =
    node_namespace_is_unresolved(endpoint.node_namespace);
  const auto expected_namespace =
    fqn_namespace(policy_.expected_writer_fqn);
  const auto expected_name =
    fqn_name(policy_.expected_writer_fqn);
  if (!name_unresolved && endpoint.node_name != expected_name) {
    return reject_mismatch(summary("partial node name mismatch"));
  }
  if (!namespace_unresolved) {
    const auto observed_namespace =
      normalized_namespace(endpoint.node_namespace);
    if (observed_namespace != expected_namespace) {
      return reject_mismatch(
        summary("partial namespace mismatch"));
    }
  }
  if (name_unresolved || namespace_unresolved) {
    if (identity_confirmed_) {
      return {
        true,
        Reason::None,
        endpoint.writer_gid,
        summary("confirmed identity retained")};
    }
    if (!pinned_writer_gid_) {
      pinned_writer_gid_ = endpoint.writer_gid;
    }
    return {
      false,
      Reason::WriterMetadataPending,
      endpoint.writer_gid,
      summary("identity unresolved")};
  }

  const auto observed_fqn = endpoint_fqn(endpoint);
  if (observed_fqn != policy_.expected_writer_fqn) {
    return reject_mismatch(summary("writer FQN mismatch"));
  }
  if (!pinned_writer_gid_) {
    pinned_writer_gid_ = endpoint.writer_gid;
  }
  identity_confirmed_ = true;
  return {
    true,
    Reason::None,
    endpoint.writer_gid,
    summary("writer ready")};
}

void WriterObservationSession::reset() noexcept
{
  pinned_writer_gid_.reset();
  identity_confirmed_ = false;
  terminal_mismatch_ = false;
  terminal_detail_.clear();
}

}  // namespace voice_nav_mission
