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

#ifndef VOICE_NAV_SIM__TEST__TF_OWNERSHIP_EVALUATOR_HPP_
#define VOICE_NAV_SIM__TEST__TF_OWNERSHIP_EVALUATOR_HPP_

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace voice_nav_sim::tf_audit
{

struct NodeIdentity
{
  std::string name;
  std::string namespace_name;
};

enum class NodeIdentityState
{
  UNRESOLVED,
  EXPECTED,
  MISMATCH,
};

struct NodeIdentityEvaluation
{
  NodeIdentityState state;
  std::set<std::string> actual_nodes;
};

inline bool node_identity_is_known(const NodeIdentity & identity)
{
  return
    !identity.name.empty() &&
    !identity.namespace_name.empty() &&
    identity.name != "_NODE_NAME_UNKNOWN_" &&
    identity.namespace_name != "_NODE_NAMESPACE_UNKNOWN_";
}

inline std::string fully_qualified_node_name(const NodeIdentity & identity)
{
  std::string node_namespace = identity.namespace_name;
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + identity.name;
  }
  if (node_namespace.front() != '/') {
    node_namespace.insert(node_namespace.begin(), '/');
  }
  if (node_namespace.back() == '/') {
    node_namespace.pop_back();
  }
  return node_namespace + "/" + identity.name;
}

inline NodeIdentityEvaluation evaluate_node_identities(
  const std::string & expected_node,
  const std::vector<NodeIdentity> & identities)
{
  std::set<std::string> actual_nodes;
  bool unresolved = identities.empty();
  for (const auto & identity : identities) {
    if (!node_identity_is_known(identity)) {
      unresolved = true;
      continue;
    }
    actual_nodes.insert(fully_qualified_node_name(identity));
  }

  const bool expected_node_found =
    actual_nodes.count(expected_node) != 0U;
  const bool resolved_wrong_node_found =
    actual_nodes.size() > (expected_node_found ? 1U : 0U);
  if (resolved_wrong_node_found && expected_node_found) {
    return {
      NodeIdentityState::MISMATCH,
      std::move(actual_nodes)};
  }
  if (expected_node_found) {
    return {
      NodeIdentityState::EXPECTED,
      std::move(actual_nodes)};
  }
  if (unresolved) {
    return {
      NodeIdentityState::UNRESOLVED,
      std::move(actual_nodes)};
  }
  return {
    NodeIdentityState::MISMATCH,
    std::move(actual_nodes)};
}

}  // namespace voice_nav_sim::tf_audit

#endif  // VOICE_NAV_SIM__TEST__TF_OWNERSHIP_EVALUATOR_HPP_
