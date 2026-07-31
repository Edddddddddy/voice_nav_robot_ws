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

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "tf_ownership_evaluator.hpp"

namespace
{

using voice_nav_sim::tf_audit::evaluate_node_identities;
using voice_nav_sim::tf_audit::NodeIdentity;
using voice_nav_sim::tf_audit::NodeIdentityState;

const std::string expected_owner{"/expected_owner"};
const std::string wrong_owner{"/wrong_owner"};

TEST(TfOwnershipEvaluator, AbsentGraphIdentityRemainsUnresolved)
{
  const auto evaluation = evaluate_node_identities(expected_owner, {});

  EXPECT_EQ(evaluation.state, NodeIdentityState::UNRESOLVED);
  EXPECT_TRUE(evaluation.actual_nodes.empty());
}

TEST(TfOwnershipEvaluator, RmwUnknownPlaceholdersRemainUnresolved)
{
  const std::vector<NodeIdentity> identities{
    {"_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::UNRESOLVED);
  EXPECT_TRUE(evaluation.actual_nodes.empty());
}

TEST(TfOwnershipEvaluator, UnknownPlaceholderCannotMatchExpectedText)
{
  const std::vector<NodeIdentity> identities{
    {"_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_"}};

  const auto evaluation = evaluate_node_identities(
    "/_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_",
    identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::UNRESOLVED);
}

TEST(TfOwnershipEvaluator, ResolvedExpectedIdentityMatches)
{
  const std::vector<NodeIdentity> identities{
    {"expected_owner", "/"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::EXPECTED);
  EXPECT_EQ(evaluation.actual_nodes, std::set<std::string>{expected_owner});
}

TEST(TfOwnershipEvaluator, ExpectedIdentityWinsOverUnresolvedDuplicate)
{
  const std::vector<NodeIdentity> identities{
    {"expected_owner", "/"},
    {"_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::EXPECTED);
}

TEST(TfOwnershipEvaluator, ExpectedIdentityCannotMaskResolvedWrongIdentity)
{
  const std::vector<NodeIdentity> identities{
    {"expected_owner", "/"},
    {"wrong_owner", "/"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::MISMATCH);
  EXPECT_EQ(
    evaluation.actual_nodes,
    (std::set<std::string>{expected_owner, wrong_owner}));
}

TEST(TfOwnershipEvaluator, WrongIdentityWaitsWhileAnyIdentityIsUnresolved)
{
  const std::vector<NodeIdentity> identities{
    {"wrong_owner", "/"},
    {"_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::UNRESOLVED);
  EXPECT_EQ(evaluation.actual_nodes, std::set<std::string>{wrong_owner});
}

TEST(TfOwnershipEvaluator, FullyResolvedWrongIdentityIsMismatch)
{
  const std::vector<NodeIdentity> identities{
    {"wrong_owner", "/"}};

  const auto evaluation =
    evaluate_node_identities(expected_owner, identities);

  EXPECT_EQ(evaluation.state, NodeIdentityState::MISMATCH);
  EXPECT_EQ(evaluation.actual_nodes, std::set<std::string>{wrong_owner});
}

}  // namespace
