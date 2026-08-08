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

#include <atomic>
#include <cstdint>
#include <memory>

#include "voice_nav_mission/runtime_completion_registry.hpp"
#include "voice_nav_mission/runtime_emergency_fence.hpp"
#include "voice_nav_mission/runtime_event_ingress.hpp"
#include "voice_nav_mission/runtime_event_queue.hpp"

namespace voice_nav_mission
{
namespace
{

MotionToken token_for(const std::uint64_t value)
{
  return MotionToken{value, 7U, 11U, 1U};
}

RelativeMotionCompletionRecordPtr record_for(const MotionToken & token)
{
  return std::make_shared<const RelativeMotionCompletionRecord>(
    RelativeMotionCompletionRecord{
        token, ChildResult{ChildResultCode::SafetyFault, "registry seam"}});
}

struct CompletionEvent
{
  MotionToken token{};
};

using CompletionQueue = RuntimeEventQueue<CompletionEvent>;

TEST(RuntimeCompletionRegistryTest, StartFailureRejectsWhenDeliveryCapacityIsFull)
{
  NodeCompletionRegistry registry;
  for (std::size_t index = 0U; index < NodeCompletionRegistry::kCapacity; ++index) {
    ASSERT_TRUE(registry.register_delivery(
      token_for(index + 1U), [](const MotionToken &, const ChildResult &) {}));
  }

  const auto rejected_token = token_for(100U);
  EXPECT_FALSE(registry.register_delivery(
    rejected_token, [](const MotionToken &, const ChildResult &) {}));
  EXPECT_EQ(registry.entry_count(), NodeCompletionRegistry::kCapacity);

  // RuntimeCore observes the false registration before calling Adapter start;
  // no completion record or user delivery owner is created for this start.
  auto record = record_for(rejected_token);
  EXPECT_FALSE(registry.accept(record));
  EXPECT_TRUE(record);
  EXPECT_EQ(registry.rejected_count(), 0U);
}

TEST(RuntimeCompletionRegistryTest, TransactionRecordSurvivesBlockedIngressUntilFenceReaper)
{
  NodeCompletionRegistry registry;
  const auto token = token_for(200U);
  auto owner = std::make_shared<int>(7);
  std::weak_ptr<int> weak_owner = owner;
  ASSERT_TRUE(registry.register_delivery(
    token,
      [owner](const MotionToken &, const ChildResult &) {}));
  owner.reset();

  auto record = record_for(token);
  std::weak_ptr<const RelativeMotionCompletionRecord> weak_record = record;
  ASSERT_TRUE(registry.accept(record));
  EXPECT_FALSE(record);

  CompletionQueue queue([] {return CompletionEvent{};});
  RuntimeEmergencyFence fence(7U);
  std::atomic<std::size_t> emergency_calls{0U};
  RuntimeEventIngress<CompletionEvent> ingress(
    queue,
    fence,
    [](const CompletionEvent &) {return CompletionQueue::Lane::Control;},
    [&]() {emergency_calls.fetch_add(1U);},
    [&](const RuntimeEmergencyFenceSnapshot &) {registry.reap_all();});

  ASSERT_TRUE(fence.raise("blocked completion ingress"));
  EXPECT_FALSE(ingress.enqueue(CompletionEvent{token}));
  ingress.request_emergency("completion ingress was blocked");

  EXPECT_TRUE(fence.pending());
  EXPECT_EQ(emergency_calls.load(), 0U);
  EXPECT_EQ(registry.entry_count(), 1U);
  EXPECT_EQ(registry.rejected_count(), 0U);
  EXPECT_TRUE(ingress.process_pending_fence());
  EXPECT_EQ(registry.entry_count(), 0U);
  EXPECT_EQ(registry.rejected_count(), 0U);
  EXPECT_TRUE(weak_owner.expired());
  EXPECT_TRUE(weak_record.expired());
}

TEST(RuntimeCompletionRegistryTest, EmergencyRecordSurvivesClosedRelayUntilNodeReaper)
{
  NodeCompletionRegistry registry;
  const auto token = token_for(300U);
  auto owner = std::make_shared<int>(9);
  std::weak_ptr<int> weak_owner = owner;
  ASSERT_TRUE(registry.register_delivery(
    token,
      [owner](const MotionToken &, const ChildResult &) {}));
  owner.reset();

  registry.close();
  auto record = record_for(token);
  std::weak_ptr<const RelativeMotionCompletionRecord> weak_record = record;
  EXPECT_FALSE(registry.accept(record));
  ASSERT_TRUE(record);
  ASSERT_TRUE(registry.retain_rejected(std::move(record)));
  EXPECT_EQ(registry.entry_count(), 1U);
  EXPECT_EQ(registry.rejected_count(), 1U);
  EXPECT_FALSE(weak_owner.expired());
  EXPECT_FALSE(weak_record.expired());

  registry.reap_all();
  EXPECT_EQ(registry.entry_count(), 0U);
  EXPECT_EQ(registry.rejected_count(), 0U);
  EXPECT_TRUE(weak_owner.expired());
  EXPECT_TRUE(weak_record.expired());
}

TEST(RuntimeCompletionRegistryTest, TransactionRecordSurvivesFullIngressUntilFenceReaper)
{
  NodeCompletionRegistry registry;
  const auto token = token_for(400U);
  ASSERT_TRUE(registry.register_delivery(
    token, [](const MotionToken &, const ChildResult &) {}));
  auto record = record_for(token);
  ASSERT_TRUE(registry.accept(record));

  CompletionQueue queue([] {return CompletionEvent{};});
  RuntimeEmergencyFence fence(7U);
  std::atomic<std::size_t> emergency_calls{0U};
  RuntimeEventIngress<CompletionEvent> ingress(
    queue,
    fence,
    [](const CompletionEvent &) {return CompletionQueue::Lane::Control;},
    [&]() {emergency_calls.fetch_add(1U);},
    [&](const RuntimeEmergencyFenceSnapshot &) {registry.reap_all();});
  for (std::size_t index = 0U; index < CompletionQueue::kControlReserve; ++index) {
    ASSERT_TRUE(ingress.enqueue(CompletionEvent{token_for(index + 500U)}));
  }

  EXPECT_FALSE(ingress.enqueue(CompletionEvent{token}));
  EXPECT_EQ(emergency_calls.load(), 1U);
  EXPECT_EQ(registry.entry_count(), 1U);
  EXPECT_TRUE(ingress.process_pending_fence());
  EXPECT_EQ(registry.entry_count(), 0U);
}

}  // namespace
}  // namespace voice_nav_mission
