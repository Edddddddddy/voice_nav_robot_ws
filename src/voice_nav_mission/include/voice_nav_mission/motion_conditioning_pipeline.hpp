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

#ifndef VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_
#define VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"
#include "voice_nav_mission/runtime_transaction_plane.hpp"

namespace voice_nav_mission
{

class MotionConditioningPipeline;

namespace detail
{

// Package-private shutdown seam.  It closes the conditioning health,
// collision, and renew ingress before the Adapter starts its independent
// Gate-zero transaction; RelativeMotionRosAdapter owns the separate odom
// stationarity ingress.
void begin_motion_conditioning_shutdown(
  MotionConditioningPipeline & pipeline) noexcept;

}  // namespace detail

enum class MotionConditioningState : std::uint8_t
{
  Stopped = 0,
  Prepared = 1,
  Running = 2,
  Failed = 3,
};

enum class MotionConditioningFailure : std::uint8_t
{
  None = 0,
  DependencyUnavailable = 1,
  SafetyFault = 2,
  ExecutionFailed = 3,
  InternalError = 4,
  Timeout = 5,
  GateLoss = 6,
};

struct MotionConditioningResult
{
  bool ok{false};
  MotionConditioningState state{MotionConditioningState::Stopped};
  MotionConditioningFailure failure{MotionConditioningFailure::None};
  bool zero_proven{false};
  bool collision_stop{false};
  std::string lease_id;
  std::string candidate_topic;
  std::string detail;
  // Steady-clock evidence for the Gate zero acknowledgement.  This remains
  // package-internal; it lets the relative-motion adapter start its bounded
  // stationarity window at the actual zero proof rather than after component
  // cleanup has completed.
  std::chrono::steady_clock::time_point zero_proven_at{};
  // Present only for the typed terminal created when an active Gate identity
  // disappears before it can acknowledge zero.  Generic SafetyFault results
  // leave this empty and can never enter the replacement rearm path.
  std::string gate_loss_instance_id;
  // A failed RENEW transport can race DDS graph convergence after a Gate
  // process loss.  Keep the old identity as a narrow recovery candidate while
  // the terminal remains SafetyFault.  It cannot authorize recovery without a
  // different identity and a fresh inhibited-zero transaction.
  std::string gate_loss_candidate_instance_id;
};

namespace detail
{

// Package-private startup seam.  Runtime uses the result to gate availability;
// it does not add a ROS endpoint or change the public Mission interface.
[[nodiscard]] MotionConditioningResult reconcile_motion_conditioning_startup(
  MotionConditioningPipeline & pipeline);

}  // namespace detail

struct MotionConditioningCorrelationToken
{
  std::uint64_t generation{0U};
  std::string lease_id;
  std::string request_id;
  std::string gate_instance_id;
};

// A terminal result crosses the RelativeMotion Adapter's worker seam as pure
// data.  Delivery ownership stays in the Node-owned completion registry; an
// Adapter worker must never carry or destroy a user callback.
struct RelativeMotionCompletionRecord
{
  MotionToken token;
  ChildResult result;
};

using RelativeMotionCompletionRecordPtr =
  std::shared_ptr<const RelativeMotionCompletionRecord>;

using RelativeMotionCompletionRelay =
  std::function<bool(RelativeMotionCompletionRecordPtr)>;

struct MotionConditioningConfig
{
  std::chrono::milliseconds component_rpc_timeout{2000};
  std::chrono::milliseconds writer_graph_timeout{1000};
  std::chrono::milliseconds startup_reconciliation_timeout{4000};
  // The production RelativeMotion Adapter invokes the package-private
  // startup transaction from Runtime availability probing.  Standalone
  // Pipeline users retain the safer default and reconcile from prepare().
  bool startup_reconciliation_on_prepare{true};
  std::chrono::milliseconds prepare_open_deadline{4000};
  std::chrono::milliseconds renew_period{100};
  std::chrono::milliseconds dependency_liveness_timeout{200};
  // Collision Monitor compares sensor ROS timestamps with its simulation
  // clock.  This skew budget is distinct from the steady-clock dependency
  // liveness deadline above.
  std::chrono::milliseconds collision_source_timeout{200};
  std::chrono::milliseconds health_rpc_timeout{100};
  std::chrono::milliseconds control_response_deadline{100};
  std::chrono::milliseconds stop_barrier{250};
  std::string container_fqn{"/motion_conditioning_container"};
  std::string raw_topic{"/voice_nav_internal/motion/raw"};
  std::string smoothed_topic{"/voice_nav_internal/motion/smoothed"};
  std::string scan_topic{"/scan"};
  std::string odom_topic{"/odom"};
  std::string clock_topic{"/clock"};
  std::string controller_manager_service{"/controller_manager/list_controllers"};
  std::string controller_name{"diff_drive_controller"};
  std::string collision_state_topic{
    "/voice_nav_internal/motion/collision_state"};
  std::function<std::string()> request_id_generator;
  std::function<void(const MotionConditioningCorrelationToken &)> before_token_claim;
  std::function<void()> before_health_callback;
  std::function<void()> after_health_callback;
  std::function<void()> before_open_callback;
  std::function<void()> before_renew_callback;
  std::function<void()> before_callback_wait;
  std::function<void()> before_renew_wait;
  // Package-private deterministic barriers used by the RelativeMotion ROS
  // Adapter seam tests. Production configuration leaves these unset.
  std::function<void()> before_adapter_odom_callback;
  std::function<void()> before_adapter_scan_callback;
  std::function<void()> before_adapter_clock_callback;
  std::function<void()> before_adapter_command_supplier;
  std::function<void()> before_adapter_ingress_wait;
  std::function<void()> before_adapter_completion_publish;
  std::function<void()> after_adapter_completion_publish;
  std::function<bool(std::uint64_t)> admission_fence_check;
  std::shared_ptr<RuntimeTransactionPlane> transaction_plane;
  std::function<std::uint64_t()> transaction_generation_provider;
  RelativeMotionCompletionRelay completion_relay;
};

class MotionProducerPort
{
public:
  virtual ~MotionProducerPort() = default;
  [[nodiscard]] virtual bool start(const std::string & raw_topic) = 0;
  virtual void stop() = 0;
};

class MotionConditioningPipeline final
{
public:
  MotionConditioningPipeline(
    rclcpp::Node & node,
    std::shared_ptr<MotionAuthorityPort> authority,
    std::shared_ptr<MotionProducerPort> producer,
    MotionConditioningConfig config = {});
  ~MotionConditioningPipeline();

  MotionConditioningPipeline(const MotionConditioningPipeline &) = delete;
  MotionConditioningPipeline & operator=(const MotionConditioningPipeline &) = delete;

  [[nodiscard]] MotionConditioningResult prepare();
  [[nodiscard]] MotionConditioningResult start();
  [[nodiscard]] MotionConditioningResult stop();
  [[nodiscard]] MotionConditioningResult fail(
    MotionConditioningCorrelationToken token,
    MotionConditioningFailure failure,
    std::string detail);

  // Releases only a Gate-loss terminal after a replacement Gate has proved a
  // current inhibited zero.  This is an internal rearm seam; generic
  // SafetyFault and cleanup-residual terminals remain non-reusable.
  [[nodiscard]] bool rearm_after_gate_replacement(
    const GateSnapshot & snapshot,
    GateSnapshot * accepted_snapshot = nullptr) noexcept;

  [[nodiscard]] MotionConditioningCorrelationToken correlation_token() const;

  [[nodiscard]] MotionConditioningState state() const noexcept;
  [[nodiscard]] MotionConditioningResult last_result() const;

private:
  friend MotionConditioningResult detail::reconcile_motion_conditioning_startup(
    MotionConditioningPipeline & pipeline);
  friend void detail::begin_motion_conditioning_shutdown(
    MotionConditioningPipeline & pipeline) noexcept;

  void begin_shutdown_ingress() noexcept;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MOTION_CONDITIONING_PIPELINE_HPP_
