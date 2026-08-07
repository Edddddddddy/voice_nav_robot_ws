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

#include "voice_nav_mission/relative_motion_ros_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Odometry = nav_msgs::msg::Odometry;
using Clock = rosgraph_msgs::msg::Clock;
using LaserScan = sensor_msgs::msg::LaserScan;

rclcpp::QoS latest_sensor_qos()
{
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  return qos;
}

bool same_token(const MotionToken & left, const MotionToken & right) noexcept
{
  return left.mission_id == right.mission_id &&
         left.admission_epoch == right.admission_epoch &&
         left.mission_generation == right.mission_generation &&
         left.step_generation == right.step_generation;
}

ChildResultCode child_code_for_conditioning(
  const MotionConditioningFailure failure) noexcept
{
  switch (failure) {
    case MotionConditioningFailure::DependencyUnavailable:
      return ChildResultCode::DependencyUnavailable;
    case MotionConditioningFailure::ExecutionFailed:
      return ChildResultCode::Failed;
    case MotionConditioningFailure::SafetyFault:
      return ChildResultCode::SafetyFault;
    case MotionConditioningFailure::InternalError:
      return ChildResultCode::InternalError;
    case MotionConditioningFailure::None:
    default:
      return ChildResultCode::SafetyFault;
  }
}

ChildResultCode child_code_for_relative(
  const RelativeMotionFailure failure) noexcept
{
  switch (failure) {
    case RelativeMotionFailure::DependencyUnavailable:
      return ChildResultCode::DependencyUnavailable;
    case RelativeMotionFailure::ExecutionFailed:
      return ChildResultCode::Failed;
    case RelativeMotionFailure::Timeout:
      return ChildResultCode::Timeout;
    case RelativeMotionFailure::SafetyFault:
      return ChildResultCode::SafetyFault;
    case RelativeMotionFailure::InternalError:
      return ChildResultCode::InternalError;
    case RelativeMotionFailure::None:
    default:
      return ChildResultCode::InternalError;
  }
}

MotionConditioningFailure conditioning_failure_for_relative(
  const RelativeMotionFailure failure) noexcept
{
  switch (failure) {
    case RelativeMotionFailure::DependencyUnavailable:
      return MotionConditioningFailure::DependencyUnavailable;
    case RelativeMotionFailure::ExecutionFailed:
    case RelativeMotionFailure::Timeout:
      return MotionConditioningFailure::ExecutionFailed;
    case RelativeMotionFailure::SafetyFault:
      return MotionConditioningFailure::SafetyFault;
    case RelativeMotionFailure::InternalError:
      return MotionConditioningFailure::InternalError;
    case RelativeMotionFailure::None:
    default:
      return MotionConditioningFailure::InternalError;
  }
}

class RawMotionProducer final : public MotionProducerPort
{
public:
  using CommandSupplier = std::function<RelativeMotionCommand()>;

  RawMotionProducer(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    CommandSupplier command_supplier)
  : node_(node),
    callback_group_(std::move(callback_group)),
    command_supplier_(std::move(command_supplier))
  {
  }

  ~RawMotionProducer() override
  {
    stop();
  }

  [[nodiscard]] bool start(const std::string & raw_topic) override
  {
    if (raw_topic.empty()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      return false;
    }
    try {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      qos.reliable().durability_volatile();
      rclcpp::PublisherOptions options;
      options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      publisher_ = node_.create_publisher<TwistStamped>(raw_topic, qos, options);
      timer_ = node_.create_wall_timer(
        50ms,
        [this]() {publish_current();},
        callback_group_);
      active_ = true;
      return true;
    } catch (...) {
      timer_.reset();
      publisher_.reset();
      return false;
    }
  }

  void stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!publisher_) {
      active_ = false;
      timer_.reset();
      return;
    }
    publish_locked(RelativeMotionCommand{});
    timer_.reset();
    publisher_.reset();
    active_ = false;
  }

private:
  void publish_current()
  {
    std::shared_ptr<rclcpp::Publisher<TwistStamped>> publisher;
    CommandSupplier supplier;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !publisher_) {
        return;
      }
      publisher = publisher_;
      supplier = command_supplier_;
    }
    const auto command = supplier ? supplier() : RelativeMotionCommand{};
    TwistStamped message;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = "base_footprint";
    message.twist.linear.x = command.linear_x_mps;
    message.twist.angular.z = command.angular_z_rps;
    publisher->publish(message);
  }

  void publish_locked(const RelativeMotionCommand & command)
  {
    if (!publisher_) {
      return;
    }
    TwistStamped message;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = "base_footprint";
    message.twist.linear.x = command.linear_x_mps;
    message.twist.angular.z = command.angular_z_rps;
    publisher_->publish(message);
  }

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  CommandSupplier command_supplier_;
  std::mutex mutex_;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool active_{false};
};

}  // namespace

class RelativeMotionRosAdapter::Impl
{
public:
  using TimePoint = SteadyClockPort::TimePoint;

  Impl(
    rclcpp::Node & node,
    std::shared_ptr<MotionAuthorityPort> authority,
    RelativeMotionPolicy policy,
    MotionConditioningConfig conditioning_config)
  : node_(node),
    authority_(std::move(authority)),
    policy_(std::move(policy)),
    conditioning_config_(std::move(conditioning_config)),
    controller_(policy_),
    callback_group_(node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    producer_(std::make_shared<RawMotionProducer>(
      node_, callback_group_, [this]() {return command();})),
    conditioning_(std::make_unique<MotionConditioningPipeline>(
      node_, authority_, producer_, conditioning_config_))
  {
    if (!authority_) {
      throw std::invalid_argument("RelativeMotionRosAdapter requires a Gate port");
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    odom_subscription_ = node_.create_subscription<Odometry>(
      conditioning_config_.odom_topic,
      rclcpp::SensorDataQoS(),
      [this](const Odometry::ConstSharedPtr message) {on_odom(message);},
      options);
    scan_subscription_ = node_.create_subscription<LaserScan>(
      conditioning_config_.scan_topic,
      latest_sensor_qos(),
      [this](const LaserScan::ConstSharedPtr) {on_scan();},
      options);
    clock_subscription_ = node_.create_subscription<Clock>(
      conditioning_config_.clock_topic,
      rclcpp::ClockQoS(),
      [this](const Clock::ConstSharedPtr message) {on_clock(message);},
      options);
  }

  ~Impl()
  {
    try {
      if (conditioning_) {
        (void)conditioning_->stop();
      }
    } catch (...) {
    }
    clock_subscription_.reset();
    scan_subscription_.reset();
    odom_subscription_.reset();
    conditioning_.reset();
    producer_.reset();
  }

  [[nodiscard]] bool healthy() const
  {
    const auto now = std::chrono::steady_clock::now();
    bool active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = active_;
      if (active) {
        return true;
      }
      if (!dependencies_fresh_locked(now)) {
        return false;
      }
    }
    return conditioning_->state() != MotionConditioningState::Failed;
  }

  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result)
  {
    ResultCallback rejected_result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        rejected_result = std::move(result);
      } else {
        active_ = true;
        starting_ = true;
        teardown_started_ = false;
        active_token_ = token;
        active_step_ = step;
        feedback_callback_ = std::move(feedback);
        result_callback_ = std::move(result);
        conditioning_token_ = {};
        command_ = {};
        zero_proven_ = false;
        stationarity_waiting_ = false;
      }
    }
    if (rejected_result) {
      rejected_result(
        token,
        ChildResult{ChildResultCode::InternalError,
          "relative-motion generation was already active"});
      return;
    }

    MotionConditioningResult prepared;
    try {
      prepared = conditioning_->prepare();
    } catch (...) {
      finish_start_failure(
        token,
        MotionConditioningResult{
          false, MotionConditioningState::Failed,
          MotionConditioningFailure::InternalError, false, false, {}, {},
          "conditioning PREPARE raised"});
      return;
    }
    if (!prepared.ok || prepared.state != MotionConditioningState::Prepared) {
      finish_start_failure(token, prepared);
      return;
    }

    MotionConditioningResult started;
    try {
      started = conditioning_->start();
    } catch (...) {
      finish_start_failure(
        token,
        MotionConditioningResult{
          false, MotionConditioningState::Failed,
          MotionConditioningFailure::InternalError, false, false, {}, {},
          "conditioning OPEN raised"});
      return;
    }
    if (!started.ok || started.state != MotionConditioningState::Running) {
      finish_start_failure(token, started);
      return;
    }

    const auto conditioning_token = conditioning_->correlation_token();
    DeliveryPlan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !same_token(active_token_, token)) {
        return;
      }
      conditioning_token_ = conditioning_token;
      const auto now = std::chrono::steady_clock::now();
      auto event = controller_.start(token, step, now);
      if (latest_odom_.has_value() && odom_seen_ &&
        now - last_odom_at_ <= policy_.dependency_liveness_timeout)
      {
        event = controller_.observe_odom(*latest_odom_, now);
      }
      starting_ = false;
      command_ = event.command;
      plan_from_event_locked(event, now, plan);
      condition_variable_.notify_all();
    }
    dispatch(plan);
  }

  [[nodiscard]] bool cancel(const MotionToken & token, const TimePoint deadline)
  {
    TeardownRequest request;
    bool execute = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!active_) {
        lock.unlock();
        return stop_without_active_mission();
      }
      if (!same_token(active_token_, token)) {
        return true;
      }
      if (teardown_started_) {
        condition_variable_.wait_until(lock, deadline, [this]() {return !active_;});
        return !active_ && zero_proven_;
      }
      const auto now = std::chrono::steady_clock::now();
      const auto event = controller_.request_safe_stop(
        RelativeMotionStopIntent::Cancel, now);
      command_ = event.command;
      teardown_started_ = true;
      request = TeardownRequest{
        TeardownKind::Cancel,
        ChildResultCode::Failed,
        MotionConditioningFailure::None,
        "relative-motion cancellation requested",
        active_token_,
        deadline};
      execute = true;
      condition_variable_.notify_all();
    }
    if (!execute) {
      return false;
    }
    return run_teardown(request, false);
  }

  void tick(const TimePoint now)
  {
    DeliveryPlan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || starting_ || teardown_started_) {
        return;
      }
      const auto event = controller_.tick(now);
      command_ = event.command;
      plan_from_event_locked(event, now, plan);
    }
    dispatch(plan);
    if (plan.teardown.has_value()) {
      return;
    }

    if (conditioning_->state() != MotionConditioningState::Failed) {
      return;
    }
    DeliveryPlan failure_plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || starting_ || teardown_started_) {
        return;
      }
      const auto conditioning_result = conditioning_->last_result();
      const auto now_again = std::chrono::steady_clock::now();
      const auto event = controller_.request_safe_stop(
        RelativeMotionStopIntent::Failure, now_again);
      command_ = event.command;
      teardown_started_ = true;
      failure_plan.teardown = TeardownRequest{
        TeardownKind::Failure,
        child_code_for_conditioning(conditioning_result.failure),
        conditioning_result.failure,
        conditioning_result.detail.empty() ?
        "Motion conditioning failed" : conditioning_result.detail,
        active_token_,
        now_again + policy_.stationarity_deadline};
      condition_variable_.notify_all();
    }
    dispatch(failure_plan);
  }

  [[nodiscard]] bool zero_proven() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return zero_proven_;
  }

private:
  enum class TeardownKind : std::uint8_t
  {
    Completion = 0,
    Failure = 1,
    Cancel = 2,
  };

  struct TeardownRequest
  {
    TeardownKind kind{TeardownKind::Failure};
    ChildResultCode child_code{ChildResultCode::SafetyFault};
    MotionConditioningFailure conditioning_failure{
      MotionConditioningFailure::None};
    std::string detail;
    MotionToken token{};
    TimePoint deadline{};
  };

  struct DeliveryPlan
  {
    bool feedback{false};
    MotionToken feedback_token{};
    double progress{0.0};
    FeedbackCallback feedback_callback;
    std::optional<TeardownRequest> teardown;
  };

  [[nodiscard]] RelativeMotionCommand command() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_;
  }

  [[nodiscard]] bool dependencies_fresh_locked(const TimePoint now) const
  {
    const auto timeout = policy_.dependency_liveness_timeout;
    return odom_seen_ && scan_seen_ && clock_seen_ && clock_advanced_ &&
           now - last_odom_at_ <= timeout &&
           now - last_scan_at_ <= timeout &&
           now - last_clock_at_ <= timeout &&
           now - last_clock_progress_at_ <= timeout &&
           node_.get_clock()->ros_time_is_active() &&
           node_.get_clock()->now().nanoseconds() > 0;
  }

  void on_odom(const Odometry::ConstSharedPtr & message)
  {
    const auto & orientation = message->pose.pose.orientation;
    const auto norm = orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z +
      orientation.w * orientation.w;
    if (!std::isfinite(norm) || norm <= 1.0e-12) {
      return;
    }
    const auto yaw_denominator = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
    const auto yaw_numerator = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
    const RelativeMotionOdom sample{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      std::atan2(yaw_numerator, yaw_denominator),
      message->twist.twist.linear.x,
      message->twist.twist.angular.z};
    const auto now = std::chrono::steady_clock::now();

    DeliveryPlan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_seen_ = true;
      last_odom_at_ = now;
      latest_odom_ = sample;
      if (active_ && !starting_ && (!teardown_started_ || stationarity_waiting_)) {
        const auto event = controller_.observe_odom(sample, now);
        command_ = event.command;
        plan_from_event_locked(event, now, plan);
      }
      condition_variable_.notify_all();
    }
    dispatch(plan);
  }

  void on_scan()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan_seen_ = true;
    last_scan_at_ = std::chrono::steady_clock::now();
    condition_variable_.notify_all();
  }

  void on_clock(const Clock::ConstSharedPtr & message)
  {
    const auto stamp = static_cast<std::int64_t>(message->clock.sec) *
      1000000000LL + static_cast<std::int64_t>(message->clock.nanosec);
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (clock_seen_ && stamp > last_clock_stamp_) {
      clock_advanced_ = true;
      last_clock_progress_at_ = now;
    }
    clock_seen_ = true;
    last_clock_stamp_ = stamp;
    last_clock_at_ = now;
    condition_variable_.notify_all();
  }

  void plan_from_event_locked(
    const RelativeMotionEvent & event,
    const TimePoint now,
    DeliveryPlan & plan)
  {
    if (active_ && feedback_callback_ &&
      event.kind == RelativeMotionEventKind::Running)
    {
      plan.feedback = true;
      plan.feedback_token = active_token_;
      plan.progress = event.progress;
      plan.feedback_callback = feedback_callback_;
    }
    if (!active_ || teardown_started_) {
      return;
    }
    if (event.kind == RelativeMotionEventKind::ZeroRequested) {
      teardown_started_ = true;
      plan.teardown = TeardownRequest{
        TeardownKind::Completion,
        ChildResultCode::Succeeded,
        MotionConditioningFailure::None,
        event.detail.empty() ? "relative-motion target reached" : event.detail,
        active_token_,
        now + policy_.stationarity_deadline};
      condition_variable_.notify_all();
      return;
    }
    if (event.kind == RelativeMotionEventKind::Failed) {
      const auto stop_event = controller_.request_safe_stop(
        RelativeMotionStopIntent::Failure, now);
      command_ = stop_event.command;
      teardown_started_ = true;
      plan.teardown = TeardownRequest{
        TeardownKind::Failure,
        child_code_for_relative(event.failure),
        conditioning_failure_for_relative(event.failure),
        event.detail.empty() ? "relative-motion controller failed" : event.detail,
        active_token_,
        now + policy_.stationarity_deadline};
      condition_variable_.notify_all();
    }
  }

  void dispatch(const DeliveryPlan & plan)
  {
    if (plan.feedback && plan.feedback_callback) {
      plan.feedback_callback(plan.feedback_token, plan.progress);
    }
    if (plan.teardown.has_value()) {
      (void)run_teardown(*plan.teardown, true);
    }
  }

  void finish_start_failure(
    const MotionToken & token,
    const MotionConditioningResult & conditioning_result)
  {
    auto code = child_code_for_conditioning(conditioning_result.failure);
    if (!conditioning_result.zero_proven ||
      conditioning_result.failure == MotionConditioningFailure::SafetyFault)
    {
      code = ChildResultCode::SafetyFault;
    }
    ResultCallback result_callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !same_token(active_token_, token)) {
        return;
      }
      active_ = false;
      starting_ = false;
      teardown_started_ = false;
      command_ = {};
      zero_proven_ = conditioning_result.zero_proven;
      result_callback = std::move(result_callback_);
      condition_variable_.notify_all();
    }
    if (result_callback) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Relative motion start failed: conditioning_failure=%u zero=%d detail=%s",
        static_cast<unsigned int>(conditioning_result.failure),
        conditioning_result.zero_proven ? 1 : 0,
        conditioning_result.detail.c_str());
      result_callback(
        token,
        ChildResult{
          code,
          conditioning_result.detail.empty() ?
          "conditioning generation could not start" : conditioning_result.detail});
    }
  }

  [[nodiscard]] bool wait_for_stationarity(
    const MotionToken & token,
    const TimePoint deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (active_ && same_token(active_token_, token)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return false;
      }
      if (last_odom_at_ == TimePoint{} ||
        now - last_odom_at_ > policy_.dependency_liveness_timeout)
      {
        const auto freshness_deadline = std::min(
          deadline,
          now + policy_.dependency_liveness_timeout);
        condition_variable_.wait_until(lock, freshness_deadline);
        continue;
      }
      const auto event = controller_.tick(now);
      if (event.kind == RelativeMotionEventKind::Completed &&
        event.stationarity_proven)
      {
        return true;
      }
      if (event.kind == RelativeMotionEventKind::Failed) {
        return false;
      }
      condition_variable_.wait_until(lock, deadline);
    }
    return false;
  }

  [[nodiscard]] bool stop_without_active_mission()
  {
    MotionConditioningResult result;
    try {
      result = conditioning_->stop();
    } catch (...) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      zero_proven_ = result.zero_proven;
      condition_variable_.notify_all();
    }
    return result.zero_proven && result.failure != MotionConditioningFailure::SafetyFault;
  }

  [[nodiscard]] bool run_teardown(
    const TeardownRequest & request,
    const bool deliver_result)
  {
    MotionConditioningResult conditioning_result;
    try {
      if (request.kind == TeardownKind::Completion ||
        request.kind == TeardownKind::Cancel)
      {
        conditioning_result = conditioning_->stop();
      } else if (conditioning_->state() == MotionConditioningState::Failed) {
        conditioning_result = conditioning_->last_result();
      } else {
        conditioning_result = conditioning_->fail(
          conditioning_token_, request.conditioning_failure, request.detail);
      }
    } catch (...) {
      conditioning_result = MotionConditioningResult{
        false, MotionConditioningState::Failed,
        MotionConditioningFailure::SafetyFault, false, false, {}, {},
        "conditioning teardown raised"};
    }

    const bool zero = conditioning_result.zero_proven;
    bool stationary = false;
    if (zero) {
      RelativeMotionEvent confirmation;
      const auto zero_proven_at = conditioning_result.zero_proven_at == TimePoint{} ?
      std::chrono::steady_clock::now() : conditioning_result.zero_proven_at;
      const auto stationarity_started_at = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stationarity_waiting_ = true;
        confirmation = controller_.confirm_gate_zero(
          zero_proven_at, stationarity_started_at);
        if (
          confirmation.kind != RelativeMotionEventKind::Failed &&
          latest_odom_.has_value() && last_odom_at_ != TimePoint{} &&
          stationarity_started_at - last_odom_at_ <=
          policy_.dependency_liveness_timeout)
        {
          confirmation = controller_.observe_odom(
            *latest_odom_, stationarity_started_at);
        }
        command_ = {};
        condition_variable_.notify_all();
      }
      if (confirmation.kind != RelativeMotionEventKind::Failed) {
        const auto stationarity_deadline = std::max(
          request.deadline,
          stationarity_started_at + policy_.stationarity_deadline);
        stationary = wait_for_stationarity(
          request.token, stationarity_deadline);
      }
    }

    ChildResultCode final_code = request.child_code;
    if (!zero || !stationary ||
      conditioning_result.failure == MotionConditioningFailure::SafetyFault)
    {
      final_code = ChildResultCode::SafetyFault;
    }
    const auto final_detail = !stationary ?
      std::string{"odometry did not prove stationarity after Gate zero"} :
    (conditioning_result.detail.empty() ? request.detail : conditioning_result.detail);

    ResultCallback result_callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stationarity_waiting_ = false;
      if (active_ && same_token(active_token_, request.token)) {
        active_ = false;
        starting_ = false;
        teardown_started_ = false;
        command_ = {};
        zero_proven_ = zero;
        result_callback = std::move(result_callback_);
        condition_variable_.notify_all();
      }
    }
    if (deliver_result && result_callback) {
      result_callback(request.token, ChildResult{final_code, final_detail});
    }
    const bool safe = zero && stationary &&
      conditioning_result.failure != MotionConditioningFailure::SafetyFault;
    if (!safe) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Relative motion teardown failed: zero=%d stationary=%d conditioning_failure=%u detail=%s",
        zero ? 1 : 0,
        stationary ? 1 : 0,
        static_cast<unsigned int>(conditioning_result.failure),
        final_detail.c_str());
    }
    return safe;
  }

  rclcpp::Node & node_;
  std::shared_ptr<MotionAuthorityPort> authority_;
  RelativeMotionPolicy policy_;
  MotionConditioningConfig conditioning_config_;
  RelativeMotionController controller_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<RawMotionProducer> producer_;
  std::unique_ptr<MotionConditioningPipeline> conditioning_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<Clock>::SharedPtr clock_subscription_;

  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
  bool active_{false};
  bool starting_{false};
  bool teardown_started_{false};
  bool stationarity_waiting_{false};
  MotionToken active_token_{};
  MissionStep active_step_{};
  MotionConditioningCorrelationToken conditioning_token_{};
  RelativeMotionCommand command_{};
  bool zero_proven_{true};
  FeedbackCallback feedback_callback_;
  ResultCallback result_callback_;

  bool odom_seen_{false};
  bool scan_seen_{false};
  bool clock_seen_{false};
  bool clock_advanced_{false};
  std::int64_t last_clock_stamp_{0};
  TimePoint last_odom_at_{};
  TimePoint last_scan_at_{};
  TimePoint last_clock_at_{};
  TimePoint last_clock_progress_at_{};
  std::optional<RelativeMotionOdom> latest_odom_;
};

RelativeMotionRosAdapter::RelativeMotionRosAdapter(
  rclcpp::Node & node,
  std::shared_ptr<MotionAuthorityPort> authority,
  RelativeMotionPolicy policy,
  MotionConditioningConfig conditioning_config)
: impl_(std::make_unique<Impl>(
    node, std::move(authority), std::move(policy),
    std::move(conditioning_config)))
{
}

RelativeMotionRosAdapter::~RelativeMotionRosAdapter() = default;

bool RelativeMotionRosAdapter::healthy() const
{
  return impl_->healthy();
}

void RelativeMotionRosAdapter::start(
  const MotionToken & token,
  const MissionStep & step,
  FeedbackCallback feedback,
  ResultCallback result)
{
  impl_->start(token, step, std::move(feedback), std::move(result));
}

bool RelativeMotionRosAdapter::cancel(
  const MotionToken & token,
  const SteadyClockPort::TimePoint deadline)
{
  return impl_->cancel(token, deadline);
}

void RelativeMotionRosAdapter::tick(const SteadyClockPort::TimePoint now)
{
  impl_->tick(now);
}

bool RelativeMotionRosAdapter::owns_authority_lifecycle() const noexcept
{
  return true;
}

bool RelativeMotionRosAdapter::zero_proven() const noexcept
{
  return impl_->zero_proven();
}

}  // namespace voice_nav_mission
