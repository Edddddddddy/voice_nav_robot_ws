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
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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

class AdapterIngressState;

class AdapterIngressGuard final
{
public:
  AdapterIngressGuard() = default;
  explicit AdapterIngressGuard(std::shared_ptr<AdapterIngressState> state);
  ~AdapterIngressGuard();

  AdapterIngressGuard(const AdapterIngressGuard &) = delete;
  AdapterIngressGuard & operator=(const AdapterIngressGuard &) = delete;
  AdapterIngressGuard(AdapterIngressGuard &&) = delete;
  AdapterIngressGuard & operator=(AdapterIngressGuard &&) = delete;

  [[nodiscard]] bool is_active() const noexcept
  {
    return active_;
  }

private:
  std::shared_ptr<AdapterIngressState> state_;
  bool active_{false};
};

class AdapterIngressState final
  : public std::enable_shared_from_this<AdapterIngressState>
{
public:
  using OdomHandler = std::function<void(const Odometry::ConstSharedPtr &)>;
  using ScanHandler = std::function<void()>;
  using ClockHandler = std::function<void(const Clock::ConstSharedPtr &)>;
  using CommandSupplier = std::function<RelativeMotionCommand()>;
  using Barrier = std::function<void()>;

  [[nodiscard]] AdapterIngressGuard enter()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_) {
      return {};
    }
    ++in_flight_;
    return AdapterIngressGuard(shared_from_this());
  }

  void disable()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    accepting_ = false;
  }

  void wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() {return in_flight_ == 0U;});
  }

  void set_odom_handler(OdomHandler handler)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_handler_ = std::move(handler);
  }

  void set_scan_handler(ScanHandler handler)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan_handler_ = std::move(handler);
  }

  void set_clock_handler(ClockHandler handler)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clock_handler_ = std::move(handler);
  }

  void set_command_supplier(CommandSupplier supplier)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_supplier_ = std::move(supplier);
  }

  void set_command_barrier(Barrier barrier)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_barrier_ = std::move(barrier);
  }

  [[nodiscard]] OdomHandler copy_odom_handler() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_handler_;
  }

  [[nodiscard]] ScanHandler copy_scan_handler() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_handler_;
  }

  [[nodiscard]] ClockHandler copy_clock_handler() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return clock_handler_;
  }

  [[nodiscard]] CommandSupplier copy_command_supplier() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_supplier_;
  }

  [[nodiscard]] Barrier copy_command_barrier() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_barrier_;
  }

private:
  friend class AdapterIngressGuard;

  void leave()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    --in_flight_;
    if (in_flight_ == 0U) {
      condition_.notify_all();
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool accepting_{true};
  std::size_t in_flight_{0U};
  OdomHandler odom_handler_;
  ScanHandler scan_handler_;
  ClockHandler clock_handler_;
  CommandSupplier command_supplier_;
  Barrier command_barrier_;
};

AdapterIngressGuard::AdapterIngressGuard(std::shared_ptr<AdapterIngressState> state)
: state_(std::move(state)), active_(state_ != nullptr)
{
}

AdapterIngressGuard::~AdapterIngressGuard()
{
  if (active_) {
    state_->leave();
  }
}

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
    case MotionConditioningFailure::Timeout:
      return ChildResultCode::Timeout;
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
      return MotionConditioningFailure::ExecutionFailed;
    case RelativeMotionFailure::Timeout:
      return MotionConditioningFailure::Timeout;
    case RelativeMotionFailure::SafetyFault:
      return MotionConditioningFailure::SafetyFault;
    case RelativeMotionFailure::InternalError:
      return MotionConditioningFailure::InternalError;
    case RelativeMotionFailure::None:
    default:
      return MotionConditioningFailure::InternalError;
  }
}

class RawMotionProducer final
  : public MotionProducerPort,
  public std::enable_shared_from_this<RawMotionProducer>
{
public:
  RawMotionProducer(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::shared_ptr<AdapterIngressState> ingress)
  : node_(node),
    callback_group_(std::move(callback_group)),
    ingress_(std::move(ingress))
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
      const auto weak_self = weak_from_this();
      const auto ingress = ingress_;
      timer_ = node_.create_wall_timer(
        50ms,
        [weak_self, ingress]() {
          auto guard = ingress->enter();
          if (!guard.is_active()) {
            return;
          }
          if (const auto self = weak_self.lock()) {
            self->publish_current();
          }
        },
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
    AdapterIngressState::CommandSupplier supplier;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !publisher_) {
        return;
      }
      publisher = publisher_;
    }
    const auto barrier = ingress_->copy_command_barrier();
    if (barrier) {
      barrier();
    }
    supplier = ingress_->copy_command_supplier();
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
  std::shared_ptr<AdapterIngressState> ingress_;
  std::mutex mutex_;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool active_{false};
};

}  // namespace

class RelativeMotionRosAdapter::Impl final
  : public std::enable_shared_from_this<RelativeMotionRosAdapter::Impl>
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
    ingress_(std::make_shared<AdapterIngressState>()),
    producer_(std::make_shared<RawMotionProducer>(
      node_, callback_group_, ingress_)),
    conditioning_(std::make_unique<MotionConditioningPipeline>(
      node_, authority_, producer_, conditioning_config_))
  {
    if (!authority_) {
      throw std::invalid_argument("RelativeMotionRosAdapter requires a Gate port");
    }
    if (!conditioning_config_.completion_relay) {
      throw std::invalid_argument(
        "RelativeMotionRosAdapter requires an external completion relay");
    }
  }

  void initialize()
  {
    const auto weak_impl = weak_from_this();
    ingress_->set_odom_handler(
      [weak_impl](const Odometry::ConstSharedPtr & message) {
        if (const auto impl = weak_impl.lock()) {
          impl->on_odom(message);
        }
      });
    ingress_->set_scan_handler(
      [weak_impl]() {
        if (const auto impl = weak_impl.lock()) {
          impl->on_scan();
        }
      });
    ingress_->set_clock_handler(
      [weak_impl](const Clock::ConstSharedPtr & message) {
        if (const auto impl = weak_impl.lock()) {
          impl->on_clock(message);
        }
      });
    ingress_->set_command_supplier(
      [weak_impl]() {
        if (const auto impl = weak_impl.lock()) {
          return impl->command();
        }
        return RelativeMotionCommand{};
      });
    ingress_->set_command_barrier(conditioning_config_.before_adapter_command_supplier);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto ingress = ingress_;
    odom_subscription_ = node_.create_subscription<Odometry>(
      conditioning_config_.odom_topic,
      rclcpp::SensorDataQoS(),
      [ingress](const Odometry::ConstSharedPtr message) {
        auto guard = ingress->enter();
        if (!guard.is_active()) {
          return;
        }
        const auto handler = ingress->copy_odom_handler();
        if (handler) {
          handler(message);
        }
      },
      options);
    scan_subscription_ = node_.create_subscription<LaserScan>(
      conditioning_config_.scan_topic,
      latest_sensor_qos(),
      [ingress](const LaserScan::ConstSharedPtr) {
        auto guard = ingress->enter();
        if (!guard.is_active()) {
          return;
        }
        const auto handler = ingress->copy_scan_handler();
        if (handler) {
          handler();
        }
      },
      options);
    clock_subscription_ = node_.create_subscription<Clock>(
      conditioning_config_.clock_topic,
      rclcpp::ClockQoS(),
      [ingress](const Clock::ConstSharedPtr message) {
        auto guard = ingress->enter();
        if (!guard.is_active()) {
          return;
        }
        const auto handler = ingress->copy_clock_handler();
        if (handler) {
          handler(message);
        }
      },
      options);
    transaction_thread_ = std::thread([this]() {
          transaction_loop();
      });
  }

  [[nodiscard]] bool start_raw_producer_for_test(const std::string & raw_topic)
  {
    return producer_ && producer_->start(raw_topic);
  }

  ~Impl()
  {
    shutdown();
  }

  void begin_shutdown() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_complete_) {
        return;
      }
      shutdown_delivery_requested_ = true;
      shutting_down_ = true;
    }
    // Stop new source callbacks, but retain the subscriptions, producer, and
    // conditioning Module until the internal terminal delivery has reached
    // the live Runtime queue/Core.
    ingress_->disable();
    request_emergency_stop();
    transaction_condition_.notify_all();
  }

  void wait_for_internal_completion() noexcept
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_variable_.wait(lock, [this]() {
        return !active_ && teardown_complete_ &&
               transaction_kind_ == TransactionKind::Idle &&
               !transaction_in_progress_ &&
               !emergency_stop_in_progress_ &&
               !completion_record_in_progress_;
      });
  }

  void finalize_shutdown() noexcept
  {
    begin_shutdown();
    wait_for_internal_completion();

    clock_subscription_.reset();
    scan_subscription_.reset();
    odom_subscription_.reset();
    if (conditioning_config_.before_adapter_ingress_wait) {
      conditioning_config_.before_adapter_ingress_wait();
    }
    ingress_->wait();
    try {
      if (producer_) {
        producer_->stop();
      }
    } catch (...) {
      // The conditioning teardown remains the unique cleanup owner; the
      // ingress barrier still prevents a callback from using released state.
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      transaction_stop_ = true;
    }
    transaction_condition_.notify_all();
    if (transaction_thread_.joinable()) {
      transaction_thread_.join();
    }
    join_emergency_thread();
    conditioning_.reset();
    producer_.reset();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_complete_ = true;
      condition_variable_.notify_all();
    }
  }

  void shutdown() noexcept
  {
    begin_shutdown();
    finalize_shutdown();
  }

  [[nodiscard]] bool healthy() const
  {
    const auto now = std::chrono::steady_clock::now();
    bool active = false;
    MotionConditioningState conditioning_state = MotionConditioningState::Failed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutting_down_ || !conditioning_) {
        return false;
      }
      active = active_;
      if (active) {
        return true;
      }
      if (!dependencies_fresh_locked(now)) {
        return false;
      }
      conditioning_state = conditioning_->state();
    }
    return conditioning_state != MotionConditioningState::Failed;
  }

  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result)
  {
    // Production RuntimeCore registers the delivery callback in the Node
    // registry before calling this method.  The Adapter receives an empty
    // result callback on that path and never stores a user delivery owner.
    (void)result;
    join_emergency_thread();
    bool rejected = false;
    ChildResultCode rejection_code = ChildResultCode::InternalError;
    std::string rejection_detail =
      "relative-motion generation was already active";
    const auto admission_allowed = [this, &token]() {
        return !conditioning_config_.admission_fence_check ||
               conditioning_config_.admission_fence_check(token.admission_epoch);
      };
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool fenced = !admission_allowed();
      if (fenced || shutting_down_ || active_ || transaction_kind_ != TransactionKind::Idle) {
        rejected = true;
        if (fenced) {
          rejection_code = ChildResultCode::SafetyFault;
          rejection_detail =
            "relative-motion start rejected by the active admission fence";
        } else if (shutting_down_) {
          rejection_code = ChildResultCode::SafetyFault;
          rejection_detail =
            "relative-motion start rejected during adapter shutdown";
        }
      } else {
        active_ = true;
        starting_ = true;
        teardown_started_ = false;
        active_token_ = token;
        active_step_ = step;
        feedback_callback_ = std::move(feedback);
        completion_record_sent_ = false;
        conditioning_token_ = {};
        command_ = {};
        zero_proven_ = false;
        teardown_complete_ = false;
        teardown_safe_ = false;
        stationarity_waiting_ = false;
        pending_teardown_.reset();
        cancel_requested_.store(false);
        transaction_kind_ = TransactionKind::Start;
      }
    }
    if (rejected) {
      publish_completion(std::make_shared<const RelativeMotionCompletionRecord>(
          RelativeMotionCompletionRecord{
          token, ChildResult{rejection_code, rejection_detail}}));
      return;
    }

    transaction_condition_.notify_one();
  }

  [[nodiscard]] bool cancel(const MotionToken & token, const TimePoint deadline)
  {
    std::optional<TeardownRequest> emergency_request;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        if (transaction_kind_ == TransactionKind::Idle &&
          !emergency_stop_in_progress_ && teardown_complete_ &&
          teardown_safe_ && zero_proven_)
        {
          return true;
        }
        if (transaction_kind_ == TransactionKind::Idle &&
          !emergency_stop_in_progress_)
        {
          zero_proven_ = false;
          teardown_complete_ = false;
          teardown_safe_ = false;
          cancel_requested_.store(true);
          pending_teardown_ = TeardownRequest{
            TeardownKind::Cancel,
            ChildResultCode::Failed,
            MotionConditioningFailure::None,
            "relative-motion cleanup requested",
            token,
            deadline,
            false};
          transaction_kind_ = TransactionKind::Teardown;
        }
      } else if (!same_token(active_token_, token)) {
        return true;
      } else {
        cancel_requested_.store(true);
        if (!teardown_started_) {
          const auto now = std::chrono::steady_clock::now();
          if (!starting_) {
            const auto event = controller_.request_safe_stop(
              RelativeMotionStopIntent::Cancel, now);
            command_ = event.command;
          } else {
            command_ = {};
          }
          teardown_started_ = true;
          teardown_complete_ = false;
          teardown_safe_ = false;
          pending_teardown_ = TeardownRequest{
            TeardownKind::Cancel,
            ChildResultCode::Failed,
            MotionConditioningFailure::None,
            "relative-motion cancellation requested",
            active_token_,
            deadline,
            shutting_down_};
          if (starting_ && !emergency_stop_in_progress_) {
            emergency_request = *pending_teardown_;
            pending_teardown_.reset();
            emergency_stop_in_progress_ = true;
          }
        } else if (pending_teardown_.has_value()) {
          pending_teardown_->kind = TeardownKind::Cancel;
          pending_teardown_->child_code = ChildResultCode::Failed;
          pending_teardown_->conditioning_failure = MotionConditioningFailure::None;
          pending_teardown_->detail = "relative-motion cancellation requested";
          pending_teardown_->deadline = deadline;
          pending_teardown_->deliver_result = shutting_down_;
        }
        if (transaction_kind_ != TransactionKind::Start) {
          transaction_kind_ = TransactionKind::Teardown;
        }
        if (starting_ && !emergency_stop_in_progress_ &&
          pending_teardown_.has_value())
        {
          emergency_request = *pending_teardown_;
          pending_teardown_.reset();
          emergency_stop_in_progress_ = true;
        }
      }
    }
    if (emergency_request.has_value()) {
      launch_emergency_teardown(std::move(*emergency_request));
    }
    transaction_condition_.notify_one();
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_variable_.wait_until(
      lock, deadline, [this]() {return !active_ && teardown_complete_;}) &&
           teardown_safe_;
  }

  void request_emergency_stop() noexcept
  {
    MotionToken token;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      token = active_token_;
    }
    try {
      (void)cancel(token, std::chrono::steady_clock::now());
    } catch (...) {
      // The caller that owns the independent path remains fail-closed even
      // when a best-effort transaction request cannot be materialized.
    }
  }

  [[nodiscard]] bool emergency_stop(const TimePoint deadline)
  {
    MotionToken token;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      token = active_token_;
    }
    return cancel(token, deadline);
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
        now_again + policy_.stationarity_deadline,
        true};
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

  enum class TransactionKind : std::uint8_t
  {
    Idle = 0,
    Start = 1,
    Teardown = 2,
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
    bool deliver_result{true};
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

  void join_emergency_thread()
  {
    std::thread thread;
    {
      std::lock_guard<std::mutex> lock(emergency_thread_mutex_);
      if (emergency_teardown_thread_.joinable()) {
        thread = std::move(emergency_teardown_thread_);
      }
    }
    if (thread.joinable()) {
      thread.join();
    }
  }

  void launch_emergency_teardown(TeardownRequest request)
  {
    const auto fallback_request = request;
    try {
      std::lock_guard<std::mutex> lock(emergency_thread_mutex_);
      if (emergency_teardown_thread_.joinable()) {
        throw std::logic_error("previous emergency teardown thread was not joined");
      }
      emergency_teardown_thread_ = std::thread(
        [this, request = std::move(request)]() mutable {
          try {
            (void)run_teardown(request);
          } catch (...) {
            fail_transaction("emergency relative-motion teardown raised");
          }
          {
            std::lock_guard<std::mutex> lock(mutex_);
            emergency_stop_in_progress_ = false;
            condition_variable_.notify_all();
          }
        });
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      emergency_stop_in_progress_ = false;
      // Preserve the request if thread creation itself fails.  The fallback
      // remains on the serial transaction worker and still fails closed.
      pending_teardown_ = fallback_request;
      transaction_kind_ = TransactionKind::Teardown;
      transaction_condition_.notify_one();
    }
  }

  void transaction_loop()
  {
    for (;; ) {
      TransactionKind kind = TransactionKind::Idle;
      std::optional<TeardownRequest> teardown;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        transaction_condition_.wait(lock, [this]() {
            return transaction_stop_ || transaction_kind_ != TransactionKind::Idle;
          });
        if (transaction_stop_) {
          return;
        }
        kind = transaction_kind_;
        transaction_kind_ = TransactionKind::Idle;
        transaction_in_progress_ = true;
        if (kind == TransactionKind::Teardown) {
          teardown = std::move(pending_teardown_);
          pending_teardown_.reset();
        }
      }

      try {
        if (kind == TransactionKind::Start) {
          run_start_transaction();
        } else if (kind == TransactionKind::Teardown && teardown.has_value()) {
          (void)run_teardown(*teardown);
        }
      } catch (const std::exception & error) {
        fail_transaction(
          std::string{"relative-motion transaction raised: "} + error.what());
      } catch (...) {
        fail_transaction("relative-motion transaction raised an unknown exception");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        transaction_in_progress_ = false;
        condition_variable_.notify_all();
      }
    }
  }

  void fail_transaction(std::string detail)
  {
    TeardownRequest request;
    bool schedule = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutting_down_) {
        return;
      }
      if (active_ && !teardown_started_) {
        cancel_requested_.store(true);
        teardown_started_ = true;
        command_ = {};
        request = TeardownRequest{
          TeardownKind::Failure,
          ChildResultCode::SafetyFault,
          MotionConditioningFailure::SafetyFault,
          std::move(detail),
          active_token_,
          std::chrono::steady_clock::now() + policy_.stationarity_deadline,
          true};
        pending_teardown_ = request;
        transaction_kind_ = TransactionKind::Teardown;
        schedule = true;
      } else if (!active_ && transaction_kind_ == TransactionKind::Idle) {
        zero_proven_ = false;
        teardown_complete_ = false;
        teardown_safe_ = false;
        cancel_requested_.store(true);
        request = TeardownRequest{
          TeardownKind::Failure,
          ChildResultCode::SafetyFault,
          MotionConditioningFailure::SafetyFault,
          std::move(detail),
          {},
          std::chrono::steady_clock::now() + policy_.stationarity_deadline,
          false};
        pending_teardown_ = request;
        transaction_kind_ = TransactionKind::Teardown;
        schedule = true;
      }
    }
    if (schedule) {
      transaction_condition_.notify_one();
    }
  }

  [[nodiscard]] bool start_is_current(const MotionToken & token) const
  {
    return active_ && starting_ && same_token(active_token_, token) &&
           !cancel_requested_.load() && !shutting_down_;
  }

  void run_start_transaction()
  {
    MotionToken token;
    MissionStep step;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !starting_ || cancel_requested_.load() || shutting_down_) {
        return;
      }
      token = active_token_;
      step = active_step_;
    }

    MotionConditioningResult prepared;
    try {
      prepared = conditioning_->prepare();
    } catch (...) {
      prepared = MotionConditioningResult{
        false, MotionConditioningState::Failed,
        MotionConditioningFailure::InternalError, false, false, {}, {},
        "conditioning PREPARE raised"};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!start_is_current(token)) {
        return;
      }
    }
    if (!prepared.ok || prepared.state != MotionConditioningState::Prepared) {
      finish_start_failure(token, prepared);
      return;
    }

    MotionConditioningResult started;
    try {
      started = conditioning_->start();
    } catch (...) {
      started = MotionConditioningResult{
        false, MotionConditioningState::Failed,
        MotionConditioningFailure::InternalError, false, false, {}, {},
        "conditioning OPEN raised"};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!start_is_current(token)) {
        return;
      }
    }
    if (!started.ok || started.state != MotionConditioningState::Running) {
      finish_start_failure(token, started);
      return;
    }

    const auto conditioning_token = conditioning_->correlation_token();
    DeliveryPlan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!start_is_current(token)) {
        return;
      }
      conditioning_token_ = conditioning_token;
      const auto now = std::chrono::steady_clock::now();
      auto event = controller_.start(token, step, now);
      if (latest_odom_.has_value() && odom_seen_ &&
        now - last_odom_at_ <= policy_.dependency_liveness_timeout)
      {
        event = controller_.observe_odom(*latest_odom_, last_odom_at_);
      }
      starting_ = false;
      command_ = event.command;
      plan_from_event_locked(event, now, plan);
      condition_variable_.notify_all();
    }
    dispatch(plan);
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
    if (conditioning_config_.before_adapter_odom_callback) {
      conditioning_config_.before_adapter_odom_callback();
    }
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
    if (conditioning_config_.before_adapter_scan_callback) {
      conditioning_config_.before_adapter_scan_callback();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    scan_seen_ = true;
    last_scan_at_ = std::chrono::steady_clock::now();
    condition_variable_.notify_all();
  }

  void on_clock(const Clock::ConstSharedPtr & message)
  {
    if (conditioning_config_.before_adapter_clock_callback) {
      conditioning_config_.before_adapter_clock_callback();
    }
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
      try {
        plan.feedback_callback(plan.feedback_token, plan.progress);
      } catch (...) {
        fail_transaction("relative-motion feedback delivery raised");
      }
    }
    if (plan.teardown.has_value()) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_teardown_.has_value() && teardown_started_) {
        return;
      }
      pending_teardown_ = *plan.teardown;
      transaction_kind_ = TransactionKind::Teardown;
      transaction_condition_.notify_one();
    }
  }

  void publish_completion(RelativeMotionCompletionRecordPtr record)
  {
    RelativeMotionCompletionRelay relay;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      relay = conditioning_config_.completion_relay;
      completion_record_in_progress_ = true;
    }

    bool accepted = false;
    try {
      accepted = relay && relay(std::move(record));
    } catch (...) {
      accepted = false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      completion_record_in_progress_ = false;
      condition_variable_.notify_all();
    }
    if (!accepted) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Relative motion completion relay rejected a terminal record");
    }
  }

  void finish_start_failure(
    const MotionToken & token,
    const MotionConditioningResult & conditioning_result)
  {
    const bool zero_proven = conditioning_result.zero_proven &&
      conditioning_result.zero_proven_at != TimePoint{};
    auto code = child_code_for_conditioning(conditioning_result.failure);
    if (!zero_proven ||
      conditioning_result.failure == MotionConditioningFailure::SafetyFault)
    {
      code = ChildResultCode::SafetyFault;
    }
    RelativeMotionCompletionRecordPtr completion;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !same_token(active_token_, token) ||
        cancel_requested_.load() || shutting_down_)
      {
        return;
      }
      active_ = false;
      starting_ = false;
      teardown_started_ = false;
      command_ = {};
      zero_proven_ = zero_proven;
      teardown_complete_ = true;
      teardown_safe_ = zero_proven &&
        conditioning_result.failure != MotionConditioningFailure::SafetyFault;
      completion_record_sent_ = true;
      completion = std::make_shared<const RelativeMotionCompletionRecord>(
        RelativeMotionCompletionRecord{
          token,
          ChildResult{
            code,
            conditioning_result.detail.empty() ?
            "conditioning generation could not start" : conditioning_result.detail}});
      condition_variable_.notify_all();
    }
    RCLCPP_ERROR(
      node_.get_logger(),
      "Relative motion start failed: conditioning_failure=%u zero=%d detail=%s",
      static_cast<unsigned int>(conditioning_result.failure),
      zero_proven ? 1 : 0,
      conditioning_result.detail.c_str());
    publish_completion(std::move(completion));
  }

  [[nodiscard]] bool wait_for_stationarity(
    const MotionToken & token,
    const TimePoint zero_proven_at,
    const TimePoint deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (active_ && same_token(active_token_, token)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return false;
      }
      if (last_odom_at_ == TimePoint{} || last_odom_at_<zero_proven_at ||
        now - last_odom_at_> policy_.dependency_liveness_timeout)
      {
        condition_variable_.wait_until(lock, deadline);
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

  [[nodiscard]] bool run_teardown(
    const TeardownRequest & request)
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

    const auto zero_proven_at = conditioning_result.zero_proven_at;
    const bool zero = conditioning_result.zero_proven &&
      zero_proven_at != TimePoint{};
    bool stationary = false;
    bool mission_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mission_active = active_ && same_token(active_token_, request.token) &&
        controller_.active();
    }
    stationary = !mission_active;
    if (zero && mission_active) {
      RelativeMotionEvent confirmation;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stationarity_waiting_ = true;
        confirmation = controller_.confirm_gate_zero(zero_proven_at);
        if (
          confirmation.kind != RelativeMotionEventKind::Failed &&
          latest_odom_.has_value() && last_odom_at_ != TimePoint{} &&
          last_odom_at_ >= zero_proven_at &&
          std::chrono::steady_clock::now() - last_odom_at_ <=
          policy_.dependency_liveness_timeout)
        {
          confirmation = controller_.observe_odom(
            *latest_odom_, last_odom_at_);
        }
        command_ = {};
        condition_variable_.notify_all();
      }
      if (confirmation.kind != RelativeMotionEventKind::Failed) {
        const auto stationarity_deadline =
          zero_proven_at + policy_.stationarity_deadline;
        stationary = wait_for_stationarity(
          request.token, zero_proven_at, stationarity_deadline);
      }
    }

    ChildResultCode final_code = request.child_code;
    if (!zero || (mission_active && !stationary) ||
      conditioning_result.failure == MotionConditioningFailure::SafetyFault)
    {
      final_code = ChildResultCode::SafetyFault;
    }
    const auto final_detail = !zero ?
      std::string{"Gate zero proof did not include a valid steady timestamp"} :
    ((mission_active && !stationary) ?
    std::string{"odometry did not prove stationarity after Gate zero"} :
    (conditioning_result.detail.empty() ? request.detail : conditioning_result.detail));

    RelativeMotionCompletionRecordPtr completion;
    bool deliver_result = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      deliver_result = request.deliver_result ||
        shutdown_delivery_requested_;
      deliver_result = deliver_result && !completion_record_sent_;
      stationarity_waiting_ = false;
      if (!active_ || same_token(active_token_, request.token)) {
        active_ = false;
        starting_ = false;
        teardown_started_ = false;
        command_ = {};
        zero_proven_ = zero;
        teardown_complete_ = true;
        teardown_safe_ = zero && stationary &&
          conditioning_result.failure != MotionConditioningFailure::SafetyFault;
        if (deliver_result) {
          completion_record_sent_ = true;
          completion = std::make_shared<const RelativeMotionCompletionRecord>(
            RelativeMotionCompletionRecord{
              request.token,
              ChildResult{final_code, final_detail}});
        }
        condition_variable_.notify_all();
      }
    }
    if (deliver_result && completion) {
      publish_completion(std::move(completion));
    }
    const bool safe = zero && (!mission_active || stationary) &&
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
  std::shared_ptr<AdapterIngressState> ingress_;
  std::shared_ptr<RawMotionProducer> producer_;
  std::unique_ptr<MotionConditioningPipeline> conditioning_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<Clock>::SharedPtr clock_subscription_;

  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::condition_variable transaction_condition_;
  std::mutex emergency_thread_mutex_;
  std::thread transaction_thread_;
  std::thread emergency_teardown_thread_;
  TransactionKind transaction_kind_{TransactionKind::Idle};
  std::optional<TeardownRequest> pending_teardown_;
  bool transaction_stop_{false};
  bool transaction_in_progress_{false};
  bool shutting_down_{false};
  bool shutdown_delivery_requested_{false};
  bool shutdown_complete_{false};
  bool completion_record_in_progress_{false};
  bool completion_record_sent_{false};
  bool emergency_stop_in_progress_{false};
  std::atomic<bool> cancel_requested_{false};
  bool active_{false};
  bool starting_{false};
  bool teardown_started_{false};
  bool stationarity_waiting_{false};
  bool teardown_complete_{true};
  bool teardown_safe_{true};
  MotionToken active_token_{};
  MissionStep active_step_{};
  MotionConditioningCorrelationToken conditioning_token_{};
  RelativeMotionCommand command_{};
  bool zero_proven_{true};
  FeedbackCallback feedback_callback_;

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
: impl_(std::make_shared<Impl>(
    node, std::move(authority), std::move(policy),
    std::move(conditioning_config)))
{
  impl_->initialize();
}

RelativeMotionRosAdapter::~RelativeMotionRosAdapter()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

bool RelativeMotionRosAdapter::healthy() const
{
  return impl_->healthy();
}

bool RelativeMotionRosAdapter::uses_external_completion_registry() const noexcept
{
  return true;
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

void RelativeMotionRosAdapter::request_emergency_stop() noexcept
{
  if (impl_) {
    impl_->request_emergency_stop();
  }
}

bool RelativeMotionRosAdapter::emergency_stop(
  const SteadyClockPort::TimePoint deadline)
{
  return impl_ && impl_->emergency_stop(deadline);
}

void RelativeMotionRosAdapter::begin_shutdown() noexcept
{
  if (impl_) {
    impl_->begin_shutdown();
  }
}

void RelativeMotionRosAdapter::wait_for_internal_completion() noexcept
{
  if (impl_) {
    impl_->wait_for_internal_completion();
  }
}

void RelativeMotionRosAdapter::finalize_shutdown() noexcept
{
  if (impl_) {
    impl_->finalize_shutdown();
  }
}

void RelativeMotionRosAdapter::shutdown() noexcept
{
  if (impl_) {
    impl_->shutdown();
  }
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

bool detail::RelativeMotionRosAdapterTestAccess::start_raw_producer(
  RelativeMotionRosAdapter & adapter,
  const std::string & raw_topic)
{
  return adapter.impl_ && adapter.impl_->start_raw_producer_for_test(raw_topic);
}

}  // namespace voice_nav_mission
