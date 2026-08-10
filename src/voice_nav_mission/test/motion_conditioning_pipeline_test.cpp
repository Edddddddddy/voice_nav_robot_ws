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

#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "voice_nav_mission/mission_authority_convergence.hpp"
#include "voice_nav_mission/motion_conditioning_pipeline.hpp"
#include "voice_nav_mission/runtime_shutdown_coordinator.hpp"
#include "voice_nav_mission/runtime_transaction_plane.hpp"

extern char ** environ;

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;
using LoadNode = composition_interfaces::srv::LoadNode;
using ListNodes = composition_interfaces::srv::ListNodes;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using CollisionState = nav2_msgs::msg::CollisionMonitorState;

constexpr char kCi64RenewChildEnvironment[] = "VOICE_NAV_CI64_03_CHILD";
constexpr char kCi64RenewReadyToken[] = "VOICE_NAV_CI64_READY\n";
constexpr int kCi64RenewChildLivenessFd = 200;

struct ChildProcessResult final
{
  bool spawned{false};
  bool ready{false};
  bool setup_failure{false};
  bool hang_deadline_started{false};
  bool timed_out{false};
  bool early_exit{false};
  bool killed{false};
  bool reaped{false};
  bool exited{false};
  int exit_code{-1};
  bool signaled{false};
  int signal_number{-1};
};

ChildProcessResult run_renew_drain_child(
  const std::string & mode,
  const std::chrono::milliseconds setup_budget,
  const std::chrono::milliseconds hang_budget)
{
  ChildProcessResult result;
  int pipe_fds[2] = {-1, -1};
  if (pipe(pipe_fds) != 0) {
    return result;
  }
  const int read_flags = fcntl(pipe_fds[0], F_GETFD);
  if (read_flags < 0 || fcntl(pipe_fds[0], F_SETFD, read_flags | FD_CLOEXEC) != 0 ||
    fcntl(pipe_fds[0], F_SETFL, fcntl(pipe_fds[0], F_GETFL) | O_NONBLOCK) != 0)
  {
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return result;
  }

  std::vector<std::string> environment;
  for (char ** entry = environ; entry != nullptr && *entry != nullptr; ++entry) {
    const std::string value(*entry);
    if (value.rfind(std::string(kCi64RenewChildEnvironment) + "=", 0) == 0 ||
      value.rfind("ROS_DOMAIN_ID=", 0) == 0)
    {
      continue;
    }
    environment.push_back(value);
  }
  environment.push_back(
    std::string(kCi64RenewChildEnvironment) + "=" + mode);
  environment.push_back(
    "ROS_DOMAIN_ID=" + std::to_string(100 + static_cast<int>(getpid() % 100)));

  std::vector<char *> environment_pointers;
  environment_pointers.reserve(environment.size() + 1U);
  for (auto & value : environment) {
    environment_pointers.push_back(value.data());
  }
  environment_pointers.push_back(nullptr);

  const std::string executable = "/proc/self/exe";
  const std::string filter =
    "--gtest_filter=MotionConditioningPipelineTest.DestructorDrainsQueuedRenewCallback";
  std::vector<char *> arguments{const_cast<char *>(executable.c_str()),
    const_cast<char *>(filter.c_str()), nullptr};

  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0) {
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return result;
  }
  if (posix_spawn_file_actions_adddup2(
      &actions, pipe_fds[1], kCi64RenewChildLivenessFd) != 0 ||
    posix_spawn_file_actions_addclose(&actions, pipe_fds[0]) != 0 ||
    posix_spawn_file_actions_addclose(&actions, pipe_fds[1]) != 0)
  {
    posix_spawn_file_actions_destroy(&actions);
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return result;
  }

  pid_t child_pid = 0;
  const int spawn_status = posix_spawn(
    &child_pid, executable.c_str(), &actions, nullptr,
    arguments.data(), environment_pointers.data());
  posix_spawn_file_actions_destroy(&actions);
  close(pipe_fds[1]);
  if (spawn_status != 0) {
    close(pipe_fds[0]);
    return result;
  }
  result.spawned = true;

  const auto reap_child = [&]() {
      int child_status = 0;
      pid_t waited_pid;
      do {
        waited_pid = waitpid(child_pid, &child_status, 0);
      } while (waited_pid < 0 && errno == EINTR);
      result.reaped = waited_pid == child_pid;
      if (result.reaped) {
        result.exited = WIFEXITED(child_status);
        result.exit_code = result.exited ? WEXITSTATUS(child_status) : -1;
        result.signaled = WIFSIGNALED(child_status);
        result.signal_number = result.signaled ? WTERMSIG(child_status) : -1;
      }
    };
  const auto kill_child = [&]() {
      const int kill_status = kill(child_pid, SIGKILL);
      result.killed = kill_status == 0 || (kill_status < 0 && errno == ESRCH);
    };

  const auto setup_deadline = std::chrono::steady_clock::now() + setup_budget;
  std::string ready_data;
  bool eof = false;
  bool read_error = false;
  while (!result.ready && !result.setup_failure) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= setup_deadline) {
      result.setup_failure = true;
      result.timed_out = true;
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      setup_deadline - now);
    const auto bounded_milliseconds = std::max<std::int64_t>(1, remaining.count());
    const auto poll_timeout = static_cast<int>(std::min<std::int64_t>(
        bounded_milliseconds, std::numeric_limits<int>::max()));
    pollfd descriptor{pipe_fds[0], POLLIN | POLLHUP | POLLERR, 0};
    int poll_status;
    do {
      poll_status = poll(&descriptor, 1, poll_timeout);
    } while (poll_status < 0 && errno == EINTR);
    if (poll_status < 0) {
      result.setup_failure = true;
      read_error = true;
      break;
    }
    if (poll_status == 0) {
      result.setup_failure = true;
      result.timed_out = true;
      break;
    }

    if ((descriptor.revents & POLLIN) != 0) {
      for (;; ) {
        char buffer[64] = {};
        const auto count = read(pipe_fds[0], buffer, sizeof(buffer));
        if (count > 0) {
          ready_data.append(buffer, static_cast<std::size_t>(count));
          if (ready_data.size() > sizeof(kCi64RenewReadyToken) - 1U ||
            ready_data.compare(0, ready_data.size(), kCi64RenewReadyToken,
            ready_data.size()) != 0)
          {
            result.setup_failure = true;
            break;
          }
          if (ready_data.size() == sizeof(kCi64RenewReadyToken) - 1U) {
            result.ready = true;
          }
          continue;
        }
        if (count == 0) {
          eof = true;
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          read_error = true;
        }
        break;
      }
    }

    if (result.setup_failure) {
      break;
    }
    const bool hangup = (descriptor.revents & POLLHUP) != 0;
    const bool error = (descriptor.revents & (POLLERR | POLLNVAL)) != 0;
    if (!result.ready && (eof || hangup || error || read_error)) {
      result.setup_failure = true;
    }
    if (result.ready && (error || read_error)) {
      result.setup_failure = true;
      result.ready = false;
    }
  }

  if (result.setup_failure) {
    kill_child();
    reap_child();
    close(pipe_fds[0]);
    return result;
  }

  const auto completion_budget = mode == "normal" ? setup_budget : hang_budget;
  const auto completion_deadline = std::chrono::steady_clock::now() + completion_budget;
  if (mode != "normal") {
    result.hang_deadline_started = true;
  }
  bool child_closed_liveness_pipe = false;
  while (!child_closed_liveness_pipe) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= completion_deadline) {
      result.timed_out = true;
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      completion_deadline - now);
    const auto bounded_milliseconds = std::max<std::int64_t>(1, remaining.count());
    const auto poll_timeout = static_cast<int>(std::min<std::int64_t>(
        bounded_milliseconds, std::numeric_limits<int>::max()));
    pollfd descriptor{pipe_fds[0], POLLIN | POLLHUP | POLLERR, 0};
    int poll_status;
    do {
      poll_status = poll(&descriptor, 1, poll_timeout);
    } while (poll_status < 0 && errno == EINTR);
    if (poll_status < 0) {
      result.timed_out = true;
      break;
    }
    if (poll_status > 0 &&
      (descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0)
    {
      child_closed_liveness_pipe = true;
      result.early_exit = mode != "normal" && !result.timed_out;
    }
  }

  if (result.timed_out) {
    kill_child();
  }
  reap_child();
  close(pipe_fds[0]);
  return result;
}

bool write_renew_ready_token()
{
  std::size_t written = 0U;
  constexpr std::size_t token_size = sizeof(kCi64RenewReadyToken) - 1U;
  while (written < token_size) {
    const auto count = write(
      kCi64RenewChildLivenessFd,
      kCi64RenewReadyToken + written,
      token_size - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

class CallbackBarrier final
{
public:
  void arm()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = true;
    entered_ = false;
    released_ = false;
  }

  bool wait_for_entry(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  void operator()()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!armed_) {
      return;
    }
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool armed_{false};
  bool entered_{false};
  bool released_{false};
};

class TransactionOperationBarrier final
{
public:
  explicit TransactionOperationBarrier(const RuntimeTransactionSideEffect target)
  : target_(target)
  {
  }

  void operator()(const RuntimeTransactionSideEffect side_effect)
  {
    if (side_effect != target_) {
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  bool wait_for_entry(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

private:
  RuntimeTransactionSideEffect target_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_{false};
  bool released_{false};
};

class TransactionQuiesceBarrier final
{
public:
  void operator()(const std::uint64_t generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_ = generation;
    entered_ = true;
    condition_.notify_all();
  }

  bool wait_for_entry(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return entered_;});
  }

  void complete(const bool result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = result;
    completed_ = true;
    condition_.notify_all();
  }

  bool wait_for_completion(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return completed_;});
  }

  std::optional<bool> result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_;
  }

  std::uint64_t generation() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t generation_{0U};
  bool entered_{false};
  bool completed_{false};
  std::optional<bool> result_;
};

class CallbackCounter final
{
public:
  void expect(std::size_t count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = count;
    count_ = 0U;
  }

  void operator()()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (target_ != 0U) {
      ++count_;
      condition_.notify_all();
    }
  }

  bool wait_for_target(std::chrono::milliseconds timeout = 2s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {
               return count_ >= target_;
      });
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t target_{0U};
  std::size_t count_{0U};
};

class FakeAuthority final : public MotionAuthorityPort
{
public:
  GateSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  AuthorityResult prepare(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Prepare);
    if (block_prepare_) {
      prepare_entered_ = true;
      prepare_cv_.notify_all();
      prepare_cv_.wait(lock, [this]() {return release_prepare_;});
    }
    ++generation_;
    snapshot_.control_seq++;
    snapshot_.lease_id = "lease-" + std::to_string(generation_);
    snapshot_.candidate_topic = "/candidate/lease_" + std::to_string(generation_);
    snapshot_.state = GateState::Prepared;
    snapshot_.motion_inhibited = true;
    snapshot_.zero_selected = true;
    snapshot_.zero_published = prepare_zero_proof_;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, prepare_zero_proof_, false, snapshot_, snapshot_.lease_id,
      "prepared"};
  }

  AuthorityResult open(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Open);
    if (block_open_) {
      open_entered_ = true;
      open_cv_.notify_all();
      open_cv_.wait(lock, [this]() {return release_open_;});
    }
    if (throw_on_open_) {
      throw std::runtime_error("scripted MotionGate OPEN failure");
    }
    snapshot_.control_seq++;
    snapshot_.state = GateState::Armed;
    snapshot_.motion_inhibited = false;
    snapshot_.zero_selected = false;
    snapshot_.authority_live = true;
    snapshot_.writer_bound = writer_bound_;
    return AuthorityResult{
      true, false, false, snapshot_, snapshot_.lease_id, "opened"};
  }

  AuthorityResult renew(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Renew);
    if (block_renew_) {
      renew_entered_ = true;
      renew_cv_.notify_all();
      renew_cv_.wait(lock, [this]() {return release_renew_;});
    }
    if (throw_on_renew_) {
      throw std::runtime_error("scripted MotionGate RENEW failure");
    }
    snapshot_.control_seq++;
    ++renew_count_;
    snapshot_.authority_live = authority_live_ &&
      (!renew_failure_after_.has_value() || renew_count_ <= *renew_failure_after_);
    return AuthorityResult{
      true, false, false, snapshot_, snapshot_.lease_id, "renewed"};
  }

  AuthorityResult inhibit(const AuthorityOperation &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(AuthorityOperationKind::Inhibit);
    if (block_inhibit_) {
      inhibit_entered_ = true;
      inhibit_cv_.notify_all();
      inhibit_cv_.wait(lock, [this]() {return release_inhibit_;});
    }
    snapshot_.control_seq++;
    snapshot_.state = GateState::Inhibited;
    snapshot_.motion_inhibited = true;
    snapshot_.zero_selected = true;
    snapshot_.zero_published = inhibit_zero_proof_;
    snapshot_.authority_live = false;
    snapshot_.writer_bound = false;
    return AuthorityResult{
      true, inhibit_zero_proof_, false, snapshot_, snapshot_.lease_id,
      "inhibited"};
  }

  void set_writer_bound(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer_bound_ = value;
  }

  void block_prepare()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_prepare_ = true;
    prepare_entered_ = false;
    release_prepare_ = false;
  }

  bool wait_for_prepare(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return prepare_cv_.wait_for(
      lock, timeout, [this]() {return prepare_entered_;});
  }

  void release_blocked_prepare()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_prepare_ = true;
    prepare_cv_.notify_all();
  }

  void set_authority_live(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_live_ = value;
  }

  void set_prepare_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prepare_zero_proof_ = value;
  }

  void set_inhibit_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    inhibit_zero_proof_ = value;
  }

  void set_initial_zero_proof(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.zero_published = value;
  }

  void set_renew_failure_after(std::size_t successful_renews)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    renew_failure_after_ = successful_renews;
  }

  void set_throw_on_open(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    throw_on_open_ = value;
  }

  void block_open()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_open_ = true;
    open_entered_ = false;
    release_open_ = false;
  }

  bool wait_for_open(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return open_cv_.wait_for(
      lock, timeout, [this]() {return open_entered_;});
  }

  void release_blocked_open()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_open_ = true;
    open_cv_.notify_all();
  }

  void set_throw_on_renew(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    throw_on_renew_ = value;
  }

  void block_renew()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_renew_ = true;
    renew_entered_ = false;
    release_renew_ = false;
  }

  bool wait_for_renew(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return renew_cv_.wait_for(
      lock, timeout, [this]() {return renew_entered_;});
  }

  void release_blocked_renew()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_renew_ = true;
    renew_cv_.notify_all();
  }

  void set_initial_armed_snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.state = GateState::Armed;
    snapshot_.motion_inhibited = false;
    snapshot_.zero_selected = false;
    snapshot_.zero_published = false;
    snapshot_.authority_live = true;
    snapshot_.writer_bound = true;
  }

  void block_inhibit()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_inhibit_ = true;
    inhibit_entered_ = false;
    release_inhibit_ = false;
  }

  bool wait_for_inhibit(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return inhibit_cv_.wait_for(
      lock, timeout, [this]() {return inhibit_entered_;});
  }

  void release_blocked_inhibit()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_inhibit_ = true;
    inhibit_cv_.notify_all();
  }

  std::vector<AuthorityOperationKind> calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  std::size_t renew_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return renew_count_;
  }

private:
  mutable std::mutex mutex_;
  GateSnapshot snapshot_{
    "gate-test", 1U, {}, GateState::Inhibited, true, true, true, true,
    {}, false, false};
  std::uint64_t generation_{0U};
  bool writer_bound_{true};
  bool authority_live_{true};
  bool prepare_zero_proof_{true};
  bool block_prepare_{false};
  bool prepare_entered_{false};
  bool release_prepare_{false};
  bool inhibit_zero_proof_{true};
  std::size_t renew_count_{0U};
  std::optional<std::size_t> renew_failure_after_;
  bool throw_on_open_{false};
  bool block_open_{false};
  bool open_entered_{false};
  bool release_open_{false};
  bool throw_on_renew_{false};
  bool block_renew_{false};
  bool renew_entered_{false};
  bool release_renew_{false};
  bool block_inhibit_{false};
  bool inhibit_entered_{false};
  bool release_inhibit_{false};
  std::condition_variable inhibit_cv_;
  std::condition_variable prepare_cv_;
  std::condition_variable renew_cv_;
  std::condition_variable open_cv_;
  std::vector<AuthorityOperationKind> calls_;
};

bool startup_calls_only_reassert_inhibit(
  const std::vector<AuthorityOperationKind> & calls)
{
  return std::all_of(
    calls.cbegin(), calls.cend(), [](const auto kind) {
      return kind == AuthorityOperationKind::Inhibit;
    });
}

class FakeProducer final : public MotionProducerPort
{
public:
  bool start(const std::string & raw_topic) override
  {
    started_topics.push_back(raw_topic);
    ++start_count;
    if (block_start) {
      std::unique_lock<std::mutex> lock(start_mutex);
      start_entered = true;
      start_cv.notify_all();
      start_cv.wait(lock, [this]() {return release_start;});
    }
    if (throw_on_start) {
      throw std::runtime_error("scripted producer start failure");
    }
    return allow_start;
  }

  void stop() override
  {
    {
      std::lock_guard<std::mutex> lock(stop_mutex);
      ++stop_count;
    }
    stop_cv.notify_all();
    if (throw_on_stop) {
      throw std::runtime_error("scripted producer stop failure");
    }
  }

  bool wait_for_start(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(start_mutex);
    return start_cv.wait_for(lock, timeout, [this]() {return start_entered;});
  }

  void release_blocked_start()
  {
    std::lock_guard<std::mutex> lock(start_mutex);
    release_start = true;
    start_cv.notify_all();
  }

  bool wait_for_stop_count(
    std::size_t expected,
    std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(stop_mutex);
    return stop_cv.wait_for(lock, timeout, [this, expected]() {
               return stop_count >= expected;
      });
  }

  bool allow_start{true};
  bool throw_on_start{false};
  bool throw_on_stop{false};
  bool block_start{false};
  std::size_t start_count{0U};
  std::size_t stop_count{0U};
  std::vector<std::string> started_topics;

private:
  std::mutex start_mutex;
  std::condition_variable start_cv;
  bool start_entered{false};
  bool release_start{false};
  std::mutex stop_mutex;
  std::condition_variable stop_cv;
};

class FakeComponentGraph final
{
public:
  FakeComponentGraph()
  : container_(std::make_shared<rclcpp::Node>("motion_conditioning_container")),
    collision_(std::make_shared<rclcpp::Node>("collision_monitor")),
    smoother_(std::make_shared<rclcpp::Node>("velocity_smoother")),
    sensors_(std::make_shared<rclcpp::Node>("conditioning_sensors")),
    load_callback_group_(container_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant)),
    unload_callback_group_(container_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant))
  {
    load_service_ = container_->create_service<LoadNode>(
      "/motion_conditioning_container/_container/load_node",
      [this](
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response) {
        std::this_thread::sleep_for(load_delay_);
        response->success = true;
        response->unique_id = next_id_++;
        response->full_node_name = wrong_fqn_ ?
        "/wrong_" + request->node_name : "/" + request->node_name;
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          loaded_[response->unique_id] = response->full_node_name;
        }
        if (request->node_name == "collision_monitor") {
          collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
          collision_candidate_ = collision_->create_generic_publisher(
            parameter_string(request, "cmd_vel_out_topic"),
            "geometry_msgs/msg/TwistStamped", rclcpp::QoS(1));
          collision_events_ = collision_->create_publisher<CollisionState>(
            parameter_string(request, "state_topic"), rclcpp::QoS(1));
        } else if (request->node_name == "velocity_smoother") {
          smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
        }
        graph_condition_.notify_all();
      }, rmw_qos_profile_services_default, load_callback_group_);
    list_nodes_service_ = container_->create_service<ListNodes>(
      "/motion_conditioning_container/_container/list_nodes",
      [this](
        const std::shared_ptr<ListNodes::Request>,
        std::shared_ptr<ListNodes::Response> response) {
        std::this_thread::sleep_for(list_delay_);
        std::lock_guard<std::mutex> lock(graph_mutex_);
        if (list_override_enabled_) {
          response->unique_ids = list_override_ids_;
          response->full_node_names = list_override_fqns_;
          return;
        }
        for (const auto & entry : loaded_) {
          response->unique_ids.push_back(entry.first);
          response->full_node_names.push_back(entry.second);
        }
      });
    unload_service_ = container_->create_service<UnloadNode>(
      "/motion_conditioning_container/_container/unload_node",
      [this](
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response) {
        std::chrono::milliseconds delay;
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          unload_requests_.push_back(request->unique_id);
          const auto delay_iterator = unload_delays_.find(request->unique_id);
          delay = delay_iterator != unload_delays_.cend() ?
          delay_iterator->second : unload_delay_;
        }
        std::this_thread::sleep_for(delay);
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          unload_completed_.push_back(request->unique_id);
          ++unload_count_;
          const bool forced_failure = unload_failures_.find(request->unique_id) !=
          unload_failures_.cend();
          response->success = !forced_failure &&
          loaded_.erase(request->unique_id) == 1U;
        }
        if (response->success && !retain_candidate_writer_) {
          collision_candidate_.reset();
          collision_events_.reset();
          collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
          smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
        graph_condition_.notify_all();
      }, rmw_qos_profile_services_default, unload_callback_group_);
    controller_service_ = container_->create_service<ListControllers>(
      "/controller_manager/list_controllers",
      [this](
        const std::shared_ptr<ListControllers::Request>,
        std::shared_ptr<ListControllers::Response> response) {
        controller_manager_msgs::msg::ControllerState controller;
        controller.name = "diff_drive_controller";
        controller.state = controller_active_ ? "active" : "inactive";
        response->controller = {controller};
      });

    create_lifecycle_services(
      collision_, "/collision_monitor", &collision_state_);
    create_lifecycle_services(
      smoother_, "/velocity_smoother", &smoother_state_);
    scan_publisher_ = sensors_->create_generic_publisher(
      "/scan", "sensor_msgs/msg/LaserScan", rclcpp::QoS(1));
    odom_publisher_ = sensors_->create_generic_publisher(
      "/odom", "nav_msgs/msg/Odometry", rclcpp::QoS(1));
    scan_message_publisher_ = sensors_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
    odom_message_publisher_ = sensors_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS());
    clock_publisher_ = sensors_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    health_timer_ = sensors_->create_wall_timer(20ms, [this]() {
          rosgraph_msgs::msg::Clock clock;
          if (publish_clock_) {
            {
              std::lock_guard<std::mutex> lock(health_mutex_);
              clock.clock = freeze_clock_ ?
              frozen_clock_ : rclcpp::Clock(RCL_SYSTEM_TIME).now();
            }
            clock_publisher_->publish(clock);
          }
          sensor_msgs::msg::LaserScan scan;
          scan.header.stamp = clock.clock;
          scan.ranges = {10.0F, 10.0F};
          if (publish_scan_) {
            scan_message_publisher_->publish(scan);
          }
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = clock.clock;
          if (publish_odom_) {
            odom_message_publisher_->publish(odom);
          }
        });
  }

  std::vector<rclcpp::Node::SharedPtr> nodes() const
  {
    return {container_, collision_, smoother_, sensors_};
  }

  std::size_t loaded_count() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return loaded_.size();
  }

  std::size_t unload_count() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_count_;
  }

  bool wait_for_loaded_count(
    std::size_t expected,
    std::chrono::milliseconds timeout = 2s) const
  {
    std::unique_lock<std::mutex> lock(graph_mutex_);
    return graph_condition_.wait_for(lock, timeout, [this, expected]() {
               return loaded_.size() >= expected;
      });
  }

  bool wait_for_empty(std::chrono::milliseconds timeout = 2s) const
  {
    std::unique_lock<std::mutex> lock(graph_mutex_);
    return graph_condition_.wait_for(lock, timeout, [this]() {
               return loaded_.empty();
      });
  }

  void set_activation_delay(std::chrono::milliseconds delay)
  {
    activation_delay_ = delay;
  }

  void set_load_delay(std::chrono::milliseconds delay)
  {
    load_delay_ = delay;
  }

  void set_unload_delay(std::chrono::milliseconds delay)
  {
    unload_delay_ = delay;
  }

  void set_unload_delay_for(
    std::uint64_t unique_id,
    std::chrono::milliseconds delay)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    unload_delays_[unique_id] = delay;
  }

  std::vector<std::uint64_t> unload_requests() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_requests_;
  }

  std::vector<std::uint64_t> unload_completed() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return unload_completed_;
  }

  rclcpp::Publisher<CollisionState>::SharedPtr collision_publisher() const
  {
    return collision_events_;
  }

  void publish_collision_stop()
  {
    CollisionState message;
    message.action_type = CollisionState::STOP;
    message.polygon_name = "stop_zone";
    if (collision_events_) {
      collision_events_->publish(message);
    }
  }

  void set_wrong_fqn(bool value)
  {
    wrong_fqn_ = value;
  }

  void set_list_delay(std::chrono::milliseconds delay)
  {
    list_delay_ = delay;
  }

  void set_list_override(
    std::vector<std::uint64_t> unique_ids,
    std::vector<std::string> node_fqns)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    list_override_ids_ = std::move(unique_ids);
    list_override_fqns_ = std::move(node_fqns);
    list_override_enabled_ = true;
  }

  void clear_list_override()
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    list_override_ids_.clear();
    list_override_fqns_.clear();
    list_override_enabled_ = false;
  }

  void set_unload_failure(std::uint64_t unique_id)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    unload_failures_.insert(unique_id);
  }

  void set_lifecycle_failure(const std::string & node_fqn, bool value)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (node_fqn == "/collision_monitor") {
      collision_lifecycle_failure_ = value;
    } else if (node_fqn == "/velocity_smoother") {
      smoother_lifecycle_failure_ = value;
    }
  }

  void seed_orphan_components(
    const std::string & candidate_topic =
    "/voice_nav_internal/motion_gate/candidate/old")
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    loaded_[41U] = "/collision_monitor";
    loaded_[42U] = "/velocity_smoother";
    next_id_ = 43U;
    collision_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    smoother_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    collision_candidate_ = collision_->create_generic_publisher(
      candidate_topic, "geometry_msgs/msg/TwistStamped", rclcpp::QoS(1));
  }

  void set_retain_candidate_writer(bool value)
  {
    retain_candidate_writer_ = value;
  }

  void set_health_sources(bool scan, bool odom, bool clock)
  {
    publish_scan_ = scan;
    publish_odom_ = odom;
    publish_clock_ = clock;
  }

  void publish_health_once()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    if (publish_clock_) {
      clock_publisher_->publish(clock);
      if (clock.clock.nanosec == 999999999U) {
        ++clock.clock.sec;
        clock.clock.nanosec = 0U;
      } else {
        ++clock.clock.nanosec;
      }
      clock_publisher_->publish(clock);
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = clock.clock;
    scan.ranges = {10.0F, 10.0F};
    if (publish_scan_) {
      scan_message_publisher_->publish(scan);
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = clock.clock;
    if (publish_odom_) {
      odom_message_publisher_->publish(odom);
    }
  }

  void set_clock_frozen(bool value)
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    freeze_clock_ = value;
    if (value) {
      frozen_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }
  }

  void enable_activation_barrier()
  {
    std::lock_guard<std::mutex> lock(activation_mutex_);
    activation_barrier_ = true;
    activation_entered_ = false;
    activation_released_ = false;
  }

  void wait_for_activation_entry()
  {
    std::unique_lock<std::mutex> lock(activation_mutex_);
    activation_cv_.wait(lock, [this]() {return activation_entered_;});
  }

  void release_activation()
  {
    std::lock_guard<std::mutex> lock(activation_mutex_);
    activation_released_ = true;
    activation_cv_.notify_all();
  }

  void remove_lifecycle_services(bool collision, bool smoother)
  {
    if (!collision) {
      change_services_[0].reset();
      get_services_[0].reset();
    }
    if (!smoother) {
      change_services_[1].reset();
      get_services_[1].reset();
    }
  }

  void set_controller_active(bool value)
  {
    controller_active_ = value;
  }

private:
  static std::string parameter_string(
    const std::shared_ptr<LoadNode::Request> & request,
    const std::string & name)
  {
    for (const auto & value : request->parameters) {
      if (value.name == name && value.value.type ==
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      {
        return value.value.string_value;
      }
    }
    return {};
  }

  void create_lifecycle_services(
    const rclcpp::Node::SharedPtr & node,
    const std::string & fqn,
    std::uint8_t * state)
  {
    change_services_.push_back(node->create_service<ChangeState>(
      fqn + "/change_state",
        [this, state, fqn](
          const std::shared_ptr<ChangeState::Request> request,
          std::shared_ptr<ChangeState::Response> response) {
          {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if ((fqn == "/collision_monitor" && collision_lifecycle_failure_) ||
            (fqn == "/velocity_smoother" && smoother_lifecycle_failure_))
            {
              response->success = false;
              return;
            }
          }
          switch (request->transition.id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
              if (fqn == "/collision_monitor") {
                std::unique_lock<std::mutex> lock(activation_mutex_);
                if (activation_barrier_) {
                  activation_entered_ = true;
                  activation_cv_.notify_all();
                  activation_cv_.wait(
                    lock, [this]() {return activation_released_;});
                }
              }
              std::this_thread::sleep_for(activation_delay_);
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
              response->success = true;
              return;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
            case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
              *state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
              response->success = true;
              return;
            default:
              response->success = false;
              return;
          }
      }));
    get_services_.push_back(node->create_service<GetState>(
      fqn + "/get_state",
        [state](
          const std::shared_ptr<GetState::Request>,
          std::shared_ptr<GetState::Response> response) {
          response->current_state.id = *state;
          response->current_state.label = "fake";
      }));
  }

  rclcpp::Node::SharedPtr container_;
  rclcpp::Node::SharedPtr collision_;
  rclcpp::Node::SharedPtr smoother_;
  rclcpp::Node::SharedPtr sensors_;
  rclcpp::Service<LoadNode>::SharedPtr load_service_;
  rclcpp::Service<ListNodes>::SharedPtr list_nodes_service_;
  rclcpp::Service<UnloadNode>::SharedPtr unload_service_;
  rclcpp::CallbackGroup::SharedPtr load_callback_group_;
  rclcpp::CallbackGroup::SharedPtr unload_callback_group_;
  rclcpp::Service<ListControllers>::SharedPtr controller_service_;
  std::vector<rclcpp::Service<ChangeState>::SharedPtr> change_services_;
  std::vector<rclcpp::Service<GetState>::SharedPtr> get_services_;
  std::unordered_map<std::uint64_t, std::string> loaded_;
  mutable std::mutex graph_mutex_;
  mutable std::condition_variable graph_condition_;
  std::uint64_t next_id_{1U};
  std::size_t unload_count_{0U};
  std::vector<std::uint64_t> unload_requests_;
  std::vector<std::uint64_t> unload_completed_;
  std::unordered_map<std::uint64_t, std::chrono::milliseconds> unload_delays_;
  std::uint8_t collision_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  std::uint8_t smoother_state_{
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  std::chrono::milliseconds activation_delay_{};
  std::chrono::milliseconds load_delay_{};
  std::chrono::milliseconds list_delay_{};
  std::chrono::milliseconds unload_delay_{};
  std::unordered_set<std::uint64_t> unload_failures_;
  std::vector<std::uint64_t> list_override_ids_;
  std::vector<std::string> list_override_fqns_;
  bool list_override_enabled_{false};
  bool collision_lifecycle_failure_{false};
  bool smoother_lifecycle_failure_{false};
  bool wrong_fqn_{false};
  bool retain_candidate_writer_{false};
  bool publish_scan_{true};
  bool publish_odom_{true};
  bool publish_clock_{true};
  bool controller_active_{true};
  bool freeze_clock_{false};
  rclcpp::Time frozen_clock_;
  mutable std::mutex health_mutex_;
  std::mutex activation_mutex_;
  std::condition_variable activation_cv_;
  bool activation_barrier_{false};
  bool activation_entered_{false};
  bool activation_released_{false};
  rclcpp::GenericPublisher::SharedPtr collision_candidate_;
  rclcpp::Publisher<CollisionState>::SharedPtr collision_events_;
  rclcpp::GenericPublisher::SharedPtr scan_publisher_;
  rclcpp::GenericPublisher::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_message_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_message_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

class MotionConditioningPipelineTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    graph = std::make_unique<FakeComponentGraph>();
    graph->set_health_sources(true, true, false);
    authority = std::make_shared<FakeAuthority>();
    producer = std::make_shared<FakeProducer>();
    client = std::make_shared<rclcpp::Node>(
      "conditioning_client",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));
    collision_state_publisher = client->create_publisher<
      nav2_msgs::msg::CollisionMonitorState>(
      "/voice_nav_internal/motion/collision_state",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, 4U);
    for (const auto & node : graph->nodes()) {
      executor->add_node(node);
    }
    executor->add_node(client);
    spinning = true;
    spin_thread = std::thread([this]() {
          executor->spin();
    });
  }

  void TearDown() override
  {
    spinning = false;
    executor->cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor->remove_node(client);
    for (const auto & node : graph->nodes()) {
      executor->remove_node(node);
    }
    client.reset();
    collision_state_publisher.reset();
    producer.reset();
    authority.reset();
    graph.reset();
    executor.reset();
  }

  MotionConditioningConfig config(bool enable_clock = true)
  {
    graph->set_health_sources(true, true, enable_clock);
    MotionConditioningConfig value;
    value.component_rpc_timeout = 200ms;
    value.writer_graph_timeout = 200ms;
    value.prepare_open_deadline = 1s;
    value.renew_period = 10ms;
    value.control_response_deadline = 100ms;
    value.stop_barrier = 100ms;
    auto request = std::make_shared<std::uint64_t>(0U);
    value.request_id_generator = [request]() {
        return std::string(31U, '0') +
               static_cast<char>('1' + (*request)++ % 8U);
      };
    return value;
  }

  bool wait_for_candidate_writer(
    const std::string & topic,
    std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto publishers = client->get_publishers_info_by_topic(topic);
      const auto found = std::any_of(
        publishers.cbegin(), publishers.cend(), [](const auto & endpoint) {
          return endpoint.node_name() == "collision_monitor" &&
                 endpoint.topic_type() == "geometry_msgs/msg/TwistStamped";
        });
      if (found) {
        return true;
      }
      std::this_thread::yield();
    }
    return false;
  }

  bool wait_for_no_candidate_writer(
    const std::string & topic,
    std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto publishers = client->get_publishers_info_by_topic(topic);
      const auto found = std::any_of(
        publishers.cbegin(), publishers.cend(), [](const auto & endpoint) {
          return endpoint.node_name() == "collision_monitor" &&
                 endpoint.topic_type() == "geometry_msgs/msg/TwistStamped";
        });
      if (!found) {
        return true;
      }
      std::this_thread::yield();
    }
    return false;
  }

  std::unique_ptr<FakeComponentGraph> graph;
  std::shared_ptr<FakeAuthority> authority;
  std::shared_ptr<FakeProducer> producer;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr
    collision_state_publisher;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  bool spinning{false};
  std::thread spin_thread;
};

TEST_F(MotionConditioningPipelineTest, StartupReconciliationCleansOrphanComponents)
{
  graph->seed_orphan_components();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto prepared = pipeline.prepare();

  ASSERT_TRUE(prepared.ok) << prepared.detail;
  const auto unload_requests = graph->unload_requests();
  ASSERT_GE(unload_requests.size(), 2U);
  EXPECT_EQ(unload_requests[0], 41U);
  EXPECT_EQ(unload_requests[1], 42U);
  EXPECT_EQ(graph->loaded_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, StartupReconciliationCleanStartDoesNotUnload)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto prepared = pipeline.prepare();

  ASSERT_TRUE(prepared.ok) << prepared.detail;
  EXPECT_TRUE(graph->unload_requests().empty());
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 2);
  ASSERT_GE(calls.size(), 2U);
  EXPECT_EQ(calls[0], AuthorityOperationKind::Inhibit);
  EXPECT_EQ(calls[1], AuthorityOperationKind::Inhibit);
}

TEST_F(
  MotionConditioningPipelineTest,
  StartupRequiresCurrentGateZeroReassertBeforeCommit)
{
  // The cached snapshot remains inhibited+zero, but the next current Gate
  // transaction proves that zero is no longer available.  Startup must not
  // commit from the stale snapshot or advance into PREPARE/OPEN/RENEW.
  authority->set_inhibit_zero_proof(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();
  const auto calls = authority->calls();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_GE(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 1U);
  EXPECT_TRUE(std::all_of(
    calls.cbegin(), calls.cend(), [](const auto kind) {
        return kind == AuthorityOperationKind::Inhibit;
    }));
}

TEST_F(MotionConditioningPipelineTest, StartupUnknownFqnFailsClosedBeforeGatePrepare)
{
  graph->set_list_override({41U}, {"/unrelated_component"});
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();
  const auto calls = authority->calls();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(calls));
  EXPECT_TRUE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, StartupZeroUniqueIdFailsClosedBeforeGatePrepare)
{
  graph->set_list_override({0U}, {"/collision_monitor"});
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_TRUE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, StartupDuplicateIdentityFailsClosedBeforeGatePrepare)
{
  graph->set_list_override(
    {41U, 41U}, {"/collision_monitor", "/velocity_smoother"});
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_TRUE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, StartupConflictingFqnIdentityFailsClosed)
{
  graph->set_list_override(
    {41U, 42U}, {"/collision_monitor", "/collision_monitor"});
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_TRUE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, StartupListNodesTimeoutFailsClosed)
{
  graph->set_list_delay(300ms);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 50ms;
  pipeline_config.startup_reconciliation_timeout = 100ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_TRUE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, StartupLifecycleFailureFailsClosedBeforeUnload)
{
  graph->seed_orphan_components();
  graph->set_lifecycle_failure("/collision_monitor", true);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_TRUE(graph->unload_requests().empty());
  EXPECT_EQ(graph->loaded_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, StartupUnloadFailureFailsClosed)
{
  graph->seed_orphan_components();
  graph->set_unload_failure(41U);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();
  const auto unload_requests = graph->unload_requests();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  ASSERT_EQ(unload_requests.size(), 1U);
  EXPECT_EQ(unload_requests.front(), 41U);
  EXPECT_EQ(graph->loaded_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, StartupWriterResidueFailsClosed)
{
  graph->seed_orphan_components();
  graph->set_retain_candidate_writer(true);
  auto pipeline_config = config();
  pipeline_config.startup_reconciliation_timeout = 300ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(authority->calls()));
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_FALSE(graph->unload_requests().empty());
}

TEST_F(MotionConditioningPipelineTest, GateCandidateAndWriterBindingAreRequired)
{
  authority->set_writer_bound(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto prepared = pipeline.prepare();
  ASSERT_TRUE(prepared.ok);
  EXPECT_EQ(prepared.candidate_topic, "/candidate/lease_1");
  EXPECT_EQ(prepared.lease_id, "lease-1");

  const auto started = pipeline.start();
  EXPECT_FALSE(started.ok);
  EXPECT_EQ(started.state, MotionConditioningState::Failed);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_EQ(started.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(started.zero_proven);
  EXPECT_NE(started.zero_proven_at, std::chrono::steady_clock::time_point{});

  const auto calls = authority->calls();
  ASSERT_GE(calls.size(), 5U);
  EXPECT_EQ(calls[0], AuthorityOperationKind::Inhibit);
  EXPECT_EQ(calls[1], AuthorityOperationKind::Inhibit);
  EXPECT_EQ(calls[2], AuthorityOperationKind::Prepare);
  EXPECT_EQ(calls[3], AuthorityOperationKind::Open);
  EXPECT_EQ(calls.back(), AuthorityOperationKind::Inhibit);
}

TEST_F(MotionConditioningPipelineTest, PrepareRequiresGateZeroProof)
{
  authority->set_prepare_zero_proof(false);
  authority->set_inhibit_zero_proof(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(producer->start_count, 0U);
}

TEST_F(MotionConditioningPipelineTest, NoLeaseRequiresCurrentGateZeroProof)
{
  authority->set_initial_zero_proof(false);
  authority->set_inhibit_zero_proof(false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();
  const auto calls = authority->calls();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_TRUE(startup_calls_only_reassert_inhibit(calls));
}

TEST_F(MotionConditioningPipelineTest, StopWithoutLeaseRequiresInhibitedZeroSnapshot)
{
  authority->set_initial_armed_snapshot();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.stop();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(producer->stop_count, 1U);
}

TEST_F(
  MotionConditioningPipelineTest,
  GateLossTerminalRearmsOnlyAfterReplacementZeroProof)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  const auto prepared = pipeline.prepare();
  ASSERT_TRUE(prepared.ok) << prepared.detail;
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  const auto started = pipeline.start();
  ASSERT_TRUE(started.ok) << started.detail;
  const auto token = pipeline.correlation_token();

  authority->set_inhibit_zero_proof(false);
  const auto failed = pipeline.fail(
    token, MotionConditioningFailure::SafetyFault, "Gate process disappeared");
  ASSERT_FALSE(failed.ok);
  EXPECT_EQ(failed.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(failed.zero_proven);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);

  const auto replacement = authority->snapshot();
  EXPECT_FALSE(pipeline.rearm_after_gate_replacement(replacement));

  authority->set_initial_zero_proof(true);
  const auto zero_proven_replacement = authority->snapshot();
  EXPECT_TRUE(pipeline.rearm_after_gate_replacement(zero_proven_replacement));
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Stopped);
  const auto rearmed = pipeline.last_result();
  EXPECT_TRUE(rearmed.ok);
  EXPECT_TRUE(rearmed.zero_proven);
  EXPECT_EQ(rearmed.failure, MotionConditioningFailure::None);
}

TEST_F(MotionConditioningPipelineTest, CollisionStopIsReportedAndFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);

  graph->publish_collision_stop();

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(pipeline.state(), MotionConditioningState::Failed);
  ASSERT_TRUE(pipeline.last_result().collision_stop);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::ExecutionFailed);
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
  EXPECT_NE(
    pipeline.last_result().zero_proven_at,
    std::chrono::steady_clock::time_point{});
}

TEST_F(MotionConditioningPipelineTest, LeaseLossDuringActivationNeverStartsProducer)
{
  graph->set_activation_delay(150ms);
  authority->set_renew_failure_after(1U);
  auto pipeline_config = config();
  pipeline_config.renew_period = 20ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(producer->start_count, 0U);
  const auto calls = authority->calls();
  EXPECT_GE(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Renew), 2);
}

TEST_F(
  MotionConditioningPipelineTest,
  RenewTimerWaitsForArmedActivationCommitAcrossPrepareAndOpen)
{
  auto renew_mutex = std::make_shared<std::mutex>();
  auto renew_condition = std::make_shared<std::condition_variable>();
  auto renew_callbacks = std::make_shared<std::size_t>(0U);
  auto armed_commit_barrier = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::Open);
  auto transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    0U,
    RuntimeTransactionPlane::BeforeCommit{},
    RuntimeTransactionPlane::BeforeOperation{},
    RuntimeTransactionPlane::QuiesceObserver{},
    [armed_commit_barrier](const RuntimeTransactionSideEffect side_effect) {
      (*armed_commit_barrier)(side_effect);
    });
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.renew_period = 1ms;
  pipeline_config.transaction_plane = transaction_plane;
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  pipeline_config.before_renew_callback = [
    renew_mutex, renew_condition, renew_callbacks]() {
      {
        std::lock_guard<std::mutex> lock(*renew_mutex);
        ++(*renew_callbacks);
      }
      renew_condition->notify_all();
    };
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);

  authority->block_prepare();
  MotionConditioningResult prepare_result;
  std::thread prepare_thread([&]() {
      prepare_result = pipeline.prepare();
    });
  const bool prepare_entered = authority->wait_for_prepare();
  EXPECT_TRUE(prepare_entered);
  {
    std::lock_guard<std::mutex> lock(*renew_mutex);
    EXPECT_EQ(*renew_callbacks, 0U);
  }
  authority->release_blocked_prepare();
  prepare_thread.join();
  ASSERT_TRUE(prepare_result.ok) << prepare_result.detail;
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());

  authority->block_open();
  MotionConditioningResult start_result;
  std::thread start_thread([&]() {
      start_result = pipeline.start();
    });
  const bool open_entered = authority->wait_for_open();
  EXPECT_TRUE(open_entered);
  {
    std::lock_guard<std::mutex> lock(*renew_mutex);
    EXPECT_EQ(*renew_callbacks, 0U);
  }
  authority->release_blocked_open();

  const bool commit_entered = armed_commit_barrier->wait_for_entry();
  EXPECT_TRUE(commit_entered);
  if (!commit_entered) {
    armed_commit_barrier->release();
    start_thread.join();
    ADD_FAILURE() << "start did not reach the Open activation commit: "
                  << start_result.detail;
    return;
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Prepared);
  {
    std::lock_guard<std::mutex> lock(*renew_mutex);
    EXPECT_EQ(*renew_callbacks, 0U);
  }

  armed_commit_barrier->release();
  start_thread.join();
  ASSERT_TRUE(start_result.ok) << start_result.detail;
  {
    std::unique_lock<std::mutex> lock(*renew_mutex);
    ASSERT_TRUE(renew_condition->wait_for(
      lock, 1s, [&]() {return *renew_callbacks != 0U;}));
  }
}

TEST_F(
  MotionConditioningPipelineTest,
  TransactionQuiesceAtPrepareOperationWaitsForCompletion)
{
  auto operation = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::Prepare);
  auto quiesce = std::make_shared<TransactionQuiesceBarrier>();
  auto pipeline_config = config();
  pipeline_config.transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    1U,
    RuntimeTransactionPlane::BeforeCommit{},
    [operation](const RuntimeTransactionSideEffect side_effect) {
      (*operation)(side_effect);
    },
    [quiesce](const std::uint64_t generation) {
      (*quiesce)(generation);
    });
  pipeline_config.transaction_generation_provider = []() {return 1U;};
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  std::optional<MotionConditioningResult> result;
  std::thread prepare_thread([&]() {result = pipeline.prepare();});
  ASSERT_TRUE(operation->wait_for_entry());
  std::thread quiesce_thread([&]() {
      quiesce->complete(pipeline_config.transaction_plane->quiesce(
        2U, std::chrono::steady_clock::now() + 1s));
    });
  ASSERT_TRUE(quiesce->wait_for_entry());
  EXPECT_EQ(quiesce->generation(), 2U);
  EXPECT_FALSE(quiesce->wait_for_completion(100ms));
  operation->release();
  prepare_thread.join();
  quiesce_thread.join();

  ASSERT_TRUE(quiesce->result().has_value());
  EXPECT_TRUE(*quiesce->result());
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ok);
  EXPECT_EQ(result->state, MotionConditioningState::Failed);
  EXPECT_EQ(result->failure, MotionConditioningFailure::SafetyFault);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(calls.cbegin(), calls.cend(), AuthorityOperationKind::Prepare), 1);
  EXPECT_EQ(
    std::count(calls.cbegin(), calls.cend(), AuthorityOperationKind::Open), 0);
  EXPECT_EQ(producer->start_count, 0U);
  const auto stopped = pipeline.stop();
  EXPECT_TRUE(stopped.zero_proven);
  const auto snapshot = authority->snapshot();
  EXPECT_EQ(snapshot.state, GateState::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_published);
}

TEST_F(
  MotionConditioningPipelineTest,
  TransactionQuiesceAtOpenOperationWaitsAndFencesControllerStart)
{
  auto operation = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::Open);
  auto quiesce = std::make_shared<TransactionQuiesceBarrier>();
  auto pipeline_config = config();
  pipeline_config.transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    1U,
    RuntimeTransactionPlane::BeforeCommit{},
    [operation](const RuntimeTransactionSideEffect side_effect) {
      (*operation)(side_effect);
    },
    [quiesce](const std::uint64_t generation) {
      (*quiesce)(generation);
    });
  pipeline_config.transaction_generation_provider = []() {return 1U;};
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);

  std::optional<MotionConditioningResult> result;
  std::thread start_thread([&]() {result = pipeline.start();});
  const auto entered = operation->wait_for_entry(2s);
  if (!entered) {
    operation->release();
    start_thread.join();
  }
  ASSERT_TRUE(entered);
  std::thread quiesce_thread([&]() {
      quiesce->complete(pipeline_config.transaction_plane->quiesce(
        2U, std::chrono::steady_clock::now() + 1s));
    });
  ASSERT_TRUE(quiesce->wait_for_entry());
  EXPECT_EQ(quiesce->generation(), 2U);
  EXPECT_FALSE(quiesce->wait_for_completion(100ms));
  operation->release();
  start_thread.join();
  quiesce_thread.join();

  ASSERT_TRUE(quiesce->result().has_value());
  EXPECT_TRUE(*quiesce->result());
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ok);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(calls.cbegin(), calls.cend(), AuthorityOperationKind::Open), 1);
  EXPECT_EQ(producer->start_count, 0U);
  const auto snapshot = authority->snapshot();
  EXPECT_EQ(snapshot.state, GateState::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_published);
}

TEST_F(
  MotionConditioningPipelineTest,
  TransactionQuiesceAtControllerStartWaitsForProducerStart)
{
  auto operation = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::ControllerStart);
  auto quiesce = std::make_shared<TransactionQuiesceBarrier>();
  auto pipeline_config = config();
  auto health_ready = std::make_shared<CallbackCounter>();
  pipeline_config.transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    1U,
    RuntimeTransactionPlane::BeforeCommit{},
    [operation](const RuntimeTransactionSideEffect side_effect) {
      (*operation)(side_effect);
    },
    [quiesce](const std::uint64_t generation) {
      (*quiesce)(generation);
    });
  pipeline_config.transaction_generation_provider = []() {return 1U;};
  pipeline_config.prepare_open_deadline = 4s;
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());

  std::optional<MotionConditioningResult> result;
  std::thread start_thread([&]() {result = pipeline.start();});
  const auto entered = operation->wait_for_entry(5s);
  if (!entered) {
    operation->release();
    start_thread.join();
  }
  ASSERT_TRUE(entered) <<
    (result.has_value() ? result->detail : "controller start did not return");
  std::thread quiesce_thread([&]() {
      quiesce->complete(pipeline_config.transaction_plane->quiesce(
        2U, std::chrono::steady_clock::now() + 1s));
    });
  ASSERT_TRUE(quiesce->wait_for_entry());
  EXPECT_EQ(quiesce->generation(), 2U);
  EXPECT_FALSE(quiesce->wait_for_completion(100ms));
  operation->release();
  start_thread.join();
  quiesce_thread.join();

  ASSERT_TRUE(quiesce->result().has_value());
  EXPECT_TRUE(*quiesce->result());
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ok);
  EXPECT_EQ(result->state, MotionConditioningState::Failed);
  EXPECT_EQ(result->failure, MotionConditioningFailure::SafetyFault);
  EXPECT_EQ(producer->start_count, 1U);
  const auto stopped = pipeline.stop();
  EXPECT_TRUE(stopped.zero_proven);
  const auto snapshot = authority->snapshot();
  EXPECT_EQ(snapshot.state, GateState::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_published);
}

TEST_F(
  MotionConditioningPipelineTest,
  TransactionQuiesceAfterOpenReturnBlocksFinalRunningCommit)
{
  auto finish = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::Open);
  auto quiesce = std::make_shared<TransactionQuiesceBarrier>();
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    1U,
    RuntimeTransactionPlane::BeforeCommit{},
    RuntimeTransactionPlane::BeforeOperation{},
    [quiesce](const std::uint64_t generation) {
      (*quiesce)(generation);
    },
    [finish](const RuntimeTransactionSideEffect side_effect) {
      (*finish)(side_effect);
    });
  pipeline_config.transaction_generation_provider = []() {return 1U;};
  pipeline_config.prepare_open_deadline = 4s;
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());

  const auto renew_before_start = authority->renew_count();
  std::optional<MotionConditioningResult> result;
  std::thread start_thread([&]() {result = pipeline.start();});
  const auto entered = finish->wait_for_entry(5s);
  if (!entered) {
    finish->release();
    start_thread.join();
  }
  ASSERT_TRUE(entered);
  EXPECT_EQ(producer->start_count, 1U);

  std::thread quiesce_thread([&]() {
      quiesce->complete(pipeline_config.transaction_plane->quiesce(
        2U, std::chrono::steady_clock::now() + 1s));
    });
  ASSERT_TRUE(quiesce->wait_for_entry());
  EXPECT_FALSE(quiesce->wait_for_completion(100ms));
  const auto renew_at_barrier = authority->renew_count();

  finish->release();
  start_thread.join();
  quiesce_thread.join();

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ok);
  EXPECT_EQ(result->state, MotionConditioningState::Failed);
  EXPECT_EQ(result->failure, MotionConditioningFailure::SafetyFault);
  EXPECT_GE(producer->stop_count, 1U);
  ASSERT_TRUE(quiesce->result().has_value());
  EXPECT_TRUE(*quiesce->result());
  EXPECT_EQ(authority->renew_count(), renew_at_barrier);
  EXPECT_GE(renew_at_barrier, renew_before_start + 2U);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  const auto snapshot = authority->snapshot();
  EXPECT_EQ(snapshot.state, GateState::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_published);
}

TEST_F(MotionConditioningPipelineTest, StopAtActivationBarrierRejectsLateProducerStart)
{
  graph->enable_activation_barrier();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  authority->block_inhibit();

  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = pipeline.start();});
  graph->wait_for_activation_entry();

  std::optional<MotionConditioningResult> stopped;
  std::atomic<bool> stop_completed{false};
  std::thread stop_thread([&]() {
      stopped = pipeline.stop();
      stop_completed.store(true);
    });
  const auto inhibit_seen = authority->wait_for_inhibit();
  EXPECT_TRUE(inhibit_seen);
  EXPECT_FALSE(stop_completed.load());
  authority->release_blocked_inhibit();
  graph->release_activation();
  start_thread.join();
  stop_thread.join();

  ASSERT_TRUE(start_result.has_value());
  ASSERT_TRUE(stopped.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_TRUE(stopped->zero_proven);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Stopped);
  EXPECT_EQ(pipeline.last_result().state, stopped->state);
  EXPECT_EQ(pipeline.last_result().failure, stopped->failure);
  EXPECT_EQ(pipeline.last_result().zero_proven, stopped->zero_proven);
}

TEST_F(MotionConditioningPipelineTest, CancelBeforeOpenFenceNeverCallsOpen)
{
  auto pipeline_config = config();
  pipeline_config.stop_barrier = 1s;
  std::mutex open_mutex;
  std::condition_variable open_condition;
  bool open_fence_entered = false;
  bool release_open_fence = false;
  pipeline_config.before_open_callback = [&]() {
      std::unique_lock<std::mutex> lock(open_mutex);
      open_fence_entered = true;
      open_condition.notify_all();
      open_condition.wait(lock, [&]() {return release_open_fence;});
    };

  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  authority->block_inhibit();

  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = pipeline.start();});
  bool entered = false;
  {
    std::unique_lock<std::mutex> lock(open_mutex);
    entered = open_condition.wait_for(lock, 1s, [&]() {
          return open_fence_entered;
      });
  }
  if (!entered) {
    {
      std::lock_guard<std::mutex> lock(open_mutex);
      release_open_fence = true;
    }
    open_condition.notify_all();
    start_thread.join();
    FAIL() << "start did not reach the pre-OPEN cancellation fence";
  }

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  ASSERT_TRUE(authority->wait_for_inhibit());
  authority->release_blocked_inhibit();
  {
    std::lock_guard<std::mutex> lock(open_mutex);
    release_open_fence = true;
  }
  open_condition.notify_all();
  start_thread.join();
  stop_thread.join();

  ASSERT_TRUE(start_result.has_value());
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_TRUE(stop_result->zero_proven);
  const auto authority_calls = authority->calls();
  EXPECT_EQ(
    std::count(
      authority_calls.cbegin(), authority_calls.cend(),
      AuthorityOperationKind::Open), 0);
  EXPECT_EQ(producer->start_count, 0U);
}

TEST_F(MotionConditioningPipelineTest, StopDuringPrepareRetainsUniqueCleanupOwner)
{
  authority->block_prepare();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  std::optional<MotionConditioningResult> prepared;
  std::thread prepare_thread([&]() {prepared = pipeline.prepare();});
  ASSERT_TRUE(authority->wait_for_prepare());

  std::optional<MotionConditioningResult> stopped;
  std::thread stop_thread([&]() {stopped = pipeline.stop();});
  const bool inhibit_seen = authority->wait_for_inhibit();

  authority->release_blocked_prepare();
  prepare_thread.join();
  stop_thread.join();

  EXPECT_FALSE(inhibit_seen);
  ASSERT_TRUE(prepared.has_value());
  ASSERT_TRUE(stopped.has_value());
  EXPECT_FALSE(prepared->ok);
  EXPECT_TRUE(stopped->ok);
  EXPECT_EQ(stopped->state, MotionConditioningState::Stopped);
  EXPECT_TRUE(stopped->zero_proven);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 0U);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Stopped);
}

TEST_F(MotionConditioningPipelineTest, TeardownDrainsBlockedActivationBeforeCleanup)
{
  graph->enable_activation_barrier();
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);

  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = pipeline.start();});
  graph->wait_for_activation_entry();

  std::optional<MotionConditioningResult> stop_result;
  std::atomic<bool> stop_completed{false};
  std::thread stop_thread([&]() {
      stop_result = pipeline.stop();
      stop_completed.store(true);
    });
  std::this_thread::sleep_for(20ms);
  EXPECT_FALSE(stop_completed.load());

  graph->release_activation();
  start_thread.join();
  stop_thread.join();

  ASSERT_TRUE(start_result.has_value());
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_TRUE(stop_result->zero_proven);
  EXPECT_EQ(graph->unload_count(), 2U);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 3);
}

TEST_F(MotionConditioningPipelineTest, TeardownDrainsProducerStartBeforeCleanup)
{
  producer->block_start = true;
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);

  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = pipeline.start();});
  if (!producer->wait_for_start()) {
    producer->release_blocked_start();
    start_thread.join();
    if (start_result.has_value()) {
      FAIL() << "conditioning start did not reach producer barrier: " <<
        start_result->detail;
    }
    FAIL() << "conditioning start did not reach producer barrier";
  }

  std::optional<MotionConditioningResult> stop_result;
  std::atomic<bool> stop_completed{false};
  std::thread stop_thread([&]() {
      stop_result = pipeline.stop();
      stop_completed.store(true);
    });
  std::this_thread::sleep_for(20ms);
  EXPECT_FALSE(stop_completed.load());

  producer->release_blocked_start();
  start_thread.join();
  stop_thread.join();

  ASSERT_TRUE(start_result.has_value());
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_TRUE(stop_result->zero_proven);
  EXPECT_EQ(graph->unload_count(), 2U);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 3);
}

TEST_F(MotionConditioningPipelineTest, SecondStartCannotClaimTeardownOrOpen)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  authority->block_open();

  std::optional<MotionConditioningResult> first_result;
  std::thread first_thread([&]() {first_result = pipeline.start();});
  const bool open_entered = authority->wait_for_open();
  if (!open_entered) {
    authority->release_blocked_open();
    first_thread.join();
    FAIL() << "first start did not reach MotionGate OPEN barrier";
  }

  std::optional<MotionConditioningResult> second_result;
  std::thread second_thread([&]() {second_result = pipeline.start();});
  const auto second_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!second_result.has_value() &&
    std::chrono::steady_clock::now() < second_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  const bool second_returned = second_result.has_value();
  const auto calls_before_release = authority->calls();
  const auto opens_before_release = static_cast<std::size_t>(std::count(
      calls_before_release.cbegin(), calls_before_release.cend(),
      AuthorityOperationKind::Open));

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  std::this_thread::sleep_for(20ms);
  const bool stop_waited_for_first = !stop_result.has_value();

  authority->release_blocked_open();
  first_thread.join();
  second_thread.join();
  stop_thread.join();

  ASSERT_TRUE(second_returned);
  ASSERT_TRUE(second_result.has_value());
  EXPECT_FALSE(second_result->ok);
  EXPECT_NE(second_result->detail.find("another start"), std::string::npos);
  EXPECT_EQ(opens_before_release, 1U);
  EXPECT_TRUE(stop_waited_for_first);
  ASSERT_TRUE(first_result.has_value());
  ASSERT_TRUE(stop_result.has_value());
  EXPECT_FALSE(first_result->ok);
  EXPECT_TRUE(stop_result->zero_proven);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Open), 1);
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 2U);
  EXPECT_EQ(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, ProducerFalseFailsClosedAndCleansUp)
{
  producer->allow_start = false;
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_NE(result.zero_proven_at, std::chrono::steady_clock::time_point{});
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, ProducerThrowFailsClosedAndCleansUp)
{
  producer->throw_on_start = true;
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_NE(result.zero_proven_at, std::chrono::steady_clock::time_point{});
  EXPECT_GE(producer->stop_count, 1U);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, CleanupResidualEscalatesCollisionFailure)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_unload_delay(500ms);
  const auto token = pipeline.correlation_token();

  const auto result = pipeline.fail(
    token,
    MotionConditioningFailure::ExecutionFailed,
    "collision execution failed while unload is blocked");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.detail.empty());
  EXPECT_NE(result.detail.find("collision execution failed"), std::string::npos);
  EXPECT_NE(result.detail.find("cleanup phase="), std::string::npos);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_NE(result.zero_proven_at, std::chrono::steady_clock::time_point{});
  EXPECT_FALSE(graph->unload_requests().empty());
  EXPECT_FALSE(pipeline.prepare().ok);
}

TEST_F(MotionConditioningPipelineTest, CleanupResidualEscalatesSourceLoss)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_unload_delay(500ms);

  const auto result = pipeline.fail(
    pipeline.correlation_token(),
    MotionConditioningFailure::DependencyUnavailable,
    "source loss while conditioning was active");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_NE(result.detail.find("source loss"), std::string::npos);
  EXPECT_NE(result.detail.find("cleanup phase="), std::string::npos);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, CleanupResidualEscalatesDeadlineFailure)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_unload_delay(500ms);

  const auto result = pipeline.fail(
    pipeline.correlation_token(),
    MotionConditioningFailure::Timeout,
    "step deadline elapsed while conditioning was active");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_NE(result.detail.find("step deadline"), std::string::npos);
  EXPECT_NE(result.detail.find("cleanup phase="), std::string::npos);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, BusinessFailureAndCachedTerminalShareZeroProof)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto token = pipeline.correlation_token();

  const auto failed = pipeline.fail(
    token,
    MotionConditioningFailure::ExecutionFailed,
    "scripted execution failure with proven zero");

  EXPECT_FALSE(failed.ok);
  EXPECT_EQ(failed.failure, MotionConditioningFailure::ExecutionFailed);
  EXPECT_TRUE(failed.zero_proven);
  ASSERT_NE(failed.zero_proven_at, std::chrono::steady_clock::time_point{});
  EXPECT_EQ(pipeline.last_result().zero_proven_at, failed.zero_proven_at);

  const auto cached = pipeline.fail(
    token,
    MotionConditioningFailure::DependencyUnavailable,
    "late failure must not replace the terminal result");
  EXPECT_EQ(cached.failure, failed.failure);
  EXPECT_EQ(cached.zero_proven, failed.zero_proven);
  EXPECT_EQ(cached.zero_proven_at, failed.zero_proven_at);
}

TEST_F(MotionConditioningPipelineTest, ZeroProofFailureEscalatesExecutionFailureToSafetyFault)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  authority->set_inhibit_zero_proof(false);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto token = pipeline.correlation_token();

  const auto result = pipeline.fail(
    token,
    MotionConditioningFailure::ExecutionFailed,
    "collision execution failed while zero proof is unavailable");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_FALSE(pipeline.prepare().ok);
}

TEST_F(MotionConditioningPipelineTest, ProducerStopFailureMakesStopSafetyFault)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  producer->throw_on_stop = true;

  const auto result = pipeline.stop();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_FALSE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, RenewThrowFailsClosedAndStopsActivation)
{
  authority->set_throw_on_renew(true);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, OpenThrowFailsClosedAndCleansUp)
{
  authority->set_throw_on_open(true);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  ASSERT_TRUE(pipeline.prepare().ok);
  MotionConditioningResult result;
  EXPECT_NO_THROW(result = pipeline.start());

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, StartAfterPrepareOpenDeadlineFailsClosed)
{
  auto pipeline_config = config();
  pipeline_config.prepare_open_deadline = 500ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(550ms);

  const auto calls_before = authority->calls();
  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.state, MotionConditioningState::Failed);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_EQ(graph->loaded_count(), 0U);
  const auto calls_after = authority->calls();
  EXPECT_EQ(
    std::count(
      calls_after.cbegin(), calls_after.cend(), AuthorityOperationKind::Inhibit),
    std::count(
      calls_before.cbegin(), calls_before.cend(), AuthorityOperationKind::Inhibit) + 1);
}

TEST_F(MotionConditioningPipelineTest, LateLoadResponseIsReconciledAndBlocksNextPrepare)
{
  graph->set_load_delay(250ms);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 20ms;
  pipeline_config.writer_graph_timeout = 100ms;
  pipeline_config.prepare_open_deadline = 100ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto first = pipeline.prepare();
  EXPECT_FALSE(first.ok);
  EXPECT_EQ(first.failure, MotionConditioningFailure::SafetyFault);
  const auto calls_after_first = authority->calls();
  const auto second = pipeline.prepare();
  const auto calls_after_second = authority->calls();
  EXPECT_FALSE(second.ok);
  EXPECT_EQ(second.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_EQ(
    std::count(
      calls_after_second.cbegin(), calls_after_second.cend(),
      AuthorityOperationKind::Prepare),
    std::count(
      calls_after_first.cbegin(), calls_after_first.cend(),
      AuthorityOperationKind::Prepare));
  EXPECT_NE(first.detail.find("phase=pending_load"), std::string::npos);
  EXPECT_NE(first.detail.find("fqn=/collision_monitor"), std::string::npos);
}

TEST_F(MotionConditioningPipelineTest, LateLoadIsConsumedBeforeComponentCleanupDeadline)
{
  graph->set_load_delay(250ms);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 600ms;
  pipeline_config.writer_graph_timeout = 50ms;
  pipeline_config.prepare_open_deadline = 100ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto result = pipeline.prepare();
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);

  std::this_thread::sleep_for(350ms);
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(graph->unload_count(), 1U);
}

TEST_F(MotionConditioningPipelineTest, FqnMismatchIsUnloadedBeforePrepareFails)
{
  graph->set_wrong_fqn(true);
  graph->remove_lifecycle_services(false, false);
  MotionConditioningPipeline pipeline(*client, authority, producer, config());

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_NE(result.detail.find("phase=load_fqn"), std::string::npos);
  EXPECT_NE(result.detail.find("fqn=/wrong_collision_monitor"), std::string::npos);
  EXPECT_NE(result.detail.find("unique_id=1"), std::string::npos);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, AbsentLifecycleServicesDoNotHideActualUnload)
{
  graph->remove_lifecycle_services(false, false);
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 80ms;
  pipeline_config.writer_graph_timeout = 120ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto result = pipeline.prepare();

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, LifecycleTimeoutRetainsIndependentUnloadBudget)
{
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 2s;
  pipeline_config.writer_graph_timeout = 1s;
  pipeline_config.prepare_open_deadline = 4s;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  graph->remove_lifecycle_services(false, false);

  const auto result = pipeline.stop();

  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.zero_proven);
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, EachUniqueIdGetsAnIndependentUnloadBudget)
{
  auto pipeline_config = config();
  pipeline_config.component_rpc_timeout = 50ms;
  pipeline_config.writer_graph_timeout = 150ms;
  pipeline_config.prepare_open_deadline = 2s;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);

  const auto prepared = pipeline.prepare();
  ASSERT_TRUE(prepared.ok) << prepared.detail;
  graph->set_unload_delay_for(1U, 500ms);

  const auto result = pipeline.stop();
  const auto unload_requests = graph->unload_requests();
  const auto unload_completed = graph->unload_completed();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure, MotionConditioningFailure::SafetyFault);
  EXPECT_NE(
    std::find(unload_requests.cbegin(), unload_requests.cend(), 1U),
    unload_requests.cend());
  EXPECT_NE(
    std::find(unload_requests.cbegin(), unload_requests.cend(), 2U),
    unload_requests.cend());
  EXPECT_NE(
    std::find(unload_completed.cbegin(), unload_completed.cend(), 2U),
    unload_completed.cend());
  EXPECT_EQ(
    std::find(unload_completed.cbegin(), unload_completed.cend(), 1U),
    unload_completed.cend());
  EXPECT_NE(result.detail.find("phase=unload"), std::string::npos);
  EXPECT_NE(result.detail.find("fqn=/collision_monitor"), std::string::npos);
  EXPECT_NE(result.detail.find("unique_id=1"), std::string::npos);
}

TEST_F(MotionConditioningPipelineTest, SensorLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(false, true, true);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::DependencyUnavailable);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, OdomLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(true, false, true);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::DependencyUnavailable);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
}

TEST_F(MotionConditioningPipelineTest, ClockLivenessLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_health_sources(true, true, false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::DependencyUnavailable);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
}

TEST_F(MotionConditioningPipelineTest, FrozenClockProgressCannotStartProducer)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  graph->set_clock_frozen(true);
  std::this_thread::sleep_for(250ms);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, FirstFrozenClockSampleCannotStartProducer)
{
  auto pipeline_config = config(false);
  graph->set_clock_frozen(true);
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  graph->set_health_sources(true, true, true);

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);

  const auto result = pipeline.start();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(producer->start_count, 0U);
  EXPECT_TRUE(result.zero_proven);
}

TEST_F(MotionConditioningPipelineTest, InactiveControllerFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  graph->set_controller_active(false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::SafetyFault);
  EXPECT_TRUE(pipeline.last_result().zero_proven);
  EXPECT_GE(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, StopAndFailShareOneTeardownOwner)
{
  auto pipeline_config = config();
  pipeline_config.renew_period = 1s;
  MotionConditioningPipeline pipeline(*client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto token = pipeline.correlation_token();
  authority->block_inhibit();

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  const bool inhibit_entered = authority->wait_for_inhibit();
  if (!inhibit_entered) {
    authority->release_blocked_inhibit();
  }
  ASSERT_TRUE(inhibit_entered);

  std::optional<MotionConditioningResult> fail_result;
  std::thread fail_thread([&]() {
      fail_result = pipeline.fail(
        token,
        MotionConditioningFailure::SafetyFault,
        "dependency failed during STOP");
    });
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_inhibit();
  stop_thread.join();
  fail_thread.join();

  ASSERT_TRUE(stop_result.has_value());
  ASSERT_TRUE(fail_result.has_value());
  EXPECT_EQ(stop_result->state, fail_result->state);
  EXPECT_EQ(stop_result->failure, fail_result->failure);
  EXPECT_EQ(stop_result->zero_proven, fail_result->zero_proven);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, TerminalRecordIgnoresLateFailureAndCollision)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto token = pipeline.correlation_token();

  const auto stopped = pipeline.stop();
  ASSERT_TRUE(stopped.ok);
  const auto terminal = pipeline.last_result();
  const auto fail_result = pipeline.fail(
    token,
    MotionConditioningFailure::SafetyFault,
    "late failure must not replace STOP");

  graph->publish_collision_stop();
  std::this_thread::sleep_for(50ms);

  const auto calls = authority->calls();
  EXPECT_TRUE(fail_result.ok);
  EXPECT_EQ(pipeline.last_result().state, terminal.state);
  EXPECT_EQ(pipeline.last_result().failure, terminal.failure);
  EXPECT_EQ(pipeline.last_result().collision_stop, terminal.collision_stop);
  EXPECT_EQ(pipeline.last_result().zero_proven, terminal.zero_proven);
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 3);
}

TEST_F(MotionConditioningPipelineTest, LateOldCollisionCannotTerminateNextGeneration)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto old_collision_publisher = graph->collision_publisher();
  ASSERT_TRUE(old_collision_publisher);

  const auto stopped = pipeline.stop();
  ASSERT_TRUE(stopped.ok);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  const auto running = pipeline.start();
  ASSERT_TRUE(running.ok);

  CollisionState late_stop;
  late_stop.action_type = CollisionState::STOP;
  late_stop.polygon_name = "stop_zone";
  old_collision_publisher->publish(late_stop);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() == MotionConditioningState::Running &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(pipeline.state(), MotionConditioningState::Running);
  EXPECT_EQ(pipeline.last_result().state, running.state);
  EXPECT_EQ(pipeline.last_result().failure, running.failure);
  EXPECT_FALSE(pipeline.last_result().collision_stop);
}

TEST_F(MotionConditioningPipelineTest, LateOldFailureTokenCannotTouchNextGeneration)
{
  auto pipeline_config = config();
  pipeline_config.renew_period = 1s;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto old_token = pipeline.correlation_token();
  const auto stopped = pipeline.stop();
  ASSERT_TRUE(stopped.ok);

  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  const auto running = pipeline.start();
  ASSERT_TRUE(running.ok);
  const auto calls_before = authority->calls();
  const auto stops_before = producer->stop_count;
  const auto late = pipeline.fail(
    old_token,
    MotionConditioningFailure::SafetyFault,
    "late failure from the previous generation");

  EXPECT_TRUE(late.ok);
  EXPECT_EQ(late.state, MotionConditioningState::Running);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Running);
  EXPECT_EQ(pipeline.last_result().state, running.state);
  EXPECT_EQ(pipeline.last_result().failure, running.failure);
  EXPECT_EQ(producer->stop_count, stops_before);
  EXPECT_EQ(authority->calls(), calls_before);
  EXPECT_EQ(graph->loaded_count(), 2U);
}

TEST_F(
  MotionConditioningPipelineTest,
  LateTokenCannotClaimAfterStopAndNextGenerationBarrier)
{
  std::mutex barrier_mutex;
  std::condition_variable barrier_cv;
  bool barrier_entered = false;
  bool barrier_released = false;
  bool observed_old_token = false;
  std::optional<MotionConditioningCorrelationToken> old_token;
  auto pipeline_config = config();
  pipeline_config.renew_period = 1s;
  pipeline_config.before_token_claim =
    [&](const MotionConditioningCorrelationToken & observed) {
      std::unique_lock<std::mutex> lock(barrier_mutex);
      observed_old_token = old_token.has_value() &&
        observed.generation == old_token->generation &&
        observed.lease_id == old_token->lease_id &&
        observed.request_id == old_token->request_id;
      barrier_entered = true;
      barrier_cv.notify_all();
      barrier_cv.wait(lock, [&]() {return barrier_released;});
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  old_token = pipeline.correlation_token();

  std::optional<MotionConditioningResult> late_result;
  std::thread late_failure([&]() {
      late_result = pipeline.fail(
        *old_token,
        MotionConditioningFailure::SafetyFault,
        "late failure crossed the generation barrier");
    });
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    ASSERT_TRUE(barrier_cv.wait_for(
      lock, 1s, [&]() {return barrier_entered;}));
  }
  ASSERT_TRUE(observed_old_token);

  const auto stopped = pipeline.stop();
  ASSERT_TRUE(stopped.ok);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  const auto running = pipeline.start();
  ASSERT_TRUE(running.ok);

  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    barrier_released = true;
  }
  barrier_cv.notify_all();
  late_failure.join();

  ASSERT_TRUE(late_result.has_value());
  EXPECT_TRUE(late_result->ok);
  EXPECT_EQ(late_result->state, MotionConditioningState::Running);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Running);
  EXPECT_EQ(pipeline.last_result().state, running.state);
  EXPECT_EQ(pipeline.last_result().failure, running.failure);
  EXPECT_EQ(graph->loaded_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, StopWaitsForRenewFailureTeardownOwner)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  authority->block_inhibit();
  authority->set_throw_on_renew(true);
  const bool inhibit_entered = authority->wait_for_inhibit();
  if (!inhibit_entered) {
    authority->release_blocked_inhibit();
  }
  ASSERT_TRUE(inhibit_entered);

  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = pipeline.stop();});
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_inhibit();
  stop_thread.join();

  ASSERT_TRUE(stop_result.has_value());
  EXPECT_EQ(stop_result->state, MotionConditioningState::Failed);
  EXPECT_EQ(stop_result->failure, MotionConditioningFailure::SafetyFault);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, ExternalFailureWaitsForActiveRenew)
{
  auto pipeline_config = config();
  pipeline_config.renew_period = 20ms;
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  const auto token = pipeline.correlation_token();

  authority->block_renew();
  ASSERT_TRUE(authority->wait_for_renew());

  std::optional<MotionConditioningResult> failure_result;
  std::thread failure_thread([&]() {
      failure_result = pipeline.fail(
        token,
        MotionConditioningFailure::SafetyFault,
        "external dependency failure");
    });
  std::this_thread::sleep_for(20ms);
  authority->release_blocked_renew();
  failure_thread.join();

  ASSERT_TRUE(failure_result.has_value());
  EXPECT_FALSE(failure_result->ok);
  EXPECT_EQ(failure_result->state, MotionConditioningState::Failed);
  EXPECT_EQ(failure_result->failure, MotionConditioningFailure::SafetyFault);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(),
      AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, DestructorReusesTeardownTerminalResult)
{
  std::size_t unload_count = 0U;
  std::size_t inhibit_count = 0U;
  std::size_t producer_stop_count = 0U;
  {
    auto pipeline_config = config();
    pipeline_config.renew_period = 1s;
    MotionConditioningPipeline pipeline(
      *client, authority, producer, pipeline_config);
    ASSERT_TRUE(pipeline.prepare().ok);
    std::this_thread::sleep_for(50ms);
    const auto running = pipeline.start();
    ASSERT_TRUE(running.ok);
    const auto stopped = pipeline.stop();
    ASSERT_TRUE(stopped.ok);
    producer_stop_count = producer->stop_count;
    unload_count = graph->unload_count();
    const auto calls = authority->calls();
    inhibit_count = static_cast<std::size_t>(std::count(
        calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit));
  }

  EXPECT_EQ(graph->unload_count(), unload_count);
  const auto calls = authority->calls();
  EXPECT_EQ(
    static_cast<std::size_t>(std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit)),
    inhibit_count);
  EXPECT_EQ(producer->stop_count, producer_stop_count);
  EXPECT_EQ(producer_stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, DestructorWaitsForActiveTeardownOwner)
{
  auto pipeline = std::make_unique<MotionConditioningPipeline>(
    *client, authority, producer, config());
  ASSERT_TRUE(pipeline->prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline->start().ok);
  authority->block_inhibit();

  auto * raw_pipeline = pipeline.get();
  std::optional<MotionConditioningResult> stop_result;
  std::thread stop_thread([&]() {stop_result = raw_pipeline->stop();});
  const bool inhibit_entered = authority->wait_for_inhibit();
  if (!inhibit_entered) {
    authority->release_blocked_inhibit();
    stop_thread.join();
    FAIL() << "teardown did not reach the inhibit barrier";
  }

  auto owned_pipeline = std::move(pipeline);
  std::atomic<bool> destructor_finished{false};
  std::thread destructor_thread(
    [owned = std::move(owned_pipeline), &destructor_finished]() mutable {
      owned.reset();
      destructor_finished.store(true);
    });
  std::this_thread::sleep_for(20ms);
  EXPECT_FALSE(destructor_finished.load());

  authority->release_blocked_inhibit();
  stop_thread.join();
  destructor_thread.join();

  ASSERT_TRUE(stop_result.has_value());
  EXPECT_TRUE(stop_result->zero_proven);
  EXPECT_TRUE(destructor_finished.load());
  EXPECT_EQ(producer->stop_count, 1U);
  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Inhibit), 3);
  EXPECT_EQ(graph->unload_count(), 2U);
}

TEST_F(MotionConditioningPipelineTest, DestructorDrainsActiveStartBeyondStopBarrier)
{
  producer->block_start = true;
  auto pipeline = std::make_unique<MotionConditioningPipeline>(
    *client, authority, producer, config());
  ASSERT_TRUE(pipeline->prepare().ok);
  std::this_thread::sleep_for(50ms);

  auto * raw_pipeline = pipeline.get();
  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = raw_pipeline->start();});
  if (!producer->wait_for_start()) {
    producer->release_blocked_start();
    start_thread.join();
    FAIL() << "conditioning start did not reach producer barrier";
  }

  auto owned_pipeline = std::move(pipeline);
  std::atomic<bool> destructor_finished{false};
  std::thread destructor_thread(
    [owned = std::move(owned_pipeline), &destructor_finished]() mutable {
      owned.reset();
      destructor_finished.store(true);
    });
  std::this_thread::sleep_for(250ms);
  EXPECT_FALSE(destructor_finished.load());

  producer->release_blocked_start();
  start_thread.join();
  destructor_thread.join();

  ASSERT_TRUE(start_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_TRUE(destructor_finished.load());
  EXPECT_EQ(graph->loaded_count(), 0U);
}

TEST_F(MotionConditioningPipelineTest, StopTimeoutContinuationOwnsFinalCleanup)
{
  producer->block_start = true;
  auto health_ready = std::make_shared<CallbackCounter>();
  auto pipeline_config = config();
  pipeline_config.writer_graph_timeout = 1s;
  pipeline_config.prepare_open_deadline = 5s;
  pipeline_config.dependency_liveness_timeout = 2s;
  pipeline_config.health_rpc_timeout = 500ms;
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  auto pipeline = std::make_unique<MotionConditioningPipeline>(
    *client, authority, producer, pipeline_config);
  const auto prepared = pipeline->prepare();
  ASSERT_TRUE(prepared.ok);
  ASSERT_TRUE(graph->wait_for_loaded_count(2U));
  ASSERT_TRUE(wait_for_candidate_writer(prepared.candidate_topic));
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());

  auto * raw_pipeline = pipeline.get();
  std::optional<MotionConditioningResult> start_result;
  std::thread start_thread([&]() {start_result = raw_pipeline->start();});
  if (!producer->wait_for_start(2s)) {
    producer->release_blocked_start();
    start_thread.join();
    FAIL() << "conditioning start did not reach producer barrier detail=" <<
      (start_result.has_value() ? start_result->detail : "no result");
  }

  std::promise<MotionConditioningResult> stop_promise;
  auto stop_future = stop_promise.get_future();
  std::thread stop_thread([&]() {
      stop_promise.set_value(raw_pipeline->stop());
    });
  ASSERT_EQ(
    stop_future.wait_for(2s),
    std::future_status::ready);
  const auto stop_result = stop_future.get();
  EXPECT_FALSE(stop_result.ok);
  EXPECT_EQ(stop_result.failure, MotionConditioningFailure::SafetyFault);
  const auto terminal_before_release = raw_pipeline->last_result();

  producer->release_blocked_start();
  start_thread.join();
  stop_thread.join();
  ASSERT_TRUE(start_result.has_value());
  EXPECT_FALSE(start_result->ok);
  EXPECT_EQ(producer->start_count, 1U);
  ASSERT_TRUE(graph->wait_for_empty());
  ASSERT_TRUE(producer->wait_for_stop_count(1U));
  EXPECT_EQ(graph->loaded_count(), 0U);
  EXPECT_EQ(graph->unload_count(), 2U);
  EXPECT_EQ(producer->stop_count, 1U);
  const auto terminal_after_cleanup = raw_pipeline->last_result();
  EXPECT_EQ(terminal_after_cleanup.state, terminal_before_release.state);
  EXPECT_EQ(terminal_after_cleanup.failure, terminal_before_release.failure);
  EXPECT_EQ(terminal_after_cleanup.zero_proven, terminal_before_release.zero_proven);
  EXPECT_EQ(terminal_after_cleanup.detail, terminal_before_release.detail);

  const auto calls = authority->calls();
  EXPECT_EQ(
    std::count(
      calls.cbegin(), calls.cend(), AuthorityOperationKind::Prepare), 1);

  pipeline.reset();
  EXPECT_EQ(producer->stop_count, 1U);
}

TEST_F(MotionConditioningPipelineTest, DestructorDrainsHealthSubscriptionCallback)
{
  auto health_barrier = std::make_shared<CallbackBarrier>();
  auto callback_wait_barrier = std::make_shared<CallbackBarrier>();
  auto pipeline_config = config();
  pipeline_config.writer_graph_timeout = 1s;
  pipeline_config.prepare_open_deadline = 5s;
  pipeline_config.dependency_liveness_timeout = 2s;
  pipeline_config.health_rpc_timeout = 500ms;
  pipeline_config.before_health_callback = [health_barrier]() {
      (*health_barrier)();
    };
  pipeline_config.before_callback_wait = [callback_wait_barrier]() {
      (*callback_wait_barrier)();
    };

  auto pipeline = std::make_unique<MotionConditioningPipeline>(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline->prepare().ok);
  health_barrier->arm();
  graph->publish_health_once();
  ASSERT_TRUE(health_barrier->wait_for_entry());
  callback_wait_barrier->arm();

  auto owned_pipeline = std::move(pipeline);
  std::promise<void> destructor_promise;
  auto destructor_future = destructor_promise.get_future();
  std::thread destructor_thread(
    [owned = std::move(owned_pipeline), &destructor_promise]() mutable {
      owned.reset();
      destructor_promise.set_value();
    });

  ASSERT_TRUE(callback_wait_barrier->wait_for_entry());
  callback_wait_barrier->release();
  health_barrier->release();
  ASSERT_EQ(
    destructor_future.wait_for(2s),
    std::future_status::ready);
  destructor_thread.join();
}

TEST_F(MotionConditioningPipelineTest, DestructorDrainsQueuedRenewCallback)
{
  const auto child_mode = std::getenv(kCi64RenewChildEnvironment);
  if (child_mode != nullptr) {
    const std::string mode(child_mode);
    if (mode != "normal" && mode != "hang" && mode != "pre-ready-hang" &&
      mode != "pre-ready-stop")
    {
      ADD_FAILURE() << "unknown CI-64-03 child mode: " << mode;
      return;
    }

    const bool pre_ready = mode == "pre-ready-hang" || mode == "pre-ready-stop";
    const bool hang_child = mode == "hang";
    auto renew_barrier = std::make_shared<CallbackBarrier>();
    auto callback_wait_barrier = std::make_shared<CallbackBarrier>();
    auto health_ready = std::make_shared<CallbackCounter>();
    auto pipeline_config = config();
    pipeline_config.writer_graph_timeout = 1s;
    pipeline_config.prepare_open_deadline = 2s;
    pipeline_config.after_health_callback = [health_ready]() {
        (*health_ready)();
      };
    pipeline_config.before_renew_callback = [renew_barrier]() {
        (*renew_barrier)();
      };
    pipeline_config.before_callback_wait = [callback_wait_barrier]() {
        (*callback_wait_barrier)();
      };

    auto pipeline = std::make_unique<MotionConditioningPipeline>(
      *client, authority, producer, pipeline_config);
    const auto prepared = pipeline->prepare();
    ASSERT_TRUE(prepared.ok);
    ASSERT_TRUE(graph->wait_for_loaded_count(2U));
    ASSERT_TRUE(wait_for_candidate_writer(prepared.candidate_topic));
    health_ready->expect(4U);
    graph->publish_health_once();
    ASSERT_TRUE(health_ready->wait_for_target());
    const auto started = pipeline->start();
    ASSERT_TRUE(started.ok) << started.detail << " calls=" << authority->calls().size()
                            << " loaded=" << graph->loaded_count();

    renew_barrier->arm();
    const bool renew_entered = renew_barrier->wait_for_entry(5s);
    if (!renew_entered) {
      renew_barrier->release();
      ADD_FAILURE() << "renew callback did not enter child barrier";
      return;
    }
    callback_wait_barrier->arm();

    auto owned_pipeline = std::move(pipeline);
    std::promise<void> destructor_promise;
    auto destructor_future = destructor_promise.get_future();
    std::thread destructor_thread(
      [owned = std::move(owned_pipeline), &destructor_promise]() mutable {
        owned.reset();
        destructor_promise.set_value();
      });

    if (pre_ready) {
      if (mode == "pre-ready-stop") {
        raise(SIGSTOP);
      }
      pause();
      return;
    }

    const bool callback_wait_entered = callback_wait_barrier->wait_for_entry(5s);
    if (!callback_wait_entered || !write_renew_ready_token()) {
      callback_wait_barrier->release();
      renew_barrier->release();
      std::_Exit(callback_wait_entered ? 4 : 3);
    }
    if (hang_child) {
      pause();
      return;
    }

    callback_wait_barrier->release();
    renew_barrier->release();
    const bool destructor_ready =
      destructor_future.wait_for(2s) ==
      std::future_status::ready;
    if (!destructor_ready) {
      std::cerr << "CI-64-03 child destructor did not complete" << std::endl;
      std::_Exit(2);
    }
    destructor_thread.join();
    EXPECT_TRUE(renew_entered);
    EXPECT_TRUE(callback_wait_entered);
    EXPECT_TRUE(destructor_ready);
    return;
  }

  const auto normal_child = run_renew_drain_child("normal", 5s, 500ms);
  EXPECT_TRUE(normal_child.spawned);
  EXPECT_TRUE(normal_child.ready);
  EXPECT_FALSE(normal_child.setup_failure);
  EXPECT_TRUE(normal_child.reaped);
  EXPECT_FALSE(normal_child.timed_out);
  EXPECT_TRUE(normal_child.exited);
  EXPECT_EQ(normal_child.exit_code, 0);
  EXPECT_FALSE(normal_child.signaled);

  const auto hanging_child = run_renew_drain_child("hang", 5s, 500ms);
  EXPECT_TRUE(hanging_child.spawned);
  EXPECT_TRUE(hanging_child.ready);
  EXPECT_FALSE(hanging_child.setup_failure);
  EXPECT_TRUE(hanging_child.hang_deadline_started);
  EXPECT_TRUE(hanging_child.timed_out);
  EXPECT_FALSE(hanging_child.early_exit);
  EXPECT_TRUE(hanging_child.killed);
  EXPECT_TRUE(hanging_child.reaped);
  EXPECT_TRUE(hanging_child.signaled);
  EXPECT_EQ(hanging_child.signal_number, SIGKILL);

  const auto pre_ready_hang = run_renew_drain_child("pre-ready-hang", 5s, 500ms);
  EXPECT_TRUE(pre_ready_hang.spawned);
  EXPECT_FALSE(pre_ready_hang.ready);
  EXPECT_TRUE(pre_ready_hang.setup_failure);
  EXPECT_TRUE(pre_ready_hang.timed_out);
  EXPECT_TRUE(pre_ready_hang.killed);
  EXPECT_TRUE(pre_ready_hang.reaped);
  EXPECT_TRUE(pre_ready_hang.signaled);
  EXPECT_EQ(pre_ready_hang.signal_number, SIGKILL);

  const auto pre_ready_stop = run_renew_drain_child("pre-ready-stop", 5s, 500ms);
  EXPECT_TRUE(pre_ready_stop.spawned);
  EXPECT_FALSE(pre_ready_stop.ready);
  EXPECT_TRUE(pre_ready_stop.setup_failure);
  EXPECT_TRUE(pre_ready_stop.timed_out);
  EXPECT_TRUE(pre_ready_stop.killed);
  EXPECT_TRUE(pre_ready_stop.reaped);
  EXPECT_TRUE(pre_ready_stop.signaled);
  EXPECT_EQ(pre_ready_stop.signal_number, SIGKILL);
}

TEST_F(MotionConditioningPipelineTest, RenewAuthorityLossFailsClosed)
{
  MotionConditioningPipeline pipeline(*client, authority, producer, config());
  ASSERT_TRUE(pipeline.prepare().ok);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(pipeline.start().ok);
  authority->set_authority_live(false);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (
    pipeline.state() != MotionConditioningState::Failed &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_EQ(
    pipeline.last_result().failure,
    MotionConditioningFailure::SafetyFault);
  EXPECT_GE(producer->stop_count, 1U);
  const auto calls = authority->calls();
  EXPECT_NE(
    std::find(calls.cbegin(), calls.cend(), AuthorityOperationKind::Renew),
    calls.cend());
}

TEST_F(
  MotionConditioningPipelineTest,
  ShutdownCoordinatorFreezesRunningGenerationAtBarrierReturn)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto renew_count = std::make_shared<std::atomic<std::size_t>>(0U);
  auto transaction_plane = std::make_shared<RuntimeTransactionPlane>(1U);
  auto pipeline_config = config();
  pipeline_config.renew_period = 1s;
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  pipeline_config.before_renew_callback = [renew_count]() {
      renew_count->fetch_add(1U, std::memory_order_relaxed);
    };
  pipeline_config.transaction_plane = transaction_plane;
  pipeline_config.transaction_generation_provider = [transaction_plane]() {
      return transaction_plane->generation();
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  ASSERT_TRUE(pipeline.start().ok);
  ASSERT_EQ(pipeline.state(), MotionConditioningState::Running);

  std::vector<std::string> order;
  std::mutex order_mutex;
  std::optional<MotionConditioningResult> stop_result;
  RuntimeShutdownCoordinator coordinator(
    [&]() {
      {
        std::lock_guard<std::mutex> lock(order_mutex);
        order.push_back("close-generation");
      }
      transaction_plane->close_generation(2U);
      return true;
    },
    [&](const RuntimeShutdownCoordinator::TimePoint) {
      {
        std::lock_guard<std::mutex> lock(order_mutex);
        order.push_back("begin-running-shutdown");
      }
      stop_result = pipeline.stop();
    },
    [&](const RuntimeShutdownCoordinator::TimePoint deadline) {
      {
        std::lock_guard<std::mutex> lock(order_mutex);
        order.push_back("wait-joint-conditions");
      }
      const auto snapshot = authority->snapshot();
      return transaction_plane->wait_for_drain_until(deadline) &&
             pipeline.state() != MotionConditioningState::Running &&
             producer->stop_count >= 1U &&
             snapshot.state == GateState::Inhibited &&
             snapshot.motion_inhibited && snapshot.zero_selected &&
             snapshot.zero_published;
    },
    []() {},
    [](std::string) {},
    [](std::string) {});

  const auto outcome = coordinator.run(
    std::chrono::steady_clock::now() + 1s);
  ASSERT_TRUE(outcome.transaction_drained);
  ASSERT_FALSE(outcome.fail_closed);
  ASSERT_TRUE(stop_result.has_value());
  ASSERT_TRUE(stop_result->zero_proven);
  ASSERT_EQ(pipeline.state(), MotionConditioningState::Stopped);
  ASSERT_EQ(order.size(), 3U);
  EXPECT_EQ(order[0], "close-generation");
  EXPECT_EQ(order[1], "begin-running-shutdown");
  EXPECT_EQ(order[2], "wait-joint-conditions");

  const auto frozen_renew_count = renew_count->load(std::memory_order_relaxed);
  const auto frozen_stop_count = producer->stop_count;
  EXPECT_EQ(renew_count->load(std::memory_order_relaxed), frozen_renew_count);
  EXPECT_EQ(producer->stop_count, frozen_stop_count);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Stopped);
}

TEST_F(
  MotionConditioningPipelineTest,
  RenewReturnedBeforeCommitIsCleanedInsideTheInFlightLease)
{
  auto health_ready = std::make_shared<CallbackCounter>();
  auto renew_commit_barrier = std::make_shared<TransactionOperationBarrier>(
    RuntimeTransactionSideEffect::Renew);
  auto transaction_plane = std::make_shared<RuntimeTransactionPlane>(
    1U, RuntimeTransactionPlane::BeforeCommit{},
    RuntimeTransactionPlane::BeforeOperation{},
    RuntimeTransactionPlane::QuiesceObserver{},
    [renew_commit_barrier](const RuntimeTransactionSideEffect side_effect) {
      (*renew_commit_barrier)(side_effect);
    });
  auto pipeline_config = config();
  pipeline_config.after_health_callback = [health_ready]() {
      (*health_ready)();
    };
  pipeline_config.transaction_plane = transaction_plane;
  pipeline_config.transaction_generation_provider = [transaction_plane]() {
      return transaction_plane->generation();
    };
  MotionConditioningPipeline pipeline(
    *client, authority, producer, pipeline_config);
  ASSERT_TRUE(pipeline.prepare().ok);
  health_ready->expect(4U);
  graph->publish_health_once();
  ASSERT_TRUE(health_ready->wait_for_target());
  ASSERT_TRUE(pipeline.start().ok);
  ASSERT_EQ(pipeline.state(), MotionConditioningState::Running);
  ASSERT_TRUE(renew_commit_barrier->wait_for_entry());

  std::promise<void> drain_waiting_promise;
  auto drain_waiting_future = drain_waiting_promise.get_future();
  std::promise<void> drain_completed_promise;
  auto drain_completed_future = drain_completed_promise.get_future();
  std::optional<bool> drain_result;
  std::thread quiescer([&]() {
      transaction_plane->close_generation(2U);
      drain_waiting_promise.set_value();
      drain_result = transaction_plane->wait_for_drain_until(
        std::chrono::steady_clock::now() + 1s);
      drain_completed_promise.set_value();
    });
  ASSERT_EQ(
    drain_waiting_future.wait_for(1s),
    std::future_status::ready);
  EXPECT_EQ(
    drain_completed_future.wait_for(0ms),
    std::future_status::timeout);
  renew_commit_barrier->release();
  quiescer.join();

  ASSERT_TRUE(drain_result.has_value());
  EXPECT_TRUE(*drain_result);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
  EXPECT_GE(producer->stop_count, 1U);
  const auto snapshot = authority->snapshot();
  EXPECT_EQ(snapshot.state, GateState::Inhibited);
  EXPECT_TRUE(snapshot.motion_inhibited);
  EXPECT_TRUE(snapshot.zero_selected);
  EXPECT_TRUE(snapshot.zero_published);
  EXPECT_NE(
    pipeline.last_result().zero_proven_at,
    std::chrono::steady_clock::time_point{});

  const auto renew_calls = authority->renew_count();
  const auto stop_count = producer->stop_count;
  EXPECT_EQ(authority->renew_count(), renew_calls);
  EXPECT_EQ(producer->stop_count, stop_count);
  EXPECT_EQ(pipeline.state(), MotionConditioningState::Failed);
}

}  // namespace
}  // namespace voice_nav_mission
