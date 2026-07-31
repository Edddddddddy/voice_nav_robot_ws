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

#include <rmw/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <rclcpp/message_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{

using Gid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;
using SteadyClock = std::chrono::steady_clock;

struct Edge
{
  std::string parent;
  std::string child;

  bool operator<(const Edge & other) const
  {
    return std::tie(parent, child) < std::tie(other.parent, other.child);
  }
};

struct Expectation
{
  std::string topic;
  Edge edge;
  std::string expected_node;
  std::size_t expected_owners;
};

struct Options
{
  std::vector<Expectation> expectations;
  bool reject_undeclared{false};
  bool show_help{false};
  double timeout_seconds{10.0};
  double stable_window_seconds{0.75};
};

enum class EvaluationState
{
  PENDING,
  SATISFIED,
  VIOLATION,
};

struct Evaluation
{
  EvaluationState state;
  std::string detail;
};

std::string usage()
{
  return
    "Usage:\n"
    "  tf_ownership_auditor"
    " --edge TOPIC PARENT CHILD EXPECTED_NODE_FQN [--edge ...]\n"
    "  tf_ownership_auditor"
    " --expect-conflict TOPIC PARENT CHILD EXPECTED_NODE_FQN\n"
    "Options:\n"
    "  TOPIC                     Must be /tf or /tf_static.\n"
    "  EXPECTED_NODE_FQN         Absolute ROS node name beginning with /.\n"
    "  --reject-undeclared       Reject every observed TF edge not listed.\n"
    "  --timeout SECONDS         Overall wall-clock timeout (default 10).\n"
    "  --stable-window SECONDS   Required continuous success window"
    " (default 0.75).\n";
}

double parse_positive_double(const std::string & text, const std::string & option)
{
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(option + " requires a finite positive number");
  }
  return value;
}

std::string require_value(
  const std::vector<std::string> & arguments,
  std::size_t index,
  const std::string & option)
{
  if (index >= arguments.size()) {
    throw std::invalid_argument(option + " is missing an argument");
  }
  return arguments[index];
}

Options parse_options(const std::vector<std::string> & arguments)
{
  Options options;
  bool conflict_mode = false;

  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto & argument = arguments[index];
    if (argument == "--help" || argument == "-h") {
      options.show_help = true;
      continue;
    }
    if (argument == "--edge" || argument == "--expect-conflict") {
      if (index + 4 >= arguments.size()) {
        throw std::invalid_argument(
                argument + " requires TOPIC PARENT CHILD EXPECTED_NODE_FQN");
      }
      const bool is_conflict = argument == "--expect-conflict";
      if (
        (is_conflict && !options.expectations.empty()) ||
        (!is_conflict && conflict_mode))
      {
        throw std::invalid_argument(
                "--expect-conflict cannot be combined with --edge or repeated");
      }
      conflict_mode = conflict_mode || is_conflict;
      Expectation expectation{
        require_value(arguments, index + 1, argument),
        Edge{
          require_value(arguments, index + 2, argument),
          require_value(arguments, index + 3, argument)},
        require_value(arguments, index + 4, argument),
        is_conflict ? 2U : 1U};
      index += 4;
      if (expectation.topic != "/tf" && expectation.topic != "/tf_static") {
        throw std::invalid_argument(
                argument + " TOPIC must be /tf or /tf_static");
      }
      if (
        expectation.edge.parent.empty() ||
        expectation.edge.child.empty() ||
        expectation.expected_node.empty())
      {
        throw std::invalid_argument(argument + " arguments must not be empty");
      }
      if (expectation.expected_node.front() != '/') {
        throw std::invalid_argument(
                argument + " EXPECTED_NODE_FQN must begin with /");
      }
      const auto duplicate = std::find_if(
        options.expectations.cbegin(),
        options.expectations.cend(),
        [&expectation](const Expectation & existing) {
          return existing.edge.parent == expectation.edge.parent &&
                 existing.edge.child == expectation.edge.child;
        });
      if (duplicate != options.expectations.cend()) {
        throw std::invalid_argument(
                "the same parent-child edge was declared more than once");
      }
      options.expectations.push_back(std::move(expectation));
      continue;
    }
    if (argument == "--reject-undeclared") {
      options.reject_undeclared = true;
      continue;
    }
    if (argument == "--timeout") {
      options.timeout_seconds = parse_positive_double(
        require_value(arguments, ++index, argument), argument);
      continue;
    }
    if (argument == "--stable-window") {
      options.stable_window_seconds = parse_positive_double(
        require_value(arguments, ++index, argument), argument);
      continue;
    }
    throw std::invalid_argument("unknown argument: " + argument);
  }

  if (!options.show_help && options.expectations.empty()) {
    throw std::invalid_argument("at least one --edge or --expect-conflict is required");
  }
  if (options.stable_window_seconds >= options.timeout_seconds) {
    throw std::invalid_argument("--stable-window must be shorter than --timeout");
  }
  return options;
}

Gid message_gid(const rclcpp::MessageInfo & message_info)
{
  Gid result{};
  const auto & raw_gid =
    message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, result.begin());
  return result;
}

bool is_zero_gid(const Gid & gid)
{
  return std::all_of(
    gid.cbegin(), gid.cend(),
    [](std::uint8_t value) {return value == 0U;});
}

std::string gid_string(const Gid & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

std::string topic_set_string(const std::set<std::string> & topics)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & topic : topics) {
    if (!first) {
      stream << ",";
    }
    first = false;
    stream << topic;
  }
  return stream.str();
}

std::string fully_qualified_node_name(const rclcpp::TopicEndpointInfo & endpoint)
{
  std::string node_namespace = endpoint.node_namespace();
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + endpoint.node_name();
  }
  if (node_namespace.front() != '/') {
    node_namespace.insert(node_namespace.begin(), '/');
  }
  if (node_namespace.back() == '/') {
    node_namespace.pop_back();
  }
  return node_namespace + "/" + endpoint.node_name();
}

bool node_matches(
  const std::string & expected_node,
  const rclcpp::TopicEndpointInfo & endpoint)
{
  return expected_node == fully_qualified_node_name(endpoint);
}

class TfOwnershipAuditor : public rclcpp::Node
{
public:
  explicit TfOwnershipAuditor(Options options)
  : Node("tf_ownership_auditor"), options_(std::move(options))
  {
    auto dynamic_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    static_qos.transient_local();

    dynamic_subscription_ =
      create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf",
      dynamic_qos,
      [this](
        const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
        const rclcpp::MessageInfo & message_info)
      {
        observe(*message, message_info, "/tf");
      });
    static_subscription_ =
      create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static",
      static_qos,
      [this](
        const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
        const rclcpp::MessageInfo & message_info)
      {
        observe(*message, message_info, "/tf_static");
      });
  }

  Evaluation evaluate() const
  {
    if (zero_gid_observed_) {
      return {
        EvaluationState::VIOLATION,
        "received a TF sample with an all-zero publisher GID"};
    }
    if (!invalid_frames_.empty()) {
      return {
        EvaluationState::VIOLATION,
        "observed an invalid TF frame pair: " + *invalid_frames_.cbegin()};
    }

    std::map<Edge, const Expectation *> declarations;
    for (const auto & expectation : options_.expectations) {
      declarations.emplace(expectation.edge, &expectation);
    }

    if (options_.reject_undeclared) {
      for (const auto & observation : owners_by_edge_) {
        if (declarations.count(observation.first) == 0U) {
          return {
            EvaluationState::VIOLATION,
            "observed undeclared edge " + observation.first.parent + " -> " +
            observation.first.child};
        }
      }
    }

    const auto endpoints = publisher_endpoints();
    for (const auto & expectation : options_.expectations) {
      const auto observation = owners_by_edge_.find(expectation.edge);
      if (observation == owners_by_edge_.cend()) {
        return {
          EvaluationState::PENDING,
          "no TF sample on " + expectation.topic + " for " +
          expectation.edge.parent + " -> " +
          expectation.edge.child};
      }
      const std::set<std::string> expected_topics{expectation.topic};
      for (const auto & owner : observation->second) {
        if (owner.second != expected_topics) {
          return {
            EvaluationState::VIOLATION,
            expectation.edge.parent + " -> " + expectation.edge.child +
            " GID " + gid_string(owner.first) + " was observed on {" +
            topic_set_string(owner.second) + "}; expected exactly {" +
            expectation.topic + "}"};
        }
      }
      if (observation->second.size() > expectation.expected_owners) {
        return {
          EvaluationState::VIOLATION,
          expectation.topic + " " + expectation.edge.parent + " -> " +
          expectation.edge.child +
          " has " + std::to_string(observation->second.size()) +
          " publisher GID(s); expected " +
          std::to_string(expectation.expected_owners)};
      }
      if (observation->second.size() < expectation.expected_owners) {
        return {
          EvaluationState::PENDING,
          expectation.topic + " " + expectation.edge.parent + " -> " +
          expectation.edge.child +
          " has " + std::to_string(observation->second.size()) +
          " publisher GID(s); waiting for " +
          std::to_string(expectation.expected_owners)};
      }

      for (const auto & owner : observation->second) {
        const auto topic_endpoints = endpoints.find(expectation.topic);
        if (topic_endpoints == endpoints.cend()) {
          return {
            EvaluationState::PENDING,
            "graph has no publisher endpoint information for " +
            expectation.topic};
        }
        const auto gid_endpoints = topic_endpoints->second.find(owner.first);
        if (gid_endpoints == topic_endpoints->second.cend()) {
          return {
            EvaluationState::PENDING,
            "observed GID " + gid_string(owner.first) +
            " is not present in the graph for " + expectation.topic};
        }
        const bool expected_endpoint_found = std::any_of(
          gid_endpoints->second.cbegin(),
          gid_endpoints->second.cend(),
          [&expectation](const rclcpp::TopicEndpointInfo & endpoint) {
            return node_matches(expectation.expected_node, endpoint);
          });
        if (!expected_endpoint_found) {
          return {
            EvaluationState::VIOLATION,
            "GID " + gid_string(owner.first) + " on " +
            expectation.topic + " does not map to expected node " +
            expectation.expected_node};
        }
      }
    }
    return {
      EvaluationState::SATISFIED,
      "all declared TF ownership expectations are satisfied"};
  }

  std::string observations() const
  {
    if (owners_by_edge_.empty()) {
      return "no TF edges observed";
    }
    std::map<std::string, EndpointsByGid> endpoints;
    std::string graph_error;
    try {
      endpoints = publisher_endpoints();
    } catch (const std::exception & error) {
      graph_error = error.what();
    }

    std::ostringstream stream;
    bool first_edge = true;
    for (const auto & edge_owners : owners_by_edge_) {
      if (!first_edge) {
        stream << "; ";
      }
      first_edge = false;
      stream << edge_owners.first.parent << " -> " <<
        edge_owners.first.child << " owners=" << edge_owners.second.size();
      for (const auto & owner : edge_owners.second) {
        for (const auto & topic : owner.second) {
          stream << " [gid=" << gid_string(owner.first) <<
            " topic=" << topic << " endpoints={";
          if (!graph_error.empty()) {
            stream << "<graph-query-error:" << graph_error << ">";
          } else {
            std::set<std::string> endpoint_names;
            const auto topic_endpoints = endpoints.find(topic);
            if (topic_endpoints != endpoints.cend()) {
              const auto gid_endpoints =
                topic_endpoints->second.find(owner.first);
              if (gid_endpoints != topic_endpoints->second.cend()) {
                for (const auto & endpoint : gid_endpoints->second) {
                  endpoint_names.insert(
                    fully_qualified_node_name(endpoint));
                }
              }
            }
            if (endpoint_names.empty()) {
              stream << "<unknown>";
            } else {
              bool first_endpoint = true;
              for (const auto & endpoint_name : endpoint_names) {
                if (!first_endpoint) {
                  stream << ",";
                }
                first_endpoint = false;
                stream << endpoint_name;
              }
            }
          }
          stream << "}]";
        }
      }
    }
    return stream.str();
  }

private:
  using TopicsByGid = std::map<Gid, std::set<std::string>>;
  using EndpointsByGid =
    std::map<Gid, std::vector<rclcpp::TopicEndpointInfo>>;

  void observe(
    const tf2_msgs::msg::TFMessage & message,
    const rclcpp::MessageInfo & message_info,
    const std::string & topic)
  {
    const Gid gid = message_gid(message_info);
    zero_gid_observed_ = zero_gid_observed_ || is_zero_gid(gid);
    for (const auto & transform : message.transforms) {
      const auto & parent = transform.header.frame_id;
      const auto & child = transform.child_frame_id;
      if (
        parent.empty() || child.empty() ||
        parent.front() == '/' || child.front() == '/')
      {
        invalid_frames_.insert(parent + " -> " + child);
        continue;
      }
      const Edge edge{
        parent,
        child};
      owners_by_edge_[edge][gid].insert(topic);
    }
  }

  std::map<std::string, EndpointsByGid> publisher_endpoints() const
  {
    std::map<std::string, EndpointsByGid> result;
    for (const std::string topic : {"/tf", "/tf_static"}) {
      for (const auto & endpoint : get_publishers_info_by_topic(topic)) {
        result[topic][endpoint.endpoint_gid()].push_back(endpoint);
      }
    }
    return result;
  }

  Options options_;
  bool zero_gid_observed_{false};
  std::set<std::string> invalid_frames_;
  std::map<Edge, TopicsByGid> owners_by_edge_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
    dynamic_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
    static_subscription_;
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    const Options options = parse_options(arguments);
    if (options.show_help) {
      std::cout << usage();
      rclcpp::shutdown();
      return 0;
    }

    auto auditor = std::make_shared<TfOwnershipAuditor>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(auditor);

    const auto timeout =
      std::chrono::duration<double>(options.timeout_seconds);
    const auto stable_window =
      std::chrono::duration<double>(options.stable_window_seconds);
    const auto deadline = SteadyClock::now() + timeout;
    std::optional<SteadyClock::time_point> satisfied_since;
    Evaluation last_evaluation{
      EvaluationState::PENDING,
      "no evaluation performed"};
    rclcpp::WallRate polling_rate(50.0);

    while (rclcpp::ok()) {
      executor.spin_some();
      const auto evaluation = auditor->evaluate();
      last_evaluation = evaluation;
      const auto now = SteadyClock::now();
      if (evaluation.state == EvaluationState::VIOLATION) {
        RCLCPP_ERROR(
          auditor->get_logger(),
          "TF ownership audit rejected: %s; observations: %s",
          evaluation.detail.c_str(),
          auditor->observations().c_str());
        executor.remove_node(auditor);
        rclcpp::shutdown();
        return 1;
      }
      if (evaluation.state == EvaluationState::SATISFIED) {
        if (!satisfied_since.has_value()) {
          satisfied_since = now;
        }
      } else {
        satisfied_since.reset();
      }
      if (now >= deadline) {
        break;
      }
      polling_rate.sleep();
    }

    const auto completed_at = SteadyClock::now();
    const bool full_window_observed = completed_at >= deadline;
    const bool final_state_stable =
      last_evaluation.state == EvaluationState::SATISFIED &&
      satisfied_since.has_value() &&
      completed_at - *satisfied_since >= stable_window;
    if (full_window_observed && final_state_stable) {
      RCLCPP_INFO(
        auditor->get_logger(),
        "TF ownership audit passed after full %.3f s observation window "
        "with a final %.3f s stable interval: %s",
        options.timeout_seconds,
        options.stable_window_seconds,
        auditor->observations().c_str());
      executor.remove_node(auditor);
      rclcpp::shutdown();
      return 0;
    }

    RCLCPP_ERROR(
      auditor->get_logger(),
      "TF ownership audit failed after observation window: %s; "
      "full_window_observed=%s; observations: %s",
      last_evaluation.detail.c_str(),
      full_window_observed ? "true" : "false",
      auditor->observations().c_str());
    executor.remove_node(auditor);
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & error) {
    std::cerr << "tf_ownership_auditor: " << error.what() << "\n" << usage();
    rclcpp::shutdown();
    return 2;
  }
}
