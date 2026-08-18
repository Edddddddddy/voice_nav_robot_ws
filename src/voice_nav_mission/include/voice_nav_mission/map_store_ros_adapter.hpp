// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_MISSION__MAP_STORE_ROS_ADAPTER_HPP_
#define VOICE_NAV_MISSION__MAP_STORE_ROS_ADAPTER_HPP_

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

// ROS-only upstream capture seam.  MapPackageWriter remains responsible for
// IDs, validation, hashes, and publication; this class only asks Nav2 and
// slam_toolbox to serialize their own artifacts into private staging paths.
class RosMapStoreUpstream final
{
public:
  explicit RosMapStoreUpstream(
    rclcpp::Node & node,
    std::chrono::milliseconds operation_timeout = std::chrono::milliseconds(5000),
    std::string pose_graph_service = "/slam_toolbox/serialize_map");

  [[nodiscard]] ChildResult capture(
    const std::filesystem::path & staging_directory);

private:
  using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;

  [[nodiscard]] static bool regular_nonempty_file(
    const std::filesystem::path & path);

  rclcpp::Node & node_;
  std::chrono::milliseconds operation_timeout_;
  std::mutex mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Client<SerializePoseGraph>::SharedPtr pose_graph_client_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MAP_STORE_ROS_ADAPTER_HPP_
