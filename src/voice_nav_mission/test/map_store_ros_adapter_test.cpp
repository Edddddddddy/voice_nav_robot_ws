// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>

#include "voice_nav_mission/map_store_ros_adapter.hpp"

namespace voice_nav_mission
{
namespace
{

using namespace std::chrono_literals;

TEST(RosMapStoreUpstreamTest, DefaultServiceCapturesPoseGraphAndData)
{
  rclcpp::init(0, nullptr);
  const auto cleanup = []() {rclcpp::shutdown();};

  const auto node = std::make_shared<rclcpp::Node>("map_store_adapter_test");
  const auto map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  std::atomic_bool map_received{false};
  const auto map_probe = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&map_received](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {
      map_received.store(true);
    });
  (void)map_probe;

  std::atomic_int service_calls{0};
  const auto pose_graph_service = node->create_service<slam_toolbox::srv::SerializePoseGraph>(
    "/slam_toolbox/serialize_map",
    [&service_calls](
      const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> request,
      std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> response) {
      ++service_calls;
      std::ofstream posegraph(request->filename + ".posegraph");
      posegraph << "posegraph\n";
      std::ofstream data(request->filename + ".data");
      data << "serialized\n";
      response->result = 0U;
    });
  (void)pose_graph_service;

  RosMapStoreUpstream adapter(*node, 2s);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05F;
  map.info.width = 1U;
  map.info.height = 1U;
  map.info.origin.orientation.w = 1.0;
  map.data = {0};
  for (int attempt = 0; attempt < 50 && !map_received.load(); ++attempt) {
    map_publisher->publish(map);
    std::this_thread::sleep_for(10ms);
  }

  if (!map_received.load()) {
    executor.cancel();
    spinner.join();
    cleanup();
    FAIL() << "map subscription did not receive the fixture";
  }

  const auto staging = std::filesystem::temp_directory_path() /
    ("voice_nav_map_store_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(staging);

  const ChildResult result = adapter.capture(staging);

  executor.cancel();
  spinner.join();
  cleanup();

  ASSERT_EQ(result.code, ChildResultCode::Succeeded) << result.detail;
  EXPECT_EQ(service_calls.load(), 1);
  EXPECT_GT(std::filesystem::file_size(staging / "map.yaml"), 0U);
  EXPECT_GT(std::filesystem::file_size(staging / "map.pgm"), 0U);
  EXPECT_GT(std::filesystem::file_size(staging / "map.posegraph"), 0U);
  EXPECT_GT(std::filesystem::file_size(staging / "map.data"), 0U);
  std::filesystem::remove_all(staging);
}

TEST(RosMapStoreUpstreamTest, FailedToWriteFileReportsEndpointAndResult)
{
  rclcpp::init(0, nullptr);
  const auto cleanup = []() {rclcpp::shutdown();};

  const auto node = std::make_shared<rclcpp::Node>("map_store_failure_test");
  const auto map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  std::atomic_bool map_received{false};
  const auto map_probe = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&map_received](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {
      map_received.store(true);
    });
  (void)map_probe;

  const auto pose_graph_service = node->create_service<slam_toolbox::srv::SerializePoseGraph>(
    "/slam_toolbox/serialize_map",
    [](const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request>,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> response) {
      response->result =
        slam_toolbox::srv::SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE;
    });
  (void)pose_graph_service;

  RosMapStoreUpstream adapter(*node, 200ms);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05F;
  map.info.width = 1U;
  map.info.height = 1U;
  map.info.origin.orientation.w = 1.0;
  map.data = {0};
  for (int attempt = 0; attempt < 50 && !map_received.load(); ++attempt) {
    map_publisher->publish(map);
    std::this_thread::sleep_for(10ms);
  }

  if (!map_received.load()) {
    executor.cancel();
    spinner.join();
    cleanup();
    FAIL() << "map subscription did not receive the fixture";
  }
  const auto staging = std::filesystem::temp_directory_path() /
    ("voice_nav_map_store_failure_test_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(staging);

  const ChildResult result = adapter.capture(staging);

  executor.cancel();
  spinner.join();
  cleanup();

  ASSERT_EQ(result.code, ChildResultCode::Failed);
  EXPECT_NE(result.detail.find("/slam_toolbox/serialize_map"), std::string::npos);
  EXPECT_NE(result.detail.find("255"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(staging / "map.posegraph"));
  EXPECT_FALSE(std::filesystem::exists(staging / "map.data"));
  std::filesystem::remove_all(staging);
}

}  // namespace
}  // namespace voice_nav_mission
