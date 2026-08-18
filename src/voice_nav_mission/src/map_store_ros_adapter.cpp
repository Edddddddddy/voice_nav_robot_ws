// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "voice_nav_mission/map_store_ros_adapter.hpp"

#include <future>
#include <stdexcept>
#include <utility>

#include <nav2_map_server/map_io.hpp>

namespace voice_nav_mission
{
namespace
{

constexpr double kMapFreeThreshold = 0.25;
constexpr double kMapOccupiedThreshold = 0.65;

ChildResult unavailable(std::string detail)
{
  return ChildResult{ChildResultCode::DependencyUnavailable, std::move(detail)};
}

ChildResult failed(std::string detail)
{
  return ChildResult{ChildResultCode::Failed, std::move(detail)};
}

}  // namespace

RosMapStoreUpstream::RosMapStoreUpstream(
  rclcpp::Node & node,
  const std::chrono::milliseconds operation_timeout,
  std::string pose_graph_service)
: node_(node), operation_timeout_(operation_timeout)
{
  if (operation_timeout_.count() <= 0) {
    throw std::invalid_argument("Map upstream timeout must be positive");
  }
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  map_qos.reliable().transient_local();
  map_subscription_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
      if (!message) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      latest_map_ = *message;
    });
  pose_graph_client_ = node_.create_client<SerializePoseGraph>(
    std::move(pose_graph_service));
}

bool RosMapStoreUpstream::regular_nonempty_file(
  const std::filesystem::path & path)
{
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return false;
  }
  const auto size = std::filesystem::file_size(path, error);
  return !error && size > 0U;
}

ChildResult RosMapStoreUpstream::capture(
  const std::filesystem::path & staging_directory)
{
  std::optional<nav_msgs::msg::OccupancyGrid> map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map = latest_map_;
  }
  if (!map.has_value() || map->info.width == 0U || map->info.height == 0U ||
    map->data.size() != static_cast<std::size_t>(map->info.width) * map->info.height)
  {
    return unavailable("Nav2 occupancy map is not available");
  }
  if (!pose_graph_client_) {
    return unavailable("slam_toolbox SerializePoseGraph client is unavailable");
  }
  try {
    nav2_map_server::SaveParameters save_parameters;
    save_parameters.map_file_name = (staging_directory / "map").string();
    save_parameters.image_format = "pgm";
    save_parameters.free_thresh = kMapFreeThreshold;
    save_parameters.occupied_thresh = kMapOccupiedThreshold;
    save_parameters.mode = nav2_map_server::MapMode::Trinary;
    if (!nav2_map_server::saveMapToFile(*map, save_parameters)) {
      return failed("nav2_map_server::saveMapToFile failed");
    }
    if (!regular_nonempty_file(staging_directory / "map.yaml") ||
      !regular_nonempty_file(staging_directory / "map.pgm"))
    {
      return failed("Nav2 did not produce map.yaml and map.pgm");
    }

    if (!pose_graph_client_->wait_for_service(operation_timeout_)) {
      return unavailable(
        std::string{"slam_toolbox SerializePoseGraph service is unavailable: "} +
        pose_graph_client_->get_service_name());
    }
    auto request = std::make_shared<SerializePoseGraph::Request>();
    request->filename = (staging_directory / "map").string();
    auto response_future = pose_graph_client_->async_send_request(request);
    if (response_future.wait_for(operation_timeout_) != std::future_status::ready) {
      return unavailable(
        std::string{"slam_toolbox SerializePoseGraph timed out at "} +
        pose_graph_client_->get_service_name() + " after " +
        std::to_string(operation_timeout_.count()) + " ms");
    }
    const auto response = response_future.get();
    if (!response) {
      return failed(
        std::string{"slam_toolbox SerializePoseGraph returned no response at "} +
        pose_graph_client_->get_service_name());
    }
    if (response->result != SerializePoseGraph::Response::RESULT_SUCCESS) {
      return failed(
        std::string{"slam_toolbox SerializePoseGraph failed at "} +
        pose_graph_client_->get_service_name() + " result=" +
        std::to_string(static_cast<unsigned int>(response->result)));
    }
    if (!regular_nonempty_file(staging_directory / "map.posegraph") ||
      !regular_nonempty_file(staging_directory / "map.data"))
    {
      return failed("slam_toolbox did not produce map.posegraph and map.data");
    }
    return ChildResult{ChildResultCode::Succeeded, "Map upstream artifacts captured"};
  } catch (const std::exception & exception) {
    return unavailable(std::string{"Map upstream capture raised: "} + exception.what());
  } catch (...) {
    return unavailable("Map upstream capture raised an unknown exception");
  }
}

}  // namespace voice_nav_mission
