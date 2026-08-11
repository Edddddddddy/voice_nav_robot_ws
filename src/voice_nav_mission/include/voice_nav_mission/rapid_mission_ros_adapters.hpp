// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_MISSION__RAPID_MISSION_ROS_ADAPTERS_HPP_
#define VOICE_NAV_MISSION__RAPID_MISSION_ROS_ADAPTERS_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

// Rapid-only Adapter that forwards one validated Runtime step to the already
// proven Python Nav2/SLAM bridge. The public /mission endpoints remain owned
// by MissionRuntimeNode; this seam is package-private and intentionally makes
// no production MotionGate claim.
class RapidMissionDelegate final
{
public:
  RapidMissionDelegate(rclcpp::Node & node, std::string action_name);
  ~RapidMissionDelegate();

  RapidMissionDelegate(const RapidMissionDelegate &) = delete;
  RapidMissionDelegate & operator=(const RapidMissionDelegate &) = delete;

  [[nodiscard]] bool healthy() const;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    MissionChildPort::FeedbackCallback feedback,
    MissionChildPort::ResultCallback result);
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline);

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

class RapidNavigationPort final : public NavigationPort
{
public:
  explicit RapidNavigationPort(std::shared_ptr<RapidMissionDelegate> delegate);

  [[nodiscard]] bool healthy() const override;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  void tick(SteadyClockPort::TimePoint) override {}

private:
  std::shared_ptr<RapidMissionDelegate> delegate_;
};

class RapidRelativeMotionPort final : public RelativeMotionPort
{
public:
  explicit RapidRelativeMotionPort(
    std::shared_ptr<RapidMissionDelegate> delegate);

  [[nodiscard]] bool healthy() const override;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  void tick(SteadyClockPort::TimePoint) override {}

private:
  std::shared_ptr<RapidMissionDelegate> delegate_;
};

class RapidMapStorePort final : public MapStorePort
{
public:
  explicit RapidMapStorePort(std::shared_ptr<RapidMissionDelegate> delegate);

  [[nodiscard]] bool healthy() const override;
  void start(
    const MotionToken & token,
    const MissionStep & step,
    FeedbackCallback feedback,
    ResultCallback result) override;
  [[nodiscard]] bool cancel(
    const MotionToken & token,
    SteadyClockPort::TimePoint deadline) override;
  void tick(SteadyClockPort::TimePoint) override {}

private:
  std::shared_ptr<RapidMissionDelegate> delegate_;
};

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__RAPID_MISSION_ROS_ADAPTERS_HPP_
