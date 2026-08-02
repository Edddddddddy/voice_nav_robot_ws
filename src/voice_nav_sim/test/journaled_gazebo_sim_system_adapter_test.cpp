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

#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace
{

using GazeboSystemInterface = gz_ros2_control::GazeboSimSystemInterface;

TEST(
  JournaledGazeboSimSystemAdapter,
  LoadsExportedAdapterAndItsPinnedUpstream)
{
  pluginlib::ClassLoader<GazeboSystemInterface> loader(
    "gz_ros2_control",
    "gz_ros2_control::GazeboSimSystemInterface");

  std::shared_ptr<GazeboSystemInterface> adapter;
  ASSERT_NO_THROW(
    adapter = loader.createSharedInstance(
      "voice_nav_sim/JournaledGazeboSimSystemAdapter"));
  ASSERT_NE(adapter, nullptr);

  adapter.reset();
}

}  // namespace
