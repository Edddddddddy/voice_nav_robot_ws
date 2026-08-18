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

#ifndef VOICE_NAV_MISSION__MAP_PACKAGE_HPP_
#define VOICE_NAV_MISSION__MAP_PACKAGE_HPP_

#include <array>
#include <filesystem>
#include <functional>
#include <string>

#include "voice_nav_mission/mission_runtime_core.hpp"

namespace voice_nav_mission
{

inline constexpr std::array<const char *, 6U> kMapPackageFileNames{
  "map.yaml", "map.pgm", "map.posegraph", "map.data", "manifest.yaml",
  "named_places.yaml"};

struct MapPackageArtifacts
{
  std::filesystem::path map_yaml;
  std::filesystem::path map_pgm;
  std::filesystem::path map_posegraph;
  std::filesystem::path map_data;
  std::filesystem::path named_places;
};

struct MapPackage
{
  std::string map_id;
  std::filesystem::path directory;
  MapPackageArtifacts files;
};

struct NamedPlace
{
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

[[nodiscard]] std::filesystem::path default_map_root();
[[nodiscard]] bool valid_map_id(const std::string & map_id) noexcept;

// Deep file transaction module.  Upstream map serializers write into source
// paths; this module only validates, hashes, and atomically publishes them.
class MapPackageWriter final
{
public:
  using SyncFunction = std::function<bool(
    const std::filesystem::path & path, bool directory)>;

  explicit MapPackageWriter(
    std::filesystem::path root,
    SyncFunction sync = {});

  [[nodiscard]] ChildResult publish(
    const std::string & map_id,
    const MapPackageArtifacts & source) const;

private:
  std::filesystem::path root_;
  SyncFunction sync_;
};

class MapPackageReader final
{
public:
  explicit MapPackageReader(std::filesystem::path root);

  [[nodiscard]] ChildResult load(
    const std::string & map_id,
    MapPackage * package) const;

  [[nodiscard]] ChildResult read_named_place(
    const MapPackage & package,
    const std::string & place_id,
    NamedPlace * place) const;

private:
  std::filesystem::path root_;
};

// Production map-store seam.  The capture callback is owned by the Runtime
// Node and delegates occupancy/pose-graph serialization to trusted upstreams.
// It receives a private staging directory and must produce the four upstream
// files there; it never receives a caller-provided root.
class ProductionMapStore final : public MapStorePort
{
public:
  using CaptureCallback = std::function<ChildResult(
    const std::filesystem::path & staging_directory)>;

  ProductionMapStore(
    std::filesystem::path root,
    CaptureCallback capture,
    std::filesystem::path trusted_named_places = {});

  [[nodiscard]] ChildResult save(const std::string & map_id) override;

private:
  std::filesystem::path root_;
  CaptureCallback capture_;
  std::filesystem::path trusted_named_places_;
  MapPackageWriter writer_;
};

using FileMapStore = ProductionMapStore;

}  // namespace voice_nav_mission

#endif  // VOICE_NAV_MISSION__MAP_PACKAGE_HPP_
