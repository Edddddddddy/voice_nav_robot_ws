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

#include "voice_nav_mission/map_package.hpp"

#include <openssl/sha.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace voice_nav_mission
{
namespace
{

constexpr std::size_t kMapIdMaximum = 32U;
constexpr std::size_t kSha256HexLength = SHA256_DIGEST_LENGTH * 2U;
constexpr char kSlamToolboxVersion[] = "2.8.5";
constexpr char kNavigation2Version[] = "1.3.12";

std::atomic<std::uint64_t> g_stage_sequence{0U};

ChildResult failure(std::string detail)
{
  return ChildResult{ChildResultCode::Failed, std::move(detail)};
}

ChildResult unavailable(std::string detail)
{
  return ChildResult{ChildResultCode::DependencyUnavailable, std::move(detail)};
}

bool regular_nonempty_file(const std::filesystem::path & path)
{
  std::error_code error;
  if (
    std::filesystem::is_symlink(path, error) || error ||
    !std::filesystem::is_regular_file(path, error) || error)
  {
    return false;
  }
  const auto size = std::filesystem::file_size(path, error);
  return !error && size > 0U;
}

bool readable_nonempty_file(const std::filesystem::path & path)
{
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return false;
  }
  const auto size = std::filesystem::file_size(path, error);
  return !error && size > 0U;
}

std::string sha256_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return {};
  }
  SHA256_CTX context;
  if (SHA256_Init(&context) != 1) {
    return {};
  }
  std::array<char, 64U * 1024U> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 && SHA256_Update(
        &context, buffer.data(), static_cast<std::size_t>(count)) != 1)
    {
      return {};
    }
  }
  if (!input.eof()) {
    return {};
  }
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  if (SHA256_Final(digest.data(), &context) != 1) {
    return {};
  }
  std::ostringstream output;
  output << std::hex << std::nouppercase << std::setfill('0');
  for (const auto byte : digest) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

std::filesystem::path stage_path(
  const std::filesystem::path & root,
  const std::string & map_id)
{
  const auto sequence = g_stage_sequence.fetch_add(1U);
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return root / (".staging-" + map_id + "-" + std::to_string(stamp) +
                "-" + std::to_string(sequence));
}

void remove_stage(const std::filesystem::path & root, const std::filesystem::path & stage)
{
  std::error_code error;
  if (stage.parent_path() == root && stage.filename().string().rfind(
      ".staging-", 0U) == 0U)
  {
    std::filesystem::remove_all(stage, error);
  }
}

const std::filesystem::path & source_for(
  const MapPackageArtifacts & source,
  const std::string & name)
{
  if (name == "map.yaml") {
    return source.map_yaml;
  }
  if (name == "map.pgm") {
    return source.map_pgm;
  }
  if (name == "map.posegraph") {
    return source.map_posegraph;
  }
  if (name == "map.data") {
    return source.map_data;
  }
  return source.named_places;
}

bool valid_hash(const std::string & value)
{
  if (value.size() != kSha256HexLength) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
    return (character >= '0' && character <= '9') ||
           (character >= 'a' && character <= 'f');
  });
}

bool exact_package_files(const std::filesystem::path & directory)
{
  std::set<std::string> names;
  std::error_code error;
  for (const auto & entry : std::filesystem::directory_iterator(directory, error)) {
    if (error || entry.is_symlink(error) ||
      !entry.is_regular_file(error) || error)
    {
      return false;
    }
    names.insert(entry.path().filename().string());
  }
  if (error || names.size() != kMapPackageFileNames.size()) {
    return false;
  }
  for (const auto * name : kMapPackageFileNames) {
    if (names.find(name) == names.end()) {
      return false;
    }
  }
  return true;
}

struct ManifestValues
{
  std::string map_id;
  std::map<std::string, std::string> hashes;
};

std::string trim(std::string value)
{
  while (!value.empty() && (value.back() == '\r' ||
    std::isspace(static_cast<unsigned char>(value.back())) != 0))
  {
    value.pop_back();
  }
  std::size_t first = 0U;
  while (first < value.size() &&
    std::isspace(static_cast<unsigned char>(value[first])) != 0)
  {
    ++first;
  }
  return value.substr(first);
}

std::string default_named_places(const std::string & map_id)
{
  std::ostringstream output;
  output << "schema_version: 1\n"
         << "map_id: " << map_id << "\n"
         << "places:\n"
         << "  study:\n"
         << "    frame: map\n"
         << "    x: 0.5\n"
         << "    y: 0.0\n"
         << "    yaw: 0.0\n";
  return output.str();
}

bool valid_fixed_file_name(const std::string & name)
{
  return std::any_of(kMapPackageFileNames.begin(), kMapPackageFileNames.end(),
    [&name](const char * expected) {return name == expected;});
}

bool parse_manifest(
  const std::filesystem::path & path,
  ManifestValues * values)
{
  if (values == nullptr) {
    return false;
  }
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  std::string line;
  if (!std::getline(input, line) || line != "schema_version: 1") {
    return false;
  }
  if (!std::getline(input, line) || line.rfind("map_id: ", 0U) != 0U) {
    return false;
  }
  values->map_id = line.substr(8U);
  if (!valid_map_id(values->map_id)) {
    return false;
  }
  if (!std::getline(input, line) || line != "files:") {
    return false;
  }
  for (const auto * expected : kMapPackageFileNames) {
    if (!std::getline(input, line) || line != std::string{"  - "} + expected) {
      return false;
    }
  }
  if (!std::getline(input, line) || line != "versions:") {
    return false;
  }
  if (!std::getline(input, line) || line !=
    std::string{"  slam_toolbox: "} + kSlamToolboxVersion)
  {
    return false;
  }
  if (!std::getline(input, line) || line !=
    std::string{"  navigation2: "} + kNavigation2Version)
  {
    return false;
  }
  if (!std::getline(input, line) || line != "sha256:") {
    return false;
  }
  values->hashes.clear();
  while (std::getline(input, line)) {
    if (line.size() <= 4U || line.rfind("  ", 0U) != 0U) {
      return false;
    }
    const auto separator = line.find(": ", 2U);
    if (separator == std::string::npos) {
      return false;
    }
    const auto name = line.substr(2U, separator - 2U);
    const auto hash = line.substr(separator + 2U);
    if (name == "manifest.yaml" || !valid_fixed_file_name(name) ||
      !valid_hash(hash) ||
      values->hashes.emplace(name, hash).second == false)
    {
      return false;
    }
  }
  if (!input.eof() || values->hashes.size() != 5U) {
    return false;
  }
  for (const auto * name : kMapPackageFileNames) {
    if (std::string{name} != "manifest.yaml" &&
      values->hashes.find(name) == values->hashes.end())
    {
      return false;
    }
  }
  return true;
}

bool validate_map_yaml(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  bool image_seen = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto value = trim(line);
    if (value.rfind("image:", 0U) != 0U) {
      continue;
    }
    const auto image = trim(value.substr(6U));
    if (image != "map.pgm") {
      return false;
    }
    image_seen = true;
  }
  return input.eof() && image_seen;
}

bool finite_yaml_number(const std::string & value)
{
  try {
    std::size_t consumed = 0U;
    const auto number = std::stod(trim(value), &consumed);
    return consumed == trim(value).size() && std::isfinite(number);
  } catch (...) {
    return false;
  }
}

bool validate_named_places(
  const std::filesystem::path & path,
  const std::string & map_id)
{
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  bool schema_version_seen = false;
  bool map_id_seen = false;
  bool study_seen = false;
  bool frame_seen = false;
  bool x_seen = false;
  bool y_seen = false;
  bool yaw_seen = false;
  bool in_study = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto value = trim(line);
    if (value == "schema_version: 1") {
      schema_version_seen = true;
    }
    if (value == std::string{"map_id: "} + map_id) {
      map_id_seen = true;
    }
    if (value == "study:") {
      study_seen = true;
      in_study = true;
      continue;
    }
    if (line.rfind("  ", 0U) == 0U && line.rfind("    ", 0U) != 0U &&
      value != "study:")
    {
      in_study = false;
    }
    if (!in_study) {
      continue;
    }
    if (value.rfind("frame:", 0U) == 0U) {
      frame_seen = trim(value.substr(6U)) == "map";
    } else if (value.rfind("x:", 0U) == 0U) {
      x_seen = finite_yaml_number(value.substr(2U));
    } else if (value.rfind("y:", 0U) == 0U) {
      y_seen = finite_yaml_number(value.substr(2U));
    } else if (value.rfind("yaw:", 0U) == 0U) {
      yaw_seen = finite_yaml_number(value.substr(4U));
    }
  }
  return input.eof() && schema_version_seen && map_id_seen && study_seen && frame_seen && x_seen &&
         y_seen && yaw_seen;
}

bool parse_named_place(
  const std::filesystem::path & path,
  const std::string & map_id,
  const std::string & place_id,
  NamedPlace * place)
{
  if (place == nullptr || place_id.empty()) {
    return false;
  }
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  const auto parse_number = [](const std::string & value, double * output) {
      try {
        const auto cleaned = trim(value);
        std::size_t consumed = 0U;
        const auto number = std::stod(cleaned, &consumed);
        if (consumed != cleaned.size() || !std::isfinite(number)) {
          return false;
        }
        *output = number;
        return true;
      } catch (...) {
        return false;
      }
    };
  bool schema_version_seen = false;
  bool map_id_seen = false;
  bool target_seen = false;
  bool in_target = false;
  bool frame_seen = false;
  bool x_seen = false;
  bool y_seen = false;
  bool yaw_seen = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto value = trim(line);
    if (value == "schema_version: 1") {
      schema_version_seen = true;
    }
    if (value == std::string{"map_id: "} + map_id) {
      map_id_seen = true;
    }
    if (value == place_id + ":") {
      target_seen = true;
      in_target = true;
      continue;
    }
    if (line.rfind("  ", 0U) == 0U && line.rfind("    ", 0U) != 0U) {
      in_target = false;
    }
    if (!in_target) {
      continue;
    }
    if (value.rfind("frame:", 0U) == 0U) {
      const auto frame = trim(value.substr(6U));
      if (frame != "map") {
        return false;
      }
      place->frame_id = frame;
      frame_seen = true;
    } else if (value.rfind("x:", 0U) == 0U) {
      x_seen = parse_number(value.substr(2U), &place->x);
    } else if (value.rfind("y:", 0U) == 0U) {
      y_seen = parse_number(value.substr(2U), &place->y);
    } else if (value.rfind("yaw:", 0U) == 0U) {
      yaw_seen = parse_number(value.substr(4U), &place->yaw);
    }
  }
  return input.eof() && schema_version_seen && map_id_seen && target_seen &&
         frame_seen && x_seen && y_seen && yaw_seen;
}

bool sync_path(const std::filesystem::path & path, bool directory)
{
#if defined(__linux__)
  const auto flags = directory ? O_RDONLY | O_DIRECTORY : O_RDONLY;
  const auto descriptor = ::open(path.c_str(), flags);
  if (descriptor < 0) {
    return false;
  }
  const auto result = ::fsync(descriptor);
  ::close(descriptor);
  return result == 0;
#else
  (void)path;
  (void)directory;
  return true;
#endif
}

bool validate_package_directory(
  const std::filesystem::path & directory,
  const std::string & map_id)
{
  std::error_code error;
  if (!std::filesystem::is_directory(directory, error) || error ||
    !exact_package_files(directory))
  {
    return false;
  }
  for (const auto * name : kMapPackageFileNames) {
    if (!regular_nonempty_file(directory / name)) {
      return false;
    }
  }
  ManifestValues manifest;
  if (!parse_manifest(directory / "manifest.yaml", &manifest) ||
    manifest.map_id != map_id)
  {
    return false;
  }
  if (!validate_map_yaml(directory / "map.yaml") ||
    !validate_named_places(directory / "named_places.yaml", map_id))
  {
    return false;
  }
  for (const auto & [name, expected] : manifest.hashes) {
    if (sha256_file(directory / name) != expected) {
      return false;
    }
  }
  return sync_path(directory / "map.yaml", false) &&
         sync_path(directory / "map.pgm", false) &&
         sync_path(directory / "map.posegraph", false) &&
         sync_path(directory / "map.data", false) &&
         sync_path(directory / "named_places.yaml", false) &&
         sync_path(directory / "manifest.yaml", false);
}

}  // namespace

std::filesystem::path default_map_root()
{
  const auto * xdg = std::getenv("XDG_DATA_HOME");
  if (xdg != nullptr && *xdg != '\0') {
    return std::filesystem::path(xdg) / "voice_nav" / "maps";
  }
  const auto * home = std::getenv("HOME");
  if (home != nullptr && *home != '\0') {
    return std::filesystem::path(home) / ".local" / "share" / "voice_nav" / "maps";
  }
  return std::filesystem::path("/tmp") / "voice_nav" / "maps";
}

bool valid_map_id(const std::string & map_id) noexcept
{
  if (map_id.empty() || map_id.size() > kMapIdMaximum ||
    map_id.front() < 'a' || map_id.front() > 'z')
  {
    return false;
  }
  return std::all_of(map_id.begin() + 1, map_id.end(), [](const char character) {
    return (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_' ||
           character == '-';
  });
}

MapPackageWriter::MapPackageWriter(
  std::filesystem::path root,
  SyncFunction sync)
: root_(std::filesystem::absolute(std::move(root))),
  sync_(sync ? std::move(sync) : sync_path)
{
}

ChildResult MapPackageWriter::publish(
  const std::string & map_id,
  const MapPackageArtifacts & source) const
{
  if (!valid_map_id(map_id)) {
    return failure("Map ID is invalid");
  }
  std::error_code error;
  std::filesystem::create_directories(root_, error);
  if (error) {
    return unavailable("Map root could not be created: " + error.message());
  }
  const auto destination = root_ / map_id;
  if (std::filesystem::exists(destination, error) || error) {
    return failure("Map package already exists; overwrite is disabled");
  }

  const auto stage = stage_path(root_, map_id);
  std::filesystem::create_directory(stage, error);
  if (error) {
    return unavailable("Map staging directory could not be created: " + error.message());
  }
  const auto cleanup = [&]() {remove_stage(root_, stage);};
  try {
    for (const auto * name : kMapPackageFileNames) {
      if (std::string{name} == "manifest.yaml") {
        continue;
      }
      const auto input = source_for(source, name);
      if (!regular_nonempty_file(input)) {
        cleanup();
        return unavailable("Map upstream artifact is missing or empty: " + input.string());
      }
      std::filesystem::copy_file(input, stage / name,
        std::filesystem::copy_options::none, error);
      if (error || !regular_nonempty_file(stage / name)) {
        cleanup();
        return unavailable("Map upstream artifact could not be staged: " + std::string{name});
      }
    }
    std::ofstream manifest(stage / "manifest.yaml", std::ios::binary | std::ios::trunc);
    if (!manifest) {
      cleanup();
      return unavailable("Map manifest could not be opened");
    }
    manifest << "schema_version: 1\nmap_id: " << map_id << "\nfiles:\n";
    for (const auto * name : kMapPackageFileNames) {
      manifest << "  - " << name << "\n";
    }
    manifest << "versions:\n"
             << "  slam_toolbox: " << kSlamToolboxVersion << "\n"
             << "  navigation2: " << kNavigation2Version << "\n"
             << "sha256:\n";
    for (const auto * name : kMapPackageFileNames) {
      if (std::string{name} == "manifest.yaml") {
        continue;
      }
      const auto hash = sha256_file(stage / name);
      if (!valid_hash(hash)) {
        cleanup();
        return unavailable("Map artifact hash could not be computed: " + std::string{name});
      }
      manifest << "  " << name << ": " << hash << "\n";
    }
    manifest.flush();
    if (!manifest) {
      cleanup();
      return unavailable("Map manifest could not be written");
    }
    manifest.close();
    if (!manifest) {
      cleanup();
      return unavailable("Map manifest could not be closed");
    }
    if (!validate_package_directory(stage, map_id) || !sync_(stage, true)) {
      cleanup();
      return failure("Map staging directory failed pre-publish validation");
    }
    if (std::filesystem::exists(destination, error) || error) {
      cleanup();
      return failure("Map package appeared during publish; overwrite is disabled");
    }
    std::filesystem::rename(stage, destination, error);
    if (error) {
      cleanup();
      return unavailable("Map package atomic publish failed: " + error.message());
    }
    if (!sync_(root_, true)) {
      std::error_code cleanup_error;
      std::filesystem::remove_all(destination, cleanup_error);
      std::error_code verify_error;
      const bool destination_removed =
        !cleanup_error &&
        !std::filesystem::exists(destination, verify_error) &&
        !verify_error;
      const bool root_resynced = sync_(root_, true);
      if (!destination_removed || !root_resynced) {
        return unavailable(
          "Map root fsync failed and completed package cleanup was not proven");
      }
      return unavailable("Map root fsync failed; completed package was removed");
    }
    return ChildResult{ChildResultCode::Succeeded, "Map package published"};
  } catch (const std::exception & exception) {
    cleanup();
    return unavailable(std::string{"Map package transaction failed: "} + exception.what());
  } catch (...) {
    cleanup();
    return unavailable("Map package transaction failed with an unknown error");
  }
}

MapPackageReader::MapPackageReader(std::filesystem::path root)
: root_(std::filesystem::absolute(std::move(root)))
{
}

ChildResult MapPackageReader::load(
  const std::string & map_id,
  MapPackage * package) const
{
  if (package == nullptr || !valid_map_id(map_id)) {
    return failure("Map ID is invalid");
  }
  const auto directory = root_ / map_id;
  std::error_code error;
  if (!std::filesystem::is_directory(directory, error) || error)
  {
    return unavailable("Map package directory is missing or incomplete");
  }
  if (!validate_package_directory(directory, map_id)) {
    return failure("Map package validation failed");
  }
  package->map_id = map_id;
  package->directory = directory;
  package->files = MapPackageArtifacts{
    directory / "map.yaml", directory / "map.pgm", directory / "map.posegraph",
    directory / "map.data", directory / "named_places.yaml"};
  return ChildResult{ChildResultCode::Succeeded, "Map package loaded"};
}

ChildResult MapPackageReader::read_named_place(
  const MapPackage & package,
  const std::string & place_id,
  NamedPlace * place) const
{
  if (place == nullptr || package.map_id.empty() ||
    package.directory != root_ / package.map_id ||
    !valid_map_id(package.map_id) || place_id.empty())
  {
    return failure("Map package named-place request is invalid");
  }
  if (!validate_package_directory(package.directory, package.map_id)) {
    return failure("Map package validation failed before named-place read");
  }
  if (!parse_named_place(package.files.named_places, package.map_id, place_id, place)) {
    return failure("Named place is missing or invalid: " + place_id);
  }
  return ChildResult{ChildResultCode::Succeeded, "Named place loaded"};
}

ProductionMapStore::ProductionMapStore(
  std::filesystem::path root,
  CaptureCallback capture,
  std::filesystem::path trusted_named_places)
: root_(std::filesystem::absolute(std::move(root))),
  capture_(std::move(capture)),
  trusted_named_places_(std::move(trusted_named_places)),
  writer_(root_)
{
}

ChildResult ProductionMapStore::save(const std::string & map_id)
{
  if (!valid_map_id(map_id)) {
    return failure("Map ID is invalid");
  }
  if (!capture_) {
    return unavailable("Map upstream capture is unavailable");
  }
  std::error_code error;
  std::filesystem::create_directories(root_, error);
  if (error) {
    return unavailable("Map root could not be created: " + error.message());
  }
  const auto destination = root_ / map_id;
  if (std::filesystem::exists(destination, error) || error) {
    return failure("Map package already exists; overwrite is disabled");
  }
  const auto stage = stage_path(root_, map_id);
  std::filesystem::create_directory(stage, error);
  if (error) {
    return unavailable("Map upstream staging directory could not be created: " + error.message());
  }
  const auto cleanup = [&]() {remove_stage(root_, stage);};
  try {
    const auto captured = capture_(stage);
    if (captured.code != ChildResultCode::Succeeded) {
      cleanup();
      return captured;
    }
    const auto named_places = stage / "named_places.yaml";
    if (!trusted_named_places_.empty()) {
      if (!readable_nonempty_file(trusted_named_places_)) {
        cleanup();
        return unavailable("Trusted named-place fixture is missing or empty");
      }
      std::filesystem::copy_file(
        trusted_named_places_, named_places,
        std::filesystem::copy_options::none, error);
      if (error) {
        cleanup();
        return unavailable("Trusted named-place fixture could not be staged");
      }
    } else {
      std::ofstream output(named_places, std::ios::binary | std::ios::trunc);
      output << default_named_places(map_id);
      if (!output) {
        cleanup();
        return unavailable("Trusted named-place fixture could not be generated");
      }
    }
    MapPackageArtifacts source{
      stage / "map.yaml", stage / "map.pgm", stage / "map.posegraph",
      stage / "map.data", named_places};
    const auto result = writer_.publish(map_id, source);
    cleanup();
    return result;
  } catch (const std::exception & exception) {
    cleanup();
    return unavailable(std::string{"Map upstream capture failed: "} + exception.what());
  } catch (...) {
    cleanup();
    return unavailable("Map upstream capture failed with an unknown error");
  }
}

}  // namespace voice_nav_mission
