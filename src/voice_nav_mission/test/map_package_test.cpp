// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "voice_nav_mission/map_package.hpp"

namespace voice_nav_mission
{
namespace
{

class MapPackageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() /
      "voice_nav_issue180_map_package_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    std::filesystem::create_directories(source_root_(), error);
    ASSERT_FALSE(error);
    for (const auto * name : kMapPackageFileNames) {
      if (std::string{name} == "manifest.yaml") {
        continue;
      }
      if (std::string{name} == "map.yaml") {
        write(source_root_() / name, "image: map.pgm\nresolution: 0.05\n");
      } else if (std::string{name} == "named_places.yaml") {
        write(
          source_root_() / name,
          "schema_version: 1\n"
          "map_id: voice_mvp\n"
          "places:\n"
          "  study:\n"
          "    frame: map\n"
          "    x: 0.5\n"
          "    y: 0.0\n"
          "    yaw: 0.0\n");
      } else {
        write(source_root_() / name, std::string{"fixture-"} + name + "\n");
      }
    }
    source_.map_yaml = source_root_() / "map.yaml";
    source_.map_pgm = source_root_() / "map.pgm";
    source_.map_posegraph = source_root_() / "map.posegraph";
    source_.map_data = source_root_() / "map.data";
    source_.named_places = source_root_() / "named_places.yaml";
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path source_root_() const
  {
    return root_ / "upstream";
  }

  static void write(const std::filesystem::path & path, const std::string & text)
  {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << text;
    ASSERT_TRUE(static_cast<bool>(output));
  }

  std::filesystem::path root_;
  MapPackageArtifacts source_;
};

TEST_F(MapPackageTest, PublishesExactlySixFilesAndReaderVerifiesHashes)
{
  MapPackageWriter writer(root_ / "maps");
  const auto published = writer.publish("voice_mvp", source_);
  ASSERT_EQ(published.code, ChildResultCode::Succeeded);

  MapPackageReader reader(root_ / "maps");
  MapPackage package;
  const auto loaded = reader.load("voice_mvp", &package);
  ASSERT_EQ(loaded.code, ChildResultCode::Succeeded);
  EXPECT_EQ(package.files.map_yaml.filename(), "map.yaml");
  EXPECT_EQ(package.files.named_places.filename(), "named_places.yaml");
  std::ifstream manifest(package.directory / "manifest.yaml", std::ios::binary);
  const std::string manifest_text((std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(manifest_text.find("schema_version: 1\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("map_id: voice_mvp\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - map.yaml\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - map.pgm\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - map.posegraph\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - map.data\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - manifest.yaml\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  - named_places.yaml\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  slam_toolbox: 2.8.5\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("  navigation2: 1.3.12\n"), std::string::npos);
  EXPECT_NE(manifest_text.find("sha256:\n"), std::string::npos);
  EXPECT_EQ(manifest_text.find("manifest.yaml: "), std::string::npos);
  std::size_t count = 0U;
  for (const auto & entry : std::filesystem::directory_iterator(package.directory)) {
    (void)entry;
    ++count;
  }
  EXPECT_EQ(count, 6U);
}

TEST_F(MapPackageTest, ReaderLoadsStudyFromThePublishedPackage)
{
  MapPackageWriter writer(root_ / "maps");
  ASSERT_EQ(writer.publish("voice_mvp", source_).code, ChildResultCode::Succeeded);

  MapPackage package;
  MapPackageReader reader(root_ / "maps");
  ASSERT_EQ(reader.load("voice_mvp", &package).code, ChildResultCode::Succeeded);

  NamedPlace study;
  ASSERT_EQ(
    reader.read_named_place(package, "study", &study).code,
    ChildResultCode::Succeeded);
  EXPECT_EQ(study.frame_id, "map");
  EXPECT_DOUBLE_EQ(study.x, 0.5);
  EXPECT_DOUBLE_EQ(study.y, 0.0);
  EXPECT_DOUBLE_EQ(study.yaw, 0.0);
}

TEST_F(MapPackageTest, RejectsInvalidIdAndDoesNotCreateDirectory)
{
  MapPackageWriter writer(root_ / "maps");
  const std::vector<std::string> invalid_ids{
    "", "../escape", "Voice", "a/child", "a" + std::string(32U, 'x')};
  for (const auto & map_id : invalid_ids) {
    const auto result = writer.publish(map_id, source_);
    EXPECT_NE(result.code, ChildResultCode::Succeeded) << map_id;
  }
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "escape"));
}

TEST_F(MapPackageTest, RejectsOverwriteAndPreservesFirstPackage)
{
  MapPackageWriter writer(root_ / "maps");
  ASSERT_EQ(writer.publish("voice_mvp", source_).code, ChildResultCode::Succeeded);
  write(source_.map_data, "changed\n");
  const auto result = writer.publish("voice_mvp", source_);
  EXPECT_NE(result.code, ChildResultCode::Succeeded);

  MapPackage package;
  ASSERT_EQ(
    MapPackageReader(root_ / "maps").load("voice_mvp", &package).code,
    ChildResultCode::Succeeded);
  std::ifstream input(package.files.map_data, std::ios::binary);
  std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_EQ(contents, "fixture-map.data\n");
}

TEST_F(MapPackageTest, MissingOrEmptyUpstreamNeverPublishes)
{
  std::filesystem::remove(source_.map_posegraph);
  MapPackageWriter writer(root_ / "maps");
  const auto result = writer.publish("voice_mvp", source_);
  EXPECT_NE(result.code, ChildResultCode::Succeeded);
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "voice_mvp"));
}

TEST_F(MapPackageTest, ReaderRejectsBadHash)
{
  MapPackageWriter writer(root_ / "maps");
  ASSERT_EQ(writer.publish("voice_mvp", source_).code, ChildResultCode::Succeeded);
  write(root_ / "maps" / "voice_mvp" / "map.data", "tampered\n");

  MapPackage package;
  const auto result = MapPackageReader(root_ / "maps").load("voice_mvp", &package);
  EXPECT_NE(result.code, ChildResultCode::Succeeded);
}

TEST_F(MapPackageTest, RejectsUnsafeMapYamlBeforePublish)
{
  write(source_.map_yaml, "image: /tmp/other.pgm\n");
  MapPackageWriter writer(root_ / "maps");
  const auto result = writer.publish("voice_mvp", source_);
  EXPECT_NE(result.code, ChildResultCode::Succeeded);
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "voice_mvp"));
}

TEST_F(MapPackageTest, RejectsNamedPlaceWithWrongMapIdBeforePublish)
{
  write(
    source_.named_places,
    "schema_version: 1\n"
    "map_id: another_map\n"
    "places:\n"
    "  study:\n"
    "    frame: map\n"
    "    x: 0.5\n"
    "    y: 0.0\n"
    "    yaw: 0.0\n");
  MapPackageWriter writer(root_ / "maps");
  const auto result = writer.publish("voice_mvp", source_);
  EXPECT_NE(result.code, ChildResultCode::Succeeded);
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "voice_mvp"));
}

TEST_F(MapPackageTest, UpstreamFailureDoesNotExposeCompletedDirectory)
{
  ProductionMapStore store(
    root_ / "maps",
    [](const std::filesystem::path & staging) {
      std::ofstream(staging / "map.yaml") << "partial\n";
      return ChildResult{ChildResultCode::DependencyUnavailable, "slam failed"};
    });
  const auto result = store.save("voice_mvp");
  EXPECT_EQ(result.code, ChildResultCode::DependencyUnavailable);
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "voice_mvp"));
}

TEST_F(MapPackageTest, ProductionStoreCopiesTrustedSymlinkFixture)
{
  const auto trusted_link = root_ / "trusted-named-places.yaml";
  std::error_code error;
  std::filesystem::create_symlink(source_.named_places, trusted_link, error);
  ASSERT_FALSE(error);
  ASSERT_TRUE(std::filesystem::is_symlink(trusted_link));

  ProductionMapStore store(
    root_ / "maps",
    [this](const std::filesystem::path & staging) {
      std::error_code copy_error;
      for (const auto * name : kMapPackageFileNames) {
        if (std::string{name} == "manifest.yaml" ||
          std::string{name} == "named_places.yaml")
        {
          continue;
        }
        std::filesystem::copy_file(
          source_root_() / name, staging / name,
          std::filesystem::copy_options::none, copy_error);
        if (copy_error) {
          return ChildResult{
            ChildResultCode::DependencyUnavailable,
            "fixture copy failed: " + copy_error.message()};
        }
      }
      return ChildResult{ChildResultCode::Succeeded, "fixture captured"};
    },
    trusted_link);

  const auto result = store.save("voice_mvp");
  ASSERT_EQ(result.code, ChildResultCode::Succeeded) << result.detail;

  const auto published_named_places = root_ / "maps" / "voice_mvp" /
    "named_places.yaml";
  EXPECT_TRUE(std::filesystem::is_regular_file(published_named_places));
  EXPECT_FALSE(std::filesystem::is_symlink(published_named_places));
  MapPackage package;
  ASSERT_EQ(
    MapPackageReader(root_ / "maps").load("voice_mvp", &package).code,
    ChildResultCode::Succeeded);
}

TEST_F(MapPackageTest, DanglingTrustedFixtureDoesNotPublish)
{
  const auto trusted_link = root_ / "dangling-named-places.yaml";
  std::error_code error;
  std::filesystem::create_symlink(root_ / "missing-named-places.yaml", trusted_link, error);
  ASSERT_FALSE(error);

  ProductionMapStore store(
    root_ / "maps",
    [this](const std::filesystem::path & staging) {
      std::error_code copy_error;
      for (const auto * name : {"map.yaml", "map.pgm", "map.posegraph", "map.data"}) {
        std::filesystem::copy_file(
          source_root_() / name, staging / name,
          std::filesystem::copy_options::none, copy_error);
        if (copy_error) {
          return ChildResult{
            ChildResultCode::DependencyUnavailable,
            "fixture copy failed: " + copy_error.message()};
        }
      }
      return ChildResult{ChildResultCode::Succeeded, "fixture captured"};
    },
    trusted_link);

  const auto result = store.save("voice_mvp");
  EXPECT_EQ(result.code, ChildResultCode::DependencyUnavailable);
  EXPECT_FALSE(std::filesystem::exists(root_ / "maps" / "voice_mvp"));
}

TEST_F(MapPackageTest, ReaderRejectsSymlinkInPublishedPackage)
{
  MapPackageWriter writer(root_ / "maps");
  ASSERT_EQ(writer.publish("voice_mvp", source_).code, ChildResultCode::Succeeded);

  const auto package_data = root_ / "maps" / "voice_mvp" / "map.data";
  std::filesystem::remove(package_data);
  std::error_code error;
  std::filesystem::create_symlink(source_.map_data, package_data, error);
  ASSERT_FALSE(error);

  MapPackage package;
  EXPECT_NE(
    MapPackageReader(root_ / "maps").load("voice_mvp", &package).code,
    ChildResultCode::Succeeded);
}

TEST_F(MapPackageTest, RootSyncFailureRemovesRenamedPackage)
{
  const auto map_root = root_ / "maps";
  bool root_sync_failed = false;
  MapPackageWriter writer(
    map_root,
    [&map_root, &root_sync_failed](const std::filesystem::path & path, bool directory) {
      if (directory && path == std::filesystem::absolute(map_root)) {
        if (!root_sync_failed) {
          root_sync_failed = true;
          return false;
        }
      }
      return true;
    });

  const auto result = writer.publish("voice_mvp", source_);
  EXPECT_EQ(result.code, ChildResultCode::DependencyUnavailable);
  EXPECT_TRUE(root_sync_failed);
  EXPECT_FALSE(std::filesystem::exists(map_root / "voice_mvp"));
}

}  // namespace
}  // namespace voice_nav_mission
