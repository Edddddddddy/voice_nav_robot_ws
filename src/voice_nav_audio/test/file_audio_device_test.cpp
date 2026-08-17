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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "voice_nav_audio/audio_engine.hpp"
#include "voice_nav_audio/portaudio_adapter.hpp"
#include "file_audio_device.hpp"

namespace voice_nav_audio
{
namespace
{

std::uint16_t read_u16(const std::array<char, 44> & header, const std::size_t offset)
{
  return static_cast<std::uint16_t>(static_cast<unsigned char>(header[offset])) |
         (static_cast<std::uint16_t>(static_cast<unsigned char>(header[offset + 1U])) << 8U);
}

std::uint32_t read_u32(const std::array<char, 44> & header, const std::size_t offset)
{
  return static_cast<std::uint32_t>(static_cast<unsigned char>(header[offset])) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(header[offset + 1U])) << 8U) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(header[offset + 2U])) << 16U) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(header[offset + 3U])) << 24U);
}

class FileAudioDeviceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    output_path_ = std::filesystem::temp_directory_path() / "voice_nav_file_audio_device_test.wav";
    std::filesystem::remove(output_path_);
    std::filesystem::remove(output_path_.string() + ".partial");
  }

  void TearDown() override
  {
    std::filesystem::remove(output_path_);
    std::filesystem::remove(output_path_.string() + ".partial");
  }

  std::filesystem::path output_path_;
};

TEST_F(FileAudioDeviceTest, WritesAudioEngineCallbackAs48KhzMonoPcmWav)
{
  AudioEngine engine;
  FileAudioDevice device(output_path_);
  PortAudioAdapter adapter(engine, device);

  ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
  ASSERT_TRUE(device.wait_for_callbacks(1U, std::chrono::seconds(2)));

  std::array<Sample, AudioEngine::kFrameSamples> pcm{};
  pcm.fill(1200);
  ASSERT_TRUE(engine.enqueue_playback(pcm.data(), pcm.size(), 170U));
  ASSERT_TRUE(device.wait_for_callbacks(3U, std::chrono::seconds(2)));

  adapter.stop();
  ASSERT_TRUE(device.commit());

  std::ifstream input(output_path_, std::ios::binary);
  ASSERT_TRUE(input.good());
  std::array<char, 44> header{};
  ASSERT_TRUE(input.read(header.data(), static_cast<std::streamsize>(header.size())));
  EXPECT_EQ(std::string(header.data(), 4), "RIFF");
  EXPECT_EQ(std::string(header.data() + 8, 4), "WAVE");
  EXPECT_EQ(std::string(header.data() + 12, 4), "fmt ");
  EXPECT_EQ(read_u16(header, 20U), 1U);
  EXPECT_EQ(read_u16(header, 22U), 1U);
  EXPECT_EQ(read_u32(header, 24U), 48000U);
  EXPECT_EQ(read_u16(header, 34U), 16U);
  EXPECT_EQ(std::string(header.data() + 36, 4), "data");
  ASSERT_GT(read_u32(header, 40U), 0U);

  const auto data_bytes = read_u32(header, 40U);
  std::vector<char> payload(data_bytes);
  ASSERT_TRUE(input.read(payload.data(), static_cast<std::streamsize>(payload.size())));
  EXPECT_NE(
    std::count(payload.begin(), payload.end(), static_cast<char>(0)),
    static_cast<std::ptrdiff_t>(payload.size()));
}

TEST_F(FileAudioDeviceTest, RejectsRelativeOrExistingOutputWithoutPartialFile)
{
  const auto relative_path = std::filesystem::path("voice_nav_relative_output.wav");
  std::filesystem::remove(relative_path);
  {
    AudioEngine engine;
    FileAudioDevice device(relative_path);
    PortAudioAdapter adapter(engine, device);
    EXPECT_EQ(adapter.start(), AdapterStartResult::NoDevice);
  }
  EXPECT_FALSE(std::filesystem::exists(relative_path));

  {
    std::ofstream existing(output_path_, std::ios::binary);
    ASSERT_TRUE(existing.good());
    existing << "keep";
  }
  {
    AudioEngine engine;
    FileAudioDevice device(output_path_);
    PortAudioAdapter adapter(engine, device);
    EXPECT_EQ(adapter.start(), AdapterStartResult::NoDevice);
  }
  std::ifstream preserved(output_path_, std::ios::binary);
  ASSERT_TRUE(preserved.good());
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(preserved), {}), "keep");
}

TEST_F(FileAudioDeviceTest, RemovesUncommittedPartialOutput)
{
  {
    AudioEngine engine;
    FileAudioDevice device(output_path_);
    PortAudioAdapter adapter(engine, device);
    ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
    ASSERT_TRUE(device.wait_for_callbacks(1U, std::chrono::seconds(2)));
    adapter.stop();
  }

  EXPECT_FALSE(std::filesystem::exists(output_path_));
  EXPECT_FALSE(std::filesystem::exists(output_path_.string() + ".partial"));
}

TEST_F(FileAudioDeviceTest, DoesNotReplaceOutputCreatedAfterOpen)
{
  AudioEngine engine;
  FileAudioDevice device(output_path_);
  PortAudioAdapter adapter(engine, device);

  ASSERT_EQ(adapter.start(), AdapterStartResult::Started);
  ASSERT_TRUE(device.wait_for_callbacks(1U, std::chrono::seconds(2)));
  {
    std::ofstream external_output(output_path_, std::ios::binary);
    ASSERT_TRUE(external_output.good());
    external_output << "external";
  }

  EXPECT_FALSE(device.commit());
  std::ifstream preserved(output_path_, std::ios::binary);
  ASSERT_TRUE(preserved.good());
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(preserved), {}), "external");
  EXPECT_FALSE(std::filesystem::exists(output_path_.string() + ".partial"));
}

TEST_F(FileAudioDeviceTest, PreservesPreexistingPartialWhenOpenFails)
{
  const auto partial_path = output_path_.string() + ".partial";
  {
    std::ofstream external_partial(partial_path, std::ios::binary);
    ASSERT_TRUE(external_partial.good());
    external_partial << "external partial";
  }

  {
    AudioEngine engine;
    FileAudioDevice device(output_path_);
    PortAudioAdapter adapter(engine, device);
    EXPECT_EQ(adapter.start(), AdapterStartResult::NoDevice);
  }

  std::ifstream preserved(partial_path, std::ios::binary);
  ASSERT_TRUE(preserved.good());
  EXPECT_EQ(
    std::string(std::istreambuf_iterator<char>(preserved), {}), "external partial");
}

}  // namespace
}  // namespace voice_nav_audio
