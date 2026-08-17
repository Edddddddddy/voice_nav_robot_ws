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

#include "voice_input_policy.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace voice_nav_audio
{
namespace
{

std::filesystem::path fixture_path(const std::string & name)
{
  return std::filesystem::temp_directory_path() /
         ("voice_nav_input_policy_" + name + ".wav");
}

void write_u16(std::ofstream & stream, const std::uint16_t value)
{
  const char bytes[] = {
    static_cast<char>(value & 0xffU),
    static_cast<char>((value >> 8U) & 0xffU),
  };
  stream.write(bytes, sizeof(bytes));
}

void write_u32(std::ofstream & stream, const std::uint32_t value)
{
  const char bytes[] = {
    static_cast<char>(value & 0xffU),
    static_cast<char>((value >> 8U) & 0xffU),
    static_cast<char>((value >> 16U) & 0xffU),
    static_cast<char>((value >> 24U) & 0xffU),
  };
  stream.write(bytes, sizeof(bytes));
}

void write_pcm_wav(
  const std::filesystem::path & path,
  const std::uint32_t frames,
  const std::uint16_t channels = 1U,
  const std::uint32_t sample_rate = 16000U)
{
  const auto data_bytes = frames * channels * 2U;
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream.write("RIFF", 4);
  write_u32(stream, 36U + data_bytes);
  stream.write("WAVEfmt ", 8);
  write_u32(stream, 16U);
  write_u16(stream, 1U);
  write_u16(stream, channels);
  write_u32(stream, sample_rate);
  write_u32(stream, sample_rate * channels * 2U);
  write_u16(stream, channels * 2U);
  write_u16(stream, 16U);
  stream.write("data", 4);
  write_u32(stream, data_bytes);
  const std::string samples(data_bytes, '\0');
  stream.write(samples.data(), static_cast<std::streamsize>(samples.size()));
}

TEST(VoiceInputPolicyTest, AcceptsBoundedNoncanonicalMonoPcmWav)
{
  const auto path = fixture_path("noncanonical");
  write_pcm_wav(path, 1600U);

  const auto result = validate_input_wav(path.string());

  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.reason.empty());
  std::filesystem::remove(path);
}

TEST(VoiceInputPolicyTest, RejectsEmptyUnsupportedAndOversizedInput)
{
  const auto empty_path = fixture_path("empty");
  std::ofstream(empty_path, std::ios::binary).close();
  EXPECT_EQ(validate_input_wav(empty_path.string()).reason, "input_wav_empty");
  std::filesystem::remove(empty_path);

  const auto unsupported_path = fixture_path("unsupported");
  std::ofstream unsupported(unsupported_path, std::ios::binary);
  unsupported << "not a wav";
  unsupported.close();
  EXPECT_EQ(
    validate_input_wav(unsupported_path.string()).reason,
    "input_wav_unsupported_format");
  std::filesystem::remove(unsupported_path);

  const auto oversized_path = fixture_path("oversized");
  write_pcm_wav(oversized_path, 240001U);
  EXPECT_EQ(
    validate_input_wav(oversized_path.string()).reason,
    "input_wav_too_large");
  std::filesystem::remove(oversized_path);
}

TEST(VoiceInputPolicyTest, RejectsUnsupportedRateOrChannelLayout)
{
  const auto path = fixture_path("stereo");
  write_pcm_wav(path, 1600U, 2U, 16000U);

  const auto result = validate_input_wav(path.string());

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "input_wav_unsupported_format");
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace voice_nav_audio
