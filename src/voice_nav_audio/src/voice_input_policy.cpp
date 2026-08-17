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

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>

#include "sensevoice_provider.hpp"

namespace voice_nav_audio
{
namespace
{

constexpr std::uint32_t kInputSampleRateHz = 16000U;
constexpr std::uint16_t kInputChannels = 1U;
constexpr std::uint16_t kInputBitsPerSample = 16U;
constexpr std::size_t kMaximumInputFrames =
  SenseVoiceProviderConfig::kDefaultMaximumUtteranceFrames * CleanedAudioFrame::kSamples;
constexpr std::uintmax_t kMaximumInputWavBytes =
  44U + static_cast<std::uintmax_t>(kMaximumInputFrames) * 2U;

VoiceInputValidation rejected(const char * const reason)
{
  return VoiceInputValidation{false, reason};
}

bool tag_equals(const std::array<char, 4U> & tag, const char (& expected)[5]) noexcept
{
  return tag[0] == expected[0] && tag[1] == expected[1] &&
         tag[2] == expected[2] && tag[3] == expected[3];
}

std::uint16_t little_endian_u16(const std::array<std::uint8_t, 16U> & bytes) noexcept
{
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(bytes[1] << 8U);
}

std::uint32_t little_endian_u32(
  const std::uint8_t first,
  const std::uint8_t second,
  const std::uint8_t third,
  const std::uint8_t fourth) noexcept
{
  return static_cast<std::uint32_t>(first) |
         (static_cast<std::uint32_t>(second) << 8U) |
         (static_cast<std::uint32_t>(third) << 16U) |
         (static_cast<std::uint32_t>(fourth) << 24U);
}

bool read_chunk_header(
  std::ifstream & stream,
  std::array<char, 4U> & tag,
  std::uint32_t & size) noexcept
{
  std::array<std::uint8_t, 8U> header{};
  stream.read(reinterpret_cast<char *>(header.data()), header.size());
  if (!stream) {
    return false;
  }
  for (std::size_t index = 0U; index < tag.size(); ++index) {
    tag[index] = static_cast<char>(header[index]);
  }
  size = static_cast<std::uint32_t>(header[4]) |
    (static_cast<std::uint32_t>(header[5]) << 8U) |
    (static_cast<std::uint32_t>(header[6]) << 16U) |
    (static_cast<std::uint32_t>(header[7]) << 24U);
  return true;
}

}  // namespace

VoiceInputValidation validate_input_wav(const std::string & path)
{
  const std::filesystem::path input_path(path);
  if (!input_path.is_absolute()) {
    return rejected("input_wav_must_be_absolute_regular_file");
  }

  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(input_path, filesystem_error) ||
    filesystem_error)
  {
    return rejected("input_wav_must_be_absolute_regular_file");
  }
  const auto file_size = std::filesystem::file_size(input_path, filesystem_error);
  if (filesystem_error) {
    return rejected("input_wav_unreadable");
  }
  if (file_size == 0U) {
    return rejected("input_wav_empty");
  }
  if (file_size > kMaximumInputWavBytes) {
    return rejected("input_wav_too_large");
  }

  std::ifstream stream(input_path, std::ios::binary);
  if (!stream) {
    return rejected("input_wav_unreadable");
  }

  std::array<char, 4U> riff{};
  std::array<char, 4U> wave{};
  std::uint32_t riff_size = 0U;
  stream.read(riff.data(), riff.size());
  stream.read(reinterpret_cast<char *>(&riff_size), sizeof(riff_size));
  stream.read(wave.data(), wave.size());
  if (!stream || !tag_equals(riff, "RIFF") || !tag_equals(wave, "WAVE")) {
    return rejected("input_wav_unsupported_format");
  }
  (void)riff_size;

  bool format_seen = false;
  bool data_seen = false;
  std::uint16_t audio_format = 0U;
  std::uint16_t channels = 0U;
  std::uint32_t sample_rate = 0U;
  std::uint16_t bits_per_sample = 0U;
  std::uint32_t data_bytes = 0U;
  while (stream && stream.tellg() >= 0) {
    const auto chunk_start = static_cast<std::uintmax_t>(stream.tellg());
    if (chunk_start + 8U > file_size) {
      break;
    }
    std::array<char, 4U> tag{};
    std::uint32_t chunk_size = 0U;
    if (!read_chunk_header(stream, tag, chunk_size)) {
      return rejected("input_wav_unsupported_format");
    }
    const auto payload_start = static_cast<std::uintmax_t>(stream.tellg());
    const auto payload_end = payload_start + static_cast<std::uintmax_t>(chunk_size);
    if (payload_end > file_size) {
      return rejected("input_wav_unsupported_format");
    }
    if (tag_equals(tag, "fmt ")) {
      if (chunk_size < 16U) {
        return rejected("input_wav_unsupported_format");
      }
      std::array<std::uint8_t, 16U> format{};
      stream.read(reinterpret_cast<char *>(format.data()), format.size());
      if (!stream) {
        return rejected("input_wav_unsupported_format");
      }
      audio_format = little_endian_u16(format);
      channels = static_cast<std::uint16_t>(format[2]) |
        static_cast<std::uint16_t>(format[3] << 8U);
      sample_rate = little_endian_u32(format[4], format[5], format[6], format[7]);
      bits_per_sample = static_cast<std::uint16_t>(format[14]) |
        static_cast<std::uint16_t>(format[15] << 8U);
      format_seen = true;
    } else if (tag_equals(tag, "data")) {
      data_seen = true;
      data_bytes = chunk_size;
    }
    const auto next_chunk = payload_end + (chunk_size % 2U);
    if (next_chunk > file_size) {
      return rejected("input_wav_unsupported_format");
    }
    stream.seekg(static_cast<std::streamoff>(next_chunk));
  }

  if (!format_seen || !data_seen || audio_format != 1U ||
    channels != kInputChannels || sample_rate != kInputSampleRateHz ||
    bits_per_sample != kInputBitsPerSample || data_bytes == 0U ||
    data_bytes % 2U != 0U || data_bytes / 2U > kMaximumInputFrames)
  {
    return rejected(data_bytes == 0U ? "input_wav_empty" : "input_wav_unsupported_format");
  }
  return VoiceInputValidation{true, {}};
}

}  // namespace voice_nav_audio
