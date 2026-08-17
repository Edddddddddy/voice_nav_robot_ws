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

#include "file_audio_device.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <unistd.h>

namespace voice_nav_audio
{
namespace
{

constexpr std::size_t kWavHeaderBytes = 44U;
constexpr std::chrono::milliseconds kCallbackPeriod{10};

void put_u16(
  std::array<char, kWavHeaderBytes> & header, const std::size_t offset,
  const std::uint16_t value) noexcept
{
  header[offset] = static_cast<char>(value & 0xffU);
  header[offset + 1U] = static_cast<char>((value >> 8U) & 0xffU);
}

void put_u32(
  std::array<char, kWavHeaderBytes> & header, const std::size_t offset,
  const std::uint32_t value) noexcept
{
  header[offset] = static_cast<char>(value & 0xffU);
  header[offset + 1U] = static_cast<char>((value >> 8U) & 0xffU);
  header[offset + 2U] = static_cast<char>((value >> 16U) & 0xffU);
  header[offset + 3U] = static_cast<char>((value >> 24U) & 0xffU);
}

void put_tag(
  std::array<char, kWavHeaderBytes> & header, const std::size_t offset,
  const char (& tag)[5]) noexcept
{
  header[offset] = tag[0];
  header[offset + 1U] = tag[1];
  header[offset + 2U] = tag[2];
  header[offset + 3U] = tag[3];
}

}  // namespace

FileAudioDevice::FileAudioDevice(std::filesystem::path output_path) noexcept
: output_path_(std::move(output_path))
{
}

FileAudioDevice::~FileAudioDevice()
{
  close();
  if (!committed_) {
    remove_partial();
  }
}

bool FileAudioDevice::open(
  const FullDuplexStreamSpec spec, const DeviceCallback callback, void * const context) noexcept
{
  if (opened_ || committed_ || callback == nullptr ||
    spec.sample_rate != AudioEngine::kSampleRate || spec.channels != AudioEngine::kChannels ||
    spec.frames_per_buffer != AudioEngine::kFrameSamples || !output_path_.is_absolute())
  {
    return false;
  }

  std::error_code error;
  const auto parent = output_path_.parent_path();
  if (parent.empty() || !std::filesystem::is_directory(parent, error) ||
    std::filesystem::exists(output_path_, error))
  {
    return false;
  }

  partial_path_ = output_path_;
  partial_path_ += ".partial";
  partial_owned_ = false;
  if (std::filesystem::exists(partial_path_, error)) {
    return false;
  }

  stream_.open(partial_path_, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!stream_.good()) {
    return false;
  }
  partial_owned_ = true;
  if (!write_header(0U)) {
    remove_partial();
    return false;
  }

  spec_ = spec;
  callback_ = callback;
  context_ = context;
  stop_requested_.store(false, std::memory_order_release);
  worker_failed_.store(false, std::memory_order_release);
  callback_count_.store(0U, std::memory_order_release);
  sample_count_ = 0U;
  opened_ = true;
  try {
    worker_ = std::thread(&FileAudioDevice::run, this);
  } catch (...) {
    opened_ = false;
    callback_ = nullptr;
    context_ = nullptr;
    stream_.close();
    remove_partial();
    return false;
  }
  return true;
}

void FileAudioDevice::close() noexcept
{
  if (!opened_ && !worker_.joinable()) {
    return;
  }
  stop_requested_.store(true, std::memory_order_release);
  if (worker_.joinable()) {
    worker_.join();
  }
  callback_ = nullptr;
  context_ = nullptr;
  callback_condition_.notify_all();
}

bool FileAudioDevice::commit() noexcept
{
  if (committed_) {
    return true;
  }
  if (!opened_ && !worker_.joinable()) {
    return false;
  }
  close();
  if (worker_failed_.load(std::memory_order_acquire) ||
    sample_count_ > std::numeric_limits<std::uint32_t>::max() / sizeof(Sample))
  {
    remove_partial();
    opened_ = false;
    return false;
  }

  const auto data_bytes = static_cast<std::uint32_t>(sample_count_ * sizeof(Sample));
  if (!write_header(data_bytes)) {
    remove_partial();
    opened_ = false;
    return false;
  }
  stream_.close();

  if (::link(partial_path_.c_str(), output_path_.c_str()) != 0) {
    remove_partial();
    opened_ = false;
    return false;
  }
  if (::unlink(partial_path_.c_str()) != 0) {
    remove_partial();
    opened_ = false;
    return false;
  }
  partial_owned_ = false;
  opened_ = false;
  committed_ = true;
  return true;
}

bool FileAudioDevice::wait_for_callbacks(
  const std::size_t target, const std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(callback_mutex_);
  callback_condition_.wait_for(lock, timeout, [this, target]() {
      return callback_count_.load(std::memory_order_acquire) >= target ||
             worker_failed_.load(std::memory_order_acquire) || !opened_;
  });
  return callback_count_.load(std::memory_order_acquire) >= target;
}

void FileAudioDevice::run() noexcept
{
  std::array<Sample, AudioEngine::kFrameSamples> capture{};
  std::array<Sample, AudioEngine::kFrameSamples> output{};
  auto next_callback = std::chrono::steady_clock::now();
  while (!stop_requested_.load(std::memory_order_acquire)) {
    next_callback += kCallbackPeriod;
    output.fill(0);
    callback_(context_, capture.data(), output.data(), spec_.frames_per_buffer, CallbackStatus{});
    stream_.write(
      reinterpret_cast<const char *>(output.data()),
      static_cast<std::streamsize>(spec_.frames_per_buffer * sizeof(Sample)));
    if (!stream_.good()) {
      worker_failed_.store(true, std::memory_order_release);
      break;
    }
    sample_count_ += spec_.frames_per_buffer;
    callback_count_.fetch_add(1U, std::memory_order_acq_rel);
    callback_condition_.notify_all();
    std::this_thread::sleep_until(next_callback);
  }
  callback_condition_.notify_all();
}

bool FileAudioDevice::write_header(const std::uint32_t data_bytes) noexcept
{
  if (!stream_.good() || data_bytes > std::numeric_limits<std::uint32_t>::max() - 36U) {
    return false;
  }
  std::array<char, kWavHeaderBytes> header{};
  put_tag(header, 0U, "RIFF");
  put_u32(header, 4U, 36U + data_bytes);
  put_tag(header, 8U, "WAVE");
  put_tag(header, 12U, "fmt ");
  put_u32(header, 16U, 16U);
  put_u16(header, 20U, 1U);
  put_u16(header, 22U, static_cast<std::uint16_t>(AudioEngine::kChannels));
  put_u32(header, 24U, static_cast<std::uint32_t>(AudioEngine::kSampleRate));
  put_u32(
    header, 28U,
    static_cast<std::uint32_t>(AudioEngine::kSampleRate * AudioEngine::kChannels * sizeof(Sample)));
  put_u16(header, 32U, static_cast<std::uint16_t>(AudioEngine::kChannels * sizeof(Sample)));
  put_u16(header, 34U, static_cast<std::uint16_t>(sizeof(Sample) * 8U));
  put_tag(header, 36U, "data");
  put_u32(header, 40U, data_bytes);

  stream_.seekp(0, std::ios::beg);
  stream_.write(header.data(), static_cast<std::streamsize>(header.size()));
  stream_.flush();
  return stream_.good();
}

void FileAudioDevice::remove_partial() noexcept
{
  if (stream_.is_open()) {
    stream_.close();
  }
  if (partial_owned_ && !partial_path_.empty()) {
    std::error_code error;
    std::filesystem::remove(partial_path_, error);
    partial_owned_ = false;
  }
}

}  // namespace voice_nav_audio
