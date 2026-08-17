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

#ifndef VOICE_NAV_AUDIO__FILE_AUDIO_DEVICE_HPP_
#define VOICE_NAV_AUDIO__FILE_AUDIO_DEVICE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>

#include "voice_nav_audio/portaudio_adapter.hpp"

namespace voice_nav_audio
{

// Package-private deterministic full-duplex device.  The callback runs on a
// 10 ms clock; file I/O starts only after the callback returns.
class FileAudioDevice final : public FullDuplexAudioDevice
{
public:
  explicit FileAudioDevice(std::filesystem::path output_path) noexcept;
  ~FileAudioDevice() override;

  FileAudioDevice(const FileAudioDevice &) = delete;
  FileAudioDevice & operator=(const FileAudioDevice &) = delete;

  bool open(
    FullDuplexStreamSpec spec, DeviceCallback callback, void * context) noexcept override;
  void close() noexcept override;
  [[nodiscard]] bool commit() noexcept;

  [[nodiscard]] bool wait_for_callbacks(
    std::size_t target, std::chrono::milliseconds timeout);

private:
  void run() noexcept;
  [[nodiscard]] bool write_header(std::uint32_t data_bytes) noexcept;
  void remove_partial() noexcept;

  std::filesystem::path output_path_;
  std::filesystem::path partial_path_;
  FullDuplexStreamSpec spec_{};
  DeviceCallback callback_{nullptr};
  void * context_{nullptr};
  std::ofstream stream_{};
  std::thread worker_{};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> worker_failed_{false};
  std::atomic<std::size_t> callback_count_{0U};
  std::mutex callback_mutex_;
  std::condition_variable callback_condition_;
  std::size_t sample_count_{0U};
  bool opened_{false};
  bool committed_{false};
  bool partial_owned_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__FILE_AUDIO_DEVICE_HPP_
