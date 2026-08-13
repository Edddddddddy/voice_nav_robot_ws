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

#ifndef VOICE_NAV_AUDIO__PORTAUDIO_ADAPTER_HPP_
#define VOICE_NAV_AUDIO__PORTAUDIO_ADAPTER_HPP_

#include <cstddef>
#include <memory>

#include "voice_nav_audio/audio_engine.hpp"

namespace voice_nav_audio
{

struct FullDuplexStreamSpec
{
  std::size_t sample_rate{AudioEngine::kSampleRate};
  std::size_t channels{AudioEngine::kChannels};
  std::size_t frames_per_buffer{AudioEngine::kFrameSamples};
};

using DeviceCallback = void (*) (
  void * context,
  const Sample * capture,
  Sample * device_output,
  std::size_t frame_count,
  CallbackStatus status) noexcept;

// The only seam that can reach a physical full-duplex stream.  Test fakes use
// the same contract; AudioEngine itself has no device or PortAudio dependency.
class FullDuplexAudioDevice
{
public:
  virtual ~FullDuplexAudioDevice() = default;

  virtual bool open(
    FullDuplexStreamSpec spec,
    DeviceCallback callback,
    void * context) noexcept = 0;
  virtual void close() noexcept = 0;
};

enum class AdapterStartResult
{
  Started,
  AlreadyStarted,
  NoDevice
};

class PortAudioAdapter final
{
public:
  // The default adapter uses PortAudio only when its explicitly provisioned
  // backend is enabled at build time; otherwise it fails closed as NoDevice.
  explicit PortAudioAdapter(AudioEngine & engine);
  PortAudioAdapter(AudioEngine & engine, FullDuplexAudioDevice & device) noexcept;
  ~PortAudioAdapter();

  PortAudioAdapter(const PortAudioAdapter &) = delete;
  PortAudioAdapter & operator=(const PortAudioAdapter &) = delete;

  [[nodiscard]] AdapterStartResult start() noexcept;
  [[nodiscard]] bool restart() noexcept;
  void stop() noexcept;
  [[nodiscard]] bool running() const noexcept;

private:
  static void callback(
    void * context,
    const Sample * capture,
    Sample * device_output,
    std::size_t frame_count,
    CallbackStatus status) noexcept;

  AudioEngine & engine_;
  std::unique_ptr<FullDuplexAudioDevice> owned_device_;
  FullDuplexAudioDevice * device_{nullptr};
  bool running_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__PORTAUDIO_ADAPTER_HPP_
