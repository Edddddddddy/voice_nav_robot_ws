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

#include "voice_nav_audio/portaudio_adapter.hpp"

#include "portaudio_native_callback.hpp"

#include <utility>

#ifdef VOICE_NAV_AUDIO_WITH_PORTAUDIO
#include <portaudio.h>
#endif

namespace voice_nav_audio
{
namespace
{

class UnavailablePortAudioDevice final : public FullDuplexAudioDevice
{
public:
  bool open(
    const FullDuplexStreamSpec,
    const DeviceCallback,
    void * const) noexcept override
  {
    return false;
  }

  void close() noexcept override
  {
  }
};

#ifdef VOICE_NAV_AUDIO_WITH_PORTAUDIO
class NativePortAudioDevice final : public FullDuplexAudioDevice
{
public:
  ~NativePortAudioDevice() override
  {
    close();
  }

  bool open(
    const FullDuplexStreamSpec spec,
    const DeviceCallback callback,
    void * const context) noexcept override
  {
    if (stream_ != nullptr || Pa_Initialize() != paNoError) {
      return false;
    }
    initialized_ = true;
    callback_context_.callback = callback;
    callback_context_.context = context;
    if (Pa_OpenDefaultStream(
        &stream_, 1, 1, paInt16, static_cast<double>(spec.sample_rate),
        static_cast<unsigned long>(spec.frames_per_buffer), &native_portaudio_callback,
        &callback_context_) != paNoError ||
      Pa_StartStream(stream_) != paNoError)
    {
      close();
      return false;
    }
    return true;
  }

  void close() noexcept override
  {
    if (stream_ != nullptr) {
      (void)Pa_StopStream(stream_);
      (void)Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    callback_context_ = NativePortAudioCallbackContext{};
    if (initialized_) {
      (void)Pa_Terminate();
      initialized_ = false;
    }
  }

private:
  PaStream * stream_{nullptr};
  NativePortAudioCallbackContext callback_context_{};
  bool initialized_{false};
};
#endif

}  // namespace

PortAudioAdapter::PortAudioAdapter(AudioEngine & engine)
: engine_(engine),
#ifdef VOICE_NAV_AUDIO_WITH_PORTAUDIO
  owned_device_(std::make_unique<NativePortAudioDevice>()),
#else
  owned_device_(std::make_unique<UnavailablePortAudioDevice>()),
#endif
  device_(owned_device_.get())
{
}

PortAudioAdapter::PortAudioAdapter(
  AudioEngine & engine,
  FullDuplexAudioDevice & device) noexcept
: engine_(engine),
  device_(&device)
{
}

PortAudioAdapter::~PortAudioAdapter()
{
  stop();
}

AdapterStartResult PortAudioAdapter::start() noexcept
{
  if (running_) {
    return AdapterStartResult::AlreadyStarted;
  }

  // A successful start has a fresh generation.  A failed start fences any
  // queued playback just as strictly, so a later recovery cannot leak it.
  engine_.mark_discontinuity();
  if (device_ == nullptr) {
    return AdapterStartResult::NoDevice;
  }
  if (!device_->open(FullDuplexStreamSpec{}, &PortAudioAdapter::callback, &engine_)) {
    device_->close();
    return AdapterStartResult::NoDevice;
  }
  running_ = true;
  return AdapterStartResult::Started;
}

bool PortAudioAdapter::restart() noexcept
{
  stop();
  return start() == AdapterStartResult::Started;
}

void PortAudioAdapter::stop() noexcept
{
  if (running_ && device_ != nullptr) {
    device_->close();
  }
  running_ = false;
  engine_.mark_discontinuity();
}

bool PortAudioAdapter::running() const noexcept
{
  return running_;
}

}  // namespace voice_nav_audio
