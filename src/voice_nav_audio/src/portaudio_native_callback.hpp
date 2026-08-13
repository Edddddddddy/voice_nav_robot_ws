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

#ifndef VOICE_NAV_AUDIO__PORTAUDIO_NATIVE_CALLBACK_HPP_
#define VOICE_NAV_AUDIO__PORTAUDIO_NATIVE_CALLBACK_HPP_

#include "voice_nav_audio/portaudio_adapter.hpp"

struct PaStreamCallbackTimeInfo;

namespace voice_nav_audio
{

struct NativePortAudioCallbackContext
{
  DeviceCallback callback{nullptr};
  void * context{nullptr};
};

int native_portaudio_callback(
  const void * input,
  void * output,
  unsigned long frame_count,
  const PaStreamCallbackTimeInfo * time_info,
  unsigned long status_flags,
  void * user_data) noexcept;

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__PORTAUDIO_NATIVE_CALLBACK_HPP_
