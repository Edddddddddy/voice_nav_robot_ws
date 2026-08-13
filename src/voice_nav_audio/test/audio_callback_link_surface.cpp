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

#include <array>

#include "portaudio_native_callback.hpp"

int main()
{
  std::array<voice_nav_audio::Sample, voice_nav_audio::AudioEngine::kFrameSamples> output{};
  voice_nav_audio::AudioEngine engine;
  engine.process_callback(nullptr, output.data(), output.size(), voice_nav_audio::CallbackStatus{});
  return voice_nav_audio::native_portaudio_callback(nullptr, nullptr, 0U, nullptr, 0U, nullptr);
}
