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

#ifndef VOICE_NAV_AUDIO__SENSEVOICE_SHERPA_ADAPTER_HPP_
#define VOICE_NAV_AUDIO__SENSEVOICE_SHERPA_ADAPTER_HPP_

#include <memory>
#include <string>

#include "sensevoice_provider.hpp"
#include "speech_input_core.hpp"

namespace voice_nav_audio
{

// Package-private explicit asset identity. KWS is one closed model root; its
// fixed file names are hidden by the factory rather than exposed to callers.
struct SherpaSenseVoiceAssetPaths
{
  std::string keyword_model_root{};
  std::string silero_vad_model{};
  std::string sensevoice_model{};
  std::string tokens{};
};

[[nodiscard]] std::unique_ptr<SpeechRecognizerAdapter> make_sherpa_speech_recognizer(
  const SherpaSenseVoiceAssetPaths & assets,
  SenseVoiceProviderConfig config = {});

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SENSEVOICE_SHERPA_ADAPTER_HPP_
