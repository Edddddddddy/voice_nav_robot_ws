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

namespace voice_nav_audio
{

// Package-private resolved asset identity. The caller selects exact local
// files; the factory never discovers, downloads, or redistributes models.
struct SherpaSenseVoiceAssetPaths
{
  std::string silero_vad_model{};
  std::string sensevoice_model{};
  std::string tokens{};
  std::string kws_encoder{};
  std::string kws_decoder{};
  std::string kws_joiner{};
  std::string kws_tokens{};
  std::string kws_keywords{};
};

[[nodiscard]] std::unique_ptr<SenseVoiceProvider> make_sherpa_sensevoice_provider(
  const SherpaSenseVoiceAssetPaths & assets,
  SenseVoiceProviderConfig config = {});

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__SENSEVOICE_SHERPA_ADAPTER_HPP_
