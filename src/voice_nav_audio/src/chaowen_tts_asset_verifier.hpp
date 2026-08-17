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

#ifndef VOICE_NAV_AUDIO__CHAOWEN_TTS_ASSET_VERIFIER_HPP_
#define VOICE_NAV_AUDIO__CHAOWEN_TTS_ASSET_VERIFIER_HPP_

#include <array>
#include <cstdint>
#include <filesystem>

namespace voice_nav_audio
{

struct ChaowenTtsAssetExpectation
{
  const char * filename;
  std::uintmax_t size;
  const char * sha256;
};

using ChaowenTtsAssetManifest = std::array<ChaowenTtsAssetExpectation, 6U>;
using ChaowenTtsAssetPaths = std::array<std::filesystem::path, 6U>;

const ChaowenTtsAssetManifest & pinned_chaowen_tts_asset_manifest() noexcept;

bool verify_chaowen_tts_assets(
  const ChaowenTtsAssetPaths & paths,
  const ChaowenTtsAssetManifest & manifest) noexcept;

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__CHAOWEN_TTS_ASSET_VERIFIER_HPP_
