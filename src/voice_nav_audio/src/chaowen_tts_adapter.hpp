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

#ifndef VOICE_NAV_AUDIO__CHAOWEN_TTS_ADAPTER_HPP_
#define VOICE_NAV_AUDIO__CHAOWEN_TTS_ADAPTER_HPP_

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "chaowen_tts_asset_verifier.hpp"
#include "speech_output_core.hpp"

namespace voice_nav_audio
{

struct ChaowenTtsModelPaths
{
  std::filesystem::path model;
  std::filesystem::path lexicon;
  std::filesystem::path tokens;
  std::filesystem::path phone_fst;
  std::filesystem::path date_fst;
  std::filesystem::path number_fst;
};

class ChaowenTtsInference
{
public:
  virtual ~ChaowenTtsInference() = default;

  virtual bool generate(
    const std::string & text, std::vector<float> & samples,
    std::uint32_t & sample_rate_hz, std::string & detail) noexcept = 0;
};

class ChaowenTtsAdapter final : public TtsAdapter
{
public:
  explicit ChaowenTtsAdapter(const std::filesystem::path & model_root) noexcept;
  ChaowenTtsAdapter(
    ChaowenTtsModelPaths paths, std::unique_ptr<ChaowenTtsInference> inference) noexcept;
  ChaowenTtsAdapter(
    ChaowenTtsModelPaths paths, std::unique_ptr<ChaowenTtsInference> inference,
    ChaowenTtsAssetManifest manifest) noexcept;

  void start(const TtsRequest & request, TtsSink & sink) noexcept override;
  void cancel(std::uint64_t scope_id) noexcept override;

private:
  ChaowenTtsModelPaths paths_{};
  std::unique_ptr<ChaowenTtsInference> inference_{};
  std::atomic<std::uint64_t> canceled_scope_{0U};
  bool paths_usable_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__CHAOWEN_TTS_ADAPTER_HPP_
