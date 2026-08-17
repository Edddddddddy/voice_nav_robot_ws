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

#include "chaowen_tts_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#ifdef VOICE_NAV_AUDIO_WITH_SHERPA_ONNX
#include "sherpa-onnx/c-api/c-api.h"
#endif

namespace voice_nav_audio
{
namespace
{

constexpr std::uint32_t kChaowenSampleRateHz = 22050U;
constexpr std::uint32_t kMono = 1U;
constexpr std::size_t kMaximumTtsChunkSamples = 220U;

ChaowenTtsModelPaths model_paths_from_root(const std::filesystem::path & root)
{
  return ChaowenTtsModelPaths{
    root / "zh_CN-chaowen-medium.onnx", root / "lexicon.txt", root / "tokens.txt",
    root / "phone.fst", root / "date.fst", root / "number.fst"};
}

ChaowenTtsAssetPaths asset_paths(const ChaowenTtsModelPaths & paths) noexcept
{
  return ChaowenTtsAssetPaths{
    paths.model, paths.lexicon, paths.tokens,
    paths.phone_fst, paths.date_fst, paths.number_fst};
}

#ifdef VOICE_NAV_AUDIO_WITH_SHERPA_ONNX
class SherpaChaowenInference final : public ChaowenTtsInference
{
public:
  explicit SherpaChaowenInference(const ChaowenTtsModelPaths & paths) noexcept
  : model_(paths.model.string()), lexicon_(paths.lexicon.string()), tokens_(paths.tokens.string()),
    phone_fst_(paths.phone_fst.string()), date_fst_(paths.date_fst.string()),
    number_fst_(paths.number_fst.string()),
    rule_fsts_(phone_fst_ + "," + date_fst_ + "," + number_fst_)
  {
    SherpaOnnxOfflineTtsConfig config{};
    config.model.vits.model = model_.c_str();
    config.model.vits.lexicon = lexicon_.c_str();
    config.model.vits.tokens = tokens_.c_str();
    config.model.num_threads = 1;
    config.model.provider = "cpu";
    config.model.debug = 0;
    config.rule_fsts = rule_fsts_.c_str();
    config.max_num_sentences = 1;
    config.silence_scale = 0.2F;
    tts_ = SherpaOnnxCreateOfflineTts(&config);
  }

  ~SherpaChaowenInference() override
  {
    if (tts_ != nullptr) {
      SherpaOnnxDestroyOfflineTts(tts_);
    }
  }

  bool generate(
    const std::string & text, std::vector<float> & samples,
    std::uint32_t & sample_rate_hz, std::string & detail) noexcept override
  {
    if (tts_ == nullptr) {
      detail = "Chaowen sherpa-onnx engine could not be created";
      return false;
    }
    SherpaOnnxGenerationConfig generation_config{};
    generation_config.silence_scale = 0.2F;
    generation_config.speed = 1.0F;
    generation_config.sid = 0;
    const auto * audio = SherpaOnnxOfflineTtsGenerateWithConfig(
      tts_, text.c_str(), &generation_config, nullptr, nullptr);
    if (audio == nullptr || audio->samples == nullptr || audio->n <= 0) {
      detail = "Chaowen sherpa-onnx inference returned no audio";
      return false;
    }
    try {
      samples.assign(audio->samples, audio->samples + audio->n);
    } catch (...) {
      SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
      detail = "Chaowen audio allocation failed";
      return false;
    }
    sample_rate_hz = static_cast<std::uint32_t>(audio->sample_rate);
    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
    return true;
  }

private:
  std::string model_;
  std::string lexicon_;
  std::string tokens_;
  std::string phone_fst_;
  std::string date_fst_;
  std::string number_fst_;
  std::string rule_fsts_;
  const SherpaOnnxOfflineTts * tts_{nullptr};
};
#endif

std::unique_ptr<ChaowenTtsInference> make_inference(
  const ChaowenTtsModelPaths & paths) noexcept
{
#ifdef VOICE_NAV_AUDIO_WITH_SHERPA_ONNX
  try {
    return std::make_unique<SherpaChaowenInference>(paths);
  } catch (...) {
    return nullptr;
  }
#else
  (void)paths;
  return nullptr;
#endif
}

}  // namespace

ChaowenTtsAdapter::ChaowenTtsAdapter(const std::filesystem::path & model_root) noexcept
: paths_(model_paths_from_root(model_root)),
  paths_usable_(verify_chaowen_tts_assets(
      asset_paths(paths_), pinned_chaowen_tts_asset_manifest()))
{
  if (paths_usable_) {
    inference_ = make_inference(paths_);
  }
}

ChaowenTtsAdapter::ChaowenTtsAdapter(
  ChaowenTtsModelPaths paths, std::unique_ptr<ChaowenTtsInference> inference) noexcept
: ChaowenTtsAdapter(
    std::move(paths), std::move(inference), pinned_chaowen_tts_asset_manifest())
{
}

ChaowenTtsAdapter::ChaowenTtsAdapter(
  ChaowenTtsModelPaths paths, std::unique_ptr<ChaowenTtsInference> inference,
  ChaowenTtsAssetManifest manifest) noexcept
: paths_(std::move(paths)), inference_(std::move(inference)),
  paths_usable_(verify_chaowen_tts_assets(asset_paths(paths_), manifest))
{
}

void ChaowenTtsAdapter::start(const TtsRequest & request, TtsSink & sink) noexcept
{
  canceled_scope_.store(0U, std::memory_order_release);
  if (!paths_usable_) {
    sink.on_failed(request.scope_id, "Chaowen asset paths are not usable");
    return;
  }
  if (inference_ == nullptr) {
    sink.on_failed(request.scope_id, "Chaowen sherpa-onnx runtime is unavailable");
    return;
  }
  if (request.text.empty()) {
    sink.on_failed(request.scope_id, "Chaowen TTS text is empty");
    return;
  }

  try {
    std::vector<float> samples;
    std::uint32_t sample_rate_hz = 0U;
    std::string detail;
    if (!inference_->generate(request.text, samples, sample_rate_hz, detail)) {
      sink.on_failed(request.scope_id, detail.empty() ? "Chaowen inference failed" : detail);
      return;
    }
    if (sample_rate_hz != kChaowenSampleRateHz || samples.empty()) {
      sink.on_failed(request.scope_id, "Chaowen inference returned invalid audio");
      return;
    }

    std::array<Sample, kMaximumTtsChunkSamples> chunk{};
    for (std::size_t offset = 0U; offset < samples.size(); offset += chunk.size()) {
      if (canceled_scope_.load(std::memory_order_acquire) == request.scope_id) {
        return;
      }
      const auto count = std::min(chunk.size(), samples.size() - offset);
      for (std::size_t index = 0U; index < count; ++index) {
        const auto clamped = std::clamp(samples[offset + index], -1.0F, 1.0F);
        chunk[index] = static_cast<Sample>(std::lround(clamped * 32767.0F));
      }
      if (!sink.on_pcm(request.scope_id, sample_rate_hz, kMono, chunk.data(), count)) {
        if (canceled_scope_.load(std::memory_order_acquire) != request.scope_id) {
          sink.on_failed(request.scope_id, "SpeechOutputCore rejected Chaowen PCM");
        }
        return;
      }
    }
    if (canceled_scope_.load(std::memory_order_acquire) != request.scope_id) {
      sink.on_complete(request.scope_id);
    }
  } catch (...) {
    if (canceled_scope_.load(std::memory_order_acquire) != request.scope_id) {
      sink.on_failed(request.scope_id, "Chaowen inference raised an exception");
    }
  }
}

void ChaowenTtsAdapter::cancel(const std::uint64_t scope_id) noexcept
{
  canceled_scope_.store(scope_id, std::memory_order_release);
}

}  // namespace voice_nav_audio
