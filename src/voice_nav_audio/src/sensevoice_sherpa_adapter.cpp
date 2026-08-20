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

#include "sensevoice_sherpa_adapter.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sherpa-onnx/c-api/c-api.h"

namespace voice_nav_audio
{
namespace
{

constexpr int32_t kSampleRateHz = 16000;

void require_regular_file(const std::string & path, const char * const description)
{
  std::error_code error;
  if (path.empty() || !std::filesystem::is_regular_file(path, error) || error) {
    throw std::invalid_argument(std::string("resolved ") + description + " is not a file");
  }
}

class SherpaSileroVadAdapter final : public SileroVadAdapter
{
public:
  static constexpr std::size_t kWindowSamples = 512U;

  explicit SherpaSileroVadAdapter(const std::string & model_path)
  {
    SherpaOnnxVadModelConfig config{};
    config.silero_vad.model = model_path.c_str();
    config.silero_vad.threshold = 0.25F;
    config.silero_vad.min_silence_duration = 0.5F;
    config.silero_vad.min_speech_duration = 0.5F;
    config.silero_vad.max_speech_duration = 10.0F;
    config.silero_vad.window_size = 512;
    config.sample_rate = kSampleRateHz;
    config.num_threads = 1;
    config.provider = "cpu";
    config.debug = 0;

    detector_ = SherpaOnnxCreateVoiceActivityDetector(&config, 30.0F);
    if (detector_ == nullptr) {
      throw std::runtime_error("sherpa-onnx could not create the Silero VAD");
    }
  }

  ~SherpaSileroVadAdapter() override
  {
    if (detector_ != nullptr) {
      SherpaOnnxDestroyVoiceActivityDetector(detector_);
    }
  }

  [[nodiscard]] SileroVadResult process(
    const CleanedAudioFrame & frame) noexcept override
  {
    if (detector_ == nullptr || frame.sample_rate_hz != CleanedAudioFrame::kSampleRateHz ||
      frame.channels != CleanedAudioFrame::kChannels || frame.valid_samples == 0U ||
      frame.valid_samples > CleanedAudioFrame::kSamples || input_finished_)
    {
      return SileroVadResult{};
    }
    accepted_samples_ += frame.valid_samples;

    std::size_t frame_offset = 0U;
    while (frame_offset < frame.valid_samples) {
      const auto copy_count = std::min(
        kWindowSamples - pending_count_, frame.valid_samples - frame_offset);
      for (std::size_t index = 0U; index < copy_count; ++index) {
        pending_samples_[pending_count_ + index] =
          static_cast<float>(frame.samples[frame_offset + index]) / 32768.0F;
      }
      pending_count_ += copy_count;
      frame_offset += copy_count;
      if (pending_count_ != kWindowSamples) {
        continue;
      }

      SherpaOnnxVoiceActivityDetectorAcceptWaveform(
        detector_, pending_samples_.data(), static_cast<int32_t>(pending_count_));
      pending_count_ = 0U;
      if (SherpaOnnxVoiceActivityDetectorEmpty(detector_) == 0) {
        std::size_t endpoint_sample_exclusive = 0U;
        bool invalid_endpoint = false;
        std::size_t segment_count = 0U;
        while (SherpaOnnxVoiceActivityDetectorEmpty(detector_) == 0) {
          const auto * const segment = SherpaOnnxVoiceActivityDetectorFront(detector_);
          ++segment_count;
          if (segment == nullptr || segment->start < 0 || segment->n <= 0) {
            invalid_endpoint = true;
          } else {
            const auto end = static_cast<std::uint64_t>(segment->start) +
              static_cast<std::uint64_t>(segment->n);
            if (end == 0U || end > accepted_samples_ ||
              end <= last_endpoint_sample_exclusive_ ||
              end > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            {
              invalid_endpoint = true;
            } else {
              endpoint_sample_exclusive = static_cast<std::size_t>(end);
              last_endpoint_sample_exclusive_ = endpoint_sample_exclusive;
            }
          }
          if (segment != nullptr) {
            SherpaOnnxDestroySpeechSegment(segment);
          }
          SherpaOnnxVoiceActivityDetectorPop(detector_);
        }
        last_decision_ = SileroVadDecision::kEndpoint;
        return invalid_endpoint || segment_count != 1U ?
               SileroVadResult{SileroVadDecision::kEndpoint, 0U} :
               SileroVadResult{SileroVadDecision::kEndpoint, endpoint_sample_exclusive};
      }
      last_decision_ = SherpaOnnxVoiceActivityDetectorDetected(detector_) != 0 ?
        SileroVadDecision::kSpeech : SileroVadDecision::kSilence;
    }
    return SileroVadResult{last_decision_, 0U};
  }

  [[nodiscard]] SileroVadFlushResult finish_input() noexcept override
  {
    if (detector_ == nullptr || input_finished_) {
      return SileroVadFlushResult{SileroVadFlushStatus::kInvalid, 0U};
    }
    input_finished_ = true;
    pending_count_ = 0U;
    SherpaOnnxVoiceActivityDetectorFlush(detector_);

    std::size_t segment_count = 0U;
    std::size_t endpoint_sample_exclusive = 0U;
    bool invalid_endpoint = false;
    while (SherpaOnnxVoiceActivityDetectorEmpty(detector_) == 0) {
      const auto * const segment = SherpaOnnxVoiceActivityDetectorFront(detector_);
      ++segment_count;
      if (segment == nullptr || segment->start < 0 || segment->n <= 0) {
        invalid_endpoint = true;
      } else {
        const auto end = static_cast<std::uint64_t>(segment->start) +
          static_cast<std::uint64_t>(segment->n);
        if (end == 0U || end > accepted_samples_ ||
          end <= last_endpoint_sample_exclusive_ ||
          end > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
        {
          invalid_endpoint = true;
        } else if (segment_count == 1U) {
          endpoint_sample_exclusive = static_cast<std::size_t>(end);
        }
      }
      if (segment != nullptr) {
        SherpaOnnxDestroySpeechSegment(segment);
      }
      SherpaOnnxVoiceActivityDetectorPop(detector_);
    }
    if (segment_count == 0U) {
      return SileroVadFlushResult{SileroVadFlushStatus::kEmpty, 0U};
    }
    if (segment_count > 1U) {
      return SileroVadFlushResult{SileroVadFlushStatus::kMultiple, 0U};
    }
    if (invalid_endpoint || endpoint_sample_exclusive == 0U) {
      return SileroVadFlushResult{SileroVadFlushStatus::kInvalid, 0U};
    }
    last_endpoint_sample_exclusive_ = endpoint_sample_exclusive;
    last_decision_ = SileroVadDecision::kEndpoint;
    return SileroVadFlushResult{
      SileroVadFlushStatus::kUnique, endpoint_sample_exclusive};
  }

  void reset() noexcept override
  {
    pending_count_ = 0U;
    last_decision_ = SileroVadDecision::kSilence;
    accepted_samples_ = 0U;
    last_endpoint_sample_exclusive_ = 0U;
    input_finished_ = false;
    if (detector_ != nullptr) {
      SherpaOnnxVoiceActivityDetectorReset(detector_);
    }
  }

private:
  const SherpaOnnxVoiceActivityDetector * detector_{nullptr};
  std::array<float, kWindowSamples> pending_samples_{};
  std::size_t pending_count_{0U};
  SileroVadDecision last_decision_{SileroVadDecision::kSilence};
  std::size_t accepted_samples_{0U};
  std::size_t last_endpoint_sample_exclusive_{0U};
  bool input_finished_{false};
};

struct OfflineStreamDeleter
{
  void operator()(const SherpaOnnxOfflineStream * stream) const noexcept
  {
    if (stream != nullptr) {
      SherpaOnnxDestroyOfflineStream(stream);
    }
  }
};

struct OfflineResultDeleter
{
  void operator()(const SherpaOnnxOfflineRecognizerResult * result) const noexcept
  {
    if (result != nullptr) {
      SherpaOnnxDestroyOfflineRecognizerResult(result);
    }
  }
};

using OfflineStreamPtr = std::unique_ptr<const SherpaOnnxOfflineStream, OfflineStreamDeleter>;
using OfflineResultPtr =
  std::unique_ptr<const SherpaOnnxOfflineRecognizerResult, OfflineResultDeleter>;

class SherpaSenseVoiceAsrAdapter final : public SenseVoiceAsrAdapter
{
public:
  SherpaSenseVoiceAsrAdapter(
    const std::string & model_path, const std::string & tokens_path)
  : model_path_(model_path), tokens_path_(tokens_path)
  {
    SherpaOnnxOfflineRecognizerConfig config{};
    config.feat_config.sample_rate = kSampleRateHz;
    config.feat_config.feature_dim = 80;
    config.model_config.debug = 0;
    config.model_config.num_threads = 1;
    config.model_config.provider = "cpu";
    config.model_config.tokens = tokens_path_.c_str();
    config.model_config.sense_voice.model = model_path_.c_str();
    config.model_config.sense_voice.language = "auto";
    config.model_config.sense_voice.use_itn = 1;
    config.decoding_method = "greedy_search";

    recognizer_ = SherpaOnnxCreateOfflineRecognizer(&config);
    if (recognizer_ == nullptr) {
      throw std::runtime_error("sherpa-onnx could not create the SenseVoice recognizer");
    }
  }

  ~SherpaSenseVoiceAsrAdapter() override
  {
    if (recognizer_ != nullptr) {
      SherpaOnnxDestroyOfflineRecognizer(recognizer_);
    }
  }

  bool infer(
    const Sample * samples, std::size_t sample_count, std::string & labeled_text) noexcept override
  {
    labeled_text.clear();
    if (recognizer_ == nullptr || samples == nullptr || sample_count == 0U ||
      sample_count > static_cast<std::size_t>(std::numeric_limits<int32_t>::max()))
    {
      return false;
    }

    try {
      std::vector<float> waveform(sample_count);
      for (std::size_t index = 0U; index < sample_count; ++index) {
        waveform[index] = static_cast<float>(samples[index]) / 32768.0F;
      }

      OfflineStreamPtr stream{SherpaOnnxCreateOfflineStream(recognizer_)};
      if (!stream) {
        return false;
      }
      SherpaOnnxAcceptWaveformOffline(
        stream.get(), kSampleRateHz, waveform.data(), static_cast<int32_t>(waveform.size()));
      SherpaOnnxDecodeOfflineStream(recognizer_, stream.get());
      OfflineResultPtr result{SherpaOnnxGetOfflineStreamResult(stream.get())};
      if (!result || result->text == nullptr || result->text[0] == '\0') {
        return false;
      }
      labeled_text.assign(result->text);
      return !labeled_text.empty();
    } catch (...) {
      labeled_text.clear();
      return false;
    }
  }

private:
  const std::string model_path_;
  const std::string tokens_path_;
  const SherpaOnnxOfflineRecognizer * recognizer_{nullptr};
};

struct KeywordSpotterDeleter
{
  void operator()(const SherpaOnnxKeywordSpotter * spotter) const noexcept
  {
    if (spotter != nullptr) {
      SherpaOnnxDestroyKeywordSpotter(spotter);
    }
  }
};

struct KeywordStreamDeleter
{
  void operator()(const SherpaOnnxOnlineStream * stream) const noexcept
  {
    if (stream != nullptr) {
      SherpaOnnxDestroyOnlineStream(stream);
    }
  }
};

struct KeywordResultDeleter
{
  void operator()(const SherpaOnnxKeywordResult * result) const noexcept
  {
    if (result != nullptr) {
      SherpaOnnxDestroyKeywordResult(result);
    }
  }
};

class SherpaKeywordSpotterAdapter final : public KeywordSpotterAdapter
{
public:
  explicit SherpaKeywordSpotterAdapter(const SherpaSenseVoiceAssetPaths & assets)
  {
    SherpaOnnxKeywordSpotterConfig config{};
    config.feat_config.sample_rate = kSampleRateHz;
    config.feat_config.feature_dim = 80;
    config.model_config.transducer.encoder = assets.kws_encoder.c_str();
    config.model_config.transducer.decoder = assets.kws_decoder.c_str();
    config.model_config.transducer.joiner = assets.kws_joiner.c_str();
    config.model_config.tokens = assets.kws_tokens.c_str();
    config.model_config.provider = "cpu";
    config.model_config.num_threads = 1;
    config.max_active_paths = 4;
    config.keywords_score = 3.0F;
    config.keywords_threshold = 0.1F;
    config.keywords_file = assets.kws_keywords.c_str();

    spotter_.reset(SherpaOnnxCreateKeywordSpotter(&config));
    if (!spotter_) {
      throw std::runtime_error("sherpa-onnx could not create the keyword spotter");
    }
    stream_.reset(SherpaOnnxCreateKeywordStream(spotter_.get()));
    if (!stream_) {
      throw std::runtime_error("sherpa-onnx could not create the keyword stream");
    }
  }

  bool process(const CleanedAudioFrame & frame) noexcept override
  {
    if (latched_ || !stream_ || frame.sample_rate_hz != CleanedAudioFrame::kSampleRateHz ||
      frame.channels != CleanedAudioFrame::kChannels || frame.valid_samples == 0U ||
      frame.valid_samples > CleanedAudioFrame::kSamples)
    {
      return false;
    }

    std::array<float, CleanedAudioFrame::kSamples> waveform{};
    for (std::size_t index = 0U; index < frame.valid_samples; ++index) {
      waveform[index] = static_cast<float>(frame.samples[index]) / 32768.0F;
    }
    SherpaOnnxOnlineStreamAcceptWaveform(
      stream_.get(), kSampleRateHz, waveform.data(), static_cast<int32_t>(frame.valid_samples));
    while (SherpaOnnxIsKeywordStreamReady(spotter_.get(), stream_.get()) != 0) {
      SherpaOnnxDecodeKeywordStream(spotter_.get(), stream_.get());
    }

    std::unique_ptr<const SherpaOnnxKeywordResult, KeywordResultDeleter> result{
      SherpaOnnxGetKeywordResult(spotter_.get(), stream_.get())};
    if (!result || result->keyword == nullptr || result->keyword[0] == '\0') {
      return false;
    }
    latched_ = true;
    return true;
  }

  void reset() noexcept override
  {
    latched_ = false;
    if (spotter_ && stream_) {
      SherpaOnnxResetKeywordStream(spotter_.get(), stream_.get());
    }
  }

private:
  std::unique_ptr<const SherpaOnnxKeywordSpotter, KeywordSpotterDeleter> spotter_{};
  std::unique_ptr<const SherpaOnnxOnlineStream, KeywordStreamDeleter> stream_{};
  bool latched_{false};
};

}  // namespace

std::unique_ptr<SenseVoiceProvider> make_sherpa_sensevoice_provider(
  const SherpaSenseVoiceAssetPaths & assets, const SenseVoiceProviderConfig config)
{
  require_regular_file(assets.silero_vad_model, "Silero VAD model");
  require_regular_file(assets.sensevoice_model, "SenseVoice model");
  require_regular_file(assets.tokens, "SenseVoice tokens");
  require_regular_file(assets.kws_encoder, "KWS encoder");
  require_regular_file(assets.kws_decoder, "KWS decoder");
  require_regular_file(assets.kws_joiner, "KWS joiner");
  require_regular_file(assets.kws_tokens, "KWS tokens");
  require_regular_file(assets.kws_keywords, "KWS keywords");

  auto vad = std::make_unique<SherpaSileroVadAdapter>(assets.silero_vad_model);
  auto asr = std::make_unique<SherpaSenseVoiceAsrAdapter>(
    assets.sensevoice_model, assets.tokens);
  auto keyword_spotter = std::make_unique<SherpaKeywordSpotterAdapter>(assets);
  return std::make_unique<SenseVoiceProvider>(
    std::move(vad), std::move(asr), std::move(keyword_spotter), config);
}

}  // namespace voice_nav_audio
