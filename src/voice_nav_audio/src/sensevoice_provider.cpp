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

#include "sensevoice_provider.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace voice_nav_audio
{
namespace
{

bool allowed_sensevoice_tag(const std::size_t axis, const std::string & label) noexcept
{
  static constexpr std::array<const char *, 5U> kLanguages{"zh", "en", "ja", "ko", "yue"};
  static constexpr std::array<const char *, 4U> kEmotions{"NEUTRAL", "HAPPY", "SAD", "ANGRY"};
  static constexpr std::array<const char *, 4U> kEvents{
    "Speech", "Applause", "Laughter", "Cry"};
  static constexpr std::array<const char *, 2U> kItn{"withitn", "woitn"};

  const auto * begin = kLanguages.cbegin();
  const auto * end = kLanguages.cend();
  if (axis == 1U) {
    begin = kEmotions.cbegin();
    end = kEmotions.cend();
  } else if (axis == 2U) {
    begin = kEvents.cbegin();
    end = kEvents.cend();
  } else if (axis == 3U) {
    begin = kItn.cbegin();
    end = kItn.cend();
  } else if (axis != 0U) {
    return false;
  }
  return std::any_of(begin, end, [&label](const char * const allowed) {
             return label == allowed;
    });
}

bool normalize_sensevoice_text(const std::string & labeled, std::string & normalized) noexcept
{
  normalized.clear();

  const auto trim = [&normalized]() {
      while (!normalized.empty() && (normalized.front() == ' ' || normalized.front() == '\t' ||
        normalized.front() == '\r' || normalized.front() == '\n'))
      {
        normalized.erase(normalized.begin());
      }
      while (!normalized.empty() && (normalized.back() == ' ' || normalized.back() == '\t' ||
        normalized.back() == '\r' || normalized.back() == '\n'))
      {
        normalized.pop_back();
      }
    };

  const bool has_tag_syntax = labeled.find("<|") != std::string::npos ||
    labeled.find("|>") != std::string::npos;
  if (!has_tag_syntax) {
    normalized = labeled;
    trim();
    return !normalized.empty();
  }

  // sherpa-onnx's C API removes the four SenseVoice metadata tokens before
  // returning result.text. A tagged seam value is accepted only in the exact
  // model order: language, emotion, event, then ITN.
  std::size_t index = 0U;
  for (std::size_t axis = 0U; axis < 4U; ++axis) {
    if (index + 1U >= labeled.size() || labeled[index] != '<' || labeled[index + 1U] != '|') {
      return false;
    }
    const auto end = labeled.find("|>", index + 2U);
    if (end == std::string::npos || end == index + 2U) {
      return false;
    }
    const std::string label = labeled.substr(index + 2U, end - index - 2U);
    if (!allowed_sensevoice_tag(axis, label)) {
      return false;
    }
    index = end + 2U;
  }

  if (labeled.find("<|", index) != std::string::npos ||
    labeled.find("|>", index) != std::string::npos)
  {
    return false;
  }
  normalized = labeled.substr(index);
  trim();
  return !normalized.empty();
}

}  // namespace

class SenseVoiceProvider::Implementation final
{
public:
  Implementation(
    std::unique_ptr<SileroVadAdapter> vad,
    std::unique_ptr<SenseVoiceAsrAdapter> asr,
    const SenseVoiceProviderConfig config)
  : vad_(std::move(vad)),
    asr_(std::move(asr)),
    maximum_utterance_frames_(std::min(
        config.maximum_utterance_frames == 0U ?
        SenseVoiceProviderConfig::kDefaultMaximumUtteranceFrames : config.maximum_utterance_frames,
        SenseVoiceProviderConfig::kDefaultMaximumUtteranceFrames)),
    queue_(maximum_utterance_frames_),
    utterance_samples_(maximum_utterance_frames_ * CleanedAudioFrame::kSamples)
  {
    if (!vad_ || !asr_) {
      throw std::invalid_argument("SenseVoiceProvider requires VAD and ASR adapters");
    }
    armed_ = true;
    worker_ = std::thread([this]() {worker_loop();});
  }

  ~Implementation()
  {
    shutdown();
  }

  void shutdown() noexcept
  {
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    const bool should_shutdown = !stop_requested_.exchange(true, std::memory_order_acq_rel);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
      armed_ = false;
      queue_count_ = 0U;
      overflow_pending_ = false;
      sink_ = nullptr;
      finish_requested_ = false;
      finish_handled_ = true;
      duplicate_finish_pending_ = false;
    }
    queue_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (should_shutdown) {
      // The worker is joined before either adapter is reset or destroyed.
      // This keeps reset and ASR shutdown out of process/infer and fences all
      // late provider events after input has stopped.
      vad_->reset();
      asr_->shutdown();
    }
  }

  void process_frame(const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept
  {
    bool notify_worker = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        return;
      }
      if (finish_requested_) {
        duplicate_finish_pending_ = true;
        notify_worker = true;
      } else {
        sink_ = &sink;
        if (queue_count_ == queue_.size()) {
          armed_ = false;
          quarantined_ = true;
          overflow_pending_ = true;
          overflow_frame_ = frame;
          queue_count_ = 0U;
          queue_read_ = queue_write_;
          notify_worker = true;
        } else {
          queue_[queue_write_] = frame;
          queue_write_ = (queue_write_ + 1U) % queue_.size();
          ++queue_count_;
          notify_worker = true;
        }
      }
    }
    if (notify_worker) {
      queue_condition_.notify_one();
    }
  }

  void finish_input() noexcept
  {
    bool notify_worker = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        return;
      }
      if (!armed_ || finish_requested_ || finish_handled_) {
        if (armed_ && (finish_requested_ || finish_handled_)) {
          duplicate_finish_pending_ = true;
          notify_worker = true;
        }
      } else {
        finish_requested_ = true;
        finish_handled_ = false;
        notify_worker = true;
      }
    }
    if (notify_worker) {
      queue_condition_.notify_one();
    }
  }

  void on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept
  {
    std::lock_guard<std::mutex> lock(scope_mutex_);
    active_scope_ = scope;
  }

  void on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept
  {
    std::lock_guard<std::mutex> lock(scope_mutex_);
    if (active_scope_.id == scope.id && active_scope_.turn_id == scope.turn_id) {
      active_scope_ = TurnScopeIdentity{};
    }
  }

private:
  void worker_loop() noexcept
  {
    while (true) {
      CleanedAudioFrame frame{};
      bool handle_overflow = false;
      bool handle_finish = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(
          lock, [this]() {
            return stopping_ || queue_count_ != 0U || overflow_pending_ ||
                   (finish_requested_ && !finish_handled_) || duplicate_finish_pending_;
          });
        if (stopping_ && queue_count_ == 0U && !overflow_pending_) {
          return;
        }
        if (overflow_pending_) {
          frame = overflow_frame_;
          handle_overflow = true;
        } else if (queue_count_ != 0U) {
          frame = queue_[queue_read_];
          queue_read_ = (queue_read_ + 1U) % queue_.size();
          --queue_count_;
        } else if (finish_requested_ && !finish_handled_) {
          finish_handled_ = true;
          handle_finish = armed_;
          if (!armed_) {
            finish_requested_ = false;
          }
        } else if (duplicate_finish_pending_) {
          duplicate_finish_pending_ = false;
          handle_finish = true;
        }
      }
      if (handle_overflow) {
        emit_overflow_failure(frame);
        continue;
      }
      if (handle_finish) {
        finish_on_worker();
        continue;
      }
      consume(frame);
    }
  }

  [[nodiscard]] bool is_armed() const noexcept
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return armed_ && !stopping_;
  }

  [[nodiscard]] SpeechEventSink * sink() const noexcept
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return stopping_ ? nullptr : sink_;
  }

  [[nodiscard]] TurnScopeIdentity active_scope() const noexcept
  {
    std::lock_guard<std::mutex> lock(scope_mutex_);
    return active_scope_;
  }

  void retire_turn() noexcept
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    armed_ = false;
    queue_count_ = 0U;
    queue_read_ = queue_write_;
  }

  void reset_turn(const bool preserve_pending_frames) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        return;
      }
      armed_ = true;
      if (!preserve_pending_frames) {
        queue_count_ = 0U;
        queue_read_ = 0U;
        queue_write_ = 0U;
      }
      admission_frames_ = 0U;
      utterance_sample_count_ = 0U;
      last_endpoint_sample_exclusive_ = 0U;
      wake_sent_ = false;
      quarantined_ = false;
      overflow_pending_ = false;
      failure_emitted_ = false;
      finish_requested_ = false;
      finish_handled_ = false;
      duplicate_finish_pending_ = false;
      has_last_frame_ = false;
      if (!preserve_pending_frames || queue_count_ == 0U) {
        sink_ = nullptr;
      }
    }
    // Reset is serialized on the provider worker: no VAD call can overlap
    // this reset, and the ASR instance remains alive for the next turn.
    vad_->reset();
  }

  [[nodiscard]] bool claim_overflow_frame(
    const CleanedAudioFrame & fallback,
    CleanedAudioFrame & event_frame) noexcept
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!quarantined_ || failure_emitted_ || stopping_) {
      return false;
    }
    failure_emitted_ = true;
    event_frame = overflow_pending_ ? overflow_frame_ : fallback;
    overflow_pending_ = false;
    return true;
  }

  void emit_overflow_failure(const CleanedAudioFrame & fallback) noexcept
  {
    CleanedAudioFrame event_frame{};
    if (!claim_overflow_frame(fallback, event_frame)) {
      return;
    }
    retire_turn();
    SpeechRecognitionEvent failure{};
    failure.kind = SpeechEventKind::kFailure;
    failure.audio_generation = event_frame.audio_generation;
    failure.audio_seq = event_frame.audio_seq;
    failure.scope = active_scope();
    emit(failure);
    reset_turn(true);
  }

  void emit(const SpeechRecognitionEvent & event) noexcept
  {
    auto * const event_sink = sink();
    if (event_sink != nullptr) {
      event_sink->on_speech_event(event);
    }
  }

  void emit_failure(const CleanedAudioFrame & frame) noexcept
  {
    retire_turn();
    SpeechRecognitionEvent failure{};
    failure.kind = SpeechEventKind::kFailure;
    failure.audio_generation = frame.audio_generation;
    failure.audio_seq = frame.audio_seq;
    failure.scope = active_scope();
    emit(failure);
    reset_turn(true);
  }

  void finish_endpoint(const std::size_t endpoint_sample_exclusive) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      armed_ = false;
      queue_count_ = 0U;
      queue_read_ = queue_write_;
      utterance_sample_count_ = endpoint_sample_exclusive;
    }
  }

  void infer_final(const CleanedAudioFrame & event_frame) noexcept
  {
    const auto scope = active_scope();
    std::string labeled_text{};
    const bool inferred = asr_->infer(
      utterance_samples_.data(), utterance_sample_count_, labeled_text);
    if (!inferred || scope.id == 0U) {
      SpeechRecognitionEvent failure{};
      failure.kind = SpeechEventKind::kFailure;
      failure.audio_generation = event_frame.audio_generation;
      failure.audio_seq = event_frame.audio_seq;
      failure.scope = scope;
      emit(failure);
      return;
    }
    std::string normalized{};
    if (!normalize_sensevoice_text(labeled_text, normalized)) {
      SpeechRecognitionEvent failure{};
      failure.kind = SpeechEventKind::kFailure;
      failure.audio_generation = event_frame.audio_generation;
      failure.audio_seq = event_frame.audio_seq;
      failure.scope = scope;
      emit(failure);
      return;
    }
    emit(SpeechRecognitionEvent::endpoint_final(
      event_frame, scope, std::move(normalized), 1.0F, VoiceTurnKind::kCommand));
  }

  void finish_on_worker() noexcept
  {
    if (!is_armed()) {
      return;
    }
    CleanedAudioFrame event_frame{};
    bool has_frame = false;
    bool duplicate = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      has_frame = has_last_frame_;
      if (has_frame) {
        event_frame = last_frame_;
      }
      duplicate = duplicate_finish_pending_;
      duplicate_finish_pending_ = false;
    }

    if (duplicate || !has_frame) {
      emit_failure(event_frame);
      return;
    }

    const auto flush = vad_->finish_input();
    if (!is_armed()) {
      return;
    }
    const auto accepted_samples = utterance_sample_count_;
    if (flush.status != SileroVadFlushStatus::kUnique || flush.endpoint_sample_exclusive == 0U ||
      flush.endpoint_sample_exclusive > accepted_samples ||
      flush.endpoint_sample_exclusive <= last_endpoint_sample_exclusive_)
    {
      emit_failure(event_frame);
      return;
    }

    finish_endpoint(flush.endpoint_sample_exclusive);

    if (!wake_sent_) {
      wake_sent_ = true;
      emit(SpeechRecognitionEvent::wake_accepted(event_frame));
    }
    if (stop_requested_.load(std::memory_order_acquire)) {
      return;
    }
    infer_final(event_frame);
    reset_turn(true);
  }

  void consume(const CleanedAudioFrame & frame) noexcept
  {
    if (!is_armed()) {
      return;
    }
    ++admission_frames_;
    last_frame_ = frame;
    has_last_frame_ = true;
    if (!copy_frame(frame)) {
      emit_failure(frame);
      return;
    }

    const auto vad_result = vad_->process(frame);
    if (!is_armed()) {
      return;
    }
    if (quarantined()) {
      emit_overflow_failure(frame);
      return;
    }

    if (admission_frames_ >= maximum_utterance_frames_) {
      retire_turn();
      SpeechRecognitionEvent timeout{};
      timeout.kind = SpeechEventKind::kTimeout;
      timeout.audio_generation = frame.audio_generation;
      timeout.audio_seq = frame.audio_seq;
      timeout.scope = active_scope();
      emit(timeout);
      reset_turn(true);
      return;
    }

    if (vad_result.decision != SileroVadDecision::kEndpoint &&
      vad_result.endpoint_sample_exclusive != 0U)
    {
      emit_failure(frame);
      return;
    }

    if (vad_result.decision == SileroVadDecision::kSilence) {
      return;
    }

    if (!wake_sent_) {
      wake_sent_ = true;
      emit(SpeechRecognitionEvent::wake_accepted(frame));
    }

    if (stop_requested_.load(std::memory_order_acquire)) {
      return;
    }

    const auto scope = active_scope();
    if (vad_result.decision == SileroVadDecision::kSpeech) {
      if (scope.id != 0U) {
        emit(SpeechRecognitionEvent::activity(frame, scope));
      }
      return;
    }
    if (vad_result.endpoint_sample_exclusive == 0U ||
      vad_result.endpoint_sample_exclusive > utterance_sample_count_ ||
      vad_result.endpoint_sample_exclusive <= last_endpoint_sample_exclusive_)
    {
      emit_failure(frame);
      return;
    }
    last_endpoint_sample_exclusive_ = vad_result.endpoint_sample_exclusive;
    finish_endpoint(vad_result.endpoint_sample_exclusive);
    infer_final(frame);
    reset_turn(true);
  }

  [[nodiscard]] bool quarantined() const noexcept
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return quarantined_;
  }

  [[nodiscard]] bool copy_frame(const CleanedAudioFrame & frame) noexcept
  {
    if (frame.valid_samples == 0U || frame.valid_samples > CleanedAudioFrame::kSamples ||
      utterance_sample_count_ > utterance_samples_.size() - frame.valid_samples)
    {
      return false;
    }
    const auto offset = utterance_sample_count_;
    std::copy(
      frame.samples.cbegin(), frame.samples.cbegin() +
      static_cast<std::ptrdiff_t>(frame.valid_samples), utterance_samples_.begin() +
      static_cast<std::ptrdiff_t>(offset));
    utterance_sample_count_ += frame.valid_samples;
    return true;
  }

  std::unique_ptr<SileroVadAdapter> vad_;
  std::unique_ptr<SenseVoiceAsrAdapter> asr_;
  const std::size_t maximum_utterance_frames_;
  std::vector<CleanedAudioFrame> queue_;
  std::size_t queue_read_{0U};
  std::size_t queue_write_{0U};
  std::size_t queue_count_{0U};
  mutable std::mutex queue_mutex_;
  std::mutex shutdown_mutex_;
  std::condition_variable queue_condition_;
  std::atomic<bool> stop_requested_{false};
  bool stopping_{false};
  bool armed_{false};
  SpeechEventSink * sink_{nullptr};
  std::thread worker_;
  std::vector<Sample> utterance_samples_;
  std::size_t admission_frames_{0U};
  std::size_t utterance_sample_count_{0U};
  std::size_t last_endpoint_sample_exclusive_{0U};
  bool wake_sent_{false};
  bool quarantined_{false};
  bool overflow_pending_{false};
  bool failure_emitted_{false};
  bool finish_requested_{false};
  bool finish_handled_{false};
  bool duplicate_finish_pending_{false};
  bool has_last_frame_{false};
  CleanedAudioFrame last_frame_{};
  CleanedAudioFrame overflow_frame_{};
  mutable std::mutex scope_mutex_;
  TurnScopeIdentity active_scope_{};
};

SenseVoiceProvider::SenseVoiceProvider(
  std::unique_ptr<SileroVadAdapter> vad,
  std::unique_ptr<SenseVoiceAsrAdapter> asr,
  const SenseVoiceProviderConfig config)
: implementation_(std::make_unique<Implementation>(std::move(vad), std::move(asr), config))
{
}

SenseVoiceProvider::~SenseVoiceProvider() = default;

void SenseVoiceProvider::shutdown() noexcept
{
  implementation_->shutdown();
}

void SenseVoiceProvider::finish_input() noexcept
{
  implementation_->finish_input();
}

void SenseVoiceProvider::process_frame(
  const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept
{
  implementation_->process_frame(frame, sink);
}

void SenseVoiceProvider::on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept
{
  implementation_->on_turn_scope_opened(scope);
}

void SenseVoiceProvider::on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept
{
  implementation_->on_turn_scope_retired(scope);
}

}  // namespace voice_nav_audio
