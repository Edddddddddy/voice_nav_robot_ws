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

namespace voice_nav_audio
{

#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_MUTATION
void callback_link_mutation_helper() noexcept;
#endif

#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_GUARD_MUTATION
void callback_link_guard_mutation_helper() noexcept;
#endif

namespace
{

constexpr unsigned long kPortAudioInputUnderflow = 0x00000001U;
constexpr unsigned long kPortAudioInputOverflow = 0x00000002U;
constexpr unsigned long kPortAudioOutputUnderflow = 0x00000004U;
constexpr unsigned long kPortAudioOutputOverflow = 0x00000008U;
constexpr int kPortAudioContinue = 0;
constexpr int kPortAudioAbort = 2;

}  // namespace

int native_portaudio_callback(
  const void * const input,
  void * const output,
  const unsigned long frame_count,
  const PaStreamCallbackTimeInfo * const,
  const unsigned long status_flags,
  void * const user_data) noexcept
{
#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_MUTATION
  callback_link_mutation_helper();
#endif
#ifdef VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_GUARD_MUTATION
  callback_link_guard_mutation_helper();
#endif
  auto * const callback_context = static_cast<NativePortAudioCallbackContext *>(user_data);
  if (callback_context == nullptr || callback_context->callback == nullptr) {
    return kPortAudioAbort;
  }
  const CallbackStatus status{
    (status_flags & (kPortAudioInputUnderflow | kPortAudioInputOverflow)) != 0U,
    (status_flags & (kPortAudioOutputUnderflow | kPortAudioOutputOverflow)) != 0U};
  callback_context->callback(
    callback_context->context, static_cast<const Sample *>(input), static_cast<Sample *>(output),
    static_cast<std::size_t>(frame_count), status);
  return kPortAudioContinue;
}

void PortAudioAdapter::callback(
  void * const context,
  const Sample * const capture,
  Sample * const device_output,
  const std::size_t frame_count,
  const CallbackStatus status) noexcept
{
  if (context == nullptr) {
    return;
  }
  static_cast<AudioEngine *>(context)->process_callback(
    capture, device_output, frame_count, status);
}

}  // namespace voice_nav_audio
