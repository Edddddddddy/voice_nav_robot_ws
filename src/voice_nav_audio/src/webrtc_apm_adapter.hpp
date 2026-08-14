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

#ifndef VOICE_NAV_AUDIO__WEBRTC_APM_ADAPTER_HPP_
#define VOICE_NAV_AUDIO__WEBRTC_APM_ADAPTER_HPP_

#include "api/scoped_refptr.h"
#include "modules/audio_processing/include/audio_processing.h"

#include "dsp_pipeline.hpp"

namespace voice_nav_audio
{

// Production Adapter for the #53-locked WebRTC APM 2.1 archive.  The CMake
// target that exposes this type only exists after provenance verification.
class WebRtcApmAdapter final : public DspAdapter
{
public:
  WebRtcApmAdapter();

  bool process_render(const DspFrame & frame) noexcept override;
  bool set_stream_delay_ms(int milliseconds) noexcept override;
  bool process_capture(DspFrame & frame) noexcept override;
  void reset() noexcept override;

private:
  void configure() noexcept;

  webrtc::StreamConfig stream_config_{48000, 1U};
  rtc::scoped_refptr<webrtc::AudioProcessing> apm_{};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__WEBRTC_APM_ADAPTER_HPP_
