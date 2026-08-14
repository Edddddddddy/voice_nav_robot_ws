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

#include "webrtc_apm_adapter.hpp"

namespace voice_nav_audio
{

WebRtcApmAdapter::WebRtcApmAdapter()
{
  reset();
}

bool WebRtcApmAdapter::process_render(const DspFrame & frame) noexcept
{
  if (!apm_) {
    return false;
  }
  DspFrame render = frame;
  return apm_->ProcessReverseStream(
    render.samples.data(), stream_config_, stream_config_, render.samples.data()) ==
         webrtc::AudioProcessing::kNoError;
}

bool WebRtcApmAdapter::set_stream_delay_ms(const int milliseconds) noexcept
{
  return apm_ != nullptr &&
         apm_->set_stream_delay_ms(milliseconds) == webrtc::AudioProcessing::kNoError;
}

bool WebRtcApmAdapter::process_capture(DspFrame & frame) noexcept
{
  return apm_ != nullptr &&
         apm_->ProcessStream(
    frame.samples.data(), stream_config_, stream_config_, frame.samples.data()) ==
         webrtc::AudioProcessing::kNoError;
}

void WebRtcApmAdapter::reset() noexcept
{
  apm_ = webrtc::AudioProcessingBuilder().Create();
  configure();
}

void WebRtcApmAdapter::configure() noexcept
{
  if (!apm_) {
    return;
  }
  webrtc::AudioProcessing::Config configuration{};
  configuration.echo_canceller.enabled = true;
  configuration.echo_canceller.mobile_mode = false;
  configuration.gain_controller1.enabled = false;
  configuration.gain_controller2.enabled = false;
  configuration.high_pass_filter.enabled = false;
  configuration.noise_suppression.enabled = false;
  apm_->ApplyConfig(configuration);
}

}  // namespace voice_nav_audio
