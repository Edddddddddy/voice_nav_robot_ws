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

#include "speech_output_core.hpp"

namespace voice_nav_audio
{

// This auditable seam deliberately does not provision, load, or execute the
// Chaowen model.  Release infrastructure must first verify its locked asset.
class ChaowenTtsAdapter final : public TtsAdapter
{
public:
  explicit ChaowenTtsAdapter(bool asset_gate_passed) noexcept;

  void start(const TtsRequest & request, TtsSink & sink) noexcept override;
  void cancel(std::uint64_t scope_id) noexcept override;

private:
  bool asset_gate_passed_{false};
};

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__CHAOWEN_TTS_ADAPTER_HPP_
