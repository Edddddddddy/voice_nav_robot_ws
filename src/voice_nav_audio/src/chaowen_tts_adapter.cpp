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

namespace voice_nav_audio
{

ChaowenTtsAdapter::ChaowenTtsAdapter(const bool asset_gate_passed) noexcept
: asset_gate_passed_(asset_gate_passed)
{
}

void ChaowenTtsAdapter::start(const TtsRequest & request, TtsSink & sink) noexcept
{
  sink.on_failed(
    request.scope_id,
    asset_gate_passed_ ? "Chaowen runtime is a Release Gate and is not enabled in this build" :
    "Chaowen asset gate rejected the TTS provider");
}

void ChaowenTtsAdapter::cancel(const std::uint64_t) noexcept
{
}

}  // namespace voice_nav_audio
