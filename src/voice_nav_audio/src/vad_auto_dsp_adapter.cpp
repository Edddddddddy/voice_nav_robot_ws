// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "vad_auto_dsp_adapter.hpp"

#ifdef VOICE_NAV_AUDIO_WITH_WEBRTC_APM
#include "webrtc_apm_adapter.hpp"
#endif

namespace voice_nav_audio
{

std::unique_ptr<DspAdapter> make_vad_auto_dsp_adapter()
{
#ifdef VOICE_NAV_AUDIO_WITH_WEBRTC_APM
  return std::make_unique<WebRtcApmAdapter>();
#else
  return nullptr;
#endif
}

}  // namespace voice_nav_audio
