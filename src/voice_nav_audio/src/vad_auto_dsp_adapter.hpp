// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef VOICE_NAV_AUDIO__VAD_AUTO_DSP_ADAPTER_HPP_
#define VOICE_NAV_AUDIO__VAD_AUTO_DSP_ADAPTER_HPP_

#include <memory>

#include "dsp_pipeline.hpp"

namespace voice_nav_audio
{

// Package-private production composition seam. vad_auto runs with the
// verified WebRTC APM for the continuous full-duplex input path.
[[nodiscard]] std::unique_ptr<DspAdapter> make_vad_auto_dsp_adapter();

}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__VAD_AUTO_DSP_ADAPTER_HPP_
