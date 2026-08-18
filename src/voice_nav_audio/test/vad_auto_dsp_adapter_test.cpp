// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>

#include "gtest/gtest.h"
#include "dsp_pipeline.hpp"
#include "vad_auto_dsp_adapter.hpp"
#include "webrtc_apm_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

TEST(VadAutoDspAdapterTest, ProductionFactorySelectsWebRtcApmAndProcessesBothDirections)
{
  auto adapter = make_vad_auto_dsp_adapter();
  ASSERT_NE(adapter, nullptr);
  EXPECT_NE(dynamic_cast<WebRtcApmAdapter *>(adapter.get()), nullptr);

  DspPipeline pipeline(*adapter);
  DspInput input{};
  input.generation = 1U;
  input.sequence = 1U;
  input.delay_ms = 100.0;
  input.final_render_reference.samples.fill(1000);
  input.capture.samples.fill(2000);
  const auto result = pipeline.process(input);

  ASSERT_EQ(result.status, DspStatus::kCleaned);
  EXPECT_TRUE(std::any_of(
    result.cleaned.cbegin(), result.cleaned.cend(), [](const Sample sample) {
      return sample != 0;
    }));
}

}  // namespace
}  // namespace voice_nav_audio
