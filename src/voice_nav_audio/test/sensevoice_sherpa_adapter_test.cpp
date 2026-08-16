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

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "sensevoice_sherpa_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

const char * required_environment(const char * const name)
{
  const auto * const value = std::getenv(name);
  return value == nullptr ? "" : value;
}

TEST(SenseVoiceSherpaAdapterTest, UsesOnlyThreeExplicitResolvedAssetPaths)
{
  const SherpaSenseVoiceAssetPaths assets{
    required_environment("VOICE_NAV_SENSEVOICE_VAD_MODEL"),
    required_environment("VOICE_NAV_SENSEVOICE_MODEL"),
    required_environment("VOICE_NAV_SENSEVOICE_TOKENS")};
  ASSERT_FALSE(assets.silero_vad_model.empty())
    << "the real adapter contract requires the resolved Silero VAD path";
  ASSERT_FALSE(assets.sensevoice_model.empty())
    << "the real adapter contract requires the resolved SenseVoice model path";
  ASSERT_FALSE(assets.tokens.empty())
    << "the real adapter contract requires the resolved SenseVoice tokens path";

  auto provider = make_sherpa_sensevoice_provider(assets);
  ASSERT_NE(provider, nullptr);
  EXPECT_TRUE(provider->arm_once());
  EXPECT_FALSE(provider->arm_once());
}

TEST(SenseVoiceSherpaAdapterTest, RejectsAnIncompleteResolvedAssetSetBeforeLoading)
{
  EXPECT_THROW(
    make_sherpa_sensevoice_provider(SherpaSenseVoiceAssetPaths{}),
    std::invalid_argument);
}

}  // namespace
}  // namespace voice_nav_audio
