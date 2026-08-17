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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "gtest/gtest.h"
#include "chaowen_tts_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

class GateSink final : public TtsSink
{
public:
  bool on_pcm(
    const std::uint64_t, const std::uint32_t sample_rate_hz, const std::uint32_t channels,
    const Sample * const samples, const std::size_t sample_count) noexcept override
  {
    valid_pcm = valid_pcm && sample_rate_hz == 22050U && channels == 1U && samples != nullptr &&
      sample_count > 0U && sample_count <= 220U;
    sample_total += sample_count;
    for (std::size_t index = 0U; index < sample_count; ++index) {
      nonzero = nonzero || samples[index] != 0;
    }
    return true;
  }

  void on_complete(const std::uint64_t) noexcept override
  {
    completed = true;
  }

  void on_failed(const std::uint64_t, const std::string &) noexcept override
  {
    failed = true;
  }

  bool valid_pcm{true};
  bool nonzero{false};
  bool completed{false};
  bool failed{false};
  std::size_t sample_total{0U};
};

TEST(ChaowenTtsRealGate, GeneratesOneSmallSentence)
{
  const auto * const root = std::getenv("VOICE_NAV_CHAOWEN_TTS_ROOT");
  ASSERT_NE(root, nullptr);
  ChaowenTtsAdapter adapter(root);
  GateSink sink;

  adapter.start(TtsRequest{1U, "这是一个安全回复。"}, sink);

  EXPECT_FALSE(sink.failed);
  EXPECT_TRUE(sink.completed);
  EXPECT_TRUE(sink.valid_pcm);
  EXPECT_TRUE(sink.nonzero);
  EXPECT_GT(sink.sample_total, 0U);
}

}  // namespace
}  // namespace voice_nav_audio
