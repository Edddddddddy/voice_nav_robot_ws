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

#include "voice_nav_audio/spsc_audio_ring.hpp"

#include <gtest/gtest.h>

TEST(SpscAudioRing, PreservesOrderAndBoundsCapacity)
{
  voice_nav_audio::SpscAudioRing<4U> ring;
  EXPECT_EQ(ring.write_available(), 3U);
  EXPECT_TRUE(ring.push(1.0F));
  EXPECT_TRUE(ring.push(2.0F));
  EXPECT_TRUE(ring.push(3.0F));
  EXPECT_EQ(ring.write_available(), 0U);
  EXPECT_FALSE(ring.push(4.0F));
  float sample = 0.0F;
  EXPECT_TRUE(ring.pop(sample));
  EXPECT_FLOAT_EQ(sample, 1.0F);
  EXPECT_EQ(ring.write_available(), 1U);
  EXPECT_TRUE(ring.push(4.0F));
  EXPECT_TRUE(ring.pop(sample));
  EXPECT_FLOAT_EQ(sample, 2.0F);
  ring.clear();
  EXPECT_FALSE(ring.pop(sample));
}
