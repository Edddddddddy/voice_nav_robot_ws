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

#ifndef VOICE_NAV_AUDIO__AUDIO_ENGINE_CALLBACK_TEST_SUPPORT_HPP_
#define VOICE_NAV_AUDIO__AUDIO_ENGINE_CALLBACK_TEST_SUPPORT_HPP_

namespace voice_nav_audio
{
namespace test_support
{

using CallbackBoundaryHook = void (*)() noexcept;

void set_callback_boundary_hook(CallbackBoundaryHook hook) noexcept;
void invoke_callback_boundary_hook() noexcept;

}  // namespace test_support
}  // namespace voice_nav_audio

#endif  // VOICE_NAV_AUDIO__AUDIO_ENGINE_CALLBACK_TEST_SUPPORT_HPP_
