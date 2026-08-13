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

#include "audio_engine_callback_test_support.hpp"

#include <atomic>

namespace voice_nav_audio
{
namespace test_support
{
namespace
{

std::atomic<CallbackBoundaryHook> callback_boundary_hook{nullptr};

}  // namespace

void set_callback_boundary_hook(const CallbackBoundaryHook hook) noexcept
{
  callback_boundary_hook.store(hook, std::memory_order_release);
}

void invoke_callback_boundary_hook() noexcept
{
  if (const auto hook = callback_boundary_hook.load(std::memory_order_acquire)) {
    hook();
  }
}

}  // namespace test_support
}  // namespace voice_nav_audio
