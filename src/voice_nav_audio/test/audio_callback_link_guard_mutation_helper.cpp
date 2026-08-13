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

namespace voice_nav_audio
{
namespace
{

volatile int callback_guard_seed{1};
volatile int callback_guard_observation{0};

int initialize_callback_guard() noexcept
{
  return callback_guard_seed;
}

}  // namespace

void callback_link_guard_mutation_helper() noexcept
{
  static const int callback_guard = initialize_callback_guard();
  callback_guard_observation = callback_guard;
}

}  // namespace voice_nav_audio
