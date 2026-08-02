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

#include "gate_event_journal_abi.h"  // NOLINT(build/include_subdir)

#include <stddef.h>
#include <stdint.h>

_Static_assert(
  sizeof(voice_nav_gate_event_journal_header_v1) == 128U,
  "Gate journal header ABI must remain 128 bytes");
_Static_assert(
  _Alignof(voice_nav_gate_event_journal_header_v1) == _Alignof(uint64_t),
  "Gate journal header ABI must retain uint64_t alignment");
_Static_assert(
  offsetof(voice_nav_gate_event_journal_header_v1, header_checksum) == 112U,
  "Gate journal header checksum offset changed");
_Static_assert(
  sizeof(voice_nav_gate_event_journal_slot_v1) == 256U,
  "Gate journal slot ABI must remain 256 bytes");
_Static_assert(
  _Alignof(voice_nav_gate_event_journal_slot_v1) == _Alignof(uint64_t),
  "Gate journal slot ABI must retain uint64_t alignment");
_Static_assert(
  offsetof(voice_nav_gate_event_journal_slot_v1, phase) == 0U,
  "Gate journal phase must remain the first slot field");
_Static_assert(
  offsetof(voice_nav_gate_event_journal_slot_v1, reserved2) == 248U,
  "Gate journal slot tail offset changed");

int main(void)
{
  if (
    VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC != UINT64_C(0x564e474154454a31) ||
    VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION != UINT64_C(1) ||
    VOICE_NAV_GATE_EVENT_JOURNAL_HEADER_BYTES != UINT64_C(128) ||
    VOICE_NAV_GATE_EVENT_JOURNAL_SLOT_BYTES != UINT64_C(256))
  {
    return 1;
  }
  return 0;
}
