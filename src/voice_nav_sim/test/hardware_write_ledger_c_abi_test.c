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

#include "hardware_write_ledger_abi.h"  // NOLINT(build/include_subdir)

#include <stddef.h>
#include <stdint.h>

_Static_assert(
  sizeof(voice_nav_hardware_write_ledger_header_v1) == 192U,
  "Hardware-write ledger header ABI must remain 192 bytes");
_Static_assert(
  _Alignof(voice_nav_hardware_write_ledger_header_v1) == _Alignof(uint64_t),
  "Hardware-write ledger header must retain uint64_t alignment");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_header_v1,
    feature_flags) == 128U,
  "Hardware-write ledger feature offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_header_v1,
    header_checksum) == 184U,
  "Hardware-write ledger header checksum offset changed");

_Static_assert(
  sizeof(voice_nav_hardware_write_ledger_control_v1) == 192U,
  "Hardware-write ledger control ABI must remain 192 bytes");
_Static_assert(
  _Alignof(voice_nav_hardware_write_ledger_control_v1) == _Alignof(uint64_t),
  "Hardware-write ledger control must retain uint64_t alignment");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_control_v1,
    request_state) == 80U,
  "Hardware-write ledger request state offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_control_v1,
    response_code) == 88U,
  "Hardware-write ledger response offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_control_v1,
    response_request_checksum) == 120U,
  "Hardware-write ledger consumed-request checksum offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_control_v1,
    response_ticket) == 136U,
  "Hardware-write ledger response publication offset changed");

_Static_assert(
  sizeof(voice_nav_hardware_write_ledger_bank_v1) == 128U,
  "Hardware-write ledger bank ABI must remain 128 bytes");
_Static_assert(
  _Alignof(voice_nav_hardware_write_ledger_bank_v1) == _Alignof(uint64_t),
  "Hardware-write ledger bank must retain uint64_t alignment");
_Static_assert(
  offsetof(voice_nav_hardware_write_ledger_bank_v1, state) == 0U,
  "Hardware-write ledger bank state must remain first");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_bank_v1,
    bank_checksum) == 120U,
  "Hardware-write ledger bank checksum offset changed");

_Static_assert(
  sizeof(voice_nav_hardware_write_ledger_segment_v1) == 64U,
  "Hardware-write ledger segment ABI must remain 64 bytes");
_Static_assert(
  _Alignof(voice_nav_hardware_write_ledger_segment_v1) == _Alignof(uint64_t),
  "Hardware-write ledger segment must retain uint64_t alignment");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_segment_v1,
    observation_and_result) == 40U,
  "Hardware-write ledger outcome offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_segment_v1,
    right_command_bits) == 56U,
  "Hardware-write ledger segment tail offset changed");

_Static_assert(
  sizeof(voice_nav_hardware_write_ledger_page_v1) == 192U,
  "Hardware-write ledger page ABI must remain 192 bytes");
_Static_assert(
  _Alignof(voice_nav_hardware_write_ledger_page_v1) == _Alignof(uint64_t),
  "Hardware-write ledger page must retain uint64_t alignment");
_Static_assert(
  offsetof(voice_nav_hardware_write_ledger_page_v1, page_index) == 96U,
  "Hardware-write ledger page index offset changed");
_Static_assert(
  offsetof(
    voice_nav_hardware_write_ledger_page_v1,
    page_checksum) == 184U,
  "Hardware-write ledger page checksum offset changed");

int main(void)
{
  if (
    VOICE_NAV_HARDWARE_WRITE_LEDGER_MAGIC !=
    UINT64_C(0x564e48574c444731) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_MAGIC !=
    UINT64_C(0x564e485750414731) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_VERSION != UINT64_C(1) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_ENDIAN_TAG !=
    UINT64_C(0x0102030405060708) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES != UINT64_C(192) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES != UINT64_C(192) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES != UINT64_C(128) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES != UINT64_C(64) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_BYTES != UINT64_C(192) ||
    VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT != UINT64_C(2))
  {
    return 1;
  }
  return 0;
}
