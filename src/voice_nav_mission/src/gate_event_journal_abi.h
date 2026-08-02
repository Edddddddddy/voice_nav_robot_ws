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

#ifndef GATE_EVENT_JOURNAL_ABI_H_
#define GATE_EVENT_JOURNAL_ABI_H_

#include <stdint.h>

#define VOICE_NAV_GATE_EVENT_JOURNAL_MAGIC UINT64_C(0x564e474154454a31)
#define VOICE_NAV_GATE_EVENT_JOURNAL_ABI_VERSION UINT64_C(1)
#define VOICE_NAV_GATE_EVENT_JOURNAL_HEADER_BYTES UINT64_C(128)
#define VOICE_NAV_GATE_EVENT_JOURNAL_SLOT_BYTES UINT64_C(256)

#define VOICE_NAV_GATE_EVENT_JOURNAL_INIT_EMPTY UINT64_C(0)
#define VOICE_NAV_GATE_EVENT_JOURNAL_INIT_READY UINT64_C(1)

#define VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_FREE UINT64_C(0)
#define VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_INTENT UINT64_C(1)
#define VOICE_NAV_GATE_EVENT_JOURNAL_PHASE_COMMITTED UINT64_C(2)

#define VOICE_NAV_GATE_EVENT_JOURNAL_KIND_CONTROL_TRANSITION UINT64_C(1)
#define VOICE_NAV_GATE_EVENT_JOURNAL_KIND_OUTPUT_ATTEMPT UINT64_C(2)

typedef struct voice_nav_gate_event_journal_header_v1
{
  uint64_t magic;
  uint64_t abi_version;
  uint64_t header_bytes;
  uint64_t slot_bytes;
  uint64_t region_bytes;
  uint64_t capacity;
  uint64_t owner_uid;
  uint64_t generation;
  uint64_t nonce_hi;
  uint64_t nonce_lo;
  uint64_t init_state;
  uint64_t claimed_slots;
  uint64_t overflow_latched;
  uint64_t writer_pid;
  uint64_t header_checksum;
  uint64_t reserved;
} voice_nav_gate_event_journal_header_v1;

typedef struct voice_nav_gate_event_journal_slot_v1
{
  uint64_t phase;
  uint64_t record_kind;
  uint64_t journal_seq;
  uint64_t generation;
  uint64_t intent_monotonic_ns;
  uint64_t transition_linearization_ns;
  uint64_t commit_monotonic_ns;
  uint64_t intent_checksum;
  uint64_t commit_checksum;
  uint64_t event_code;
  uint64_t reason;
  uint64_t before_state_seq;
  uint64_t after_state_seq;
  uint64_t before_control_seq;
  uint64_t after_control_seq;
  uint64_t output_attempt_seq;
  uint64_t intended_output_seq;
  uint64_t ros_stamp_sec_bits;
  uint64_t ros_stamp_nanosec;
  uint64_t linear_x_bits;
  uint64_t angular_z_bits;
  uint64_t before_lease_hi;
  uint64_t before_lease_lo;
  uint64_t after_lease_hi;
  uint64_t after_lease_lo;
  uint64_t gate_instance_hi;
  uint64_t gate_instance_lo;
  uint64_t cause_transition_journal_seq;
  uint64_t flags;
  uint64_t reserved0;
  uint64_t reserved1;
  uint64_t reserved2;
} voice_nav_gate_event_journal_slot_v1;

#endif  // GATE_EVENT_JOURNAL_ABI_H_
