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

#ifndef VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_H_
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_H_

#include <stdint.h>

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_MAGIC \
  UINT64_C(0x564e48574c444731)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_MAGIC \
  UINT64_C(0x564e485750414731)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_VERSION UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_ENDIAN_TAG \
  UINT64_C(0x0102030405060708)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES UINT64_C(192)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES UINT64_C(192)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES UINT64_C(128)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES UINT64_C(64)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_PAGE_BYTES UINT64_C(192)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT UINT64_C(2)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_INIT_EMPTY UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_INIT_READY UINT64_C(1)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_FREE UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_ACTIVE UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_SEALED_OK UINT64_C(2)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_SEALED_FAULT UINT64_C(3)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_NONE UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_ARM UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_SEAL UINT64_C(2)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_IDLE UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_WRITING UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READY UINT64_C(2)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_REQUEST_READING UINT64_C(3)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_NONE UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_OK UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_INVALID UINT64_C(2)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_BUSY UINT64_C(3)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_RESPONSE_NO_FREE_BANK UINT64_C(4)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_ZERO_REQUIRED UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FLAG_EXACT_SEAL_STAMP UINT64_C(2)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_OBSERVATION_VALID UINT64_C(0)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_OBSERVATION_MISSING_ENTITY UINT64_C(1)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_OBSERVATION_MISSING_COMPONENT \
  UINT64_C(2)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_OBSERVATION_EMPTY_COMPONENT \
  UINT64_C(3)

#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SEQUENCE (UINT64_C(1) << 0U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_GENERATION (UINT64_C(1) << 1U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_NONFINITE (UINT64_C(1) << 2U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_SIM_STAMP (UINT64_C(1) << 3U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_CAPACITY (UINT64_C(1) << 4U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_ZERO_REQUIRED \
  (UINT64_C(1) << 5U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_OBSERVATION \
  (UINT64_C(1) << 6U)
#define VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL (UINT64_C(1) << 7U)

typedef struct voice_nav_hardware_write_ledger_header_v1
{
  uint64_t magic;
  uint64_t abi_version;
  uint64_t endian_tag;
  uint64_t header_bytes;
  uint64_t control_bytes;
  uint64_t bank_bytes;
  uint64_t segment_bytes;
  uint64_t page_bytes;
  uint64_t region_bytes;
  uint64_t bank_count;
  uint64_t segment_capacity_per_bank;
  uint64_t page_segment_limit;
  uint64_t owner_uid;
  uint64_t generation;
  uint64_t nonce_hi;
  uint64_t nonce_lo;
  uint64_t feature_flags;
  uint64_t init_state;
  uint64_t writer_pid;
  uint64_t last_completed_write_seq;
  uint64_t global_oracle_faults;
  uint64_t reserved0;
  uint64_t reserved1;
  uint64_t header_checksum;
} voice_nav_hardware_write_ledger_header_v1;

typedef struct voice_nav_hardware_write_ledger_control_v1
{
  uint64_t request_op;
  uint64_t request_flags;
  uint64_t request_interval_id;
  uint64_t request_bank_index;
  uint64_t request_bank_epoch;
  uint64_t request_segment_budget;
  uint64_t request_invocation_budget;
  uint64_t request_not_before_sim_stamp_ns_bits;
  uint64_t request_checksum;
  uint64_t request_ticket;
  uint64_t request_state;
  uint64_t response_code;
  uint64_t response_bank_index;
  uint64_t response_bank_epoch;
  uint64_t response_fence_write_seq;
  uint64_t response_request_checksum;
  uint64_t response_checksum;
  uint64_t response_ticket;
  uint64_t reserved0;
  uint64_t reserved1;
  uint64_t reserved2;
  uint64_t reserved3;
  uint64_t reserved4;
  uint64_t reserved5;
} voice_nav_hardware_write_ledger_control_v1;

typedef struct voice_nav_hardware_write_ledger_bank_v1
{
  uint64_t state;
  uint64_t bank_epoch;
  uint64_t interval_id;
  uint64_t arm_fence_write_seq;
  uint64_t seal_fence_write_seq;
  uint64_t segment_budget;
  uint64_t invocation_budget;
  uint64_t predicate_flags;
  uint64_t seal_not_before_sim_stamp_ns_bits;
  uint64_t segment_count;
  uint64_t invocation_count;
  uint64_t first_write_seq;
  uint64_t last_write_seq;
  uint64_t oracle_faults;
  uint64_t page_count;
  uint64_t bank_checksum;
} voice_nav_hardware_write_ledger_bank_v1;

typedef struct voice_nav_hardware_write_ledger_segment_v1
{
  uint64_t generation;
  uint64_t first_write_seq;
  uint64_t last_write_seq;
  uint64_t invocation_count;
  uint64_t sim_stamp_ns_bits;
  uint64_t observation_and_result;
  uint64_t left_command_bits;
  uint64_t right_command_bits;
} voice_nav_hardware_write_ledger_segment_v1;

typedef struct voice_nav_hardware_write_ledger_page_v1
{
  uint64_t page_magic;
  uint64_t abi_version;
  uint64_t page_bytes;
  uint64_t segment_bytes;
  uint64_t bank_index;
  uint64_t bank_epoch;
  uint64_t generation;
  uint64_t interval_id;
  uint64_t arm_fence_write_seq;
  uint64_t seal_fence_write_seq;
  uint64_t seal_not_before_sim_stamp_ns_bits;
  uint64_t predicate_flags;
  uint64_t page_index;
  uint64_t page_count;
  uint64_t total_segment_count;
  uint64_t total_invocation_count;
  uint64_t page_segment_count;
  uint64_t page_invocation_count;
  uint64_t page_first_write_seq;
  uint64_t page_last_write_seq;
  uint64_t previous_page_checksum;
  uint64_t oracle_faults;
  uint64_t bank_checksum;
  uint64_t page_checksum;
} voice_nav_hardware_write_ledger_page_v1;

#endif  // VOICE_NAV_HARDWARE_WRITE_LEDGER_ABI_H_
