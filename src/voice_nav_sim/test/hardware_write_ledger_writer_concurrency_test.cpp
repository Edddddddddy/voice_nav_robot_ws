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

#include "hardware_write_ledger_abi.h"
#include "hardware_write_ledger_writer.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

namespace
{

constexpr std::size_t kSegmentCapacity{1U};
constexpr std::size_t kRegionBytes =
  VOICE_NAV_HARDWARE_WRITE_LEDGER_HEADER_BYTES +
  VOICE_NAV_HARDWARE_WRITE_LEDGER_CONTROL_BYTES +
  VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_COUNT *
  (VOICE_NAV_HARDWARE_WRITE_LEDGER_BANK_BYTES +
  kSegmentCapacity * VOICE_NAV_HARDWARE_WRITE_LEDGER_SEGMENT_BYTES);

bool run_one_concurrent_admission()
{
  alignas(64) std::array<std::uint64_t, kRegionBytes / sizeof(std::uint64_t)>
  region{};
  auto * header =
    reinterpret_cast<voice_nav_hardware_write_ledger_header_v1 *>(
    region.data());
  header->region_bytes = kRegionBytes;
  header->segment_capacity_per_bank = kSegmentCapacity;
  header->generation = 81U;

  voice_nav_sim::HardwareWriteLedgerWriter writer{
    region.data(), kRegionBytes};
  std::array<voice_nav_sim::HardwareWriteTicket, 2U> tickets{};
  std::atomic<std::uint64_t> ready{0U};
  std::atomic<bool> start{false};
  std::array<std::thread, 2U> callers{
    std::thread{[&writer, &tickets, &ready, &start]() {
        ready.fetch_add(1U, std::memory_order_release);
        while (!start.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        tickets[0U] = writer.begin_write(100);
      }},
    std::thread{[&writer, &tickets, &ready, &start]() {
        ready.fetch_add(1U, std::memory_order_release);
        while (!start.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        tickets[1U] = writer.begin_write(100);
      }}};
  while (ready.load(std::memory_order_acquire) != callers.size()) {
    std::this_thread::yield();
  }
  start.store(true, std::memory_order_release);
  for (auto & caller : callers) {
    caller.join();
  }

  const auto valid_ticket_count =
    static_cast<std::uint64_t>(tickets[0U].write_seq != 0U) +
    static_cast<std::uint64_t>(tickets[1U].write_seq != 0U);
  if (valid_ticket_count != 1U) {
    std::cerr << "concurrent begin issued " << valid_ticket_count <<
      " valid tickets\n";
    return false;
  }
  const auto & valid_ticket =
    tickets[tickets[0U].write_seq != 0U ? 0U : 1U];
  writer.finish_write(
    valid_ticket,
    0U,
    voice_nav_sim::HardwareWriteWheelObservation{
      voice_nav_sim::HardwareWriteObservationStatus::kValid, 0U, 0U});
  if (
    __atomic_load_n(&header->last_completed_write_seq, __ATOMIC_ACQUIRE) != 1U ||
    (__atomic_load_n(&header->global_oracle_faults, __ATOMIC_ACQUIRE) &
    VOICE_NAV_HARDWARE_WRITE_LEDGER_FAULT_PROTOCOL) == 0U)
  {
    std::cerr << "concurrent begin did not fail fast and preserve sequence one\n";
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  for (std::uint64_t attempt = 0U; attempt < 32U; ++attempt) {
    if (!run_one_concurrent_admission()) {
      return 1;
    }
  }
  return 0;
}
