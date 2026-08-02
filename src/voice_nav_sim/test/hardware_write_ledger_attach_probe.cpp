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

#include "attached_hardware_write_ledger.hpp"

#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

std::uint64_t parse_word(const char * text)
{
  std::size_t consumed{0U};
  const auto value = std::stoull(text, &consumed, 0);
  if (text[consumed] != '\0') {
    throw std::invalid_argument("invalid numeric probe argument");
  }
  return value;
}

int execute_command(
  voice_nav_sim::AttachedHardwareWriteLedger & ledger,
  const std::string & command,
  voice_nav_sim::HardwareWriteTicket & pending_ticket,
  bool & has_pending_ticket,
  bool & keep_running)
{
  if (command == "CHECK") {
    keep_running = false;
    return ledger.claimed_writer_pid() ==
           static_cast<std::uint64_t>(getpid()) ? 0 : 66;
  }
  if (command == "EXIT") {
    keep_running = false;
    return has_pending_ticket ? 68 : 0;
  }

  std::istringstream input{command};
  std::string operation;
  input >> operation;
  if (operation == "BEGIN") {
    std::int64_t sim_stamp_ns{0};
    std::string trailing;
    if (
      has_pending_ticket || !(input >> sim_stamp_ns) ||
      (input >> trailing))
    {
      return 69;
    }
    pending_ticket = ledger.writer().begin_write(sim_stamp_ns);
    has_pending_ticket = pending_ticket.write_seq != 0U;
    keep_running = true;
    std::cout << "BEGAN " << pending_ticket.write_seq << std::endl;
    return 0;
  }

  if (operation == "FINISH") {
    std::uint64_t delegated_result{0U};
    std::uint64_t observation_status{0U};
    std::uint64_t left_command_bits{0U};
    std::uint64_t right_command_bits{0U};
    std::string trailing;
    if (
      !has_pending_ticket ||
      !(input >> delegated_result >> observation_status >>
      left_command_bits >> right_command_bits) ||
      (input >> trailing))
    {
      return 70;
    }
    ledger.writer().finish_write(
      pending_ticket,
      delegated_result,
      voice_nav_sim::HardwareWriteWheelObservation{
        static_cast<voice_nav_sim::HardwareWriteObservationStatus>(
          observation_status),
        left_command_bits,
        right_command_bits});
    std::cout << "FINISHED " << pending_ticket.write_seq << std::endl;
    has_pending_ticket = false;
    keep_running = true;
    return 0;
  }

  std::int64_t sim_stamp_ns{0};
  std::uint64_t delegated_result{0U};
  std::uint64_t observation_status{0U};
  std::uint64_t left_command_bits{0U};
  std::uint64_t right_command_bits{0U};
  std::string trailing;
  if (
    !(input >> sim_stamp_ns >> delegated_result >>
    observation_status >> left_command_bits >> right_command_bits) ||
    operation != "WRITE" || (input >> trailing))
  {
    return 67;
  }

  auto & writer = ledger.writer();
  const auto ticket = writer.begin_write(sim_stamp_ns);
  writer.finish_write(
    ticket,
    delegated_result,
    voice_nav_sim::HardwareWriteWheelObservation{
      static_cast<voice_nav_sim::HardwareWriteObservationStatus>(
        observation_status),
      left_command_bits,
      right_command_bits});
  std::cout << "WROTE " << ticket.write_seq << std::endl;
  keep_running = false;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 8) {
    return 64;
  }

  try {
    std::unique_ptr<voice_nav_sim::AttachedHardwareWriteLedger> ledger;
    if (argc == 3) {
      ledger = std::make_unique<
        voice_nav_sim::AttachedHardwareWriteLedger>(
        voice_nav_sim::HardwareWriteLedgerDiscoveryConfig{
          argv[1], argv[2]});
    } else {
      ledger = std::make_unique<
        voice_nav_sim::AttachedHardwareWriteLedger>(
        voice_nav_sim::HardwareWriteLedgerAttachmentConfig{
          argv[1],
          voice_nav_sim::HardwareWriteLedgerIdentity{
            parse_word(argv[2]),
            parse_word(argv[3]),
            parse_word(argv[4]),
            parse_word(argv[5])},
          voice_nav_sim::HardwareWriteLedgerLayout{
            parse_word(argv[6]),
            parse_word(argv[7])}});
    }

    std::cout << "READY" << std::endl;
    voice_nav_sim::HardwareWriteTicket pending_ticket{};
    bool has_pending_ticket{false};
    std::string command;
    while (std::getline(std::cin, command)) {
      bool keep_running{false};
      const auto result = execute_command(
        *ledger,
        command,
        pending_ticket,
        has_pending_ticket,
        keep_running);
      if (result != 0 || !keep_running) {
        return result;
      }
    }
    return 65;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
