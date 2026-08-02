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

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 8) {
    return 64;
  }

  try {
    voice_nav_sim::AttachedHardwareWriteLedger ledger(
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

    std::cout << "READY" << std::endl;
    std::string command;
    if (!std::getline(std::cin, command) || command != "CHECK") {
      return 65;
    }
    if (
      ledger.claimed_writer_pid() !=
      static_cast<std::uint64_t>(getpid()))
    {
      return 66;
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
