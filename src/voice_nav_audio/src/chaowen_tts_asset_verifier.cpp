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

#include "chaowen_tts_asset_verifier.hpp"

#include <array>
#include <fstream>
#include <memory>
#include <string_view>
#include <system_error>

#include <openssl/evp.h>

namespace voice_nav_audio
{
namespace
{

constexpr std::size_t kSha256Bytes = 32U;
constexpr std::array<char, 16U> kHexDigits{
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

struct EvpContextDeleter
{
  void operator()(EVP_MD_CTX * context) const noexcept
  {
    EVP_MD_CTX_free(context);
  }
};

using EvpContext = std::unique_ptr<EVP_MD_CTX, EvpContextDeleter>;

bool is_locked_regular_file(const std::filesystem::path & path) noexcept
{
  if (!path.is_absolute()) {
    return false;
  }
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  return !error && status.type() == std::filesystem::file_type::regular;
}

bool sha256_matches(
  const std::filesystem::path & path,
  const char * const expected_sha256) noexcept
{
  if (expected_sha256 == nullptr) {
    return false;
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }

  EvpContext context{EVP_MD_CTX_new()};
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    return false;
  }

  std::array<unsigned char, 64U * 1024U> buffer{};
  while (true) {
    input.read(reinterpret_cast<char *>(buffer.data()),
      static_cast<std::streamsize>(buffer.size()));
    const auto bytes_read = input.gcount();
    if (bytes_read > 0 && EVP_DigestUpdate(
        context.get(), buffer.data(), static_cast<std::size_t>(bytes_read)) != 1)
    {
      return false;
    }
    if (input.eof()) {
      break;
    }
    if (!input) {
      return false;
    }
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
    digest_size != kSha256Bytes)
  {
    return false;
  }

  std::array<char, 2U * kSha256Bytes> actual_sha256{};
  for (std::size_t index = 0U; index < kSha256Bytes; ++index) {
    actual_sha256[2U * index] = kHexDigits[digest[index] >> 4U];
    actual_sha256[2U * index + 1U] = kHexDigits[digest[index] & 0x0FU];
  }
  return std::string_view(actual_sha256.data(), actual_sha256.size()) == expected_sha256;
}

}  // namespace

const ChaowenTtsAssetManifest & pinned_chaowen_tts_asset_manifest() noexcept
{
  static constexpr ChaowenTtsAssetManifest manifest{{
#define CHAOWEN_TTS_ASSET(name, size, sha256) {name, size, sha256},
#include "chaowen_tts_asset_manifest.def"
#undef CHAOWEN_TTS_ASSET
  }};
  return manifest;
}

bool verify_chaowen_tts_assets(
  const ChaowenTtsAssetPaths & paths,
  const ChaowenTtsAssetManifest & manifest) noexcept
{
  for (std::size_t index = 0U; index < paths.size(); ++index) {
    const auto & path = paths[index];
    const auto & expected = manifest[index];
    if (expected.filename == nullptr || !is_locked_regular_file(path) ||
      path.filename() != std::filesystem::path(expected.filename))
    {
      return false;
    }

    std::error_code error;
    if (std::filesystem::file_size(path, error) != expected.size || error ||
      !sha256_matches(path, expected.sha256))
    {
      return false;
    }
  }
  return true;
}

}  // namespace voice_nav_audio
