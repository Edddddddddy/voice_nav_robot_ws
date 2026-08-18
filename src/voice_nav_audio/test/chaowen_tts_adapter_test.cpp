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

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "chaowen_tts_adapter.hpp"

namespace voice_nav_audio
{
namespace
{

class RecordingSink final : public TtsSink
{
public:
  bool on_pcm(
    const std::uint64_t scope_id, const std::uint32_t sample_rate_hz,
    const std::uint32_t channels, const Sample * const samples,
    const std::size_t sample_count) noexcept override
  {
    scopes.push_back(scope_id);
    sample_rates.push_back(sample_rate_hz);
    channel_counts.push_back(channels);
    chunks.emplace_back(samples, samples + sample_count);
    return accept_pcm;
  }

  void on_complete(const std::uint64_t scope_id) noexcept override
  {
    completed_scopes.push_back(scope_id);
  }

  void on_failed(const std::uint64_t scope_id, const std::string & detail) noexcept override
  {
    failed_scopes.push_back(scope_id);
    failure_details.push_back(detail);
  }

  bool accept_pcm{true};
  std::vector<std::uint64_t> scopes;
  std::vector<std::uint32_t> sample_rates;
  std::vector<std::uint32_t> channel_counts;
  std::vector<std::vector<Sample>> chunks;
  std::vector<std::uint64_t> completed_scopes;
  std::vector<std::uint64_t> failed_scopes;
  std::vector<std::string> failure_details;
};

class CoreObserver final : public SpeechOutputObserver
{
public:
  void on_played(const std::uint64_t scope_id, const std::uint64_t samples) noexcept override
  {
    played_scope_ids.push_back(scope_id);
    played_samples.push_back(samples);
  }

  void on_result(const SpeechResult & result) noexcept override
  {
    results.push_back(result);
  }

  std::vector<std::uint64_t> played_scope_ids;
  std::vector<std::uint64_t> played_samples;
  std::vector<SpeechResult> results;
};

class FakeInference final : public ChaowenTtsInference
{
public:
  bool generate(
    const std::string &, std::vector<float> & samples,
    std::uint32_t & sample_rate_hz, std::string & detail) noexcept override
  {
    ++calls;
    if (!succeeds) {
      detail = failure_detail;
      return false;
    }
    if (xrun_engine != nullptr) {
      std::array<Sample, AudioEngine::kFrameSamples> output{};
      xrun_engine->process_callback(
        nullptr, output.data(), output.size(), CallbackStatus{false, true});
    }
    samples = generated_samples;
    sample_rate_hz = generated_sample_rate_hz;
    return true;
  }

  bool succeeds{true};
  std::string failure_detail{"fake inference failed"};
  std::vector<float> generated_samples{};
  std::uint32_t generated_sample_rate_hz{22050U};
  AudioEngine * xrun_engine{nullptr};
  std::size_t calls{0U};
};

class ChaowenTtsAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() / "voice_nav_chaowen_adapter_test";
    std::filesystem::remove_all(root_);
    ASSERT_TRUE(std::filesystem::create_directory(root_));
    paths_ = ChaowenTtsModelPaths{
      root_ / "model.onnx", root_ / "lexicon.txt", root_ / "tokens.txt",
      root_ / "phone.fst", root_ / "date.fst", root_ / "number.fst"};
    for (const auto & path : {paths_.model, paths_.lexicon, paths_.tokens, paths_.phone_fst,
        paths_.date_fst, paths_.number_fst})
    {
      std::ofstream file(path, std::ios::binary);
      ASSERT_TRUE(file.good());
      file << "fixture";
    }
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  std::filesystem::path root_;
  ChaowenTtsModelPaths paths_{};
};

constexpr char kFixtureSha256[] =
  "f16d05ec6b29248d2c61adb1e9263f78e4f7bace1b955014a2d17872cfe4064d";

ChaowenTtsAssetManifest fixture_manifest()
{
  return {{
      {"model.onnx", 7U, kFixtureSha256},
      {"lexicon.txt", 7U, kFixtureSha256},
      {"tokens.txt", 7U, kFixtureSha256},
      {"phone.fst", 7U, kFixtureSha256},
      {"date.fst", 7U, kFixtureSha256},
      {"number.fst", 7U, kFixtureSha256},
    }};
}

TEST_F(ChaowenTtsAdapterTest, EmitsBoundedMonoPcmAndCompletes)
{
  auto inference = std::make_unique<FakeInference>();
  inference->generated_samples.assign(441U, 0.5F);
  auto * inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{17U, "安全回复"}, sink);

  ASSERT_EQ(inference_ptr->calls, 1U);
  ASSERT_TRUE(sink.failed_scopes.empty());
  ASSERT_EQ(sink.completed_scopes, std::vector<std::uint64_t>({17U}));
  ASSERT_EQ(sink.scopes.size(), 3U);
  EXPECT_EQ(sink.sample_rates, std::vector<std::uint32_t>({22050U, 22050U, 22050U}));
  EXPECT_EQ(sink.channel_counts, std::vector<std::uint32_t>({1U, 1U, 1U}));
  EXPECT_LE(sink.chunks[0].size(), 220U);
  EXPECT_LE(sink.chunks[1].size(), 220U);
  EXPECT_LE(sink.chunks[2].size(), 220U);
  EXPECT_EQ(sink.chunks[0].size() + sink.chunks[1].size() + sink.chunks[2].size(), 441U);
  EXPECT_EQ(sink.chunks[0][0], 16384);
}

TEST_F(ChaowenTtsAdapterTest, FastGeneratedBurstReachesAudioEngineAndCompletes)
{
  constexpr std::size_t packet_count = 1100U;
  auto inference = std::make_unique<FakeInference>();
  inference->generated_samples.assign(packet_count * 220U, 0.5F);
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  AudioEngine engine;
  CoreObserver observer;
  SpeechOutputCore core(engine, adapter, observer);

  const SpeechGoal goal{
    "voice-instance", 7U, "session", "turn", SpeechPriority::Normal, "安全回复", false};
  const auto admission = core.start(goal);
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  bool saw_nonzero_output = false;
  for (std::size_t packet = 0U; packet < packet_count; ++packet) {
    engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
    saw_nonzero_output = saw_nonzero_output || std::any_of(
      output.begin(), output.end(), [](const Sample sample) {return sample != 0;});
    AudioFrame capture{};
    AudioFrame reference{};
    (void)engine.try_pop_capture(capture);
    (void)engine.try_pop_reference(reference);
    (void)core.advance();
  }

  EXPECT_TRUE(saw_nonzero_output);
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
  EXPECT_EQ(observer.results.front().played_samples, packet_count * 478U);
}

TEST_F(ChaowenTtsAdapterTest, RecoversAfterFullDuplexXrunDuringInference)
{
  constexpr std::size_t generated_sample_count = 30522U;
  constexpr std::size_t packet_count = (generated_sample_count + 219U) / 220U;
  AudioEngine engine;
  auto inference = std::make_unique<FakeInference>();
  inference->generated_samples.assign(generated_sample_count, 0.5F);
  inference->xrun_engine = &engine;
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  CoreObserver observer;
  SpeechOutputCore core(engine, adapter, observer);

  const SpeechGoal goal{
    "voice-instance", 7U, "session", "turn", SpeechPriority::Normal, "任务已完成。", false};
  const auto admission = core.start(goal);
  ASSERT_TRUE(admission.start_synthesis);
  ASSERT_TRUE(core.begin_synthesis(admission.scope_id));

  std::array<Sample, AudioEngine::kFrameSamples> output{};
  bool saw_nonzero_output = false;
  for (std::size_t packet = 0U; packet < packet_count; ++packet) {
    engine.process_callback(nullptr, output.data(), output.size(), CallbackStatus{});
    saw_nonzero_output = saw_nonzero_output || std::any_of(
      output.begin(), output.end(), [](const Sample sample) {return sample != 0;});
    AudioFrame capture{};
    AudioFrame reference{};
    (void)engine.try_pop_capture(capture);
    (void)engine.try_pop_reference(reference);
    (void)core.advance();
  }

  EXPECT_TRUE(saw_nonzero_output);
  ASSERT_EQ(observer.results.size(), 1U);
  EXPECT_EQ(observer.results.front().code, SpeechResultCode::Completed);
  EXPECT_GT(observer.results.front().played_samples, 0U);
}

TEST_F(ChaowenTtsAdapterTest, RejectsSameSizeTamperedAssetBeforeInference)
{
  std::ofstream tampered(paths_.model, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(tampered.good());
  tampered << "tampred";
  tampered.close();

  auto inference = std::make_unique<FakeInference>();
  auto * inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{18U, "安全回复"}, sink);

  EXPECT_EQ(inference_ptr->calls, 0U);
  ASSERT_EQ(sink.failed_scopes, std::vector<std::uint64_t>({18U}));
  EXPECT_NE(sink.failure_details.front().find("asset"), std::string::npos);
  EXPECT_TRUE(sink.completed_scopes.empty());
}

TEST_F(ChaowenTtsAdapterTest, RejectsWrongSizedAssetBeforeInference)
{
  std::ofstream truncated(paths_.model, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(truncated.good());
  truncated << "short";
  truncated.close();

  auto inference = std::make_unique<FakeInference>();
  auto * inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{20U, "安全回复"}, sink);

  EXPECT_EQ(inference_ptr->calls, 0U);
  ASSERT_EQ(sink.failed_scopes, std::vector<std::uint64_t>({20U}));
  EXPECT_NE(sink.failure_details.front().find("asset"), std::string::npos);
  EXPECT_TRUE(sink.completed_scopes.empty());
}

TEST_F(ChaowenTtsAdapterTest, ProductionConstructorRejectsWrongSizedAsset)
{
  const auto production_root = root_ / "production";
  ASSERT_TRUE(std::filesystem::create_directory(production_root));
  for (const auto & filename : {
      "zh_CN-chaowen-medium.onnx", "lexicon.txt", "tokens.txt", "phone.fst", "date.fst",
      "number.fst"})
  {
    std::ofstream file(production_root / filename, std::ios::binary);
    ASSERT_TRUE(file.good());
    file << "fixture";
  }
  std::ofstream wrong_size(
    production_root / "zh_CN-chaowen-medium.onnx", std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(wrong_size.good());
  wrong_size << "short";

  ChaowenTtsAdapter adapter(production_root);
  RecordingSink sink;

  adapter.start(TtsRequest{21U, "安全回复"}, sink);

  ASSERT_EQ(sink.failed_scopes, std::vector<std::uint64_t>({21U}));
  EXPECT_NE(sink.failure_details.front().find("asset"), std::string::npos);
  EXPECT_TRUE(sink.completed_scopes.empty());
}

TEST_F(ChaowenTtsAdapterTest, RealRootPassesPinnedManifestBeforeFakeInference)
{
  const char * const root_value = std::getenv("VOICE_NAV_CHAOWEN_TTS_ROOT");
  if (root_value == nullptr || *root_value == '\0') {
    GTEST_SKIP() << "VOICE_NAV_CHAOWEN_TTS_ROOT is not set";
  }

  const std::filesystem::path production_root(root_value);
  const ChaowenTtsModelPaths production_paths{
    production_root / "zh_CN-chaowen-medium.onnx", production_root / "lexicon.txt",
    production_root / "tokens.txt", production_root / "phone.fst",
    production_root / "date.fst", production_root / "number.fst"};
  auto inference = std::make_unique<FakeInference>();
  inference->generated_samples.assign(32U, 0.25F);
  auto * const inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(
    production_paths, std::move(inference), pinned_chaowen_tts_asset_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{22U, "真实模型校验"}, sink);

  ASSERT_EQ(inference_ptr->calls, 1U);
  EXPECT_TRUE(sink.failed_scopes.empty());
  EXPECT_EQ(sink.completed_scopes, std::vector<std::uint64_t>({22U}));
}

TEST_F(ChaowenTtsAdapterTest, RejectsMissingModelPathBeforeInference)
{
  std::filesystem::remove(paths_.tokens);
  auto inference = std::make_unique<FakeInference>();
  auto * inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{18U, "安全回复"}, sink);

  EXPECT_EQ(inference_ptr->calls, 0U);
  ASSERT_EQ(sink.failed_scopes, std::vector<std::uint64_t>({18U}));
  EXPECT_NE(sink.failure_details.front().find("asset"), std::string::npos);
  EXPECT_TRUE(sink.completed_scopes.empty());
}

TEST_F(ChaowenTtsAdapterTest, ReportsInferenceFailureWithoutCompletion)
{
  auto inference = std::make_unique<FakeInference>();
  inference->succeeds = false;
  auto * inference_ptr = inference.get();
  ChaowenTtsAdapter adapter(paths_, std::move(inference), fixture_manifest());
  RecordingSink sink;

  adapter.start(TtsRequest{19U, "安全回复"}, sink);

  EXPECT_EQ(inference_ptr->calls, 1U);
  ASSERT_EQ(sink.failed_scopes, std::vector<std::uint64_t>({19U}));
  EXPECT_EQ(sink.failure_details.front(), "fake inference failed");
  EXPECT_TRUE(sink.completed_scopes.empty());
  EXPECT_TRUE(sink.chunks.empty());
}

}  // namespace
}  // namespace voice_nav_audio
