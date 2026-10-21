// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "test_helpers.h"

namespace {
using nlohmann::json;
namespace fs = std::filesystem;

std::string ShellQuote(const fs::path &path) {
  std::string quoted = "'";
  for (char c : path.string())
    quoted += c == '\'' ? "'\"'\"'" : std::string(1, c);
  return quoted + "'";
}

std::vector<uint8_t> Payload(std::string_view form = "") {
  std::vector<uint8_t> bytes{'P', 'S'};
  const auto text = [&](std::string_view value) {
    const auto size = static_cast<uint32_t>(value.size());
    for (int shift : {24, 16, 8, 0})
      bytes.push_back(static_cast<uint8_t>(size >> shift));
    bytes.insert(bytes.end(), value.begin(), value.end());
  };
  text(": TRANSMIT FILE " + std::string(form) +
       "created by modeller version 1234567");
  text("SCH_1234567_12345_12345");
  bytes.insert(bytes.end(), {0, 1, 2, 3});
  return bytes;
}

void AppendWrapper(std::vector<uint8_t> &bytes,
                   std::span<const uint8_t> payload) {
  const std::array<uint8_t, 16> marker{0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81,
                                       0x48, 0xa2, 0xa8, 0x58, 0x98, 0xb2,
                                       0x1b, 0x89, 0xef, 0x99};
  bytes.insert(bytes.end(), marker.begin(), marker.end());
  const auto compressed = openswx::test::DeflateZlib(payload);
  openswx::test::AppendU32Le(&bytes, static_cast<uint32_t>(payload.size()));
  openswx::test::AppendU32Le(&bytes, static_cast<uint32_t>(compressed.size()));
  bytes.insert(bytes.end(), compressed.begin(), compressed.end());
}

void WriteBytes(const fs::path &path, std::span<const uint8_t> bytes) {
  std::ofstream out(path, std::ios::binary);
  ASSERT_TRUE(out.is_open());
  out.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(out.good());
}

std::vector<uint8_t> ReadBytes(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

class ParasolidCliTest : public testing::Test {
protected:
  fs::path root;
  void SetUp() override {
    // Include shell metacharacters to exercise argument quoting as well.
    root = fs::path(testing::TempDir()) /
           ("swx parasolid ' " +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
  }
  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }
  int Run(std::string arguments = "") {
    const std::string command = ShellQuote(SWX_DUMP_EXECUTABLE) +
                                " --extract-parasolid " +
                                ShellQuote(root / "output") + " " + arguments +
                                " " + ShellQuote(root / "input.SLDPRT") +
                                " > " + ShellQuote(root / "stdout.json") +
                                " 2> " + ShellQuote(root / "stderr.txt");
    return std::system(command.c_str());
  }
  json Manifest() {
    std::ifstream in(root / "output/manifest.json");
    return json::parse(in);
  }
};

TEST_F(ParasolidCliTest, ExportsSeparatedPayloadsAndExactProvenance) {
  const auto partition = Payload("(partition) ");
  const auto delta = Payload("(deltas) ");
  std::vector<uint8_t> bytes{0, 0, 0, 0};
  AppendWrapper(bytes, partition);
  const auto delta_offset = bytes.size();
  AppendWrapper(bytes, delta);
  WriteBytes(root / "input.SLDPRT", openswx::test::MakeModernChunkFile(
                                        "Contents/Config-7-Partition", bytes));
  ASSERT_EQ(Run(), 0);
  const auto manifest = Manifest();
  ASSERT_EQ(manifest["payload_count"], 2);
  EXPECT_EQ(manifest["issue_count"], 0);
  const auto &a = manifest["payloads"][0];
  const auto &b = manifest["payloads"][1];
  EXPECT_EQ(a["category"], "partitions");
  EXPECT_EQ(a["configuration_index"], 7);
  EXPECT_EQ(a["source_stream"], "Contents/Config-7-Partition");
  EXPECT_EQ(a["marker_offset"], 4);
  EXPECT_EQ(b["marker_offset"], delta_offset);
  EXPECT_EQ(b["category"], "deltas");
  EXPECT_EQ(b["base_partition_candidate"], 0);
  EXPECT_EQ(b["requires_base"], true);
  EXPECT_EQ(manifest["standalone_readability"], "unverified");
  EXPECT_EQ(ReadBytes(root / "output" / a["file"].get<std::string>()),
            partition);
  EXPECT_EQ(ReadBytes(root / "output" / b["file"].get<std::string>()), delta);
  EXPECT_EQ(fs::path(b["file"].get<std::string>()).extension(), ".bin");
  std::ifstream stdout_file(root / "stdout.json");
  EXPECT_EQ(json::parse(stdout_file), manifest);
  // A second export must leave existing output untouched.
  EXPECT_NE(Run(), 0);
  EXPECT_EQ(Manifest(), manifest);
}

TEST_F(ParasolidCliTest, WritesCompleteFrameChainAndPerFrameManifest) {
  const auto payload = Payload("(partition) ");
  const std::vector<uint8_t> first(payload.begin(), payload.begin() + 3);
  const std::vector<uint8_t> second(payload.begin() + 3, payload.end());
  std::vector<uint8_t> bytes(4u);
  AppendWrapper(bytes, first);
  const auto compressed = openswx::test::DeflateZlib(second);
  const auto second_header = bytes.size();
  openswx::test::AppendU32Le(&bytes, static_cast<uint32_t>(second.size()));
  openswx::test::AppendU32Le(&bytes, static_cast<uint32_t>(compressed.size()));
  bytes.insert(bytes.end(), compressed.begin(), compressed.end());
  bytes.resize(bytes.size() + 5u, 0u);
  const auto length = static_cast<uint32_t>(bytes.size() - 4u);
  for (std::size_t i = 0; i < 4u; ++i)
    bytes[i] = static_cast<uint8_t>(length >> (8u * i));
  WriteBytes(root / "input.SLDPRT", openswx::test::MakeModernChunkFile(
                                        "Contents/Config-0-Partition", bytes));
  ASSERT_EQ(Run(), 0);
  const auto manifest = Manifest();
  ASSERT_EQ(manifest["payload_count"], 1);
  const auto &item = manifest["payloads"][0];
  EXPECT_EQ(item["frame_count"], 2);
  EXPECT_EQ(item["section_offset"], 0);
  EXPECT_EQ(item["stored_size"], bytes.size());
  EXPECT_EQ(item["uncompressed_size"], payload.size());
  EXPECT_EQ(item["frames"][0]["header_offset"], 20);
  EXPECT_EQ(item["frames"][1]["header_offset"], second_header);
  const auto file = root / "output" / item["file"].get<std::string>();
  EXPECT_EQ(ReadBytes(file), payload);
}

TEST_F(ParasolidCliTest, ReportsDamageWithoutDiscardingRecoverablePayloads) {
  std::vector<uint8_t> bytes;
  AppendWrapper(bytes, Payload());
  bytes[24] ^= 0xffu;
  AppendWrapper(bytes, Payload());
  WriteBytes(root / "input.SLDPRT",
             openswx::test::MakeModernChunkFile("Contents/Definition", bytes));
  EXPECT_NE(Run(), 0);
  const auto manifest = Manifest();
  EXPECT_EQ(manifest["issue_count"], 1);
  EXPECT_EQ(manifest["payload_count"], 1);
  EXPECT_EQ(manifest["issues"][0]["marker_offset"], 0);
  const auto file = manifest["payloads"][0]["file"].get<std::string>();
  EXPECT_EQ(ReadBytes(root / "output" / file), Payload());
}

TEST_F(ParasolidCliTest,
       ExportsGhostAndSectionBodiesAndIgnoresDuplicateHistory) {
  std::vector<uint8_t> bytes;
  AppendWrapper(bytes, Payload("(partition) "));
  auto file = openswx::test::MakeModernChunkFile(
      "Contents/Config-0-GhostPartition", bytes);
  for (const auto &name :
       {"SectionBodies/View-1", "Contents/Config-0-GhostPartition"}) {
    const auto chunk = openswx::test::MakeModernChunkFile(name, bytes);
    file.insert(file.end(), chunk.begin() + 12, chunk.end());
  }
  WriteBytes(root / "input.SLDPRT", file);
  ASSERT_EQ(Run(), 0);
  const auto manifest = Manifest();
  ASSERT_EQ(manifest["payload_count"], 2);
  std::vector<std::string> categories;
  for (const auto &entry : manifest["payloads"])
    categories.push_back(entry["category"]);
  std::sort(categories.begin(), categories.end());
  EXPECT_EQ(categories,
            (std::vector<std::string>{"ghost-partitions", "section-bodies"}));
}

TEST_F(ParasolidCliTest, ProducesEmptyManifestAndRejectsConflictingActions) {
  std::vector<uint8_t> unrelated;
  AppendWrapper(unrelated, std::vector<uint8_t>{1, 2, 3});
  WriteBytes(root / "input.SLDPRT", openswx::test::MakeModernChunkFile(
                                        "Contents/Definition", unrelated));
  EXPECT_NE(Run("--list-streams"), 0);
  EXPECT_FALSE(fs::exists(root / "output"));
  ASSERT_EQ(Run(), 0);
  EXPECT_EQ(Manifest()["payload_count"], 0);
  EXPECT_EQ(Manifest()["issue_count"], 0);
  EXPECT_EQ(Manifest()["non_parasolid_block_count"], 1);
}

} // namespace
