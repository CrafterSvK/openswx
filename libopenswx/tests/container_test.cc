// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "openswx/container.h"
#include "test_helpers.h"

namespace openswx {
namespace {

void AppendEmbeddedParasolid(std::vector<uint8_t>* stream,
                             std::span<const uint8_t> payload) {
  static constexpr std::array<uint8_t, 16> kMarker = {
      0x23, 0x1D, 0xD5, 0x71, 0xDA, 0x81, 0x48, 0xA2,
      0xA8, 0x58, 0x98, 0xB2, 0x1B, 0x89, 0xEF, 0x99};
  const auto compressed = test::DeflateZlib(payload);
  stream->insert(stream->end(), kMarker.begin(), kMarker.end());
  test::AppendU32Le(stream, static_cast<uint32_t>(payload.size()));
  test::AppendU32Le(stream, static_cast<uint32_t>(compressed.size()));
  stream->insert(stream->end(), compressed.begin(), compressed.end());
}

TEST(ContainerTest, InspectsModernFile) {
  const std::string payload = "drawing-data";
  const std::vector<uint8_t> file = test::MakeModernChunkFile(
      "Contents/Definition",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(payload.data()), payload.size()),
      0x12345678u);
  const std::filesystem::path path =
      std::filesystem::path(testing::TempDir()) / "inspect-modern.SLDDRW";
  {
    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output.is_open());
    output.write(reinterpret_cast<const char*>(file.data()),
                 static_cast<std::streamsize>(file.size()));
    ASSERT_TRUE(output.good());
  }

  auto result = InspectContainer(path);
  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().streams.size(), 1u);
  const StreamInfo& stream = result.value().streams[0];
  EXPECT_EQ(stream.name, "Contents/Definition");
  EXPECT_EQ(stream.type_id, 0x12345678u);
  EXPECT_EQ(stream.status, StreamDecodeStatus::kDecoded);
  EXPECT_TRUE(stream.crc_matches);
  EXPECT_TRUE(stream.size_matches);
  EXPECT_EQ(std::string(stream.decoded_data.begin(),
                        stream.decoded_data.end()),
            payload);
  EXPECT_EQ(result.value().file_size, file.size());
  ASSERT_EQ(result.value().unknown_regions.size(), 1u);
  EXPECT_EQ(result.value().unknown_regions[0].offset, 8u);
  EXPECT_EQ(result.value().unknown_regions[0].size, 4u);
  EXPECT_EQ(result.value().claimed_bytes + result.value().unknown_bytes,
            result.value().file_size);

  std::error_code error;
  std::filesystem::remove(path, error);
}

TEST(ContainerTest, ExtractsEmbeddedParasolidStreamsAcrossSerializedRecords) {
  const std::vector<uint8_t> first = {'P', 'S', 0, 0, 'f', 'i', 'r', 's', 't'};
  const std::vector<uint8_t> second = {'P', 'S', 0, 0, 's', 'e', 'c', 'o', 'n', 'd'};
  std::vector<uint8_t> stream = {1, 0, 0, 0, 14, 0, 0, 0, 0xA5};
  const std::size_t first_offset = stream.size();
  AppendEmbeddedParasolid(&stream, first);
  stream.insert(stream.end(), {0xFF, 0xFE, 0x10, 0x00, 'm', 'o', 'O', 'b', 'j'});
  const std::size_t second_offset = stream.size();
  AppendEmbeddedParasolid(&stream, second);

  const auto extracted = ExtractEmbeddedParasolidStreams(stream);

  ASSERT_EQ(extracted.size(), 2u);
  EXPECT_EQ(extracted[0].marker_offset, first_offset);
  EXPECT_EQ(extracted[0].data, first);
  EXPECT_EQ(extracted[1].marker_offset, second_offset);
  EXPECT_EQ(extracted[1].data, second);
}

TEST(ContainerTest, IgnoresDamagedEmbeddedParasolidCandidate) {
  static constexpr std::array<uint8_t, 16> kMarker = {
      0x23, 0x1D, 0xD5, 0x71, 0xDA, 0x81, 0x48, 0xA2,
      0xA8, 0x58, 0x98, 0xB2, 0x1B, 0x89, 0xEF, 0x99};
  std::vector<uint8_t> stream(kMarker.begin(), kMarker.end());
  test::AppendU32Le(&stream, 100);
  test::AppendU32Le(&stream, 5000);
  stream.insert(stream.end(), {0x78, 0x01});

  EXPECT_TRUE(ExtractEmbeddedParasolidStreams(stream).empty());
}

}  // namespace
}  // namespace openswx
