// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <algorithm>
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

std::vector<uint8_t> ParasolidFixture(std::string_view form = "") {
  std::vector<uint8_t> data{'P', 'S'};
  const auto append_string = [&](std::string_view value) {
    const auto size = static_cast<uint32_t>(value.size());
    for (int shift : {24, 16, 8, 0})
      data.push_back(static_cast<uint8_t>((size >> shift) & 255u));
    data.insert(data.end(), value.begin(), value.end());
  };
  append_string(": TRANSMIT FILE " + std::string(form) +
                "created by modeller version 1234567");
  append_string("SCH_1234567_12345_12345");
  data.insert(data.end(), {0, 1, 2, 3});
  return data;
}

StreamInfo ParasolidSource(std::string name, std::vector<uint8_t> data) {
  StreamInfo stream;
  stream.name = std::move(name);
  stream.status = StreamDecodeStatus::kDecoded;
  stream.selected_as_primary = true;
  stream.marker_offset = 120u;
  stream.decoded_data = std::move(data);
  return stream;
}

TEST(ContainerTest, ExtractsAllActiveParasolidStreamsWithProvenance) {
  const auto partition = ParasolidFixture("(partition) ");
  const auto delta = ParasolidFixture("(deltas) ");
  std::vector<uint8_t> bytes{0, 0, 0, 0};
  AppendEmbeddedParasolid(&bytes, partition);
  const auto delta_offset = bytes.size();
  AppendEmbeddedParasolid(&bytes, delta);
  ContainerInspection inspection;
  inspection.streams.push_back(
      ParasolidSource("Contents/Config-17-Partition", bytes));
  inspection.streams.push_back(
      ParasolidSource("Contents/Config-17-GhostPartition", bytes));
  inspection.streams.push_back(ParasolidSource("SectionBodies/View-1", bytes));
  inspection.streams.push_back(ParasolidSource("Contents/Definition", bytes));
  auto historical = ParasolidSource("Contents/Config-17-Partition", bytes);
  historical.selected_as_primary = false;
  inspection.streams.push_back(std::move(historical));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_TRUE(result.issues.empty());
  EXPECT_EQ(result.inspected_stream_count, 4u);
  EXPECT_EQ(result.wrapper_count, 8u);
  ASSERT_EQ(result.payloads.size(), 8u);
  const auto &first = result.payloads.front();
  EXPECT_EQ(first.source_stream, "Contents/Config-17-Partition");
  EXPECT_EQ(first.container_marker_offset, 120u);
  EXPECT_EQ(first.configuration_index, 17u);
  EXPECT_EQ(first.kind, ParasolidPayloadKind::kPartition);
  EXPECT_TRUE(first.header_valid);
  EXPECT_EQ(first.schema, "SCH_1234567_12345_12345");
  EXPECT_EQ(first.payload.marker_offset, 4u);
  EXPECT_EQ(first.payload.data, partition);
  EXPECT_EQ(result.payloads[1].kind, ParasolidPayloadKind::kDeltas);
  EXPECT_EQ(result.payloads[1].payload.data, delta);
  EXPECT_EQ(result.payloads[1].payload.marker_offset, delta_offset);
  EXPECT_TRUE(result.payloads[2].ghost_partition);
  EXPECT_FALSE(result.payloads[4].configuration_index.has_value());
}

TEST(ContainerTest, ReportsDamagedWrapperAndRecoversLaterValidBlock) {
  auto damaged = ParasolidFixture();
  std::vector<uint8_t> bytes;
  AppendEmbeddedParasolid(&bytes, damaged);
  bytes[24] ^= 0xffu; // Damage the zlib header.
  const auto valid_offset = bytes.size();
  AppendEmbeddedParasolid(&bytes, damaged);
  ContainerInspection inspection;
  inspection.streams.push_back(ParasolidSource("Contents/Definition", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_EQ(result.wrapper_count, 2u);
  ASSERT_EQ(result.issues.size(), 1u);
  EXPECT_EQ(result.issues.front().marker_offset, 0u);
  ASSERT_EQ(result.payloads.size(), 1u);
  EXPECT_EQ(result.payloads.front().payload.marker_offset, valid_offset);
  EXPECT_EQ(result.payloads.front().payload.data, damaged);
}

TEST(ContainerTest, DoesNotSkipLaterBlocksWhenCompressedSizeIsOverstated) {
  std::vector<uint8_t> bytes;
  const auto payload = ParasolidFixture();
  AppendEmbeddedParasolid(&bytes, payload);
  const auto second_offset = bytes.size();
  AppendEmbeddedParasolid(&bytes, payload);
  // The first declaration incorrectly includes the complete second wrapper.
  const auto wrong_size = static_cast<uint32_t>(bytes.size() - 24u);
  for (std::size_t i = 0; i < 4u; ++i)
    bytes[20u + i] = static_cast<uint8_t>(wrong_size >> (8u * i));
  ContainerInspection inspection;
  inspection.streams.push_back(ParasolidSource("Contents/Definition", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  ASSERT_EQ(result.issues.size(), 1u);
  EXPECT_EQ(result.issues.front().message, "compressed_size_mismatch");
  ASSERT_EQ(result.payloads.size(), 1u);
  EXPECT_EQ(result.payloads.front().payload.marker_offset, second_offset);
  EXPECT_EQ(result.payloads.front().payload.data, payload);
}

TEST(ContainerTest, ReportsTruncatedAndMismatchedWrappers) {
  std::vector<uint8_t> bytes;
  AppendEmbeddedParasolid(&bytes, ParasolidFixture());
  const auto truncated = bytes;
  bytes[16] ^= 1u; // Declared uncompressed size differs from actual data.
  ContainerInspection inspection;
  inspection.streams.push_back(ParasolidSource("size-mismatch", bytes));
  bytes = truncated;
  bytes.pop_back();
  inspection.streams.push_back(ParasolidSource("truncated-payload", bytes));
  bytes.resize(20u);
  inspection.streams.push_back(ParasolidSource("truncated-header", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_TRUE(result.payloads.empty());
  ASSERT_EQ(result.issues.size(), 3u);
  EXPECT_EQ(result.issues[0].message, "uncompressed_size_mismatch");
  EXPECT_EQ(result.issues[1].message, "truncated_wrapper_payload");
  EXPECT_EQ(result.issues[2].message, "truncated_wrapper_header");
}

TEST(ContainerTest, PreservesUnknownHeadersWithoutTrustingTheirLengths) {
  const std::vector<uint8_t> unknown{'P', 'S', 0, 0, 0xff, 0xff};
  std::vector<uint8_t> bytes;
  AppendEmbeddedParasolid(&bytes, unknown);
  ContainerInspection inspection;
  inspection.streams.push_back(
      ParasolidSource("Contents/Config-invalid-Partition", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  ASSERT_EQ(result.payloads.size(), 1u);
  EXPECT_FALSE(result.payloads.front().header_valid);
  EXPECT_FALSE(result.payloads.front().configuration_index);
  EXPECT_EQ(result.payloads.front().kind, ParasolidPayloadKind::kUnknown);
  EXPECT_EQ(result.payloads.front().payload.data, unknown);
  ASSERT_EQ(result.issues.size(), 1u);
  EXPECT_EQ(result.issues.front().message, "unrecognised_parasolid_header");
}

TEST(ContainerTest, SkipsOtherZlibPayloadsAndReportsContainerDecodeFailures) {
  std::vector<uint8_t> bytes;
  const std::vector<uint8_t> unrelated{'n', 'o', 't', 'P', 'S'};
  AppendEmbeddedParasolid(&bytes, unrelated);
  ContainerInspection inspection;
  inspection.streams.push_back(ParasolidSource("Contents/Definition", bytes));
  auto bad = ParasolidSource("Contents/Config-0-Partition", {});
  bad.status = StreamDecodeStatus::kCrcMismatch;
  inspection.streams.push_back(std::move(bad));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_TRUE(result.payloads.empty());
  EXPECT_EQ(result.non_parasolid_block_count, 1u);
  ASSERT_EQ(result.issues.size(), 1u);
  EXPECT_EQ(result.issues[0].message, "container_stream_crc_mismatch");
  EXPECT_FALSE(result.issues[0].marker_offset);
}

std::vector<uint8_t>
ChainedFixture(const std::vector<std::vector<uint8_t>> &frames,
               std::size_t padding = 0u) {
  std::vector<uint8_t> bytes(4u);
  if (frames.empty())
    return bytes;
  AppendEmbeddedParasolid(&bytes, frames.front());
  for (std::size_t index = 1u; index < frames.size(); ++index) {
    const auto compressed = test::DeflateZlib(frames[index]);
    test::AppendU32Le(&bytes, static_cast<uint32_t>(frames[index].size()));
    test::AppendU32Le(&bytes, static_cast<uint32_t>(compressed.size()));
    bytes.insert(bytes.end(), compressed.begin(), compressed.end());
  }
  bytes.resize(bytes.size() + padding, 0u);
  const auto length = static_cast<uint32_t>(bytes.size() - 4u);
  for (std::size_t i = 0; i < 4u; ++i)
    bytes[i] = static_cast<uint8_t>(length >> (8u * i));
  return bytes;
}

TEST(ContainerTest, ConcatenatesEveryFrameBeforeCheckingSignatureAndHeader) {
  const auto payload = ParasolidFixture("(partition) ");
  const auto section =
      ChainedFixture({{payload.begin(), payload.begin() + 3},
                      {payload.begin() + 3, payload.begin() + 17},
                      {payload.begin() + 17, payload.end()}},
                     7u);
  auto bytes = section;
  const auto next = ChainedFixture({ParasolidFixture("(deltas) ")});
  bytes.insert(bytes.end(), next.begin(), next.end());
  ContainerInspection inspection;
  inspection.streams.push_back(
      ParasolidSource("Contents/Config-0-Partition", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_TRUE(result.issues.empty());
  ASSERT_EQ(result.payloads.size(), 2u);
  const auto &first = result.payloads[0];
  EXPECT_TRUE(first.header_valid);
  EXPECT_EQ(first.payload.data, payload);
  EXPECT_EQ(first.payload.frames.size(), 3u);
  EXPECT_EQ(first.payload.section_offset, 0u);
  EXPECT_EQ(first.payload.marker_offset, 4u);
  EXPECT_EQ(first.payload.stored_size, section.size());
  EXPECT_EQ(first.payload.uncompressed_size, payload.size());
  EXPECT_EQ(first.payload.frames.front().header_offset, 20u);
  EXPECT_EQ(result.payloads[1].payload.section_offset, section.size());
  EXPECT_EQ(result.payloads[1].kind, ParasolidPayloadKind::kDeltas);
  const auto legacy_api = ExtractEmbeddedParasolidStreams(bytes);
  ASSERT_EQ(legacy_api.size(), 2u);
  EXPECT_EQ(legacy_api[0].data, payload);
}

TEST(ContainerTest,
     RejectsWholeChainWithBrokenContinuationAndRecoversNextSection) {
  const auto payload = ParasolidFixture();
  auto bytes = ChainedFixture({{payload.begin(), payload.begin() + 40},
                               {payload.begin() + 40, payload.end()}});
  const auto compressed =
      test::DeflateZlib(std::span<const uint8_t>(payload).first(40));
  const std::size_t second_header = 28u + compressed.size();
  bytes[second_header + 8u] ^= 0xffu;
  const auto next_offset = bytes.size();
  const auto next = ChainedFixture({payload});
  bytes.insert(bytes.end(), next.begin(), next.end());
  ContainerInspection inspection;
  inspection.streams.push_back(ParasolidSource("Contents/Definition", bytes));
  const auto result = ExtractParasolidStreams(inspection);
  ASSERT_EQ(result.issues.size(), 1u);
  EXPECT_EQ(result.issues[0].message, "decompression_failed_or_size_limit");
  ASSERT_EQ(result.payloads.size(), 1u);
  EXPECT_EQ(result.payloads[0].payload.data, payload);
  EXPECT_EQ(result.payloads[0].payload.section_offset, next_offset);
}

TEST(ContainerTest, DoesNotEmitFirstFrameOfTruncatedOrOversizedChain) {
  const auto payload = ParasolidFixture();
  const auto whole = ChainedFixture({{payload.begin(), payload.begin() + 40},
                                     {payload.begin() + 40, payload.end()}});
  const auto first_compressed =
      test::DeflateZlib(std::span<const uint8_t>(payload).first(40));
  const auto second_header = 28u + first_compressed.size();
  ContainerInspection inspection;
  auto truncated = whole;
  truncated.pop_back();
  inspection.streams.push_back(ParasolidSource("truncated", truncated));
  auto oversized = whole;
  const uint32_t size = 128u * 1024u * 1024u;
  for (std::size_t i = 0; i < 4u; ++i)
    oversized[second_header + i] = static_cast<uint8_t>(size >> (8u * i));
  inspection.streams.push_back(ParasolidSource("oversized", oversized));
  const auto result = ExtractParasolidStreams(inspection);
  EXPECT_TRUE(result.payloads.empty());
  ASSERT_EQ(result.issues.size(), 2u);
  EXPECT_EQ(result.issues[0].message, "truncated_chain_section");
  EXPECT_EQ(result.issues[1].message, "stream_size_limit");
}

TEST(ContainerTest, ReadsU16DescriptionAndVariablePaddingBeforeU8Schema) {
  const std::string description =
      ": TRANSMIT FILE created by modeller version 1234567";
  const std::string schema = "SCH_1234567_12345_12345";
  for (const std::vector<uint8_t> &padding :
       {std::vector<uint8_t>{}, std::vector<uint8_t>{0},
        std::vector<uint8_t>{0xff, 0, 3, 7, 0xff, 1, 0}}) {
    std::vector<uint8_t> payload{'P',
                                 'S',
                                 0,
                                 0,
                                 static_cast<uint8_t>(description.size() >> 8u),
                                 static_cast<uint8_t>(description.size())};
    payload.insert(payload.end(), description.begin(), description.end());
    payload.insert(payload.end(), padding.begin(), padding.end());
    payload.push_back(static_cast<uint8_t>(schema.size()));
    payload.insert(payload.end(), schema.begin(), schema.end());
    payload.insert(payload.end(), {'A', 'B', 'C', 0, 0xff});
    ContainerInspection inspection;
    inspection.streams.push_back(
        ParasolidSource("Contents/Definition", ChainedFixture({payload})));
    const auto result = ExtractParasolidStreams(inspection);
    ASSERT_TRUE(result.issues.empty());
    ASSERT_EQ(result.payloads.size(), 1u);
    EXPECT_TRUE(result.payloads[0].header_valid);
    EXPECT_EQ(result.payloads[0].description, description);
    EXPECT_EQ(result.payloads[0].schema, schema);
    EXPECT_EQ(result.payloads[0].kind, ParasolidPayloadKind::kTransmit);
  }
}

TEST(ContainerTest, RejectsSchemaWithInvalidCharactersOrTruncatedLength) {
  for (const bool oversized_length : {false, true}) {
    auto payload = ParasolidFixture();
    constexpr std::string_view token = "SCH_";
    const auto schema =
        std::search(payload.begin(), payload.end(), token.begin(), token.end());
    ASSERT_NE(schema, payload.end());
    if (oversized_length)
      *(schema - 1) = 255u;
    else
      *(schema + 4) = '!';
    ContainerInspection inspection;
    inspection.streams.push_back(
        ParasolidSource("Contents/Definition", ChainedFixture({payload})));
    const auto result = ExtractParasolidStreams(inspection);
    ASSERT_EQ(result.payloads.size(), 1u);
    EXPECT_FALSE(result.payloads[0].header_valid);
    ASSERT_EQ(result.issues.size(), 1u);
    EXPECT_EQ(result.issues[0].message, "unrecognised_parasolid_header");
  }
}

}  // namespace
}  // namespace openswx
