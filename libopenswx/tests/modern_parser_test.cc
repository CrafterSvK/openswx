// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "internal/modern_parser.h"
#include "internal/stream_store.h"
#include "test_helpers.h"

namespace openswx::internal {
namespace {

TEST(ModernParserTest, RejectsTooShortFile) {
  const std::array<uint8_t, 7> file = {0};
  StreamMap streams;
  EXPECT_FALSE(ParseModernFormat(file, streams));
  EXPECT_TRUE(streams.empty());
}

TEST(ModernParserTest, ParsesValidInlineChunk) {
  const std::string payload = "preview-png";
  const std::vector<uint8_t> file = test::MakeModernChunkFile(
      "PreviewPNG",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(payload.data()), payload.size()));

  StreamMap streams;
  ASSERT_TRUE(ParseModernFormat(file, streams));
  ASSERT_EQ(streams.size(), 1u);
  EXPECT_EQ(std::string(streams.at("PreviewPNG").begin(),
                        streams.at("PreviewPNG").end()),
            payload);
}

TEST(ModernParserTest, ReportsBlockMetadataAndValidatesPayload) {
  const std::string payload = "preview-png";
  const std::vector<uint8_t> file = test::MakeModernChunkFile(
      "PreviewPNG",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(payload.data()), payload.size()),
      0x12345678u);

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  const auto& info = inspection.streams;
  ASSERT_EQ(info.size(), 1u);
  EXPECT_EQ(info[0].marker_offset, 12u);
  EXPECT_EQ(info[0].type_id, 0x12345678u);
  EXPECT_EQ(info[0].compressed_size, file.size() - 48u);
  EXPECT_EQ(info[0].uncompressed_size, payload.size());
  EXPECT_EQ(info[0].encoded_name_size, 10u);
  EXPECT_EQ(info[0].decoded_size, payload.size());
  EXPECT_EQ(info[0].status, StreamDecodeStatus::kDecoded);
  EXPECT_TRUE(info[0].size_matches);
  EXPECT_TRUE(info[0].crc_matches);
  EXPECT_TRUE(info[0].selected_as_primary);
  EXPECT_FALSE(info[0].duplicate_name);
  EXPECT_TRUE(info[0].decoded_data.empty());
}

TEST(ModernParserTest, ParsesPayloadWhoseCrcIsBelowOldInlineThreshold) {
  const std::vector<uint8_t> file = test::MakeModernChunkFile("Empty", {});

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  const auto& info = inspection.streams;
  ASSERT_EQ(info.size(), 1u);
  EXPECT_EQ(info[0].expected_crc32, 0u);
  EXPECT_EQ(info[0].status, StreamDecodeStatus::kEmpty);
  EXPECT_TRUE(info[0].crc_matches);
  EXPECT_TRUE(streams.contains("Empty"));
}

TEST(ModernParserTest, RejectsControlCharactersInStreamName) {
  const std::string name("Bad\0Name", 8);
  const std::array<uint8_t, 1> payload = {0x42};
  const std::vector<uint8_t> file = test::MakeModernChunkFile(name, payload);

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  EXPECT_TRUE(streams.empty());
  ASSERT_EQ(inspection.unknown_markers.size(), 1u);
  EXPECT_EQ(inspection.unknown_markers[0].inline_decode_status,
            StreamDecodeStatus::kInvalidName);
}

TEST(ModernParserTest, RejectsStreamWithCrcMismatch) {
  const std::string payload = "payload";
  std::vector<uint8_t> file = test::MakeModernChunkFile(
      "Data", std::span<const uint8_t>(
                  reinterpret_cast<const uint8_t*>(payload.data()),
                  payload.size()));
  file[22] ^= 0x01u;

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  EXPECT_TRUE(inspection.streams.empty());
  ASSERT_EQ(inspection.unknown_markers.size(), 1u);
  EXPECT_EQ(inspection.unknown_markers[0].inline_decode_status,
            StreamDecodeStatus::kCrcMismatch);
  EXPECT_FALSE(streams.contains("Data"));
}

TEST(ModernParserTest, ClassifiesCacheCell) {
  std::vector<uint8_t> file =
      test::MakeCacheCellFile("Contents/Definition", 4096, 0x11223344u);
  file.insert(file.begin() + 8, {0xAA, 0xBB, 0xCC});

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  EXPECT_TRUE(streams.empty());
  ASSERT_EQ(inspection.cache_cells.size(), 1u);
  const auto& cell = inspection.cache_cells[0];
  EXPECT_EQ(cell.marker_offset, 11u);
  EXPECT_EQ(cell.type_id, 0x11223344u);
  EXPECT_EQ(cell.logical_size, 4096u);
  EXPECT_EQ(cell.name, "Contents/Definition");
  EXPECT_TRUE(inspection.directory_entries.empty());
  EXPECT_TRUE(inspection.unknown_markers.empty());
  ASSERT_EQ(inspection.unknown_regions.size(), 1u);
  EXPECT_EQ(inspection.unknown_regions[0].offset, 8u);
  EXPECT_EQ(inspection.unknown_regions[0].size, 3u);
  EXPECT_EQ(inspection.unknown_bytes, 3u);
  EXPECT_EQ(inspection.claimed_bytes + inspection.unknown_bytes,
            inspection.file_size);
}

TEST(ModernParserTest, ClassifiesTailDirectoryEntry) {
  const std::vector<uint8_t> file = test::MakeDirectoryEntryFile(
      "ThirdPty/PBPDFlt", 2, 0x55667788u);

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  EXPECT_TRUE(streams.empty());
  ASSERT_EQ(inspection.directory_entries.size(), 1u);
  const auto& entry = inspection.directory_entries[0];
  EXPECT_EQ(entry.marker_offset, 8u);
  EXPECT_EQ(entry.type_id, 0x55667788u);
  EXPECT_EQ(entry.stored_size, 2u);
  EXPECT_EQ(entry.name, "ThirdPty/PBPDFlt");
  EXPECT_EQ(entry.record_size, file.size() - 8u);
  EXPECT_TRUE(std::all_of(entry.descriptor.begin(), entry.descriptor.end(),
                          [](uint8_t value) { return value == 0xA5; }));
  EXPECT_TRUE(std::all_of(entry.trailer.begin(), entry.trailer.end(),
                          [](uint8_t value) { return value == 0; }));
  EXPECT_TRUE(inspection.cache_cells.empty());
  EXPECT_TRUE(inspection.unknown_markers.empty());
  EXPECT_TRUE(inspection.unknown_regions.empty());
  EXPECT_EQ(inspection.claimed_bytes, file.size());
}

TEST(ModernParserTest, AccountsForUnknownMarkerAndSurroundingRanges) {
  std::vector<uint8_t> file(10, 0);
  test::AppendModernMarker(&file);
  file.insert(file.end(), 20, 0);

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  ASSERT_EQ(inspection.unknown_markers.size(), 1u);
  EXPECT_EQ(inspection.unknown_markers[0].marker_offset, 10u);
  ASSERT_EQ(inspection.unknown_regions.size(), 2u);
  EXPECT_EQ(inspection.unknown_regions[0].offset, 8u);
  EXPECT_EQ(inspection.unknown_regions[0].size, 2u);
  EXPECT_EQ(inspection.unknown_regions[1].offset, 16u);
  EXPECT_EQ(inspection.unknown_regions[1].size, 20u);
  EXPECT_EQ(inspection.claimed_bytes + inspection.unknown_bytes,
            inspection.file_size);
}

TEST(ModernParserTest, IgnoresChunkWithOversizedNameLength) {
  const std::array<uint8_t, 1> payload = {0x42};
  std::vector<uint8_t> file =
      test::MakeModernChunkFile("A", payload);
  const std::size_t name_size_offset = 8 + 0x1A;
  file[name_size_offset] = 0x01;
  file[name_size_offset + 1] = 0x02;
  file[name_size_offset + 2] = 0x00;
  file[name_size_offset + 3] = 0x00;

  StreamMap streams;
  EXPECT_TRUE(ParseModernFormat(file, streams));
  EXPECT_TRUE(streams.empty());
}

TEST(ModernParserTest, IgnoresChunkWithTruncatedPayload) {
  const std::array<uint8_t, 1> payload = {0x42};
  std::vector<uint8_t> file =
      test::MakeModernChunkFile("PreviewPNG", payload);
  file.pop_back();

  StreamMap streams;
  EXPECT_TRUE(ParseModernFormat(file, streams));
  EXPECT_TRUE(streams.empty());
}

TEST(ModernParserTest, IgnoresChunkWithOversizedCompressedSize) {
  const std::array<uint8_t, 1> payload = {0x42};
  std::vector<uint8_t> file =
      test::MakeModernChunkFile("PreviewPNG", payload);
  const std::size_t compressed_size_offset = 8 + 0x12;
  file[compressed_size_offset] = 0x00;
  file[compressed_size_offset + 1] = 0x00;
  file[compressed_size_offset + 2] = 0x00;
  file[compressed_size_offset + 3] = 0x08;

  StreamMap streams;
  EXPECT_TRUE(ParseModernFormat(file, streams));
  EXPECT_TRUE(streams.empty());
}

TEST(ModernParserTest, IgnoresChunkWhenDecompressionFails) {
  const std::array<uint8_t, 1> payload = {0x42};
  std::vector<uint8_t> file =
      test::MakeModernChunkFile("PreviewPNG", payload);
  file.back() ^= 0xFFu;

  StreamMap streams;
  EXPECT_TRUE(ParseModernFormat(file, streams));
  EXPECT_TRUE(streams.empty());
}

TEST(ModernParserTest, KeepsFirstStreamForDuplicateNames) {
  const std::string first_payload = "first";
  const std::string second_payload = "second";
  std::vector<uint8_t> file = test::MakeModernChunkFile(
      "PreviewPNG",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(first_payload.data()),
          first_payload.size()));
  std::vector<uint8_t> second_chunk = test::MakeModernChunkFile(
      "PreviewPNG",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(second_payload.data()),
          second_payload.size()));
  file.insert(file.end(), second_chunk.begin() + 8, second_chunk.end());

  StreamMap streams;
  ContainerInspection inspection;
  ASSERT_TRUE(ParseModernFormat(file, streams, &inspection));
  const auto& info = inspection.streams;
  ASSERT_EQ(streams.size(), 1u);
  ASSERT_EQ(info.size(), 2u);
  EXPECT_FALSE(info[0].duplicate_name);
  EXPECT_TRUE(info[1].duplicate_name);
  EXPECT_FALSE(info[1].selected_as_primary);
  EXPECT_EQ(std::string(info[1].decoded_data.begin(),
                        info[1].decoded_data.end()),
            second_payload);
  EXPECT_EQ(std::string(streams.at("PreviewPNG").begin(),
                        streams.at("PreviewPNG").end()),
            first_payload);
}

TEST(ModernParserTest, AcceptsUnknownSectionTypeForInlineData) {
  const std::string payload = "unknown-section";
  const std::vector<uint8_t> file = test::MakeModernChunkFile(
      "PreviewPNG",
      std::span<const uint8_t>(
          reinterpret_cast<const uint8_t*>(payload.data()), payload.size()),
      0x77);

  StreamMap streams;
  ASSERT_TRUE(ParseModernFormat(file, streams));
  ASSERT_EQ(streams.size(), 1u);
  EXPECT_EQ(std::string(streams.at("PreviewPNG").begin(),
                        streams.at("PreviewPNG").end()),
            payload);
}

}  // namespace
}  // namespace openswx::internal
