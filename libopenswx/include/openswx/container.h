// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "openswx/document.h"

namespace openswx {

enum class ContainerKind {
  kModern,
};

enum class StreamDecodeStatus {
  kDecoded,
  kEmpty,
  kTruncatedHeader,
  kNameTooLarge,
  kCompressedSizeTooLarge,
  kTruncatedName,
  kInvalidName,
  kTruncatedPayload,
  kDecompressionFailed,
  kUncompressedSizeMismatch,
  kCrcMismatch,
};

struct StreamInfo {
  uint64_t marker_offset = 0;
  uint64_t record_size = 0;
  uint32_t type_id = 0;
  uint32_t expected_crc32 = 0;
  uint32_t compressed_size = 0;
  uint32_t uncompressed_size = 0;
  uint32_t encoded_name_size = 0;
  uint64_t decoded_size = 0;
  std::string name;
  StreamDecodeStatus status = StreamDecodeStatus::kTruncatedHeader;
  bool size_matches = false;
  bool crc_matches = false;
  bool duplicate_name = false;
  bool selected_as_primary = false;
  std::vector<uint8_t> decoded_data;
};

struct CacheCellInfo {
  uint64_t marker_offset = 0;
  uint64_t record_size = 0;
  uint32_t type_id = 0;
  uint32_t logical_size = 0;
  std::string name;
};

struct DirectoryEntryInfo {
  uint64_t marker_offset = 0;
  uint64_t record_size = 0;
  uint32_t type_id = 0;
  uint32_t stored_size = 0;
  std::string name;
  std::array<uint8_t, 14> descriptor{};
  std::array<uint8_t, 6> trailer{};
};

struct UnknownMarkerInfo {
  uint64_t marker_offset = 0;
  uint32_t type_id = 0;
  StreamDecodeStatus inline_decode_status =
      StreamDecodeStatus::kTruncatedHeader;
  std::string candidate_name;
};

struct UnknownRegion {
  uint64_t offset = 0;
  uint64_t size = 0;
};

struct ContainerInspection {
  ContainerKind kind = ContainerKind::kModern;
  uint64_t file_size = 0;
  uint64_t claimed_bytes = 0;
  uint64_t unknown_bytes = 0;
  std::vector<StreamInfo> streams;
  std::vector<CacheCellInfo> cache_cells;
  std::vector<DirectoryEntryInfo> directory_entries;
  std::vector<UnknownMarkerInfo> unknown_markers;
  std::vector<UnknownRegion> unknown_regions;
};

/// One compressed frame within a decoded SolidWorks stream.
struct ParasolidFrame {
  /// Offset of the two size fields within the decoded source stream.
  uint64_t header_offset = 0;
  uint32_t uncompressed_size = 0;
  uint32_t compressed_size = 0;
};

/// Wrappers carry a marker and one frame, or a length-prefixed section with
/// multiple frames. Data concatenates all frames of one complete stream.
struct EmbeddedParasolidStream {
  uint64_t marker_offset = 0;
  /// Sums over all frames, excluding framing and padding.
  uint64_t uncompressed_size = 0;
  uint64_t compressed_size = 0;
  std::vector<uint8_t> data;
  std::vector<ParasolidFrame> frames;
  /// Present for a section; points to the length before the marker.
  std::optional<uint64_t> section_offset;
  /// Complete section or standalone wrapper, including its framing/padding.
  uint64_t stored_size = 0;
};

enum class ParasolidPayloadKind { kTransmit, kPartition, kDeltas, kUnknown };

struct ParasolidPayload {
  std::string source_stream;
  uint64_t container_marker_offset = 0;
  /// Numeric stream slot, not a cross-family configuration/state identity.
  std::optional<uint32_t> configuration_index;
  bool ghost_partition = false;
  ParasolidPayloadKind kind = ParasolidPayloadKind::kUnknown;
  bool header_valid = false;
  std::string description;
  std::string schema;
  EmbeddedParasolidStream payload;
};

struct ParasolidExtractionIssue {
  std::string source_stream;
  /// Offset within the decoded stream, absent for container decode failures.
  std::optional<uint64_t> marker_offset;
  std::string message;
};

struct ParasolidExtraction {
  uint64_t inspected_stream_count = 0;
  uint64_t wrapper_count = 0;
  uint64_t non_parasolid_block_count = 0;
  std::vector<ParasolidPayload> payloads;
  std::vector<ParasolidExtractionIssue> issues;
};

/// Inspects the modern SolidWorks container without interpreting its streams.
/// Legacy OLE2/CFB and ZIP/OPC files are intentionally rejected.
[[nodiscard]] Result<ContainerInspection> InspectContainer(
    const std::filesystem::path& path);

/// Finds and inflates valid Parasolid transmit payloads embedded in a decoded
/// stream such as `SectionBodies/View-N`.  Invalid marker candidates are
/// ignored, so unrelated serialized records surrounding the payloads are safe.
[[nodiscard]] std::vector<EmbeddedParasolidStream>
ExtractEmbeddedParasolidStreams(std::span<const uint8_t> decoded_stream);

/// Extracts wrapped binary Parasolid from every active, decoded container
/// stream. Historical duplicate chunks are excluded. Records configuration
/// indices encoded in stream names and reports damaged wrappers. This does
/// not resolve delta dependencies or establish standalone kernel readability.
[[nodiscard]] ParasolidExtraction
ExtractParasolidStreams(const ContainerInspection &inspection);

[[nodiscard]] const char *ToString(ParasolidPayloadKind kind) noexcept;

[[nodiscard]] const char* ToString(StreamDecodeStatus status) noexcept;

}  // namespace openswx
