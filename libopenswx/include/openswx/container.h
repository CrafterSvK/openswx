// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
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

/// A Parasolid transmit file embedded in a decoded SolidWorks stream.
///
/// Modern drawing section bodies use a repeated 24-byte wrapper consisting of
/// a 16-byte marker, two little-endian sizes and a zlib payload.  `data` starts
/// with the binary Parasolid `PS\0\0` signature after successful inflation.
struct EmbeddedParasolidStream {
  uint64_t marker_offset = 0;
  uint32_t uncompressed_size = 0;
  uint32_t compressed_size = 0;
  std::vector<uint8_t> data;
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

[[nodiscard]] const char* ToString(StreamDecodeStatus status) noexcept;

}  // namespace openswx
