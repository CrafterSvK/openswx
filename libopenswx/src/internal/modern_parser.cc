// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "internal/modern_parser.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <zlib.h>

#include "internal/decompressor.h"
#include "internal/rol_codec.h"

namespace openswx::internal {

// ── Format-detection helpers ──────────────────────────────────────────────────

bool IsZipOpc(std::span<const uint8_t> data) noexcept {
  // ZIP local-file-header magic: PK\x03\x04
  return data.size() >= 4 &&
         data[0] == 0x50 && data[1] == 0x4B &&
         data[2] == 0x03 && data[3] == 0x04;
}

bool IsModernSwx(std::span<const uint8_t> data) noexcept {
  // Reject OLE2 and ZIP/OPC — those are handled by their own parsers.
  if (data.size() < 22) return false;  // need at least header + one marker
  if (data[0] == 0xD0 && data[1] == 0xCF &&
      data[2] == 0x11 && data[3] == 0xE0) return false;  // OLE2
  if (data[0] == 0x50 && data[1] == 0x4B &&
      data[2] == 0x03 && data[3] == 0x04) return false;  // ZIP/OPC

  // The 6-byte chunk marker always appears within the first 64 bytes.
  // Empirically offset 13–21 in known SW files; scan up to 64 for safety.
  constexpr std::size_t kSearchLimit = 64;
  const std::size_t limit = std::min(data.size(), kSearchLimit);
  constexpr uint8_t kMarker[] = {0x14, 0x00, 0x06, 0x00, 0x08, 0x00};
  auto it = std::search(data.begin(), data.begin() + limit,
                        std::begin(kMarker), std::end(kMarker));
  return it != data.begin() + limit;
}

bool IsValidStreamName(const std::string& name) noexcept {
  if (name.empty()) return false;
  for (unsigned char c : name)
    if (c < 0x20 || c >= 0x7F) return false;
  return true;
}

// ── Internal chunk-parsing constants ─────────────────────────────────────────

namespace {

// Inline block layout, relative to the six-byte marker:
//   +0x00   14 00 06 00 08 00
//   +0x06   type_id  uint32 LE
//   +0x0a   crc32    uint32 LE, over the uncompressed payload
//   +0x0e   csz      uint32 LE
//   +0x12   usz      uint32 LE
//   +0x16   nsz      uint32 LE
//   +0x1a   name[nsz], ROL-encoded UTF-8
//   +0x1a+nsz raw-deflate payload
constexpr std::size_t kHeaderSizeFromMarker = 0x1A;
constexpr uint32_t kMaxNameSize = 512;
constexpr uint32_t kMaxCompressedSize = 64 * 1024 * 1024;  // 64 MiB sanity cap
constexpr std::size_t kDirectoryHeaderSize = 40;
constexpr std::size_t kDirectoryTrailerSize = 6;

// Stable six-byte marker shared by modern container record families.
constexpr uint8_t kMarkerPrefix[] = {0x14, 0x00, 0x06, 0x00, 0x08, 0x00};
constexpr std::size_t kMarkerLen = sizeof(kMarkerPrefix);

// Reads a uint32_t LE at the given absolute byte offset within buf.
[[nodiscard]] uint32_t U32At(std::span<const uint8_t> buf,
                              std::size_t offset) noexcept {
  if (offset + 4 > buf.size()) return 0;
  return static_cast<uint32_t>(buf[offset]) |
         (static_cast<uint32_t>(buf[offset + 1]) << 8) |
         (static_cast<uint32_t>(buf[offset + 2]) << 16) |
         (static_cast<uint32_t>(buf[offset + 3]) << 24);
}

[[nodiscard]] std::optional<CacheCellInfo> TryCacheCell(
    std::span<const uint8_t> data, std::size_t marker_pos, int key) {
  if (marker_pos + kHeaderSizeFromMarker > data.size()) return std::nullopt;

  const uint32_t two_l = U32At(data, marker_pos + 0x0A);
  const uint32_t half_l = U32At(data, marker_pos + 0x0E);
  const uint32_t logical_size = U32At(data, marker_pos + 0x12);
  const uint32_t name_size = U32At(data, marker_pos + 0x16);
  if (logical_size == 0 || two_l != logical_size * 2u ||
      half_l != logical_size / 2u || name_size == 0 ||
      name_size >= 500) {
    return std::nullopt;
  }

  const std::size_t name_start = marker_pos + kHeaderSizeFromMarker;
  const std::size_t name_end = name_start + name_size;
  if (name_end < name_start || name_end > data.size()) return std::nullopt;
  std::string name = RolDecode(data.subspan(name_start, name_size), key);
  if (!IsValidStreamName(name)) return std::nullopt;

  CacheCellInfo cell;
  cell.marker_offset = marker_pos;
  cell.record_size = kHeaderSizeFromMarker + name_size;
  cell.type_id = U32At(data, marker_pos + 0x06);
  cell.logical_size = logical_size;
  cell.name = std::move(name);
  return cell;
}

[[nodiscard]] std::optional<DirectoryEntryInfo> TryDirectoryEntry(
    std::span<const uint8_t> data, std::size_t marker_pos, int key) {
  if (marker_pos + kDirectoryHeaderSize > data.size()) return std::nullopt;
  if (U32At(data, marker_pos + 0x0A) != 0 ||
      U32At(data, marker_pos + 0x12) != 0) {
    return std::nullopt;
  }

  const uint32_t name_size = U32At(data, marker_pos + 0x16);
  if (name_size == 0 || name_size >= 500) return std::nullopt;
  const std::size_t name_start = marker_pos + kDirectoryHeaderSize;
  const std::size_t name_end = name_start + name_size;
  const std::size_t record_end = name_end + kDirectoryTrailerSize;
  if (name_end < name_start || record_end < name_end ||
      record_end > data.size()) {
    return std::nullopt;
  }

  std::string name = RolDecode(data.subspan(name_start, name_size), key);
  if (!IsValidStreamName(name)) return std::nullopt;

  DirectoryEntryInfo entry;
  entry.marker_offset = marker_pos;
  entry.record_size = record_end - marker_pos;
  entry.type_id = U32At(data, marker_pos + 0x06);
  entry.stored_size = U32At(data, marker_pos + 0x0E);
  entry.name = std::move(name);
  std::copy_n(data.begin() + marker_pos + kHeaderSizeFromMarker,
              entry.descriptor.size(), entry.descriptor.begin());
  std::copy_n(data.begin() + name_end, entry.trailer.size(),
              entry.trailer.begin());
  return entry;
}

bool ClassifyIndexRecord(std::span<const uint8_t> data,
                         std::size_t marker_pos, int key,
                         ContainerInspection* inspection) {
  if (auto cell = TryCacheCell(data, marker_pos, key)) {
    if (inspection != nullptr) inspection->cache_cells.push_back(*cell);
    return true;
  }
  if (auto entry = TryDirectoryEntry(data, marker_pos, key)) {
    if (inspection != nullptr) inspection->directory_entries.push_back(*entry);
    return true;
  }
  return false;
}

void BuildCoverage(std::size_t file_size, ContainerInspection* inspection) {
  if (inspection == nullptr) return;
  inspection->file_size = file_size;

  std::vector<std::pair<std::size_t, std::size_t>> ranges;
  ranges.emplace_back(0, std::min<std::size_t>(8, file_size));
  for (const auto& stream : inspection->streams) {
    ranges.emplace_back(static_cast<std::size_t>(stream.marker_offset),
                        static_cast<std::size_t>(stream.marker_offset +
                                                 stream.record_size));
  }
  for (const auto& cell : inspection->cache_cells) {
    ranges.emplace_back(static_cast<std::size_t>(cell.marker_offset),
                        static_cast<std::size_t>(cell.marker_offset +
                                                 cell.record_size));
  }
  for (const auto& entry : inspection->directory_entries) {
    ranges.emplace_back(static_cast<std::size_t>(entry.marker_offset),
                        static_cast<std::size_t>(entry.marker_offset +
                                                 entry.record_size));
  }
  for (const auto& marker : inspection->unknown_markers) {
    ranges.emplace_back(static_cast<std::size_t>(marker.marker_offset),
                        std::min(file_size,
                                 static_cast<std::size_t>(marker.marker_offset) +
                                     kMarkerLen));
  }

  std::sort(ranges.begin(), ranges.end());
  std::size_t covered_end = 0;
  for (const auto& [raw_start, raw_end] : ranges) {
    const std::size_t start = std::min(raw_start, file_size);
    const std::size_t end = std::min(raw_end, file_size);
    if (start > covered_end) {
      inspection->unknown_regions.push_back(
          {covered_end, start - covered_end});
    }
    if (end > covered_end) covered_end = end;
  }
  if (covered_end < file_size) {
    inspection->unknown_regions.push_back(
        {covered_end, file_size - covered_end});
  }
  for (const auto& region : inspection->unknown_regions) {
    inspection->unknown_bytes += region.size;
  }
  inspection->claimed_bytes = inspection->file_size - inspection->unknown_bytes;
}

}  // namespace

bool ParseModernFormat(std::span<const uint8_t> data, StreamMap& streams,
                       ContainerInspection* inspection) {
  if (data.size() < 8) return false;

  // ROL key is stored at byte 7 of the file header.
  const int key = static_cast<int>(data[7]);

  std::size_t search_pos = 0;
  while (true) {
    // Find the next occurrence of the 6-byte marker prefix.
    auto it = std::search(data.begin() + search_pos, data.end(),
                          std::begin(kMarkerPrefix), std::end(kMarkerPrefix));
    if (it == data.end()) break;

    const std::size_t marker_pos = static_cast<std::size_t>(it - data.begin());
    StreamInfo info;
    info.marker_offset = marker_pos;
    info.type_id = U32At(data, marker_pos + 0x06);

    auto finish_unknown = [&](StreamDecodeStatus status) {
      if (inspection == nullptr) return;
      inspection->unknown_markers.push_back(
          {marker_pos, info.type_id, status, std::move(info.name)});
    };

    if (marker_pos + kHeaderSizeFromMarker > data.size()) {
      finish_unknown(StreamDecodeStatus::kTruncatedHeader);
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    info.expected_crc32 = U32At(data, marker_pos + 0x0A);
    info.compressed_size = U32At(data, marker_pos + 0x0E);
    info.uncompressed_size = U32At(data, marker_pos + 0x12);
    info.encoded_name_size = U32At(data, marker_pos + 0x16);

    // Reject obviously invalid chunk headers.
    if (info.encoded_name_size > kMaxNameSize) {
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(StreamDecodeStatus::kNameTooLarge);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }
    if (info.compressed_size > kMaxCompressedSize) {
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(StreamDecodeStatus::kCompressedSizeTooLarge);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    const std::size_t name_start = marker_pos + kHeaderSizeFromMarker;
    const std::size_t name_end = name_start + info.encoded_name_size;
    if (name_end > data.size()) {
      finish_unknown(StreamDecodeStatus::kTruncatedName);
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    info.name = RolDecode(data.subspan(name_start, info.encoded_name_size), key);

    if (!IsValidStreamName(info.name)) {
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(StreamDecodeStatus::kInvalidName);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    const std::size_t data_start = name_end;
    const std::size_t data_end = data_start + info.compressed_size;
    if (data_end < data_start || data_end > data.size()) {
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(StreamDecodeStatus::kTruncatedPayload);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    auto decompressed = InflateRaw(
        data.subspan(data_start, info.compressed_size));
    if (!decompressed) {
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(StreamDecodeStatus::kDecompressionFailed);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }

    info.decoded_size = decompressed->size();
    info.record_size = data_end - marker_pos;
    info.size_matches = info.decoded_size == info.uncompressed_size;
    const uLong actual_crc = crc32(
        crc32(0L, Z_NULL, 0), decompressed->data(),
        static_cast<uInt>(decompressed->size()));
    info.crc_matches = static_cast<uint32_t>(actual_crc) ==
                       info.expected_crc32;
    if (!info.size_matches || !info.crc_matches) {
      const StreamDecodeStatus status =
          !info.size_matches
              ? StreamDecodeStatus::kUncompressedSizeMismatch
              : StreamDecodeStatus::kCrcMismatch;
      if (!ClassifyIndexRecord(data, marker_pos, key, inspection)) {
        finish_unknown(status);
      }
      search_pos = marker_pos + kMarkerLen;
      continue;
    }
    info.duplicate_name = streams.find(info.name) != streams.end();
    if (!info.duplicate_name) {
      streams.emplace(info.name, std::move(*decompressed));
      info.selected_as_primary = true;
    }
    if (inspection != nullptr && info.duplicate_name) {
      info.decoded_data = std::move(*decompressed);
    }

    info.status = info.decoded_size == 0 ? StreamDecodeStatus::kEmpty
                                         : StreamDecodeStatus::kDecoded;
    if (inspection != nullptr) inspection->streams.push_back(std::move(info));
    search_pos = data_end;
  }

  BuildCoverage(data.size(), inspection);
  return true;
}

}  // namespace openswx::internal
