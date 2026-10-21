// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/container.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <span>
#include <utility>

#include "internal/modern_parser.h"
#include "internal/ole2_parser.h"
#include "internal/decompressor.h"
#include "internal/stream_store.h"

namespace openswx {

namespace {

Result<std::vector<uint8_t>> ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return Result<std::vector<uint8_t>>::Err("Cannot open file: " +
                                              path.string());
  }

  const auto size = file.tellg();
  if (size <= 0) {
    return Result<std::vector<uint8_t>>::Err("File is empty: " + path.string());
  }

  std::vector<uint8_t> data(static_cast<std::size_t>(size));
  file.seekg(0);
  file.read(reinterpret_cast<char*>(data.data()), size);
  if (!file) {
    return Result<std::vector<uint8_t>>::Err("Read error: " + path.string());
  }
  return Result<std::vector<uint8_t>>::Ok(std::move(data));
}

uint32_t ReadU32Le(std::span<const uint8_t> bytes, std::size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
         (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

}  // namespace

Result<ContainerInspection> InspectContainer(
    const std::filesystem::path& path) {
  auto read_result = ReadFile(path);
  if (!read_result.ok()) {
    return Result<ContainerInspection>::Err(read_result.error());
  }

  const auto& data = read_result.value();
  const std::span<const uint8_t> bytes(data);
  if (internal::IsOle2(bytes)) {
    return Result<ContainerInspection>::Err(
        "SolidWorks pre-2015 format (OLE2/CFB) is not supported");
  }
  if (internal::IsZipOpc(bytes)) {
    return Result<ContainerInspection>::Err(
        "ZIP/OPC (3DExperience) format is not supported");
  }
  if (!internal::IsModernSwx(bytes)) {
    return Result<ContainerInspection>::Err(
        "Unrecognised SolidWorks container format");
  }

  ContainerInspection inspection;
  internal::StreamMap streams;
  if (!internal::ParseModernFormat(bytes, streams, &inspection)) {
    return Result<ContainerInspection>::Err(
        "Failed to parse modern SolidWorks container");
  }
  for (auto& info : inspection.streams) {
    if (!info.selected_as_primary) continue;
    const auto stream = streams.find(info.name);
    if (stream != streams.end()) info.decoded_data = std::move(stream->second);
  }
  return Result<ContainerInspection>::Ok(std::move(inspection));
}

std::vector<EmbeddedParasolidStream> ExtractEmbeddedParasolidStreams(
    std::span<const uint8_t> decoded_stream) {
  // The marker is also used by cad readers that identify SolidWorks' embedded
  // Parasolid wrapper.  Records around it are ordinary SolidWorks serialized
  // objects and have variable size, therefore marker scanning is intentional.
  static constexpr std::array<uint8_t, 16> kMarker = {
      0x23, 0x1D, 0xD5, 0x71, 0xDA, 0x81, 0x48, 0xA2,
      0xA8, 0x58, 0x98, 0xB2, 0x1B, 0x89, 0xEF, 0x99};
  static constexpr std::size_t kHeaderSize = kMarker.size() + 8;

  std::vector<EmbeddedParasolidStream> result;
  auto cursor = decoded_stream.begin();
  while (cursor != decoded_stream.end()) {
    const auto marker =
        std::search(cursor, decoded_stream.end(), kMarker.begin(), kMarker.end());
    if (marker == decoded_stream.end()) break;

    const std::size_t offset =
        static_cast<std::size_t>(marker - decoded_stream.begin());
    if (decoded_stream.size() - offset < kHeaderSize) break;
    const uint32_t uncompressed_size =
        ReadU32Le(decoded_stream, offset + kMarker.size());
    const uint32_t compressed_size =
        ReadU32Le(decoded_stream, offset + kMarker.size() + 4);
    const std::size_t payload_offset = offset + kHeaderSize;

    if (compressed_size <= decoded_stream.size() - payload_offset) {
      const auto compressed = decoded_stream.subspan(payload_offset,
                                                     compressed_size);
      auto inflated = internal::InflateZlib(compressed, uncompressed_size);
      if (inflated && inflated->size() == uncompressed_size &&
          inflated->size() >= 4 && (*inflated)[0] == 'P' &&
          (*inflated)[1] == 'S' && (*inflated)[2] == 0 &&
          (*inflated)[3] == 0) {
        result.push_back(EmbeddedParasolidStream{
            static_cast<uint64_t>(offset), uncompressed_size, compressed_size,
            std::move(*inflated)});
        cursor = decoded_stream.begin() +
                 static_cast<std::ptrdiff_t>(payload_offset + compressed_size);
        continue;
      }
    }

    // A false-positive or damaged candidate must not hide a later valid block.
    cursor = marker + 1;
  }
  return result;
}

const char* ToString(StreamDecodeStatus status) noexcept {
  switch (status) {
    case StreamDecodeStatus::kDecoded: return "decoded";
    case StreamDecodeStatus::kEmpty: return "empty";
    case StreamDecodeStatus::kTruncatedHeader: return "truncated_header";
    case StreamDecodeStatus::kNameTooLarge: return "name_too_large";
    case StreamDecodeStatus::kCompressedSizeTooLarge:
      return "compressed_size_too_large";
    case StreamDecodeStatus::kTruncatedName: return "truncated_name";
    case StreamDecodeStatus::kInvalidName: return "invalid_name";
    case StreamDecodeStatus::kTruncatedPayload: return "truncated_payload";
    case StreamDecodeStatus::kDecompressionFailed:
      return "decompression_failed";
    case StreamDecodeStatus::kUncompressedSizeMismatch:
      return "uncompressed_size_mismatch";
    case StreamDecodeStatus::kCrcMismatch: return "crc_mismatch";
  }
  return "unknown";
}

}  // namespace openswx
