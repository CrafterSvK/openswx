// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/container.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <span>
#include <string_view>
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

namespace {

std::vector<EmbeddedParasolidStream>
ScanParasolidBlocks(std::span<const uint8_t> decoded_stream,
                    std::vector<ParasolidExtractionIssue> *issues,
                    std::string_view source_stream, uint64_t *wrapper_count,
                    uint64_t *non_parasolid_block_count = nullptr) {
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
    if (wrapper_count)
      ++*wrapper_count;
    const auto issue = [&](std::string message) {
      if (issues)
        issues->push_back(
            {std::string(source_stream), offset, std::move(message)});
    };
    if (decoded_stream.size() - offset < kHeaderSize) {
      issue("truncated_wrapper_header");
      break;
    }
    constexpr std::size_t kMaximumStreamSize = 128u * 1024u * 1024u;
    const std::size_t frame_start = offset + kMarker.size();
    std::size_t section_end = decoded_stream.size();
    bool chained = false;
    if (offset >= 4u) {
      const std::size_t chain_length = ReadU32Le(decoded_stream, offset - 4u);
      if (chain_length >= kHeaderSize &&
          (chain_length <= decoded_stream.size() - offset || offset == 4u)) {
        if (chain_length > decoded_stream.size() - offset) {
          issue("truncated_chain_section");
          cursor = marker + 1;
          continue;
        }
        chained = true;
        section_end = offset + chain_length;
      }
    }
    EmbeddedParasolidStream stream;
    stream.marker_offset = offset;
    if (chained)
      stream.section_offset = offset - 4u;
    std::size_t frame_at = frame_start;
    bool valid = true;
    while (frame_at < section_end) {
      const auto remaining =
          decoded_stream.subspan(frame_at, section_end - frame_at);
      if (chained && std::all_of(remaining.begin(), remaining.end(),
                                 [](uint8_t byte) { return byte == 0u; })) {
        frame_at = section_end;
        break;
      }
      if (remaining.size() < 8u) {
        issue("truncated_frame_header");
        valid = false;
        break;
      }
      const uint32_t uncompressed_size = ReadU32Le(decoded_stream, frame_at);
      const uint32_t compressed_size = ReadU32Le(decoded_stream, frame_at + 4u);
      const std::size_t payload_offset = frame_at + 8u;
      if (compressed_size == 0u || uncompressed_size == 0u) {
        issue("empty_frame");
        valid = false;
        break;
      }
      if (compressed_size > section_end - payload_offset) {
        issue("truncated_wrapper_payload");
        valid = false;
        break;
      }
      if (uncompressed_size > kMaximumStreamSize - stream.data.size()) {
        issue("stream_size_limit");
        valid = false;
        break;
      }
      const auto compressed =
          decoded_stream.subspan(payload_offset, compressed_size);
      std::size_t consumed_size = 0;
      auto inflated =
          internal::InflateZlib(compressed, uncompressed_size, &consumed_size);
      if (!inflated) {
        issue("decompression_failed_or_size_limit");
        valid = false;
        break;
      }
      if (consumed_size != compressed_size) {
        issue("compressed_size_mismatch");
        valid = false;
        break;
      }
      if (inflated->size() != uncompressed_size) {
        issue("uncompressed_size_mismatch");
        valid = false;
        break;
      }
      stream.frames.push_back({frame_at, uncompressed_size, compressed_size});
      stream.uncompressed_size += uncompressed_size;
      stream.compressed_size += compressed_size;
      stream.data.insert(stream.data.end(), inflated->begin(), inflated->end());
      frame_at = payload_offset + compressed_size;
      if (!chained)
        break;
    }
    if (valid && stream.frames.empty())
      issue("empty_chain_section");
    if (valid && !stream.frames.empty()) {
      stream.stored_size = frame_at - stream.section_offset.value_or(offset);
      if (stream.data.size() >= 4u && stream.data[0] == 'P' &&
          stream.data[1] == 'S' && stream.data[2] == 0u &&
          stream.data[3] == 0u) {
        result.push_back(std::move(stream));
      } else if (non_parasolid_block_count) {
        // The wrapper also carries unrelated serialized SolidWorks objects.
        ++*non_parasolid_block_count;
      }
      cursor = decoded_stream.begin() + static_cast<std::ptrdiff_t>(frame_at);
      continue;
    }

    // A false-positive or damaged candidate must not hide a later valid block.
    cursor = marker + 1;
  }
  return result;
}

bool ReadParasolidHeader(ParasolidPayload &result) {
  const std::span<const uint8_t> data(result.payload.data);
  if (data.size() < 6u)
    return false;
  const std::size_t length =
      (static_cast<std::size_t>(data[4]) << 8u) | data[5];
  if (length == 0u || length > data.size() - 6u)
    return false;
  const auto description_bytes = data.subspan(6u, length);
  if (!std::all_of(description_bytes.begin(), description_bytes.end(),
                   [](uint8_t c) { return c >= 0x20u && c <= 0x7eu; }))
    return false;
  std::string description(description_bytes.begin(), description_bytes.end());
  if (!description.starts_with(": TRANSMIT FILE"))
    return false;
  const std::size_t description_end = 6u + length;
  const std::size_t window_end =
      description_end +
      std::min<std::size_t>(64u, data.size() - description_end);
  std::string schema;
  // Padding is variable. A schema is bounded by its preceding u8 length,
  // including when the first body byte is an ASCII token character.
  for (std::size_t at = description_end; at < window_end; ++at) {
    const std::size_t schema_length = data[at];
    if (schema_length <= 4u || schema_length > window_end - at - 1u)
      continue;
    const auto token = data.subspan(at + 1u, schema_length);
    if (token[0] != 'S' || token[1] != 'C' || token[2] != 'H' ||
        token[3] != '_')
      continue;
    if (!std::all_of(token.begin(), token.end(), [](uint8_t c) {
          return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') || c == '_';
        }))
      continue;
    schema.assign(token.begin(), token.end());
    break;
  }
  if (schema.empty())
    return false;
  result.description = std::move(description);
  result.schema = std::move(schema);
  if (result.description.starts_with(": TRANSMIT FILE (partition)"))
    result.kind = ParasolidPayloadKind::kPartition;
  else if (result.description.starts_with(": TRANSMIT FILE (deltas)"))
    result.kind = ParasolidPayloadKind::kDeltas;
  else if (result.description.starts_with(": TRANSMIT FILE created"))
    result.kind = ParasolidPayloadKind::kTransmit;
  return true;
}

std::optional<uint32_t> ConfigurationIndex(std::string_view name) {
  constexpr std::string_view prefix = "Contents/Config-";
  if (!name.starts_with(prefix))
    return std::nullopt;
  name.remove_prefix(prefix.size());
  const auto separator = name.find('-');
  if (separator == std::string_view::npos || separator == 0u)
    return std::nullopt;
  uint32_t index = 0;
  const auto end = name.data() + separator;
  const auto parsed = std::from_chars(name.data(), end, index);
  if (parsed.ec != std::errc{} || parsed.ptr != end)
    return std::nullopt;
  return index;
}

} // namespace

std::vector<EmbeddedParasolidStream>
ExtractEmbeddedParasolidStreams(std::span<const uint8_t> decoded_stream) {
  return ScanParasolidBlocks(decoded_stream, nullptr, {}, nullptr);
}

ParasolidExtraction
ExtractParasolidStreams(const ContainerInspection &inspection) {
  ParasolidExtraction result;
  for (const auto &stream : inspection.streams) {
    if (stream.status != StreamDecodeStatus::kDecoded &&
        stream.status != StreamDecodeStatus::kEmpty) {
      result.issues.push_back(
          {stream.name, std::nullopt,
           std::string("container_stream_") + ToString(stream.status)});
      continue;
    }
    if (!stream.selected_as_primary)
      continue;
    ++result.inspected_stream_count;
    for (auto &block : ScanParasolidBlocks(stream.decoded_data, &result.issues,
                                           stream.name, &result.wrapper_count,
                                           &result.non_parasolid_block_count)) {
      ParasolidPayload payload;
      payload.source_stream = stream.name;
      payload.container_marker_offset = stream.marker_offset;
      payload.configuration_index = ConfigurationIndex(stream.name);
      payload.ghost_partition = stream.name.ends_with("-GhostPartition");
      payload.payload = std::move(block);
      payload.header_valid = ReadParasolidHeader(payload);
      if (!payload.header_valid)
        result.issues.push_back({stream.name, payload.payload.marker_offset,
                                 "unrecognised_parasolid_header"});
      result.payloads.push_back(std::move(payload));
    }
  }
  return result;
}

const char *ToString(ParasolidPayloadKind kind) noexcept {
  switch (kind) {
  case ParasolidPayloadKind::kTransmit:
    return "transmit";
  case ParasolidPayloadKind::kPartition:
    return "partition";
  case ParasolidPayloadKind::kDeltas:
    return "deltas";
  case ParasolidPayloadKind::kUnknown:
    return "unknown";
  }
  return "unknown";
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
