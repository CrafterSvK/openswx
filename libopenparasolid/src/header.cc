// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <algorithm>
#include <charconv>
#include <string_view>

#include "openparasolid/parser.h"

namespace openparasolid {
namespace {

std::optional<SchemaVersion> ParseVersion(std::string_view schema) {
  schema.remove_prefix(4u);
  std::array<uint32_t, 3> values{};
  std::size_t count = 0;
  while (!schema.empty() && count < values.size()) {
    const auto separator = schema.find('_');
    const auto part = schema.substr(0, separator);
    const auto parsed =
        std::from_chars(part.data(), part.data() + part.size(), values[count]);
    if (part.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != part.data() + part.size())
      return std::nullopt;
    ++count;
    if (separator == std::string_view::npos) {
      schema = {};
      break;
    }
    schema.remove_prefix(separator + 1u);
    if (schema.empty()) return std::nullopt;
  }
  if (!schema.empty() || count < 2u) return std::nullopt;
  return SchemaVersion{
      values[0], values[1],
      count == 3u ? std::optional<uint32_t>(values[2]) : std::nullopt};
}

}  // namespace

Result<StreamHeader> ParasolidReader::ReadCommonHeader(
    std::span<const uint8_t> data) {
  const auto invalid = [](uint64_t offset, std::string message) {
    return Result<StreamHeader>::Err(
        {ErrorCode::kInvalidHeader, offset, std::move(message)});
  };
  if (data.size() < 4u || data[0] != 'P' || data[1] != 'S' || data[2] != 0u ||
      data[3] != 0u)
    return Result<StreamHeader>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Expected an extracted binary Parasolid PS\\0\\0 stream"});
  if (data.size() < 6u) return invalid(4u, "Truncated description length");
  const std::size_t length =
      (static_cast<std::size_t>(data[4]) << 8u) | data[5];
  if (length == 0u || length > data.size() - 6u)
    return invalid(4u, "Description length exceeds the input or is zero");
  const auto description = data.subspan(6u, length);
  if (!std::all_of(description.begin(), description.end(),
                   [](uint8_t c) { return c >= 0x20u && c <= 0x7eu; }))
    return invalid(6u, "Description contains non-printable bytes");
  StreamHeader header;
  header.description.assign(description.begin(), description.end());
  if (!header.description.starts_with(": TRANSMIT FILE"))
    return invalid(6u, "Unrecognised transmit description");
  if (header.description.starts_with(": TRANSMIT FILE (partition)"))
    header.kind = StreamKind::kPartition;
  else if (header.description.starts_with(": TRANSMIT FILE (deltas)"))
    header.kind = StreamKind::kDeltas;
  else if (header.description.starts_with(": TRANSMIT FILE created"))
    header.kind = StreamKind::kTransmit;

  const std::size_t description_end = 6u + length;
  const std::size_t window_end =
      description_end +
      std::min<std::size_t>(64u, data.size() - description_end);
  for (std::size_t at = description_end; at < window_end; ++at) {
    const std::size_t schema_length = data[at];
    if (schema_length <= 4u || schema_length > window_end - at - 1u) continue;
    const auto token = data.subspan(at + 1u, schema_length);
    if (token[0] != 'S' || token[1] != 'C' || token[2] != 'H' ||
        token[3] != '_')
      continue;
    if (!std::all_of(token.begin(), token.end(), [](uint8_t c) {
          return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') || c == '_';
        }))
      continue;
    header.schema.assign(token.begin(), token.end());
    header.schema_offset = at + 1u;
    header.schema_end_offset = at + 1u + schema_length;
    header.body_offset = header.schema_end_offset;
    header.schema_version = ParseVersion(header.schema);
    return Result<StreamHeader>::Ok(std::move(header));
  }
  return invalid(
      description_end,
      "No complete length-prefixed schema within the 64-byte header window");
}

Result<StreamHeader> ParasolidReader::ReadHeader(
    std::span<const uint8_t> data) {
  auto common = ReadCommonHeader(data);
  if (!common.ok()) return common;
  StreamHeader header = std::move(common).value();
  if (!header.schema_version)
    return Result<StreamHeader>::Ok(std::move(header));
  std::size_t at = static_cast<std::size_t>(header.schema_end_offset);
  if (header.schema_version->base_schema) {
    if (data.size() - at < 2u)
      return Result<StreamHeader>::Err(
          {ErrorCode::kInvalidHeader, at, "Truncated maximum schema type"});
    header.schema_max_type = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[at]) << 8u) | data[at + 1u]);
    at += 2u;
  }
  if (data.size() - at < 4u)
    return Result<StreamHeader>::Err(
        {ErrorCode::kInvalidHeader, at, "Truncated user-field word count"});
  uint32_t words = 0;
  for (std::size_t i = 0; i < 4u; ++i) words = (words << 8u) | data[at + i];
  if (words > 16u)
    return Result<StreamHeader>::Err(
        {ErrorCode::kInvalidHeader, at,
         "User-field word count must be between 0 and 16"});
  header.user_field_words = static_cast<uint8_t>(words);
  header.body_offset = at + 4u;
  header.internal_header_complete = true;
  return Result<StreamHeader>::Ok(std::move(header));
}

}  // namespace openparasolid
