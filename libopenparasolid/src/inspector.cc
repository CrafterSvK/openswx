// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <utility>

#include "openparasolid/parser.h"

namespace openparasolid {
namespace {

constexpr std::array<uint8_t, 8> kTopologyMarker = {0xc2, 0xbc, 0x92, 0x8f,
                                                    0x99, 0x6e, 0x00, 0x00};
constexpr uint64_t kMaximumCandidates = 1000000;
constexpr std::size_t kPreviewBytes = 16;

uint16_t U16(std::span<const uint8_t> data, std::size_t at) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[at]) << 8u) |
                               static_cast<uint16_t>(data[at + 1u]));
}
uint32_t U32(std::span<const uint8_t> data, std::size_t at) {
  uint32_t value = 0;
  for (std::size_t i = 0; i < 4u; ++i) value = (value << 8u) | data[at + i];
  return value;
}
double F64(std::span<const uint8_t> data, std::size_t at) {
  uint64_t value = 0;
  for (std::size_t i = 0; i < 8u; ++i) value = (value << 8u) | data[at + i];
  return std::bit_cast<double>(value);
}
bool HasMarker(std::span<const uint8_t> data, std::size_t at) {
  return at <= data.size() && kTopologyMarker.size() <= data.size() - at &&
         std::equal(kTopologyMarker.begin(), kTopologyMarker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(at));
}

std::optional<RecordCandidate> ReadCandidate(std::span<const uint8_t> data,
                                             std::size_t offset) {
  if (data.size() - offset < 3u || data[offset] != 0u) return std::nullopt;
  RecordCandidate candidate;
  candidate.offset = offset;
  candidate.tag = data[offset + 1u];
  candidate.has_schema_marker = data[offset + 2u] == 0xffu;
  const std::size_t body =
      offset + 2u + (candidate.has_schema_marker ? 1u : 0u);
  std::size_t required = 0, prefix = 0, refs_at = 0, refs_count = 0;
  std::optional<std::size_t> sense_at;
  std::optional<std::size_t> marker_at;
  switch (candidate.tag) {
    case 0x0e:
      candidate.kind = RecordKind::kFaceUse;
      required = 37;
      prefix = 27;
      refs_at = 16;
      refs_count = 5;
      marker_at = 8;
      sense_at = 26;
      break;
    case 0x0f:
      candidate.kind = RecordKind::kLoop;
      required = prefix = 14;
      refs_at = 6;
      refs_count = 4;
      break;
    case 0x10:
      candidate.kind = RecordKind::kEdgeUse;
      required = prefix = 28;
      refs_at = 16;
      refs_count = 6;
      marker_at = 8;
      break;
    case 0x11:
      candidate.kind = RecordKind::kCoedge;
      required = prefix = 21;
      refs_at = 2;
      refs_count = 9;
      sense_at = 20;
      break;
    case 0x12:
      candidate.kind = RecordKind::kVertexUse;
      required = prefix = 24;
      refs_at = 6;
      refs_count = 5;
      marker_at = 16;
      break;
    case 0x1d:
      candidate.kind = RecordKind::kPoint;
      required = prefix = 38;
      refs_at = 6;
      refs_count = 4;
      break;
    default:
      return std::nullopt;
  }
  if (required > data.size() - body) return std::nullopt;
  candidate.attribute = U16(data, body);
  if (candidate.attribute <= 1u || candidate.attribute >= 0x8000u)
    return std::nullopt;
  if (marker_at && !HasMarker(data, body + *marker_at)) return std::nullopt;
  if (sense_at) {
    const auto marker = data[body + *sense_at];
    if (marker != '+' && marker != '-') return std::nullopt;
    candidate.sense = static_cast<char>(marker);
  }
  if (candidate.kind != RecordKind::kCoedge &&
      candidate.kind != RecordKind::kLoop)
    candidate.sequence = U32(data, body + 2u);
  for (std::size_t i = 0; i < refs_count; ++i) {
    RecordReference reference;
    reference.field = "ref" + std::to_string(i);
    reference.offset = body + refs_at + i * 2u;
    reference.attribute = U16(data, static_cast<std::size_t>(reference.offset));
    // Extended and tripled pointer layouts remain uninterpreted in this stage.
    if (reference.attribute >= 0x8000u) return std::nullopt;
    const auto target = [&](std::string name, RecordKind kind) {
      reference.field = std::move(name);
      reference.expected_kind = kind;
    };
    switch (candidate.kind) {
      case RecordKind::kFaceUse:
        if (i == 2u) target("loop_head", RecordKind::kLoop);
        if (i == 4u) reference.field = "surface";
        break;
      case RecordKind::kLoop:
        if (i == 1u) target("first_coedge", RecordKind::kCoedge);
        if (i == 2u) target("face_use", RecordKind::kFaceUse);
        if (i == 3u) target("next_loop", RecordKind::kLoop);
        break;
      case RecordKind::kEdgeUse:
        if (i == 0u) target("canonical_coedge", RecordKind::kCoedge);
        if (i == 3u) reference.field = "curve";
        break;
      case RecordKind::kCoedge:
        if (i == 1u) target("loop", RecordKind::kLoop);
        if (i == 2u) target("previous", RecordKind::kCoedge);
        if (i == 3u) target("next", RecordKind::kCoedge);
        if (i == 4u) target("vertex_use", RecordKind::kVertexUse);
        if (i == 5u) target("twin", RecordKind::kCoedge);
        if (i == 6u) target("edge_use", RecordKind::kEdgeUse);
        break;
      case RecordKind::kVertexUse:
        if (i == 4u) target("point", RecordKind::kPoint);
        break;
      case RecordKind::kPoint:
        break;
    }
    candidate.references.push_back(std::move(reference));
  }
  if (candidate.kind == RecordKind::kPoint) {
    if (candidate.references[0].attribute > 1u) return std::nullopt;
    const std::array<double, 3> xyz{
        F64(data, body + 14u), F64(data, body + 22u), F64(data, body + 30u)};
    if (!std::all_of(xyz.begin(), xyz.end(),
                     [](double v) { return std::isfinite(v); }))
      return std::nullopt;
    candidate.position = xyz;
  }
  candidate.prefix_size = body + prefix - offset;
  return candidate;
}

void AddUnknown(StreamInspection &inspection, std::span<const uint8_t> data,
                std::size_t begin, std::size_t end) {
  if (begin == end) return;
  UnknownRegion region;
  region.offset = begin;
  region.size = end - begin;
  const auto preview =
      data.subspan(begin, std::min(kPreviewBytes, end - begin));
  region.preview.assign(preview.begin(), preview.end());
  inspection.uninterpreted_bytes += region.size;
  inspection.unknown_regions.push_back(std::move(region));
}

void ResolveReferences(StreamInspection &inspection) {
  using Key = std::pair<RecordKind, uint16_t>;
  std::map<Key, std::vector<uint64_t>> index;
  for (std::size_t i = 0; i < inspection.candidates.size(); ++i) {
    const auto &candidate = inspection.candidates[i];
    index[{candidate.kind, candidate.attribute}].push_back(i);
  }
  for (auto &candidate : inspection.candidates) {
    for (auto &reference : candidate.references) {
      if (reference.attribute <= 1u)
        reference.status = ReferenceStatus::kSentinel;
      else if (!reference.expected_kind)
        reference.status = ReferenceStatus::kUninterpretedTarget;
      else {
        const auto found =
            index.find({*reference.expected_kind, reference.attribute});
        if (found == index.end())
          reference.status = ReferenceStatus::kUnresolved;
        else if (found->second.size() != 1u)
          reference.status = ReferenceStatus::kAmbiguous;
        else {
          reference.status = ReferenceStatus::kUniqueCandidate;
          reference.target_candidate = found->second.front();
        }
      }
    }
  }
}

}  // namespace

Result<StreamInspection> ParasolidInspector::Inspect(
    std::span<const uint8_t> data, const InspectOptions &options) {
  if (options.max_input_bytes == 0u || options.max_records > kMaximumCandidates)
    return Result<StreamInspection>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Input limit must be positive and max_records must not exceed "
         "1000000"});
  if (data.size() > options.max_input_bytes)
    return Result<StreamInspection>::Err(
        {ErrorCode::kResourceLimit, 0, "Input exceeds max_input_bytes"});
  auto header = ParasolidReader::ReadHeader(data);
  if (!header.ok()) return Result<StreamInspection>::Err(header.error());
  StreamInspection inspection;
  inspection.header = std::move(header).value();
  inspection.byte_size = data.size();
  std::size_t at = static_cast<std::size_t>(inspection.header.body_offset);
  std::size_t unknown_begin = at;
  if (inspection.header.schema_version &&
      (inspection.header.schema_version->base_schema == 13006u ||
       (!inspection.header.schema_version->base_schema &&
        inspection.header.schema_version->schema == 13006u)) &&
      inspection.header.internal_header_complete &&
      inspection.header.user_field_words == 0u) {
    inspection.record_scan_performed = true;
    while (at < data.size()) {
      if (inspection.candidates.size() >= options.max_records) {
        inspection.notes.push_back(
            "Record candidate limit reached; remaining "
            "body bytes were not scanned");
        break;
      }
      auto candidate = ReadCandidate(data, at);
      if (!candidate) {
        ++at;
        continue;
      }
      AddUnknown(inspection, data, unknown_begin, at);
      at += static_cast<std::size_t>(candidate->prefix_size);
      inspection.candidate_prefix_bytes += candidate->prefix_size;
      inspection.candidates.push_back(std::move(*candidate));
      unknown_begin = at;
    }
    inspection.record_scan_complete = at == data.size();
  } else {
    inspection.notes.push_back(
        "Schema or user-field layout has no supported record candidate "
        "layouts");
  }
  AddUnknown(inspection, data, unknown_begin, data.size());
  ResolveReferences(inspection);
  return Result<StreamInspection>::Ok(std::move(inspection));
}

Result<StreamInspection> ParasolidInspector::InspectFile(
    const std::filesystem::path &path, const InspectOptions &options) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input.is_open())
    return Result<StreamInspection>::Err(
        {ErrorCode::kIo, 0, "Cannot open input file"});
  const auto size = input.tellg();
  if (size < 0)
    return Result<StreamInspection>::Err(
        {ErrorCode::kIo, 0, "Cannot determine input size"});
  if (static_cast<uint64_t>(size) > options.max_input_bytes ||
      static_cast<uint64_t>(size) > std::numeric_limits<std::size_t>::max())
    return Result<StreamInspection>::Err(
        {ErrorCode::kResourceLimit, 0, "Input exceeds max_input_bytes"});
  std::vector<uint8_t> bytes(static_cast<std::size_t>(size));
  input.seekg(0);
  if (!bytes.empty()) input.read(reinterpret_cast<char *>(bytes.data()), size);
  if (!input)
    return Result<StreamInspection>::Err(
        {ErrorCode::kIo, 0, "Cannot read input file"});
  return Inspect(bytes, options);
}

}  // namespace openparasolid
