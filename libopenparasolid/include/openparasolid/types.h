// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace openparasolid {

enum class StreamKind { kTransmit, kPartition, kDeltas, kUnknown };

struct SchemaVersion {
  uint32_t modeller = 0;
  uint32_t schema = 0;
  std::optional<uint32_t> base_schema;
};

struct StreamHeader {
  StreamKind kind = StreamKind::kUnknown;
  std::string description;
  std::string schema;
  std::optional<SchemaVersion> schema_version;
  uint64_t description_offset = 6;
  uint64_t schema_offset = 0;
  uint64_t schema_end_offset = 0;
  uint64_t body_offset = 0;
  std::optional<uint16_t> schema_max_type;
  std::optional<uint8_t> user_field_words;
  bool internal_header_complete = false;
};

enum class RecordKind {
  kFaceUse,
  kLoop,
  kEdgeUse,
  kCoedge,
  kVertexUse,
  kPoint
};
enum class ReferenceStatus {
  kSentinel,
  kUniqueCandidate,
  kAmbiguous,
  kUnresolved,
  kUninterpretedTarget
};

struct RecordReference {
  std::string field;
  uint64_t offset = 0;
  uint16_t attribute = 0;
  std::optional<RecordKind> expected_kind;
  ReferenceStatus status = ReferenceStatus::kUninterpretedTarget;
  std::optional<uint64_t> target_candidate;
};

/// A syntactically matched prefix, not a proven entity or full record boundary.
/// IDs and reference resolution are scoped to this input stream only.
struct RecordCandidate {
  uint64_t offset = 0;
  uint64_t prefix_size = 0;
  uint16_t tag = 0;
  uint16_t attribute = 0;
  RecordKind kind = RecordKind::kPoint;
  bool has_schema_marker = false;
  std::optional<uint32_t> sequence;
  std::optional<char> sense;
  std::optional<std::array<double, 3>> position;
  std::vector<RecordReference> references;
};

struct UnknownRegion {
  uint64_t offset = 0;
  uint64_t size = 0;
  std::vector<uint8_t> preview;
};

struct InspectOptions {
  uint64_t max_input_bytes = 128u * 1024u * 1024u;
  uint64_t max_records = 10000;
};

struct StreamInspection {
  StreamHeader header;
  uint64_t byte_size = 0;
  bool record_scan_performed = false;
  bool record_scan_complete = false;
  uint64_t candidate_prefix_bytes = 0;
  uint64_t uninterpreted_bytes = 0;
  std::vector<RecordCandidate> candidates;
  /// Disjoint ranges after the header and outside candidate prefixes.
  /// Original bytes can be recovered from the input with these offsets/sizes.
  std::vector<UnknownRegion> unknown_regions;
  std::vector<std::string> notes;
};

[[nodiscard]] const char *ToString(StreamKind kind) noexcept;
[[nodiscard]] const char *ToString(RecordKind kind) noexcept;
[[nodiscard]] const char *ToString(ReferenceStatus status) noexcept;

}  // namespace openparasolid
