// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

/**
 * @file
 * @brief Stream headers and conservative record inspection data.
 * @see @ref xt_header "XT 3.1: Common header (p. 19)", @ref xt_binary "XT 3.3:
 * Binary (p. 22)", @ref xt_terminology "XT 1.1.1: Parasolid XT format
 * terminology (p. 7)"
 */
#ifndef OPENPARASOLID_TYPES_H_
#define OPENPARASOLID_TYPES_H_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/**
 * @brief Readers and exact B-rep queries for extracted binary Parasolid
 * streams.
 *
 * Input spans, models and catalogs are borrowed for the duration of a call.
 * Reports and bindings own their returned data unless explicitly documented
 * as references or pointers. Node indices are scoped to their source model or
 * logical delta state; mixing identities from unrelated models is invalid.
 * Result-returning queries require checking ok() before value()/error().
 * Geometry stays in source coordinates and units; no placement is implicit.
 * @see @ref xt_reference
 */
namespace openparasolid {

/// @brief Document kind reported by the extracted stream prologue.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
enum class StreamKind {
  /// Ordinary part transmit stream.
  kTransmit,
  /// Partition transmit stream.
  kPartition,
  /// Internal delta/history stream; automatic history application is not
  /// implied.
  kDeltas,
  /// Unrecognized stream kind.
  kUnknown,
};

/// @brief Modeller/schema numbers parsed from the source schema key.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct SchemaVersion {
  /// Modeller version number extracted from the schema key.
  uint32_t modeller = 0;
  /// Schema version number extracted from the schema key.
  uint32_t schema = 0;
  /// Optional base schema version declared by the source key.
  std::optional<uint32_t> base_schema;
};

/// @brief Decoded prologue with byte offsets relative to the input stream.
///
/// @see @ref xt_header "XT 3.1: Common header (p. 19)", @ref xt_binary "XT 3.3:
/// Binary (p. 22)"
struct StreamHeader {
  /// Recognized stream document kind.
  StreamKind kind = StreamKind::kUnknown;
  /// Source descriptive text.
  std::string description;
  /// Exact source schema key text.
  std::string schema;
  /// Optional parsed modeller/schema numbers from the schema key.
  std::optional<SchemaVersion> schema_version;
  /// Input byte offset where the stream description starts.
  uint64_t description_offset = 6;
  /// Input byte offset where the schema key starts.
  uint64_t schema_offset = 0;
  /// Input byte offset immediately after the schema key.
  uint64_t schema_end_offset = 0;
  /// Input byte offset where body decoding or scanning starts.
  uint64_t body_offset = 0;
  /// Optional declared maximum numeric node type.
  std::optional<uint16_t> schema_max_type;
  /// Optional declared internal user-field word count.
  std::optional<uint8_t> user_field_words;
  /// Whether the supported internal header fields were fully read.
  bool internal_header_complete = false;
};

/// @brief Library classification of conservative fixed-prefix candidates.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
enum class RecordKind {
  /// Candidate face-use prefix.
  kFaceUse,
  /// Candidate loop prefix.
  kLoop,
  /// Candidate edge-use prefix.
  kEdgeUse,
  /// Candidate coedge prefix; not a certified FIN node.
  kCoedge,
  /// Candidate vertex-use prefix.
  kVertexUse,
  /// Candidate point prefix.
  kPoint,
};
/// @brief Confidence of candidate matching; not verified schema pointer
/// resolution.
///
/// @see @ref xt_pointers "XT 2.1.4.2: Pointer classes (p. 16)"
enum class ReferenceStatus {
  /// Recognized sentinel with no candidate target.
  kSentinel,
  /// Exactly one compatible candidate was found.
  kUniqueCandidate,
  /// Multiple compatible candidates were found.
  kAmbiguous,
  /// No compatible candidate was found.
  kUnresolved,
  /// Target encoding was retained without interpretation.
  kUninterpretedTarget,
};

/// @brief Unverified reference extracted from a candidate prefix.
///
/// @see @ref xt_pointers "XT 2.1.4.2: Pointer classes (p. 16)"
struct RecordReference {
  /// Source field name associated with this reference or finding.
  std::string field;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Raw prefix/reference attribute word used by candidate matching.
  uint16_t attribute = 0;
  /// Optional expected candidate classification, not a verified pointer class.
  std::optional<RecordKind> expected_kind;
  /// Conservative candidate-reference matching outcome.
  ReferenceStatus status = ReferenceStatus::kUninterpretedTarget;
  /// Optional zero-based index in StreamInspection::candidates.
  std::optional<uint64_t> target_candidate;
};

/// @brief Conservative byte-pattern candidate, not a decoded XT node.
///
/// A syntactically matched prefix, not a proven entity or full record boundary.
/// IDs and reference resolution are scoped to this input stream only.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct RecordCandidate {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Matched candidate prefix size; does not imply a full node boundary.
  uint64_t prefix_size = 0;
  /// Raw matched prefix tag.
  uint16_t tag = 0;
  /// Raw prefix/reference attribute word used by candidate matching.
  uint16_t attribute = 0;
  /// Library classification of this matched prefix.
  RecordKind kind = RecordKind::kPoint;
  /// Whether the matched prefix contains the recognized schema marker.
  bool has_schema_marker = false;
  /// Optional raw candidate sequence number.
  std::optional<uint32_t> sequence;
  /// Optional raw candidate orientation byte; not verified topology.
  std::optional<char> sense;
  /// Position in source coordinates and source length units.
  std::optional<std::array<double, 3>> position;
  /// Conservative pointer-prefix matches, without schema verification.
  std::vector<RecordReference> references;
};

/// @brief Uninterpreted byte range retained by conservative inspection.
///
/// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
struct UnknownRegion {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Owned short byte preview; full bytes remain in the caller's input.
  std::vector<uint8_t> preview;
};

/// @brief Per-call input and candidate budgets for conservative inspection.
///
/// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
struct InspectOptions {
  /// Upper bound on input bytes.
  uint64_t max_input_bytes = 128u * 1024u * 1024u;
  /// Upper bound on candidate records.
  uint64_t max_records = 10000;
};

/// @brief Owned conservative inspection report, including undecoded byte
/// ranges.
///
/// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
struct StreamInspection {
  /// Decoded input prologue.
  StreamHeader header;
  /// Total input size in bytes.
  uint64_t byte_size = 0;
  /// Whether conservative candidate scanning was attempted.
  bool record_scan_performed = false;
  /// Whether candidate scanning completed within its budget.
  bool record_scan_complete = false;
  /// Total bytes covered by matched prefixes, not full decoded records.
  uint64_t candidate_prefix_bytes = 0;
  /// Total body bytes outside recognized candidate prefixes.
  uint64_t uninterpreted_bytes = 0;
  /// Owned conservative prefix matches in input order.
  std::vector<RecordCandidate> candidates;
  /// Disjoint ranges after the header and outside candidate prefixes.
  /// Original bytes can be recovered from the input with these offsets/sizes.
  std::vector<UnknownRegion> unknown_regions;
  /// Human-readable inspection limitations and observations.
  std::vector<std::string> notes;
};

/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param kind Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char *ToString(StreamKind kind) noexcept;
/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param kind Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char *ToString(RecordKind kind) noexcept;
/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param status Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char *ToString(ReferenceStatus status) noexcept;

}  // namespace openparasolid

#endif  // OPENPARASOLID_TYPES_H_
