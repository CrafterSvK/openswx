// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <filesystem>
#include <span>
#include <variant>

#include "openparasolid/result.h"
#include "openparasolid/types.h"

namespace openparasolid {

enum class FieldType : char {
  kByte = 'u',
  kCharacter = 'c',
  kLogical = 'l',
  kShort = 'n',
  kUnicode = 'w',
  kInteger = 'd',
  kPointer = 'p',
  kOpaque = 'q',
  kTag = 't',
  kDouble = 'f',
  kInterval = 'i',
  kVector = 'v',
  kBox = 'b',
  kIntersectionPoint = 'h'
};

struct FieldDefinition {
  std::string name;
  FieldType type = FieldType::kInteger;
  uint16_t pointer_class = 0;
  /// 0: scalar, 1: per-node variable count, >=2: fixed array count.
  uint32_t elements = 0;
  bool transmitted = true;
};

struct TypeDefinition {
  uint16_t node_type = 0;
  std::string name;
  std::string description;
  std::vector<FieldDefinition> fields;
};

/// Exact input key and explicit base membership. Omission means unknown,
/// never absent. Only absent_types may carry a complete embedded definition.
struct SchemaCatalog {
  std::string schema_key;
  std::string source;
  std::vector<TypeDefinition> base_types;
  std::vector<uint16_t> absent_types;
};

enum class SchemaSource { kBase, kUnchanged, kEdited, kFull };
struct SchemaEdit {
  char opcode = 'Z';
  uint64_t offset = 0;
  std::optional<FieldDefinition> field;
};
struct ResolvedSchema {
  TypeDefinition definition;
  SchemaSource source = SchemaSource::kBase;
  uint64_t offset = 0;
  uint64_t size = 0;
  std::vector<SchemaEdit> edits;
};

/// Integer/pointer values retain their decoded transmit meaning. monostate
/// represents a wire unset sentinel. Composite fields flatten their scalar
/// components in element order; the field's components value states the stride.
using ScalarValue =
    std::variant<std::monostate, bool, int64_t, uint64_t, double>;
struct DecodedField {
  uint64_t definition_index = 0;
  uint64_t offset = 0;
  uint64_t size = 0;
  uint32_t elements = 0;
  uint8_t components = 1;
  std::vector<ScalarValue> values;
  /// One status byte per pointer value in status-framed delta fields.
  std::vector<uint8_t> reference_statuses;
};
struct DecodedNode {
  uint16_t node_type = 0;
  uint32_t index = 0;
  uint64_t schema_index = 0;
  uint64_t offset = 0;
  uint64_t size = 0;
  std::optional<uint32_t> variable_length;
  bool status_framed = false;
  std::vector<DecodedField> fields;
};
struct SchemaLimits {
  uint64_t max_input_bytes = 128u * 1024u * 1024u;
  uint64_t max_nodes = 10000;
  uint64_t max_types = 1024;
  uint64_t max_fields_per_type = 4096;
  /// Applied separately to catalog fields, resolved fields and decoded fields.
  uint64_t max_total_fields = 65536;
  uint64_t max_values = 1000000;
  uint64_t max_delta_packets = 10000;
  uint64_t max_delta_entries = 100000;
};
struct SchemaDiagnostic {
  ParseError error;
  std::optional<uint16_t> node_type;
};
/// Stream-local indices use decoded compact encoding (zero is null).
struct DeltaPartition {
  uint64_t offset = 0;
  uint32_t index = 0;
  uint32_t current_pmark = 0;
  uint32_t highest_id = 0;
};
struct DeltaMark {
  uint64_t offset = 0;
  uint32_t index = 0;
  uint32_t preceding = 0;
  uint32_t first_following = 0;
  uint32_t next_sibling = 0;
  uint32_t prev_sibling = 0;
  uint32_t n_new_nodes = 0;
  uint32_t n_del_nodes = 0;
  uint32_t n_copy_mod_nodes = 0;
  uint32_t delta_key = 0;
  bool delta_is_forward = false;
  uint32_t identifier = 0;
};
/// Validated leading PARTITION/current-PMARK pair; no history is applied.
/// Persistent IDs, counters and keys preserve all 32 wire bits.
struct DeltaPrefix {
  uint64_t offset = 0;
  uint64_t size = 0;
  DeltaPartition partition;
  DeltaMark current_mark;
};
enum class DeltaPacketKind { kReferenceMap, kHistoricalMarks };
struct DeltaReferenceEntry {
  uint64_t offset = 0;
  uint64_t size = 0;
  uint32_t index = 0;
  uint16_t node_type = 0;
};
struct DeltaMarkOccurrence {
  DeltaMark mark;
  uint64_t size = 0;
};
/// Occurrences retain source order and repeated identities; no state is
/// applied.
struct DeltaPacket {
  DeltaPacketKind kind = DeltaPacketKind::kReferenceMap;
  uint64_t offset = 0;
  uint64_t size = 0;
  bool has_terminal = false;
  /// Raw map terminal word, or the supported historical terminal value 0/1.
  std::optional<uint16_t> terminal_word;
  std::vector<DeltaReferenceEntry> references;
  std::vector<DeltaMarkOccurrence> marks;
};
class DeltaPacketReader {
 public:
  /// Reads only at an established packet boundary. Reference maps require a
  /// null-reference/zero/nonzero-word terminal. PMARK packets use the already
  /// established base layout and optionally end with the ten-byte historical
  /// terminal (final value 0 or 1). Ambiguous interpretations are unsupported.
  [[nodiscard]] static Result<DeltaPacket> Read(
      std::span<const uint8_t> data, uint64_t offset,
      uint16_t schema_max_type = 32767, uint64_t max_entries = 100000);
};
struct SchemaParseReport {
  StreamHeader header;
  std::string catalog_source;
  bool complete = false;
  /// End of fully decoded nodes and packets; final terminator is separate.
  uint64_t decoded_end = 0;
  uint64_t byte_size = 0;
  std::optional<uint64_t> terminator_offset;
  uint64_t terminator_size = 0;
  uint64_t uninterpreted_offset = 0;
  uint64_t uninterpreted_size = 0;
  std::optional<DeltaPrefix> delta_prefix;
  std::vector<DeltaPacket> delta_packets;
  std::vector<ResolvedSchema> schemas;
  std::vector<DecodedNode> nodes;
  /// Ordered delta occurrences, including repeated indices; no state is
  /// applied.
  std::vector<DecodedNode> delta_nodes;
  std::vector<SchemaDiagnostic> diagnostics;
};

class SchemaParser {
 public:
  /// Reads one first-occurrence schema definition. A null base asserts that
  /// this type is absent from the base; the caller must establish membership.
  [[nodiscard]] static Result<ResolvedSchema> ReadDefinition(
      std::span<const uint8_t> data, uint64_t offset, uint16_t node_type,
      const TypeDefinition* base, const SchemaLimits& limits = {});
  /// No resynchronization or candidate matching: nodes are consumed in order.
  /// Record errors retain the completed prefix and a diagnostic in the report.
  /// Header, option and catalog errors return Result::Err.
  /// Without a catalog, a reviewed 13006 administration, attribute, topology
  /// and analytic, spline and procedural geometry subset is supplied. Delta
  /// occurrences retain statuses. Ambiguous packets may be resolved only by
  /// excluding an out-of-range following node type in the sequential stream.
  [[nodiscard]] static Result<SchemaParseReport> Parse(
      std::span<const uint8_t> data, const SchemaCatalog* catalog = nullptr,
      const SchemaLimits& limits = {});
  [[nodiscard]] static Result<SchemaParseReport> ParseFile(
      const std::filesystem::path& path, const SchemaCatalog* catalog = nullptr,
      const SchemaLimits& limits = {});
};

[[nodiscard]] const char* ToString(SchemaSource source) noexcept;
[[nodiscard]] const char* ToString(DeltaPacketKind kind) noexcept;
[[nodiscard]] std::optional<FieldType> FieldTypeFromCode(char code) noexcept;
[[nodiscard]] uint8_t ComponentCount(FieldType type) noexcept;

}  // namespace openparasolid
