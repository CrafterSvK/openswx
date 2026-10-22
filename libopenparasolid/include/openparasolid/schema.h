// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

/**
 * @file
 * @brief Schema resolution and sequential binary node decoding.
 * @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_embedding "XT 2.1.2:
 * Embedded schemas (p. 13)", @ref xt_fields "XT 2.1.4: Field types (p. 14)",
 * @ref xt_binary "XT 3.3: Binary (p. 22)"
 */
#ifndef OPENPARASOLID_SCHEMA_H_
#define OPENPARASOLID_SCHEMA_H_

#include <filesystem>
#include <span>
#include <variant>

#include "openparasolid/result.h"
#include "openparasolid/types.h"

namespace openparasolid {

/// @brief Scalar and composite schema field codes used in transmission.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)", @ref xt_underlying
/// "XT 5.1: Underlying types (p. 33)"
enum class FieldType : char {
  /// Unsigned byte.
  kByte = 'u',
  /// Character.
  kCharacter = 'c',
  /// Logical value.
  kLogical = 'l',
  /// Short integer.
  kShort = 'n',
  /// Unicode code unit.
  kUnicode = 'w',
  /// Integer.
  kInteger = 'd',
  /// Node pointer with a declared pointer class.
  kPointer = 'p',
  /// Opaque pointer-like value.
  kOpaque = 'q',
  /// Tag value.
  kTag = 't',
  /// Floating-point scalar.
  kDouble = 'f',
  /// Parameter interval.
  kInterval = 'i',
  /// Three-component vector.
  kVector = 'v',
  /// Bounding box.
  kBox = 'b',
  /// Intersection-point composite.
  kIntersectionPoint = 'h',
};

/// @brief One scalar or array field in a node type definition.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)", @ref xt_variable
/// "XT 2.1.4.3: Variable-length nodes (p. 16)", @ref xt_pointers "XT 2.1.4.2:
/// Pointer classes (p. 16)"
struct FieldDefinition {
  /// Source schema name.
  std::string name;
  /// Transmitted scalar or composite field code.
  FieldType type = FieldType::kInteger;
  /// Schema pointer-class code; zero does not assert a verified target class.
  uint16_t pointer_class = 0;
  /// 0: scalar, 1: per-node variable count, >=2: fixed array count.
  uint32_t elements = 0;
  /// Whether the field occupies bytes in this transmitted layout.
  bool transmitted = true;
};

/// @brief Schema layout for a numeric node type.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_nodetypes "XT 6.1:
/// Node types (p. 117)"
struct TypeDefinition {
  /// Numeric schema node-type code (chapter 6.1).
  uint16_t node_type = 0;
  /// Source schema name.
  std::string name;
  /// Source descriptive text.
  std::string description;
  /// Owned fields in schema or transmission order.
  std::vector<FieldDefinition> fields;
};

/// @brief Caller-supplied base schema definitions and explicit absence
/// information.
///
/// Exact input key and explicit base membership. Omission means unknown,
/// never absent. Only absent_types may carry a complete embedded definition.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_embedding "XT 2.1.2:
/// Embedded schemas (p. 13)"
struct SchemaCatalog {
  /// Exact schema key used for base-definition lookup.
  std::string schema_key;
  /// Human-readable origin of the supplied catalog.
  std::string source;
  /// Reviewed type definitions known to belong to the base schema.
  std::vector<TypeDefinition> base_types;
  /// Type codes explicitly known to be absent from the base schema.
  std::vector<uint16_t> absent_types;
};

/// @brief How a transmitted node layout was resolved against the base schema.
///
/// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)"
enum class SchemaSource {
  /// Supplied base definition.
  kBase,
  /// Embedded marker declaring the base unchanged.
  kUnchanged,
  /// Embedded edits applied to a reviewed base.
  kEdited,
  /// Full embedded definition for a type absent from the base.
  kFull,
};
/// @brief One embedded-schema edit with its input location.
///
/// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)"
struct SchemaEdit {
  /// Transmitted embedded-schema edit opcode.
  char opcode = 'Z';
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Optional field definition associated with this edit opcode.
  std::optional<FieldDefinition> field;
};
/// @brief Owned effective layout and the edits used to resolve it.
///
/// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)"
struct ResolvedSchema {
  /// Effective owned type definition after embedded-schema resolution.
  TypeDefinition definition;
  /// Resolution method: base, unchanged, edited or full embedded definition.
  SchemaSource source = SchemaSource::kBase;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Embedded edits in source order.
  std::vector<SchemaEdit> edits;
};

/// Integer/pointer values retain their decoded transmit meaning. monostate
/// represents a wire unset sentinel. Composite fields flatten their scalar
/// components in element order; the field's components value states the stride.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)"
using ScalarValue =
    std::variant<std::monostate, bool, int64_t, uint64_t, double>;
/// @brief Decoded field values with source byte range and component stride.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)"
struct DecodedField {
  /// Zero-based field-definition index within the resolved type.
  uint64_t definition_index = 0;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Decoded array element count, independent of the scalar component stride.
  uint32_t elements = 0;
  /// Number of flattened scalar components per array element.
  uint8_t components = 1;
  /// Decoded scalar values in element/component order.
  std::vector<ScalarValue> values;
  /// One status byte per pointer value in status-framed delta fields.
  std::vector<uint8_t> reference_statuses;
};
/// @brief One decoded node occurrence with its resolved schema index.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_variable
/// "XT 2.1.4.3: Variable-length nodes (p. 16)"
struct DecodedNode {
  /// Numeric schema node-type code (chapter 6.1).
  uint16_t node_type = 0;
  /// Decoded stream-local node index; distinct delta occurrences may repeat it.
  uint32_t index = 0;
  /// Zero-based index into SchemaParseReport::schemas.
  uint64_t schema_index = 0;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Optional per-node variable-array count when the schema declares one.
  std::optional<uint32_t> variable_length;
  /// Whether fields retain internal delta status framing.
  bool status_framed = false;
  /// Owned fields in schema or transmission order.
  std::vector<DecodedField> fields;
};
/// @brief Per-parse bounds on input, schema work and decoded storage.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct SchemaLimits {
  /// Upper bound on input bytes.
  uint64_t max_input_bytes = 128u * 1024u * 1024u;
  /// Upper bound on node occurrences.
  uint64_t max_nodes = 10000;
  /// Upper bound on resolved schema types.
  uint64_t max_types = 1024;
  /// Upper bound on fields in one schema type.
  uint64_t max_fields_per_type = 4096;
  /// Applied separately to catalog fields, resolved fields and decoded fields.
  uint64_t max_total_fields = 65536;
  /// Upper bound on decoded scalar values.
  uint64_t max_values = 1000000;
  /// Upper bound on internal delta packets.
  uint64_t max_delta_packets = 10000;
  /// Upper bound on entries in internal delta packets.
  uint64_t max_delta_entries = 100000;
};
/// @brief Sequential decoding failure retaining the affected node type if
/// known.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct SchemaDiagnostic {
  /// Sequential decoding failure with source context.
  ParseError error;
  /// Numeric schema node-type code (chapter 6.1).
  std::optional<uint16_t> node_type;
};
/// @brief Inspected internal PARTITION metadata; no history interpretation.
///
/// Stream-local indices use decoded compact encoding (zero is null).
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaPartition {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Decoded stream-local PARTITION index.
  uint32_t index = 0;
  /// Stream-local reference to the current PMARK.
  uint32_t current_pmark = 0;
  /// Raw transmitted highest-id counter, retaining all 32 bits.
  uint32_t highest_id = 0;
};
/// @brief Inspected internal PMARK fields; counters and links remain source
/// metadata.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaMark {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Decoded stream-local PMARK index.
  uint32_t index = 0;
  /// Raw reference to the preceding PMARK.
  uint32_t preceding = 0;
  /// Raw reference to the first following PMARK.
  uint32_t first_following = 0;
  /// Raw reference to the next sibling PMARK.
  uint32_t next_sibling = 0;
  /// Raw reference to the previous sibling PMARK.
  uint32_t prev_sibling = 0;
  /// Transmitted count of new nodes.
  uint32_t n_new_nodes = 0;
  /// Transmitted count of deleted nodes.
  uint32_t n_del_nodes = 0;
  /// Transmitted count of copied/modified nodes.
  uint32_t n_copy_mod_nodes = 0;
  /// Raw transmitted delta key; not interpreted as a revision selector.
  uint32_t delta_key = 0;
  /// Transmitted forward-delta flag.
  bool delta_is_forward = false;
  /// Raw transmitted mark identifier.
  uint32_t identifier = 0;
};
/// @brief Inspected partition/current-mark prefix, without applying history.
///
/// Validated leading PARTITION/current-PMARK pair; no history is applied.
/// Persistent IDs, counters and keys preserve all 32 wire bits.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaPrefix {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Inspected leading PARTITION node metadata.
  DeltaPartition partition;
  /// Inspected current PMARK node metadata.
  DeltaMark current_mark;
};
/// @brief Library classification of supported internal delta packet layouts.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
enum class DeltaPacketKind {
  /// Internal reference-map packet.
  kReferenceMap,
  /// Internal historical PMARK packet.
  kHistoricalMarks,
};
/// @brief One internal delta reference-map entry in source order.
///
/// @see @ref xt_indices "XT 2.1.4.4: Unresolved indices (p. 17)"
struct DeltaReferenceEntry {
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Decoded wire reference index.
  uint32_t index = 0;
  /// Numeric schema node-type code (chapter 6.1).
  uint16_t node_type = 0;
};
/// @brief A historical mark occurrence, including repeated mark identities.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaMarkOccurrence {
  /// Inspected historical PMARK fields.
  DeltaMark mark;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
};
/// @brief Owned internal packet inspection; packet order is retained.
///
/// Occurrences retain source order and repeated identities; no state is
/// applied.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaPacket {
  /// Supported internal packet layout.
  DeltaPacketKind kind = DeltaPacketKind::kReferenceMap;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Encoded byte count of this source occurrence or range.
  uint64_t size = 0;
  /// Whether the supported packet terminal was consumed.
  bool has_terminal = false;
  /// Raw map terminal word, or the supported historical terminal value 0/1.
  std::optional<uint16_t> terminal_word;
  /// Reference-map entries in source order.
  std::vector<DeltaReferenceEntry> references;
  /// Historical mark occurrences in source order, including repeated
  /// identities.
  std::vector<DeltaMarkOccurrence> marks;
};
/// @brief Reads a supported internal packet at a caller-established boundary.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
class DeltaPacketReader {
 public:
  /// Reads only at an established packet boundary. Reference maps require a
  /// null-reference/zero/nonzero-word terminal. PMARK packets use the already
  /// established base layout and optionally end with the ten-byte historical
  /// terminal (final value 0 or 1). Ambiguous interpretations are unsupported.
  ///
  /// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @param offset Established input byte boundary at which to begin reading.
  /// @param schema_max_type Maximum allowed node-type code in this stream.
  /// @param max_entries Positive bound on packet entries.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<DeltaPacket> Read(
      std::span<const uint8_t> data, uint64_t offset,
      uint16_t schema_max_type = 32767, uint64_t max_entries = 100000);
};
/// @brief Owned sequential decode report; inspect complete and diagnostics.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_binary "XT 3.3:
/// Binary (p. 22)"
struct SchemaParseReport {
  /// Decoded input prologue.
  StreamHeader header;
  /// Description of the supplied or built-in base schema catalog.
  std::string catalog_source;
  /// Whether sequential decoding reached a supported final terminator.
  bool complete = false;
  /// End of fully decoded nodes and packets; final terminator is separate.
  uint64_t decoded_end = 0;
  /// Total input size in bytes.
  uint64_t byte_size = 0;
  /// Optional input byte offset of the final stream terminator.
  std::optional<uint64_t> terminator_offset;
  /// Encoded final terminator length in bytes.
  uint64_t terminator_size = 0;
  /// Start of the undecoded suffix in the original input.
  uint64_t uninterpreted_offset = 0;
  /// Length of the undecoded suffix in bytes.
  uint64_t uninterpreted_size = 0;
  /// Optional inspected partition/current-mark prefix.
  std::optional<DeltaPrefix> delta_prefix;
  /// Internal delta packet occurrences in source order.
  std::vector<DeltaPacket> delta_packets;
  /// Resolved type layouts used by decoded node occurrences.
  std::vector<ResolvedSchema> schemas;
  /// Sequentially decoded ordinary node occurrences.
  std::vector<DecodedNode> nodes;
  /// Ordered delta occurrences, including repeated indices; no state is
  /// applied.
  std::vector<DecodedNode> delta_nodes;
  /// Sequential parsing failures; a report may retain a completed prefix.
  std::vector<SchemaDiagnostic> diagnostics;
};

/// @brief Resolves embedded schemas and sequentially decodes supported binary
/// nodes.
///
/// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)", @ref xt_binary
/// "XT 3.3: Binary (p. 22)"
class SchemaParser {
 public:
  /// Reads one first-occurrence schema definition. A null base asserts that
  /// this type is absent from the base; the caller must establish membership.
  ///
  /// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @param offset Established input byte boundary at which to begin reading.
  /// @param node_type Numeric node-type code of the definition to read.
  /// @param base Baseline snapshot/state, or reviewed base type definition as
  /// documented.
  /// @param limits Per-call work and storage limits.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
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
  ///
  /// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)", @ref
  /// xt_binary "XT 3.3: Binary (p. 22)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @param catalog Optional reviewed base schema; null selects the built-in
  /// subset.
  /// @param limits Per-call work and storage limits.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<SchemaParseReport> Parse(
      std::span<const uint8_t> data, const SchemaCatalog* catalog = nullptr,
      const SchemaLimits& limits = {});
  /// @brief Reads a file and applies Parse() with the same limits and catalog
  /// rules.
  ///
  /// @see @ref xt_embedding "XT 2.1.2: Embedded schemas (p. 13)", @ref
  /// xt_binary "XT 3.3: Binary (p. 22)"
  ///
  /// @param path Input file to read under the same limits as the span overload.
  /// @param catalog Optional reviewed base schema; null selects the built-in
  /// subset.
  /// @param limits Per-call work and storage limits.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<SchemaParseReport> ParseFile(
      const std::filesystem::path& path, const SchemaCatalog* catalog = nullptr,
      const SchemaLimits& limits = {});
};

/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param source Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char* ToString(SchemaSource source) noexcept;
/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param kind Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char* ToString(DeltaPacketKind kind) noexcept;
/// @brief Returns the supported field type for a schema code, or no value.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)"
///
/// @param code Single source schema field code.
/// @return Supported field type, or std::nullopt for an unknown code.
[[nodiscard]] std::optional<FieldType> FieldTypeFromCode(char code) noexcept;
/// @brief Returns the flattened scalar stride for one element of this field
/// type.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)"
///
/// @param type Schema field kind whose scalar stride is requested.
/// @return Scalar stride of one element; all other codes use stride one.
[[nodiscard]] uint8_t ComponentCount(FieldType type) noexcept;

}  // namespace openparasolid

#endif  // OPENPARASOLID_SCHEMA_H_
