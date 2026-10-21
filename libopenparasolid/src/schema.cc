// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openparasolid/schema.h"

#include <algorithm>
#include <array>
#include <bit>
#include <fstream>
#include <limits>
#include <map>
#include <set>

#include "base_catalog.h"
#include "binary_reader.h"
#include "delta_packets.h"
#include "openparasolid/parser.h"

namespace openparasolid {
namespace {
using internal::Cursor;
using internal::Fail;
void CheckLimits(const SchemaLimits& l) {
  if (!l.max_input_bytes || !l.max_nodes || !l.max_types ||
      !l.max_fields_per_type || !l.max_total_fields || !l.max_values ||
      !l.max_delta_packets || !l.max_delta_entries)
    Fail(ErrorCode::kInvalidOptions, 0, "Schema limits must be positive");
}
void Validate(const TypeDefinition& type, const SchemaLimits& l, uint64_t at) {
  if (type.node_type < 2 || type.node_type > 32767)
    Fail(ErrorCode::kInvalidSchema, at, "Invalid node type in schema");
  if (type.fields.size() > l.max_fields_per_type)
    Fail(ErrorCode::kResourceLimit, at, "Schema field limit exceeded");
  std::set<std::string> names;
  for (std::size_t i = 0; i < type.fields.size(); ++i) {
    const auto& f = type.fields[i];
    if (f.name.empty() || !names.insert(f.name).second ||
        !FieldTypeFromCode(static_cast<char>(f.type)) ||
        (f.pointer_class != 0 && f.type != FieldType::kPointer) ||
        (!f.transmitted && f.elements != 1) ||
        (f.elements == 1 && i + 1 != type.fields.size()))
      Fail(ErrorCode::kInvalidSchema, at,
           "Invalid field definition or variable field order");
  }
}
FieldDefinition ReadField(Cursor& c) {
  FieldDefinition field;
  field.name = c.Text();
  field.pointer_class = static_cast<uint16_t>(c.Read(2));
  field.elements = c.Index();
  if (field.pointer_class) {
    field.type = FieldType::kPointer;
  } else {
    const auto code = c.Text();
    const auto type =
        code.size() == 1 ? FieldTypeFromCode(code.front()) : std::nullopt;
    if (!type)
      Fail(ErrorCode::kInvalidSchema, c.pos, "Unknown scalar field codec");
    field.type = *type;
  }
  if (field.elements == 1) field.transmitted = c.Logical();
  return field;
}
ResolvedSchema Definition(Cursor& c, uint16_t type, const TypeDefinition* base,
                          const SchemaLimits& l) {
  ResolvedSchema result;
  result.offset = c.pos;
  result.definition.node_type = type;
  if (base) {
    Validate(*base, l, c.pos);
    if (base->node_type != type)
      Fail(ErrorCode::kInvalidSchema, c.pos,
           "Base definition has a different node type");
    result.definition.name = base->name;
    result.definition.description = base->description;
  }
  const auto count = c.Read(1);
  if (count == 255 && base) {
    result.definition = *base;
    result.source = SchemaSource::kUnchanged;
  } else if (!base) {
    if (count == 255)
      Fail(ErrorCode::kInvalidSchema, result.offset,
           "Unchanged schema marker requires a base definition");
    result.source = SchemaSource::kFull;
    result.definition.name = c.Text();
    result.definition.description = c.Text(true);
    if (count > l.max_fields_per_type)
      Fail(ErrorCode::kResourceLimit, c.pos, "Schema field limit exceeded");
    for (uint64_t i = 0; i < count; ++i)
      result.definition.fields.push_back(ReadField(c));
  } else {
    result.source = SchemaSource::kEdited;
    std::size_t index = 0;
    bool appending = false;
    while (true) {
      SchemaEdit edit;
      edit.offset = c.pos;
      edit.opcode = static_cast<char>(c.Read(1));
      if (edit.opcode == 'Z') {
        result.edits.push_back(edit);
        break;
      }
      const bool remaining = index < base->fields.size();
      switch (edit.opcode) {
        case 'C':
        case 'D':
          if (appending || !remaining)
            Fail(ErrorCode::kInvalidSchema, edit.offset,
                 "Schema edit exceeds base fields");
          if (edit.opcode == 'C')
            result.definition.fields.push_back(base->fields[index]);
          ++index;
          break;
        case 'I':
          if (appending || !remaining)
            Fail(ErrorCode::kInvalidSchema, edit.offset,
                 "Insert requires a remaining base field");
          edit.field = ReadField(c);
          result.definition.fields.push_back(*edit.field);
          break;
        case 'A':
          if (remaining)
            Fail(ErrorCode::kInvalidSchema, edit.offset,
                 "Append precedes unconsumed base fields");
          appending = true;
          edit.field = ReadField(c);
          result.definition.fields.push_back(*edit.field);
          break;
        default:
          Fail(ErrorCode::kInvalidSchema, edit.offset,
               "Unknown schema edit opcode");
      }
      if (result.definition.fields.size() > count ||
          result.definition.fields.size() > l.max_fields_per_type)
        Fail(ErrorCode::kInvalidSchema, edit.offset,
             "Schema edit field count exceeded");
      result.edits.push_back(std::move(edit));
    }
    if (result.definition.fields.size() != count)
      Fail(ErrorCode::kInvalidSchema, c.pos,
           "Schema edit field count mismatch");
  }
  Validate(result.definition, l, result.offset);
  result.size = c.pos - result.offset;
  return result;
}
ScalarValue Scalar(Cursor& c, FieldType type, bool delta = false) {
  switch (type) {
    case FieldType::kByte:
    case FieldType::kCharacter:
      return c.Read(1);
    case FieldType::kLogical:
      return c.Logical();
    case FieldType::kUnicode:
      return c.Read(2);
    case FieldType::kPointer:
      return uint64_t{c.Index()};
    case FieldType::kShort: {
      const auto v = static_cast<uint16_t>(c.Read(2));
      if (v == 0x8004) return std::monostate{};
      return int64_t{std::bit_cast<int16_t>(v)};
    }
    case FieldType::kInteger:
    case FieldType::kTag: {
      const auto v = static_cast<uint32_t>(c.Read(4));
      if (delta) return uint64_t{v};
      if (v == 0xffff8004) return std::monostate{};
      return int64_t{std::bit_cast<int32_t>(v)};
    }
    case FieldType::kOpaque:
      Fail(ErrorCode::kUnsupportedSchema, c.pos,
           "Opaque transmitted field has no supported codec");
    default: {
      const auto bits = c.Read(8);
      if (bits == 0xc2bc928f996e0000ULL) return std::monostate{};
      return std::bit_cast<double>(bits);
    }
  }
}
TypeDefinition DeltaDefinition(uint16_t type) {
  TypeDefinition definition;
  definition.node_type = type;
  definition.name = type == 3 ? "PARTITION" : "PMARK";
  definition.description = "Reviewed 13006 delta administration layout";
  if (type == 3) {
    definition.fields.push_back({"current_pmark", FieldType::kPointer, 4});
    definition.fields.push_back({"highest_id", FieldType::kInteger});
  } else {
    for (const char* name :
         {"preceding", "first_following", "next_sibling", "prev_sibling"})
      definition.fields.push_back({name, FieldType::kPointer, 4});
    for (const char* name :
         {"n_new_nodes", "n_del_nodes", "n_copy_mod_nodes", "delta_key"})
      definition.fields.push_back({name, FieldType::kInteger});
    definition.fields.push_back({"delta_is_forward", FieldType::kLogical});
    definition.fields.push_back({"identifier", FieldType::kInteger});
  }
  return definition;
}
void CheckDeltaLayout(const DecodedNode& node,
                      const TypeDefinition& definition) {
  const auto expected = DeltaDefinition(node.node_type);
  if (definition.fields.size() != expected.fields.size())
    Fail(ErrorCode::kUnsupportedSchema, node.offset,
         "Delta prefix requires the reviewed PARTITION/PMARK field layout");
  for (std::size_t i = 0; i < expected.fields.size(); ++i) {
    const auto& field = definition.fields[i];
    const auto& base = expected.fields[i];
    if (field.name != base.name || field.type != base.type ||
        field.pointer_class != base.pointer_class || field.elements != 0 ||
        !field.transmitted)
      Fail(ErrorCode::kUnsupportedSchema, node.fields[i].offset,
           "Delta prefix field roles differ from the reviewed base layout");
  }
}
uint32_t DeltaWord(std::span<const uint8_t> data, const DecodedField& field) {
  // IDs and counts are bit-preserving words, even for a generic unset sentinel.
  Cursor cursor{data, field.offset};
  return static_cast<uint32_t>(cursor.Read(4));
}
uint32_t DeltaReference(const DecodedField& field) {
  return static_cast<uint32_t>(std::get<uint64_t>(field.values.front()));
}
DeltaPartition ReadDeltaPartition(std::span<const uint8_t> data,
                                  const DecodedNode& node,
                                  const TypeDefinition& definition) {
  CheckDeltaLayout(node, definition);
  DeltaPartition result{node.offset, node.index, DeltaReference(node.fields[0]),
                        DeltaWord(data, node.fields[1])};
  if (result.current_pmark == 0 || result.current_pmark == result.index)
    Fail(ErrorCode::kInvalidRecord, node.fields[0].offset,
         "PARTITION must name a distinct non-null current PMARK");
  return result;
}
DeltaPrefix ReadDeltaPrefix(std::span<const uint8_t> data,
                            const DeltaPartition& partition,
                            const DecodedNode& node,
                            const TypeDefinition& definition) {
  CheckDeltaLayout(node, definition);
  if (node.index != partition.current_pmark)
    Fail(ErrorCode::kInvalidRecord, node.offset,
         "PARTITION current_pmark does not name the following PMARK");
  DeltaMark mark;
  mark.offset = node.offset;
  mark.index = node.index;
  mark.preceding = DeltaReference(node.fields[0]);
  mark.first_following = DeltaReference(node.fields[1]);
  mark.next_sibling = DeltaReference(node.fields[2]);
  mark.prev_sibling = DeltaReference(node.fields[3]);
  mark.n_new_nodes = DeltaWord(data, node.fields[4]);
  mark.n_del_nodes = DeltaWord(data, node.fields[5]);
  mark.n_copy_mod_nodes = DeltaWord(data, node.fields[6]);
  mark.delta_key = DeltaWord(data, node.fields[7]);
  mark.delta_is_forward = std::get<bool>(node.fields[8].values.front());
  mark.identifier = DeltaWord(data, node.fields[9]);
  return {partition.offset, node.offset + node.size - partition.offset,
          partition, mark};
}
SchemaCatalog DefaultCatalog(const StreamHeader& header) {
  SchemaCatalog catalog;
  catalog.schema_key = header.schema;
  catalog.source = "no-built-in-definition";
  if (!header.schema_version || header.schema_version->base_schema != 13006)
    return catalog;
  catalog.source = "reviewed-13006-base-subset";
  catalog.base_types.push_back(DeltaDefinition(3));
  catalog.base_types.push_back(DeltaDefinition(4));
  // Public XT Format Reference (April 2008), pp. 82-83 and 118.
  TypeDefinition world{101, "WORLD", "Public 13006 WORLD base layout", {}};
  constexpr uint16_t classes[]{10, 81, 12, 100, 1006, 1008, 29, 0, 80, 0, 0};
  constexpr const char* names[]{
      "assemblies", "attributes", "bodies",
      "transforms", "surfaces",   "curves",
      "points",     "alive",      "attribute_definitions",
      "highest_id", "current_id"};
  for (unsigned i = 0; i < 11; ++i) {
    const auto type = i == 7   ? FieldType::kLogical
                      : i >= 9 ? FieldType::kInteger
                               : FieldType::kPointer;
    world.fields.push_back({names[i], type, classes[i], 0, true});
  }
  catalog.base_types.push_back(std::move(world));
  internal::AddValueAndTopologyDefinitions(catalog);
  internal::AddGeometryDefinitions(catalog);
  internal::AddSplineAndListDefinitions(catalog);
  internal::AddProceduralDefinitions(catalog);
  // INTERSECTION_DATA is newer than the reviewed 13006 schema (max type 185).
  // Its complete first-use definition must come from the stream.
  catalog.absent_types.push_back(204);
  return catalog;
}
}  // namespace

std::optional<FieldType> FieldTypeFromCode(char code) noexcept {
  switch (code) {
    case 'u':
    case 'c':
    case 'l':
    case 'n':
    case 'w':
    case 'd':
    case 'p':
    case 'q':
    case 't':
    case 'f':
    case 'i':
    case 'v':
    case 'b':
    case 'h':
      return static_cast<FieldType>(code);
    default:
      return std::nullopt;
  }
}
uint8_t ComponentCount(FieldType type) noexcept {
  switch (type) {
    case FieldType::kInterval:
      return 2;
    case FieldType::kVector:
    case FieldType::kIntersectionPoint:
      return 3;
    case FieldType::kBox:
      return 6;
    default:
      return 1;
  }
}
const char* ToString(SchemaSource source) noexcept {
  switch (source) {
    case SchemaSource::kBase:
      return "base";
    case SchemaSource::kUnchanged:
      return "unchanged";
    case SchemaSource::kEdited:
      return "edited";
    case SchemaSource::kFull:
      return "full";
  }
  return "unknown";
}
Result<ResolvedSchema> SchemaParser::ReadDefinition(
    std::span<const uint8_t> data, uint64_t offset, uint16_t type,
    const TypeDefinition* base, const SchemaLimits& limits) {
  try {
    CheckLimits(limits);
    if (data.size() > limits.max_input_bytes)
      Fail(ErrorCode::kResourceLimit, 0, "Input byte limit exceeded");
    Cursor c{data, offset};
    return Result<ResolvedSchema>::Ok(Definition(c, type, base, limits));
  } catch (const ParseError& error) {
    return Result<ResolvedSchema>::Err(error);
  }
}
Result<SchemaParseReport> SchemaParser::Parse(std::span<const uint8_t> data,
                                              const SchemaCatalog* supplied,
                                              const SchemaLimits& limits) {
  try {
    CheckLimits(limits);
    if (data.size() > limits.max_input_bytes)
      Fail(ErrorCode::kResourceLimit, 0, "Input byte limit exceeded");
    const auto header = ParasolidReader::ReadHeader(data);
    if (!header.ok()) return Result<SchemaParseReport>::Err(header.error());
    SchemaParseReport report;
    report.header = header.value();
    report.byte_size = data.size();
    report.decoded_end = report.header.body_offset;
    const auto fallback =
        supplied ? SchemaCatalog{} : DefaultCatalog(report.header);
    const auto& catalog = supplied ? *supplied : fallback;
    if (catalog.schema_key != report.header.schema)
      Fail(ErrorCode::kInvalidSchema, report.header.schema_offset,
           "Catalog key must match the complete input schema key");
    report.catalog_source = catalog.source;
    std::map<uint16_t, const TypeDefinition*> bases;
    std::set<uint16_t> absent;
    uint64_t catalog_fields = 0;
    if (catalog.base_types.size() + catalog.absent_types.size() >
        limits.max_types)
      Fail(ErrorCode::kResourceLimit, 0, "Catalog type limit exceeded");
    for (const auto& type : catalog.base_types) {
      Validate(type, limits, 0);
      if (!bases.emplace(type.node_type, &type).second)
        Fail(ErrorCode::kInvalidSchema, 0, "Duplicate catalog type");
      catalog_fields += type.fields.size();
      if (catalog_fields > limits.max_total_fields)
        Fail(ErrorCode::kResourceLimit, 0, "Catalog field limit exceeded");
    }
    for (auto type : catalog.absent_types)
      if (type < 2 || type > 32767 || bases.contains(type) ||
          !absent.insert(type).second)
        Fail(ErrorCode::kInvalidSchema, 0,
             "Invalid or duplicate absent catalog type");
    Cursor c{data, report.header.body_offset};
    std::map<uint16_t, std::size_t> resolved;
    std::set<uint32_t> indices;
    uint64_t values = 0, fields = 0, decoded_fields = 0, delta_entries = 0;
    uint64_t start = c.pos;
    std::optional<uint16_t> current;
    std::optional<DeltaPartition> delta_partition;
    const bool deltas = report.header.kind == StreamKind::kDeltas;
    try {
      if (!report.header.internal_header_complete ||
          report.header.user_field_words != 0)
        Fail(ErrorCode::kUnsupportedSchema, c.pos,
             "Sequential decoding requires a numeric schema and zero "
             "user-field words");
      while (true) {
        start = c.pos;
        current.reset();
        const auto type = static_cast<uint16_t>(c.Read(2));
        if (type == 1) {
          const auto index = c.Index();
          if (deltas && index == 0 && data.size() - start == 8 &&
              std::equal(data.begin() + static_cast<std::ptrdiff_t>(c.pos),
                         data.end(),
                         std::array<uint8_t, 4>{0, 1, 0, 1}.begin())) {
            if (!report.delta_prefix)
              Fail(ErrorCode::kInvalidRecord, start,
                   "Delta trailer precedes the administration prefix");
            c.pos = data.size();
            report.terminator_offset = start;
            report.terminator_size = 8;
            report.complete = true;
            break;
          }
          if (deltas && index == 0 && c.pos != data.size()) {
            if (!report.delta_prefix)
              Fail(ErrorCode::kInvalidRecord, start,
                   "Delta packet precedes the administration prefix");
            if (report.delta_packets.size() >= limits.max_delta_packets ||
                delta_entries >= limits.max_delta_entries)
              Fail(ErrorCode::kResourceLimit, start,
                   "Delta packet resource limit exceeded");
            auto packet = internal::ReadSequentialDeltaPacket(
                data, start, report.header.schema_max_type.value_or(32767),
                limits.max_delta_entries - delta_entries);
            if (!packet.ok()) throw packet.error();
            delta_entries +=
                packet.value().references.size() + packet.value().marks.size();
            c.pos = start + packet.value().size;
            report.delta_packets.push_back(std::move(packet).value());
            report.decoded_end = c.pos;
            continue;
          }
          if (index != 0 || c.pos != data.size())
            Fail(ErrorCode::kInvalidRecord, start,
                 "Terminator must have index zero and end the stream");
          if (deltas && !report.delta_prefix)
            Fail(ErrorCode::kInvalidRecord, start,
                 "Delta stream ended without a PARTITION/current-PMARK pair");
          report.terminator_offset = start;
          report.terminator_size = c.pos - start;
          report.complete = true;
          break;
        }
        current = type;
        if (type < 2 || type > 32767 ||
            (report.header.schema_max_type &&
             type > *report.header.schema_max_type))
          Fail(ErrorCode::kInvalidRecord, start,
               "Node type outside the header range");
        if (deltas && !report.delta_prefix && type != (delta_partition ? 4 : 3))
          Fail(ErrorCode::kUnsupportedFormat, start,
               "Delta decoding requires leading PARTITION and current PMARK "
               "nodes");
        if (report.nodes.size() + report.delta_nodes.size() >= limits.max_nodes)
          Fail(ErrorCode::kResourceLimit, start, "Node limit exceeded");
        if (!resolved.contains(type)) {
          if (resolved.size() >= limits.max_types)
            Fail(ErrorCode::kResourceLimit, start,
                 "Resolved type limit exceeded");
          const auto base = bases.find(type);
          if (base == bases.end() && !absent.contains(type))
            Fail(ErrorCode::kUnsupportedSchema, c.pos,
                 "Base membership is unknown for node type " +
                     std::to_string(type));
          ResolvedSchema schema;
          if (report.header.schema_version->base_schema) {
            schema = Definition(
                c, type, base == bases.end() ? nullptr : base->second, limits);
          } else {
            if (base == bases.end())
              Fail(ErrorCode::kUnsupportedSchema, c.pos,
                   "No definition for a standard-schema node");
            schema.definition = *base->second;
            schema.offset = c.pos;
          }
          fields += schema.definition.fields.size();
          if (fields > limits.max_total_fields)
            Fail(ErrorCode::kResourceLimit, c.pos,
                 "Resolved schema field limit exceeded");
          resolved.emplace(type, report.schemas.size());
          report.schemas.push_back(std::move(schema));
        }
        const bool status_framed = deltas && report.delta_prefix.has_value();
        if (status_framed && (type == 3 || type == 4))
          Fail(ErrorCode::kUnsupportedFormat, start,
               "Administrative delta nodes require an established history "
               "packet");
        DecodedNode node;
        node.status_framed = status_framed;
        node.offset = start;
        node.node_type = type;
        node.schema_index = resolved.at(type);
        const auto& schema = report.schemas[node.schema_index].definition;
        if (!schema.fields.empty() && schema.fields.back().elements == 1) {
          const auto count = static_cast<uint32_t>(c.Read(4));
          if (count > 0x7fffffff)
            Fail(ErrorCode::kInvalidRecord, c.pos - 4,
                 "Negative variable element count");
          node.variable_length = count;
        }
        node.index = c.Index();
        if (!node.index || (!status_framed && indices.contains(node.index)))
          Fail(ErrorCode::kInvalidRecord, c.pos,
               "Node index is zero or duplicated");
        if (schema.fields.size() > limits.max_total_fields - decoded_fields)
          Fail(ErrorCode::kResourceLimit, c.pos,
               "Decoded field limit exceeded");
        decoded_fields += schema.fields.size();
        for (std::size_t i = 0; i < schema.fields.size(); ++i) {
          const auto& def = schema.fields[i];
          DecodedField field;
          field.definition_index = i;
          field.offset = c.pos;
          field.elements = !def.transmitted    ? 0
                           : def.elements == 0 ? 1
                           : def.elements == 1 ? *node.variable_length
                                               : def.elements;
          field.components =
              status_framed && def.type == FieldType::kIntersectionPoint
                  ? 11
                  : ComponentCount(def.type);
          const uint64_t count = uint64_t{field.elements} * field.components;
          if (count > limits.max_values - values)
            Fail(ErrorCode::kResourceLimit, c.pos,
                 "Decoded value limit exceeded");
          values += count;
          // Read before growing: truncated inputs never allocate the declared
          // array size.
          for (uint64_t j = 0; j < count; ++j) {
            field.values.push_back(Scalar(c, def.type, status_framed));
            if (status_framed && def.type == FieldType::kPointer) {
              const auto at = c.pos;
              const auto status = static_cast<uint8_t>(c.Read(1));
              if (status > 1)
                Fail(ErrorCode::kInvalidRecord, at,
                     "Delta reference status must be zero or one");
              field.reference_statuses.push_back(status);
            }
          }
          field.size = c.pos - field.offset;
          node.fields.push_back(std::move(field));
        }
        node.size = c.pos - node.offset;
        if (deltas && !report.delta_prefix) {
          if (!delta_partition)
            delta_partition = ReadDeltaPartition(data, node, schema);
          else
            report.delta_prefix =
                ReadDeltaPrefix(data, *delta_partition, node, schema);
        }
        if (status_framed)
          report.delta_nodes.push_back(std::move(node));
        else {
          indices.insert(node.index);
          report.nodes.push_back(std::move(node));
        }
        report.decoded_end = c.pos;
      }
    } catch (const ParseError& error) {
      report.diagnostics.push_back({error, current});
    }
    report.uninterpreted_offset = report.complete ? c.pos : start;
    report.uninterpreted_size = data.size() - report.uninterpreted_offset;
    return Result<SchemaParseReport>::Ok(std::move(report));
  } catch (const ParseError& error) {
    return Result<SchemaParseReport>::Err(error);
  }
}
Result<SchemaParseReport> SchemaParser::ParseFile(
    const std::filesystem::path& path, const SchemaCatalog* catalog,
    const SchemaLimits& limits) {
  try {
    CheckLimits(limits);
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) Fail(ErrorCode::kIo, 0, "Cannot open input file");
    const auto size = input.tellg();
    if (size < 0) Fail(ErrorCode::kIo, 0, "Cannot determine input size");
    const auto length = static_cast<uint64_t>(size);
    if (length > limits.max_input_bytes ||
        length > std::numeric_limits<std::size_t>::max() ||
        length >
            static_cast<uint64_t>(std::numeric_limits<std::streamsize>::max()))
      Fail(ErrorCode::kResourceLimit, 0, "Input byte limit exceeded");
    std::vector<uint8_t> data(static_cast<std::size_t>(length));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char*>(data.data()),
                    static_cast<std::streamsize>(length)))
      Fail(ErrorCode::kIo, 0, "Cannot read input file");
    return Parse(data, catalog, limits);
  } catch (const ParseError& error) {
    return Result<SchemaParseReport>::Err(error);
  }
}
}  // namespace openparasolid
