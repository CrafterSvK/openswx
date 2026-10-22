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
Result<bool> CheckLimits(const SchemaLimits& l) {
  using Return = Result<bool>;
  if (!l.max_input_bytes || !l.max_nodes || !l.max_types ||
      !l.max_fields_per_type || !l.max_total_fields || !l.max_values ||
      !l.max_delta_packets || !l.max_delta_entries)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0, "Schema limits must be positive"});
  return Return::Ok(true);
}
Result<bool> Validate(const TypeDefinition& type, const SchemaLimits& l,
                      uint64_t at) {
  using Return = Result<bool>;
  if (type.node_type < 2 || type.node_type > 32767)
    return Return::Err(
        {ErrorCode::kInvalidSchema, at, "Invalid node type in schema"});
  if (type.fields.size() > l.max_fields_per_type)
    return Return::Err(
        {ErrorCode::kResourceLimit, at, "Schema field limit exceeded"});
  std::set<std::string> names;
  for (std::size_t i = 0; i < type.fields.size(); ++i) {
    const auto& f = type.fields[i];
    if (f.name.empty() || !names.insert(f.name).second ||
        !FieldTypeFromCode(static_cast<char>(f.type)) ||
        (f.pointer_class != 0 && f.type != FieldType::kPointer) ||
        (!f.transmitted && f.elements != 1) ||
        (f.elements == 1 && i + 1 != type.fields.size()))
      return Return::Err({ErrorCode::kInvalidSchema, at,
                          "Invalid field definition or variable field order"});
  }
  return Return::Ok(true);
}
Result<FieldDefinition> ReadField(Cursor& c) {
  using Return = Result<FieldDefinition>;
  FieldDefinition field;
  const auto name_result = c.Text();
  if (!name_result.ok()) return Return::Err(name_result.error());
  field.name = name_result.value();
  const auto class_result = c.Read(2);
  if (!class_result.ok()) return Return::Err(class_result.error());
  field.pointer_class = static_cast<uint16_t>(class_result.value());
  const auto elements_result = c.Index();
  if (!elements_result.ok()) return Return::Err(elements_result.error());
  field.elements = elements_result.value();
  if (field.pointer_class) {
    field.type = FieldType::kPointer;
  } else {
    const auto codec_result = c.Text();
    if (!codec_result.ok()) return Return::Err(codec_result.error());
    const auto code = codec_result.value();
    const auto type =
        code.size() == 1 ? FieldTypeFromCode(code.front()) : std::nullopt;
    if (!type)
      return Return::Err(
          {ErrorCode::kInvalidSchema, c.pos, "Unknown scalar field codec"});
    field.type = *type;
  }
  if (field.elements == 1) {
    const auto transmitted_result = c.Logical();
    if (!transmitted_result.ok())
      return Return::Err(transmitted_result.error());
    field.transmitted = transmitted_result.value();
  }
  return Return::Ok(std::move(field));
}
Result<ResolvedSchema> Definition(Cursor& c, uint16_t type,
                                  const TypeDefinition* base,
                                  const SchemaLimits& l) {
  using Return = Result<ResolvedSchema>;
  ResolvedSchema result;
  result.offset = c.pos;
  result.definition.node_type = type;
  if (base) {
    const auto valid = Validate(*base, l, c.pos);
    if (!valid.ok()) return Return::Err(valid.error());
    if (base->node_type != type)
      return Return::Err({ErrorCode::kInvalidSchema, c.pos,
                          "Base definition has a different node type"});
    result.definition.name = base->name;
    result.definition.description = base->description;
  }
  const auto count_result = c.Read(1);
  if (!count_result.ok()) return Return::Err(count_result.error());
  const auto count = count_result.value();
  if (count == 255 && base) {
    result.definition = *base;
    result.source = SchemaSource::kUnchanged;
  } else if (!base) {
    if (count == 255)
      return Return::Err(
          {ErrorCode::kInvalidSchema, result.offset,
           "Unchanged schema marker requires a base definition"});
    result.source = SchemaSource::kFull;
    const auto name_result = c.Text();
    if (!name_result.ok()) return Return::Err(name_result.error());
    result.definition.name = name_result.value();
    const auto description_result = c.Text(true);
    if (!description_result.ok())
      return Return::Err(description_result.error());
    result.definition.description = description_result.value();
    if (count > l.max_fields_per_type)
      return Return::Err(
          {ErrorCode::kResourceLimit, c.pos, "Schema field limit exceeded"});
    for (uint64_t i = 0; i < count; ++i) {
      auto field = ReadField(c);
      if (!field.ok()) return Return::Err(field.error());
      result.definition.fields.push_back(std::move(field).value());
    }
  } else {
    result.source = SchemaSource::kEdited;
    std::size_t index = 0;
    bool appending = false;
    while (true) {
      SchemaEdit edit;
      edit.offset = c.pos;
      const auto opcode_result = c.Read(1);
      if (!opcode_result.ok()) return Return::Err(opcode_result.error());
      edit.opcode = static_cast<char>(opcode_result.value());
      if (edit.opcode == 'Z') {
        result.edits.push_back(edit);
        break;
      }
      const bool remaining = index < base->fields.size();
      switch (edit.opcode) {
        case 'C':
        case 'D':
          if (appending || !remaining)
            return Return::Err({ErrorCode::kInvalidSchema, edit.offset,
                                "Schema edit exceeds base fields"});
          if (edit.opcode == 'C')
            result.definition.fields.push_back(base->fields[index]);
          ++index;
          break;
        case 'I': {
          if (appending || !remaining)
            return Return::Err({ErrorCode::kInvalidSchema, edit.offset,
                                "Insert requires a remaining base field"});
          auto field = ReadField(c);
          if (!field.ok()) return Return::Err(field.error());
          edit.field = std::move(field).value();
          result.definition.fields.push_back(*edit.field);
          break;
        }
        case 'A': {
          if (remaining)
            return Return::Err({ErrorCode::kInvalidSchema, edit.offset,
                                "Append precedes unconsumed base fields"});
          appending = true;
          auto field = ReadField(c);
          if (!field.ok()) return Return::Err(field.error());
          edit.field = std::move(field).value();
          result.definition.fields.push_back(*edit.field);
          break;
        }
        default:
          return Return::Err({ErrorCode::kInvalidSchema, edit.offset,
                              "Unknown schema edit opcode"});
      }
      if (result.definition.fields.size() > count ||
          result.definition.fields.size() > l.max_fields_per_type)
        return Return::Err({ErrorCode::kInvalidSchema, edit.offset,
                            "Schema edit field count exceeded"});
      result.edits.push_back(std::move(edit));
    }
    if (result.definition.fields.size() != count)
      return Return::Err({ErrorCode::kInvalidSchema, c.pos,
                          "Schema edit field count mismatch"});
  }
  const auto valid = Validate(result.definition, l, result.offset);
  if (!valid.ok()) return Return::Err(valid.error());
  result.size = c.pos - result.offset;
  return Return::Ok(std::move(result));
}
Result<ScalarValue> Scalar(Cursor& c, FieldType type, bool delta = false) {
  using Return = Result<ScalarValue>;
  switch (type) {
    case FieldType::kByte:
    case FieldType::kCharacter:
    case FieldType::kUnicode: {
      const auto value = c.Read(type == FieldType::kUnicode ? 2 : 1);
      if (!value.ok()) return Return::Err(value.error());
      return Return::Ok(value.value());
    }
    case FieldType::kLogical: {
      const auto value = c.Logical();
      if (!value.ok()) return Return::Err(value.error());
      return Return::Ok(value.value());
    }
    case FieldType::kPointer: {
      const auto value = c.Index();
      if (!value.ok()) return Return::Err(value.error());
      return Return::Ok(uint64_t{value.value()});
    }
    case FieldType::kShort: {
      const auto value = c.Read(2);
      if (!value.ok()) return Return::Err(value.error());
      const auto v = static_cast<uint16_t>(value.value());
      if (v == 0x8004) return Return::Ok(std::monostate{});
      return Return::Ok(int64_t{std::bit_cast<int16_t>(v)});
    }
    case FieldType::kInteger:
    case FieldType::kTag: {
      const auto value = c.Read(4);
      if (!value.ok()) return Return::Err(value.error());
      const auto v = static_cast<uint32_t>(value.value());
      if (delta) return Return::Ok(uint64_t{v});
      if (v == 0xffff8004) return Return::Ok(std::monostate{});
      return Return::Ok(int64_t{std::bit_cast<int32_t>(v)});
    }
    case FieldType::kOpaque:
      return Return::Err({ErrorCode::kUnsupportedSchema, c.pos,
                          "Opaque transmitted field has no supported codec"});
    default: {
      const auto bits = c.Read(8);
      if (!bits.ok()) return Return::Err(bits.error());
      if (bits.value() == 0xc2bc928f996e0000ULL)
        return Return::Ok(std::monostate{});
      return Return::Ok(std::bit_cast<double>(bits.value()));
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
Result<bool> CheckDeltaLayout(const DecodedNode& node,
                              const TypeDefinition& definition) {
  using Return = Result<bool>;
  const auto expected = DeltaDefinition(node.node_type);
  if (definition.fields.size() != expected.fields.size())
    return Return::Err(
        {ErrorCode::kUnsupportedSchema, node.offset,
         "Delta prefix requires the reviewed PARTITION/PMARK field layout"});
  for (std::size_t i = 0; i < expected.fields.size(); ++i) {
    const auto& field = definition.fields[i];
    const auto& base = expected.fields[i];
    if (field.name != base.name || field.type != base.type ||
        field.pointer_class != base.pointer_class || field.elements != 0 ||
        !field.transmitted)
      return Return::Err(
          {ErrorCode::kUnsupportedSchema, node.fields[i].offset,
           "Delta prefix field roles differ from the reviewed base layout"});
  }
  return Return::Ok(true);
}
Result<uint32_t> DeltaWord(std::span<const uint8_t> data,
                           const DecodedField& field) {
  using Return = Result<uint32_t>;
  // IDs and counts are bit-preserving words, even for a generic unset sentinel.
  Cursor cursor{data, field.offset};
  const auto word_result = cursor.Read(4);
  if (!word_result.ok()) return Return::Err(word_result.error());
  return Return::Ok(static_cast<uint32_t>(word_result.value()));
}
uint32_t DeltaReference(const DecodedField& field) {
  return static_cast<uint32_t>(std::get<uint64_t>(field.values.front()));
}
Result<DeltaPartition> ReadDeltaPartition(std::span<const uint8_t> data,
                                          const DecodedNode& node,
                                          const TypeDefinition& definition) {
  using Return = Result<DeltaPartition>;
  const auto layout = CheckDeltaLayout(node, definition);
  if (!layout.ok()) return Return::Err(layout.error());
  const auto highest = DeltaWord(data, node.fields[1]);
  if (!highest.ok()) return Return::Err(highest.error());
  DeltaPartition result{node.offset, node.index, DeltaReference(node.fields[0]),
                        highest.value()};
  if (result.current_pmark == 0 || result.current_pmark == result.index)
    return Return::Err(
        {ErrorCode::kInvalidRecord, node.fields[0].offset,
         "PARTITION must name a distinct non-null current PMARK"});
  return Return::Ok(std::move(result));
}
Result<DeltaPrefix> ReadDeltaPrefix(std::span<const uint8_t> data,
                                    const DeltaPartition& partition,
                                    const DecodedNode& node,
                                    const TypeDefinition& definition) {
  using Return = Result<DeltaPrefix>;
  const auto layout = CheckDeltaLayout(node, definition);
  if (!layout.ok()) return Return::Err(layout.error());
  if (node.index != partition.current_pmark)
    return Return::Err(
        {ErrorCode::kInvalidRecord, node.offset,
         "PARTITION current_pmark does not name the following PMARK"});
  DeltaMark mark;
  mark.offset = node.offset;
  mark.index = node.index;
  mark.preceding = DeltaReference(node.fields[0]);
  mark.first_following = DeltaReference(node.fields[1]);
  mark.next_sibling = DeltaReference(node.fields[2]);
  mark.prev_sibling = DeltaReference(node.fields[3]);
  const auto new_count = DeltaWord(data, node.fields[4]);
  if (!new_count.ok()) return Return::Err(new_count.error());
  mark.n_new_nodes = new_count.value();
  const auto deleted_count = DeltaWord(data, node.fields[5]);
  if (!deleted_count.ok()) return Return::Err(deleted_count.error());
  mark.n_del_nodes = deleted_count.value();
  const auto modified_count = DeltaWord(data, node.fields[6]);
  if (!modified_count.ok()) return Return::Err(modified_count.error());
  mark.n_copy_mod_nodes = modified_count.value();
  const auto delta_key = DeltaWord(data, node.fields[7]);
  if (!delta_key.ok()) return Return::Err(delta_key.error());
  mark.delta_key = delta_key.value();
  mark.delta_is_forward = std::get<bool>(node.fields[8].values.front());
  const auto identifier = DeltaWord(data, node.fields[9]);
  if (!identifier.ok()) return Return::Err(identifier.error());
  mark.identifier = identifier.value();
  return Return::Ok({partition.offset,
                     node.offset + node.size - partition.offset, partition,
                     mark});
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
  using Return = Result<ResolvedSchema>;
  const auto valid = CheckLimits(limits);
  if (!valid.ok()) return Return::Err(valid.error());
  if (data.size() > limits.max_input_bytes)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Input byte limit exceeded"});
  Cursor c{data, offset};
  return Definition(c, type, base, limits);
}
Result<SchemaParseReport> SchemaParser::Parse(std::span<const uint8_t> data,
                                              const SchemaCatalog* supplied,
                                              const SchemaLimits& limits) {
  using Return = Result<SchemaParseReport>;
  const auto valid = CheckLimits(limits);
  if (!valid.ok()) return Return::Err(valid.error());
  if (data.size() > limits.max_input_bytes)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Input byte limit exceeded"});
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
    return Return::Err(
        {ErrorCode::kInvalidSchema, report.header.schema_offset,
         "Catalog key must match the complete input schema key"});
  report.catalog_source = catalog.source;
  std::map<uint16_t, const TypeDefinition*> bases;
  std::set<uint16_t> absent;
  uint64_t catalog_fields = 0;
  if (catalog.base_types.size() + catalog.absent_types.size() >
      limits.max_types)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Catalog type limit exceeded"});
  for (const auto& type : catalog.base_types) {
    const auto checked = Validate(type, limits, 0);
    if (!checked.ok()) return Return::Err(checked.error());
    if (!bases.emplace(type.node_type, &type).second)
      return Return::Err(
          {ErrorCode::kInvalidSchema, 0, "Duplicate catalog type"});
    catalog_fields += type.fields.size();
    if (catalog_fields > limits.max_total_fields)
      return Return::Err(
          {ErrorCode::kResourceLimit, 0, "Catalog field limit exceeded"});
  }
  for (auto type : catalog.absent_types)
    if (type < 2 || type > 32767 || bases.contains(type) ||
        !absent.insert(type).second)
      return Return::Err({ErrorCode::kInvalidSchema, 0,
                          "Invalid or duplicate absent catalog type"});
  Cursor c{data, report.header.body_offset};
  std::map<uint16_t, std::size_t> resolved;
  std::set<uint32_t> indices;
  uint64_t values = 0, fields = 0, decoded_fields = 0, delta_entries = 0;
  uint64_t start = c.pos;
  std::optional<uint16_t> current;
  std::optional<DeltaPartition> delta_partition;
  const bool deltas = report.header.kind == StreamKind::kDeltas;
  const auto decoded = [&]() -> Result<bool> {
    using DecodeResult = Result<bool>;
    if (!report.header.internal_header_complete ||
        report.header.user_field_words != 0)
      return DecodeResult::Err(
          {ErrorCode::kUnsupportedSchema, c.pos,
           "Sequential decoding requires a numeric schema and zero "
           "user-field words"});
    while (true) {
      start = c.pos;
      current.reset();
      const auto type_result = c.Read(2);
      if (!type_result.ok()) return DecodeResult::Err(type_result.error());
      const auto type = static_cast<uint16_t>(type_result.value());
      if (type == 1) {
        const auto index_result = c.Index();
        if (!index_result.ok()) return DecodeResult::Err(index_result.error());
        const auto index = index_result.value();
        if (deltas && index == 0 && data.size() - start == 8 &&
            std::equal(data.begin() + static_cast<std::ptrdiff_t>(c.pos),
                       data.end(),
                       std::array<uint8_t, 4>{0, 1, 0, 1}.begin())) {
          if (!report.delta_prefix)
            return DecodeResult::Err(
                {ErrorCode::kInvalidRecord, start,
                 "Delta trailer precedes the administration prefix"});
          c.pos = data.size();
          report.terminator_offset = start;
          report.terminator_size = 8;
          report.complete = true;
          break;
        }
        if (deltas && index == 0 && c.pos != data.size()) {
          if (!report.delta_prefix)
            return DecodeResult::Err(
                {ErrorCode::kInvalidRecord, start,
                 "Delta packet precedes the administration prefix"});
          if (report.delta_packets.size() >= limits.max_delta_packets ||
              delta_entries >= limits.max_delta_entries)
            return DecodeResult::Err({ErrorCode::kResourceLimit, start,
                                      "Delta packet resource limit exceeded"});
          auto packet = internal::ReadSequentialDeltaPacket(
              data, start, report.header.schema_max_type.value_or(32767),
              limits.max_delta_entries - delta_entries);
          if (!packet.ok()) return DecodeResult::Err(packet.error());
          delta_entries +=
              packet.value().references.size() + packet.value().marks.size();
          c.pos = start + packet.value().size;
          report.delta_packets.push_back(std::move(packet).value());
          report.decoded_end = c.pos;
          continue;
        }
        if (index != 0 || c.pos != data.size())
          return DecodeResult::Err(
              {ErrorCode::kInvalidRecord, start,
               "Terminator must have index zero and end the stream"});
        if (deltas && !report.delta_prefix)
          return DecodeResult::Err(
              {ErrorCode::kInvalidRecord, start,
               "Delta stream ended without a PARTITION/current-PMARK pair"});
        report.terminator_offset = start;
        report.terminator_size = c.pos - start;
        report.complete = true;
        break;
      }
      current = type;
      if (type < 2 || type > 32767 ||
          (report.header.schema_max_type &&
           type > *report.header.schema_max_type))
        return DecodeResult::Err({ErrorCode::kInvalidRecord, start,
                                  "Node type outside the header range"});
      if (deltas && !report.delta_prefix && type != (delta_partition ? 4 : 3))
        return DecodeResult::Err(
            {ErrorCode::kUnsupportedFormat, start,
             "Delta decoding requires leading PARTITION and current PMARK "
             "nodes"});
      if (report.nodes.size() + report.delta_nodes.size() >= limits.max_nodes)
        return DecodeResult::Err(
            {ErrorCode::kResourceLimit, start, "Node limit exceeded"});
      if (!resolved.contains(type)) {
        if (resolved.size() >= limits.max_types)
          return DecodeResult::Err({ErrorCode::kResourceLimit, start,
                                    "Resolved type limit exceeded"});
        const auto base = bases.find(type);
        if (base == bases.end() && !absent.contains(type))
          return DecodeResult::Err(
              {ErrorCode::kUnsupportedSchema, c.pos,
               "Base membership is unknown for node type " +
                   std::to_string(type)});
        ResolvedSchema schema;
        if (report.header.schema_version->base_schema) {
          auto definition = Definition(
              c, type, base == bases.end() ? nullptr : base->second, limits);
          if (!definition.ok()) return DecodeResult::Err(definition.error());
          schema = std::move(definition).value();
        } else {
          if (base == bases.end())
            return DecodeResult::Err(
                {ErrorCode::kUnsupportedSchema, c.pos,
                 "No definition for a standard-schema node"});
          schema.definition = *base->second;
          schema.offset = c.pos;
        }
        fields += schema.definition.fields.size();
        if (fields > limits.max_total_fields)
          return DecodeResult::Err({ErrorCode::kResourceLimit, c.pos,
                                    "Resolved schema field limit exceeded"});
        resolved.emplace(type, report.schemas.size());
        report.schemas.push_back(std::move(schema));
      }
      const bool status_framed = deltas && report.delta_prefix.has_value();
      if (status_framed && (type == 3 || type == 4))
        return DecodeResult::Err(
            {ErrorCode::kUnsupportedFormat, start,
             "Administrative delta nodes require an established history "
             "packet"});
      DecodedNode node;
      node.status_framed = status_framed;
      node.offset = start;
      node.node_type = type;
      node.schema_index = resolved.at(type);
      const auto& schema = report.schemas[node.schema_index].definition;
      if (!schema.fields.empty() && schema.fields.back().elements == 1) {
        const auto count_result = c.Read(4);
        if (!count_result.ok()) return DecodeResult::Err(count_result.error());
        const auto count = static_cast<uint32_t>(count_result.value());
        if (count > 0x7fffffff)
          return DecodeResult::Err({ErrorCode::kInvalidRecord, c.pos - 4,
                                    "Negative variable element count"});
        node.variable_length = count;
      }
      const auto node_index = c.Index();
      if (!node_index.ok()) return DecodeResult::Err(node_index.error());
      node.index = node_index.value();
      if (!node.index || (!status_framed && indices.contains(node.index)))
        return DecodeResult::Err({ErrorCode::kInvalidRecord, c.pos,
                                  "Node index is zero or duplicated"});
      if (schema.fields.size() > limits.max_total_fields - decoded_fields)
        return DecodeResult::Err(
            {ErrorCode::kResourceLimit, c.pos, "Decoded field limit exceeded"});
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
          return DecodeResult::Err({ErrorCode::kResourceLimit, c.pos,
                                    "Decoded value limit exceeded"});
        values += count;
        // Read before growing: truncated inputs never allocate the declared
        // array size.
        for (uint64_t j = 0; j < count; ++j) {
          auto scalar = Scalar(c, def.type, status_framed);
          if (!scalar.ok()) return DecodeResult::Err(scalar.error());
          field.values.push_back(std::move(scalar).value());
          if (status_framed && def.type == FieldType::kPointer) {
            const auto at = c.pos;
            const auto status_result = c.Read(1);
            if (!status_result.ok())
              return DecodeResult::Err(status_result.error());
            const auto status = static_cast<uint8_t>(status_result.value());
            if (status > 1)
              return DecodeResult::Err(
                  {ErrorCode::kInvalidRecord, at,
                   "Delta reference status must be zero or one"});
            field.reference_statuses.push_back(status);
          }
        }
        field.size = c.pos - field.offset;
        node.fields.push_back(std::move(field));
      }
      node.size = c.pos - node.offset;
      if (deltas && !report.delta_prefix) {
        if (!delta_partition) {
          auto partition = ReadDeltaPartition(data, node, schema);
          if (!partition.ok()) return DecodeResult::Err(partition.error());
          delta_partition = std::move(partition).value();
        } else {
          auto prefix = ReadDeltaPrefix(data, *delta_partition, node, schema);
          if (!prefix.ok()) return DecodeResult::Err(prefix.error());
          report.delta_prefix = std::move(prefix).value();
        }
      }
      if (status_framed)
        report.delta_nodes.push_back(std::move(node));
      else {
        indices.insert(node.index);
        report.nodes.push_back(std::move(node));
      }
      report.decoded_end = c.pos;
    }
    return DecodeResult::Ok(true);
  }();
  if (!decoded.ok()) report.diagnostics.push_back({decoded.error(), current});
  report.uninterpreted_offset = report.complete ? c.pos : start;
  report.uninterpreted_size = data.size() - report.uninterpreted_offset;
  return Result<SchemaParseReport>::Ok(std::move(report));
}
Result<SchemaParseReport> SchemaParser::ParseFile(
    const std::filesystem::path& path, const SchemaCatalog* catalog,
    const SchemaLimits& limits) {
  using Return = Result<SchemaParseReport>;
  const auto valid = CheckLimits(limits);
  if (!valid.ok()) return Return::Err(valid.error());
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) return Return::Err({ErrorCode::kIo, 0, "Cannot open input file"});
  const auto size = input.tellg();
  if (size < 0)
    return Return::Err({ErrorCode::kIo, 0, "Cannot determine input size"});
  const auto length = static_cast<uint64_t>(size);
  if (length > limits.max_input_bytes ||
      length > std::numeric_limits<std::size_t>::max() ||
      length >
          static_cast<uint64_t>(std::numeric_limits<std::streamsize>::max()))
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Input byte limit exceeded"});
  std::vector<uint8_t> data(static_cast<std::size_t>(length));
  input.seekg(0);
  if (!input.read(reinterpret_cast<char*>(data.data()),
                  static_cast<std::streamsize>(length)))
    return Return::Err({ErrorCode::kIo, 0, "Cannot read input file"});
  return Parse(data, catalog, limits);
}
}  // namespace openparasolid
