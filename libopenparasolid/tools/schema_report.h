// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#ifndef OPENPARASOLID_TOOLS_SCHEMA_REPORT_H_
#define OPENPARASOLID_TOOLS_SCHEMA_REPORT_H_

#include <bit>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string_view>

#include "openparasolid/schema.h"
#include "openparasolid/version.h"

namespace openparasolid::cli {
using nlohmann::json;
inline Result<uint64_t> Unsigned(const json& object, const char* name,
                                 uint64_t maximum) {
  const auto& value = object.at(name);
  if (!value.is_number_unsigned() || value.get<uint64_t>() > maximum)
    return Result<uint64_t>::Err(
        {ErrorCode::kInvalidSchema, 0,
         std::string("Invalid catalog integer: ") + name});
  return Result<uint64_t>::Ok(value.get<uint64_t>());
}
// Check nesting before entering the recursive third-party JSON parser. Ignore
// braces inside strings, including escaped quotes and escaped backslashes.
inline bool CatalogNestingAllowed(std::string_view bytes) {
  unsigned depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (char ch : bytes) {
    if (in_string) {
      if (escaped)
        escaped = false;
      else if (ch == '\\')
        escaped = true;
      else if (ch == '"')
        in_string = false;
      continue;
    }
    if (ch == '}' || ch == ']') {
      if (depth != 0) --depth;
      continue;
    }
    if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') continue;
    // Container starts at depth 64 are allowed only when empty, matching the
    // JSON callback's depth rule for leaf events as well as container events.
    if (depth > 64) return false;
    if (ch == '"')
      in_string = true;
    else if (ch == '{' || ch == '[')
      ++depth;
  }
  return true;
}
inline Result<SchemaCatalog> LoadCatalog(const std::filesystem::path& path) {
  using Return = Result<SchemaCatalog>;
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file || file.tellg() < 0)
    return Return::Err({ErrorCode::kIo, 0, "Cannot read schema catalog"});
  if (file.tellg() > 4 * 1024 * 1024)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Cannot read schema catalog (maximum 4 MiB)"});
  const auto length = static_cast<std::size_t>(file.tellg());
  std::string bytes(length, '\0');
  file.seekg(0);
  if (!file.read(bytes.data(), static_cast<std::streamsize>(length)))
    return Return::Err({ErrorCode::kIo, 0, "Cannot read schema catalog"});
  if (!CatalogNestingAllowed(bytes))
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Catalog JSON nesting limit exceeded"});
  // JSON decoding is an infrastructure boundary. Translate only exceptions
  // raised by the third-party library; our own validation returns Result.
  try {
    const auto data = json::parse(bytes);
    SchemaCatalog result;
    result.schema_key = data.at("schema_key").get<std::string>();
    result.source = data.at("source").get<std::string>();
    if (!data.at("base_types").is_array() ||
        !data.at("absent_types").is_array())
      return Return::Err(
          {ErrorCode::kInvalidSchema, 0, "Catalog types must be arrays"});
    for (const auto& type_json : data.at("base_types")) {
      TypeDefinition type;
      const auto node_type = Unsigned(type_json, "node_type", 32767);
      if (!node_type.ok()) return Return::Err(node_type.error());
      type.node_type = static_cast<uint16_t>(node_type.value());
      type.name = type_json.at("name").get<std::string>();
      type.description = type_json.value("description", std::string{});
      if (!type_json.at("fields").is_array())
        return Return::Err(
            {ErrorCode::kInvalidSchema, 0, "Catalog fields must be an array"});
      for (const auto& field_json : type_json.at("fields")) {
        const auto code = field_json.at("type").get<std::string>();
        const auto kind =
            code.size() == 1 ? FieldTypeFromCode(code.front()) : std::nullopt;
        if (!kind)
          return Return::Err(
              {ErrorCode::kInvalidSchema, 0, "Invalid catalog field codec"});
        FieldDefinition field;
        field.name = field_json.at("name").get<std::string>();
        field.type = *kind;
        const auto pointer_class = Unsigned(field_json, "pointer_class", 65535);
        if (!pointer_class.ok()) return Return::Err(pointer_class.error());
        field.pointer_class = static_cast<uint16_t>(pointer_class.value());
        const auto elements = Unsigned(field_json, "elements", UINT32_MAX);
        if (!elements.ok()) return Return::Err(elements.error());
        field.elements = static_cast<uint32_t>(elements.value());
        field.transmitted = field_json.at("transmitted").get<bool>();
        type.fields.push_back(std::move(field));
      }
      result.base_types.push_back(std::move(type));
    }
    for (const auto& type : data.at("absent_types")) {
      if (!type.is_number_unsigned() || type.get<uint64_t>() > 32767)
        return Return::Err(
            {ErrorCode::kInvalidSchema, 0, "Invalid absent catalog type"});
      result.absent_types.push_back(type.get<uint16_t>());
    }
    return Return::Ok(std::move(result));
  } catch (const json::exception& error) {
    return Return::Err(
        {ErrorCode::kInvalidSchema, 0,
         std::string("Invalid schema catalog: ") + error.what()});
  }
}
inline json DefinitionJson(const FieldDefinition& f) {
  return {{"name", f.name},
          {"type", std::string(1, static_cast<char>(f.type))},
          {"pointer_class", f.pointer_class},
          {"elements", f.elements},
          {"transmitted", f.transmitted}};
}
inline json ValueJson(const ScalarValue& value) {
  return std::visit(
      [](const auto& v) -> json {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::monostate>)
          return nullptr;
        else if constexpr (std::is_same_v<T, double>) {
          if (std::isfinite(v)) return v;
          std::ostringstream hex;
          hex << std::hex << std::setw(16) << std::setfill('0')
              << std::bit_cast<uint64_t>(v);
          return {{"nonfinite_f64_bits", hex.str()}};
        } else
          return v;
      },
      value);
}
inline json MarkJson(const DeltaMark& mark) {
  return {{"offset", mark.offset},
          {"index", mark.index},
          {"preceding", mark.preceding},
          {"first_following", mark.first_following},
          {"next_sibling", mark.next_sibling},
          {"prev_sibling", mark.prev_sibling},
          {"n_new_nodes", mark.n_new_nodes},
          {"n_del_nodes", mark.n_del_nodes},
          {"n_copy_mod_nodes", mark.n_copy_mod_nodes},
          {"delta_key", mark.delta_key},
          {"delta_is_forward", mark.delta_is_forward},
          {"identifier", mark.identifier}};
}
inline json SchemaReport(const SchemaParseReport& r,
                         const std::filesystem::path& source, bool summary) {
  json out{{"report_version", 1},
           {"library_version", OPENPARASOLID_VERSION_STRING},
           {"source_file", source.string()},
           {"format", "binary_parasolid"},
           {"interpretation", "schema_nodes"},
           {"byte_size", r.byte_size},
           {"header",
            {{"schema", r.header.schema},
             {"kind", ToString(r.header.kind)},
             {"body_offset", r.header.body_offset}}},
           {"catalog_source", r.catalog_source},
           {"complete", r.complete},
           {"node_count", r.nodes.size()},
           {"delta_node_count", r.delta_nodes.size()},
           {"schema_count", r.schemas.size()},
           {"decoded_end", r.decoded_end},
           {"terminator_offset",
            r.terminator_offset ? json(*r.terminator_offset) : json(nullptr)},
           {"terminator_size", r.terminator_size},
           {"uninterpreted_offset", r.uninterpreted_offset},
           {"uninterpreted_size", r.uninterpreted_size},
           {"diagnostics", json::array()}};
  for (const auto& d : r.diagnostics)
    out["diagnostics"].push_back(
        {{"code", ToString(d.error.code)},
         {"offset", d.error.offset},
         {"message", d.error.message},
         {"node_type", d.node_type ? json(*d.node_type) : json(nullptr)}});
  out["delta_prefix"] = nullptr;
  if (r.delta_prefix) {
    const auto& prefix = *r.delta_prefix;
    const auto& partition = prefix.partition;
    const auto& mark = prefix.current_mark;
    out["delta_prefix"] = {{"offset", prefix.offset},
                           {"size", prefix.size},
                           {"partition",
                            {{"offset", partition.offset},
                             {"index", partition.index},
                             {"current_pmark", partition.current_pmark},
                             {"highest_id", partition.highest_id}}},
                           {"current_mark", MarkJson(mark)}};
  }
  uint64_t reference_entries = 0, mark_occurrences = 0;
  for (const auto& packet : r.delta_packets) {
    reference_entries += packet.references.size();
    mark_occurrences += packet.marks.size();
  }
  out["delta_packet_count"] = r.delta_packets.size();
  out["delta_reference_count"] = reference_entries;
  out["delta_historical_mark_count"] = mark_occurrences;
  if (summary) return out;
  out["delta_packets"] = json::array();
  for (const auto& packet : r.delta_packets) {
    json value{
        {"kind", ToString(packet.kind)},
        {"offset", packet.offset},
        {"size", packet.size},
        {"has_terminal", packet.has_terminal},
        {"terminal_word",
         packet.terminal_word ? json(*packet.terminal_word) : json(nullptr)},
        {"references", json::array()},
        {"marks", json::array()}};
    for (const auto& entry : packet.references)
      value["references"].push_back({{"offset", entry.offset},
                                     {"size", entry.size},
                                     {"index", entry.index},
                                     {"node_type", entry.node_type}});
    for (const auto& entry : packet.marks) {
      auto mark = MarkJson(entry.mark);
      mark["size"] = entry.size;
      value["marks"].push_back(std::move(mark));
    }
    out["delta_packets"].push_back(std::move(value));
  }
  out["schemas"] = json::array();
  for (const auto& s : r.schemas) {
    json schema{{"node_type", s.definition.node_type},
                {"name", s.definition.name},
                {"description", s.definition.description},
                {"source", ToString(s.source)},
                {"offset", s.offset},
                {"size", s.size},
                {"fields", json::array()},
                {"edits", json::array()}};
    for (const auto& f : s.definition.fields)
      schema["fields"].push_back(DefinitionJson(f));
    for (const auto& e : s.edits)
      schema["edits"].push_back(
          {{"opcode", std::string(1, e.opcode)},
           {"offset", e.offset},
           {"field", e.field ? DefinitionJson(*e.field) : json(nullptr)}});
    out["schemas"].push_back(std::move(schema));
  }
  auto NodesJson = [](const std::vector<DecodedNode>& nodes) {
    json result = json::array();
    for (const auto& n : nodes) {
      json node{{"node_type", n.node_type},
                {"index", n.index},
                {"schema_index", n.schema_index},
                {"status_framed", n.status_framed},
                {"offset", n.offset},
                {"size", n.size},
                {"variable_length",
                 n.variable_length ? json(*n.variable_length) : json(nullptr)},
                {"fields", json::array()}};
      for (const auto& f : n.fields) {
        json values = json::array();
        for (const auto& value : f.values) values.push_back(ValueJson(value));
        node["fields"].push_back({{"definition_index", f.definition_index},
                                  {"offset", f.offset},
                                  {"size", f.size},
                                  {"elements", f.elements},
                                  {"components", f.components},
                                  {"reference_statuses", f.reference_statuses},
                                  {"values", std::move(values)}});
      }
      result.push_back(std::move(node));
    }
    return result;
  };
  out["nodes"] = NodesJson(r.nodes);
  out["delta_nodes"] = NodesJson(r.delta_nodes);
  return out;
}
}  // namespace openparasolid::cli

#endif  // OPENPARASOLID_TOOLS_SCHEMA_REPORT_H_
