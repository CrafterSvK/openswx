// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <bit>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "openparasolid/delta.h"
namespace openparasolid {
namespace {
ParseError Error(const char* message,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, 0, message};
}
Result<Model> Checked(const SchemaParseReport& report,
                      const ModelOptions& limits) {
  auto model = ModelBuilder::Build(report, limits);
  if (!model.ok()) return model;
  if (model.value().statistics().missing_references ||
      model.value().statistics().wrong_class_references)
    return Result<Model>::Err(
        Error("Delta state contains dangling or wrong-class references"));
  return model;
}
bool Complete(const SchemaParseReport& report) {
  return report.complete && report.diagnostics.empty() &&
         !report.uninterpreted_size && report.terminator_offset &&
         *report.terminator_offset == report.decoded_end &&
         report.byte_size >= report.decoded_end &&
         report.terminator_size == report.byte_size - report.decoded_end &&
         (report.terminator_size == 4 || report.terminator_size == 8);
}
}  // namespace
Result<DeltaState> CreateDeltaState(const SchemaParseReport& base,
                                    uint64_t revision,
                                    const ModelOptions& limits) {
  auto model = Checked(base, limits);
  if (!model.ok()) return Result<DeltaState>::Err(model.error());
  return Result<DeltaState>::Ok({revision, base});
}
Result<DeltaState> ApplyDelta(const DeltaState& base,
                              const SchemaParseReport& delta,
                              const DeltaApplicationPlan& plan,
                              const ModelOptions& limits) {
  auto valid = Checked(base.snapshot, limits);
  if (!valid.ok()) return Result<DeltaState>::Err(valid.error());
  if (plan.base_revision != base.revision ||
      plan.target_revision == base.revision)
    return Result<DeltaState>::Err(
        Error("Stale or unchanged delta revision", ErrorCode::kInvalidOptions));
  if (delta.header.kind != StreamKind::kDeltas || !Complete(delta) ||
      !delta.delta_prefix)
    return Result<DeltaState>::Err(
        Error("Delta application requires a complete decoded delta stream"));
  if (plan.operations.size() > limits.max_nodes ||
      delta.delta_nodes.size() > limits.max_nodes ||
      plan.references.size() > limits.max_references ||
      delta.schemas.size() > limits.max_fields)
    return Result<DeltaState>::Err(
        Error("Delta input limit", ErrorCode::kResourceLimit));
  std::unordered_map<uint64_t, NodeId> mappings;
  for (const auto& ref : plan.references) {
    if (!ref.wire_index || ref.status > 1 || !ref.target.value ||
        !mappings
             .emplace((uint64_t{ref.wire_index} << 1) | ref.status, ref.target)
             .second)
      return Result<DeltaState>::Err(
          Error("Ambiguous or invalid delta reference mapping",
                ErrorCode::kInvalidOptions));
  }
  struct Record {
    ResolvedSchema schema;
    DecodedNode node;
  };
  std::vector<Record> records;
  std::unordered_map<uint32_t, std::size_t> indices;
  uint64_t fields = 0, values = 0;
  for (const auto& node : base.snapshot.nodes) {
    indices.emplace(node.index, records.size());
    records.push_back(
        {ResolvedSchema{base.snapshot.schemas[node.schema_index].definition,
                        SchemaSource::kFull,
                        0,
                        0,
                        {}},
         node});
    fields += node.fields.size();
    for (const auto& f : node.fields) values += f.values.size();
  }
  std::unordered_set<uint32_t> changed;
  std::unordered_set<std::size_t> occurrences;
  for (const auto& op : plan.operations) {
    if (!op.target.value || !changed.insert(op.target.value).second)
      return Result<DeltaState>::Err(
          Error("Duplicate or null delta target", ErrorCode::kInvalidOptions));
    auto previous = indices.find(op.target.value);
    if (op.kind == DeltaOperationKind::kErase) {
      if (previous == indices.end())
        return Result<DeltaState>::Err(
            Error("Delta deletion target is absent"));
      auto& node = records[previous->second].node;
      fields -= node.fields.size();
      for (const auto& f : node.fields) values -= f.values.size();
      node.index = 0;
      indices.erase(previous);
      continue;
    }
    if (op.kind != DeltaOperationKind::kCreate &&
        op.kind != DeltaOperationKind::kReplace)
      return Result<DeltaState>::Err(
          Error("Unknown delta operation", ErrorCode::kInvalidOptions));
    if ((op.kind == DeltaOperationKind::kCreate) != (previous == indices.end()))
      return Result<DeltaState>::Err(
          Error("Delta creation/replacement precondition failed"));
    if (op.occurrence >= delta.delta_nodes.size() ||
        !occurrences.insert(op.occurrence).second)
      return Result<DeltaState>::Err(Error("Invalid or reused delta occurrence",
                                           ErrorCode::kInvalidOptions));
    const auto& source = delta.delta_nodes[op.occurrence];
    if (!source.status_framed || source.schema_index >= delta.schemas.size())
      return Result<DeltaState>::Err(
          Error("Delta occurrence lacks reviewed status framing",
                ErrorCode::kUnsupportedFormat));
    const auto& schema = delta.schemas[source.schema_index];
    if (!source.index || source.node_type != schema.definition.node_type ||
        source.fields.size() != schema.definition.fields.size() ||
        source.offset < delta.header.body_offset ||
        source.offset > delta.decoded_end || !source.size ||
        source.size > delta.decoded_end - source.offset)
      return Result<DeltaState>::Err(Error("Malformed delta source node"));
    if (previous != indices.end() &&
        records[previous->second].node.node_type != source.node_type)
      return Result<DeltaState>::Err(
          Error("Delta replacement changes node type"));
    if (source.fields.size() > limits.max_fields)
      return Result<DeltaState>::Err(
          Error("Delta source field limit", ErrorCode::kResourceLimit));
    uint64_t added_values = 0;
    for (const auto& f : source.fields) {
      if (f.offset < source.offset || f.offset > source.offset + source.size ||
          f.size > source.offset + source.size - f.offset)
        return Result<DeltaState>::Err(
            Error("Invalid delta source field range"));
      if (f.values.size() > limits.max_values - added_values)
        return Result<DeltaState>::Err(
            Error("Delta source value limit", ErrorCode::kResourceLimit));
      added_values += f.values.size();
    }
    if (previous != indices.end()) {
      const auto& old = records[previous->second].node;
      fields -= old.fields.size();
      for (const auto& f : old.fields) values -= f.values.size();
    }
    if (source.fields.size() > limits.max_fields - fields ||
        added_values > limits.max_values - values ||
        (previous == indices.end() && indices.size() >= limits.max_nodes))
      return Result<DeltaState>::Err(
          Error("Delta state resource limit", ErrorCode::kResourceLimit));
    Record record{
        ResolvedSchema{schema.definition, SchemaSource::kFull, 0, 0, {}},
        source};
    record.node.index = op.target.value;
    record.node.status_framed = false;
    for (std::size_t i = 0; i < record.node.fields.size(); ++i) {
      auto& f = record.node.fields[i];
      const auto type = schema.definition.fields[i].type;
      if (type == FieldType::kPointer) {
        if (f.reference_statuses.size() != f.values.size())
          return Result<DeltaState>::Err(
              Error("Missing per-pointer delta status"));
        for (std::size_t j = 0; j < f.values.size(); ++j) {
          auto* wire = std::get_if<uint64_t>(&f.values[j]);
          if (!wire || *wire > UINT32_MAX || f.reference_statuses[j] > 1)
            return Result<DeltaState>::Err(
                Error("Invalid delta pointer codec"));
          if (*wire) {
            auto target = mappings.find((*wire << 1) | f.reference_statuses[j]);
            if (target == mappings.end())
              return Result<DeltaState>::Err(
                  Error("Unresolved delta reference mapping",
                        ErrorCode::kUnsupportedFormat));
            *wire = target->second.value;
          }
        }
      } else {
        if (!f.reference_statuses.empty())
          return Result<DeltaState>::Err(Error("Status on non-pointer field"));
        for (auto& scalar : f.values)
          if (auto* raw = std::get_if<uint64_t>(&scalar)) {
            if (type == FieldType::kInteger && *raw <= UINT32_MAX)
              scalar =
                  int64_t{std::bit_cast<int32_t>(static_cast<uint32_t>(*raw))};
            else if (type == FieldType::kShort && *raw <= UINT16_MAX)
              scalar =
                  int64_t{std::bit_cast<int16_t>(static_cast<uint16_t>(*raw))};
          }
      }
      f.reference_statuses.clear();
    }
    fields += record.node.fields.size();
    values += added_values;
    if (previous == indices.end()) {
      indices.emplace(op.target.value, records.size());
      records.push_back(std::move(record));
    } else
      records[previous->second] = std::move(record);
  }
  SchemaParseReport snapshot;
  snapshot.header = base.snapshot.header;
  snapshot.header.body_offset = 0;
  snapshot.catalog_source = "explicit-delta-state";
  snapshot.complete = true;
  uint64_t cursor = 0;
  for (auto& record : records)
    if (record.node.index) {
      auto& node = record.node;
      if (cursor > UINT64_MAX - 8)
        return Result<DeltaState>::Err(
            Error("Logical delta address limit", ErrorCode::kResourceLimit));
      node.offset = cursor;
      cursor += 4;
      for (auto& f : node.fields) {
        if (cursor == UINT64_MAX ||
            f.values.size() > (UINT64_MAX - cursor - 1) / 8)
          return Result<DeltaState>::Err(
              Error("Logical delta address limit", ErrorCode::kResourceLimit));
        f.offset = cursor;
        f.size = 1 + 8 * f.values.size();
        cursor += f.size;
      }
      node.size = cursor - node.offset;
      node.schema_index = snapshot.schemas.size();
      snapshot.schemas.push_back(std::move(record.schema));
      snapshot.nodes.push_back(std::move(node));
    }
  if (cursor > UINT64_MAX - 4)
    return Result<DeltaState>::Err(
        Error("Logical delta terminator limit", ErrorCode::kResourceLimit));
  snapshot.decoded_end = cursor;
  snapshot.terminator_offset = cursor;
  snapshot.terminator_size = 4;
  snapshot.byte_size = cursor + 4;
  snapshot.uninterpreted_offset = cursor + 4;
  auto checked = Checked(snapshot, limits);
  if (!checked.ok()) return Result<DeltaState>::Err(checked.error());
  return Result<DeltaState>::Ok({plan.target_revision, std::move(snapshot)});
}
}  // namespace openparasolid
