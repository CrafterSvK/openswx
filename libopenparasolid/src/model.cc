// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/model.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <unordered_set>

#include "model_internal.h"

namespace openparasolid {
const ModelNode* Model::FindNode(NodeId id) const noexcept {
  const auto found = positions_.find(id.value);
  return found == positions_.end() ? nullptr : &nodes_[found->second];
}
bool Model::references_verified() const noexcept {
  return statistics_.missing_references == 0 &&
         statistics_.wrong_class_references == 0 &&
         statistics_.unverified_references == 0;
}
bool Model::typed_geometry_complete() const noexcept {
  return geometry_.size() == statistics_.geometry_candidates;
}
Result<Model> ModelBuilder::Build(const SchemaParseReport& report,
                                  const ModelOptions& options) {
  return internal::ModelAssembler(report, options).Build();
}
const char* ToString(ModelReferenceStatus status) noexcept {
  switch (status) {
    case ModelReferenceStatus::kNull:
      return "null";
    case ModelReferenceStatus::kResolved:
      return "resolved";
    case ModelReferenceStatus::kUnverifiedClass:
      return "unverified_class";
    case ModelReferenceStatus::kMissingTarget:
      return "missing_target";
    case ModelReferenceStatus::kWrongClass:
      return "wrong_class";
  }
  return "unknown";
}
const char* ToString(ModelIssueCode code) noexcept {
  switch (code) {
    case ModelIssueCode::kMissingTarget:
      return "missing_target";
    case ModelIssueCode::kWrongClass:
      return "wrong_class";
    case ModelIssueCode::kUnverifiedClass:
      return "unverified_class";
    case ModelIssueCode::kInvalidGeometry:
      return "invalid_geometry";
    case ModelIssueCode::kUnsupportedGeometry:
      return "unsupported_geometry";
  }
  return "unknown";
}
const char* GeometryName(const GeometryPayload& geometry) noexcept {
  constexpr const char* names[]{
      "point",        "line",        "circle",        "ellipse",
      "plane",        "cylinder",    "cone",          "sphere",
      "torus",        "nurbs_curve", "nurbs_surface", "surface_curve",
      "trimmed_curve"};
  static_assert(std::size(names) == std::variant_size_v<GeometryPayload>);
  return geometry.valueless_by_exception() ? "unknown"
                                           : names[geometry.index()];
}
namespace internal {
namespace {
bool Contains(std::initializer_list<uint16_t> values, uint16_t value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}
bool IsCurve(uint16_t type) {
  return Contains({30, 31, 32, 38, 130, 133, 134, 137, 200}, type);
}
bool IsSurface(uint16_t type) {
  return Contains({50, 51, 52, 53, 54, 56, 59, 60, 67, 68, 120, 124, 201},
                  type);
}
bool ReviewedTarget(uint16_t type) {
  return Contains(
      {3,   4,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  29,
       30,  31,  32,  38,  40,  41,  45,  50,  51,  52,  53,  54,  56,
       59,  60,  67,  68,  70,  74,  79,  80,  81,  82,  83,  84,  86,
       90,  91,  98,  99,  100, 101, 120, 124, 125, 126, 127, 128, 130,
       133, 134, 135, 136, 137, 141, 163, 184, 200, 201, 204},
      type);
}
std::optional<bool> ClassMatches(uint16_t expected, uint16_t actual) {
  if (expected == 0) return std::nullopt;
  if (expected < 1000) return expected == actual;
  // Pointer-list entries are void* in the XT definition.
  if (expected == 1001) return true;
  if (!ReviewedTarget(actual)) return std::nullopt;
  switch (expected) {
    case 1003:
      return actual == 29 || IsCurve(actual) || IsSurface(actual);
    case 1005:
      return Contains({10, 12}, actual);
    case 1006:
      return IsSurface(actual);
    case 1007:
      return Contains({10, 12, 14, 101}, actual);
    case 1008:
      return IsCurve(actual);
    case 1010:
      return Contains({10, 12, 16, 17, 101}, actual);
    case 1011:
      return Contains({10, 12, 18, 101}, actual);
    case 1012:
      return actual == 74 ? std::optional<bool>(true) : std::nullopt;
    case 1013:
      return actual == 12 ? std::optional<bool>(true) : std::nullopt;
    case 1015:
      if (actual == 29 || IsCurve(actual) || IsSurface(actual) ||
          Contains({10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, actual))
        return true;
      return std::nullopt;
    case 1018:
      return Contains({82, 83, 84, 85, 86, 87, 88, 89, 98}, actual);
    case 1019:
      return Contains({81, 90, 91}, actual);
    case 1029:
      return Contains({12, 13}, actual);
    case 1030:
      return actual == 163;
    case 1036:
      return actual == 184;
    case 1037:
      return Contains({84, 98}, actual);
    default:
      return std::nullopt;
  }
}
bool ScalarMatches(const ScalarValue& value, FieldType type) {
  switch (type) {
    case FieldType::kLogical:
      return std::holds_alternative<bool>(value);
    case FieldType::kByte:
    case FieldType::kCharacter:
    case FieldType::kUnicode:
    case FieldType::kPointer: {
      const auto* number = std::get_if<uint64_t>(&value);
      if (!number) return false;
      const uint64_t maximum = type == FieldType::kUnicode   ? UINT16_MAX
                               : type == FieldType::kPointer ? UINT32_MAX
                                                             : UINT8_MAX;
      return *number <= maximum;
    }
    case FieldType::kShort:
    case FieldType::kInteger:
    case FieldType::kTag:
      if (std::holds_alternative<std::monostate>(value)) return true;
      if (const auto* number = std::get_if<int64_t>(&value))
        return type == FieldType::kShort
                   ? *number >= INT16_MIN && *number <= INT16_MAX
                   : *number >= INT32_MIN && *number <= INT32_MAX;
      return false;
    case FieldType::kDouble:
    case FieldType::kInterval:
    case FieldType::kVector:
    case FieldType::kBox:
    case FieldType::kIntersectionPoint:
      return std::holds_alternative<double>(value) ||
             std::holds_alternative<std::monostate>(value);
    case FieldType::kOpaque:
      return false;
  }
  return false;
}
ParseError Invalid(uint64_t offset, const char* message) {
  return {ErrorCode::kInvalidRecord, offset, message};
}
ParseError Limit(uint64_t offset) {
  return {ErrorCode::kResourceLimit, offset, "Model resource limit exceeded"};
}
Result<BuiltGeometry> BuildSurfaceCurve(const DecodedNode& node,
                                        ModelFields& fields) {
  SurfaceCurveGeometry curve;
  if (!fields.Reference("surface", curve.surface) ||
      !fields.Reference("b_curve", curve.parameter_curve))
    return Result<BuiltGeometry>::Err(fields.Error());
  if (curve.surface.value == 0 || curve.parameter_curve.value == 0)
    return Result<BuiltGeometry>::Err(Invalid(
        node.offset,
        "Surface curve requires non-null surface and B-curve references"));
  return Result<BuiltGeometry>::Ok({GeometryPayload{curve}, 2});
}
Result<BuiltGeometry> BuildTrimmedCurve(const DecodedNode& node,
                                        ModelFields& fields) {
  TrimmedCurveGeometry curve;
  uint8_t sense = 0;
  if (!fields.Reference("basis_curve", curve.basis_curve) ||
      !fields.Vector("point_1", curve.point_1) ||
      !fields.Vector("point_2", curve.point_2) ||
      !fields.Number("parm_1", curve.parameter_1) ||
      !fields.Number("parm_2", curve.parameter_2) ||
      !fields.Byte("sense", sense, FieldType::kCharacter))
    return Result<BuiltGeometry>::Err(fields.Error());
  if (curve.basis_curve.value == 0 || curve.parameter_1 == curve.parameter_2 ||
      sense != '+')
    return Result<BuiltGeometry>::Err(
        Invalid(node.offset,
                "Trimmed curve requires a basis, distinct parameters and "
                "positive sense"));
  return Result<BuiltGeometry>::Ok({GeometryPayload{curve}, 9});
}
}  // namespace
bool IsGeometryType(uint16_t type) {
  return type == 29 || IsCurve(type) || IsSurface(type);
}
const DecodedField* ModelFields::Find(std::string_view name,
                                      FieldType type) const {
  for (std::size_t i = 0; i < schema_.fields.size(); ++i)
    if (schema_.fields[i].name == name && schema_.fields[i].type == type)
      return &node_.fields[i];
  return nullptr;
}
bool ModelFields::Invalid(std::string_view name) {
  invalid_field_ = name;
  return false;
}
ParseError ModelFields::Error() const {
  return {ErrorCode::kInvalidRecord, node_.offset,
          "Missing, unset, non-finite or incompatible geometry field: " +
              invalid_field_};
}
bool ModelFields::Number(std::string_view name, double& value) {
  const auto* field = Find(name, FieldType::kDouble);
  if (!field || field->elements != 1 || field->values.size() != 1)
    return Invalid(name);
  const auto* number = std::get_if<double>(&field->values[0]);
  if (!number || !std::isfinite(*number)) return Invalid(name);
  value = *number;
  return true;
}
bool ModelFields::Vector(std::string_view name, Vector3& value) {
  const auto* field = Find(name, FieldType::kVector);
  if (!field || field->elements != 1 || field->values.size() != 3)
    return Invalid(name);
  double xyz[3];
  for (std::size_t i = 0; i < 3; ++i) {
    const auto* number = std::get_if<double>(&field->values[i]);
    if (!number || !std::isfinite(*number)) return Invalid(name);
    xyz[i] = *number;
  }
  value = {xyz[0], xyz[1], xyz[2]};
  return true;
}
bool ModelFields::Integer(std::string_view name, uint32_t& value) {
  const auto* field = Find(name, FieldType::kInteger);
  if (!field) field = Find(name, FieldType::kShort);
  if (!field || field->elements != 1 || field->values.size() != 1)
    return Invalid(name);
  const auto* number = std::get_if<int64_t>(&field->values[0]);
  if (!number || *number < 0 || static_cast<uint64_t>(*number) > UINT32_MAX)
    return Invalid(name);
  value = static_cast<uint32_t>(*number);
  return true;
}
bool ModelFields::Byte(std::string_view name, uint8_t& value, FieldType type) {
  const auto* field = Find(name, type);
  if (!field || field->elements != 1 || field->values.size() != 1)
    return Invalid(name);
  const auto* number = std::get_if<uint64_t>(&field->values[0]);
  if (!number || *number > UINT8_MAX) return Invalid(name);
  value = static_cast<uint8_t>(*number);
  return true;
}
bool ModelFields::Logical(std::string_view name, bool& value) {
  const auto* field = Find(name, FieldType::kLogical);
  if (!field || field->elements != 1 || field->values.size() != 1)
    return Invalid(name);
  const auto* logical = std::get_if<bool>(&field->values[0]);
  if (!logical) return Invalid(name);
  value = *logical;
  return true;
}
bool ModelFields::Reference(std::string_view name, NodeId& value) {
  const auto* field = Find(name, FieldType::kPointer);
  if (!field || field->elements != 1 || field->values.size() != 1)
    return Invalid(name);
  const auto* number = std::get_if<uint64_t>(&field->values[0]);
  if (!number || *number > UINT32_MAX) return Invalid(name);
  value = NodeId{static_cast<uint32_t>(*number)};
  return true;
}
const DecodedNode* GeometryContext::Resolve(NodeId id, uint16_t type) const {
  const auto found = positions.find(id.value);
  if (found == positions.end()) return nullptr;
  const auto& node = report.nodes[found->second];
  return node.node_type == type ? &node : nullptr;
}
ModelFields GeometryContext::Fields(const DecodedNode& node) const {
  return {node, report.schemas[node.schema_index].definition};
}
std::optional<ParseError> ModelAssembler::ValidateAndIndex() {
  if (!options_.max_nodes || !options_.max_fields || !options_.max_references ||
      !options_.max_values || !options_.max_diagnostics)
    return ParseError{ErrorCode::kInvalidOptions, 0,
                      "Model limits must be positive"};
  if (report_.header.kind != StreamKind::kTransmit &&
      report_.header.kind != StreamKind::kPartition)
    return ParseError{ErrorCode::kUnsupportedFormat, 0,
                      "Model building requires a complete partition or "
                      "transmit stream; apply deltas first"};
  if (!report_.complete || !report_.diagnostics.empty() ||
      report_.uninterpreted_size != 0 ||
      report_.decoded_end > report_.byte_size || !report_.terminator_offset ||
      *report_.terminator_offset != report_.decoded_end ||
      report_.terminator_size != report_.byte_size - report_.decoded_end ||
      (report_.terminator_size != 4 && report_.terminator_size != 8) ||
      report_.delta_prefix || !report_.delta_nodes.empty() ||
      !report_.delta_packets.empty())
    return Invalid(
        report_.decoded_end,
        "Model building requires a complete, non-delta parse report");
  if (report_.nodes.size() > options_.max_nodes) return Limit(0);
  if (report_.schemas.size() > options_.max_fields) return Limit(0);
  std::vector<bool> checked(report_.schemas.size(), false);
  uint64_t fields = 0, values = 0, references = 0;
  for (const auto& node : report_.nodes) {
    if (!node.index || node.node_type < 2 || node.node_type > 32767 ||
        node.status_framed || node.schema_index >= report_.schemas.size() ||
        node.offset < report_.header.body_offset ||
        node.offset > report_.decoded_end || node.size == 0 ||
        node.size > report_.decoded_end - node.offset)
      return Invalid(node.offset, "Invalid model source node");
    const auto& schema = report_.schemas[node.schema_index].definition;
    const bool variable =
        !schema.fields.empty() && schema.fields.back().elements == 1;
    if (schema.node_type != node.node_type ||
        node.fields.size() != schema.fields.size() ||
        node.variable_length.has_value() != variable)
      return Invalid(node.offset,
                     "Node fields do not match the resolved schema");
    if (node.fields.size() > options_.max_fields - fields)
      return Limit(node.offset);
    fields += node.fields.size();
    if (!checked[node.schema_index]) {
      std::unordered_set<std::string_view> names;
      for (std::size_t i = 0; i < schema.fields.size(); ++i) {
        const auto& field = schema.fields[i];
        if (field.name.empty() || !names.insert(field.name).second ||
            !FieldTypeFromCode(static_cast<char>(field.type)) ||
            (field.pointer_class != 0 && field.type != FieldType::kPointer) ||
            (!field.transmitted && field.elements != 1) ||
            (field.elements == 1 && i + 1 != schema.fields.size()))
          return Invalid(node.offset,
                         "Invalid or ambiguous model source schema");
      }
      checked[node.schema_index] = true;
    }
    for (std::size_t i = 0; i < node.fields.size(); ++i) {
      const auto& field = node.fields[i];
      const auto& def = schema.fields[i];
      if (!FieldTypeFromCode(static_cast<char>(def.type)) || def.name.empty() ||
          (def.elements == 1 && i + 1 != node.fields.size()))
        return Invalid(node.offset, "Invalid model source schema");
      const uint32_t count = !def.transmitted    ? 0
                             : def.elements == 0 ? 1
                             : def.elements == 1 ? *node.variable_length
                                                 : def.elements;
      if (field.definition_index != i || field.elements != count ||
          field.components != ComponentCount(def.type) ||
          field.values.size() != uint64_t{count} * field.components ||
          !field.reference_statuses.empty() || field.offset < node.offset ||
          field.offset > node.offset + node.size ||
          field.size > node.offset + node.size - field.offset)
        return Invalid(node.offset,
                       "Invalid model source field shape or byte range");
      if (field.values.size() > options_.max_values - values)
        return Limit(field.offset);
      values += field.values.size();
      for (const auto& value : field.values) {
        if (!ScalarMatches(value, def.type))
          return Invalid(field.offset, "Invalid model source scalar codec");
        if (def.type == FieldType::kPointer &&
            *std::get_if<uint64_t>(&value) > UINT32_MAX)
          return Invalid(field.offset,
                         "Model reference exceeds compact index range");
      }
      if (def.type == FieldType::kPointer) {
        if (field.values.size() > options_.max_references - references)
          return Limit(field.offset);
        references += field.values.size();
      }
    }
    const auto position = model_.nodes_.size();
    if (!model_.positions_.emplace(node.index, position).second)
      return Invalid(node.offset, "Duplicate model node index");
    model_.nodes_.push_back(
        {NodeId{node.index}, node.node_type, node.offset, {}, {}});
  }
  return std::nullopt;
}
void ModelAssembler::Issue(ModelIssueCode code, const DecodedNode& node,
                           std::string field, std::string message,
                           uint64_t offset) {
  ++model_.statistics_.total_issues;
  if (model_.issues_.size() < options_.max_diagnostics)
    model_.issues_.push_back({code, NodeId{node.index}, offset,
                              std::move(field), std::move(message)});
}
void ModelAssembler::LinkReferences() {
  for (std::size_t i = 0; i < report_.nodes.size(); ++i) {
    const auto& node = report_.nodes[i];
    const auto& schema = report_.schemas[node.schema_index].definition;
    for (std::size_t f = 0; f < node.fields.size(); ++f) {
      if (schema.fields[f].type != FieldType::kPointer) continue;
      const auto& def = schema.fields[f];
      const auto& field = node.fields[f];
      for (uint32_t element = 0; element < field.elements; ++element) {
        const auto target = static_cast<uint32_t>(
            *std::get_if<uint64_t>(&field.values[element]));
        ModelReference link{
            def.name,          element, NodeId{target},
            def.pointer_class, {},      ModelReferenceStatus::kNull};
        auto& stats = model_.statistics_;
        if (!target) {
          ++stats.null_references;
        } else {
          const auto found = model_.positions_.find(target);
          if (found == model_.positions_.end()) {
            link.status = ModelReferenceStatus::kMissingTarget;
            ++stats.missing_references;
            Issue(ModelIssueCode::kMissingTarget, node, def.name,
                  "Reference target " + std::to_string(target) +
                      " is absent from this stream",
                  field.offset);
          } else {
            link.target_node = found->second;
            const auto compatible = ClassMatches(
                def.pointer_class, report_.nodes[found->second].node_type);
            if (!compatible) {
              link.status = ModelReferenceStatus::kUnverifiedClass;
              ++stats.unverified_references;
              Issue(ModelIssueCode::kUnverifiedClass, node, def.name,
                    "Pointer class " + std::to_string(def.pointer_class) +
                        " has no reviewed membership rule for this target type",
                    field.offset);
            } else if (!*compatible) {
              link.status = ModelReferenceStatus::kWrongClass;
              ++stats.wrong_class_references;
              Issue(ModelIssueCode::kWrongClass, node, def.name,
                    "Reference target has an incompatible node type",
                    field.offset);
            } else {
              link.status = ModelReferenceStatus::kResolved;
              ++stats.resolved_references;
            }
          }
        }
        model_.nodes_[i].references.push_back(std::move(link));
      }
    }
  }
}
std::optional<ParseError> ModelAssembler::BuildGeometry() {
  uint64_t copied_values = 0;
  for (std::size_t i = 0; i < report_.nodes.size(); ++i) {
    const auto& node = report_.nodes[i];
    if (!IsGeometryType(node.node_type)) continue;
    ++model_.statistics_.geometry_candidates;
    ModelFields fields(node, report_.schemas[node.schema_index].definition);
    const GeometryContext context{report_, model_.positions_,
                                  options_.max_values - copied_values};
    auto built = node.node_type == 133   ? BuildTrimmedCurve(node, fields)
                 : node.node_type == 137 ? BuildSurfaceCurve(node, fields)
                 : node.node_type == 124 || node.node_type == 134
                     ? BuildSpline(node, fields, context)
                     : BuildAnalytic(node, fields);
    if (!built.ok()) {
      if (built.error().code == ErrorCode::kResourceLimit) return built.error();
      const bool unsupported =
          built.error().code == ErrorCode::kUnsupportedFormat;
      if (unsupported)
        ++model_.statistics_.unsupported_geometry;
      else
        ++model_.statistics_.invalid_geometry;
      Issue(unsupported ? ModelIssueCode::kUnsupportedGeometry
                        : ModelIssueCode::kInvalidGeometry,
            node, "", built.error().message, built.error().offset);
      continue;
    }
    std::optional<uint8_t> sense;
    if (node.node_type != 29) {
      uint8_t raw = 0;
      if (!fields.Byte("sense", raw, FieldType::kCharacter)) {
        ++model_.statistics_.invalid_geometry;
        Issue(ModelIssueCode::kInvalidGeometry, node, "sense",
              fields.Error().message, node.offset);
        continue;
      }
      sense = raw;
    }
    if (built.value().copied_values > options_.max_values - copied_values)
      return Limit(node.offset);
    copied_values += built.value().copied_values;
    model_.nodes_[i].geometry = model_.geometry_.size();
    model_.geometry_.push_back(
        {NodeId{node.index}, sense, std::move(built).value().payload});
  }
  return std::nullopt;
}
Result<Model> ModelAssembler::Build() {
  if (auto error = ValidateAndIndex())
    return Result<Model>::Err(std::move(*error));
  LinkReferences();
  if (auto error = BuildGeometry())
    return Result<Model>::Err(std::move(*error));
  return Result<Model>::Ok(std::move(model_));
}
}  // namespace internal
}  // namespace openparasolid
