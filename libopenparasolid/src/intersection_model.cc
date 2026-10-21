// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <cmath>
#include <string_view>
#include <utility>

#include "model_internal.h"

namespace openparasolid::internal {
namespace {
ParseError Invalid(const DecodedNode& node, const char* message) {
  return {ErrorCode::kInvalidRecord, node.offset, message};
}
bool Declared(const GeometryContext& context, const DecodedNode& node,
              std::string_view name) {
  for (const auto& field :
       context.report.schemas[node.schema_index].definition.fields)
    if (field.name == name) return true;
  return false;
}
bool Nullable(const ScalarValue& value, std::optional<double>& result) {
  if (std::holds_alternative<std::monostate>(value)) {
    result.reset();
    return true;
  }
  const auto* number = std::get_if<double>(&value);
  if (!number || !std::isfinite(*number)) return false;
  result = *number;
  return true;
}
bool NullableField(ModelFields& fields, const char* name,
                   std::optional<double>& result) {
  const auto* field = fields.Find(name, FieldType::kDouble);
  return field && field->elements == 1 && field->values.size() == 1 &&
         Nullable(field->values[0], result);
}
bool Nonnegative(const std::optional<double>& value) {
  return !value || *value >= 0;
}
const DecodedField* Positions(ModelFields& fields) {
  const auto* field = fields.Find("positions", FieldType::kIntersectionPoint);
  if (!field || field->elements == 0 || field->components != 3 ||
      field->values.size() != uint64_t{field->elements} * 3)
    return nullptr;
  for (const auto& value : field->values) {
    const auto* number = std::get_if<double>(&value);
    if (!number || !std::isfinite(*number)) return nullptr;
  }
  return field;
}
void CopyPositions(const DecodedField& field, std::vector<Vector3>& result) {
  result.reserve(field.elements);
  for (std::size_t i = 0; i < field.values.size(); i += 3)
    result.push_back({std::get<double>(field.values[i]),
                      std::get<double>(field.values[i + 1]),
                      std::get<double>(field.values[i + 2])});
}
std::optional<ParseError> ReadLimit(const GeometryContext& context,
                                    const DecodedNode& node,
                                    IntersectionLimit& result,
                                    const DecodedField*& positions) {
  auto fields = context.Fields(node);
  if (!fields.Byte("type", result.type, FieldType::kCharacter))
    return fields.Error();
  if (Declared(context, node, "term_use")) {
    uint8_t use = 0;
    if (!fields.Byte("term_use", use, FieldType::kCharacter))
      return fields.Error();
    result.term_use = use;
  }
  positions = Positions(fields);
  if (!positions)
    return Invalid(node, "Intersection limit requires finite positions");
  if (result.type != 'H' && result.type != 'L' && result.type != 'T')
    return ParseError{ErrorCode::kUnsupportedFormat, node.offset,
                      "Unsupported intersection limit type"};
  const uint32_t expected = result.type == 'T' ? 2 : 1;
  if (positions->elements != expected)
    return Invalid(node,
                   "Intersection limit position count does not match type");
  return std::nullopt;
}
}  // namespace

Result<BuiltGeometry> BuildIntersection(const DecodedNode& node,
                                        ModelFields& fields,
                                        const GeometryContext& context) {
  IntersectionCurveGeometry curve;
  const auto* surfaces = fields.Find("surface", FieldType::kPointer);
  if (!surfaces || surfaces->elements != 2 || surfaces->values.size() != 2)
    return Result<BuiltGeometry>::Err(
        Invalid(node, "Intersection requires two surface references"));
  for (std::size_t i = 0; i < curve.surfaces.size(); ++i) {
    const auto* id = std::get_if<uint64_t>(&surfaces->values[i]);
    if (!id || *id == 0 || *id > UINT32_MAX)
      return Result<BuiltGeometry>::Err(
          Invalid(node, "Intersection requires non-null surface references"));
    curve.surfaces[i] = NodeId{static_cast<uint32_t>(*id)};
  }
  if (!fields.Reference("chart", curve.chart.source) ||
      !fields.Reference("start", curve.start.source) ||
      !fields.Reference("end", curve.end.source))
    return Result<BuiltGeometry>::Err(fields.Error());
  if (Declared(context, node, "intersection_data")) {
    NodeId id;
    if (!fields.Reference("intersection_data", id))
      return Result<BuiltGeometry>::Err(fields.Error());
    curve.intersection_data = id;
  }
  const auto* chart = context.Resolve(curve.chart.source, 40);
  const auto* start = context.Resolve(curve.start.source, 41);
  const auto* end = context.Resolve(curve.end.source, 41);
  if (!chart || !start || !end)
    return Result<BuiltGeometry>::Err(
        Invalid(node, "Missing or incompatible intersection chart or limits"));
  auto chart_fields = context.Fields(*chart);
  uint32_t count = 0;
  if (!chart_fields.Number("base_parameter", curve.chart.base_parameter) ||
      !chart_fields.Number("base_scale", curve.chart.base_scale) ||
      !chart_fields.Integer("chart_count", count))
    return Result<BuiltGeometry>::Err(chart_fields.Error());
  if (!NullableField(chart_fields, "chordal_error",
                     curve.chart.chordal_error) ||
      !NullableField(chart_fields, "angular_error",
                     curve.chart.angular_error) ||
      !Nonnegative(curve.chart.chordal_error) ||
      !Nonnegative(curve.chart.angular_error))
    return Result<BuiltGeometry>::Err(
        Invalid(*chart, "Invalid intersection chart error estimates"));
  const auto* error = chart_fields.Find("parameter_error", FieldType::kDouble);
  if (!error || error->elements != 2 || error->values.size() != 2 ||
      !Nullable(error->values[0], curve.chart.parameter_error[0]) ||
      !Nullable(error->values[1], curve.chart.parameter_error[1]))
    return Result<BuiltGeometry>::Err(
        Invalid(*chart, "Invalid intersection chart parameter errors"));
  const auto* points = Positions(chart_fields);
  if (!points || count != points->elements || curve.chart.base_scale <= 0)
    return Result<BuiltGeometry>::Err(Invalid(
        *chart, "Invalid intersection chart count, scale or positions"));
  const DecodedField* start_points = nullptr;
  const DecodedField* end_points = nullptr;
  if (auto failure = ReadLimit(context, *start, curve.start, start_points))
    return Result<BuiltGeometry>::Err(std::move(*failure));
  if (auto failure = ReadLimit(context, *end, curve.end, end_points))
    return Result<BuiltGeometry>::Err(std::move(*failure));
  // Charge metadata and each copied occurrence, even for shared CHART/LIMITs.
  constexpr uint64_t kMetadataSlots = 17;
  const uint64_t copied = kMetadataSlots + points->values.size() +
                          start_points->values.size() +
                          end_points->values.size();
  if (copied > context.remaining_values)
    return Result<BuiltGeometry>::Err(
        {ErrorCode::kResourceLimit, node.offset,
         "Copied intersection data exceeds model value limit"});
  CopyPositions(*points, curve.chart.positions);
  CopyPositions(*start_points, curve.start.positions);
  CopyPositions(*end_points, curve.end.positions);
  return Result<BuiltGeometry>::Ok({std::move(curve), copied});
}
}  // namespace openparasolid::internal
