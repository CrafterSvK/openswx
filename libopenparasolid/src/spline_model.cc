// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <cmath>
#include <string>
#include <utility>

#include "model_internal.h"

namespace openparasolid::internal {
namespace {
ParseError Invalid(const DecodedNode& node, const char* message) {
  return {ErrorCode::kInvalidRecord, node.offset, message};
}
const DecodedField* Array(const GeometryContext& context, NodeId id,
                          uint16_t type, const char* name, FieldType codec) {
  const auto* node = context.Resolve(id, type);
  return node ? context.Fields(*node).Find(name, codec) : nullptr;
}
bool Doubles(const DecodedField& field) {
  for (const auto& value : field.values) {
    const auto* number = std::get_if<double>(&value);
    if (!number || !std::isfinite(*number)) return false;
  }
  return true;
}
struct AxisSource {
  SplineAxis axis;
  uint32_t knot_count = 0;
  NodeId knots;
  NodeId multiplicities;
};
bool ReadAxis(ModelFields& fields, bool surface, bool v, AxisSource& source) {
  const std::string prefix = surface ? v ? "v_" : "u_" : "";
  const std::string count =
      surface ? v ? "n_v_vertices" : "n_u_vertices" : "n_vertices";
  const std::string knots = surface ? v ? "n_v_knots" : "n_u_knots" : "n_knots";
  return fields.Integer(prefix + "degree", source.axis.degree) &&
         fields.Integer(count, source.axis.control_count) &&
         fields.Integer(knots, source.knot_count) &&
         fields.Logical(prefix + "periodic", source.axis.periodic) &&
         fields.Logical(prefix + "closed", source.axis.closed) &&
         fields.Byte(prefix + "knot_type", source.axis.knot_type) &&
         fields.Reference(prefix + "knots", source.knots) &&
         fields.Reference(prefix + "knot_mult", source.multiplicities);
}
std::optional<ParseError> ResolveAxis(const DecodedNode& node,
                                      const GeometryContext& context,
                                      AxisSource& source, uint64_t& remaining) {
  auto& axis = source.axis;
  if (!axis.degree || axis.degree >= axis.control_count ||
      source.knot_count < 2 || (axis.periodic && !axis.closed))
    return Invalid(node,
                   "Invalid spline degree, control count or closure flags");
  const auto* knots =
      Array(context, source.knots, 128, "knots", FieldType::kDouble);
  const auto* mult = Array(context, source.multiplicities, 127,
                           "multiplicities", FieldType::kShort);
  if (!knots || !mult || knots->values.size() != source.knot_count ||
      mult->values.size() != source.knot_count || !Doubles(*knots))
    return Invalid(node,
                   "Spline knot arrays are missing, unset or inconsistent with "
                   "the descriptor");
  uint64_t expanded = 0;
  double previous = 0;
  for (std::size_t i = 0; i < knots->values.size(); ++i) {
    const double knot = *std::get_if<double>(&knots->values[i]);
    const auto* count = std::get_if<int64_t>(&mult->values[i]);
    if (!count || *count <= 0 ||
        static_cast<uint64_t>(*count) > uint64_t{axis.degree} + 1 ||
        (i != 0 && knot <= previous) ||
        (i != 0 && i + 1 != knots->values.size() &&
         static_cast<uint64_t>(*count) > axis.degree))
      return Invalid(node, "Invalid distinct knot order or multiplicity");
    expanded += static_cast<uint64_t>(*count);
    previous = knot;
  }
  if (expanded != uint64_t{axis.control_count} + axis.degree + 1)
    return Invalid(
        node, "Expanded knot count does not match control count and degree");
  const uint64_t cost = uint64_t{source.knot_count} * 2;
  if (cost > remaining)
    return ParseError{ErrorCode::kResourceLimit, node.offset,
                      "Copied spline data exceeds model value limit"};
  remaining -= cost;
  axis.knots.reserve(source.knot_count);
  axis.multiplicities.reserve(source.knot_count);
  for (std::size_t i = 0; i < knots->values.size(); ++i) {
    axis.knots.push_back(*std::get_if<double>(&knots->values[i]));
    axis.multiplicities.push_back(
        static_cast<uint16_t>(*std::get_if<int64_t>(&mult->values[i])));
  }
  return std::nullopt;
}
std::optional<ParseError> ResolveControls(const DecodedNode& node,
                                          ModelFields& fields,
                                          const GeometryContext& context,
                                          bool surface, uint64_t count,
                                          SplineControlNet& controls,
                                          uint64_t& remaining) {
  uint32_t dimension = 0;
  if (!fields.Integer("vertex_dim", dimension) ||
      !fields.Logical("rational", controls.rational) ||
      !fields.Reference("bspline_vertices", controls.source))
    return fields.Error();
  const uint32_t spatial = surface ? 3u
                           : controls.rational
                               ? dimension - (dimension > 0 ? 1u : 0u)
                               : dimension;
  if ((surface && dimension != (controls.rational ? 4u : 3u)) ||
      (!surface && spatial != 2 && spatial != 3))
    return Invalid(node, "Unsupported spline control dimension");
  const auto* data =
      Array(context, controls.source, 45, "vertices", FieldType::kDouble);
  if (!data || count > UINT64_MAX / dimension ||
      data->values.size() != count * dimension || !Doubles(*data))
    return Invalid(node,
                   "Spline control array is missing, unset or inconsistent "
                   "with the descriptor");
  if (controls.rational) {
    for (std::size_t i = dimension - 1; i < data->values.size(); i += dimension)
      if (*std::get_if<double>(&data->values[i]) <= 0)
        return Invalid(node, "Rational spline weights must be positive");
  }
  if (data->values.size() > remaining)
    return ParseError{ErrorCode::kResourceLimit, node.offset,
                      "Copied spline data exceeds model value limit"};
  remaining -= data->values.size();
  controls.dimension = static_cast<uint16_t>(dimension);
  controls.values.reserve(data->values.size());
  for (const auto& value : data->values)
    controls.values.push_back(*std::get_if<double>(&value));
  return std::nullopt;
}
}  // namespace
Result<BuiltGeometry> BuildSpline(const DecodedNode& node, ModelFields& fields,
                                  const GeometryContext& context) {
  const bool surface = node.node_type == 124;
  NodeId descriptor;
  if (!fields.Reference("nurbs", descriptor))
    return Result<BuiltGeometry>::Err(fields.Error());
  const auto* data = context.Resolve(descriptor, surface ? 126 : 136);
  if (!data)
    return Result<BuiltGeometry>::Err(
        Invalid(node, "Missing or incompatible NURBS descriptor"));
  auto descriptor_fields = context.Fields(*data);
  AxisSource u, v;
  if (!ReadAxis(descriptor_fields, surface, false, u) ||
      (surface && !ReadAxis(descriptor_fields, true, true, v)))
    return Result<BuiltGeometry>::Err(descriptor_fields.Error());
  uint64_t remaining = context.remaining_values;
  if (auto error = ResolveAxis(node, context, u, remaining))
    return Result<BuiltGeometry>::Err(std::move(*error));
  if (surface) {
    if (auto error = ResolveAxis(node, context, v, remaining))
      return Result<BuiltGeometry>::Err(std::move(*error));
  }
  SplineControlNet controls;
  const uint64_t count =
      uint64_t{u.axis.control_count} * (surface ? v.axis.control_count : 1u);
  if (auto error = ResolveControls(node, descriptor_fields, context, surface,
                                   count, controls, remaining))
    return Result<BuiltGeometry>::Err(std::move(*error));
  uint8_t form = 0;
  if (!descriptor_fields.Byte(surface ? "surface_form" : "curve_form", form))
    return Result<BuiltGeometry>::Err(descriptor_fields.Error());
  const uint64_t copied = context.remaining_values - remaining;
  if (surface)
    return Result<BuiltGeometry>::Ok(
        {NurbsSurfaceGeometry{descriptor, std::move(u.axis), std::move(v.axis),
                              std::move(controls), form},
         copied});
  return Result<BuiltGeometry>::Ok(
      {NurbsCurveGeometry{descriptor, std::move(u.axis), std::move(controls),
                          form},
       copied});
}
}  // namespace openparasolid::internal
