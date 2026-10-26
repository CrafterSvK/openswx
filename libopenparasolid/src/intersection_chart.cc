// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "evaluate_internal.h"

namespace openparasolid {
namespace {
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
Result<const GeometryObject*> InvertibleSurface(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  if (!node)
    return Result<const GeometryObject*>::Err(
        Invalid("Intersection chart surface target is missing"));
  switch (node->node_type) {
    case 50:
    case 51:
    case 52:
    case 53:
    case 54:
    case 60:
    case 67:
    case 68:
    case 124:
      break;
    case 56:
    case 59:
    case 120:
    case 201:
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kUnsupportedFormat, node->source_offset,
           "Intersection chart requires supported surface inversion"});
    default:
      return Result<const GeometryObject*>::Err(
          Invalid("Intersection chart target is not a surface"));
  }
  if (!node->geometry)
    return Result<const GeometryObject*>::Err(
        {ErrorCode::kUnsupportedFormat, node->source_offset,
         "Intersection chart surface has no typed payload"});
  const auto* geometry = &model.geometry()[*node->geometry];
  if (!geometry->sense || (*geometry->sense != '+' && *geometry->sense != '-'))
    return Result<const GeometryObject*>::Err(
        {ErrorCode::kUnsupportedFormat, node->source_offset,
         "Intersection chart surface sense is unknown"});
  return Result<const GeometryObject*>::Ok(geometry);
}
ParseError AtPoint(ParseError error, std::size_t point, std::size_t surface) {
  error.message = "Intersection chart point " + std::to_string(point) +
                  ", surface " + std::to_string(surface) + ": " + error.message;
  return error;
}
}  // namespace

Result<IntersectionChartEvaluation> EvaluateIntersectionChart(
    const Model& model, NodeId curve, double tolerance,
    double normal_refinement_tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0 ||
      !std::isfinite(normal_refinement_tolerance) ||
      normal_refinement_tolerance < 0)
    return Result<IntersectionChartEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Intersection chart requires a nonnegative finite distance "
         "tolerance"});
  const auto* node = model.FindNode(curve);
  if (!node || node->node_type != 38)
    return Result<IntersectionChartEvaluation>::Err(
        Invalid("Intersection chart requires an INTERSECTION node"));
  if (!node->geometry)
    return Result<IntersectionChartEvaluation>::Err(
        {ErrorCode::kUnsupportedFormat, node->source_offset,
         "Intersection chart definition has no typed payload"});
  const auto& intersection = std::get<IntersectionCurveGeometry>(
      model.geometry()[*node->geometry].payload);
  const auto& chart = intersection.chart;
  constexpr std::size_t kMaxCoordinateScalars = 1000000;
  const auto count = chart.positions.size();
  if (count > kMaxCoordinateScalars / 3)
    return Result<IntersectionChartEvaluation>::Err(
        {ErrorCode::kResourceLimit, node->source_offset,
         "Intersection chart exceeds coordinate value limit"});
  if (count < 2 || !std::isfinite(chart.base_parameter) ||
      !std::isfinite(chart.base_scale) || chart.base_scale <= 0)
    return Result<IntersectionChartEvaluation>::Err(
        Invalid("Invalid intersection chart count or base parameter/scale"));
  // Borrowed surface pointers remain valid throughout this const Model call.
  std::array<const GeometryObject*, 2> surfaces{};
  for (std::size_t side = 0; side < surfaces.size(); ++side) {
    const auto surface = InvertibleSurface(model, intersection.surfaces[side]);
    if (!surface.ok())
      return Result<IntersectionChartEvaluation>::Err(surface.error());
    surfaces[side] = surface.value();
  }
  IntersectionChartEvaluation result;
  result.surface_parameters.reserve(count);
  result.tangents.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::array<SurfaceParameters, 2> parameters{};
    std::array<Vector3, 2> normals{};
    for (std::size_t side = 0; side < surfaces.size(); ++side) {
      const auto invert = [&](double goal) {
        return i == 0 ? InvertSurface(model, surfaces[side]->source,
                                      chart.positions[i], goal)
                      : InvertSurface(
                            model, surfaces[side]->source, chart.positions[i],
                            goal,
                            SurfaceParameterSeed{
                                result.surface_parameters.back()[side].u,
                                result.surface_parameters.back()[side].v});
      };
      const double goal = normal_refinement_tolerance > 0
                              ? std::min(tolerance, normal_refinement_tolerance)
                              : tolerance;
      auto uv = invert(goal);
      if (!uv.ok() && goal < tolerance &&
          uv.error().code == ErrorCode::kInvalidRecord) {
        // The preferred solve goal must not remove the independently requested
        // membership allowance for approximate stored chart positions.
        uv = invert(tolerance);
        ++result.normal_refinement_fallbacks;
      }
      if (!uv.ok())
        return Result<IntersectionChartEvaluation>::Err(
            AtPoint(uv.error(), i, side));
      parameters[side] = uv.value();
      const auto normal =
          EvaluateSurfaceNormal(model, surfaces[side]->source,
                                parameters[side].u, parameters[side].v);
      if (!normal.ok())
        return Result<IntersectionChartEvaluation>::Err(
            AtPoint(normal.error(), i, side));
      normals[side] = normal.value();
    }
    const auto tangent =
        internal::UnitCrossProduct(normals[0], normals[1], false);
    if (!tangent.ok())
      return Result<IntersectionChartEvaluation>::Err(Invalid(
          "Intersection chart has parallel or unresolvable surface normals"));
    result.surface_parameters.push_back(parameters);
    result.tangents.push_back(tangent.value());
  }
  auto parameterized = ParameterizeIntersectionChart(chart, result.tangents);
  if (!parameterized.ok())
    return Result<IntersectionChartEvaluation>::Err(parameterized.error());
  result.parameterization = std::move(parameterized).value();
  return Result<IntersectionChartEvaluation>::Ok(std::move(result));
}
}  // namespace openparasolid
