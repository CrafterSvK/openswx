// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "nurbs_wire.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace openswstep::internal {
namespace {
using namespace openparasolid;
Vector3 Control(const NurbsCurveGeometry& curve, std::size_t index) {
  const auto at = index * curve.controls.dimension;
  const long double weight =
      curve.controls.rational ? curve.controls.values[at + 3] : 1;
  return {static_cast<double>(curve.controls.values[at] / weight),
          static_cast<double>(curve.controls.values[at + 1] / weight),
          static_cast<double>(curve.controls.values[at + 2] / weight)};
}
Result<bool> Unsupported(const char* message) {
  return Result<bool>::Err({ErrorCode::kUnsupportedFormat, 0, message});
}
Result<bool> InsertKnot(NurbsCurveGeometry& curve, std::vector<double>& knots,
                        double parameter, uint64_t& checks_remaining) {
  const auto degree = curve.axis.degree;
  const auto count = curve.axis.control_count;
  const auto dimension = curve.controls.dimension;
  const auto end = std::upper_bound(knots.begin(), knots.end(), parameter);
  const auto span = static_cast<std::size_t>(end - knots.begin() - 1);
  const auto multiplicity = static_cast<std::size_t>(
      end - std::lower_bound(knots.begin(), end, parameter));
  // Keep temporary arrays inside the reader evaluator's scalar limit.
  constexpr std::size_t kMaxValues = 1000000;
  if (curve.controls.values.size() > kMaxValues - dimension ||
      checks_remaining < uint64_t{count} + 1)
    return Result<bool>::Err({ErrorCode::kResourceLimit, 0,
                              "STEP spline knot insertion limit exceeded"});
  checks_remaining -= uint64_t{count} + 1;
  std::vector<double> controls((std::size_t{count} + 1) * dimension);
  for (std::size_t i = 0; i <= span - degree; ++i)
    std::copy_n(curve.controls.values.begin() + i * dimension, dimension,
                controls.begin() + i * dimension);
  for (std::size_t i = span - multiplicity; i < count; ++i)
    std::copy_n(curve.controls.values.begin() + i * dimension, dimension,
                controls.begin() + (i + 1) * dimension);
  for (std::size_t i = span - degree + 1; i <= span - multiplicity; ++i) {
    const long double width =
        static_cast<long double>(knots[i + degree]) - knots[i];
    if (!std::isfinite(width) || width <= 0)
      return Result<bool>::Err({ErrorCode::kInvalidRecord, 0,
                                "STEP spline knot insertion span is invalid"});
    const long double fraction =
        (static_cast<long double>(parameter) - knots[i]) / width;
    for (std::size_t j = 0; j < dimension; ++j) {
      const long double value = std::lerp(
          static_cast<long double>(
              curve.controls.values[(i - 1) * dimension + j]),
          static_cast<long double>(curve.controls.values[i * dimension + j]),
          fraction);
      const auto converted = static_cast<double>(value);
      if (!std::isfinite(converted) ||
          (curve.controls.rational && j + 1 == dimension && converted <= 0))
        return Result<bool>::Err(
            {ErrorCode::kInvalidRecord, 0,
             "STEP spline inserted control is not representable"});
      controls[i * dimension + j] = converted;
    }
  }
  knots.insert(end, parameter);
  curve.controls.values = std::move(controls);
  ++curve.axis.control_count;
  return Result<bool>::Ok(true);
}
}  // namespace
Result<openparasolid::NurbsCurveGeometry> RestrictNurbsWire(
    const openparasolid::NurbsCurveGeometry& curve, double first, double last,
    uint64_t& checks_remaining) {
  using Return = Result<openparasolid::NurbsCurveGeometry>;
  // Validate all sizes, scalars and source parameter bounds before indexing.
  const auto a = openparasolid::EvaluateCurve(curve, first);
  if (!a.ok()) return Return::Err(a.error());
  const auto b = openparasolid::EvaluateCurve(curve, last);
  if (!b.ok()) return Return::Err(b.error());
  if (first == last)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "STEP spline interval must have distinct endpoints"});
  if (curve.controls.dimension != (curve.controls.rational ? 4 : 3) ||
      curve.axis.periodic || curve.axis.closed ||
      curve.axis.multiplicities.front() != curve.axis.degree + 1 ||
      curve.axis.multiplicities.back() != curve.axis.degree + 1)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "STEP wire splines require clamped open 3D knot axes"});
  const auto low = std::min(first, last), high = std::max(first, last);
  if (low == curve.axis.knots.front() && high == curve.axis.knots.back())
    return Return::Ok(curve);
  if (checks_remaining < curve.axis.control_count)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "STEP spline restriction copy limit exceeded"});
  checks_remaining -= curve.axis.control_count;
  auto restricted = curve;
  std::vector<double> knots;
  for (std::size_t i = 0; i < curve.axis.knots.size(); ++i)
    knots.insert(knots.end(), curve.axis.multiplicities[i],
                 curve.axis.knots[i]);
  // Interior multiplicity degree makes both adjoining pieces independently
  // selectable. Adding one endpoint knot after cropping clamps the selection.
  for (double parameter : {low, high}) {
    auto multiplicity = static_cast<uint32_t>(
        std::count(knots.begin(), knots.end(), parameter));
    while (multiplicity < curve.axis.degree) {
      const auto inserted =
          InsertKnot(restricted, knots, parameter, checks_remaining);
      if (!inserted.ok()) return Return::Err(inserted.error());
      ++multiplicity;
    }
  }
  const auto first_control = static_cast<std::size_t>(
      std::upper_bound(knots.begin(), knots.end(), low) - knots.begin() - 1 -
      curve.axis.degree);
  const auto after_control = static_cast<std::size_t>(
      std::lower_bound(knots.begin(), knots.end(), high) - knots.begin());
  const auto dimension = curve.controls.dimension;
  std::vector<double> controls(
      restricted.controls.values.begin() + first_control * dimension,
      restricted.controls.values.begin() + after_control * dimension);
  restricted.controls.values = std::move(controls);
  restricted.axis.control_count =
      static_cast<uint32_t>(after_control - first_control);
  restricted.axis.knots = {low};
  restricted.axis.multiplicities = {
      static_cast<uint16_t>(curve.axis.degree + 1)};
  for (std::size_t i = 0; i < curve.axis.knots.size(); ++i)
    if (curve.axis.knots[i] > low && curve.axis.knots[i] < high) {
      restricted.axis.knots.push_back(curve.axis.knots[i]);
      restricted.axis.multiplicities.push_back(curve.axis.multiplicities[i]);
    }
  restricted.axis.knots.push_back(high);
  restricted.axis.multiplicities.push_back(
      static_cast<uint16_t>(curve.axis.degree + 1));
  // Cropped poles and end multiplicities generally change these source
  // classification bytes. Serialize an unspecified STEP form instead.
  restricted.axis.knot_type = 0;
  restricted.curve_form = 0;
  return Return::Ok(std::move(restricted));
}
Result<bool> ValidateNurbsWire(const openparasolid::NurbsCurveGeometry& curve,
                               double first, double last, double tolerance,
                               uint64_t& checks_remaining) {
  // The public evaluator validates dimensions, finite scalars, positive
  // weights, knot ordering/counts and its independent resource limits before
  // any direct access below. It also checks both requested parameters.
  const auto start = openparasolid::EvaluateCurve(curve, first);
  if (!start.ok()) return Result<bool>::Err(start.error());
  const auto end = openparasolid::EvaluateCurve(curve, last);
  if (!end.ok()) return Result<bool>::Err(end.error());
  if (curve.controls.dimension != (curve.controls.rational ? 4 : 3))
    return Unsupported("STEP wire splines require three spatial coordinates");
  if (curve.axis.periodic || curve.axis.closed ||
      curve.axis.multiplicities.front() != curve.axis.degree + 1 ||
      curve.axis.multiplicities.back() != curve.axis.degree + 1)
    return Unsupported("STEP wire splines require clamped open knot axes");
  if (std::min(first, last) != curve.axis.knots.front() ||
      std::max(first, last) != curve.axis.knots.back())
    return Unsupported(
        "STEP wire splines require their complete natural domain");

  const auto a = Control(curve, 0);
  const auto b = Control(curve, curve.axis.control_count - 1);
  const std::array<long double, 3> delta{static_cast<long double>(b.x) - a.x,
                                         static_cast<long double>(b.y) - a.y,
                                         static_cast<long double>(b.z) - a.z};
  const auto length = std::hypot(delta[0], delta[1], delta[2]);
  const long double scale =
      std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z), std::abs(b.x),
                std::abs(b.y), std::abs(b.z)});
  const auto clearance = 2 * static_cast<long double>(tolerance) +
                         128 * std::numeric_limits<double>::epsilon() * scale;
  if (!std::isfinite(length) || length <= 2 * clearance)
    return Unsupported("STEP spline endpoints are not resolved and separated");
  for (std::size_t i = 1; i + 1 < curve.axis.control_count; ++i) {
    if (checks_remaining == 0)
      return Result<bool>::Err({ErrorCode::kResourceLimit, 0,
                                "STEP spline control check limit exceeded"});
    --checks_remaining;
    const auto point = Control(curve, i);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      return Result<bool>::Err({ErrorCode::kInvalidRecord, 0,
                                "STEP spline control projection overflow"});
    const auto projection =
        ((static_cast<long double>(point.x) - a.x) * delta[0] +
         (static_cast<long double>(point.y) - a.y) * delta[1] +
         (static_cast<long double>(point.z) - a.z) * delta[2]) /
        length;
    const auto margin =
        clearance +
        128 * std::numeric_limits<double>::epsilon() *
            std::max({std::abs(point.x), std::abs(point.y), std::abs(point.z)});
    if (!std::isfinite(projection) || projection <= margin ||
        length - projection <= margin)
      return Unsupported(
          "STEP spline endpoint supporting planes are unresolved");
  }
  return Result<bool>::Ok(true);
}
Id WriteNurbsCurve(StepWriter& writer,
                   const openparasolid::NurbsCurveGeometry& curve) {
  std::vector<Id> points;
  for (std::size_t i = 0; i < curve.axis.control_count; ++i) {
    const auto point = writer.Point(Control(curve, i));
    if (!point) return 0;
    points.push_back(point);
  }
  std::string multiplicities = "(", knots = "(", weights = "(";
  for (std::size_t i = 0; i < curve.axis.knots.size(); ++i) {
    if (i) {
      multiplicities += ',';
      knots += ',';
    }
    multiplicities += std::to_string(curve.axis.multiplicities[i]);
    knots += writer.Number(curve.axis.knots[i]);
  }
  multiplicities += ')';
  knots += ')';
  const auto attributes = std::to_string(curve.axis.degree) + ',' +
                          References(points) + ",.UNSPECIFIED.,.F.,.U.";
  const auto knot_attributes = multiplicities + ',' + knots + ",.UNSPECIFIED.";
  if (!curve.controls.rational)
    return writer.Add("B_SPLINE_CURVE_WITH_KNOTS(''," + attributes + ',' +
                      knot_attributes + ')');
  for (std::size_t i = 0; i < curve.axis.control_count; ++i) {
    if (i) weights += ',';
    weights += writer.Number(curve.controls.values[i * 4 + 3]);
  }
  weights += ')';
  return writer.Add("(BOUNDED_CURVE() B_SPLINE_CURVE(" + attributes +
                    ") B_SPLINE_CURVE_WITH_KNOTS(" + knot_attributes +
                    ") CURVE() GEOMETRIC_REPRESENTATION_ITEM() "
                    "RATIONAL_B_SPLINE_CURVE(" +
                    weights + ") REPRESENTATION_ITEM(''))");
}
}  // namespace openswstep::internal
