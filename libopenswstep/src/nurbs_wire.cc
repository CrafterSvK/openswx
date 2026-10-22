// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "nurbs_wire.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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
}  // namespace
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
