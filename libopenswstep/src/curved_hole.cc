// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "curved_hole.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
long double Dot(Vector3 a, Vector3 b) {
  return static_cast<long double>(a.x) * b.x +
         static_cast<long double>(a.y) * b.y +
         static_cast<long double>(a.z) * b.z;
}
long double Project(Vector3 p, Vector3 origin, Vector3 axis) {
  return (static_cast<long double>(p.x) - origin.x) * axis.x +
         (static_cast<long double>(p.y) - origin.y) * axis.y +
         (static_cast<long double>(p.z) - origin.z) * axis.z;
}
Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vector3 Unit(Vector3 a) {
  const auto length = std::hypot(a.x, a.y, a.z);
  return {a.x / length, a.y / length, a.z / length};
}
struct Envelope {
  std::vector<Vector3> points;
  long double area;
};
Result<Envelope> ReadEnvelope(std::span<const PlanarCurve> curves,
                              const PlaneGeometry& plane, bool natural_sense,
                              double tolerance, uint64_t& remaining) {
  using Return = Result<Envelope>;
  for (const auto& curve : curves)
    if (!curve.geometry ||
        (!std::holds_alternative<LineGeometry>(*curve.geometry) &&
         !std::holds_alternative<CircleGeometry>(*curve.geometry)))
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Convex curved holes require LINE/CIRCLE bases"});
  const auto valid = ValidateConvexPlanarLoop(curves, plane, !natural_sense,
                                              tolerance, remaining);
  if (!valid.ok()) return Return::Err(valid.error());
  const auto origin = valid.value().reference_point;
  const auto n = Unit(plane.normal);
  const auto dot = Dot(plane.x_axis, n);
  const auto x = Unit({static_cast<double>(plane.x_axis.x - dot * n.x),
                       static_cast<double>(plane.x_axis.y - dot * n.y),
                       static_cast<double>(plane.x_axis.z - dot * n.z)});
  auto y = Unit(Cross(n, x));
  if (natural_sense) y = {-y.x, -y.y, -y.z};
  std::array<long double, 2> minimum{
      std::numeric_limits<long double>::infinity(),
      std::numeric_limits<long double>::infinity()};
  auto maximum = std::array<long double, 2>{-minimum[0], -minimum[1]};
  const std::array<Vector3, 2> axes{x, y};
  for (const auto& curve : curves) {
    if (!remaining)
      return Return::Err({ErrorCode::kResourceLimit, 0,
                          "Curved hole envelope limit exceeded"});
    --remaining;
    for (std::size_t i = 0; i < axes.size(); ++i) {
      long double low, high;
      if (const auto* circle = std::get_if<CircleGeometry>(curve.geometry)) {
        const auto center = Project(circle->center, origin, axes[i]);
        const auto a = circle->radius * Dot(circle->x_axis, axes[i]);
        const auto b = circle->radius *
                       Dot(Cross(circle->normal, circle->x_axis), axes[i]);
        low = center - MaximumArcProjection(-a, -b, curve.start, curve.end);
        high = center + MaximumArcProjection(a, b, curve.start, curve.end);
      } else {
        const auto a = EvaluateCurve(*curve.geometry, curve.start);
        const auto b = EvaluateCurve(*curve.geometry, curve.end);
        if (!a.ok()) return Return::Err(a.error());
        if (!b.ok()) return Return::Err(b.error());
        const auto u = Project(a.value().position, origin, axes[i]);
        const auto v = Project(b.value().position, origin, axes[i]);
        low = std::min(u, v);
        high = std::max(u, v);
      }
      if (!std::isfinite(low) || !std::isfinite(high))
        return Return::Err({ErrorCode::kInvalidRecord, 0,
                            "Curved hole extrema are unrepresentable"});
      minimum[i] = std::min(minimum[i], low);
      maximum[i] = std::max(maximum[i], high);
    }
  }
  // Expand for source tolerance and the conversion back to double coordinates.
  // This envelope is never serialized, fitted to the source, or used for area.
  const auto scale = std::max({std::abs(static_cast<long double>(origin.x)),
                               std::abs(static_cast<long double>(origin.y)),
                               std::abs(static_cast<long double>(origin.z)),
                               std::abs(minimum[0]), std::abs(minimum[1]),
                               std::abs(maximum[0]), std::abs(maximum[1])});
  const auto padding = 2 * static_cast<long double>(tolerance) +
                       16 * std::numeric_limits<double>::epsilon() * scale;
  for (std::size_t i = 0; i < axes.size(); ++i) {
    minimum[i] -= padding;
    maximum[i] += padding;
  }
  Envelope result{{}, valid.value().area};
  for (const auto uv :
       {std::array{minimum[0], minimum[1]}, std::array{maximum[0], minimum[1]},
        std::array{maximum[0], maximum[1]},
        std::array{minimum[0], maximum[1]}}) {
    const std::array<long double, 3> coordinates{
        origin.x + uv[0] * x.x + uv[1] * y.x,
        origin.y + uv[0] * x.y + uv[1] * y.y,
        origin.z + uv[0] * x.z + uv[1] * y.z};
    for (const auto coordinate : coordinates)
      if (!std::isfinite(coordinate) ||
          std::abs(coordinate) > std::numeric_limits<double>::max())
        return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                            "Curved hole envelope is unrepresentable"});
    result.points.push_back({static_cast<double>(coordinates[0]),
                             static_cast<double>(coordinates[1]),
                             static_cast<double>(coordinates[2])});
  }
  return Return::Ok(std::move(result));
}
}  // namespace
Result<PlanarRegion> ValidateConvexCurvedHoles(
    std::span<const std::vector<PlanarCurve>> curved_holes,
    std::span<const std::vector<Vector3>> polygons,
    std::span<const ConicBoundary> circles, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<PlanarRegion>;
  const auto base = ValidateMixedPlanarLoops(
      polygons, circles, plane, natural_sense, tolerance, remaining_checks);
  if (!base.ok()) return base;
  auto result = base.value();
  std::vector<std::vector<Vector3>> enclosed(polygons.begin(), polygons.end());
  for (const auto& curves : curved_holes) {
    const auto hole =
        ReadEnvelope(curves, plane, natural_sense, tolerance, remaining_checks);
    if (!hole.ok()) return Return::Err(hole.error());
    result.area -= hole.value().area;
    enclosed.push_back(hole.value().points);
  }
  const auto checked = ValidateMixedPlanarLoops(
      enclosed, circles, plane, natural_sense, tolerance, remaining_checks);
  if (!checked.ok()) {
    if (checked.error().code == ErrorCode::kResourceLimit)
      return Return::Err(checked.error());
    return Return::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Curved hole envelopes cannot certify disjoint containment: " +
             checked.error().message});
  }
  if (!std::isfinite(result.area) || result.area <= 0)
    return Return::Err({ErrorCode::kInvalidRecord, 0,
                        "Curved hole face net area is nonpositive"});
  return Return::Ok(result);
}
}  // namespace openswstep::internal
