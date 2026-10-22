// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "planar_boundary.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>

namespace openswstep::internal {
namespace {
using openparasolid::ErrorCode;
using openparasolid::ParseError;
using openparasolid::Result;
using openparasolid::Vector3;
struct Point2 {
  long double x;
  long double y;
};
struct Loop {
  std::vector<Point2> points;
  long double area = 0;
};
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
std::optional<ParseError> Charge(uint64_t& remaining) {
  if (!remaining)
    return ParseError{ErrorCode::kResourceLimit, 0,
                      "Planar boundary comparison limit exceeded"};
  --remaining;
  return std::nullopt;
}
long double Turn(Point2 a, Point2 b, Point2 c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
long double Project(Vector3 point, Vector3 origin, Vector3 axis) {
  return (static_cast<long double>(point.x) - origin.x) * axis.x +
         (static_cast<long double>(point.y) - origin.y) * axis.y +
         (static_cast<long double>(point.z) - origin.z) * axis.z;
}
Result<Loop> ProjectLoop(std::span<const Vector3> polygon,
                         const openparasolid::PlaneGeometry& plane,
                         Vector3 origin, double tolerance) {
  const auto& normal = plane.normal;
  const auto& x = plane.x_axis;
  const Vector3 y{normal.y * x.z - normal.z * x.y,
                  normal.z * x.x - normal.x * x.z,
                  normal.x * x.y - normal.y * x.x};
  Loop loop;
  loop.points.reserve(polygon.size());
  for (const auto point : polygon) {
    const auto residual = Project(point, plane.origin, normal);
    const Point2 uv{Project(point, origin, x), Project(point, origin, y)};
    if (!std::isfinite(residual) || std::abs(residual) > tolerance ||
        !std::isfinite(uv.x) || !std::isfinite(uv.y))
      return Result<Loop>::Err(Invalid(
          "Polygon vertex is outside the plane tolerance or projection range"));
    loop.points.push_back(uv);
  }
  long double perimeter = 0;
  // Relative coordinates keep each loop's area accurate even if a different
  // source loop supplied the common projection origin.
  const auto first = loop.points.front();
  for (std::size_t i = 0; i < loop.points.size(); ++i) {
    const auto a = loop.points[i],
               b = loop.points[(i + 1) % loop.points.size()];
    const auto length = std::hypot(b.x - a.x, b.y - a.y);
    if (!std::isfinite(length) || length <= tolerance)
      return Result<Loop>::Err(
          Invalid("Polygon edge is degenerate at the export tolerance"));
    loop.area += Turn(first, a, b);
    perimeter += length;
  }
  if (!std::isfinite(loop.area) || !std::isfinite(perimeter) ||
      std::abs(loop.area) <= tolerance * perimeter)
    return Result<Loop>::Err(
        Invalid("Polygon area is degenerate or unrepresentable"));
  return Result<Loop>::Ok(std::move(loop));
}
std::optional<ParseError> CheckSegments(Point2 a, Point2 b, Point2 c, Point2 d,
                                        double tolerance, uint64_t& remaining) {
  if (auto error = Charge(remaining)) return error;
  if (std::max(a.x, b.x) + tolerance < std::min(c.x, d.x) ||
      std::max(c.x, d.x) + tolerance < std::min(a.x, b.x) ||
      std::max(a.y, b.y) + tolerance < std::min(c.y, d.y) ||
      std::max(c.y, d.y) + tolerance < std::min(a.y, b.y))
    return std::nullopt;
  const auto epsilon = tolerance * std::max(std::hypot(b.x - a.x, b.y - a.y),
                                            std::hypot(d.x - c.x, d.y - c.y));
  const auto ab_c = Turn(a, b, c), ab_d = Turn(a, b, d), cd_a = Turn(c, d, a),
             cd_b = Turn(c, d, b);
  if (!std::isfinite(epsilon) || !std::isfinite(ab_c) || !std::isfinite(ab_d) ||
      !std::isfinite(cd_a) || !std::isfinite(cd_b))
    return Invalid("Planar boundary comparison is unrepresentable");
  const auto opposite = [epsilon](long double u, long double v) {
    return !(u > epsilon && v > epsilon) && !(u < -epsilon && v < -epsilon);
  };
  if (opposite(ab_c, ab_d) && opposite(cd_a, cd_b))
    return Invalid(
        "Planar boundary edges cross, touch or are tolerance-ambiguous");
  return std::nullopt;
}
Result<bool> Contains(const Loop& loop, Point2 point, uint64_t& remaining) {
  bool inside = false;
  for (std::size_t i = 0; i < loop.points.size(); ++i) {
    if (const auto error = Charge(remaining)) return Result<bool>::Err(*error);
    const auto a = loop.points[i],
               b = loop.points[(i + 1) % loop.points.size()];
    if ((a.y > point.y) == (b.y > point.y)) continue;
    const auto fraction = (point.y - a.y) / (b.y - a.y);
    const auto crossing = a.x + fraction * (b.x - a.x);
    if (!std::isfinite(crossing))
      return Result<bool>::Err(
          Invalid("Planar boundary containment is unrepresentable"));
    if (point.x < crossing) inside = !inside;
  }
  return Result<bool>::Ok(inside);
}
}  // namespace
openparasolid::Result<std::size_t> ValidatePlanarLoops(
    std::span<const std::vector<Vector3>> polygons,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks) {
  using Return = Result<std::size_t>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Planar boundary tolerance must be positive and finite"});
  if (polygons.empty())
    return Return::Err(Invalid("Planar face has no boundary loops"));
  for (const auto& polygon : polygons)
    if (polygon.size() < 3)
      return Return::Err(Invalid("Polygon requires at least three vertices"));
  std::vector<Loop> loops;
  loops.reserve(polygons.size());
  std::optional<std::size_t> outer;
  const auto origin = polygons.front().front();
  for (const auto& polygon : polygons) {
    auto loop = ProjectLoop(polygon, plane, origin, tolerance);
    if (!loop.ok()) return Return::Err(loop.error());
    if ((loop.value().area > 0) == natural_sense) {
      if (outer)
        return Return::Err(
            Invalid("Planar face has multiple outward-wound loops"));
      outer = loops.size();
    }
    loops.push_back(std::move(loop).value());
  }
  if (!outer)
    return Return::Err(
        Invalid("Planar face has no correctly wound outer loop"));
  for (std::size_t l = 0; l < loops.size(); ++l) {
    const auto& points = loops[l].points;
    for (std::size_t i = 0; i < points.size(); ++i)
      for (std::size_t j = i + 1; j < points.size(); ++j) {
        if (j == i + 1 || (i == 0 && j + 1 == points.size())) continue;
        if (const auto error = CheckSegments(
                points[i], points[(i + 1) % points.size()], points[j],
                points[(j + 1) % points.size()], tolerance, remaining_checks))
          return Return::Err(*error);
      }
    for (std::size_t other = l + 1; other < loops.size(); ++other)
      for (std::size_t i = 0; i < points.size(); ++i)
        for (std::size_t j = 0; j < loops[other].points.size(); ++j) {
          const auto& second = loops[other].points;
          if (const auto error = CheckSegments(
                  points[i], points[(i + 1) % points.size()], second[j],
                  second[(j + 1) % second.size()], tolerance, remaining_checks))
            return Return::Err(*error);
        }
  }
  for (std::size_t i = 0; i < loops.size(); ++i) {
    if (i == *outer) continue;
    const auto inside =
        Contains(loops[*outer], loops[i].points.front(), remaining_checks);
    if (!inside.ok()) return Return::Err(inside.error());
    if (!inside.value())
      return Return::Err(Invalid("Hole lies outside the outer polygon"));
    for (std::size_t j = i + 1; j < loops.size(); ++j) {
      if (j == *outer) continue;
      const auto first =
          Contains(loops[i], loops[j].points.front(), remaining_checks);
      if (!first.ok()) return Return::Err(first.error());
      const auto second =
          Contains(loops[j], loops[i].points.front(), remaining_checks);
      if (!second.ok()) return Return::Err(second.error());
      if (first.value() || second.value())
        return Return::Err(Invalid(
            "Nested hole loops do not define one connected planar face"));
    }
  }
  return Return::Ok(*outer);
}
Result<PerforatedPolygon> ValidatePolygonCircularHoles(
    std::span<const std::vector<Vector3>> polygons,
    std::span<const ConicBoundary> circles,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks) {
  using Return = Result<PerforatedPolygon>;
  for (std::size_t i = 0; i < circles.size(); ++i) {
    if (!circles[i].circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Polygon conic holes require CIRCLE bases"});
    // A hole is a correctly wound single conic for the opposite face normal.
    // Reuse the full-turn, entire-plane and radius checks of the conic subset.
    const auto valid =
        ValidatePlanarConics(circles.subspan(i, 1), plane, !natural_sense,
                             tolerance, remaining_checks);
    if (!valid.ok()) {
      const auto outward =
          ValidatePlanarConics(circles.subspan(i, 1), plane, natural_sense,
                               tolerance, remaining_checks);
      if (outward.ok())
        return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                            "Circular outer loops with polygon boundaries "
                            "are not supported"});
      return Return::Err(valid.error());
    }
  }
  const auto outer = ValidatePlanarLoops(polygons, plane, natural_sense,
                                         tolerance, remaining_checks);
  if (!outer.ok()) return Return::Err(outer.error());
  const auto origin = polygons.front().front();
  const auto& n = plane.normal;
  const auto& x = plane.x_axis;
  const Vector3 y{n.y * x.z - n.z * x.y, n.z * x.x - n.x * x.z,
                  n.x * x.y - n.y * x.x};
  const long double clearance = 2 * static_cast<long double>(tolerance);
  std::vector<Loop> loops;
  long double area = 0;
  for (const auto& polygon : polygons) {
    auto projected = ProjectLoop(polygon, plane, origin, tolerance);
    if (!projected.ok()) return Return::Err(projected.error());
    area += (natural_sense ? 1 : -1) * projected.value().area / 2;
    loops.push_back(std::move(projected).value());
  }
  for (std::size_t i = 0; i < circles.size(); ++i) {
    const auto& circle = circles[i];
    const Point2 center{Project(circle.center, origin, x),
                        Project(circle.center, origin, y)};
    const long double radius = circle.major_radius;
    for (std::size_t l = 0; l < loops.size(); ++l) {
      const auto& loop = loops[l];
      for (std::size_t e = 0; e < loop.points.size(); ++e) {
        if (const auto error = Charge(remaining_checks))
          return Return::Err(*error);
        const auto a = loop.points[e],
                   b = loop.points[(e + 1) % loop.points.size()];
        const auto length = std::hypot(b.x - a.x, b.y - a.y);
        const auto dx = (b.x - a.x) / length, dy = (b.y - a.y) / length;
        const auto along = std::clamp(
            (center.x - a.x) * dx + (center.y - a.y) * dy, 0.L, length);
        const auto distance = std::hypot(center.x - a.x - along * dx,
                                         center.y - a.y - along * dy);
        if (!std::isfinite(distance) || distance <= radius + clearance)
          return Return::Err(Invalid(
              "Circular hole crosses, encloses, touches or is too close to "
              "a polygon boundary"));
      }
      const auto inside = Contains(loop, center, remaining_checks);
      if (!inside.ok()) return Return::Err(inside.error());
      if (inside.value() != (l == outer.value()))
        return Return::Err(
            Invalid("Circular hole center is outside the polygon material"));
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (const auto error = Charge(remaining_checks))
        return Return::Err(*error);
      const auto& other = circles[j];
      const auto separation = std::hypot(
          static_cast<long double>(circle.center.x) - other.center.x,
          static_cast<long double>(circle.center.y) - other.center.y,
          static_cast<long double>(circle.center.z) - other.center.z);
      if (!std::isfinite(separation) ||
          separation <= radius + other.major_radius + clearance)
        return Return::Err(
            Invalid("Circular holes overlap, nest, touch or are too close"));
    }
    area -= std::numbers::pi_v<long double> * radius * radius;
  }
  if (!std::isfinite(area) || area <= 0)
    return Return::Err(
        Invalid("Perforated polygon area is nonpositive or "
                "unrepresentable"));
  return Return::Ok({outer.value(), area});
}
}  // namespace openswstep::internal
