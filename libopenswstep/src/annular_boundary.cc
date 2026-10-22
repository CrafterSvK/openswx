// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "annular_boundary.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
struct Point2 {
  long double x, y;
};
long double Cross(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }
long double Length(Point2 p) { return std::hypot(p.x, p.y); }
long double Dot(Vector3 a, Vector3 b) {
  return static_cast<long double>(a.x) * b.x +
         static_cast<long double>(a.y) * b.y +
         static_cast<long double>(a.z) * b.z;
}
Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
long double Project(Vector3 p, Vector3 origin, Vector3 axis) {
  return (static_cast<long double>(p.x) - origin.x) * axis.x +
         (static_cast<long double>(p.y) - origin.y) * axis.y +
         (static_cast<long double>(p.z) - origin.z) * axis.z;
}
struct Frame {
  Vector3 origin, x, y;
  Point2 Point(Vector3 p) const {
    return {Project(p, origin, x), Project(p, origin, y)};
  }
  Point2 Direction(Vector3 p) const { return {Dot(p, x), Dot(p, y)}; }
};
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
}  // namespace
Result<AnnularBoundary> ValidateAnnularSector(
    std::span<const PlanarCurve> curves, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<AnnularBoundary>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Annular sector tolerance must be positive and finite"});
  if (curves.size() != 4)
    return Return::Err(Unsupported(
        "Annular sectors require four alternating arc/radial edges"));
  constexpr uint64_t kSectorChecks = 5;
  if (remaining_checks < kSectorChecks)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Annular sector comparison limit exceeded"});
  remaining_checks -= kSectorChecks;
  std::array<Vector3, 4> first{}, last{};
  std::array<const CircleGeometry*, 2> circles{};
  std::array<std::size_t, 2> indices{};
  std::array<long double, 2> sweeps{};
  std::size_t count = 0;
  const auto& n = plane.normal;
  const double sign = natural_sense ? 1 : -1;
  const auto py = Cross(n, plane.x_axis);
  Frame frame{{}, plane.x_axis, {sign * py.x, sign * py.y, sign * py.z}};
  long double area2 = 0, perimeter = 0;
  constexpr long double kAlignmentTolerance = 1e-10L;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    const auto& curve = curves[i];
    if (!curve.geometry || !std::isfinite(curve.start) ||
        !std::isfinite(curve.end) || curve.start == curve.end)
      return Return::Err(Invalid("Annular sector interval is degenerate"));
    const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
    if (!circle && !std::holds_alternative<LineGeometry>(*curve.geometry))
      return Return::Err(
          Unsupported("Annular sectors require LINE/CIRCLE bases"));
    const auto a = EvaluateCurve(*curve.geometry, curve.start),
               b = EvaluateCurve(*curve.geometry, curve.end);
    if (!a.ok()) return Return::Err(a.error());
    if (!b.ok()) return Return::Err(b.error());
    first[i] = a.value().position;
    last[i] = b.value().position;
    if (i == 0) frame.origin = first[i];
    for (const auto p : {first[i], last[i]}) {
      const auto residual = Project(p, plane.origin, n);
      if (!std::isfinite(residual) || std::abs(residual) > tolerance)
        return Return::Err(
            Invalid("Annular sector endpoint is outside the plane"));
    }
    const auto p = frame.Point(first[i]), q = frame.Point(last[i]);
    if (!circle) {
      const auto length = Length({q.x - p.x, q.y - p.y});
      if (!std::isfinite(length) || length <= tolerance)
        return Return::Err(Invalid("Annular sector radial edge is degenerate"));
      area2 += Cross(p, q);
      perimeter += length;
      continue;
    }
    if (count == 2)
      return Return::Err(
          Unsupported("Annular sectors require exactly two circular arcs"));
    const auto y = Cross(circle->normal, circle->x_axis);
    const auto alignment = Dot(circle->normal, n);
    const auto residual =
        std::abs(Project(circle->center, plane.origin, n)) +
        circle->radius * std::hypot(Dot(circle->x_axis, n), Dot(y, n));
    if (!std::isfinite(circle->radius) || circle->radius <= tolerance ||
        !std::isfinite(residual) || residual > tolerance ||
        !std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kAlignmentTolerance)
      return Return::Err(
          Invalid("Annular sector supporting circle is outside the plane"));
    const auto span = static_cast<long double>(curve.end) - curve.start;
    if (!std::isfinite(span) || std::abs(span) >= 2 * std::numbers::pi ||
        std::abs(span) * circle->radius <= tolerance)
      return Return::Err(
          Unsupported("Annular sector arcs must be resolved and shorter than "
                      "one full turn"));
    const auto center = frame.Point(circle->center);
    const auto determinant =
        Cross(frame.Direction(circle->x_axis), frame.Direction(y));
    area2 += Cross(center, {q.x - p.x, q.y - p.y}) +
             static_cast<long double>(circle->radius) * circle->radius *
                 determinant * span;
    perimeter += circle->radius * std::abs(span);
    circles[count] = circle;
    indices[count] = i;
    sweeps[count] = (alignment > 0 ? sign : -sign) * span;
    ++count;
  }
  if (count != 2 || (indices[0] + 2) % 4 != indices[1])
    return Return::Err(
        Unsupported("Annular sector arcs and radial edges must alternate"));
  const std::size_t outer = circles[0]->radius > circles[1]->radius ? 0 : 1;
  const auto& center = circles[outer]->center;
  const auto& other = circles[1 - outer]->center;
  const auto offset = std::hypot(static_cast<long double>(center.x) - other.x,
                                 static_cast<long double>(center.y) - other.y,
                                 static_cast<long double>(center.z) - other.z);
  if (!std::isfinite(offset) || offset > tolerance)
    return Return::Err(Unsupported("Annular sector arcs must be concentric"));
  const long double radius = circles[outer]->radius,
                    inner = circles[1 - outer]->radius;
  if (radius - inner <= offset + 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Invalid("Annular sector width is unresolved at the export tolerance"));
  if (sweeps[outer] <= 0 || sweeps[1 - outer] >= 0 ||
      std::abs(sweeps[0] + sweeps[1]) * radius > tolerance)
    return Return::Err(Invalid("Annular sector arc winding or spans disagree"));
  const Frame radial{center, frame.x, frame.y};
  for (const auto index : indices) {
    const auto p = radial.Point(first[index]), q = radial.Point(last[index]);
    const auto separation = Length({p.x - q.x, p.y - q.y});
    if (!std::isfinite(separation) ||
        separation <= 2 * static_cast<long double>(tolerance))
      return Return::Err(Invalid(
          "Annular sector radial boundaries coincide or are too close"));
  }
  for (std::size_t i = 0; i < curves.size(); ++i) {
    if (std::holds_alternative<CircleGeometry>(*curves[i].geometry)) continue;
    auto a = radial.Point(first[i]), b = radial.Point(last[i]);
    if (Length(a) < Length(b)) std::swap(a, b);
    const auto ra = Length(a), rb = Length(b);
    if (!std::isfinite(ra) || !std::isfinite(rb) || ra <= tolerance ||
        rb <= tolerance)
      return Return::Err(
          Invalid("Annular sector radial endpoint is unresolved"));
    const auto drift = std::abs(Cross(a, b)) / ra;
    if (!std::isfinite(drift) || std::abs(ra - radius) > tolerance ||
        std::abs(rb - inner) > tolerance || drift > tolerance ||
        a.x * b.x + a.y * b.y <= rb * rb)
      return Return::Err(
          Unsupported("Annular sector straight edges must be radial"));
  }
  if (!std::isfinite(area2) || !std::isfinite(perimeter) ||
      area2 <= tolerance * perimeter)
    return Return::Err(
        Invalid("Annular sector area is degenerate or incorrectly wound"));
  return Return::Ok({area2 / 2, first.front()});
}
}  // namespace openswstep::internal
