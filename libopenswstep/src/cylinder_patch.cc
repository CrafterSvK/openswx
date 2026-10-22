// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "cylinder_patch.h"

#include <array>
#include <cmath>
#include <numbers>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
long double Dot(Vector3 a, Vector3 b) {
  return static_cast<long double>(a.x) * b.x +
         static_cast<long double>(a.y) * b.y +
         static_cast<long double>(a.z) * b.z;
}
long double Project(Vector3 point, Vector3 origin, Vector3 axis) {
  return (static_cast<long double>(point.x) - origin.x) * axis.x +
         (static_cast<long double>(point.y) - origin.y) * axis.y +
         (static_cast<long double>(point.z) - origin.z) * axis.z;
}
Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
long double RadialDistance(Vector3 point, Vector3 origin, Vector3 axis) {
  const auto level = Project(point, origin, axis);
  return std::hypot(
      static_cast<long double>(point.x) - origin.x - level * axis.x,
      static_cast<long double>(point.y) - origin.y - level * axis.y,
      static_cast<long double>(point.z) - origin.z - level * axis.z);
}
Vector3 RadialDirection(Vector3 point, Vector3 center, Vector3 axis) {
  const auto level = Project(point, center, axis);
  const auto x = static_cast<long double>(point.x) - center.x - level * axis.x;
  const auto y = static_cast<long double>(point.y) - center.y - level * axis.y;
  const auto z = static_cast<long double>(point.z) - center.z - level * axis.z;
  const auto length = std::hypot(x, y, z);
  return {static_cast<double>(x / length), static_cast<double>(y / length),
          static_cast<double>(z / length)};
}
}  // namespace
Result<CylinderPatch> ValidateCylinderPatch(std::span<const PlanarCurve> curves,
                                            const CylinderGeometry& cylinder,
                                            bool natural_sense,
                                            double tolerance,
                                            uint64_t& remaining_checks) {
  using Return = Result<CylinderPatch>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Cylinder patch tolerance must be positive and finite"});
  if (!std::isfinite(cylinder.radius) || cylinder.radius <= tolerance)
    return Return::Err(Invalid("Cylinder patch radius is degenerate"));
  if (curves.size() != 4)
    return Return::Err(Unsupported(
        "Cylinder patch requires four alternating arc/generator edges"));
  constexpr uint64_t kPatchChecks = 5;
  if (remaining_checks < kPatchChecks)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Cylinder patch comparison limit exceeded"});
  remaining_checks -= kPatchChecks;
  std::array<Vector3, 4> first{}, last{};
  std::array<const CircleGeometry*, 2> circles{};
  std::array<std::size_t, 2> circle_indices{};
  std::array<long double, 2> levels{}, sweeps{};
  std::size_t circle_count = 0;
  constexpr long double kFrameTolerance = 1e-10L;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    const auto& curve = curves[i];
    if (!curve.geometry || !std::isfinite(curve.start) ||
        !std::isfinite(curve.end) || curve.start == curve.end)
      return Return::Err(Invalid("Cylinder patch interval is degenerate"));
    const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
    if (!circle && !std::holds_alternative<LineGeometry>(*curve.geometry))
      return Return::Err(
          Unsupported("Cylinder patch requires LINE/CIRCLE bases"));
    const auto a = EvaluateCurve(*curve.geometry, curve.start);
    const auto b = EvaluateCurve(*curve.geometry, curve.end);
    if (!a.ok()) return Return::Err(a.error());
    if (!b.ok()) return Return::Err(b.error());
    first[i] = a.value().position;
    last[i] = b.value().position;
    if (!circle) continue;
    if (circle_count == 2)
      return Return::Err(
          Unsupported("Cylinder patch requires exactly two circular arcs"));
    const auto alignment = Dot(circle->normal, cylinder.axis);
    const auto variation =
        circle->radius *
        std::hypot(Dot(circle->x_axis, cylinder.axis),
                   Dot(Cross(circle->normal, circle->x_axis), cylinder.axis));
    const auto error =
        RadialDistance(circle->center, cylinder.origin, cylinder.axis) +
        std::abs(static_cast<long double>(circle->radius) - cylinder.radius) +
        2 * variation;
    if (!std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kFrameTolerance || !std::isfinite(error) ||
        error > tolerance)
      return Return::Err(Invalid(
          "Cylinder patch supporting circle is outside the coaxial profile"));
    const auto span = static_cast<long double>(curve.end) - curve.start;
    if (!std::isfinite(span) || std::abs(span) >= 2 * std::numbers::pi ||
        std::abs(span) * circle->radius <= tolerance)
      return Return::Err(
          Unsupported("Cylinder patch arc must be resolved and shorter than "
                      "one full turn"));
    circles[circle_count] = circle;
    circle_indices[circle_count] = i;
    levels[circle_count] =
        Project(circle->center, cylinder.origin, cylinder.axis);
    sweeps[circle_count] = alignment > 0 ? span : -span;
    ++circle_count;
  }
  if (circle_count != 2 || (circle_indices[0] + 2) % 4 != circle_indices[1])
    return Return::Err(Unsupported(
        "Cylinder patch requires alternating circular arcs and generators"));
  const std::size_t lower = levels[0] < levels[1] ? 0 : 1;
  const auto height = levels[1 - lower] - levels[lower];
  if (!std::isfinite(height) ||
      height <= 2 * static_cast<long double>(tolerance))
    return Return::Err(Invalid(
        "Cylinder patch cut levels coincide or are tolerance-ambiguous"));
  if ((sweeps[lower] > 0) != natural_sense ||
      (sweeps[1 - lower] > 0) == natural_sense ||
      std::abs(sweeps[0] + sweeps[1]) * cylinder.radius > tolerance)
    return Return::Err(
        Invalid("Cylinder patch arc winding or azimuth spans disagree"));
  const auto index = circle_indices[lower];
  const auto u =
      RadialDirection(first[index], circles[lower]->center, cylinder.axis);
  const auto v =
      RadialDirection(last[index], circles[lower]->center, cylinder.axis);
  const auto separation = std::hypot(static_cast<long double>(u.x) - v.x,
                                     static_cast<long double>(u.y) - v.y,
                                     static_cast<long double>(u.z) - v.z) *
                          cylinder.radius;
  if (!std::isfinite(separation) ||
      separation <= 2 * static_cast<long double>(tolerance))
    return Return::Err(Invalid(
        "Cylinder patch generators coincide or are tolerance-ambiguous"));
  for (std::size_t i = 0; i < curves.size(); ++i) {
    if (std::holds_alternative<CircleGeometry>(*curves[i].geometry)) continue;
    const auto axial = std::abs(Project(last[i], first[i], cylinder.axis));
    const auto drift = RadialDistance(last[i], first[i], cylinder.axis);
    const auto residual =
        std::abs(RadialDistance(first[i], cylinder.origin, cylinder.axis) -
                 cylinder.radius);
    if (!std::isfinite(axial) || !std::isfinite(drift) ||
        !std::isfinite(residual) ||
        std::abs(axial - height) > 2 * static_cast<long double>(tolerance) ||
        drift + residual > tolerance)
      return Return::Err(
          Invalid("Cylinder patch straight edge is not an axial generator"));
  }
  return Return::Ok({height, std::abs(sweeps[lower]), sweeps[lower] > 0 ? u : v,
                     sweeps[lower] > 0 ? v : u, first[index]});
}
long double CylinderPatchVolume6(const CylinderGeometry& cylinder,
                                 const CylinderPatch& patch, Vector3 origin) {
  const auto a = patch.first_direction, b = patch.last_direction;
  const auto integral = Cross(cylinder.axis, {a.x - b.x, a.y - b.y, a.z - b.z});
  const long double radius = cylinder.radius;
  return 2 * radius * patch.height *
         (radius * patch.span + Project(cylinder.origin, origin, integral));
}
}  // namespace openswstep::internal
