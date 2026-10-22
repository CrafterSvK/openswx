// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "cone_patch.h"

#include <algorithm>
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
Result<ConePatch> ValidateConePatch(std::span<const PlanarCurve> curves,
                                    const ConeGeometry& cone,
                                    bool natural_sense, double tolerance,
                                    uint64_t& remaining_checks) {
  using Return = Result<ConePatch>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Cone patch tolerance must be positive and finite"});
  constexpr long double kFrameTolerance = 1e-10L;
  if (!std::isfinite(cone.radius) || cone.radius < 0 ||
      !std::isfinite(cone.sin_half_angle) || cone.sin_half_angle == 0 ||
      !std::isfinite(cone.cos_half_angle) || cone.cos_half_angle <= 0 ||
      std::abs(std::hypot(cone.sin_half_angle, cone.cos_half_angle) - 1) >
          kFrameTolerance)
    return Return::Err(
        Invalid("Cone patch requires a nonnegative reference "
                "radius and nonzero signed acute half-angle"));
  const auto slope =
      static_cast<long double>(cone.sin_half_angle) / cone.cos_half_angle;
  if (!std::isfinite(slope))
    return Return::Err(Invalid("Cone patch slope is unrepresentable"));
  if (curves.size() != 4)
    return Return::Err(Unsupported(
        "Cone patch requires four alternating arc/generator edges"));
  constexpr uint64_t kPatchChecks = 5;
  if (remaining_checks < kPatchChecks)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Cone patch comparison limit exceeded"});
  remaining_checks -= kPatchChecks;
  std::array<Vector3, 4> first{}, last{};
  std::array<const CircleGeometry*, 2> circles{};
  std::array<std::size_t, 2> circle_indices{};
  std::array<long double, 2> levels{}, sweeps{}, radii{};
  std::size_t circle_count = 0;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    const auto& curve = curves[i];
    if (!curve.geometry || !std::isfinite(curve.start) ||
        !std::isfinite(curve.end) || curve.start == curve.end)
      return Return::Err(Invalid("Cone patch interval is degenerate"));
    const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
    if (!circle && !std::holds_alternative<LineGeometry>(*curve.geometry))
      return Return::Err(Unsupported("Cone patch requires LINE/CIRCLE bases"));
    const auto a = EvaluateCurve(*curve.geometry, curve.start);
    const auto b = EvaluateCurve(*curve.geometry, curve.end);
    if (!a.ok()) return Return::Err(a.error());
    if (!b.ok()) return Return::Err(b.error());
    first[i] = a.value().position;
    last[i] = b.value().position;
    if (!circle) continue;
    if (circle_count == 2)
      return Return::Err(
          Unsupported("Cone patch requires exactly two circular arcs"));
    const auto alignment = Dot(circle->normal, cone.axis);
    const auto variation =
        circle->radius *
        std::hypot(Dot(circle->x_axis, cone.axis),
                   Dot(Cross(circle->normal, circle->x_axis), cone.axis));
    const auto level = Project(circle->center, cone.origin, cone.axis);
    const auto radius = std::fma(slope, level, cone.radius);
    if (!std::isfinite(radius) || radius <= tolerance)
      return Return::Err(Invalid("Cone patch cut radius is degenerate"));
    const auto error =
        RadialDistance(circle->center, cone.origin, cone.axis) +
        std::abs(static_cast<long double>(circle->radius) - radius) +
        (2 + std::abs(slope)) * variation;
    if (!std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kFrameTolerance || !std::isfinite(error) ||
        error > tolerance)
      return Return::Err(Invalid(
          "Cone patch supporting circle is outside the coaxial profile"));
    const auto span = static_cast<long double>(curve.end) - curve.start;
    if (!std::isfinite(span) || std::abs(span) >= 2 * std::numbers::pi ||
        std::abs(span) * circle->radius <= tolerance)
      return Return::Err(
          Unsupported("Cone patch arc must be resolved and shorter than "
                      "one full turn"));
    circles[circle_count] = circle;
    circle_indices[circle_count] = i;
    levels[circle_count] = level;
    radii[circle_count] = radius;
    sweeps[circle_count] = alignment > 0 ? span : -span;
    ++circle_count;
  }
  if (circle_count != 2 || (circle_indices[0] + 2) % 4 != circle_indices[1])
    return Return::Err(Unsupported(
        "Cone patch requires alternating circular arcs and generators"));
  const std::size_t lower = levels[0] < levels[1] ? 0 : 1;
  const auto height = levels[1 - lower] - levels[lower];
  if (!std::isfinite(height) ||
      height <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Invalid("Cone patch cut levels coincide or are tolerance-ambiguous"));
  if ((sweeps[lower] > 0) != natural_sense ||
      (sweeps[1 - lower] > 0) == natural_sense ||
      std::abs(sweeps[0] + sweeps[1]) * std::max(radii[0], radii[1]) >
          tolerance)
    return Return::Err(
        Invalid("Cone patch arc winding or azimuth spans disagree"));
  const auto index = circle_indices[lower];
  const auto u =
      RadialDirection(first[index], circles[lower]->center, cone.axis);
  const auto v =
      RadialDirection(last[index], circles[lower]->center, cone.axis);
  const auto separation = std::hypot(static_cast<long double>(u.x) - v.x,
                                     static_cast<long double>(u.y) - v.y,
                                     static_cast<long double>(u.z) - v.z) *
                          std::min(radii[0], radii[1]);
  if (!std::isfinite(separation) ||
      separation <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Invalid("Cone patch generators coincide or are tolerance-ambiguous"));
  for (std::size_t i = 0; i < curves.size(); ++i) {
    if (std::holds_alternative<CircleGeometry>(*curves[i].geometry)) continue;
    // A straight segment belongs to this positive-radius branch only when
    // its transverse displacement follows the same radial ruling as r(z).
    const auto axial = Project(last[i], first[i], cone.axis);
    const auto direction = RadialDirection(first[i], cone.origin, cone.axis);
    const auto level = Project(first[i], cone.origin, cone.axis);
    const auto radius = std::fma(slope, level, cone.radius);
    const auto residual =
        std::abs(RadialDistance(first[i], cone.origin, cone.axis) - radius);
    const auto drift =
        std::hypot(static_cast<long double>(last[i].x) - first[i].x -
                       axial * (cone.axis.x + slope * direction.x),
                   static_cast<long double>(last[i].y) - first[i].y -
                       axial * (cone.axis.y + slope * direction.y),
                   static_cast<long double>(last[i].z) - first[i].z -
                       axial * (cone.axis.z + slope * direction.z));
    if (!std::isfinite(axial) || !std::isfinite(drift) ||
        !std::isfinite(residual) || radius <= tolerance ||
        std::abs(std::abs(axial) - height) >
            2 * static_cast<long double>(tolerance) ||
        drift + residual > tolerance)
      return Return::Err(
          Invalid("Cone patch straight edge is not a cone generator"));
  }
  return Return::Ok({height, std::abs(sweeps[lower]), radii[lower],
                     radii[1 - lower], sweeps[lower] > 0 ? u : v,
                     sweeps[lower] > 0 ? v : u, first[index]});
}
long double ConePatchVolume6(const ConeGeometry& cone, const ConePatch& patch,
                             Vector3 origin) {
  const auto a = patch.first_direction, b = patch.last_direction;
  const auto integral = Cross(cone.axis, {a.x - b.x, a.y - b.y, a.z - b.z});
  const auto slope =
      static_cast<long double>(cone.sin_half_angle) / cone.cos_half_angle;
  // X=O+z*a+r(z)*u, r(z)=R+k*z. The outward area vector is
  // r(z)*(u-k*a) dz dtheta, so (X-origin) dot dA contains both
  // transverse and axial origin terms. Integrating r(z) gives h*(r0+r1)/2.
  const auto axial_offset = Project(cone.origin, origin, cone.axis);
  return patch.height * (patch.lower_radius + patch.upper_radius) *
         (Project(cone.origin, origin, integral) +
          std::fma(-slope, axial_offset, cone.radius) * patch.span);
}
}  // namespace openswstep::internal
