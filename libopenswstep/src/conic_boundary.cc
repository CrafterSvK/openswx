// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "conic_boundary.h"

#include <cmath>
#include <numbers>
#include <optional>

namespace openswstep::internal {
namespace {
using openparasolid::ErrorCode;
using openparasolid::ParseError;
using openparasolid::Result;
using openparasolid::Vector3;
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
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
std::optional<ParseError> Charge(uint64_t& remaining) {
  if (!remaining)
    return ParseError{ErrorCode::kResourceLimit, 0,
                      "Conic boundary comparison limit exceeded"};
  --remaining;
  return std::nullopt;
}
long double CenterDistance(const ConicBoundary& a, const ConicBoundary& b) {
  return std::hypot(static_cast<long double>(a.center.x) - b.center.x,
                    static_cast<long double>(a.center.y) - b.center.y,
                    static_cast<long double>(a.center.z) - b.center.z);
}
}  // namespace
Result<std::size_t> ValidatePlanarConics(
    std::span<const ConicBoundary> loops,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks) {
  using Return = Result<std::size_t>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Conic boundary tolerance must be positive and finite"});
  if (loops.empty()) return Return::Err(Invalid("Conic face has no loops"));
  std::optional<std::size_t> outer;
  constexpr double kFullTurn = 2 * std::numbers::pi;
  constexpr double kTurnTolerance = 1e-12;
  constexpr long double kNormalAlignmentTolerance = 1e-10L;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    const auto& loop = loops[i];
    if (loops.size() > 1 && !loop.circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Conic face holes currently require CIRCLE bases"});
    const auto span = std::abs(loop.end - loop.start);
    if (!std::isfinite(span) || span > kFullTurn ||
        kFullTurn - span > kTurnTolerance)
      return Return::Err(
          Invalid("Conic face loop requires exactly one full turn"));
    if (!std::isfinite(loop.major_radius) ||
        !std::isfinite(loop.minor_radius) ||
        loop.major_radius < loop.minor_radius || loop.minor_radius <= tolerance)
      return Return::Err(
          Invalid("Conic face radii are degenerate or unordered"));
    // The maximum plane residual over the entire curve is |c|+hypot(a,b).
    // Sampling endpoints or quarter turns alone cannot establish coplanarity.
    const auto residual =
        std::abs(Project(loop.center, plane.origin, plane.normal)) +
        std::hypot(loop.major_radius * Dot(loop.x_axis, plane.normal),
                   loop.minor_radius *
                       Dot(Cross(loop.normal, loop.x_axis), plane.normal));
    const auto alignment = Dot(loop.normal, plane.normal);
    if (!std::isfinite(residual) || residual > tolerance ||
        !std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kNormalAlignmentTolerance)
      return Return::Err(
          Invalid("Complete conic boundary is outside the face plane"));
    const bool forward_normal = (loop.end > loop.start) == (alignment > 0);
    if (forward_normal == natural_sense) {
      if (outer)
        return Return::Err(
            Invalid("Conic face has multiple outward-wound loops"));
      outer = i;
    }
  }
  if (!outer)
    return Return::Err(Invalid("Conic face has no correctly wound outer loop"));
  // Reserve two tolerances for the in-plane displacement of near-coplanar
  // circles, in addition to a strictly positive separation at source precision.
  const long double clearance = 2 * static_cast<long double>(tolerance);
  for (std::size_t i = 0; i < loops.size(); ++i) {
    if (i == *outer) continue;
    if (const auto error = Charge(remaining_checks)) return Return::Err(*error);
    const auto distance = CenterDistance(loops[i], loops[*outer]);
    if (!std::isfinite(distance) ||
        distance + loops[i].major_radius + clearance >=
            loops[*outer].major_radius)
      return Return::Err(
          Invalid("Circular hole is outside, touching or too close to the "
                  "outer boundary"));
    for (std::size_t j = i + 1; j < loops.size(); ++j) {
      if (j == *outer) continue;
      if (const auto error = Charge(remaining_checks))
        return Return::Err(*error);
      const auto separation = CenterDistance(loops[i], loops[j]);
      if (!std::isfinite(separation) ||
          separation <=
              loops[i].major_radius + loops[j].major_radius + clearance)
        return Return::Err(
            Invalid("Circular holes overlap, nest, touch or are too close"));
    }
  }
  return Return::Ok(*outer);
}
namespace {
Result<CircularBand> ValidateCircularBand(std::span<const ConicBoundary> loops,
                                          Vector3 origin, Vector3 axis,
                                          double reference_radius,
                                          long double slope, bool natural_sense,
                                          double tolerance,
                                          uint64_t& remaining_checks) {
  using Return = Result<CircularBand>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Circular band tolerance must be positive and finite"});
  if (loops.size() != 2)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Circular band requires two complete circular loops"});
  bool forward[2];
  long double radii[2];
  constexpr double kFullTurn = 2 * std::numbers::pi;
  constexpr double kTurnTolerance = 1e-12;
  constexpr long double kNormalAlignmentTolerance = 1e-10L;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    const auto& loop = loops[i];
    if (!loop.circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Circular band boundaries require CIRCLE bases"});
    const auto span = std::abs(loop.end - loop.start);
    if (!std::isfinite(span) || span > kFullTurn ||
        kFullTurn - span > kTurnTolerance)
      return Return::Err(
          Invalid("Circular band boundary requires one full turn"));
    const auto alignment = Dot(loop.normal, axis);
    const auto axial_variation =
        loop.major_radius *
        std::hypot(Dot(loop.x_axis, axis),
                   Dot(Cross(loop.normal, loop.x_axis), axis));
    if (!std::isfinite(loop.major_radius) || loop.major_radius <= tolerance ||
        !std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kNormalAlignmentTolerance ||
        !std::isfinite(axial_variation) || axial_variation > tolerance)
      return Return::Err(
          Invalid("Circular band cut circle is tilted or degenerate"));
    const auto level = Project(loop.center, origin, axis);
    const auto dx = static_cast<long double>(loop.center.x) - origin.x;
    const auto dy = static_cast<long double>(loop.center.y) - origin.y;
    const auto dz = static_cast<long double>(loop.center.z) - origin.z;
    const auto radial_offset = std::hypot(
        dx - level * axis.x, dy - level * axis.y, dz - level * axis.z);
    const auto expected_radius =
        std::fma(level, slope, static_cast<long double>(reference_radius));
    // Bound the whole cut's profile error, including axial variation on a
    // sloping cone; the binding samples alone cannot establish this bound.
    const auto profile_error =
        std::abs(static_cast<long double>(loop.major_radius) -
                 expected_radius) +
        std::abs(slope) * axial_variation;
    if (!std::isfinite(level) || !std::isfinite(radial_offset) ||
        !std::isfinite(expected_radius) || expected_radius <= tolerance ||
        !std::isfinite(profile_error) ||
        radial_offset + profile_error > tolerance)
      return Return::Err(Invalid(
          "Circular band cut circle is not coaxial at the source radius"));
    radii[i] = expected_radius;
    forward[i] = (loop.end > loop.start) == (alignment > 0);
  }
  if (const auto error = Charge(remaining_checks)) return Return::Err(*error);
  const auto delta = Project(loops[1].center, loops[0].center, axis);
  const auto height = std::abs(delta);
  if (!std::isfinite(height) ||
      height <= 2 * static_cast<long double>(tolerance))
    return Return::Err(Invalid(
        "Circular band cut levels coincide or are tolerance-ambiguous"));
  const std::size_t lower = delta > 0 ? 0 : 1;
  if (forward[lower] != natural_sense || forward[1 - lower] == natural_sense)
    return Return::Err(Invalid(
        "Circular band boundary winding disagrees with the face normal"));
  return Return::Ok(
      {height, loops[lower].center, radii[lower], radii[1 - lower]});
}
}  // namespace
Result<long double> ValidateCylinderLoops(
    std::span<const ConicBoundary> loops,
    const openparasolid::CylinderGeometry& cylinder, bool natural_sense,
    double tolerance, uint64_t& remaining_checks) {
  if (!std::isfinite(cylinder.radius) || cylinder.radius <= tolerance)
    return Result<long double>::Err(Invalid("Cylinder radius is degenerate"));
  const auto band = ValidateCircularBand(loops, cylinder.origin, cylinder.axis,
                                         cylinder.radius, 0, natural_sense,
                                         tolerance, remaining_checks);
  if (!band.ok()) return Result<long double>::Err(band.error());
  return Result<long double>::Ok(band.value().height);
}
Result<CircularBand> ValidateConeLoops(std::span<const ConicBoundary> loops,
                                       const openparasolid::ConeGeometry& cone,
                                       bool natural_sense, double tolerance,
                                       uint64_t& remaining_checks) {
  constexpr double kFrameTolerance = 1e-10;
  if (!std::isfinite(cone.radius) || cone.radius < 0 ||
      !std::isfinite(cone.sin_half_angle) || cone.sin_half_angle == 0 ||
      !std::isfinite(cone.cos_half_angle) || cone.cos_half_angle <= 0 ||
      std::abs(std::hypot(cone.sin_half_angle, cone.cos_half_angle) - 1) >
          kFrameTolerance)
    return Result<CircularBand>::Err(
        Invalid("Cone requires a nonnegative reference radius and nonzero "
                "signed acute half-angle"));
  const auto slope =
      static_cast<long double>(cone.sin_half_angle) / cone.cos_half_angle;
  if (!std::isfinite(slope))
    return Result<CircularBand>::Err(Invalid("Cone slope is unrepresentable"));
  return ValidateCircularBand(loops, cone.origin, cone.axis, cone.radius, slope,
                              natural_sense, tolerance, remaining_checks);
}
}  // namespace openswstep::internal
