// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "torus_boundary.h"

#include <array>
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
}  // namespace
Result<ConicBoundary> CombineCircularCut(std::span<const ConicBoundary> arcs,
                                         double tolerance,
                                         uint64_t& remaining_checks) {
  using Return = Result<ConicBoundary>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Circular cut tolerance must be positive and finite"});
  if (arcs.empty()) return Return::Err(Invalid("Circular cut has no arcs"));
  const auto& first = arcs.front();
  long double winding = 0;
  int direction = 0;
  constexpr double kFullTurn = 2 * std::numbers::pi;
  for (const auto& arc : arcs) {
    if (!remaining_checks)
      return Return::Err(
          {ErrorCode::kResourceLimit, 0, "Circular cut arc limit exceeded"});
    --remaining_checks;
    if (!arc.circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Split torus cuts require CIRCLE arcs"});
    const auto span = static_cast<long double>(arc.end) - arc.start;
    const auto alignment = Dot(arc.normal, first.normal);
    const auto tilt =
        std::hypot(Dot(arc.x_axis, first.normal),
                   Dot(Cross(arc.normal, arc.x_axis), first.normal));
    const auto displacement =
        std::hypot(static_cast<long double>(arc.center.x) - first.center.x,
                   static_cast<long double>(arc.center.y) - first.center.y,
                   static_cast<long double>(arc.center.z) - first.center.z);
    const auto error = displacement +
                       std::abs(arc.major_radius - first.major_radius) +
                       2 * arc.major_radius * tilt;
    if (!std::isfinite(arc.major_radius) || arc.major_radius <= tolerance ||
        !std::isfinite(alignment) || std::abs(alignment) < 1 - 1e-10L ||
        !std::isfinite(error) || error > tolerance)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Split torus cut arcs need one supporting circle"});
    if (!std::isfinite(span) || std::abs(span) > kFullTurn ||
        std::abs(span) * arc.major_radius <= tolerance)
      return Return::Err(
          Invalid("Circular cut arc interval is degenerate or repeated"));
    const auto directed_span = alignment > 0 ? span : -span;
    const int sign = directed_span > 0 ? 1 : -1;
    if (direction && sign != direction)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Circular cut arc winding must be monotone"});
    direction = sign;
    winding += directed_span;
  }
  if (std::abs(std::abs(winding) - kFullTurn) > 1e-12L)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Circular cut arcs must span one complete turn"});
  auto result = first;
  result.start = 0;
  result.end = direction * kFullTurn;
  return Return::Ok(result);
}
Result<TorusBand> ValidateTorusLoops(std::span<const ConicBoundary> loops,
                                     const openparasolid::TorusGeometry& torus,
                                     bool natural_sense, double tolerance,
                                     uint64_t& remaining_checks) {
  using Return = Result<TorusBand>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Torus tolerance must be positive and finite"});
  if (!std::isfinite(torus.major_radius) ||
      !std::isfinite(torus.minor_radius) || torus.minor_radius <= tolerance ||
      static_cast<long double>(torus.major_radius) - torus.minor_radius <=
          tolerance)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Toroidal bands require a regular ring torus"});
  if (loops.size() != 2)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Toroidal band requires two full circular cuts"});
  constexpr double kFullTurn = 2 * std::numbers::pi;
  constexpr double kTurnTolerance = 1e-12;
  constexpr long double kFrameTolerance = 1e-10L;
  std::array<long double, 2> angles{}, heights{};
  std::optional<std::size_t> first;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    const auto& loop = loops[i];
    if (!loop.circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Torus cuts require CIRCLE bases"});
    const auto span = std::abs(loop.end - loop.start);
    if (!std::isfinite(span) || span > kFullTurn ||
        kFullTurn - span > kTurnTolerance)
      return Return::Err(Invalid("Torus boundary requires one complete turn"));
    const auto alignment = Dot(loop.normal, torus.axis);
    const auto variation =
        loop.major_radius *
        std::hypot(Dot(loop.x_axis, torus.axis),
                   Dot(Cross(loop.normal, loop.x_axis), torus.axis));
    if (!std::isfinite(loop.major_radius) || loop.major_radius <= tolerance ||
        !std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kFrameTolerance ||
        !std::isfinite(variation) || variation > tolerance)
      return Return::Err(Invalid("Torus cut circle is tilted or degenerate"));
    const auto height = Project(loop.center, torus.center, torus.axis);
    const auto dx = static_cast<long double>(loop.center.x) - torus.center.x;
    const auto dy = static_cast<long double>(loop.center.y) - torus.center.y;
    const auto dz = static_cast<long double>(loop.center.z) - torus.center.z;
    const auto radial_offset =
        std::hypot(dx - height * torus.axis.x, dy - height * torus.axis.y,
                   dz - height * torus.axis.z);
    const auto radial =
        static_cast<long double>(loop.major_radius) - torus.major_radius;
    const auto profile_radius = std::hypot(radial, height);
    // Coaxial displacement and tilt bound the entire circle, beyond the
    // endpoint/midpoint surface-membership samples used by body binding.
    const auto error = radial_offset + variation +
                       std::abs(profile_radius - torus.minor_radius);
    if (!std::isfinite(error) || error > tolerance)
      return Return::Err(
          Invalid("Torus cut circle is outside the coaxial meridian profile"));
    heights[i] = height;
    angles[i] = std::atan2(height, radial);
    const bool positive = (loop.end > loop.start) == (alignment > 0);
    if (positive == natural_sense) {
      if (first)
        return Return::Err(
            Invalid("Torus cut loops have inconsistent winding"));
      first = i;
    }
  }
  if (!first)
    return Return::Err(Invalid("Torus cut loops have inconsistent winding"));
  if (!remaining_checks)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Torus cut comparison limit exceeded"});
  --remaining_checks;
  const auto separation = std::hypot(
      static_cast<long double>(loops[1].major_radius) - loops[0].major_radius,
      heights[1] - heights[0]);
  if (!std::isfinite(separation) ||
      separation <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Invalid("Torus cut circles coincide or are tolerance-ambiguous"));
  auto span = angles[1 - *first] - angles[*first];
  if (span <= 0) span += 2 * std::numbers::pi_v<long double>;
  return Return::Ok({angles[*first], span});
}
long double TorusBandVolume6(const openparasolid::TorusGeometry& torus,
                             TorusBand band, Vector3 origin) {
  const long double major = torus.major_radius, minor = torus.minor_radius;
  const auto distance = Project(torus.center, origin, torus.axis);
  const auto end = band.start + band.span;
  const auto s0 = std::sin(band.start), s1 = std::sin(end);
  const auto c0 = std::cos(band.start), c1 = std::cos(end);
  // Integrate (P-origin).n dA around the full azimuth and the selected
  // meridian interval; transverse translation terms cancel around the axis.
  const auto integral =
      (major * major + minor * minor) * (s1 - s0) +
      major * minor * (1.5L * band.span + (s1 * c1 - s0 * c0) / 2) -
      distance * major * (c1 - c0) + distance * minor * (s1 * s1 - s0 * s0) / 2;
  return 4 * std::numbers::pi_v<long double> * minor * integral;
}
}  // namespace openswstep::internal
