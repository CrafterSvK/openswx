// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "sphere_boundary.h"

#include <array>
#include <cmath>
#include <numbers>

namespace openswstep::internal {
namespace {
using namespace openparasolid;
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
Result<SphereBand> ValidateSphereLoops(std::span<const ConicBoundary> loops,
                                       const SphereGeometry& sphere,
                                       bool natural_sense, double tolerance,
                                       uint64_t& remaining_checks) {
  using Return = Result<SphereBand>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Sphere tolerance must be positive and finite"});
  if (!std::isfinite(sphere.radius) || sphere.radius <= tolerance)
    return Return::Err(Invalid("Sphere radius is degenerate"));
  if (loops.empty() || loops.size() > 2)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Sphere requires one or two full circular cuts"});
  constexpr double kFullTurn = 2 * std::numbers::pi;
  constexpr double kTurnTolerance = 1e-12;
  constexpr long double kFrameTolerance = 1e-10L;
  std::array<long double, 2> levels{};
  std::array<bool, 2> forward{};
  for (std::size_t i = 0; i < loops.size(); ++i) {
    const auto& loop = loops[i];
    if (!loop.circle)
      return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Sphere cuts require CIRCLE bases"});
    const auto span = std::abs(loop.end - loop.start);
    if (!std::isfinite(span) || span > kFullTurn ||
        kFullTurn - span > kTurnTolerance)
      return Return::Err(Invalid("Sphere boundary requires one complete turn"));
    const auto alignment = Dot(loop.normal, sphere.axis);
    const auto variation =
        loop.major_radius *
        std::hypot(Dot(loop.x_axis, sphere.axis),
                   Dot(Cross(loop.normal, loop.x_axis), sphere.axis));
    if (!std::isfinite(loop.major_radius) || loop.major_radius <= tolerance ||
        !std::isfinite(alignment) ||
        std::abs(alignment) < 1 - kFrameTolerance || !std::isfinite(variation))
      return Return::Err(Invalid("Sphere cut circle is tilted or degenerate"));
    const auto level = Project(loop.center, sphere.center, sphere.axis);
    const auto dx = static_cast<long double>(loop.center.x) - sphere.center.x;
    const auto dy = static_cast<long double>(loop.center.y) - sphere.center.y;
    const auto dz = static_cast<long double>(loop.center.z) - sphere.center.z;
    const auto radial_offset =
        std::hypot(dx - level * sphere.axis.x, dy - level * sphere.axis.y,
                   dz - level * sphere.axis.z);
    // For the permitted small tilt, twice the axial variation bounds the
    // displacement from a coaxial latitude circle over the whole source cut.
    const auto error =
        radial_offset + 2 * variation +
        std::abs(std::hypot(loop.major_radius, level) - sphere.radius);
    if (!std::isfinite(error) || error > tolerance)
      return Return::Err(
          Invalid("Sphere cut circle is outside the coaxial profile"));
    if (!std::isfinite(level) || sphere.radius - std::abs(level) <= tolerance)
      return Return::Err(
          {ErrorCode::kUnsupportedFormat, 0,
           "Sphere cuts at or too close to a pole are unsupported"});
    levels[i] = level;
    forward[i] = (loop.end > loop.start) == (alignment > 0);
  }
  if (!remaining_checks)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Sphere cut comparison limit exceeded"});
  --remaining_checks;
  if (loops.size() == 1) {
    // A positive lower boundary selects the cap above the cut; a negative
    // upper boundary selects the cap below it, relative to the face sense.
    // The pole is interior to the source face, not an invented boundary loop.
    const long double radius = sphere.radius;
    return Return::Ok(forward[0] == natural_sense
                          ? SphereBand{levels[0], radius}
                          : SphereBand{-radius, levels[0]});
  }
  const std::size_t lower = levels[0] < levels[1] ? 0 : 1;
  const auto height = levels[1 - lower] - levels[lower];
  if (!std::isfinite(height) ||
      height <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Invalid("Sphere cut levels coincide or are tolerance-ambiguous"));
  if (forward[lower] != natural_sense || forward[1 - lower] == natural_sense)
    return Return::Err(
        Invalid("Sphere cut winding disagrees with the face normal"));
  return Return::Ok({levels[lower], levels[1 - lower]});
}
long double SphereBandVolume6(const SphereGeometry& sphere, SphereBand band,
                              Vector3 origin) {
  const long double radius = sphere.radius;
  const auto distance = Project(sphere.center, origin, sphere.axis);
  // Integrate (P-origin).n dA over the complete azimuth and latitude band:
  // 2*pi*R^2*(z1-z0) + pi*d*(z1^2-z0^2). Twice this flux is six times volume.
  return 2 * std::numbers::pi_v<long double> * (band.upper - band.lower) *
         (2 * radius * radius + distance * (band.upper + band.lower));
}
}  // namespace openswstep::internal
