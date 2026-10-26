// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "torus_meridian.h"

#include <array>
#include <cmath>
#include <numbers>
#include <optional>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vector = std::array<long double, 3>;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
Vector Components(Vector3 p) { return {p.x, p.y, p.z}; }
Vector Difference(Vector3 a, Vector3 b) {
  return {static_cast<long double>(a.x) - b.x,
          static_cast<long double>(a.y) - b.y,
          static_cast<long double>(a.z) - b.z};
}
long double Dot(Vector a, Vector b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
long double Length(Vector a) { return std::hypot(a[0], a[1], a[2]); }
Vector Cross(Vector a, Vector b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
Vector3 Narrow(Vector a) {
  return {static_cast<double>(a[0]), static_cast<double>(a[1]),
          static_cast<double>(a[2])};
}
long double FrameError(Vector3 a, Vector3 x) {
  return std::abs(Length(Components(a)) - 1) +
         std::abs(Length(Components(x)) - 1) +
         std::abs(Dot(Components(a), Components(x)));
}
}  // namespace
Result<TorusMeridianBand> ValidateTorusMeridians(
    std::span<const ConicBoundary> loops, const TorusGeometry& torus,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<TorusMeridianBand>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Torus meridian tolerance must be positive and finite"});
  if (loops.size() != 2 || !std::isfinite(torus.major_radius) ||
      !std::isfinite(torus.minor_radius) || torus.minor_radius <= tolerance ||
      static_cast<long double>(torus.major_radius) - torus.minor_radius <=
          tolerance)
    return Return::Err(Unsupported(
        "Torus meridian bands require two cuts on a regular ring support"));
  const auto valid = EvaluateSurface(GeometryPayload{torus}, 0, 0);
  if (!valid.ok()) return Return::Err(valid.error());
  // EvaluateSurface has certified a near-orthonormal frame. Four times its
  // residual bounds displacement from an orthonormal support over all angles.
  const auto frame_error =
      4 * (static_cast<long double>(torus.major_radius) + torus.minor_radius) *
      FrameError(torus.axis, torus.x_axis);
  auto axis = Components(torus.axis);
  const auto axis_length = Length(axis);
  for (auto& v : axis) v /= axis_length;
  std::array<Vector, 2> directions{};
  std::optional<std::size_t> first;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    const auto& loop = loops[i];
    if (!loop.circle)
      return Return::Err(
          Unsupported("Torus meridian cuts require CIRCLE bases"));
    const auto span = std::abs(loop.end - loop.start);
    constexpr double kTurn = 2 * std::numbers::pi;
    if (!std::isfinite(span) || span > kTurn || kTurn - span > 1e-12)
      return Return::Err(
          Unsupported("Torus meridian cut must span one complete turn"));
    const CircleGeometry circle{loop.center, loop.normal, loop.x_axis,
                                loop.major_radius};
    const auto curve = EvaluateCurve(GeometryPayload{circle}, 0);
    if (!curve.ok()) return Return::Err(curve.error());
    const auto relative = Difference(loop.center, torus.center);
    const auto height = Dot(relative, axis);
    auto radial = relative;
    for (std::size_t k = 0; k < 3; ++k) radial[k] -= height * axis[k];
    const auto radius = Length(radial);
    if (!std::isfinite(radius) || radius <= tolerance)
      return Return::Err(
          Unsupported("Torus meridian cut has no resolved radial direction"));
    for (auto& v : radial) v /= radius;
    directions[i] = radial;
    const auto normal = Cross(axis, radial);
    auto actual = Components(loop.normal);
    const auto n_length = Length(actual);
    for (auto& v : actual) v /= n_length;
    const auto alignment = Dot(actual, normal);
    Vector difference{};
    for (std::size_t k = 0; k < 3; ++k)
      difference[k] = actual[k] - (alignment > 0 ? 1 : -1) * normal[k];
    const auto error =
        frame_error + std::hypot(height, radius - torus.major_radius) +
        std::abs(static_cast<long double>(loop.major_radius) -
                 torus.minor_radius) +
        torus.minor_radius * (2 * Length(difference)) +
        loop.major_radius * (4 * FrameError(loop.normal, loop.x_axis));
    if (!std::isfinite(error) || error > tolerance)
      return Return::Err(Unsupported(
          "Torus meridian circle is outside the complete profile or plane"));
    const bool positive = (loop.end > loop.start) == (alignment > 0);
    if (positive == natural_sense) {
      if (first)
        return Return::Err(
            Unsupported("Torus meridian cuts have inconsistent winding"));
      first = i;
    }
  }
  if (!first)
    return Return::Err(
        Unsupported("Torus meridian cuts have inconsistent winding"));
  if (!remaining_checks)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Torus meridian comparison limit exceeded"});
  --remaining_checks;
  Vector delta{};
  for (std::size_t k = 0; k < 3; ++k)
    delta[k] = directions[1][k] - directions[0][k];
  const auto separation =
      (static_cast<long double>(torus.major_radius) - torus.minor_radius) *
      Length(delta);
  if (!std::isfinite(separation) ||
      separation <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Unsupported("Torus meridian cuts coincide or are tolerance-ambiguous"));
  const auto& a = directions[*first];
  const auto& b = directions[1 - *first];
  auto span = std::atan2(Dot(axis, Cross(a, b)), Dot(a, b));
  if (span <= 0) span += 2 * std::numbers::pi_v<long double>;
  return Return::Ok({span, Narrow(axis), Narrow(a), Narrow(b)});
}
long double TorusMeridianVolume6(const TorusGeometry& torus,
                                 const TorusMeridianBand& band,
                                 Vector3 origin) {
  const auto integral =
      Cross(Components(band.axis),
            Difference(band.first_direction, band.last_direction));
  // Integrating around the full minor circle leaves 3*R per azimuth unit;
  // the integrated transverse radial direction is axis x (first-last).
  return 2 * std::numbers::pi_v<long double> * torus.minor_radius *
         torus.minor_radius *
         (3 * torus.major_radius * band.span +
          Dot(Difference(torus.center, origin), integral));
}
}  // namespace openswstep::internal
