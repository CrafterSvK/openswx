// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using Vector = std::array<long double, 3>;
Vector Components(Vector3 v) { return {v.x, v.y, v.z}; }
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector Cross(const Vector& a, const Vector& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
bool Representable(const Vector& value) {
  return std::all_of(value.begin(), value.end(), [](long double v) {
    return std::isfinite(v) &&
           std::abs(v) <= std::numeric_limits<double>::max();
  });
}
// Call only after checking all components before narrowing to double.
Vector3 Narrow(const Vector& v) {
  return {static_cast<double>(v[0]), static_cast<double>(v[1]),
          static_cast<double>(v[2])};
}
Result<SurfaceEvaluation> Invalid(const char* message) {
  return Result<SurfaceEvaluation>::Err(
      {ErrorCode::kInvalidRecord, 0, message});
}
}  // namespace
Result<SurfaceEvaluation> EvaluateSpunProfile(
    const SpunSurfaceGeometry& surface, const CurveEvaluation& profile,
    double v) {
  constexpr double kAxisTolerance = 1e-10;
  if (!std::isfinite(surface.axis.x) || !std::isfinite(surface.axis.y) ||
      !std::isfinite(surface.axis.z) ||
      std::abs(std::hypot(surface.axis.x, surface.axis.y, surface.axis.z) - 1) >
          kAxisTolerance) {
    return Invalid("Spun surface axis must be finite and unit length");
  }
  const auto base = Components(surface.base);
  const auto axis = Components(surface.axis);
  const auto point = Components(profile.position);
  const auto tangent = Components(profile.first_derivative);
  Vector relative{};
  for (std::size_t i = 0; i < relative.size(); ++i)
    relative[i] = point[i] - base[i];
  const long double axial_position = Dot(relative, axis);
  const long double axial_tangent = Dot(tangent, axis);
  Vector radial{}, radial_tangent{}, projection{}, projection_tangent{};
  for (std::size_t i = 0; i < radial.size(); ++i) {
    projection[i] = base[i] + axial_position * axis[i];
    projection_tangent[i] = axial_tangent * axis[i];
    radial[i] = relative[i] - axial_position * axis[i];
    radial_tangent[i] = tangent[i] - projection_tangent[i];
  }
  const auto perpendicular = Cross(axis, radial);
  const auto perpendicular_tangent = Cross(axis, radial_tangent);
  const long double cosine = std::cos(v);
  const long double sine = std::sin(v);
  Vector position{}, du{}, dv{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    position[i] = projection[i] + radial[i] * cosine + perpendicular[i] * sine;
    du[i] = projection_tangent[i] + radial_tangent[i] * cosine +
            perpendicular_tangent[i] * sine;
    dv[i] = -radial[i] * sine + perpendicular[i] * cosine;
  }
  if (!Representable(position) || !Representable(du) || !Representable(dv)) {
    return Invalid(
        "Spun surface evaluation produced a nonfinite or out-of-range result");
  }
  return Result<SurfaceEvaluation>::Ok(
      {Narrow(position), Narrow(du), Narrow(dv)});
}
}  // namespace openparasolid::internal
