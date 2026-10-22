// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "spun_torus.h"

#include <array>
#include <cmath>
#include <limits>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vec = std::array<long double, 3>;
Vec Components(Vector3 p) { return {p.x, p.y, p.z}; }
Vec Cross(Vec a, Vec b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Dot(Vec a, Vec b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
long double Norm(Vec v) { return std::hypot(v[0], v[1], v[2]); }
Result<SpunTorus> Unsupported(const char* message) {
  return Result<SpunTorus>::Err({ErrorCode::kUnsupportedFormat, 0, message});
}
}  // namespace
Result<SpunTorus> RecognizeSpunTorus(const SpunSurfaceGeometry& spun,
                                     const CircleGeometry& circle,
                                     double tolerance) {
  using Return = Result<SpunTorus>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Spun torus tolerance must be positive and finite"});
  if (spun.start_parameter || spun.end_parameter)
    return Unsupported("Bounded spun profiles need parameter-domain mapping");
  // Reuse the reader's complete circle-frame validation, not sampled
  // membership.
  const auto sample = EvaluateCurve(GeometryPayload{circle}, 0);
  if (!sample.ok()) return Return::Err(sample.error());
  const auto axis = Components(spun.axis), base = Components(spun.base),
             center = Components(circle.center),
             normal = Components(circle.normal);
  constexpr long double kFrameTolerance = 1e-10L;
  if (!std::isfinite(Norm(axis)) ||
      std::abs(Norm(axis) - 1) > kFrameTolerance || !std::isfinite(Norm(base)))
    return Return::Err({ErrorCode::kInvalidRecord, 0,
                        "Spun torus axis must be finite and unit length"});
  // Projection uses the recorded axis line; normalize its accepted roundoff so
  // the canonical STEP frame remains orthonormal.
  Vec unit = axis;
  for (auto& value : unit) value /= Norm(axis);
  Vec relative{}, radial{}, projected{};
  for (std::size_t i = 0; i < 3; ++i) relative[i] = center[i] - base[i];
  const auto height = Dot(relative, unit);
  for (std::size_t i = 0; i < 3; ++i) {
    projected[i] = base[i] + height * unit[i];
    radial[i] = relative[i] - height * unit[i];
  }
  const auto major = Norm(radial);
  if (!std::isfinite(major) || circle.radius <= tolerance ||
      major - circle.radius <= tolerance)
    return Unsupported("Spun circle does not generate a regular ring torus");
  for (auto& value : radial) value /= major;
  const auto expected_normal = Cross(unit, radial);
  const auto orientation = Dot(normal, expected_normal);
  Vec difference{};
  for (std::size_t i = 0; i < 3; ++i)
    difference[i] = normal[i] - (orientation > 0 ? 1 : -1) * expected_normal[i];
  const auto frame_error = Norm(difference);
  // This bounds the whole supporting circle's plane, rather than inferring
  // equivalence from its endpoints. Near-horn rings stay unsupported as above.
  const auto displacement = 2 * circle.radius * frame_error +
                            (major + circle.radius) * std::abs(Norm(axis) - 1);
  if (!std::isfinite(frame_error) || frame_error > kFrameTolerance ||
      !std::isfinite(displacement) || displacement > tolerance)
    return Unsupported("Spun circle is not in an axial meridian plane");
  for (auto value : projected)
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<double>::max())
      return Return::Err({ErrorCode::kInvalidRecord, 0,
                          "Spun torus center is unrepresentable"});
  if (major > std::numeric_limits<double>::max())
    return Return::Err(
        {ErrorCode::kInvalidRecord, 0, "Spun torus radius is unrepresentable"});
  const auto narrow = [](Vec v) {
    return Vector3{static_cast<double>(v[0]), static_cast<double>(v[1]),
                   static_cast<double>(v[2])};
  };
  const TorusGeometry torus{narrow(projected), narrow(unit),
                            static_cast<double>(major), circle.radius,
                            narrow(radial)};
  const auto rounding = [](Vec exact, Vector3 rounded) {
    const auto actual = Components(rounded);
    for (std::size_t i = 0; i < 3; ++i) exact[i] -= actual[i];
    return Norm(exact);
  };
  const auto output_error =
      rounding(projected, torus.center) + std::abs(major - torus.major_radius) +
      (major + circle.radius) *
          (rounding(unit, torus.axis) + rounding(radial, torus.x_axis));
  if (displacement + output_error > tolerance)
    return Unsupported("Spun torus frame cannot preserve the source tolerance");
  // At the outer meridian the rotational tangent is axis x radial. Taking
  // profile du x rotational dv yields an outward normal exactly when the
  // generator normal agrees with axis x radial, regardless of profile phase.
  return Return::Ok({torus, orientation > 0});
}
}  // namespace openswstep::internal
