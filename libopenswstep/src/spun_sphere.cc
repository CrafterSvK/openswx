// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "spun_sphere.h"

#include <array>
#include <cmath>
#include <numbers>
#include <numeric>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vec = std::array<long double, 3>;
Vec Components(Vector3 v) { return {v.x, v.y, v.z}; }
long double Dot(const Vec& a, const Vec& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec Cross(const Vec& a, const Vec& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Norm(const Vec& v) { return std::hypot(v[0], v[1], v[2]); }
Result<SpunSphere> Unsupported(const char* message) {
  return Result<SpunSphere>::Err({ErrorCode::kUnsupportedFormat, 0, message});
}
}  // namespace
Result<SpunSphere> RecognizeSpunSphere(const SpunSurfaceGeometry& spun,
                                       const CircleGeometry& circle,
                                       double tolerance) {
  using Return = Result<SpunSphere>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Spun sphere tolerance must be positive and finite"});
  const auto valid = EvaluateCurve(GeometryPayload{circle}, 0);
  if (!valid.ok()) return Return::Err(valid.error());
  constexpr long double kFrameTolerance = 1e-10L;
  const auto raw_axis = Components(spun.axis);
  const auto length = Norm(raw_axis);
  if (!std::isfinite(length) || std::abs(length - 1) > kFrameTolerance)
    return Return::Err({ErrorCode::kInvalidRecord, 0,
                        "Spun sphere axis must be finite and unit length"});
  auto axis = raw_axis;
  for (auto& v : axis) v /= length;
  Vec center{static_cast<long double>(circle.center.x) - spun.base.x,
             static_cast<long double>(circle.center.y) - spun.base.y,
             static_cast<long double>(circle.center.z) - spun.base.z};
  const auto height = Dot(center, axis);
  auto radial = center;
  for (std::size_t i = 0; i < 3; ++i) radial[i] -= height * axis[i];
  const auto offset = Norm(radial);
  if (!std::isfinite(offset) || offset > kFrameTolerance * circle.radius)
    return Unsupported("Spun sphere requires the circle center on its axis");
  if (!spun.start_parameter || !spun.end_parameter ||
      !std::isfinite(*spun.start_parameter) ||
      !std::isfinite(*spun.end_parameter))
    return Unsupported("Spun sphere requires a bounded pole-to-pole meridian");
  const long double first = *spun.start_parameter, last = *spun.end_parameter;
  const auto span_error =
      std::abs(last - first - std::numbers::pi_v<long double>);
  const auto x = Components(circle.x_axis);
  const auto y = Cross(Components(circle.normal), x);
  const auto a = Dot(x, axis), b = Dot(y, axis);
  const auto derivative = [a, b](long double u) {
    return -a * std::sin(u) + b * std::cos(u);
  };
  const auto alignment = std::abs(Dot(Components(circle.normal), axis));
  const auto pole_error =
      std::abs(derivative(first)) + std::abs(derivative(last));
  if (span_error > kFrameTolerance || alignment > kFrameTolerance ||
      pole_error > kFrameTolerance)
    return Unsupported("Spun sphere requires meridian endpoints at the poles");
  // Circle basis norm errors, axis residuals, center displacement and endpoint
  // angular errors bound the whole rotational support, not just sampled points.
  const auto frame_error = std::abs(Norm(x) - 1) + std::abs(Norm(y) - 1);
  const auto error = 2 * offset +
                     circle.radius * (frame_error + 4 * alignment +
                                      4 * pole_error + span_error) +
                     8 * std::abs(length - 1) * (Norm(center) + circle.radius);
  if (!std::isfinite(error) || error > tolerance)
    return Unsupported("Spun sphere cannot preserve the source tolerance");
  const auto narrow = [](Vec v) {
    return Vector3{static_cast<double>(v[0]), static_cast<double>(v[1]),
                   static_cast<double>(v[2])};
  };
  const auto middle = std::midpoint(first, last);
  Vec reference{};
  for (std::size_t i = 0; i < 3; ++i)
    reference[i] = x[i] * std::cos(middle) + y[i] * std::sin(middle);
  const auto axial = Dot(reference, axis);
  for (std::size_t i = 0; i < 3; ++i) reference[i] -= axial * axis[i];
  const auto radial_length = Norm(reference);
  if (!std::isfinite(radial_length) || radial_length == 0)
    return Unsupported("Spun sphere meridian has no radial direction");
  for (auto& v : reference) v /= radial_length;
  return Return::Ok({{.center = circle.center,
                      .radius = circle.radius,
                      .axis = narrow(axis),
                      .x_axis = narrow(reference)},
                     derivative(middle) < 0});
}
}  // namespace openswstep::internal
