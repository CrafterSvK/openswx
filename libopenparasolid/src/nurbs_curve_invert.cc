// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using Vector = std::array<long double, 3>;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
Vector Difference(Vector3 a, Vector3 b) {
  return {static_cast<long double>(a.x) - b.x,
          static_cast<long double>(a.y) - b.y,
          static_cast<long double>(a.z) - b.z};
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
long double Length(const Vector& a) { return std::hypot(a[0], a[1], a[2]); }
Vector3 Control(const NurbsCurveGeometry& curve, std::size_t i) {
  const auto& values = curve.controls.values;
  if (!curve.controls.rational)
    return {values[3 * i], values[3 * i + 1], values[3 * i + 2]};
  const auto weight = static_cast<long double>(values[4 * i + 3]);
  return {static_cast<double>(values[4 * i] / weight),
          static_cast<double>(values[4 * i + 1] / weight),
          static_cast<double>(values[4 * i + 2] / weight)};
}
Result<Vector> MonotoneProjection(const NurbsCurveGeometry& curve) {
  auto direction = Difference(Control(curve, curve.axis.control_count - 1),
                              Control(curve, 0));
  const auto length = Length(direction);
  if (!std::isfinite(length) || length == 0)
    return Result<Vector>::Err(
        Unsupported("Spline inversion requires a nonzero endpoint chord"));
  for (auto& coordinate : direction) coordinate /= length;
  const auto certified = CertifyMonotoneNurbsProjection(curve, direction);
  if (!certified.ok()) return Result<Vector>::Err(certified.error());
  if (!certified.value())
    return Result<Vector>::Err(Unsupported(
        "Spline inversion requires a certified monotone projection"));
  return Result<Vector>::Ok(direction);
}
}  // namespace
Result<double> InvertMonotoneNurbsCurve(const NurbsCurveGeometry& curve,
                                        Vector3 point, double tolerance) {
  using Return = Result<double>;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(tolerance) || tolerance < 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Invalid spline inversion point or tolerance"});
  const auto domain = NurbsCurveDomain(curve);
  if (!domain.ok()) return Return::Err(domain.error());
  const auto& axis = curve.axis;
  if (curve.controls.dimension != (curve.controls.rational ? 4 : 3) ||
      axis.closed || axis.periodic ||
      axis.multiplicities.front() != axis.degree + 1 ||
      axis.multiplicities.back() != axis.degree + 1)
    return Return::Err(Unsupported(
        "Partial spline inversion requires open clamped 3D controls"));
  constexpr uint64_t kMaxWork = 32000000;
  constexpr unsigned kMaxSteps = 128;
  const uint64_t work = curve.controls.values.size() + axis.knots.size() +
                        uint64_t{axis.degree + 1} * (axis.degree + 1);
  if (work > kMaxWork / (kMaxSteps + 2))
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Partial spline inversion exceeds its work limit"});
  const auto projection = MonotoneProjection(curve);
  if (!projection.ok()) return Return::Err(projection.error());
  const auto origin = Control(curve, 0);
  const auto target = Dot(Difference(point, origin), projection.value());
  double low = domain.value()[0], high = domain.value()[1], best = low;
  long double error = std::numeric_limits<long double>::infinity();
  Vector3 position{};
  bool finished = false;
  for (unsigned i = 0; i < kMaxSteps + 2; ++i) {
    const double u = i == 0 ? low : i == 1 ? high : std::midpoint(low, high);
    const auto sample = EvaluateNurbsCurve(curve, u);
    if (!sample.ok()) return Return::Err(sample.error());
    const auto value =
        Dot(Difference(sample.value().position, origin), projection.value());
    const auto residual = std::abs(value - target);
    if (residual < error) {
      error = residual;
      best = u;
      position = sample.value().position;
    }
    if (i == 0) continue;
    if (i == 1) {
      if (target <= 0 || target >= value) {
        finished = true;
        break;
      }
      continue;
    }
    if (value == target || u == low || u == high) {
      finished = true;
      break;
    }
    if (value < target)
      low = u;
    else
      high = u;
  }
  if (!finished)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0,
         "Partial spline inversion exhausted its search budget"});
  // Projection supplies a unique candidate, not a closest-point assertion.
  // Retain the original knot units and require full 3D source membership.
  if (Length(Difference(position, point)) > tolerance)
    return Return::Err(Unsupported(
        "Spline vertex has no verified monotone parameter within tolerance"));
  return Return::Ok(best);
}
}  // namespace openparasolid::internal
