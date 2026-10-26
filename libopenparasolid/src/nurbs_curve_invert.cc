// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
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
Result<bool> CertifyBezierProjection(const NurbsCurveGeometry& curve,
                                     const Vector& direction) {
  const auto degree = curve.axis.degree;
  if (degree > 7 || curve.axis.control_count != degree + 1 ||
      curve.axis.knots.size() != 2)
    return Result<bool>::Ok(false);
  constexpr unsigned kMaxDepth = 16;
  struct Piece {
    std::array<long double, 7> coefficients{};
    unsigned depth = 0;
  };
  std::array<Piece, kMaxDepth + 1> pending{};
  unsigned count = 1;
  long double scale = 0;
  for (unsigned i = 0; i < degree; ++i) {
    const auto step = Difference(Control(curve, i + 1), Control(curve, i));
    pending[0].coefficients[i] = Dot(step, direction);
    scale = std::max(scale, Length(step));
  }
  // Bernstein subdivision is a convex combination. This conservative margin
  // covers projection and all bounded averaging steps, including cancellation.
  const auto margin = 256 * (kMaxDepth + degree) *
                      std::numeric_limits<long double>::epsilon() * scale;
  constexpr uint64_t kMaxCertificateWork = 1000000;
  uint64_t remaining = kMaxCertificateWork;
  while (count) {
    const auto piece = pending[--count];
    bool positive = true;
    for (unsigned i = 0; i < degree; ++i)
      positive = positive && piece.coefficients[i] > margin;
    if (positive) continue;
    if (piece.depth == kMaxDepth) return Result<bool>::Ok(false);
    Piece left, right;
    left.depth = right.depth = piece.depth + 1;
    auto row = piece.coefficients;
    for (unsigned level = 0; level < degree; ++level) {
      left.coefficients[level] = row[0];
      right.coefficients[degree - 1 - level] = row[degree - 1 - level];
      for (unsigned j = 0; j + level + 1 < degree; ++j) {
        if (!remaining)
          return Result<bool>::Err(
              {ErrorCode::kResourceLimit, 0,
               "Spline projection certificate exhausted its work budget"});
        --remaining;
        row[j] = (row[j] + row[j + 1]) / 2;
      }
    }
    pending[count++] = right;
    pending[count++] = left;
  }
  return Result<bool>::Ok(true);
}
uint32_t Binomial(unsigned n, unsigned k) {
  uint32_t result = 1;
  for (unsigned i = 1; i <= k; ++i) result = result * (n + 1 - i) / i;
  return result;
}
Result<bool> CertifyRationalProjection(const NurbsCurveGeometry& curve,
                                       const Vector& direction) {
  using Return = Result<bool>;
  const auto degree = curve.axis.degree;
  if (degree > 7) return Return::Ok(false);
  // A clamped spline with every interior multiplicity equal to its degree is
  // already a chain of Bezier pieces. No knot insertion or fitting is needed.
  const auto spans = curve.axis.knots.size() - 1;
  for (std::size_t i = 1; i < spans; ++i)
    if (curve.axis.multiplicities[i] != degree) return Return::Ok(false);
  constexpr uint64_t kMaxCertificateWork = 1000000;
  const uint64_t work_per_span = (degree + 1) * (degree + 1);
  if (spans > kMaxCertificateWork / work_per_span)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Rational spline projection exceeds its work limit"});
  const auto& values = curve.controls.values;
  for (std::size_t span = 0; span < spans; ++span) {
    std::array<long double, 8> projected{}, weights{};
    long double max_weight = 0;
    for (unsigned i = 0; i <= degree; ++i)
      max_weight = std::max(
          max_weight,
          static_cast<long double>(values[4 * (span * degree + i) + 3]));
    const auto first = 4 * span * degree;
    const long double first_weight = values[first + 3];
    const Vector origin{values[first] / first_weight,
                        values[first + 1] / first_weight,
                        values[first + 2] / first_weight};
    long double scale = Length(origin);
    for (unsigned i = 0; i <= degree; ++i) {
      const auto at = 4 * (span * degree + i);
      const long double weight = values[at + 3];
      Vector point{values[at] / weight, values[at + 1] / weight,
                   values[at + 2] / weight};
      scale = std::max(scale, Length(point));
      for (unsigned k = 0; k < 3; ++k) point[k] -= origin[k];
      weights[i] = weight / max_weight;
      projected[i] = Dot(point, direction) * weights[i];
      if (!std::isfinite(projected[i]) || !(weights[i] > 0))
        return Return::Ok(false);
    }
    // For C=A/W, the sign of projected C' is the sign of A'W-AW'.
    // Its Bernstein product coefficients cover the entire original knot span;
    // positive weights make W^2 positive. Degree and operation counts are
    // bounded. The absolute-coordinate margin also covers cancellation in
    // dehomogenization and the local translation used to form A.
    std::array<long double, 14> numerator{};
    const unsigned product_degree = 2 * degree - 1;
    for (unsigned i = 0; i < degree; ++i)
      for (unsigned j = 0; j <= degree; ++j) {
        const auto factor = static_cast<long double>(degree) *
                            Binomial(degree - 1, i) * Binomial(degree, j) /
                            Binomial(product_degree, i + j);
        numerator[i + j] +=
            factor * ((projected[i + 1] - projected[i]) * weights[j] -
                      (weights[i + 1] - weights[i]) * projected[j]);
      }
    const auto margin = 1024 * (product_degree + 1) *
                        std::numeric_limits<long double>::epsilon() * scale;
    if (!std::isfinite(margin)) return Return::Ok(false);
    for (unsigned k = 0; k <= product_degree; ++k)
      if (!std::isfinite(numerator[k]) || !(numerator[k] > margin))
        return Return::Ok(false);
  }
  return Return::Ok(true);
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
Result<bool> CertifyMonotoneNurbsProjection(
    const NurbsCurveGeometry& curve, std::array<long double, 3> direction) {
  const auto domain = NurbsCurveDomain(curve);
  if (!domain.ok()) return Result<bool>::Err(domain.error());
  const auto& axis = curve.axis;
  if (curve.controls.dimension != (curve.controls.rational ? 4 : 3) ||
      axis.closed || axis.periodic ||
      axis.multiplicities.front() != axis.degree + 1 ||
      axis.multiplicities.back() != axis.degree + 1)
    return Result<bool>::Ok(false);
  const auto length = Length(direction);
  if (!std::isfinite(length) || length == 0)
    return Result<bool>::Err({ErrorCode::kInvalidOptions, 0,
                              "Invalid monotone spline projection direction"});
  for (auto& coordinate : direction) coordinate /= length;
  if (curve.controls.rational)
    return CertifyRationalProjection(curve, direction);
  for (std::size_t i = 1; i < curve.axis.control_count; ++i) {
    const auto step = Difference(Control(curve, i), Control(curve, i - 1));
    // Nonrational B-spline derivative controls are positive multiples of
    // successive pole differences. Strictly positive projection proves global
    // injectivity, independently of local Newton convergence or curve samples.
    const auto margin =
        64 * std::numeric_limits<long double>::epsilon() * Length(step);
    if (!(Dot(step, direction) > margin)) {
      // A negative derivative control does not alone prove a fold. For one
      // Bezier span, subdivision can still certify positivity over all of u.
      const auto refined = CertifyBezierProjection(curve, direction);
      if (!refined.ok()) return Result<bool>::Err(refined.error());
      return refined;
    }
  }
  return Result<bool>::Ok(true);
}
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
