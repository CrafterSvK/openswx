// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "rational_integral.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include "nurbs_wire.h"
namespace openswstep::internal {
namespace {
using namespace openparasolid;
struct Interval {
  long double low, high;
};
constexpr auto kInfinity = std::numeric_limits<long double>::infinity();
long double Down(long double x) { return std::nextafter(x, -kInfinity); }
long double Up(long double x) { return std::nextafter(x, kInfinity); }
bool Valid(Interval a) {
  return std::isfinite(a.low) && std::isfinite(a.high) && a.low <= a.high;
}
Interval Unbounded() { return {-kInfinity, kInfinity}; }
Interval Exact(long double x) { return {x, x}; }
Interval Add(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  return {Down(a.low + b.low), Up(a.high + b.high)};
}
Interval Sub(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  return {Down(a.low - b.high), Up(a.high - b.low)};
}
Interval Mul(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  const std::array<long double, 4> products{a.low * b.low, a.low * b.high,
                                            a.high * b.low, a.high * b.high};
  for (const auto value : products)
    if (!std::isfinite(value)) return Unbounded();
  return {Down(*std::min_element(products.begin(), products.end())),
          Up(*std::max_element(products.begin(), products.end()))};
}
Interval Divide(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b) || !(b.low > 0)) return Unbounded();
  return Mul(a, {Down(1 / b.high), Up(1 / b.low)});
}
unsigned Choose(unsigned n, unsigned k) {
  unsigned value = 1;
  for (unsigned i = 1; i <= k; ++i) value = value * (n + 1 - i) / i;
  return value;
}
using Vector = std::array<Interval, 3>;
using Coefficients = std::array<Interval, 10>;
struct Piece {
  Coefficients numerator{}, denominator{};
  unsigned depth = 0;
};
Interval CrossDot(const Vector& a, const Vector& b, const Vector& n) {
  Interval value{};
  for (unsigned k = 0; k < 3; ++k)
    value = Add(value, Mul(Sub(Mul(a[(k + 1) % 3], b[(k + 2) % 3]),
                               Mul(a[(k + 2) % 3], b[(k + 1) % 3])),
                           n[k]));
  return value;
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Rational integral work limit exceeded"};
}
Result<Interval> IntegratePiece(const NurbsCurveGeometry& curve, Vector3 origin,
                                Vector3 normal, long double normal_error,
                                uint64_t& checks,
                                std::span<const ScalarBound> heights = {}) {
  using Return = Result<Interval>;
  const unsigned p = curve.axis.degree;
  unsigned degree = 2 * p;
  if (curve.axis.control_count != p + 1 || curve.axis.knots.size() != 2)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Rational integral requires a single Bezier span"});
  const uint64_t products = (p + 1) * (p + 1) + p * (p + 1);
  if (checks < products) return Return::Err(Limit());
  checks -= products;
  std::array<Vector, 4> controls{};
  std::array<Interval, 4> weights{};
  Vector n{};
  const std::array<double, 3> o{origin.x, origin.y, origin.z},
      axis{normal.x, normal.y, normal.z};
  for (unsigned k = 0; k < 3; ++k)
    n[k] = {Down(axis[k] - normal_error), Up(axis[k] + normal_error)};
  const auto& values = curve.controls.values;
  long double scale = 0;
  for (unsigned i = 0; i <= p; ++i)
    scale = std::max(scale, static_cast<long double>(values[4 * i + 3]));
  for (unsigned i = 0; i <= p; ++i) {
    weights[i] = Divide(Exact(values[4 * i + 3]), Exact(scale));
    for (unsigned k = 0; k < 3; ++k)
      controls[i][k] = Sub(Divide(Exact(values[4 * i + k]), Exact(scale)),
                           Mul(weights[i], Exact(o[k])));
  }
  Piece root;
  Coefficients numerator{};
  for (unsigned i = 0; i <= p; ++i)
    for (unsigned j = 0; j <= p; ++j) {
      auto factor = Divide(Exact(Choose(p, i) * Choose(p, j)),
                           Exact(Choose(degree, i + j)));
      root.denominator[i + j] = Add(root.denominator[i + j],
                                    Mul(factor, Mul(weights[i], weights[j])));
    }
  for (unsigned i = 0; i <= p; ++i)
    for (unsigned j = 0; j < p; ++j) {
      Vector derivative{};
      for (unsigned k = 0; k < 3; ++k)
        derivative[k] = Mul(Exact(p), Sub(controls[j + 1][k], controls[j][k]));
      auto factor = Divide(Exact(Choose(p, i) * Choose(p - 1, j)),
                           Exact(Choose(degree - 1, i + j)));
      numerator[i + j] = Add(numerator[i + j],
                             Mul(factor, CrossDot(controls[i], derivative, n)));
    }
  // Elevate the numerator to the denominator's degree. Their coefficient
  // ratios bound the rational function because denominator weights are
  // positive.
  for (unsigned k = 0; k <= degree; ++k) {
    if (k < degree)
      root.numerator[k] =
          Mul(numerator[k], Divide(Exact(degree - k), Exact(degree)));
    if (k)
      root.numerator[k] =
          Add(root.numerator[k],
              Mul(numerator[k - 1], Divide(Exact(k), Exact(degree))));
  }
  if (!heights.empty()) {
    const auto weighted_products = uint64_t{2} * (degree + 1) * (p + 1);
    if (checks < weighted_products) return Return::Err(Limit());
    checks -= weighted_products;
    Piece weighted;
    for (unsigned i = 0; i <= degree; ++i)
      for (unsigned j = 0; j <= p; ++j) {
        const auto factor = Divide(Exact(Choose(degree, i) * Choose(p, j)),
                                   Exact(Choose(degree + p, i + j)));
        const auto height =
            Mul({heights[j].lower, heights[j].upper}, weights[j]);
        weighted.numerator[i + j] =
            Add(weighted.numerator[i + j],
                Mul(factor, Mul(root.numerator[i], height)));
        weighted.denominator[i + j] =
            Add(weighted.denominator[i + j],
                Mul(factor, Mul(root.denominator[i], weights[j])));
      }
    root = weighted;
    degree += p;
  }
  constexpr unsigned kDepth = 6;
  std::array<Piece, kDepth + 1> stack{};
  stack[0] = root;
  unsigned size = 1;
  Interval result{};
  while (size) {
    const auto piece = stack[--size];
    if (piece.depth == kDepth) {
      if (checks < degree + 1) return Return::Err(Limit());
      checks -= degree + 1;
      Interval bound{kInfinity, -kInfinity};
      for (unsigned i = 0; i <= degree; ++i) {
        const auto ratio = Divide(piece.numerator[i], piece.denominator[i]);
        if (!Valid(ratio))
          return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                              "Rational integral quotient is unrepresentable"});
        bound.low = std::min(bound.low, ratio.low);
        bound.high = std::max(bound.high, ratio.high);
      }
      result = Add(result, Mul(bound, Exact(1.L / (1u << kDepth))));
      continue;
    }
    const auto cost = degree * (degree + 1);
    if (checks < cost) return Return::Err(Limit());
    checks -= cost;
    Piece left, right;
    left.depth = right.depth = piece.depth + 1;
    for (unsigned which = 0; which < 2; ++which) {
      auto row = which ? piece.denominator : piece.numerator;
      auto& a = which ? left.denominator : left.numerator;
      auto& b = which ? right.denominator : right.numerator;
      for (unsigned level = 0; level <= degree; ++level) {
        a[level] = row[0];
        b[degree - level] = row[degree - level];
        for (unsigned j = 0; j + level < degree; ++j)
          row[j] = Mul(Add(row[j], row[j + 1]), Exact(.5L));
      }
    }
    stack[size++] = right;
    stack[size++] = left;
  }
  if (!std::isfinite(result.low) || !std::isfinite(result.high) ||
      result.low > result.high)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Rational integral cannot certify finite bounds"});
  return Return::Ok(result);
}
}  // namespace
Result<BoundedIntegral> RationalWeightedCurveIntegral(
    const NurbsCurveGeometry& curve, std::span<const ScalarBound> heights,
    Vector3 origin, Vector3 normal, uint64_t& checks) {
  using Return = Result<BoundedIntegral>;
  for (const double value :
       {origin.x, origin.y, origin.z, normal.x, normal.y, normal.z})
    if (!std::isfinite(value))
      return Return::Err(
          {ErrorCode::kInvalidOptions, 0, "Invalid weighted integral frame"});
  if (!curve.controls.rational || curve.controls.dimension != 4 ||
      (curve.axis.degree != 2 && curve.axis.degree != 3) ||
      curve.axis.knots.size() != 2 ||
      curve.axis.control_count != curve.axis.degree + 1 ||
      heights.size() != curve.axis.control_count)
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Weighted rational integral requires one "
                        "quadratic/cubic span and matching heights"});
  for (const auto& height : heights)
    if (!std::isfinite(height.lower) || !std::isfinite(height.upper) ||
        height.lower <= 0 || height.upper < height.lower)
      return Return::Err({ErrorCode::kInvalidOptions, 0,
                          "Invalid weighted integral height bound"});
  const auto validated = RestrictNurbsWire(curve, curve.axis.knots.front(),
                                           curve.axis.knots.back(), checks);
  if (!validated.ok()) return Return::Err(validated.error());
  const auto result =
      IntegratePiece(validated.value(), origin, normal, 0, checks, heights);
  if (!result.ok()) return Return::Err(result.error());
  const auto value = std::midpoint(result.value().low, result.value().high);
  const auto error =
      Up(std::max(value - result.value().low, result.value().high - value));
  if (!std::isfinite(value) || !std::isfinite(error))
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Weighted rational integral is unrepresentable"});
  return Return::Ok({value, error});
}
Result<BoundedIntegral> RationalCurveIntegral(const NurbsCurveGeometry& curve,
                                              Vector3 origin, Vector3 normal,
                                              uint64_t& checks,
                                              long double normal_error) {
  using Return = Result<BoundedIntegral>;
  if (!std::isfinite(normal_error) || normal_error < 0 ||
      !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
      !std::isfinite(origin.z) || !std::isfinite(normal.x) ||
      !std::isfinite(normal.y) || !std::isfinite(normal.z))
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Invalid rational integral normal error"});
  if (!curve.controls.rational || curve.controls.dimension != 4 ||
      (curve.axis.degree != 2 && curve.axis.degree != 3) ||
      curve.axis.knots.size() < 2)
    return Return::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Rational integral requires quadratic/cubic 3D controls"});
  Interval total{};
  for (std::size_t i = 0; i + 1 < curve.axis.knots.size(); ++i) {
    const auto span = RestrictNurbsWire(curve, curve.axis.knots[i],
                                        curve.axis.knots[i + 1], checks);
    if (!span.ok()) return Return::Err(span.error());
    const auto value =
        IntegratePiece(span.value(), origin, normal, normal_error, checks);
    if (!value.ok()) return Return::Err(value.error());
    total = Add(total, value.value());
  }
  const auto value = std::midpoint(total.low, total.high);
  const auto error = Up(std::max(value - total.low, total.high - value));
  if (!std::isfinite(value) || !std::isfinite(error))
    return Return::Err({ErrorCode::kUnsupportedFormat, 0,
                        "Rational integral sum is unrepresentable"});
  return Return::Ok({value, error});
}
}  // namespace openswstep::internal
