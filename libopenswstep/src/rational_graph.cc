// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "rational_graph.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace openswstep::internal {
namespace {
namespace bounds = sweep_bounds;
using namespace openparasolid;
unsigned Choose(unsigned n, unsigned k) {
  unsigned out = 1;
  for (unsigned i = 1; i <= k; ++i) out = out * (n + 1 - i) / i;
  return out;
}
ParseError Unsupported() {
  return {
      ErrorCode::kUnsupportedFormat, 0,
      "Rational swept graph has no certified globally unique chord projection"};
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Rational swept graph derivative budget exhausted"};
}
}  // namespace
Result<long double> RationalGraphChordCondition(
    std::span<const bounds::Vector> homogeneous,
    std::span<const double> weights, Vector3 origin, bounds::Vector chord,
    int direction, uint64_t& checks) {
  using Return = Result<long double>;
  if (homogeneous.size() != weights.size() || weights.size() < 3 ||
      weights.size() > 4 || (direction != 1 && direction != -1))
    return Return::Err(Unsupported());
  const unsigned p = static_cast<unsigned>(weights.size() - 1),
                 degree = 2 * p - 1;
  if (!bounds::Spend(checks, 128 * (p + 1) * (p + 1)))
    return Return::Err(Limit());
  long double scale = 0;
  for (auto w : weights) {
    if (!std::isfinite(w) || w <= 0) return Return::Err(Unsupported());
    scale = std::max(scale, static_cast<long double>(w));
  }
  std::array<bounds::Interval, 4> normalized{};
  std::array<bounds::Vector, 4> relative{};
  long double min_weight = bounds::kInfinity, max_weight = 0;
  const auto anchor = bounds::Vec(origin);
  for (unsigned i = 0; i <= p; ++i) {
    normalized[i] =
        bounds::Div(bounds::Exact(weights[i]), bounds::Exact(scale));
    min_weight = std::min(min_weight, normalized[i].low);
    max_weight = std::max(max_weight, normalized[i].high);
    for (unsigned k = 0; k < 3; ++k)
      relative[i][k] =
          bounds::Sub(bounds::Div(homogeneous[i][k], bounds::Exact(scale)),
                      bounds::Mul(anchor[k], normalized[i]));
  }
  if (!(min_weight > 0) || !std::isfinite(max_weight))
    return Return::Err(Unsupported());
  std::array<bounds::Vector, 6> numerator{};
  for (unsigned i = 0; i < p; ++i)
    for (unsigned j = 0; j <= p; ++j) {
      const auto factor =
          bounds::Div(bounds::Exact(p * Choose(p - 1, i) * Choose(p, j)),
                      bounds::Exact(Choose(degree, i + j)));
      for (unsigned k = 0; k < 3; ++k) {
        const auto value = bounds::Sub(
            bounds::Mul(bounds::Sub(relative[i + 1][k], relative[i][k]),
                        normalized[j]),
            bounds::Mul(bounds::Sub(normalized[i + 1], normalized[i]),
                        relative[j][k]));
        numerator[i + j][k] =
            bounds::Add(numerator[i + j][k], bounds::Mul(factor, value));
      }
    }
  struct Piece {
    std::array<bounds::Interval, 6> coefficients{};
    unsigned depth = 0;
  };
  constexpr unsigned kMaxDepth = 16;
  std::array<Piece, kMaxDepth + 1> pending{};
  unsigned count = 1;
  long double maximum_speed = 0, minimum_projection = bounds::kInfinity;
  for (unsigned i = 0; i <= degree; ++i) {
    maximum_speed = std::max(maximum_speed, bounds::Length(numerator[i]));
    pending[0].coefficients[i] =
        bounds::Mul(bounds::Dot(numerator[i], chord), bounds::Exact(direction));
  }
  while (count) {
    const auto piece = pending[--count];
    if (!bounds::Spend(checks, degree + 1)) return Return::Err(Limit());
    bool positive = true;
    long double lower = bounds::kInfinity;
    for (unsigned i = 0; i <= degree; ++i) {
      if (!bounds::Valid(piece.coefficients[i]))
        return Return::Err(Unsupported());
      lower = std::min(lower, piece.coefficients[i].low);
      positive = positive && piece.coefficients[i].low > 0;
    }
    if (positive) {
      minimum_projection = std::min(minimum_projection, lower);
      continue;
    }
    if (piece.depth == kMaxDepth || piece.coefficients[0].high <= 0 ||
        piece.coefficients[degree].high <= 0)
      return Return::Err(Unsupported());
    if (!bounds::Spend(checks, 4 * degree * (degree + 1)))
      return Return::Err(Limit());
    Piece left, right;
    left.depth = right.depth = piece.depth + 1;
    auto row = piece.coefficients;
    for (unsigned level = 0; level <= degree; ++level) {
      left.coefficients[level] = row[0];
      right.coefficients[degree - level] = row[degree - level];
      for (unsigned j = 0; j + level < degree; ++j)
        row[j] =
            bounds::Mul(bounds::Add(row[j], row[j + 1]), bounds::Exact(.5L));
    }
    pending[count++] = right;
    pending[count++] = left;
  }
  // C' = N/W^2. Positive denominator bounds and the whole numerator hull
  // condition the query-plane residual in physical length units.
  const auto ratio = bounds::Div(
      bounds::Mul(
          bounds::Mul(bounds::Exact(maximum_speed),
                      bounds::Exact(bounds::Length(chord))),
          bounds::Mul(bounds::Exact(max_weight), bounds::Exact(max_weight))),
      bounds::Mul(
          bounds::Exact(minimum_projection),
          bounds::Mul(bounds::Exact(min_weight), bounds::Exact(min_weight))));
  if (!bounds::Valid(ratio)) return Return::Err(Unsupported());
  return Return::Ok(ratio.high);
}
}  // namespace openswstep::internal
