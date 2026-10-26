// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
// Scratch arithmetic encloses exact operations on the recorded double data.
// It is used only for certification; neither knots nor controls are replaced.
constexpr unsigned kMaxDegree = 7;
constexpr unsigned kMaxDepth = 16;
constexpr uint64_t kMaxWork = 1000000;
constexpr auto kInfinity = std::numeric_limits<long double>::infinity();
struct Interval {
  long double low = 0, high = 0;
};
Interval Exact(long double x) { return {x, x}; }
bool Finite(Interval a) {
  return std::isfinite(a.low) && std::isfinite(a.high) && a.low <= a.high;
}
Interval Enclose(long double low, long double high) {
  return {std::nextafter(low, -kInfinity), std::nextafter(high, kInfinity)};
}
Interval Add(Interval a, Interval b) {
  return Enclose(a.low + b.low, a.high + b.high);
}
Interval Subtract(Interval a, Interval b) {
  return Enclose(a.low - b.high, a.high - b.low);
}
Interval Multiply(Interval a, Interval b) {
  const std::array<long double, 4> products{a.low * b.low, a.low * b.high,
                                            a.high * b.low, a.high * b.high};
  for (const auto p : products)
    if (!std::isfinite(p)) return {-kInfinity, kInfinity};
  return Enclose(*std::min_element(products.begin(), products.end()),
                 *std::max_element(products.begin(), products.end()));
}
Interval Divide(Interval a, Interval b) {
  if (!Finite(b) || b.low <= 0) return {-kInfinity, kInfinity};
  return Multiply(a, Enclose(1 / b.high, 1 / b.low));
}
Interval Mix(Interval a, Interval b, Interval t) {
  return Add(Multiply(Subtract(Exact(1), t), a), Multiply(t, b));
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Spline projection certificate exhausted its work budget"};
}
bool Spend(uint64_t& remaining, uint64_t count) {
  if (count > remaining) return false;
  remaining -= count;
  return true;
}
uint32_t Binomial(unsigned n, unsigned k) {
  uint32_t result = 1;
  for (unsigned i = 1; i <= k; ++i) result = result * (n + 1 - i) / i;
  return result;
}
struct HomogeneousProjection {
  Interval value, weight;
};
using SpanControls = std::array<HomogeneousProjection, kMaxDegree + 1>;
// The p+1 Bernstein controls are blossom values with p-i arguments at the
// left knot and i at the right knot. Generalized de Boor evaluates each using
// only the p+1 original poles active on this span. This avoids global repeated
// insertion/copying and bounds extraction independently of earlier spans.
SpanControls ExtractSpan(const SpanControls& source, unsigned degree,
                         const std::vector<double>& knots, std::size_t span) {
  SpanControls result{};
  for (unsigned control = 0; control <= degree; ++control) {
    auto row = source;
    for (unsigned level = 1; level <= degree; ++level) {
      const auto argument = knots[span + (level > degree - control ? 1 : 0)];
      for (unsigned j = degree; j >= level; --j) {
        const auto low = knots[span - degree + j],
                   high = knots[span + 1 + j - level];
        const auto alpha = Divide(Subtract(Exact(argument), Exact(low)),
                                  Subtract(Exact(high), Exact(low)));
        row[j].value = Mix(row[j - 1].value, row[j].value, alpha);
        row[j].weight = Mix(row[j - 1].weight, row[j].weight, alpha);
      }
    }
    result[control] = row[degree];
  }
  return result;
}
using Numerator = std::array<Interval, 2 * kMaxDegree>;
Numerator DerivativeNumerator(const SpanControls& controls, unsigned degree) {
  Numerator result{};
  // For C=A/W, sign(C') = sign(A'W-AW') because all weights are positive.
  // The positive span width does not affect the sign. Bernstein products
  // enclose the entire rational derivative numerator in original knot units.
  for (unsigned i = 0; i < degree; ++i)
    for (unsigned j = 0; j <= degree; ++j) {
      const auto factor =
          Divide(Exact(uint64_t{degree} * Binomial(degree - 1, i) *
                       Binomial(degree, j)),
                 Exact(Binomial(2 * degree - 1, i + j)));
      const auto difference = Subtract(
          Multiply(Subtract(controls[i + 1].value, controls[i].value),
                   controls[j].weight),
          Multiply(Subtract(controls[i + 1].weight, controls[i].weight),
                   controls[j].value));
      result[i + j] = Add(result[i + j], Multiply(factor, difference));
    }
  return result;
}
Result<bool> CertifyNumerator(Numerator coefficients, unsigned degree,
                              uint64_t& remaining) {
  struct Piece {
    Numerator coefficients{};
    unsigned depth = 0;
  };
  std::array<Piece, kMaxDepth + 1> pending{};
  pending[0].coefficients = coefficients;
  unsigned count = 1;
  while (count) {
    const auto piece = pending[--count];
    if (!Spend(remaining, degree + 1)) return Result<bool>::Err(Limit());
    bool positive = true;
    for (unsigned i = 0; i <= degree; ++i) {
      if (!Finite(piece.coefficients[i])) return Result<bool>::Ok(false);
      positive = positive && piece.coefficients[i].low > 0;
    }
    if (positive) continue;
    // End coefficients are actual endpoint values, unlike interior poles.
    if (piece.coefficients[0].high <= 0 ||
        piece.coefficients[degree].high <= 0 || piece.depth == kMaxDepth)
      return Result<bool>::Ok(false);
    const uint64_t work = uint64_t{degree} * (degree + 1) / 2;
    if (!Spend(remaining, 4 * work)) return Result<bool>::Err(Limit());
    Piece left, right;
    left.depth = right.depth = piece.depth + 1;
    auto row = piece.coefficients;
    for (unsigned level = 0; level <= degree; ++level) {
      left.coefficients[level] = row[0];
      right.coefficients[degree - level] = row[degree - level];
      for (unsigned j = 0; j + level < degree; ++j)
        row[j] = Multiply(Add(row[j], row[j + 1]), Exact(.5L));
    }
    pending[count++] = right;
    pending[count++] = left;
  }
  return Result<bool>::Ok(true);
}
Result<bool> CertifySpans(const NurbsCurveGeometry& curve,
                          const std::array<long double, 3>& direction) {
  using Return = Result<bool>;
  const unsigned degree = curve.axis.degree;
  if (degree > kMaxDegree) return Return::Ok(false);
  const auto spans = curve.axis.knots.size() - 1;
  // Charge extraction and numerator arithmetic before allocation or span work.
  // Each blend bounds two homogeneous scalars; 32 units cover its arithmetic.
  const uint64_t per_span =
      32 * ((degree + 1) * degree * (degree + 1) / 2 + degree * (degree + 1));
  uint64_t remaining = kMaxWork;
  const uint64_t expanded_size =
      uint64_t{curve.axis.control_count} + degree + 1;
  if (spans > remaining / per_span ||
      !Spend(remaining, curve.controls.values.size() + expanded_size))
    return Return::Err(Limit());
  std::vector<double> knots;
  knots.reserve(static_cast<std::size_t>(expanded_size));
  for (std::size_t i = 0; i < curve.axis.knots.size(); ++i)
    knots.insert(knots.end(), curve.axis.multiplicities[i],
                 curve.axis.knots[i]);
  const auto& values = curve.controls.values;
  const auto dim = curve.controls.dimension;
  // Any fixed origin preserves the derivative numerator. Its rounded value
  // is an exact chosen constant; subsequent translation and positive local
  // weight normalization are enclosed, without changing source controls.
  const long double first_weight = curve.controls.rational ? values[3] : 1;
  const std::array<long double, 3> origin{values[0] / first_weight,
                                          values[1] / first_weight,
                                          values[2] / first_weight};
  for (const auto coordinate : origin)
    if (!std::isfinite(coordinate)) return Return::Ok(false);
  std::size_t span = degree;
  for (std::size_t index = 0; index < spans; ++index) {
    if (!Spend(remaining, per_span)) return Return::Err(Limit());
    SpanControls local{};
    long double max_weight = 1;
    if (curve.controls.rational) {
      max_weight = 0;
      for (unsigned i = 0; i <= degree; ++i)
        max_weight = std::max(
            max_weight,
            static_cast<long double>(values[dim * (span - degree + i) + 3]));
    }
    for (unsigned i = 0; i <= degree; ++i) {
      const auto at = dim * (span - degree + i);
      const auto weight = Exact(curve.controls.rational ? values[at + 3] : 1);
      auto& pole = local[i];
      pole.weight = Divide(weight, Exact(max_weight));
      for (unsigned k = 0; k < 3; ++k) {
        const auto translated =
            Subtract(Exact(values[at + k]), Multiply(Exact(origin[k]), weight));
        pole.value =
            Add(pole.value, Multiply(Divide(translated, Exact(max_weight)),
                                     Exact(direction[k])));
      }
    }
    const auto bezier = ExtractSpan(local, degree, knots, span);
    for (unsigned i = 0; i <= degree; ++i)
      if (!Finite(bezier[i].weight) || bezier[i].weight.low <= 0)
        return Return::Ok(false);
    const auto certified = CertifyNumerator(DerivativeNumerator(bezier, degree),
                                            2 * degree - 1, remaining);
    if (!certified.ok() || !certified.value()) return certified;
    span += curve.axis.multiplicities[index + 1];
  }
  return Return::Ok(true);
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
  const auto length = std::hypot(direction[0], direction[1], direction[2]);
  if (!std::isfinite(length) || length == 0)
    return Result<bool>::Err({ErrorCode::kInvalidOptions, 0,
                              "Invalid monotone spline projection direction"});
  for (auto& coordinate : direction) coordinate /= length;
  if (!curve.controls.rational) {
    // Preserve the cheap sufficient certificate, including degrees above seven:
    // every B-spline derivative pole is a positive multiple of a pole step.
    bool positive = true;
    for (std::size_t i = 1; i < axis.control_count; ++i) {
      const auto at = 3 * i;
      std::array<long double, 3> step{};
      long double projection = 0;
      for (unsigned k = 0; k < 3; ++k) {
        step[k] = static_cast<long double>(curve.controls.values[at + k]) -
                  curve.controls.values[at - 3 + k];
        projection += step[k] * direction[k];
      }
      const auto margin = 64 * std::numeric_limits<long double>::epsilon() *
                          std::hypot(step[0], step[1], step[2]);
      if (!(projection > margin)) {
        positive = false;
        break;
      }
    }
    if (positive) return Result<bool>::Ok(true);
  }
  return CertifySpans(curve, direction);
}
}  // namespace openparasolid::internal
