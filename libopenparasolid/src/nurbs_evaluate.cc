// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
constexpr uint32_t kMaxDegree = 64;
constexpr std::size_t kMaxValues = 1000000;

ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}

std::optional<ParseError> Validate(const NurbsCurveGeometry& curve) {
  const auto& axis = curve.axis;
  const auto& controls = curve.controls;
  if (axis.degree > kMaxDegree || axis.knots.size() > kMaxValues ||
      axis.multiplicities.size() > kMaxValues ||
      controls.values.size() > kMaxValues ||
      uint64_t{axis.control_count} + axis.degree + 1 > kMaxValues) {
    return ParseError{ErrorCode::kResourceLimit, 0,
                      "NURBS evaluation exceeds degree or value limits"};
  }
  if (axis.degree == 0 || axis.degree >= axis.control_count ||
      axis.knots.size() < 2 ||
      axis.knots.size() != axis.multiplicities.size() ||
      (axis.periodic && !axis.closed)) {
    return Invalid("Invalid NURBS degree, knot arrays or closure flags");
  }
  const uint16_t dimension = controls.dimension;
  if ((controls.rational && dimension != 3 && dimension != 4) ||
      (!controls.rational && dimension != 2 && dimension != 3) ||
      controls.values.size() != uint64_t{axis.control_count} * dimension) {
    return Invalid("Invalid NURBS control dimension or array size");
  }
  uint64_t expanded = 0;
  for (std::size_t i = 0; i < axis.knots.size(); ++i) {
    const auto count = axis.multiplicities[i];
    if (!std::isfinite(axis.knots[i]) ||
        (i > 0 && axis.knots[i] <= axis.knots[i - 1]) || count == 0 ||
        count > axis.degree + 1 ||
        (i > 0 && i + 1 < axis.knots.size() && count > axis.degree)) {
      return Invalid("Invalid NURBS distinct knots or multiplicities");
    }
    expanded += count;
  }
  if (expanded != uint64_t{axis.control_count} + axis.degree + 1) {
    return Invalid(
        "NURBS expanded knot count does not match controls and degree");
  }
  for (std::size_t i = 0; i < controls.values.size(); ++i) {
    if (!std::isfinite(controls.values[i]) ||
        (controls.rational && i % dimension + 1 == dimension &&
         controls.values[i] <= 0)) {
      return Invalid(
          "NURBS controls must be finite with positive rational weights");
    }
  }
  return std::nullopt;
}

// Homogeneous value and its derivative in the original parameter. Extended
// precision reduces intermediate overflow; output must still fit in double.
struct Jet {
  std::array<long double, 4> value{};
  std::array<long double, 4> derivative{};
};

Jet Control(const SplineControlNet& controls, std::size_t index) {
  Jet result;
  const std::size_t dimension = controls.dimension;
  const std::size_t spatial = dimension - (controls.rational ? 1u : 0u);
  for (std::size_t i = 0; i < spatial; ++i) {
    result.value[i] = controls.values[index * dimension + i];
  }
  result.value[3] =
      controls.rational ? controls.values[index * dimension + spatial] : 1;
  return result;
}

Result<CurveEvaluation> Project(const Jet& jet) {
  const long double weight = jet.value[3];
  if (!std::isfinite(weight) || weight <= 0) {
    return Result<CurveEvaluation>::Err(
        Invalid("NURBS evaluation produced an invalid homogeneous weight"));
  }
  std::array<double, 3> position{}, derivative{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    const long double point = jet.value[i] / weight;
    const long double tangent =
        (jet.derivative[i] - point * jet.derivative[3]) / weight;
    // Test representability before conversion; an out-of-range floating-point
    // conversion is not a portable way to obtain infinity.
    constexpr long double limit = std::numeric_limits<double>::max();
    if (!std::isfinite(point) || !std::isfinite(tangent) ||
        std::abs(point) > limit || std::abs(tangent) > limit) {
      return Result<CurveEvaluation>::Err(Invalid(
          "NURBS evaluation produced a nonfinite or out-of-range result"));
    }
    position[i] = static_cast<double>(point);
    derivative[i] = static_cast<double>(tangent);
  }
  return Result<CurveEvaluation>::Ok(
      {{position[0], position[1], position[2]},
       {derivative[0], derivative[1], derivative[2]}});
}
}  // namespace

Result<CurveEvaluation> EvaluateNurbsCurve(const NurbsCurveGeometry& curve,
                                           double parameter) {
  if (auto error = Validate(curve)) {
    return Result<CurveEvaluation>::Err(std::move(*error));
  }
  const auto& axis = curve.axis;
  std::vector<double> knots;
  knots.reserve(std::size_t{axis.control_count} + axis.degree + 1);
  for (std::size_t i = 0; i < axis.knots.size(); ++i) {
    knots.insert(knots.end(), axis.multiplicities[i], axis.knots[i]);
  }
  const std::size_t degree = axis.degree;
  const double start = knots[degree];
  const double end = knots[axis.control_count];
  if (!(start < end)) {
    return Result<CurveEvaluation>::Err(
        Invalid("NURBS parameter domain is empty"));
  }
  if (parameter < start || parameter > end) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "NURBS parameter is outside its knot domain"});
  }
  // At a repeated interior knot use the right-hand polynomial; at the domain
  // endpoint use the left-hand polynomial, including for unclamped knots.
  const auto boundary =
      parameter == end
          ? std::lower_bound(knots.begin(), knots.end(), end)
          : std::upper_bound(knots.begin(), knots.end(), parameter);
  const auto span = static_cast<std::size_t>(boundary - knots.begin() - 1);
  std::vector<Jet> work;
  work.reserve(degree + 1);
  for (std::size_t j = 0; j <= degree; ++j) {
    work.push_back(Control(curve.controls, span - degree + j));
  }
  // De Boor interpolation with derivative propagation. Only degree+1 controls
  // are needed; descending updates preserve the previous recursion level.
  for (std::size_t level = 1; level <= degree; ++level) {
    for (std::size_t j = degree; j >= level; --j) {
      const std::size_t i = span - degree + j;
      const long double low = knots[i];
      const long double width =
          static_cast<long double>(knots[i + degree + 1 - level]) - low;
      if (!std::isfinite(width) || width <= 0) {
        return Result<CurveEvaluation>::Err(
            Invalid("NURBS knot interval is degenerate or not representable"));
      }
      const long double alpha =
          (static_cast<long double>(parameter) - low) / width;
      const Jet left = work[j - 1];
      const Jet right = work[j];
      for (std::size_t c = 0; c < 4; ++c) {
        work[j].value[c] = std::lerp(left.value[c], right.value[c], alpha);
        work[j].derivative[c] =
            std::lerp(left.derivative[c], right.derivative[c], alpha) +
            (right.value[c] - left.value[c]) / width;
      }
    }
  }
  return Project(work[degree]);
}
}  // namespace openparasolid::internal
