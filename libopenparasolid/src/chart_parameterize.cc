// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using Vector = std::array<long double, 3>;
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
bool Finite(Vector3 value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}
std::optional<Vector> Unit(Vector3 value) {
  if (!Finite(value)) return std::nullopt;
  const double scale =
      std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
  if (scale == 0) return std::nullopt;
  Vector result{static_cast<long double>(value.x) / scale,
                static_cast<long double>(value.y) / scale,
                static_cast<long double>(value.z) / scale};
  const long double length = std::hypot(result[0], result[1], result[2]);
  for (auto& component : result) component /= length;
  return result;
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
struct Chord {
  Vector direction;
  long double length;
};
std::optional<Chord> Between(Vector3 start, Vector3 end) {
  Vector direction{static_cast<long double>(end.x) - start.x,
                   static_cast<long double>(end.y) - start.y,
                   static_cast<long double>(end.z) - start.z};
  const long double length =
      std::hypot(direction[0], direction[1], direction[2]);
  if (!std::isfinite(length) || length == 0) return std::nullopt;
  for (auto& component : direction) component /= length;
  return Chord{direction, length};
}
bool Representable(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}
}  // namespace

Result<IntersectionChartParameters> ParameterizeIntersectionChart(
    const IntersectionChart& chart, std::span<const Vector3> tangents) {
  constexpr std::size_t kMaxCoordinateScalars = 1000000;
  const auto count = chart.positions.size();
  if (count > kMaxCoordinateScalars / 3 ||
      tangents.size() > kMaxCoordinateScalars / 3) {
    return Result<IntersectionChartParameters>::Err(
        {ErrorCode::kResourceLimit, 0,
         "Intersection chart exceeds coordinate value limit"});
  }
  if (tangents.size() != count) {
    return Result<IntersectionChartParameters>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Intersection chart requires one tangent per position"});
  }
  if (count < 2 || !std::isfinite(chart.base_parameter) ||
      !std::isfinite(chart.base_scale) || chart.base_scale <= 0) {
    return Result<IntersectionChartParameters>::Err(
        Invalid("Invalid intersection chart count or base parameter/scale"));
  }
  // Validate all input before allocating the result. Unit directions are
  // recomputed in the linear pass to avoid retaining another O(n) array.
  for (std::size_t i = 0; i < count; ++i) {
    if (!Finite(chart.positions[i]) || !Unit(tangents[i])) {
      return Result<IntersectionChartParameters>::Err(Invalid(
          "Intersection chart requires finite positions and nonzero tangents"));
    }
  }
  IntersectionChartParameters result;
  result.parameters.reserve(count);
  result.chord_scales.reserve(count - 1);
  result.parameters.push_back(chart.base_parameter);
  long double parameter = chart.base_parameter;
  long double scale = chart.base_scale;
  Vector previous{};
  for (std::size_t i = 0; i + 1 < count; ++i) {
    const auto chord = Between(chart.positions[i], chart.positions[i + 1]);
    if (!chord) {
      return Result<IntersectionChartParameters>::Err(
          Invalid("Intersection chart has a zero or unrepresentable chord"));
    }
    const auto tangent = *Unit(tangents[i]);
    const long double forward = Dot(tangent, chord->direction);
    const long double backward = i == 0 ? forward : Dot(tangent, previous);
    if (forward <= 0 || backward <= 0) {
      return Result<IntersectionChartParameters>::Err(Invalid(
          "Intersection chart tangent does not follow adjacent chords"));
    }
    if (i != 0) scale *= backward / forward;
    if (!Representable(scale) || scale <= 0 ||
        static_cast<double>(scale) == 0) {
      return Result<IntersectionChartParameters>::Err(
          Invalid("Intersection chart scale is not representable"));
    }
    // Keep the recurrence in wide precision until the published samples are
    // checked. fma also preserves cancellation against a negative base value.
    parameter = std::fma(chord->length, scale, parameter);
    if (!Representable(parameter) ||
        static_cast<double>(parameter) <= result.parameters.back()) {
      return Result<IntersectionChartParameters>::Err(
          Invalid("Intersection chart parameters are not representable and "
                  "increasing"));
    }
    result.chord_scales.push_back(static_cast<double>(scale));
    result.parameters.push_back(static_cast<double>(parameter));
    previous = chord->direction;
  }
  if (Dot(*Unit(tangents.back()), previous) <= 0) {
    return Result<IntersectionChartParameters>::Err(
        Invalid("Intersection chart end tangent does not follow its chord"));
  }
  return Result<IntersectionChartParameters>::Ok(std::move(result));
}
}  // namespace openparasolid
