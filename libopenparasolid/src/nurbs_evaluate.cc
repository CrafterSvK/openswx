// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
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

std::optional<ParseError> ValidateAxis(const SplineAxis& axis) {
  if (axis.degree > kMaxDegree || axis.knots.size() > kMaxValues ||
      axis.multiplicities.size() > kMaxValues ||
      uint64_t{axis.control_count} + axis.degree + 1 > kMaxValues) {
    return ParseError{ErrorCode::kResourceLimit, 0,
                      "NURBS evaluation exceeds degree or knot limits"};
  }
  if (axis.degree == 0 || axis.degree >= axis.control_count ||
      axis.knots.size() < 2 ||
      axis.knots.size() != axis.multiplicities.size() ||
      (axis.periodic && !axis.closed)) {
    return Invalid("Invalid NURBS degree, knot arrays or closure flags");
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
  return std::nullopt;
}

std::optional<ParseError> ValidateControls(const SplineControlNet& controls,
                                           uint64_t count, bool surface) {
  const uint16_t dimension = controls.dimension;
  if ((surface && dimension != (controls.rational ? 4 : 3)) ||
      (!surface && controls.rational && dimension != 3 && dimension != 4) ||
      (!surface && !controls.rational && dimension != 2 && dimension != 3)) {
    return Invalid("Invalid NURBS control dimension");
  }
  // Check the product bound before multiplication or allocating any scratch
  // storage. A surface has u_count*v_count controls, not u_count+v_count.
  if (controls.values.size() > kMaxValues || count > kMaxValues / dimension) {
    return ParseError{ErrorCode::kResourceLimit, 0,
                      "NURBS evaluation exceeds control value limit"};
  }
  if (controls.values.size() != count * dimension) {
    return Invalid("Invalid NURBS control array size");
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

struct AxisEvaluation {
  std::vector<double> knots;
  std::size_t degree;
  std::size_t span;
  double parameter;
};

// Call only after axis and control data have passed all size/shape checks.
Result<AxisEvaluation> PrepareAxis(const SplineAxis& axis, double parameter) {
  std::vector<double> knots;
  knots.reserve(std::size_t{axis.control_count} + axis.degree + 1);
  for (std::size_t i = 0; i < axis.knots.size(); ++i) {
    knots.insert(knots.end(), axis.multiplicities[i], axis.knots[i]);
  }
  const double start = knots[axis.degree];
  const double end = knots[axis.control_count];
  if (!(start < end)) {
    return Result<AxisEvaluation>::Err(
        Invalid("NURBS parameter domain is empty"));
  }
  if (parameter < start || parameter > end) {
    return Result<AxisEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "NURBS parameter is outside its knot domain"});
  }
  // Right-hand polynomial at interior knots, left-hand polynomial at the end.
  const auto boundary =
      parameter == end
          ? std::lower_bound(knots.begin(), knots.end(), end)
          : std::upper_bound(knots.begin(), knots.end(), parameter);
  const auto span = static_cast<std::size_t>(boundary - knots.begin() - 1);
  return Result<AxisEvaluation>::Ok(
      {std::move(knots), axis.degree, span, parameter});
}

// Homogeneous value and natural partials. Extended precision reduces
// intermediate overflow; projected output must still fit in double.
struct Jet {
  std::array<long double, 4> value{};
  std::array<long double, 4> du{};
  std::array<long double, 4> dv{};
};
enum class ParameterAxis { kU, kV };

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

Result<Jet> Interpolate(const AxisEvaluation& axis, std::span<Jet> work,
                        ParameterAxis direction) {
  // Descending updates preserve the previous de Boor recursion level.
  for (std::size_t level = 1; level <= axis.degree; ++level) {
    for (std::size_t j = axis.degree; j >= level; --j) {
      const std::size_t i = axis.span - axis.degree + j;
      const long double low = axis.knots[i];
      const long double width =
          static_cast<long double>(axis.knots[i + axis.degree + 1 - level]) -
          low;
      if (!std::isfinite(width) || width <= 0) {
        return Result<Jet>::Err(
            Invalid("NURBS knot interval is degenerate or not representable"));
      }
      const long double alpha =
          (static_cast<long double>(axis.parameter) - low) / width;
      const Jet left = work[j - 1];
      const Jet right = work[j];
      for (std::size_t c = 0; c < 4; ++c) {
        work[j].value[c] = std::lerp(left.value[c], right.value[c], alpha);
        work[j].du[c] = std::lerp(left.du[c], right.du[c], alpha);
        work[j].dv[c] = std::lerp(left.dv[c], right.dv[c], alpha);
        auto& derivative =
            direction == ParameterAxis::kU ? work[j].du : work[j].dv;
        derivative[c] += (right.value[c] - left.value[c]) / width;
      }
    }
  }
  return Result<Jet>::Ok(work[axis.degree]);
}

Result<SurfaceEvaluation> Project(const Jet& jet) {
  const long double weight = jet.value[3];
  if (!std::isfinite(weight) || weight <= 0) {
    return Result<SurfaceEvaluation>::Err(
        Invalid("NURBS evaluation produced an invalid homogeneous weight"));
  }
  std::array<double, 3> position{}, du{}, dv{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    const long double point = jet.value[i] / weight;
    const long double u_tangent = (jet.du[i] - point * jet.du[3]) / weight;
    const long double v_tangent = (jet.dv[i] - point * jet.dv[3]) / weight;
    constexpr long double limit = std::numeric_limits<double>::max();
    if (!std::isfinite(point) || !std::isfinite(u_tangent) ||
        !std::isfinite(v_tangent) || std::abs(point) > limit ||
        std::abs(u_tangent) > limit || std::abs(v_tangent) > limit) {
      return Result<SurfaceEvaluation>::Err(Invalid(
          "NURBS evaluation produced a nonfinite or out-of-range result"));
    }
    position[i] = static_cast<double>(point);
    du[i] = static_cast<double>(u_tangent);
    dv[i] = static_cast<double>(v_tangent);
  }
  return Result<SurfaceEvaluation>::Ok({{position[0], position[1], position[2]},
                                        {du[0], du[1], du[2]},
                                        {dv[0], dv[1], dv[2]}});
}
}  // namespace

Result<CurveEvaluation> EvaluateNurbsCurve(const NurbsCurveGeometry& curve,
                                           double parameter) {
  if (auto error = ValidateAxis(curve.axis)) {
    return Result<CurveEvaluation>::Err(std::move(*error));
  }
  if (auto error =
          ValidateControls(curve.controls, curve.axis.control_count, false)) {
    return Result<CurveEvaluation>::Err(std::move(*error));
  }
  const auto axis = PrepareAxis(curve.axis, parameter);
  if (!axis.ok()) return Result<CurveEvaluation>::Err(axis.error());
  std::vector<Jet> work;
  work.reserve(axis.value().degree + 1);
  for (std::size_t j = 0; j <= axis.value().degree; ++j) {
    work.push_back(
        Control(curve.controls, axis.value().span - axis.value().degree + j));
  }
  auto jet = Interpolate(axis.value(), work, ParameterAxis::kU);
  if (!jet.ok()) return Result<CurveEvaluation>::Err(jet.error());
  auto projected = Project(jet.value());
  if (!projected.ok()) return Result<CurveEvaluation>::Err(projected.error());
  return Result<CurveEvaluation>::Ok(
      {projected.value().position, projected.value().u_derivative});
}

Result<std::array<double, 4>> NurbsSurfaceDomain(
    const NurbsSurfaceGeometry& surface) {
  for (const auto* axis : {&surface.u, &surface.v})
    if (auto error = ValidateAxis(*axis))
      return Result<std::array<double, 4>>::Err(std::move(*error));
  if (auto error = ValidateControls(
          surface.controls,
          uint64_t{surface.u.control_count} * surface.v.control_count, true))
    return Result<std::array<double, 4>>::Err(std::move(*error));
  std::array<double, 4> result{};
  std::size_t slot = 0;
  for (const auto* axis : {&surface.u, &surface.v}) {
    for (uint32_t index : {axis->degree, axis->control_count}) {
      uint64_t end = 0;
      for (std::size_t i = 0; i < axis->knots.size(); ++i) {
        end += axis->multiplicities[i];
        if (index < end) {
          result[slot++] = axis->knots[i];
          break;
        }
      }
    }
  }
  if (slot != result.size() || result[0] >= result[1] || result[2] >= result[3])
    return Result<std::array<double, 4>>::Err(
        Invalid("Empty NURBS surface domain"));
  return Result<std::array<double, 4>>::Ok(result);
}

Result<SurfaceEvaluation> EvaluateNurbsSurface(
    const NurbsSurfaceGeometry& surface, double u, double v) {
  for (const auto* axis : {&surface.u, &surface.v}) {
    if (auto error = ValidateAxis(*axis)) {
      return Result<SurfaceEvaluation>::Err(std::move(*error));
    }
  }
  const uint64_t count =
      uint64_t{surface.u.control_count} * surface.v.control_count;
  if (auto error = ValidateControls(surface.controls, count, true)) {
    return Result<SurfaceEvaluation>::Err(std::move(*error));
  }
  const auto u_axis = PrepareAxis(surface.u, u);
  if (!u_axis.ok()) return Result<SurfaceEvaluation>::Err(u_axis.error());
  const auto v_axis = PrepareAxis(surface.v, v);
  if (!v_axis.ok()) return Result<SurfaceEvaluation>::Err(v_axis.error());
  std::vector<Jet> u_work, v_work;
  u_work.reserve(u_axis.value().degree + 1);
  v_work.reserve(v_axis.value().degree + 1);
  for (std::size_t i = 0; i <= u_axis.value().degree; ++i) {
    const std::size_t u_index = u_axis.value().span - u_axis.value().degree + i;
    v_work.clear();
    for (std::size_t j = 0; j <= v_axis.value().degree; ++j) {
      const std::size_t v_index =
          v_axis.value().span - v_axis.value().degree + j;
      // XT stores contiguous v rows: control(u,v) is at u*n_v + v.
      v_work.push_back(Control(surface.controls,
                               u_index * surface.v.control_count + v_index));
    }
    auto row = Interpolate(v_axis.value(), v_work, ParameterAxis::kV);
    if (!row.ok()) return Result<SurfaceEvaluation>::Err(row.error());
    u_work.push_back(std::move(row).value());
  }
  auto jet = Interpolate(u_axis.value(), u_work, ParameterAxis::kU);
  if (!jet.ok()) return Result<SurfaceEvaluation>::Err(jet.error());
  return Project(jet.value());
}
}  // namespace openparasolid::internal
