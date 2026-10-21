// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

#include "evaluate_internal.h"

namespace openparasolid {
namespace {
using Vector = std::array<long double, 3>;
using Parameters = std::array<double, 4>;
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
Vector Components(Vector3 p) { return {p.x, p.y, p.z}; }
Vector Difference(Vector3 a, Vector3 b) {
  return {static_cast<long double>(a.x) - b.x,
          static_cast<long double>(a.y) - b.y,
          static_cast<long double>(a.z) - b.z};
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
bool Representable(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}
struct Plane {
  Vector3 point;
  Vector normal;
};
struct State {
  std::array<SurfaceEvaluation, 2> surfaces;
  std::array<long double, 4> residual;
  long double error;
};
Result<State> Sample(const std::array<const GeometryPayload*, 2>& surfaces,
                     const Parameters& uv, const Plane& plane) {
  State state{};
  for (std::size_t side = 0; side < surfaces.size(); ++side) {
    const auto sample =
        EvaluateSurface(*surfaces[side], uv[2 * side], uv[2 * side + 1]);
    if (!sample.ok()) return Result<State>::Err(sample.error());
    state.surfaces[side] = sample.value();
  }
  const auto delta =
      Difference(state.surfaces[0].position, state.surfaces[1].position);
  const long double distance =
      Dot(Difference(state.surfaces[0].position, plane.point), plane.normal);
  state.residual = {delta[0], delta[1], delta[2], distance};
  state.error =
      std::max(std::hypot(delta[0], delta[1], delta[2]), std::abs(distance));
  if (!std::isfinite(state.error))
    return Result<State>::Err(Invalid("Intersection residual is not finite"));
  return Result<State>::Ok(state);
}
std::optional<std::array<long double, 4>> NewtonStep(const State& state,
                                                     const Plane& plane) {
  std::array<Vector, 4> columns{Components(state.surfaces[0].u_derivative),
                                Components(state.surfaces[0].v_derivative),
                                Components(state.surfaces[1].u_derivative),
                                Components(state.surfaces[1].v_derivative)};
  std::array<std::array<long double, 5>, 4> matrix{};
  std::array<long double, 4> scales{};
  for (std::size_t col = 0; col < columns.size(); ++col) {
    for (std::size_t row = 0; row < 3; ++row)
      matrix[row][col] = (col < 2 ? 1 : -1) * columns[col][row];
    matrix[3][col] = col < 2 ? Dot(columns[col], plane.normal) : 0;
    for (std::size_t row = 0; row < 4; ++row)
      scales[col] = std::max(scales[col], std::abs(matrix[row][col]));
    if (!std::isfinite(scales[col]) || scales[col] == 0) return std::nullopt;
    for (std::size_t row = 0; row < 4; ++row) matrix[row][col] /= scales[col];
  }
  for (std::size_t row = 0; row < 4; ++row)
    matrix[row][4] = -state.residual[row];
  // Partial pivoting with independently scaled UV columns. A zero pivot is
  // an unresolved local system, never a reason to fabricate a curve point.
  for (std::size_t col = 0; col < 4; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 4; ++row)
      if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col]))
        pivot = row;
    if (!std::isfinite(matrix[pivot][col]) || matrix[pivot][col] == 0)
      return std::nullopt;
    std::swap(matrix[pivot], matrix[col]);
    for (std::size_t row = col + 1; row < 4; ++row) {
      const long double factor = matrix[row][col] / matrix[col][col];
      for (std::size_t j = col + 1; j < 5; ++j)
        matrix[row][j] -= factor * matrix[col][j];
    }
  }
  std::array<long double, 4> step{};
  for (std::size_t i = 4; i-- > 0;) {
    long double value = matrix[i][4];
    for (std::size_t j = i + 1; j < 4; ++j) value -= matrix[i][j] * step[j];
    step[i] = value / matrix[i][i];
    if (!std::isfinite(step[i])) return std::nullopt;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    step[i] /= scales[i];
    if (!std::isfinite(step[i])) return std::nullopt;
  }
  return step;
}
double Interpolate(double a, double b, double fraction, bool periodic) {
  if (periodic) b = a + std::remainder(b - a, 2 * std::numbers::pi);
  return std::lerp(a, b, fraction);
}
Result<CurveEvaluation> Finish(Vector3 position, Vector3 tangent,
                               const Vector& chord_normal,
                               long double chord_length, long double span) {
  const auto t = Components(tangent);
  const long double projection = Dot(t, chord_normal);
  if (!std::isfinite(projection) || projection <= 0)
    return Result<CurveEvaluation>::Err(
        Invalid("Intersection tangent does not follow the selected chord"));
  const long double factor = chord_length / (span * projection);
  if (!std::isfinite(factor) || factor <= 0)
    return Result<CurveEvaluation>::Err(
        Invalid("Intersection derivative scale is not representable"));
  std::array<double, 3> derivative{};
  for (std::size_t i = 0; i < 3; ++i) {
    const long double value = t[i] * factor;
    if (!Representable(value))
      return Result<CurveEvaluation>::Err(
          Invalid("Intersection derivative is not representable"));
    derivative[i] = static_cast<double>(value);
  }
  return Result<CurveEvaluation>::Ok(
      {position, {derivative[0], derivative[1], derivative[2]}});
}
Result<State> Solve(const std::array<const GeometryPayload*, 2>& surfaces,
                    Parameters& uv, const Plane& plane, double tolerance) {
  auto sampled = Sample(surfaces, uv, plane);
  if (!sampled.ok()) return Result<State>::Err(sampled.error());
  State state = sampled.value();
  constexpr std::size_t kMaxUpdates = 32;
  constexpr std::size_t kMaxDampingTrials = 16;
  for (std::size_t iteration = 0;
       state.error > tolerance && iteration < kMaxUpdates; ++iteration) {
    const auto step = NewtonStep(state, plane);
    if (!step)
      return Result<State>::Err(
          Invalid("Intersection Newton system is singular"));
    bool accepted = false;
    long double damping = 1;
    for (std::size_t trial = 0; trial < kMaxDampingTrials;
         ++trial, damping /= 2) {
      Parameters candidate{};
      bool valid = true;
      for (std::size_t i = 0; i < 4; ++i) {
        const long double value =
            std::fma(damping, (*step)[i], static_cast<long double>(uv[i]));
        valid = valid && Representable(value);
        if (valid) candidate[i] = static_cast<double>(value);
      }
      if (!valid) continue;
      auto next = Sample(surfaces, candidate, plane);
      if (next.ok() && next.value().error < state.error) {
        uv = candidate;
        state = next.value();
        accepted = true;
        break;
      }
    }
    if (!accepted)
      return Result<State>::Err(
          Invalid("Intersection Newton iteration stalled"));
  }
  if (state.error > tolerance)
    return Result<State>::Err(
        Invalid("Intersection Newton iteration did not converge"));
  return Result<State>::Ok(state);
}
}  // namespace

Result<CurveEvaluation> EvaluateIntersectionCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options) {
  if (!std::isfinite(parameter) || !std::isfinite(options.chart_tolerance) ||
      options.chart_tolerance < 0 || !std::isfinite(options.solve_tolerance) ||
      options.solve_tolerance <= 0)
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Intersection evaluation requires a finite parameter and valid "
         "chart/solve tolerances"});
  const auto prepared =
      EvaluateIntersectionChart(model, curve, options.chart_tolerance);
  if (!prepared.ok()) return Result<CurveEvaluation>::Err(prepared.error());
  const auto& parameters = prepared.value().parameterization.parameters;
  if (parameter < parameters.front() || parameter > parameters.back())
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Intersection parameter is outside its chart"});
  const auto& geometry = model.geometry()[*model.FindNode(curve)->geometry];
  const auto& intersection =
      std::get<IntersectionCurveGeometry>(geometry.payload);
  const auto& points = intersection.chart.positions;
  const auto upper =
      std::upper_bound(parameters.begin(), parameters.end(), parameter);
  const std::size_t index =
      upper == parameters.end()
          ? parameters.size() - 2
          : static_cast<std::size_t>(upper - parameters.begin() - 1);
  const auto chord = Difference(points[index + 1], points[index]);
  const long double length = std::hypot(chord[0], chord[1], chord[2]);
  Vector normal = chord;
  for (auto& value : normal) value /= length;
  const long double span =
      static_cast<long double>(parameters[index + 1]) - parameters[index];
  if (!std::isfinite(length) || !std::isfinite(span) || length <= 0 ||
      span <= 0)
    return Result<CurveEvaluation>::Err(
        Invalid("Intersection chord or parameter span is not representable"));
  if (parameter == parameters[index] || parameter == parameters[index + 1]) {
    const auto knot = parameter == parameters[index] ? index : index + 1;
    return Finish(points[knot], prepared.value().tangents[knot], normal, length,
                  span);
  }
  const double fraction = static_cast<double>(
      (static_cast<long double>(parameter) - parameters[index]) / span);
  const Plane plane{{std::lerp(points[index].x, points[index + 1].x, fraction),
                     std::lerp(points[index].y, points[index + 1].y, fraction),
                     std::lerp(points[index].z, points[index + 1].z, fraction)},
                    normal};
  std::array<const GeometryPayload*, 2> surfaces{};
  Parameters uv{};
  for (std::size_t side = 0; side < 2; ++side) {
    const auto* node = model.FindNode(intersection.surfaces[side]);
    surfaces[side] = &model.geometry()[*node->geometry].payload;
    const auto& a = prepared.value().surface_parameters[index][side];
    const auto& b = prepared.value().surface_parameters[index + 1][side];
    uv[2 * side] = Interpolate(a.u, b.u, fraction, node->node_type != 50);
    const auto* torus = std::get_if<TorusGeometry>(surfaces[side]);
    uv[2 * side + 1] = Interpolate(
        a.v, b.v, fraction, torus && torus->major_radius > torus->minor_radius);
  }
  const auto solved = Solve(surfaces, uv, plane, options.solve_tolerance);
  if (!solved.ok()) return Result<CurveEvaluation>::Err(solved.error());
  std::array<Vector3, 2> normals{};
  for (std::size_t side = 0; side < 2; ++side) {
    const auto sampled_normal = EvaluateSurfaceNormal(
        model, intersection.surfaces[side], uv[2 * side], uv[2 * side + 1]);
    if (!sampled_normal.ok())
      return Result<CurveEvaluation>::Err(sampled_normal.error());
    normals[side] = sampled_normal.value();
  }
  const auto tangent =
      internal::UnitCrossProduct(normals[0], normals[1], false);
  if (!tangent.ok()) return Result<CurveEvaluation>::Err(tangent.error());
  return Finish(solved.value().surfaces[0].position, tangent.value(), normal,
                length, span);
}
}  // namespace openparasolid
