// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using UV = std::array<double, 2>;
using Vector = std::array<long double, 3>;
using Domain = std::array<double, 4>;
constexpr std::size_t kMaxEvaluations = 512;
constexpr uint64_t kMaxWork = 32000000;
struct Sample {
  SurfaceEvaluation geometry;
  UV parameters;
  long double error;
};
struct Budget {
  std::size_t evaluations = 0;
  uint64_t work = 0;
  uint64_t cost = 0;
};
Result<Sample> Probe(const NurbsSurfaceGeometry& surface, Vector3 point,
                     const Domain& domain, UV uv, Budget& budget) {
  if (budget.evaluations == kMaxEvaluations ||
      budget.cost > kMaxWork - budget.work)
    return Result<Sample>::Err(
        {ErrorCode::kResourceLimit, 0,
         "NURBS inversion evaluation/work budget exceeded"});
  ++budget.evaluations;
  budget.work += budget.cost;
  const UV parameters{std::lerp(domain[0], domain[1], uv[0]),
                      std::lerp(domain[2], domain[3], uv[1])};
  auto value = EvaluateNurbsSurface(surface, parameters[0], parameters[1]);
  if (!value.ok()) return Result<Sample>::Err(value.error());
  const auto& p = value.value().position;
  const long double error = std::hypot(static_cast<long double>(p.x) - point.x,
                                       static_cast<long double>(p.y) - point.y,
                                       static_cast<long double>(p.z) - point.z);
  if (!std::isfinite(error))
    return Result<Sample>::Err({ErrorCode::kInvalidRecord, 0,
                                "NURBS inversion residual is not finite"});
  return Result<Sample>::Ok({value.value(), parameters, error});
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
std::optional<Vector> Unit(Vector value) {
  const long double length = std::hypot(value[0], value[1], value[2]);
  if (!std::isfinite(length) || length == 0) return std::nullopt;
  for (auto& x : value) x /= length;
  return value;
}
std::optional<std::array<long double, 2>> Step(const Sample& sample,
                                               Vector3 point,
                                               const Domain& domain) {
  const auto& du = sample.geometry.u_derivative;
  const auto& dv = sample.geometry.v_derivative;
  const long double width_u = static_cast<long double>(domain[1]) - domain[0];
  const long double width_v = static_cast<long double>(domain[3]) - domain[2];
  const Vector a{du.x * width_u, du.y * width_u, du.z * width_u};
  const Vector b{dv.x * width_v, dv.y * width_v, dv.z * width_v};
  const auto first = Unit(a);
  if (!first) return std::nullopt;
  const long double projection = Dot(*first, b);
  Vector orthogonal{};
  for (std::size_t i = 0; i < 3; ++i)
    orthogonal[i] = b[i] - projection * (*first)[i];
  const auto second = Unit(orthogonal);
  if (!second) return std::nullopt;
  const auto& p = sample.geometry.position;
  const Vector residual{static_cast<long double>(point.x) - p.x,
                        static_cast<long double>(point.y) - p.y,
                        static_cast<long double>(point.z) - p.z};
  // QR least-squares step in normalized knot coordinates. Avoid forming the
  // squared-condition normal equations or assuming equally scaled UV domains.
  const long double v_step =
      Dot(*second, residual) /
      std::hypot(orthogonal[0], orthogonal[1], orthogonal[2]);
  const long double u_step = (Dot(*first, residual) - projection * v_step) /
                             std::hypot(a[0], a[1], a[2]);
  if (!std::isfinite(u_step) || !std::isfinite(v_step)) return std::nullopt;
  return std::array<long double, 2>{u_step, v_step};
}
Result<std::optional<SurfaceParameters>> Search(
    const NurbsSurfaceGeometry& surface, Vector3 point, double tolerance,
    const Domain& domain, UV uv, Budget& budget) {
  auto initial = Probe(surface, point, domain, uv, budget);
  if (!initial.ok()) {
    if (initial.error().code == ErrorCode::kResourceLimit)
      return Result<std::optional<SurfaceParameters>>::Err(initial.error());
    return Result<std::optional<SurfaceParameters>>::Ok(std::nullopt);
  }
  auto sample = initial.value();
  constexpr std::size_t kMaxUpdates = 32;
  constexpr std::size_t kMaxTrials = 12;
  for (std::size_t update = 0; update <= kMaxUpdates; ++update) {
    if (sample.error <= tolerance)
      return Result<std::optional<SurfaceParameters>>::Ok(
          SurfaceParameters{sample.parameters[0], sample.parameters[1],
                            static_cast<double>(sample.error)});
    if (update == kMaxUpdates) break;
    const auto step = Step(sample, point, domain);
    if (!step) break;
    bool accepted = false;
    long double damping = 1;
    for (std::size_t trial = 0; trial < kMaxTrials; ++trial, damping /= 2) {
      UV candidate{};
      for (std::size_t i = 0; i < 2; ++i)
        candidate[i] = static_cast<double>(std::clamp(
            std::fma(damping, (*step)[i], static_cast<long double>(uv[i])), 0.L,
            1.L));
      if (candidate == uv) continue;
      auto next = Probe(surface, point, domain, candidate, budget);
      if (!next.ok()) {
        if (next.error().code == ErrorCode::kResourceLimit)
          return Result<std::optional<SurfaceParameters>>::Err(next.error());
        continue;
      }
      if (next.value().error < sample.error) {
        sample = next.value();
        uv = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) break;
  }
  return Result<std::optional<SurfaceParameters>>::Ok(std::nullopt);
}
double Greville(const SplineAxis& axis, std::size_t control) {
  const uint64_t first = control + 1, last = first + axis.degree;
  uint64_t begin = 0;
  long double value = 0;
  for (std::size_t i = 0; i < axis.knots.size() && begin < last; ++i) {
    const uint64_t end = begin + axis.multiplicities[i];
    const uint64_t low = std::max(begin, first), high = std::min(end, last);
    if (low < high)
      value += static_cast<long double>(axis.knots[i]) *
               (static_cast<long double>(high - low) / axis.degree);
    begin = end;
  }
  return static_cast<double>(value);
}
struct Starts {
  // One optional hint, four nearest-control seeds and a 3x3 domain grid.
  std::array<UV, 14> values{};
  std::size_t count = 0;
  void Add(UV uv) {
    if (!std::isfinite(uv[0]) || !std::isfinite(uv[1])) return;
    for (std::size_t i = 0; i < count; ++i)
      if (values[i] == uv) return;
    values[count++] = uv;
  }
};
UV Normalize(SurfaceParameterSeed seed, const Domain& domain) {
  return {
      static_cast<double>((static_cast<long double>(seed.u) - domain[0]) /
                          (static_cast<long double>(domain[1]) - domain[0])),
      static_cast<double>((static_cast<long double>(seed.v) - domain[2]) /
                          (static_cast<long double>(domain[3]) - domain[2]))};
}
void ControlStarts(const NurbsSurfaceGeometry& surface, Vector3 point,
                   const Domain& domain, Starts& starts) {
  struct Nearest {
    long double distance = std::numeric_limits<long double>::infinity();
    std::size_t index = 0;
  };
  std::array<Nearest, 4> nearest{};
  const auto& net = surface.controls;
  for (std::size_t i = 0; i < net.values.size() / net.dimension; ++i) {
    const auto at = i * net.dimension;
    const long double weight = net.rational ? net.values[at + 3] : 1;
    const long double distance =
        std::hypot(net.values[at] / weight - point.x,
                   net.values[at + 1] / weight - point.y,
                   net.values[at + 2] / weight - point.z);
    if (distance < nearest.back().distance) {
      nearest.back() = {distance, i};
      std::sort(
          nearest.begin(), nearest.end(),
          [](const auto& a, const auto& b) { return a.distance < b.distance; });
    }
  }
  for (const auto& candidate : nearest) {
    if (!std::isfinite(candidate.distance)) continue;
    starts.Add(Normalize(
        {std::clamp(
             Greville(surface.u, candidate.index / surface.v.control_count),
             domain[0], domain[1]),
         std::clamp(
             Greville(surface.v, candidate.index % surface.v.control_count),
             domain[2], domain[3])},
        domain));
  }
}
}  // namespace

Result<SurfaceParameters> InvertNurbsSurface(
    const NurbsSurfaceGeometry& surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed) {
  const auto bounds = NurbsSurfaceDomain(surface);
  if (!bounds.ok()) return Result<SurfaceParameters>::Err(bounds.error());
  const auto& domain = bounds.value();
  Starts starts;
  if (seed) {
    if (seed->u < domain[0] || seed->u > domain[1] || seed->v < domain[2] ||
        seed->v > domain[3])
      return Result<SurfaceParameters>::Err(
          {ErrorCode::kInvalidOptions, 0,
           "NURBS inversion seed is outside its knot domain"});
    starts.Add(Normalize(*seed, domain));
  }
  ControlStarts(surface, point, domain, starts);
  for (double u : {0., .5, 1.})
    for (double v : {0., .5, 1.}) starts.Add({u, v});
  const uint64_t u_degree = surface.u.degree + 1,
                 v_degree = surface.v.degree + 1;
  Budget budget{0, 0,
                surface.controls.values.size() + surface.u.knots.size() * 2 +
                    surface.v.knots.size() * 2 +
                    u_degree * v_degree * v_degree + u_degree * u_degree};
  for (std::size_t i = 0; i < starts.count; ++i) {
    auto result =
        Search(surface, point, tolerance, domain, starts.values[i], budget);
    if (!result.ok()) return Result<SurfaceParameters>::Err(result.error());
    if (result.value()) return Result<SurfaceParameters>::Ok(*result.value());
  }
  return Result<SurfaceParameters>::Err(
      {ErrorCode::kInvalidRecord, 0,
       "NURBS inversion found no point within tolerance using bounded local "
       "searches"});
}
}  // namespace openparasolid::internal
