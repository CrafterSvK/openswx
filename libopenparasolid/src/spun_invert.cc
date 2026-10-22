// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <utility>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using Vector = std::array<long double, 3>;
ParseError Error(const char* message,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, 0, message};
}
Vector Components(Vector3 p) { return {p.x, p.y, p.z}; }
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector Cross(const Vector& a, const Vector& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Length(const Vector& a) { return std::hypot(a[0], a[1], a[2]); }
bool Representable(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}
bool Finite(Vector3 point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}
double Clamp(const SpunSurfaceGeometry& spun, double u) {
  if (spun.start_parameter) u = std::max(u, *spun.start_parameter);
  if (spun.end_parameter) u = std::min(u, *spun.end_parameter);
  return u;
}
Result<SurfaceParameterSeed> Start(const SpunSurfaceGeometry& spun,
                                   const CircleGeometry& circle,
                                   Vector3 point) {
  auto axis = Components(spun.axis);
  const long double axis_length = Length(axis);
  // Normalization is only for the initial guess. Refinement uses the actual
  // source evaluator, including accepted non-unit/orthogonal frame residuals.
  for (auto& value : axis) value /= axis_length;
  const Vector center{static_cast<long double>(circle.center.x) - spun.base.x,
                      static_cast<long double>(circle.center.y) - spun.base.y,
                      static_cast<long double>(circle.center.z) - spun.base.z};
  Vector radial = center;
  const long double height = Dot(center, axis);
  for (std::size_t i = 0; i < 3; ++i) radial[i] -= height * axis[i];
  const long double major = Length(radial);
  if (!std::isfinite(major) || major <= circle.radius)
    return Result<SurfaceParameterSeed>::Err(
        Error("Spun inversion requires a regular circular ring",
              ErrorCode::kUnsupportedFormat));
  for (auto& value : radial) value /= major;
  const auto normal = Components(circle.normal);
  constexpr long double kMeridianTolerance = 1e-10L;
  if (std::abs(Dot(normal, axis)) > kMeridianTolerance ||
      std::abs(Dot(normal, radial)) > kMeridianTolerance)
    return Result<SurfaceParameterSeed>::Err(
        Error("Spun circle must lie in a meridian plane",
              ErrorCode::kUnsupportedFormat));
  const auto side = Cross(axis, radial);
  Vector relative{static_cast<long double>(point.x) - circle.center.x,
                  static_cast<long double>(point.y) - circle.center.y,
                  static_cast<long double>(point.z) - circle.center.z};
  // Measure from the circle centre's projection onto the spin axis.
  for (std::size_t i = 0; i < 3; ++i) relative[i] += major * radial[i];
  const long double axial = Dot(relative, axis);
  const long double x = Dot(relative, radial), y = Dot(relative, side);
  const long double radius = std::hypot(x, y);
  Vector meridian{};
  for (std::size_t i = 0; i < 3; ++i)
    meridian[i] = (radius - major) * radial[i] + axial * axis[i];
  const auto profile_x = Components(circle.x_axis);
  const auto profile_y = Cross(normal, profile_x);
  const long double cosine =
      Dot(meridian, profile_x) / Dot(profile_x, profile_x);
  const long double sine = Dot(meridian, profile_y) / Dot(profile_y, profile_y);
  long double u = std::atan2(sine, cosine);
  const long double v = radius == 0 ? 0 : std::atan2(y, x);
  constexpr long double kTurn = 2 * std::numbers::pi_v<long double>;
  long double target = 0;
  if (spun.start_parameter && spun.end_parameter)
    target = std::midpoint(static_cast<long double>(*spun.start_parameter),
                           static_cast<long double>(*spun.end_parameter));
  else if (spun.start_parameter)
    target = static_cast<long double>(*spun.start_parameter) + kTurn / 2;
  else if (spun.end_parameter)
    target = static_cast<long double>(*spun.end_parameter) - kTurn / 2;
  u += std::round((target - u) / kTurn) * kTurn;
  if (!Representable(u) || !Representable(v))
    return Result<SurfaceParameterSeed>::Err(
        Error("Spun inverse parameters are not representable"));
  return Result<SurfaceParameterSeed>::Ok(
      {Clamp(spun, static_cast<double>(u)), static_cast<double>(v)});
}
struct Sample {
  SurfaceEvaluation geometry;
  SurfaceParameterSeed uv;
  long double residual;
};
Result<Sample> Probe(const Model& model, NodeId id, Vector3 point,
                     SurfaceParameterSeed uv, uint32_t& remaining) {
  if (!remaining)
    return Result<Sample>::Err(
        Error("Spun inversion evaluation limit", ErrorCode::kResourceLimit));
  --remaining;
  auto value = EvaluateSurface(model, id, uv.u, uv.v);
  if (!value.ok()) return Result<Sample>::Err(value.error());
  const auto& p = value.value().position;
  const long double residual =
      std::hypot(static_cast<long double>(p.x) - point.x,
                 static_cast<long double>(p.y) - point.y,
                 static_cast<long double>(p.z) - point.z);
  if (!std::isfinite(residual))
    return Result<Sample>::Err(Error("Nonfinite spun inversion residual"));
  return Result<Sample>::Ok({value.value(), uv, residual});
}
std::optional<std::array<long double, 2>> Step(const Sample& sample,
                                               Vector3 point) {
  auto a = Components(sample.geometry.u_derivative);
  const auto b = Components(sample.geometry.v_derivative);
  const long double length = Length(a);
  if (!std::isfinite(length) || length == 0) return std::nullopt;
  for (auto& value : a) value /= length;
  const long double projection = Dot(a, b);
  Vector orthogonal{};
  for (std::size_t i = 0; i < 3; ++i) orthogonal[i] = b[i] - projection * a[i];
  const long double perpendicular = Length(orthogonal);
  if (!std::isfinite(perpendicular) || perpendicular == 0) return std::nullopt;
  for (auto& value : orthogonal) value /= perpendicular;
  const auto& p = sample.geometry.position;
  const Vector residual{static_cast<long double>(point.x) - p.x,
                        static_cast<long double>(point.y) - p.y,
                        static_cast<long double>(point.z) - p.z};
  const long double dv = Dot(orthogonal, residual) / perpendicular;
  const long double du = (Dot(a, residual) - projection * dv) / length;
  if (!std::isfinite(du) || !std::isfinite(dv)) return std::nullopt;
  return std::array<long double, 2>{du, dv};
}
Result<SurfaceParameters> Refine(const Model& model, NodeId id,
                                 const SpunSurfaceGeometry& spun, Vector3 point,
                                 double tolerance, SurfaceParameterSeed uv,
                                 uint32_t& remaining) {
  uv.u = Clamp(spun, uv.u);
  auto sample = Probe(model, id, point, uv, remaining);
  if (!sample.ok()) return Result<SurfaceParameters>::Err(sample.error());
  constexpr int kMaxUpdates = 24, kMaxDampingTrials = 8;
  for (int update = 0; update < kMaxUpdates; ++update) {
    if (sample.value().residual <= tolerance)
      return Result<SurfaceParameters>::Ok(
          {sample.value().uv.u, sample.value().uv.v,
           static_cast<double>(sample.value().residual)});
    const auto step = Step(sample.value(), point);
    if (!step)
      return Result<SurfaceParameters>::Err(
          Error("Singular spun inverse step"));
    bool improved = false;
    for (int trial = 0; trial < kMaxDampingTrials; ++trial) {
      const long double factor = std::ldexp(1.L, -trial);
      const long double u = sample.value().uv.u + factor * (*step)[0];
      const long double v = sample.value().uv.v + factor * (*step)[1];
      if (!Representable(u) || !Representable(v)) continue;
      const SurfaceParameterSeed candidate{Clamp(spun, static_cast<double>(u)),
                                           static_cast<double>(v)};
      if (candidate.u == sample.value().uv.u &&
          candidate.v == sample.value().uv.v)
        continue;
      auto next = Probe(model, id, point, candidate, remaining);
      if (!next.ok()) return Result<SurfaceParameters>::Err(next.error());
      if (next.value().residual < sample.value().residual) {
        sample = std::move(next);
        improved = true;
        break;
      }
    }
    if (!improved) break;
  }
  if (sample.value().residual <= tolerance)
    return Result<SurfaceParameters>::Ok(
        {sample.value().uv.u, sample.value().uv.v,
         static_cast<double>(sample.value().residual)});
  return Result<SurfaceParameters>::Err(
      Error("Spun surface point does not match within tolerance"));
}
}  // namespace
Result<SurfaceParameters> InvertSpunSurface(
    const Model& model, NodeId id, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed) {
  if (!Finite(point) || !std::isfinite(tolerance) || tolerance < 0 ||
      (seed && (!std::isfinite(seed->u) || !std::isfinite(seed->v))))
    return Result<SurfaceParameters>::Err(
        Error("Invalid spun inversion point, tolerance or seed",
              ErrorCode::kInvalidOptions));
  const auto* node = model.FindNode(id);
  const auto* spun = node && node->geometry
                         ? std::get_if<SpunSurfaceGeometry>(
                               &model.geometry()[*node->geometry].payload)
                         : nullptr;
  if (!spun)
    return Result<SurfaceParameters>::Err(Error("Missing spun surface"));
  const auto* profile = model.FindNode(spun->profile);
  if (!profile)
    return Result<SurfaceParameters>::Err(Error("Missing spun profile"));
  if (profile->node_type != 30 && profile->node_type != 31 &&
      profile->node_type != 32 && profile->node_type != 134)
    return Result<SurfaceParameters>::Err(Error("Invalid spun profile type"));
  if (profile->node_type != 31)
    return Result<SurfaceParameters>::Err(
        Error("Spun inversion requires a circle profile",
              ErrorCode::kUnsupportedFormat));
  if (!profile->geometry)
    return Result<SurfaceParameters>::Err(
        Error("Untyped spun profile", ErrorCode::kUnsupportedFormat));
  const auto& geometry = model.geometry()[*profile->geometry].payload;
  const auto* circle = std::get_if<CircleGeometry>(&geometry);
  if (!circle)
    return Result<SurfaceParameters>::Err(Error("Invalid spun circle payload"));
  const auto circle_sample = EvaluateCurve(geometry, 0);
  if (!circle_sample.ok())
    return Result<SurfaceParameters>::Err(circle_sample.error());
  const auto valid = EvaluateSpunProfile(*spun, circle_sample.value(), 0);
  if (!valid.ok()) return Result<SurfaceParameters>::Err(valid.error());
  const auto initial = Start(*spun, *circle, point);
  if (!initial.ok()) return Result<SurfaceParameters>::Err(initial.error());
  uint32_t remaining = 128;
  if (seed) {
    auto result = Refine(model, id, *spun, point, tolerance, *seed, remaining);
    if (result.ok() || result.error().code != ErrorCode::kInvalidRecord)
      return result;
  }
  return Refine(model, id, *spun, point, tolerance, initial.value(), remaining);
}
}  // namespace openparasolid::internal
