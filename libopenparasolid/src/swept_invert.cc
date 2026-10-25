// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using Vector = std::array<long double, 3>;
ParseError Error(const char* message,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, 0, message};
}
bool Finite(Vector3 p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
Vector Components(Vector3 p) { return {p.x, p.y, p.z}; }
Vector Difference(Vector3 a, Vector3 b) {
  return {static_cast<long double>(a.x) - b.x,
          static_cast<long double>(a.y) - b.y,
          static_cast<long double>(a.z) - b.z};
}
Vector Cross(const Vector& a, const Vector& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
bool Representable(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}
struct Parameters {
  long double u;
  long double v;
};
Result<Parameters> InvertLine(const LineGeometry& line, Vector3 sweep,
                              Vector3 point) {
  const auto x = Components(line.direction), y = Components(sweep);
  const auto normal = Cross(x, y);
  const long double determinant = Dot(normal, normal);
  if (!std::isfinite(determinant) || determinant == 0)
    return Result<Parameters>::Err(Error("Singular swept line frame"));
  const auto q = Difference(point, line.origin);
  return Result<Parameters>::Ok({Dot(q, Cross(y, normal)) / determinant,
                                 Dot(q, Cross(normal, x)) / determinant});
}
Result<Parameters> InvertConic(Vector3 center, Vector3 normal, Vector3 x_axis,
                               double major, double minor, Vector3 sweep,
                               Vector3 point) {
  // Compute y with the forward evaluator's double-precision operations.
  // Invert its actual basis; accepted frame residuals are not normalized away.
  const Vector3 y_axis{normal.y * x_axis.z - normal.z * x_axis.y,
                       normal.z * x_axis.x - normal.x * x_axis.z,
                       normal.x * x_axis.y - normal.y * x_axis.x};
  const auto x = Components(x_axis), y = Components(y_axis),
             z = Components(sweep);
  const auto yz = Cross(y, z);
  const long double determinant = Dot(x, yz);
  if (!std::isfinite(determinant) || determinant == 0)
    return Result<Parameters>::Err(
        Error("Swept conic requires a transverse sweep direction",
              ErrorCode::kUnsupportedFormat));
  const auto q = Difference(point, center);
  const long double cosine = (Dot(q, yz) / determinant) / major;
  const long double sine = (Dot(q, Cross(z, x)) / determinant) / minor;
  if (!std::isfinite(cosine) || !std::isfinite(sine))
    return Result<Parameters>::Err(Error("Nonfinite swept conic coordinates"));
  return Result<Parameters>::Ok(
      {std::atan2(sine, cosine), Dot(q, Cross(x, y)) / determinant});
}
// A strictly increasing projection of the control polygon proves that the
// nonrational spline has exactly one projected preimage on its knot domain.
// No profile-plane assumption is needed; the final 3D residual tests
// membership.
Result<Parameters> InvertSpline(const NurbsCurveGeometry& curve, Vector3 sweep,
                                Vector3 point) {
  const auto domain = NurbsCurveDomain(curve);
  if (!domain.ok()) return Result<Parameters>::Err(domain.error());
  const auto& axis = curve.axis;
  if (curve.controls.rational || curve.controls.dimension != 3 || axis.closed ||
      axis.periodic || axis.multiplicities.front() != axis.degree + 1 ||
      axis.multiplicities.back() != axis.degree + 1)
    return Result<Parameters>::Err(
        Error("Swept spline inversion requires an open nonrational clamped 3D "
              "section",
              ErrorCode::kUnsupportedFormat));
  constexpr uint64_t kMaxWork = 32000000;
  constexpr int kMaxSamples = 128;
  const uint64_t work = curve.controls.values.size() + axis.knots.size() +
                        uint64_t{axis.degree + 1} * (axis.degree + 1);
  if (work > kMaxWork / (kMaxSamples + 3))
    return Result<Parameters>::Err(
        Error("Swept spline inversion exceeds work limit",
              ErrorCode::kResourceLimit));
  const auto control = [&](std::size_t i) -> Vector3 {
    const auto& values = curve.controls.values;
    return {values[3 * i], values[3 * i + 1], values[3 * i + 2]};
  };
  const auto origin = control(0);
  const auto direction = Components(sweep);
  const auto chord = Difference(control(axis.control_count - 1), origin);
  auto projection = Cross(direction, Cross(chord, direction));
  const long double length =
      std::hypot(projection[0], projection[1], projection[2]);
  const long double chord_margin =
      64 * std::numeric_limits<long double>::epsilon() *
      std::hypot(chord[0], chord[1], chord[2]) * Dot(direction, direction);
  if (!std::isfinite(length) || !(length > chord_margin))
    return Result<Parameters>::Err(
        Error("Swept spline requires a transverse monotone projection",
              ErrorCode::kUnsupportedFormat));
  for (auto& coordinate : projection) coordinate /= length;
  for (std::size_t i = 1; i < axis.control_count; ++i) {
    const auto step = Difference(control(i), control(i - 1));
    // Require a numerical margin as well as a positive projected derivative.
    // Nearly parallel/flat control steps cannot certify injectivity.
    const long double margin = 64 *
                               std::numeric_limits<long double>::epsilon() *
                               std::hypot(step[0], step[1], step[2]);
    if (!(Dot(step, projection) > margin))
      return Result<Parameters>::Err(
          Error("Swept spline requires a transverse monotone projection",
                ErrorCode::kUnsupportedFormat));
  }
  const long double target = Dot(Difference(point, origin), projection);
  double low = domain.value()[0], high = domain.value()[1];
  double best_u = low;
  long double best_error = std::numeric_limits<long double>::infinity();
  Vector3 best_position{};
  // Include both boundaries, then bisect without changing source knot units.
  // A fixed budget also bounds deliberately ill-conditioned parameter ranges.
  for (int i = 0; i < kMaxSamples + 2; ++i) {
    const double u = i == 0 ? low : i == 1 ? high : std::midpoint(low, high);
    const auto sample = EvaluateNurbsCurve(curve, u);
    if (!sample.ok()) return Result<Parameters>::Err(sample.error());
    const long double value =
        Dot(Difference(sample.value().position, origin), projection);
    const long double error = std::abs(value - target);
    if (error < best_error) {
      best_error = error;
      best_u = u;
      best_position = sample.value().position;
    }
    if (i == 0) continue;
    if (i == 1) {
      if (target <= 0 || target >= value) break;
      continue;
    }
    if (value == target || u == low || u == high) break;
    if (value < target)
      low = u;
    else
      high = u;
    if (i == kMaxSamples + 1)
      return Result<Parameters>::Err(
          Error("Swept spline inversion exhausted its search budget",
                ErrorCode::kResourceLimit));
  }
  const long double v = Dot(Difference(point, best_position), direction) /
                        Dot(direction, direction);
  return Result<Parameters>::Ok({best_u, v});
}
}  // namespace
Result<SurfaceParameters> InvertSweptSurface(
    const Model& model, NodeId id, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed) {
  if (!Finite(point) || !std::isfinite(tolerance) || tolerance < 0 ||
      (seed && (!std::isfinite(seed->u) || !std::isfinite(seed->v))))
    return Result<SurfaceParameters>::Err(
        Error("Invalid swept inversion point, tolerance or seed",
              ErrorCode::kInvalidOptions));
  const auto* node = model.FindNode(id);
  const auto* swept = node && node->geometry
                          ? std::get_if<SweptSurfaceGeometry>(
                                &model.geometry()[*node->geometry].payload)
                          : nullptr;
  if (!swept)
    return Result<SurfaceParameters>::Err(Error("Missing swept surface"));
  const auto* section = model.FindNode(swept->section);
  if (!section || (section->node_type != 30 && section->node_type != 31 &&
                   section->node_type != 32 && section->node_type != 134))
    return Result<SurfaceParameters>::Err(
        Error("Invalid swept section target"));
  if (!section->geometry)
    return Result<SurfaceParameters>::Err(
        Error("Swept inversion requires a typed analytic section",
              ErrorCode::kUnsupportedFormat));
  const auto& geometry = model.geometry()[*section->geometry].payload;
  double initial_u = 0;
  if (const auto* spline = std::get_if<NurbsCurveGeometry>(&geometry)) {
    const auto domain = NurbsCurveDomain(*spline);
    if (!domain.ok()) return Result<SurfaceParameters>::Err(domain.error());
    initial_u = domain.value()[0];
  }
  // Validate the stored sweep using the same forward evaluation as membership.
  const auto valid = EvaluateSurface(model, id, initial_u, 0);
  if (!valid.ok()) return Result<SurfaceParameters>::Err(valid.error());
  const auto parameters = [&]() -> Result<Parameters> {
    if (const auto* line = std::get_if<LineGeometry>(&geometry))
      return InvertLine(*line, swept->sweep, point);
    if (const auto* circle = std::get_if<CircleGeometry>(&geometry))
      return InvertConic(circle->center, circle->normal, circle->x_axis,
                         circle->radius, circle->radius, swept->sweep, point);
    if (const auto* ellipse = std::get_if<EllipseGeometry>(&geometry))
      return InvertConic(ellipse->center, ellipse->normal, ellipse->x_axis,
                         ellipse->major_radius, ellipse->minor_radius,
                         swept->sweep, point);
    if (const auto* spline = std::get_if<NurbsCurveGeometry>(&geometry))
      return InvertSpline(*spline, swept->sweep, point);
    return Result<Parameters>::Err(Error("Invalid swept section payload"));
  }();
  if (!parameters.ok())
    return Result<SurfaceParameters>::Err(parameters.error());
  if (!Representable(parameters.value().u) ||
      !Representable(parameters.value().v))
    return Result<SurfaceParameters>::Err(
        Error("Swept inverse parameters are not representable"));
  const double u = static_cast<double>(parameters.value().u);
  const double v = static_cast<double>(parameters.value().v);
  const auto sample = EvaluateSurface(model, id, u, v);
  if (!sample.ok()) return Result<SurfaceParameters>::Err(sample.error());
  const auto residual = Difference(sample.value().position, point);
  const long double distance =
      std::hypot(residual[0], residual[1], residual[2]);
  if (!std::isfinite(distance) || distance > tolerance)
    return Result<SurfaceParameters>::Err(
        Error("Swept surface point does not match within tolerance"));
  return Result<SurfaceParameters>::Ok({u, v, static_cast<double>(distance)});
}
}  // namespace openparasolid::internal
