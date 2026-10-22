// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <limits>
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
  if (!section->geometry || section->node_type == 134)
    return Result<SurfaceParameters>::Err(
        Error("Swept inversion requires a typed analytic section",
              ErrorCode::kUnsupportedFormat));
  // Validate source geometry and sweep coefficients before solving. This
  // initial sample and the final membership sample bound all evaluation work.
  const auto valid = EvaluateSurface(model, id, 0, 0);
  if (!valid.ok()) return Result<SurfaceParameters>::Err(valid.error());
  const auto& geometry = model.geometry()[*section->geometry].payload;
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
