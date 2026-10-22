// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include "evaluate_internal.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using Vector = std::array<long double, 3>;
struct Frame {
  Vector3 origin;
  Vector3 axis;
  Vector3 x;
};
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
bool Finite(Vector3 point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}
Vector Components(Vector3 point) { return {point.x, point.y, point.z}; }
Vector Cross(const Vector& a, const Vector& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
std::optional<Frame> SourceFrame(const GeometryPayload& geometry) {
  if (const auto* p = std::get_if<PlaneGeometry>(&geometry))
    return Frame{p->origin, p->normal, p->x_axis};
  if (const auto* p = std::get_if<CylinderGeometry>(&geometry))
    return Frame{p->origin, p->axis, p->x_axis};
  if (const auto* p = std::get_if<ConeGeometry>(&geometry))
    return Frame{p->origin, p->axis, p->x_axis};
  if (const auto* p = std::get_if<SphereGeometry>(&geometry))
    return Frame{p->center, p->axis, p->x_axis};
  if (const auto* p = std::get_if<TorusGeometry>(&geometry))
    return Frame{p->center, p->axis, p->x_axis};
  return std::nullopt;
}
std::optional<Vector> LocalPoint(const Frame& frame, Vector3 point) {
  if (!Finite(frame.origin) || !Finite(frame.axis) || !Finite(frame.x))
    return std::nullopt;
  const auto x = Components(frame.x);
  const auto z = Components(frame.axis);
  const auto y = Cross(z, x);
  const auto yz = Cross(y, z);
  const long double determinant = Dot(x, yz);
  if (!std::isfinite(determinant) || determinant == 0) return std::nullopt;
  Vector offset{static_cast<long double>(point.x) - frame.origin.x,
                static_cast<long double>(point.y) - frame.origin.y,
                static_cast<long double>(point.z) - frame.origin.z};
  // Invert the actual source basis, rather than assuming perfectly unit and
  // orthogonal axes. The forward evaluator accepts small frame residuals.
  Vector local{Dot(offset, yz) / determinant,
               Dot(offset, Cross(z, x)) / determinant,
               Dot(offset, Cross(x, y)) / determinant};
  for (long double value : local)
    if (!std::isfinite(value)) return std::nullopt;
  return local;
}
bool Representable(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}
// The mathematical apex need not have an exactly representable v. Choose the
// nearest representable parameter on the valid side; the caller still checks
// the resulting 3D point against its original absolute tolerance.
std::optional<double> ConeApexParameter(const ConeGeometry& cone) {
  const long double slope =
      static_cast<long double>(cone.sin_half_angle) / cone.cos_half_angle;
  const long double apex = -static_cast<long double>(cone.radius) / slope;
  if (!Representable(apex)) return std::nullopt;
  double v = static_cast<double>(apex);
  constexpr int kMaxRoundingSteps = 3;
  for (int step = 0; step < kMaxRoundingSteps; ++step) {
    if (std::fma(static_cast<long double>(v), slope,
                 static_cast<long double>(cone.radius)) >= 0)
      return v;
    v = std::nextafter(v, std::copysign(std::numeric_limits<double>::infinity(),
                                        cone.sin_half_angle));
    if (!std::isfinite(v)) return std::nullopt;
  }
  return std::nullopt;
}
}  // namespace

Result<SurfaceParameters> InvertSurface(const GeometryPayload& geometry,
                                        Vector3 point, double tolerance) {
  if (!Finite(point) || !std::isfinite(tolerance) || tolerance < 0) {
    return Result<SurfaceParameters>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Surface inversion requires a finite point and nonnegative finite "
         "tolerance"});
  }
  if (const auto* nurbs = std::get_if<NurbsSurfaceGeometry>(&geometry))
    return internal::InvertNurbsSurface(*nurbs, point, tolerance, std::nullopt);
  const auto frame = SourceFrame(geometry);
  if (!frame) {
    return Result<SurfaceParameters>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Surface inversion supports only analytic and NURBS surfaces"});
  }
  const auto local = LocalPoint(*frame, point);
  if (!local) {
    return Result<SurfaceParameters>::Err(Invalid(
        "Surface inversion has an invalid or unrepresentable source frame"));
  }
  const auto& q = *local;
  const long double radial = std::hypot(q[0], q[1]);
  long double u = radial == 0 ? 0 : std::atan2(q[1], q[0]);
  long double v = q[2];
  if (std::holds_alternative<PlaneGeometry>(geometry)) {
    u = q[0];
    v = q[1];
  } else if (std::holds_alternative<SphereGeometry>(geometry)) {
    v = std::atan2(q[2], radial);
  } else if (const auto* torus = std::get_if<TorusGeometry>(&geometry)) {
    v = std::atan2(q[2], radial - torus->major_radius);
    if (std::isfinite(torus->major_radius) &&
        std::isfinite(torus->minor_radius) && torus->minor_radius > 0 &&
        std::abs(torus->major_radius) <= torus->minor_radius) {
      const double bound =
          std::acos(-torus->major_radius / torus->minor_radius);
      constexpr double kAngularRounding =
          8 * std::numeric_limits<double>::epsilon();
      if (std::abs(v) > bound &&
          std::abs(v) - bound <= kAngularRounding * std::max(1., bound))
        v = std::copysign(static_cast<long double>(bound), v);
    }
  }
  if (!Representable(u) || !Representable(v)) {
    return Result<SurfaceParameters>::Err(
        Invalid("Surface inversion parameters are not representable"));
  }
  const double result_u = static_cast<double>(u);
  double result_v = static_cast<double>(v);
  auto sample = EvaluateSurface(geometry, result_u, result_v);
  if (!sample.ok() && sample.error().code == ErrorCode::kInvalidOptions) {
    if (const auto* cone = std::get_if<ConeGeometry>(&geometry)) {
      const auto apex = ConeApexParameter(*cone);
      if (apex) {
        result_v = *apex;
        sample = EvaluateSurface(geometry, result_u, result_v);
      }
    }
  }
  if (!sample.ok()) {
    if (sample.error().code == ErrorCode::kInvalidOptions)
      return Result<SurfaceParameters>::Err(Invalid(
          "Surface inversion candidate lies outside the natural domain"));
    return Result<SurfaceParameters>::Err(sample.error());
  }
  const auto& p = sample.value().position;
  const long double residual =
      std::hypot(static_cast<long double>(p.x) - point.x,
                 static_cast<long double>(p.y) - point.y,
                 static_cast<long double>(p.z) - point.z);
  if (!Representable(residual) || residual > tolerance) {
    return Result<SurfaceParameters>::Err(
        Invalid("Surface inversion point does not match within tolerance"));
  }
  return Result<SurfaceParameters>::Ok(
      {result_u, result_v, static_cast<double>(residual)});
}
Result<SurfaceParameters> InvertSurface(const GeometryPayload& geometry,
                                        Vector3 point, double tolerance,
                                        SurfaceParameterSeed seed) {
  if (!Finite(point) || !std::isfinite(tolerance) || tolerance < 0 ||
      !std::isfinite(seed.u) || !std::isfinite(seed.v))
    return Result<SurfaceParameters>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Invalid surface inversion request or seed"});
  if (const auto* nurbs = std::get_if<NurbsSurfaceGeometry>(&geometry))
    return internal::InvertNurbsSurface(*nurbs, point, tolerance, seed);
  return InvertSurface(geometry, point, tolerance);
}
}  // namespace openparasolid
