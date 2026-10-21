// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include "evaluate_internal.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
constexpr double kFrameTolerance = 1e-10;
using Coordinate = std::array<long double, 3>;

bool Finite(Vector3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool Unit(Vector3 v) {
  return Finite(v) &&
         std::abs(std::hypot(v.x, v.y, v.z) - 1) <= kFrameTolerance;
}

bool Frame(Vector3 axis, Vector3 x_axis) {
  return Unit(axis) && Unit(x_axis) &&
         std::abs(axis.x * x_axis.x + axis.y * x_axis.y + axis.z * x_axis.z) <=
             kFrameTolerance;
}

bool Positive(double value) { return std::isfinite(value) && value > 0; }

Result<SurfaceEvaluation> Invalid(const char* message) {
  return Result<SurfaceEvaluation>::Err(
      {ErrorCode::kInvalidRecord, 0, message});
}

Result<SurfaceEvaluation> Outside(const char* message) {
  return Result<SurfaceEvaluation>::Err(
      {ErrorCode::kInvalidOptions, 0, message});
}

Coordinate Components(Vector3 v) { return {v.x, v.y, v.z}; }

Coordinate Cross(Vector3 a, Vector3 b) {
  const auto ac = Components(a);
  const auto bc = Components(b);
  return {ac[1] * bc[2] - ac[2] * bc[1], ac[2] * bc[0] - ac[0] * bc[2],
          ac[0] * bc[1] - ac[1] * bc[0]};
}

bool Representable(const Coordinate& value) {
  return std::all_of(value.begin(), value.end(), [](long double number) {
    return std::isfinite(number) &&
           std::abs(number) <= std::numeric_limits<double>::max();
  });
}

// Called only after all coordinates pass the representability check.
Vector3 ToVector(const Coordinate& value) {
  return {static_cast<double>(value[0]), static_cast<double>(value[1]),
          static_cast<double>(value[2])};
}

Result<SurfaceEvaluation> Checked(const Coordinate& position,
                                  const Coordinate& du, const Coordinate& dv) {
  if (!Representable(position) || !Representable(du) || !Representable(dv)) {
    return Invalid(
        "Surface evaluation produced a nonfinite or out-of-range result");
  }
  return Result<SurfaceEvaluation>::Ok(
      {ToVector(position), ToVector(du), ToVector(dv)});
}

struct Profile {
  long double radius;
  long double height;
  long double radial_derivative;
  long double axial_derivative;
};

Result<SurfaceEvaluation> Revolution(Vector3 origin, Vector3 axis,
                                     Vector3 x_axis, Profile profile,
                                     double u) {
  if (!Finite(origin) || !Frame(axis, x_axis)) {
    return Invalid("Surface requires a finite origin and an orthonormal frame");
  }
  const auto p = Components(origin);
  const auto a = Components(axis);
  const auto x = Components(x_axis);
  const auto y = Cross(axis, x_axis);
  const long double cosine = std::cos(u);
  const long double sine = std::sin(u);
  Coordinate position{}, du{}, dv{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    const long double radial = x[i] * cosine + y[i] * sine;
    const long double tangent = -x[i] * sine + y[i] * cosine;
    position[i] = p[i] + profile.radius * radial + profile.height * a[i];
    du[i] = profile.radius * tangent;
    dv[i] =
        profile.radial_derivative * radial + profile.axial_derivative * a[i];
  }
  return Checked(position, du, dv);
}

Result<SurfaceEvaluation> Plane(const PlaneGeometry& plane, double u,
                                double v) {
  if (!Finite(plane.origin) || !Frame(plane.normal, plane.x_axis)) {
    return Invalid("Plane requires a finite origin and an orthonormal frame");
  }
  const auto p = Components(plane.origin);
  const auto x = Components(plane.x_axis);
  const auto y = Cross(plane.normal, plane.x_axis);
  Coordinate position{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    position[i] = p[i] + static_cast<long double>(u) * x[i] +
                  static_cast<long double>(v) * y[i];
  }
  return Checked(position, x, y);
}

Result<SurfaceEvaluation> Cone(const ConeGeometry& cone, double u, double v) {
  if (!std::isfinite(cone.radius) || cone.radius < 0 ||
      !Positive(cone.sin_half_angle) || !Positive(cone.cos_half_angle) ||
      std::abs(std::hypot(cone.sin_half_angle, cone.cos_half_angle) - 1) >
          kFrameTolerance) {
    return Invalid(
        "Cone requires a nonnegative radius and a positive acute half-angle");
  }
  const long double slope =
      static_cast<long double>(cone.sin_half_angle) / cone.cos_half_angle;
  const long double radius = std::fma(static_cast<long double>(v), slope,
                                      static_cast<long double>(cone.radius));
  if (!std::isfinite(slope) || !std::isfinite(radius)) {
    return Invalid(
        "Cone evaluation produced an unrepresentable intermediate result");
  }
  if (radius < 0) return Outside("Cone parameter lies beyond the apex");
  // XT axes point away from the selected half-cone; PK axes point towards it.
  return Revolution(cone.origin, cone.axis, cone.x_axis,
                    {radius, -static_cast<long double>(v), slope, -1}, u);
}

Result<SurfaceEvaluation> Sphere(const SphereGeometry& sphere, double u,
                                 double v) {
  if (!Positive(sphere.radius))
    return Invalid("Sphere radius must be positive and finite");
  constexpr double kHalfPi = std::numbers::pi / 2;
  if (v < -kHalfPi || v > kHalfPi) {
    return Outside("Sphere latitude must lie in [-pi/2, pi/2]");
  }
  const long double radial =
      static_cast<long double>(sphere.radius) * std::cos(v);
  const long double axial =
      static_cast<long double>(sphere.radius) * std::sin(v);
  return Revolution(sphere.center, sphere.axis, sphere.x_axis,
                    {radial, axial, -axial, radial}, u);
}

Result<SurfaceEvaluation> Torus(const TorusGeometry& torus, double u,
                                double v) {
  const double major = torus.major_radius;
  const double minor = torus.minor_radius;
  if (!std::isfinite(major) || major == 0 || !Positive(minor) ||
      (major < 0 && -major >= minor)) {
    return Invalid(
        "Invalid torus radii, including degenerate lemon or sphere cases");
  }
  // The principal v interval selects the XT apple/lemon sheet. Comparing
  // angles also accepts endpoints rounded by acos without radius clamping.
  if (major <= minor && std::abs(v) > std::acos(-major / minor)) {
    return Outside(
        "Torus parameter lies outside its principal apple/lemon branch");
  }
  const long double radial_offset =
      static_cast<long double>(minor) * std::cos(v);
  const long double axial = static_cast<long double>(minor) * std::sin(v);
  return Revolution(torus.center, torus.axis, torus.x_axis,
                    {major + radial_offset, axial, -axial, radial_offset}, u);
}
}  // namespace

Result<SurfaceEvaluation> EvaluateSurface(const GeometryPayload& geometry,
                                          double u, double v) {
  if (!std::isfinite(u) || !std::isfinite(v)) {
    return Outside("Surface parameters must be finite");
  }
  if (const auto* plane = std::get_if<PlaneGeometry>(&geometry)) {
    return Plane(*plane, u, v);
  }
  if (const auto* cylinder = std::get_if<CylinderGeometry>(&geometry)) {
    if (!Positive(cylinder->radius)) {
      return Invalid("Cylinder radius must be positive and finite");
    }
    return Revolution(cylinder->origin, cylinder->axis, cylinder->x_axis,
                      {cylinder->radius, v, 0, 1}, u);
  }
  if (const auto* cone = std::get_if<ConeGeometry>(&geometry))
    return Cone(*cone, u, v);
  if (const auto* sphere = std::get_if<SphereGeometry>(&geometry))
    return Sphere(*sphere, u, v);
  if (const auto* torus = std::get_if<TorusGeometry>(&geometry))
    return Torus(*torus, u, v);
  if (const auto* surface = std::get_if<NurbsSurfaceGeometry>(&geometry))
    return internal::EvaluateNurbsSurface(*surface, u, v);
  return Result<SurfaceEvaluation>::Err(
      {ErrorCode::kUnsupportedFormat, 0,
       "Surface evaluation is not implemented for this geometry payload"});
}
}  // namespace openparasolid
