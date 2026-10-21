// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/evaluate.h"

#include <cmath>

#include "evaluate_internal.h"

namespace openparasolid {
namespace {
constexpr double kFrameTolerance = 1e-10;

bool Finite(const Vector3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool Unit(const Vector3& v) {
  return Finite(v) &&
         std::abs(std::hypot(v.x, v.y, v.z) - 1) <= kFrameTolerance;
}

bool Frame(const Vector3& normal, const Vector3& x_axis) {
  return Unit(normal) && Unit(x_axis) &&
         std::abs(normal.x * x_axis.x + normal.y * x_axis.y +
                  normal.z * x_axis.z) <= kFrameTolerance;
}

Result<CurveEvaluation> Invalid(const char* message) {
  return Result<CurveEvaluation>::Err({ErrorCode::kInvalidRecord, 0, message});
}

Result<CurveEvaluation> Checked(CurveEvaluation result) {
  if (!Finite(result.position) || !Finite(result.first_derivative)) {
    return Invalid("Curve evaluation produced a nonfinite result");
  }
  return Result<CurveEvaluation>::Ok(result);
}

Result<CurveEvaluation> EvaluateConic(const Vector3& center,
                                      const Vector3& normal,
                                      const Vector3& x_axis, double a, double b,
                                      double parameter) {
  if (!Finite(center) || !Frame(normal, x_axis) || !std::isfinite(a) ||
      !std::isfinite(b) || a <= 0 || b <= 0) {
    return Invalid(
        "Conic requires finite positive radii and an orthonormal frame");
  }
  const Vector3 y_axis{normal.y * x_axis.z - normal.z * x_axis.y,
                       normal.z * x_axis.x - normal.x * x_axis.z,
                       normal.x * x_axis.y - normal.y * x_axis.x};
  const double cosine = std::cos(parameter);
  const double sine = std::sin(parameter);
  const auto component = [&](double c, double x, double y) {
    return c + a * x * cosine + b * y * sine;
  };
  const auto derivative = [&](double x, double y) {
    return -a * x * sine + b * y * cosine;
  };
  return Checked(
      {{component(center.x, x_axis.x, y_axis.x),
        component(center.y, x_axis.y, y_axis.y),
        component(center.z, x_axis.z, y_axis.z)},
       {derivative(x_axis.x, y_axis.x), derivative(x_axis.y, y_axis.y),
        derivative(x_axis.z, y_axis.z)}});
}
}  // namespace

Result<CurveEvaluation> EvaluateCurve(const GeometryPayload& geometry,
                                      double parameter) {
  if (!std::isfinite(parameter)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0, "Curve parameter must be finite"});
  }
  if (const auto* line = std::get_if<LineGeometry>(&geometry)) {
    if (!Finite(line->origin) || !Unit(line->direction)) {
      return Invalid("Line requires a finite origin and unit direction");
    }
    return Checked({{std::fma(parameter, line->direction.x, line->origin.x),
                     std::fma(parameter, line->direction.y, line->origin.y),
                     std::fma(parameter, line->direction.z, line->origin.z)},
                    line->direction});
  }
  if (const auto* circle = std::get_if<CircleGeometry>(&geometry)) {
    return EvaluateConic(circle->center, circle->normal, circle->x_axis,
                         circle->radius, circle->radius, parameter);
  }
  if (const auto* ellipse = std::get_if<EllipseGeometry>(&geometry)) {
    return EvaluateConic(ellipse->center, ellipse->normal, ellipse->x_axis,
                         ellipse->major_radius, ellipse->minor_radius,
                         parameter);
  }
  if (const auto* curve = std::get_if<NurbsCurveGeometry>(&geometry)) {
    return internal::EvaluateNurbsCurve(*curve, parameter);
  }
  if (std::holds_alternative<SurfaceCurveGeometry>(geometry)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Surface curves require model context"});
  }
  if (std::holds_alternative<TrimmedCurveGeometry>(geometry)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Trimmed curves require model context"});
  }
  if (std::holds_alternative<IntersectionCurveGeometry>(geometry)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Intersection solver requires explicit options via EvaluateCurve or "
         "EvaluateIntersectionCurve"});
  }
  return Result<CurveEvaluation>::Err(
      {ErrorCode::kUnsupportedFormat, 0,
       "Curve evaluation is not implemented for this geometry payload"});
}
}  // namespace openparasolid
