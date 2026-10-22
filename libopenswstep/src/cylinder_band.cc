// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "cylinder_band.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vec = std::array<long double, 3>;
Vec ToVec(Vector3 p) { return {p.x, p.y, p.z}; }
Vec Sub(Vec a, Vec b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec Add(Vec a, Vec b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec Scale(Vec a, long double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
long double Dot(Vec a, Vec b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec Cross(Vec a, Vec b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
long double Norm(Vec a) { return std::hypot(a[0], a[1], a[2]); }
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
struct BandEdge {
  long double lower, upper;
  long double winding = 0, integral = 0;
  Vec radial{};
  Vector3 reference;
  bool line = false;
};
Result<BandEdge> ReadEdge(const PlanarCurve& curve,
                          const CylinderGeometry& cylinder, double tolerance) {
  using Return = Result<BandEdge>;
  if (!curve.geometry || !std::isfinite(curve.start) ||
      !std::isfinite(curve.end) || curve.start == curve.end)
    return Return::Err(Invalid("Cylinder band edge interval is degenerate"));
  const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
  const auto* ellipse = std::get_if<EllipseGeometry>(curve.geometry);
  const bool line = std::holds_alternative<LineGeometry>(*curve.geometry);
  if (!circle && !ellipse && !line)
    return Return::Err(
        Unsupported("Cylinder band requires LINE/CIRCLE/ELLIPSE bases"));
  const auto first = EvaluateCurve(*curve.geometry, curve.start);
  const auto last = EvaluateCurve(*curve.geometry, curve.end);
  if (!first.ok()) return Return::Err(first.error());
  if (!last.ok()) return Return::Err(last.error());
  const auto axis = ToVec(cylinder.axis), origin = ToVec(cylinder.origin);
  const auto p = ToVec(first.value().position),
             q = ToVec(last.value().position);
  const auto z0 = Dot(Sub(p, origin), axis), z1 = Dot(Sub(q, origin), axis);
  BandEdge result{std::min(z0, z1),
                  std::max(z0, z1),
                  0,
                  0,
                  {},
                  first.value().position,
                  line};
  if (line) {
    const auto delta = Sub(q, p);
    const auto drift = Norm(Sub(delta, Scale(axis, Dot(delta, axis))));
    const auto radial = Sub(Sub(p, origin), Scale(axis, z0));
    if (!std::isfinite(drift) ||
        drift + std::abs(Norm(radial) - cylinder.radius) > tolerance ||
        std::abs(z1 - z0) <= tolerance)
      return Return::Err(
          Invalid("Cylinder band LINE is not a resolved axial generator"));
    return Return::Ok(result);
  }
  const auto center = ToVec(circle ? circle->center : ellipse->center);
  const auto normal = ToVec(circle ? circle->normal : ellipse->normal);
  const auto reference = ToVec(circle ? circle->x_axis : ellipse->x_axis);
  const auto a =
      Scale(reference, circle ? circle->radius : ellipse->major_radius);
  const auto b = Scale(Cross(normal, reference),
                       circle ? circle->radius : ellipse->minor_radius);
  const auto az = Dot(a, axis), bz = Dot(b, axis),
             level = Dot(Sub(center, origin), axis);
  const auto ar = Sub(a, Scale(axis, az)), br = Sub(b, Scale(axis, bz));
  const auto length = Norm(ar), determinant = Dot(Cross(ar, br), axis);
  if (!std::isfinite(length) || length == 0 || !std::isfinite(determinant) ||
      determinant == 0)
    return Return::Err(
        Invalid("Cylinder band conic has a degenerate radial frame"));
  const long double direction = determinant > 0 ? 1 : -1;
  const auto u = Scale(ar, 1 / length);
  const auto cross = Cross(axis, u);
  const auto v = Scale(cross, direction / Norm(cross));
  const auto residual = Norm(Sub(Sub(center, origin), Scale(axis, level))) +
                        Norm(Sub(ar, Scale(u, cylinder.radius))) +
                        Norm(Sub(br, Scale(v, cylinder.radius)));
  if (!std::isfinite(residual) || residual > tolerance)
    return Return::Err(
        Invalid("Supporting conic does not project to the cylinder circle"));
  const long double span = static_cast<long double>(curve.end) - curve.start;
  if (std::abs(span) > 2 * std::numbers::pi ||
      std::abs(span) * cylinder.radius <= tolerance)
    return Return::Err(Unsupported(
        "Cylinder band conic azimuth must be resolved and at most one turn"));
  result.lower = level - MaximumArcProjection(-az, -bz, curve.start, curve.end);
  result.upper = level + MaximumArcProjection(az, bz, curve.start, curve.end);
  result.winding = direction * span;
  const long double start = curve.start, end = curve.end;
  const auto s0 = std::sin(start), c0 = std::cos(start), s1 = std::sin(end),
             c1 = std::cos(end);
  const auto cos_integral = s1 - s0, sin_integral = c0 - c1;
  const auto twice = (s1 * c1 - s0 * c0) / 2;
  const auto cos2 = span / 2 + twice, sin2 = span / 2 - twice;
  const auto product = (s1 * s1 - s0 * s0) / 2;
  result.integral =
      direction * (level * span + az * cos_integral + bz * sin_integral);
  result.radial =
      Scale(Add(Scale(u, level * cos_integral + az * cos2 + bz * product),
                Scale(v, level * sin_integral + az * product + bz * sin2)),
            direction);
  return Return::Ok(result);
}
struct Profile {
  long double lower = std::numeric_limits<long double>::infinity();
  long double upper = -std::numeric_limits<long double>::infinity();
  long double winding = 0, integral = 0;
  Vec radial{};
  Vector3 reference;
};
Result<Profile> ReadProfile(std::span<const PlanarCurve> curves,
                            const CylinderGeometry& cylinder, double tolerance,
                            uint64_t& remaining) {
  using Return = Result<Profile>;
  if (curves.empty())
    return Return::Err(Unsupported("Cylinder band profile has no edges"));
  Profile result;
  bool previous_line = false, first_line = false;
  int direction = 0;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    if (!remaining)
      return Return::Err(
          {ErrorCode::kResourceLimit, 0, "Cylinder band edge limit exceeded"});
    --remaining;
    const auto edge = ReadEdge(curves[i], cylinder, tolerance);
    if (!edge.ok()) return Return::Err(edge.error());
    const auto& value = edge.value();
    if (i == 0) {
      result.reference = value.reference;
      first_line = value.line;
    }
    if (value.line && previous_line)
      return Return::Err(Unsupported(
          "Cylinder band profile has consecutive axial generators"));
    previous_line = value.line;
    if (!value.line) {
      const int sign = value.winding > 0 ? 1 : -1;
      if (direction && sign != direction)
        return Return::Err(
            Unsupported("Cylinder band profile azimuth is not monotone"));
      direction = sign;
    }
    result.lower = std::min(result.lower, value.lower);
    result.upper = std::max(result.upper, value.upper);
    result.winding += value.winding;
    result.integral += value.integral;
    result.radial = Add(result.radial, value.radial);
  }
  if (previous_line && first_line)
    return Return::Err(
        Unsupported("Cylinder band profile has consecutive axial generators"));
  constexpr long double kWindingTolerance = 1e-10L;
  if (!std::isfinite(result.winding) ||
      std::abs(std::abs(result.winding) - 2 * std::numbers::pi_v<long double>) >
          kWindingTolerance)
    return Return::Err(Unsupported(
        "Cylinder band profile must span one full monotone azimuth"));
  result.integral *= direction;
  result.radial = Scale(result.radial, direction);
  return Return::Ok(result);
}
}  // namespace
Result<CylinderBand> ValidateCylinderBand(
    std::span<const std::vector<PlanarCurve>> loops,
    const CylinderGeometry& cylinder, bool natural_sense, double tolerance,
    uint64_t& remaining_checks) {
  using Return = Result<CylinderBand>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Cylinder band tolerance must be positive and finite"});
  if (!std::isfinite(cylinder.radius) || cylinder.radius <= tolerance)
    return Return::Err(Invalid("Cylinder band radius is degenerate"));
  if (loops.size() != 2)
    return Return::Err(
        Unsupported("Cylinder band requires two closed profiles"));
  const auto first =
      ReadProfile(loops[0], cylinder, tolerance, remaining_checks);
  if (!first.ok()) return Return::Err(first.error());
  const auto second =
      ReadProfile(loops[1], cylinder, tolerance, remaining_checks);
  if (!second.ok()) return Return::Err(second.error());
  if (!remaining_checks)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Cylinder band cut comparison limit exceeded"});
  --remaining_checks;
  const auto* lower = &first.value();
  const auto* upper = &second.value();
  if (upper->upper < lower->lower) std::swap(lower, upper);
  if (!std::isfinite(lower->upper) || !std::isfinite(upper->lower) ||
      upper->lower - lower->upper <= 2 * static_cast<long double>(tolerance))
    return Return::Err(
        Unsupported("Cylinder band profile axial ranges overlap or are "
                    "tolerance-ambiguous"));
  if ((lower->winding > 0) != natural_sense ||
      (upper->winding > 0) == natural_sense)
    return Return::Err(Invalid(
        "Cylinder band profile winding disagrees with the face normal"));
  CylinderBand result{upper->integral - lower->integral,
                      Sub(upper->radial, lower->radial), lower->reference};
  if (!std::isfinite(result.height_integral) || result.height_integral <= 0 ||
      !std::isfinite(Norm(result.radial_integral)))
    return Return::Err(Invalid("Cylinder band integrals are unrepresentable"));
  return Return::Ok(result);
}
long double CylinderBandVolume6(const CylinderGeometry& cylinder,
                                const CylinderBand& band, Vector3 origin) {
  return 2 * static_cast<long double>(cylinder.radius) *
         (cylinder.radius * band.height_integral +
          Dot(Sub(ToVec(cylinder.origin), ToVec(origin)),
              band.radial_integral));
}
}  // namespace openswstep::internal
