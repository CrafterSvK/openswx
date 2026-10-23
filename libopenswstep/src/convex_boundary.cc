// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "convex_boundary.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "nurbs_wire.h"
#include "openparasolid/evaluate.h"
#include "planar_boundary.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
constexpr long double kConicPlaneAlignmentTolerance = 1e-9L;
struct Point2 {
  long double x, y;
};
struct Segment {
  Point2 first_tangent, last_tangent;
  long double area2;
  long double curvature;
  long double length;
};
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidRecord, 0, message};
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
long double Cross(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }
long double Project(Vector3 point, Vector3 origin, Vector3 axis) {
  return (static_cast<long double>(point.x) - origin.x) * axis.x +
         (static_cast<long double>(point.y) - origin.y) * axis.y +
         (static_cast<long double>(point.z) - origin.z) * axis.z;
}
struct Frame {
  Vector3 origin, x, y;
  Point2 Point(Vector3 point) const {
    return {Project(point, origin, x), Project(point, origin, y)};
  }
  Point2 Direction(Vector3 vector) const {
    return {Project(vector, {}, x), Project(vector, {}, y)};
  }
};
Result<Point2> Tangent(Vector3 value, const Frame& frame, bool forward) {
  const auto t = frame.Direction(value);
  const auto length = std::hypot(t.x, t.y);
  if (!std::isfinite(length) || length == 0)
    return Result<Point2>::Err(
        Invalid("Planar boundary tangent is degenerate"));
  const long double sign = forward ? 1 : -1;
  return Result<Point2>::Ok({sign * t.x / length, sign * t.y / length});
}
// Integrate the change of tangent angle, not the ellipse parameter. Four
// equal steps keep each parameter interval at most pi/2 and avoid angle-wrap
// ambiguity, including arcs longer than pi and complete revolutions.
long double EllipseTurning(double start, double end, long double major,
                           long double minor) {
  const auto a = major / std::max(major, minor);
  const auto b = minor / std::max(major, minor);
  constexpr int kParameterSteps = 4;
  const auto step = (static_cast<long double>(end) - start) / kParameterSteps;
  long double turning = 0;
  for (int i = 0; i < kParameterSteps; ++i) {
    const auto t = static_cast<long double>(start) + i * step;
    const auto dot = a * a * std::sin(t) * std::sin(t + step) +
                     b * b * std::cos(t) * std::cos(t + step);
    turning += std::atan2(a * b * std::abs(std::sin(step)), dot);
  }
  return turning;
}
Result<Segment> ReadQuadraticSegment(const PlanarCurve& curve,
                                     const PlaneGeometry& plane,
                                     const Frame& frame, double tolerance,
                                     uint64_t& remaining_checks) {
  using Return = Result<Segment>;
  const auto* spline = std::get_if<NurbsCurveGeometry>(curve.geometry);
  if (!spline || spline->controls.rational || spline->axis.degree != 2)
    return Return::Err(Unsupported(
        "Planar spline bounds require polynomial quadratic Bezier sections"));
  const auto section =
      RestrictNurbsWire(*spline, curve.start, curve.end, remaining_checks);
  if (!section.ok()) return Return::Err(section.error());
  const auto& bezier = section.value();
  if (bezier.axis.control_count != 3 || bezier.axis.knots.size() != 2)
    return Return::Err(Unsupported(
        "Planar spline bounds require a single quadratic Bezier section"));
  std::array<Point2, 3> controls;
  for (std::size_t i = 0; i < controls.size(); ++i) {
    if (!remaining_checks)
      return Return::Err({ErrorCode::kResourceLimit, 0,
                          "Planar spline control check limit exceeded"});
    --remaining_checks;
    const auto& values = bezier.controls.values;
    const Vector3 point{values[i * 3], values[i * 3 + 1], values[i * 3 + 2]};
    // The polynomial lies in its control hull, so all three controls certify
    // membership of the entire selected curve in the face plane.
    const auto residual = Project(point, plane.origin, plane.normal);
    if (!std::isfinite(residual) || std::abs(residual) > tolerance)
      return Return::Err(
          Unsupported("Spline control hull is outside the face plane"));
    controls[i] = frame.Point(point);
  }
  const auto p = controls[0], q = controls[1], r = controls[2];
  const auto a_length = std::hypot(q.x - p.x, q.y - p.y);
  const auto b_length = std::hypot(r.x - q.x, r.y - q.y);
  if (!std::isfinite(a_length) || !std::isfinite(b_length) || a_length == 0 ||
      b_length == 0)
    return Return::Err(
        Invalid("Planar spline endpoint derivative is degenerate"));
  const Point2 a{(q.x - p.x) / a_length, (q.y - p.y) / a_length};
  const Point2 b{(r.x - q.x) / b_length, (r.y - q.y) / b_length};
  const Point2 u = curve.end > curve.start ? a : Point2{-b.x, -b.y};
  const Point2 v = curve.end > curve.start ? b : Point2{-a.x, -a.y};
  Segment result{u, v, 0, 0, a_length + b_length};
  result.curvature = std::atan2(Cross(u, v), u.x * v.x + u.y * v.y);
  // A quadratic derivative interpolates its endpoint derivatives. Turning
  // shorter than pi excludes an interior reversal without sampling.
  if (!std::isfinite(result.curvature) || result.curvature < 0 ||
      result.curvature >= std::numbers::pi_v<long double> - 1e-10L)
    return Return::Err(
        Unsupported("Planar quadratic spline bounds require nonnegative "
                    "curvature without reversals"));
  const long double sign = curve.end > curve.start ? 1 : -1;
  // Exact integral of x*dy-y*dx for the selected quadratic polynomial.
  result.area2 = sign * (2 * Cross(p, q) + Cross(p, r) + 2 * Cross(q, r)) / 3;
  if (!std::isfinite(result.area2) || !std::isfinite(result.length) ||
      result.length <= tolerance)
    return Return::Err(
        Invalid("Planar spline area or perimeter is unresolved"));
  return Return::Ok(result);
}
Result<Segment> ReadSegment(const PlanarCurve& curve,
                            const PlaneGeometry& plane, const Frame& frame,
                            double tolerance, uint64_t& remaining_checks,
                            bool convex = true) {
  using Return = Result<Segment>;
  if (!curve.geometry || !std::isfinite(curve.start) ||
      !std::isfinite(curve.end) || curve.start == curve.end)
    return Return::Err(
        Invalid("Planar curve requires a finite nonzero interval"));
  const auto* line = std::get_if<LineGeometry>(curve.geometry);
  const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
  const auto* ellipse = std::get_if<EllipseGeometry>(curve.geometry);
  const auto* spline = std::get_if<NurbsCurveGeometry>(curve.geometry);
  if (spline)
    return ReadQuadraticSegment(curve, plane, frame, tolerance,
                                remaining_checks);
  if (!line && !circle && !ellipse)
    return Return::Err(
        Unsupported("Convex boundaries require LINE/CIRCLE/ELLIPSE or "
                    "polynomial quadratic B_CURVE bases"));
  const auto first = EvaluateCurve(*curve.geometry, curve.start);
  if (!first.ok()) return Return::Err(first.error());
  const auto last = EvaluateCurve(*curve.geometry, curve.end);
  if (!last.ok()) return Return::Err(last.error());
  for (const auto p : {first.value().position, last.value().position}) {
    const auto distance = Project(p, plane.origin, plane.normal);
    if (!std::isfinite(distance) || std::abs(distance) > tolerance)
      return Return::Err(
          Invalid("Curved boundary endpoint is outside the plane"));
  }
  const auto first_tangent =
      Tangent(first.value().first_derivative, frame, curve.end > curve.start);
  if (!first_tangent.ok()) return Return::Err(first_tangent.error());
  const auto last_tangent =
      Tangent(last.value().first_derivative, frame, curve.end > curve.start);
  if (!last_tangent.ok()) return Return::Err(last_tangent.error());
  const auto a = frame.Point(first.value().position),
             b = frame.Point(last.value().position);
  Segment result{first_tangent.value(), last_tangent.value(), Cross(a, b), 0,
                 std::hypot(b.x - a.x, b.y - a.y)};
  if (circle || ellipse) {
    const auto center3 = circle ? circle->center : ellipse->center;
    const long double major = circle ? circle->radius : ellipse->major_radius;
    const long double minor = circle ? circle->radius : ellipse->minor_radius;
    if (!std::isfinite(major) || !std::isfinite(minor) || major <= 0 ||
        minor <= 0)
      return Return::Err(Invalid("Supporting conic radii must be positive"));
    const auto span = curve.end - curve.start;
    if (!std::isfinite(span) || std::abs(span) > 2 * std::numbers::pi)
      return Return::Err(
          Unsupported("Curved boundary arc exceeds one conic turn"));
    const auto& n = circle ? circle->normal : ellipse->normal;
    const auto& x = circle ? circle->x_axis : ellipse->x_axis;
    const Vector3 y{n.y * x.z - n.z * x.y, n.z * x.x - n.x * x.z,
                    n.x * x.y - n.y * x.x};
    const auto center_residual =
        std::abs(Project(center3, plane.origin, plane.normal));
    const auto excursion = std::hypot(major * Project(x, {}, plane.normal),
                                      minor * Project(y, {}, plane.normal));
    if (!std::isfinite(center_residual + excursion) ||
        center_residual + excursion > tolerance)
      return Return::Err(
          Invalid("Complete supporting conic is outside the face plane"));
    const auto rx = frame.Direction(x), ry = frame.Direction(y);
    const auto determinant = Cross(rx, ry);
    if (!std::isfinite(determinant) ||
        std::abs(determinant) < 1 - kConicPlaneAlignmentTolerance)
      return Return::Err(Invalid("Supporting conic plane is misaligned"));
    result.curvature = determinant > 0 ? span : -span;
    if (convex && result.curvature <= 0)
      return Return::Err(Unsupported(
          "Curved planar export requires a convex source-oriented boundary"));
    if (ellipse)
      result.curvature = EllipseTurning(curve.start, curve.end, major, minor);
    const auto center = frame.Point(center3);
    result.area2 = Cross(center, {b.x - a.x, b.y - a.y}) +
                   major * minor * determinant * span;
    // Conservative length bounds avoid numerical arc-length quadrature. The
    // upper bound also makes area and corner-noise checks conservative.
    if (std::min(major, minor) * std::abs(span) <= tolerance)
      return Return::Err(Invalid("Conic arc length cannot be resolved"));
    result.length = std::max(major, minor) * std::abs(span);
  }
  if (!std::isfinite(result.area2) || !std::isfinite(result.length) ||
      result.length <= tolerance)
    return Return::Err(
        Invalid("Curved boundary segment is degenerate or unrepresentable"));
  return Return::Ok(result);
}
// Maximum of A*cos(t)+B*sin(t) on a directed interval no longer than 2*pi.
// The endpoints and at most one interior maximum cover the entire arc.
}  // namespace
long double MaximumArcProjection(long double a, long double b, double start,
                                 double end) {
  const long double first = start, last = end;
  auto maximum = std::max(a * std::cos(first) + b * std::sin(first),
                          a * std::cos(last) + b * std::sin(last));
  // Rotate coefficients by the start phase before locating the peak. Avoid
  // subtracting a huge absolute parameter from atan2's small principal angle.
  const auto c = std::cos(first), s = std::sin(first);
  const auto along = a * c + b * s;
  const auto across = b * c - a * s;
  auto offset = std::atan2(last > first ? across : -across, along);
  if (offset < 0) offset += 2 * std::numbers::pi_v<long double>;
  if (offset <= std::abs(last - first)) maximum = std::hypot(a, b);
  return maximum;
}
namespace {
Result<long double> DiskClearance(const PlanarCurve& curve, Vector3 center,
                                  const Frame& frame) {
  using Return = Result<long double>;
  if (const auto* circle = std::get_if<CircleGeometry>(curve.geometry)) {
    const auto& n = circle->normal;
    const auto& x = circle->x_axis;
    const auto rx = frame.Direction(x);
    const auto ry = frame.Direction(
        {n.y * x.z - n.z * x.y, n.z * x.x - n.x * x.z, n.x * x.y - n.y * x.x});
    const auto point = frame.Point(center),
               origin = frame.Point(circle->center);
    const Point2 d{point.x - origin.x, point.y - origin.y};
    const long double sign = curve.end > curve.start ? 1 : -1;
    // Outward normals to projected tangents are N*cos(t)+M*sin(t).
    // Use the largest singular value of their two-column frame as a bound
    // on every normal length. It is one for exact coplanar unit frames; the
    // explicit bound also covers accepted frame roundoff and small tilts.
    const Point2 nc{sign * ry.y, -sign * ry.x};
    const Point2 ns{-sign * rx.y, sign * rx.x};
    const auto maximum =
        MaximumArcProjection(d.x * nc.x + d.y * nc.y, d.x * ns.x + d.y * ns.y,
                             curve.start, curve.end);
    const auto aa = nc.x * nc.x + nc.y * nc.y;
    const auto bb = ns.x * ns.x + ns.y * ns.y;
    const auto ab = nc.x * ns.x + nc.y * ns.y;
    const auto normal_bound =
        std::sqrt((aa + bb + std::hypot(aa - bb, 2 * ab)) / 2);
    return Return::Ok((circle->radius * std::abs(Cross(rx, ry)) - maximum) /
                      normal_bound);
  }
  const auto a = EvaluateCurve(*curve.geometry, curve.start);
  const auto b = EvaluateCurve(*curve.geometry, curve.end);
  if (!a.ok()) return Return::Err(a.error());
  if (!b.ok()) return Return::Err(b.error());
  const auto first = frame.Point(a.value().position),
             last = frame.Point(b.value().position),
             point = frame.Point(center);
  const Point2 tangent{last.x - first.x, last.y - first.y};
  return Return::Ok(Cross(tangent, {point.x - first.x, point.y - first.y}) /
                    std::hypot(tangent.x, tangent.y));
}
}  // namespace
Result<ConvexBoundary> ValidateConvexPlanarLoop(
    std::span<const PlanarCurve> curves, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<ConvexBoundary>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Convex boundary tolerance must be positive and finite"});
  if (curves.size() < 2 || !curves.front().geometry)
    return Return::Err(
        Unsupported("Convex curved loop requires at least two edges"));
  const auto first =
      EvaluateCurve(*curves.front().geometry, curves.front().start);
  if (!first.ok()) return Return::Err(first.error());
  const auto& n = plane.normal;
  const auto& x = plane.x_axis;
  const double sign = natural_sense ? 1 : -1;
  const Frame frame{
      first.value().position,
      x,
      {sign * (n.y * x.z - n.z * x.y), sign * (n.z * x.x - n.x * x.z),
       sign * (n.x * x.y - n.y * x.x)}};
  std::vector<Segment> segments;
  segments.reserve(curves.size());
  long double area2 = 0, perimeter = 0, turning = 0;
  for (const auto& curve : curves) {
    const auto segment =
        ReadSegment(curve, plane, frame, tolerance, remaining_checks);
    if (!segment.ok()) return Return::Err(segment.error());
    segments.push_back(segment.value());
    area2 += segment.value().area2;
    perimeter += segment.value().length;
    turning += segment.value().curvature;
  }
  if (!std::isfinite(area2) || !std::isfinite(perimeter) ||
      area2 <= tolerance * perimeter)
    return Return::Err(
        Invalid("Curved boundary area is degenerate or has incorrect winding"));
  constexpr long double kAngleTolerance = 1e-10L;
  // Bound accepted negative corner noise by both angle and source length.
  const auto corner_tolerance =
      std::min(kAngleTolerance, static_cast<long double>(tolerance) /
                                    perimeter / segments.size());
  for (std::size_t i = 0; i < segments.size(); ++i) {
    if (!remaining_checks)
      return Return::Err({ErrorCode::kResourceLimit, 0,
                          "Convex boundary corner limit exceeded"});
    --remaining_checks;
    const auto a = segments[i].last_tangent,
               b = segments[(i + 1) % segments.size()].first_tangent;
    const auto corner = std::atan2(Cross(a, b), a.x * b.x + a.y * b.y);
    if (!std::isfinite(corner) || corner < -corner_tolerance ||
        corner >= std::numbers::pi_v<long double> - kAngleTolerance)
      return Return::Err(Unsupported(
          "Curved planar export requires convex corners without reversals"));
    turning += std::max(0.L, corner);
  }
  if (!std::isfinite(turning) ||
      std::abs(turning - 2 * std::numbers::pi_v<long double>) >
          kAngleTolerance * (segments.size() + 1))
    return Return::Err(
        Invalid("Curved planar loop does not make one convex revolution"));
  return Return::Ok({area2 / 2, first.value().position});
}
Result<ConvexBoundary> ValidateMinorArcPlanarLoop(
    std::span<const PlanarCurve> curves, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<ConvexBoundary>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Minor-arc boundary tolerance must be positive and finite"});
  if (curves.size() < 3)
    return Return::Err(
        Unsupported("Minor-arc boundary requires at least three edges"));
  for (const auto& curve : curves)
    if (!curve.geometry ||
        (!std::holds_alternative<LineGeometry>(*curve.geometry) &&
         !std::holds_alternative<CircleGeometry>(*curve.geometry)))
      return Return::Err(
          Unsupported("Minor-arc boundary requires LINE/CIRCLE bases"));
  const auto first =
      EvaluateCurve(*curves.front().geometry, curves.front().start);
  if (!first.ok()) return Return::Err(first.error());
  const auto& n = plane.normal;
  const auto& x = plane.x_axis;
  const double sign = natural_sense ? 1 : -1;
  const Frame frame{
      first.value().position,
      x,
      {sign * (n.y * x.z - n.z * x.y), sign * (n.z * x.x - n.x * x.z),
       sign * (n.x * x.y - n.y * x.x)}};
  std::vector<Vector3> vertices, last;
  std::vector<Point2> points;
  std::vector<long double> sagitta, departure;
  long double area2 = 0, perimeter = 0;
  for (const auto& curve : curves) {
    const auto segment =
        ReadSegment(curve, plane, frame, tolerance, remaining_checks, false);
    if (!segment.ok()) return Return::Err(segment.error());
    const auto a = EvaluateCurve(*curve.geometry, curve.start);
    const auto b = EvaluateCurve(*curve.geometry, curve.end);
    if (!a.ok()) return Return::Err(a.error());
    if (!b.ok()) return Return::Err(b.error());
    vertices.push_back(a.value().position);
    last.push_back(b.value().position);
    points.push_back(frame.Point(a.value().position));
    long double bulge = 0, angle = 0;
    if (const auto* circle = std::get_if<CircleGeometry>(curve.geometry)) {
      const auto span =
          std::abs(static_cast<long double>(curve.end) - curve.start);
      if (span > std::numbers::pi)
        return Return::Err(
            Unsupported("Minor-arc certificate excludes major arcs"));
      // A factor two over the circular endpoint-cone half-angle also covers
      // the tiny distortion of accepted source/face frames under projection.
      angle = span;
      const auto sine = std::sin(span / 4);
      bulge = 2 * static_cast<long double>(circle->radius) * sine * sine;
    }
    const auto gap = std::hypot(
        static_cast<long double>(b.value().position.x) - a.value().position.x,
        static_cast<long double>(b.value().position.y) - a.value().position.y,
        static_cast<long double>(b.value().position.z) - a.value().position.z);
    if (!std::isfinite(bulge) || !std::isfinite(gap) || gap <= tolerance)
      return Return::Err(Invalid("Minor-arc chord is degenerate"));
    sagitta.push_back(bulge);
    departure.push_back(angle);
    area2 += segment.value().area2;
    perimeter += segment.value().length;
  }
  const std::vector<std::vector<Vector3>> polygon{vertices};
  const auto simple = ValidatePlanarLoops(polygon, plane, natural_sense,
                                          tolerance, remaining_checks);
  if (!simple.ok()) return Return::Err(simple.error());
  std::vector<long double> lengths, closures;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto j = (i + 1) % points.size();
    lengths.push_back(
        std::hypot(points[j].x - points[i].x, points[j].y - points[i].y));
    const auto endpoint_gap =
        std::hypot(static_cast<long double>(last[i].x) - vertices[j].x,
                   static_cast<long double>(last[i].y) - vertices[j].y,
                   static_cast<long double>(last[i].z) - vertices[j].z);
    const auto coordinate_scale =
        std::max({std::abs(static_cast<long double>(last[i].x)),
                  std::abs(static_cast<long double>(last[i].y)),
                  std::abs(static_cast<long double>(last[i].z)),
                  std::abs(static_cast<long double>(vertices[j].x)),
                  std::abs(static_cast<long double>(vertices[j].y)),
                  std::abs(static_cast<long double>(vertices[j].z))});
    const auto closure =
        endpoint_gap +
        16 * std::numeric_limits<double>::epsilon() * coordinate_scale;
    closures.push_back(closure);
    if (!std::isfinite(closure) || closure > tolerance / 4)
      return Return::Err(
          Unsupported("Minor-arc joins are tolerance-ambiguous"));
    // Projection does not enlarge a circular sagitta for exact unit frames;
    // the factor two also bounds accepted frame roundoff and endpoint closure.
    sagitta[i] = 2 * (sagitta[i] + closure);
    departure[i] += 2 * std::asin(std::min(1.L, closure / lengths.back()));
  }
  const auto distance = [](Point2 p, Point2 a, Point2 b) {
    const auto dx = b.x - a.x, dy = b.y - a.y;
    const auto length2 = dx * dx + dy * dy;
    const auto t =
        std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.L, 1.L);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
  };
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!remaining_checks)
      return Return::Err(
          {ErrorCode::kResourceLimit, 0, "Minor-arc join limit exceeded"});
    --remaining_checks;
    const auto previous = (i + points.size() - 1) % points.size();
    const auto next = (i + 1) % points.size();
    const auto ax = (points[previous].x - points[i].x) / lengths[previous];
    const auto ay = (points[previous].y - points[i].y) / lengths[previous];
    const auto bx = (points[next].x - points[i].x) / lengths[i];
    const auto by = (points[next].y - points[i].y) / lengths[i];
    const auto separation = std::acos(std::clamp(ax * bx + ay * by, -1.L, 1.L));
    // Limit possible crossings from endpoint uncertainty to the already
    // admitted source join tolerance; outside it the endpoint cones separate.
    const auto join_margin = std::asin(4 * closures[previous] / tolerance);
    if (!std::isfinite(separation) || !std::isfinite(join_margin) ||
        separation <= departure[previous] + departure[i] + join_margin + 1e-10L)
      return Return::Err(Unsupported(
          "Minor-arc endpoint cones cannot certify separated joins"));
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      if (j == i + 1 || (i == 0 && j + 1 == points.size())) continue;
      if (!remaining_checks)
        return Return::Err({ErrorCode::kResourceLimit, 0,
                            "Minor-arc envelope limit exceeded"});
      --remaining_checks;
      const auto a = points[i], b = points[next], c = points[j],
                 d = points[(j + 1) % points.size()];
      const auto clearance = std::min({distance(a, c, d), distance(b, c, d),
                                       distance(c, a, b), distance(d, a, b)});
      if (!std::isfinite(clearance) ||
          clearance <=
              2 * static_cast<long double>(tolerance) + sagitta[i] + sagitta[j])
        return Return::Err(Unsupported(
            "Minor-arc envelopes cannot certify nonadjacent separation"));
    }
  }
  if (!std::isfinite(area2) || !std::isfinite(perimeter) ||
      area2 <= tolerance * perimeter)
    return Return::Err(
        Invalid("Minor-arc source area is degenerate or miswound"));
  return Return::Ok({area2 / 2, vertices.front()});
}
Result<ConvexBoundary> ValidateConvexPlanarHoles(
    std::span<const PlanarCurve> curves,
    std::span<const std::vector<Vector3>> polygons,
    std::span<const ConicBoundary> holes, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<ConvexBoundary>;
  for (const auto& curve : curves)
    if (!curve.geometry ||
        (!std::holds_alternative<LineGeometry>(*curve.geometry) &&
         !std::holds_alternative<CircleGeometry>(*curve.geometry)))
      return Return::Err(Unsupported(
          "Curved contours with holes require LINE/CIRCLE outer edges"));
  const auto outer = ValidateConvexPlanarLoop(curves, plane, natural_sense,
                                              tolerance, remaining_checks);
  if (!outer.ok()) return outer;
  const auto& n = plane.normal;
  const auto& x = plane.x_axis;
  const double sign = natural_sense ? 1 : -1;
  const Frame frame{
      outer.value().reference_point,
      x,
      {sign * (n.y * x.z - n.z * x.y), sign * (n.z * x.x - n.x * x.z),
       sign * (n.x * x.y - n.y * x.x)}};
  auto result = outer.value();
  const long double clearance = 2 * static_cast<long double>(tolerance);
  for (std::size_t i = 0; i < holes.size(); ++i) {
    const auto& hole = holes[i];
    if (!hole.circle)
      return Return::Err(Unsupported("Curved face holes require CIRCLE bases"));
    // A hole must be a full coplanar circle wound against the outer normal.
    const auto valid =
        ValidatePlanarConics(holes.subspan(i, 1), plane, !natural_sense,
                             tolerance, remaining_checks);
    if (!valid.ok()) return Return::Err(valid.error());
    for (const auto& curve : curves) {
      if (!remaining_checks)
        return Return::Err({ErrorCode::kResourceLimit, 0,
                            "Curved hole comparison limit exceeded"});
      --remaining_checks;
      const auto distance = DiskClearance(curve, hole.center, frame);
      if (!distance.ok()) return Return::Err(distance.error());
      if (!std::isfinite(distance.value()) ||
          distance.value() <= hole.major_radius + clearance)
        return Return::Err(Invalid(
            "Circular hole is outside or touches the convex curved contour"));
    }
    const auto a = frame.Point(hole.center);
    for (std::size_t j = 0; j < i; ++j) {
      if (!remaining_checks)
        return Return::Err({ErrorCode::kResourceLimit, 0,
                            "Curved hole comparison limit exceeded"});
      --remaining_checks;
      const auto b = frame.Point(holes[j].center);
      const auto distance = std::hypot(a.x - b.x, a.y - b.y);
      if (!std::isfinite(distance) ||
          distance <= hole.major_radius + holes[j].major_radius + clearance)
        return Return::Err(
            Invalid("Circular curved-face holes overlap, touch "
                    "or are nested"));
    }
    result.area -=
        std::numbers::pi_v<long double> * hole.major_radius * hole.minor_radius;
  }
  const auto polygon_area = ValidatePolygonHoleSet(
      polygons, holes, plane, natural_sense, tolerance, remaining_checks);
  if (!polygon_area.ok()) return Return::Err(polygon_area.error());
  for (const auto& polygon : polygons)
    for (const auto point : polygon)
      for (const auto& curve : curves) {
        if (!remaining_checks)
          return Return::Err({ErrorCode::kResourceLimit, 0,
                              "Curved polygon hole comparison limit exceeded"});
        --remaining_checks;
        const auto distance = DiskClearance(curve, point, frame);
        if (!distance.ok()) return Return::Err(distance.error());
        if (!std::isfinite(distance.value()) || distance.value() <= clearance)
          return Return::Err(
              Invalid("Polygon hole is outside, touching or too "
                      "close to the convex curved contour"));
      }
  result.area -= polygon_area.value();
  if (!std::isfinite(result.area) || result.area <= 0)
    return Return::Err(Invalid("Curved face net area is nonpositive"));
  return Return::Ok(result);
}
}  // namespace openswstep::internal
