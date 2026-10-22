// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "tolerant_circle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>

#include "spun_torus.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
constexpr unsigned kMaxTrimDepth = 64;
constexpr unsigned kMaxBezierDepth = 16;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
Result<const GeometryObject*> Geometry(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  if (!node || !node->geometry)
    return Result<const GeometryObject*>::Err(
        Unsupported("Tolerant edge geometry is unavailable"));
  return Result<const GeometryObject*>::Ok(&model.geometry()[*node->geometry]);
}
Result<const GeometryObject*> Basis(const Model& model, NodeId id) {
  std::set<uint32_t> visited;
  for (unsigned depth = 0; depth < kMaxTrimDepth; ++depth) {
    if (!visited.insert(id.value).second)
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kInvalidRecord, 0, "Cyclic tolerant trim chain"});
    const auto object = Geometry(model, id);
    if (!object.ok()) return object;
    const auto* trim =
        std::get_if<TrimmedCurveGeometry>(&object.value()->payload);
    if (!trim) return object;
    id = trim->basis_curve;
  }
  return Result<const GeometryObject*>::Err(
      {ErrorCode::kResourceLimit, 0, "Tolerant trim chain exceeds 64"});
}
struct ParameterCurve {
  const SurfaceCurveGeometry* surface_curve;
  const NurbsCurveGeometry* spline;
};
Result<ParameterCurve> ReadParameterCurve(const Model& model,
                                          const BoundFinGeometry& fin) {
  using Return = Result<ParameterCurve>;
  const auto basis = Basis(model, fin.curve);
  if (!basis.ok()) return Return::Err(basis.error());
  const auto* curve =
      std::get_if<SurfaceCurveGeometry>(&basis.value()->payload);
  if (!curve || curve->surface != fin.surface)
    return Return::Err(
        Unsupported("Tolerant circular edges require SP_CURVE on their "
                    "supporting surface"));
  const auto object = Geometry(model, curve->parameter_curve);
  if (!object.ok()) return Return::Err(object.error());
  const auto* spline =
      std::get_if<NurbsCurveGeometry>(&object.value()->payload);
  if (!spline || spline->controls.rational || spline->controls.dimension != 2 ||
      spline->axis.periodic || spline->axis.knots.size() < 2 ||
      spline->axis.knots.size() != spline->axis.multiplicities.size() ||
      spline->controls.values.size() !=
          2 * uint64_t{spline->axis.control_count})
    return Return::Err(Unsupported(
        "Tolerant circular edges require nonperiodic polynomial 2D splines"));
  const auto& axis = spline->axis;
  if (axis.multiplicities.front() != axis.degree + 1 ||
      axis.multiplicities.back() != axis.degree + 1 ||
      !((fin.start == axis.knots.front() && fin.end == axis.knots.back()) ||
        (fin.end == axis.knots.front() && fin.start == axis.knots.back())))
    return Return::Err(Unsupported(
        "Tolerant circular edges require the full clamped spline domain"));
  return Return::Ok({curve, spline});
}
long double Dot(Vector3 a, Vector3 b) {
  return static_cast<long double>(a.x) * b.x +
         static_cast<long double>(a.y) * b.y +
         static_cast<long double>(a.z) * b.z;
}
Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
struct Point2 {
  long double x, y;
};
Point2 Midpoint(Point2 a, Point2 b) {
  return {(a.x + b.x) / 2, (a.y + b.y) / 2};
}
long double Dot(Point2 a, Point2 b) { return a.x * b.x + a.y * b.y; }
long double Cross(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }
Result<bool> CheckBezier(const std::array<Point2, 3>& points,
                         long double lower2, long double upper2,
                         uint64_t& remaining, unsigned depth = 0) {
  using Return = Result<bool>;
  if (!remaining)
    return Return::Err({ErrorCode::kResourceLimit, 0,
                        "Tolerant edge Bernstein limit exceeded"});
  --remaining;
  constexpr std::array<int, 3> kChoose2{1, 2, 1};
  constexpr std::array<int, 5> kChoose4{1, 4, 6, 4, 1};
  std::array<long double, 5> coefficients{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      coefficients[i + j] += kChoose2[i] * kChoose2[j] *
                             Dot(points[i], points[j]) / kChoose4[i + j];
  const auto [low, high] =
      std::minmax_element(coefficients.begin(), coefficients.end());
  const auto roundoff = 64 * std::numeric_limits<long double>::epsilon() *
                        std::max(std::abs(*low), std::abs(*high));
  if (!std::isfinite(*low) || !std::isfinite(*high))
    return Return::Err(
        Unsupported("Tolerant circular spline bounds are unrepresentable"));
  if (*low - roundoff >= lower2 && *high + roundoff <= upper2)
    return Return::Ok(true);
  if (*low > upper2 || *high < lower2 || depth == kMaxBezierDepth)
    return Return::Err(
        Unsupported("Planar spline exceeds the circular edge tolerance"));
  const auto a = Midpoint(points[0], points[1]),
             b = Midpoint(points[1], points[2]), c = Midpoint(a, b);
  const auto left =
      CheckBezier({points[0], a, c}, lower2, upper2, remaining, depth + 1);
  if (!left.ok()) return left;
  return CheckBezier({c, b, points[2]}, lower2, upper2, remaining, depth + 1);
}
Result<bool> CheckPlanarSpline(const NurbsCurveGeometry& spline,
                               const PlaneGeometry& plane,
                               const CircleGeometry& circle, double tolerance,
                               double expected_span, bool forward,
                               uint64_t& remaining) {
  using Return = Result<bool>;
  const auto& axis = spline.axis;
  if (axis.degree != 2 || axis.control_count != 2 * (axis.knots.size() - 1) + 1)
    return Return::Err(
        Unsupported("Circular plane pcurves require quadratic Bezier spans"));
  for (std::size_t i = 1; i + 1 < axis.knots.size(); ++i)
    if (axis.multiplicities[i] != 2)
      return Return::Err(Unsupported(
          "Circular plane pcurves require separate quadratic spans"));
  const auto py = Cross(plane.normal, plane.x_axis),
             cy = Cross(circle.normal, circle.x_axis);
  std::vector<Point2> points;
  long double axial = 0, scale = 0, parameter_scale = 0;
  for (std::size_t i = 0; i < axis.control_count; ++i) {
    const long double u = spline.controls.values[2 * i],
                      v = spline.controls.values[2 * i + 1];
    parameter_scale = std::max(parameter_scale, std::hypot(u, v));
    const std::array<long double, 3> world{
        plane.origin.x + u * plane.x_axis.x + v * py.x,
        plane.origin.y + u * plane.x_axis.y + v * py.y,
        plane.origin.z + u * plane.x_axis.z + v * py.z};
    const std::array<long double, 3> relative{world[0] - circle.center.x,
                                              world[1] - circle.center.y,
                                              world[2] - circle.center.z};
    const auto project = [&](Vector3 direction) {
      return relative[0] * direction.x + relative[1] * direction.y +
             relative[2] * direction.z;
    };
    points.push_back({project(circle.x_axis), project(cy)});
    axial = std::max(axial, std::abs(project(circle.normal)));
    for (auto value : world) scale = std::max(scale, std::abs(value));
  }
  // Plane and circle frames originate in doubles. Reserve their coordinate
  // roundoff before certifying the entire polynomial curve in an annulus.
  const auto norm = [](Vector3 value) {
    return std::hypot(value.x, value.y, value.z);
  };
  const auto plane_frame_error = std::abs(norm(plane.normal) - 1) +
                                 std::abs(norm(plane.x_axis) - 1) +
                                 std::abs(Dot(plane.normal, plane.x_axis));
  const auto circle_frame_error = std::abs(norm(circle.normal) - 1) +
                                  std::abs(norm(circle.x_axis) - 1) +
                                  std::abs(Dot(circle.normal, circle.x_axis));
  const long double radial_tolerance =
      tolerance - axial -
      8 * (parameter_scale * plane_frame_error +
           circle.radius * circle_frame_error) -
      64 * std::numeric_limits<double>::epsilon() * scale;
  if (!std::isfinite(radial_tolerance) || radial_tolerance <= 0 ||
      radial_tolerance >= circle.radius / 2)
    return Return::Err(
        Unsupported("Tolerant circular edge clearance is unresolved"));
  const auto lo = circle.radius - radial_tolerance,
             hi = circle.radius + radial_tolerance;
  long double angle = 0;
  constexpr std::array<int, 3> kChoose2{1, 2, 1};
  constexpr std::array<int, 4> kChoose3{1, 3, 3, 1};
  const int direction = expected_span * (forward ? 1 : -1) > 0 ? 1 : -1;
  for (std::size_t i = 0; i + 2 < points.size(); i += 2) {
    const std::array<Point2, 3> p{points[i], points[i + 1], points[i + 2]};
    // A common open half-plane excludes hidden turns within this source span.
    for (auto value : p)
      if (Dot(value, p[0]) <= 0)
        return Return::Err(
            Unsupported("Tolerant circular spline branch is ambiguous"));
    const std::array<Point2, 2> derivative{
        {{2 * (p[1].x - p[0].x), 2 * (p[1].y - p[0].y)},
         {2 * (p[2].x - p[1].x), 2 * (p[2].y - p[1].y)}}};
    std::array<long double, 4> turning{};
    for (std::size_t a = 0; a < 3; ++a)
      for (std::size_t b = 0; b < 2; ++b)
        turning[a + b] +=
            kChoose2[a] * Cross(p[a], derivative[b]) / kChoose3[a + b];
    long double turning_scale = 0;
    for (auto value : p)
      for (auto tangent : derivative)
        turning_scale =
            std::max(turning_scale, std::abs(value.x * tangent.y) +
                                        std::abs(value.y * tangent.x));
    const auto turning_error =
        64 * std::numeric_limits<long double>::epsilon() * turning_scale;
    for (auto value : turning)
      if (!std::isfinite(value) || direction * value <= turning_error)
        return Return::Err(
            Unsupported("Tolerant circular spline azimuth is not monotone"));
    const auto valid = CheckBezier(p, lo * lo, hi * hi, remaining);
    if (!valid.ok()) return valid;
    angle += std::atan2(Cross(p[0], p[2]), Dot(p[0], p[2]));
  }
  if (!forward) angle = -angle;
  if (!std::isfinite(angle) ||
      std::abs(angle - expected_span) > 4 * tolerance / circle.radius)
    return Return::Err(
        Unsupported("Tolerant circular pcurves select different branches"));
  return Return::Ok(true);
}
Result<CircularEdgeRecovery> RecoverPair(
    const Model& model, std::span<const BoundFinGeometry* const> fins,
    const std::map<uint32_t, Vector3>& vertices, double tolerance,
    bool use_source_tolerances, uint64_t& remaining) {
  using Return = Result<CircularEdgeRecovery>;
  if (fins.size() != 2 || !fins[0]->use.fin_curve.value ||
      !fins[1]->use.fin_curve.value || fins[0]->use.edge_curve.value ||
      fins[1]->use.edge_curve.value)
    return Return::Err(
        Unsupported("Circular tolerant recovery requires two fin-specific "
                    "curves without an edge curve"));
  const BoundFinGeometry* planar = nullptr;
  const BoundFinGeometry* rotational = nullptr;
  const PlaneGeometry* plane = nullptr;
  const SpunSurfaceGeometry* spun = nullptr;
  for (auto* fin : fins) {
    if (!fin->use.start_vertex.value || !fin->use.end_vertex.value ||
        fin->use.start_vertex == fin->use.end_vertex)
      return Return::Err(Unsupported(
          "Tolerant circular recovery requires distinct endpoint vertices"));
    const auto surface = Geometry(model, fin->surface);
    if (!surface.ok()) return Return::Err(surface.error());
    if (const auto* p = std::get_if<PlaneGeometry>(&surface.value()->payload)) {
      plane = p;
      planar = fin;
    }
    if (const auto* s =
            std::get_if<SpunSurfaceGeometry>(&surface.value()->payload)) {
      spun = s;
      rotational = fin;
    }
  }
  if (!plane || !spun)
    return Return::Err(Unsupported(
        "Tolerant circular recovery requires a PLANE/SPUN_SURF pair"));
  const auto pc = ReadParameterCurve(model, *planar),
             rc = ReadParameterCurve(model, *rotational);
  if (!pc.ok()) return Return::Err(pc.error());
  if (!rc.ok()) return Return::Err(rc.error());
  const auto& spline = *rc.value().spline;
  if (spline.axis.degree != 1 || spline.axis.control_count != 2 ||
      spline.axis.knots.size() != 2 ||
      spline.controls.values[0] != spline.controls.values[2])
    return Return::Err(Unsupported(
        "Rotational pcurve must be one constant-profile linear segment"));
  const auto profile = Geometry(model, spun->profile);
  if (!profile.ok()) return Return::Err(profile.error());
  const auto* generator =
      std::get_if<CircleGeometry>(&profile.value()->payload);
  if (!generator)
    return Return::Err(
        Unsupported("Tolerant circular recovery needs a circular generator"));
  const auto torus = RecognizeSpunTorus(*spun, *generator, tolerance);
  if (!torus.ok()) return Return::Err(torus.error());
  const auto sample =
      EvaluateCurve(profile.value()->payload, spline.controls.values[0]);
  if (!sample.ok()) return Return::Err(sample.error());
  const auto& t = torus.value().geometry;
  const auto& point = sample.value().position;
  const Vector3 relative{point.x - t.center.x, point.y - t.center.y,
                         point.z - t.center.z};
  const auto height = Dot(relative, t.axis);
  CircleGeometry circle;
  circle.center = {t.center.x + static_cast<double>(height) * t.axis.x,
                   t.center.y + static_cast<double>(height) * t.axis.y,
                   t.center.z + static_cast<double>(height) * t.axis.z};
  circle.normal = t.axis;
  circle.x_axis = t.x_axis;
  circle.radius =
      std::hypot(relative.x - static_cast<double>(height) * t.axis.x,
                 relative.y - static_cast<double>(height) * t.axis.y,
                 relative.z - static_cast<double>(height) * t.axis.z);
  if (!std::isfinite(circle.radius) || circle.radius <= 2 * tolerance)
    return Return::Err(
        Unsupported("Tolerant circular recovery has a degenerate radius"));
  // The constant profile's radial phase need not agree with the torus frame.
  const auto y = Cross(t.axis, t.x_axis);
  const auto phase = std::atan2(Dot(relative, y), Dot(relative, t.x_axis));
  double first = spline.controls.values[1] + static_cast<double>(phase),
         last = spline.controls.values[3] + static_cast<double>(phase);
  if (rotational->start > rotational->end) std::swap(first, last);
  if (rotational->use.sense == TopologySense::kReverse) std::swap(first, last);
  if (!std::isfinite(first) || !std::isfinite(last) || first == last ||
      circle.radius * std::abs(last - first) <= tolerance ||
      std::abs(last - first) >= 2 * std::numbers::pi)
    return Return::Err(Unsupported(
        "Tolerant circular recovery needs a resolved partial turn"));
  const auto* edge = model.FindNode(planar->use.edge);
  if (use_source_tolerances && edge && edge->edge_tolerance) {
    const auto transmitted = *edge->edge_tolerance;
    if (!std::isfinite(transmitted) || transmitted <= 0)
      return Return::Err({ErrorCode::kInvalidRecord, 0,
                          "Source edge tolerance must be positive and finite"});
    tolerance = std::max(tolerance, transmitted);
  }
  if (2 * circle.radius * std::abs(std::sin((last - first) / 2)) <=
      2 * tolerance)
    return Return::Err(
        Unsupported("Tolerant circular endpoint separation is ambiguous"));
  const auto desired_first =
      planar->use.sense == TopologySense::kForward ? first : last;
  const auto desired_last =
      planar->use.sense == TopologySense::kForward ? last : first;
  const auto certified = CheckPlanarSpline(
      *pc.value().spline, *plane, circle, tolerance,
      desired_last - desired_first, planar->end > planar->start, remaining);
  if (!certified.ok()) return Return::Err(certified.error());
  CircularEdgeRecovery result{
      planar->use.edge, planar->use.edge, circle, tolerance, {}};
  for (auto* fin : fins) {
    const auto a = fin->use.sense == TopologySense::kForward ? first : last;
    const auto b = fin->use.sense == TopologySense::kForward ? last : first;
    for (const auto parameter : {a, b}) {
      const auto canonical = EvaluateCurve(GeometryPayload{circle}, parameter);
      const auto source = EvaluateCurve(model, fin->curve,
                                        parameter == a ? fin->start : fin->end);
      if (!canonical.ok()) return Return::Err(canonical.error());
      if (!source.ok()) return Return::Err(source.error());
      const auto p = canonical.value().position, q = source.value().position;
      const auto vertex = vertices.find(
          (parameter == a ? fin->use.start_vertex : fin->use.end_vertex).value);
      if (vertex == vertices.end())
        return Return::Err(
            {ErrorCode::kInvalidRecord, 0, "Recovered edge vertex is absent"});
      const auto v = vertex->second;
      if (std::hypot(p.x - v.x, p.y - v.y, p.z - v.z) > tolerance)
        return Return::Err(Unsupported(
            "Recovered circular edge exceeds its vertex tolerance"));
      if (std::hypot(p.x - q.x, p.y - q.y, p.z - q.z) > tolerance)
        return Return::Err(Unsupported(
            "Tolerant circular source endpoints select different arcs"));
    }
    NurbsCurveGeometry pcurve =
        fin == planar ? *pc.value().spline : *rc.value().spline;
    if (fin == rotational) {
      // STEP torus U is azimuth and V is meridian. SPUN's source pcurve is
      // (generator parameter, azimuth); this is an exact affine change here.
      const double meridian = std::atan2(static_cast<double>(height),
                                         circle.radius - t.major_radius);
      pcurve.controls.values = {
          spline.controls.values[1] + static_cast<double>(phase), meridian,
          spline.controls.values[3] + static_cast<double>(phase), meridian};
    }
    result.uses.push_back(
        {fin->use.fin, fin->surface, a, b, std::move(pcurve)});
  }
  return Return::Ok(std::move(result));
}
}  // namespace
Result<std::vector<CircularEdgeRecovery>> RecoverTolerantCircles(
    const Model& model, const BodyGeometryBinding& body, double tolerance,
    bool use_source_tolerances, uint64_t& remaining_checks) {
  using Return = Result<std::vector<CircularEdgeRecovery>>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Tolerant edge tolerance must be positive and finite"});
  std::map<uint32_t, std::vector<const BoundFinGeometry*>> edges;
  for (const auto& face : body.faces)
    for (const auto& loop : face.loops)
      for (const auto& fin : loop.fins)
        edges[fin.use.edge.value].push_back(&fin);
  std::map<uint32_t, Vector3> vertices;
  for (const auto& vertex : body.vertices)
    vertices.emplace(vertex.vertex.value, vertex.position);
  std::vector<CircularEdgeRecovery> result;
  for (const auto& [id, fins] : edges) {
    if (std::none_of(fins.begin(), fins.end(),
                     [](auto* fin) { return fin->use.fin_curve.value != 0; }))
      continue;
    const auto recovered = RecoverPair(model, fins, vertices, tolerance,
                                       use_source_tolerances, remaining_checks);
    if (!recovered.ok())
      return Return::Err({recovered.error().code, recovered.error().offset,
                          "STEP tolerant edge " + std::to_string(id) + ": " +
                              recovered.error().message});
    result.push_back(recovered.value());
  }
  return Return::Ok(std::move(result));
}
}  // namespace openswstep::internal
