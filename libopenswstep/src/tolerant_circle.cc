// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "tolerant_circle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <set>

#include "spun_torus.h"
#include "tolerant_cylinder.h"

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
  NurbsCurveGeometry spline;
};
Result<ParameterCurve> ReadParameterCurve(const Model& model,
                                          const BoundFinGeometry& fin,
                                          bool ring) {
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
      (spline->axis.periodic && !ring) || spline->axis.knots.size() < 2 ||
      spline->axis.knots.size() != spline->axis.multiplicities.size() ||
      spline->controls.values.size() !=
          2 * uint64_t{spline->axis.control_count})
    return Return::Err(Unsupported(
        "Tolerant circular edges require nonperiodic polynomial 2D splines"));
  auto result = *spline;
  auto& axis = result.axis;
  const bool periodic_source = axis.periodic;
  if (axis.knots.size() != axis.multiplicities.size() ||
      uint64_t{axis.control_count} + axis.degree + 1 > 1000000)
    return Return::Err(Unsupported("Circular pcurve knot arrays are invalid"));
  std::vector<double> source_knots;
  source_knots.reserve(uint64_t{axis.control_count} + axis.degree + 1);
  for (std::size_t i = 0; i < axis.knots.size(); ++i)
    source_knots.insert(source_knots.end(), axis.multiplicities[i],
                        axis.knots[i]);
  if (source_knots.size() != uint64_t{axis.control_count} + axis.degree + 1)
    return Return::Err(Unsupported("Circular pcurve knot vector is invalid"));
  const double source_domain_start = source_knots[axis.degree];
  const double source_domain_end = source_knots[axis.control_count];
  if (ring && axis.degree == 2 && axis.knots.size() >= 4 &&
      axis.multiplicities.front() == 1 && axis.multiplicities.back() == 1 &&
      axis.multiplicities[1] == 2 &&
      axis.multiplicities[axis.knots.size() - 2] == 2) {
    // These exterior knots have no influence inside the natural domain:
    // multiplicity p at each domain endpoint already splits quadratic spans.
    // Replace only the inactive exterior knots by endpoint clamps. The source
    // controls and every polynomial piece in the selected domain stay exact.
    axis.knots.erase(axis.knots.begin());
    axis.knots.pop_back();
    axis.multiplicities.erase(axis.multiplicities.begin());
    axis.multiplicities.pop_back();
    axis.multiplicities.front() = 3;
    axis.multiplicities.back() = 3;
  }
  const double domain_width = axis.knots.back() - axis.knots.front();
  const double trim_width = std::abs(fin.end - fin.start);
  const double period_roundoff =
      64 * std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(domain_width), std::abs(trim_width)});
  const bool full_domain =
      (fin.start == axis.knots.front() && fin.end == axis.knots.back()) ||
      (fin.end == axis.knots.front() && fin.start == axis.knots.back());
  const bool full_period =
      periodic_source &&
      std::abs(trim_width - (source_domain_end - source_domain_start)) <=
          period_roundoff;
  if (axis.multiplicities.front() != axis.degree + 1 ||
      axis.multiplicities.back() != axis.degree + 1 ||
      (!full_domain && !full_period))
    return Return::Err(Unsupported(
        "Tolerant circular edges require the full clamped spline domain"));
  axis.periodic = false;
  return Return::Ok({curve, std::move(result)});
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

Result<CurveEvaluation> EvaluateRingUse(const Model& model,
                                        const BoundFinGeometry& fin,
                                        double parameter) {
  using Return = Result<CurveEvaluation>;
  const auto basis = Basis(model, fin.curve);
  if (!basis.ok()) return Return::Err(basis.error());
  const auto* surface_curve =
      std::get_if<SurfaceCurveGeometry>(&basis.value()->payload);
  if (!surface_curve) return EvaluateCurve(model, fin.curve, parameter);
  const auto pcurve = Geometry(model, surface_curve->parameter_curve);
  if (!pcurve.ok()) return Return::Err(pcurve.error());
  const auto* spline =
      std::get_if<NurbsCurveGeometry>(&pcurve.value()->payload);
  if (!spline || !spline->axis.periodic || !spline->axis.closed)
    return EvaluateCurve(model, fin.curve, parameter);
  if (!spline->axis.degree ||
      spline->axis.degree >= spline->axis.control_count ||
      spline->axis.knots.size() != spline->axis.multiplicities.size())
    return Return::Err(Unsupported("Periodic ring pcurve has invalid knots"));

  std::vector<double> knots;
  knots.reserve(uint64_t{spline->axis.control_count} + spline->axis.degree + 1);
  for (std::size_t i = 0; i < spline->axis.knots.size(); ++i)
    knots.insert(knots.end(), spline->axis.multiplicities[i],
                 spline->axis.knots[i]);
  if (knots.size() !=
      uint64_t{spline->axis.control_count} + spline->axis.degree + 1)
    return Return::Err(
        Unsupported("Periodic ring pcurve has an inconsistent knot vector"));
  const double start = knots[spline->axis.degree];
  const double end = knots[spline->axis.control_count];
  const double period = end - start;
  if (!std::isfinite(parameter) || !std::isfinite(period) || period <= 0)
    return Return::Err(Unsupported("Periodic ring pcurve has no valid period"));
  double wrapped = start + std::fmod(parameter - start, period);
  if (wrapped < start) wrapped += period;
  if (wrapped >= end) wrapped = start;
  // Evaluate the untrimmed SP_CURVE at an equivalent periodic parameter. The
  // trim's recorded unwrapped interval is retained for topology and export.
  if (!basis.value()->source.value)
    return Return::Err(Unsupported("Periodic SP_CURVE has no source identity"));
  return EvaluateCurve(model, basis.value()->source, wrapped);
}

Result<CircularEdgeRecovery> RecoverCylinderPlaneRing(
    const Model& model, std::span<const BoundFinGeometry* const> fins,
    double tolerance, bool use_source_tolerances, uint64_t& remaining) {
  using Return = Result<CircularEdgeRecovery>;
  if (fins.size() != 2 || !fins[0]->use.fin_curve.value ||
      !fins[1]->use.fin_curve.value || fins[0]->use.edge_curve.value ||
      fins[1]->use.edge_curve.value || fins[0]->use.edge != fins[1]->use.edge)
    return Return::Err(Unsupported(
        "Cylinder/plane recovery needs two fin-specific ring curves"));
  const BoundFinGeometry* cylinder_fin = nullptr;
  const BoundFinGeometry* plane_fin = nullptr;
  const CylinderGeometry* cylinder = nullptr;
  const PlaneGeometry* plane = nullptr;
  for (const auto* fin : fins) {
    if (fin->use.start_vertex != fin->use.end_vertex)
      return Return::Err(Unsupported(
          "Cylinder/plane circular recovery supports complete rings"));
    const auto support = Geometry(model, fin->surface);
    if (!support.ok()) return Return::Err(support.error());
    if (const auto* cylinder_support =
            std::get_if<CylinderGeometry>(&support.value()->payload)) {
      cylinder_fin = fin;
      cylinder = cylinder_support;
    } else if (const auto* plane_support =
                   std::get_if<PlaneGeometry>(&support.value()->payload)) {
      plane_fin = fin;
      plane = plane_support;
    }
  }
  if (!cylinder || !plane)
    return Return::Err(Unsupported("Expected a CYLINDER/PLANE support pair"));
  if (cylinder_fin->use.sense == plane_fin->use.sense)
    return Return::Err(
        Unsupported("Cylinder/plane ring uses require opposite directions"));
  const auto cylinder_curve = ReadParameterCurve(model, *cylinder_fin, true);
  const auto plane_curve = ReadParameterCurve(model, *plane_fin, true);
  if (!cylinder_curve.ok()) return Return::Err(cylinder_curve.error());
  if (!plane_curve.ok()) return Return::Err(plane_curve.error());
  const auto& cp = cylinder_curve.value().spline;
  if (cp.axis.degree != 1 || cp.axis.control_count != 2 ||
      cp.axis.knots.size() != 2 || cp.controls.rational ||
      cp.controls.dimension != 2 || cp.controls.values.size() != 4)
    return Return::Err(Unsupported(
        "Cylinder ring pcurve must be one affine full-turn segment"));
  double first = cp.controls.values[0], last = cp.controls.values[2];
  const double first_height = cp.controls.values[1];
  const double last_height = cp.controls.values[3];
  if (cylinder_fin->end < cylinder_fin->start) std::swap(first, last);
  if (cylinder_fin->use.sense == TopologySense::kReverse)
    std::swap(first, last);
  constexpr double kFullTurn = 2 * std::numbers::pi;
  if (!std::isfinite(first) || !std::isfinite(last) ||
      std::abs(std::abs(last - first) - kFullTurn) > 1e-12 ||
      std::abs(last_height - first_height) > tolerance / 4)
    return Return::Err(Unsupported(
        "Cylinder ring pcurve does not describe one constant-height turn"));

  const auto normal = plane->normal;
  const auto denominator = Dot(normal, cylinder->axis);
  const auto normal_length = std::hypot(normal.x, normal.y, normal.z);
  const auto axis_length =
      std::hypot(cylinder->axis.x, cylinder->axis.y, cylinder->axis.z);
  if (!std::isfinite(denominator) || !std::isfinite(normal_length) ||
      !std::isfinite(axis_length) || normal_length == 0 || axis_length == 0 ||
      std::abs(denominator) <= 64 * std::numeric_limits<double>::epsilon() *
                                   normal_length * axis_length)
    return Return::Err(Unsupported(
        "Cylinder axis is parallel to the plane and has no unique ring"));
  const auto height = (static_cast<long double>(normal.x) *
                           (plane->origin.x - cylinder->origin.x) +
                       static_cast<long double>(normal.y) *
                           (plane->origin.y - cylinder->origin.y) +
                       static_cast<long double>(normal.z) *
                           (plane->origin.z - cylinder->origin.z)) /
                      denominator;
  if (!std::isfinite(height) ||
      std::abs(height) > std::numeric_limits<double>::max())
    return Return::Err(
        Unsupported("Cylinder/plane ring center is unrepresentable"));
  const auto pcurve_height = std::midpoint(first_height, last_height);
  CircleGeometry circle;
  circle.center = {static_cast<double>(std::fma(
                       height, static_cast<long double>(cylinder->axis.x),
                       cylinder->origin.x)),
                   static_cast<double>(std::fma(
                       height, static_cast<long double>(cylinder->axis.y),
                       cylinder->origin.y)),
                   static_cast<double>(std::fma(
                       height, static_cast<long double>(cylinder->axis.z),
                       cylinder->origin.z))};
  circle.normal = cylinder->axis;
  circle.x_axis = cylinder->x_axis;
  circle.radius = cylinder->radius;
  if (!std::isfinite(circle.radius) || circle.radius <= 2 * tolerance)
    return Return::Err(Unsupported("Cylinder/plane ring radius is unresolved"));
  const auto pcurve_center = Vector3{
      static_cast<double>(std::fma(static_cast<long double>(pcurve_height),
                                   cylinder->axis.x, cylinder->origin.x)),
      static_cast<double>(std::fma(static_cast<long double>(pcurve_height),
                                   cylinder->axis.y, cylinder->origin.y)),
      static_cast<double>(std::fma(static_cast<long double>(pcurve_height),
                                   cylinder->axis.z, cylinder->origin.z))};
  if (std::hypot(circle.center.x - pcurve_center.x,
                 circle.center.y - pcurve_center.y,
                 circle.center.z - pcurve_center.z) > tolerance / 4)
    return Return::Err(Unsupported(
        "Cylinder pcurve height disagrees with the plane intersection"));
  const auto* edge = model.FindNode(cylinder_fin->use.edge);
  if (use_source_tolerances && edge && edge->edge_tolerance) {
    if (!std::isfinite(*edge->edge_tolerance) || *edge->edge_tolerance <= 0)
      return Return::Err({ErrorCode::kInvalidRecord, 0,
                          "Source edge tolerance must be positive and finite"});
    tolerance = std::max(tolerance, *edge->edge_tolerance);
  }
  // A shifted periodic trim's unwrapped interval does not determine the
  // natural direction of its closed pcurve. The cylinder pcurve supplies that
  // direction while the opposite fin senses retain face boundary orientation.
  const auto certified =
      CheckPlanarSpline(plane_curve.value().spline, *plane, circle, tolerance,
                        last - first, true, remaining);
  if (!certified.ok()) return Return::Err(certified.error());

  const auto transverse = Cross(circle.normal, circle.x_axis);
  const auto phase = first;
  circle.x_axis = {
      circle.x_axis.x * std::cos(phase) + transverse.x * std::sin(phase),
      circle.x_axis.y * std::cos(phase) + transverse.y * std::sin(phase),
      circle.x_axis.z * std::cos(phase) + transverse.z * std::sin(phase)};

  NurbsCurveGeometry cylinder_pcurve = cp;
  cylinder_pcurve.controls.values = {first, pcurve_height, last, pcurve_height};
  CircularEdgeRecovery result{
      cylinder_fin->use.edge, cylinder_fin->use.edge, circle, tolerance, {}};
  for (const auto* fin : fins) {
    const auto& pcurve =
        fin == cylinder_fin ? cylinder_pcurve : plane_curve.value().spline;
    const double edge_first = fin->use.sense == TopologySense::kForward
                                  ? 0.0
                                  : last - first,
                 edge_last = fin->use.sense == TopologySense::kForward
                                 ? last - first
                                 : 0.0;
    for (const auto& [parameter, edge_parameter] :
         {std::pair{fin->start, edge_first}, std::pair{fin->end, edge_last}}) {
      const auto source = EvaluateRingUse(model, *fin, parameter);
      if (!source.ok()) return Return::Err(source.error());
      const auto expected =
          EvaluateCurve(GeometryPayload{circle}, edge_parameter);
      if (!expected.ok()) return Return::Err(expected.error());
      if (std::hypot(source.value().position.x - expected.value().position.x,
                     source.value().position.y - expected.value().position.y,
                     source.value().position.z - expected.value().position.z) >
          tolerance)
        return Return::Err(Unsupported(
            "Cylinder/plane ring endpoints select different seams"));
    }
    result.uses.push_back(
        {fin->use.fin, fin->surface, edge_first, edge_last, pcurve});
  }
  return Return::Ok(std::move(result));
}

Result<CircularEdgeRecovery> RecoverPair(
    const Model& model, std::span<const BoundFinGeometry* const> fins,
    const std::map<uint32_t, Vector3>& vertices, double tolerance,
    bool use_source_tolerances, uint64_t& remaining) {
  using Return = Result<CircularEdgeRecovery>;
  if (fins.size() == 2) {
    const auto a = Geometry(model, fins[0]->surface),
               b = Geometry(model, fins[1]->surface);
    if (!a.ok()) return Return::Err(a.error());
    if (!b.ok()) return Return::Err(b.error());
    if ((std::holds_alternative<CylinderGeometry>(a.value()->payload) &&
         std::holds_alternative<NurbsSurfaceGeometry>(b.value()->payload)) ||
        (std::holds_alternative<CylinderGeometry>(b.value()->payload) &&
         std::holds_alternative<NurbsSurfaceGeometry>(a.value()->payload)))
      return RecoverCylinderNurbsEdge(model, fins, vertices, tolerance,
                                      use_source_tolerances, remaining);
    if ((std::holds_alternative<CylinderGeometry>(a.value()->payload) &&
         std::holds_alternative<PlaneGeometry>(b.value()->payload)) ||
        (std::holds_alternative<PlaneGeometry>(a.value()->payload) &&
         std::holds_alternative<CylinderGeometry>(b.value()->payload)))
      return RecoverCylinderPlaneRing(model, fins, tolerance,
                                      use_source_tolerances, remaining);
  }
  if (fins.size() != 2 || !fins[0]->use.fin_curve.value ||
      !fins[1]->use.fin_curve.value || fins[0]->use.edge_curve.value ||
      fins[1]->use.edge_curve.value)
    return Return::Err(
        Unsupported("Circular tolerant recovery requires two fin-specific "
                    "curves without an edge curve"));
  const bool ring = fins[0]->use.start_vertex == fins[0]->use.end_vertex;
  const BoundFinGeometry* planar = nullptr;
  const BoundFinGeometry* rotational = nullptr;
  const PlaneGeometry* plane = nullptr;
  const SpunSurfaceGeometry* spun = nullptr;
  for (auto* fin : fins) {
    if ((fin->use.start_vertex == fin->use.end_vertex) != ring ||
        (!ring && (!fin->use.start_vertex.value || !fin->use.end_vertex.value)))
      return Return::Err(Unsupported(
          "Tolerant circular edge uses disagree about their endpoints"));
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
  const auto pc = ReadParameterCurve(model, *planar, ring),
             rc = ReadParameterCurve(model, *rotational, ring);
  if (!pc.ok()) return Return::Err(pc.error());
  if (!rc.ok()) return Return::Err(rc.error());
  const auto& spline = rc.value().spline;
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
  constexpr double kFullTurn = 2 * std::numbers::pi;
  const auto span = last - first;
  if (!std::isfinite(first) || !std::isfinite(last) || first == last ||
      circle.radius * std::abs(span) <= tolerance ||
      (ring ? std::abs(std::abs(span) - kFullTurn) > 1e-12
            : std::abs(span) >= kFullTurn))
    return Return::Err(Unsupported(
        "Tolerant circular recovery needs one resolved arc or ring turn"));
  if (ring) {
    // Make the recorded seam parameter zero on the exact 3D master. This
    // preserves its spatial phase while avoiding roundoff in shifted 2*pi
    // interval subtraction. Each original surface pcurve keeps its own phase.
    const auto angle = std::remainder(first, kFullTurn);
    const auto x = circle.x_axis, perpendicular = Cross(circle.normal, x);
    circle.x_axis = {x.x * std::cos(angle) + perpendicular.x * std::sin(angle),
                     x.y * std::cos(angle) + perpendicular.y * std::sin(angle),
                     x.z * std::cos(angle) + perpendicular.z * std::sin(angle)};
    first = 0;
    last = span > 0 ? kFullTurn : -kFullTurn;
  }
  const auto* edge = model.FindNode(planar->use.edge);
  if (use_source_tolerances && edge && edge->edge_tolerance) {
    const auto transmitted = *edge->edge_tolerance;
    if (!std::isfinite(transmitted) || transmitted <= 0)
      return Return::Err({ErrorCode::kInvalidRecord, 0,
                          "Source edge tolerance must be positive and finite"});
    tolerance = std::max(tolerance, transmitted);
  }
  if (!ring && 2 * circle.radius * std::abs(std::sin((last - first) / 2)) <=
                   2 * tolerance)
    return Return::Err(
        Unsupported("Tolerant circular endpoint separation is ambiguous"));
  const auto desired_first =
      planar->use.sense == TopologySense::kForward ? first : last;
  const auto desired_last =
      planar->use.sense == TopologySense::kForward ? last : first;
  const auto certified = CheckPlanarSpline(
      pc.value().spline, *plane, circle, tolerance,
      desired_last - desired_first, planar->end > planar->start, remaining);
  if (!certified.ok()) return Return::Err(certified.error());
  CircularEdgeRecovery result{
      planar->use.edge, planar->use.edge, circle, tolerance, {}};
  for (auto* fin : fins) {
    const auto a = fin->use.sense == TopologySense::kForward ? first : last;
    const auto b = fin->use.sense == TopologySense::kForward ? last : first;
    for (const auto parameter : {a, b}) {
      const auto canonical = EvaluateCurve(GeometryPayload{circle}, parameter);
      const auto source =
          EvaluateRingUse(model, *fin, parameter == a ? fin->start : fin->end);
      if (!canonical.ok()) return Return::Err(canonical.error());
      if (!source.ok()) return Return::Err(source.error());
      const auto p = canonical.value().position, q = source.value().position;
      const auto vertex_id =
          parameter == a ? fin->use.start_vertex : fin->use.end_vertex;
      if (vertex_id.value) {
        const auto vertex = vertices.find(vertex_id.value);
        if (vertex == vertices.end())
          return Return::Err({ErrorCode::kInvalidRecord, 0,
                              "Recovered edge vertex is absent"});
        const auto v = vertex->second;
        if (std::hypot(p.x - v.x, p.y - v.y, p.z - v.z) > tolerance)
          return Return::Err(Unsupported(
              "Recovered circular edge exceeds its vertex tolerance"));
      }
      if (std::hypot(p.x - q.x, p.y - q.y, p.z - q.z) > tolerance)
        return Return::Err(Unsupported(
            "Tolerant circular source endpoints select different arcs"));
    }
    NurbsCurveGeometry pcurve =
        fin == planar ? pc.value().spline : rc.value().spline;
    pcurve.axis.closed = ring && fin == planar;
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
Result<std::vector<FinParameterInterval>> ResolveClosedTolerantRingIntervals(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> overrides, double tolerance,
    bool use_source_tolerances, const BodyGeometryBindingOptions& options,
    uint64_t& remaining_checks) {
  using Return = Result<std::vector<FinParameterInterval>>;
  if (!std::isfinite(tolerance) || tolerance <= 0 || !options.max_faces ||
      !options.max_fins || !options.max_vertices || !options.face.max_loops ||
      !options.face.max_fins)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Closed ring recovery requires positive bounds and tolerance"});
  const auto topology = ValidateBodyTopology(model, body, options.topology);
  if (!topology.ok()) return Return::Err(topology.error());
  std::map<uint32_t, const FinParameterInterval*> supplied;
  for (const auto& interval : overrides)
    if (!supplied.emplace(interval.fin.value, &interval).second)
      return Return::Err({ErrorCode::kInvalidOptions, 0,
                          "Duplicate closed-ring fin override"});
  std::set<uint32_t> faces;
  BodyGeometryBinding provisional;
  std::vector<FinParameterInterval> intervals;
  uint64_t visits = 0;
  for (const auto& region : topology.value().boundary.regions)
    for (const auto& shell : region.shells)
      for (const auto& face : shell.faces) {
        if (!faces.insert(face.face.value).second) continue;
        if (faces.size() > options.max_faces ||
            face.loops.size() > options.face.max_loops)
          return Return::Err({ErrorCode::kResourceLimit, 0,
                              "Closed-ring face limit exceeded"});
        FaceTrimming candidate{face.face, face.surface, face.face_sense, {}};
        uint64_t face_visits = 0;
        for (const auto& loop : face.loops) {
          const auto count = loop.isolated_vertex ? 1 : loop.fins.size();
          if (count > options.max_fins - visits ||
              count > options.face.max_fins - face_visits)
            return Return::Err({ErrorCode::kResourceLimit, 0,
                                "Closed-ring fin limit exceeded"});
          visits += count;
          face_visits += count;
          if (loop.isolated_vertex || loop.fins.size() != 1) continue;
          const auto& use = loop.fins.front();
          if (!use.fin_curve.value || use.start_vertex != use.end_vertex)
            continue;
          const auto geometry = Geometry(model, use.fin_curve);
          if (!geometry.ok()) return Return::Err(geometry.error());
          const auto* trim =
              std::get_if<TrimmedCurveGeometry>(&geometry.value()->payload);
          if (!trim || geometry.value()->sense != '+')
            return Return::Err(
                Unsupported("Closed tolerant rings require an explicit "
                            "positive-sense trim"));
          for (const auto& [parameter, point] :
               {std::pair{trim->parameter_1, trim->point_1},
                std::pair{trim->parameter_2, trim->point_2}}) {
            const BoundFinGeometry ring_fin{use,
                                            face.face,
                                            face.surface,
                                            use.fin_curve,
                                            trim->parameter_1,
                                            trim->parameter_2,
                                            options.intersection};
            const auto sample = EvaluateRingUse(model, ring_fin, parameter);
            if (!sample.ok()) return Return::Err(sample.error());
            const auto p = sample.value().position;
            if (std::hypot(p.x - point.x, p.y - point.y, p.z - point.z) >
                tolerance)
              return Return::Err({ErrorCode::kInvalidRecord, 0,
                                  "Stored closed-ring trim endpoint disagrees "
                                  "with its curve"});
          }
          const auto a = trim->point_1, b = trim->point_2;
          if (std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) > tolerance)
            return Return::Err(
                Unsupported("Closed tolerant trim endpoints do not close"));
          FinParameterInterval interval{use.fin, trim->parameter_1,
                                        trim->parameter_2};
          const auto found = supplied.find(use.fin.value);
          if (found != supplied.end())
            interval = *found->second;
          else
            intervals.push_back(interval);
          candidate.loops.push_back(
              {loop.loop,
               {BoundFinGeometry{use, face.face, face.surface, use.fin_curve,
                                 interval.start, interval.end,
                                 options.intersection}},
               {}});
        }
        if (!candidate.loops.empty())
          provisional.faces.push_back(std::move(candidate));
      }
  if (intervals.empty()) return Return::Ok({});
  if (topology.value().vertices.size() > options.max_vertices)
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Closed-ring vertex limit exceeded"});
  for (const auto& vertex : topology.value().vertices) {
    const auto point = Geometry(model, vertex.point);
    if (!point.ok()) return Return::Err(point.error());
    const auto* position = std::get_if<PointGeometry>(&point.value()->payload);
    if (!position)
      return Return::Err({ErrorCode::kInvalidRecord, 0,
                          "Closed-ring vertex has no point geometry"});
    provisional.vertices.push_back(
        {vertex.vertex, vertex.point, position->position});
  }
  // Stored trim order is only a candidate here. Full circular closure, both
  // pcurve windings and their paired source edge orientation must agree before
  // any interval is supplied to the reader. Final face/solid checks still
  // apply.
  const auto recovered = RecoverTolerantCircles(
      model, provisional, tolerance, use_source_tolerances, remaining_checks);
  if (!recovered.ok()) return Return::Err(recovered.error());
  return Return::Ok(std::move(intervals));
}
}  // namespace openswstep::internal
