// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "intersection_circle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>

#include "openparasolid/evaluate.h"
#include "spun_sphere.h"
namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vec = std::array<long double, 3>;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
Result<const GeometryObject*> Geometry(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  if (!node || !node->geometry || *node->geometry >= model.geometry().size())
    return Result<const GeometryObject*>::Err(
        Unsupported("Intersection support geometry is unavailable"));
  return Result<const GeometryObject*>::Ok(&model.geometry()[*node->geometry]);
}
long double Dot(Vec a, Vec b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
long double Norm(Vec a) { return std::hypot(a[0], a[1], a[2]); }
Vector3 Narrow(Vec a) {
  return {static_cast<double>(a[0]), static_cast<double>(a[1]),
          static_cast<double>(a[2])};
}
Result<SphereGeometry> Sphere(const Model& model, const GeometryObject& support,
                              double tolerance) {
  if (const auto* sphere = std::get_if<SphereGeometry>(&support.payload)) {
    const Vec axis{sphere->axis.x, sphere->axis.y, sphere->axis.z};
    const Vec x{sphere->x_axis.x, sphere->x_axis.y, sphere->x_axis.z};
    const auto frame_error = 4 * sphere->radius *
                             (std::abs(Norm(axis) - 1) + std::abs(Norm(x) - 1) +
                              std::abs(Dot(axis, x)));
    if (!std::isfinite(frame_error) || frame_error > tolerance)
      return Result<SphereGeometry>::Err(
          Unsupported("Spherical intersection frame exceeds its error budget"));
    return Result<SphereGeometry>::Ok(*sphere);
  }
  const auto* spun = std::get_if<SpunSurfaceGeometry>(&support.payload);
  if (!spun)
    return Result<SphereGeometry>::Err(
        Unsupported("Intersection circle requires a spherical support"));
  const auto profile = Geometry(model, spun->profile);
  if (!profile.ok()) return Result<SphereGeometry>::Err(profile.error());
  const auto* circle = std::get_if<CircleGeometry>(&profile.value()->payload);
  if (!circle)
    return Result<SphereGeometry>::Err(
        Unsupported("Spherical intersection requires a circular meridian"));
  const auto recognized = RecognizeSpunSphere(*spun, *circle, tolerance);
  if (!recognized.ok()) return Result<SphereGeometry>::Err(recognized.error());
  return Result<SphereGeometry>::Ok(recognized.value().geometry);
}
Result<CircleGeometry> Circle(const Model& model, const GeometryObject& source,
                              const BoundFinGeometry& fin, double tolerance) {
  const auto& intersection =
      std::get<IntersectionCurveGeometry>(source.payload);
  const auto a = Geometry(model, intersection.surfaces[0]);
  const auto b = Geometry(model, intersection.surfaces[1]);
  if (!a.ok()) return Result<CircleGeometry>::Err(a.error());
  if (!b.ok()) return Result<CircleGeometry>::Err(b.error());
  const auto* plane = std::get_if<PlaneGeometry>(&a.value()->payload);
  const GeometryObject* spherical = b.value();
  if (!plane) {
    plane = std::get_if<PlaneGeometry>(&b.value()->payload);
    spherical = a.value();
  }
  if (!plane)
    return Result<CircleGeometry>::Err(
        Unsupported("Intersection circle requires a planar support"));
  // Reserve separate portions of the unchanged tolerance for support
  // equivalence and rounded circle construction; never widen STEP uncertainty.
  const double budget = tolerance / 4;
  if (!(budget > 0))
    return Result<CircleGeometry>::Err(
        Unsupported("Intersection circle tolerance is unrepresentable"));
  const auto sphere = Sphere(model, *spherical, budget);
  if (!sphere.ok()) return Result<CircleGeometry>::Err(sphere.error());
  const auto valid_plane = EvaluateSurface(GeometryPayload{*plane}, 0, 0);
  const auto valid_sphere =
      EvaluateSurface(GeometryPayload{sphere.value()}, 0, 0);
  if (!valid_plane.ok())
    return Result<CircleGeometry>::Err(valid_plane.error());
  if (!valid_sphere.ok())
    return Result<CircleGeometry>::Err(valid_sphere.error());
  const auto& s = sphere.value();
  Vec n{plane->normal.x, plane->normal.y, plane->normal.z};
  const auto length = Norm(n);
  for (auto& v : n) v /= length;
  const Vec separation{static_cast<long double>(s.center.x) - plane->origin.x,
                       static_cast<long double>(s.center.y) - plane->origin.y,
                       static_cast<long double>(s.center.z) - plane->origin.z};
  const auto distance = Dot(separation, n);
  const long double radius_squared =
      (static_cast<long double>(s.radius) - std::abs(distance)) *
      (static_cast<long double>(s.radius) + std::abs(distance));
  if (!std::isfinite(radius_squared) || radius_squared <= 0)
    return Result<CircleGeometry>::Err(
        Unsupported("Sphere/plane intersection is not a regular circle"));
  const auto radius = std::sqrt(radius_squared);
  if (radius <= tolerance)
    return Result<CircleGeometry>::Err(Unsupported(
        "Intersection circle is unresolved at the source tolerance"));
  // A near-tangent cut amplifies support error by R/r. Re-certify the support
  // with that conditioning included, rather than spending a surface tolerance
  // directly as a curve tolerance.
  const double support_budget =
      static_cast<double>(budget * (radius / s.radius) / 8);
  if (!(support_budget > 0))
    return Result<CircleGeometry>::Err(
        Unsupported("Conditioned intersection tolerance is unrepresentable"));
  const auto certified = Sphere(model, *spherical, support_budget);
  if (!certified.ok()) return Result<CircleGeometry>::Err(certified.error());
  const Vec center{static_cast<long double>(s.center.x) - distance * n[0],
                   static_cast<long double>(s.center.y) - distance * n[1],
                   static_cast<long double>(s.center.z) - distance * n[2]};
  const auto seam = intersection.chart.positions.front();
  Vec x{static_cast<long double>(seam.x) - center[0],
        static_cast<long double>(seam.y) - center[1],
        static_cast<long double>(seam.z) - center[2]};
  const auto height = Dot(x, n);
  for (std::size_t i = 0; i < 3; ++i) x[i] -= height * n[i];
  const auto radial = Norm(x);
  if (!std::isfinite(radial) || radial == 0 || std::abs(height) > budget ||
      std::abs(radial - radius) > budget)
    return Result<CircleGeometry>::Err(
        Unsupported("Intersection help point disagrees with analytic circle"));
  for (auto& v : x) v /= radial;
  const auto sample = EvaluateIntersectionCurve(
      model, source.source, intersection.chart.base_parameter,
      *fin.intersection);
  if (!sample.ok()) return Result<CircleGeometry>::Err(sample.error());
  const Vec tangent{n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2],
                    n[0] * x[1] - n[1] * x[0]};
  const auto derivative = sample.value().first_derivative;
  const auto alignment =
      Dot(tangent, {derivative.x, derivative.y, derivative.z});
  if (!std::isfinite(alignment) || alignment == 0)
    return Result<CircleGeometry>::Err(
        Unsupported("Intersection circle direction is unresolved"));
  if (alignment < 0)
    for (auto& v : n) v = -v;
  CircleGeometry circle{Narrow(center), Narrow(n), Narrow(x),
                        static_cast<double>(radius)};
  const auto valid = EvaluateCurve(GeometryPayload{circle}, 0);
  if (!valid.ok()) return Result<CircleGeometry>::Err(valid.error());
  // Narrowing must also fit the construction budget, independently of chart
  // or topology tolerances and independently of the model coordinate scale.
  const auto rounded = circle.center;
  if (Norm({center[0] - rounded.x, center[1] - rounded.y,
            center[2] - rounded.z}) +
          std::abs(radius - circle.radius) >
      budget)
    return Result<CircleGeometry>::Err(
        Unsupported("Rounded intersection circle exceeds tolerance"));
  return Result<CircleGeometry>::Ok(circle);
}
}  // namespace
Result<std::vector<IntersectionCircleRecovery>> RecoverIntersectionCircles(
    const Model& model, const BodyGeometryBinding& body, double tolerance,
    uint64_t& remaining_checks) {
  using Return = Result<std::vector<IntersectionCircleRecovery>>;
  std::map<uint32_t, std::vector<const BoundFinGeometry*>> edges;
  for (const auto& face : body.faces)
    for (const auto& loop : face.loops)
      for (const auto& fin : loop.fins) {
        const auto geometry = Geometry(model, fin.curve);
        if (!geometry.ok()) return Return::Err(geometry.error());
        if (!std::holds_alternative<IntersectionCurveGeometry>(
                geometry.value()->payload))
          continue;
        if (loop.fins.size() != 1 || fin.use.fin_curve.value ||
            !fin.intersection)
          return Return::Err(Unsupported(
              "Intersection circle requires complete accurate ring loops"));
        edges[fin.use.edge.value].push_back(&fin);
      }
  std::vector<IntersectionCircleRecovery> result;
  for (const auto& [edge, uses] : edges) {
    if (uses.size() != 2)
      return Return::Err(
          Unsupported("Intersection circle requires two paired face uses"));
    const auto source = Geometry(model, uses[0]->curve);
    if (!source.ok()) return Return::Err(source.error());
    const auto& intersection =
        std::get<IntersectionCurveGeometry>(source.value()->payload);
    if (!source.value()->sense ||
        (*source.value()->sense != '+' && *source.value()->sense != '-') ||
        intersection.chart.positions.size() < 4 ||
        intersection.start.positions.size() != 1 ||
        intersection.end.positions.size() != 1 ||
        intersection.start.type != 'H' || intersection.end.type != 'H' ||
        intersection.start.source != intersection.end.source ||
        uses[0]->curve != uses[1]->curve ||
        uses[0]->use.sense == uses[1]->use.sense)
      return Return::Err(Unsupported(
          "Intersection circle has no certified complete shared-help branch"));
    const bool supported_faces =
        (uses[0]->surface == intersection.surfaces[0] &&
         uses[1]->surface == intersection.surfaces[1]) ||
        (uses[1]->surface == intersection.surfaces[0] &&
         uses[0]->surface == intersection.surfaces[1]);
    if (!supported_faces)
      return Return::Err(Unsupported(
          "Intersection circle face supports disagree with source"));
    const auto count = intersection.chart.positions.size();
    constexpr uint64_t kFrameChecks = 32;
    if (count > remaining_checks || kFrameChecks > remaining_checks - count)
      return Return::Err({ErrorCode::kResourceLimit, 0,
                          "Intersection circle comparison budget exhausted"});
    remaining_checks -= count + kFrameChecks;
    const auto& points = intersection.chart.positions;
    const auto seam = intersection.start.positions.front();
    const double seam_tolerance =
        std::min(tolerance / 4, uses[0]->intersection->chart_tolerance);
    const auto distance = [](Vector3 a, Vector3 b) {
      return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    };
    if (distance(points.front(), seam) > seam_tolerance ||
        distance(points.back(), seam) > seam_tolerance ||
        distance(intersection.end.positions.front(), seam) > seam_tolerance)
      return Return::Err(Unsupported(
          "Intersection circle chart does not close at its help seam"));
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
      if (distance(points[i], seam) <= seam_tolerance)
        return Return::Err(
            Unsupported("Intersection circle chart repeats its help seam"));
    const auto chart = EvaluateIntersectionChart(
        model, source.value()->source, uses[0]->intersection->chart_tolerance);
    if (!chart.ok()) return Return::Err(chart.error());
    const auto& parameters = chart.value().parameterization.parameters;
    const auto circle = Circle(model, *source.value(), *uses[0], tolerance);
    if (!circle.ok()) return Return::Err(circle.error());
    IntersectionCircleRecovery recovered{NodeId{edge}, circle.value(), {}};
    for (const auto* use : uses) {
      const bool increasing =
          use->start == parameters.front() && use->end == parameters.back();
      const bool decreasing =
          use->start == parameters.back() && use->end == parameters.front();
      if (!increasing && !decreasing)
        return Return::Err(Unsupported(
            "Intersection circle use does not span exactly its chart period"));
      recovered.uses.push_back({use->use.fin,
                                increasing ? 0 : 2 * std::numbers::pi,
                                increasing ? 2 * std::numbers::pi : 0});
    }
    result.push_back(std::move(recovered));
  }
  return Return::Ok(std::move(result));
}
}  // namespace openswstep::internal
