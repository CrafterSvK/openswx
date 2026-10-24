// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "annular_boundary.h"
#include "cone_patch.h"
#include "conic_boundary.h"
#include "convex_boundary.h"
#include "curved_hole.h"
#include "cylinder_band.h"
#include "cylinder_patch.h"
#include "intersection_circle.h"
#include "nurbs_wire.h"
#include "planar_boundary.h"
#include "sphere_boundary.h"
#include "spun_sphere.h"
#include "spun_torus.h"
#include "step_writer.h"
#include "tolerant_circle.h"
#include "torus_boundary.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
Vector3 Difference(Vector3 a, Vector3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
bool IsConicHole(std::span<const PlanarCurve> curves,
                 const PlaneGeometry& plane, bool natural_sense) {
  bool conic = false;
  for (const auto& curve : curves) {
    if (std::holds_alternative<LineGeometry>(*curve.geometry)) continue;
    const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
    const auto* ellipse = std::get_if<EllipseGeometry>(curve.geometry);
    if (!circle && !ellipse) return false;
    conic = true;
    const auto& a = circle ? circle->normal : ellipse->normal;
    const auto& b = plane.normal;
    const auto alignment = a.x * b.x + a.y * b.y + a.z * b.z;
    if ((curve.end - curve.start) * alignment * (natural_sense ? 1 : -1) >= 0)
      return false;
  }
  return conic;
}
// Winding selects a candidate route only. The mixed-boundary validator still
// proves the complete base region and every additional curved hole.
bool HasBasicOuter(std::span<const std::vector<Vector3>> polygons,
                   std::span<const ConicBoundary> circles,
                   const PlaneGeometry& plane, bool natural_sense) {
  const long double sign = natural_sense ? 1 : -1;
  for (const auto& circle : circles) {
    const auto& a = circle.normal;
    const auto& b = plane.normal;
    const auto alignment = static_cast<long double>(a.x) * b.x +
                           static_cast<long double>(a.y) * b.y +
                           static_cast<long double>(a.z) * b.z;
    if (sign * alignment * (circle.end - circle.start) > 0) return true;
  }
  for (const auto& polygon : polygons) {
    if (polygon.size() < 3) continue;
    const auto origin = polygon.front();
    long double area = 0;
    for (std::size_t i = 1; i + 1 < polygon.size(); ++i) {
      const std::array<long double, 3> a{
          static_cast<long double>(polygon[i].x) - origin.x,
          static_cast<long double>(polygon[i].y) - origin.y,
          static_cast<long double>(polygon[i].z) - origin.z};
      const std::array<long double, 3> b{
          static_cast<long double>(polygon[i + 1].x) - origin.x,
          static_cast<long double>(polygon[i + 1].y) - origin.y,
          static_cast<long double>(polygon[i + 1].z) - origin.z};
      area += (a[1] * b[2] - a[2] * b[1]) * plane.normal.x +
              (a[2] * b[0] - a[0] * b[2]) * plane.normal.y +
              (a[0] * b[1] - a[1] * b[0]) * plane.normal.z;
    }
    if (std::isfinite(area) && sign * area > 0) return true;
  }
  return false;
}
struct FaceData {
  const FaceTrimming* source = nullptr;
  const PlaneGeometry* plane = nullptr;
  const CylinderGeometry* cylinder = nullptr;
  const ConeGeometry* cone = nullptr;
  std::optional<TorusGeometry> torus;
  std::optional<SphereGeometry> sphere;
  bool spherical_spun = false;
  bool natural_sense = true;
  std::vector<std::vector<Vector3>> polygons;
  std::optional<std::size_t> outer_loop = 0;
  long double planar_area = 0;
  long double axial_height = 0;
  Vector3 reference_point;
  std::optional<CircularBand> cone_band;
  std::optional<TorusBand> torus_band;
  std::optional<SphereBand> sphere_band;
  std::optional<CylinderPatch> cylinder_patch;
  std::optional<CylinderBand> cylinder_band;
  std::optional<ConePatch> cone_patch;
};
struct EdgeUse {
  std::size_t face;
  bool forward;
};
struct FaceComponent {
  std::vector<std::size_t> faces;
  bool closed = true;
};

class BodyWriter {
 public:
  BodyWriter(const Model& model, const BodyGeometryBinding& body,
             const ExportOptions& options,
             std::span<const CircularEdgeRecovery> recoveries, uint64_t checks,
             std::span<const IntersectionCircleRecovery> intersections = {})
      : model_(model),
        body_(body),
        options_(options),
        writer_(options),
        polygon_checks_remaining_(checks) {
    for (const auto& recovery : intersections)
      recovered_curves_.emplace(
          recovery.edge.value,
          GeometryObject{recovery.edge, '+', recovery.circle});
    for (const auto& recovery : recoveries) {
      recovered_curves_.emplace(
          recovery.curve.value,
          GeometryObject{recovery.curve, '+', recovery.circle});
      recovered_edges_.emplace(recovery.curve.value, &recovery);
      writer_.IncludeSourceTolerance(recovery.tolerance);
    }
  }
  Result<StepDocument> Build();

 private:
  Result<const GeometryObject*> Geometry(NodeId id) const;
  Result<const GeometryObject*> Basis(NodeId id) const;
  ParseError Error(NodeId id, ErrorCode code, const char* message) const;
  Result<Id> Curve(NodeId id);
  Id Surface(const FaceData& face);
  Id ParameterCurve(const NurbsCurveGeometry& curve, Id surface);
  Result<Id> Edge(NodeId edge, NodeId curve, NodeId start, NodeId end,
                  double first, double last);
  Id Vertex(NodeId id);
  Result<bool> PrepareFaces();
  Result<bool> PrepareConicFace(FaceData& face);
  Result<bool> PrepareCurvedPlanarFace(FaceData& face);
  Result<bool> PrepareRuledPatchFace(FaceData& face);
  Result<bool> PrepareCylinderBandFace(FaceData& face);
  Result<ConicBoundary> ReadConicLoop(NodeId face, const FaceTrimLoop& loop);
  Result<std::vector<ConicBoundary>> ConicLoops(const FaceData& face);
  Result<bool> CheckVertexLinks();
  Result<bool> OrientFaces();
  Result<bool> CheckSolid();
  Result<StepDocument> Wires();
  Result<StepDocument> Faces();
  Result<Id> Face(std::size_t index);

  const Model& model_;
  const BodyGeometryBinding& body_;
  const ExportOptions& options_;
  StepWriter writer_;
  ExportStatistics statistics_;
  std::map<uint32_t, Vector3> points_;
  std::map<uint32_t, Id> vertices_;
  std::map<uint32_t, Id> edges_;
  std::map<uint32_t, Id> curves_;
  std::map<uint32_t, Id> surfaces_;
  std::map<uint32_t, GeometryObject> recovered_curves_;
  std::map<uint32_t, const CircularEdgeRecovery*> recovered_edges_;
  std::map<uint32_t, std::vector<EdgeUse>> uses_;
  std::vector<FaceData> faces_;
  std::vector<bool> reversed_;
  std::vector<FaceComponent> components_;
  uint64_t polygon_checks_remaining_;
  bool solid_ = false;
};
ParseError BodyWriter::Error(NodeId id, ErrorCode code,
                             const char* message) const {
  const auto* node = model_.FindNode(id);
  return {code, node ? node->source_offset : 0,
          "STEP node " + std::to_string(id.value) + ": " + message};
}
Result<const GeometryObject*> BodyWriter::Geometry(NodeId id) const {
  const auto* node = model_.FindNode(id);
  if (!node || !node->geometry || *node->geometry >= model_.geometry().size())
    return Result<const GeometryObject*>::Err(Error(
        id, ErrorCode::kUnsupportedFormat, "Typed geometry is unavailable"));
  return Result<const GeometryObject*>::Ok(&model_.geometry()[*node->geometry]);
}
Result<const GeometryObject*> BodyWriter::Basis(NodeId id) const {
  if (const auto found = recovered_curves_.find(id.value);
      found != recovered_curves_.end())
    return Result<const GeometryObject*>::Ok(&found->second);
  std::set<uint32_t> visited;
  for (unsigned depth = 0; depth <= 64; ++depth) {
    if (!visited.insert(id.value).second)
      return Result<const GeometryObject*>::Err(
          Error(id, ErrorCode::kInvalidRecord, "Cyclic trim chain"));
    auto object = Geometry(id);
    if (!object.ok()) return object;
    const auto* trim =
        std::get_if<TrimmedCurveGeometry>(&object.value()->payload);
    if (!trim) return object;
    id = trim->basis_curve;
  }
  return Result<const GeometryObject*>::Err(
      Error(id, ErrorCode::kResourceLimit, "Trim wrapper limit exceeded"));
}
Result<Id> BodyWriter::Curve(NodeId id) {
  const auto basis = Basis(id);
  if (!basis.ok()) return Result<Id>::Err(basis.error());
  const auto key = basis.value()->source.value;
  if (const auto found = curves_.find(key); found != curves_.end())
    return Result<Id>::Ok(found->second);
  const auto& geometry = basis.value()->payload;
  Id result = 0;
  if (const auto* line = std::get_if<LineGeometry>(&geometry)) {
    const auto origin = writer_.Point(line->origin);
    const auto direction = writer_.Direction(line->direction);
    const auto vector =
        writer_.Add("VECTOR(''," + Reference(direction) + ",1.)");
    result = writer_.Add("LINE(''," + Reference(origin) + ',' +
                         Reference(vector) + ')');
  } else if (const auto* circle = std::get_if<CircleGeometry>(&geometry)) {
    const auto frame =
        writer_.Placement(circle->center, circle->normal, circle->x_axis);
    result = writer_.Add("CIRCLE(''," + Reference(frame) + ',' +
                         writer_.Number(circle->radius) + ')');
  } else if (const auto* ellipse = std::get_if<EllipseGeometry>(&geometry)) {
    if (ellipse->major_radius < ellipse->minor_radius)
      return Result<Id>::Err(
          Error(id, ErrorCode::kUnsupportedFormat,
                "STEP ellipse requires ordered major/minor radii"));
    const auto frame =
        writer_.Placement(ellipse->center, ellipse->normal, ellipse->x_axis);
    result = writer_.Add("ELLIPSE(''," + Reference(frame) + ',' +
                         writer_.Number(ellipse->major_radius) + ',' +
                         writer_.Number(ellipse->minor_radius) + ')');
  } else if (const auto* spline = std::get_if<NurbsCurveGeometry>(&geometry)) {
    result = WriteNurbsCurve(writer_, *spline);
  } else {
    return Result<Id>::Err(Error(id, ErrorCode::kUnsupportedFormat,
                                 "Curve export requires LINE, CIRCLE, ELLIPSE "
                                 "or supported B_CURVE bases"));
  }
  if (const auto found = recovered_edges_.find(key);
      found != recovered_edges_.end()) {
    std::vector<Id> pcurves;
    for (const auto& use : found->second->uses) {
      const auto face =
          std::find_if(faces_.begin(), faces_.end(), [&](const auto& value) {
            return value.source->surface == use.surface;
          });
      if (face == faces_.end())
        return Result<Id>::Err(Error(use.surface, ErrorCode::kInvalidRecord,
                                     "Recovered pcurve surface is absent"));
      pcurves.push_back(ParameterCurve(use.pcurve, Surface(*face)));
    }
    result = writer_.Add("SURFACE_CURVE(''," + Reference(result) + ',' +
                         References(pcurves) + ",.CURVE_3D.)");
  }
  curves_.emplace(key, result);
  return Result<Id>::Ok(result);
}
Id BodyWriter::Vertex(NodeId id) {
  if (const auto found = vertices_.find(id.value); found != vertices_.end())
    return found->second;
  // Complete body validation/binding established every selected vertex point.
  const auto point = writer_.Point(points_.find(id.value)->second);
  const auto vertex = writer_.Add("VERTEX_POINT(''," + Reference(point) + ')');
  vertices_.emplace(id.value, vertex);
  ++statistics_.vertices;
  return vertex;
}
Result<Id> BodyWriter::Edge(NodeId edge, NodeId curve, NodeId start, NodeId end,
                            double first, double last) {
  if (const auto found = edges_.find(edge.value); found != edges_.end())
    return Result<Id>::Ok(found->second);
  const auto basis = Basis(curve);
  if (!basis.ok()) return Result<Id>::Err(basis.error());
  const bool conic =
      std::holds_alternative<CircleGeometry>(basis.value()->payload) ||
      std::holds_alternative<EllipseGeometry>(basis.value()->payload);
  std::optional<NurbsCurveGeometry> restricted;
  if (const auto* spline =
          std::get_if<NurbsCurveGeometry>(&basis.value()->payload)) {
    auto section =
        RestrictNurbsWire(*spline, first, last, polygon_checks_remaining_);
    if (!section.ok())
      return Result<Id>::Err(
          Error(edge, section.error().code, section.error().message.c_str()));
    restricted = std::move(section).value();
    const auto valid =
        ValidateNurbsWire(*restricted, first, last, options_.tolerance,
                          polygon_checks_remaining_);
    if (!valid.ok())
      return Result<Id>::Err(
          Error(edge, valid.error().code, valid.error().message.c_str()));
    // Knot insertion is algebraically exact, but its stored doubles must
    // still meet the actual source vertices' length tolerance after rounding.
    for (const auto& [vertex, parameter] :
         {std::pair{start, first}, std::pair{end, last}}) {
      const auto position = points_.find(vertex.value);
      if (position == points_.end())
        return Result<Id>::Err(
            Error(edge, ErrorCode::kUnsupportedFormat,
                  "Spline wire endpoints require named source vertices"));
      const auto sample = EvaluateCurve(*restricted, parameter);
      if (!sample.ok()) return Result<Id>::Err(sample.error());
      const auto& p = sample.value().position;
      const auto& q = position->second;
      if (std::hypot(static_cast<long double>(p.x) - q.x,
                     static_cast<long double>(p.y) - q.y,
                     static_cast<long double>(p.z) - q.z) > options_.tolerance)
        return Result<Id>::Err(Error(
            edge, ErrorCode::kUnsupportedFormat,
            "Restricted spline endpoints exceed source length tolerance"));
    }
  }
  if (conic && std::abs(last - first) > 2 * std::numbers::pi)
    return Result<Id>::Err(
        Error(edge, ErrorCode::kUnsupportedFormat,
              "Multiple conic turns cannot be represented by one EDGE_CURVE"));
  // A selected spline section belongs to this source edge. Distinct intervals
  // on one basis must never reuse the other edge's restricted geometry.
  const auto geometry =
      restricted ? Result<Id>::Ok(WriteNurbsCurve(writer_, *restricted))
                 : Curve(curve);
  if (!geometry.ok()) return geometry;
  Id start_id = 0, end_id = 0;
  if (start.value == 0 && end.value == 0) {
    if (!conic ||
        std::abs(std::abs(last - first) - 2 * std::numbers::pi) > 1e-12)
      return Result<Id>::Err(
          Error(edge, ErrorCode::kUnsupportedFormat,
                "Vertex-free edges require one complete conic turn"));
    const auto sample = EvaluateCurve(basis.value()->payload, first);
    if (!sample.ok()) return Result<Id>::Err(sample.error());
    const auto point = writer_.Point(sample.value().position);
    start_id = end_id =
        writer_.Add("VERTEX_POINT(''," + Reference(point) + ')');
    ++statistics_.vertices;
  } else {
    start_id = Vertex(start);
    end_id = Vertex(end);
  }
  const auto result = writer_.Add(
      "EDGE_CURVE(''," + Reference(start_id) + ',' + Reference(end_id) + ',' +
      Reference(geometry.value()) + ',' + Logical(last > first) + ')');
  edges_.emplace(edge.value, result);
  ++statistics_.edges;
  return Result<Id>::Ok(result);
}
Result<ConicBoundary> BodyWriter::ReadConicLoop(NodeId face,
                                                const FaceTrimLoop& loop) {
  using Return = Result<ConicBoundary>;
  if (loop.isolated_vertex || loop.fins.size() != 1)
    return Return::Err(Error(
        face, ErrorCode::kUnsupportedFormat,
        "Conic face loops require one complete CIRCLE or ELLIPSE edge each"));
  const auto& fin = loop.fins.front();
  if (fin.use.fin_curve.value)
    return Return::Err(
        Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
              "Tolerant fin curves need an explicit STEP pcurve mapping"));
  if (fin.use.start_vertex != fin.use.end_vertex)
    return Return::Err(Error(
        fin.use.fin, ErrorCode::kInvalidRecord,
        "Complete conic loop must use one seam vertex or be vertex-free"));
  const auto curve = Basis(fin.curve);
  if (!curve.ok()) return Return::Err(curve.error());
  uses_[fin.use.edge.value].push_back(
      {faces_.size(), fin.use.sense == TopologySense::kForward});
  if (const auto* circle =
          std::get_if<CircleGeometry>(&curve.value()->payload)) {
    return Return::Ok({circle->center, circle->normal, circle->x_axis,
                       circle->radius, circle->radius, fin.start, fin.end,
                       true});
  } else if (const auto* ellipse =
                 std::get_if<EllipseGeometry>(&curve.value()->payload)) {
    return Return::Ok({ellipse->center, ellipse->normal, ellipse->x_axis,
                       ellipse->major_radius, ellipse->minor_radius, fin.start,
                       fin.end, false});
  } else {
    return Return::Err(
        Error(fin.curve, ErrorCode::kUnsupportedFormat,
              "Single-edge conic loops require CIRCLE or ELLIPSE bases"));
  }
}
Result<std::vector<ConicBoundary>> BodyWriter::ConicLoops(
    const FaceData& face) {
  using Return = Result<std::vector<ConicBoundary>>;
  std::vector<ConicBoundary> loops;
  for (const auto& loop : face.source->loops) {
    if (face.torus && !loop.isolated_vertex && loop.fins.size() > 1) {
      std::vector<ConicBoundary> arcs;
      std::set<uint32_t> vertices;
      for (const auto& fin : loop.fins) {
        if (fin.use.fin_curve.value)
          return Return::Err(Error(
              fin.use.fin, ErrorCode::kUnsupportedFormat,
              "Tolerant fin curves need an explicit STEP pcurve mapping"));
        if (!fin.use.start_vertex.value || !fin.use.end_vertex.value ||
            !vertices.insert(fin.use.start_vertex.value).second)
          return Return::Err(
              Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
                    "Split torus cuts need distinct non-null vertices"));
        const auto basis = Basis(fin.curve);
        if (!basis.ok()) return Return::Err(basis.error());
        const auto* circle =
            std::get_if<CircleGeometry>(&basis.value()->payload);
        if (!circle)
          return Return::Err(Error(fin.curve, ErrorCode::kUnsupportedFormat,
                                   "Split torus cuts require CIRCLE arcs"));
        arcs.push_back({circle->center, circle->normal, circle->x_axis,
                        circle->radius, circle->radius, fin.start, fin.end,
                        true});
        uses_[fin.use.edge.value].push_back(
            {faces_.size(), fin.use.sense == TopologySense::kForward});
      }
      const auto cut = CombineCircularCut(arcs, options_.tolerance,
                                          polygon_checks_remaining_);
      if (!cut.ok())
        return Return::Err(Error(face.source->face, cut.error().code,
                                 cut.error().message.c_str()));
      loops.push_back(cut.value());
      continue;
    }
    const auto conic = ReadConicLoop(face.source->face, loop);
    if (!conic.ok()) return Return::Err(conic.error());
    loops.push_back(conic.value());
  }
  return Return::Ok(std::move(loops));
}
Result<bool> BodyWriter::PrepareConicFace(FaceData& face) {
  const auto loops = ConicLoops(face);
  if (!loops.ok()) return Result<bool>::Err(loops.error());
  if (face.sphere) {
    if (face.spherical_spun) {
      // The spherical support is independent of its polar frame. Align the
      // canonical frame with the saved cut, even for a cut oblique to SPUN's
      // rotation axis. Source circle edges and outward normal stay unchanged.
      face.sphere->axis = loops.value().front().normal;
      face.sphere->x_axis = loops.value().front().x_axis;
    }
    const auto band =
        ValidateSphereLoops(loops.value(), *face.sphere, face.natural_sense,
                            options_.tolerance, polygon_checks_remaining_);
    if (!band.ok())
      return Result<bool>::Err(Error(face.source->face, band.error().code,
                                     band.error().message.c_str()));
    face.sphere_band = band.value();
    face.reference_point = loops.value().front().center;
    face.outer_loop = std::nullopt;
    return Result<bool>::Ok(true);
  }
  if (face.torus) {
    const auto band =
        ValidateTorusLoops(loops.value(), *face.torus, face.natural_sense,
                           options_.tolerance, polygon_checks_remaining_);
    if (!band.ok())
      return Result<bool>::Err(Error(face.source->face, band.error().code,
                                     band.error().message.c_str()));
    face.torus_band = band.value();
    face.reference_point = loops.value().front().center;
    face.outer_loop = std::nullopt;
    return Result<bool>::Ok(true);
  }
  if (face.cone) {
    const auto band =
        ValidateConeLoops(loops.value(), *face.cone, face.natural_sense,
                          options_.tolerance, polygon_checks_remaining_);
    if (!band.ok())
      return Result<bool>::Err(Error(face.source->face, band.error().code,
                                     band.error().message.c_str()));
    face.cone_band = band.value();
    face.reference_point = band.value().lower_center;
    face.outer_loop = std::nullopt;
    return Result<bool>::Ok(true);
  }
  if (face.cylinder) {
    const auto height =
        ValidateCylinderLoops(loops.value(), *face.cylinder, face.natural_sense,
                              options_.tolerance, polygon_checks_remaining_);
    if (!height.ok())
      return Result<bool>::Err(Error(face.source->face, height.error().code,
                                     height.error().message.c_str()));
    face.axial_height = height.value();
    face.reference_point = loops.value().front().center;
    face.outer_loop = std::nullopt;
    return Result<bool>::Ok(true);
  }
  const auto boundary =
      ValidatePlanarConics(loops.value(), *face.plane, face.natural_sense,
                           options_.tolerance, polygon_checks_remaining_);
  if (!boundary.ok())
    return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                   boundary.error().message.c_str()));
  face.outer_loop = boundary.value();
  face.reference_point = loops.value()[boundary.value()].center;
  for (std::size_t i = 0; i < loops.value().size(); ++i) {
    const auto& loop = loops.value()[i];
    const auto area =
        std::numbers::pi_v<long double> * loop.major_radius * loop.minor_radius;
    face.planar_area += i == boundary.value() ? area : -area;
  }
  if (!std::isfinite(face.planar_area) || face.planar_area <= 0)
    return Result<bool>::Err(
        Error(face.source->face, ErrorCode::kInvalidRecord,
              "Conic face area is nonpositive or unrepresentable"));
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::PrepareCurvedPlanarFace(FaceData& face) {
  std::vector<std::size_t> circle_indices, polygon_indices, curved_indices;
  std::vector<std::vector<PlanarCurve>> curved_loops;
  std::vector<ConicBoundary> holes;
  std::vector<std::vector<Vector3>> polygons;
  for (std::size_t i = 0; i < face.source->loops.size(); ++i) {
    const auto& loop = face.source->loops[i];
    if (loop.fins.size() == 1) {
      const auto hole = ReadConicLoop(face.source->face, loop);
      if (!hole.ok()) return Result<bool>::Err(hole.error());
      holes.push_back(hole.value());
      circle_indices.push_back(i);
      continue;
    }
    if (loop.isolated_vertex || loop.fins.empty())
      return Result<bool>::Err(
          Error(face.source->face, ErrorCode::kUnsupportedFormat,
                "Curved face loops require fin boundaries"));
    std::set<uint32_t> vertices;
    std::vector<PlanarCurve> candidate;
    bool curved = false;
    for (const auto& fin : loop.fins) {
      if (fin.use.fin_curve.value)
        return Result<bool>::Err(
            Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
                  "Tolerant fin curves need an explicit STEP pcurve mapping"));
      if (!fin.use.start_vertex.value || !fin.use.end_vertex.value ||
          !vertices.insert(fin.use.start_vertex.value).second)
        return Result<bool>::Err(Error(face.source->face,
                                       ErrorCode::kUnsupportedFormat,
                                       "Multi-edge planar loops require "
                                       "distinct non-null source vertices"));
      const auto basis = Basis(fin.curve);
      if (!basis.ok()) return Result<bool>::Err(basis.error());
      candidate.push_back({&basis.value()->payload, fin.start, fin.end});
      curved = curved ||
               !std::holds_alternative<LineGeometry>(basis.value()->payload);
      uses_[fin.use.edge.value].push_back(
          {faces_.size(), fin.use.sense == TopologySense::kForward});
    }
    if (curved) {
      curved_indices.push_back(i);
      curved_loops.push_back(std::move(candidate));
    } else {
      std::vector<Vector3> polygon;
      for (const auto& fin : loop.fins) {
        const auto point = points_.find(fin.use.start_vertex.value);
        if (point == points_.end())
          return Result<bool>::Err(Error(fin.use.start_vertex,
                                         ErrorCode::kInvalidRecord,
                                         "Bound vertex point is unavailable"));
        polygon.push_back(point->second);
      }
      polygons.push_back(std::move(polygon));
      polygon_indices.push_back(i);
    }
  }
  const bool inward =
      !curved_loops.empty() &&
      std::all_of(curved_loops.begin(), curved_loops.end(),
                  [&face](const auto& loop) {
                    return IsConicHole(loop, *face.plane, face.natural_sense);
                  });
  bool spline = false;
  for (const auto& loop : curved_loops)
    for (const auto& curve : loop)
      spline =
          spline || std::holds_alternative<NurbsCurveGeometry>(*curve.geometry);
  const bool spline_holes =
      spline && HasBasicOuter(polygons, holes, *face.plane, face.natural_sense);
  if ((inward || spline_holes) && face.source->loops.size() > 1) {
    const auto region = ValidateConvexCurvedHoles(
        curved_loops, polygons, holes, *face.plane, face.natural_sense,
        options_.tolerance, polygon_checks_remaining_);
    if (!region.ok())
      return Result<bool>::Err(Error(face.source->face, region.error().code,
                                     region.error().message.c_str()));
    face.outer_loop = region.value().circular_outer
                          ? circle_indices[region.value().outer_loop]
                          : polygon_indices[region.value().outer_loop];
    face.planar_area = region.value().area;
    face.reference_point =
        polygons.empty() ? holes.front().center : polygons.front().front();
    return Result<bool>::Ok(true);
  }
  if (curved_loops.size() != 1)
    return Result<bool>::Err(Error(face.source->face,
                                   ErrorCode::kUnsupportedFormat,
                                   "Curved planar faces require one curved "
                                   "outer contour or convex curved holes"));
  const auto outer_index = curved_indices.front();
  const auto& curves = curved_loops.front();
  if (!holes.empty() || !polygons.empty()) {
    const auto boundary = ValidateConvexPlanarHoles(
        curves, polygons, holes, *face.plane, face.natural_sense,
        options_.tolerance, polygon_checks_remaining_);
    if (!boundary.ok())
      return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                     boundary.error().message.c_str()));
    face.planar_area = boundary.value().area;
    face.reference_point = boundary.value().reference_point;
    face.outer_loop = outer_index;
    return Result<bool>::Ok(true);
  }
  bool reverse_circle = false;
  for (const auto& curve : curves) {
    const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
    if (!circle) continue;
    const auto& a = circle->normal;
    const auto& b = face.plane->normal;
    const auto alignment = a.x * b.x + a.y * b.y + a.z * b.z;
    reverse_circle = reverse_circle || (curve.end - curve.start) * alignment *
                                               (face.natural_sense ? 1 : -1) <
                                           0;
  }
  if (curves.size() == 4 && reverse_circle) {
    const auto boundary =
        ValidateAnnularSector(curves, *face.plane, face.natural_sense,
                              options_.tolerance, polygon_checks_remaining_);
    if (!boundary.ok())
      return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                     boundary.error().message.c_str()));
    face.planar_area = boundary.value().area;
    face.reference_point = boundary.value().reference_point;
    face.outer_loop = outer_index;
    return Result<bool>::Ok(true);
  }
  auto boundary =
      ValidateConvexPlanarLoop(curves, *face.plane, face.natural_sense,
                               options_.tolerance, polygon_checks_remaining_);
  if (!boundary.ok() && boundary.error().code == ErrorCode::kUnsupportedFormat) {
    // The minor-arc certificate cannot validate ellipses or splines. Keep the
    // original failure instead of replacing it with an inapplicable route's
    // type restriction, and identify the source curve for further inspection.
    for (std::size_t i = 0; i < curves.size(); ++i) {
      const auto& geometry = *curves[i].geometry;
      if (std::holds_alternative<LineGeometry>(geometry) ||
          std::holds_alternative<CircleGeometry>(geometry))
        continue;
      const auto& fin = face.source->loops[outer_index].fins[i];
      const auto message =
          boundary.error().message +
          "; minor-arc fallback unavailable for source curve " +
          std::to_string(fin.curve.value) + " (" + GeometryName(geometry) +
          "), edge " + std::to_string(fin.use.edge.value);
      return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                     message.c_str()));
    }
    if (curves.size() >= 3)
      boundary = ValidateMinorArcPlanarLoop(
          curves, *face.plane, face.natural_sense, options_.tolerance,
          polygon_checks_remaining_);
  }
  if (!boundary.ok())
    return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                   boundary.error().message.c_str()));
  face.planar_area = boundary.value().area;
  face.reference_point = boundary.value().reference_point;
  face.outer_loop = outer_index;
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::PrepareCylinderBandFace(FaceData& face) {
  std::vector<std::vector<PlanarCurve>> loops;
  for (const auto& source : face.source->loops) {
    if (source.isolated_vertex)
      return Result<bool>::Err(
          Error(face.source->face, ErrorCode::kUnsupportedFormat,
                "Cylinder band profiles require fin loops"));
    std::set<uint32_t> vertices;
    std::vector<PlanarCurve> curves;
    for (const auto& fin : source.fins) {
      if (fin.use.fin_curve.value)
        return Result<bool>::Err(
            Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
                  "Tolerant fin curves need an explicit STEP pcurve mapping"));
      if (source.fins.size() > 1 &&
          (!fin.use.start_vertex.value || !fin.use.end_vertex.value ||
           !vertices.insert(fin.use.start_vertex.value).second))
        return Result<bool>::Err(Error(
            face.source->face, ErrorCode::kUnsupportedFormat,
            "Multi-edge cylinder profiles require distinct non-null vertices"));
      const auto basis = Basis(fin.curve);
      if (!basis.ok()) return Result<bool>::Err(basis.error());
      curves.push_back({&basis.value()->payload, fin.start, fin.end});
      uses_[fin.use.edge.value].push_back(
          {faces_.size(), fin.use.sense == TopologySense::kForward});
    }
    loops.push_back(std::move(curves));
  }
  const auto band =
      ValidateCylinderBand(loops, *face.cylinder, face.natural_sense,
                           options_.tolerance, polygon_checks_remaining_);
  if (!band.ok())
    return Result<bool>::Err(Error(face.source->face, band.error().code,
                                   band.error().message.c_str()));
  face.cylinder_band = band.value();
  face.reference_point = band.value().reference_point;
  face.outer_loop = std::nullopt;
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::PrepareRuledPatchFace(FaceData& face) {
  if (face.source->loops.size() != 1 ||
      face.source->loops.front().isolated_vertex)
    return Result<bool>::Err(
        Error(face.source->face, ErrorCode::kUnsupportedFormat,
              "Ruled patches require one non-isolated loop"));
  std::set<uint32_t> vertices;
  std::vector<PlanarCurve> curves;
  for (const auto& fin : face.source->loops.front().fins) {
    if (fin.use.fin_curve.value)
      return Result<bool>::Err(
          Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
                "Tolerant fin curves need an explicit STEP pcurve mapping"));
    if (!fin.use.start_vertex.value || !fin.use.end_vertex.value ||
        !vertices.insert(fin.use.start_vertex.value).second)
      return Result<bool>::Err(
          Error(face.source->face, ErrorCode::kUnsupportedFormat,
                "Ruled patch corners require distinct non-null vertices"));
    const auto basis = Basis(fin.curve);
    if (!basis.ok()) return Result<bool>::Err(basis.error());
    curves.push_back({&basis.value()->payload, fin.start, fin.end});
    uses_[fin.use.edge.value].push_back(
        {faces_.size(), fin.use.sense == TopologySense::kForward});
  }
  if (face.cone) {
    const auto patch =
        ValidateConePatch(curves, *face.cone, face.natural_sense,
                          options_.tolerance, polygon_checks_remaining_);
    if (!patch.ok())
      return Result<bool>::Err(Error(face.source->face, patch.error().code,
                                     patch.error().message.c_str()));
    face.cone_patch = patch.value();
    face.reference_point = patch.value().reference_point;
  } else {
    const auto patch =
        ValidateCylinderPatch(curves, *face.cylinder, face.natural_sense,
                              options_.tolerance, polygon_checks_remaining_);
    if (!patch.ok())
      return Result<bool>::Err(Error(face.source->face, patch.error().code,
                                     patch.error().message.c_str()));
    face.cylinder_patch = patch.value();
    face.reference_point = patch.value().reference_point;
  }
  face.outer_loop = 0;
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::PrepareFaces() {
  for (const auto& source : body_.faces) {
    if (source.loops.empty())
      return Result<bool>::Err(Error(source.face, ErrorCode::kUnsupportedFormat,
                                     "Face has no boundary loops"));
    const auto surface = Geometry(source.surface);
    if (!surface.ok()) return Result<bool>::Err(surface.error());
    const auto* plane = std::get_if<PlaneGeometry>(&surface.value()->payload);
    const auto* cylinder =
        std::get_if<CylinderGeometry>(&surface.value()->payload);
    const auto* cone = std::get_if<ConeGeometry>(&surface.value()->payload);
    const auto* torus = std::get_if<TorusGeometry>(&surface.value()->payload);
    const auto* sphere = std::get_if<SphereGeometry>(&surface.value()->payload);
    std::optional<SpunTorus> spun_torus;
    std::optional<SpunSphere> spun_sphere;
    if (const auto* spun =
            std::get_if<SpunSurfaceGeometry>(&surface.value()->payload)) {
      const auto profile = Geometry(spun->profile);
      if (!profile.ok()) return Result<bool>::Err(profile.error());
      const auto* circle =
          std::get_if<CircleGeometry>(&profile.value()->payload);
      if (!circle)
        return Result<bool>::Err(
            Error(source.face, ErrorCode::kUnsupportedFormat,
                  "Spun surface export requires a CIRCLE generator"));
      const auto spherical =
          RecognizeSpunSphere(*spun, *circle, options_.tolerance);
      if (spherical.ok()) {
        spun_sphere = spherical.value();
      } else {
        // The torus route excludes all bounded profiles. Do not replace a
        // spherical-domain or invalid-frame failure with its unrelated error.
        if (spherical.error().code != ErrorCode::kUnsupportedFormat ||
            spun->start_parameter || spun->end_parameter)
          return Result<bool>::Err(Error(source.face, spherical.error().code,
                                         spherical.error().message.c_str()));
        const auto recognized =
            RecognizeSpunTorus(*spun, *circle, options_.tolerance);
        if (!recognized.ok())
          return Result<bool>::Err(Error(source.face, recognized.error().code,
                                         recognized.error().message.c_str()));
        spun_torus = recognized.value();
      }
    }
    if (!plane && !cylinder && !cone && !torus && !sphere && !spun_torus &&
        !spun_sphere)
      return Result<bool>::Err(
          Error(source.face, ErrorCode::kUnsupportedFormat,
                "Face export requires PLANE/CYLINDER/CONE/SPHERE "
                "or ring TORUS surfaces"));
    if (!source.face_sense ||
        (*source.face_sense != '+' && *source.face_sense != '-') ||
        !surface.value()->sense ||
        (*surface.value()->sense != '+' && *surface.value()->sense != '-'))
      return Result<bool>::Err(Error(source.face, ErrorCode::kUnsupportedFormat,
                                     "Face and surface senses must be known"));
    FaceData face;
    face.source = &source;
    face.plane = plane;
    face.cylinder = cylinder;
    face.cone = cone;
    if (torus) face.torus = *torus;
    if (spun_torus) face.torus = spun_torus->geometry;
    if (sphere) face.sphere = *sphere;
    if (spun_sphere) {
      face.sphere = spun_sphere->geometry;
      face.spherical_spun = true;
    }
    face.natural_sense = source.face_sense == surface.value()->sense;
    if (spun_sphere && !spun_sphere->same_normal)
      face.natural_sense = !face.natural_sense;
    if (spun_torus && !spun_torus->same_normal)
      face.natural_sense = !face.natural_sense;
    const bool all_conic =
        std::all_of(source.loops.begin(), source.loops.end(),
                    [](const auto& loop) { return loop.fins.size() == 1; });
    bool profiled_cylinder = cylinder && source.loops.size() == 2 && !all_conic;
    if (cylinder && source.loops.size() == 2 && all_conic)
      for (const auto& loop : source.loops) {
        const auto basis = Basis(loop.fins.front().curve);
        if (!basis.ok()) return Result<bool>::Err(basis.error());
        profiled_cylinder =
            profiled_cylinder ||
            !std::holds_alternative<CircleGeometry>(basis.value()->payload);
      }
    if (profiled_cylinder) {
      const auto band = PrepareCylinderBandFace(face);
      if (!band.ok()) return band;
      faces_.push_back(std::move(face));
      continue;
    }
    if ((cylinder || cone) && !all_conic) {
      const auto patch = PrepareRuledPatchFace(face);
      if (!patch.ok()) return patch;
      faces_.push_back(std::move(face));
      continue;
    }
    if (cylinder || cone || face.torus || sphere || all_conic) {
      const auto conic = PrepareConicFace(face);
      if (!conic.ok()) return conic;
      faces_.push_back(std::move(face));
      continue;
    }
    if (plane) {
      bool curved = false;
      for (const auto& loop : source.loops) {
        if (loop.fins.size() == 1) continue;
        for (const auto& fin : loop.fins) {
          const auto basis = Basis(fin.curve);
          if (!basis.ok()) return Result<bool>::Err(basis.error());
          curved = curved || !std::holds_alternative<LineGeometry>(
                                 basis.value()->payload);
        }
      }
      if (curved) {
        const auto boundary = PrepareCurvedPlanarFace(face);
        if (!boundary.ok()) return boundary;
        faces_.push_back(std::move(face));
        continue;
      }
    }
    std::vector<ConicBoundary> circles;
    std::vector<std::size_t> circle_indices;
    std::vector<std::size_t> polygon_indices;
    for (std::size_t i = 0; i < source.loops.size(); ++i) {
      const auto& loop = source.loops[i];
      if (loop.fins.size() == 1) {
        const auto conic = ReadConicLoop(source.face, loop);
        if (!conic.ok()) return Result<bool>::Err(conic.error());
        circles.push_back(conic.value());
        circle_indices.push_back(i);
        continue;
      }
      if (loop.isolated_vertex || loop.fins.size() < 3)
        return Result<bool>::Err(
            Error(source.face, ErrorCode::kUnsupportedFormat,
                  "Planar polygon loops require at least three line edges"));
      std::set<uint32_t> vertices;
      std::vector<Vector3> polygon;
      for (const auto& fin : loop.fins) {
        if (fin.use.fin_curve.value != 0)
          return Result<bool>::Err(Error(
              fin.use.fin, ErrorCode::kUnsupportedFormat,
              "Tolerant fin curves need an explicit STEP pcurve mapping"));
        const auto curve = Basis(fin.curve);
        if (!curve.ok()) return Result<bool>::Err(curve.error());
        if (!std::holds_alternative<LineGeometry>(curve.value()->payload))
          return Result<bool>::Err(
              Error(fin.use.edge, ErrorCode::kUnsupportedFormat,
                    "Planar polygon faces require LINE edge bases"));
        if (!fin.use.start_vertex.value ||
            !vertices.insert(fin.use.start_vertex.value).second)
          return Result<bool>::Err(
              Error(source.face, ErrorCode::kInvalidRecord,
                    "Polygon vertices must be distinct and non-null"));
        const auto point = points_.find(fin.use.start_vertex.value);
        if (point == points_.end())
          return Result<bool>::Err(Error(fin.use.start_vertex,
                                         ErrorCode::kInvalidRecord,
                                         "Bound vertex point is unavailable"));
        polygon.push_back(point->second);
        uses_[fin.use.edge.value].push_back(
            {faces_.size(), fin.use.sense == TopologySense::kForward});
      }
      polygon_indices.push_back(i);
      face.polygons.push_back(std::move(polygon));
    }
    if (!circles.empty()) {
      const auto boundary = ValidateMixedPlanarLoops(
          face.polygons, circles, *plane, face.natural_sense,
          options_.tolerance, polygon_checks_remaining_);
      if (!boundary.ok())
        return Result<bool>::Err(Error(source.face, boundary.error().code,
                                       boundary.error().message.c_str()));
      face.outer_loop = boundary.value().circular_outer
                            ? circle_indices[boundary.value().outer_loop]
                            : polygon_indices[boundary.value().outer_loop];
      face.planar_area = boundary.value().area;
      face.reference_point = face.polygons.front().front();
      faces_.push_back(std::move(face));
      continue;
    }
    const auto boundary =
        ValidatePlanarLoops(face.polygons, *plane, face.natural_sense,
                            options_.tolerance, polygon_checks_remaining_);
    if (!boundary.ok())
      return Result<bool>::Err(Error(source.face, boundary.error().code,
                                     boundary.error().message.c_str()));
    face.outer_loop = boundary.value();
    faces_.push_back(std::move(face));
  }
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::CheckVertexLinks() {
  // Each polygon corner links its incoming and outgoing edges at one vertex.
  // A manifold vertex has one connected cycle (interior) or chain (boundary).
  // Edge valence alone would miss two sheets/shells pinched at a vertex.
  // Distinguish the two ends of a closed edge at its single seam vertex.
  // Collapsing both ends to the edge ID would falsely report four-valence
  // when a circular edge is shared by a cap and a cylindrical mantle.
  std::map<uint32_t, std::map<uint64_t, std::vector<uint64_t>>> links;
  for (const auto& face : faces_) {
    for (const auto& loop : face.source->loops) {
      const auto& fins = loop.fins;
      for (std::size_t i = 0; i < fins.size(); ++i) {
        // Vertex-free conic rings have no source vertex link; serialization
        // creates their seam locally on the shared STEP edge.
        if (!fins[i].use.start_vertex.value) continue;
        const auto& previous = fins[(i + fins.size() - 1) % fins.size()].use;
        const auto incoming =
            static_cast<uint64_t>(previous.edge.value) * 2 +
            (previous.sense == TopologySense::kForward ? 1u : 0u);
        const auto outgoing =
            static_cast<uint64_t>(fins[i].use.edge.value) * 2 +
            (fins[i].use.sense == TopologySense::kForward ? 0u : 1u);
        auto& link = links[fins[i].use.start_vertex.value];
        link[incoming].push_back(outgoing);
        link[outgoing].push_back(incoming);
      }
    }
  }
  for (const auto& [vertex, link] : links) {
    unsigned ends = 0;
    for (const auto& [edge, adjacent] : link) {
      (void)edge;
      if (adjacent.size() == 1)
        ++ends;
      else if (adjacent.size() != 2)
        return Result<bool>::Err(Error(NodeId{vertex},
                                       ErrorCode::kUnsupportedFormat,
                                       "Nonmanifold vertex link"));
    }
    if ((solid_ && ends != 0) || (ends != 0 && ends != 2))
      return Result<bool>::Err(
          Error(NodeId{vertex}, ErrorCode::kUnsupportedFormat,
                "Vertex link is not a single manifold cycle or chain"));
    std::set<uint64_t> visited{link.begin()->first};
    std::vector<uint64_t> pending{link.begin()->first};
    for (std::size_t i = 0; i < pending.size(); ++i) {
      for (const auto next : link.find(pending[i])->second)
        if (visited.insert(next).second) pending.push_back(next);
    }
    if (visited.size() != link.size())
      return Result<bool>::Err(Error(NodeId{vertex},
                                     ErrorCode::kUnsupportedFormat,
                                     "Disconnected vertex link"));
  }
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::OrientFaces() {
  reversed_.assign(faces_.size(), false);
  std::vector<std::vector<std::pair<std::size_t, bool>>> neighbors(
      faces_.size());
  std::vector<bool> boundary(faces_.size(), false);
  for (const auto& [edge, uses] : uses_) {
    if (uses.size() > 2 || (uses.size() == 2 && uses[0].face == uses[1].face))
      return Result<bool>::Err(Error(
          NodeId{edge}, ErrorCode::kUnsupportedFormat,
          "STEP shell export requires manifold edge incidence without seams"));
    if (uses.size() == 1)
      boundary[uses[0].face] = true;
    else {
      const bool relation = uses[0].forward == uses[1].forward;
      neighbors[uses[0].face].push_back({uses[1].face, relation});
      neighbors[uses[1].face].push_back({uses[0].face, relation});
    }
  }
  if (solid_) {
    const RegionBoundary* material = nullptr;
    for (const auto& region : body_.topology.boundary.regions) {
      if (region.kind != RegionKind::kSolid) continue;
      if (material || region.shells.size() != 1)
        return Result<bool>::Err(
            Error(region.region, ErrorCode::kUnsupportedFormat,
                  "Solid export requires one material region with one shell; "
                  "voids are not yet supported"));
      material = &region;
    }
    if (!material || material->shells.front().faces.size() != faces_.size() ||
        material->shells.front().face_uses.size() != faces_.size())
      return Result<bool>::Err(Error(
          body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
          "Material shell must use every exported face on exactly one side"));
    std::map<uint32_t, std::size_t> indices;
    for (std::size_t i = 0; i < faces_.size(); ++i)
      indices.emplace(faces_[i].source->face.value, i);
    const auto& shell = material->shells.front();
    for (const auto use : shell.face_uses) {
      const auto found = indices.find(shell.faces[use.boundary].face.value);
      if (found == indices.end())
        return Result<bool>::Err(
            Error(shell.shell, ErrorCode::kInvalidRecord,
                  "Material shell contains an unbound face"));
      // Region lies on the named side; outward is away from the material.
      reversed_[found->second] = use.side == ShellFaceSide::kFront;
    }
  }
  std::vector<bool> visited(faces_.size(), false);
  for (std::size_t root = 0; root < faces_.size(); ++root) {
    if (visited[root]) continue;
    FaceComponent component;
    component.faces.push_back(root);
    visited[root] = true;
    for (std::size_t at = 0; at < component.faces.size(); ++at) {
      const auto current = component.faces[at];
      component.closed = component.closed && !boundary[current];
      for (const auto& [neighbor, relation] : neighbors[current]) {
        const bool reverse = reversed_[current] != relation;
        if (!visited[neighbor]) {
          if (solid_ && reversed_[neighbor] != reverse)
            return Result<bool>::Err(
                Error(faces_[neighbor].source->face, ErrorCode::kInvalidRecord,
                      "Material face sides disagree along a shared edge"));
          reversed_[neighbor] = reverse;
          visited[neighbor] = true;
          component.faces.push_back(neighbor);
        } else if (reversed_[neighbor] != reverse)
          return Result<bool>::Err(Error(
              faces_[neighbor].source->face, ErrorCode::kUnsupportedFormat,
              "Nonorientable or inconsistently oriented face component"));
      }
    }
    components_.push_back(std::move(component));
  }
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::CheckSolid() {
  if (components_.size() != 1 || !components_.front().closed)
    return Result<bool>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
              "Solid export requires one connected closed face shell"));
  // Use a local boundary point rather than an arbitrary distant surface origin.
  const auto& first = faces_.front();
  const auto origin = first.polygons.empty() ? first.reference_point
                                             : first.polygons.front().front();
  long double volume6 = 0;
  for (std::size_t i = 0; i < faces_.size(); ++i) {
    const auto& face = faces_[i];
    if (face.cylinder || face.cone || face.torus || face.sphere ||
        face.planar_area > 0) {
      const auto sign = face.natural_sense != reversed_[i] ? 1.L : -1.L;
      if (face.sphere) {
        volume6 +=
            sign * SphereBandVolume6(*face.sphere, *face.sphere_band, origin);
      } else if (face.torus) {
        volume6 +=
            sign * TorusBandVolume6(*face.torus, *face.torus_band, origin);
      } else if (face.cone_patch) {
        volume6 +=
            sign * ConePatchVolume6(*face.cone, *face.cone_patch, origin);
      } else if (face.cone) {
        const auto& band = *face.cone_band;
        const auto& axis = face.cone->axis;
        const auto distance =
            (static_cast<long double>(band.lower_center.x) - origin.x) *
                axis.x +
            (static_cast<long double>(band.lower_center.y) - origin.y) *
                axis.y +
            (static_cast<long double>(band.lower_center.z) - origin.z) * axis.z;
        const auto slope = static_cast<long double>(face.cone->sin_half_angle) /
                           face.cone->cos_half_angle;
        // For r(z)=r0+k*z, the full radial flux is
        // pi*h*(r0+r1)*(r0-k*d), where d is the lower cut's axial offset.
        volume6 += sign * 2 * std::numbers::pi_v<long double> * band.height *
                   (band.lower_radius + band.upper_radius) *
                   std::fma(-slope, distance, band.lower_radius);
      } else if (face.cylinder_band) {
        volume6 += sign * CylinderBandVolume6(*face.cylinder,
                                              *face.cylinder_band, origin);
      } else if (face.cylinder_patch) {
        volume6 += sign * CylinderPatchVolume6(*face.cylinder,
                                               *face.cylinder_patch, origin);
      } else if (face.cylinder) {
        // Divergence theorem: the complete radial integral of a cylindrical
        // band is 2*pi*r^2*h; translation terms cancel around the full turn.
        const auto radius = static_cast<long double>(face.cylinder->radius);
        volume6 += sign * 4 * std::numbers::pi_v<long double> * radius *
                   radius * face.axial_height;
      } else {
        const auto& p = face.reference_point;
        const auto& n = face.plane->normal;
        const auto distance = (static_cast<long double>(p.x) - origin.x) * n.x +
                              (static_cast<long double>(p.y) - origin.y) * n.y +
                              (static_cast<long double>(p.z) - origin.z) * n.z;
        volume6 += sign * 2 * face.planar_area * distance;
      }
      continue;
    }
    for (const auto& polygon : face.polygons) {
      const auto a = Difference(polygon.front(), origin);
      for (std::size_t j = 1; j + 1 < polygon.size(); ++j) {
        const auto b = Difference(polygon[j], origin),
                   c = Difference(polygon[j + 1], origin);
        const long double determinant =
            static_cast<long double>(a.x) *
                (static_cast<long double>(b.y) * c.z -
                 static_cast<long double>(b.z) * c.y) +
            static_cast<long double>(a.y) *
                (static_cast<long double>(b.z) * c.x -
                 static_cast<long double>(b.x) * c.z) +
            static_cast<long double>(a.z) *
                (static_cast<long double>(b.x) * c.y -
                 static_cast<long double>(b.y) * c.x);
        volume6 += reversed_[i] ? -determinant : determinant;
      }
    }
  }
  if (!std::isfinite(volume6) || volume6 <= 0)
    return Result<bool>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kInvalidRecord,
              "Closed shell has nonpositive or unrepresentable signed volume"));
  return Result<bool>::Ok(true);
}
Id BodyWriter::Surface(const FaceData& face) {
  if (const auto found = surfaces_.find(face.source->surface.value);
      found != surfaces_.end())
    return found->second;
  Id surface = 0;
  if (face.sphere) {
    const auto& sphere = *face.sphere;
    const auto placement =
        writer_.Placement(sphere.center, sphere.axis, sphere.x_axis);
    surface = writer_.Add("SPHERICAL_SURFACE(''," + Reference(placement) + ',' +
                          writer_.Number(sphere.radius) + ')');
  } else if (face.torus) {
    const auto& torus = *face.torus;
    const auto placement =
        writer_.Placement(torus.center, torus.axis, torus.x_axis);
    surface = writer_.Add("TOROIDAL_SURFACE(''," + Reference(placement) + ',' +
                          writer_.Number(torus.major_radius) + ',' +
                          writer_.Number(torus.minor_radius) + ')');
  } else if (face.cone) {
    const auto& cone = *face.cone;
    // Use a positive STEP semi-angle. A narrowing source cone retains its
    // geometry and outward normal by reversing the exported axis instead.
    const auto axis = cone.sin_half_angle < 0
                          ? Vector3{-cone.axis.x, -cone.axis.y, -cone.axis.z}
                          : cone.axis;
    const auto placement = writer_.Placement(cone.origin, axis, cone.x_axis);
    const auto angle =
        std::atan2(std::abs(cone.sin_half_angle), cone.cos_half_angle);
    surface = writer_.Add("CONICAL_SURFACE(''," + Reference(placement) + ',' +
                          writer_.Number(cone.radius) + ',' +
                          writer_.Number(angle) + ')');
  } else if (face.cylinder) {
    const auto& cylinder = *face.cylinder;
    const auto placement =
        writer_.Placement(cylinder.origin, cylinder.axis, cylinder.x_axis);
    surface = writer_.Add("CYLINDRICAL_SURFACE(''," + Reference(placement) +
                          ',' + writer_.Number(cylinder.radius) + ')');
  } else {
    const auto placement = writer_.Placement(
        face.plane->origin, face.plane->normal, face.plane->x_axis);
    surface = writer_.Add("PLANE(''," + Reference(placement) + ')');
  }
  surfaces_.emplace(face.source->surface.value, surface);
  return surface;
}
Id BodyWriter::ParameterCurve(const NurbsCurveGeometry& curve, Id surface) {
  std::vector<Id> points;
  for (std::size_t i = 0; i < curve.controls.values.size(); i += 2)
    points.push_back(writer_.Add(
        "CARTESIAN_POINT('',(" + writer_.Number(curve.controls.values[i]) +
        ',' + writer_.Number(curve.controls.values[i + 1]) + "))"));
  std::string multiplicities = "(", knots = "(";
  for (std::size_t i = 0; i < curve.axis.knots.size(); ++i) {
    if (i) {
      multiplicities += ',';
      knots += ',';
    }
    multiplicities += std::to_string(curve.axis.multiplicities[i]);
    knots += writer_.Number(curve.axis.knots[i]);
  }
  multiplicities += ')';
  knots += ')';
  const auto spline =
      writer_.Add("B_SPLINE_CURVE_WITH_KNOTS(''," +
                  std::to_string(curve.axis.degree) + ',' + References(points) +
                  ",.UNSPECIFIED.," + Logical(curve.axis.closed) + ",.F.," +
                  multiplicities + ',' + knots + ",.UNSPECIFIED.)");
  const auto context = writer_.Add(
      "(GEOMETRIC_REPRESENTATION_CONTEXT(2) REPRESENTATION_CONTEXT('','2D'))");
  const auto representation =
      writer_.Add("DEFINITIONAL_REPRESENTATION('',(" + Reference(spline) +
                  ")," + Reference(context) + ')');
  return writer_.Add("PCURVE(''," + Reference(surface) + ',' +
                     Reference(representation) + ')');
}
Result<Id> BodyWriter::Face(std::size_t index) {
  const auto& face = faces_[index];
  std::vector<Id> bounds;
  for (std::size_t i = 0; i < face.source->loops.size(); ++i) {
    std::vector<Id> uses;
    for (const auto& fin : face.source->loops[i].fins) {
      const bool forward = fin.use.sense == TopologySense::kForward;
      const auto edge =
          Edge(fin.use.edge, fin.curve,
               forward ? fin.use.start_vertex : fin.use.end_vertex,
               forward ? fin.use.end_vertex : fin.use.start_vertex,
               forward ? fin.start : fin.end, forward ? fin.end : fin.start);
      if (!edge.ok()) return edge;
      uses.push_back(writer_.Add("ORIENTED_EDGE('',*,*," +
                                 Reference(edge.value()) + ',' +
                                 Logical(forward) + ')'));
    }
    const auto loop = writer_.Add("EDGE_LOOP(''," + References(uses) + ')');
    bounds.push_back(writer_.Add(
        std::string(face.outer_loop && i == *face.outer_loop
                        ? "FACE_OUTER_BOUND"
                        : "FACE_BOUND") +
        "(''," + Reference(loop) + ',' + Logical(!reversed_[index]) + ')'));
  }
  const auto surface = Surface(face);
  const auto result = writer_.Add(
      "ADVANCED_FACE(''," + References(bounds) + "," + Reference(surface) +
      ',' + Logical(face.natural_sense != reversed_[index]) + ')');
  ++statistics_.faces;
  return Result<Id>::Ok(result);
}
Result<StepDocument> BodyWriter::Faces() {
  if (!body_.wires.empty())
    return Result<StepDocument>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
              "Mixed face and wire geometry is not yet supported"));
  const auto prepared = PrepareFaces();
  if (!prepared.ok()) return Result<StepDocument>::Err(prepared.error());
  const auto links = CheckVertexLinks();
  if (!links.ok()) return Result<StepDocument>::Err(links.error());
  const auto oriented = OrientFaces();
  if (!oriented.ok()) return Result<StepDocument>::Err(oriented.error());
  if (solid_) {
    const auto solid = CheckSolid();
    if (!solid.ok()) return Result<StepDocument>::Err(solid.error());
  }
  std::vector<Id> shells;
  for (const auto& component : components_) {
    std::vector<Id> faces;
    for (const auto index : component.faces) {
      const auto face = Face(index);
      if (!face.ok()) return Result<StepDocument>::Err(face.error());
      faces.push_back(face.value());
    }
    shells.push_back(writer_.Add(
        std::string(component.closed ? "CLOSED_SHELL" : "OPEN_SHELL") + "(''," +
        References(faces) + ')'));
    ++statistics_.shells;
  }
  if (vertices_.size() != points_.size())
    return Result<StepDocument>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
              "Body contains vertices outside its exported face boundaries"));
  if (solid_) {
    const auto solid =
        writer_.Add("MANIFOLD_SOLID_BREP(" + Quote(options_.product_name) +
                    ',' + Reference(shells.front()) + ')');
    statistics_.solids = 1;
    const std::array<Id, 1> items{solid};
    return writer_.Finish("ADVANCED_BREP_SHAPE_REPRESENTATION", items,
                          statistics_);
  }
  const auto surface =
      writer_.Add("SHELL_BASED_SURFACE_MODEL(''," + References(shells) + ')');
  const std::array<Id, 1> items{surface};
  return writer_.Finish("MANIFOLD_SURFACE_SHAPE_REPRESENTATION", items,
                        statistics_);
}
Result<StepDocument> BodyWriter::Wires() {
  if (solid_)
    return Result<StepDocument>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
              "Solid body has no exportable faces"));
  if (body_.wires.empty()) {
    if (body_.vertices.empty())
      return Result<StepDocument>::Err(
          Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
                "Empty body has no STEP representation"));
    std::vector<Id> points;
    for (const auto& vertex : body_.vertices)
      points.push_back(writer_.Point(vertex.position));
    const auto geometry =
        writer_.Add("GEOMETRIC_CURVE_SET(''," + References(points) + ')');
    const std::array<Id, 1> items{geometry};
    return writer_.Finish(
        "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION", items,
        statistics_);
  }
  std::vector<Id> sets;
  for (const auto& wire : body_.wires) {
    const auto edge = Edge(wire.use.edge, wire.use.curve, wire.use.start_vertex,
                           wire.use.end_vertex, wire.start, wire.end);
    if (!edge.ok()) return Result<StepDocument>::Err(edge.error());
    sets.push_back(writer_.Add("CONNECTED_EDGE_SET('',(" +
                               Reference(edge.value()) + "))"));
  }
  if (vertices_.size() != points_.size())
    return Result<StepDocument>::Err(
        Error(body_.topology.boundary.body, ErrorCode::kUnsupportedFormat,
              "Mixed wire edges and isolated vertices are not yet supported"));
  const auto wires =
      writer_.Add("EDGE_BASED_WIREFRAME_MODEL(''," + References(sets) + ')');
  const std::array<Id, 1> items{wires};
  return writer_.Finish("EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION", items,
                        statistics_);
}
Result<StepDocument> BodyWriter::Build() {
  const auto* node = model_.FindNode(body_.topology.boundary.body);
  if (!node)
    return Result<StepDocument>::Err(Error(body_.topology.boundary.body,
                                           ErrorCode::kInvalidRecord,
                                           "Body is unavailable"));
  for (const auto& reference : node->references)
    if (reference.field == "child" && reference.target.value)
      return Result<StepDocument>::Err(
          Error(node->id, ErrorCode::kUnsupportedFormat,
                "Compound body export needs explicit child grouping"));
  if (body_.topology.boundary.kind == BodyKind::kGeneral)
    return Result<StepDocument>::Err(
        Error(node->id, ErrorCode::kUnsupportedFormat,
              "General body export is not yet supported"));
  if (body_.topology.boundary.kind == BodyKind::kWire && !body_.faces.empty())
    return Result<StepDocument>::Err(Error(node->id, ErrorCode::kInvalidRecord,
                                           "Wire body contains face geometry"));
  if (body_.topology.boundary.kind == BodyKind::kSheet && body_.faces.empty())
    return Result<StepDocument>::Err(Error(node->id,
                                           ErrorCode::kUnsupportedFormat,
                                           "Sheet body has no face geometry"));
  solid_ = body_.topology.boundary.kind == BodyKind::kSolid;
  for (const auto& vertex : body_.vertices)
    points_.emplace(vertex.vertex.value, vertex.position);
  return body_.faces.empty() ? Wires() : Faces();
}
}  // namespace
Result<StepDocument> BuildBody(const Model& model,
                               const BodyGeometryBinding& body,
                               const ExportOptions& options) {
  uint64_t remaining = options.max_polygon_checks;
  const auto recovered =
      RecoverTolerantCircles(model, body, options.tolerance,
                             options.use_source_edge_tolerances, remaining);
  if (!recovered.ok()) return Result<StepDocument>::Err(recovered.error());
  const auto intersections =
      RecoverIntersectionCircles(model, body, options.tolerance, remaining);
  if (!intersections.ok())
    return Result<StepDocument>::Err(intersections.error());
  if (recovered.value().empty() && intersections.value().empty())
    return BodyWriter(model, body, options, {}, remaining).Build();
  auto normalized = body;
  std::map<uint32_t, std::pair<NodeId, const FinParameterInterval*>>
      intersection_uses;
  for (const auto& edge : intersections.value())
    for (const auto& use : edge.uses)
      intersection_uses.emplace(use.fin.value, std::pair{edge.edge, &use});
  for (auto& face : normalized.faces)
    for (auto& loop : face.loops)
      for (auto& fin : loop.fins) {
        const auto found = intersection_uses.find(fin.use.fin.value);
        if (found == intersection_uses.end()) continue;
        fin.curve = found->second.first;
        fin.start = found->second.second->start;
        fin.end = found->second.second->end;
      }
  std::map<uint32_t, std::pair<NodeId, const CircularEdgeRecovery::Use*>> uses;
  for (const auto& edge : recovered.value())
    for (const auto& use : edge.uses)
      uses.emplace(use.fin.value, std::pair{edge.curve, &use});
  for (auto& face : normalized.faces)
    for (auto& loop : face.loops)
      for (auto& fin : loop.fins) {
        const auto found = uses.find(fin.use.fin.value);
        if (found == uses.end()) continue;
        fin.curve = found->second.first;
        fin.start = found->second.second->first;
        fin.end = found->second.second->last;
        fin.use.fin_curve = {};
      }
  return BodyWriter(model, normalized, options, recovered.value(), remaining,
                    intersections.value())
      .Build();
}
}  // namespace openswstep::internal
