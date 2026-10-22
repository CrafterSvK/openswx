// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <utility>
#include <vector>

#include "conic_boundary.h"
#include "planar_boundary.h"
#include "step_writer.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
Vector3 Difference(Vector3 a, Vector3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
struct FaceData {
  const FaceTrimming* source = nullptr;
  const PlaneGeometry* plane = nullptr;
  bool natural_sense = true;
  std::vector<std::vector<Vector3>> polygons;
  std::size_t outer_loop = 0;
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
             const ExportOptions& options)
      : model_(model),
        body_(body),
        options_(options),
        writer_(options),
        polygon_checks_remaining_(options.max_polygon_checks) {}
  Result<StepDocument> Build();

 private:
  Result<const GeometryObject*> Geometry(NodeId id) const;
  Result<const GeometryObject*> Basis(NodeId id) const;
  ParseError Error(NodeId id, ErrorCode code, const char* message) const;
  Result<Id> Curve(NodeId id);
  Result<Id> Edge(NodeId edge, NodeId curve, NodeId start, NodeId end,
                  double first, double last);
  Id Vertex(NodeId id);
  Result<bool> PrepareFaces();
  Result<bool> PrepareConicFace(FaceData& face);
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
  } else {
    return Result<Id>::Err(Error(
        id, ErrorCode::kUnsupportedFormat,
        "Curve export currently supports LINE, CIRCLE and ELLIPSE bases"));
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
  if (conic && std::abs(last - first) > 2 * std::numbers::pi)
    return Result<Id>::Err(
        Error(edge, ErrorCode::kUnsupportedFormat,
              "Multiple conic turns cannot be represented by one EDGE_CURVE"));
  const auto geometry = Curve(curve);
  if (!geometry.ok()) return geometry;
  Id start_id = 0, end_id = 0;
  if (start.value == 0 && end.value == 0) {
    if (!conic ||
        std::abs(std::abs(last - first) - 2 * std::numbers::pi) > 1e-12)
      return Result<Id>::Err(
          Error(edge, ErrorCode::kUnsupportedFormat,
                "Vertex-free edges require one complete conic turn"));
    const auto sample = EvaluateCurve(model_, curve, first);
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
Result<bool> BodyWriter::PrepareConicFace(FaceData& face) {
  if (solid_)
    return Result<bool>::Err(
        Error(face.source->face, ErrorCode::kUnsupportedFormat,
              "Conic face export currently requires a sheet body"));
  std::vector<ConicBoundary> loops;
  for (const auto& loop : face.source->loops) {
    if (loop.isolated_vertex || loop.fins.size() != 1)
      return Result<bool>::Err(Error(
          face.source->face, ErrorCode::kUnsupportedFormat,
          "Conic face loops require one complete CIRCLE or ELLIPSE edge each"));
    const auto& fin = loop.fins.front();
    if (fin.use.fin_curve.value)
      return Result<bool>::Err(
          Error(fin.use.fin, ErrorCode::kUnsupportedFormat,
                "Tolerant fin curves need an explicit STEP pcurve mapping"));
    if (fin.use.start_vertex != fin.use.end_vertex)
      return Result<bool>::Err(Error(
          fin.use.fin, ErrorCode::kInvalidRecord,
          "Complete conic loop must use one seam vertex or be vertex-free"));
    const auto curve = Basis(fin.curve);
    if (!curve.ok()) return Result<bool>::Err(curve.error());
    if (const auto* circle =
            std::get_if<CircleGeometry>(&curve.value()->payload)) {
      loops.push_back({circle->center, circle->normal, circle->x_axis,
                       circle->radius, circle->radius, fin.start, fin.end,
                       true});
    } else if (const auto* ellipse =
                   std::get_if<EllipseGeometry>(&curve.value()->payload)) {
      loops.push_back({ellipse->center, ellipse->normal, ellipse->x_axis,
                       ellipse->major_radius, ellipse->minor_radius, fin.start,
                       fin.end, false});
    } else {
      return Result<bool>::Err(
          Error(fin.curve, ErrorCode::kUnsupportedFormat,
                "Single-edge planar loops require CIRCLE or ELLIPSE bases"));
    }
    uses_[fin.use.edge.value].push_back(
        {faces_.size(), fin.use.sense == TopologySense::kForward});
  }
  const auto boundary =
      ValidatePlanarConics(loops, *face.plane, face.natural_sense,
                           options_.tolerance, polygon_checks_remaining_);
  if (!boundary.ok())
    return Result<bool>::Err(Error(face.source->face, boundary.error().code,
                                   boundary.error().message.c_str()));
  face.outer_loop = boundary.value();
  return Result<bool>::Ok(true);
}
Result<bool> BodyWriter::PrepareFaces() {
  for (const auto& source : body_.faces) {
    if (source.loops.empty())
      return Result<bool>::Err(Error(source.face, ErrorCode::kUnsupportedFormat,
                                     "Planar face has no boundary loops"));
    const auto surface = Geometry(source.surface);
    if (!surface.ok()) return Result<bool>::Err(surface.error());
    const auto* plane = std::get_if<PlaneGeometry>(&surface.value()->payload);
    if (!plane)
      return Result<bool>::Err(
          Error(source.face, ErrorCode::kUnsupportedFormat,
                "Face export currently supports PLANE surfaces"));
    if (!source.face_sense ||
        (*source.face_sense != '+' && *source.face_sense != '-') ||
        !surface.value()->sense ||
        (*surface.value()->sense != '+' && *surface.value()->sense != '-'))
      return Result<bool>::Err(Error(source.face, ErrorCode::kUnsupportedFormat,
                                     "Face and surface senses must be known"));
    FaceData face{
        &source, plane, source.face_sense == surface.value()->sense, {}};
    if (source.loops.front().fins.size() == 1) {
      const auto conic = PrepareConicFace(face);
      if (!conic.ok()) return conic;
      faces_.push_back(std::move(face));
      continue;
    }
    for (const auto& loop : source.loops) {
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
      face.polygons.push_back(std::move(polygon));
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
  std::map<uint32_t, std::map<uint32_t, std::vector<uint32_t>>> links;
  for (const auto& face : faces_) {
    for (const auto& loop : face.source->loops) {
      const auto& fins = loop.fins;
      for (std::size_t i = 0; i < fins.size(); ++i) {
        // Vertex-free conic rings have no source vertex link; serialization
        // creates their seam locally on the shared STEP edge.
        if (!fins[i].use.start_vertex.value) continue;
        const auto incoming =
            fins[(i + fins.size() - 1) % fins.size()].use.edge.value;
        const auto outgoing = fins[i].use.edge.value;
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
    std::set<uint32_t> visited{link.begin()->first};
    std::vector<uint32_t> pending{link.begin()->first};
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
  const auto origin = faces_.front().polygons.front().front();
  long double volume6 = 0;
  for (std::size_t i = 0; i < faces_.size(); ++i) {
    for (const auto& polygon : faces_[i].polygons) {
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
        std::string(i == face.outer_loop ? "FACE_OUTER_BOUND" : "FACE_BOUND") +
        "(''," + Reference(loop) + ',' + Logical(!reversed_[index]) + ')'));
  }
  const auto placement = writer_.Placement(
      face.plane->origin, face.plane->normal, face.plane->x_axis);
  const auto plane = writer_.Add("PLANE(''," + Reference(placement) + ')');
  const auto result = writer_.Add(
      "ADVANCED_FACE(''," + References(bounds) + "," + Reference(plane) + ',' +
      Logical(face.natural_sense != reversed_[index]) + ')');
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
  return BodyWriter(model, body, options).Build();
}
}  // namespace openswstep::internal
