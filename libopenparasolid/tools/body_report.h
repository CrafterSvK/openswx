// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOOLS_BODY_REPORT_H_
#define OPENPARASOLID_TOOLS_BODY_REPORT_H_

#include <cstddef>
#include <utility>

#include "model_report.h"
#include "openparasolid/brep.h"

namespace openparasolid::cli {
inline json SurfaceParametersJson(const SurfaceParameters& uv) {
  return {{"u", uv.u}, {"v", uv.v}, {"residual", uv.residual}};
}
inline json BodyGeometryReport(const BodyGeometryBinding& body,
                               double tolerance,
                               const BodyGeometryBindingOptions& options,
                               bool summary) {
  std::size_t loops = 0, fins = 0, shells = 0, uses = 0;
  for (const auto& face : body.faces)
    for (const auto& loop : face.loops) {
      ++loops;
      fins += loop.fins.size();
    }
  for (const auto& region : body.topology.boundary.regions) {
    shells += region.shells.size();
    for (const auto& shell : region.shells) uses += shell.face_uses.size();
  }
  json out{{"body", body.topology.boundary.body.value},
           {"validation_scope", "sampled_source_boundary_geometry"},
           {"binding_tolerance", tolerance},
           {"intersection", nullptr},
           {"counts",
            {{"regions", body.topology.boundary.regions.size()},
             {"shells", shells},
             {"face_uses", uses},
             {"faces", body.faces.size()},
             {"loops", loops},
             {"fins", fins},
             {"edges", body.topology.edges.size()},
             {"wire_edges", body.wires.size()},
             {"vertices", body.vertices.size()}}},
           {"limits",
            {{"max_faces", options.max_faces},
             {"max_fins", options.max_fins},
             {"max_wire_edges", options.max_wire_edges},
             {"max_vertices", options.max_vertices},
             {"max_model_nodes", options.topology.max_model_nodes},
             {"max_model_references", options.topology.max_model_references}}}};
  if (options.intersection) {
    const auto& i = *options.intersection;
    out["intersection"] = {
        {"chart_tolerance", i.chart_tolerance},
        {"solve_tolerance", i.solve_tolerance},
        {"chart_parameter_tolerance", i.chart_parameter_tolerance},
        {"minimum_normal_sine", i.minimum_normal_sine},
        {"minimum_chord_projection", i.minimum_chord_projection}};
  }
  if (summary) return out;
  out["regions"] = json::array();
  for (const auto& region : body.topology.boundary.regions) {
    json r{{"region", region.region.value},
           {"kind", region.kind == RegionKind::kSolid ? "solid" : "void"},
           {"shells", json::array()}};
    for (const auto& shell : region.shells) {
      json s{{"shell", shell.shell.value},
             {"face_uses", json::array()},
             {"wire_edges", json::array()},
             {"isolated_vertex", nullptr}};
      for (const auto& use : shell.face_uses)
        s["face_uses"].push_back(
            {{"face", shell.faces[use.boundary].face.value},
             {"side", use.side == ShellFaceSide::kFront ? "front" : "back"}});
      for (const auto& wire : shell.wire_edges)
        s["wire_edges"].push_back(wire.edge.value);
      if (shell.isolated_vertex)
        s["isolated_vertex"] = {{"vertex", shell.isolated_vertex->vertex.value},
                                {"point", shell.isolated_vertex->point.value}};
      r["shells"].push_back(std::move(s));
    }
    out["regions"].push_back(std::move(r));
  }
  out["faces"] = json::array();
  for (const auto& face : body.faces) {
    json f{{"face", face.face.value},
           {"surface", face.surface.value},
           {"face_sense",
            face.face_sense ? json(*face.face_sense) : json(nullptr)},
           {"loops", json::array()}};
    for (const auto& loop : face.loops) {
      json l{
          {"loop", loop.loop.value},
          {"fins", json::array()},
          {"isolated_vertex_uv",
           loop.isolated_vertex ? SurfaceParametersJson(*loop.isolated_vertex)
                                : json(nullptr)}};
      for (const auto& fin : loop.fins)
        l["fins"].push_back(
            {{"fin", fin.use.fin.value},
             {"edge", fin.use.edge.value},
             {"start_vertex", fin.use.start_vertex.value},
             {"end_vertex", fin.use.end_vertex.value},
             {"edge_curve", fin.use.edge_curve.value},
             {"fin_curve", fin.use.fin_curve.value},
             {"sense",
              fin.use.sense == TopologySense::kForward ? "forward" : "reverse"},
             {"curve", fin.curve.value},
             {"start", fin.start},
             {"end", fin.end}});
      f["loops"].push_back(std::move(l));
    }
    out["faces"].push_back(std::move(f));
  }
  out["wires"] = json::array();
  for (const auto& wire : body.wires)
    out["wires"].push_back({{"edge", wire.use.edge.value},
                            {"start_vertex", wire.use.start_vertex.value},
                            {"end_vertex", wire.use.end_vertex.value},
                            {"curve", wire.use.curve.value},
                            {"start", wire.start},
                            {"end", wire.end}});
  out["vertices"] = json::array();
  for (const auto& vertex : body.vertices)
    out["vertices"].push_back({{"vertex", vertex.vertex.value},
                               {"point", vertex.point.value},
                               {"position", VectorJson(vertex.position)}});
  return out;
}
}  // namespace openparasolid::cli
#endif  // OPENPARASOLID_TOOLS_BODY_REPORT_H_
