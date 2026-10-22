// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOOLS_TOPOLOGY_REPORT_H_
#define OPENPARASOLID_TOOLS_TOPOLOGY_REPORT_H_

#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

#include "model_report.h"
#include "openparasolid/brep.h"

namespace openparasolid::cli {
inline json FinUseReport(const FinUse& fin) {
  return {
      {"fin", fin.fin.value},
      {"edge", fin.edge.value},
      {"start_vertex", fin.start_vertex.value},
      {"end_vertex", fin.end_vertex.value},
      {"edge_curve", fin.edge_curve.value},
      {"fin_curve", fin.fin_curve.value},
      {"sense", fin.sense == TopologySense::kForward ? "forward" : "reverse"}};
}
inline json ShellIncidenceReport(const ShellEdgeIncidence& incidence,
                                 bool summary) {
  json out{{"boundary_edges", incidence.boundary_edges},
           {"paired_edges", incidence.paired_edges},
           {"higher_valence_edges", incidence.higher_valence_edges},
           {"face_fins", incidence.face_fins},
           {"isolated_loops", incidence.isolated_loops}};
  if (!summary) {
    out["edges"] = json::array();
    for (const auto& edge : incidence.edges) {
      const auto uses = edge.forward_fins + edge.reverse_fins;
      out["edges"].push_back(
          {{"edge", edge.edge.value},
           {"forward_fins", edge.forward_fins},
           {"reverse_fins", edge.reverse_fins},
           {"uses", uses},
           {"classification", uses == 1   ? "boundary"
                              : uses == 2 ? "paired"
                                          : "higher_valence"}});
    }
  }
  return out;
}
inline Result<json> BodyTopologyReport(const ValidatedBodyTopology& body,
                                       bool summary) {
  std::unordered_set<uint32_t> faces;
  std::size_t shells = 0, uses = 0, wires = 0, radial_fins = 0;
  std::vector<json> incidence_reports;
  ShellEdgeIncidence total_incidence;

  for (const auto& region : body.boundary.regions) {
    shells += region.shells.size();
    for (const auto& shell : region.shells) {
      uses += shell.face_uses.size();
      wires += shell.wire_edges.size();
      for (const auto& face : shell.faces) faces.insert(face.face.value);
      const auto incidence = AnalyzeShellEdgeIncidence(shell);
      if (!incidence.ok()) return Result<json>::Err(incidence.error());
      const auto& counts = incidence.value();
      total_incidence.boundary_edges += counts.boundary_edges;
      total_incidence.paired_edges += counts.paired_edges;
      total_incidence.higher_valence_edges += counts.higher_valence_edges;
      total_incidence.face_fins += counts.face_fins;
      total_incidence.isolated_loops += counts.isolated_loops;
      incidence_reports.push_back(ShellIncidenceReport(counts, summary));
    }
  }
  for (const auto& edge : body.edges) radial_fins += edge.fins.size();
  const char* kind = "general";
  switch (body.boundary.kind) {
    case BodyKind::kSolid:
      kind = "solid";
      break;
    case BodyKind::kSheet:
      kind = "sheet";
      break;
    case BodyKind::kWire:
      kind = "wire";
      break;
    case BodyKind::kGeneral:
      break;
  }
  json out{{"body", body.boundary.body.value},
           {"kind", kind},
           {"validation_scope", "recorded_body_topology"},
           {"counts",
            {{"regions", body.boundary.regions.size()},
             {"shells", shells},
             {"faces", faces.size()},
             {"face_uses", uses},
             {"wire_uses", wires},
             {"edges", body.edges.size()},
             {"radial_fins", radial_fins},
             {"vertices", body.vertices.size()}}}};
  out["shell_edge_incidence"] = ShellIncidenceReport(total_incidence, true);
  if (summary) return Result<json>::Ok(std::move(out));
  std::size_t shell_index = 0;
  out["regions"] = json::array();
  for (const auto& region : body.boundary.regions) {
    json r{{"region", region.region.value},
           {"body", region.body.value},
           {"kind", region.kind == RegionKind::kSolid ? "solid" : "void"},
           {"shells", json::array()}};
    for (const auto& shell : region.shells) {
      json s{{"shell", shell.shell.value},
             {"face_uses", json::array()},
             {"faces", json::array()},
             {"wires", json::array()},
             {"isolated_vertex", nullptr}};
      s["edge_incidence"] = std::move(incidence_reports[shell_index++]);
      for (const auto& use : shell.face_uses)
        s["face_uses"].push_back(
            {{"face", shell.faces[use.boundary].face.value},
             {"side", use.side == ShellFaceSide::kFront ? "front" : "back"}});
      for (const auto& face : shell.faces) {
        json f{{"face", face.face.value},
               {"surface", face.surface.value},
               {"face_sense",
                face.face_sense ? json(*face.face_sense) : json(nullptr)},
               {"loops", json::array()}};
        for (const auto& loop : face.loops) {
          json l{{"loop", loop.loop.value},
                 {"fins", json::array()},
                 {"isolated_vertex", nullptr}};
          for (const auto& fin : loop.fins)
            l["fins"].push_back(FinUseReport(fin));
          if (loop.isolated_vertex)
            l["isolated_vertex"] = {
                {"fin", loop.isolated_vertex->fin.value},
                {"vertex", loop.isolated_vertex->vertex.value}};
          f["loops"].push_back(std::move(l));
        }
        s["faces"].push_back(std::move(f));
      }
      for (const auto& wire : shell.wire_edges)
        s["wires"].push_back({{"edge", wire.edge.value},
                              {"curve", wire.curve.value},
                              {"start_vertex", wire.start_vertex.value},
                              {"end_vertex", wire.end_vertex.value}});
      if (shell.isolated_vertex)
        s["isolated_vertex"] = {{"vertex", shell.isolated_vertex->vertex.value},
                                {"point", shell.isolated_vertex->point.value}};
      r["shells"].push_back(std::move(s));
    }
    out["regions"].push_back(std::move(r));
  }
  out["edges"] = json::array();
  for (const auto& edge : body.edges) {
    json e{{"edge", edge.edge.value},
           {"curve", edge.curve.value},
           {"start_vertex", edge.start_vertex.value},
           {"end_vertex", edge.end_vertex.value},
           {"fins", json::array()}};
    for (const auto& fin : edge.fins) {
      auto f = FinUseReport(fin.use);
      f["loop"] = fin.loop.value;
      e["fins"].push_back(std::move(f));
    }
    out["edges"].push_back(std::move(e));
  }
  out["vertices"] = json::array();
  for (const auto& vertex : body.vertices) {
    json v{{"vertex", vertex.vertex.value},
           {"owner", vertex.owner.value},
           {"point", vertex.point.value},
           {"fins", json::array()}};
    for (const auto& fin : vertex.fins)
      v["fins"].push_back(
          {{"fin", fin.fin.value},
           {"loop", fin.loop.value},
           {"edge_use",
            fin.edge_use ? FinUseReport(*fin.edge_use) : json(nullptr)}});
    out["vertices"].push_back(std::move(v));
  }
  return Result<json>::Ok(std::move(out));
}
}  // namespace openparasolid::cli
#endif  // OPENPARASOLID_TOOLS_TOPOLOGY_REPORT_H_
