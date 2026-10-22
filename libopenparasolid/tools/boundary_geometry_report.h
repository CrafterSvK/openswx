// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOOLS_BOUNDARY_GEOMETRY_REPORT_H_
#define OPENPARASOLID_TOOLS_BOUNDARY_GEOMETRY_REPORT_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "model_report.h"
#include "openparasolid/brep.h"

namespace openparasolid::cli {
inline Result<json> BoundaryGeometryReport(
    const Model& model, const BodyGeometryBinding& body, double tolerance,
    bool summary, const BoundaryPathBindingOptions& options = {}) {
  const auto validated =
      BindBoundaryPathsGeometry(model, {}, {}, tolerance, options);
  if (!validated.ok()) return Result<json>::Err(validated.error());
  struct Location {
    NodeId region;
    NodeId shell;
    std::size_t component;
    std::size_t group;
  };
  std::vector<ShellBoundaryGroup> groups;
  std::vector<Location> locations;
  uint64_t remaining = options.max_edges;
  for (const auto& region : body.topology.boundary.regions)
    for (const auto& shell : region.shells) {
      const auto incidence = AnalyzeShellEdgeIncidence(shell);
      if (!incidence.ok()) return Result<json>::Err(incidence.error());
      for (std::size_t c = 0; c < incidence.value().components.size(); ++c) {
        const auto& component = incidence.value().components[c];
        for (std::size_t g = 0; g < component.boundary_groups.size(); ++g) {
          const auto& group = component.boundary_groups[g];
          if (group.kind == BoundaryGroupKind::kBranched)
            return Result<json>::Err({ErrorCode::kUnsupportedFormat, 0,
                                      "Branched boundary has no single path"});
          if (groups.size() == options.max_paths ||
              group.path.size() > remaining)
            return Result<json>::Err({ErrorCode::kResourceLimit, 0,
                                      "Boundary report path or step limit"});
          remaining -= group.path.size();
          groups.push_back(group);
          locations.push_back({region.region, shell.shell, c, g});
        }
      }
    }
  const auto bound =
      BindBoundaryPathsGeometry(model, groups, body.faces, tolerance, options);
  if (!bound.ok()) return Result<json>::Err(bound.error());
  std::size_t cycles = 0, chains = 0, edges = 0;
  for (const auto& path : bound.value()) {
    cycles += path.kind == BoundaryGroupKind::kCycle;
    chains += path.kind == BoundaryGroupKind::kChain;
    edges += path.edges.size();
  }
  json out{{"body", body.topology.boundary.body.value},
           {"binding_tolerance", tolerance},
           {"validation_scope", "sampled_source_boundary_geometry"},
           {"counts",
            {{"paths", bound.value().size()},
             {"cycles", cycles},
             {"chains", chains},
             {"edges", edges}}},
           {"limits",
            {{"max_faces", options.max_faces},
             {"max_loops", options.max_loops},
             {"max_fins", options.max_fins},
             {"max_paths", options.max_paths},
             {"max_edges", options.max_edges}}}};
  if (summary) return Result<json>::Ok(std::move(out));
  out["paths"] = json::array();
  for (std::size_t i = 0; i < bound.value().size(); ++i) {
    const auto& location = locations[i];
    const auto& path = bound.value()[i];
    json p{{"region", location.region.value},
           {"shell", location.shell.value},
           {"component", location.component},
           {"group", location.group},
           {"kind", path.kind == BoundaryGroupKind::kCycle ? "cycle" : "chain"},
           {"edges", json::array()}};
    for (const auto& edge : path.edges) {
      const auto& source = edge.source;
      const bool reversed = edge.use.sense != source.use.sense;
      json profile = nullptr;
      if (source.intersection)
        profile = {{"chart_tolerance", source.intersection->chart_tolerance},
                   {"solve_tolerance", source.intersection->solve_tolerance}};
      p["edges"].push_back(
          {{"edge", edge.use.edge.value},
           {"fin", edge.use.fin.value},
           {"face", source.face.value},
           {"surface", source.surface.value},
           {"curve", source.curve.value},
           {"start_vertex", edge.use.start_vertex.value},
           {"end_vertex", edge.use.end_vertex.value},
           {"sense",
            edge.use.sense == TopologySense::kForward ? "forward" : "reverse"},
           {"source_sense", source.use.sense == TopologySense::kForward
                                ? "forward"
                                : "reverse"},
           {"reversed", reversed},
           {"start", reversed ? source.end : source.start},
           {"end", reversed ? source.start : source.end},
           {"intersection", std::move(profile)}});
    }
    out["paths"].push_back(std::move(p));
  }
  return Result<json>::Ok(std::move(out));
}
}  // namespace openparasolid::cli
#endif  // OPENPARASOLID_TOOLS_BOUNDARY_GEOMETRY_REPORT_H_
