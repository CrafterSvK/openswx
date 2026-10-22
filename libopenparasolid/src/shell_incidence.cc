// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <unordered_map>
#include <utility>

#include "openparasolid/topology.h"

namespace openparasolid {
Result<ShellEdgeIncidence> AnalyzeShellEdgeIncidence(
    const ShellBoundary& shell, const ShellEdgeIncidenceOptions& options) {
  auto fail = [](ErrorCode code, const char* message) {
    return Result<ShellEdgeIncidence>::Err({code, 0, message});
  };
  if (!options.max_faces || !options.max_loops || !options.max_fins ||
      !options.max_edges)
    return fail(ErrorCode::kInvalidOptions, "Invalid shell incidence limits");
  if (shell.faces.size() > options.max_faces)
    return fail(ErrorCode::kResourceLimit, "Shell incidence face limit");
  ShellEdgeIncidence result;
  result.shell = shell.shell;
  std::unordered_map<uint32_t, ShellEdgeUseCount> edges;
  uint64_t remaining_loops = options.max_loops;
  uint64_t remaining_fins = options.max_fins;
  for (const auto& face : shell.faces) {
    if (face.loops.size() > remaining_loops)
      return fail(ErrorCode::kResourceLimit, "Shell incidence loop limit");
    remaining_loops -= face.loops.size();
    for (const auto& loop : face.loops) {
      if (loop.fins.size() > remaining_fins)
        return fail(ErrorCode::kResourceLimit, "Shell incidence fin limit");
      remaining_fins -= loop.fins.size();
      if (loop.isolated_vertex) {
        if (!loop.fins.empty())
          return fail(ErrorCode::kInvalidRecord,
                      "Isolated shell loop also contains edge fins");
        if (!remaining_fins)
          return fail(ErrorCode::kResourceLimit, "Shell incidence fin limit");
        --remaining_fins;
        ++result.isolated_loops;
      }
      for (const auto& fin : loop.fins) {
        if (!fin.edge.value)
          return fail(ErrorCode::kInvalidRecord, "Null shell fin edge");
        auto found = edges.find(fin.edge.value);
        if (found == edges.end()) {
          if (edges.size() == options.max_edges)
            return fail(ErrorCode::kResourceLimit,
                        "Shell incidence edge limit");
          found =
              edges.emplace(fin.edge.value, ShellEdgeUseCount{fin.edge}).first;
        }
        if (fin.sense == TopologySense::kForward)
          ++found->second.forward_fins;
        else if (fin.sense == TopologySense::kReverse)
          ++found->second.reverse_fins;
        else
          return fail(ErrorCode::kInvalidRecord, "Unknown shell fin sense");
        ++result.face_fins;
      }
    }
  }
  result.edges.reserve(edges.size());
  for (const auto& [id, count] : edges) {
    const auto uses = count.forward_fins + count.reverse_fins;
    if (uses == 1)
      ++result.boundary_edges;
    else if (uses == 2)
      ++result.paired_edges;
    else
      ++result.higher_valence_edges;
    result.edges.push_back(count);
  }
  std::sort(result.edges.begin(), result.edges.end(),
            [](const auto& a, const auto& b) { return a.edge < b.edge; });
  return Result<ShellEdgeIncidence>::Ok(std::move(result));
}
}  // namespace openparasolid
