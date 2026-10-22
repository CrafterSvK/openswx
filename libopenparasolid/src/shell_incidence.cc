// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "openparasolid/topology.h"

namespace openparasolid {
namespace {
// Iterative path compression and union by size avoid recursion and pairwise
// expansion of high-valence edges or vertices.
class DisjointSets {
 public:
  explicit DisjointSets(std::size_t count) : parents_(count), sizes_(count, 1) {
    std::iota(parents_.begin(), parents_.end(), std::size_t{0});
  }
  std::size_t Find(std::size_t node) {
    while (parents_[node] != node) {
      parents_[node] = parents_[parents_[node]];
      node = parents_[node];
    }
    return node;
  }
  void Join(std::size_t first, std::size_t second) {
    first = Find(first);
    second = Find(second);
    if (first == second) return;
    if (sizes_[first] < sizes_[second]) std::swap(first, second);
    parents_[second] = first;
    sizes_[first] += sizes_[second];
  }

 private:
  std::vector<std::size_t> parents_;
  std::vector<std::size_t> sizes_;
};
struct BoundaryEdge {
  NodeId edge;
  NodeId start;
  NodeId end;
};
Result<std::vector<ShellBoundaryGroup>> GroupBoundaryEdges(
    std::span<const BoundaryEdge> edges) {
  struct Vertex {
    std::size_t first_edge;
    uint64_t degree = 0;
  };
  DisjointSets sets(edges.size());
  std::unordered_map<uint32_t, Vertex> vertices;
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const auto& edge = edges[i];
    if ((edge.start.value == 0) != (edge.end.value == 0))
      return Result<std::vector<ShellBoundaryGroup>>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Partially missing boundary edge vertices"});
    if (!edge.start.value) continue;
    for (const auto vertex : {edge.start, edge.end}) {
      auto [entry, inserted] = vertices.try_emplace(vertex.value, Vertex{i});
      if (!inserted) sets.Join(i, entry->second.first_edge);
      ++entry->second.degree;
    }
  }
  std::vector<ShellBoundaryGroup> result;
  std::vector<std::size_t> group_indices(edges.size(), edges.size());
  for (std::size_t i = 0; i < edges.size(); ++i) {
    auto& group = group_indices[sets.Find(i)];
    if (group == edges.size()) {
      group = result.size();
      result.push_back({});
    }
    result[group].edges.push_back(edges[i].edge);
  }
  for (const auto& [id, vertex] : vertices) {
    auto& group = result[group_indices[sets.Find(vertex.first_edge)]];
    group.vertices.push_back(NodeId{id});
    if (vertex.degree == 1)
      group.end_vertices.push_back(NodeId{id});
    else if (vertex.degree > 2)
      group.branch_vertices.push_back(NodeId{id});
  }
  for (auto& group : result) {
    if (!group.branch_vertices.empty())
      group.kind = BoundaryGroupKind::kBranched;
    else if (!group.end_vertices.empty())
      group.kind = BoundaryGroupKind::kChain;
    std::sort(group.edges.begin(), group.edges.end());
    std::sort(group.vertices.begin(), group.vertices.end());
    std::sort(group.end_vertices.begin(), group.end_vertices.end());
    std::sort(group.branch_vertices.begin(), group.branch_vertices.end());
  }
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.edges.front() < b.edges.front();
  });
  return Result<std::vector<ShellBoundaryGroup>>::Ok(std::move(result));
}
}  // namespace
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
  struct EdgeState {
    ShellEdgeUseCount count;
    std::size_t first_face;
    NodeId start;
    NodeId end;
  };
  std::unordered_map<uint32_t, EdgeState> edges;
  std::unordered_set<uint32_t> face_ids;
  DisjointSets components(shell.faces.size());
  struct FaceCounts {
    uint64_t fins = 0;
    uint64_t isolated_loops = 0;
  };
  std::vector<FaceCounts> face_counts(shell.faces.size());
  uint64_t remaining_loops = options.max_loops;
  uint64_t remaining_fins = options.max_fins;
  for (std::size_t face_index = 0; face_index < shell.faces.size();
       ++face_index) {
    const auto& face = shell.faces[face_index];
    if (!face.face.value || !face_ids.insert(face.face.value).second)
      return fail(ErrorCode::kInvalidRecord, "Null or duplicate shell face ID");
    if (face.loops.size() > remaining_loops)
      return fail(ErrorCode::kResourceLimit, "Shell incidence loop limit");
    remaining_loops -= face.loops.size();
    for (const auto& loop : face.loops) {
      if (loop.fins.size() > remaining_fins)
        return fail(ErrorCode::kResourceLimit, "Shell incidence fin limit");
      remaining_fins -= loop.fins.size();
      face_counts[face_index].fins += loop.fins.size();
      if (loop.isolated_vertex) {
        if (!loop.fins.empty())
          return fail(ErrorCode::kInvalidRecord,
                      "Isolated shell loop also contains edge fins");
        if (!remaining_fins)
          return fail(ErrorCode::kResourceLimit, "Shell incidence fin limit");
        --remaining_fins;
        ++result.isolated_loops;
        ++face_counts[face_index].isolated_loops;
      }
      for (const auto& fin : loop.fins) {
        if (!fin.edge.value)
          return fail(ErrorCode::kInvalidRecord, "Null shell fin edge");
        auto found = edges.find(fin.edge.value);
        if (found == edges.end()) {
          if (edges.size() == options.max_edges)
            return fail(ErrorCode::kResourceLimit,
                        "Shell incidence edge limit");
          found = edges
                      .emplace(fin.edge.value, EdgeState{{fin.edge},
                                                         face_index,
                                                         fin.start_vertex,
                                                         fin.end_vertex})
                      .first;
        }
        components.Join(face_index, found->second.first_face);
        if (fin.sense == TopologySense::kForward)
          ++found->second.count.forward_fins;
        else if (fin.sense == TopologySense::kReverse)
          ++found->second.count.reverse_fins;
        else
          return fail(ErrorCode::kInvalidRecord, "Unknown shell fin sense");
        ++result.face_fins;
      }
    }
  }
  const auto face_count = shell.faces.size();
  std::vector<std::size_t> group_indices(face_count, face_count);
  for (std::size_t i = 0; i < face_count; ++i) {
    const auto root = components.Find(i);
    auto& group = group_indices[root];
    if (group == face_count) {
      group = result.components.size();
      result.components.push_back({});
    }
    auto& component = result.components[group];
    component.faces.push_back(shell.faces[i].face);
    component.face_fins += face_counts[i].fins;
    component.isolated_loops += face_counts[i].isolated_loops;
  }
  std::vector<std::vector<BoundaryEdge>> boundary_edges(
      result.components.size());
  result.edges.reserve(edges.size());
  for (const auto& [id, state] : edges) {
    const auto& count = state.count;
    const auto uses = count.forward_fins + count.reverse_fins;
    auto& component =
        result.components[group_indices[components.Find(state.first_face)]];
    component.edges.push_back(NodeId{id});
    if (uses == 1) {
      ++result.boundary_edges;
      ++component.boundary_edges;
      boundary_edges[group_indices[components.Find(state.first_face)]]
          .push_back({NodeId{id}, state.start, state.end});
    } else if (uses == 2) {
      ++result.paired_edges;
      ++component.paired_edges;
    } else {
      ++result.higher_valence_edges;
      ++component.higher_valence_edges;
    }
    result.edges.push_back(count);
  }
  for (std::size_t i = 0; i < result.components.size(); ++i) {
    auto groups = GroupBoundaryEdges(boundary_edges[i]);
    if (!groups.ok()) return Result<ShellEdgeIncidence>::Err(groups.error());
    auto& component = result.components[i];
    component.boundary_groups = std::move(groups).value();
    for (const auto& group : component.boundary_groups) {
      if (group.kind == BoundaryGroupKind::kCycle) {
        ++component.boundary_cycles;
        ++result.boundary_cycles;
      } else if (group.kind == BoundaryGroupKind::kChain) {
        ++component.boundary_chains;
        ++result.boundary_chains;
      } else {
        ++component.branched_boundaries;
        ++result.branched_boundaries;
      }
    }
  }
  std::sort(result.edges.begin(), result.edges.end(),
            [](const auto& a, const auto& b) { return a.edge < b.edge; });
  for (auto& component : result.components) {
    std::sort(component.faces.begin(), component.faces.end());
    std::sort(component.edges.begin(), component.edges.end());
  }
  std::sort(result.components.begin(), result.components.end(),
            [](const auto& a, const auto& b) {
              return a.faces.front() < b.faces.front();
            });
  result.face_components = result.components.size();
  return Result<ShellEdgeIncidence>::Ok(std::move(result));
}
}  // namespace openparasolid
