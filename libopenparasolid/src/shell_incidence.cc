// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "openparasolid/topology.h"

namespace openparasolid {
namespace {
// Iterative path compression and union by size avoid recursion and pairwise
// face comparisons, even for a high-valence edge shared by many faces.
class FaceComponents {
 public:
  explicit FaceComponents(std::size_t count)
      : parents_(count), sizes_(count, 1) {
    std::iota(parents_.begin(), parents_.end(), std::size_t{0});
  }
  std::size_t Find(std::size_t face) {
    while (parents_[face] != face) {
      parents_[face] = parents_[parents_[face]];
      face = parents_[face];
    }
    return face;
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
  };
  std::unordered_map<uint32_t, EdgeState> edges;
  std::unordered_set<uint32_t> face_ids;
  FaceComponents components(shell.faces.size());
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
              edges.emplace(fin.edge.value, EdgeState{{fin.edge}, face_index})
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
  result.edges.reserve(edges.size());
  for (const auto& [id, state] : edges) {
    const auto& count = state.count;
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
  const auto face_count = shell.faces.size();
  std::vector<std::size_t> group_indices(face_count, face_count);
  for (std::size_t i = 0; i < face_count; ++i) {
    const auto root = components.Find(i);
    auto& group = group_indices[root];
    if (group == face_count) {
      group = result.components.size();
      result.components.push_back({});
    }
    result.components[group].faces.push_back(shell.faces[i].face);
  }
  for (auto& component : result.components)
    std::sort(component.faces.begin(), component.faces.end());
  std::sort(result.components.begin(), result.components.end(),
            [](const auto& a, const auto& b) {
              return a.faces.front() < b.faces.front();
            });
  result.face_components = result.components.size();
  return Result<ShellEdgeIncidence>::Ok(std::move(result));
}
}  // namespace openparasolid
