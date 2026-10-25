// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "saved_geometry_history.h"

#include <algorithm>
#include <map>
#include <set>
#include <tuple>

namespace openswx::internal {
namespace {
using Mark = openparasolid::DeltaMark;
auto Fields(const Mark& mark) {
  return std::tie(mark.index, mark.preceding, mark.first_following,
                  mark.next_sibling, mark.prev_sibling, mark.n_new_nodes,
                  mark.n_del_nodes, mark.n_copy_mod_nodes, mark.delta_key,
                  mark.delta_is_forward, mark.identifier);
}
std::optional<std::string> MarkGraph(const SavedHistoryData& history,
                                     std::map<uint32_t, Mark>& marks) {
  marks.emplace(history.prefix->current_mark.index,
                history.prefix->current_mark);
  for (const auto& mark : history.marks) {
    const auto [it, inserted] = marks.emplace(mark.index, mark);
    if (!inserted && Fields(it->second) != Fields(mark))
      return "Conflicting occurrences of one PMARK identity";
  }
  std::set<uint32_t> identifiers;
  std::optional<uint32_t> root;
  for (const auto& [index, mark] : marks) {
    if (!index || !identifiers.insert(mark.identifier).second)
      return "Null mark identity or ambiguous persistent PMARK identifiers";
    for (uint32_t link : {mark.preceding, mark.first_following,
                          mark.next_sibling, mark.prev_sibling})
      if (link && (!marks.contains(link) || link == index))
        return "Missing or self-referencing PMARK link";
    if (!mark.preceding) {
      if (root || mark.next_sibling || mark.prev_sibling)
        return "Ambiguous initial PMARK or initial mark has siblings";
      root = index;
    }
  }
  if (!root) return "No initial PMARK";
  // Walk each parent's child/sibling chain once. All non-root nodes must occur
  // in exactly one chain, with reciprocal links and the declared predecessor.
  std::set<uint32_t> children;
  for (const auto& [index, parent] : marks) {
    uint32_t previous = 0;
    for (uint32_t child = parent.first_following; child;) {
      const auto& mark = marks.find(child)->second;
      if (!children.insert(child).second || mark.preceding != index ||
          mark.prev_sibling != previous)
        return "Cyclic or inconsistent PMARK child/sibling links";
      previous = child;
      child = mark.next_sibling;
    }
  }
  if (children.size() != marks.size() - 1 || children.contains(*root))
    return "Incomplete PMARK child/sibling graph";
  // A disconnected predecessor cycle can satisfy local reciprocal links.
  // Colour all predecessor walks to reject it with bounded, nonrecursive work.
  std::set<uint32_t> checked;
  for (const auto& [index, unused] : marks) {
    std::set<uint32_t> visiting;
    uint32_t current = index;
    while (current && !checked.contains(current)) {
      if (!visiting.insert(current).second)
        return "Cyclic PMARK predecessor graph";
      current = marks.find(current)->second.preceding;
    }
    checked.insert(visiting.begin(), visiting.end());
  }
  return {};
}
}  // namespace
void InspectSavedState(SavedGeometryResolution& result,
                       const std::vector<SavedHistoryData>& histories) {
  auto& evidence = result.state_evidence;
  evidence = {};
  const auto selected = std::find_if(
      result.candidates.begin(), result.candidates.end(),
      [](const auto& candidate) { return candidate.selected_partition; });
  if (selected == result.candidates.end()) {
    evidence.issue = "No unique baseline partition selected";
    return;
  }
  evidence.baseline_payload_index = selected->payload_index;
  evidence.baseline_current_id = selected->world_current_id;
  std::vector<std::size_t> deltas;
  for (const auto& candidate : result.candidates)
    if (candidate.kind == ParasolidPayloadKind::kDeltas &&
        candidate.source.stream == selected->source.stream &&
        candidate.source.container_offset == selected->source.container_offset)
      deltas.push_back(candidate.payload_index);
  if (deltas.size() != 1) {
    evidence.issue =
        deltas.empty()
            ? "No associated delta target metadata"
            : "Multiple associated delta candidates; no target selected";
    return;
  }
  evidence.delta_payload_index = deltas.front();
  const auto& history = histories[deltas.front()];
  if (!selected->decoded_complete || !history.complete || !history.prefix ||
      !evidence.baseline_current_id) {
    evidence.issue = "Complete base WORLD and delta target metadata required";
    return;
  }
  evidence.target_identifier = history.prefix->current_mark.identifier;
  evidence.relation = evidence.baseline_current_id == evidence.target_identifier
                          ? SavedStateRelation::kMatchingIdentifiers
                          : SavedStateRelation::kDifferentIdentifiers;
  std::map<uint32_t, Mark> marks;
  if (const auto issue = MarkGraph(history, marks)) {
    evidence.issue = *issue;
    return;
  }
  std::vector<uint32_t> path;
  for (uint32_t current = history.prefix->current_mark.index; current;
       current = marks.find(current)->second.preceding) {
    path.push_back(current);
    if (marks.find(current)->second.identifier ==
        *evidence.baseline_current_id) {
      std::reverse(path.begin(), path.end());
      evidence.mark_path = std::move(path);
      evidence.history_path_status = GeometryResolutionStatus::kResolved;
      return;
    }
  }
  evidence.issue =
      "Baseline identifier is not on the current mark's ancestry; "
      "no branch or replay direction inferred";
}
}  // namespace openswx::internal
