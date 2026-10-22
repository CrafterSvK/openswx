// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <cmath>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "openparasolid/brep.h"

namespace openparasolid {
namespace {
ParseError Invalid(const char* message) {
  return {ErrorCode::kInvalidOptions, 0, message};
}
bool MatchesSource(const BoundBoundaryPathEdge& edge) {
  const auto& use = edge.use;
  const auto& fin = edge.source;
  if (!use.edge.value || !use.fin.value || use.fin != fin.use.fin ||
      use.edge != fin.use.edge ||
      (use.sense != TopologySense::kForward &&
       use.sense != TopologySense::kReverse) ||
      (fin.use.sense != TopologySense::kForward &&
       fin.use.sense != TopologySense::kReverse))
    return false;
  const bool same = use.sense == fin.use.sense;
  return use.start_vertex ==
             (same ? fin.use.start_vertex : fin.use.end_vertex) &&
         use.end_vertex == (same ? fin.use.end_vertex : fin.use.start_vertex);
}
long double Distance(Vector3 a, Vector3 b) {
  return std::hypot(static_cast<long double>(a.x) - b.x,
                    static_cast<long double>(a.y) - b.y,
                    static_cast<long double>(a.z) - b.z);
}
Result<BoundBoundaryPath> BindPath(
    const Model& model, const ShellBoundaryGroup& group,
    const std::unordered_map<uint32_t, const BoundFinGeometry*>& fins,
    double tolerance) {
  if (group.kind == BoundaryGroupKind::kBranched)
    return Result<BoundBoundaryPath>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Branched boundary has no single path"});
  if ((group.kind != BoundaryGroupKind::kCycle &&
       group.kind != BoundaryGroupKind::kChain) ||
      group.path.empty() || group.path.size() != group.edges.size())
    return Result<BoundBoundaryPath>::Err(
        Invalid("Invalid boundary path coverage"));
  std::unordered_set<uint32_t> remaining, used_fins;
  for (const auto edge : group.edges)
    if (!edge.value || !remaining.insert(edge.value).second)
      return Result<BoundBoundaryPath>::Err(
          Invalid("Duplicate or null path edge"));
  BoundBoundaryPath result;
  result.kind = group.kind;
  std::optional<Vector3> first, previous;
  for (const auto& use : group.path) {
    const auto found = fins.find(use.fin.value);
    if (!remaining.erase(use.edge.value) ||
        !used_fins.insert(use.fin.value).second || found == fins.end())
      return Result<BoundBoundaryPath>::Err(
          Invalid("Foreign or repeated path use"));
    BoundBoundaryPathEdge edge{use, *found->second};
    if (!MatchesSource(edge) ||
        (!result.edges.empty() &&
         result.edges.back().use.end_vertex != use.start_vertex))
      return Result<BoundBoundaryPath>::Err(
          Invalid("Path disagrees with source fin incidence"));
    for (double fraction : {0., .5, 1.}) {
      const auto sample =
          EvaluateBoundBoundaryPathEdge(model, edge, fraction, tolerance);
      if (!sample.ok()) return Result<BoundBoundaryPath>::Err(sample.error());
      const auto position = sample.value().curve.position;
      if (fraction == 0) {
        if (previous && Distance(*previous, position) > tolerance)
          return Result<BoundBoundaryPath>::Err(
              {ErrorCode::kInvalidRecord, 0,
               "Boundary path endpoints do not join"});
        if (!first) first = position;
      } else if (fraction == 1)
        previous = position;
    }
    result.edges.push_back(std::move(edge));
  }
  if (group.kind == BoundaryGroupKind::kCycle) {
    if (result.edges.front().use.start_vertex !=
        result.edges.back().use.end_vertex)
      return Result<BoundBoundaryPath>::Err(
          Invalid("Boundary cycle is topologically open"));
    if (Distance(*first, *previous) > tolerance)
      return Result<BoundBoundaryPath>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Boundary cycle is geometrically open"});
  } else if (group.end_vertices.size() != 2 ||
             result.edges.front().use.start_vertex !=
                 group.end_vertices.front() ||
             result.edges.back().use.end_vertex != group.end_vertices.back() ||
             !group.end_vertices.front().value ||
             group.end_vertices.front() == group.end_vertices.back()) {
    return Result<BoundBoundaryPath>::Err(
        Invalid("Boundary chain endpoints disagree"));
  }
  return Result<BoundBoundaryPath>::Ok(std::move(result));
}
}  // namespace
Result<FinGeometryEvaluation> EvaluateBoundBoundaryPathEdge(
    const Model& model, const BoundBoundaryPathEdge& edge, double fraction,
    double tolerance) {
  if (!MatchesSource(edge))
    return Result<FinGeometryEvaluation>::Err(
        Invalid("Path disagrees with source fin incidence"));
  auto fin = edge.source;
  if (edge.use.sense != fin.use.sense) std::swap(fin.start, fin.end);
  return EvaluateBoundFin(model, fin, fraction, tolerance);
}
Result<std::vector<BoundBoundaryPath>> BindBoundaryPathsGeometry(
    const Model& model, std::span<const ShellBoundaryGroup> groups,
    std::span<const FaceTrimming> faces, double tolerance,
    const BoundaryPathBindingOptions& options) {
  using Output = Result<std::vector<BoundBoundaryPath>>;
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_faces ||
      !options.max_loops || !options.max_fins || !options.max_paths ||
      !options.max_edges)
    return Output::Err(Invalid("Invalid boundary path binding options"));
  if (faces.size() > options.max_faces || groups.size() > options.max_paths)
    return Output::Err(
        {ErrorCode::kResourceLimit, 0, "Boundary path input limit"});
  uint64_t remaining_edges = options.max_edges;
  for (const auto& group : groups) {
    if (group.kind == BoundaryGroupKind::kBranched)
      return Output::Err({ErrorCode::kUnsupportedFormat, 0,
                          "Branched boundary has no single path"});
    if (group.path.size() > remaining_edges ||
        group.edges.size() != group.path.size())
      return Output::Err(group.edges.size() != group.path.size()
                             ? Invalid("Invalid boundary path coverage")
                             : ParseError{ErrorCode::kResourceLimit, 0,
                                          "Aggregate boundary step limit"});
    remaining_edges -= group.path.size();
  }
  std::unordered_map<uint32_t, const BoundFinGeometry*> fins;
  uint64_t remaining_loops = options.max_loops,
           remaining_fins = options.max_fins;
  for (const auto& face : faces) {
    if (face.loops.size() > remaining_loops)
      return Output::Err(
          {ErrorCode::kResourceLimit, 0, "Boundary source loop limit"});
    remaining_loops -= face.loops.size();
    for (const auto& loop : face.loops) {
      if (loop.fins.size() > remaining_fins)
        return Output::Err(
            {ErrorCode::kResourceLimit, 0, "Boundary source fin limit"});
      remaining_fins -= loop.fins.size();
      if (loop.isolated_vertex) {
        if (!remaining_fins)
          return Output::Err(
              {ErrorCode::kResourceLimit, 0, "Boundary source fin limit"});
        --remaining_fins;
      }
      for (const auto& fin : loop.fins)
        if (!fin.use.fin.value || fin.face != face.face ||
            fin.surface != face.surface ||
            !fins.emplace(fin.use.fin.value, &fin).second)
          return Output::Err(
              Invalid("Duplicate or inconsistent source fin binding"));
    }
  }
  std::vector<BoundBoundaryPath> result;
  for (const auto& group : groups) {
    auto path = BindPath(model, group, fins, tolerance);
    if (!path.ok()) return Output::Err(path.error());
    result.push_back(std::move(path).value());
  }
  return Output::Ok(std::move(result));
}
}  // namespace openparasolid
