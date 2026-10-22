// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/topology.h"

#include <algorithm>
#include <initializer_list>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openparasolid {
namespace {
bool PositiveBodyLimits(const BodyTraversalOptions& options) {
  return options.max_regions && options.max_shells && options.max_face_uses &&
         options.max_loops && options.max_fins && options.max_wire_edges;
}
bool Matches(uint16_t expected, uint16_t actual) {
  if (expected < 1000) return expected == actual;
  const std::initializer_list<uint16_t> curves{30,  31,  32,  38, 130,
                                               133, 134, 137, 200};
  const std::initializer_list<uint16_t> surfaces{50, 51, 52, 53,  54,  56, 59,
                                                 60, 67, 68, 120, 124, 201};
  const auto& kinds = expected == 1008 ? curves : surfaces;
  return std::find(kinds.begin(), kinds.end(), actual) != kinds.end();
}
class TopologyReader {
 public:
  explicit TopologyReader(const Model& model) : model_(model) {}
  Result<LoopBoundary> ReadLoop(NodeId id, const LoopTraversalOptions& options);
  Result<EdgeBoundary> ReadEdge(NodeId id, const EdgeTraversalOptions& options);
  Result<FaceBoundary> ReadFace(NodeId id, const FaceTraversalOptions& options);
  Result<ShellBoundary> ReadShell(NodeId id,
                                  const ShellTraversalOptions& options,
                                  bool allow_zero_limits = false);
  Result<RegionBoundary> ReadRegion(NodeId id,
                                    const RegionTraversalOptions& options,
                                    bool allow_zero_limits = false);
  Result<BodyBoundary> ReadBody(NodeId id, const BodyTraversalOptions& options);
  Result<CompoundBodyBoundary> ReadCompound(
      NodeId id, const CompoundBodyTraversalOptions& options);
  Result<ChildBodyBoundary> ReadChild(
      NodeId id, const CompoundBodyTraversalOptions& options);

 private:
  bool Fail(const ModelNode& node, std::string_view field, const char* message,
            ErrorCode code = ErrorCode::kInvalidRecord);
  const ModelNode* Node(NodeId id, uint16_t type);
  bool Reference(const ModelNode& node, std::string_view name, uint16_t type,
                 bool nullable, NodeId& target);
  bool Sense(const ModelNode& node, TopologySense& sense);
  bool Kind(const ModelNode& node, BodyKind& kind);
  bool Children(CompoundBodyBoundary& result, uint64_t limit);
  bool EdgeUse(const ModelNode& fin, NodeId edge, NodeId other, FinUse& use);
  struct ShellBudget {
    uint64_t loops;
    uint64_t fins;
    std::unordered_map<uint32_t, std::size_t> faces;
  };
  bool FaceUses(NodeId first, ShellFaceSide side, ShellBoundary& result,
                ShellBudget& budget, const ShellTraversalOptions& options);
  bool WireEdges(NodeId first, ShellBoundary& result, uint64_t limit);
  bool Dummy(const ModelNode& fin);
  bool LoopLinks(const ModelNode& fin, NodeId loop);
  bool VertexNext(const ModelNode& fin);
  bool SeparateShell(const ShellBoundary& shell,
                     std::unordered_map<uint32_t, NodeId>& owners);
  const Model& model_;
  std::optional<ParseError> error_;
};
bool TopologyReader::Fail(const ModelNode& node, std::string_view field,
                          const char* message, ErrorCode code) {
  if (!error_)
    error_ = ParseError{code, node.source_offset,
                        "Topology node " + std::to_string(node.id.value) +
                            " field " + std::string(field) + ": " + message};
  return false;
}
const ModelNode* TopologyReader::Node(NodeId id, uint16_t type) {
  const auto* node = model_.FindNode(id);
  if (!node) {
    error_ =
        ParseError{ErrorCode::kInvalidRecord, 0,
                   "Topology target is missing: " + std::to_string(id.value)};
    return nullptr;
  }
  if (!Matches(type, node->node_type)) {
    Fail(*node, "target", "Target has the wrong node type");
    return nullptr;
  }
  return node;
}
bool TopologyReader::Reference(const ModelNode& node, std::string_view name,
                               uint16_t type, bool nullable, NodeId& target) {
  const ModelReference* found = nullptr;
  for (const auto& reference : node.references) {
    if (reference.field != name) continue;
    if (found || reference.element != 0)
      return Fail(node, name, "Expected one unambiguous pointer role");
    found = &reference;
  }
  if (!found)
    return Fail(node, name, "Pointer role is unavailable",
                ErrorCode::kUnsupportedFormat);
  if (found->status == ModelReferenceStatus::kUnverifiedClass)
    return Fail(node, name, "Pointer class is unverified",
                ErrorCode::kUnsupportedFormat);
  if (found->status != ModelReferenceStatus::kResolved &&
      found->status != ModelReferenceStatus::kNull)
    return Fail(node, name, "Reference target is missing or incompatible");
  target = found->target;
  if (!target.value)
    return nullable || Fail(node, name, "Required reference is null");
  return Node(target, type) != nullptr;
}
bool TopologyReader::Sense(const ModelNode& node, TopologySense& sense) {
  if (!node.topology_sense ||
      (*node.topology_sense != '+' && *node.topology_sense != '-'))
    return Fail(node, "sense", "Fin sense is unavailable or unknown",
                ErrorCode::kUnsupportedFormat);
  sense = *node.topology_sense == '+' ? TopologySense::kForward
                                      : TopologySense::kReverse;
  return true;
}
bool TopologyReader::EdgeUse(const ModelNode& fin, NodeId edge_id, NodeId other,
                             FinUse& use) {
  const auto& edge = *model_.FindNode(edge_id);
  NodeId primary_id, secondary_id, primary_edge, secondary_edge, other_edge;
  NodeId end, start, vertex;
  if (!other.value || other == fin.id)
    return Fail(fin, "other", "Edge use requires another fin");
  const auto& other_fin = *model_.FindNode(other);
  if (!Reference(other_fin, "edge", 16, false, other_edge) ||
      !Reference(edge, "halfedge", 17, false, primary_id))
    return false;
  const auto& primary = *model_.FindNode(primary_id);
  if (!Reference(primary, "other", 17, false, secondary_id)) return false;
  const auto& secondary = *model_.FindNode(secondary_id);
  TopologySense positive, negative;
  if (!Sense(fin, use.sense) || !Sense(primary, positive) ||
      !Sense(secondary, negative) ||
      !Reference(primary, "edge", 16, false, primary_edge) ||
      !Reference(secondary, "edge", 16, false, secondary_edge) ||
      !Reference(primary, "vertex", 18, true, end) ||
      !Reference(secondary, "vertex", 18, true, start) ||
      !Reference(fin, "vertex", 18, true, vertex) ||
      !Reference(edge, "curve", 1008, true, use.edge_curve) ||
      !Reference(fin, "curve", 1008, true, use.fin_curve))
    return false;
  if (positive != TopologySense::kForward ||
      negative != TopologySense::kReverse || primary_id == secondary_id ||
      primary_edge != edge_id || secondary_edge != edge_id ||
      other_edge != edge_id)
    return Fail(edge, "halfedge", "Inconsistent primary/secondary edge fins");
  if ((start.value == 0) != (end.value == 0))
    return Fail(edge, "halfedge", "Edge has only one null endpoint");
  use.fin = fin.id;
  use.edge = edge_id;
  use.start_vertex = use.sense == TopologySense::kForward ? start : end;
  use.end_vertex = use.sense == TopologySense::kForward ? end : start;
  if (vertex != use.end_vertex)
    return Fail(fin, "vertex",
                "Forward vertex disagrees with oriented edge endpoints");
  return true;
}
bool TopologyReader::LoopLinks(const ModelNode& fin, NodeId loop) {
  NodeId forward, backward, forward_loop, backward_loop, close_forward,
      close_backward;
  if (!Reference(fin, "forward", 17, false, forward) ||
      !Reference(fin, "backward", 17, false, backward))
    return false;
  const auto& next = *model_.FindNode(forward);
  const auto& previous = *model_.FindNode(backward);
  if (!Reference(next, "loop", 15, false, forward_loop) ||
      !Reference(previous, "loop", 15, false, backward_loop) ||
      !Reference(next, "backward", 17, false, close_backward) ||
      !Reference(previous, "forward", 17, false, close_forward))
    return false;
  if (forward_loop != loop || backward_loop != loop ||
      close_forward != fin.id || close_backward != fin.id)
    return Fail(fin, "forward/backward",
                "Inconsistent local loop links of radial fin");
  return true;
}
Result<EdgeBoundary> TopologyReader::ReadEdge(
    NodeId id, const EdgeTraversalOptions& options) {
  if (!options.max_fins)
    return Result<EdgeBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Edge traversal requires a positive fin limit"});
  const auto* edge = Node(id, 16);
  if (!edge) return Result<EdgeBoundary>::Err(*error_);
  NodeId first;
  if (!Reference(*edge, "halfedge", 17, false, first))
    return Result<EdgeBoundary>::Err(*error_);
  EdgeBoundary result;
  result.edge = id;
  NodeId current = first;
  std::unordered_set<uint32_t> visited;
  do {
    const auto& fin = *model_.FindNode(current);
    if (visited.contains(current.value)) {
      Fail(fin, "other", "Radial fin cycle does not close at its primary fin");
      return Result<EdgeBoundary>::Err(*error_);
    }
    if (visited.size() >= options.max_fins)
      return Result<EdgeBoundary>::Err({ErrorCode::kResourceLimit,
                                        fin.source_offset,
                                        "Edge radial fin limit exceeded"});
    visited.insert(current.value);
    NodeId owner, next;
    EdgeFinUse use;
    if (!Reference(fin, "edge", 16, false, owner) ||
        !Reference(fin, "other", 17, false, next) ||
        !Reference(fin, "loop", 15, true, use.loop))
      return Result<EdgeBoundary>::Err(*error_);
    if (owner != id) {
      Fail(fin, "edge", "Radial fin belongs to another edge");
      return Result<EdgeBoundary>::Err(*error_);
    }
    if (!EdgeUse(fin, id, next, use.use))
      return Result<EdgeBoundary>::Err(*error_);
    if (!use.loop.value) {
      if (result.fins.size() >= 2) {
        Fail(fin, "loop", "A dummy fin must be a primary or secondary anchor");
        return Result<EdgeBoundary>::Err(*error_);
      }
      if (!Dummy(fin)) return Result<EdgeBoundary>::Err(*error_);
    } else if (!LoopLinks(fin, use.loop))
      return Result<EdgeBoundary>::Err(*error_);
    if (result.fins.empty()) {
      result.start_vertex = use.use.start_vertex;
      result.end_vertex = use.use.end_vertex;
      result.curve = use.use.edge_curve;
    }
    result.fins.push_back(use);
    current = next;
  } while (current != first);
  return Result<EdgeBoundary>::Ok(std::move(result));
}
Result<LoopBoundary> TopologyReader::ReadLoop(
    NodeId id, const LoopTraversalOptions& options) {
  if (options.max_fins == 0)
    return Result<LoopBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Loop traversal requires a positive fin limit"});
  const auto* loop = Node(id, 15);
  if (!loop) return Result<LoopBoundary>::Err(*error_);
  LoopBoundary result;
  result.loop = id;
  NodeId first;
  if (!Reference(*loop, "face", 14, false, result.face) ||
      !Reference(*loop, "halfedge", 17, false, first))
    return Result<LoopBoundary>::Err(*error_);
  const auto& face = *model_.FindNode(result.face);
  result.face_sense = face.topology_sense;
  if (!Reference(face, "surface", 1006, true, result.surface))
    return Result<LoopBoundary>::Err(*error_);
  std::unordered_set<uint32_t> visited;
  std::unordered_set<uint64_t> uses;
  NodeId current = first, previous, first_backward;
  do {
    const auto& fin = *model_.FindNode(current);
    if (visited.size() >= options.max_fins)
      return Result<LoopBoundary>::Err({ErrorCode::kResourceLimit,
                                        fin.source_offset,
                                        "Loop traversal fin limit exceeded"});
    if (!visited.insert(current.value).second) {
      Fail(fin, "forward", "Fin cycle does not close at the loop's first fin");
      return Result<LoopBoundary>::Err(*error_);
    }
    NodeId owner, forward, backward, edge, other;
    if (!Reference(fin, "loop", 15, false, owner) ||
        !Reference(fin, "forward", 17, false, forward) ||
        !Reference(fin, "backward", 17, false, backward) ||
        !Reference(fin, "edge", 16, true, edge) ||
        !Reference(fin, "other", 17, true, other))
      return Result<LoopBoundary>::Err(*error_);
    if (owner != id || (previous.value && backward != previous)) {
      Fail(fin, "loop/backward",
           "Inconsistent loop ownership or reciprocal links");
      return Result<LoopBoundary>::Err(*error_);
    }
    if (!previous.value) first_backward = backward;
    if (!edge.value) {
      NodeId vertex, curve;
      if (!Reference(fin, "vertex", 18, false, vertex) ||
          !Reference(fin, "curve", 1008, true, curve))
        return Result<LoopBoundary>::Err(*error_);
      if (current != first || forward != current || backward != current ||
          other.value || curve.value) {
        Fail(fin, "edge", "Invalid isolated vertex loop");
        return Result<LoopBoundary>::Err(*error_);
      }
      result.isolated_vertex = IsolatedLoopVertex{current, vertex};
    } else {
      FinUse use;
      if (!EdgeUse(fin, edge, other, use))
        return Result<LoopBoundary>::Err(*error_);
      const uint64_t key = (uint64_t{edge.value} << 1) |
                           (use.sense == TopologySense::kReverse ? 1 : 0);
      if (!uses.insert(key).second ||
          (!result.fins.empty() &&
           result.fins.back().end_vertex != use.start_vertex) ||
          (!use.start_vertex.value &&
           (current != first || forward != current || backward != current))) {
        Fail(fin, "edge/vertex",
             "Repeated oriented edge, disconnected vertices or multi-fin ring "
             "edge");
        return Result<LoopBoundary>::Err(*error_);
      }
      result.fins.push_back(use);
    }
    previous = current;
    current = forward;
  } while (current != first);
  if (first_backward != previous ||
      (!result.fins.empty() &&
       result.fins.back().end_vertex != result.fins.front().start_vertex)) {
    Fail(*loop, "halfedge",
         "Loop closure or vertex continuity is inconsistent");
    return Result<LoopBoundary>::Err(*error_);
  }
  return Result<LoopBoundary>::Ok(std::move(result));
}
Result<FaceBoundary> TopologyReader::ReadFace(
    NodeId id, const FaceTraversalOptions& options) {
  if (options.max_loops == 0 || options.max_fins == 0)
    return Result<FaceBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Face traversal requires positive loop and fin limits"});
  const auto* face = Node(id, 14);
  if (!face) return Result<FaceBoundary>::Err(*error_);
  FaceBoundary result;
  result.face = id;
  result.face_sense = face->topology_sense;
  NodeId current;
  if (!Reference(*face, "surface", 1006, true, result.surface) ||
      !Reference(*face, "loop", 15, true, current))
    return Result<FaceBoundary>::Err(*error_);
  std::unordered_set<uint32_t> visited;
  uint64_t remaining_fins = options.max_fins;
  while (current.value) {
    const auto& loop = *model_.FindNode(current);
    if (visited.contains(current.value)) {
      Fail(loop, "next", "Face loop list contains a cycle");
      return Result<FaceBoundary>::Err(*error_);
    }
    if (visited.size() >= options.max_loops || remaining_fins == 0)
      return Result<FaceBoundary>::Err(
          {ErrorCode::kResourceLimit, loop.source_offset,
           "Face traversal loop or aggregate fin limit exceeded"});
    visited.insert(current.value);
    NodeId owner, next;
    if (!Reference(loop, "face", 14, false, owner) ||
        !Reference(loop, "next", 15, true, next))
      return Result<FaceBoundary>::Err(*error_);
    if (owner != id) {
      Fail(loop, "face", "Listed loop belongs to another face");
      return Result<FaceBoundary>::Err(*error_);
    }
    auto boundary = ReadLoop(current, {remaining_fins});
    if (!boundary.ok()) {
      auto error = boundary.error();
      error.message = "Face " + std::to_string(id.value) + ", loop " +
                      std::to_string(current.value) + ": " + error.message;
      return Result<FaceBoundary>::Err(std::move(error));
    }
    const auto& value = boundary.value();
    remaining_fins -= value.isolated_vertex ? 1 : value.fins.size();
    result.loops.push_back(std::move(boundary).value());
    current = next;
  }
  return Result<FaceBoundary>::Ok(std::move(result));
}
bool TopologyReader::FaceUses(NodeId current, ShellFaceSide side,
                              ShellBoundary& result, ShellBudget& budget,
                              const ShellTraversalOptions& options) {
  const bool front = side == ShellFaceSide::kFront;
  const char* owner_field = front ? "front_shell" : "shell";
  const char* previous_field = front ? "previous_front" : "previous";
  const char* next_field = front ? "next_front" : "next";
  std::unordered_set<uint32_t> visited;
  NodeId previous;
  while (current.value) {
    const auto& face = *model_.FindNode(current);
    if (visited.contains(current.value))
      return Fail(face, next_field, "Shell face chain contains a cycle");
    if (result.face_uses.size() >= options.max_face_uses)
      return Fail(face, next_field, "Shell face-use limit exceeded",
                  ErrorCode::kResourceLimit);
    visited.insert(current.value);
    NodeId owner, backward, next;
    if (!Reference(face, owner_field, 13, false, owner) ||
        !Reference(face, previous_field, 14, true, backward) ||
        !Reference(face, next_field, 14, true, next))
      return false;
    if (owner != result.shell || backward != previous)
      return Fail(face, owner_field,
                  "Inconsistent shell ownership or reciprocal face links");
    auto cached = budget.faces.find(current.value);
    std::size_t index = result.faces.size();
    if (cached != budget.faces.end())
      index = cached->second;
    else {
      NodeId head;
      if (!Reference(face, "loop", 15, true, head)) return false;
      if (head.value && (budget.loops == 0 || budget.fins == 0))
        return Fail(face, "loop", "Shell aggregate loop or fin limit exceeded",
                    ErrorCode::kResourceLimit);
      // A face with no loops consumes no remaining loop/fin budget. ReadFace
      // still requires positive options even when that budget is exhausted.
      auto boundary = ReadFace(current, {std::max(budget.loops, uint64_t{1}),
                                         std::max(budget.fins, uint64_t{1})});
      if (!boundary.ok()) {
        auto error = boundary.error();
        error.message = "Shell " + std::to_string(result.shell.value) +
                        ", face " + std::to_string(current.value) + ": " +
                        error.message;
        error_ = std::move(error);
        return false;
      }
      budget.loops -= boundary.value().loops.size();
      for (const auto& loop : boundary.value().loops)
        budget.fins -= loop.isolated_vertex ? 1 : loop.fins.size();
      budget.faces.emplace(current.value, index);
      result.faces.push_back(std::move(boundary).value());
    }
    result.face_uses.push_back({index, side});
    previous = current;
    current = next;
  }
  return true;
}
bool TopologyReader::Dummy(const ModelNode& fin) {
  constexpr std::pair<const char*, uint16_t> roles[]{
      {"loop", 15}, {"forward", 17}, {"backward", 17}, {"curve", 1008}};
  for (const auto& [role, type] : roles) {
    NodeId target;
    if (!Reference(fin, role, type, true, target)) return false;
    if (target.value)
      return Fail(fin, role, "Dummy fin retains loop or curve links");
  }
  // Wire anchors can retain vertex-chain links even though their loop-chain
  // roles are null. Check the local vertex link rather than discarding it.
  return VertexNext(fin);
}
bool TopologyReader::VertexNext(const ModelNode& fin) {
  NodeId next, vertex, next_vertex, edge_id, primary_id, secondary_id;
  if (!Reference(fin, "next_at_vx", 17, true, next)) return false;
  if (!next.value) return true;
  const auto& target = *model_.FindNode(next);
  if (!Reference(fin, "vertex", 18, false, vertex) ||
      !Reference(target, "vertex", 18, false, next_vertex) ||
      !Reference(target, "edge", 16, false, edge_id))
    return false;
  const auto& edge = *model_.FindNode(edge_id);
  if (!Reference(edge, "halfedge", 17, false, primary_id)) return false;
  const auto& primary = *model_.FindNode(primary_id);
  if (!Reference(primary, "other", 17, false, secondary_id)) return false;
  if (next == fin.id || vertex != next_vertex ||
      (next != primary_id && next != secondary_id))
    return Fail(fin, "next_at_vx",
                "Vertex-chain link is not another anchor at the same vertex");
  return true;
}
bool TopologyReader::WireEdges(NodeId current, ShellBoundary& result,
                               uint64_t limit) {
  std::unordered_set<uint32_t> visited;
  NodeId previous;
  while (current.value) {
    const auto& edge = *model_.FindNode(current);
    if (visited.contains(current.value))
      return Fail(edge, "next", "Shell wire-edge chain contains a cycle");
    if (result.wire_edges.size() >= limit)
      return Fail(edge, "next", "Shell wire-edge limit exceeded",
                  ErrorCode::kResourceLimit);
    visited.insert(current.value);
    NodeId owner, backward, next, primary_id, secondary_id, close;
    if (!Reference(edge, "owner", 13, false, owner) ||
        !Reference(edge, "previous", 16, true, backward) ||
        !Reference(edge, "next", 16, true, next) ||
        !Reference(edge, "halfedge", 17, false, primary_id))
      return false;
    if (owner != result.shell || backward != previous)
      return Fail(edge, "owner/previous",
                  "Inconsistent shell ownership or reciprocal edge links");
    const auto& primary = *model_.FindNode(primary_id);
    if (!Reference(primary, "other", 17, false, secondary_id)) return false;
    const auto& secondary = *model_.FindNode(secondary_id);
    FinUse use;
    if (!EdgeUse(primary, current, secondary_id, use) || !Dummy(primary) ||
        !Dummy(secondary) || !Reference(secondary, "other", 17, false, close))
      return false;
    if (close != primary_id)
      return Fail(secondary, "other",
                  "Wire edge does not have a two-dummy-fin ring");
    result.wire_edges.push_back(
        {current, use.start_vertex, use.end_vertex, use.edge_curve});
    previous = current;
    current = next;
  }
  return true;
}
Result<ShellBoundary> TopologyReader::ReadShell(
    NodeId id, const ShellTraversalOptions& options, bool allow_zero_limits) {
  if (!allow_zero_limits && (!options.max_face_uses || !options.max_loops ||
                             !options.max_fins || !options.max_wire_edges))
    return Result<ShellBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Shell traversal requires positive limits"});
  const auto* shell = Node(id, 13);
  if (!shell) return Result<ShellBoundary>::Err(*error_);
  ShellBoundary result;
  result.shell = id;
  NodeId back, front, edges, vertex;
  if (!Reference(*shell, "region", 19, false, result.region) ||
      !Reference(*shell, "face", 14, true, back) ||
      !Reference(*shell, "front_face", 14, true, front) ||
      !Reference(*shell, "edge", 16, true, edges) ||
      !Reference(*shell, "vertex", 18, true, vertex))
    return Result<ShellBoundary>::Err(*error_);
  if (vertex.value) {
    if (back.value || front.value || edges.value) {
      Fail(*shell, "vertex", "Acorn shell also lists faces or edges");
      return Result<ShellBoundary>::Err(*error_);
    }
    const auto& node = *model_.FindNode(vertex);
    NodeId owner, fin, point;
    if (!Reference(node, "owner", 13, false, owner) ||
        !Reference(node, "halfedge", 17, true, fin) ||
        !Reference(node, "point", 29, false, point))
      return Result<ShellBoundary>::Err(*error_);
    if (owner != id || fin.value) {
      Fail(node, "owner/halfedge",
           "Invalid acorn vertex ownership or fin chain");
      return Result<ShellBoundary>::Err(*error_);
    }
    result.isolated_vertex = IsolatedShellVertex{vertex, point};
  } else {
    if (!back.value && !front.value && !edges.value) {
      Fail(*shell, "face/front_face/edge", "Shell has no listed entities");
      return Result<ShellBoundary>::Err(*error_);
    }
    ShellBudget budget{options.max_loops, options.max_fins, {}};
    if (!FaceUses(back, ShellFaceSide::kBack, result, budget, options) ||
        !FaceUses(front, ShellFaceSide::kFront, result, budget, options) ||
        !WireEdges(edges, result, options.max_wire_edges))
      return Result<ShellBoundary>::Err(*error_);
  }
  return Result<ShellBoundary>::Ok(std::move(result));
}
bool TopologyReader::SeparateShell(
    const ShellBoundary& shell, std::unordered_map<uint32_t, NodeId>& owners) {
  auto add = [&](NodeId entity) {
    if (!entity.value) return true;
    const auto [found, inserted] = owners.emplace(entity.value, shell.shell);
    if (inserted || found->second == shell.shell) return true;
    return Fail(*model_.FindNode(entity), "region shells",
                "Topology entity is shared by distinct shells in one region");
  };
  for (const auto& face : shell.faces) {
    if (!add(face.face)) return false;
    for (const auto& loop : face.loops) {
      if (loop.isolated_vertex && !add(loop.isolated_vertex->vertex))
        return false;
      for (const auto& fin : loop.fins)
        if (!add(fin.edge) || !add(fin.start_vertex) || !add(fin.end_vertex))
          return false;
    }
  }
  for (const auto& edge : shell.wire_edges)
    if (!add(edge.edge) || !add(edge.start_vertex) || !add(edge.end_vertex))
      return false;
  return !shell.isolated_vertex || add(shell.isolated_vertex->vertex);
}
Result<RegionBoundary> TopologyReader::ReadRegion(
    NodeId id, const RegionTraversalOptions& options, bool allow_zero_limits) {
  if (!allow_zero_limits &&
      (!options.max_shells || !options.max_face_uses || !options.max_loops ||
       !options.max_fins || !options.max_wire_edges))
    return Result<RegionBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Region traversal requires positive limits"});
  const auto* region = Node(id, 19);
  if (!region) return Result<RegionBoundary>::Err(*error_);
  if (!region->region_type ||
      (*region->region_type != 'S' && *region->region_type != 'V'))
    return Result<RegionBoundary>::Err(
        {ErrorCode::kUnsupportedFormat, region->source_offset,
         "Region type is unavailable or unknown"});
  RegionBoundary result;
  result.region = id;
  result.kind =
      *region->region_type == 'S' ? RegionKind::kSolid : RegionKind::kVoid;
  NodeId current;
  if (!Reference(*region, "body", 12, false, result.body) ||
      !Reference(*region, "shell", 13, true, current))
    return Result<RegionBoundary>::Err(*error_);
  std::unordered_set<uint32_t> visited;
  std::unordered_map<uint32_t, NodeId> owners;
  ShellTraversalOptions remaining{options.max_face_uses, options.max_loops,
                                  options.max_fins, options.max_wire_edges};
  while (current.value) {
    const auto& shell = *model_.FindNode(current);
    if (visited.contains(current.value)) {
      Fail(shell, "next", "Region shell list contains a cycle");
      return Result<RegionBoundary>::Err(*error_);
    }
    if (visited.size() >= options.max_shells)
      return Result<RegionBoundary>::Err({ErrorCode::kResourceLimit,
                                          shell.source_offset,
                                          "Region shell limit exceeded"});
    visited.insert(current.value);
    NodeId owner, next;
    if (!Reference(shell, "region", 19, false, owner) ||
        !Reference(shell, "next", 13, true, next))
      return Result<RegionBoundary>::Err(*error_);
    if (owner != id) {
      Fail(shell, "region", "Listed shell belongs to another region");
      return Result<RegionBoundary>::Err(*error_);
    }
    // Remaining aggregate budgets may reach zero. Nested traversal then permits
    // only shells whose actual corresponding counts are zero (e.g. an acorn).
    auto boundary = ReadShell(current, remaining, true);
    if (!boundary.ok()) {
      auto error = boundary.error();
      error.message = "Region " + std::to_string(id.value) + ", shell " +
                      std::to_string(current.value) + ": " + error.message;
      return Result<RegionBoundary>::Err(std::move(error));
    }
    const auto& value = boundary.value();
    if (!SeparateShell(value, owners))
      return Result<RegionBoundary>::Err(*error_);
    remaining.max_face_uses -= value.face_uses.size();
    remaining.max_wire_edges -= value.wire_edges.size();
    for (const auto& face : value.faces) {
      remaining.max_loops -= face.loops.size();
      for (const auto& loop : face.loops)
        remaining.max_fins -= loop.isolated_vertex ? 1 : loop.fins.size();
    }
    result.shells.push_back(std::move(boundary).value());
    current = next;
  }
  return Result<RegionBoundary>::Ok(std::move(result));
}
bool TopologyReader::Kind(const ModelNode& node, BodyKind& kind) {
  switch (node.body_type.value_or(0)) {
    case 1:
      kind = BodyKind::kSolid;
      break;
    case 2:
      kind = BodyKind::kWire;
      break;
    case 3:
      kind = BodyKind::kSheet;
      break;
    case 6:
      kind = BodyKind::kGeneral;
      break;
    default:
      return Fail(node, "body_type", "Body type is unavailable or unknown",
                  ErrorCode::kUnsupportedFormat);
  }
  return true;
}
Result<BodyBoundary> TopologyReader::ReadBody(
    NodeId id, const BodyTraversalOptions& options) {
  if (!PositiveBodyLimits(options))
    return Result<BodyBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Body traversal requires positive limits"});
  const auto* body = Node(id, 12);
  if (!body) return Result<BodyBoundary>::Err(*error_);
  BodyBoundary result;
  result.body = id;
  if (!Kind(*body, result.kind)) return Result<BodyBoundary>::Err(*error_);
  // Select the concrete owner type before Reference validates cardinality,
  // pointer status and target. Both WORLD and parent BODY are possible.
  uint16_t owner_type = 101;
  for (const auto& reference : body->references) {
    if (reference.field != "owner") continue;
    const auto* target = model_.FindNode(reference.target);
    if (target && target->node_type == 12) owner_type = 12;
  }
  NodeId owner, current;
  if (!Reference(*body, "owner", owner_type, true, owner))
    return Result<BodyBoundary>::Err(*error_);
  if (owner.value && owner_type == 12)
    return Result<BodyBoundary>::Err(
        {ErrorCode::kUnsupportedFormat, body->source_offset,
         "Child body traversal requires querying the compound parent"});
  if (!Reference(*body, "region", 19, true, current))
    return Result<BodyBoundary>::Err(*error_);
  RegionTraversalOptions remaining{options.max_shells, options.max_face_uses,
                                   options.max_loops, options.max_fins,
                                   options.max_wire_edges};
  std::unordered_set<uint32_t> visited;
  NodeId previous;
  while (current.value) {
    const auto& region = *model_.FindNode(current);
    if (visited.contains(current.value)) {
      Fail(region, "next", "Body region list contains a cycle");
      return Result<BodyBoundary>::Err(*error_);
    }
    if (visited.size() >= options.max_regions)
      return Result<BodyBoundary>::Err({ErrorCode::kResourceLimit,
                                        region.source_offset,
                                        "Body region limit exceeded"});
    visited.insert(current.value);
    NodeId parent, backward, next;
    if (!Reference(region, "body", 12, false, parent) ||
        !Reference(region, "previous", 19, true, backward) ||
        !Reference(region, "next", 19, true, next))
      return Result<BodyBoundary>::Err(*error_);
    if (parent != id || backward != previous) {
      Fail(region, "body/previous",
           "Inconsistent body ownership or reciprocal region links");
      return Result<BodyBoundary>::Err(*error_);
    }
    auto boundary = ReadRegion(current, remaining, true);
    if (!boundary.ok()) {
      auto error = boundary.error();
      error.message = "Body " + std::to_string(id.value) + ", region " +
                      std::to_string(current.value) + ": " + error.message;
      return Result<BodyBoundary>::Err(std::move(error));
    }
    const auto& value = boundary.value();
    remaining.max_shells -= value.shells.size();
    for (const auto& shell : value.shells) {
      remaining.max_face_uses -= shell.face_uses.size();
      remaining.max_wire_edges -= shell.wire_edges.size();
      for (const auto& face : shell.faces) {
        remaining.max_loops -= face.loops.size();
        for (const auto& loop : face.loops)
          remaining.max_fins -= loop.isolated_vertex ? 1 : loop.fins.size();
      }
    }
    result.regions.push_back(std::move(boundary).value());
    previous = current;
    current = next;
  }
  return Result<BodyBoundary>::Ok(std::move(result));
}

bool TopologyReader::Children(CompoundBodyBoundary& result, uint64_t limit) {
  const auto& boundary = result.boundary;
  const auto& parent = *model_.FindNode(boundary.body);
  NodeId current, previous;
  if (!Reference(parent, "child", 12, true, current)) return false;
  std::unordered_set<uint32_t> visited;
  std::size_t region_index = 0;
  bool mixed_kinds = false;
  while (current.value) {
    const auto& child = *model_.FindNode(current);
    if (current == parent.id || visited.contains(current.value))
      return Fail(child, "next",
                  "Compound child list contains a cycle or parent");
    if (visited.size() >= limit)
      return Fail(child, "next", "Compound child limit exceeded",
                  ErrorCode::kResourceLimit);
    visited.insert(current.value);
    NodeId owner, backward, next, nested, head;
    if (!Reference(child, "owner", 12, false, owner) ||
        !Reference(child, "previous", 12, true, backward) ||
        !Reference(child, "next", 12, true, next) ||
        !Reference(child, "child", 12, true, nested) ||
        !Reference(child, "region", 19, false, head))
      return false;
    if (owner != parent.id || backward != previous || nested.value)
      return Fail(child, "owner/previous/child",
                  "Inconsistent compound child ownership or links");
    ChildBodyRange range;
    range.body = current;
    range.first_region = region_index;
    if (!Kind(child, range.kind)) return false;
    if (region_index == boundary.regions.size() ||
        boundary.regions[region_index].region != head)
      return Fail(child, "region",
                  "Child region head does not match its ordered range");
    while (region_index < boundary.regions.size()) {
      const auto& region =
          *model_.FindNode(boundary.regions[region_index].region);
      NodeId region_owner;
      if (!Reference(region, "owner", 12, false, region_owner)) return false;
      if (region_owner != current) break;
      ++region_index;
    }
    range.region_count = region_index - range.first_region;
    if (!range.region_count)
      return Fail(child, "region",
                  "Child region head belongs to another child");
    if (!result.children.empty() && range.kind != result.children.front().kind)
      mixed_kinds = true;
    result.children.push_back(range);
    previous = current;
    current = next;
  }
  if (region_index != boundary.regions.size())
    return Fail(parent, "region",
                "Regions remain outside the listed child ranges");
  if (!result.children.empty()) {
    const auto expected =
        mixed_kinds ? BodyKind::kGeneral : result.children.front().kind;
    if (boundary.kind != expected)
      return Fail(parent, "body_type",
                  "Compound kind does not match its child kinds");
  }
  return true;
}
Result<CompoundBodyBoundary> TopologyReader::ReadCompound(
    NodeId id, const CompoundBodyTraversalOptions& options) {
  if (!options.max_children)
    return Result<CompoundBodyBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Compound traversal requires positive limits"});
  auto boundary = ReadBody(id, options.body);
  if (!boundary.ok())
    return Result<CompoundBodyBoundary>::Err(boundary.error());
  CompoundBodyBoundary result;
  result.boundary = std::move(boundary).value();
  if (!Children(result, options.max_children)) {
    auto error = *error_;
    error.message =
        "Compound body " + std::to_string(id.value) + ": " + error.message;
    return Result<CompoundBodyBoundary>::Err(std::move(error));
  }
  return Result<CompoundBodyBoundary>::Ok(std::move(result));
}

Result<ChildBodyBoundary> TopologyReader::ReadChild(
    NodeId id, const CompoundBodyTraversalOptions& options) {
  if (!options.max_children || !PositiveBodyLimits(options.body))
    return Result<ChildBodyBoundary>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Child traversal requires positive limits"});
  const auto* child = Node(id, 12);
  if (!child) return Result<ChildBodyBoundary>::Err(*error_);
  NodeId parent;
  if (!Reference(*child, "owner", 12, false, parent))
    return Result<ChildBodyBoundary>::Err(*error_);
  if (parent == id) {
    Fail(*child, "owner", "Child body cannot own itself");
    return Result<ChildBodyBoundary>::Err(*error_);
  }
  auto compound = ReadCompound(parent, options);
  if (!compound.ok()) {
    auto error = compound.error();
    error.message = "Child body " + std::to_string(id.value) + ", parent " +
                    std::to_string(parent.value) + ": " + error.message;
    return Result<ChildBodyBoundary>::Err(std::move(error));
  }
  auto& value = compound.value();
  const auto found = std::find_if(
      value.children.begin(), value.children.end(),
      [id](const ChildBodyRange& range) { return range.body == id; });
  if (found == value.children.end()) {
    Fail(*child, "owner", "Child body is not listed by its compound parent");
    return Result<ChildBodyBoundary>::Err(*error_);
  }
  ChildBodyBoundary result;
  result.parent = parent;
  result.boundary.body = id;
  result.boundary.kind = found->kind;
  result.boundary.regions.reserve(found->region_count);
  for (std::size_t i = 0; i < found->region_count; ++i)
    result.boundary.regions.push_back(
        std::move(value.boundary.regions[found->first_region + i]));
  return Result<ChildBodyBoundary>::Ok(std::move(result));
}

}  // namespace
Result<EdgeBoundary> TraverseEdgeBoundary(const Model& model, NodeId edge,
                                          const EdgeTraversalOptions& options) {
  return TopologyReader(model).ReadEdge(edge, options);
}

Result<LoopBoundary> TraverseLoopBoundary(const Model& model, NodeId loop,
                                          const LoopTraversalOptions& options) {
  return TopologyReader(model).ReadLoop(loop, options);
}
Result<FaceBoundary> TraverseFaceBoundary(const Model& model, NodeId face,
                                          const FaceTraversalOptions& options) {
  return TopologyReader(model).ReadFace(face, options);
}

Result<ShellBoundary> TraverseShellBoundary(
    const Model& model, NodeId shell, const ShellTraversalOptions& options) {
  return TopologyReader(model).ReadShell(shell, options);
}

Result<RegionBoundary> TraverseRegionBoundary(
    const Model& model, NodeId region, const RegionTraversalOptions& options) {
  return TopologyReader(model).ReadRegion(region, options);
}

Result<BodyBoundary> TraverseBodyBoundary(const Model& model, NodeId body,
                                          const BodyTraversalOptions& options) {
  return TopologyReader(model).ReadBody(body, options);
}

Result<CompoundBodyBoundary> TraverseCompoundBodyBoundary(
    const Model& model, NodeId body,
    const CompoundBodyTraversalOptions& options) {
  return TopologyReader(model).ReadCompound(body, options);
}

Result<ChildBodyBoundary> TraverseChildBodyBoundary(
    const Model& model, NodeId child,
    const CompoundBodyTraversalOptions& options) {
  return TopologyReader(model).ReadChild(child, options);
}

}  // namespace openparasolid
