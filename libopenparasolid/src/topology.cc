// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/topology.h"

#include <algorithm>
#include <initializer_list>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace openparasolid {
namespace {
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
  Result<FaceBoundary> ReadFace(NodeId id, const FaceTraversalOptions& options);

 private:
  bool Fail(const ModelNode& node, std::string_view field, const char* message,
            ErrorCode code = ErrorCode::kInvalidRecord);
  const ModelNode* Node(NodeId id, uint16_t type);
  bool Reference(const ModelNode& node, std::string_view name, uint16_t type,
                 bool nullable, NodeId& target);
  bool Sense(const ModelNode& node, TopologySense& sense);
  bool EdgeUse(const ModelNode& fin, NodeId edge, NodeId other, FinUse& use);
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
}  // namespace
Result<LoopBoundary> TraverseLoopBoundary(const Model& model, NodeId loop,
                                          const LoopTraversalOptions& options) {
  return TopologyReader(model).ReadLoop(loop, options);
}
Result<FaceBoundary> TraverseFaceBoundary(const Model& model, NodeId face,
                                          const FaceTraversalOptions& options) {
  return TopologyReader(model).ReadFace(face, options);
}

}  // namespace openparasolid
