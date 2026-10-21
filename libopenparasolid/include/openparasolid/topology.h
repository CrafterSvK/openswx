// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOPOLOGY_H_
#define OPENPARASOLID_TOPOLOGY_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "openparasolid/model.h"

namespace openparasolid {
enum class TopologySense { kForward, kReverse };
struct FinUse {
  NodeId fin;
  NodeId edge;
  /// Oriented in loop order, independently of the edge's natural direction.
  /// Both are null for a vertex-free ring edge.
  NodeId start_vertex;
  NodeId end_vertex;
  /// Source references, without substituting one for the other. Either may
  /// be null; typed geometry and tolerant-edge constraints are not certified.
  NodeId edge_curve;
  NodeId fin_curve;
  TopologySense sense = TopologySense::kForward;
};
struct IsolatedLoopVertex {
  NodeId fin;
  NodeId vertex;
};
struct LoopBoundary {
  NodeId loop;
  NodeId face;
  /// May be null when the source face has no surface.
  NodeId surface;
  /// Raw FACE sense, including unknown bytes; no default is inferred.
  std::optional<uint8_t> face_sense;
  /// Starts at LOOP.halfedge and follows FIN.forward, without
  /// rotation/reversal.
  std::vector<FinUse> fins;
  /// An isolated vertex loop has no edge uses. Its fin sense is insignificant.
  std::optional<IsolatedLoopVertex> isolated_vertex;
};
struct LoopTraversalOptions {
  /// Positive limit on visited loop fins, including an isolated vertex's fin.
  uint64_t max_fins = 100000;
};

/// Reads one LOOP's ordered boundary from the owned model graph. Verifies
/// reciprocal forward/backward links, loop ownership, closure and vertex
/// continuity. Edge endpoints come from its positive primary fin and negative
/// secondary fin, including dummy fins; the FIN vertex is the forward vertex.
/// An edge may occur at most once per fin sense. A vertex-free ring edge must
/// be the only fin in the loop. Isolated vertex loops retain their own fin.
/// Ambiguous roles, wrong/missing targets and invalid connectivity
/// return kInvalidRecord; missing roles, unknown senses or unreviewed classes
/// kUnsupportedFormat; visit exhaustion kResourceLimit; zero max_fins
/// kInvalidOptions. Results own their data; the model is unchanged.
/// Does not certify face-list membership, full radial rings, vertex chains,
/// shell/body manifoldness, point/curve/surface geometry or tolerance bounds.
/// No edge-to-curve sense, curve parameters, outer/hole classification, face
/// normal orientation, placement or unit conversions are inferred.
[[nodiscard]] Result<LoopBoundary> TraverseLoopBoundary(
    const Model& model, NodeId loop, const LoopTraversalOptions& options = {});
}  // namespace openparasolid

#endif  // OPENPARASOLID_TOPOLOGY_H_
