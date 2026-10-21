// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOPOLOGY_H_
#define OPENPARASOLID_TOPOLOGY_H_

#include <cstddef>
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

struct FaceBoundary {
  NodeId face;
  NodeId surface;
  std::optional<uint8_t> face_sense;
  /// Source FACE.loop / LOOP.next order; empty for a face with a null loop
  /// head. This order does not classify outer boundaries and holes.
  std::vector<LoopBoundary> loops;
};
struct FaceTraversalOptions {
  /// Positive bound on the number of listed loops in this face.
  uint64_t max_loops = 10000;
  /// Positive aggregate bound on loop fins across the entire face, counting
  /// one for every isolated vertex loop. Not a separate allowance per loop.
  uint64_t max_fins = 100000;
};

/// Reads a FACE's null-terminated loop list and all listed boundaries. Retains
/// the nullable surface and raw/absent face sense. A face may have no loops.
/// Verifies list acyclicity, loop-to-face ownership and every loop's local
/// connectivity through the same rules as TraverseLoopBoundary. Resource
/// limits apply to the whole face; errors never return a partial boundary.
/// Results own their data; the source model is unchanged.
/// Both limits must be positive; error codes follow TraverseLoopBoundary.
/// Does not scan the whole model for unlisted loops claiming this face, certify
/// disjointness/containment of loops, orient normals or validate shell/body
/// topology or geometry. No outer/hole classification or tolerance is inferred.
[[nodiscard]] Result<FaceBoundary> TraverseFaceBoundary(
    const Model& model, NodeId face, const FaceTraversalOptions& options = {});

enum class ShellFaceSide { kBack, kFront };
struct ShellFaceUse {
  /// Index into ShellBoundary.faces, stable across copies and moves.
  std::size_t boundary = 0;
  /// Relative to the face normal, not the underlying surface's raw sense.
  ShellFaceSide side = ShellFaceSide::kBack;
};
struct WireEdgeUse {
  NodeId edge;
  /// Natural edge direction, without inferring the edge-to-curve sense.
  /// Both vertices may be null for a ring edge.
  NodeId start_vertex;
  NodeId end_vertex;
  NodeId curve;
};
struct IsolatedShellVertex {
  NodeId vertex;
  NodeId point;
};
struct ShellBoundary {
  NodeId shell;
  NodeId region;
  /// One boundary per distinct listed face, in first-encounter order.
  std::vector<FaceBoundary> faces;
  /// Back-face chain first, then front-face chain; source order within each.
  /// A face may occur once on each side, sharing one boundary above.
  std::vector<ShellFaceUse> face_uses;
  std::vector<WireEdgeUse> wire_edges;
  /// Present only for an acorn shell with no faces or wire edges.
  std::optional<IsolatedShellVertex> isolated_vertex;
};
struct ShellTraversalOptions {
  uint64_t max_face_uses = 10000;
  /// Aggregate limits across distinct face boundaries, not per side/use.
  uint64_t max_loops = 10000;
  uint64_t max_fins = 100000;
  uint64_t max_wire_edges = 100000;
};

/// Reads both null-terminated face-use chains and the wire-edge chain of a
/// SHELL, or its isolated vertex. Verifies selected ownership, reciprocal list
/// links, cycles and nested face/loop connectivity. A wire edge requires its
/// positive/negative anchor fins to be the two dummy fins of a closed radial
/// ring. Non-null wire-anchor vertex links are checked locally. Acorn
/// vertices must belong to the shell and have no fin-chain head.
/// All limits are positive and aggregate over this query; isolated loop fins
/// count toward max_fins, while wire anchors are bounded by max_wire_edges.
/// Errors follow the other traversal APIs and never return partial results.
/// Results own their data; the model is unchanged. The obsolete SHELL.body
/// field is ignored. No complete vertex chains, region-list membership,
/// connectedness, manifoldness,
/// enclosed volume or geometric/tolerance validity is certified. Unlisted
/// entities, outer/hole classification and face-normal flips are not inferred.
[[nodiscard]] Result<ShellBoundary> TraverseShellBoundary(
    const Model& model, NodeId shell,
    const ShellTraversalOptions& options = {});

enum class RegionKind { kSolid, kVoid };
struct RegionBoundary {
  NodeId region;
  /// Source REGION.body; may reference a compound parent for child regions.
  /// Does not resolve or infer an effective child-body owner.
  NodeId body;
  RegionKind kind = RegionKind::kVoid;
  /// REGION.shell / SHELL.next order, without outer/inner classification.
  std::vector<ShellBoundary> shells;
};
struct RegionTraversalOptions {
  uint64_t max_shells = 10000;
  /// Aggregate counts over the whole region. Face uses count both sides;
  /// loop/fin data count once per stored face boundary within a shell.
  uint64_t max_face_uses = 10000;
  uint64_t max_loops = 10000;
  uint64_t max_fins = 100000;
  uint64_t max_wire_edges = 100000;
};

/// Reads a REGION's null-terminated shell list and nested boundaries. Requires
/// known solid ('S') or void ('V') source type and a non-null BODY reference.
/// Unknown/unavailable type returns kUnsupportedFormat; raw model metadata is
/// retained. A null shell head is allowed without inferring boundedness.
/// Checks list cycles, shell-to-region ownership, nested connectivity and
/// distinct-shell separation by face/edge/vertex IDs. Reuse within one shell
/// retains existing rules, including two-sided faces. Limits are positive and
/// aggregate over this query, including acorn shells and isolated loop fins.
/// Errors follow other traversal APIs without returning partial results.
/// Results own their data; the model is unchanged. Does not scan for unlisted
/// entities, resolve child ownership or certify body-list membership,
/// geometric disjointness, positive/outer shell identity, manifoldness,
/// volume, exterior/finite region status or complete vertex/radial incidence.
[[nodiscard]] Result<RegionBoundary> TraverseRegionBoundary(
    const Model& model, NodeId region,
    const RegionTraversalOptions& options = {});

enum class BodyKind { kSolid, kWire, kSheet, kGeneral };
struct BodyBoundary {
  NodeId body;
  BodyKind kind = BodyKind::kGeneral;
  /// BODY.region / REGION.next order, including nested shell boundaries.
  /// For compound parents this is the combined chain of all child regions.
  /// Child grouping and exterior-region classification are not inferred.
  std::vector<RegionBoundary> regions;
};
struct BodyTraversalOptions {
  uint64_t max_regions = 10000;
  /// Aggregate counts over all stored region boundaries, including repeated
  /// face boundaries shared between different regions.
  uint64_t max_shells = 10000;
  uint64_t max_face_uses = 10000;
  uint64_t max_loops = 10000;
  uint64_t max_fins = 100000;
  uint64_t max_wire_edges = 100000;
};

/// Reads a standard or compound-parent BODY's null-terminated region chain.
/// Requires a known scalar byte body_type (1, 2, 3 or 6). BODY.owner must be
/// null or reference a WORLD; querying a child BODY returns kUnsupportedFormat
/// because its region chain continues through its siblings. Query the parent
/// instead. Validates region back-references and reciprocal previous/next
/// links, cycles and nested boundaries. A null region head is accepted
/// structurally (e.g. an empty compound), without asserting body validity.
/// All limits must be positive and apply across the entire result. Nested
/// failures retain code/offset and add body/region context. Results own their
/// data; errors never return partial results and the model is unchanged.
/// The obsolete BODY.shell and unrelated geometry/edge/vertex lists are not
/// read. Does not resolve child grouping, scan for unlisted entities or prove
/// exterior-region identity, full incidence, manifoldness, volume or geometric
/// validity. Shared faces across regions are retained as separate boundaries.
[[nodiscard]] Result<BodyBoundary> TraverseBodyBoundary(
    const Model& model, NodeId body, const BodyTraversalOptions& options = {});

}  // namespace openparasolid

#endif  // OPENPARASOLID_TOPOLOGY_H_
