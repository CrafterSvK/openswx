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
struct EdgeFinUse {
  FinUse use;
  /// Null for a dummy fin. Non-null uses retain their source loop ID.
  NodeId loop;
};
struct EdgeBoundary {
  NodeId edge;
  /// Natural edge direction, determined by primary/secondary anchor fins.
  /// Both may be null for a ring edge.
  NodeId start_vertex;
  NodeId end_vertex;
  NodeId curve;
  /// EDGE.halfedge followed by FIN.other, including dummy fins. The first two
  /// entries are the positive primary and negative secondary anchor fins.
  std::vector<EdgeFinUse> fins;
};
struct EdgeTraversalOptions {
  /// Positive bound on all visited radial fins, including dummy anchors.
  uint64_t max_fins = 100000;
};

/// Reads an EDGE's full ordered radial fin ring, with natural endpoints and
/// source edge/fin curve references. Requires at least the distinct positive
/// primary and negative secondary anchors; every listed fin must reference
/// this edge and its oriented endpoint. The FIN.other ring must close at its
/// primary head without repeating another fin. Additional fins may share a
/// sense; no manifoldness or angular order is inferred.
/// A dummy must be an anchor, have null loop/forward/backward/curve roles and
/// locally valid nullable vertex-chain links. Real fins require a loop and
/// locally reciprocal forward/backward links within that loop. Does not walk
/// entire loops or vertex chains, prove complete incidence by scanning for
/// unlisted fins, or validate geometry, tolerances or edge-to-curve sense.
/// Limits include dummy fins. Zero max_fins returns kInvalidOptions, exhaustion
/// kResourceLimit, malformed connectivity/targets kInvalidRecord, unavailable
/// roles/senses or unreviewed classes kUnsupportedFormat. Errors never return
/// partial results. Results own their data and leave the model unchanged.
[[nodiscard]] Result<EdgeBoundary> TraverseEdgeBoundary(
    const Model& model, NodeId edge, const EdgeTraversalOptions& options = {});

struct VertexFinUse {
  NodeId fin;
  /// Null for a dummy edge anchor; non-null for real anchors/isolated loops.
  NodeId loop;
  /// Present only for a primary or secondary edge anchor. An isolated-loop
  /// entry has no edge use and its raw fin sense is insignificant.
  std::optional<FinUse> edge_use;
};
struct VertexIncidence {
  NodeId vertex;
  /// Source BODY owner (possibly a compound parent), or SHELL for an acorn.
  NodeId owner;
  NodeId point;
  /// VERTEX.halfedge / FIN.next_at_vx source order, null-terminated.
  std::vector<VertexFinUse> fins;
};
struct VertexTraversalOptions {
  uint64_t max_fins = 100000;
};

/// Reads a vertex's listed incident anchor/isolated-loop fins. Requires a
/// non-null POINT and a BODY or SHELL owner. A SHELL-owned acorn must have a
/// null fin head; a BODY-owned null head is accepted structurally without
/// asserting complete incidence. Every listed fin must reference this vertex.
/// Edge entries must be primary/secondary anchors with consistent endpoints,
/// senses and local loop/dummy links. Isolated-loop fins must be their loop's
/// head with self forward/backward links and null edge/other/curve; their sense
/// is ignored. FIN.next_at_vx must terminate without repeating a fin.
/// max_fins is positive and counts all entries, including dummy and isolated
/// fins. Zero returns kInvalidOptions, exhaustion kResourceLimit, malformed
/// topology/targets kInvalidRecord, unavailable roles/senses or unreviewed
/// classes kUnsupportedFormat. No error returns partial data; results own
/// their data and leave the model unchanged. Does not scan for unlisted fins,
/// certify owner-list membership, walk complete edge rings/loops or establish
/// geometric, tolerance, manifoldness or body validity. Point data may remain
/// untyped and child-body ownership is not inferred.
[[nodiscard]] Result<VertexIncidence> TraverseVertexIncidence(
    const Model& model, NodeId vertex,
    const VertexTraversalOptions& options = {});

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

struct ShellEdgeUseCount {
  NodeId edge;
  uint64_t forward_fins = 0;
  uint64_t reverse_fins = 0;
};
struct ShellFaceComponent {
  // Sorted source face IDs; components are ordered by their smallest face ID.
  std::vector<NodeId> faces;
};
struct ShellEdgeIncidence {
  NodeId shell;
  uint64_t boundary_edges = 0;
  uint64_t paired_edges = 0;
  uint64_t higher_valence_edges = 0;
  uint64_t face_fins = 0;
  uint64_t isolated_loops = 0;
  // Sorted by edge ID; wires and dummy radial anchors are excluded.
  std::vector<ShellEdgeUseCount> edges;
  uint64_t face_components = 0;
  std::vector<ShellFaceComponent> components;
};
struct ShellEdgeIncidenceOptions {
  uint64_t max_faces = 10000;
  uint64_t max_loops = 10000;
  uint64_t max_fins = 100000;
  uint64_t max_edges = 100000;
};
// Counts source face-boundary incidence in a previously traversed shell.
// Stored faces count once, even when used on both shell sides. One, two and
// more fin uses classify boundary, paired and higher-valence edges. A seam's
// two fins on one face count separately. Isolated loops consume the fin budget
// but have no edge use. Positive aggregate limits bound work and storage.
// Groups faces connected by shared edge IDs, including higher-valence edges.
// Vertex-only contacts do not connect faces. Faces without edge fins each form
// a component; wire/acorn shells without faces have none. Null or duplicate
// face IDs are invalid records. Component work/storage use the same budgets.
// Does not revalidate connectivity, orient face sides or compare geometry.
// Paired edges alone do not certify manifold vertices, geometric connectedness,
// shell closure, hole orientation or volume. Empty/wire/acorn shells have no
// entries.
[[nodiscard]] Result<ShellEdgeIncidence> AnalyzeShellEdgeIncidence(
    const ShellBoundary& shell, const ShellEdgeIncidenceOptions& options = {});

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
/// instead, or use TraverseChildBodyBoundary. Validates region back-references
/// and reciprocal previous/next
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

struct ChildBodyRange {
  NodeId body;
  BodyKind kind = BodyKind::kGeneral;
  /// Contiguous range in CompoundBodyBoundary.boundary.regions. Indices remain
  /// valid across copies and moves; nested boundary data are stored only once.
  std::size_t first_region = 0;
  std::size_t region_count = 0;
};
struct CompoundBodyBoundary {
  BodyBoundary boundary;
  /// BODY.child / child BODY.next order, with reciprocal previous links.
  std::vector<ChildBodyRange> children;
};
struct CompoundBodyTraversalOptions {
  BodyTraversalOptions body;
  uint64_t max_children = 10000;
};

/// Reads a compound parent's combined boundary and groups its regions by child.
/// In addition to TraverseBodyBoundary checks, requires BODY.child and every
/// child's owner/previous/next/child/region roles and REGION.owner. Missing or
/// unreviewed roles return kUnsupportedFormat. Each child must belong to the
/// parent, have no children, and own a nonempty contiguous range starting at
/// its region head. Ranges cover the entire parent's region chain in child
/// order. Parent kind must equal the common child kind, or general for mixed
/// kinds. Empty compounds require null child and region heads.
/// All limits are positive. Body limits apply to the whole nested boundary;
/// max_children additionally bounds child metadata. Errors follow the other
/// traversal APIs, preserving nested error codes/offsets without partial data.
/// Results own their data and leave the model unchanged. Direct child queries
/// use TraverseChildBodyBoundary below. Does not scan for unlisted children,
/// certify geometric validity or classify exterior regions. An older schema
/// lacking BODY.child or REGION.owner remains usable through
/// TraverseBodyBoundary.
[[nodiscard]] Result<CompoundBodyBoundary> TraverseCompoundBodyBoundary(
    const Model& model, NodeId body,
    const CompoundBodyTraversalOptions& options = {});

struct ChildBodyBoundary {
  NodeId parent;
  /// Identifies the child and contains only its region range. The contained
  /// RegionBoundary.body values retain the transmitted compound parent ID.
  BodyBoundary boundary;
};

/// Reads one child BODY through a fully checked compound-parent boundary.
/// Requires a non-null BODY.owner pointing to its parent and membership in
/// that parent's child list. Moves only the selected child's region range
/// into the owned result; no sibling regions are returned. Preserves source
/// REGION.body references to the parent, without rewriting topology IDs.
/// All options and aggregate limits apply to the entire parent, including
/// siblings, as in TraverseCompoundBodyBoundary. Invalid sibling topology or
/// grouping therefore blocks the query. Options are checked before lookup.
/// Missing/unlisted/wrong-type targets and malformed ownership return
/// kInvalidRecord; unavailable metadata and unreviewed classes remain
/// kUnsupportedFormat. Nested errors retain code/offset with child/parent
/// context. Errors never return partial results and the model is unchanged.
/// No geometry, placement, exterior status or additional body validity is
/// inferred. TraverseBodyBoundary retains its standard/parent-only contract.
[[nodiscard]] Result<ChildBodyBoundary> TraverseChildBodyBoundary(
    const Model& model, NodeId child,
    const CompoundBodyTraversalOptions& options = {});

}  // namespace openparasolid

#endif  // OPENPARASOLID_TOPOLOGY_H_
