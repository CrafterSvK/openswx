// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
/**
 * @file
 * @brief Bounded traversal of recorded B-rep connectivity.
 * @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_halfedge "XT 4.3.8:
 * Halfedge (p. 29)", @ref xt_fin "XT 5.3.9: Fin (p. 96)"
 */
#ifndef OPENPARASOLID_TOPOLOGY_H_
#define OPENPARASOLID_TOPOLOGY_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "openparasolid/model.h"

namespace openparasolid {
/// @brief Direction relative to the natural edge direction, not a surface
/// sense.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
enum class TopologySense {
  /// Same direction as the natural edge.
  kForward,
  /// Opposite to the natural edge.
  kReverse,
};
/// @brief Oriented use of an EDGE by a LOOP: the conceptual XT halfedge.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_fin "XT 5.3.9:
/// Fin (p. 96)"
struct FinUse {
  /// Source FIN identity (conceptual halfedge).
  NodeId fin;
  /// Source EDGE identity.
  NodeId edge;
  /// Oriented in loop order, independently of the edge's natural direction.
  /// Both are null for a vertex-free ring edge.
  NodeId start_vertex;
  /// Last endpoint in this use's documented direction; zero for a vertex-free
  /// ring.
  NodeId end_vertex;
  /// Source references, without substituting one for the other. Either may
  /// be null; typed geometry and tolerant-edge constraints are not certified.
  NodeId edge_curve;
  /// Source FIN curve reference, possibly null; not replaced by EDGE.curve.
  NodeId fin_curve;
  /// Loop direction relative to the natural edge direction.
  TopologySense sense = TopologySense::kForward;
};
/// @brief Radial FIN occurrence with its nullable source loop reference.
///
/// @see @ref xt_edgeschema "XT 5.3.11: Edge (p. 98)", @ref xt_dummy
/// "XT 5.3.9.1: Dummy fins (p. 96)"
struct EdgeFinUse {
  /// Source fin use oriented in its loop direction.
  FinUse use;
  /// Null for a dummy fin. Non-null uses retain their source loop ID.
  NodeId loop;
};
/// @brief Owned EDGE endpoints and ordered radial FIN ring, including dummy
/// anchors.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_edgeschema "XT 5.3.11:
/// Edge (p. 98)", @ref xt_dummy "XT 5.3.9.1: Dummy fins (p. 96)"
struct EdgeBoundary {
  /// Source EDGE identity.
  NodeId edge;
  /// Natural edge direction, determined by primary/secondary anchor fins.
  /// Both may be null for a ring edge.
  NodeId start_vertex;
  /// Last endpoint in this use's documented direction; zero for a vertex-free
  /// ring.
  NodeId end_vertex;
  /// Referenced curve identity.
  NodeId curve;
  /// EDGE.halfedge followed by FIN.other, including dummy fins. The first two
  /// entries are the positive primary and negative secondary anchor fins.
  std::vector<EdgeFinUse> fins;
};
/// @brief Positive aggregate budgets for one edge traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
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
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_edgeschema "XT 5.3.11:
/// Edge (p. 98)", @ref xt_dummy "XT 5.3.9.1: Dummy fins (p. 96)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param edge Stream-local EDGE identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<EdgeBoundary> TraverseEdgeBoundary(
    const Model& model, NodeId edge, const EdgeTraversalOptions& options = {});

/// @brief Vertex-chain anchor or isolated-loop FIN occurrence.
///
/// @see @ref xt_vertexchain "XT 5.3.9.2: Fin chain at a vertex (p. 97)", @ref
/// xt_isolated "XT 5.3.8.1: Isolated loops (p. 96)"
struct VertexFinUse {
  /// Source FIN identity (conceptual halfedge).
  NodeId fin;
  /// Null for a dummy edge anchor; non-null for real anchors/isolated loops.
  NodeId loop;
  /// Present only for a primary or secondary edge anchor. An isolated-loop
  /// entry has no edge use and its raw fin sense is insignificant.
  std::optional<FinUse> edge_use;
};
/// @brief Owned VERTEX metadata and listed incident anchor/isolated-loop fins.
///
/// @see @ref xt_vertexschema "XT 5.3.10: Vertex (p. 97)", @ref xt_vertexchain
/// "XT 5.3.9.2: Fin chain at a vertex (p. 97)"
struct VertexIncidence {
  /// Source VERTEX identity.
  NodeId vertex;
  /// Source BODY owner (possibly a compound parent), or SHELL for an acorn.
  NodeId owner;
  /// Referenced POINT node identity.
  NodeId point;
  /// VERTEX.halfedge / FIN.next_at_vx source order, null-terminated.
  std::vector<VertexFinUse> fins;
};
/// @brief Positive aggregate budgets for one vertex traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
struct VertexTraversalOptions {
  /// Upper bound on fin visits, including isolated-loop anchors.
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
///
/// @see @ref xt_vertexschema "XT 5.3.10: Vertex (p. 97)", @ref xt_vertexchain
/// "XT 5.3.9.2: Fin chain at a vertex (p. 97)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param vertex Stream-local VERTEX identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<VertexIncidence> TraverseVertexIncidence(
    const Model& model, NodeId vertex,
    const VertexTraversalOptions& options = {});

/// @brief Vertex-only LOOP represented by its schema FIN anchor.
///
/// @see @ref xt_isolated "XT 5.3.8.1: Isolated loops (p. 96)"
struct IsolatedLoopVertex {
  /// Source FIN identity (conceptual halfedge).
  NodeId fin;
  /// Source VERTEX identity.
  NodeId vertex;
};
/// @brief Owned LOOP boundary in transmitted fin order or as an isolated
/// vertex.
///
/// @see @ref xt_loop "XT 4.3.7: Loop (p. 28)", @ref xt_loopschema "XT 5.3.8:
/// Loop (p. 95)", @ref xt_isolated "XT 5.3.8.1: Isolated loops (p. 96)"
struct LoopBoundary {
  /// Source LOOP identity.
  NodeId loop;
  /// Source FACE identity.
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
/// @brief Positive aggregate budgets for one loop traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
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
///
/// @see @ref xt_loop "XT 4.3.7: Loop (p. 28)", @ref xt_loopschema "XT 5.3.8:
/// Loop (p. 95)", @ref xt_fin "XT 5.3.9: Fin (p. 96)", @ref xt_isolated
/// "XT 5.3.8.1: Isolated loops (p. 96)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param loop Stream-local LOOP identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<LoopBoundary> TraverseLoopBoundary(
    const Model& model, NodeId loop, const LoopTraversalOptions& options = {});

/// @brief Owned FACE surface reference and source-ordered loop boundaries.
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_faceschema "XT 5.3.7:
/// Face (p. 94)"
struct FaceBoundary {
  /// Source FACE identity.
  NodeId face;
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Unmodified optional FACE sense byte; absence does not imply a default.
  std::optional<uint8_t> face_sense;
  /// Source FACE.loop / LOOP.next order; empty for a face with a null loop
  /// head. This order does not classify outer boundaries and holes.
  std::vector<LoopBoundary> loops;
};
/// @brief Positive aggregate budgets for one face traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
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
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_faceschema "XT 5.3.7:
/// Face (p. 94)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param face Stream-local FACE identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FaceBoundary> TraverseFaceBoundary(
    const Model& model, NodeId face, const FaceTraversalOptions& options = {});

/// @brief Front or back relative to the face normal, not the surface normal.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)"
enum class ShellFaceSide {
  /// Back side relative to the face normal.
  kBack,
  /// Front side relative to the face normal.
  kFront,
};
/// @brief One side of a face used in the boundary of a region.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)"
struct ShellFaceUse {
  /// Index into ShellBoundary.faces, stable across copies and moves.
  std::size_t boundary = 0;
  /// Relative to the face normal, not the underlying surface's raw sense.
  ShellFaceSide side = ShellFaceSide::kBack;
};
/// @brief Shell wireframe EDGE with natural endpoints and source curve
/// reference.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_edge "XT 4.3.9: Edge
/// (p. 29)"
struct WireEdgeUse {
  /// Source EDGE identity.
  NodeId edge;
  /// Natural edge direction, without inferring the edge-to-curve sense.
  /// Both vertices may be null for a ring edge.
  NodeId start_vertex;
  /// Last endpoint in this use's documented direction; zero for a vertex-free
  /// ring.
  NodeId end_vertex;
  /// Referenced curve identity.
  NodeId curve;
};
/// @brief Single vertex of an acorn shell.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)"
struct IsolatedShellVertex {
  /// Source VERTEX identity.
  NodeId vertex;
  /// Referenced POINT node identity.
  NodeId point;
};
/// @brief Owned SHELL face-side uses, wireframe edges or an acorn vertex.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_shellschema "XT 5.3.6:
/// Shell (p. 94)"
struct ShellBoundary {
  /// Source SHELL identity.
  NodeId shell;
  /// Source REGION identity.
  NodeId region;
  /// One boundary per distinct listed face, in first-encounter order.
  std::vector<FaceBoundary> faces;
  /// Back-face chain first, then front-face chain; source order within each.
  /// A face may occur once on each side, sharing one boundary above.
  std::vector<ShellFaceUse> face_uses;
  /// Shell wireframe edge uses in source order.
  std::vector<WireEdgeUse> wire_edges;
  /// Present only for an acorn shell with no faces or wire edges.
  std::optional<IsolatedShellVertex> isolated_vertex;
};
/// @brief Positive aggregate budgets for one shell traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
struct ShellTraversalOptions {
  /// Upper bound on face-side occurrences, counting front and back separately.
  uint64_t max_face_uses = 10000;
  /// Aggregate limits across distinct face boundaries, not per side/use.
  uint64_t max_loops = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on shell wireframe edge entries.
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
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_shellschema "XT 5.3.6:
/// Shell (p. 94)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param shell Previously traversed shell, or its source identity for
/// traversal.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<ShellBoundary> TraverseShellBoundary(
    const Model& model, NodeId shell,
    const ShellTraversalOptions& options = {});

/// @brief Source face-fin incidence count for one edge; excludes dummy fins.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_fin "XT 5.3.9: Fin (p.
/// 96)"
struct ShellEdgeUseCount {
  /// Source EDGE identity.
  NodeId edge;
  /// Number of source face fins with forward sense on this edge.
  uint64_t forward_fins = 0;
  /// Number of source face fins with reverse sense on this edge.
  uint64_t reverse_fins = 0;
};
/// @brief Library graph classification of connected valence-one edge groups.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_edge "XT 4.3.9: Edge
/// (p. 29)"
enum class BoundaryGroupKind {
  /// Every recorded vertex has degree two, or a vertex-free ring.
  kCycle,
  /// Two degree-one ends and no branch vertices.
  kChain,
  /// At least one vertex has degree above two; no canonical path.
  kBranched,
};
/// @brief Library canonical path step with source FIN and directed endpoints.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_edge "XT 4.3.9:
/// Edge (p. 29)"
struct BoundaryPathEdge {
  /// Source EDGE identity.
  NodeId edge;
  /// Source FIN identity (conceptual halfedge).
  NodeId fin;
  /// First endpoint in this use's documented direction; zero for a vertex-free
  /// ring.
  NodeId start_vertex;
  /// Last endpoint in this use's documented direction; zero for a vertex-free
  /// ring.
  NodeId end_vertex;
  /// Relative to the natural edge direction, not the source fin or face side.
  TopologySense sense = TopologySense::kForward;
};
/// @brief Library grouping of valence-one edges within one face component.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_edge "XT 4.3.9: Edge
/// (p. 29)"
struct ShellBoundaryGroup {
  /// Library classification by recorded vertex degrees.
  BoundaryGroupKind kind = BoundaryGroupKind::kCycle;
  /// All IDs sorted; groups are ordered by smallest edge ID. Ring edges with
  /// no recorded vertices form separate cycles with an empty vertex list.
  std::vector<NodeId> edges;
  /// Sorted non-null vertices of this group.
  std::vector<NodeId> vertices;
  /// Sorted degree-one vertices of this boundary group.
  std::vector<NodeId> end_vertices;
  /// Sorted vertices whose degree in this boundary group exceeds two.
  std::vector<NodeId> branch_vertices;
  /// Canonical graph walk for chains/cycles; empty for branched groups. No
  /// inference of face orientation, parameter intervals or geometric closure.
  std::vector<BoundaryPathEdge> path;
};
/// @brief Library shared-edge-connected face component and its edge
/// diagnostics.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)"
struct ShellFaceComponent {
  /// Sorted source face IDs; components are ordered by their smallest face ID.
  std::vector<NodeId> faces;
  /// Sorted source edge IDs, each assigned to exactly one component.
  std::vector<NodeId> edges;
  /// Number of edges used by exactly one stored face fin.
  uint64_t boundary_edges = 0;
  /// Number of edges used by exactly two stored face fins; no manifold
  /// certificate.
  uint64_t paired_edges = 0;
  /// Number of edges used by more than two stored face fins.
  uint64_t higher_valence_edges = 0;
  /// Total source face fin uses, excluding dummy anchors and isolated loops.
  uint64_t face_fins = 0;
  /// Number of vertex-only loops, which have no edge incidence.
  uint64_t isolated_loops = 0;
  /// Number of valence-one edge groups classified as cycles.
  uint64_t boundary_cycles = 0;
  /// Number of valence-one edge groups classified as open chains.
  uint64_t boundary_chains = 0;
  /// Number of valence-one edge groups with a vertex degree above two.
  uint64_t branched_boundaries = 0;
  /// Connected valence-one edge groups, ordered by smallest edge ID.
  std::vector<ShellBoundaryGroup> boundary_groups;
};
/// @brief Library shell incidence summary; not a manifoldness certificate.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_manifold "XT 4.5:
/// Representation of manifold bodies (p. 31)"
struct ShellEdgeIncidence {
  /// Source SHELL identity.
  NodeId shell;
  /// Number of edges used by exactly one stored face fin.
  uint64_t boundary_edges = 0;
  /// Number of edges used by exactly two stored face fins; no manifold
  /// certificate.
  uint64_t paired_edges = 0;
  /// Number of edges used by more than two stored face fins.
  uint64_t higher_valence_edges = 0;
  /// Total source face fin uses, excluding dummy anchors and isolated loops.
  uint64_t face_fins = 0;
  /// Number of vertex-only loops, which have no edge incidence.
  uint64_t isolated_loops = 0;
  /// Sorted by edge ID; wires and dummy radial anchors are excluded.
  std::vector<ShellEdgeUseCount> edges;
  /// Number of shared-edge-connected face components.
  uint64_t face_components = 0;
  /// Face components ordered by smallest source face ID.
  std::vector<ShellFaceComponent> components;
  /// Number of valence-one edge groups classified as cycles.
  uint64_t boundary_cycles = 0;
  /// Number of valence-one edge groups classified as open chains.
  uint64_t boundary_chains = 0;
  /// Number of valence-one edge groups with a vertex degree above two.
  uint64_t branched_boundaries = 0;
};
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct ShellEdgeIncidenceOptions {
  /// Upper bound on distinct face boundaries.
  uint64_t max_faces = 10000;
  /// Upper bound on loop boundaries.
  uint64_t max_loops = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on edge entries or path steps for this query.
  uint64_t max_edges = 100000;
};
/// Counts source face-boundary incidence in a previously traversed shell.
/// Stored faces count once, even when used on both shell sides. One, two and
/// more fin uses classify boundary, paired and higher-valence edges. A seam's
/// two fins on one face count separately. Isolated loops consume the fin budget
/// but have no edge use. Positive aggregate limits bound work and storage.
/// Groups faces connected by shared edge IDs, including higher-valence edges.
/// Vertex-only contacts do not connect faces. Faces without edge fins each form
/// a component; wire/acorn shells without faces have none. Null or duplicate
/// face IDs are invalid records. Components retain edge IDs and incidence
/// counts; their counts sum to the shell totals. Component work/storage use the
/// same budgets, without another traversal or curve evaluation.
/// Groups valence-one edges by recorded vertex IDs within each face component.
/// Vertex degree two gives a cycle; degree one gives chain ends; degree above
/// two gives a branched group. Self-loops contribute degree two. A partially
/// missing endpoint pair is invalid. These are unordered graph diagnostics,
/// not face-oriented paths or geometric closure checks. Chains additionally
/// retain a walk from the smaller endpoint; cycles start with the smallest edge
/// from its smaller vertex. Vertex-free rings and self-loops use forward sense.
/// Branched groups have no path. Existing budgets bound path work/storage.
/// Does not revalidate connectivity, orient face sides or compare geometry.
/// Paired edges alone do not certify manifold vertices, geometric
/// connectedness, shell closure, hole orientation or volume. Empty/wire/acorn
/// shells have no entries.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_edge "XT 4.3.9: Edge
/// (p. 29)", @ref xt_manifold "XT 4.5: Representation of manifold bodies (p.
/// 31)"
///
/// @param shell Previously traversed shell, or its source identity for
/// traversal.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<ShellEdgeIncidence> AnalyzeShellEdgeIncidence(
    const ShellBoundary& shell, const ShellEdgeIncidenceOptions& options = {});

/// @brief Known source REGION material classification, solid or void.
///
/// @see @ref xt_region "XT 4.3.4: Region (p. 27)", @ref xt_regionschema
/// "XT 5.3.5: Region (p. 93)"
enum class RegionKind {
  /// Source material region (REGION.type S).
  kSolid,
  /// Source void region (REGION.type V); boundedness is not inferred.
  kVoid,
};
/// @brief Owned REGION shell list with the transmitted body reference.
///
/// @see @ref xt_region "XT 4.3.4: Region (p. 27)", @ref xt_regionschema
/// "XT 5.3.5: Region (p. 93)"
struct RegionBoundary {
  /// Source REGION identity.
  NodeId region;
  /// Source REGION.body; may reference a compound parent for child regions.
  /// Does not resolve or infer an effective child-body owner.
  NodeId body;
  /// Source material type: solid or void; no boundedness inference.
  RegionKind kind = RegionKind::kVoid;
  /// REGION.shell / SHELL.next order, without outer/inner classification.
  std::vector<ShellBoundary> shells;
};
/// @brief Positive aggregate budgets for one region traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
struct RegionTraversalOptions {
  /// Upper bound on shell boundaries.
  uint64_t max_shells = 10000;
  /// Aggregate counts over the whole region. Face uses count both sides;
  /// loop/fin data count once per stored face boundary within a shell.
  uint64_t max_face_uses = 10000;
  /// Upper bound on loop boundaries.
  uint64_t max_loops = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on shell wireframe edge entries.
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
///
/// @see @ref xt_region "XT 4.3.4: Region (p. 27)", @ref xt_regionschema
/// "XT 5.3.5: Region (p. 93)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param region Stream-local REGION identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<RegionBoundary> TraverseRegionBoundary(
    const Model& model, NodeId region,
    const RegionTraversalOptions& options = {});

/// @brief Reviewed BODY dimensional classification, including mixed general
/// bodies.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_manifold "XT 4.5:
/// Representation of manifold bodies (p. 31)"
enum class BodyKind {
  /// Source solid-body classification; not a validity certificate.
  kSolid,
  /// Source wire-body classification, including acorn bodies.
  kWire,
  /// Source sheet-body classification.
  kSheet,
  /// Source general or mixed-dimensional body classification.
  kGeneral,
};
/// @brief Owned BODY region hierarchy, preserving transmitted references.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_bodyschema "XT 5.3.4:
/// Body (p. 88)"
struct BodyBoundary {
  /// Source BODY identity.
  NodeId body;
  /// Reviewed source BODY dimensional type.
  BodyKind kind = BodyKind::kGeneral;
  /// BODY.region / REGION.next order, including nested shell boundaries.
  /// For compound parents this is the combined chain of all child regions.
  /// Child grouping and exterior-region classification are not inferred.
  std::vector<RegionBoundary> regions;
};
/// @brief Positive aggregate budgets for one body traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
struct BodyTraversalOptions {
  /// Upper bound on body regions.
  uint64_t max_regions = 10000;
  /// Aggregate counts over all stored region boundaries, including repeated
  /// face boundaries shared between different regions.
  uint64_t max_shells = 10000;
  /// Upper bound on face-side occurrences, counting front and back separately.
  uint64_t max_face_uses = 10000;
  /// Upper bound on loop boundaries.
  uint64_t max_loops = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on shell wireframe edge entries.
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
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_bodyschema "XT 5.3.4:
/// Body (p. 88)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyBoundary> TraverseBodyBoundary(
    const Model& model, NodeId body, const BodyTraversalOptions& options = {});

/// @brief Library range selecting a child representation within the parent
/// region chain.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_bodyschema
/// "XT 5.3.4: Body (p. 88)"
struct ChildBodyRange {
  /// Source BODY identity.
  NodeId body;
  /// Reviewed dimensional type of this child BODY.
  BodyKind kind = BodyKind::kGeneral;
  /// Contiguous range in CompoundBodyBoundary.boundary.regions. Indices remain
  /// valid across copies and moves; nested boundary data are stored only once.
  std::size_t first_region = 0;
  /// Number of contiguous region entries belonging to this child.
  std::size_t region_count = 0;
};
/// @brief Owned compound parent boundary and checked child region ranges.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_bodyschema
/// "XT 5.3.4: Body (p. 88)"
struct CompoundBodyBoundary {
  /// Owned parent boundary containing the combined child-region chain.
  BodyBoundary boundary;
  /// BODY.child / child BODY.next order, with reciprocal previous links.
  std::vector<ChildBodyRange> children;
};
/// @brief Positive aggregate budgets for one compoundbody traversal query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)"
struct CompoundBodyTraversalOptions {
  /// Aggregate budgets for the whole compound parent boundary.
  BodyTraversalOptions body;
  /// Upper bound on child-body metadata entries.
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
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_bodyschema
/// "XT 5.3.4: Body (p. 88)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<CompoundBodyBoundary> TraverseCompoundBodyBoundary(
    const Model& model, NodeId body,
    const CompoundBodyTraversalOptions& options = {});

/// @brief Owned selected child boundary with its transmitted parent identity.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_bodyschema
/// "XT 5.3.4: Body (p. 88)"
struct ChildBodyBoundary {
  /// Source compound parent BODY identity.
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
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_bodyschema
/// "XT 5.3.4: Body (p. 88)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param child Stream-local child BODY identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<ChildBodyBoundary> TraverseChildBodyBoundary(
    const Model& model, NodeId child,
    const CompoundBodyTraversalOptions& options = {});

}  // namespace openparasolid

#endif  // OPENPARASOLID_TOPOLOGY_H_
