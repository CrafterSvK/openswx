// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
/**
 * @file
 * @brief Topology validation and exact boundary geometry binding.
 * @see @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p. 92)",
 * @ref xt_loop "XT 4.3.7: Loop (p. 28)", @ref xt_halfedge "XT 4.3.8: Halfedge
 * (p. 29)", @ref xt_senses "XT 5.2.8: Curve, surface and lattice senses (p.
 * 82)"
 */
#ifndef OPENPARASOLID_BREP_H_
#define OPENPARASOLID_BREP_H_
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "openparasolid/evaluate.h"
#include "openparasolid/topology.h"
namespace openparasolid {
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BodyValidationOptions {
  /// Body/compound traversal budgets used before incidence validation.
  CompoundBodyTraversalOptions traversal;
  /// Upper bound on model nodes scanned during body validation.
  uint64_t max_model_nodes = 300000;
  /// Upper bound on model references scanned during body validation.
  uint64_t max_model_references = 5000000;
  /// Upper bound on radial fin visits, including dummy anchors.
  uint64_t max_radial_fins = 100000;
  /// Upper bound on vertex-chain fin visits.
  uint64_t max_vertex_fins = 100000;
};
/// @brief Owned recorded body incidence after bounded structural validation.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_fin "XT 5.3.9: Fin
/// (p. 96)", @ref xt_vertexchain "XT 5.3.9.2: Fin chain at a vertex (p. 97)"
struct ValidatedBodyTopology {
  /// Owned selected-body region/shell/face hierarchy.
  BodyBoundary boundary;
  /// Validated complete edge radial boundaries.
  std::vector<EdgeBoundary> edges;
  /// Validated complete selected vertex incidence.
  std::vector<VertexIncidence> vertices;
};
/// Checks the complete selected body's recorded incidence against a bounded
/// scan of the model: owner lists, both face sides, loops, radial fins and
/// vertex chains. Compound child groups are checked when present. Results own
/// their data. No geometry, manifoldness or positive-volume certificate.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_fin "XT 5.3.9: Fin
/// (p. 96)", @ref xt_vertexchain "XT 5.3.9.2: Fin chain at a vertex (p. 97)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<ValidatedBodyTopology> ValidateBodyTopology(
    const Model& model, NodeId body, const BodyValidationOptions& options = {});
/// @brief Caller-selected directed curve interval for one source FIN.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_trim
/// "XT 5.2.1.6: Trimmed_curve (p. 49)"
struct FinParameterInterval {
  /// Source FIN identity (conceptual halfedge).
  NodeId fin;
  /// Directed, unwrapped parameters supplied explicitly by the caller. Periodic
  /// branch/turn counts cannot in general be inferred uniquely from endpoints.
  double start = 0;
  /// Directed end parameter on the selected source curve.
  double end = 0;
};
/// @brief Owned source FIN use and directed curve/surface geometry binding.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BoundFinGeometry {
  /// Unchanged source fin use in its original loop direction.
  FinUse use;
  /// Source FACE identity.
  NodeId face;
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Referenced curve identity.
  NodeId curve;
  /// Directed start parameter on the selected source curve.
  double start = 0;
  /// Directed end parameter on the selected source curve.
  double end = 0;
  /// Owned solver settings for all subsequent evaluations of this binding.
  std::optional<IntersectionEvaluationOptions> intersection = std::nullopt;
};
/// @brief One bound LOOP or an isolated vertex mapped to surface parameters.
///
/// @see @ref xt_loop "XT 4.3.7: Loop (p. 28)", @ref xt_isolated "XT 5.3.8.1:
/// Isolated loops (p. 96)"
struct FaceTrimLoop {
  /// Source LOOP identity.
  NodeId loop;
  /// Bound source fin uses in loop order.
  std::vector<BoundFinGeometry> fins;
  /// Optional isolated vertex represented by its recovered surface parameters.
  std::optional<SurfaceParameters> isolated_vertex;
};
/// @brief Exact source-coordinate FACE boundary with bound loop geometry.
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_attachment "XT 5.3.4.1:
/// Attaching geometry to topology (p. 92)"
struct FaceTrimming {
  /// Source FACE identity.
  NodeId face;
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Unmodified optional FACE sense byte; absence does not imply a default.
  std::optional<uint8_t> face_sense;
  /// Bound face loops in source order, without outer/hole classification.
  std::vector<FaceTrimLoop> loops;
};
/// @brief Directed fin curve sample and corresponding supporting-surface UV.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_spcurve
/// "XT 5.2.1.8: SP_CURVE (p. 51)"
struct FinGeometryEvaluation {
  /// Source-coordinate sample with derivative relative to the directed fin
  /// fraction.
  CurveEvaluation curve;
  /// Supporting-surface natural parameters and measured membership residual.
  SurfaceParameters uv;
};
/// Builds a source-order exact boundary representation, without tessellation.
/// Requires exactly one directed interval per non-isolated fin. Endpoints are
/// checked against vertex points; endpoints and midpoint against the surface.
/// Consecutive fin endpoints and the last/first pair must also join in 3D
/// within tolerance, including vertex-free single-fin loops. UV seam
/// coordinates need not coincide. These finite checks do not certify interior
/// membership, loop simplicity, hole orientation or global solid validity.
/// Optional intersection settings enable the existing bounded local solver,
/// including through trims. Settings are validated even for empty boundaries
/// and copied into each fin; omission preserves unsupported intersections.
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_halfedge "XT 4.3.8:
/// Halfedge (p. 29)", @ref xt_attachment "XT 5.3.4.1: Attaching geometry to
/// topology (p. 92)", @ref xt_trim "XT 5.2.1.6: Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param face Stream-local FACE identity to query.
/// @param intervals Explicit directed intervals; required entity coverage is
/// described above.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @param intersection Optional explicit solver settings, copied into
/// successful bindings.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
/// Resolves lines, accurate source trim intervals and accurate circle/ellipse
/// edges. NURBS/SP curves use the complete natural spline domain when its
/// endpoint correspondence is unambiguous. No partial-domain inversion or
/// periodic wrapping is inferred. Edge trims retain transmitted parameters,
/// ordered by fin sense, even for closed/vertex-free edges. Stored trim points
/// must match curve samples. With solver settings, direct intersections use
/// stored chart matches or bounded chord-plane parameter recovery, verified by
/// solved 3D curve membership. A shared H limit and complete chart returning
/// once to that help seam establish a closed accurate intersection's period;
/// known curve/fin senses determine direction. A single endpoint vertex must
/// coincide with that seam. For a partial edge ending at the proven help seam,
/// traversal sense selects the first or last parameter occurrence. Interior
/// duplicate chart matches and unproven periods remain ambiguous. Other
/// closed/missing endpoints and unrecoverable parameters require explicit
/// intervals.
/// Accurate edge orientation must agree with curve sense. Fin trims require
/// unambiguous vertex correspondence. Conics select one directed arc, or a full
/// turn for closed/vertex-free edges. Distinct conic vertices separated by no
/// more than the caller tolerance are rejected before periodic angle wrapping;
/// rounding cannot turn an unresolved short edge into a nearly full circle.
/// Coincident distinct conic vertices, fin
/// conics, ambiguous fin trims and partial or ambiguous spline domains require
/// the explicit overload.
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_halfedge "XT 4.3.8:
/// Halfedge (p. 29)", @ref xt_attachment "XT 5.3.4.1: Attaching geometry to
/// topology (p. 92)", @ref xt_trim "XT 5.2.1.6: Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param face Stream-local FACE identity to query.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @param intersection Optional explicit solver settings, copied into
/// successful bindings.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
/// Resolves all unspecified fins automatically. Overrides use the same directed
/// interval and membership checks as the complete explicit overload. Duplicate,
/// foreign or isolated-loop fin IDs are invalid options. Empty overrides retain
/// automatic behavior; unsupported unspecified branches remain errors.
///
/// @see @ref xt_face "XT 4.3.6: Face (p. 28)", @ref xt_halfedge "XT 4.3.8:
/// Halfedge (p. 29)", @ref xt_trim "XT 5.2.1.6: Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param face Stream-local FACE identity to query.
/// @param overrides Partial caller-selected directed intervals; unspecified
/// entities use inference.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @param intersection Optional explicit solver settings, copied into
/// successful bindings.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimmingWithOverrides(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
/// Fraction is in [0,1]. Tangent is dP/d(fraction), including directed
/// reversal. Uses transmitted SP-curve UVs on the same surface; otherwise maps
/// the 3D curve point to the face through bounded surface inversion. Uses the
/// binding's owned intersection settings when supplied. No periodic UV
/// continuation or inside/outside classification.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_spcurve
/// "XT 5.2.1.8: SP_CURVE (p. 51)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param fin Previously bound source fin with its owned interval and solver
/// settings.
/// @param fraction Directed local interval fraction in [0,1].
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FinGeometryEvaluation> EvaluateBoundFin(
    const Model& model, const BoundFinGeometry& fin, double fraction,
    double tolerance);
/// @brief Library path step owning the original bound source fin geometry.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)"
struct BoundBoundaryPathEdge {
  /// Directed canonical path step with source fin identity.
  BoundaryPathEdge use;
  /// Owned, unchanged source binding. Interval direction is reversed during
  /// evaluation when the path and source fin senses differ.
  BoundFinGeometry source;
};
/// @brief Library ordered boundary path with owned exact curve bindings.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)"
struct BoundBoundaryPath {
  /// Library cycle/chain classification; branched paths cannot be bound.
  BoundaryGroupKind kind = BoundaryGroupKind::kCycle;
  /// Owned canonical steps in directed path order.
  std::vector<BoundBoundaryPathEdge> edges;
};
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BoundaryPathBindingOptions {
  /// Upper bound on distinct face boundaries.
  uint64_t max_faces = 10000;
  /// Upper bound on loop boundaries.
  uint64_t max_loops = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on boundary paths.
  uint64_t max_paths = 10000;
  /// Upper bound on edge entries or path steps for this query.
  uint64_t max_edges = 100000;
};
/// Connects previously analyzed unbranched groups to previously bound faces.
/// Indexes source fins once. Each path must cover its listed edges exactly once
/// and match source fin IDs, edge direction and vertices. Repeated source fins
/// across different paths are allowed (for different shell/region occurrences).
/// Checks endpoints/midpoints on source surfaces and consecutive 3D endpoints,
/// including cycle closure, with the caller's finite nonnegative tolerance.
/// Positive budgets bound the aggregate face/loop/fin scan and total
/// paths/steps. Branched groups return unsupported; no partial result on any
/// failure. Does not revalidate body topology or the original face bindings,
/// select new curve branches, infer face/hole orientation or certify
/// whole-curve validity.
///
/// @see @ref xt_shell "XT 4.3.5: Shell (p. 28)", @ref xt_halfedge "XT 4.3.8:
/// Halfedge (p. 29)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param groups Previously analyzed boundary groups; branched groups are
/// unsupported.
/// @param faces Previously bound source face geometries used to resolve path
/// fins.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<std::vector<BoundBoundaryPath>> BindBoundaryPathsGeometry(
    const Model& model, std::span<const ShellBoundaryGroup> groups,
    std::span<const FaceTrimming> faces, double tolerance,
    const BoundaryPathBindingOptions& options = {});
/// Fraction in [0,1]; curve tangent is dP/d(path fraction). Retains the source
/// surface mapping and owned solver settings, including through reversed trims.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param edge Previously bound path edge with its unchanged owned source fin.
/// @param fraction Directed local interval fraction in [0,1].
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<FinGeometryEvaluation> EvaluateBoundBoundaryPathEdge(
    const Model& model, const BoundBoundaryPathEdge& edge, double fraction,
    double tolerance);
/// @brief Library choice of one-sided edge sample at an interior integer path
/// join.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)"
enum class BoundaryPathJoinSide {
  /// Previous edge at fraction one.
  kIncoming,
  /// Next edge at fraction zero.
  kOutgoing,
};
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BoundaryPathEvaluationOptions {
  /// Upper bound on edge entries or path steps for this query.
  uint64_t max_edges = 100000;
  /// Which one-sided edge sample to select at an interior integer path join.
  BoundaryPathJoinSide join_side = BoundaryPathJoinSide::kOutgoing;
};
/// @brief Library path sample retaining the selected edge and source face
/// context.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)"
struct BoundaryPathEvaluation {
  /// Zero-based index of the selected edge in BoundBoundaryPath::edges.
  std::size_t edge_index = 0;
  /// Selected edge's directed local fraction in [0,1].
  double edge_fraction = 0;
  /// Selected canonical path step with source fin identity.
  BoundaryPathEdge use;
  /// Source FACE identity.
  NodeId face;
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Referenced curve identity.
  NodeId curve;
  /// Selected fin sample with derivative relative to the whole path parameter.
  FinGeometryEvaluation geometry;
};
/// Evaluates a previously bound path with one parameter unit per edge. Domain
/// is [0, edge count]. At interior integer joins, selects the incoming end or
/// outgoing start explicitly; endpoints always use the first/last edge without
/// cycle wrapping. Tangent is dP/d(path parameter), retaining the selected
/// edge's surface mapping and owned solver profile. No tangent/UV blending.
/// Performs constant-work edge selection under a positive size budget. Does
/// not revalidate the complete path or certify smoothness/arc-length spacing.
///
/// @see @ref xt_halfedge "XT 4.3.8: Halfedge (p. 29)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param path Previously bound unbranched path; parameter domain is [0, edge
/// count].
/// @param parameter Finite piecewise path parameter in [0, edge count]; each
/// edge spans one unit.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BoundaryPathEvaluation> EvaluateBoundBoundaryPath(
    const Model& model, const BoundBoundaryPath& path, double parameter,
    double tolerance, const BoundaryPathEvaluationOptions& options = {});
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BodyFaceBindingOptions {
  /// Options for complete selected-body topology validation.
  BodyValidationOptions topology;
  /// Per-face traversal budgets during geometry binding.
  FaceTraversalOptions face;
  /// Owned optional settings for supported bounded intersection evaluation.
  std::optional<IntersectionEvaluationOptions> intersection;
  /// Upper bound on distinct face boundaries.
  uint64_t max_faces = 10000;
  /// Aggregate bound across unique faces, including isolated vertex loops.
  uint64_t max_fins = 100000;
};
/// @brief Validated body topology and one binding per distinct face.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_face "XT 4.3.6: Face (p.
/// 28)", @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
/// 92)"
struct BodyFaceBinding {
  /// Owned complete selected-body validated topology.
  ValidatedBodyTopology topology;
  /// Exactly one binding per distinct face, sorted by face ID. Source uses
  /// (front/back sides, regions and shells) remain in topology.boundary.
  std::vector<FaceTrimming> faces;
};
/// Validates the selected body's complete recorded topology, then binds every
/// distinct face's loop geometry. Atomic result: any failure returns no partial
/// body. Explicit mode requires one interval per non-isolated fin across all
/// selected faces; duplicate, missing or foreign fin IDs are invalid options.
/// Automatic mode inherits BindFaceTrimming's supported interval inference.
/// Face/solver options and positive aggregate limits are validated up front.
/// Fin endpoint pairs are compared in natural edge direction against the first
/// bound fin of each shared edge, within the same tolerance. Vertex-free rings
/// are excluded because their parameter seams need not coincide. This adds at
/// most two curve samples per fin of shared edges, bounded by max_fins; it does
/// not compare interior points or all pairs of a nonmanifold radial ring.
/// Wire edges/acorns are retained as topology, without binding their geometry.
/// Does not certify manifoldness, volume, hole classification or agreement at
/// every curve parameter. Errors preserve their code and source offset.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_face "XT 4.3.6: Face (p.
/// 28)", @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
/// 92)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param intervals Explicit directed intervals; required entity coverage is
/// described above.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyFaceBinding> BindBodyFaces(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const BodyFaceBindingOptions& options = {});
/// @brief Automatically binds distinct faces using the same topology and
/// membership checks.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_face "XT 4.3.6: Face (p.
/// 28)", @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
/// 92)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyFaceBinding> BindBodyFaces(
    const Model& model, NodeId body, double tolerance,
    const BodyFaceBindingOptions& options = {});
/// Body-wide partial intervals; unspecified fins use automatic inference.
/// Keeps full topology validation, aggregate limits and atomic error behavior.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_face "XT 4.3.6: Face (p.
/// 28)", @ref xt_trim "XT 5.2.1.6: Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param overrides Partial caller-selected directed intervals; unspecified
/// entities use inference.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyFaceBinding> BindBodyFacesWithOverrides(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const BodyFaceBindingOptions& options = {});
/// @brief Caller-selected directed curve interval for one shell wireframe edge.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)"
struct WireParameterInterval {
  /// Source EDGE identity.
  NodeId edge;
  /// Parameters run in the natural edge direction, with explicit branch/turns.
  double start = 0;
  /// Directed end parameter on the selected source curve.
  double end = 0;
};
/// @brief Owned wireframe edge use and directed curve interval.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_attachment "XT 5.3.4.1:
/// Attaching geometry to topology (p. 92)"
struct BoundWireGeometry {
  /// Unchanged source wireframe edge use in natural edge direction.
  WireEdgeUse use;
  /// Directed start parameter on the selected source curve.
  double start = 0;
  /// Directed end parameter on the selected source curve.
  double end = 0;
  /// Owned optional settings for supported bounded intersection evaluation.
  std::optional<IntersectionEvaluationOptions> intersection = std::nullopt;
};
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BodyWireBindingOptions {
  /// Options for complete selected-body topology validation.
  BodyValidationOptions topology;
  /// Owned optional settings for supported bounded intersection evaluation.
  std::optional<IntersectionEvaluationOptions> intersection;
  /// Upper bound on edge entries or path steps for this query.
  uint64_t max_edges = 100000;
};
/// @brief Validated body topology and one binding per distinct shell wireframe
/// edge.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_shell "XT 4.3.5: Shell
/// (p. 28)", @ref xt_edge "XT 4.3.9: Edge (p. 29)"
struct BodyWireBinding {
  /// Owned complete selected-body validated topology.
  ValidatedBodyTopology topology;
  /// One binding per distinct shell wire edge, sorted by edge ID.
  std::vector<BoundWireGeometry> wires;
};
/// Validates complete body topology and binds shell wire edges atomically.
/// Requires exactly one finite nonzero directed interval per selected wire
/// edge. Duplicate, missing and foreign edge IDs are invalid options. Curve
/// sense must be known and agree with edge direction; source trims define that
/// sense through their ordered parameters. Endpoints must match vertex points
/// within the explicit nonnegative tolerance. Vertex-free rings must close
/// within it. Endpoints and midpoint must evaluate, including their oriented
/// tangents. These finite checks do not certify whole-curve validity or unique
/// branches. Faces/acorns are retained as topology, without binding their
/// geometry. Solver settings are validated even for empty wire sets and owned
/// by bindings.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_shell "XT 4.3.5: Shell
/// (p. 28)", @ref xt_edge "XT 4.3.9: Edge (p. 29)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param intervals Explicit directed intervals; required entity coverage is
/// described above.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyWireBinding> BindBodyWires(
    const Model& model, NodeId body,
    std::span<const WireParameterInterval> intervals, double tolerance,
    const BodyWireBindingOptions& options = {});
/// Automatic intervals share face-fin inference: lines from vertices, directed
/// circle/ellipse arcs (one turn for rings), transmitted trims, complete spline
/// domains with unambiguous endpoints, and intersections with explicit solver
/// settings. Partial/ambiguous domains and extra turns need the explicit API.
/// Keeps the same full topology, resource, direction and membership checks.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_shell "XT 4.3.5: Shell
/// (p. 28)", @ref xt_edge "XT 4.3.9: Edge (p. 29)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyWireBinding> BindBodyWires(
    const Model& model, NodeId body, double tolerance,
    const BodyWireBindingOptions& options = {});
/// Supplied intervals replace inference only for their selected wire edges.
/// Other wires are resolved automatically; empty overrides retain automatic
/// behavior. Duplicate, invalid, foreign and face-owned edge IDs are rejected.
/// Keeps complete topology validation, aggregate limits and atomic membership
/// checks. Overrides do not enable unsupported curve evaluators or bypass trim
/// domains, direction, vertex matching or ring closure checks.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param overrides Partial caller-selected directed intervals; unspecified
/// entities use inference.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyWireBinding> BindBodyWiresWithOverrides(
    const Model& model, NodeId body,
    std::span<const WireParameterInterval> overrides, double tolerance,
    const BodyWireBindingOptions& options = {});
/// Fraction is in [0,1]; tangent is dP/d(fraction), including interval
/// reversal. Uses the owned solver settings. Inherits curve domain/resource
/// checks; does not revalidate topology, endpoints or closure after binding.
///
/// @see @ref xt_edge "XT 4.3.9: Edge (p. 29)", @ref xt_senses "XT 5.2.8: Curve,
/// surface and lattice senses (p. 82)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param wire Previously bound wireframe edge with its owned interval and
/// solver settings.
/// @param fraction Directed local interval fraction in [0,1].
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<CurveEvaluation> EvaluateBoundWire(
    const Model& model, const BoundWireGeometry& wire, double fraction);
/// @brief Body vertex and its typed source POINT position.
///
/// @see @ref xt_vertex "XT 4.3.10: Vertex (p. 30)", @ref xt_point "XT 5.2.5:
/// Point (p. 77)"
struct BoundBodyVertex {
  /// Source VERTEX identity.
  NodeId vertex;
  /// Referenced POINT node identity.
  NodeId point;
  /// Position in source coordinates and source length units.
  Vector3 position;
};
/// @brief Library resource limits and settings for the associated bounded
/// query.
///
/// @see @ref xt_topology "XT 4.1: Topology (p. 25)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct BodyGeometryBindingOptions {
  /// Options for each of the two body topology validation passes.
  BodyValidationOptions topology;
  /// Per-face traversal budgets during geometry binding.
  FaceTraversalOptions face;
  /// Owned optional settings for supported bounded intersection evaluation.
  std::optional<IntersectionEvaluationOptions> intersection;
  /// Upper bound on distinct face boundaries.
  uint64_t max_faces = 10000;
  /// Upper bound on fin visits, including isolated-loop anchors.
  uint64_t max_fins = 100000;
  /// Upper bound on shell wireframe edge entries.
  uint64_t max_wire_edges = 100000;
  /// Upper bound on body vertex points.
  uint64_t max_vertices = 100000;
};
/// @brief Owned validated topology, distinct face/wire bindings and vertex
/// points.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_attachment "XT 5.3.4.1:
/// Attaching geometry to topology (p. 92)"
struct BodyGeometryBinding {
  /// Single owned topology shared by all returned geometry bindings.
  ValidatedBodyTopology topology;
  /// Sorted by face, wire edge and vertex ID respectively. Source hierarchy,
  /// face sides and incidence remain in the single owned topology object.
  std::vector<FaceTrimming> faces;
  /// Distinct bound shell wireframe edges sorted by edge ID.
  std::vector<BoundWireGeometry> wires;
  /// Typed vertex points sorted by vertex ID, including acorns.
  std::vector<BoundBodyVertex> vertices;
};
/// Atomically combines face binding, wire binding and every selected vertex's
/// typed point, including acorns. One tolerance and solver profile apply to
/// both curve paths. Options and positive limits are checked even for empty
/// bodies. Results own their data. Uses the existing bounded face/wire queries,
/// including two topology validation passes with the same per-pass limits.
/// Does not certify global geometric membership, manifoldness or volume.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_attachment "XT 5.3.4.1:
/// Attaching geometry to topology (p. 92)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyGeometryBinding> BindBodyGeometry(
    const Model& model, NodeId body, double tolerance,
    const BodyGeometryBindingOptions& options = {});
/// Caller-selected fin and wire intervals replace automatic inference only for
/// those entities. Keeps both queries' direction, domain and membership checks;
/// foreign/duplicate intervals are rejected. No partial result on any failure.
///
/// @see @ref xt_body "XT 4.3.3: Body (p. 27)", @ref xt_attachment "XT 5.3.4.1:
/// Attaching geometry to topology (p. 92)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param body Stream-local BODY identity to query.
/// @param fin_overrides Partial directed FIN intervals; unspecified fins use
/// inference.
/// @param wire_overrides Partial natural-direction wire EDGE intervals;
/// unspecified wires use inference.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<BodyGeometryBinding> BindBodyGeometryWithOverrides(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> fin_overrides,
    std::span<const WireParameterInterval> wire_overrides, double tolerance,
    const BodyGeometryBindingOptions& options = {});
}  // namespace openparasolid
#endif  // OPENPARASOLID_BREP_H_
