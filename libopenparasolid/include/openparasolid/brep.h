// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_BREP_H_
#define OPENPARASOLID_BREP_H_
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "openparasolid/evaluate.h"
#include "openparasolid/topology.h"
namespace openparasolid {
struct BodyValidationOptions {
  CompoundBodyTraversalOptions traversal;
  uint64_t max_model_nodes = 300000;
  uint64_t max_model_references = 5000000;
  uint64_t max_radial_fins = 100000;
  uint64_t max_vertex_fins = 100000;
};
struct ValidatedBodyTopology {
  BodyBoundary boundary;
  std::vector<EdgeBoundary> edges;
  std::vector<VertexIncidence> vertices;
};
// Checks the complete selected body's recorded incidence against a bounded
// scan of the model: owner lists, both face sides, loops, radial fins and
// vertex chains. Compound child groups are checked when present. Results own
// their data. No geometry, manifoldness or positive-volume certificate.
[[nodiscard]] Result<ValidatedBodyTopology> ValidateBodyTopology(
    const Model& model, NodeId body, const BodyValidationOptions& options = {});
struct FinParameterInterval {
  NodeId fin;
  // Directed, unwrapped parameters supplied explicitly by the caller. Periodic
  // branch/turn counts cannot in general be inferred uniquely from endpoints.
  double start = 0;
  double end = 0;
};
struct BoundFinGeometry {
  FinUse use;
  NodeId face;
  NodeId surface;
  NodeId curve;
  double start = 0;
  double end = 0;
  // Owned solver settings for all subsequent evaluations of this binding.
  std::optional<IntersectionEvaluationOptions> intersection = std::nullopt;
};
struct FaceTrimLoop {
  NodeId loop;
  std::vector<BoundFinGeometry> fins;
  std::optional<SurfaceParameters> isolated_vertex;
};
struct FaceTrimming {
  NodeId face;
  NodeId surface;
  std::optional<uint8_t> face_sense;
  std::vector<FaceTrimLoop> loops;
};
struct FinGeometryEvaluation {
  CurveEvaluation curve;
  SurfaceParameters uv;
};
// Builds a source-order exact boundary representation, without tessellation.
// Requires exactly one directed interval per non-isolated fin. Endpoints are
// checked against vertex points; endpoints and midpoint against the surface.
// These finite checks are not a proof of membership for every parameter.
// Optional intersection settings enable the existing bounded local solver,
// including through trims. Settings are validated even for empty boundaries
// and copied into each fin; omission preserves unsupported intersections.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
// Resolves lines, accurate source trim intervals and accurate circle/ellipse
// edges. NURBS/SP curves use the complete natural spline domain when its
// endpoint correspondence is unambiguous. No partial-domain inversion or
// periodic wrapping is inferred. Edge trims retain transmitted parameters,
// ordered by fin sense, even for closed/vertex-free edges. Stored trim points
// must match curve samples. With solver settings, direct intersections use
// stored chart matches or bounded chord-plane parameter recovery, verified by
// solved 3D curve membership. Closed/missing endpoints and ambiguous or
// unrecoverable parameters require explicit intervals.
// Accurate edge orientation must agree with curve sense. Fin trims require
// unambiguous vertex correspondence. Conics select one directed arc, or a full
// turn for closed/vertex-free edges. Coincident distinct conic vertices, fin
// conics, ambiguous fin trims and partial or ambiguous spline domains require
// the explicit overload.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
// Resolves all unspecified fins automatically. Overrides use the same directed
// interval and membership checks as the complete explicit overload. Duplicate,
// foreign or isolated-loop fin IDs are invalid options. Empty overrides retain
// automatic behavior; unsupported unspecified branches remain errors.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimmingWithOverrides(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const FaceTraversalOptions& options = {},
    const std::optional<IntersectionEvaluationOptions>& intersection =
        std::nullopt);
// Fraction is in [0,1]. Tangent is dP/d(fraction), including directed reversal.
// Uses transmitted SP-curve UVs on the same surface; otherwise maps the 3D
// curve point to the face through bounded surface inversion. Uses the binding's
// owned intersection settings when supplied. No periodic UV continuation or
// inside/outside classification.
[[nodiscard]] Result<FinGeometryEvaluation> EvaluateBoundFin(
    const Model& model, const BoundFinGeometry& fin, double fraction,
    double tolerance);
struct BodyFaceBindingOptions {
  BodyValidationOptions topology;
  FaceTraversalOptions face;
  std::optional<IntersectionEvaluationOptions> intersection;
  uint64_t max_faces = 10000;
  // Aggregate bound across unique faces, including isolated vertex loops.
  uint64_t max_fins = 100000;
};
struct BodyFaceBinding {
  ValidatedBodyTopology topology;
  // Exactly one binding per distinct face, sorted by face ID. Source uses
  // (front/back sides, regions and shells) remain in topology.boundary.
  std::vector<FaceTrimming> faces;
};
// Validates the selected body's complete recorded topology, then binds every
// distinct face's loop geometry. Atomic result: any failure returns no partial
// body. Explicit mode requires one interval per non-isolated fin across all
// selected faces; duplicate, missing or foreign fin IDs are invalid options.
// Automatic mode inherits BindFaceTrimming's supported interval inference.
// Face/solver options and positive aggregate limits are validated up front.
// Wire edges/acorns are retained as topology, without binding their geometry.
// Does not certify manifoldness, volume, hole classification or agreement at
// every curve parameter. Errors preserve their code and source offset.
[[nodiscard]] Result<BodyFaceBinding> BindBodyFaces(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const BodyFaceBindingOptions& options = {});
[[nodiscard]] Result<BodyFaceBinding> BindBodyFaces(
    const Model& model, NodeId body, double tolerance,
    const BodyFaceBindingOptions& options = {});
// Body-wide partial intervals; unspecified fins use automatic inference.
// Keeps full topology validation, aggregate limits and atomic error behavior.
[[nodiscard]] Result<BodyFaceBinding> BindBodyFacesWithOverrides(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const BodyFaceBindingOptions& options = {});
}  // namespace openparasolid
#endif  // OPENPARASOLID_BREP_H_
