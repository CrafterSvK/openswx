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
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const FaceTraversalOptions& options = {});
// Resolves intervals automatically for lines and trimmed curves whose endpoint
// correspondence is unambiguous within tolerance. Periodic/untrimmed NURBS
// branches and ambiguous closed endpoints require the explicit overload.
[[nodiscard]] Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face, double tolerance,
    const FaceTraversalOptions& options = {});
// Fraction is in [0,1]. Tangent is dP/d(fraction), including directed reversal.
// Uses transmitted SP-curve UVs when available; otherwise bounded surface
// inversion. No periodic UV continuation or inside/outside classification.
[[nodiscard]] Result<FinGeometryEvaluation> EvaluateBoundFin(
    const Model& model, const BoundFinGeometry& fin, double fraction,
    double tolerance);
}  // namespace openparasolid
#endif  // OPENPARASOLID_BREP_H_
