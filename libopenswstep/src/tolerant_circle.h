// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_TOLERANT_CIRCLE_H_
#define OPENSWSTEP_SRC_TOLERANT_CIRCLE_H_

#include <map>
#include <vector>

#include "openparasolid/brep.h"

namespace openswstep::internal {
struct CircularEdgeRecovery {
  openparasolid::NodeId edge;
  // Writer-local analytic curve key: the original EDGE ID cannot collide with
  // a source geometry ID. Never evaluate this key through the unchanged Model.
  openparasolid::NodeId curve;
  openparasolid::CircleGeometry circle;
  double tolerance;
  struct Use {
    openparasolid::NodeId fin;
    openparasolid::NodeId surface;
    double first, last;
    // Original planar spline or an affine reparameterization of the rotational
    // source pcurve into the canonical torus UV frame.
    openparasolid::NurbsCurveGeometry pcurve;
  };
  std::vector<Use> uses;
};
// Recovers only paired, vertex-bounded tolerant edges between a planar SP_CURVE
// and a constant-profile SP_CURVE on an unbounded circular SPUN_SURF. Retains
// both parameter curves. The rotational use supplies the exact 3D circle;
// bounded Bernstein checks certify the planar spline's whole-curve agreement,
// angular branch and monotonicity. No fitting or sampled equivalence test.
[[nodiscard]] openparasolid::Result<std::vector<CircularEdgeRecovery>>
RecoverTolerantCircles(const openparasolid::Model& model,
                       const openparasolid::BodyGeometryBinding& body,
                       double tolerance, bool use_source_tolerances,
                       uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_TOLERANT_CIRCLE_H_
