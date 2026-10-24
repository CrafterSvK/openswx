// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
#define OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
#include <vector>

#include "openparasolid/brep.h"
namespace openswstep::internal {
// Writer-local circle and angular intervals. The source Model and chart units
// are unchanged; the EDGE identity is used only as a writer-local curve key.
struct IntersectionCircleRecovery {
  openparasolid::NodeId edge;
  openparasolid::CircleGeometry circle;
  std::vector<openparasolid::FinParameterInterval> uses;
};
// Only a complete shared-help chart on a certified sphere/plane pair is
// recognized. Its analytic support determines the circle; no points are fitted.
[[nodiscard]] openparasolid::Result<std::vector<IntersectionCircleRecovery>>
RecoverIntersectionCircles(const openparasolid::Model& model,
                           const openparasolid::BodyGeometryBinding& body,
                           double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
