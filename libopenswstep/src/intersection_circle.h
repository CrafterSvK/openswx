// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
#define OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
#include <vector>

#include "openparasolid/brep.h"
namespace openswstep::internal {
// Writer-local analytic/spline curve and natural intervals. The source Model
// and chart units are unchanged; the EDGE identity is used only as a
// writer-local curve key.
struct IntersectionCurveRecovery {
  openparasolid::NodeId edge;
  openparasolid::GeometryPayload geometry;
  std::vector<openparasolid::FinParameterInterval> uses;
};
// Recognizes complete sphere/plane rings and supported polynomial swept/plane
// intersections. Source supports determine the curve; no points are fitted.
[[nodiscard]] openparasolid::Result<std::vector<IntersectionCurveRecovery>>
RecoverIntersectionCurves(const openparasolid::Model& model,
                          const openparasolid::BodyGeometryBinding& body,
                          double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_INTERSECTION_CIRCLE_H_
