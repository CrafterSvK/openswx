// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SWEPT_INTERSECTION_H_
#define OPENSWSTEP_SRC_SWEPT_INTERSECTION_H_
#include "intersection_circle.h"
namespace openswstep::internal {
[[nodiscard]] bool IsSweptPlaneIntersection(
    const openparasolid::Model& model,
    const openparasolid::GeometryObject& source);
// A transverse plane gives one polynomial graph along a certified open swept
// profile. Whole-pole interval bounds control the derived curve's roundoff;
// global chord-projection certificates isolate the recorded chart branch.
[[nodiscard]] openparasolid::Result<IntersectionCurveRecovery>
RecoverSweptPlaneIntersection(
    const openparasolid::Model& model,
    const openparasolid::GeometryObject& source,
    std::span<const openparasolid::BoundFinGeometry* const> uses,
    std::span<const openparasolid::BoundBodyVertex> vertices, double tolerance,
    uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SWEPT_INTERSECTION_H_
