// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SIMPLE_BOUNDARY_H_
#define OPENSWSTEP_SRC_SIMPLE_BOUNDARY_H_

#include "convex_boundary.h"
#include "planar_boundary.h"

namespace openswstep::internal {
// Certifies a possibly concave loop using exact Bezier control hulls, with
// bounded subdivision for validation only. Nonadjacent hulls must be separated
// by the caller's length tolerance. Adjacent hulls may meet only within the
// shared endpoint tolerance neighbourhood. Polynomial/conic areas are analytic;
// no validation chords replace the source STEP curves.
[[nodiscard]] openparasolid::Result<ConvexBoundary> ValidateSimplePlanarLoop(
    std::span<const PlanarCurve> curves,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Classifies one positively wound outer loop and negative holes. Cross-loop
// hull separation precedes chord winding tests for strict, unnested
// containment. The winding polygon closes certified sub-tolerance endpoint
// gaps, including the last/first join; each chord and connector consumes one
// shared budget check without changing source curves or analytic areas.
[[nodiscard]] openparasolid::Result<PlanarRegion> ValidateSimplePlanarRegion(
    std::span<const std::vector<PlanarCurve>> loops,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SIMPLE_BOUNDARY_H_
