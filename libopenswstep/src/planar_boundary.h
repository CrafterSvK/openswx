// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_
#define OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "conic_boundary.h"
#include "openparasolid/model.h"

namespace openswstep::internal {
// Validates simple, disjoint polygon loops on a plane. Exactly one outer loop
// agrees with the face normal; all other loops must be oppositely wound,
// strictly contained, non-touching holes without nesting. Source order stays
// unchanged. Comparisons and containment edge tests consume the shared budget.
[[nodiscard]] openparasolid::Result<std::size_t> ValidatePlanarLoops(
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Validates an all-hole polygon set: opposite winding, no self/inter-loop
// crossings or nesting, and no overlap/nesting with already validated circles.
// Containment in the curved outer contour is checked separately by the caller.
// Returns the positive sum of polygon hole areas; consumes shared segment,
// containment-edge and disk/segment checks. An empty set has area zero.
[[nodiscard]] openparasolid::Result<long double> ValidatePolygonHoleSet(
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    std::span<const ConicBoundary> circles,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
struct PlanarRegion {
  std::size_t outer_loop;
  long double area;
  bool circular_outer = false;
};
// Validates mixed polygon/full-circle loops with one polygonal or circular
// outer loop and disjoint polygonal/circular holes. Circle-related clearances
// exceed two source tolerances. Segment, containment-edge, disk-distance,
// vertex-in-circle and circle-pair checks share the comparison budget. The
// result's outer index addresses the circle span when circular_outer is true,
// otherwise polygons.
[[nodiscard]] openparasolid::Result<PlanarRegion> ValidateMixedPlanarLoops(
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    std::span<const ConicBoundary> circles,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_
