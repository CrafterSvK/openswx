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
struct PerforatedPolygon {
  std::size_t outer_loop;
  long double area;
};
// Adds full circular holes to the polygon subset above. Circle centers must
// lie in the material, with entire circles separated from every polygon edge
// and other circle by more than two source tolerances. Each edge-distance,
// containment-edge and circle-pair check consumes the shared budget. Returns
// the outer index in the polygon span and the exact net planar area.
[[nodiscard]] openparasolid::Result<PerforatedPolygon>
ValidatePolygonCircularHoles(
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    std::span<const ConicBoundary> circles,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_
