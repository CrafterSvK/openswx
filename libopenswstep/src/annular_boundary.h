// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_ANNULAR_BOUNDARY_H_
#define OPENSWSTEP_SRC_ANNULAR_BOUNDARY_H_

#include "convex_boundary.h"

namespace openswstep::internal {
struct AnnularBoundary {
  long double area;
  openparasolid::Vector3 reference_point;
};
// Validates one planar ring-sector loop: two concentric circular arcs, with
// positive outer/opposite inner winding and equal spans shorter than one turn,
// joined by two radial LINE edges. Topology and endpoint closure are already
// established by binding. Five shared checks cover four edges and the arc pair.
// Area is integrated analytically over the original source curves.
[[nodiscard]] openparasolid::Result<AnnularBoundary> ValidateAnnularSector(
    std::span<const PlanarCurve> curves,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_ANNULAR_BOUNDARY_H_
