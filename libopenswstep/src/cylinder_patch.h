// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CYLINDER_PATCH_H_
#define OPENSWSTEP_SRC_CYLINDER_PATCH_H_

#include "convex_boundary.h"

namespace openswstep::internal {
struct CylinderPatch {
  long double height;
  long double span;
  openparasolid::Vector3 first_direction;
  openparasolid::Vector3 last_direction;
  openparasolid::Vector3 reference_point;
};
// Validates one alternating CIRCLE/LINE loop: two coaxial circular arcs at
// distinct levels and two straight axial generators. Winding determines one
// positive azimuth interval shorter than a full turn, including major arcs.
// Binding has already established topology and endpoint closure. Five shared
// checks cover the four edges and cut-pair comparison.
[[nodiscard]] openparasolid::Result<CylinderPatch> ValidateCylinderPatch(
    std::span<const PlanarCurve> curves,
    const openparasolid::CylinderGeometry& cylinder, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Six times the exact outward divergence contribution, including transverse
// translation terms that no longer cancel over a partial azimuth interval.
[[nodiscard]] long double CylinderPatchVolume6(
    const openparasolid::CylinderGeometry& cylinder, const CylinderPatch& patch,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CYLINDER_PATCH_H_
