// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CONE_PATCH_H_
#define OPENSWSTEP_SRC_CONE_PATCH_H_

#include "convex_boundary.h"

namespace openswstep::internal {
struct ConePatch {
  long double height;
  long double span;
  long double lower_radius;
  long double upper_radius;
  openparasolid::Vector3 first_direction;
  openparasolid::Vector3 last_direction;
  openparasolid::Vector3 reference_point;
};
// Validates one alternating CIRCLE/LINE loop: two coaxial circular arcs at
// distinct positive-radius levels and two straight cone generators. Winding
// determines one positive azimuth interval shorter than a full turn, including
// major arcs. Binding has already established topology and endpoint closure.
// Five shared checks cover the four edges and cut-pair comparison.
[[nodiscard]] openparasolid::Result<ConePatch> ValidateConePatch(
    std::span<const PlanarCurve> curves,
    const openparasolid::ConeGeometry& cone, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Six times the exact outward divergence contribution, including transverse
// and axial origin terms for a partial azimuth interval.
[[nodiscard]] long double ConePatchVolume6(
    const openparasolid::ConeGeometry& cone, const ConePatch& patch,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CONE_PATCH_H_
