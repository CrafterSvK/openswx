// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CYLINDER_BAND_H_
#define OPENSWSTEP_SRC_CYLINDER_BAND_H_

#include <array>
#include <vector>

#include "convex_boundary.h"

namespace openswstep::internal {
struct CylinderBand {
  long double height_integral;
  std::array<long double, 3> radial_integral;
  openparasolid::Vector3 reference_point;
};
// Two closed profiles spanning one full azimuth each, bounded by CIRCLE/ELLIPSE
// arcs and isolated axial LINE generators. Whole supporting conics must project
// to the source cylinder circle. Conic azimuth is strictly monotone within each
// profile; winding is opposite and lower/upper axial ranges are disjoint.
// Binding already established source incidence and endpoint closure. One check
// per edge plus one cut-pair check shares the caller's comparison budget.
[[nodiscard]] openparasolid::Result<CylinderBand> ValidateCylinderBand(
    std::span<const std::vector<PlanarCurve>> loops,
    const openparasolid::CylinderGeometry& cylinder, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Exact outward divergence contribution multiplied by two, i.e. six times
// volume after all shell faces are added. Variable profile heights retain
// transverse-origin terms that cancel only for complete horizontal cuts.
[[nodiscard]] long double CylinderBandVolume6(
    const openparasolid::CylinderGeometry& cylinder, const CylinderBand& band,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CYLINDER_BAND_H_
