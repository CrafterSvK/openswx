// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPHERE_BOUNDARY_H_
#define OPENSWSTEP_SRC_SPHERE_BOUNDARY_H_

#include "conic_boundary.h"

namespace openswstep::internal {
// Axial region limits relative to the sphere center, ordered along its axis.
// A cap includes one sphere pole as an interior point, without a source cut.
struct SphereBand {
  long double lower;
  long double upper;
};
// Validates a cap with one latitude circle or a band with two distinct,
// oppositely wound latitude circles. Winding selects the cap's side, including
// caps larger than a hemisphere. Pole cuts and partial azimuths are
// unsupported. The entire supporting circles must match the sphere profile
// within source tolerance. One shared budget check is consumed per cap or cut
// pair.
[[nodiscard]] openparasolid::Result<SphereBand> ValidateSphereLoops(
    std::span<const ConicBoundary> loops,
    const openparasolid::SphereGeometry& sphere, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Six times the outward divergence contribution relative to origin. The
// caller applies the effective face sense; transverse translations cancel.
[[nodiscard]] long double SphereBandVolume6(
    const openparasolid::SphereGeometry& sphere, SphereBand band,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPHERE_BOUNDARY_H_
