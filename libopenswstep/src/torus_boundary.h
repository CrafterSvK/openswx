// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_TORUS_BOUNDARY_H_
#define OPENSWSTEP_SRC_TORUS_BOUNDARY_H_

#include "conic_boundary.h"

namespace openswstep::internal {
// Directed positive meridian interval selected by the source boundary winding.
struct TorusBand {
  long double start;
  long double span;
};
// Validates a regular ring torus band with two coaxial full-circle cuts.
// Horn/apple/lemon tori, coincident cuts and partial azimuths are unsupported.
// The winding selects the meridian interval, never a shortest-arc assumption.
[[nodiscard]] openparasolid::Result<TorusBand> ValidateTorusLoops(
    std::span<const ConicBoundary> loops,
    const openparasolid::TorusGeometry& torus, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Returns six times the oriented divergence contribution relative to origin.
// The normal is the torus's natural outward normal; caller applies face sense.
[[nodiscard]] long double TorusBandVolume6(
    const openparasolid::TorusGeometry& torus, TorusBand band,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_TORUS_BOUNDARY_H_
