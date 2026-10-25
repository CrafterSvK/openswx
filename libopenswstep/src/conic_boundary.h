// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CONIC_BOUNDARY_H_
#define OPENSWSTEP_SRC_CONIC_BOUNDARY_H_

#include <cstddef>
#include <cstdint>
#include <span>

#include "openparasolid/model.h"

namespace openswstep::internal {
// A complete directed conic loop, already validated by body geometry binding.
struct ConicBoundary {
  openparasolid::Vector3 center;
  openparasolid::Vector3 normal;
  openparasolid::Vector3 x_axis;
  double major_radius;
  double minor_radius;
  double start;
  double end;
  bool circle;
};
// Validates a single conic disk, or circular outer/hole loops. Returns the
// outer loop index without changing source order. Every containment and hole
// pair comparison consumes one unit of the shared boundary-check budget.
[[nodiscard]] openparasolid::Result<std::size_t> ValidatePlanarConics(
    std::span<const ConicBoundary> loops,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Validates a complete cylindrical band bounded by two full circular loops.
// Both cuts must be coaxial, at distinct axial levels and oppositely wound.
// Returns the positive height and consumes one shared boundary comparison.
[[nodiscard]] openparasolid::Result<long double> ValidateCylinderLoops(
    std::span<const ConicBoundary> loops,
    const openparasolid::CylinderGeometry& cylinder, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Analytic data for a full conical band, ordered along the source axis.
struct CircularBand {
  long double height;
  openparasolid::Vector3 lower_center;
  long double lower_radius;
  long double upper_radius;
};
// Validates two coaxial circular cuts on the positive-radius cone branch.
// Apex loops and partial bands are unsupported. One shared check is consumed.
[[nodiscard]] openparasolid::Result<CircularBand> ValidateConeLoops(
    std::span<const ConicBoundary> loops,
    const openparasolid::ConeGeometry& cone, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Validates a complete cone cap: one full coaxial circle and an isolated
// vertex at the actual apex. Returns a band with one exactly zero radius.
// One shared check is consumed. The singular point needs no STEP edge loop.
[[nodiscard]] openparasolid::Result<CircularBand> ValidateConeApex(
    const ConicBoundary& circle, openparasolid::Vector3 apex,
    const openparasolid::ConeGeometry& cone, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CONIC_BOUNDARY_H_
