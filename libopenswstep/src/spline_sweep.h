// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPLINE_SWEEP_H_
#define OPENSWSTEP_SRC_SPLINE_SWEEP_H_

#include "convex_boundary.h"
#include "rational_integral.h"

namespace openswstep::internal {
struct SplineSweepPatch {
  openparasolid::NurbsCurveGeometry profile;
  openparasolid::Vector3 sweep;
  double lower = 0, upper = 0;
};
// Certifies one rectangular parameter patch with two exactly corresponding
// polynomial or corresponding rational profiles with identical weights and two straight generators.
// Control comparisons bound the entire supporting edges, not just a finite set
// of samples.
[[nodiscard]] openparasolid::Result<SplineSweepPatch> RecognizeSplineSweepPatch(
    const openparasolid::Model& model, openparasolid::NodeId surface,
    std::span<const PlanarCurve> curves, bool natural_sense, double tolerance,
    uint64_t& remaining_checks);
// Polynomial flux is analytic. Rational flux uses outward-rounded integral
// bounds; callers must retain its error when certifying signed volume.
// The optional error result is required for rational patches.
[[nodiscard]] openparasolid::Result<long double> SplineSweepVolume6(
    const SplineSweepPatch& patch, openparasolid::Vector3 origin,
    uint64_t& remaining_checks, long double* error = nullptr);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPLINE_SWEEP_H_
