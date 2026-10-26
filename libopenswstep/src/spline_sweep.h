// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPLINE_SWEEP_H_
#define OPENSWSTEP_SRC_SPLINE_SWEEP_H_

#include <vector>

#include "convex_boundary.h"
#include "rational_integral.h"

namespace openswstep::internal {
struct SplineSweepPatch {
  openparasolid::NurbsCurveGeometry profile;
  openparasolid::Vector3 sweep;
  double lower = 0, upper = 0;
  // Empty for rectangular patches. Otherwise encloses the Cartesian height
  // coefficient at each restricted profile pole; every lower bound is positive.
  using Height = ScalarBound;
  std::vector<Height> height_controls;
};
// Certifies a rectangle or a supported polynomial/rational graph strip with two
// corresponding profiles and two straight generators. The rectangular path
// compares translated profiles, including rational profiles of identical
// weights. The strip path proves whole-profile transverse agreement, positive
// height and oriented corner closure. All attempts share one work budget.
// Control comparisons bound whole supporting edges, rather than samples.
[[nodiscard]] openparasolid::Result<SplineSweepPatch> RecognizeSplineSweepPatch(
    const openparasolid::Model& model, openparasolid::NodeId surface,
    std::span<const PlanarCurve> curves, bool natural_sense, double tolerance,
    uint64_t& remaining_checks);
// Non-rectangular single-span strips use corresponding boundary profiles whose
// entire difference from the section is along the sweep. Positive height
// coefficients certify separation, with topology fixing the directed loop.
[[nodiscard]] openparasolid::Result<SplineSweepPatch> RecognizeSplineSweepStrip(
    const openparasolid::Model& model, openparasolid::NodeId surface,
    std::span<const PlanarCurve> curves, bool natural_sense, double tolerance,
    uint64_t& remaining_checks);
// Outward coefficient arithmetic encloses twice the exact
// surface flux (bounded quotient integration for rational strips); callers
// retain its uncertainty in the signed-volume check.
[[nodiscard]] openparasolid::Result<BoundedIntegral> SplineSweepStripVolume6(
    const SplineSweepPatch& patch, openparasolid::Vector3 origin,
    uint64_t& remaining_checks);
// Polynomial flux is analytic. Rational flux uses outward-rounded integral
// bounds; callers must retain its error when certifying signed volume.
// The optional error result is required for rational and variable-height
// patches.
[[nodiscard]] openparasolid::Result<long double> SplineSweepVolume6(
    const SplineSweepPatch& patch, openparasolid::Vector3 origin,
    uint64_t& remaining_checks, long double* error = nullptr);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPLINE_SWEEP_H_
