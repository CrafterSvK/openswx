// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPLINE_SWEEP_H_
#define OPENSWSTEP_SRC_SPLINE_SWEEP_H_

#include "convex_boundary.h"

namespace openswstep::internal {
struct SplineSweepPatch {
  openparasolid::NurbsCurveGeometry profile;
  openparasolid::Vector3 sweep;
  double lower = 0, upper = 0;
};
// Certifies one rectangular parameter patch with two exactly corresponding
// polynomial profiles and two straight generators. Control comparisons bound
// the entire supporting edges, not just a finite set of samples.
[[nodiscard]] openparasolid::Result<SplineSweepPatch> RecognizeSplineSweepPatch(
    const openparasolid::Model& model, openparasolid::NodeId surface,
    std::span<const PlanarCurve> curves, bool natural_sense, double tolerance,
    uint64_t& remaining_checks);
// Exact polynomial surface flux, multiplied by two for the signed volume6 sum.
[[nodiscard]] openparasolid::Result<long double> SplineSweepVolume6(
    const SplineSweepPatch& patch, openparasolid::Vector3 origin,
    uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPLINE_SWEEP_H_
