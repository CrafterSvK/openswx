// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_STRIP_PCURVE_H_
#define OPENSWSTEP_SRC_STRIP_PCURVE_H_
#include "spline_sweep.h"
namespace openswstep::internal {
// Constructs (u,v(u)) in homogeneous degree p+1: u*W increases the degree.
// The original 3D boundary and supporting rational surface remain unchanged.
// Outward bounds certify the emitted UV controls after lifting into 3D.
[[nodiscard]] openparasolid::Result<openparasolid::NurbsCurveGeometry>
BuildRationalStripPcurve(const SplineSweepPatch& patch,
                         const openparasolid::NurbsCurveGeometry& boundary,
                         double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_STRIP_PCURVE_H_
