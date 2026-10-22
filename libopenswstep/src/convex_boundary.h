// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CONVEX_BOUNDARY_H_
#define OPENSWSTEP_SRC_CONVEX_BOUNDARY_H_

#include <cstdint>
#include <span>

#include "conic_boundary.h"
#include "openparasolid/model.h"

namespace openswstep::internal {
// Borrowed analytic basis and directed natural parameters for one source fin.
struct PlanarCurve {
  const openparasolid::GeometryPayload* geometry;
  double start;
  double end;
};
struct ConvexBoundary {
  long double area;
  openparasolid::Vector3 reference_point;
};
// Checks one closed convex LINE/CIRCLE/ELLIPSE loop, with topology and endpoint
// closure already established by body binding. Nonnegative directed curvature
// and corner turns must total one revolution. Each corner consumes one shared
// comparison check. No polygonal approximation is constructed.
[[nodiscard]] openparasolid::Result<ConvexBoundary> ValidateConvexPlanarLoop(
    std::span<const PlanarCurve> curves,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Validates a convex LINE/CIRCLE outer contour and complete circular holes.
// Every disk must lie strictly inside every supporting tangent half-plane;
// circle-arc extrema are analytic, including parameter seams and major arcs.
// Each hole/outer-edge and hole/hole pair consumes one shared check, in
// addition to the outer contour's corner checks. Returns the net planar area.
[[nodiscard]] openparasolid::Result<ConvexBoundary> ValidateConvexCircularHoles(
    std::span<const PlanarCurve> curves, std::span<const ConicBoundary> holes,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CONVEX_BOUNDARY_H_
