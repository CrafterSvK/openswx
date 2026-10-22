// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_CURVED_HOLE_H_
#define OPENSWSTEP_SRC_CURVED_HOLE_H_

#include "convex_boundary.h"
#include "planar_boundary.h"

namespace openswstep::internal {
// Convex multi-edge LINE/CIRCLE/ELLIPSE holes inside a polygonal or complete
// circular outer bound, optionally alongside polygonal/full-circle holes.
// Directed arc extrema produce inflated enclosing rectangles used only for
// validation. Original loops and analytic hole areas determine the exported
// B-rep. Returns unsupported when the envelopes cannot certify disjoint
// containment; valid close contours may therefore remain unsupported. One check
// per curved edge and corner, plus two passes of the existing mixed-boundary
// checks, share the caller's budget. The returned outer index addresses the
// original polygon or circle span, according to circular_outer.
[[nodiscard]] openparasolid::Result<PlanarRegion> ValidateConvexCurvedHoles(
    std::span<const std::vector<PlanarCurve>> curved_holes,
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    std::span<const ConicBoundary> circles,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CURVED_HOLE_H_
