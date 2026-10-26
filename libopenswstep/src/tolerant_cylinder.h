// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_TOLERANT_CYLINDER_H_
#define OPENSWSTEP_SRC_TOLERANT_CYLINDER_H_
#include "tolerant_circle.h"
namespace openswstep::internal {
// Existing-master, open tolerant arcs between CYLINDER and piecewise-Bezier
// rational B_SURF carriers. Affine source pcurves are retained. Certificates
// bound the complete pcurve traces and, for a plane intersection master, an
// isolated implicit graph on the original rational surface. No curve fitting.
[[nodiscard]] openparasolid::Result<CircularEdgeRecovery>
RecoverCylinderNurbsEdge(
    const openparasolid::Model& model,
    std::span<const openparasolid::BoundFinGeometry* const> fins,
    const std::map<uint32_t, openparasolid::Vector3>& vertices,
    double tolerance, bool use_source_tolerances, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_TOLERANT_CYLINDER_H_
