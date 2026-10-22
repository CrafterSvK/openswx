// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_
#define OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "openparasolid/model.h"

namespace openswstep::internal {
// Validates simple, disjoint polygon loops on a plane. Exactly one outer loop
// agrees with the face normal; all other loops must be oppositely wound,
// strictly contained, non-touching holes without nesting. Source order stays
// unchanged. Comparisons and containment edge tests consume the shared budget.
[[nodiscard]] openparasolid::Result<std::size_t> ValidatePlanarLoops(
    std::span<const std::vector<openparasolid::Vector3>> polygons,
    const openparasolid::PlaneGeometry& plane, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_PLANAR_BOUNDARY_H_
