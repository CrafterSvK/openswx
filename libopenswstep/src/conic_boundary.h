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
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_CONIC_BOUNDARY_H_
