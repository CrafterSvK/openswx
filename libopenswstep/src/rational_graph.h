// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_RATIONAL_GRAPH_H_
#define OPENSWSTEP_SRC_RATIONAL_GRAPH_H_
#include <span>

#include "sweep_bounds.h"
namespace openswstep::internal {
// Certifies the whole rational quadratic/cubic graph's strictly signed chord
// projection and returns a bound on speed / projected speed. Homogeneous
// controls are outward enclosures of the original plane-derived graph;
// normalization, translation and subdivision are proof-only operations.
[[nodiscard]] openparasolid::Result<long double> RationalGraphChordCondition(
    std::span<const sweep_bounds::Vector> homogeneous_controls,
    std::span<const double> weights, openparasolid::Vector3 origin,
    sweep_bounds::Vector chord, int direction, uint64_t& remaining_checks);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_RATIONAL_GRAPH_H_
