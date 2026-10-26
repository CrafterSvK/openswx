// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_TORUS_MERIDIAN_H_
#define OPENSWSTEP_SRC_TORUS_MERIDIAN_H_
#include "conic_boundary.h"

namespace openswstep::internal {
struct TorusMeridianBand {
  long double span;
  openparasolid::Vector3 axis;
  openparasolid::Vector3 first_direction;
  openparasolid::Vector3 last_direction;
};
// Certifies two full meridian circles bounding a regular ring-torus sector.
// Recorded winding selects its positive azimuth span, including major sectors.
// Whole-frame, circle-plane and profile bounds share the source tolerance.
// One shared comparison certifies the pair and its strict boundary separation.
[[nodiscard]] openparasolid::Result<TorusMeridianBand> ValidateTorusMeridians(
    std::span<const ConicBoundary> loops,
    const openparasolid::TorusGeometry& torus, bool natural_sense,
    double tolerance, uint64_t& remaining_checks);
// Exact outward surface flux multiplied by two, including transverse origin
// terms. No source seam, boundary edge or parameterization is synthesized.
[[nodiscard]] long double TorusMeridianVolume6(
    const openparasolid::TorusGeometry& torus, const TorusMeridianBand& band,
    openparasolid::Vector3 origin);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_TORUS_MERIDIAN_H_
