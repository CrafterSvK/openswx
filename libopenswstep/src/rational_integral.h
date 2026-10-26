// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_RATIONAL_INTEGRAL_H_
#define OPENSWSTEP_SRC_RATIONAL_INTEGRAL_H_
#include <span>

#include "openparasolid/model.h"
namespace openswstep::internal {
struct BoundedIntegral {
  long double value = 0;
  long double error = 0;
};
// Bounds a scalar Bernstein ordinate; positive weighted ordinates define
// the certified height of a rational graph strip with the same knot basis.
struct ScalarBound {
  long double lower, upper;
};
// Encloses integral H*dot(cross(C-origin,C'),normal) on one clamped rational
// quadratic/cubic span, where H has these Cartesian height ordinates and the
// curve's unchanged positive weights. Products have denominator W^3. Six
// bounded subdivision levels verify the integral without fitting geometry.
[[nodiscard]] openparasolid::Result<BoundedIntegral>
RationalWeightedCurveIntegral(const openparasolid::NurbsCurveGeometry& curve,
                              std::span<const ScalarBound> height_controls,
                              openparasolid::Vector3 origin,
                              openparasolid::Vector3 normal,
                              uint64_t& remaining_checks);
// Encloses integral dot(cross(C-origin,C'),normal) on the entire selected
// clamped rational quadratic/cubic curve. Outward-rounded Bernstein quotient
// bounds are refined a fixed six levels; no geometry or weights are fitted.
// Normal component uncertainty can account for constructing a plane frame.
// Every coefficient product and subdivision consumes the shared work budget.
[[nodiscard]] openparasolid::Result<BoundedIntegral> RationalCurveIntegral(
    const openparasolid::NurbsCurveGeometry& curve,
    openparasolid::Vector3 origin, openparasolid::Vector3 normal,
    uint64_t& remaining_checks, long double normal_error = 0);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_RATIONAL_INTEGRAL_H_
