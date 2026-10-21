// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>

#include "evaluate_internal.h"

namespace openparasolid::internal {
namespace {
using Vector = std::array<long double, 3>;
bool Scale(Vector3 source, Vector& result) {
  if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
      !std::isfinite(source.z))
    return false;
  const double scale =
      std::max({std::abs(source.x), std::abs(source.y), std::abs(source.z)});
  if (scale == 0) return false;
  result = {static_cast<long double>(source.x) / scale,
            static_cast<long double>(source.y) / scale,
            static_cast<long double>(source.z) / scale};
  return true;
}
}  // namespace
Result<Vector3> UnitSurfaceNormal(const SurfaceEvaluation& sample,
                                  bool reverse) {
  Vector u{}, v{};
  if (!Scale(sample.u_derivative, u) || !Scale(sample.v_derivative, v)) {
    return Result<Vector3>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface normal requires finite nonzero partial derivatives"});
  }
  // Scale each partial independently before crossing, so extreme derivative
  // magnitudes do not overflow or underflow an otherwise regular normal.
  Vector cross{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
               u[0] * v[1] - u[1] * v[0]};
  const long double length = std::hypot(cross[0], cross[1], cross[2]);
  if (!std::isfinite(length) || length == 0) {
    return Result<Vector3>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface normal is singular or numerically unresolvable"});
  }
  const long double sign = reverse ? -1 : 1;
  for (auto& value : cross) value = sign * value / length;
  return Result<Vector3>::Ok({static_cast<double>(cross[0]),
                              static_cast<double>(cross[1]),
                              static_cast<double>(cross[2])});
}
}  // namespace openparasolid::internal
