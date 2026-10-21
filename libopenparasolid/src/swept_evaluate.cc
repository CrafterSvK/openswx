// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <limits>

#include "evaluate_internal.h"

namespace openparasolid::internal {
Result<SurfaceEvaluation> EvaluateSweptSection(
    const SweptSurfaceGeometry& surface, const CurveEvaluation& section,
    double v) {
  constexpr double kDirectionTolerance = 1e-10;
  const auto& d = surface.sweep;
  if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z) ||
      std::abs(std::hypot(d.x, d.y, d.z) - 1) > kDirectionTolerance) {
    return Result<SurfaceEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Swept surface direction must be finite and unit length"});
  }
  const std::array<double, 3> p{section.position.x, section.position.y,
                                section.position.z};
  const std::array<double, 3> sweep{d.x, d.y, d.z};
  std::array<double, 3> position{};
  for (std::size_t i = 0; i < position.size(); ++i) {
    const long double value = std::fma(static_cast<long double>(v),
                                       static_cast<long double>(sweep[i]),
                                       static_cast<long double>(p[i]));
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<double>::max()) {
      return Result<SurfaceEvaluation>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Swept surface position is not representable"});
    }
    position[i] = static_cast<double>(value);
  }
  return Result<SurfaceEvaluation>::Ok(
      {{position[0], position[1], position[2]}, section.first_derivative, d});
}
}  // namespace openparasolid::internal
