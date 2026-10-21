// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
#define OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_

#include <array>
#include <optional>

#include "openparasolid/evaluate.h"

namespace openparasolid::internal {
// Bounds are u-low, u-high, v-low, v-high; validates before extracting knots.
[[nodiscard]] Result<std::array<double, 4>> NurbsSurfaceDomain(
    const NurbsSurfaceGeometry& surface);
[[nodiscard]] Result<SurfaceParameters> InvertNurbsSurface(
    const NurbsSurfaceGeometry& surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed);
[[nodiscard]] Result<Vector3> UnitCrossProduct(Vector3 first, Vector3 second,
                                              bool reverse);
[[nodiscard]] Result<Vector3> UnitSurfaceNormal(
    const SurfaceEvaluation& sample, bool reverse);
// The public dispatchers validate that all parameters are finite.
[[nodiscard]] Result<CurveEvaluation> EvaluateNurbsCurve(
    const NurbsCurveGeometry& curve, double parameter);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateNurbsSurface(
    const NurbsSurfaceGeometry& surface, double u, double v);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSpunProfile(
    const SpunSurfaceGeometry& surface, const CurveEvaluation& profile,
    double v);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSweptSection(
    const SweptSurfaceGeometry& surface, const CurveEvaluation& section,
    double v);
}  // namespace openparasolid::internal

#endif  // OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
