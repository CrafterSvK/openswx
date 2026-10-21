// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
#define OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_

#include "openparasolid/evaluate.h"

namespace openparasolid::internal {
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
