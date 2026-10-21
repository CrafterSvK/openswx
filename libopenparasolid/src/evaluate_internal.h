// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
#define OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_

#include "openparasolid/evaluate.h"

namespace openparasolid::internal {
// The public dispatcher validates that parameter is finite.
[[nodiscard]] Result<CurveEvaluation> EvaluateNurbsCurve(
    const NurbsCurveGeometry& curve, double parameter);
}  // namespace openparasolid::internal

#endif  // OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
