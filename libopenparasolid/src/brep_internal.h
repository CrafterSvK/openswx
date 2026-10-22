// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_SRC_BREP_INTERNAL_H_
#define OPENPARASOLID_SRC_BREP_INTERNAL_H_

#include "openparasolid/brep.h"

namespace openparasolid::internal {
// Shared interval inference for real fins and validated primary wire anchors.
// Does not replace subsequent curve/vertex/surface membership checks.
[[nodiscard]] Result<FinParameterInterval> InferFinInterval(
    const Model& model, const FinUse& use, double tolerance,
    const std::optional<IntersectionEvaluationOptions>& intersection);
}  // namespace openparasolid::internal
#endif  // OPENPARASOLID_SRC_BREP_INTERNAL_H_
