// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_NURBS_WIRE_H_
#define OPENSWSTEP_SRC_NURBS_WIRE_H_

#include "step_writer.h"

namespace openswstep::internal {
// A complete, clamped, open spline has unambiguous STEP edge limits when
// strict endpoint supporting planes exclude all interior control points.
// Positive weights then exclude either endpoint everywhere in the interior.
[[nodiscard]] Result<bool> ValidateNurbsWire(
    const openparasolid::NurbsCurveGeometry& curve, double first, double last,
    double tolerance, uint64_t& checks_remaining);
Id WriteNurbsCurve(StepWriter& writer,
                   const openparasolid::NurbsCurveGeometry& curve);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_NURBS_WIRE_H_
