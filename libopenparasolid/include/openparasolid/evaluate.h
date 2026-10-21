// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_EVALUATE_H_
#define OPENPARASOLID_EVALUATE_H_

#include "openparasolid/model.h"

namespace openparasolid {

struct CurveEvaluation {
  Vector3 position;
  /// Natural derivative dP/dt, not an oriented or normalized tangent.
  Vector3 first_derivative;
};

/// Evaluates an untrimmed LINE, CIRCLE, ELLIPSE or NURBS curve in source
/// coordinates. Angular parameters are radians. Curve sense and topology do not
/// change t or the derivative. No units, axes or directions are normalized.
/// Directions must be unit length and conic frames orthogonal within 1e-10.
/// NURBS parameters must lie in [expanded_knots[degree],
/// expanded_knots[control_count]]. No periodic wrapping is performed. At knots
/// the derivative is right-sided, except at the domain end (left-sided).
/// 2D NURBS controls produce z=0. Rational controls are already homogeneous.
/// NURBS evaluation bounds: degree <=64; at most 1,000,000 control scalars,
/// distinct knots, multiplicities and expanded knots (each separately).
/// Invalid parameters/data, nonfinite results and unsupported payloads return
/// explicit errors (offset zero, since payloads have no source byte offset).
[[nodiscard]] Result<CurveEvaluation> EvaluateCurve(
    const GeometryPayload& geometry, double parameter);

}  // namespace openparasolid

#endif  // OPENPARASOLID_EVALUATE_H_
