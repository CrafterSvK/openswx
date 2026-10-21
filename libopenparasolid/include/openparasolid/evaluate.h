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

/// Resolves a curve by ID in this model, including SP_CURVE's 2D NURBS and
/// supporting surface. Applies the natural chain rule without sense reversal,
/// trimming, placement transforms or periodic parameter wrapping. References
/// are scoped to this model; a standalone SP_CURVE payload needs this context.
/// Missing/wrong targets and non-2D parameter curves are invalid records;
/// untyped/unsupported geometry returns kUnsupportedFormat. No recursion is
/// performed. Inherited parameter/data/resource checks remain in force.
[[nodiscard]] Result<CurveEvaluation> EvaluateCurve(const Model& model,
                                                    NodeId curve,
                                                    double parameter);

struct SurfaceEvaluation {
  Vector3 position;
  /// Natural partial derivatives, without normalization or sense reversal.
  Vector3 u_derivative;
  Vector3 v_derivative;
};

/// Evaluates a PLANE, CYLINDER, CONE, SPHERE, TORUS or NURBS surface in source
/// coordinates. Angular parameters are radians. Frames must be orthonormal
/// within 1e-10. Sphere latitude v lies in [-pi/2, pi/2]. Cones require a
/// positive acute half-angle and r + v*tan(angle) >= 0 (XT axis convention:
/// axial term -v*A). Apple/lemon tori use the principal branch |v| <=
/// acos(-major/minor); ring tori have no v restriction. No periodic branch
/// wrapping is performed. Singular boundary points are allowed: partial
/// derivatives may be zero. Surface/face sense, face trimming, placement
/// transforms and units are not applied. NURBS controls have dimension 3
/// (nonrational) or 4 (homogeneous), with v varying fastest: control(u,v) is at
/// u*n_v + v. Each knot axis follows EvaluateCurve's domain, derivative-side
/// and resource rules; the shared control net is limited to 1,000,000 scalars.
/// Errors follow EvaluateCurve.
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSurface(
    const GeometryPayload& geometry, double u, double v);

}  // namespace openparasolid

#endif  // OPENPARASOLID_EVALUATE_H_
