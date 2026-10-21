// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_EVALUATE_H_
#define OPENPARASOLID_EVALUATE_H_

#include <array>
#include <span>
#include <vector>

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

/// Resolves a curve by ID, including TRIMMED_CURVE bases and SP_CURVE's 2D
/// NURBS and supporting surface. Applies the natural chain rule without sense
/// reversal, face trimming, placement transforms or periodic parameter
/// wrapping. References are scoped to this model; a standalone SP_CURVE payload
/// needs this context. Missing/wrong targets and non-2D parameter curves are
/// invalid records; untyped/unsupported geometry returns kUnsupportedFormat. No
/// recursion is performed. Trim chains are iterative, with cycle detection and
/// at most 64 wrappers. The parameter is the original basis parameter, bounded
/// by every inclusive trim interval. Basis sense must be known and match
/// endpoint order; the natural derivative is not reversed or scaled. Stored
/// endpoint points and periodic span legality are not certified. Inherited
/// parameter/data/ resource checks remain in force.
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
/// nonzero signed acute half-angle and r + v*tan(angle) >= 0 (writer
/// convention: axial term +v*A). Apple/lemon tori use the principal branch |v|
/// <= acos(-major/minor); ring tori have no v restriction. No periodic branch
/// wrapping is performed. Singular boundary points are allowed: partial
/// derivatives may be zero. Surface/face sense, face trimming, placement
/// transforms and units are not applied. NURBS controls have dimension 3
/// (nonrational) or 4 (homogeneous), with v varying fastest: control(u,v) is at
/// u*n_v + v. Each knot axis follows EvaluateCurve's domain, derivative-side
/// and resource rules; the shared control net is limited to 1,000,000 scalars.
/// Errors follow EvaluateCurve.
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSurface(
    const GeometryPayload& geometry, double u, double v);

/// Resolves a surface by ID, including SPUN_SURF or SWEPT_SURF with an
/// analytic or 3D NURBS generator. u is the original curve parameter. For a
/// spun surface v is an unbounded angle in radians about the stored unit axis;
/// for a swept surface v is signed displacement along its stored unit sweep.
/// Spun start/end parameters bound u; the profile's own domain and resource
/// checks also apply. Natural partials are returned without sense reversal or
/// normalization. Auxiliary start/end points, x_axis and internal scale are
/// retained, not applied or certified. Missing/wrong profiles are invalid
/// records; unsupported dependencies remain explicit errors. No recursive
/// profile graphs are evaluated.
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSurface(const Model& model,
                                                        NodeId surface,
                                                        double u, double v);

/// Evaluates a unit normal in source coordinates. Uses the natural cross
/// product dP/du x dP/dv, reversed for surface sense '-'. The surface must
/// have known sense '+' or '-'. Face sense and placement are not applied.
/// Inherits EvaluateSurface's domain and resource checks. Zero or numerically
/// unresolvable cross products return kInvalidRecord; unknown surface sense
/// returns kUnsupportedFormat. No normal is invented at singular parameters.
[[nodiscard]] Result<Vector3> EvaluateSurfaceNormal(const Model& model,
                                                   NodeId surface,
                                                   double u, double v);

struct SurfaceParameters {
  double u = 0;
  double v = 0;
  /// Distance to the point returned by EvaluateSurface at these parameters.
  double residual = 0;
};
struct SurfaceParameterSeed {
  double u = 0;
  double v = 0;
};

/// Finds natural parameters on an analytic surface (plane, cylinder, cone,
/// sphere or torus) or a NURBS surface. The finite nonnegative absolute
/// tolerance is in source units. Success requires the forward-evaluated point
/// to be within tolerance; this is not a general closest-point projection.
/// Angular parameters use atan2's principal branch; an undefined azimuth is
/// represented by zero. Sphere poles and cone/torus singularities may yield
/// parameters while the normal remains undefined. The torus's principal sheet
/// is retained. Source frames, senses and units are not changed. Invalid
/// request values return kInvalidOptions, nonmatching points or invalid
/// geometry/results kInvalidRecord, other geometry kinds kUnsupportedFormat.
/// Inherits the forward evaluator's validity/domain checks. No implicit
/// tolerance is added. NURBS uses bounded damped least-squares searches in
/// normalized knot domains: at most 512 evaluations and 32,000,000 estimated
/// validation/evaluation work units. Exhaustion returns kResourceLimit. A
/// failed local search is not proof that the point has no preimage; unique
/// roots are not guaranteed.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const GeometryPayload& geometry, Vector3 point, double tolerance);
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance);

/// Supplies a preferred starting UV pair for bounded NURBS inversion. Both
/// coordinates must be finite and, for NURBS, inside the defined knot domain.
/// Other deterministic starts are tried if local iteration fails. Analytic
/// surfaces validate the finite seed and use their direct inversion formulas.
/// No closest-point, unique-root or periodic-continuation guarantee is made.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const GeometryPayload& geometry, Vector3 point, double tolerance,
    SurfaceParameterSeed seed);
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance,
    SurfaceParameterSeed seed);

struct IntersectionChartParameters {
  /// One strictly increasing parameter per chart position.
  std::vector<double> parameters;
  /// One positive projection scale per chord, with no final unused scale.
  std::vector<double> chord_scales;
};

/// Reconstructs chart parameters using the XT chord/tangent recurrence.
/// Supply one finite nonzero tangent per chart position, oriented in chart
/// order (normally N0 x N1 using the oriented surface normals). Tangent
/// magnitudes need not be one. Positions and tangents are not modified.
/// Every adjacent chord must be nonzero and project positively onto both end
/// tangents. The base scale must be finite and positive. Parameters and scales
/// must remain representable, with strictly increasing double parameters.
/// At least two positions are required; at most 1,000,000 coordinate scalars
/// per input array are accepted. Count mismatch returns kInvalidOptions,
/// oversized charts kResourceLimit, invalid data/results kInvalidRecord.
/// Does not find surface parameters/tangents, check curve membership or error
/// estimates, extend to terminators, or evaluate between chart positions.
[[nodiscard]] Result<IntersectionChartParameters> ParameterizeIntersectionChart(
    const IntersectionChart& chart, std::span<const Vector3> tangents);

struct IntersectionChartEvaluation {
  /// Natural parameters and measured residual for each stored surface, per point.
  std::vector<std::array<SurfaceParameters, 2>> surface_parameters;
  /// Unit N0 x N1 directions in stored surface order, without curve sense flips.
  std::vector<Vector3> tangents;
  IntersectionChartParameters parameterization;
};

/// Reconstructs a directly referenced INTERSECTION's chart from the model.
/// Both surface targets must support analytic or NURBS inversion and have
/// known senses. The previous recovered UV supplies the next point's hint;
/// local NURBS roots and periodic seam continuity are not certified.
/// Each stored chart position must invert onto both surfaces within the finite
/// nonnegative absolute tolerance in source units. Normals are sampled at the
/// recovered UV values; their cross product supplies the natural tangent.
/// Returns one UV pair, tangent and parameter per point, plus one scale per
/// chord. Inherits inversion, normal and chart parameterisation checks,
/// including the limit of 1,000,000 position scalars before allocating output
/// arrays. Missing/wrong roles, singular normals and incompatible chart order
/// are explicit errors; unsupported dependencies remain kUnsupportedFormat.
/// Does not unwrap trims, evaluate between chart points or extend to
/// terminators. Auxiliary UV data/error estimates, curve/face senses and
/// placement are not applied. The source model and chart are not modified.
[[nodiscard]] Result<IntersectionChartEvaluation> EvaluateIntersectionChart(
    const Model& model, NodeId curve, double tolerance);

struct IntersectionEvaluationOptions {
  /// Nonnegative absolute membership tolerance for the stored chart points.
  double chart_tolerance = 0;
  /// Positive absolute residual tolerance for interior coincidence/plane solves.
  double solve_tolerance = 0;
  /// Nonnegative absolute tolerance in original curve parameter units. Allows
  /// snapping just outside outer chart endpoints; zero retains strict bounds.
  /// Does not relax trim intervals or move interior parameters/chart knots.
  double chart_parameter_tolerance = 0;
};

/// Evaluates model curves with explicit options for supported intersections,
/// including intersections reached through TRIMMED_CURVE chains. Validates
/// options for every geometry kind. Other curves retain the existing evaluator
/// behavior. Trim intervals, basis sense, cycle/depth and resource checks still
/// apply; the original basis parameter and natural derivative are preserved.
/// Intersection chart bounds and optional endpoint snapping also apply, with
/// no terminator extension. Stored trim endpoints and branch topology are not
/// certified. The overload without
/// options retains its unsupported result for intersections.
[[nodiscard]] Result<CurveEvaluation> EvaluateCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options);

/// Evaluates a directly referenced INTERSECTION with analytic/NURBS supports
/// within its chart.
/// Requires a finite parameter and explicit finite chart/solve tolerances in
/// source units. Optional endpoint tolerance uses original parameter units.
/// Prepares the chart, then locally solves coincidence of its
/// two supporting surfaces with the XT chord-normal plane. Returns the natural
/// dP/dt; curve sense does not reverse it. Exact chart parameters return their
/// stored positions (subject to chart_tolerance, not solve_tolerance).
/// Interior solves use at most 32 Newton updates and 16
/// damping trials per update; singular/stalled/nonconvergent solves are errors.
/// Chart/inversion/normal resource and validity checks remain in force.
/// Outside-chart parameters beyond chart_parameter_tolerance are invalid
/// options. Within that explicit parameter tolerance, an outside query returns
/// the nearest outer chart endpoint and its natural derivative (not the zero
/// derivative of a clamped extension). Inside queries are unchanged.
/// No terminator extension, trim unwrapping, global branch certification or
/// inversion of other surface kinds is provided. EvaluateCurve without options
/// retains its unsupported result for intersections.
[[nodiscard]] Result<CurveEvaluation> EvaluateIntersectionCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options);

}  // namespace openparasolid

#endif  // OPENPARASOLID_EVALUATE_H_
