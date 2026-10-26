// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
/**
 * @file
 * @brief Bounded evaluation and inversion of source geometry.
 * @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_surfaces "XT 5.2.2:
 * Surfaces (p. 53)", @ref xt_senses "XT 5.2.8: Curve, surface and lattice
 * senses (p. 82)", @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
 */
#ifndef OPENPARASOLID_EVALUATE_H_
#define OPENPARASOLID_EVALUATE_H_

#include <array>
#include <span>
#include <vector>

#include "openparasolid/model.h"

namespace openparasolid {

/// @brief Curve position and natural parameter derivative in source
/// coordinates.
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_senses "XT 5.2.8:
/// Curve, surface and lattice senses (p. 82)"
struct CurveEvaluation {
  /// Position in source coordinates and source length units.
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
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_senses "XT 5.2.8:
/// Curve, surface and lattice senses (p. 82)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)", @ref xt_spcurve "XT 5.2.1.8: SP_CURVE (p. 51)"
///
/// @param geometry Typed source-coordinate geometry payload.
/// @param parameter Finite natural source parameter, or the documented
/// piecewise path parameter.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
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
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_senses "XT 5.2.8:
/// Curve, surface and lattice senses (p. 82)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)", @ref xt_spcurve "XT 5.2.1.8: SP_CURVE (p. 51)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param curve Stream-local curve identity to evaluate.
/// @param parameter Finite natural source parameter, or the documented
/// piecewise path parameter.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<CurveEvaluation> EvaluateCurve(const Model& model,
                                                    NodeId curve,
                                                    double parameter);

/// @brief Surface position and natural partial derivatives in source
/// coordinates.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
struct SurfaceEvaluation {
  /// Position in source coordinates and source length units.
  Vector3 position;
  /// Natural partial derivatives, without normalization or sense reversal.
  Vector3 u_derivative;
  /// Natural partial derivative dP/dv; not normalized or sense-reversed.
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
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)", @ref xt_spun
/// "XT 5.2.2.11: Spun_surf (p. 68)", @ref xt_swept "XT 5.2.2.10: Swept_surf (p.
/// 67)", @ref xt_offset "XT 5.2.2.8: Offset_surf (p. 62)"
///
/// @param geometry Typed source-coordinate geometry payload.
/// @param u Natural source U parameter; angular coordinates use radians.
/// @param v Natural source V parameter; angular coordinates use radians.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSurface(
    const GeometryPayload& geometry, double u, double v);

/// Resolves a surface by ID, including SPUN_SURF or SWEPT_SURF with an
/// analytic or 3D NURBS generator, and supported analytic OFFSET_SURF chains.
/// Offset bases may be planes, cylinders, cones, spheres or regular ring tori;
/// basis sense affects signed offsets, with a 64-layer/cycle bound. General
/// NURBS/blend offsets and singular offset transitions remain unsupported.
/// u is the original curve parameter. For a
/// spun surface v is an unbounded angle in radians about the stored unit axis;
/// for a swept surface v is signed displacement along its stored unit sweep.
/// Spun start/end parameters bound u; the profile's own domain and resource
/// checks also apply. Natural partials are returned without sense reversal or
/// normalization. Auxiliary start/end points, x_axis and internal scale are
/// retained, not applied or certified. Missing/wrong profiles are invalid
/// records; unsupported dependencies remain explicit errors. No recursive
/// profile graphs are evaluated.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)", @ref xt_spun
/// "XT 5.2.2.11: Spun_surf (p. 68)", @ref xt_swept "XT 5.2.2.10: Swept_surf (p.
/// 67)", @ref xt_offset "XT 5.2.2.8: Offset_surf (p. 62)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param surface Stream-local surface identity to evaluate.
/// @param u Natural source U parameter; angular coordinates use radians.
/// @param v Natural source V parameter; angular coordinates use radians.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSurface(const Model& model,
                                                        NodeId surface,
                                                        double u, double v);

/// Evaluates a unit normal in source coordinates. Uses the natural cross
/// product dP/du x dP/dv, reversed for surface sense '-'. The surface must
/// have known sense '+' or '-'. Face sense and placement are not applied.
/// Inherits EvaluateSurface's domain and resource checks. Zero or numerically
/// unresolvable cross products return kInvalidRecord; unknown surface sense
/// returns kUnsupportedFormat. No normal is invented at singular parameters.
///
/// @see @ref xt_senses "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param surface Stream-local surface identity to evaluate.
/// @param u Natural source U parameter; angular coordinates use radians.
/// @param v Natural source V parameter; angular coordinates use radians.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<Vector3> EvaluateSurfaceNormal(const Model& model,
                                                    NodeId surface, double u,
                                                    double v);

/// @brief Recovered natural UV parameters and measured 3D membership residual.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
struct SurfaceParameters {
  /// Natural U parameter in the source surface parameterization.
  double u = 0;
  /// Natural V parameter in the source surface parameterization.
  double v = 0;
  /// Distance to the point returned by EvaluateSurface at these parameters.
  double residual = 0;
};
/// @brief Caller-selected initial UV pair for bounded inversion.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
struct SurfaceParameterSeed {
  /// Natural U parameter in the source surface parameterization.
  double u = 0;
  /// Natural V parameter in the source surface parameterization.
  double v = 0;
};

/// Finds natural parameters on an analytic surface (plane, cylinder, cone,
/// sphere or torus) or a NURBS surface. The finite nonnegative absolute
/// tolerance is in source units. Success requires the forward-evaluated point
/// to be within tolerance; this is not a general closest-point projection.
/// Angular parameters use atan2's principal branch; an undefined azimuth is
/// represented by zero. Sphere poles and cone/torus singularities may yield
/// parameters while the normal remains undefined. An axial cone candidate
/// beyond the apex also tries the nearest representable apex parameter on the
/// valid side, subject to the same absolute 3D tolerance. The torus's principal
/// sheet is retained. Source frames, senses and units are not changed. Invalid
/// request values return kInvalidOptions, nonmatching points or invalid
/// geometry/results kInvalidRecord, other geometry kinds kUnsupportedFormat.
/// Inherits the forward evaluator's validity/domain checks. No implicit
/// tolerance is added. NURBS uses bounded damped least-squares searches in
/// normalized knot domains: at most 512 evaluations and 32,000,000 estimated
/// validation/evaluation work units. Exhaustion returns kResourceLimit. A
/// failed local search is not proof that the point has no preimage; unique
/// roots are not guaranteed.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
///
/// @param geometry Typed source-coordinate geometry payload.
/// @param point Point in source coordinates to test for surface membership.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const GeometryPayload& geometry, Vector3 point, double tolerance);
/// The model overload also inverts circular SPUN_SURF rings whose profile
/// lies in a meridian plane and whose major radius exceeds its profile radius.
/// It also supports circular pole-to-pole meridians centered on the spin axis
/// (spherical support), with both profile bounds selecting one half-turn and
/// placing its endpoints at the poles. These bounds select the source polar
/// branch; horn/spindle rings and incomplete or repeated meridians remain
/// unsupported. Polar spin angles are nonunique at the poles.
/// It preserves source profile bounds and refines against the source evaluator:
/// at most 128 evaluations shared by the preferred and analytic starts, with
/// 24 updates and eight damping trials per update. Exhaustion returns
/// kResourceLimit. Other spun profiles remain unsupported.
/// SWEPT_SURF with a line, circle or ellipse section uses direct inversion
/// of the stored basis. Lines require independent section/sweep directions;
/// conics require a sweep transverse to the section plane. Conic u uses atan2's
/// principal branch, and v is signed source sweep displacement. At most two
/// model surface evaluations validate (0,0) and the recovered point, with no
/// implicit tolerance. Open nonrational clamped 3D B_CURVE sections are also
/// supported when their projected derivative is certified strictly positive
/// perpendicular to the sweep. Positive control differences provide the fast
/// proof; original knot spans up to degree seven additionally use outward
/// blossom extraction and bounded Bernstein subdivision (depth 16). This proves
/// a unique u on the source knot domain, including nonplanar sections and
/// repeated interior knots. At most
/// 130 curve samples and two surface samples are used, with a 32,000,000
/// estimated work-unit limit; exhausted budgets return kResourceLimit.
/// Rational sections up to degree seven use a positive interval Bernstein
/// certificate for A'W-AW' on every span, with at most one million estimated
/// proof operations. All valid interior multiplicities are supported; original
/// homogeneous controls/weights and knots remain unchanged (XT 5.2.1.4:
/// B_CURVE, p. 38; 5.2.2.10: Swept_surf, p. 67). Higher-degree rational, closed,
/// periodic and uncertified sections remain unsupported.
/// Finite seeds do not change this unique branch. The final 3D membership
/// residual uses the caller's unchanged tolerance; no closest-point claim is
/// made. Other swept profiles remain unsupported.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)", @ref xt_spun
/// "XT 5.2.2.11: Spun_surf (p. 68)", @ref xt_swept
/// "XT 5.2.2.10: Swept_surf (p. 67)", @ref xt_bcurve
/// "XT 5.2.1.4: B_CURVE (B-spline curve) (p. 38)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param surface Stream-local surface identity to evaluate.
/// @param point Point in source coordinates to test for surface membership.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(const Model& model,
                                                      NodeId surface,
                                                      Vector3 point,
                                                      double tolerance);

/// Supplies a preferred starting UV pair for bounded NURBS or model-based
/// circular spun inversion. Spun seeds are clamped to stored profile bounds.
/// Both coordinates must be finite and, for NURBS, inside the defined knot
/// domain. Other deterministic starts are tried if local iteration fails.
/// Analytic surfaces and analytic sweeps validate the finite seed and use their
/// direct inversion formulas. No closest-point, unique-root or
/// periodic-continuation guarantee is made.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
///
/// @param geometry Typed source-coordinate geometry payload.
/// @param point Point in source coordinates to test for surface membership.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param seed Finite preferred starting UV pair for bounded inversion.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const GeometryPayload& geometry, Vector3 point, double tolerance,
    SurfaceParameterSeed seed);
/// @brief Recovers UV parameters with the supplied seed and membership
/// tolerance.
///
/// @see @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param surface Stream-local surface identity to evaluate.
/// @param point Point in source coordinates to test for surface membership.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param seed Finite preferred starting UV pair for bounded inversion.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<SurfaceParameters> InvertSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance,
    SurfaceParameterSeed seed);

/// @brief Reconstructed chart parameters and per-chord projection scales.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
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
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
///
/// @param chart Stored intersection chart to parameterize.
/// @param tangents One finite nonzero tangent per chart position, oriented in
/// chart order.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<IntersectionChartParameters> ParameterizeIntersectionChart(
    const IntersectionChart& chart, std::span<const Vector3> tangents);

/// @brief Chart samples with recovered UV pairs, tangents and parameterization.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionChartEvaluation {
  /// Validated UVs that could not meet the optional stricter normal goal.
  /// Original chart membership is retained; no tolerance is increased.
  uint64_t normal_refinement_fallbacks = 0;
  /// Natural parameters and measured residual for each stored surface, per
  /// point.
  std::vector<std::array<SurfaceParameters, 2>> surface_parameters;
  /// Unit N0 x N1 directions in stored surface order, without curve sense
  /// flips.
  std::vector<Vector3> tangents;
  /// Reconstructed natural chart parameters and per-chord scales.
  IntersectionChartParameters parameterization;
};

/// Reconstructs a directly referenced INTERSECTION's chart from the model.
/// Both targets must have known senses and support analytic, NURBS, circular
/// spun ring/sphere, supported swept-section or analytic-offset inversion.
/// The previous UV supplies the next hint;
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
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param curve Stream-local curve identity to evaluate.
/// @param tolerance Finite nonnegative absolute 3D membership tolerance in
/// source length units.
/// @param normal_refinement_tolerance Optional stricter UV residual goal before
/// computing normals and the XT chord/tangent recurrence. Zero disables it.
/// If the stricter membership solve fails, the original membership allowance
/// is tried and a fallback is recorded; resource, option and unsupported errors remain explicit. At most
/// two inversions per chart point/surface are performed. Stored points,
/// chart positions and the caller's membership allowance are not changed.
/// Reconstructed parameters may improve because the tangent estimates improve.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<IntersectionChartEvaluation> EvaluateIntersectionChart(
    const Model& model, NodeId curve, double tolerance,
    double normal_refinement_tolerance = 0);

/// @brief Explicit caller tolerances and regularity thresholds for local
/// intersection solves.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionEvaluationOptions {
  /// Nonnegative absolute membership tolerance for the stored chart points.
  double chart_tolerance = 0;
  /// Positive absolute residual tolerance for interior coincidence/plane
  /// solves. Also requests this UV residual goal for chart normals; the
  /// separate chart membership allowance remains unchanged.
  double solve_tolerance = 0;
  /// Nonnegative absolute tolerance in original curve parameter units. Allows
  /// snapping just outside outer chart endpoints; zero retains strict bounds.
  /// Does not relax trim intervals or move interior parameters/chart knots.
  double chart_parameter_tolerance = 0;
  /// Minimum sine of the angle between the two unit surface normals at the
  /// query. Finite in [0,1]; zero adds no minimum beyond existing singularity
  /// checks. Reversing a surface sense does not change this measure.
  double minimum_normal_sine = 0;
  /// Minimum positive unit tangent/chord dot product at the query, finite in
  /// [0,1]. Zero preserves the existing strict positive-direction check.
  double minimum_chord_projection = 0;
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
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_senses "XT 5.2.8:
/// Curve, surface and lattice senses (p. 82)", @ref xt_trim "XT 5.2.1.6:
/// Trimmed_curve (p. 49)", @ref xt_spcurve "XT 5.2.1.8: SP_CURVE (p. 51)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param curve Stream-local curve identity to evaluate.
/// @param parameter Finite natural source parameter, or the documented
/// piecewise path parameter.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<CurveEvaluation> EvaluateCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options);

/// Evaluates a directly referenced INTERSECTION with analytic, NURBS, supported
/// circular spun ring/sphere, supported swept-section or analytic-offset surfaces
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
/// Model surface evaluation retains spun profile bounds. Spin angles and
/// unbounded circular profile angles use local periodic interpolation; bounded
/// profile parameters interpolate directly without extending their domain.
/// Swept conic sections use local angular interpolation. Swept B-spline
/// parameters retain the source knot domain. Swept line parameters and signed
/// sweep displacement retain linear units without angular wrapping.
/// Analytic offsets inherit their resolved basis's angular/linear coordinates.
/// Model-based offset checks, senses, cycle detection and the 64-layer limit
/// apply during chart preparation, interpolation, sampling and normals.
/// Optional geometric regularity minimums apply at the query and reject
/// samples below a minimum with kInvalidRecord.
/// Outside-chart parameters beyond chart_parameter_tolerance are invalid
/// options. Within that explicit parameter tolerance, an outside query returns
/// the nearest outer chart endpoint and its natural derivative (not the zero
/// derivative of a clamped extension). Inside queries are unchanged.
/// No terminator extension, trim unwrapping, global branch certification or
/// inversion of other surface kinds is provided. EvaluateCurve without options
/// retains its unsupported result for intersections.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param curve Stream-local curve identity to evaluate.
/// @param parameter Finite natural source parameter, or the documented
/// piecewise path parameter.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<CurveEvaluation> EvaluateIntersectionCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options);

/// @brief Local regularity measures; no global accuracy or branch certificate.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionCurveDiagnostics {
  /// Sine of the angle between the unit normals, in (0,1]. Near zero means
  /// the supporting surfaces are nearly tangent.
  double surface_normal_sine = 0;
  /// Unit tangent dotted with the selected unit chord, in (0,1]. Near zero
  /// amplifies the chart's natural derivative scale.
  double tangent_chord_projection = 0;
};
/// @brief Intersection sample and its local regularity diagnostics.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionCurveEvaluation {
  /// Natural intersection position and dP/dt sample.
  CurveEvaluation curve;
  /// Local regularity measures computed at this intersection sample.
  IntersectionCurveDiagnostics diagnostics;
};

/// Returns the same curve sample as EvaluateIntersectionCurve, plus local
/// geometric regularity measures. Both APIs apply the options' optional
/// minimums at the query, including exact/snapped chart endpoints, returning
/// kInvalidRecord below a minimum. Measures use recovered/solved UV values;
/// at chart knots the returned position remains the stored chart point.
/// Neither the measures nor passing the minimums certify position/derivative
/// accuracy, chart-wide regularity, a Jacobian condition bound or topology.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
///
/// @param model Source model providing stream-local identities and typed
/// geometry.
/// @param curve Stream-local curve identity to evaluate.
/// @param parameter Finite natural source parameter, or the documented
/// piecewise path parameter.
/// @param options Per-call settings and resource limits; see the associated
/// options type.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<IntersectionCurveEvaluation>
EvaluateIntersectionCurveWithDiagnostics(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options);

}  // namespace openparasolid

#endif  // OPENPARASOLID_EVALUATE_H_
