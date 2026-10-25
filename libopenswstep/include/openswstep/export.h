// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_EXPORT_H_
#define OPENSWSTEP_EXPORT_H_

/// @file
/// @brief Exact STEP export of selected SolidWorks/Parasolid body geometry.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "openparasolid/brep.h"
#include "openparasolid/schema.h"
#include "openswx/container.h"

/// @brief Bounded, kernel-independent STEP export using openswx and
/// openparasolid. Input models and payloads are borrowed during calls; output
/// documents own all data. Units and stream/body selection are always explicit.
namespace openswstep {
/// @brief Explicit source length unit; coordinates are retained without
/// scaling.
enum class LengthUnit {
  kUnspecified,  ///< Invalid until the caller supplies the source unit.
  kMetre,        ///< Coordinates and tolerances are expressed in metres.
  kMillimetre,   ///< Coordinates and tolerances are expressed in millimetres.
};

/// @brief Caller settings and positive resource budgets for one selected body.
struct ExportOptions {
  /// Required source unit. No unit is guessed from model coordinates.
  LengthUnit length_unit = LengthUnit::kUnspecified;
  /// Positive absolute binding tolerance and base STEP length uncertainty in
  /// source units. Used source edge tolerances can raise only the output
  /// uncertainty, as described by use_source_edge_tolerances.
  double tolerance = 1e-7;
  /// Honor positive transmitted EDGE tolerances only for certified paired
  /// tolerant circular edges. Their source pcurves are retained, and the STEP
  /// length uncertainty records the maximum used tolerance. False requires
  /// agreement within tolerance alone; binding tolerances are never widened.
  bool use_source_edge_tolerances = true;
  /// Product name, printable ASCII, at most 1024 bytes. Quotes are escaped.
  std::string product_name = "SolidWorks body";
  /// Aggregate cap on generated STEP instances, including product metadata.
  uint64_t max_entities = 1000000;
  /// Cap on the complete owned Part 21 output, including its header.
  uint64_t max_output_bytes = 128u * 1024u * 1024u;
  /// Shared boundary-check budget across the selected body: segment,
  /// containment, separation, convex corners, curved hole/edge and polygon
  /// vertex/edge pairs, curved-hole extrema, cut pairs and spherical cap
  /// selections. Each cylindrical/conical patch or annular sector consumes five
  /// checks. Profiled cylinder bands consume one check per edge and one per
  /// cut pair. Split torus cuts consume one check per arc in addition to the
  /// cut-pair check. Concave minor-arc contours share chord-polygon, join-cone
  /// and nonadjacent envelope checks, including a preceding convex attempt.
  /// Automatic closed tolerant-ring recovery charges both its preflight and
  /// final Bernstein certificates to this same budget. Open B-spline wires
  /// consume one endpoint separation check per interior control point.
  /// Partial or unclamped restriction also charges the original control count
  /// for copying and the new control count at each homogeneous knot insertion.
  /// Planar polynomial spline bounds consume one plane check per control;
  /// cubics additionally consume one derivative-cone check per Bezier piece.
  /// Both C1 and explicitly verified tangent joins consume one check each. Each
  /// piece restriction is charged separately. Face preparation and final edge
  /// serialization share this budget, including restriction work in both stages
  /// and the existing corner checks. Spline-hole envelopes additionally charge
  /// one check per retained control and their own interval restriction work.
  uint64_t max_polygon_checks = 1000000;
  /// Limits for sequential decoding of an extracted binary payload.
  openparasolid::SchemaLimits parsing;
  /// Limits for building the source model graph.
  openparasolid::ModelOptions model;
  /// Limits and optional intersection settings for complete body binding.
  openparasolid::BodyGeometryBindingOptions binding;
  /// Optional directed fin intervals; supplied intervals take precedence.
  /// Missing closed tolerant-ring intervals may be resolved by the exporter
  /// only after the paired circular pcurves are certified.
  std::vector<openparasolid::FinParameterInterval> fin_intervals;
  /// Optional directed natural-edge wire intervals; other wires use inference.
  std::vector<openparasolid::WireParameterInterval> wire_intervals;
};

/// @brief Export counts for a successfully serialized complete selected body.
struct ExportStatistics {
  uint64_t entities = 0;  ///< Number of generated Part 21 instances.
  uint64_t vertices =
      0;               ///< STEP vertices, including synthetic conic ring seams.
  uint64_t edges = 0;  ///< Distinct exported topological edges.
  uint64_t faces =
      0;  ///< Exported planar, cylindrical, conical or toroidal faces.
  uint64_t shells = 0;  ///< Exported sheet or solid shells.
  uint64_t solids = 0;  ///< Exported manifold solid representations.

  /// Actual STEP length uncertainty, including any used source edge tolerance.
  double length_uncertainty = 0;
};

/// @brief Owned ISO 10303-21/AP214 document; no source model lifetime
/// dependency.
struct StepDocument {
  std::string text;             ///< Complete STEP text, using LF line endings.
  ExportStatistics statistics;  ///< Counts of the serialized representation.
};

/// @brief Binds and exports one body in source coordinates, without
/// tessellation. Supports LINE/CIRCLE/ELLIPSE wire bodies and open
/// B_CURVE wires with three spatial coordinates, including unclamped axes.
/// Rational homogeneous controls become Cartesian poles plus positive STEP
/// weights; degree and directed source endpoints are retained. Complete domains
/// preserve knots and controls unchanged for clamped axes; partial or unclamped
/// domains use homogeneous knot insertion and cropping, with original
/// parameters and retained interior knots. Rounded restricted endpoints must
/// still meet the named source vertex tolerance. Source trim wrappers and
/// caller wire intervals can select partial domains. The natural source domain
/// is U[p] through U[N] in the expanded knot vector, with degree p and control
/// count N; exterior knots do not allow extrapolation. All retained interior
/// control points must lie strictly between the endpoint planes normal to the
/// endpoint chord, with tolerance/roundoff clearance. Positive weights then
/// exclude ambiguous interior occurrences of the edge endpoints. This is not a
/// wire self-intersection certificate. Periodic or closed spline wires remain
/// unsupported. The planar polynomial face subset is described below.
/// Supports point-only acorns, and planar
/// polygon faces with LINE edges, one outer loop and disjoint, nonnested
/// polygonal and full-circle hole loops. Source loop order does not determine
/// the outer loop. Planar faces also support one-edge full CIRCLE/ELLIPSE disks
/// and circular faces with disjoint polygonal and/or circular holes. Complete
/// CYLINDER/CONE bands use two coaxial full-circle loops at distinct axial
/// levels and positive cut radii. CYLINDER/CONE also support a single four-edge
/// loop with two coaxial circular arcs and two straight LINE generators,
/// spanning less than one full turn. A complete cone cap additionally supports
/// one full circular cut and an isolated vertex at its certified apex. The
/// singular point is implicit in the CONICAL_SURFACE; only the circular
/// FACE_OUTER_BOUND is serialized, matching native SolidWorks AP214 output.
/// No regular source edge or face is dropped. Additional loops, partial apex
/// cuts and apices shared with other face boundaries remain unsupported.
/// Generators follow the respective
/// cylinder/cone profile. Full-azimuth CYLINDER bands also support monotone
/// CIRCLE/ELLIPSE arc profiles with isolated axial LINE steps. Whole supporting
/// conics must project onto the cylinder circle; complete profile axial ranges
/// must be separated by more than two tolerances. Overlapping ranges remain
/// unsupported. Exact transverse and axial flux terms are retained for solid
/// patches and variable-height complete cylinder bands. SPHERE faces use one
/// full latitude circle for a cap or two full coaxial latitude circles for a
/// band, away from the poles. Winding selects either cap, including caps larger
/// than a hemisphere. Regular ring TORUS bands use two coaxial full-circle
/// cuts; winding selects the meridian interval, including its periodic seam and
/// complementary band. Each complete cut can also comprise monotone arcs on one
/// supporting circle, retaining the individual source edges. Unbounded
/// SPUN_SURF circular generators in an axial meridian plane of a regular ring
/// torus use the equivalent TOROIDAL_SURFACE with corrected normal orientation.
/// A circular pole-to-pole meridian centered on the spin axis uses an equivalent
/// SPHERICAL_SURFACE. Its complete half-turn, meridian frame and source normal
/// are certified within the unchanged length tolerance. Full circular cuts may
/// be oblique to the source spin axis; the equivalent sphere frame aligns with
/// the cut, retaining exact source edges. Other bounded generators, tilted/skew
/// circle planes and other generators remain unsupported.
/// Planar faces also support a single convex
/// LINE/CIRCLE/ELLIPSE contour, including split conics, elliptic arcs and
/// two-edge segments/lenses. A hole-free convex contour may additionally use
/// polynomial quadratic/cubic B_CURVE edges, including smooth multi-span
/// sections. Each selected knot span is restricted to a Bezier piece for
/// validation; all three/four controls must lie in the face plane within length
/// tolerance. Their convex hull certifies the whole selected curve. Nonnegative
/// tangent turning shorter than pi excludes derivative reversals. Cubic middle
/// derivative controls must lie in their ordered endpoint tangent cone;
/// nonnegative curvature Bernstein coefficients certify the whole piece. Zero
/// middle controls are allowed when endpoint derivatives remain nonzero. The
/// sufficient cone certificate can reject other valid convex cubics. Existing
/// convex corners and one-full-revolution checks remain in force. Polynomial
/// area is integrated analytically; no control polygon replaces the source
/// curve. At full-degree knots, internal tangent directions must agree within
/// an angular allowance bounded by 1e-10 radians and the source
/// tolerance/perimeter ratio, divided across the pieces. Lower multiplicities
/// prove C1 continuity; rounded piece tangents must agree with the shared
/// source derivative within the length tolerance/perimeter ratio divided across
/// pieces. Its signed roundoff correction changes no source geometry or STEP
/// uncertainty. Speeds may differ; genuine internal kinks fail explicitly.
/// Source traversal order determines accumulated area and tangent turning. The
/// selected spline is serialized once per source edge, preserving interior
/// knots and multiplicities; validation pieces do not change output topology.
/// Partial/unclamped source axes use exact knot insertion and cropping; edge
/// endpoint certificates and source vertex tolerances still apply. Rational
/// spline face bounds, polynomial degrees above three, internal kinks and
/// holes inside spline outer contours are unsupported. Convex LINE/CIRCLE outer
/// contours also support polygonal and complete circular holes. Circles and
/// polygon vertices are checked against all supporting tangent half-planes,
/// with two tolerances of clearance. Convexity contains entire polygon edges
/// and interiors. Shared polygon/disk checks exclude intersections and nesting.
/// Each circle/outer-edge, polygon-vertex/outer-edge and circle/circle pair
/// consumes one check, alongside polygon segment/containment comparisons.
/// Polygonal or complete circular outer bounds also support convex multi-edge
/// LINE/CIRCLE/ELLIPSE holes, optionally containing supported polynomial
/// quadratic/cubic B_CURVE edges. Restricted spline control hulls and analytic
/// conic extrema provide inflated enclosing rectangles that conservatively
/// certify containment and separation; only original curves and
/// analytic areas are exported. Uncertified envelopes return unsupported-format
/// errors, even for some valid close contours. Each curved edge consumes one
/// envelope check plus its corner check; spline holes additionally charge one
/// check per retained control and all restriction/curve-certificate work.
/// Original and envelope-augmented mixed-boundary checks both share the budget.
/// Planar annular sectors support
/// two concentric circular arcs joined by two radial lines, including major
/// sectors. Single hole-free LINE/CIRCLE contours with minor arcs also support
/// concave corners when chord simplicity, adjacent departure cones and
/// nonadjacent sagitta envelopes certify separation. These checks can reject
/// valid close contours; exported curves and areas remain exact source
/// geometry. Other nonconvex planar curved contours, holes in
/// elliptic/nonconvex curved contours, curved outer contours with multi-edge
/// curved holes, nonconvex curved holes, uncertified envelopes, other
/// non-planar trimming contours, spherical pole cuts, uncertified apex loops,
/// explicit seam fins, horn/apple/lemon tori and single-edge elliptical holes remain
/// unsupported. Paired tolerant edges without a recorded 3D curve can use an
/// exact constant-profile circular SP_CURVE on a recognized spun torus and a
/// quadratic polynomial planar SP_CURVE. Whole-span Bernstein bounds certify
/// radial agreement, monotone azimuth and branch selection. STEP SURFACE_CURVE
/// retains the original planar PCURVE and the affine-mapped rotational PCURVE,
/// with the exact rotational circle as its 3D master. Transmitted positive EDGE
/// tolerances may be honored separately from binding tolerance;
/// ExportStatistics::length_uncertainty reports the output maximum. Complete
/// rings can be vertex-free or retain one source seam vertex. Missing ring
/// intervals use stored trim order only after paired full-turn proof; generic
/// reader inference remains unchanged. Quadratic ring pcurves with one inactive
/// exterior knot on each side and endpoint multiplicity two can be clamped
/// exactly to their natural domain without changing controls or polynomial
/// pieces. Other periodic layouts, rational/partial-domain or other
/// fin-specific mappings remain unsupported. Sheet orientation is propagated
/// across shared edges. Solid export requires one material region with one
/// closed, connected shell, consistent face-side uses and positive signed
/// volume. Foreign geometry, other tolerant fin curves, isolated face loops,
/// compound bodies, general bodies and other curve/surface types return
/// explicit errors. Existing finite membership checks are retained; no global
/// intersection or full CAD-kernel validity certificate is implied.
/// @see Parasolid XT Format Reference (July 2022), 4.3.3-4.3.10 (topology),
/// 5.2.1.1-5.2.1.3 (line/circle/ellipse), 5.2.1.4 (B_curve, p.
/// 38), 5.2.2.1-5.2.2.3 (plane/cylinder/cone), 5.2.2.4 (sphere), 5.2.2.5
/// (torus), 5.2.2.11 (Spun_surf, p. 68), 5.2.8 (senses), 5.2.1.8 (SP_curve, p.
/// 51), 5.3.4.1 (geometry attachment) and 5.3.11 (Edge, p. 98).
/// @param model Borrowed model with stream-local node identities.
/// @param body Explicit BODY identity in this model.
/// @param options Units, tolerances and bounded work/storage settings.
/// @return Complete owned document, or ParseError with source context. No
/// partial document is returned. Calls on independent inputs may run
/// concurrently.
[[nodiscard]] openparasolid::Result<StepDocument> ExportBody(
    const openparasolid::Model& model, openparasolid::NodeId body,
    const ExportOptions& options);

/// @brief Sequentially decodes a complete ordinary transmit/partition then
/// exports.
/// @param data Borrowed extracted binary stream, starting at its prologue.
/// @param body Explicit stream-local BODY identity.
/// @param options Export, parser and model budgets.
/// @param catalog Optional matching reviewed base schema; null uses the
/// built-in subset.
/// @return Complete document or explicit failure. Unapplied deltas and
/// incomplete reports are rejected, preserving the first sequential diagnostic
/// if present.
[[nodiscard]] openparasolid::Result<StepDocument> ExportParasolid(
    std::span<const uint8_t> data, openparasolid::NodeId body,
    const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief Exports an explicitly selected libopenswx extraction payload.
/// Ghost/history payloads and unknown kinds are rejected; no history is
/// guessed.
/// @param payload Borrowed extraction result; its bytes are decoded
/// independently.
/// @param body Explicit BODY identity within this payload.
/// @param options Export and source decoding settings.
/// @param catalog Optional reviewed schema catalog for this payload.
/// @return Complete document or explicit failure; no assembly placement is
/// applied.
[[nodiscard]] openparasolid::Result<StepDocument> ExportPayload(
    const openswx::ParasolidPayload& payload, openparasolid::NodeId body,
    const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief File adapter: inspects a modern SolidWorks container and selects a
/// payload.
/// @param input Source file. Size is bounded by options.parsing.max_input_bytes
/// before invoking libopenswx; its container/extraction checks remain in force.
/// @param payload_index Zero-based index in ExtractParasolidStreams().payloads;
/// no configuration, body, component placement or active payload is guessed.
/// @param body Explicit BODY identity within the selected payload.
/// @param options Export and source decoding settings.
/// @param catalog Optional matching reviewed schema catalog.
/// @return Complete document or explicit failure. Extraction issues are errors.
[[nodiscard]] openparasolid::Result<StepDocument> ExportSolidWorksFile(
    const std::filesystem::path& input, std::size_t payload_index,
    openparasolid::NodeId body, const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief File adapter writing a complete in-memory document, replacing the
/// target. Serialize first so unsupported inputs never open/truncate the output
/// file. An I/O error may leave a partial file and must be handled by the
/// caller.
/// @param document Previously generated complete STEP document.
/// @param output Destination path; parent directories must already exist.
/// @return Written byte count, or kIo on open/write/close failure.
[[nodiscard]] openparasolid::Result<uint64_t> WriteStepFile(
    const StepDocument& document, const std::filesystem::path& output);
}  // namespace openswstep
#endif  // OPENSWSTEP_EXPORT_H_
