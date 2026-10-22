// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
/**
 * @file
 * @brief Owned node graph and typed source-coordinate geometry.
 * @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_attachment
 * "XT 5.3.4.1: Attaching geometry to topology (p. 92)", @ref xt_curves
 * "XT 5.2.1: Curves (p. 34)", @ref xt_surfaces "XT 5.2.2: Surfaces (p. 53)"
 */
#ifndef OPENPARASOLID_MODEL_H_
#define OPENPARASOLID_MODEL_H_

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "openparasolid/schema.h"

namespace openparasolid {
/// @brief Decoded stream-local node index; zero denotes null, not a persistent
/// node-id.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_identifiers
/// "XT 4.3.13: Identifiers (p. 31)"
struct NodeId {
  /// Decoded compact node index; zero is the null reference.
  uint32_t value = 0;
  /// Compares stream-local indices; has no cross-model identity meaning.
  /// @return Ordering of the two numeric stream-local index values.
  auto operator<=>(const NodeId&) const = default;
};
/// @brief Three source-coordinate components; no implicit unit or frame
/// conversion.
///
/// @see @ref xt_fields "XT 2.1.4: Field types (p. 14)"
struct Vector3 {
  /// X component in the source coordinate frame.
  double x = 0;
  /// Y component in the source coordinate frame.
  double y = 0;
  /// Z component in the source coordinate frame.
  double z = 0;
};
/// @brief Typed POINT position.
///
/// @see @ref xt_point "XT 5.2.5: Point (p. 77)"
struct PointGeometry {
  /// Position in source coordinates and source length units.
  Vector3 position;
};
/// @brief Typed LINE origin and natural parameter direction.
///
/// @see @ref xt_line "XT 5.2.1.1: Line (p. 35)"
struct LineGeometry {
  /// Reference origin in source coordinates.
  Vector3 origin;
  /// Stored LINE direction, defining the natural parameter derivative.
  Vector3 direction;
};
/// @brief Typed CIRCLE frame and radius.
///
/// @see @ref xt_circle "XT 5.2.1.2: Circle (p. 36)"
struct CircleGeometry {
  /// Center in source coordinates.
  Vector3 center;
  /// Stored normal direction; evaluators validate its unit length.
  Vector3 normal;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
  /// Source radius at the reference origin, in source length units.
  double radius = 0;
};
/// @brief Typed ELLIPSE frame and major/minor radii.
///
/// @see @ref xt_ellipse "XT 5.2.1.3: Ellipse (p. 37)"
struct EllipseGeometry {
  /// Center in source coordinates.
  Vector3 center;
  /// Stored normal direction; evaluators validate its unit length.
  Vector3 normal;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
  /// Source major radius in source length units.
  double major_radius = 0;
  /// Source minor radius in source length units.
  double minor_radius = 0;
};
/// @brief Typed PLANE frame.
///
/// @see @ref xt_plane "XT 5.2.2.1: Plane (p. 53)"
struct PlaneGeometry {
  /// Reference origin in source coordinates.
  Vector3 origin;
  /// Stored normal direction; evaluators validate its unit length.
  Vector3 normal;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
};
/// @brief Typed CYLINDER frame and radius.
///
/// @see @ref xt_cylinder "XT 5.2.2.2: Cylinder (p. 54)"
struct CylinderGeometry {
  /// Reference origin in source coordinates.
  Vector3 origin;
  /// Stored axial direction; evaluators validate its unit length.
  Vector3 axis;
  /// Source radius at the reference origin, in source length units.
  double radius = 0;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
};
/// @brief Typed CONE frame, reference radius and signed half-angle data.
///
/// @see @ref xt_cone "XT 5.2.2.3: Cone (p. 56)"
struct ConeGeometry {
  /// Reference origin in source coordinates.
  Vector3 origin;
  /// Stored axial direction; evaluators validate its unit length.
  Vector3 axis;
  /// Source radius at the reference origin, in source length units.
  double radius = 0;
  /// Stored sine of the signed cone half-angle.
  double sin_half_angle = 0;
  /// Stored cosine of the cone half-angle.
  double cos_half_angle = 0;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
};
/// @brief Typed SPHERE frame and radius.
///
/// @see @ref xt_sphere "XT 5.2.2.4: Sphere (p. 57)"
struct SphereGeometry {
  /// Center in source coordinates.
  Vector3 center;
  /// Source radius at the reference origin, in source length units.
  double radius = 0;
  /// Stored axial direction; evaluators validate its unit length.
  Vector3 axis;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
};
/// @brief Typed TORUS frame and radii.
///
/// @see @ref xt_torus "XT 5.2.2.5: Torus (p. 58)"
struct TorusGeometry {
  /// Center in source coordinates.
  Vector3 center;
  /// Stored axial direction; evaluators validate its unit length.
  Vector3 axis;
  /// Source major radius in source length units.
  double major_radius = 0;
  /// Source minor radius in source length units.
  double minor_radius = 0;
  /// Stored reference X direction of the source frame.
  Vector3 x_axis;
};
/// @brief One B-spline knot axis, retaining distinct knots and multiplicities.
///
/// @see @ref xt_bcurve "XT 5.2.1.4: B_CURVE (B-spline curve) (p. 38)", @ref
/// xt_bsurface "XT 5.2.2.9: B_surface (p. 63)"
struct SplineAxis {
  /// Polynomial degree of this spline axis.
  uint32_t degree = 0;
  /// Number of control points along this axis.
  uint32_t control_count = 0;
  /// Transmitted periodic flag; does not enable automatic parameter wrapping.
  bool periodic = false;
  /// Transmitted closure flag; not a geometric closure certificate.
  bool closed = false;
  /// Unmodified source knot classification byte.
  uint8_t knot_type = 0;
  /// Distinct knot values in source order; use multiplicities to expand them.
  std::vector<double> knots;
  /// Multiplicity for each distinct knot, in the same order.
  std::vector<uint16_t> multiplicities;
};
/// @brief Owned spline control scalars in transmitted order.
///
/// @see @ref xt_bcurve "XT 5.2.1.4: B_CURVE (B-spline curve) (p. 38)", @ref
/// xt_bsurface "XT 5.2.2.9: B_surface (p. 63)"
struct SplineControlNet {
  /// Source control-data node identity.
  NodeId source;
  /// Number of scalars per control point, including weight when rational.
  uint16_t dimension = 0;
  /// Whether controls contain homogeneous weighted coordinates and weights.
  bool rational = false;
  /// Original order and homogeneous coordinates; weights are not divided out.
  std::vector<double> values;
};
/// @brief Typed B_CURVE with its descriptor and control net.
///
/// @see @ref xt_bcurve "XT 5.2.1.4: B_CURVE (B-spline curve) (p. 38)"
struct NurbsCurveGeometry {
  /// Source B-spline descriptor node identity.
  NodeId descriptor;
  /// Natural spline parameter axis.
  SplineAxis axis;
  /// Owned control net in source order.
  SplineControlNet controls;
  /// Unmodified source spline curve-form classification byte.
  uint8_t curve_form = 0;
};
/// @brief Typed B_SURFACE with two knot axes and a shared control net.
///
/// @see @ref xt_bsurface "XT 5.2.2.9: B_surface (p. 63)"
struct NurbsSurfaceGeometry {
  /// Source B-spline descriptor node identity.
  NodeId descriptor;
  /// Spline knot axis for the U parameter.
  SplineAxis u;
  /// Spline knot axis for the V parameter.
  SplineAxis v;
  /// Owned control net in source order.
  SplineControlNet controls;
  /// Unmodified source spline surface-form classification byte.
  uint8_t surface_form = 0;
};
/// @brief Typed SP_CURVE linking a 2D parameter curve to a supporting surface.
///
/// @see @ref xt_spcurve "XT 5.2.1.8: SP_CURVE (p. 51)"
struct SurfaceCurveGeometry {
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Referenced two-dimensional curve in the surface parameter domain.
  NodeId parameter_curve;
};
/// @brief Typed TRIMMED_CURVE retaining basis parameters and stored endpoint
/// points.
///
/// @see @ref xt_trim "XT 5.2.1.6: Trimmed_curve (p. 49)"
struct TrimmedCurveGeometry {
  /// Referenced untrimmed or further wrapped basis curve.
  NodeId basis_curve;
  /// Stored first trim endpoint in source coordinates.
  Vector3 point_1;
  /// Stored second trim endpoint in source coordinates.
  Vector3 point_2;
  /// Stored first parameter on the basis curve.
  double parameter_1 = 0;
  /// Stored second parameter on the basis curve.
  double parameter_2 = 0;
};
/// @brief Typed OFFSET_SURF referencing its basis and signed distance.
///
/// @see @ref xt_offset "XT 5.2.2.8: Offset_surf (p. 62)", @ref xt_senses
/// "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
struct OffsetSurfaceGeometry {
  /// Referenced supporting surface identity.
  NodeId surface;
  /// Signed source offset distance; basis sense affects evaluation.
  double distance = 0;
};
/// @brief Typed SPUN_SURF with generator curve and axis.
///
/// @see @ref xt_spun "XT 5.2.2.11: Spun_surf (p. 68)"
struct SpunSurfaceGeometry {
  /// Referenced generator curve identity.
  NodeId profile;
  /// Point on the source rotation axis.
  Vector3 base;
  /// Stored axial direction; evaluators validate its unit length.
  Vector3 axis;
  /// Optional stored generator start point; not an evaluation transform.
  std::optional<Vector3> start;
  /// Optional stored generator end point; not an evaluation transform.
  std::optional<Vector3> end;
  /// Optional stored generator start parameter.
  std::optional<double> start_parameter;
  /// Optional stored generator end parameter.
  std::optional<double> end_parameter;
  /// Optional auxiliary source reference direction; retained without applying
  /// it.
  std::optional<Vector3> x_axis;
  /// Internal source metadata, not an evaluation scale factor.
  std::optional<double> scale;
};
/// @brief Typed SWEPT_SURF with section curve and sweep direction.
///
/// @see @ref xt_swept "XT 5.2.2.10: Swept_surf (p. 67)"
struct SweptSurfaceGeometry {
  /// Referenced generator section curve identity.
  NodeId section;
  /// Stored sweep direction; evaluators require unit length.
  Vector3 sweep;
  /// Internal source metadata, not an evaluation scale factor.
  std::optional<double> scale;
};
/// @brief Owned INTERSECTION chart positions and source parameterization
/// metadata.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionChart {
  /// Source chart node identity.
  NodeId source;
  /// Stored natural parameter at the first chart position.
  double base_parameter = 0;
  /// Stored initial positive chart projection scale.
  double base_scale = 0;
  /// Optional transmitted chordal-error estimate; not a solver tolerance.
  std::optional<double> chordal_error;
  /// Optional transmitted angular-error estimate; not a solver tolerance.
  std::optional<double> angular_error;
  /// Optional transmitted parameter-error estimates for the two surfaces.
  std::array<std::optional<double>, 2> parameter_error;
  /// Only positions are transmitted, without tangents or surface parameters.
  std::vector<Vector3> positions;
};
/// @brief Owned INTERSECTION terminator metadata; not an evaluated extension.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionLimit {
  /// Source intersection terminator node identity.
  NodeId source;
  /// Unmodified source terminator classification byte.
  uint8_t type = 0;
  /// Absent in older schemas; retain the raw byte when declared.
  std::optional<uint8_t> term_use;
  /// Transmitted sample positions in source order.
  std::vector<Vector3> positions;
};
/// @brief Typed INTERSECTION with ordered support surfaces, chart and limits.
///
/// @see @ref xt_intersection "XT 5.2.1.5: Intersection (p. 44)"
struct IntersectionCurveGeometry {
  /// Ordered pair of supporting surface node identities.
  std::array<NodeId, 2> surfaces;
  /// Owned source intersection chart.
  IntersectionChart chart;
  /// Stored start terminator; evaluation does not extend the chart to it.
  IntersectionLimit start;
  /// Stored end terminator; evaluation does not extend the chart to it.
  IntersectionLimit end;
  /// Absent in older schemas. Optional UV data remains in the raw graph.
  std::optional<NodeId> intersection_data;
};
/// @brief Supported typed geometry alternatives; other nodes remain in the
/// graph.
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_surfaces "XT 5.2.2:
/// Surfaces (p. 53)"
using GeometryPayload =
    std::variant<PointGeometry, LineGeometry, CircleGeometry, EllipseGeometry,
                 PlaneGeometry, CylinderGeometry, ConeGeometry, SphereGeometry,
                 TorusGeometry, NurbsCurveGeometry, NurbsSurfaceGeometry,
                 SurfaceCurveGeometry, TrimmedCurveGeometry,
                 SpunSurfaceGeometry, SweptSurfaceGeometry,
                 IntersectionCurveGeometry, OffsetSurfaceGeometry>;
/// @brief Typed payload associated with a source node and its unmodified sense.
///
/// @see @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
/// 92)", @ref xt_senses "XT 5.2.8: Curve, surface and lattice senses (p. 82)"
struct GeometryObject {
  /// Identity of the graph node from which this payload was built.
  NodeId source;
  /// Raw sense byte, including '?'; points have no sense field.
  std::optional<uint8_t> sense;
  /// Owned supported typed geometry alternative.
  GeometryPayload payload;
};
/// @brief Resolution and pointer-class verification outcome in the model graph.
///
/// @see @ref xt_pointers "XT 2.1.4.2: Pointer classes (p. 16)", @ref xt_classes
/// "XT 6.2: Node classes (p. 119)"
enum class ModelReferenceStatus {
  /// Null source pointer.
  kNull,
  /// Resolved target with a verified compatible pointer class.
  kResolved,
  /// Resolved target whose pointer class is not reviewed.
  kUnverifiedClass,
  /// Non-null pointer without a model target.
  kMissingTarget,
  /// Resolved target with a known incompatible pointer class.
  kWrongClass,
};
/// @brief One decoded pointer with target lookup and class-verification status.
///
/// @see @ref xt_pointers "XT 2.1.4.2: Pointer classes (p. 16)"
struct ModelReference {
  /// Source field name associated with this reference or finding.
  std::string field;
  /// Zero-based index within the source pointer field.
  uint32_t element = 0;
  /// Decoded stream-local target identity; zero denotes null.
  NodeId target;
  /// Schema pointer-class code; zero does not assert a verified target class.
  uint16_t pointer_class = 0;
  /// Optional index in Model::nodes() after target lookup.
  std::optional<std::size_t> target_node;
  /// Target-resolution and class-verification outcome.
  ModelReferenceStatus status = ModelReferenceStatus::kNull;
};
/// @brief Owned graph node with references and optional typed geometry index.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
struct ModelNode {
  /// Stream-local node index, unique within this model.
  NodeId id;
  /// Numeric schema node-type code (chapter 6.1).
  uint16_t node_type = 0;
  /// Byte offset of the source node in its decoded snapshot.
  uint64_t source_offset = 0;
  /// Decoded source pointer occurrences with resolution outcomes.
  std::vector<ModelReference> references;
  /// Optional zero-based index into Model::geometry().
  std::optional<std::size_t> geometry;
  /// Raw FACE/FIN sense from a scalar character field. Missing or incompatible
  /// fields remain absent; no sense is inferred.
  std::optional<uint8_t> topology_sense;
  /// Raw scalar character REGION.type, including unknown bytes. Missing or
  /// incompatible fields remain absent; material/void status is not inferred.
  std::optional<uint8_t> region_type;
  /// Raw scalar byte BODY.body_type. Unknown values are retained.
  std::optional<uint8_t> body_type;
  /// Transmitted scalar EDGE.tolerance in source length units, when present.
  /// Retained without widening caller binding tolerances or certifying
  /// geometry. Consumers must check finiteness and positivity before using it.
  /// @see @ref xt_edgeschema "XT 5.3.11: Edge (p. 98)"
  std::optional<double> edge_tolerance;
};
/// @brief Library semantic findings recorded while building the model.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
enum class ModelIssueCode {
  /// Missing pointer target.
  kMissingTarget,
  /// Known pointer-class mismatch.
  kWrongClass,
  /// Unreviewed pointer class.
  kUnverifiedClass,
  /// Malformed supported geometry.
  kInvalidGeometry,
  /// Recognized geometry not supported by typed decoding.
  kUnsupportedGeometry,
};
/// @brief One semantic graph or geometry finding with source context.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct ModelIssue {
  /// Library semantic finding category.
  ModelIssueCode code = ModelIssueCode::kInvalidGeometry;
  /// Source node identity associated with this finding.
  NodeId node;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Source field name associated with this reference or finding.
  std::string field;
  /// Human-readable diagnostic explanation.
  std::string message;
};
/// @brief Aggregate graph/geometry findings, including diagnostics omitted by a
/// limit.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct ModelStatistics {
  /// Number of decoded null pointers.
  uint64_t null_references = 0;
  /// Number of resolved and class-verified references.
  uint64_t resolved_references = 0;
  /// Number of resolved references whose pointer classes are unreviewed.
  uint64_t unverified_references = 0;
  /// Number of non-null references without a target node.
  uint64_t missing_references = 0;
  /// Number of references whose target has a known incompatible class.
  uint64_t wrong_class_references = 0;
  /// Number of recognized candidate geometry nodes.
  uint64_t geometry_candidates = 0;
  /// Number of recognized geometry nodes rejected as invalid.
  uint64_t invalid_geometry = 0;
  /// Number of recognized geometry candidates not supported by typed decoding.
  uint64_t unsupported_geometry = 0;
  /// Total semantic findings, including those omitted from retained
  /// diagnostics.
  uint64_t total_issues = 0;
};
/// @brief Per-build work and diagnostic-storage budgets.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
struct ModelOptions {
  /// Upper bound on node occurrences.
  uint64_t max_nodes = 10000;
  /// Upper bound on decoded fields.
  uint64_t max_fields = 65536;
  /// Upper bound on decoded pointer references.
  uint64_t max_references = 1000000;
  /// Bounds decoded input values and copied geometry scalars separately.
  uint64_t max_values = 1000000;
  /// Upper bound on retained diagnostics.
  uint64_t max_diagnostics = 1000;
};
namespace internal {
class ModelAssembler;
}
/// @brief Owned graph and typed geometry; indices survive copies and moves.
///
/// Owns all data. Node/geometry indices remain valid across copies and moves.
///
/// @see @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
class Model {
 public:
  /// @brief Returns the owned node array in decoded order.
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
  ///
  /// @return Borrowed node array, valid until model assignment, move or
  /// destruction.
  [[nodiscard]] const std::vector<ModelNode>& nodes() const noexcept {
    return nodes_;
  }
  /// @brief Returns the owned typed geometry array referenced by
  /// ModelNode::geometry.
  ///
  /// @see @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
  /// 92)"
  ///
  /// @return Borrowed geometry array, valid until model assignment, move or
  /// destruction.
  [[nodiscard]] const std::vector<GeometryObject>& geometry() const noexcept {
    return geometry_;
  }
  /// @brief Returns retained semantic diagnostics, possibly limited by
  /// max_diagnostics.
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
  ///
  /// @return Borrowed retained diagnostics, valid until model assignment, move
  /// or destruction.
  [[nodiscard]] const std::vector<ModelIssue>& issues() const noexcept {
    return issues_;
  }
  /// @brief Returns all finding counts, including omitted diagnostic entries.
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
  ///
  /// @return Borrowed aggregate counts, valid until model assignment, move or
  /// destruction.
  [[nodiscard]] const ModelStatistics& statistics() const noexcept {
    return statistics_;
  }
  /// @brief Reports whether the diagnostic budget omitted any findings.
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
  ///
  /// @return True when the retained issue count is less than the aggregate
  /// total.
  [[nodiscard]] bool issues_truncated() const noexcept {
    return issues_.size() < statistics_.total_issues;
  }
  /// @brief Reports whether every non-null reference was resolved and
  /// class-verified.
  ///
  /// @see @ref xt_pointers "XT 2.1.4.2: Pointer classes (p. 16)"
  ///
  /// @return True when no missing, wrong-class or unverified references remain.
  [[nodiscard]] bool references_verified() const noexcept;
  /// @brief Reports whether all recognized geometry candidates were typed
  /// successfully.
  ///
  /// @see @ref xt_attachment "XT 5.3.4.1: Attaching geometry to topology (p.
  /// 92)"
  ///
  /// @return True when typed geometry count equals the recognized candidate
  /// count.
  [[nodiscard]] bool typed_geometry_complete() const noexcept;
  /// Non-owning result, valid for this Model's lifetime (until
  /// assignment/move).
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)"
  ///
  /// @param id Stream-local node index to look up; zero has no node.
  /// @return Borrowed matching node, or nullptr if absent; invalidated by model
  /// assignment/move.
  [[nodiscard]] const ModelNode* FindNode(NodeId id) const noexcept;

 private:
  friend class internal::ModelAssembler;
  std::vector<ModelNode> nodes_;
  std::vector<GeometryObject> geometry_;
  std::vector<ModelIssue> issues_;
  std::unordered_map<uint32_t, std::size_t> positions_;
  ModelStatistics statistics_;
};
/// @brief Builds an owned graph from a complete decoded logical snapshot.
///
/// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_attachment
/// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
class ModelBuilder {
 public:
  /// Requires a complete partition/transmit report. Deltas must first be
  /// applied to a base state. Structural report errors return Err; semantic
  /// problems remain explicit model issues and never produce partial geometry
  /// objects.
  ///
  /// @see @ref xt_schema "XT 2.1.1: Schema (p. 12)", @ref xt_attachment
  /// "XT 5.3.4.1: Attaching geometry to topology (p. 92)"
  ///
  /// @param report Complete ordinary transmit/partition snapshot from
  /// sequential decoding.
  /// @param options Per-call settings and resource limits; see the associated
  /// options type.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<Model> Build(const SchemaParseReport& report,
                                           const ModelOptions& options = {});
};
/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param status Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char* ToString(ModelReferenceStatus status) noexcept;
/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param code Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char* ToString(ModelIssueCode code) noexcept;
/// @brief Returns the static XT-style name of a supported typed payload.
///
/// @see @ref xt_curves "XT 5.2.1: Curves (p. 34)", @ref xt_surfaces "XT 5.2.2:
/// Surfaces (p. 53)"
/// @param geometry Typed payload whose diagnostic name is requested.
/// @return Static geometry name, valid for the program lifetime.
[[nodiscard]] const char* GeometryName(
    const GeometryPayload& geometry) noexcept;
}  // namespace openparasolid

#endif  // OPENPARASOLID_MODEL_H_
