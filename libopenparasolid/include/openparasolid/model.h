// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_MODEL_H_
#define OPENPARASOLID_MODEL_H_

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
struct NodeId {
  uint32_t value = 0;
  auto operator<=>(const NodeId&) const = default;
};
struct Vector3 {
  double x = 0;
  double y = 0;
  double z = 0;
};
struct PointGeometry {
  Vector3 position;
};
struct LineGeometry {
  Vector3 origin;
  Vector3 direction;
};
struct CircleGeometry {
  Vector3 center;
  Vector3 normal;
  Vector3 x_axis;
  double radius = 0;
};
struct EllipseGeometry {
  Vector3 center;
  Vector3 normal;
  Vector3 x_axis;
  double major_radius = 0;
  double minor_radius = 0;
};
struct PlaneGeometry {
  Vector3 origin;
  Vector3 normal;
  Vector3 x_axis;
};
struct CylinderGeometry {
  Vector3 origin;
  Vector3 axis;
  double radius = 0;
  Vector3 x_axis;
};
struct ConeGeometry {
  Vector3 origin;
  Vector3 axis;
  double radius = 0;
  double sin_half_angle = 0;
  double cos_half_angle = 0;
  Vector3 x_axis;
};
struct SphereGeometry {
  Vector3 center;
  double radius = 0;
  Vector3 axis;
  Vector3 x_axis;
};
struct TorusGeometry {
  Vector3 center;
  Vector3 axis;
  double major_radius = 0;
  double minor_radius = 0;
  Vector3 x_axis;
};
struct SplineAxis {
  uint32_t degree = 0;
  uint32_t control_count = 0;
  bool periodic = false;
  bool closed = false;
  uint8_t knot_type = 0;
  std::vector<double> knots;
  std::vector<uint16_t> multiplicities;
};
struct SplineControlNet {
  NodeId source;
  uint16_t dimension = 0;
  bool rational = false;
  /// Original order and homogeneous coordinates; weights are not divided out.
  std::vector<double> values;
};
struct NurbsCurveGeometry {
  NodeId descriptor;
  SplineAxis axis;
  SplineControlNet controls;
  uint8_t curve_form = 0;
};
struct NurbsSurfaceGeometry {
  NodeId descriptor;
  SplineAxis u;
  SplineAxis v;
  SplineControlNet controls;
  uint8_t surface_form = 0;
};
struct SurfaceCurveGeometry {
  NodeId surface;
  NodeId parameter_curve;
};
using GeometryPayload =
    std::variant<PointGeometry, LineGeometry, CircleGeometry, EllipseGeometry,
                 PlaneGeometry, CylinderGeometry, ConeGeometry, SphereGeometry,
                 TorusGeometry, NurbsCurveGeometry, NurbsSurfaceGeometry,
                 SurfaceCurveGeometry>;
struct GeometryObject {
  NodeId source;
  /// Raw sense byte, including '?'; points have no sense field.
  std::optional<uint8_t> sense;
  GeometryPayload payload;
};
enum class ModelReferenceStatus {
  kNull,
  kResolved,
  kUnverifiedClass,
  kMissingTarget,
  kWrongClass
};
struct ModelReference {
  std::string field;
  uint32_t element = 0;
  NodeId target;
  uint16_t pointer_class = 0;
  std::optional<std::size_t> target_node;
  ModelReferenceStatus status = ModelReferenceStatus::kNull;
};
struct ModelNode {
  NodeId id;
  uint16_t node_type = 0;
  uint64_t source_offset = 0;
  std::vector<ModelReference> references;
  std::optional<std::size_t> geometry;
};
enum class ModelIssueCode {
  kMissingTarget,
  kWrongClass,
  kUnverifiedClass,
  kInvalidGeometry,
  kUnsupportedGeometry
};
struct ModelIssue {
  ModelIssueCode code = ModelIssueCode::kInvalidGeometry;
  NodeId node;
  uint64_t offset = 0;
  std::string field;
  std::string message;
};
struct ModelStatistics {
  uint64_t null_references = 0;
  uint64_t resolved_references = 0;
  uint64_t unverified_references = 0;
  uint64_t missing_references = 0;
  uint64_t wrong_class_references = 0;
  uint64_t geometry_candidates = 0;
  uint64_t invalid_geometry = 0;
  uint64_t unsupported_geometry = 0;
  uint64_t total_issues = 0;
};
struct ModelOptions {
  uint64_t max_nodes = 10000;
  uint64_t max_fields = 65536;
  uint64_t max_references = 1000000;
  /// Bounds decoded input values and copied geometry scalars separately.
  uint64_t max_values = 1000000;
  uint64_t max_diagnostics = 1000;
};
namespace internal {
class ModelAssembler;
}
/// Owns all data. Node/geometry indices remain valid across copies and moves.
class Model {
 public:
  [[nodiscard]] const std::vector<ModelNode>& nodes() const noexcept {
    return nodes_;
  }
  [[nodiscard]] const std::vector<GeometryObject>& geometry() const noexcept {
    return geometry_;
  }
  [[nodiscard]] const std::vector<ModelIssue>& issues() const noexcept {
    return issues_;
  }
  [[nodiscard]] const ModelStatistics& statistics() const noexcept {
    return statistics_;
  }
  [[nodiscard]] bool issues_truncated() const noexcept {
    return issues_.size() < statistics_.total_issues;
  }
  [[nodiscard]] bool references_verified() const noexcept;
  [[nodiscard]] bool typed_geometry_complete() const noexcept;
  /// Non-owning result, valid for this Model's lifetime (until
  /// assignment/move).
  [[nodiscard]] const ModelNode* FindNode(NodeId id) const noexcept;

 private:
  friend class internal::ModelAssembler;
  std::vector<ModelNode> nodes_;
  std::vector<GeometryObject> geometry_;
  std::vector<ModelIssue> issues_;
  std::unordered_map<uint32_t, std::size_t> positions_;
  ModelStatistics statistics_;
};
class ModelBuilder {
 public:
  /// Requires a complete partition/transmit report. Deltas must first be
  /// applied to a base state. Structural report errors return Err; semantic
  /// problems remain explicit model issues and never produce partial geometry
  /// objects.
  [[nodiscard]] static Result<Model> Build(const SchemaParseReport& report,
                                           const ModelOptions& options = {});
};
[[nodiscard]] const char* ToString(ModelReferenceStatus status) noexcept;
[[nodiscard]] const char* ToString(ModelIssueCode code) noexcept;
[[nodiscard]] const char* GeometryName(
    const GeometryPayload& geometry) noexcept;
}  // namespace openparasolid

#endif  // OPENPARASOLID_MODEL_H_
