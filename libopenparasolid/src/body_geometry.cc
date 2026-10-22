// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <cmath>
#include <string>
#include <utility>

#include "evaluate_internal.h"
#include "openparasolid/brep.h"

namespace openparasolid {
namespace {
Result<BoundBodyVertex> BindVertex(const Model& model,
                                   const VertexIncidence& vertex) {
  const auto* node = model.FindNode(vertex.point);
  if (!node || node->node_type != 29)
    return Result<BoundBodyVertex>::Err(
        {ErrorCode::kInvalidRecord, 0, "Missing body vertex point"});
  if (!node->geometry)
    return Result<BoundBodyVertex>::Err(
        {ErrorCode::kUnsupportedFormat, node->source_offset,
         "Body vertex point has no typed payload"});
  const auto* point =
      std::get_if<PointGeometry>(&model.geometry()[*node->geometry].payload);
  if (!point || !std::isfinite(point->position.x) ||
      !std::isfinite(point->position.y) || !std::isfinite(point->position.z))
    return Result<BoundBodyVertex>::Err({ErrorCode::kInvalidRecord,
                                         node->source_offset,
                                         "Invalid body vertex point geometry"});
  return Result<BoundBodyVertex>::Ok(
      {vertex.vertex, vertex.point, point->position});
}
}  // namespace
Result<BodyGeometryBinding> BindBodyGeometryWithOverrides(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> fin_overrides,
    std::span<const WireParameterInterval> wire_overrides, double tolerance,
    const BodyGeometryBindingOptions& options) {
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_faces ||
      !options.max_fins || !options.max_wire_edges || !options.max_vertices ||
      !options.face.max_loops || !options.face.max_fins)
    return Result<BodyGeometryBinding>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Invalid body geometry binding options"});
  if (options.intersection)
    if (auto error =
            internal::ValidateIntersectionOptions(*options.intersection))
      return Result<BodyGeometryBinding>::Err(*error);
  if (fin_overrides.size() > options.max_fins ||
      wire_overrides.size() > options.max_wire_edges)
    return Result<BodyGeometryBinding>::Err(
        {ErrorCode::kResourceLimit, 0, "Body geometry interval limit"});
  BodyFaceBindingOptions face_options;
  face_options.topology = options.topology;
  face_options.face = options.face;
  face_options.intersection = options.intersection;
  face_options.max_faces = options.max_faces;
  face_options.max_fins = options.max_fins;
  auto faces = BindBodyFacesWithOverrides(model, body, fin_overrides, tolerance,
                                          face_options);
  if (!faces.ok()) return Result<BodyGeometryBinding>::Err(faces.error());
  if (faces.value().topology.vertices.size() > options.max_vertices)
    return Result<BodyGeometryBinding>::Err(
        {ErrorCode::kResourceLimit, 0, "Body geometry vertex limit"});
  BodyWireBindingOptions wire_options;
  wire_options.topology = options.topology;
  wire_options.intersection = options.intersection;
  wire_options.max_edges = options.max_wire_edges;
  auto wires = BindBodyWiresWithOverrides(model, body, wire_overrides,
                                          tolerance, wire_options);
  if (!wires.ok()) return Result<BodyGeometryBinding>::Err(wires.error());
  std::vector<BoundBodyVertex> vertices;
  vertices.reserve(faces.value().topology.vertices.size());
  for (const auto& vertex : faces.value().topology.vertices) {
    auto binding = BindVertex(model, vertex);
    if (!binding.ok()) {
      auto error = binding.error();
      error.message = "Body " + std::to_string(body.value) + ", vertex " +
                      std::to_string(vertex.vertex.value) + ": " +
                      error.message;
      return Result<BodyGeometryBinding>::Err(std::move(error));
    }
    vertices.push_back(std::move(binding).value());
  }
  auto face_binding = std::move(faces).value();
  auto wire_binding = std::move(wires).value();
  return Result<BodyGeometryBinding>::Ok(
      {std::move(face_binding.topology), std::move(face_binding.faces),
       std::move(wire_binding.wires), std::move(vertices)});
}
Result<BodyGeometryBinding> BindBodyGeometry(
    const Model& model, NodeId body, double tolerance,
    const BodyGeometryBindingOptions& options) {
  return BindBodyGeometryWithOverrides(model, body, {}, {}, tolerance, options);
}
}  // namespace openparasolid
