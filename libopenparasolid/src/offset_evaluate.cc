// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <unordered_set>
#include <utility>

#include "evaluate_internal.h"
namespace openparasolid::internal {
namespace {
ParseError Error(const char* message,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, 0, message};
}
void Shift(Vector3& point, Vector3 axis, double distance) {
  point = {point.x + axis.x * distance, point.y + axis.y * distance,
           point.z + axis.z * distance};
}
}  // namespace
Result<GeometryPayload> ResolveOffsetSurface(
    const Model& model, NodeId id,
    std::optional<SurfaceParameterSeed> parameters) {
  struct Layer {
    double distance;
    bool reverse;
  };
  std::array<Layer, 64> layers{};
  std::size_t count = 0;
  std::unordered_set<uint32_t> seen;
  const GeometryObject* geometry = nullptr;
  while (true) {
    const auto* node = model.FindNode(id);
    if (!node)
      return Result<GeometryPayload>::Err(Error("Missing offset basis"));
    switch (node->node_type) {
      case 50:
      case 51:
      case 52:
      case 53:
      case 54:
      case 56:
      case 59:
      case 60:
      case 67:
      case 68:
      case 120:
      case 124:
      case 201:
        break;
      default:
        return Result<GeometryPayload>::Err(
            Error("Offset basis is not a surface"));
    }
    if (!node->geometry)
      return Result<GeometryPayload>::Err(
          Error("Untyped offset basis", ErrorCode::kUnsupportedFormat));
    geometry = &model.geometry()[*node->geometry];
    const auto* offset = std::get_if<OffsetSurfaceGeometry>(&geometry->payload);
    if (!offset) break;
    if (!seen.insert(id.value).second)
      return Result<GeometryPayload>::Err(Error("Cyclic offset surface"));
    if (count == layers.size())
      return Result<GeometryPayload>::Err(
          Error("Offset dependency depth limit", ErrorCode::kResourceLimit));
    const auto* basis = model.FindNode(offset->surface);
    if (!basis)
      return Result<GeometryPayload>::Err(Error("Missing offset surface"));
    if (!basis->geometry)
      return Result<GeometryPayload>::Err(
          Error("Untyped offset surface", ErrorCode::kUnsupportedFormat));
    if (basis->node_type == 29 || basis->node_type == 30 ||
        basis->node_type == 31 || basis->node_type == 32 ||
        basis->node_type == 38 || basis->node_type == 133 ||
        basis->node_type == 134 || basis->node_type == 137)
      return Result<GeometryPayload>::Err(
          Error("Offset basis is not a surface"));
    const auto sense = model.geometry()[*basis->geometry].sense;
    if (!sense || (*sense != '+' && *sense != '-'))
      return Result<GeometryPayload>::Err(
          Error("Unknown offset basis sense", ErrorCode::kUnsupportedFormat));
    if (!std::isfinite(offset->distance))
      return Result<GeometryPayload>::Err(Error("Nonfinite offset distance"));
    layers[count++] = {offset->distance, *sense == '-'};
    id = offset->surface;
  }
  if (!std::holds_alternative<PlaneGeometry>(geometry->payload) &&
      !std::holds_alternative<CylinderGeometry>(geometry->payload) &&
      !std::holds_alternative<SphereGeometry>(geometry->payload) &&
      !std::holds_alternative<ConeGeometry>(geometry->payload) &&
      !std::holds_alternative<TorusGeometry>(geometry->payload))
    return Result<GeometryPayload>::Err(
        Error("Unsupported offset basis", ErrorCode::kUnsupportedFormat));
  if (const auto* torus = std::get_if<TorusGeometry>(&geometry->payload))
    if (torus->major_radius <= torus->minor_radius)
      return Result<GeometryPayload>::Err(
          Error("Offset basis must be a regular ring torus",
                ErrorCode::kUnsupportedFormat));
  GeometryPayload payload = geometry->payload;
  const double u = parameters ? parameters->u : 0;
  const double v = parameters ? parameters->v : 0;
  // Validate the underlying parameterisation before modifying its coefficients.
  auto valid = EvaluateSurface(payload, u, v);
  if (!valid.ok()) return Result<GeometryPayload>::Err(valid.error());
  while (count) {
    const auto layer = layers[--count];
    const double d = layer.reverse ? -layer.distance : layer.distance;
    if (auto* plane = std::get_if<PlaneGeometry>(&payload))
      Shift(plane->origin, plane->normal, d);
    else if (auto* cylinder = std::get_if<CylinderGeometry>(&payload))
      cylinder->radius += d;
    else if (auto* sphere = std::get_if<SphereGeometry>(&payload))
      sphere->radius += d;
    else if (auto* cone = std::get_if<ConeGeometry>(&payload)) {
      Shift(cone->origin, cone->axis, -d * cone->sin_half_angle);
      cone->radius += d * cone->cos_half_angle;
    } else if (auto* torus = std::get_if<TorusGeometry>(&payload)) {
      torus->minor_radius += d;
      if (torus->major_radius <= torus->minor_radius ||
          torus->minor_radius <= 0)
        return Result<GeometryPayload>::Err(
            Error("Offset crosses ring-torus singularity",
                  ErrorCode::kUnsupportedFormat));
    } else
      return Result<GeometryPayload>::Err(
          Error("Offset evaluation requires a supported analytic basis",
                ErrorCode::kUnsupportedFormat));
    valid = EvaluateSurface(payload, u, v);
    if (!valid.ok()) return Result<GeometryPayload>::Err(valid.error());
  }
  return Result<GeometryPayload>::Ok(std::move(payload));
}
Result<SurfaceEvaluation> EvaluateOffsetSurface(const Model& model, NodeId id,
                                                double u, double v) {
  auto payload = ResolveOffsetSurface(model, id, SurfaceParameterSeed{u, v});
  if (!payload.ok()) return Result<SurfaceEvaluation>::Err(payload.error());
  return EvaluateSurface(payload.value(), u, v);
}

Result<SurfaceParameters> InvertOffsetSurface(
    const Model& model, NodeId id, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed) {
  auto payload = ResolveOffsetSurface(model, id);
  if (!payload.ok()) return Result<SurfaceParameters>::Err(payload.error());
  auto uv = seed ? InvertSurface(payload.value(), point, tolerance, *seed)
                 : InvertSurface(payload.value(), point, tolerance);
  if (!uv.ok()) return uv;
  auto checked = EvaluateOffsetSurface(model, id, uv.value().u, uv.value().v);
  if (!checked.ok()) {
    auto error = checked.error();
    if (error.code == ErrorCode::kInvalidOptions)
      error.code = ErrorCode::kInvalidRecord;
    return Result<SurfaceParameters>::Err(std::move(error));
  }
  return uv;
}

}  // namespace openparasolid::internal
