// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <limits>

#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
// Non-owning results remain valid for the duration of the const Model call.
Result<const GeometryObject*> Geometry(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  if (!node) {
    return Result<const GeometryObject*>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Geometry target is absent from this model"});
  }
  if (!node->geometry) {
    return Result<const GeometryObject*>::Err(
        {ErrorCode::kUnsupportedFormat, node->source_offset,
         "Geometry target has no typed payload"});
  }
  return Result<const GeometryObject*>::Ok(&model.geometry()[*node->geometry]);
}

bool SurfaceType(uint16_t type) {
  switch (type) {
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
      return true;
    default:
      return false;
  }
}

Result<CurveEvaluation> Compose(const GeometryPayload& parameter_curve,
                                const GeometryPayload& surface, double t) {
  const auto* nurbs = std::get_if<NurbsCurveGeometry>(&parameter_curve);
  if (!nurbs ||
      nurbs->controls.dimension != (nurbs->controls.rational ? 3 : 2)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface curve requires a 2D NURBS parameter curve"});
  }
  const auto uv = EvaluateCurve(parameter_curve, t);
  if (!uv.ok()) return Result<CurveEvaluation>::Err(uv.error());
  const auto sampled =
      EvaluateSurface(surface, uv.value().position.x, uv.value().position.y);
  if (!sampled.ok()) return Result<CurveEvaluation>::Err(sampled.error());
  const auto& du = sampled.value().u_derivative;
  const auto& dv = sampled.value().v_derivative;
  const std::array<double, 3> u_components{du.x, du.y, du.z};
  const std::array<double, 3> v_components{dv.x, dv.y, dv.z};
  std::array<double, 3> derivative{};
  for (std::size_t i = 0; i < derivative.size(); ++i) {
    const long double value = static_cast<long double>(u_components[i]) *
                                  uv.value().first_derivative.x +
                              static_cast<long double>(v_components[i]) *
                                  uv.value().first_derivative.y;
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<double>::max()) {
      return Result<CurveEvaluation>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Surface curve derivative is not representable"});
    }
    derivative[i] = static_cast<double>(value);
  }
  return Result<CurveEvaluation>::Ok(
      {sampled.value().position,
       {derivative[0], derivative[1], derivative[2]}});
}
}  // namespace

Result<CurveEvaluation> EvaluateCurve(const Model& model, NodeId curve,
                                      double parameter) {
  if (!std::isfinite(parameter)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0, "Curve parameter must be finite"});
  }
  const auto geometry = Geometry(model, curve);
  if (!geometry.ok()) return Result<CurveEvaluation>::Err(geometry.error());
  const auto* sp =
      std::get_if<SurfaceCurveGeometry>(&geometry.value()->payload);
  if (!sp) return EvaluateCurve(geometry.value()->payload, parameter);
  const auto* uv_node = model.FindNode(sp->parameter_curve);
  const auto* surface_node = model.FindNode(sp->surface);
  if (!uv_node || !surface_node || uv_node->node_type != 134 ||
      !SurfaceType(surface_node->node_type)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface curve has missing or incompatible targets"});
  }
  const auto uv = Geometry(model, sp->parameter_curve);
  if (!uv.ok()) return Result<CurveEvaluation>::Err(uv.error());
  const auto surface = Geometry(model, sp->surface);
  if (!surface.ok()) return Result<CurveEvaluation>::Err(surface.error());
  return Compose(uv.value()->payload, surface.value()->payload, parameter);
}
}  // namespace openparasolid
