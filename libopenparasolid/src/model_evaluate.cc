// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include "evaluate_internal.h"
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

bool CurveType(uint16_t type) {
  switch (type) {
    case 30:
    case 31:
    case 32:
    case 38:
    case 130:
    case 133:
    case 134:
    case 137:
    case 200:
      return true;
    default:
      return false;
  }
}

Result<const GeometryObject*> ResolveTrimmed(const Model& model,
                                             const GeometryObject* geometry,
                                             double parameter) {
  constexpr std::size_t kMaxTrimDepth = 64;
  std::array<NodeId, kMaxTrimDepth> visited{};
  std::size_t depth = 0;
  while (const auto* trim =
             std::get_if<TrimmedCurveGeometry>(&geometry->payload)) {
    for (std::size_t i = 0; i < depth; ++i) {
      if (visited[i] == geometry->source) {
        return Result<const GeometryObject*>::Err(
            {ErrorCode::kInvalidRecord, 0, "Cyclic trimmed curve dependency"});
      }
    }
    if (depth == visited.size()) {
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kResourceLimit, 0,
           "Trimmed curve dependency depth exceeds 64"});
    }
    visited[depth++] = geometry->source;
    const auto* basis_node = model.FindNode(trim->basis_curve);
    if (!basis_node || !CurveType(basis_node->node_type)) {
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Trimmed curve basis is missing or incompatible"});
    }
    const auto basis = Geometry(model, trim->basis_curve);
    if (!basis.ok()) return basis;
    const auto sense = basis.value()->sense;
    if (!sense || (*sense != '+' && *sense != '-')) {
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kUnsupportedFormat, 0,
           "Trimmed curve basis sense is unknown"});
    }
    const bool increasing = trim->parameter_2 > trim->parameter_1;
    if (increasing != (*sense == '+')) {
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kInvalidRecord, 0,
           "Trim parameters disagree with basis sense"});
    }
    const double low = increasing ? trim->parameter_1 : trim->parameter_2;
    const double high = increasing ? trim->parameter_2 : trim->parameter_1;
    if (parameter < low || parameter > high) {
      return Result<const GeometryObject*>::Err(
          {ErrorCode::kInvalidOptions, 0,
           "Curve parameter is outside a trim interval"});
    }
    geometry = basis.value();
  }
  return Result<const GeometryObject*>::Ok(geometry);
}

Result<CurveEvaluation> SampleSection(const Model& model, NodeId profile_id,
                                      double u) {
  const auto* node = model.FindNode(profile_id);
  if (!node || (node->node_type != 30 && node->node_type != 31 &&
                node->node_type != 32 && node->node_type != 134)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Profile surface requires an analytic or B-curve profile"});
  }
  const auto profile = Geometry(model, profile_id);
  if (!profile.ok()) return Result<CurveEvaluation>::Err(profile.error());
  const auto* nurbs =
      std::get_if<NurbsCurveGeometry>(&profile.value()->payload);
  if (nurbs &&
      nurbs->controls.dimension != (nurbs->controls.rational ? 4 : 3)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface generator must be three-dimensional"});
  }
  const auto sampled = EvaluateCurve(profile.value()->payload, u);
  if (!sampled.ok()) return Result<CurveEvaluation>::Err(sampled.error());
  return sampled;
}

Result<CurveEvaluation> Compose(const Model& model,
                                const GeometryPayload& parameter_curve,
                                NodeId surface, double t) {
  const auto* nurbs = std::get_if<NurbsCurveGeometry>(&parameter_curve);
  if (!nurbs ||
      nurbs->controls.dimension != (nurbs->controls.rational ? 3 : 2)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface curve requires a 2D NURBS parameter curve"});
  }
  const auto uv = EvaluateCurve(parameter_curve, t);
  if (!uv.ok()) return Result<CurveEvaluation>::Err(uv.error());
  const auto sampled = EvaluateSurface(model, surface, uv.value().position.x,
                                       uv.value().position.y);
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
Result<CurveEvaluation> EvaluateModelCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions* intersection_options) {
  if (!std::isfinite(parameter)) {
    return Result<CurveEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0, "Curve parameter must be finite"});
  }
  const auto root = Geometry(model, curve);
  if (!root.ok()) return Result<CurveEvaluation>::Err(root.error());
  const auto geometry = ResolveTrimmed(model, root.value(), parameter);
  if (!geometry.ok()) return Result<CurveEvaluation>::Err(geometry.error());
  if (intersection_options &&
      std::holds_alternative<IntersectionCurveGeometry>(
          geometry.value()->payload))
    return EvaluateIntersectionCurve(model, geometry.value()->source, parameter,
                                      *intersection_options);
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
  return Compose(model, uv.value()->payload, sp->surface, parameter);
}
}  // namespace

Result<CurveEvaluation> EvaluateCurve(const Model& model, NodeId curve,
                                      double parameter) {
  return EvaluateModelCurve(model, curve, parameter, nullptr);
}

Result<CurveEvaluation> EvaluateCurve(
    const Model& model, NodeId curve, double parameter,
    const IntersectionEvaluationOptions& options) {
  if (auto error = internal::ValidateIntersectionOptions(options))
    return Result<CurveEvaluation>::Err(std::move(*error));
  return EvaluateModelCurve(model, curve, parameter, &options);
}

Result<SurfaceEvaluation> EvaluateSurface(const Model& model, NodeId surface,
                                          double u, double v) {
  if (!std::isfinite(u) || !std::isfinite(v)) {
    return Result<SurfaceEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0, "Surface parameters must be finite"});
  }
  const auto geometry = Geometry(model, surface);
  if (!geometry.ok()) return Result<SurfaceEvaluation>::Err(geometry.error());
  if (std::holds_alternative<OffsetSurfaceGeometry>(geometry.value()->payload))
    return internal::EvaluateOffsetSurface(model, surface, u, v);
  if (const auto* swept =
          std::get_if<SweptSurfaceGeometry>(&geometry.value()->payload)) {
    const auto sampled = SampleSection(model, swept->section, u);
    if (!sampled.ok()) return Result<SurfaceEvaluation>::Err(sampled.error());
    return internal::EvaluateSweptSection(*swept, sampled.value(), v);
  }
  const auto* spun =
      std::get_if<SpunSurfaceGeometry>(&geometry.value()->payload);
  if (!spun) return EvaluateSurface(geometry.value()->payload, u, v);
  if ((spun->start_parameter && u < *spun->start_parameter) ||
      (spun->end_parameter && u > *spun->end_parameter)) {
    return Result<SurfaceEvaluation>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Spun surface parameter is outside its profile bounds"});
  }
  const auto sampled = SampleSection(model, spun->profile, u);
  if (!sampled.ok()) return Result<SurfaceEvaluation>::Err(sampled.error());
  return internal::EvaluateSpunProfile(*spun, sampled.value(), v);
}
Result<Vector3> EvaluateSurfaceNormal(const Model& model, NodeId surface,
                                      double u, double v) {
  const auto sample = EvaluateSurface(model, surface, u, v);
  if (!sample.ok()) return Result<Vector3>::Err(sample.error());
  const auto geometry = Geometry(model, surface);
  if (!geometry.ok()) return Result<Vector3>::Err(geometry.error());
  const auto sense = geometry.value()->sense;
  if (!sense || (*sense != '+' && *sense != '-')) {
    return Result<Vector3>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "Surface normal requires known surface sense"});
  }
  // Rounded trigonometric endpoint values can leave tiny nonzero partials
  // at known singular parameter boundaries. Do not normalize that residue.
  const auto* sphere =
      std::get_if<SphereGeometry>(&geometry.value()->payload);
  const auto* torus = std::get_if<TorusGeometry>(&geometry.value()->payload);
  if ((sphere && std::abs(v) == std::numbers::pi / 2) ||
      (torus && torus->major_radius <= torus->minor_radius &&
       std::abs(v) == std::acos(-torus->major_radius / torus->minor_radius))) {
    return Result<Vector3>::Err(
        {ErrorCode::kInvalidRecord, 0,
         "Surface normal is undefined at a singular parameter boundary"});
  }
  return internal::UnitSurfaceNormal(sample.value(), *sense == '-');
}
Result<SurfaceParameters> InvertSurface(const Model& model, NodeId surface,
                                        Vector3 point, double tolerance) {
  const auto geometry = Geometry(model, surface);
  if (!geometry.ok()) return Result<SurfaceParameters>::Err(geometry.error());
  if (std::holds_alternative<OffsetSurfaceGeometry>(
          geometry.value()->payload)) {
    return internal::InvertOffsetSurface(model, surface, point, tolerance);
  }
  if (std::holds_alternative<SpunSurfaceGeometry>(geometry.value()->payload))
    return internal::InvertSpunSurface(model, surface, point, tolerance);
  return InvertSurface(geometry.value()->payload, point, tolerance);
}
Result<SurfaceParameters> InvertSurface(const Model& model, NodeId surface,
                                        Vector3 point, double tolerance,
                                        SurfaceParameterSeed seed) {
  const auto geometry = Geometry(model, surface);
  if (!geometry.ok()) return Result<SurfaceParameters>::Err(geometry.error());
  if (std::holds_alternative<OffsetSurfaceGeometry>(
          geometry.value()->payload)) {
    return internal::InvertOffsetSurface(model, surface, point, tolerance,
                                         seed);
  }
  if (std::holds_alternative<SpunSurfaceGeometry>(geometry.value()->payload))
    return internal::InvertSpunSurface(model, surface, point, tolerance, seed);
  return InvertSurface(geometry.value()->payload, point, tolerance, seed);
}
}  // namespace openparasolid
