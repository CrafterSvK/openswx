// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "brep_internal.h"
#include "evaluate_internal.h"
#include "openparasolid/brep.h"

namespace openparasolid {
namespace {
ParseError Error(const char* message,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, 0, message};
}
Result<Vector3> VertexPoint(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  if (!node || node->node_type != 18)
    return Result<Vector3>::Err(Error("Missing wire vertex"));
  const ModelReference* reference = nullptr;
  for (const auto& r : node->references) {
    if (r.field != "point") continue;
    if (reference || r.element || r.status != ModelReferenceStatus::kResolved)
      return Result<Vector3>::Err(Error("Invalid wire vertex point role"));
    reference = &r;
  }
  const auto* point = reference ? model.FindNode(reference->target) : nullptr;
  if (!point || point->node_type != 29)
    return Result<Vector3>::Err(Error("Missing wire vertex point"));
  if (!point->geometry)
    return Result<Vector3>::Err(
        Error("Untyped wire vertex point", ErrorCode::kUnsupportedFormat));
  const auto* geometry =
      std::get_if<PointGeometry>(&model.geometry()[*point->geometry].payload);
  if (!geometry)
    return Result<Vector3>::Err(Error("Wire vertex target is not a point"));
  return Result<Vector3>::Ok(geometry->position);
}
long double Distance(Vector3 a, Vector3 b) {
  return std::hypot(static_cast<long double>(a.x) - b.x,
                    static_cast<long double>(a.y) - b.y,
                    static_cast<long double>(a.z) - b.z);
}
Result<BoundWireGeometry> BindWire(
    const Model& model, const WireEdgeUse& use,
    const WireParameterInterval& interval, double tolerance,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  const auto* curve = model.FindNode(use.curve);
  if (!curve || !curve->geometry)
    return Result<BoundWireGeometry>::Err(
        Error("Wire requires a typed curve", ErrorCode::kUnsupportedFormat));
  const auto& geometry = model.geometry()[*curve->geometry];
  bool increasing;
  if (const auto* trim = std::get_if<TrimmedCurveGeometry>(&geometry.payload)) {
    increasing = trim->parameter_2 > trim->parameter_1;
  } else {
    if (!geometry.sense || (*geometry.sense != '+' && *geometry.sense != '-'))
      return Result<BoundWireGeometry>::Err(
          Error("Unknown wire curve sense", ErrorCode::kUnsupportedFormat));
    increasing = *geometry.sense == '+';
  }
  if ((interval.end > interval.start) != increasing)
    return Result<BoundWireGeometry>::Err(
        Error("Wire interval contradicts edge/curve sense"));
  BoundWireGeometry binding{use, interval.start, interval.end, intersection};
  std::optional<Vector3> start;
  for (double fraction : {0., .5, 1.}) {
    const auto sample = EvaluateBoundWire(model, binding, fraction);
    if (!sample.ok()) return Result<BoundWireGeometry>::Err(sample.error());
    if (fraction == 0) start = sample.value().position;
    const NodeId vertex = fraction == 0   ? use.start_vertex
                          : fraction == 1 ? use.end_vertex
                                          : NodeId{};
    if (vertex.value) {
      const auto point = VertexPoint(model, vertex);
      if (!point.ok()) return Result<BoundWireGeometry>::Err(point.error());
      if (Distance(sample.value().position, point.value()) > tolerance)
        return Result<BoundWireGeometry>::Err(
            Error("Wire interval endpoint disagrees with vertex"));
    }
    if (fraction == 1 && !use.start_vertex.value && !use.end_vertex.value &&
        Distance(*start, sample.value().position) > tolerance)
      return Result<BoundWireGeometry>::Err(
          Error("Vertex-free wire interval is not closed within tolerance"));
  }
  return Result<BoundWireGeometry>::Ok(std::move(binding));
}
}  // namespace
Result<CurveEvaluation> EvaluateBoundWire(const Model& model,
                                          const BoundWireGeometry& wire,
                                          double fraction) {
  if (!std::isfinite(fraction) || fraction < 0 || fraction > 1 ||
      !std::isfinite(wire.start) || !std::isfinite(wire.end) ||
      wire.start == wire.end)
    return Result<CurveEvaluation>::Err(
        Error("Invalid wire interval or fraction", ErrorCode::kInvalidOptions));
  if (wire.intersection)
    if (auto error = internal::ValidateIntersectionOptions(*wire.intersection))
      return Result<CurveEvaluation>::Err(*error);
  const double t = std::lerp(wire.start, wire.end, fraction);
  auto result = wire.intersection ? EvaluateCurve(model, wire.use.curve, t,
                                                  *wire.intersection)
                                  : EvaluateCurve(model, wire.use.curve, t);
  if (!result.ok()) return result;
  auto evaluated = std::move(result).value();
  const auto& derivative = evaluated.first_derivative;
  const long double span = static_cast<long double>(wire.end) - wire.start;
  const std::array<long double, 3> scaled{
      derivative.x * span, derivative.y * span, derivative.z * span};
  for (const auto value : scaled)
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<double>::max())
      return Result<CurveEvaluation>::Err(
          Error("Nonfinite oriented wire tangent"));
  evaluated.first_derivative = {static_cast<double>(scaled[0]),
                                static_cast<double>(scaled[1]),
                                static_cast<double>(scaled[2])};
  return Result<CurveEvaluation>::Ok(evaluated);
}
namespace {
Result<BodyWireBinding> Bind(
    const Model& model, NodeId body,
    std::optional<std::span<const WireParameterInterval>> intervals,
    double tolerance, const BodyWireBindingOptions& options) {
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_edges)
    return Result<BodyWireBinding>::Err(
        Error("Invalid body wire options", ErrorCode::kInvalidOptions));
  if (options.intersection)
    if (auto error =
            internal::ValidateIntersectionOptions(*options.intersection))
      return Result<BodyWireBinding>::Err(*error);
  if (intervals && intervals->size() > options.max_edges)
    return Result<BodyWireBinding>::Err(
        Error("Body wire interval limit", ErrorCode::kResourceLimit));
  std::unordered_map<uint32_t, WireParameterInterval> parameters;
  if (intervals)
    for (const auto& p : *intervals)
      if (!p.edge.value || !std::isfinite(p.start) || !std::isfinite(p.end) ||
          p.start == p.end || !parameters.emplace(p.edge.value, p).second)
        return Result<BodyWireBinding>::Err(Error(
            "Duplicate or invalid wire interval", ErrorCode::kInvalidOptions));
  auto topology = ValidateBodyTopology(model, body, options.topology);
  if (!topology.ok()) return Result<BodyWireBinding>::Err(topology.error());
  std::unordered_set<uint32_t> seen;
  std::vector<const WireEdgeUse*> wires;
  for (const auto& region : topology.value().boundary.regions)
    for (const auto& shell : region.shells)
      for (const auto& wire : shell.wire_edges) {
        if (!seen.insert(wire.edge.value).second) continue;
        if (wires.size() == options.max_edges)
          return Result<BodyWireBinding>::Err(
              Error("Body wire binding limit", ErrorCode::kResourceLimit));
        wires.push_back(&wire);
      }
  if (intervals && wires.size() != parameters.size())
    return Result<BodyWireBinding>::Err(
        Error("Wire intervals must cover exactly the selected wires",
              ErrorCode::kInvalidOptions));
  if (intervals)
    for (const auto* wire : wires)
      if (!parameters.contains(wire->edge.value))
        return Result<BodyWireBinding>::Err(Error(
            "Missing or foreign wire interval", ErrorCode::kInvalidOptions));
  std::sort(wires.begin(), wires.end(),
            [](const auto* a, const auto* b) { return a->edge < b->edge; });
  std::vector<BoundWireGeometry> bindings;
  for (const auto* wire : wires) {
    auto binding = [&]() -> Result<BoundWireGeometry> {
      if (intervals)
        return BindWire(model, *wire, parameters.find(wire->edge.value)->second,
                        tolerance, options.intersection);
      const auto& edges = topology.value().edges;
      const auto edge = std::lower_bound(
          edges.begin(), edges.end(), wire->edge,
          [](const auto& boundary, NodeId id) { return boundary.edge < id; });
      if (edge == edges.end() || edge->edge != wire->edge || edge->fins.empty())
        return Result<BoundWireGeometry>::Err(
            Error("Missing validated wire anchor"));
      const auto interval = internal::InferFinInterval(
          model, edge->fins.front().use, tolerance, options.intersection);
      if (!interval.ok())
        return Result<BoundWireGeometry>::Err(interval.error());
      return BindWire(
          model, *wire,
          {wire->edge, interval.value().start, interval.value().end}, tolerance,
          options.intersection);
    }();
    if (!binding.ok()) {
      auto error = binding.error();
      error.message = "Body " + std::to_string(body.value) + ", wire edge " +
                      std::to_string(wire->edge.value) + ": " + error.message;
      return Result<BodyWireBinding>::Err(std::move(error));
    }
    bindings.push_back(std::move(binding).value());
  }
  return Result<BodyWireBinding>::Ok(
      {std::move(topology).value(), std::move(bindings)});
}
}  // namespace
Result<BodyWireBinding> BindBodyWires(
    const Model& model, NodeId body,
    std::span<const WireParameterInterval> intervals, double tolerance,
    const BodyWireBindingOptions& options) {
  return Bind(model, body, intervals, tolerance, options);
}
Result<BodyWireBinding> BindBodyWires(const Model& model, NodeId body,
                                      double tolerance,
                                      const BodyWireBindingOptions& options) {
  return Bind(model, body, std::nullopt, tolerance, options);
}
}  // namespace openparasolid
