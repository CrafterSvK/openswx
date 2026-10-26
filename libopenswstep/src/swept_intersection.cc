// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "swept_intersection.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "openparasolid/evaluate.h"
#include "rational_graph.h"
#include "sweep_bounds.h"
namespace openswstep::internal {
namespace {
using namespace openparasolid;
namespace bounds = sweep_bounds;
const GeometryObject* Geometry(const Model& model, NodeId id) {
  const auto* node = model.FindNode(id);
  return node && node->geometry ? &model.geometry()[*node->geometry] : nullptr;
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Swept intersection proof budget exhausted"};
}
Vector3 Pole(const NurbsCurveGeometry& c, unsigned i) {
  const auto at = i * c.controls.dimension;
  const long double w = c.controls.rational ? c.controls.values[at + 3] : 1;
  return {static_cast<double>(c.controls.values[at] / w),
          static_cast<double>(c.controls.values[at + 1] / w),
          static_cast<double>(c.controls.values[at + 2] / w)};
}
struct Supports {
  NodeId swept_id;
  const SweptSurfaceGeometry* swept = nullptr;
  const PlaneGeometry* plane = nullptr;
};
Supports ReadSupports(const Model& model, const GeometryObject& source) {
  Supports out;
  const auto* intersection =
      std::get_if<IntersectionCurveGeometry>(&source.payload);
  if (!intersection) return out;
  for (auto id : intersection->surfaces) {
    const auto* object = Geometry(model, id);
    if (!object) continue;
    if (const auto* swept =
            std::get_if<SweptSurfaceGeometry>(&object->payload)) {
      out.swept_id = id;
      out.swept = swept;
    }
    if (const auto* plane = std::get_if<PlaneGeometry>(&object->payload))
      out.plane = plane;
  }
  return out;
}
}  // namespace
bool IsSweptPlaneIntersection(const Model& model,
                              const GeometryObject& source) {
  const auto supports = ReadSupports(model, source);
  return supports.swept && supports.plane;
}
Result<IntersectionCurveRecovery> RecoverSweptPlaneIntersection(
    const Model& model, const GeometryObject& source,
    std::span<const BoundFinGeometry* const> uses,
    std::span<const BoundBodyVertex> vertices, double tolerance,
    uint64_t& checks) {
  using Return = Result<IntersectionCurveRecovery>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Invalid swept intersection tolerance"});
  if (uses.size() != 2 || uses[0]->curve != source.source ||
      uses[1]->curve != source.source || !uses[0]->use.edge.value ||
      !uses[0]->use.start_vertex.value || !uses[0]->use.end_vertex.value ||
      uses[0]->use.start_vertex == uses[0]->use.end_vertex ||
      uses[0]->use.edge != uses[1]->use.edge ||
      uses[0]->use.sense == uses[1]->use.sense ||
      uses[0]->use.start_vertex != uses[1]->use.end_vertex ||
      uses[0]->use.end_vertex != uses[1]->use.start_vertex)
    return Return::Err(
        Unsupported("Swept intersection needs opposite paired accurate fins"));
  for (const auto* use : uses)
    if (use->use.fin_curve.value || !use->intersection ||
        use->use.edge_curve != source.source ||
        (use->use.sense != TopologySense::kForward &&
         use->use.sense != TopologySense::kReverse))
      return Return::Err(
          Unsupported("Swept intersection needs accurate fins and explicit "
                      "solver settings"));
  if (uses[0]->start != uses[1]->end || uses[0]->end != uses[1]->start ||
      uses[0]->intersection->chart_tolerance !=
          uses[1]->intersection->chart_tolerance ||
      uses[0]->intersection->solve_tolerance !=
          uses[1]->intersection->solve_tolerance)
    return Return::Err(Unsupported(
        "Paired swept intersection intervals or solver goals disagree"));
  const auto supports = ReadSupports(model, source);
  const auto* intersection =
      std::get_if<IntersectionCurveGeometry>(&source.payload);
  if (!supports.swept || !supports.plane || !intersection ||
      (source.sense != '+' && source.sense != '-') ||
      intersection->start.type != 'L' || intersection->end.type != 'L' ||
      intersection->start.positions.size() != 1 ||
      intersection->end.positions.size() != 1)
    return Return::Err(
        Unsupported("Swept intersection requires an ordinary open plane cut"));
  if (!((uses[0]->surface == intersection->surfaces[0] &&
         uses[1]->surface == intersection->surfaces[1]) ||
        (uses[1]->surface == intersection->surfaces[0] &&
         uses[0]->surface == intersection->surfaces[1])))
    return Return::Err(Unsupported(
        "Swept intersection face supports disagree with the source"));
  const auto* section_object = Geometry(model, supports.swept->section);
  const auto* section =
      section_object ? std::get_if<NurbsCurveGeometry>(&section_object->payload)
                     : nullptr;
  if (!section ||
      section->controls.dimension != (section->controls.rational ? 4 : 3) ||
      (section->axis.degree != 2 && section->axis.degree != 3) ||
      section->axis.closed || section->axis.periodic ||
      section->axis.knots.size() != 2 ||
      section->axis.control_count != section->axis.degree + 1 ||
      section->axis.multiplicities.front() != section->axis.degree + 1 ||
      section->axis.multiplicities.back() != section->axis.degree + 1)
    return Return::Err(
        Unsupported("Swept plane recovery requires one open clamped "
                    "quadratic/cubic span"));
  const auto count = intersection->chart.positions.size();
  if (count < 2)
    return Return::Err(Unsupported("Swept intersection needs an open chart"));
  if (checks < 128 || count > (checks - 128) / 128 ||
      !bounds::Spend(checks, 128 * (count + 1)))
    return Return::Err(Limit());
  const auto first = section->axis.knots.front(),
             last = section->axis.knots.back();
  const auto sample = EvaluateSurface(model, supports.swept_id, first, 0);
  if (!sample.ok()) return Return::Err(sample.error());
  // The public inverse performs the whole transverse monotonicity certificate.
  // It proves global injectivity after adding any displacement along the sweep.
  const auto inverse = InvertSurface(model, supports.swept_id,
                                     sample.value().position, tolerance);
  if (!inverse.ok()) return Return::Err(inverse.error());
  const auto plane_sample =
      EvaluateSurface(GeometryPayload{*supports.plane}, 0, 0);
  if (!plane_sample.ok()) return Return::Err(plane_sample.error());
  // Use the forward evaluator's actual plane basis instead of treating the
  // stored normal as an exact normal to accepted, rounded basis vectors.
  const auto normal =
      bounds::Cross(bounds::Vec(plane_sample.value().u_derivative),
                    bounds::Vec(plane_sample.value().v_derivative));
  const auto direction = bounds::Vec(supports.swept->sweep);
  const auto denominator = bounds::Dot(normal, direction);
  if (!bounds::Valid(denominator) ||
      (denominator.low <= 0 && denominator.high >= 0))
    return Return::Err(
        Unsupported("Plane does not give an isolated swept graph"));
  auto curve = *section;
  std::array<bounds::Vector, 4> graph_controls{}, homogeneous_controls{};
  std::array<double, 4> weights{};
  long double construction_error = 0;
  for (unsigned i = 0; i < section->axis.control_count; ++i) {
    const auto at = i * section->controls.dimension;
    const auto w =
        section->controls.rational ? section->controls.values[at + 3] : 1.;
    weights[i] = w;
    bounds::Vector numerator{};
    const auto origin = bounds::Vec(supports.plane->origin);
    for (unsigned k = 0; k < 3; ++k)
      numerator[k] =
          bounds::Sub(bounds::Mul(origin[k], bounds::Exact(w)),
                      bounds::Exact(section->controls.values[at + k]));
    const auto height =
        bounds::Div(bounds::Dot(normal, numerator), denominator);
    bounds::Vector error{}, model_error{};
    for (unsigned k = 0; k < 3; ++k) {
      const auto value =
          bounds::Add(bounds::Exact(section->controls.values[at + k]),
                      bounds::Mul(height, direction[k]));
      if (!bounds::Valid(value))
        return Return::Err(
            Unsupported("Swept graph controls are unrepresentable"));
      homogeneous_controls[i][k] = value;
      graph_controls[i][k] = section->controls.rational
                                 ? bounds::Div(value, bounds::Exact(w))
                                 : value;
      const auto rounded =
          static_cast<double>(std::midpoint(value.low, value.high));
      curve.controls.values[at + k] = rounded;
      const auto cartesian =
          static_cast<double>(static_cast<long double>(rounded) / w);
      error[k] = bounds::Sub(graph_controls[i][k], bounds::Exact(cartesian));
      model_error[k] =
          bounds::Sub(graph_controls[i][k],
                      bounds::Div(bounds::Exact(rounded), bounds::Exact(w)));
    }
    const auto extra =
        section->controls.rational ? bounds::Length(model_error) : 0;
    construction_error =
        std::max(construction_error, bounds::Length(error) + extra);
  }
  if (!(construction_error <= tolerance / 16))
    return Return::Err(
        Unsupported("Swept graph construction exceeds its error budget"));
  const auto chart = EvaluateIntersectionChart(
      model, source.source, uses[0]->intersection->chart_tolerance,
      uses[0]->intersection->solve_tolerance);
  if (!chart.ok()) return Return::Err(chart.error());
  const unsigned side = intersection->surfaces[0] == supports.swept_id ? 0 : 1;
  int chart_direction = 0;
  long double conditioning = 0;
  for (std::size_t i = 1; i < count; ++i) {
    const auto a = chart.value().surface_parameters[i - 1][side].u;
    const auto b = chart.value().surface_parameters[i][side].u;
    const int sign = b > a ? 1 : b < a ? -1 : 0;
    if (!sign || (chart_direction && sign != chart_direction) || a < first ||
        a > last || b < first || b > last)
      return Return::Err(Unsupported(
          "Swept chart does not select one ordered profile branch"));
    chart_direction = sign;
    const auto chord = bounds::Diff(intersection->chart.positions[i],
                                    intersection->chart.positions[i - 1]);
    if (curve.controls.rational) {
      const auto bounded = RationalGraphChordCondition(
          std::span<const bounds::Vector>(homogeneous_controls.data(),
                                          curve.axis.control_count),
          std::span<const double>(weights.data(), curve.axis.control_count),
          Pole(curve, 0), chord, sign, checks);
      if (!bounded.ok()) return Return::Err(bounded.error());
      conditioning = std::max(conditioning, bounded.value());
      continue;
    }
    for (unsigned j = 1; j < curve.axis.control_count; ++j) {
      bounds::Vector step{};
      for (unsigned k = 0; k < 3; ++k)
        step[k] = bounds::Sub(graph_controls[j][k], graph_controls[j - 1][k]);
      const auto projection = bounds::Dot(step, chord);
      // This whole-domain derivative cone makes each chart projection globally
      // one-to-one. Chart samples alone are never an equivalence certificate.
      if (!bounds::Valid(projection) ||
          (sign > 0 ? projection.low <= 0 : projection.high >= 0))
        return Return::Err(Unsupported(
            "Swept intersection chord projection is not globally unique"));
      const auto minimum = sign > 0 ? projection.low : -projection.high;
      long double maximum_step = 0;
      for (unsigned pole_index = 1; pole_index < curve.axis.control_count;
           ++pole_index) {
        bounds::Vector derivative{};
        for (unsigned k = 0; k < 3; ++k)
          derivative[k] = bounds::Sub(graph_controls[pole_index][k],
                                      graph_controls[pole_index - 1][k]);
        maximum_step = std::max(maximum_step, bounds::Length(derivative));
      }
      const auto ratio =
          bounds::Div(bounds::Mul(bounds::Exact(maximum_step),
                                  bounds::Exact(bounds::Length(chord))),
                      bounds::Exact(minimum));
      conditioning = std::max(conditioning, ratio.high);
    }
  }
  IntersectionCurveRecovery result{uses[0]->use.edge, curve, {}};
  const auto& times = chart.value().parameterization.parameters;
  for (const auto* use : uses) {
    if (!std::isfinite(use->start) || !std::isfinite(use->end) ||
        use->start == use->end ||
        std::min(use->start, use->end) < times.front() ||
        std::max(use->start, use->end) > times.back())
      return Return::Err(
          Unsupported("Swept intersection interval leaves its recorded chart"));
    std::array<double, 2> parameters{};
    for (unsigned end = 0; end < 2; ++end) {
      const auto point = EvaluateIntersectionCurve(model, source.source,
                                                   end ? use->end : use->start,
                                                   *use->intersection);
      if (!point.ok()) return Return::Err(point.error());
      const auto uv = InvertSurface(
          model, supports.swept_id, point.value().position,
          std::min(tolerance / 8, use->intersection->solve_tolerance));
      if (!uv.ok()) return Return::Err(uv.error());
      parameters[end] = uv.value().u;
      const auto exact = EvaluateCurve(GeometryPayload{curve}, parameters[end]);
      if (!exact.ok()) return Return::Err(exact.error());
      const auto id = end ? use->use.end_vertex : use->use.start_vertex;
      const auto vertex = std::lower_bound(
          vertices.begin(), vertices.end(), id.value,
          [](const auto& v, uint32_t value) { return v.vertex.value < value; });
      if (vertex == vertices.end() || vertex->vertex != id)
        return Return::Err({ErrorCode::kInvalidRecord, 0,
                            "Swept intersection vertex is absent"});
      const auto error =
          std::max(bounds::Length(bounds::Diff(exact.value().position,
                                               point.value().position)),
                   bounds::Length(
                       bounds::Diff(exact.value().position, vertex->position)));
      // The chart solve's query-plane residual and endpoint membership error
      // are amplified by the whole-graph chord condition number. Bound the
      // omitted/added endpoint trace, not only the final point distance.
      const auto endpoint_bound = bounds::Add(
          bounds::Exact(construction_error),
          bounds::Mul(
              bounds::Exact(conditioning),
              bounds::Add(bounds::Add(bounds::Exact(error),
                                      bounds::Exact(construction_error)),
                          bounds::Exact(use->intersection->solve_tolerance))));
      if (!bounds::Valid(endpoint_bound) || endpoint_bound.high > tolerance)
        return Return::Err(Unsupported(
            "Conditioned swept endpoint uncertainty exceeds caller tolerance"));
    }
    if ((parameters[1] > parameters[0]) !=
            ((use->end > use->start) == (chart_direction > 0)) ||
        parameters[0] == parameters[1])
      return Return::Err(Unsupported(
          "Swept intersection interval reverses its certified branch"));
    result.uses.push_back({use->use.fin, parameters[0], parameters[1]});
  }
  return Return::Ok(std::move(result));
}
}  // namespace openswstep::internal
