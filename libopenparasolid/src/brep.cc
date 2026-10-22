// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/brep.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "brep_internal.h"
#include "evaluate_internal.h"
namespace openparasolid {
namespace {
using Ids = std::unordered_set<uint32_t>;
ParseError Error(const char* message, uint64_t offset = 0,
                 ErrorCode code = ErrorCode::kInvalidRecord) {
  return {code, offset, message};
}
Result<NodeId> Ref(const Model& model, NodeId id, const char* role,
                   uint16_t type = 0, bool nullable = true) {
  const auto* node = model.FindNode(id);
  if (!node) return Result<NodeId>::Err(Error("Missing BRep node"));
  const ModelReference* found = nullptr;
  for (const auto& r : node->references)
    if (r.field == role) {
      if (found || r.element)
        return Result<NodeId>::Err(
            Error("Ambiguous BRep role", node->source_offset));
      found = &r;
    }
  if (!found)
    return Result<NodeId>::Err(Error("Unavailable BRep role",
                                     node->source_offset,
                                     ErrorCode::kUnsupportedFormat));
  if (found->status == ModelReferenceStatus::kUnverifiedClass)
    return Result<NodeId>::Err(Error("Unreviewed BRep pointer class",
                                     node->source_offset,
                                     ErrorCode::kUnsupportedFormat));
  if (found->status != ModelReferenceStatus::kResolved &&
      found->status != ModelReferenceStatus::kNull)
    return Result<NodeId>::Err(
        Error("Unresolved BRep reference", node->source_offset));
  if (!found->target.value) {
    if (nullable) return Result<NodeId>::Ok(found->target);
    return Result<NodeId>::Err(
        Error("Null required BRep reference", node->source_offset));
  }
  const auto* target = model.FindNode(found->target);
  if (!target || (type && target->node_type != type))
    return Result<NodeId>::Err(
        Error("Wrong BRep target type", node->source_offset));
  return Result<NodeId>::Ok(found->target);
}
bool Claims(const ModelNode& node, const char* role, const Ids& owners) {
  for (const auto& r : node.references)
    if (r.field == role && owners.contains(r.target.value)) return true;
  return false;
}
Result<Ids> List(const Model& model, NodeId body, const char* role,
                 uint16_t type, const Ids& expected) {
  auto head = Ref(model, body, role, type);
  if (!head.ok()) return Result<Ids>::Err(head.error());
  Ids seen;
  NodeId previous, current = head.value();
  while (current.value) {
    if (!expected.contains(current.value) || !seen.insert(current.value).second)
      return Result<Ids>::Err(Error("Unexpected or cyclic body owner list"));
    const auto before = Ref(model, current, "previous", type);
    const auto after = Ref(model, current, "next", type);
    if (!before.ok()) return Result<Ids>::Err(before.error());
    if (!after.ok()) return Result<Ids>::Err(after.error());
    if (before.value() != previous)
      return Result<Ids>::Err(Error("Nonreciprocal body list"));
    previous = current;
    current = after.value();
  }
  if (seen != expected)
    return Result<Ids>::Err(Error("Incomplete body owner list"));
  return Result<Ids>::Ok(std::move(seen));
}
Result<Vector3> Point(const Model& model, NodeId vertex) {
  auto point = Ref(model, vertex, "point", 29, false);
  if (!point.ok()) return Result<Vector3>::Err(point.error());
  const auto* node = model.FindNode(point.value());
  if (!node->geometry)
    return Result<Vector3>::Err(Error("Untyped vertex point",
                                      node->source_offset,
                                      ErrorCode::kUnsupportedFormat));
  const auto* p =
      std::get_if<PointGeometry>(&model.geometry()[*node->geometry].payload);
  if (!p) return Result<Vector3>::Err(Error("Vertex target is not a point"));
  return Result<Vector3>::Ok(p->position);
}
const GeometryObject* Geometry(const Model& model, NodeId id) {
  const auto* n = model.FindNode(id);
  return n && n->geometry ? &model.geometry()[*n->geometry] : nullptr;
}
double Distance(Vector3 a, Vector3 b) {
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}
// Accurate conics are simple periodic curves: the curve sense and fin sense
// select the directed arc, including a full turn for a topologically closed
// edge.
Result<FinParameterInterval> ConicInterval(const Model& model,
                                           const GeometryObject& curve,
                                           const FinUse& use) {
  if (use.fin_curve.value || !curve.sense ||
      (*curve.sense != '+' && *curve.sense != '-'))
    return Result<FinParameterInterval>::Err(
        Error("Conic interval requires accurate edge orientation", 0,
              ErrorCode::kUnsupportedFormat));
  const bool increasing =
      (*curve.sense == '+') == (use.sense == TopologySense::kForward);
  constexpr double turn = 2 * std::numbers::pi;
  if (!use.start_vertex.value && !use.end_vertex.value)
    return Result<FinParameterInterval>::Ok(
        {use.fin, 0, increasing ? turn : -turn});
  if (!use.start_vertex.value || !use.end_vertex.value)
    return Result<FinParameterInterval>::Err(
        Error("Conic interval has only one endpoint", 0,
              ErrorCode::kUnsupportedFormat));
  Vector3 center, x, normal;
  double major, minor;
  if (const auto* circle = std::get_if<CircleGeometry>(&curve.payload)) {
    center = circle->center;
    x = circle->x_axis;
    normal = circle->normal;
    major = minor = circle->radius;
  } else {
    const auto& ellipse = std::get<EllipseGeometry>(curve.payload);
    center = ellipse.center;
    x = ellipse.x_axis;
    normal = ellipse.normal;
    major = ellipse.major_radius;
    minor = ellipse.minor_radius;
  }
  const Vector3 y{normal.y * x.z - normal.z * x.y,
                  normal.z * x.x - normal.x * x.z,
                  normal.x * x.y - normal.y * x.x};
  auto angle = [&](NodeId vertex) -> Result<double> {
    auto point = Point(model, vertex);
    if (!point.ok()) return Result<double>::Err(point.error());
    const long double dx = static_cast<long double>(point.value().x) - center.x;
    const long double dy = static_cast<long double>(point.value().y) - center.y;
    const long double dz = static_cast<long double>(point.value().z) - center.z;
    const long double xx = static_cast<long double>(x.x) * x.x +
                           static_cast<long double>(x.y) * x.y +
                           static_cast<long double>(x.z) * x.z;
    const long double yy = static_cast<long double>(y.x) * y.x +
                           static_cast<long double>(y.y) * y.y +
                           static_cast<long double>(y.z) * y.z;
    const long double cosine = (dx * x.x + dy * x.y + dz * x.z) / xx / major;
    const long double sine = (dx * y.x + dy * y.y + dz * y.z) / yy / minor;
    if (!std::isfinite(cosine) || !std::isfinite(sine) ||
        (cosine == 0 && sine == 0))
      return Result<double>::Err(Error("Invalid conic endpoint projection"));
    return Result<double>::Ok(static_cast<double>(std::atan2(sine, cosine)));
  };
  auto start = angle(use.start_vertex), end = angle(use.end_vertex);
  if (!start.ok()) return Result<FinParameterInterval>::Err(start.error());
  if (!end.ok()) return Result<FinParameterInterval>::Err(end.error());
  double span = end.value() - start.value();
  if (use.start_vertex == use.end_vertex) {
    span = increasing ? turn : -turn;
  } else {
    if (span == 0 || std::abs(span) == turn)
      return Result<FinParameterInterval>::Err(
          Error("Distinct conic vertices have ambiguous coincident parameters",
                0, ErrorCode::kUnsupportedFormat));
    if (increasing && span < 0) span += turn;
    if (!increasing && span > 0) span -= turn;
  }
  return Result<FinParameterInterval>::Ok(
      {use.fin, start.value(), start.value() + span});
}
struct ChartRecoveryBudget {
  uint32_t evaluations = 64;
  uint64_t point_visits = 1000000;
};
Result<double> RecoverChartParameter(
    const Model& model, const GeometryObject& curve,
    const IntersectionChartEvaluation& chart, Vector3 point, double tolerance,
    const IntersectionEvaluationOptions& options, ChartRecoveryBudget& budget) {
  const auto& positions =
      std::get<IntersectionCurveGeometry>(curve.payload).chart.positions;
  const auto& parameters = chart.parameterization.parameters;
  std::optional<double> match;
  std::unordered_set<double> visited;
  for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
    const auto a = positions[i], b = positions[i + 1];
    const long double dx = static_cast<long double>(b.x) - a.x;
    const long double dy = static_cast<long double>(b.y) - a.y;
    const long double dz = static_cast<long double>(b.z) - a.z;
    const long double length = std::hypot(dx, dy, dz);
    if (!(length > 0) || !std::isfinite(length))
      return Result<double>::Err(Error("Unrepresentable chart chord length"));
    const long double projection =
        ((static_cast<long double>(point.x) - a.x) / length) * (dx / length) +
        ((static_cast<long double>(point.y) - a.y) / length) * (dy / length) +
        ((static_cast<long double>(point.z) - a.z) / length) * (dz / length);
    if (!std::isfinite(projection))
      return Result<double>::Err(
          Error("Unrepresentable chart chord projection"));
    if (projection < 0 || projection > 1) continue;
    // The existing solver's chord-normal plane is linear in the original
    // chart parameter. Project only to recover that plane, then verify the
    // actual solved 3D curve point rather than interpolating curve geometry.
    const double parameter = std::lerp(parameters[i], parameters[i + 1],
                                       static_cast<double>(projection));
    if (!visited.insert(parameter).second) continue;
    if (!budget.evaluations || positions.size() > budget.point_visits)
      return Result<double>::Err(
          Error("Intersection parameter recovery work limit", 0,
                ErrorCode::kResourceLimit));
    --budget.evaluations;
    budget.point_visits -= positions.size();
    const auto sample =
        EvaluateIntersectionCurve(model, curve.source, parameter, options);
    if (!sample.ok()) return Result<double>::Err(sample.error());
    if (Distance(point, sample.value().position) > tolerance) continue;
    if (match && *match != parameter)
      return Result<double>::Err(
          Error("Ambiguous solved intersection parameter", 0,
                ErrorCode::kUnsupportedFormat));
    match = parameter;
  }
  if (!match)
    return Result<double>::Err(
        Error("No verified intersection parameter inside chart chords", 0,
              ErrorCode::kUnsupportedFormat));
  return Result<double>::Ok(*match);
}
Result<FinParameterInterval> ChartInterval(
    const Model& model, const GeometryObject& curve, const FinUse& use,
    double tolerance,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  if (!intersection)
    return Result<FinParameterInterval>::Err(
        Error("Intersection interval requires explicit solver settings", 0,
              ErrorCode::kUnsupportedFormat));
  if (!use.start_vertex.value || !use.end_vertex.value ||
      use.start_vertex == use.end_vertex)
    return Result<FinParameterInterval>::Err(
        Error("Closed or vertex-free intersection needs an explicit interval",
              0, ErrorCode::kUnsupportedFormat));
  if (!use.fin_curve.value &&
      (!curve.sense || (*curve.sense != '+' && *curve.sense != '-')))
    return Result<FinParameterInterval>::Err(
        Error("Intersection edge/curve sense is unknown", 0,
              ErrorCode::kUnsupportedFormat));
  const auto start = Point(model, use.start_vertex);
  const auto end = Point(model, use.end_vertex);
  if (!start.ok()) return Result<FinParameterInterval>::Err(start.error());
  if (!end.ok()) return Result<FinParameterInterval>::Err(end.error());
  const auto chart = EvaluateIntersectionChart(model, curve.source,
                                               intersection->chart_tolerance);
  if (!chart.ok()) return Result<FinParameterInterval>::Err(chart.error());
  const auto& positions =
      std::get<IntersectionCurveGeometry>(curve.payload).chart.positions;
  std::optional<std::size_t> first, last;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (Distance(positions[i], start.value()) <= tolerance) {
      if (first)
        return Result<FinParameterInterval>::Err(
            Error("Ambiguous intersection start chart point", 0,
                  ErrorCode::kUnsupportedFormat));
      first = i;
    }
    if (Distance(positions[i], end.value()) <= tolerance) {
      if (last)
        return Result<FinParameterInterval>::Err(
            Error("Ambiguous intersection end chart point", 0,
                  ErrorCode::kUnsupportedFormat));
      last = i;
    }
  }
  const auto& parameters = chart.value().parameterization.parameters;
  ChartRecoveryBudget budget;
  if (positions.size() > budget.point_visits)
    return Result<FinParameterInterval>::Err(
        Error("Intersection parameter recovery chart limit", 0,
              ErrorCode::kResourceLimit));
  budget.point_visits -= positions.size();  // The initial chart preparation.
  const auto first_parameter =
      first ? Result<double>::Ok(parameters[*first])
            : RecoverChartParameter(model, curve, chart.value(), start.value(),
                                    tolerance, *intersection, budget);
  if (!first_parameter.ok())
    return Result<FinParameterInterval>::Err(first_parameter.error());
  const auto last_parameter =
      last ? Result<double>::Ok(parameters[*last])
           : RecoverChartParameter(model, curve, chart.value(), end.value(),
                                   tolerance, *intersection, budget);
  if (!last_parameter.ok())
    return Result<FinParameterInterval>::Err(last_parameter.error());
  if (first_parameter.value() == last_parameter.value())
    return Result<FinParameterInterval>::Err(
        Error("Intersection endpoints have a collapsed parameter interval", 0,
              ErrorCode::kUnsupportedFormat));
  const FinParameterInterval interval{use.fin, first_parameter.value(),
                                      last_parameter.value()};
  if (!use.fin_curve.value &&
      ((interval.end > interval.start) !=
       ((*curve.sense == '+') == (use.sense == TopologySense::kForward))))
    return Result<FinParameterInterval>::Err(
        Error("Chart endpoint order contradicts edge/curve sense"));
  return Result<FinParameterInterval>::Ok(interval);
}
Result<FinParameterInterval> SplineInterval(const Model& model,
                                            const GeometryObject& curve,
                                            const FinUse& use,
                                            double tolerance) {
  if (!use.start_vertex.value || !use.end_vertex.value)
    return Result<FinParameterInterval>::Err(
        Error("Spline interval requires explicit endpoints or interval", 0,
              ErrorCode::kUnsupportedFormat));
  if (!use.fin_curve.value &&
      (!curve.sense || (*curve.sense != '+' && *curve.sense != '-')))
    return Result<FinParameterInterval>::Err(
        Error("Spline edge/curve sense is unknown", 0,
              ErrorCode::kUnsupportedFormat));
  const GeometryObject* basis = &curve;
  if (const auto* sp = std::get_if<SurfaceCurveGeometry>(&curve.payload)) {
    const auto* node = model.FindNode(sp->parameter_curve);
    if (!node || node->node_type != 134)
      return Result<FinParameterInterval>::Err(
          Error("Surface curve has no compatible parameter curve"));
    basis = Geometry(model, sp->parameter_curve);
  }
  if (!basis)
    return Result<FinParameterInterval>::Err(Error(
        "Untyped spline parameter curve", 0, ErrorCode::kUnsupportedFormat));
  const auto* spline = std::get_if<NurbsCurveGeometry>(&basis->payload);
  if (!spline)
    return Result<FinParameterInterval>::Err(
        Error("Curve has no NURBS parameter domain", 0,
              ErrorCode::kUnsupportedFormat));
  const auto domain = internal::NurbsCurveDomain(*spline);
  if (!domain.ok()) return Result<FinParameterInterval>::Err(domain.error());
  const double low = domain.value()[0], high = domain.value()[1];
  const auto first = EvaluateCurve(model, curve.source, low);
  const auto second = EvaluateCurve(model, curve.source, high);
  if (!first.ok()) return Result<FinParameterInterval>::Err(first.error());
  if (!second.ok()) return Result<FinParameterInterval>::Err(second.error());
  const auto start = Point(model, use.start_vertex);
  const auto end = Point(model, use.end_vertex);
  if (!start.ok()) return Result<FinParameterInterval>::Err(start.error());
  if (!end.ok()) return Result<FinParameterInterval>::Err(end.error());
  const bool forward =
      Distance(first.value().position, start.value()) <= tolerance &&
      Distance(second.value().position, end.value()) <= tolerance;
  const bool backward =
      Distance(second.value().position, start.value()) <= tolerance &&
      Distance(first.value().position, end.value()) <= tolerance;
  if (forward == backward)
    return Result<FinParameterInterval>::Err(
        Error("Spline domain endpoints need unambiguous vertex correspondence",
              0, ErrorCode::kUnsupportedFormat));
  return Result<FinParameterInterval>::Ok(
      {use.fin, forward ? low : high, forward ? high : low});
}
Result<CurveEvaluation> SampleCurve(
    const Model& model, NodeId curve, double parameter,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  return intersection ? EvaluateCurve(model, curve, parameter, *intersection)
                      : EvaluateCurve(model, curve, parameter);
}
Result<FinParameterInterval> TrimInterval(
    const Model& model, NodeId curve_id, const TrimmedCurveGeometry& trim,
    const FinUse& use, double tolerance,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  const auto first =
      SampleCurve(model, curve_id, trim.parameter_1, intersection);
  const auto second =
      SampleCurve(model, curve_id, trim.parameter_2, intersection);
  if (!first.ok()) return Result<FinParameterInterval>::Err(first.error());
  if (!second.ok()) return Result<FinParameterInterval>::Err(second.error());
  if (Distance(first.value().position, trim.point_1) > tolerance ||
      Distance(second.value().position, trim.point_2) > tolerance)
    return Result<FinParameterInterval>::Err(
        Error("Stored trim points disagree with curve parameters"));
  bool forward = use.sense == TopologySense::kForward;
  if (use.fin_curve.value) {
    // A tolerant fin's curve has no documented edge/curve orientation here.
    // Retain endpoint correspondence and reject ambiguous closed branches.
    if (!use.start_vertex.value || !use.end_vertex.value)
      return Result<FinParameterInterval>::Err(
          Error("Fin trim requires explicit endpoints or interval", 0,
                ErrorCode::kUnsupportedFormat));
    const auto start = Point(model, use.start_vertex);
    const auto end = Point(model, use.end_vertex);
    if (!start.ok()) return Result<FinParameterInterval>::Err(start.error());
    if (!end.ok()) return Result<FinParameterInterval>::Err(end.error());
    forward = Distance(first.value().position, start.value()) <= tolerance &&
              Distance(second.value().position, end.value()) <= tolerance;
    const bool backward =
        Distance(second.value().position, start.value()) <= tolerance &&
        Distance(first.value().position, end.value()) <= tolerance;
    if (forward == backward)
      return Result<FinParameterInterval>::Err(Error(
          "Ambiguous or nonmatching fin trim endpoints", 0,
          forward ? ErrorCode::kUnsupportedFormat : ErrorCode::kInvalidRecord));
  } else if (!use.start_vertex.value && !use.end_vertex.value &&
             Distance(first.value().position, second.value().position) >
                 tolerance) {
    return Result<FinParameterInterval>::Err(
        Error("Vertex-free edge trim is not closed within tolerance"));
  }
  return Result<FinParameterInterval>::Ok(
      {use.fin, forward ? trim.parameter_1 : trim.parameter_2,
       forward ? trim.parameter_2 : trim.parameter_1});
}
}  // namespace
namespace internal {
Result<FinParameterInterval> InferFinInterval(
    const Model& model, const FinUse& use, double tolerance,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  const NodeId curve_id = use.fin_curve.value ? use.fin_curve : use.edge_curve;
  const auto* curve = Geometry(model, curve_id);
  if (!curve)
    return Result<FinParameterInterval>::Err(
        Error("Curve interval needs explicit branch/endpoints", 0,
              ErrorCode::kUnsupportedFormat));
  if (std::holds_alternative<NurbsCurveGeometry>(curve->payload) ||
      std::holds_alternative<SurfaceCurveGeometry>(curve->payload))
    return SplineInterval(model, *curve, use, tolerance);
  if (std::holds_alternative<IntersectionCurveGeometry>(curve->payload))
    return ChartInterval(model, *curve, use, tolerance, intersection);
  if (std::holds_alternative<CircleGeometry>(curve->payload) ||
      std::holds_alternative<EllipseGeometry>(curve->payload))
    return ConicInterval(model, *curve, use);
  if (const auto* trim = std::get_if<TrimmedCurveGeometry>(&curve->payload))
    return TrimInterval(model, curve_id, *trim, use, tolerance, intersection);
  if (!use.start_vertex.value || !use.end_vertex.value)
    return Result<FinParameterInterval>::Err(
        Error("Curve interval needs explicit endpoints", 0,
              ErrorCode::kUnsupportedFormat));
  const auto start = Point(model, use.start_vertex),
             end = Point(model, use.end_vertex);
  if (!start.ok()) return Result<FinParameterInterval>::Err(start.error());
  if (!end.ok()) return Result<FinParameterInterval>::Err(end.error());
  FinParameterInterval interval{use.fin, 0, 0};
  if (const auto* line = std::get_if<LineGeometry>(&curve->payload)) {
    const long double denominator =
        static_cast<long double>(line->direction.x) * line->direction.x +
        static_cast<long double>(line->direction.y) * line->direction.y +
        static_cast<long double>(line->direction.z) * line->direction.z;
    if (!(denominator > 0) || !std::isfinite(denominator))
      return Result<FinParameterInterval>::Err(Error("Invalid line direction"));
    auto parameter = [&](Vector3 point) {
      const long double value =
          ((static_cast<long double>(point.x) - line->origin.x) *
               line->direction.x +
           (static_cast<long double>(point.y) - line->origin.y) *
               line->direction.y +
           (static_cast<long double>(point.z) - line->origin.z) *
               line->direction.z) /
          denominator;
      if (!std::isfinite(value) ||
          std::abs(value) > std::numeric_limits<double>::max())
        return std::numeric_limits<double>::quiet_NaN();
      return static_cast<double>(value);
    };
    interval.start = parameter(start.value());
    interval.end = parameter(end.value());
  } else
    return Result<FinParameterInterval>::Err(
        Error("Curve type needs explicit directed interval", 0,
              ErrorCode::kUnsupportedFormat));
  if (!std::isfinite(interval.start) || !std::isfinite(interval.end) ||
      interval.start == interval.end)
    return Result<FinParameterInterval>::Err(
        Error("Unresolvable curve interval", 0, ErrorCode::kUnsupportedFormat));
  return Result<FinParameterInterval>::Ok(interval);
}
}  // namespace internal
Result<ValidatedBodyTopology> ValidateBodyTopology(
    const Model& model, NodeId body, const BodyValidationOptions& options) {
  const auto& b = options.traversal.body;
  if (!options.max_model_nodes || !options.max_model_references ||
      !options.max_radial_fins || !options.max_vertex_fins ||
      !options.traversal.max_children || !b.max_regions || !b.max_shells ||
      !b.max_face_uses || !b.max_loops || !b.max_fins || !b.max_wire_edges)
    return Result<ValidatedBodyTopology>::Err(
        Error("Positive BRep limits required", 0, ErrorCode::kInvalidOptions));
  if (model.nodes().size() > options.max_model_nodes)
    return Result<ValidatedBodyTopology>::Err(
        Error("BRep model scan limit", 0, ErrorCode::kResourceLimit));
  uint64_t scanned_references = 0;
  for (const auto& node : model.nodes()) {
    if (node.references.size() >
        options.max_model_references - scanned_references)
      return Result<ValidatedBodyTopology>::Err(
          Error("BRep reference scan limit", node.source_offset,
                ErrorCode::kResourceLimit));
    scanned_references += node.references.size();
  }
  auto boundary = TraverseBodyBoundary(model, body, b);
  if (!boundary.ok())
    return Result<ValidatedBodyTopology>::Err(boundary.error());
  const auto* root = model.FindNode(body);
  Ids child_bodies;
  for (const auto& r : root->references)
    if (r.field == "child" && r.target.value) {
      auto compound =
          TraverseCompoundBodyBoundary(model, body, options.traversal);
      if (!compound.ok())
        return Result<ValidatedBodyTopology>::Err(compound.error());
      for (const auto& child : compound.value().children)
        child_bodies.insert(child.body.value);
      break;
    }
  ValidatedBodyTopology result;
  result.boundary = std::move(boundary).value();
  Ids bodies{body.value}, regions, shells, faces, loops, loop_fins, edges,
      wire_edges, vertices, radial_fins;
  std::unordered_set<uint64_t> face_sides;
  std::unordered_map<uint32_t, Ids> wanted_vertex_fins;
  for (const auto& region : result.boundary.regions) {
    regions.insert(region.region.value);
    for (const auto& shell : region.shells) {
      shells.insert(shell.shell.value);
      for (const auto& use : shell.face_uses)
        face_sides.insert(
            (uint64_t{shell.faces[use.boundary].face.value} << 1) |
            (use.side == ShellFaceSide::kFront ? 1u : 0u));
      if (shell.isolated_vertex)
        vertices.insert(shell.isolated_vertex->vertex.value);
      for (const auto& edge : shell.wire_edges) {
        edges.insert(edge.edge.value);
        wire_edges.insert(edge.edge.value);
        if (edge.start_vertex.value) vertices.insert(edge.start_vertex.value);
        if (edge.end_vertex.value) vertices.insert(edge.end_vertex.value);
      }
      for (const auto& face : shell.faces)
        if (faces.insert(face.face.value).second) {
          for (const auto& loop : face.loops) {
            if (!loops.insert(loop.loop.value).second)
              return Result<ValidatedBodyTopology>::Err(
                  Error("Loop shared by distinct faces"));
            if (loop.isolated_vertex) {
              vertices.insert(loop.isolated_vertex->vertex.value);
              wanted_vertex_fins[loop.isolated_vertex->vertex.value].insert(
                  loop.isolated_vertex->fin.value);
              loop_fins.insert(loop.isolated_vertex->fin.value);
            }
            for (const auto& fin : loop.fins) {
              if (!loop_fins.insert(fin.fin.value).second)
                return Result<ValidatedBodyTopology>::Err(
                    Error("Fin shared by distinct loops"));
              edges.insert(fin.edge.value);
              if (fin.start_vertex.value)
                vertices.insert(fin.start_vertex.value);
              if (fin.end_vertex.value) vertices.insert(fin.end_vertex.value);
            }
          }
        }
    }
  }
  uint64_t radial_budget = options.max_radial_fins;
  for (uint32_t id : edges) {
    if (!radial_budget)
      return Result<ValidatedBodyTopology>::Err(
          Error("Aggregate radial fin limit", 0, ErrorCode::kResourceLimit));
    auto edge = TraverseEdgeBoundary(model, NodeId{id}, {radial_budget});
    if (!edge.ok()) return Result<ValidatedBodyTopology>::Err(edge.error());
    radial_budget -= edge.value().fins.size();
    auto owner = Ref(model, NodeId{id}, "owner", 0, false);
    if (!owner.ok()) return Result<ValidatedBodyTopology>::Err(owner.error());
    if (wire_edges.contains(id) ? !shells.contains(owner.value().value)
                                : owner.value() != body)
      return Result<ValidatedBodyTopology>::Err(
          Error("Edge belongs outside selected body"));
    for (std::size_t i = 0; i < edge.value().fins.size(); ++i) {
      const auto& f = edge.value().fins[i];
      radial_fins.insert(f.use.fin.value);
      if (f.loop.value && !loop_fins.contains(f.use.fin.value))
        return Result<ValidatedBodyTopology>::Err(
            Error("Radial fin missing from body loops"));
      if (i < 2 && f.use.end_vertex.value)
        wanted_vertex_fins[f.use.end_vertex.value].insert(f.use.fin.value);
    }
    result.edges.push_back(std::move(edge).value());
  }
  uint64_t vertex_budget = options.max_vertex_fins;
  for (uint32_t id : vertices) {
    if (!vertex_budget) {
      auto head = Ref(model, NodeId{id}, "halfedge", 17);
      if (!head.ok()) return Result<ValidatedBodyTopology>::Err(head.error());
      if (head.value().value)
        return Result<ValidatedBodyTopology>::Err(
            Error("Aggregate vertex fin limit", 0, ErrorCode::kResourceLimit));
    }
    auto vertex = TraverseVertexIncidence(
        model, NodeId{id}, {std::max(uint64_t{1}, vertex_budget)});
    if (!vertex.ok()) return Result<ValidatedBodyTopology>::Err(vertex.error());
    vertex_budget -= vertex.value().fins.size();
    if (vertex.value().owner != body &&
        !shells.contains(vertex.value().owner.value))
      return Result<ValidatedBodyTopology>::Err(
          Error("Vertex belongs outside selected body"));
    Ids actual;
    for (const auto& f : vertex.value().fins) actual.insert(f.fin.value);
    if (actual != wanted_vertex_fins[id])
      return Result<ValidatedBodyTopology>::Err(
          Error("Incomplete or extraneous vertex incidence"));
    result.vertices.push_back(std::move(vertex).value());
  }
  Ids nonwire = edges;
  for (auto id : wire_edges) nonwire.erase(id);
  auto edge_list = List(model, body, "edge", 16, nonwire);
  if (!edge_list.ok())
    return Result<ValidatedBodyTopology>::Err(edge_list.error());
  auto vertex_list = List(model, body, "vertex", 18, vertices);
  if (!vertex_list.ok())
    return Result<ValidatedBodyTopology>::Err(vertex_list.error());
  for (const auto& node : model.nodes()) {
    bool missing = false;
    if (node.node_type == 12 && Claims(node, "owner", bodies))
      missing = !child_bodies.contains(node.id.value);
    if (node.node_type == 19 && Claims(node, "body", bodies))
      missing = !regions.contains(node.id.value);
    if (node.node_type == 13 && Claims(node, "region", regions))
      missing = !shells.contains(node.id.value);
    if (node.node_type == 14 && (Claims(node, "shell", shells) ||
                                 Claims(node, "front_shell", shells))) {
      auto back = Ref(model, node.id, "shell", 13, false),
           front = Ref(model, node.id, "front_shell", 13, false);
      if (!back.ok()) return Result<ValidatedBodyTopology>::Err(back.error());
      if (!front.ok()) return Result<ValidatedBodyTopology>::Err(front.error());
      missing = !shells.contains(back.value().value) ||
                !shells.contains(front.value().value) ||
                !face_sides.contains(uint64_t{node.id.value} << 1) ||
                !face_sides.contains((uint64_t{node.id.value} << 1) | 1u);
    }
    if (node.node_type == 15 && Claims(node, "face", faces))
      missing = !loops.contains(node.id.value);
    if (node.node_type == 16 &&
        (Claims(node, "owner", bodies) || Claims(node, "owner", shells)))
      missing = !edges.contains(node.id.value);
    if (node.node_type == 18 &&
        (Claims(node, "owner", bodies) || Claims(node, "owner", shells)))
      missing = !vertices.contains(node.id.value);
    if (node.node_type == 17) {
      if (Claims(node, "vertex", vertices) &&
          !radial_fins.contains(node.id.value) &&
          !loop_fins.contains(node.id.value))
        missing = true;
      if (Claims(node, "loop", loops) && !loop_fins.contains(node.id.value))
        missing = true;
      if (Claims(node, "edge", edges) && !radial_fins.contains(node.id.value))
        missing = true;
    }
    if (missing)
      return Result<ValidatedBodyTopology>::Err(Error(
          "Unlisted or inconsistent body topology entity", node.source_offset));
  }
  std::sort(result.edges.begin(), result.edges.end(),
            [](const auto& a, const auto& z) { return a.edge < z.edge; });
  std::sort(result.vertices.begin(), result.vertices.end(),
            [](const auto& a, const auto& z) { return a.vertex < z.vertex; });
  return Result<ValidatedBodyTopology>::Ok(std::move(result));
}
Result<FinGeometryEvaluation> EvaluateBoundFin(const Model& model,
                                               const BoundFinGeometry& fin,
                                               double fraction,
                                               double tolerance) {
  if (!std::isfinite(fraction) || fraction < 0 || fraction > 1 ||
      !std::isfinite(tolerance) || tolerance < 0 || !std::isfinite(fin.start) ||
      !std::isfinite(fin.end) || fin.start == fin.end)
    return Result<FinGeometryEvaluation>::Err(
        Error("Invalid fin interval, fraction or tolerance", 0,
              ErrorCode::kInvalidOptions));
  const double t = std::lerp(fin.start, fin.end, fraction);
  auto sample = SampleCurve(model, fin.curve, t, fin.intersection);
  if (!sample.ok()) return Result<FinGeometryEvaluation>::Err(sample.error());
  Result<SurfaceParameters> uv = Result<SurfaceParameters>::Err(
      Error("Unsupported surface mapping", 0, ErrorCode::kUnsupportedFormat));
  const GeometryObject* geometry = Geometry(model, fin.curve);
  Ids visited;
  while (geometry &&
         std::holds_alternative<TrimmedCurveGeometry>(geometry->payload)) {
    if (!visited.insert(geometry->source.value).second || visited.size() > 64)
      return Result<FinGeometryEvaluation>::Err(
          Error("Trim dependency cycle or limit"));
    geometry = Geometry(
        model, std::get<TrimmedCurveGeometry>(geometry->payload).basis_curve);
  }
  if (geometry &&
      std::holds_alternative<SurfaceCurveGeometry>(geometry->payload) &&
      std::get<SurfaceCurveGeometry>(geometry->payload).surface ==
          fin.surface) {
    const auto& sp = std::get<SurfaceCurveGeometry>(geometry->payload);
    auto parameter =
        SampleCurve(model, sp.parameter_curve, t, fin.intersection);
    if (!parameter.ok())
      return Result<FinGeometryEvaluation>::Err(parameter.error());
    auto surface =
        EvaluateSurface(model, fin.surface, parameter.value().position.x,
                        parameter.value().position.y);
    if (!surface.ok())
      return Result<FinGeometryEvaluation>::Err(surface.error());
    uv = Result<SurfaceParameters>::Ok(
        {parameter.value().position.x, parameter.value().position.y,
         Distance(surface.value().position, sample.value().position)});
  } else
    uv = InvertSurface(model, fin.surface, sample.value().position, tolerance);
  if (!uv.ok()) return Result<FinGeometryEvaluation>::Err(uv.error());
  if (uv.value().residual > tolerance)
    return Result<FinGeometryEvaluation>::Err(
        Error("Fin curve exceeds face surface tolerance"));
  const long double span = static_cast<long double>(fin.end) - fin.start;
  auto evaluated = std::move(sample).value();
  auto& d = evaluated.first_derivative;
  d = {static_cast<double>(d.x * span), static_cast<double>(d.y * span),
       static_cast<double>(d.z * span)};
  if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z))
    return Result<FinGeometryEvaluation>::Err(
        Error("Nonfinite oriented fin tangent"));
  return Result<FinGeometryEvaluation>::Ok({evaluated, uv.value()});
}
Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const FaceTraversalOptions& options,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  if (intersection) {
    if (auto error = internal::ValidateIntersectionOptions(*intersection))
      return Result<FaceTrimming>::Err(*error);
  }
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_loops ||
      !options.max_fins)
    return Result<FaceTrimming>::Err(
        Error("Invalid face binding options", 0, ErrorCode::kInvalidOptions));
  if (intervals.size() > options.max_fins)
    return Result<FaceTrimming>::Err(
        Error("Face parameter limit", 0, ErrorCode::kResourceLimit));
  std::unordered_map<uint32_t, const FinParameterInterval*> parameters;
  for (const auto& p : intervals) {
    if (!std::isfinite(p.start) || !std::isfinite(p.end) || p.start == p.end ||
        !parameters.emplace(p.fin.value, &p).second)
      return Result<FaceTrimming>::Err(
          Error("Duplicate or invalid directed interval", 0,
                ErrorCode::kInvalidOptions));
  }
  auto boundary = TraverseFaceBoundary(model, face, options);
  if (!boundary.ok()) return Result<FaceTrimming>::Err(boundary.error());
  FaceTrimming result{
      face, boundary.value().surface, boundary.value().face_sense, {}};
  for (const auto& loop : boundary.value().loops) {
    FaceTrimLoop out;
    out.loop = loop.loop;
    if (loop.isolated_vertex) {
      auto p = Point(model, loop.isolated_vertex->vertex);
      if (!p.ok()) return Result<FaceTrimming>::Err(p.error());
      auto uv = InvertSurface(model, result.surface, p.value(), tolerance);
      if (!uv.ok()) return Result<FaceTrimming>::Err(uv.error());
      out.isolated_vertex = uv.value();
    }
    for (const auto& use : loop.fins) {
      auto found = parameters.find(use.fin.value);
      if (found == parameters.end())
        return Result<FaceTrimming>::Err(Error("Missing fin parameter interval",
                                               0, ErrorCode::kInvalidOptions));
      const auto& p = *found->second;
      if (!use.fin_curve.value) {
        const auto* geometry = Geometry(model, use.edge_curve);
        std::optional<bool> increasing;
        if (geometry) {
          if (const auto* trim =
                  std::get_if<TrimmedCurveGeometry>(&geometry->payload))
            increasing = trim->parameter_2 > trim->parameter_1;
          else if (geometry->sense &&
                   (*geometry->sense == '+' || *geometry->sense == '-'))
            increasing = *geometry->sense == '+';
        }
        if (increasing &&
            ((p.end > p.start) !=
             (*increasing == (use.sense == TopologySense::kForward))))
          return Result<FaceTrimming>::Err(
              Error("Directed interval contradicts edge/curve sense"));
      }

      BoundFinGeometry fin{use,
                           face,
                           result.surface,
                           use.fin_curve.value ? use.fin_curve : use.edge_curve,
                           p.start,
                           p.end,
                           intersection};
      for (double fraction : {0., 0.5, 1.}) {
        auto sample = EvaluateBoundFin(model, fin, fraction, tolerance);
        if (!sample.ok()) return Result<FaceTrimming>::Err(sample.error());
        const NodeId vertex = fraction == 0   ? use.start_vertex
                              : fraction == 1 ? use.end_vertex
                                              : NodeId{};
        if (vertex.value) {
          auto point = Point(model, vertex);
          if (!point.ok()) return Result<FaceTrimming>::Err(point.error());
          if (Distance(point.value(), sample.value().curve.position) >
              tolerance)
            return Result<FaceTrimming>::Err(
                Error("Fin interval endpoint disagrees with vertex"));
        }
      }
      out.fins.push_back(fin);
      parameters.erase(found);
    }
    result.loops.push_back(std::move(out));
  }
  if (!parameters.empty())
    return Result<FaceTrimming>::Err(Error("Intervals reference unlisted fins",
                                           0, ErrorCode::kInvalidOptions));
  return Result<FaceTrimming>::Ok(std::move(result));
}
Result<FaceTrimming> BindFaceTrimmingWithOverrides(
    const Model& model, NodeId face,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const FaceTraversalOptions& options,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  if (intersection) {
    if (auto error = internal::ValidateIntersectionOptions(*intersection))
      return Result<FaceTrimming>::Err(*error);
  }
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_loops ||
      !options.max_fins)
    return Result<FaceTrimming>::Err(
        Error("Invalid automatic trim options", 0, ErrorCode::kInvalidOptions));
  auto boundary = TraverseFaceBoundary(model, face, options);
  if (!boundary.ok()) return Result<FaceTrimming>::Err(boundary.error());
  if (overrides.size() > options.max_fins)
    return Result<FaceTrimming>::Err(
        Error("Face interval override limit", 0, ErrorCode::kResourceLimit));
  std::unordered_map<uint32_t, FinParameterInterval> supplied;
  for (const auto& p : overrides) {
    if (!std::isfinite(p.start) || !std::isfinite(p.end) || p.start == p.end ||
        !supplied.emplace(p.fin.value, p).second)
      return Result<FaceTrimming>::Err(Error(
          "Duplicate or invalid fin override", 0, ErrorCode::kInvalidOptions));
  }
  std::size_t listed = 0;
  for (const auto& loop : boundary.value().loops)
    for (const auto& fin : loop.fins)
      if (supplied.contains(fin.fin.value)) ++listed;
  if (listed != supplied.size())
    return Result<FaceTrimming>::Err(
        Error("Overrides reference fins outside selected face edges", 0,
              ErrorCode::kInvalidOptions));
  std::vector<FinParameterInterval> intervals;
  for (const auto& loop : boundary.value().loops)
    for (const auto& use : loop.fins) {
      const auto specified = supplied.find(use.fin.value);
      if (specified != supplied.end()) {
        intervals.push_back(specified->second);
        supplied.erase(specified);
        continue;
      }
      const auto interval =
          internal::InferFinInterval(model, use, tolerance, intersection);
      if (!interval.ok()) return Result<FaceTrimming>::Err(interval.error());
      intervals.push_back(interval.value());
    }
  return BindFaceTrimming(model, face, intervals, tolerance, options,
                          intersection);
}
Result<FaceTrimming> BindFaceTrimming(
    const Model& model, NodeId face, double tolerance,
    const FaceTraversalOptions& options,
    const std::optional<IntersectionEvaluationOptions>& intersection) {
  return BindFaceTrimmingWithOverrides(model, face, {}, tolerance, options,
                                       intersection);
}

}  // namespace openparasolid
