// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOOLS_MODEL_REPORT_H_
#define OPENPARASOLID_TOOLS_MODEL_REPORT_H_
#include <map>
#include <type_traits>

#include "openparasolid/model.h"
#include "schema_report.h"

namespace openparasolid::cli {
inline json VectorJson(const Vector3& v) {
  return json::array({v.x, v.y, v.z});
}
inline json PositionsJson(const std::vector<Vector3>& positions) {
  auto out = json::array();
  for (const auto& point : positions) out.push_back(VectorJson(point));
  return out;
}
inline json LimitJson(const IntersectionLimit& limit) {
  json out{{"source", limit.source.value},
           {"type", limit.type},
           {"positions", PositionsJson(limit.positions)}};
  if (limit.term_use) out["term_use"] = *limit.term_use;
  return out;
}
inline json AxisJson(const SplineAxis& axis) {
  return {{"degree", axis.degree},
          {"control_count", axis.control_count},
          {"periodic", axis.periodic},
          {"closed", axis.closed},
          {"knot_type", axis.knot_type},
          {"knots", axis.knots},
          {"multiplicities", axis.multiplicities}};
}
inline json ControlsJson(const SplineControlNet& controls) {
  return {{"source", controls.source.value},
          {"dimension", controls.dimension},
          {"rational", controls.rational},
          {"values", controls.values}};
}
inline json GeometryJson(const GeometryObject& geometry) {
  json out{{"source", geometry.source.value},
           {"kind", GeometryName(geometry.payload)},
           {"sense", geometry.sense ? json(*geometry.sense) : json(nullptr)}};
  std::visit(
      [&out](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, PointGeometry>)
          out["position"] = VectorJson(p.position);
        else if constexpr (std::is_same_v<T, IntersectionCurveGeometry>) {
          out["surfaces"] = {p.surfaces[0].value, p.surfaces[1].value};
          out["chart"] = {
              {"source", p.chart.source.value},
              {"base_parameter", p.chart.base_parameter},
              {"base_scale", p.chart.base_scale},
              {"chordal_error", p.chart.chordal_error
                                    ? json(*p.chart.chordal_error) : json(nullptr)},
              {"angular_error", p.chart.angular_error
                                    ? json(*p.chart.angular_error) : json(nullptr)},
              {"parameter_error", json::array()},
              {"positions", PositionsJson(p.chart.positions)}};
          for (const auto& error : p.chart.parameter_error)
            out["chart"]["parameter_error"].push_back(
                error ? json(*error) : json(nullptr));
          out["start"] = LimitJson(p.start);
          out["end"] = LimitJson(p.end);
          if (p.intersection_data)
            out["intersection_data"] = p.intersection_data->value;
        } else if constexpr (std::is_same_v<T, NurbsCurveGeometry>) {
          out["descriptor"] = p.descriptor.value;
          out["axis"] = AxisJson(p.axis);
          out["controls"] = ControlsJson(p.controls);
          out["curve_form"] = p.curve_form;
        } else if constexpr (std::is_same_v<T, NurbsSurfaceGeometry>) {
          out["descriptor"] = p.descriptor.value;
          out["u"] = AxisJson(p.u);
          out["v"] = AxisJson(p.v);
          out["controls"] = ControlsJson(p.controls);
          out["surface_form"] = p.surface_form;
        } else if constexpr (std::is_same_v<T, SurfaceCurveGeometry>) {
          out["surface"] = p.surface.value;
          out["parameter_curve"] = p.parameter_curve.value;
        } else if constexpr (std::is_same_v<T, TrimmedCurveGeometry>) {
          out["basis_curve"] = p.basis_curve.value;
          out["point_1"] = VectorJson(p.point_1);
          out["point_2"] = VectorJson(p.point_2);
          out["parameter_1"] = p.parameter_1;
          out["parameter_2"] = p.parameter_2;
        } else if constexpr (std::is_same_v<T, SweptSurfaceGeometry>) {
          out["section"] = p.section.value;
          out["sweep"] = VectorJson(p.sweep);
          out["scale"] = p.scale ? json(*p.scale) : json(nullptr);
        } else if constexpr (std::is_same_v<T, SpunSurfaceGeometry>) {
          out["profile"] = p.profile.value;
          out["base"] = VectorJson(p.base);
          out["axis"] = VectorJson(p.axis);
          out["start"] = p.start ? VectorJson(*p.start) : json(nullptr);
          out["end"] = p.end ? VectorJson(*p.end) : json(nullptr);
          out["x_axis"] = p.x_axis ? VectorJson(*p.x_axis) : json(nullptr);
          out["start_parameter"] =
              p.start_parameter ? json(*p.start_parameter) : json(nullptr);
          out["end_parameter"] =
              p.end_parameter ? json(*p.end_parameter) : json(nullptr);
          out["scale"] = p.scale ? json(*p.scale) : json(nullptr);
        } else {
          if constexpr (requires { p.origin; })
            out["origin"] = VectorJson(p.origin);
          if constexpr (requires { p.center; })
            out["center"] = VectorJson(p.center);
          if constexpr (requires { p.direction; })
            out["direction"] = VectorJson(p.direction);
          if constexpr (requires { p.axis; }) out["axis"] = VectorJson(p.axis);
          if constexpr (requires { p.normal; })
            out["normal"] = VectorJson(p.normal);
          if constexpr (requires { p.x_axis; })
            out["x_axis"] = VectorJson(p.x_axis);
          if constexpr (requires { p.radius; }) out["radius"] = p.radius;
          if constexpr (requires { p.major_radius; })
            out["major_radius"] = p.major_radius;
          if constexpr (requires { p.minor_radius; })
            out["minor_radius"] = p.minor_radius;
          if constexpr (std::is_same_v<T, ConeGeometry>) {
            out["sin_half_angle"] = p.sin_half_angle;
            out["cos_half_angle"] = p.cos_half_angle;
          }
        }
      },
      geometry.payload);
  return out;
}
inline json ModelReport(const Model& model, const ModelOptions& options,
                        bool summary) {
  const auto& s = model.statistics();
  std::map<std::string, uint64_t> kinds, types;
  for (const auto& g : model.geometry()) ++kinds[GeometryName(g.payload)];
  for (const auto& n : model.nodes()) ++types[std::to_string(n.node_type)];
  json out{{"node_count", model.nodes().size()},
           {"geometry_count", model.geometry().size()},
           {"node_type_counts", types},
           {"geometry_counts", kinds},
           {"references_verified", model.references_verified()},
           {"typed_geometry_complete", model.typed_geometry_complete()},
           {"null_reference_count", s.null_references},
           {"resolved_reference_count", s.resolved_references},
           {"unverified_reference_count", s.unverified_references},
           {"missing_reference_count", s.missing_references},
           {"wrong_class_reference_count", s.wrong_class_references},
           {"geometry_candidate_count", s.geometry_candidates},
           {"invalid_geometry_count", s.invalid_geometry},
           {"unsupported_geometry_count", s.unsupported_geometry},
           {"issue_count", s.total_issues},
           {"issues_truncated", model.issues_truncated()},
           {"validation_scope", "references_and_supported_geometry_data"},
           {"limits",
            {{"max_nodes", options.max_nodes},
             {"max_fields", options.max_fields},
             {"max_references", options.max_references},
             {"max_values", options.max_values},
             {"max_diagnostics", options.max_diagnostics}}}};
  if (summary) return out;
  out["nodes"] = json::array();
  for (const auto& n : model.nodes()) {
    json node{
        {"id", n.id.value},
        {"node_type", n.node_type},
        {"source_offset", n.source_offset},
        {"geometry", n.geometry ? json(*n.geometry) : json(nullptr)},
        {"topology_sense",
         n.topology_sense ? json(*n.topology_sense) : json(nullptr)},
        {"region_type", n.region_type ? json(*n.region_type) : json(nullptr)},
        {"body_type", n.body_type ? json(*n.body_type) : json(nullptr)},
        {"references", json::array()}};
    for (const auto& r : n.references)
      node["references"].push_back(
          {{"field", r.field},
           {"element", r.element},
           {"target", r.target.value},
           {"pointer_class", r.pointer_class},
           {"status", ToString(r.status)},
           {"target_node",
            r.target_node ? json(*r.target_node) : json(nullptr)}});
    out["nodes"].push_back(std::move(node));
  }
  out["geometry"] = json::array();
  for (const auto& g : model.geometry())
    out["geometry"].push_back(GeometryJson(g));
  out["issues"] = json::array();
  for (const auto& i : model.issues())
    out["issues"].push_back({{"code", ToString(i.code)},
                             {"node", i.node.value},
                             {"offset", i.offset},
                             {"field", i.field},
                             {"message", i.message}});
  return out;
}
}  // namespace openparasolid::cli

#endif  // OPENPARASOLID_TOOLS_MODEL_REPORT_H_
