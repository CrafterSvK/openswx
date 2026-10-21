// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "base_catalog.h"

namespace openparasolid::internal {
namespace {
FieldDefinition P(const char* name, uint16_t target, uint32_t extent = 0) {
  return {name, FieldType::kPointer, target, extent, true};
}
FieldDefinition D(const char* name) { return {name, FieldType::kInteger}; }
FieldDefinition F(const char* name) { return {name, FieldType::kDouble}; }
FieldDefinition V(const char* name) { return {name, FieldType::kVector}; }
FieldDefinition C(const char* name) { return {name, FieldType::kCharacter}; }
FieldDefinition U(const char* name) { return {name, FieldType::kByte}; }
void Add(SchemaCatalog& catalog, uint16_t type, const char* name,
         std::vector<FieldDefinition> fields) {
  catalog.base_types.push_back(
      {type, name, "Reviewed 13006 base layout", std::move(fields)});
}
std::vector<FieldDefinition> Geometry(bool surface) {
  return {D("node_id"),
          P("attributes_features", 1019),
          P("owner", surface ? 1007 : 1010),
          P("next", surface ? 1006 : 1008),
          P("previous", surface ? 1006 : 1008),
          P("geometric_owner", 141),
          C("sense")};
}
void Analytic(SchemaCatalog& catalog, uint16_t type, const char* name,
              bool surface, std::initializer_list<FieldDefinition> tail) {
  auto fields = Geometry(surface);
  fields.insert(fields.end(), tail.begin(), tail.end());
  Add(catalog, type, name, std::move(fields));
}
}  // namespace
void AddGeometryDefinitions(SchemaCatalog& catalog) {
  // The base layout precedes all stream-declared mesh, index and owner edits.
  Add(catalog, 12, "BODY",
      {D("highest_node_id"),
       P("attributes_features", 1019),
       P("attribute_chains", 70),
       P("surface", 1006),
       P("curve", 1008),
       P("point", 29),
       P("key", 102),
       F("res_size"),
       F("res_linear"),
       P("ref_instance", 11),
       P("next", 12),
       P("previous", 12),
       U("state"),
       P("owner", 101),
       U("body_type"),
       U("nom_geom_state"),
       P("shell", 13),
       P("boundary_surface", 1006),
       P("boundary_curve", 1008),
       P("boundary_point", 29),
       P("region", 19),
       P("edge", 16),
       P("vertex", 18)});
  Add(catalog, 13, "SHELL",
      {D("node_id"), P("attributes_features", 1019), P("body", 12),
       P("next", 13), P("face", 14), P("edge", 16), P("vertex", 18),
       P("region", 19), P("front_face", 14)});
  Add(catalog, 14, "FACE",
      {D("node_id"), P("attributes_features", 1019), F("tolerance"),
       P("next", 14), P("previous", 14), P("loop", 15), P("shell", 13),
       P("surface", 1006), C("sense"), P("next_on_surface", 14),
       P("previous_on_surface", 14), P("next_front", 14),
       P("previous_front", 14), P("front_shell", 13)});
  Add(catalog, 16, "EDGE",
      {D("node_id"), P("attributes_features", 1019), F("tolerance"),
       P("halfedge", 17), P("previous", 16), P("next", 16), P("curve", 1008),
       P("next_on_curve", 16), P("previous_on_curve", 16), P("owner", 1029)});
  Add(catalog, 18, "VERTEX",
      {D("node_id"), P("attributes_features", 1019), P("halfedge", 17),
       P("previous", 18), P("next", 18), P("point", 29), F("tolerance"),
       P("owner", 1029)});
  Add(catalog, 19, "REGION",
      {D("node_id"), P("attributes_features", 1019), P("body", 12),
       P("next", 19), P("previous", 19), P("shell", 13), C("type")});
  Add(catalog, 29, "POINT",
      {D("node_id"), P("attributes_features", 1019), P("owner", 1011),
       P("next", 29), P("previous", 29), V("position")});
  Analytic(catalog, 30, "LINE", false, {V("origin"), V("direction")});
  Analytic(catalog, 31, "CIRCLE", false,
           {V("center"), V("normal"), V("x_axis"), F("radius")});
  Analytic(catalog, 32, "ELLIPSE", false,
           {V("center"), V("normal"), V("x_axis"), F("major_radius"),
            F("minor_radius")});
  Analytic(catalog, 50, "PLANE", true, {V("origin"), V("normal"), V("x_axis")});
  Analytic(catalog, 51, "CYLINDER", true,
           {V("origin"), V("axis"), F("radius"), V("x_axis")});
  Analytic(catalog, 52, "CONE", true,
           {V("origin"), V("axis"), F("radius"), F("sin_half_angle"),
            F("cos_half_angle"), V("x_axis")});
  Analytic(catalog, 53, "SPHERE", true,
           {V("center"), F("radius"), V("axis"), V("x_axis")});
  Analytic(catalog, 54, "TORUS", true,
           {V("center"), V("axis"), F("major_radius"), F("minor_radius"),
            V("x_axis")});
  Analytic(
      catalog, 38, "INTERSECTION", false,
      {P("surface", 1006, 2), P("chart", 40), P("start", 41), P("end", 41)});
  Add(catalog, 40, "CHART",
      {F("base_parameter"),
       F("base_scale"),
       D("chart_count"),
       F("chordal_error"),
       F("angular_error"),
       {"parameter_error", FieldType::kDouble, 0, 2, true},
       {"positions", FieldType::kIntersectionPoint, 0, 1, true}});
  Add(catalog, 41, "LIMIT",
      {C("type"), {"positions", FieldType::kIntersectionPoint, 0, 1, true}});
  Add(catalog, 86, "VECTOR_VALUES",
      {{"values", FieldType::kVector, 0, 1, true}});
  Add(catalog, 141, "GEOMETRIC_OWNER",
      {P("owner", 1003), P("next", 141), P("previous", 141),
       P("shared_geometry", 1003)});
}
}  // namespace openparasolid::internal
