// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "base_catalog.h"

namespace openparasolid::internal {
namespace {
FieldDefinition P(const char* name, uint16_t target, uint32_t extent = 0) {
  return {name, FieldType::kPointer, target, extent, true};
}
FieldDefinition D(const char* name) { return {name, FieldType::kInteger}; }
FieldDefinition N(const char* name) { return {name, FieldType::kShort}; }
FieldDefinition F(const char* name) { return {name, FieldType::kDouble}; }
FieldDefinition V(const char* name) { return {name, FieldType::kVector}; }
FieldDefinition C(const char* name) { return {name, FieldType::kCharacter}; }
FieldDefinition U(const char* name) { return {name, FieldType::kByte}; }
FieldDefinition L(const char* name) { return {name, FieldType::kLogical}; }
void Add(SchemaCatalog& catalog, uint16_t type, const char* name,
         std::vector<FieldDefinition> fields) {
  catalog.base_types.push_back(
      {type, name, "Reviewed 13006 base layout", std::move(fields)});
}
void Curve(SchemaCatalog& catalog, uint16_t type, const char* name,
           std::initializer_list<FieldDefinition> tail, bool surface = false) {
  std::vector<FieldDefinition> fields{D("node_id"),
                                      P("attributes_features", 1019),
                                      P("owner", surface ? 1007 : 1010),
                                      P("next", surface ? 1006 : 1008),
                                      P("previous", surface ? 1006 : 1008),
                                      P("geometric_owner", 141),
                                      C("sense")};
  fields.insert(fields.end(), tail.begin(), tail.end());
  Add(catalog, type, name, std::move(fields));
}
}  // namespace
void AddSplineAndListDefinitions(SchemaCatalog& catalog) {
  Add(catalog, 70, "LIST",
      {D("node_id"), P("owner", 1013), P("next", 70), P("previous", 70),
       D("list_type"), D("list_length"), D("block_length"), D("size_of_entry"),
       P("list_block", 1012), P("finger_block", 1012), D("finger_index"),
       L("notransmit")});
  Add(catalog, 74, "POINTER_LIS_BLOCK",
      {D("n_entries"), P("next_block", 74), P("entries", 1001, 1)});
  Add(catalog, 45, "BSPLINE_VERTICES",
      {{"vertices", FieldType::kDouble, 0, 1, true}});
  Add(catalog, 127, "KNOT_MULT",
      {{"multiplicities", FieldType::kShort, 0, 1, true}});
  Add(catalog, 128, "KNOT_SET", {{"knots", FieldType::kDouble, 0, 1, true}});
  Curve(catalog, 133, "TRIMMED_CURVE",
        {P("basis_curve", 1008), V("point_1"), V("point_2"), F("parm_1"),
         F("parm_2")});
  Curve(catalog, 134, "B_CURVE", {P("nurbs", 136), P("data", 135)});
  Add(catalog, 135, "CURVE_DATA", {U("self_int"), P("analytic_form", 1036)});
  Add(catalog, 136, "NURBS_CURVE",
      {N("degree"), D("n_vertices"), N("vertex_dim"), D("n_knots"),
       U("knot_type"), L("periodic"), L("closed"), L("rational"),
       U("curve_form"), P("bspline_vertices", 45), P("knot_mult", 127),
       P("knots", 128)});
  Curve(catalog, 137, "SP_CURVE",
        {P("surface", 1006), P("b_curve", 134), P("original", 1008),
         F("tolerance_to_original")});
  Curve(catalog, 124, "B_SURFACE", {P("nurbs", 126), P("data", 125)}, true);
  std::vector<FieldDefinition> surface_data{
      {"original_u_range", FieldType::kInterval},
      {"original_v_range", FieldType::kInterval},
      {"extended_u_range", FieldType::kInterval},
      {"extended_v_range", FieldType::kInterval},
      U("self_int")};
  for (const char* name :
       {"original_u_start", "original_u_end", "original_v_start",
        "original_v_end", "extended_u_start", "extended_u_end",
        "extended_v_start", "extended_v_end", "analytic_form_type",
        "swept_form_type", "spun_form_type", "blend_form_type"})
    surface_data.push_back(C(name));
  surface_data.push_back(P("analytic_form", 1030));
  surface_data.push_back(P("swept_form", 1031));
  surface_data.push_back(P("spun_form", 1032));
  surface_data.push_back(P("blend_form", 1033));
  Add(catalog, 125, "SURFACE_DATA", std::move(surface_data));
  Add(catalog, 126, "NURBS_SURFACE",
      {L("u_periodic"),       L("v_periodic"),
       N("u_degree"),         N("v_degree"),
       D("n_u_vertices"),     D("n_v_vertices"),
       U("u_knot_type"),      U("v_knot_type"),
       D("n_u_knots"),        D("n_v_knots"),
       L("rational"),         L("u_closed"),
       L("v_closed"),         U("surface_form"),
       N("vertex_dim"),       P("bspline_vertices", 45),
       P("u_knot_mult", 127), P("v_knot_mult", 127),
       P("u_knots", 128),     P("v_knots", 128)});
  Add(catalog, 100, "TRANSFORM",
      {D("node_id"),
       P("owner", 1023),
       P("next", 100),
       P("previous", 100),
       {"rotation_matrix", FieldType::kDouble, 0, 9, true},
       V("translation_vector"),
       F("scale"),
       D("flag"),
       V("perspective_vector")});
}
}  // namespace openparasolid::internal
