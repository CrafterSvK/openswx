// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "base_catalog.h"

namespace openparasolid::internal {
namespace {
FieldDefinition P(const char* name, uint16_t target, uint32_t extent = 0) {
  return {name, FieldType::kPointer, target, extent, true};
}
FieldDefinition F(const char* name, uint32_t extent = 0) {
  return {name, FieldType::kDouble, 0, extent, true};
}
FieldDefinition V(const char* name) { return {name, FieldType::kVector}; }
FieldDefinition C(const char* name) { return {name, FieldType::kCharacter}; }
void Surface(SchemaCatalog& catalog, uint16_t type, const char* name,
             std::initializer_list<FieldDefinition> tail) {
  std::vector<FieldDefinition> fields{{"node_id", FieldType::kInteger},
                                      P("attributes_features", 1019),
                                      P("owner", 1007),
                                      P("next", 1006),
                                      P("previous", 1006),
                                      P("geometric_owner", 141),
                                      C("sense")};
  fields.insert(fields.end(), tail.begin(), tail.end());
  catalog.base_types.push_back(
      {type, name, "Reviewed 13006 base layout", std::move(fields)});
}
}  // namespace
void AddProceduralDefinitions(SchemaCatalog& catalog) {
  Surface(catalog, 56, "BLENDED_EDGE",
          {C("blend_type"), P("surface", 1006, 2), P("spine", 1008),
           F("range", 2), F("thumb_weight", 2), P("boundary", 1006, 2),
           P("start", 41), P("end", 41)});
  Surface(catalog, 59, "BLEND_BOUND",
          {{"boundary", FieldType::kShort}, P("blend", 1006)});
  Surface(catalog, 60, "OFFSET_SURF",
          {C("check"),
           {"true_offset", FieldType::kLogical},
           P("surface", 1006),
           F("offset"),
           F("scale")});
  Surface(catalog, 67, "SWEPT_SURF",
          {P("section", 1008), V("sweep"), F("scale")});
  Surface(catalog, 68, "SPUN_SURF",
          {P("profile", 1008), V("base"), V("axis"), V("start"), V("end"),
           F("start_param"), F("end_param"), V("x_axis"), F("scale")});
  // A form is an auxiliary record, without the ANY_SURF common fields.
  catalog.base_types.push_back({163,
                                "HELIX_SU_FORM",
                                "Reviewed 13006 base layout",
                                {V("axis_pt"),
                                 V("axis_dir"),
                                 C("hand"),
                                 {"turns", FieldType::kInterval},
                                 F("pitch"),
                                 F("gap"),
                                 F("tol")}});
  catalog.base_types.push_back({184,
                                "HELIX_CU_FORM",
                                "Reviewed 13006 base layout",
                                {V("axis_pt"),
                                 V("axis_dir"),
                                 V("point"),
                                 C("hand"),
                                 {"turns", FieldType::kInterval},
                                 F("pitch"),
                                 F("tol")}});
}
}  // namespace openparasolid::internal
