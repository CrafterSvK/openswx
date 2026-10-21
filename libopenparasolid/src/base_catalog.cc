// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "base_catalog.h"

namespace openparasolid::internal {
namespace {
FieldDefinition Pointer(const char* name, uint16_t target,
                        uint32_t elements = 0) {
  return {name, FieldType::kPointer, target, elements, true};
}
void Add(SchemaCatalog& catalog, uint16_t type, const char* name,
         std::vector<FieldDefinition> fields) {
  catalog.base_types.push_back(
      {type, name, "Reviewed 13006 base layout", std::move(fields)});
}
}  // namespace
void AddValueAndTopologyDefinitions(SchemaCatalog& catalog) {
  Add(catalog, 79, "ATT_DEF_ID",
      {{"string", FieldType::kCharacter, 0, 1, true}});
  Add(catalog, 80, "ATTRIB_DEF",
      {Pointer("next", 80),
       Pointer("identifier", 79),
       {"type_id", FieldType::kInteger},
       {"actions", FieldType::kByte, 0, 8, true},
       Pointer("field_names", 99),
       {"legal_owners", FieldType::kLogical, 0, 14, true},
       {"fields", FieldType::kByte, 0, 1, true}});
  Add(catalog, 99, "FIELD_NAMES", {Pointer("names", 1037, 1)});
  Add(catalog, 81, "ATTRIBUTE",
      {{"node_id", FieldType::kInteger},
       Pointer("definition", 80),
       Pointer("owner", 1015),
       Pointer("next", 1019),
       Pointer("previous", 1019),
       Pointer("next_of_type", 81),
       Pointer("previous_of_type", 81),
       Pointer("fields", 1018, 1)});
  Add(catalog, 82, "INT_VALUES", {{"values", FieldType::kInteger, 0, 1, true}});
  Add(catalog, 83, "REAL_VALUES", {{"values", FieldType::kDouble, 0, 1, true}});
  Add(catalog, 84, "CHAR_VALUES",
      {{"values", FieldType::kCharacter, 0, 1, true}});
  Add(catalog, 98, "UNICODE_VALUES",
      {{"values", FieldType::kUnicode, 0, 1, true}});
  Add(catalog, 15, "LOOP",
      {{"node_id", FieldType::kInteger},
       Pointer("attributes_features", 1019),
       Pointer("halfedge", 17),
       Pointer("face", 14),
       Pointer("next", 15)});
  Add(catalog, 17, "FIN",
      {Pointer("attributes_features", 1019),
       Pointer("loop", 15),
       Pointer("forward", 17),
       Pointer("backward", 17),
       Pointer("vertex", 18),
       Pointer("other", 17),
       Pointer("edge", 16),
       Pointer("curve", 1008),
       Pointer("next_at_vx", 17),
       {"sense", FieldType::kCharacter}});
}
}  // namespace openparasolid::internal
