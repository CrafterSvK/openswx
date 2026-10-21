// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#pragma once
#include <span>
#include <string_view>

#include "openparasolid/model.h"

namespace openparasolid::internal {
class ModelFields {
 public:
  ModelFields(const DecodedNode& node, const TypeDefinition& schema)
      : node_(node), schema_(schema) {}
  const DecodedField* Find(std::string_view name, FieldType type) const;
  bool Number(std::string_view name, double& value);
  bool Vector(std::string_view name, Vector3& value);
  bool Integer(std::string_view name, uint32_t& value);
  bool Byte(std::string_view name, uint8_t& value,
            FieldType type = FieldType::kByte);
  bool Logical(std::string_view name, bool& value);
  bool Reference(std::string_view name, NodeId& value);
  [[nodiscard]] ParseError Error() const;

 private:
  bool Invalid(std::string_view name);
  const DecodedNode& node_;
  const TypeDefinition& schema_;
  std::string invalid_field_;
};
struct GeometryContext {
  const SchemaParseReport& report;
  const std::unordered_map<uint32_t, std::size_t>& positions;
  uint64_t remaining_values;
  const DecodedNode* Resolve(NodeId id, uint16_t type) const;
  ModelFields Fields(const DecodedNode& node) const;
};
struct BuiltGeometry {
  GeometryPayload payload;
  uint64_t copied_values = 0;
};
Result<BuiltGeometry> BuildAnalytic(const DecodedNode& node,
                                    ModelFields& fields);
Result<BuiltGeometry> BuildSpline(const DecodedNode& node, ModelFields& fields,
                                  const GeometryContext& context);
bool IsGeometryType(uint16_t type);
class ModelAssembler {
 public:
  ModelAssembler(const SchemaParseReport& report, const ModelOptions& options)
      : report_(report), options_(options) {}
  Result<Model> Build();

 private:
  std::optional<ParseError> ValidateAndIndex();
  void LinkReferences();
  std::optional<ParseError> BuildGeometry();
  void Issue(ModelIssueCode code, const DecodedNode& node, std::string field,
             std::string message, uint64_t offset);
  const SchemaParseReport& report_;
  const ModelOptions& options_;
  Model model_;
};
}  // namespace openparasolid::internal
