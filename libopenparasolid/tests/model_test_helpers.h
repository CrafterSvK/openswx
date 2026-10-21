// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#pragma once
#include <initializer_list>

#include "openparasolid/model.h"

namespace openparasolid::test {
struct ModelField {
  FieldDefinition definition;
  std::vector<ScalarValue> values;
};
inline ModelField Numbers(const char* name,
                          std::initializer_list<double> values,
                          FieldType type = FieldType::kDouble) {
  ModelField f{{name, type}, {}};
  for (double v : values) f.values.emplace_back(v);
  return f;
}
inline ModelField Integer(const char* name, int64_t value,
                          FieldType type = FieldType::kInteger) {
  return {{name, type}, {value}};
}
inline ModelField ByteValue(const char* name, uint8_t value,
                            FieldType type = FieldType::kByte) {
  return {{name, type}, {uint64_t{value}}};
}
inline ModelField Logical(const char* name, bool value) {
  return {{name, FieldType::kLogical}, {value}};
}
inline ModelField Pointer(const char* name, uint32_t value, uint16_t target) {
  return {{name, FieldType::kPointer, target}, {uint64_t{value}}};
}
inline ModelField Array(const char* name, FieldType type,
                        std::vector<ScalarValue> values) {
  return {{name, type, 0, 1, true}, std::move(values)};
}
class ModelFixture {
 public:
  SchemaParseReport report;
  ModelFixture() {
    report.header.kind = StreamKind::kTransmit;
    report.complete = true;
    Finish();
  }
  void Add(uint16_t type, uint32_t id, std::vector<ModelField> fields) {
    DecodedNode node;
    node.node_type = type;
    node.index = id;
    node.schema_index = report.schemas.size();
    node.offset = report.decoded_end;
    TypeDefinition definition{type, "synthetic", "", {}};
    uint64_t at = node.offset + 4;
    for (auto& field : fields) {
      const auto components = ComponentCount(field.definition.type);
      const auto count =
          static_cast<uint32_t>(field.values.size() / components);
      if (field.definition.elements == 1) node.variable_length = count;
      const uint64_t width =
          field.definition.type == FieldType::kDouble || components > 1 ? 8
          : field.definition.type == FieldType::kInteger                ? 4
          : field.definition.type == FieldType::kShort ||
                  field.definition.type == FieldType::kPointer
              ? 2
              : 1;
      const auto size = width * field.values.size();
      node.fields.push_back({definition.fields.size(),
                             at,
                             size,
                             count,
                             components,
                             std::move(field.values),
                             {}});
      definition.fields.push_back(std::move(field.definition));
      at += size;
    }
    node.size = at - node.offset;
    report.decoded_end = at;
    report.schemas.push_back(
        {std::move(definition), SchemaSource::kFull, 0, 0, {}});
    report.nodes.push_back(std::move(node));
    Finish();
  }
  void Point(uint32_t id) {
    Add(29, id, {Numbers("position", {1., -2., 3.}, FieldType::kVector)});
  }
  void Curve(uint32_t id = 1, uint32_t descriptor = 2) {
    Add(134, id,
        {Pointer("nurbs", descriptor, 136),
         ByteValue("sense", '+', FieldType::kCharacter)});
  }
  void Spline(bool rational = false, bool surface = false) {
    if (surface) {
      Add(124, 1,
          {Pointer("nurbs", 2, 126),
           ByteValue("sense", '-', FieldType::kCharacter)});
      Add(126, 2,
          {Integer("u_degree", 1, FieldType::kShort),
           Integer("v_degree", 1, FieldType::kShort),
           Integer("n_u_vertices", 2),
           Integer("n_v_vertices", 2),
           Integer("n_u_knots", 2),
           Integer("n_v_knots", 2),
           Logical("u_periodic", false),
           Logical("v_periodic", false),
           Logical("u_closed", false),
           Logical("v_closed", false),
           ByteValue("u_knot_type", 2),
           ByteValue("v_knot_type", 2),
           ByteValue("surface_form", 1),
           Logical("rational", rational),
           Integer("vertex_dim", rational ? 4 : 3, FieldType::kShort),
           Pointer("bspline_vertices", 3, 45),
           Pointer("u_knots", 4, 128),
           Pointer("v_knots", 4, 128),
           Pointer("u_knot_mult", 5, 127),
           Pointer("v_knot_mult", 5, 127)});
    } else {
      Curve();
      Add(136, 2,
          {Integer("degree", 2, FieldType::kShort), Integer("n_vertices", 3),
           Integer("vertex_dim", rational ? 4 : 3, FieldType::kShort),
           Integer("n_knots", 2), ByteValue("knot_type", 2),
           Logical("periodic", false), Logical("closed", false),
           Logical("rational", rational), ByteValue("curve_form", 1),
           Pointer("bspline_vertices", 3, 45), Pointer("knots", 4, 128),
           Pointer("knot_mult", 5, 127)});
    }
    std::vector<ScalarValue> controls;
    const int count = surface ? 4 : 3;
    for (int i = 0; i < count; ++i) {
      for (double v : {double(i), double(i + 1), double(i + 2)})
        controls.emplace_back(v);
      if (rational) controls.emplace_back(double(i + 1));
    }
    Add(45, 3, {Array("vertices", FieldType::kDouble, std::move(controls))});
    Add(128, 4, {Array("knots", FieldType::kDouble, {0., 1.})});
    const int64_t multiplicity = surface ? 2 : 3;
    Add(127, 5,
        {Array("multiplicities", FieldType::kShort,
               {multiplicity, multiplicity})});
  }

 private:
  void Finish() {
    report.terminator_offset = report.decoded_end;
    report.terminator_size = 4;
    report.byte_size = report.decoded_end + 4;
    report.uninterpreted_offset = report.byte_size;
  }
};
}  // namespace openparasolid::test
