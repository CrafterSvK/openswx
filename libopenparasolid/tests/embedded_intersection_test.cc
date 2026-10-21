// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <cmath>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void Text(std::vector<uint8_t>& b, std::string_view text) {
  b.push_back(static_cast<uint8_t>(text.size()));
  b.insert(b.end(), text.begin(), text.end());
}
void Field(std::vector<uint8_t>& b, const char* name, char type,
           uint32_t extent = 0, bool transmitted = true) {
  Text(b, name);
  U16(b, 0);
  CompactIndex(b, extent);
  Text(b, std::string(1, type));
  if (extent == 1) b.push_back(transmitted);
}
void Definition(std::vector<uint8_t>& b, bool transmitted = true) {
  U16(b, 204);
  b.push_back(2);
  Text(b, "INTERSECTION_DATA");
  Text(b, "Intersection data");
  Field(b, "uv_type", 'u');
  Field(b, "values", 'f', 1, transmitted);
}
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  CompactIndex(b, 0);
}
const DecodedNode& First(const SchemaParseReport& r, bool delta) {
  return delta ? r.delta_nodes.at(0) : r.nodes.at(0);
}
TEST(ParasolidEmbeddedIntersection,
     FullDefinitionAndRepeatedNodesUseStreamLayout) {
  for (bool delta : {false, true})
    for (uint32_t count : {0u, 1u, 4u}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      const auto start = b.size();
      Definition(b);
      U32(b, count);
      CompactIndex(b, 32767);
      b.push_back(255);  // Preserve the code without inferring an enum range.
      if (count) Marker(b);
      if (count == 4) {
        F64(b, -0.);
        F64(b, 1.25);
        F64(b, -2.5);
      }
      const auto next = b.size();
      U16(b, 204);  // No schema metadata at subsequent occurrences.
      U32(b, 1);
      CompactIndex(b, 8);
      b.push_back(4);
      F64(b, 42.5);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(n.offset, start);
      EXPECT_EQ(n.offset + n.size, next);
      EXPECT_EQ(n.index, 32767);
      EXPECT_EQ(n.variable_length, count);
      ASSERT_EQ(n.fields.size(), 2);
      EXPECT_EQ(n.fields[0].values[0], ScalarValue(uint64_t{255}));
      EXPECT_EQ(n.fields[1].values.size(), count);
      if (count)
        EXPECT_TRUE(
            std::holds_alternative<std::monostate>(n.fields[1].values[0]));
      if (count == 4) {
        EXPECT_TRUE(std::signbit(std::get<double>(n.fields[1].values[1])));
        EXPECT_EQ(n.fields[1].values[2], ScalarValue(1.25));
      }
      const auto& schema = r.value().schemas[n.schema_index];
      EXPECT_EQ(schema.source, SchemaSource::kFull);
      EXPECT_EQ(schema.definition.name, "INTERSECTION_DATA");
      EXPECT_EQ(schema.definition.fields[1].type, FieldType::kDouble);
      const auto& nodes = delta ? r.value().delta_nodes : r.value().nodes;
      ASSERT_EQ(nodes.size(), 2);
      EXPECT_EQ(nodes[1].schema_index, n.schema_index);
      EXPECT_EQ(nodes[1].fields[1].values[0], ScalarValue(42.5));
    }
}
TEST(ParasolidEmbeddedIntersection,
     DoesNotHardcodeNameFieldsOrPointerStatuses) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    U16(b, 204);
    b.push_back(1);
    Text(b, "SYNTHETIC_EXTENSION");
    Text(b, "");
    Text(b, "references");
    U16(b, 29);
    CompactIndex(b, 2);
    CompactIndex(b, 7);
    CompactIndex(b, 0);
    if (delta) b.push_back(0);
    CompactIndex(b, 32767);
    if (delta) b.push_back(1);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    EXPECT_FALSE(n.variable_length.has_value());
    ASSERT_EQ(n.fields.size(), 1);
    EXPECT_EQ(n.fields[0].values,
              (std::vector<ScalarValue>{uint64_t{0}, uint64_t{32767}}));
    EXPECT_EQ(n.fields[0].reference_statuses,
              (delta ? std::vector<uint8_t>{0, 1} : std::vector<uint8_t>{}));
    EXPECT_EQ(
        r.value().schemas[n.schema_index].definition.fields[0].pointer_class,
        29);
  }
}
TEST(ParasolidEmbeddedIntersection,
     CallerCatalogAndUnknownMembershipRemainAuthoritative) {
  auto b = Header();
  const auto start = b.size();
  Definition(b);
  U32(b, 0);
  CompactIndex(b, 7);
  b.push_back(4);
  End(b);
  SchemaCatalog catalog{"SCH_1234567_12345_13006", "caller", {}, {}};
  auto r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
  // A full declaration alone does not establish absence for other types.
  b[start + 1] = 200;
  r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
}
TEST(ParasolidEmbeddedIntersection, AbsenceIsSpecificToBase13006) {
  auto b = Header("", "SCH_1_2_13007");
  U16(b, 204);
  U32(b, 0);
  const auto start = b.size();
  Definition(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
}
TEST(ParasolidEmbeddedIntersection, RejectsUnchangedMarkerWithoutABase) {
  auto b = Header();
  const auto start = b.size();
  U16(b, 204);
  b.push_back(255);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidSchema);
  const std::vector<uint8_t> marker{255};
  const auto definition = SchemaParser::ReadDefinition(marker, 0, 204, nullptr);
  ASSERT_FALSE(definition.ok());
  EXPECT_EQ(definition.error().code, ErrorCode::kInvalidSchema);
  EXPECT_EQ(definition.error().offset, 0);
}
TEST(ParasolidEmbeddedIntersection,
     TruncationAndValueLimitKeepNodeUncommitted) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    const auto start = b.size();
    Definition(b);
    U32(b, 2);
    CompactIndex(b, 7);
    b.push_back(4);
    F64(b, .25);
    F64(b, -.5);
    for (auto n = start; n < b.size(); ++n) {
      const auto r = SchemaParser::Parse(std::span(b).first(n));
      ASSERT_TRUE(r.ok());
      EXPECT_FALSE(r.value().complete);
      EXPECT_EQ(r.value().decoded_end, start);
      EXPECT_TRUE(r.value().delta_nodes.empty());
      EXPECT_EQ(r.value().nodes.size(), delta ? 2 : 0);
    }
  }
  auto b = Header();
  const auto start = b.size();
  Definition(b);
  U32(b, 1000000);
  CompactIndex(b, 7);
  b.push_back(4);
  SchemaLimits limits;
  limits.max_values = 16;
  const auto r = SchemaParser::Parse(b, nullptr, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
}
TEST(ParasolidEmbeddedIntersection, NontransmittedArrayConsumesNoValues) {
  auto b = Header();
  Definition(b, false);
  U32(b, 1000000);
  CompactIndex(b, 7);
  b.push_back(4);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  EXPECT_EQ(r.value().nodes[0].fields[1].elements, 0);
  EXPECT_TRUE(r.value().nodes[0].fields[1].values.empty());
  EXPECT_FALSE(r.value().schemas[0].definition.fields[1].transmitted);
}
}  // namespace
}  // namespace openparasolid
