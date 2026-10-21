// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openparasolid/schema.h"

#include <gtest/gtest.h>

#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void Text(std::vector<uint8_t>& b, std::string_view s) {
  b.push_back(static_cast<uint8_t>(s.size()));
  b.insert(b.end(), s.begin(), s.end());
}
void Index(std::vector<uint8_t>& b, uint32_t v) {
  if (v < 32767)
    U16(b, static_cast<uint16_t>(v + 1));
  else {
    U16(b, static_cast<uint16_t>(-static_cast<int32_t>(v % 32767 + 1)));
    U16(b, static_cast<uint16_t>(v / 32767));
  }
}
void Field(std::vector<uint8_t>& b, std::string_view name, char type,
           uint32_t elements = 0, bool transmitted = true) {
  Text(b, name);
  U16(b, 0);
  Index(b, elements);
  Text(b, std::string(1, type));
  if (elements == 1) b.push_back(transmitted);
}
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  Index(b, 0);
}
TypeDefinition Base() {
  return {12,
          "sample",
          "synthetic",
          {{"old", FieldType::kInteger}, {"removed", FieldType::kShort}}};
}
SchemaCatalog Catalog() {
  return {"SCH_1234567_12345_13006", "synthetic", {}, {12}};
}
std::vector<uint8_t> Complete() {
  auto b = Header();
  U16(b, 12);
  b.push_back(3);
  Text(b, "synthetic");
  Text(b, "test");
  Field(b, "ref", 'p');
  Field(b, "unset", 'd');
  Field(b, "coordinates", 'v', 1);
  U32(b, 2);
  Index(b, 1);
  Index(b, 65534);
  U32(b, 0xffff8004);
  for (double value : {1., 2., 3., 4., 5., 6.}) F64(b, value);
  U16(b, 12);
  U32(b, 0);
  Index(b, 2);
  Index(b, 0);
  U32(b, 42);
  End(b);
  return b;
}
TEST(ParasolidSchema, FullDefinitionAndSequentialNodes) {
  auto catalog = Catalog();
  const auto b = Complete();
  const auto result = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(result.ok());
  const auto& r = result.value();
  ASSERT_TRUE(r.complete);
  ASSERT_EQ(r.nodes.size(), 2);
  EXPECT_EQ(r.schemas.size(), 1);
  EXPECT_EQ(r.schemas[0].source, SchemaSource::kFull);
  const auto& node = r.nodes[0];
  EXPECT_EQ(node.index, 1);
  EXPECT_EQ(std::get<uint64_t>(node.fields[0].values[0]), 65534);
  EXPECT_TRUE(std::holds_alternative<std::monostate>(node.fields[1].values[0]));
  EXPECT_EQ(node.fields[2].values.size(), 6);
  EXPECT_EQ(node.fields[2].components, 3);
  EXPECT_EQ(std::get<double>(node.fields[2].values.back()), 6.);
  EXPECT_EQ(node.offset + node.size, r.nodes[1].offset);
  EXPECT_EQ(r.decoded_end, *r.terminator_offset);
  EXPECT_EQ(r.decoded_end + r.terminator_size, b.size());
  EXPECT_EQ(r.uninterpreted_size, 0);
}
TEST(ParasolidSchema, EditAllOperationsAndImplicitTailDrop) {
  const auto base = Base();
  std::vector<uint8_t> b{3, 'I'};
  Field(b, "inserted", 'f');
  b.push_back('C');
  b.push_back('D');
  b.push_back('A');
  Field(b, "appended", 'c');
  b.push_back('Z');
  auto r = SchemaParser::ReadDefinition(b, 0, 12, &base);
  ASSERT_TRUE(r.ok());
  ASSERT_EQ(r.value().definition.fields.size(), 3);
  EXPECT_EQ(r.value().definition.fields[1].name, "old");
  EXPECT_EQ(r.value().edits.size(), 5);
  EXPECT_EQ(r.value().size, b.size());
  b = {1, 'C', 'Z'};
  r = SchemaParser::ReadDefinition(b, 0, 12, &base);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().definition.fields.size(), 1);
}
TEST(ParasolidSchema, RejectsInvalidEditSequences) {
  const auto base = Base();
  for (auto b : {std::vector<uint8_t>{1, 'A'},
                 {0, 'D', 'D', 'I'},
                 {1, 'C', 'C', 'Z'},
                 {2, 'C', 'Z'},
                 {0, 'X'},
                 {0, 'D', 'D', 'C'},
                 {0, 'D', 'D', 'D'}})
    EXPECT_FALSE(SchemaParser::ReadDefinition(b, 0, 12, &base).ok());
}
TEST(ParasolidSchema, UnchangedDefinitionAndStandardSchema) {
  const auto base = Base();
  const std::vector<uint8_t> b{255};
  const auto r = SchemaParser::ReadDefinition(b, 0, 12, &base);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().source, SchemaSource::kUnchanged);
  auto bytes = Header("", "SCH_1300000_13006");
  U16(bytes, 12);
  Index(bytes, 1);
  U32(bytes, 5);
  U16(bytes, 7);
  End(bytes);
  SchemaCatalog catalog{"SCH_1300000_13006", "synthetic", {base}, {}};
  const auto parsed = SchemaParser::Parse(bytes, &catalog);
  ASSERT_TRUE(parsed.ok());
  EXPECT_TRUE(parsed.value().complete);
  EXPECT_EQ(parsed.value().schemas[0].size, 0);
}
TEST(ParasolidSchema, StopsAtUnknownBaseMembership) {
  auto b = Header();
  const auto offset = b.size();
  U16(b, 200);
  b.push_back(255);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().nodes.empty());
  EXPECT_EQ(r.value().uninterpreted_offset, offset);
  ASSERT_EQ(r.value().diagnostics.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
  EXPECT_EQ(r.value().diagnostics[0].node_type, 200);
}
TEST(ParasolidSchema, EveryTruncationIsReported) {
  const auto b = Complete();
  const auto catalog = Catalog();
  for (std::size_t length = 0; length < b.size(); ++length) {
    const auto r = SchemaParser::Parse(std::span(b).first(length), &catalog);
    if (r.ok()) {
      EXPECT_FALSE(r.value().complete) << length;
      EXPECT_EQ(r.value().diagnostics.size(), 1) << length;
      EXPECT_LE(r.value().decoded_end, length);
      EXPECT_EQ(r.value().uninterpreted_offset + r.value().uninterpreted_size,
                length);
    }
  }
}
TEST(ParasolidSchema, RejectsCatalogMismatchAndConflictingMembership) {
  auto catalog = Catalog();
  catalog.schema_key += "0";
  EXPECT_FALSE(SchemaParser::Parse(Complete(), &catalog).ok());
  catalog = Catalog();
  catalog.base_types.push_back(Base());
  EXPECT_FALSE(SchemaParser::Parse(Complete(), &catalog).ok());
}
TEST(ParasolidSchema, RejectsInvalidArrayDefinitions) {
  std::vector<uint8_t> b{2};
  Text(b, "sample");
  Text(b, "");
  Field(b, "variable", 'd', 1);
  Field(b, "after", 'u');
  EXPECT_FALSE(SchemaParser::ReadDefinition(b, 0, 12, nullptr).ok());
  b = {1};
  Text(b, "sample");
  Text(b, "");
  Field(b, "bad", 'x');
  EXPECT_FALSE(SchemaParser::ReadDefinition(b, 0, 12, nullptr).ok());
}
TEST(ParasolidSchema, NontransmittedVariableAndSentinels) {
  auto b = Header();
  U16(b, 12);
  b.push_back(4);
  Text(b, "sample");
  Text(b, "");
  Field(b, "n", 'n');
  Field(b, "f", 'f');
  Field(b, "fixed", 'u', 2);
  Field(b, "opaque", 'q', 1, false);
  U32(b, 123);
  Index(b, 1);
  U16(b, 0x8004);
  Marker(b);
  b.insert(b.end(), {7, 8});
  End(b);
  const auto catalog = Catalog();
  const auto r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& fields = r.value().nodes[0].fields;
  EXPECT_TRUE(std::holds_alternative<std::monostate>(fields[0].values[0]));
  EXPECT_TRUE(std::holds_alternative<std::monostate>(fields[1].values[0]));
  EXPECT_EQ(fields[2].values.size(), 2);
  EXPECT_TRUE(fields[3].values.empty());
  EXPECT_EQ(fields[3].size, 0);
}
TEST(ParasolidSchema, RejectsDuplicateIndicesAndTrailingBytes) {
  const auto catalog = Catalog();
  auto b = Complete();
  b.push_back(0);
  auto r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  b = Header();
  U16(b, 12);
  b.push_back(0);
  Text(b, "empty");
  Text(b, "");
  Index(b, 1);
  U16(b, 12);
  Index(b, 1);
  End(b);
  r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_EQ(r.value().nodes.size(), 1);
}
TEST(ParasolidSchema, ResourceLimitsRetainCompletedPrefix) {
  const auto catalog = Catalog();
  SchemaLimits limits;
  limits.max_nodes = 1;
  auto r = SchemaParser::Parse(Complete(), &catalog, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().nodes.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
  limits = {};
  limits.max_values = 1;
  r = SchemaParser::Parse(Complete(), &catalog, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().nodes.empty());
  limits.max_nodes = 0;
  EXPECT_FALSE(SchemaParser::Parse(Complete(), &catalog, limits).ok());
}
TEST(ParasolidSchema, DefaultWorldCanBeReadWithoutGuessingNextType) {
  auto b = Header();
  U16(b, 101);
  b.push_back(255);
  Index(b, 1);
  for (int i = 0; i < 7; ++i) Index(b, 0);
  b.push_back(1);
  Index(b, 0);
  U32(b, 22);
  U32(b, 23);
  const auto end = b.size();
  U16(b, 200);
  b.push_back(255);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_EQ(r.value().nodes.size(), 1);
  EXPECT_EQ(r.value().decoded_end, end);
  EXPECT_EQ(std::get<int64_t>(r.value().nodes[0].fields.back().values[0]), 23);
  EXPECT_FALSE(r.value().complete);
}
TEST(ParasolidSchema, RejectsInvalidCompactIndicesAndLogical) {
  auto catalog = Catalog();
  for (const std::vector<uint8_t> raw : {std::vector<uint8_t>{0, 0},
                                         {128, 0},
                                         {255, 255, 0, 0},
                                         {255, 255, 255, 255}}) {
    auto b = Header();
    U16(b, 12);
    b.push_back(0);
    Text(b, "empty");
    Text(b, "");
    b.insert(b.end(), raw.begin(), raw.end());
    End(b);
    const auto r = SchemaParser::Parse(b, &catalog);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
  }
  auto b = Header();
  U16(b, 12);
  b.push_back(1);
  Text(b, "sample");
  Text(b, "");
  Field(b, "logical", 'l');
  Index(b, 1);
  b.push_back(2);
  End(b);
  const auto r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
}
TEST(ParasolidSchema, RejectsNonzeroUserFields) {
  auto b = Header();
  b.back() = 1;
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
}
}  // namespace
}  // namespace openparasolid
