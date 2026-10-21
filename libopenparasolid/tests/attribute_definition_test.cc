// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void Ref(std::vector<uint8_t>& b, uint32_t ref, bool delta) {
  CompactIndex(b, ref);
  if (delta) b.push_back(ref % 2);
}
void Begin(std::vector<uint8_t>& b, uint16_t type, uint32_t count,
           uint32_t index = 7) {
  U16(b, type);
  b.push_back(255);
  U32(b, count);
  CompactIndex(b, index);
}
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  CompactIndex(b, 0);
}
const DecodedNode& First(const SchemaParseReport& r, bool delta) {
  return delta ? r.delta_nodes.at(0) : r.nodes.at(0);
}
void DefinitionFields(std::vector<uint8_t>& b, bool delta, uint32_t count,
                      uint8_t owner = 1) {
  Ref(b, 0, delta);
  Ref(b, 8, delta);
  U32(b, 0xffff8004);
  for (uint8_t action : {0, 1, 2, 3, 128, 253, 254, 255}) b.push_back(action);
  Ref(b, 32767, delta);
  for (int i = 0; i < 14; ++i) b.push_back(i == 13 ? owner : i % 2);
  for (uint32_t i = 0; i < count; ++i) b.push_back(static_cast<uint8_t>(i));
}
TEST(ParasolidAttributeDefinition, FixedAndVariableArraysKeepDistinctWidths) {
  for (bool delta : {false, true})
    for (uint32_t count : {0u, 1u, 3u}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Begin(b, 80, count);
      DefinitionFields(b, delta, count);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(n.variable_length, count);
      ASSERT_EQ(n.fields.size(), 7);  // Nontransmitted callbacks are excluded.
      EXPECT_EQ(n.fields[3].elements, 8);
      EXPECT_EQ(n.fields[3].values.back(), ScalarValue(uint64_t{255}));
      EXPECT_EQ(n.fields[4].values[0], ScalarValue(uint64_t{32767}));
      EXPECT_EQ(n.fields[4].size, delta ? 5 : 4);
      EXPECT_EQ(n.fields[4].reference_statuses,
                (delta ? std::vector<uint8_t>{1} : std::vector<uint8_t>{}));
      EXPECT_EQ(n.fields[5].elements, 14);
      EXPECT_EQ(n.fields[5].values.back(), ScalarValue(true));
      EXPECT_EQ(n.fields[6].elements, count);
      EXPECT_EQ(n.fields[6].values.size(), count);
      if (delta)
        EXPECT_EQ(n.fields[2].values[0], ScalarValue(uint64_t{0xffff8004}));
      else
        EXPECT_TRUE(
            std::holds_alternative<std::monostate>(n.fields[2].values[0]));
      const auto& fields = r.value().schemas[n.schema_index].definition.fields;
      EXPECT_EQ(fields[0].pointer_class, 80);
      EXPECT_EQ(fields[1].pointer_class, 79);
      EXPECT_EQ(fields[4].pointer_class, 99);
    }
}
TEST(ParasolidAttributeDefinition, IdentifierKeepsRawBytesIncludingNull) {
  for (bool delta : {false, true})
    for (uint32_t count : {0u, 1u, 4u}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Begin(b, 79, count);
      const std::vector<uint8_t> text{'A', 0, 128, 255};
      b.insert(b.end(), text.begin(), text.begin() + count);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      ASSERT_EQ(n.fields.size(), 1);
      EXPECT_EQ(n.fields[0].values.size(), count);
      for (uint32_t i = 0; i < count; ++i)
        EXPECT_EQ(n.fields[0].values[i], ScalarValue(uint64_t{text[i]}));
    }
}
TEST(ParasolidAttributeDefinition, FieldNamesRetainEveryReferenceAndStatus) {
  for (bool delta : {false, true})
    for (uint32_t count : {0u, 1u, 3u}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Begin(b, 99, count);
      const std::vector<uint32_t> refs{0, 32767, 8};
      for (uint32_t i = 0; i < count; ++i) Ref(b, refs[i], delta);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      ASSERT_EQ(n.fields.size(), 1);
      EXPECT_EQ(n.fields[0].values.size(), count);
      EXPECT_EQ(n.fields[0].reference_statuses.size(), delta ? count : 0);
      for (uint32_t i = 0; i < count; ++i) {
        EXPECT_EQ(n.fields[0].values[i], ScalarValue(uint64_t{refs[i]}));
        if (delta) EXPECT_EQ(n.fields[0].reference_statuses[i], refs[i] % 2);
      }
      EXPECT_EQ(
          r.value().schemas[n.schema_index].definition.fields[0].pointer_class,
          1037);
    }
}
TEST(ParasolidAttributeDefinition, RejectsInvalidLogicalAndEveryTruncation) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    const auto start = b.size();
    Begin(b, 80, 3);
    DefinitionFields(b, delta, 3);
    const auto end = b.size();
    for (auto length = start; length < end; ++length) {
      const auto r = SchemaParser::Parse(std::span(b).first(length));
      ASSERT_TRUE(r.ok());
      EXPECT_FALSE(r.value().complete);
      EXPECT_EQ(r.value().decoded_end, start);
      EXPECT_TRUE(r.value().delta_nodes.empty());
      EXPECT_EQ(r.value().nodes.size(), delta ? 2 : 0);
    }
    b.resize(start);
    Begin(b, 80, 0);
    DefinitionFields(b, delta, 0, 2);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_EQ(r.value().decoded_end, start);
    EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidAttributeDefinition,
     EmbeddedInsertionDoesNotChangeBaseArraySize) {
  auto b = DeltaPrefixBytes();
  U16(b, 80);
  b.push_back(8);
  b.insert(b.end(), 6, 'C');
  b.push_back('I');
  b.insert(b.end(), {4, 'f', 'l', 'a', 'g'});
  U16(b, 0);
  CompactIndex(b, 0);
  b.insert(b.end(), {1, 'u'});
  b.push_back('C');
  b.push_back('Z');
  U32(b, 2);
  CompactIndex(b, 7);
  DefinitionFields(b, true, 0);
  b.insert(b.end(), {255, 17, 18});
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  ASSERT_EQ(n.fields.size(), 8);
  EXPECT_EQ(n.fields[5].values.size(), 14);
  EXPECT_EQ(n.fields[6].values[0], ScalarValue(uint64_t{255}));
  EXPECT_EQ(n.fields[7].values,
            (std::vector<ScalarValue>{uint64_t{17}, uint64_t{18}}));
  EXPECT_EQ(r.value().schemas[n.schema_index].source, SchemaSource::kEdited);
}
TEST(ParasolidAttributeDefinition, CountsRespectValueLimits) {
  for (uint16_t type : {uint16_t{79}, uint16_t{80}, uint16_t{99}}) {
    auto b = Header();
    const auto start = b.size();
    Begin(b, type, 1000000);
    if (type == 80) DefinitionFields(b, false, 0);
    SchemaLimits limits;
    limits.max_values = 32;
    const auto r = SchemaParser::Parse(b, nullptr, limits);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_EQ(r.value().decoded_end, start);
    EXPECT_TRUE(r.value().nodes.empty());
    EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
  }
}
}  // namespace
}  // namespace openparasolid
