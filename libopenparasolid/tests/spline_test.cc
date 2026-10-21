// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  CompactIndex(b, 0);
}
void Ref(std::vector<uint8_t>& b, uint32_t index, bool delta) {
  CompactIndex(b, index);
  if (delta) b.push_back(index % 2);
}
void Node(std::vector<uint8_t>& b, uint16_t type) {
  U16(b, type);
  b.push_back(255);
  CompactIndex(b, 7);
}
void ArrayNode(std::vector<uint8_t>& b, uint16_t type, uint32_t count) {
  U16(b, type);
  b.push_back(255);
  U32(b, count);
  CompactIndex(b, 7);
}
const DecodedNode& First(const SchemaParseReport& r, bool delta) {
  return delta ? r.delta_nodes[0] : r.nodes[0];
}
void Common(std::vector<uint8_t>& b, bool delta) {
  U32(b, 123);
  for (uint32_t i = 11; i < 16; ++i) Ref(b, i, delta);
  b.push_back('+');
}
TEST(ParasolidSpline, ListUsesTwelveFieldsAndFourByteListType) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    Node(b, 70);
    U32(b, 123);
    Ref(b, 11, delta);
    Ref(b, 12, delta);
    Ref(b, 13, delta);
    for (uint32_t word : {0x01020304u, 20u, 20u, 4u}) U32(b, word);
    Ref(b, 14, delta);
    Ref(b, 15, delta);
    U32(b, 9);
    b.push_back(1);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 12);
    EXPECT_EQ(n.fields[4].size, 4);
    EXPECT_EQ(n.fields[8].reference_statuses.size(), delta ? 1 : 0);
    EXPECT_EQ(std::get<bool>(n.fields[11].values[0]), true);
    if (delta)
      EXPECT_EQ(std::get<uint64_t>(n.fields[4].values[0]), 0x01020304u);
    else
      EXPECT_EQ(std::get<int64_t>(n.fields[4].values[0]), 0x01020304);
  }
}
TEST(ParasolidSpline, ListEditCanDropTheUntransmittedBaseTailAtEnd) {
  auto b = DeltaPrefixBytes();
  U16(b, 70);
  b.push_back(9);
  b.insert(b.end(), 9, 'C');
  b.push_back('Z');
  CompactIndex(b, 7);
  U32(b, 123);
  for (uint32_t i : {11u, 12u, 13u}) Ref(b, i, true);
  for (uint32_t i : {1u, 2u, 3u, 4u}) U32(b, i);
  Ref(b, 14, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  EXPECT_EQ(r.value().delta_nodes[0].fields.size(), 9);
}
TEST(ParasolidSpline, PointerBlockInsertionPrecedesTheNextBlockPointer) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    U16(b, 74);
    b.insert(b.end(), {4, 'C', 'I'});
    const std::string name = "index_map_offset";
    b.push_back(static_cast<uint8_t>(name.size()));
    b.insert(b.end(), name.begin(), name.end());
    U16(b, 0);
    CompactIndex(b, 0);
    b.insert(b.end(), {1, 'd', 'C', 'C', 'Z'});
    U32(b, 4);
    CompactIndex(b, 7);
    U32(b, 2);
    U32(b, 0);
    Ref(b, 0, delta);
    for (uint32_t i : {50000u, 8u, 0u, 0u}) Ref(b, i, delta);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 4);
    EXPECT_EQ(n.variable_length, 4);
    EXPECT_EQ(r.value().schemas[n.schema_index].definition.fields[1].name,
              "index_map_offset");
    ASSERT_EQ(n.fields[3].values.size(), 4);
    EXPECT_EQ(std::get<uint64_t>(n.fields[3].values[0]), 50000);
    EXPECT_EQ(std::get<uint64_t>(n.fields[3].values[2]), 0);
    EXPECT_EQ(n.fields[3].reference_statuses.size(), delta ? 4 : 0);
  }
}
TEST(ParasolidSpline, VariablePayloadsRetainWeightsKnotsAndShortUnsetValues) {
  for (bool delta : {false, true})
    for (uint16_t type : {uint16_t{45}, uint16_t{127}, uint16_t{128}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      ArrayNode(b, type, 4);
      if (type == 127)
        for (uint16_t value :
             {uint16_t{3}, uint16_t{0x8004}, uint16_t{1}, uint16_t{3}})
          U16(b, value);
      else
        for (double value : {2., 4., 6., 2.}) F64(b, value);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& f = First(r.value(), delta).fields[0];
      ASSERT_EQ(f.values.size(), 4);
      EXPECT_TRUE(f.reference_statuses.empty());
      if (type == 127)
        EXPECT_TRUE(std::holds_alternative<std::monostate>(f.values[1]));
      else {
        EXPECT_EQ(std::get<double>(f.values[0]), 2.);
        EXPECT_EQ(std::get<double>(f.values[2]), 6.);
      }
    }
}
TEST(ParasolidSpline, NurbsCurveKeepsDegreesCountsFlagsAndDependencyPointers) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    Node(b, 136);
    U16(b, 3);
    U32(b, 5);
    U16(b, 4);
    U32(b, 9);
    b.insert(b.end(), {2, 0, 1, 1, 7});
    for (uint32_t i : {11u, 12u, 13u}) Ref(b, i, delta);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 12);
    EXPECT_EQ(std::get<int64_t>(n.fields[0].values[0]), 3);
    EXPECT_EQ(std::get<int64_t>(n.fields[2].values[0]), 4);
    EXPECT_EQ(std::get<bool>(n.fields[7].values[0]), true);
    EXPECT_EQ(std::get<uint64_t>(n.fields[9].values[0]), 11);
    EXPECT_EQ(
        r.value().schemas[n.schema_index].definition.fields[11].pointer_class,
        128);
  }
}
TEST(ParasolidSpline, NurbsSurfaceKeepsIndependentUAndVDefinition) {
  auto b = DeltaPrefixBytes();
  Node(b, 126);
  b.insert(b.end(), {0, 1});
  U16(b, 2);
  U16(b, 3);
  U32(b, 4);
  U32(b, 5);
  b.insert(b.end(), {1, 2});
  U32(b, 7);
  U32(b, 9);
  b.insert(b.end(), {1, 0, 1, 2});
  U16(b, 4);
  for (uint32_t i = 11; i < 16; ++i) Ref(b, i, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  ASSERT_EQ(n.fields.size(), 20);
  EXPECT_EQ(std::get<int64_t>(n.fields[2].values[0]), 2);
  EXPECT_EQ(std::get<int64_t>(n.fields[3].values[0]), 3);
  EXPECT_EQ(std::get<uint64_t>(n.fields[4].values[0]), 4);
  EXPECT_EQ(std::get<uint64_t>(n.fields[5].values[0]), 5);
  EXPECT_EQ(std::get<bool>(n.fields[10].values[0]), true);
  EXPECT_EQ(std::get<uint64_t>(n.fields[19].values[0]), 15);
}
TEST(ParasolidSpline,
     SurfaceDataHasFourIntervalsTwelveCharactersAndFourReferences) {
  auto b = DeltaPrefixBytes();
  Node(b, 125);
  for (int i = 0; i < 8; ++i) F64(b, i + .5);
  b.push_back(1);
  for (int i = 0; i < 12; ++i) b.push_back(static_cast<uint8_t>('A' + i));
  for (uint32_t i = 11; i < 15; ++i) Ref(b, i, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  ASSERT_EQ(n.fields.size(), 21);
  EXPECT_EQ(n.fields[3].components, 2);
  EXPECT_EQ(std::get<double>(n.fields[3].values[1]), 7.5);
  EXPECT_EQ(std::get<uint64_t>(n.fields[16].values[0]), 'L');
  EXPECT_EQ(std::get<uint64_t>(n.fields[20].values[0]), 14);
}
TEST(ParasolidSpline, CurveWrappersRetainDistinctTailLayouts) {
  for (uint16_t type :
       {uint16_t{124}, uint16_t{133}, uint16_t{134}, uint16_t{137}}) {
    auto b = DeltaPrefixBytes();
    Node(b, type);
    Common(b, true);
    if (type == 133) {
      Ref(b, 21, true);
      for (int i = 0; i < 8; ++i) F64(b, i + .5);
    } else if (type == 137) {
      for (uint32_t i : {21u, 22u, 23u}) Ref(b, i, true);
      F64(b, .125);
    } else {
      Ref(b, 21, true);
      Ref(b, 22, true);
    }
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete) << type;
    const auto& n = r.value().delta_nodes[0];
    EXPECT_EQ(n.fields.size(), type == 133 ? 12 : type == 137 ? 11 : 9);
    if (type == 133) {
      EXPECT_EQ(n.fields[8].components, 3);
      EXPECT_EQ(std::get<double>(n.fields[11].values[0]), 7.5);
    }
    if (type == 137) EXPECT_EQ(std::get<double>(n.fields[10].values[0]), .125);
  }
}
TEST(ParasolidSpline, CurveDataUsesItsFormUnionClass) {
  auto b = DeltaPrefixBytes();
  Node(b, 135);
  b.push_back(2);
  Ref(b, 11, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  EXPECT_EQ(
      r.value().schemas[n.schema_index].definition.fields[1].pointer_class,
      1036);
}
TEST(ParasolidSpline,
     TransformPreservesFixedMatrixAndVectorGroupsWithoutApplyingThem) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    Node(b, 100);
    U32(b, 123);
    for (uint32_t i : {11u, 12u, 13u}) Ref(b, i, delta);
    for (int i = 0; i < 12; ++i) F64(b, i + .5);
    F64(b, 2.);
    U32(b, 7);
    for (double v : {-1., -2., -3.}) F64(b, v);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 9);
    EXPECT_EQ(n.fields[4].elements, 9);
    EXPECT_EQ(n.fields[4].components, 1);
    EXPECT_EQ(std::get<double>(n.fields[5].values[2]), 11.5);
    EXPECT_EQ(std::get<double>(n.fields[6].values[0]), 2.);
    EXPECT_EQ(std::get<double>(n.fields[8].values[2]), -3.);
  }
}
TEST(ParasolidSpline, TruncatedPointerArrayNeverCommitsPartialNode) {
  auto b = DeltaPrefixBytes();
  const auto start = b.size();
  ArrayNode(b, 74, 3);
  U32(b, 2);
  Ref(b, 0, true);
  Ref(b, 50000, true);
  Ref(b, 8, true);
  Ref(b, 0, true);
  for (std::size_t length = start; length < b.size(); ++length) {
    const auto r = SchemaParser::Parse(std::span(b).first(length));
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_TRUE(r.value().delta_nodes.empty()) << length;
    EXPECT_EQ(r.value().decoded_end, start);
  }
}
TEST(ParasolidSpline, HugeControlArrayCountHitsValueLimitBeforeAllocation) {
  auto b = DeltaPrefixBytes();
  ArrayNode(b, 45, 0x7fffffff);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().delta_nodes.empty());
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
