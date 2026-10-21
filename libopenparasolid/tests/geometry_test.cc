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
void Ref(std::vector<uint8_t>& b, uint32_t value, bool delta) {
  CompactIndex(b, value);
  if (delta) b.push_back(value % 2);
}
void Node(std::vector<uint8_t>& b, uint16_t type) {
  U16(b, type);
  b.push_back(255);
  CompactIndex(b, 7);
}
const DecodedNode& First(const SchemaParseReport& r, bool delta) {
  return delta ? r.delta_nodes[0] : r.nodes[0];
}
void CommonGeometry(std::vector<uint8_t>& b, bool delta) {
  U32(b, 123);
  for (uint32_t value : {11u, 12u, 13u, 14u, 15u}) Ref(b, value, delta);
  b.push_back('-');
}
TEST(ParasolidGeometry, FaceAndEdgeKeepEveryFieldInWireOrder) {
  for (bool delta : {false, true})
    for (uint16_t type : {uint16_t{14}, uint16_t{16}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      const auto start = b.size();
      Node(b, type);
      U32(b, 123);
      Ref(b, 11, delta);
      F64(b, .125);
      if (type == 14) {
        for (uint32_t v : {12u, 13u, 14u, 15u, 16u}) Ref(b, v, delta);
        b.push_back('+');
        for (uint32_t v : {17u, 18u, 19u, 20u, 21u}) Ref(b, v, delta);
      } else
        for (uint32_t v : {12u, 13u, 14u, 15u, 16u, 17u, 18u}) Ref(b, v, delta);
      const auto end = b.size();
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete) << type;
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(n.offset, start);
      EXPECT_EQ(n.size, end - start);
      EXPECT_EQ(n.fields.size(), type == 14 ? 14 : 10);
      EXPECT_EQ(std::get<double>(n.fields[2].values[0]), .125);
      if (type == 14) EXPECT_EQ(std::get<uint64_t>(n.fields[8].values[0]), '+');
      EXPECT_EQ(std::get<uint64_t>(n.fields.back().values[0]),
                type == 14 ? 21 : 18);
      EXPECT_EQ(n.fields[1].reference_statuses.size(), delta ? 1 : 0);
    }
}
TEST(ParasolidGeometry, BodyMeshFieldComesFromEmbeddedInsertion) {
  auto b = DeltaPrefixBytes();
  U16(b, 12);
  b.push_back(24);
  b.insert(b.end(), 6, 'C');
  b.push_back('I');
  b.insert(b.end(), {4, 'm', 'e', 's', 'h'});
  U16(b, 1006);
  CompactIndex(b, 0);
  b.insert(b.end(), 17, 'C');
  b.push_back('Z');
  CompactIndex(b, 7);
  U32(b, 123);
  for (uint32_t v : {11u, 12u, 13u, 14u, 15u, 16u, 17u}) Ref(b, v, true);
  F64(b, .25);
  F64(b, .5);
  for (uint32_t v : {18u, 19u, 20u}) Ref(b, v, true);
  b.push_back(3);
  Ref(b, 21, true);
  b.push_back(1);
  b.push_back(2);
  for (uint32_t v : {22u, 23u, 24u, 25u, 26u, 27u, 28u}) Ref(b, v, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  const auto& definition = r.value().schemas[n.schema_index];
  EXPECT_EQ(definition.source, SchemaSource::kEdited);
  ASSERT_EQ(definition.definition.fields.size(), 24);
  EXPECT_EQ(definition.definition.fields[6].name, "mesh");
  EXPECT_EQ(std::get<uint64_t>(n.fields[6].values[0]), 16);
  EXPECT_EQ(std::get<double>(n.fields[8].values[0]), .25);
  EXPECT_EQ(std::get<uint64_t>(n.fields[15].values[0]), 1);
}
TEST(ParasolidGeometry, ShellRegionAndVertexPreserveOwnerFields) {
  for (uint16_t type : {uint16_t{13}, uint16_t{18}, uint16_t{19}}) {
    auto b = DeltaPrefixBytes();
    Node(b, type);
    U32(b, 123);
    const unsigned pointers = type == 13 ? 8 : 5;
    for (unsigned i = 0; i < pointers; ++i) Ref(b, 11 + i, true);
    if (type == 18) {
      F64(b, .0625);
      Ref(b, 99, true);
    }
    if (type == 19) b.push_back('s');
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete) << type;
    const auto& n = r.value().delta_nodes[0];
    EXPECT_EQ(n.fields.size(), type == 13 ? 9 : type == 18 ? 8 : 7);
    if (type == 18) EXPECT_EQ(std::get<double>(n.fields[6].values[0]), .0625);
    if (type == 19) EXPECT_EQ(std::get<uint64_t>(n.fields[6].values[0]), 's');
  }
}
TEST(ParasolidGeometry, PointHasFourReferencesAndThreeCoordinates) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    Node(b, 29);
    U32(b, 123);
    for (uint32_t v : {11u, 12u, 13u, 14u}) Ref(b, v, delta);
    for (double v : {1.25, -2.5, 3.75}) F64(b, v);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    EXPECT_EQ(n.fields[5].components, 3);
    EXPECT_EQ(std::get<double>(n.fields[5].values[1]), -2.5);
    EXPECT_EQ(
        r.value().schemas[n.schema_index].definition.fields[2].pointer_class,
        1011);
  }
}
TEST(ParasolidGeometry, AnalyticCoefficientGroupsFollowTheirDeclaredShapes) {
  const std::vector<std::pair<uint16_t, std::vector<uint8_t>>> cases{
      {30, {3, 3}},       {31, {3, 3, 3, 1}},   {32, {3, 3, 3, 1, 1}},
      {50, {3, 3, 3}},    {51, {3, 3, 1, 3}},   {52, {3, 3, 1, 1, 1, 3}},
      {53, {3, 1, 3, 3}}, {54, {3, 3, 1, 1, 3}}};
  for (bool delta : {false, true})
    for (const auto& [type, components] : cases) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Node(b, type);
      CommonGeometry(b, delta);
      double next = .25;
      for (auto size : components)
        for (unsigned i = 0; i < size; ++i) {
          F64(b, next);
          next += 1.;
        }
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete) << type;
      const auto& n = First(r.value(), delta);
      ASSERT_EQ(n.fields.size(), 7 + components.size());
      next = .25;
      for (std::size_t i = 0; i < components.size(); ++i) {
        EXPECT_EQ(n.fields[7 + i].components, components[i]) << type;
        for (const auto& v : n.fields[7 + i].values) {
          EXPECT_EQ(std::get<double>(v), next) << type;
          next += 1.;
        }
      }
      EXPECT_EQ(
          r.value().schemas[n.schema_index].definition.fields[2].pointer_class,
          type >= 50 ? 1007 : 1010);
    }
}
TEST(ParasolidGeometry,
     IntersectionFixedPointerArrayRetainsIndependentStatuses) {
  auto b = DeltaPrefixBytes();
  Node(b, 38);
  CommonGeometry(b, true);
  for (uint32_t v : {21u, 22u, 23u, 24u, 25u}) Ref(b, v, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& n = r.value().delta_nodes[0];
  EXPECT_EQ(n.fields[7].elements, 2);
  EXPECT_EQ(n.fields[7].reference_statuses, (std::vector<uint8_t>{1, 0}));
  EXPECT_EQ(std::get<uint64_t>(n.fields.back().values[0]), 25);
}
TEST(ParasolidGeometry, ChartAndLimitVariableHvecFollowStreamEncoding) {
  for (bool delta : {false, true})
    for (uint16_t type : {uint16_t{40}, uint16_t{41}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      U16(b, type);
      b.push_back(255);
      U32(b, 2);
      CompactIndex(b, 7);
      if (type == 40) {
        F64(b, .25);
        F64(b, 1.25);
        U32(b, 2);
        F64(b, 2.25);
        F64(b, 3.25);
        F64(b, 4.25);
        F64(b, 5.25);
      } else
        b.push_back(2);
      const unsigned count = delta ? 22 : 6;
      for (unsigned i = 0; i < count; ++i) F64(b, i + .5);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete) << type;
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(n.fields.back().components, delta ? 11 : 3);
      EXPECT_EQ(n.fields.back().values.size(), count);
      EXPECT_EQ(std::get<double>(n.fields.back().values.back()), count - .5);
    }
}
TEST(ParasolidGeometry, VectorValuesAndGeometryOwnerAreDistinctFamilies) {
  auto b = DeltaPrefixBytes();
  U16(b, 86);
  b.push_back(255);
  U32(b, 2);
  CompactIndex(b, 7);
  for (int i = 0; i < 6; ++i) F64(b, i + .5);
  U16(b, 141);
  b.push_back(255);
  CompactIndex(b, 8);
  for (uint32_t v : {1u, 2u, 3u, 4u}) Ref(b, v, true);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_nodes.size(), 2);
  EXPECT_EQ(r.value().delta_nodes[0].fields[0].values.size(), 6);
  EXPECT_EQ(r.value().delta_nodes[1].fields.size(), 4);
}
TEST(ParasolidGeometry, EveryTruncatedFaceKeepsTheNodeUninterpreted) {
  auto b = DeltaPrefixBytes();
  const auto start = b.size();
  Node(b, 14);
  U32(b, 1);
  Ref(b, 0, true);
  Marker(b);
  for (int i = 0; i < 5; ++i) Ref(b, 0, true);
  b.push_back('+');
  for (int i = 0; i < 5; ++i) Ref(b, 0, true);
  for (std::size_t n = start; n < b.size(); ++n) {
    const auto r = SchemaParser::Parse(std::span(b).first(n));
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_TRUE(r.value().delta_nodes.empty()) << n;
    EXPECT_EQ(r.value().decoded_end, start);
  }
}
}  // namespace
}  // namespace openparasolid
