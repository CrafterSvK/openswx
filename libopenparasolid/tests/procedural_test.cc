// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void Ref(std::vector<uint8_t>& b, uint32_t value, bool delta) {
  CompactIndex(b, value);
  if (delta) b.push_back(value % 2);
}
void Begin(std::vector<uint8_t>& b, uint16_t type, bool delta) {
  U16(b, type);
  b.push_back(255);
  CompactIndex(b, 7);
  if (type == 163) return;
  U32(b, 123);
  for (uint32_t v : {11u, 12u, 13u, 14u, 15u}) Ref(b, v, delta);
  b.push_back('-');
}
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  CompactIndex(b, 0);
}
const DecodedNode& First(const SchemaParseReport& r, bool delta) {
  return delta ? r.delta_nodes.at(0) : r.nodes.at(0);
}
TEST(ParasolidProcedural, BlendArraysKeepValuesAndIndependentNullStatuses) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    Begin(b, 56, delta);
    b.push_back('R');
    Ref(b, 21, delta);
    Ref(b, 22, delta);
    Ref(b, 23, delta);
    for (double v : {-2.5, 3.5, .25, .75}) F64(b, v);
    Ref(b, 0, delta);
    Ref(b, 25, delta);
    Ref(b, 26, delta);
    Ref(b, 0, delta);
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 15);
    EXPECT_EQ(n.fields[8].values,
              (std::vector<ScalarValue>{uint64_t{21}, uint64_t{22}}));
    EXPECT_EQ(n.fields[8].reference_statuses,
              (delta ? std::vector<uint8_t>{1, 0} : std::vector<uint8_t>{}));
    EXPECT_EQ(n.fields[10].values, (std::vector<ScalarValue>{-2.5, 3.5}));
    EXPECT_EQ(n.fields[11].values, (std::vector<ScalarValue>{.25, .75}));
    EXPECT_EQ(n.fields[12].values,
              (std::vector<ScalarValue>{uint64_t{0}, uint64_t{25}}));
    EXPECT_EQ(n.fields[12].reference_statuses,
              (delta ? std::vector<uint8_t>{0, 1} : std::vector<uint8_t>{}));
  }
}
TEST(ParasolidProcedural, BlendBoundaryUsesSignedNullableShort) {
  for (bool delta : {false, true})
    for (uint16_t value : {uint16_t{0xfffe}, uint16_t{0x8004}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Begin(b, 59, delta);
      U16(b, value);
      Ref(b, 21, delta);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(n.fields[7].size, 2);
      if (value == 0x8004)
        EXPECT_TRUE(
            std::holds_alternative<std::monostate>(n.fields[7].values[0]));
      else
        EXPECT_EQ(std::get<int64_t>(n.fields[7].values[0]), -2);
      EXPECT_EQ(std::get<uint64_t>(n.fields[8].values[0]), 21);
    }
}
TEST(ParasolidProcedural, OffsetPreservesSignLogicalFlagAndUnsetScale) {
  for (bool delta : {false, true})
    for (uint8_t flag : {uint8_t{0}, uint8_t{1}, uint8_t{2}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      const auto start = b.size();
      Begin(b, 60, delta);
      b.push_back('?');
      b.push_back(flag);
      Ref(b, 0, delta);
      F64(b, -1.25);
      Marker(b);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      EXPECT_EQ(r.value().complete, flag < 2);
      if (flag == 2) {
        EXPECT_EQ(r.value().decoded_end, start);
        EXPECT_TRUE(r.value().delta_nodes.empty());
        continue;
      }
      const auto& n = First(r.value(), delta);
      EXPECT_EQ(std::get<bool>(n.fields[8].values[0]), flag == 1);
      EXPECT_EQ(std::get<double>(n.fields[10].values[0]), -1.25);
      EXPECT_TRUE(
          std::holds_alternative<std::monostate>(n.fields[11].values[0]));
    }
}
TEST(ParasolidProcedural, SweptAndSpunRetainVectorAndTrimOrder) {
  for (bool delta : {false, true})
    for (uint16_t type : {uint16_t{67}, uint16_t{68}}) {
      auto b = delta ? DeltaPrefixBytes() : Header();
      Begin(b, type, delta);
      Ref(b, 21, delta);
      const int coordinates = type == 67 ? 3 : 12;
      for (int i = 0; i < coordinates; ++i) F64(b, i + .25);
      if (type == 68) {
        Marker(b);
        F64(b, -3.5);
        for (int i = 0; i < 3; ++i) Marker(b);
      }
      Marker(b);
      End(b);
      const auto r = SchemaParser::Parse(b);
      ASSERT_TRUE(r.ok());
      ASSERT_TRUE(r.value().complete);
      const auto& n = First(r.value(), delta);
      ASSERT_EQ(n.fields.size(), type == 67 ? 10 : 16);
      for (int i = 0; i < coordinates / 3; ++i) {
        EXPECT_EQ(n.fields[8 + i].components, 3);
        EXPECT_EQ(std::get<double>(n.fields[8 + i].values[2]), i * 3 + 2.25);
      }
      if (type == 68) {
        EXPECT_TRUE(
            std::holds_alternative<std::monostate>(n.fields[12].values[0]));
        EXPECT_EQ(std::get<double>(n.fields[13].values[0]), -3.5);
        EXPECT_TRUE(
            std::holds_alternative<std::monostate>(n.fields[14].values[2]));
      }
      EXPECT_TRUE(
          std::holds_alternative<std::monostate>(n.fields.back().values[0]));
    }
}
TEST(ParasolidProcedural, HelixHasNoSurfaceHeaderAndTruncatesAtomically) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    const auto start = b.size();
    Begin(b, 163, delta);
    for (int i = 0; i < 6; ++i) F64(b, i + .5);
    b.push_back('-');
    F64(b, -2.);
    F64(b, 10.);
    F64(b, 5.);
    F64(b, .125);
    F64(b, .001);
    const auto end = b.size();
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    ASSERT_EQ(n.fields.size(), 7);
    EXPECT_EQ(n.size, end - start);
    EXPECT_EQ(n.fields[3].components, 2);
    EXPECT_EQ(n.fields[3].values, (std::vector<ScalarValue>{-2., 10.}));
    EXPECT_EQ(std::get<double>(n.fields[6].values[0]), .001);
    for (std::size_t length = start; length < end; ++length) {
      const auto partial = SchemaParser::Parse(std::span(b).first(length));
      ASSERT_TRUE(partial.ok());
      EXPECT_FALSE(partial.value().complete);
      EXPECT_EQ(partial.value().decoded_end, start);
      EXPECT_TRUE(partial.value().delta_nodes.empty());
      EXPECT_EQ(partial.value().nodes.size(), delta ? 2 : 0);
    }
  }
}
TEST(ParasolidProcedural, CurveHelixRetainsRepresentativePointAndDistinctTail) {
  for (bool delta : {false, true}) {
    auto b = delta ? DeltaPrefixBytes() : Header();
    const auto start = b.size();
    U16(b, 184);
    b.push_back(255);
    CompactIndex(b, 32767);
    for (int i = 0; i < 9; ++i) F64(b, i - .25);
    b.push_back('+');
    F64(b, -2.5);
    F64(b, 10.5);
    F64(b, -1.25);
    Marker(b);
    const auto end = b.size();
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    const auto& n = First(r.value(), delta);
    EXPECT_EQ(n.index, 32767);
    EXPECT_EQ(n.offset, start);
    EXPECT_EQ(n.size, end - start);
    ASSERT_EQ(n.fields.size(), 7);
    EXPECT_EQ(n.fields[2].components, 3);
    EXPECT_EQ(n.fields[2].values, (std::vector<ScalarValue>{5.75, 6.75, 7.75}));
    EXPECT_EQ(n.fields[3].values[0], ScalarValue(uint64_t{'+'}));
    EXPECT_EQ(n.fields[4].values, (std::vector<ScalarValue>{-2.5, 10.5}));
    EXPECT_EQ(n.fields[5].values[0], ScalarValue(-1.25));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(n.fields[6].values[0]));
    const auto& fields = r.value().schemas[n.schema_index].definition.fields;
    EXPECT_EQ(fields[2].name, "point");
    EXPECT_EQ(fields[5].name, "pitch");
    EXPECT_EQ(fields[6].name, "tol");
    for (auto length = start; length < end; ++length) {
      const auto partial = SchemaParser::Parse(std::span(b).first(length));
      ASSERT_TRUE(partial.ok());
      EXPECT_FALSE(partial.value().complete);
      EXPECT_EQ(partial.value().decoded_end, start);
      EXPECT_TRUE(partial.value().delta_nodes.empty());
      EXPECT_EQ(partial.value().nodes.size(), delta ? 2 : 0);
    }
  }
}
TEST(ParasolidProcedural, EmbeddedEditsRemainAuthoritative) {
  auto b = DeltaPrefixBytes();
  U16(b, 67);
  b.push_back(9);
  b.insert(b.end(), 9, 'C');
  b.push_back('D');  // Drop the base scale field.
  b.push_back('Z');
  CompactIndex(b, 7);
  U32(b, 123);
  for (uint32_t v : {11u, 12u, 13u, 14u, 15u}) Ref(b, v, true);
  b.push_back('-');
  Ref(b, 21, true);
  for (int i = 0; i < 3; ++i) F64(b, i + .25);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_nodes[0].fields.size(), 9);
  EXPECT_EQ(r.value().schemas.back().source, SchemaSource::kEdited);
}
}  // namespace
}  // namespace openparasolid
