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
void Reference(std::vector<uint8_t>& b, uint32_t index, uint8_t status) {
  CompactIndex(b, index);
  b.push_back(status);
}
void World(std::vector<uint8_t>& b, uint32_t index = 7) {
  U16(b, 101);
  b.push_back(255);
  CompactIndex(b, index);
  for (int i = 0; i < 7; ++i) Reference(b, i == 0 ? 50000u : 0u, i % 2);
  b.push_back(1);
  Reference(b, 0, 1);
  U32(b, 0xffffffff);
  U32(b, 0xffff8004);
}
void Attribute(std::vector<uint8_t>& b, bool first = true) {
  U16(b, 81);
  if (first) b.push_back(255);
  U32(b, 2);
  CompactIndex(b, 7);
  U32(b, 99);
  for (uint32_t i : {80u, 0u, 7u, 0u, 50000u, 0u, 23u, 0u}) Reference(b, i, 1);
}
TEST(ParasolidDeltaStatus, WorldPointersRetainStatusesAndUnsignedIntegerWords) {
  auto b = DeltaPrefixBytes();
  const auto start = b.size();
  World(b);
  const auto end = b.size();
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  EXPECT_EQ(r.value().nodes.size(), 2);
  ASSERT_EQ(r.value().delta_nodes.size(), 1);
  const auto& node = r.value().delta_nodes[0];
  EXPECT_TRUE(node.status_framed);
  EXPECT_EQ(node.offset, start);
  EXPECT_EQ(node.size, end - start);
  EXPECT_EQ(std::get<uint64_t>(node.fields[0].values[0]), 50000);
  EXPECT_EQ(node.fields[0].reference_statuses, std::vector<uint8_t>{0});
  EXPECT_EQ(node.fields[1].reference_statuses, std::vector<uint8_t>{1});
  EXPECT_EQ(node.fields[7].size, 1);
  EXPECT_TRUE(node.fields[7].reference_statuses.empty());
  EXPECT_EQ(std::get<uint64_t>(node.fields[9].values[0]), 0xffffffffu);
  EXPECT_EQ(std::get<uint64_t>(node.fields[10].values[0]), 0xffff8004u);
}
TEST(ParasolidDeltaStatus,
     AttributeCountAndEachArrayPointerHaveSeparateStorage) {
  auto b = DeltaPrefixBytes();
  Attribute(b);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& node = r.value().delta_nodes[0];
  EXPECT_EQ(node.variable_length, 2);
  EXPECT_EQ(std::get<uint64_t>(node.fields[0].values[0]), 99);
  ASSERT_EQ(node.fields.back().values.size(), 2);
  EXPECT_EQ(node.fields.back().reference_statuses,
            (std::vector<uint8_t>{1, 1}));
  EXPECT_EQ(node.fields.back().size, 6);
}
TEST(ParasolidDeltaStatus, RepeatedIndicesStayOrderedWithoutReplacingPrefix) {
  auto b = DeltaPrefixBytes();
  Attribute(b);
  Attribute(b, false);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_nodes.size(), 2);
  EXPECT_EQ(r.value().delta_nodes[0].index, 7);
  EXPECT_EQ(r.value().delta_nodes[1].index, 7);
  EXPECT_LT(r.value().delta_nodes[0].offset, r.value().delta_nodes[1].offset);
  EXPECT_EQ(r.value().nodes.size(), 2);
  EXPECT_EQ(r.value().delta_prefix->partition.index, 1);
}
TEST(ParasolidDeltaStatus, InvalidStatusDoesNotCommitPartialNode) {
  auto b = DeltaPrefixBytes();
  const auto start = b.size();
  U16(b, 15);
  b.push_back(255);
  CompactIndex(b, 7);
  U32(b, 1);
  Reference(b, 0, 2);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().delta_nodes.empty());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidDeltaStatus, EveryTruncatedAttributeRetainsOnlyCompletedPrefix) {
  auto b = DeltaPrefixBytes();
  const auto start = b.size();
  Attribute(b);
  const auto end = b.size();
  End(b);
  for (std::size_t n = start; n < b.size(); ++n) {
    const auto r = SchemaParser::Parse(std::span(b).first(n));
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_EQ(r.value().delta_nodes.size(), n >= end ? 1 : 0) << n;
    EXPECT_EQ(r.value().decoded_end, r.value().uninterpreted_offset);
  }
}
TEST(ParasolidDeltaStatus, ValueArraysKeepRawIntegerAndCharacterWords) {
  for (uint16_t type :
       {uint16_t{82}, uint16_t{83}, uint16_t{84}, uint16_t{98}}) {
    auto b = DeltaPrefixBytes();
    U16(b, type);
    b.push_back(255);
    U32(b, 2);
    CompactIndex(b, 7);
    if (type == 82) {
      U32(b, 0xffffffff);
      U32(b, 0xffff8004);
    }
    if (type == 83) {
      F64(b, 1.25);
      Marker(b);
    }
    if (type == 84) b.insert(b.end(), {'A', 0});
    if (type == 98) {
      U16(b, 0x20ac);
      U16(b, 0);
    }
    End(b);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete) << type;
    const auto& field = r.value().delta_nodes[0].fields[0];
    EXPECT_EQ(field.values.size(), 2);
    EXPECT_TRUE(field.reference_statuses.empty());
    if (type == 82) EXPECT_EQ(std::get<uint64_t>(field.values[1]), 0xffff8004u);
    if (type == 83)
      EXPECT_TRUE(std::holds_alternative<std::monostate>(field.values[1]));
    if (type == 98) EXPECT_EQ(std::get<uint64_t>(field.values[0]), 0x20ac);
  }
}
TEST(ParasolidDeltaStatus, LoopAndFinFieldsConsumeExactlyTheirOwnStatuses) {
  auto b = DeltaPrefixBytes();
  U16(b, 15);
  b.push_back(255);
  CompactIndex(b, 7);
  U32(b, 3);
  for (int i = 0; i < 4; ++i) Reference(b, 0, 1);
  const auto boundary = b.size();
  U16(b, 17);
  b.push_back(255);
  CompactIndex(b, 8);
  for (int i = 0; i < 9; ++i) Reference(b, 0, 0);
  b.push_back('-');
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_nodes.size(), 2);
  EXPECT_EQ(r.value().delta_nodes[0].offset + r.value().delta_nodes[0].size,
            boundary);
  EXPECT_EQ(r.value().delta_nodes[1].offset, boundary);
  EXPECT_EQ(
      std::get<uint64_t>(r.value().delta_nodes[1].fields.back().values[0]),
      '-');
}
TEST(ParasolidDeltaStatus, LimitsCountAllDeltaOccurrencesAndStatusValues) {
  auto b = DeltaPrefixBytes();
  Attribute(b);
  Attribute(b, false);
  End(b);
  SchemaLimits limits;
  limits.max_nodes = 3;
  auto r = SchemaParser::Parse(b, nullptr, limits);
  ASSERT_TRUE(r.ok());
  ASSERT_EQ(r.value().delta_nodes.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
  limits = {};
  limits.max_values = 20;
  r = SchemaParser::Parse(b, nullptr, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
}
TEST(ParasolidDeltaStatus,
     NonDeltaAttributeUsesNoStatusBytesOrUnsignedOverride) {
  auto b = Header();
  U16(b, 81);
  b.push_back(255);
  U32(b, 0);
  CompactIndex(b, 7);
  U32(b, 0xffffffff);
  for (int i = 0; i < 6; ++i) CompactIndex(b, 0);
  End(b);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  EXPECT_TRUE(r.value().delta_nodes.empty());
  EXPECT_EQ(std::get<int64_t>(r.value().nodes[0].fields[0].values[0]), -1);
  EXPECT_TRUE(r.value().nodes[0].fields[1].reference_statuses.empty());
}
TEST(ParasolidDeltaStatus, RecognizesFourNullTrailerOnlyAtExactStreamEnd) {
  auto b = DeltaPrefixBytes();
  World(b);
  const auto end = b.size();
  b.insert(b.end(), {0, 1, 0, 1, 0, 1, 0, 1});
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().complete);
  EXPECT_EQ(r.value().terminator_offset, end);
  EXPECT_EQ(r.value().terminator_size, 8);
  b.push_back(0);
  const auto extra = SchemaParser::Parse(b);
  ASSERT_TRUE(extra.ok());
  EXPECT_FALSE(extra.value().complete);
}
TEST(ParasolidDeltaStatus,
     DeltaHvecUsesElevenComponentsAndNontransmittedTailUsesNone) {
  auto b = DeltaPrefixBytes();
  auto with_end = b;
  End(with_end);
  const auto prefix = SchemaParser::Parse(with_end);
  ASSERT_TRUE(prefix.ok());
  SchemaCatalog catalog{"SCH_1234567_12345_13006", "synthetic", {}, {}};
  for (const auto& schema : prefix.value().schemas)
    catalog.base_types.push_back(schema.definition);
  catalog.base_types.push_back(
      {200,
       "synthetic",
       "test",
       {{"positions", FieldType::kIntersectionPoint, 0, 2, true},
        {"not_sent", FieldType::kPointer, 4, 1, false}}});
  U16(b, 200);
  b.push_back(255);
  U32(b, 999);
  CompactIndex(b, 8);
  for (int i = 0; i < 22; ++i) F64(b, i + .25);
  End(b);
  const auto r = SchemaParser::Parse(b, &catalog);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  const auto& fields = r.value().delta_nodes[0].fields;
  EXPECT_EQ(fields[0].components, 11);
  EXPECT_EQ(fields[0].values.size(), 22);
  EXPECT_EQ(std::get<double>(fields[0].values.back()), 21.25);
  EXPECT_TRUE(fields[1].values.empty());
  EXPECT_TRUE(fields[1].reference_statuses.empty());
}

}  // namespace
}  // namespace openparasolid
