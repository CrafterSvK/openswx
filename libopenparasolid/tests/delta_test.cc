// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using test::U16;
using test::U32;
void Index(std::vector<uint8_t>& out, uint32_t value) {
  if (value < 32767) {
    U16(out, static_cast<uint16_t>(value + 1));
  } else {
    U16(out, static_cast<uint16_t>(-static_cast<int32_t>(value % 32767 + 1)));
    U16(out, static_cast<uint16_t>(value / 32767));
  }
}
struct Fixture {
  std::vector<uint8_t> bytes;
  uint64_t partition_offset;
  uint64_t reference_offset;
  uint64_t mark_offset;
  uint64_t logical_offset;
};
Fixture Delta(uint32_t partition = 7, uint32_t current = 50000,
              uint32_t mark = 50000, bool edits = false) {
  Fixture f;
  f.bytes = test::Header("(deltas) ");
  f.partition_offset = f.bytes.size();
  U16(f.bytes, 3);
  if (edits)
    f.bytes.insert(f.bytes.end(), {2, 'C', 'C', 'Z'});
  else
    f.bytes.push_back(255);
  Index(f.bytes, partition);
  f.reference_offset = f.bytes.size();
  Index(f.bytes, current);
  U32(f.bytes, 0xffff8004);
  f.mark_offset = f.bytes.size();
  U16(f.bytes, 4);
  if (edits) {
    f.bytes.push_back(10);
    f.bytes.insert(f.bytes.end(), 10, 'C');
    f.bytes.push_back('Z');
  } else
    f.bytes.push_back(255);
  Index(f.bytes, mark);
  for (uint32_t value :
       {uint32_t{70000}, uint32_t{0}, uint32_t{90000}, uint32_t{0}})
    Index(f.bytes, value);
  for (uint32_t value :
       {uint32_t{0x80000000}, uint32_t{2}, uint32_t{3}, uint32_t{0xffff8004}})
    U32(f.bytes, value);
  f.logical_offset = f.bytes.size();
  f.bytes.push_back(1);
  U32(f.bytes, 0xffffffff);
  return f;
}
void End(std::vector<uint8_t>& out) {
  U16(out, 1);
  Index(out, 0);
}
TEST(ParasolidDelta, ReadsPartitionAndCurrentMarkWithExtendedReferences) {
  auto f = Delta();
  const auto end = f.bytes.size();
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().nodes.size(), 2);
  ASSERT_TRUE(r.value().delta_prefix);
  const auto& prefix = *r.value().delta_prefix;
  EXPECT_EQ(prefix.offset, f.partition_offset);
  EXPECT_EQ(prefix.offset + prefix.size, end);
  EXPECT_EQ(prefix.partition.index, 7);
  EXPECT_EQ(prefix.partition.current_pmark, 50000);
  EXPECT_EQ(prefix.current_mark.index, 50000);
  EXPECT_EQ(prefix.current_mark.preceding, 70000);
  EXPECT_EQ(prefix.current_mark.first_following, 0);
  EXPECT_EQ(prefix.current_mark.next_sibling, 90000);
  EXPECT_EQ(prefix.current_mark.prev_sibling, 0);
  EXPECT_EQ(prefix.current_mark.n_del_nodes, 2);
  EXPECT_EQ(prefix.current_mark.n_copy_mod_nodes, 3);
  EXPECT_TRUE(prefix.current_mark.delta_is_forward);
  EXPECT_EQ(r.value().nodes[0].size, f.mark_offset - f.partition_offset);
  EXPECT_EQ(r.value().nodes[1].offset, f.mark_offset);
  EXPECT_EQ(r.value().uninterpreted_size, 0);
}
TEST(ParasolidDelta, PersistentWordsPreserveSignAndUnsetBitPatterns) {
  auto f = Delta();
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().delta_prefix);
  const auto& prefix = *r.value().delta_prefix;
  EXPECT_EQ(prefix.partition.highest_id, 0xffff8004u);
  EXPECT_EQ(prefix.current_mark.n_new_nodes, 0x80000000u);
  EXPECT_EQ(prefix.current_mark.delta_key, 0xffff8004u);
  EXPECT_EQ(prefix.current_mark.identifier, 0xffffffffu);
  EXPECT_TRUE(std::holds_alternative<std::monostate>(
      r.value().nodes[0].fields[1].values[0]));
}
TEST(ParasolidDelta, AcceptsEquivalentEmbeddedSchemaEdits) {
  auto f = Delta(7, 50000, 50000, true);
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().complete);
  ASSERT_TRUE(r.value().delta_prefix);
  ASSERT_EQ(r.value().schemas.size(), 2);
  EXPECT_EQ(r.value().schemas[0].source, SchemaSource::kEdited);
  EXPECT_EQ(r.value().schemas[1].source, SchemaSource::kEdited);
}
TEST(ParasolidDelta, DoesNotRequireConsecutivePartitionAndMarkIndices) {
  auto f = Delta(32767, 9, 9);
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().complete);
  ASSERT_TRUE(r.value().delta_prefix);
  EXPECT_EQ(r.value().delta_prefix->partition.index, 32767);
  EXPECT_EQ(r.value().delta_prefix->current_mark.index, 9);
}
TEST(ParasolidDelta, RejectsNullAndSelfCurrentMarkReferences) {
  for (uint32_t current : {uint32_t{0}, uint32_t{7}}) {
    auto f = Delta(7, current);
    End(f.bytes);
    const auto r = SchemaParser::Parse(f.bytes);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_FALSE(r.value().delta_prefix);
    EXPECT_TRUE(r.value().nodes.empty());
    ASSERT_EQ(r.value().diagnostics.size(), 1);
    EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
    EXPECT_EQ(r.value().diagnostics[0].error.offset, f.reference_offset);
    EXPECT_EQ(r.value().uninterpreted_offset, f.partition_offset);
  }
}
TEST(ParasolidDelta, RejectsWrongOrDuplicateMarkIdentity) {
  for (uint32_t mark : {uint32_t{7}, uint32_t{50001}}) {
    auto f = Delta(7, 50000, mark);
    End(f.bytes);
    const auto r = SchemaParser::Parse(f.bytes);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_FALSE(r.value().delta_prefix);
    ASSERT_EQ(r.value().nodes.size(), 1);
    EXPECT_EQ(r.value().uninterpreted_offset, f.mark_offset);
    EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidDelta, RejectsNonLogicalDirection) {
  auto f = Delta();
  f.bytes[f.logical_offset] = 2;
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().delta_prefix);
  ASSERT_EQ(r.value().nodes.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.offset, f.logical_offset);
}
TEST(ParasolidDelta, KeepsAllTruncatedPrefixesIncomplete) {
  auto f = Delta();
  End(f.bytes);
  for (std::size_t size = 0; size < f.bytes.size(); ++size) {
    const auto r = SchemaParser::Parse(std::span(f.bytes).first(size));
    if (!r.ok()) continue;
    EXPECT_FALSE(r.value().complete) << size;
    ASSERT_EQ(r.value().diagnostics.size(), 1) << size;
    EXPECT_EQ(r.value().decoded_end, r.value().uninterpreted_offset);
    EXPECT_EQ(r.value().uninterpreted_offset + r.value().uninterpreted_size,
              size);
    EXPECT_EQ(r.value().delta_prefix.has_value(), size >= f.bytes.size() - 4);
  }
}
TEST(ParasolidDelta, RejectsIncompleteChangeMapWithoutCommittingItsBytes) {
  auto f = Delta();
  const auto end = f.bytes.size();
  f.bytes.insert(f.bytes.end(), {0, 1, 0, 1, 0, 37, 0, 81, 0, 1});
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  ASSERT_TRUE(r.value().delta_prefix);
  EXPECT_EQ(r.value().decoded_end, end);
  EXPECT_EQ(r.value().uninterpreted_offset, end);
  EXPECT_EQ(r.value().uninterpreted_size, 10);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
  EXPECT_FALSE(r.value().terminator_offset);
}
TEST(ParasolidDelta, RejectsMissingOrReversedPrefixNodes) {
  for (uint16_t first : {uint16_t{1}, uint16_t{4}, uint16_t{101}}) {
    auto b = test::Header("(deltas) ");
    U16(b, first);
    Index(b, 0);
    const auto r = SchemaParser::Parse(b);
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_FALSE(r.value().delta_prefix);
    EXPECT_TRUE(r.value().nodes.empty());
    EXPECT_EQ(r.value().diagnostics.size(), 1);
  }
  auto f = Delta();
  f.bytes.resize(f.mark_offset);
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_FALSE(r.value().delta_prefix);
  EXPECT_EQ(r.value().nodes.size(), 1);
}
TEST(ParasolidDelta, HonorsNodeLimitAndExplicitCatalogMembership) {
  auto f = Delta();
  End(f.bytes);
  SchemaLimits limits;
  limits.max_nodes = 1;
  const auto limited = SchemaParser::Parse(f.bytes, nullptr, limits);
  ASSERT_TRUE(limited.ok());
  EXPECT_FALSE(limited.value().delta_prefix);
  ASSERT_EQ(limited.value().nodes.size(), 1);
  EXPECT_EQ(limited.value().diagnostics[0].error.code,
            ErrorCode::kResourceLimit);
  SchemaCatalog catalog{
      "SCH_1234567_12345_13006", "empty caller catalog", {}, {}};
  const auto supplied = SchemaParser::Parse(f.bytes, &catalog);
  ASSERT_TRUE(supplied.ok());
  EXPECT_TRUE(supplied.value().nodes.empty());
  EXPECT_EQ(supplied.value().diagnostics[0].error.code,
            ErrorCode::kUnsupportedSchema);
}
TEST(ParasolidDelta, DoesNotAssignUnreviewedEditedFieldRoles) {
  auto f = Delta();
  const auto b = f.partition_offset;
  // A valid one-field edit drops highest_id; typed prefix interpretation is
  // unavailable.
  f.bytes.resize(b);
  U16(f.bytes, 3);
  f.bytes.insert(f.bytes.end(), {1, 'C', 'Z'});
  Index(f.bytes, 7);
  Index(f.bytes, 8);
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().nodes.empty());
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
}
TEST(ParasolidDelta, ConsumesReferenceMapBeforeUnknownMeshPayload) {
  auto f = Delta();
  const auto start = f.bytes.size();
  f.bytes.insert(f.bytes.end(),
                 {0, 1, 0, 1, 0, 9, 0, 81, 0, 1, 0, 0, 0, 1, 0, 200});
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  ASSERT_EQ(r.value().delta_packets.size(), 1);
  EXPECT_EQ(r.value().delta_packets[0].references[0].index, 8);
  EXPECT_EQ(r.value().decoded_end, start + 14);
  EXPECT_EQ(r.value().uninterpreted_offset, start + 14);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedSchema);
  EXPECT_EQ(r.value().nodes.size(), 2);
}
TEST(ParasolidDelta, PacketAndEntryCapsKeepFailedPacketUncommitted) {
  auto f = Delta();
  const auto start = f.bytes.size();
  f.bytes.insert(f.bytes.end(),
                 {0, 1, 0, 1, 0, 9, 0, 81, 0, 10, 0, 84, 0, 1, 0, 0, 0, 1});
  End(f.bytes);
  SchemaLimits limits;
  limits.max_delta_entries = 1;
  auto r = SchemaParser::Parse(f.bytes, nullptr, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().delta_packets.empty());
  EXPECT_EQ(r.value().decoded_end, start);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
  f = Delta();
  for (int i = 0; i < 2; ++i)
    f.bytes.insert(f.bytes.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, 1});
  End(f.bytes);
  limits = {};
  limits.max_delta_packets = 1;
  r = SchemaParser::Parse(f.bytes, nullptr, limits);
  ASSERT_TRUE(r.ok());
  ASSERT_EQ(r.value().delta_packets.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
}
TEST(ParasolidDelta,
     HistoricalPacketsRetainRepeatedIdentitiesOutsideCurrentNodes) {
  auto f = Delta();
  f.bytes.insert(f.bytes.end(), {0, 1, 0, 1});
  for (uint32_t id : {1u, 2u}) {
    U16(f.bytes, 4);
    Index(f.bytes, 50000);
    for (int i = 0; i < 4; ++i) Index(f.bytes, 0);
    for (int i = 0; i < 4; ++i) U32(f.bytes, 0);
    f.bytes.push_back(0);
    U32(f.bytes, id);
  }
  End(f.bytes);
  const auto r = SchemaParser::Parse(f.bytes);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_packets.size(), 1);
  ASSERT_EQ(r.value().delta_packets[0].marks.size(), 2);
  EXPECT_EQ(r.value().delta_packets[0].marks[0].mark.identifier, 1);
  EXPECT_EQ(r.value().delta_packets[0].marks[1].mark.identifier, 2);
  EXPECT_EQ(r.value().delta_prefix->current_mark.identifier, 0xffffffffu);
  EXPECT_EQ(r.value().nodes.size(), 2);
}

TEST(ParasolidDelta, ZeroHistoricalTerminalBelongsToItsMarkPacket) {
  for (bool trailer : {false, true}) {
    auto f = Delta();
    const auto start = f.bytes.size();
    f.bytes.insert(f.bytes.end(), {0, 1, 0, 1});
    U16(f.bytes, 4);
    Index(f.bytes, 50000);
    for (int i = 0; i < 4; ++i) Index(f.bytes, 0);
    for (int i = 0; i < 4; ++i) U32(f.bytes, 0);
    f.bytes.push_back(0);
    U32(f.bytes, 123);
    f.bytes.insert(f.bytes.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, 0});
    const auto end = f.bytes.size();
    End(f.bytes);
    if (trailer) End(f.bytes);
    const auto r = SchemaParser::Parse(f.bytes);
    ASSERT_TRUE(r.ok());
    ASSERT_TRUE(r.value().complete);
    ASSERT_EQ(r.value().delta_packets.size(), 1);
    const auto& packet = r.value().delta_packets[0];
    EXPECT_EQ(packet.offset, start);
    EXPECT_EQ(packet.size, end - start);
    EXPECT_EQ(packet.kind, DeltaPacketKind::kHistoricalMarks);
    EXPECT_TRUE(packet.references.empty());
    EXPECT_TRUE(packet.has_terminal);
    EXPECT_EQ(packet.terminal_word, 0);
    ASSERT_EQ(packet.marks.size(), 1);
    EXPECT_EQ(packet.marks[0].mark.identifier, 123);
    EXPECT_EQ(r.value().terminator_offset, end);
    EXPECT_EQ(r.value().terminator_size, trailer ? 8 : 4);
    EXPECT_EQ(r.value().nodes.size(), 2);
    EXPECT_TRUE(r.value().delta_nodes.empty());
  }
}

}  // namespace
}  // namespace openparasolid
