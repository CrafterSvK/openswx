// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
std::vector<uint8_t> AmbiguousPacket() {
  // As a map: three references, ending at byte 22. As a PMARK: one
  // historical occurrence, ending at byte 37. Neither reading needs edits.
  std::vector<uint8_t> b{0, 1, 0, 1, 0, 4, 0, 5, 0, 2, 0, 4, 0, 2, 0, 2};
  U32(b, 0x00010000);
  U32(b, 0x00010000);
  U32(b, 0);
  U32(b, 0);
  b.push_back(0);
  U32(b, 0);
  return b;
}
void End(std::vector<uint8_t>& b) {
  U16(b, 1);
  CompactIndex(b, 0);
}
std::vector<uint8_t> Stream(const std::vector<uint8_t>& packet) {
  auto b = DeltaPrefixBytes();
  b.insert(b.end(), packet.begin(), packet.end());
  return b;
}
TEST(ParasolidDeltaBoundary, SelectsHistoryOnlyWhenMapFollowingTypeIsInvalid) {
  auto packet = AmbiguousPacket();
  End(packet);
  EXPECT_FALSE(DeltaPacketReader::Read(packet, 0, 204).ok());
  const auto b = Stream(packet);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_packets.size(), 1);
  const auto& p = r.value().delta_packets[0];
  EXPECT_EQ(p.kind, DeltaPacketKind::kHistoricalMarks);
  EXPECT_EQ(p.size, 37);
  ASSERT_EQ(p.marks.size(), 1);
  EXPECT_EQ(p.marks[0].mark.index, 4);
  EXPECT_EQ(r.value().terminator_offset, p.offset + p.size);
}
TEST(ParasolidDeltaBoundary, SelectsMapWhenHistoryEndsInsideFollowingLoop) {
  auto packet = AmbiguousPacket();
  packet.resize(22);
  U16(packet, 15);
  packet.push_back(255);
  CompactIndex(packet, 7);
  U32(packet, 123);
  for (uint32_t ref : {0u, 0u, 256u, 0u}) {
    CompactIndex(packet, ref);
    packet.push_back(0);
  }
  End(packet);
  EXPECT_FALSE(DeltaPacketReader::Read(packet, 0, 204).ok());
  const auto b = Stream(packet);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  ASSERT_TRUE(r.value().complete);
  ASSERT_EQ(r.value().delta_packets.size(), 1);
  const auto& p = r.value().delta_packets[0];
  EXPECT_EQ(p.kind, DeltaPacketKind::kReferenceMap);
  EXPECT_EQ(p.size, 22);
  ASSERT_EQ(p.references.size(), 3);
  EXPECT_EQ(p.references[0].index, 3);
  ASSERT_EQ(r.value().delta_nodes.size(), 1);
  const auto& n = r.value().delta_nodes[0];
  EXPECT_EQ(n.node_type, 15);
  EXPECT_EQ(n.offset, p.offset + p.size);
  EXPECT_EQ(std::get<uint64_t>(n.fields[3].values[0]), 256);
}
TEST(ParasolidDeltaBoundary, UnknownSchemaDoesNotEliminateInRangeBoundary) {
  auto packet = AmbiguousPacket();
  packet[23] = 200;  // Unknown base type, but within the header's range.
  End(packet);
  const auto b = Stream(packet);
  const auto r = SchemaParser::Parse(b);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().delta_packets.empty());
  EXPECT_EQ(r.value().decoded_end, DeltaPrefixBytes().size());
  ASSERT_EQ(r.value().diagnostics.size(), 1);
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidDeltaBoundary, RejectsBothInvalidBoundariesAtomically) {
  auto packet = AmbiguousPacket();
  U16(packet, 0);
  const auto r = SchemaParser::Parse(Stream(packet));
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().delta_packets.empty());
  EXPECT_EQ(r.value().decoded_end, DeltaPrefixBytes().size());
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidDeltaBoundary, DoesNotUseMissingBoundaryBytesToChooseCandidate) {
  for (bool one_byte : {false, true}) {
    auto packet = AmbiguousPacket();
    if (one_byte) packet.push_back(0);
    const auto r = SchemaParser::Parse(Stream(packet));
    ASSERT_TRUE(r.ok());
    EXPECT_FALSE(r.value().complete);
    EXPECT_TRUE(r.value().delta_packets.empty());
    EXPECT_EQ(r.value().decoded_end, DeltaPrefixBytes().size());
    EXPECT_EQ(r.value().diagnostics[0].error.code,
              ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidDeltaBoundary, EntryLimitCannotSelectHistoricalInterpretation) {
  auto packet = AmbiguousPacket();
  End(packet);
  SchemaLimits limits;
  limits.max_delta_entries = 2;
  const auto r = SchemaParser::Parse(Stream(packet), nullptr, limits);
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().complete);
  EXPECT_TRUE(r.value().delta_packets.empty());
  EXPECT_EQ(r.value().decoded_end, DeltaPrefixBytes().size());
  EXPECT_EQ(r.value().diagnostics[0].error.code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
