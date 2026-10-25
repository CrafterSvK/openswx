// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/schema.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
using test::U16;
using test::U32;
std::vector<uint8_t> Map() {
  // Two occurrences of the same index and an extended reference (decoded
  // 32767).
  return {0,   1,   0, 1, 0, 9,  0, 81, 0, 9, 0, 84,
          255, 255, 0, 1, 0, 98, 0, 1,  0, 0, 0, 7};
}
void Mark(std::vector<uint8_t>& b, uint16_t index = 9) {
  U16(b, 4);
  U16(b, index);
  U16(b, 1);
  U16(b, 2);
  U16(b, 1);
  U16(b, 1);
  for (uint32_t word : {1u, 2u, 0xffff8004u, 0xffffffffu}) U32(b, word);
  b.push_back(1);
  U32(b, 0x80000000u);
}
TEST(ParasolidDeltaPacket, MapRetainsRepeatedReferencesTypesAndByteRanges) {
  const auto b = Map();
  const auto r = DeltaPacketReader::Read(b, 0, 204);
  ASSERT_TRUE(r.ok());
  const auto& p = r.value();
  EXPECT_EQ(p.kind, DeltaPacketKind::kReferenceMap);
  EXPECT_EQ(p.size, b.size());
  EXPECT_TRUE(p.has_terminal);
  EXPECT_EQ(p.terminal_word, 7);
  ASSERT_EQ(p.references.size(), 3);
  EXPECT_EQ(p.references[0].index, 8);
  EXPECT_EQ(p.references[1].index, 8);
  EXPECT_EQ(p.references[2].index, 32767);
  EXPECT_EQ(p.references[2].offset, 12);
  EXPECT_EQ(p.references[2].size, 6);
  EXPECT_EQ(p.references[2].node_type, 98);
}
TEST(ParasolidDeltaPacket, AllowsEmptySelfDelimitedMapAtNonzeroOffset) {
  const std::vector<uint8_t> b{99, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 88};
  const auto r = DeltaPacketReader::Read(b, 1);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().references.empty());
  EXPECT_EQ(r.value().offset, 1);
  EXPECT_EQ(r.value().size, 10);
}
TEST(ParasolidDeltaPacket, EmptyZeroTerminalDoesNotConsumeFollowingHistory) {
  // Observed before a separate historical packet in the original native
  // saved-geometry-001 sheet capture. No operation meaning is assigned.
  std::vector<uint8_t> b{99, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0};
  const auto boundary = b.size();
  b.insert(b.end(), {0, 1, 0, 1});
  Mark(b);
  const auto r = DeltaPacketReader::Read(b, 1);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().kind, DeltaPacketKind::kReferenceMap);
  EXPECT_TRUE(r.value().references.empty());
  EXPECT_TRUE(r.value().has_terminal);
  EXPECT_EQ(r.value().terminal_word, 0);
  EXPECT_EQ(r.value().offset + r.value().size, boundary);
  const auto next = DeltaPacketReader::Read(b, boundary);
  ASSERT_TRUE(next.ok());
  EXPECT_EQ(next.value().kind, DeltaPacketKind::kHistoricalMarks);
  ASSERT_EQ(next.value().marks.size(), 1);
  for (std::size_t n = 1; n < boundary; ++n)
    EXPECT_FALSE(DeltaPacketReader::Read(std::span(b).first(n), 1).ok()) << n;
}
TEST(ParasolidDeltaPacket, MapRejectsEveryTruncationAndMalformedTerminal) {
  const auto b = Map();
  for (std::size_t n = 0; n < b.size(); ++n)
    EXPECT_FALSE(DeltaPacketReader::Read(std::span(b).first(n), 0).ok()) << n;
  auto bad = b;
  bad.back() = 0;
  EXPECT_FALSE(DeltaPacketReader::Read(bad, 0).ok());
  bad = b;
  bad[3] = 2;
  EXPECT_FALSE(DeltaPacketReader::Read(bad, 0).ok());
}
TEST(ParasolidDeltaPacket, DoesNotInferUnterminatedOrSharedKindMaps) {
  // A null reference followed by a node type is not this self-delimited
  // grammar.
  const std::vector<uint8_t> b{0, 1, 0, 1, 0, 9, 0, 81, 0, 1, 0, 101};
  const auto r = DeltaPacketReader::Read(b, 0);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidDeltaPacket, BoundsTypesReferencesEntriesAndOptions) {
  const auto b = Map();
  EXPECT_FALSE(DeltaPacketReader::Read(b, 0, 80).ok());
  auto bad = b;
  bad[5] = 0;
  EXPECT_FALSE(DeltaPacketReader::Read(bad, 0).ok());
  auto r = DeltaPacketReader::Read(b, 0, 204, 2);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, ErrorCode::kResourceLimit);
  EXPECT_FALSE(DeltaPacketReader::Read(b, 0, 204, 0).ok());
  EXPECT_FALSE(DeltaPacketReader::Read(b, UINT64_MAX).ok());
}
TEST(ParasolidDeltaPacket, HistoricalMarkHasTwentyOneByteScalarTail) {
  std::vector<uint8_t> b{0, 1, 0, 1};
  Mark(b);
  const auto r = DeltaPacketReader::Read(b, 0);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().kind, DeltaPacketKind::kHistoricalMarks);
  ASSERT_EQ(r.value().marks.size(), 1);
  const auto& entry = r.value().marks[0];
  EXPECT_EQ(entry.size, 33);
  EXPECT_EQ(entry.mark.index, 8);
  EXPECT_EQ(entry.mark.n_copy_mod_nodes, 0xffff8004u);
  EXPECT_EQ(entry.mark.delta_key, 0xffffffffu);
  EXPECT_EQ(entry.mark.identifier, 0x80000000u);
  EXPECT_TRUE(entry.mark.delta_is_forward);
  EXPECT_FALSE(r.value().has_terminal);
}
TEST(ParasolidDeltaPacket, KeepsHistoricalOccurrencesAndOptionalTerminal) {
  std::vector<uint8_t> b{0, 1, 0, 1};
  Mark(b);
  Mark(b);
  b.insert(b.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, 1});
  const auto r = DeltaPacketReader::Read(b, 0);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().size, b.size());
  ASSERT_EQ(r.value().marks.size(), 2);
  EXPECT_EQ(r.value().marks[0].mark.index, r.value().marks[1].mark.index);
  EXPECT_TRUE(r.value().has_terminal);
  EXPECT_EQ(r.value().terminal_word, 1);
}
TEST(ParasolidDeltaPacket, HistoricalTerminalRetainsZeroAndOneAtExactBoundary) {
  for (uint8_t word : {uint8_t{0}, uint8_t{1}}) {
    std::vector<uint8_t> b{99, 0, 1, 0, 1};
    Mark(b);
    const auto before_terminal = b.size();
    b.insert(b.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, word});
    const auto end = b.size();
    // A following packet must remain separate, even if it repeats the mark.
    b.insert(b.end(), {0, 1, 0, 1});
    Mark(b);
    const auto r = DeltaPacketReader::Read(b, 1);
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value().kind, DeltaPacketKind::kHistoricalMarks);
    EXPECT_EQ(r.value().terminal_word, word);
    EXPECT_EQ(r.value().size, end - 1);
    ASSERT_EQ(r.value().marks.size(), 1);
    const auto next = DeltaPacketReader::Read(b, end);
    ASSERT_TRUE(next.ok());
    EXPECT_EQ(next.value().size, b.size() - end);
    EXPECT_FALSE(next.value().has_terminal);
    EXPECT_FALSE(next.value().terminal_word.has_value());
    // The optional terminal is consumed only when all ten bytes are present.
    for (auto n = before_terminal; n < end; ++n) {
      const auto partial = DeltaPacketReader::Read(std::span(b).first(n), 1);
      ASSERT_TRUE(partial.ok()) << n;
      EXPECT_EQ(partial.value().size, before_terminal - 1);
      EXPECT_FALSE(partial.value().has_terminal);
    }
  }
}
TEST(ParasolidDeltaPacket, DoesNotExtendHistoricalTerminalToUnobservedWords) {
  for (uint8_t word : {uint8_t{2}, uint8_t{255}}) {
    std::vector<uint8_t> b{0, 1, 0, 1};
    Mark(b);
    const auto end = b.size();
    b.insert(b.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, word});
    const auto r = DeltaPacketReader::Read(b, 0);
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value().size, end);
    EXPECT_FALSE(r.value().has_terminal);
    EXPECT_FALSE(r.value().terminal_word.has_value());
  }
}
TEST(ParasolidDeltaPacket, TruncatedHistoricalNodeIsNotCommitted) {
  std::vector<uint8_t> b{0, 1, 0, 1};
  Mark(b);
  for (std::size_t n = 0; n < b.size(); ++n)
    EXPECT_FALSE(DeltaPacketReader::Read(std::span(b).first(n), 0).ok()) << n;
  Mark(b);
  b.pop_back();
  EXPECT_FALSE(DeltaPacketReader::Read(b, 0).ok());
}
TEST(ParasolidDeltaPacket, RejectsBadHistoricalIdentityDirectionAndLimit) {
  std::vector<uint8_t> b{0, 1, 0, 1};
  Mark(b, 1);
  EXPECT_FALSE(DeltaPacketReader::Read(b, 0).ok());
  b = {0, 1, 0, 1};
  Mark(b);
  b[b.size() - 5] = 2;
  EXPECT_FALSE(DeltaPacketReader::Read(b, 0).ok());
  b = {0, 1, 0, 1};
  Mark(b);
  Mark(b);
  const auto r = DeltaPacketReader::Read(b, 0, 204, 1);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidDeltaPacket, RefusesAmbiguousMapAndHistoricalInterpretations) {
  std::vector<uint8_t> b{0, 1, 0, 1, 0, 4, 0, 5, 0, 2, 0, 4, 0, 2, 0, 2};
  U32(b, 0x00010000);
  U32(b, 0x00010000);
  U32(b, 0);
  U32(b, 0);
  b.push_back(0);
  U32(b, 0);
  const auto r = DeltaPacketReader::Read(b, 0);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
}
}  // namespace
}  // namespace openparasolid
