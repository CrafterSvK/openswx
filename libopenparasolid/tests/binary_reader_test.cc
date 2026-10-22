// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "../src/binary_reader.h"

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace openparasolid::internal {
namespace {
TEST(ParasolidBinaryReader, ReportsShortReadsWithoutConsumingUnavailableBytes) {
  const std::array<uint8_t, 3> bytes{1, 2, 3};
  Cursor cursor{bytes, 1};
  const auto short_read = cursor.Read(4);
  ASSERT_FALSE(short_read.ok());
  EXPECT_EQ(short_read.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(short_read.error().offset, 1);
  EXPECT_EQ(cursor.pos, 1);
  const auto valid = cursor.Read(2);
  ASSERT_TRUE(valid.ok());
  EXPECT_EQ(valid.value(), 0x0203);
  EXPECT_EQ(cursor.pos, 3);
  const auto end = cursor.Read(1);
  ASSERT_FALSE(end.ok());
  EXPECT_EQ(end.error().offset, 3);
  cursor.pos = std::numeric_limits<uint64_t>::max();
  const auto beyond = cursor.Read(1);
  ASSERT_FALSE(beyond.ok());
  EXPECT_EQ(beyond.error().offset, cursor.pos);
}
TEST(ParasolidBinaryReader, BoundsWordWidthAndPreservesAllEightBytes) {
  const std::array<uint8_t, 8> bytes{0x80, 1, 2, 3, 4, 5, 6, 0xff};
  Cursor cursor{bytes, 0};
  const auto invalid = cursor.Read(9);
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
  EXPECT_EQ(cursor.pos, 0);
  const auto word = cursor.Read(8);
  ASSERT_TRUE(word.ok());
  EXPECT_EQ(word.value(), 0x80010203040506ffULL);
}
TEST(ParasolidBinaryReader,
     PropagatesCompactIndexFailuresAtTheirSourceOffsets) {
  for (const auto bytes : {std::array<uint8_t, 4>{0, 0, 0, 1},
                           std::array<uint8_t, 4>{0x80, 0, 0, 1},
                           std::array<uint8_t, 4>{0xff, 0xff, 0, 0}}) {
    Cursor cursor{bytes, 0};
    const auto index = cursor.Index();
    ASSERT_FALSE(index.ok());
    EXPECT_EQ(index.error().code, ErrorCode::kInvalidRecord);
    EXPECT_EQ(index.error().offset, 0);
  }
  const std::array<uint8_t, 3> truncated{0xff, 0xff, 0};
  Cursor cursor{truncated, 0};
  const auto index = cursor.Index();
  ASSERT_FALSE(index.ok());
  EXPECT_EQ(index.error().offset, 2);
  EXPECT_EQ(cursor.pos, 2);
  const std::array<uint8_t, 4> valid{0xff, 0xff, 0, 2};
  Cursor valid_cursor{valid, 0};
  const auto large = valid_cursor.Index();
  ASSERT_TRUE(large.ok());
  EXPECT_EQ(large.value(), 65534);
}
TEST(ParasolidBinaryReader, ReportsEmptyNonAsciiAndTruncatedSchemaText) {
  const std::array<uint8_t, 1> empty{0};
  Cursor cursor{empty, 0};
  const auto name = cursor.Text();
  ASSERT_FALSE(name.ok());
  EXPECT_EQ(name.error().code, ErrorCode::kInvalidSchema);
  EXPECT_EQ(name.error().offset, 0);
  cursor.pos = 0;
  const auto description = cursor.Text(true);
  ASSERT_TRUE(description.ok());
  EXPECT_TRUE(description.value().empty());
  const std::array<uint8_t, 2> non_ascii{1, 0x80};
  Cursor non_ascii_cursor{non_ascii, 0};
  const auto invalid = non_ascii_cursor.Text();
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidSchema);
  EXPECT_EQ(invalid.error().offset, 1);
  const std::array<uint8_t, 2> truncated{2, 'a'};
  Cursor truncated_cursor{truncated, 0};
  const auto short_name = truncated_cursor.Text();
  ASSERT_FALSE(short_name.ok());
  EXPECT_EQ(short_name.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(short_name.error().offset, 2);
}
TEST(ParasolidBinaryReader, ValidatesLogicalBytesAndPropagatesTruncation) {
  const std::array<uint8_t, 3> bytes{0, 1, 2};
  Cursor cursor{bytes, 0};
  const auto no = cursor.Logical();
  const auto yes = cursor.Logical();
  ASSERT_TRUE(no.ok());
  ASSERT_TRUE(yes.ok());
  EXPECT_FALSE(no.value());
  EXPECT_TRUE(yes.value());
  const auto invalid = cursor.Logical();
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(invalid.error().offset, 2);
  const auto missing = cursor.Logical();
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.error().offset, 3);
}
}  // namespace
}  // namespace openparasolid::internal
