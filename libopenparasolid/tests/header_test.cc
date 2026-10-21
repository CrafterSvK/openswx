// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include "openparasolid/parser.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {

TEST(ParasolidHeaderTest, ReadsStreamKindsSchemaAndOffsets) {
  for (const auto &[form, kind] :
       std::array<std::pair<std::string_view, StreamKind>, 3>{
           {{"", StreamKind::kTransmit},
            {"(partition) ", StreamKind::kPartition},
            {"(deltas) ", StreamKind::kDeltas}}}) {
    auto bytes = test::Header(form);
    const auto result = ParasolidReader::ReadHeader(bytes);
    ASSERT_TRUE(result.ok()) << result.error().message;
    const auto &h = result.value();
    EXPECT_EQ(h.kind, kind);
    EXPECT_EQ(h.body_offset, bytes.size());
    EXPECT_EQ(h.description_offset, 6u);
    EXPECT_EQ(h.schema, "SCH_1234567_12345_13006");
    ASSERT_TRUE(h.schema_version);
    EXPECT_EQ(h.schema_version->modeller, 1234567u);
    EXPECT_EQ(h.schema_version->schema, 12345u);
    EXPECT_EQ(h.schema_version->base_schema, 13006u);
  }
}

TEST(ParasolidHeaderTest, HonorsPaddingAndDeclaredSchemaLength) {
  for (const std::vector<uint8_t> &padding :
       {std::vector<uint8_t>{}, std::vector<uint8_t>{0, 0, 0},
        std::vector<uint8_t>{0xff, 3, 0, 7, 0xff}}) {
    auto bytes = test::Header("", "SCH_1_2_13006", padding);
    const auto body = bytes.size();
    bytes.insert(bytes.end(), {'A', 'B', 'C', '_'});
    auto result = ParasolidReader::ReadHeader(bytes);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().schema, "SCH_1_2_13006");
    EXPECT_EQ(result.value().body_offset, body);
  }
}

TEST(ParasolidHeaderTest, RejectsEveryTruncatedHeaderPrefix) {
  const auto bytes = test::Header();
  for (std::size_t size = 0; size < bytes.size(); ++size) {
    SCOPED_TRACE(size);
    EXPECT_FALSE(
        ParasolidReader::ReadHeader(std::span<const uint8_t>(bytes).first(size))
            .ok());
  }
}

TEST(ParasolidHeaderTest, RejectsInvalidLengthsDescriptionsAndTokens) {
  const auto original = test::Header();
  for (int failure : {0, 1, 2, 3}) {
    auto bytes = original;
    if (failure == 0) {
      bytes[4] = 0xff;
      bytes[5] = 0xff;
    }
    if (failure == 1) bytes[6] = 0;
    if (failure == 2) bytes[bytes.size() - 7u] = '!';
    if (failure == 3)
      bytes[bytes.size() - std::string_view("SCH_1234567_12345_13006").size() -
            1u] = 255;
    const auto result = ParasolidReader::ReadHeader(bytes);
    EXPECT_FALSE(result.ok());
  }
}

TEST(ParasolidHeaderTest, RejectsTextAndWrappedInput) {
  for (const std::string text : {"**PARASOLID XT", "not a transmit stream"}) {
    const std::vector<uint8_t> bytes(text.begin(), text.end());
    auto result = ParasolidReader::ReadHeader(bytes);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_EQ(result.error().offset, 0u);
  }
}

TEST(ParasolidHeaderTest, RetainsUnknownSchemaWithoutInventingNumericVersion) {
  const auto bytes = test::Header("", "SCH_future_schema");
  auto result = ParasolidReader::ReadHeader(bytes);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_FALSE(result.value().schema_version);
  EXPECT_EQ(result.value().schema, "SCH_future_schema");
}

TEST(ParasolidHeaderTest, ReadsBaseLessInternalHeaderAndUserFields) {
  const auto bytes = test::Header("", "SCH_1300000_13006");
  const auto result = ParasolidReader::ReadHeader(bytes);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_FALSE(result.value().schema_version->base_schema);
  EXPECT_FALSE(result.value().schema_max_type);
  EXPECT_EQ(result.value().user_field_words, 0);
  EXPECT_TRUE(result.value().internal_header_complete);
  EXPECT_EQ(result.value().body_offset, bytes.size());
}

TEST(ParasolidHeaderTest,
     DistinguishesCommonPrologueFromCompleteInternalHeader) {
  auto bytes = test::Header();
  bytes.resize(bytes.size() - 6u);
  const auto common = ParasolidReader::ReadCommonHeader(bytes);
  ASSERT_TRUE(common.ok()) << common.error().message;
  EXPECT_FALSE(common.value().internal_header_complete);
  EXPECT_EQ(common.value().schema_end_offset, bytes.size());
  const auto full = ParasolidReader::ReadHeader(bytes);
  ASSERT_FALSE(full.ok());
  EXPECT_EQ(full.error().offset, bytes.size());
}

TEST(ParasolidHeaderTest, RejectsInvalidUserFieldCount) {
  auto bytes = test::Header();
  bytes.back() = 17;
  EXPECT_TRUE(ParasolidReader::ReadCommonHeader(bytes).ok());
  const auto result = ParasolidReader::ReadHeader(bytes);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().offset, bytes.size() - 4u);
}

}  // namespace
}  // namespace openparasolid
