// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>

#include "openparasolid/parser.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {

void ExpectAccounting(const StreamInspection &report) {
  EXPECT_EQ(report.header.body_offset + report.candidate_prefix_bytes +
                report.uninterpreted_bytes,
            report.byte_size);
  uint64_t previous_end = report.header.body_offset;
  for (const auto &region : report.unknown_regions) {
    EXPECT_GE(region.offset, previous_end);
    EXPECT_LE(region.offset + region.size, report.byte_size);
    EXPECT_LE(region.preview.size(), 16u);
    previous_end = region.offset + region.size;
  }
}

TEST(ParasolidInspectorTest,
     ReportsLinkedCandidatePrefixesAndReferenceTargets) {
  const auto bytes = test::LinkedRecords();
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto &report = result.value();
  EXPECT_TRUE(report.record_scan_performed);
  EXPECT_TRUE(report.record_scan_complete);
  ASSERT_EQ(report.candidates.size(), 6u);
  EXPECT_EQ(report.candidates[0].kind, RecordKind::kFaceUse);
  EXPECT_EQ(report.candidates[0].references[2].status,
            ReferenceStatus::kUniqueCandidate);
  EXPECT_EQ(report.candidates[0].references[2].target_candidate, 1u);
  EXPECT_EQ(report.candidates[0].references[4].status,
            ReferenceStatus::kUninterpretedTarget);
  EXPECT_EQ(report.candidates[4].references[4].target_candidate, 5u);
  EXPECT_EQ(report.candidates[5].kind, RecordKind::kPoint);
  ASSERT_TRUE(report.candidates[5].position);
  EXPECT_DOUBLE_EQ((*report.candidates[5].position)[0], .1);
  EXPECT_EQ(report.uninterpreted_bytes, 10u);
  ExpectAccounting(report);
}

TEST(ParasolidInspectorTest, KeepsDuplicateIdentitiesAmbiguousWithinTheStream) {
  auto bytes = test::LinkedRecords();
  test::Point(bytes);
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().candidates.size(), 7u);
  const auto &reference = result.value().candidates[4].references[4];
  EXPECT_EQ(reference.status, ReferenceStatus::kAmbiguous);
  EXPECT_FALSE(reference.target_candidate);
  ExpectAccounting(result.value());
}

TEST(ParasolidInspectorTest, LeavesMissingCandidateReferencesUnresolved) {
  auto bytes = test::LinkedRecords();
  bytes.resize(bytes.size() - 40u);
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().candidates.size(), 5u);
  EXPECT_EQ(result.value().candidates[4].references[4].status,
            ReferenceStatus::kUnresolved);
  EXPECT_FALSE(result.value().candidates[4].references[4].target_candidate);
  ExpectAccounting(result.value());
}

TEST(ParasolidInspectorTest, SupportsSchemaMarkerAndPreservesUnknownOffsets) {
  auto bytes = test::Header("(deltas) ");
  const auto body = bytes.size();
  bytes.insert(bytes.end(), {0x80, 0x81, 0x82});
  test::Point(bytes, 80, {1, -2, 3}, true);
  bytes.insert(bytes.end(), {0x90, 0x91});
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  const auto &report = result.value();
  ASSERT_EQ(report.candidates.size(), 1u);
  EXPECT_EQ(report.candidates[0].offset, body + 3u);
  EXPECT_TRUE(report.candidates[0].has_schema_marker);
  ASSERT_EQ(report.unknown_regions.size(), 2u);
  EXPECT_EQ(report.unknown_regions[0].offset, body);
  EXPECT_EQ(report.unknown_regions[0].preview,
            (std::vector<uint8_t>{0x80, 0x81, 0x82}));
  EXPECT_EQ(report.unknown_regions[1].size, 2u);
  ExpectAccounting(report);
}

TEST(ParasolidInspectorTest,
     DoesNotClaimPointAdjacencyTablesOrNonfiniteCoordinates) {
  for (int failure : {0, 1, 2}) {
    auto bytes = test::Header();
    const auto start = bytes.size();
    test::Point(bytes);
    if (failure == 0) bytes[start + 9u] = 5;  // ref0 is not a sentinel.
    if (failure == 1) {
      bytes.resize(start + 16u);
      test::F64(bytes, std::numeric_limits<double>::quiet_NaN());
      test::F64(bytes, 0);
      test::F64(bytes, 0);
    }
    if (failure == 2)
      bytes[start + 2u] = 0x80;  // Extended attribute is unsupported.
    const auto result = ParasolidInspector::Inspect(bytes);
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().candidates.empty());
    ExpectAccounting(result.value());
  }
}

TEST(ParasolidInspectorTest, LeavesTruncatedPointPrefixesUninterpreted) {
  auto bytes = test::Header();
  const auto body = bytes.size();
  test::Point(bytes);
  for (std::size_t count = 0; count < 40u; ++count) {
    const auto result = ParasolidInspector::Inspect(
        std::span<const uint8_t>(bytes).first(body + count));
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().candidates.empty());
    EXPECT_EQ(result.value().uninterpreted_bytes, count);
    ExpectAccounting(result.value());
  }
}

TEST(ParasolidInspectorTest,
     RequiresTopologyMarkersAndSenseForAnchoredCandidates) {
  auto bytes = test::LinkedRecords();
  const auto body = test::Header("(partition) ").size();
  bytes[body + 10u] ^= 1u;  // Face-use magic at tag + body+8.
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(std::count_if(result.value().candidates.begin(),
                          result.value().candidates.end(),
                          [](const auto &candidate) {
                            return candidate.kind == RecordKind::kFaceUse;
                          }),
            0);
  ExpectAccounting(result.value());
}

TEST(ParasolidInspectorTest,
     MakesScanLimitsExplicitAndPreservesRemainingBytes) {
  const auto bytes = test::LinkedRecords();
  InspectOptions options;
  options.max_records = 2;
  const auto result = ParasolidInspector::Inspect(bytes, options);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().candidates.size(), 2u);
  EXPECT_FALSE(result.value().record_scan_complete);
  EXPECT_FALSE(result.value().notes.empty());
  ExpectAccounting(result.value());
  options.max_records = 0;
  const auto header_only = ParasolidInspector::Inspect(bytes, options);
  ASSERT_TRUE(header_only.ok());
  EXPECT_TRUE(header_only.value().candidates.empty());
  EXPECT_FALSE(header_only.value().record_scan_complete);
  ExpectAccounting(header_only.value());
}

TEST(ParasolidInspectorTest, RetainsUnknownSchemaBodyWithoutScanning) {
  auto bytes = test::Header("", "SCH_future_schema");
  test::Point(bytes);
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result.value().record_scan_performed);
  EXPECT_FALSE(result.value().record_scan_complete);
  EXPECT_TRUE(result.value().candidates.empty());
  EXPECT_EQ(result.value().uninterpreted_bytes, 40u);
  ExpectAccounting(result.value());
}

TEST(ParasolidInspectorTest, KeepsUserFieldLayoutsOpaque) {
  auto bytes = test::Header();
  bytes.back() = 1;
  test::Point(bytes);
  const auto result = ParasolidInspector::Inspect(bytes);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().header.user_field_words, 1);
  EXPECT_FALSE(result.value().record_scan_performed);
  EXPECT_TRUE(result.value().candidates.empty());
  ExpectAccounting(result.value());
}

TEST(ParasolidInspectorTest, EnforcesInputAndOptionLimits) {
  const auto bytes = test::Header();
  InspectOptions options;
  options.max_input_bytes = bytes.size() - 1u;
  const auto limited = ParasolidInspector::Inspect(bytes, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  options.max_input_bytes = 0;
  EXPECT_FALSE(ParasolidInspector::Inspect(bytes, options).ok());
  options = {};
  options.max_records = 1000001u;
  const auto invalid = ParasolidInspector::Inspect(bytes, options);
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
}

TEST(ParasolidInspectorTest, ReadsFilesAndReportsIoErrors) {
  const auto path = std::filesystem::path(testing::TempDir()) /
                    "openparasolid-inspector-test.x_b";
  const auto bytes = test::LinkedRecords();
  {
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(file.good());
  }
  const auto result = ParasolidInspector::InspectFile(path);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().candidates.size(), 6u);
  InspectOptions options;
  options.max_input_bytes = bytes.size() - 1u;
  const auto limited = ParasolidInspector::InspectFile(path, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  std::error_code error;
  std::filesystem::remove(path, error);
  const auto missing = ParasolidInspector::InspectFile(path);
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.error().code, ErrorCode::kIo);
}

}  // namespace
}  // namespace openparasolid
