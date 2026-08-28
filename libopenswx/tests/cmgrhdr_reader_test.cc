// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "internal/cmgrhdr_reader.h"

namespace openswx::internal {
namespace {

void AppendU16Le(std::vector<uint8_t>* data, uint16_t value) {
  data->push_back(static_cast<uint8_t>(value & 0xFFu));
  data->push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

void AppendU32Le(std::vector<uint8_t>* data, uint32_t value) {
  data->push_back(static_cast<uint8_t>(value & 0xFFu));
  data->push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
  data->push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
  data->push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}

// MFC WriteObject: declares a brand-new class (schema + ASCII name).
void AppendNewClassTag(std::vector<uint8_t>* data, const std::string& name) {
  AppendU16Le(data, 0xFFFF);
  AppendU16Le(data, 1);  // schema — value is irrelevant to the reader
  AppendU16Le(data, static_cast<uint16_t>(name.size()));
  data->insert(data->end(), name.begin(), name.end());
}

// MFC WriteObject: back-reference to a class already declared via
// AppendNewClassTag (1-based index, matching cmgrhdr_detail::ReadClassTag).
void AppendClassBackRef(std::vector<uint8_t>* data, int class_index) {
  AppendU16Le(data, static_cast<uint16_t>(0x8000 | class_index));
}

// MFC CString, short Unicode form: FF FE <len_byte> <UTF-16LE>.
void AppendCString(std::vector<uint8_t>* data, const std::string& ascii) {
  data->push_back(0xFF);
  data->push_back(0xFE);
  data->push_back(static_cast<uint8_t>(ascii.size()));
  for (char c : ascii) {
    data->push_back(static_cast<uint8_t>(c));
    data->push_back(0x00);
  }
}

// Appends one dmConfigHeader_c record.
//   first_record: whether this is the first config (declares the class new)
//     vs. a later one (back-references it, class_index 2 — the outer
//     dmConfigMgrHeader_c object always takes class_index 1).
//   with_parent_field: whether to serialise fields 5–6 (requires the caller's
//     chosen version to be >= kCfgVerParent).
void AppendConfigRecord(std::vector<uint8_t>* data, bool first_record,
                        const std::string& name, uint32_t config_id,
                        bool with_unk1_field, bool with_parent_field,
                        std::optional<uint32_t> parent_id) {
  if (first_record)
    AppendNewClassTag(data, "dmConfigHeader_c");
  else
    AppendClassBackRef(data, 2);

  AppendU32Le(data, 0);              // field 1: DWORD this+0x08
  AppendCString(data, name);         // field 2: config name
  AppendU32Le(data, config_id);      // field 3: configId

  if (!with_unk1_field) return;
  AppendU32Le(data, 0);              // field 4: DWORD this+0x38

  if (!with_parent_field) return;
  AppendCString(data, "");           // field 5: CStr (unlabelled)
  AppendU32Le(data, parent_id.value_or(0xFFFFFFFFu));  // field 6: parent_index
}

std::vector<uint8_t> BuildStream(
    const std::vector<std::tuple<std::string, uint32_t,
                                 std::optional<uint32_t>>>& configs,
    bool with_unk1_field, bool with_parent_field) {
  std::vector<uint8_t> data;
  AppendNewClassTag(&data, "dmConfigMgrHeader_c");  // outer object, class_index 1
  AppendU16Le(&data, static_cast<uint16_t>(configs.size()));  // WriteCount

  bool first = true;
  for (const auto& [name, config_id, parent_id] : configs) {
    AppendConfigRecord(&data, first, name, config_id, with_unk1_field,
                       with_parent_field, parent_id);
    first = false;
  }
  return data;
}

TEST(CMgrHdr2ReaderTest, EmptyStreamReturnsEmptyMap) {
  EXPECT_TRUE(ParseCMgrHdr2({}).empty());
}

TEST(CMgrHdr2ReaderTest, ParsesNamesWithoutParentField) {
  // version 0 — only the three mandatory fields are serialised, matching
  // very old documents that predate the parent-index field entirely.
  auto data = BuildStream(
      {{"rv1", 12, std::nullopt}, {"rv2", 13, std::nullopt}},
      /*with_unk1_field=*/false, /*with_parent_field=*/false);

  auto result = ParseCMgrHdr2(data, /*version=*/0);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result.at(12).name, "rv1");
  EXPECT_FALSE(result.at(12).parent_index.has_value());
  EXPECT_EQ(result.at(13).name, "rv2");
  EXPECT_FALSE(result.at(13).parent_index.has_value());
}

// Mirrors a real pairing observed across 6 production files: an independent
// base configuration, a derived configuration pointing back at it (created
// either via "Add Derived Configuration" or as an auto-generated Sheet-Metal
// flat pattern — both use this same field), and a second independent
// configuration to confirm the parser doesn't desynchronise partway through.
TEST(CMgrHdr2ReaderTest, ParsesParentIndexForDerivedConfiguration) {
  auto data = BuildStream(
      {{"rv1", 12, std::nullopt},
       {"rv1SM-FLAT-PATTERN", 16, 12u},
       {"rv2", 13, std::nullopt}},
      /*with_unk1_field=*/true, /*with_parent_field=*/true);

  auto result = ParseCMgrHdr2(data, /*version=*/static_cast<int>(kCfgVerParent));

  ASSERT_EQ(result.size(), 3u);

  EXPECT_EQ(result.at(12).name, "rv1");
  EXPECT_FALSE(result.at(12).parent_index.has_value());

  EXPECT_EQ(result.at(16).name, "rv1SM-FLAT-PATTERN");
  ASSERT_TRUE(result.at(16).parent_index.has_value());
  EXPECT_EQ(*result.at(16).parent_index, 12);

  EXPECT_EQ(result.at(13).name, "rv2");
  EXPECT_FALSE(result.at(13).parent_index.has_value());
}

TEST(CMgrHdr2ReaderTest, VersionBelowThresholdLeavesParentIndexUnset) {
  // A document version between kCfgVerUnk1 and kCfgVerParent: field 4 is
  // present in the bytes and read, but fields 5-6 (the parent-index pair)
  // were never serialised by this SW version — matching a real older file,
  // not just a truncated stream.
  auto data = BuildStream(
      {{"rv1", 12, std::nullopt}, {"rv1SM-FLAT-PATTERN", 16, std::nullopt}},
      /*with_unk1_field=*/true, /*with_parent_field=*/false);

  auto result = ParseCMgrHdr2(
      data, /*version=*/static_cast<int>(kCfgVerParent) - 1);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result.at(12).name, "rv1");
  EXPECT_EQ(result.at(16).name, "rv1SM-FLAT-PATTERN");
  EXPECT_FALSE(result.at(16).parent_index.has_value());
}

}  // namespace
}  // namespace openswx::internal
