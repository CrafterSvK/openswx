// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>

#include "openswx/drawing_geometry.h"

namespace openswx {
namespace {

void AppendF64(std::vector<uint8_t> &data, double value) {
  const std::size_t offset = data.size();
  data.resize(offset + sizeof(value));
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void AppendF32(std::vector<uint8_t> &data, float value) {
  const std::size_t offset = data.size();
  data.resize(offset + sizeof(value));
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void AppendU64(std::vector<uint8_t> &data, uint64_t value) {
  for (std::size_t index = 0; index < 8u; ++index) {
    data.push_back(static_cast<uint8_t>((value >> (index * 8u)) & 0xFFu));
  }
}

void AppendU32(std::vector<uint8_t> &data, uint32_t value) {
  for (std::size_t index = 0; index < 4u; ++index) {
    data.push_back(static_cast<uint8_t>((value >> (index * 8u)) & 0xFFu));
  }
}

void AppendTessellationDescriptor(std::vector<uint8_t> &data,
                                  uint32_t item_size, uint32_t kind,
                                  uint32_t count) {
  AppendU32(data, item_size);
  AppendU32(data, kind);
  AppendU32(data, 2u);
  AppendU32(data, count);
}

void AppendUtf16(std::vector<uint8_t> &data, std::string_view value);

void AppendDisplayTextStyle(std::vector<uint8_t> &data, double height) {
  AppendU32(data, 108u);
  AppendU32(data, 10u);
  AppendF64(data, height);
  AppendF64(data, 0.0);
  data.resize(data.size() + 16u, 0);
  AppendUtf16(data, "SWTxt");
}

void AppendDisplayObjectReference(std::vector<uint8_t> &data, uint32_t id) {
  AppendUtf16(data, "<objID=" + std::to_string(id) + ">");
}

void AppendDefinitionObjectId(std::vector<uint8_t> &data, uint32_t id) {
  AppendU32(data, id);
  data.insert(data.end(), {0, 0, 0, 0, 0, 0, 0x38, 0x4A});
}

void AppendDisplayLine(std::vector<uint8_t> &data, Vec3 start, Vec3 end) {
  AppendU32(data, 80u);
  AppendU32(data, 4u);
  AppendF64(data, start.x);
  AppendF64(data, start.y);
  AppendF64(data, start.z);
  AppendF64(data, end.x);
  AppendF64(data, end.y);
  AppendF64(data, end.z);
}

void AppendDisplayArc(std::vector<uint8_t> &data, Vec3 start, Vec3 end,
                      Vec3 center, double normal_z = 1.0) {
  AppendU32(data, 128u);
  AppendU32(data, 2u);
  for (const Vec3 point : {start, end, center}) {
    AppendF64(data, point.x);
    AppendF64(data, point.y);
    AppendF64(data, point.z);
  }
  AppendF64(data, 0.0);
  AppendF64(data, 0.0);
  AppendF64(data, normal_z);
}

void AppendDisplayText(std::vector<uint8_t> &data, uint32_t opcode,
                       std::string_view text, Vec3 origin, double rotation) {
  AppendU32(data, opcode);
  AppendU32(data, 18u);
  AppendF64(data, origin.x);
  AppendF64(data, origin.y);
  AppendF64(data, origin.z);
  AppendU32(data, 0u);
  AppendF64(data, rotation);
  AppendU32(data, 0u);
  AppendUtf16(data, text);
}

void AppendUtf16(std::vector<uint8_t> &data, std::string_view value) {
  data.insert(data.end(),
              {0xFF, 0xFE, 0xFF, static_cast<uint8_t>(value.size())});
  for (const char character : value) {
    data.push_back(static_cast<uint8_t>(character));
    data.push_back(0);
  }
}

void SetF64(std::vector<uint8_t> &data, std::size_t offset, double value) {
  ASSERT_LE(offset + sizeof(value), data.size());
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void AppendSheetSketchPoint(std::vector<uint8_t> &data, double x, double y,
                            bool current = false, uint16_t state = 4u,
                            uint16_t coordinate_tag = 0x001Eu) {
  const std::size_t offset = data.size();
  data.resize(offset + 142u, 0);
  const std::array<uint8_t, 5> marker =
      current ? std::array<uint8_t, 5>{0xFF, 0xFF, 0x1F, 0x00, 0x03}
              : std::array<uint8_t, 5>{0xFF, 0xFF, 0x07, 0x00, 0x01};
  std::copy(marker.begin(), marker.end(), data.begin() + offset);
  const std::array<uint8_t, 4> locus = {0x05, 0x00, 0x01, 0x00};
  std::copy(locus.begin(), locus.end(), data.begin() + offset + 23u);
  data[offset + 27u] = 1u;
  data[offset + 33u] = 0x80u;
  data[offset + 34u] = 0xBFu;
  data[offset + 37u] = static_cast<uint8_t>(state & 0xFFu);
  data[offset + 38u] = static_cast<uint8_t>(state >> 8u);
  SetF64(data, offset + 48u, 1.0);
  data[offset + 64u] = static_cast<uint8_t>(coordinate_tag & 0xFFu);
  data[offset + 65u] = static_cast<uint8_t>(coordinate_tag >> 8u);
  SetF64(data, offset + 66u, x);
  SetF64(data, offset + 74u, y);
}

void AppendSheetSketchLine(std::vector<uint8_t> &data, uint16_t start,
                           uint16_t end, uint16_t state = 4u) {
  const std::size_t offset = data.size();
  data.resize(offset + 92u, 0);
  const std::array<uint8_t, 5> marker = {0xFF, 0xFF, 0x07, 0x00, 0x01};
  std::copy(marker.begin(), marker.end(), data.begin() + offset);
  const std::array<uint8_t, 4> locus = {0x05, 0x00, 0x01, 0x00};
  std::copy(locus.begin(), locus.end(), data.begin() + offset + 23u);
  data[offset + 27u] = 1u;
  data[offset + 33u] = 0x80u;
  data[offset + 34u] = 0xBFu;
  data[offset + 37u] = static_cast<uint8_t>(state & 0xFFu);
  data[offset + 38u] = static_cast<uint8_t>(state >> 8u);
  SetF64(data, offset + 48u, 1.0);
  data[offset + 64u] = static_cast<uint8_t>(start & 0xFFu);
  data[offset + 65u] = static_cast<uint8_t>(start >> 8u);
  data[offset + 66u] = static_cast<uint8_t>(end & 0xFFu);
  data[offset + 67u] = static_cast<uint8_t>(end >> 8u);
  SetF64(data, offset + 72u, -1.0);
}

void AppendSheetFormatNote(std::vector<uint8_t> &data, std::string_view text,
                           double x, double y, std::string_view font,
                           double height, uint16_t serializer_tag = 0x830Du) {
  data.insert(data.end(), {0x05, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           static_cast<uint8_t>(serializer_tag & 0xFFu),
                           static_cast<uint8_t>(serializer_tag >> 8u)});
  AppendUtf16(data, text);
  AppendF64(data, x);
  AppendF64(data, y);
  AppendF64(data, 0.0);
  data.insert(data.end(), {1, 0, 0, 0, 0x0F, 0x83});
  AppendUtf16(data, font);
  AppendF64(data, height);
}

void AppendWeldmentCellText(std::vector<uint8_t> &data, std::string_view text,
                            double x, double y, double width,
                            double char_format_height = -1.0,
                            double measured_height = 0.003) {
  AppendUtf16(data, text);
  AppendF64(data, x);
  AppendF64(data, y);
  AppendF64(data, 0.0);
  AppendUtf16(data, "Century Gothic");
  const std::size_t font_end = data.size();
  data.resize(font_end + 120u, 0);
  SetF64(data, font_end, char_format_height);
  AppendF64(data, width);
  AppendF64(data, measured_height);
}

TEST(DrawingViewGeometryTest, ReadsAndProjectsConfirmedCachedSegment) {
  const std::string_view class_name = "uiViewBucket_c";
  std::vector<uint8_t> data = {
      0x01, 0x00, 0x00,
      0x00, 0xFF, 0xFF,
      0x01, 0x00, static_cast<uint8_t>(class_name.size()),
      0x00};
  data.insert(data.end(), class_name.begin(), class_name.end());
  AppendF64(data, 0.0);
  AppendF64(data, 1.0);
  AppendF64(data, 0.0);
  AppendF64(data, 1.0);
  AppendF64(data, 2.0);
  data.push_back(25);
  data.push_back(0);
  const std::array<uint8_t, 8> marker = {0x21, 0x03, 0x00, 0x00,
                                         0x02, 0x00, 0x00, 0x00};
  data.insert(data.end(), marker.begin(), marker.end());
  for (double value : {2.0, 0.0, 4.0, 4.0, 0.0, 8.0}) {
    AppendF64(data, value);
  }
  const std::array<uint8_t, 8> alternate_marker = {0x23, 0x03, 0x00, 0x00,
                                                   0x02, 0x00, 0x00, 0x00};
  data.insert(data.end(), alternate_marker.begin(), alternate_marker.end());
  for (double value : {6.0, 0.0, 10.0, 8.0, 0.0, 12.0}) {
    AppendF64(data, value);
  }
  // The cache can repeat a shared topological edge in the opposite direction.
  // It must only be emitted and classified once.
  data.insert(data.end(), marker.begin(), marker.end());
  for (double value : {4.0 + 1e-9, 0.0, 8.0, 2.0, 0.0, 4.0}) {
    AppendF64(data, value);
  }
  const std::array<uint8_t, 8> suppressed_marker = {0x01, 0x03, 0x00, 0x00,
                                                    0x02, 0x00, 0x00, 0x00};
  data.insert(data.end(), suppressed_marker.begin(), suppressed_marker.end());
  for (double value : {10.0, 0.0, 14.0, 12.0, 0.0, 16.0}) {
    AppendF64(data, value);
  }
  const std::string_view reference = "part@Drawing View1";
  data.insert(data.end(),
              {0xFF, 0xFE, 0xFF, static_cast<uint8_t>(reference.size())});
  for (const char character : reference) {
    data.push_back(static_cast<uint8_t>(character));
    data.push_back(0);
  }

  auto result = DrawingGeometryReader::ReadViewBuckets(data);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  const CachedDrawingView &view = result.value().front();
  EXPECT_EQ(view.name, "Drawing View1");
  EXPECT_EQ(view.referenced_document, "part");
  EXPECT_DOUBLE_EQ(view.scale_denominator, 2.0);
  ASSERT_EQ(view.geometry.entities.size(), 2u);
  const Line2D &line = std::get<Line2D>(view.geometry.entities.front());
  EXPECT_DOUBLE_EQ(line.start.x, -1.0);
  EXPECT_DOUBLE_EQ(line.start.y, 2.0);
  EXPECT_DOUBLE_EQ(line.end.x, -2.0);
  ASSERT_EQ(view.source_lines.size(), 2u);
  EXPECT_EQ(view.source_lines.front().source_offset, 66u);
  EXPECT_EQ(view.source_lines.front().source_record_type, 0x21u);
  EXPECT_EQ(view.source_lines[1].source_offset, 122u);
  EXPECT_EQ(view.source_lines[1].source_record_type, 0x23u);
  EXPECT_TRUE(view.has_suppressed_edge_cache);
  EXPECT_DOUBLE_EQ(line.end.y, 4.0);
}

TEST(DrawingViewGeometryTest, ReadsDefinitionMatrixPlacementAndClipBounds) {
  constexpr std::size_t placement_offset = 400u;
  constexpr std::size_t matrix_offset = placement_offset - 317u;
  std::vector<uint8_t> data(placement_offset + 88u, 0);
  const std::string_view name = "Drawing View1";
  const std::array<uint8_t, 5> name_marker = {0x09, 0x89, 0xFF, 0xFE, 0xFF};
  std::copy(name_marker.begin(), name_marker.end(), data.begin());
  data[name_marker.size()] = static_cast<uint8_t>(name.size());
  for (std::size_t index = 0; index < name.size(); ++index) {
    data[name_marker.size() + 1u + index * 2u] =
        static_cast<uint8_t>(name[index]);
  }
  const std::array<double, 9> matrix = {1.0, 0.0, 0.0, 0.0, 1.0,
                                        0.0, 0.0, 0.0, 1.0};
  for (std::size_t index = 0; index < matrix.size(); ++index) {
    SetF64(data, matrix_offset + index * 8u, matrix[index]);
  }
  SetF64(data, matrix_offset + 72u, 0.1);
  SetF64(data, matrix_offset + 80u, 0.2);
  SetF64(data, matrix_offset + 88u, 0.3);
  SetF64(data, matrix_offset + 96u, 1.0 / 15.0);
  const std::array<double, 11> placement = {
      0.10, 0.05, 0.30, 0.25, 0.106, 0.057, 0.291, 0.238, 0.34, 0.02, 0.0};
  for (std::size_t index = 0; index < placement.size(); ++index) {
    SetF64(data, placement_offset + index * 8u, placement[index]);
  }
  const std::vector<std::string> names = {std::string(name)};

  auto result = DrawingGeometryReader::ReadViewPlacements(data, names);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  const DrawingViewPlacement &parsed = result.value().front();
  EXPECT_EQ(parsed.name, name);
  EXPECT_DOUBLE_EQ(parsed.scale, 1.0 / 15.0);
  EXPECT_DOUBLE_EQ(parsed.center.x, 0.34);
  EXPECT_DOUBLE_EQ(parsed.center.y, 0.02);
  EXPECT_DOUBLE_EQ(parsed.clip_bounds.min.x, 0.106);
  EXPECT_DOUBLE_EQ(parsed.clip_bounds.max.y, 0.238);
  EXPECT_DOUBLE_EQ(parsed.translation.x, 0.1);
  EXPECT_DOUBLE_EQ(parsed.translation.y, 0.2);
  EXPECT_DOUBLE_EQ(parsed.translation.z, 0.3);
  EXPECT_EQ(parsed.model_to_view, matrix);
}

TEST(DrawingViewGeometryTest, ReadsDisplayListLinesOwnedBySelectedView) {
  std::vector<uint8_t> display_lists;
  AppendUtf16(display_lists, "part@Drawing View1");
  AppendDisplayLine(display_lists, {0.01, 0.02, -0.1}, {0.11, 0.02, -0.1});
  AppendDisplayLine(display_lists, {0.11, 0.02, -0.1}, {0.01, 0.02, -0.1});
  AppendUtf16(display_lists, "part@Drawing View2");
  AppendDisplayLine(display_lists, {0.20, 0.03, 0.0}, {0.30, 0.03, 0.0});
  AppendDisplayLine(display_lists, {0.20, 0.40, 0.0}, {0.30, 0.40, 0.0});
  const std::vector<std::string> all = {"Drawing View1", "Drawing View2"};
  const std::vector<std::string> selected = {"Drawing View1"};

  auto result = DrawingGeometryReader::ReadDisplayListLines(
      display_lists, all, selected, 0.420, 0.297);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_DOUBLE_EQ(result.value().front().start.x, 0.01);
  EXPECT_DOUBLE_EQ(result.value().front().end.x, 0.11);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/DisplayLists/line/Drawing View1");
}

TEST(DrawingViewGeometryTest, ReadsDisplayListCircularArcOwnedByObject) {
  std::vector<uint8_t> display_lists;
  AppendDisplayObjectReference(display_lists, 42u);
  AppendDisplayArc(display_lists, {0.08, 0.12, -0.1}, {0.08, 0.16, -0.1},
                   {0.06, 0.14, -0.1});
  const std::array<uint32_t, 1> selected_ids = {42u};

  auto result = DrawingGeometryReader::ReadDisplayListArcs(
      display_lists, {}, {}, 0.420, 0.297, selected_ids);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_DOUBLE_EQ(result.value().front().start.x, 0.08);
  EXPECT_DOUBLE_EQ(result.value().front().end.y, 0.16);
  EXPECT_DOUBLE_EQ(result.value().front().center.x, 0.06);
  EXPECT_DOUBLE_EQ(result.value().front().normal_z, 1.0);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/DisplayLists/arc/objID=42");
}

TEST(DrawingViewGeometryTest, ReadsLinePatternAndFlatSheetEndOwnership) {
  std::vector<uint8_t> display_lists;
  AppendUtf16(display_lists, "CONTINUOUS");
  AppendDisplayLine(display_lists, {0.01, 0.02, 0.0}, {0.11, 0.02, 0.0});
  AppendUtf16(display_lists, "List1");
  AppendUtf16(display_lists, "CENTER");
  AppendDisplayLine(display_lists, {0.20, 0.03, 0.0}, {0.30, 0.03, 0.0});
  AppendUtf16(display_lists, "List2-Active");
  const std::vector<std::string> sheets = {"List1", "List2"};

  auto first = DrawingGeometryReader::ReadDisplayListLines(
      display_lists, {}, {}, 0.420, 0.297, {}, sheets, "List1");
  auto second = DrawingGeometryReader::ReadDisplayListLines(
      display_lists, {}, {}, 0.420, 0.297, {}, sheets, "List2");

  ASSERT_TRUE(first.ok()) << first.error();
  ASSERT_TRUE(second.ok()) << second.error();
  ASSERT_EQ(first.value().size(), 1u);
  ASSERT_EQ(second.value().size(), 1u);
  EXPECT_EQ(first.value().front().style.pattern, LinePattern::kSolid);
  EXPECT_EQ(second.value().front().style.pattern, LinePattern::kCenter);
  EXPECT_EQ(second.value().front().source_table,
            "Contents/DisplayLists/line/sheet=List2");
}

TEST(DrawingViewGeometryTest, ReadsExtendedTextOpcodeOwnedBySheetEnd) {
  std::vector<uint8_t> display_lists;
  AppendDisplayText(display_lists, 356u, "Resolved title", {0.30, 0.02, 0.0},
                    0.0);
  AppendUtf16(display_lists, "List1");
  AppendDisplayText(display_lists, 356u, "Other sheet", {0.30, 0.02, 0.0}, 0.0);
  AppendUtf16(display_lists, "List2-Active");
  const std::vector<std::string> sheets = {"List1", "List2"};

  auto result = DrawingGeometryReader::ReadDisplayListText(
      display_lists, {}, {}, 0.420, 0.297, {}, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front().text, "Resolved title");
  EXPECT_DOUBLE_EQ(result.value().front().height, 0.0015);
  EXPECT_EQ(result.value().front().anchor, TextAnchor::kStart);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/DisplayLists/text/356/sheet=List1");
}

TEST(DrawingViewGeometryTest, ReadsRawRgbaSketchBitmapPlacement) {
  std::vector<uint8_t> definition;
  const std::string_view bitmap_class = "moSketchBitmapData_c";
  definition.insert(definition.end(), bitmap_class.begin(), bitmap_class.end());
  AppendF64(definition, 0.38);
  AppendF64(definition, 0.03);
  AppendF64(definition, 0.0);
  AppendF64(definition, 0.0001);
  AppendF64(definition, 0.0002);
  AppendU64(definition, 0u);
  AppendU32(definition, 2u);
  AppendU32(definition, 1u);
  std::vector<uint8_t> bitmap = {0xFF, 0xFF};
  AppendU32(bitmap, 8u);
  bitmap.insert(bitmap.end(), {255, 0, 0, 255, 0, 255, 0, 255});

  auto result = DrawingGeometryReader::ReadSketchBitmap(definition, bitmap);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_DOUBLE_EQ(result.value().front().bottom_left.x, 0.38);
  EXPECT_DOUBLE_EQ(result.value().front().bottom_left.y, 0.03);
  EXPECT_DOUBLE_EQ(result.value().front().width, 0.0002);
  EXPECT_DOUBLE_EQ(result.value().front().height, 0.0002);
  EXPECT_EQ(result.value().front().pixel_width, 2u);
  EXPECT_EQ(result.value().front().pixel_height, 1u);
  EXPECT_EQ(result.value().front().rgba.size(), 8u);
}

TEST(DrawingViewGeometryTest, ReadsRotatedDisplayListTextBySelectedView) {
  std::vector<uint8_t> display_lists;
  AppendUtf16(display_lists, "part@Drawing View1");
  AppendDisplayTextStyle(display_lists, 0.00146666666666667);
  AppendDisplayText(display_lists, 176u, "22deg", {0.10, 0.20, -0.1},
                    0.2617993877991494);
  AppendDisplayText(display_lists, 186u, "1301", {0.12, 0.22, -0.1},
                    6.283185307179586);
  AppendUtf16(display_lists, "part@Drawing View2");
  AppendDisplayText(display_lists, 166u, "hidden", {0.20, 0.20, 0.0}, 0.0);
  const std::vector<std::string> all = {"Drawing View1", "Drawing View2"};
  const std::vector<std::string> selected = {"Drawing View1"};

  auto result = DrawingGeometryReader::ReadDisplayListText(
      display_lists, all, selected, 0.420, 0.297);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 2u);
  EXPECT_EQ(result.value().front().text, "22deg");
  EXPECT_DOUBLE_EQ(result.value().front().origin.x, 0.10);
  EXPECT_NEAR(result.value().front().rotation_radians, 0.2617993877991494,
              1e-12);
  EXPECT_NEAR(result.value().front().height, 0.00146666666666667, 1e-15);
  EXPECT_EQ(result.value().front().anchor, TextAnchor::kStart);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/DisplayLists/text/176/Drawing View1");
  EXPECT_DOUBLE_EQ(result.value().back().rotation_radians, 0.0);
}

TEST(DrawingViewGeometryTest, PrefersDefinitionObjectOwnershipOverViewHint) {
  std::vector<uint8_t> display_lists;
  AppendUtf16(display_lists, "part@Drawing View1");
  AppendDisplayObjectReference(display_lists, 101u);
  AppendDisplayLine(display_lists, {0.01, 0.02, 0.0}, {0.11, 0.02, 0.0});
  AppendDisplayObjectReference(display_lists, 202u);
  AppendDisplayLine(display_lists, {0.20, 0.03, 0.0}, {0.30, 0.03, 0.0});
  const std::vector<std::string> views = {"Drawing View1"};
  const std::vector<uint32_t> selected_ids = {202u};

  auto result = DrawingGeometryReader::ReadDisplayListLines(
      display_lists, views, views, 0.420, 0.297, selected_ids);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_DOUBLE_EQ(result.value().front().start.x, 0.20);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/DisplayLists/line/objID=202");
}

TEST(DrawingViewGeometryTest, ReadsDefinitionDisplayObjectIdsBySheet) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List3");
  AppendDefinitionObjectId(definition, 101u);
  AppendUtf16(definition, "List4");
  AppendDefinitionObjectId(definition, 202u);
  const std::vector<std::string> sheets = {"List3", "List4"};

  auto result =
      DrawingGeometryReader::ReadDisplayObjectIds(definition, sheets, "List4");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front(), 202u);
}

TEST(DrawingViewGeometryTest, ReadsSolidWorks19000NameWithoutObjectPrefix) {
  constexpr std::size_t placement_offset = 400u;
  constexpr std::size_t matrix_offset = placement_offset - 317u;
  std::vector<uint8_t> data(placement_offset + 88u, 0);
  const std::string_view name = "Drawing View1";
  const std::array<uint8_t, 3> name_marker = {0xFF, 0xFE, 0xFF};
  std::copy(name_marker.begin(), name_marker.end(), data.begin());
  data[name_marker.size()] = static_cast<uint8_t>(name.size());
  for (std::size_t index = 0; index < name.size(); ++index) {
    data[name_marker.size() + 1u + index * 2u] =
        static_cast<uint8_t>(name[index]);
  }
  const std::array<double, 9> matrix = {1.0,  0.0, 0.0, 0.0, 0.0,
                                        -1.0, 0.0, 1.0, 0.0};
  for (std::size_t index = 0; index < matrix.size(); ++index) {
    SetF64(data, matrix_offset + index * 8u, matrix[index]);
  }
  SetF64(data, matrix_offset + 96u, 0.1);
  const std::array<double, 11> placement = {
      0.05, 0.03, 0.36, 0.25, 0.056, 0.036, 0.354, 0.244, 0.205, 0.14, 0.0};
  for (std::size_t index = 0; index < placement.size(); ++index) {
    SetF64(data, placement_offset + index * 8u, placement[index]);
  }
  const std::vector<std::string> names = {std::string(name)};

  auto result = DrawingGeometryReader::ReadViewPlacements(data, names);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front().name, name);
  EXPECT_DOUBLE_EQ(result.value().front().scale, 0.1);
}

TEST(DrawingViewGeometryTest, ReadsSolidWorks19000LinearDimension) {
  std::vector<uint8_t> data(32u, 0);
  AppendUtf16(data, "List1");
  data.insert(data.end(), {0xFF, 0xFE, 0xFF, 0x00});
  AppendF64(data, 0.03);
  AppendF64(data, 0.12);
  AppendF64(data, -0.08295);
  AppendU64(data, 18u);
  data.resize(data.size() + 48u, 0);
  const std::size_t rd_offset = data.size();
  AppendUtf16(data, "RD1");
  data.resize(data.size() + 32u, 0);
  for (double value : {0.0, -0.4, 0.8, 0.0, 0.1, -0.5}) {
    AppendF64(data, value);
  }
  data.resize(data.size() + 24u, 0);
  AppendF64(data, 1.3);
  AppendF64(data, 0.0);
  AppendF64(data, 1.3);
  AppendF64(data, 0.0);
  AppendF64(data, 0.0);
  data.resize(data.size() + 16u, 0);
  AppendUtf16(data, "1300");
  const std::vector<std::string> sheets = {"List1", "List2"};

  auto result =
      DrawingGeometryReader::ReadLegacyDimensions(data, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  const Dimension &dimension = result.value().front();
  EXPECT_EQ(dimension.text, "1300");
  EXPECT_DOUBLE_EQ(dimension.text_origin.x, 0.03);
  EXPECT_DOUBLE_EQ(dimension.text_origin.y, 0.12);
  EXPECT_DOUBLE_EQ(dimension.measured_value_m, 1.3);
  EXPECT_EQ(dimension.orientation, DimensionOrientation::kVertical);
  EXPECT_EQ(dimension.source_offset, rd_offset);
}

TEST(DrawingViewGeometryTest, ReadsSolidWorks19000BomBalloonAndLeader) {
  std::vector<uint8_t> data;
  AppendUtf16(data, "List1");
  data.resize(96u, 0);
  AppendF64(data, 0.281063416);
  AppendF64(data, 0.191343772);
  AppendF64(data, 0.044809957);
  data.resize(data.size() + 128u, 0);
  const std::size_t label_offset = data.size();
  AppendUtf16(data, "2");
  AppendF64(data, 0.263291903);
  AppendF64(data, 0.188279718);
  AppendF64(data, 0.044809957);
  data.resize(data.size() + 64u, 0);
  AppendF64(data, 0.281063416);
  AppendF64(data, 0.191343772);
  AppendF64(data, 0.044809957);
  const std::vector<std::string> sheets = {"List1"};

  auto result =
      DrawingGeometryReader::ReadLegacyBomBalloons(data, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  const BomBalloon &balloon = result.value().front();
  EXPECT_EQ(balloon.text, "2");
  EXPECT_DOUBLE_EQ(balloon.text_origin.x, 0.263291903);
  EXPECT_DOUBLE_EQ(balloon.text_origin.y, 0.188279718);
  EXPECT_DOUBLE_EQ(balloon.leader_anchor.x, 0.281063416);
  EXPECT_DOUBLE_EQ(balloon.leader_anchor.y, 0.191343772);
  EXPECT_EQ(balloon.source_offset, label_offset);
}

TEST(DrawingViewGeometryTest, UsesSerializedBomBalloonSheetOwnership) {
  std::vector<uint8_t> data;
  AppendUtf16(data, "List1");
  data.resize(96u, 0);
  AppendF64(data, 0.28);
  AppendF64(data, 0.19);
  AppendF64(data, 0.04);
  data.resize(data.size() + 32u, 0);
  AppendUtf16(data, "1");
  AppendF64(data, 0.30);
  AppendF64(data, 0.20);
  AppendF64(data, 0.04);
  AppendF64(data, 0.28);
  AppendF64(data, 0.19);
  AppendF64(data, 0.04);
  AppendUtf16(data, "List2");
  data.resize(data.size() + 32u, 0);
  const std::vector<std::string> sheets = {"List1", "List2"};

  auto result =
      DrawingGeometryReader::ReadLegacyBomBalloons(data, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front().text, "1");
}

TEST(DrawingViewGeometryTest, RejectsIndirectMagnetLineBalloonLabels) {
  std::vector<uint8_t> data;
  AppendUtf16(data, "List1");
  data.insert(data.end(),
              {'m', 'o', 'D', 'i', 's', 'p', 'l', 'a', 'y', 'M', 'a',
               'g', 'n', 'e', 't', 'L', 'i', 'n', 'e', '_', 'c'});
  const std::vector<std::string> sheets = {"List1"};

  auto result =
      DrawingGeometryReader::ReadLegacyBomBalloons(data, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_TRUE(result.value().empty());
}

TEST(DrawingViewGeometryTest, ComposesTableOnEmptyDrawingSheet) {
  const std::string keywords =
      "<Keywords><Sheet Name=\"List1\" Type=\"Sheet\"/>"
      "<Sheet Name=\"Sheet Format1\" Type=\"Sheet Format\"/></Keywords>";
  std::vector<uint8_t> definition(32u, 0);
  SetF64(definition, 0u, 0.420);
  SetF64(definition, 8u, 0.297);
  const std::vector<uint8_t> vb_lists(8u, 0);
  const std::string tables =
      "<Tables><Table Display_Name=\"Revision table\" SheetName=\"List1\">"
      "<GridLines><Line><StartPt X=\"0.3\" Y=\"0.02\"/>"
      "<EndPt X=\"0.4\" Y=\"0.02\"/></Line></GridLines>"
      "</Table></Tables>";
  const auto bytes = [](const std::string &value) {
    return std::span<const uint8_t>(
        reinterpret_cast<const uint8_t *>(value.data()), value.size());
  };

  auto result = DrawingGeometryReader::ReadCachedSheet(
      bytes(keywords), definition, vb_lists, bytes(tables), "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().name, "List1");
  EXPECT_DOUBLE_EQ(result.value().width_m, 0.420);
  EXPECT_DOUBLE_EQ(result.value().height_m, 0.297);
  ASSERT_EQ(result.value().entities.size(), 1u);
  EXPECT_NE(std::get_if<Line2D>(&result.value().entities.front()), nullptr);
}

TEST(DrawingViewGeometryTest, ReadsNativeWeldmentTableGrid) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  const auto append_class = [&](std::string_view value) {
    definition.insert(definition.end(), value.begin(), value.end());
  };
  append_class("moWeldmentTableColumn_c");
  AppendF64(definition, 0.01);
  for (double width : {0.05, 0.02}) {
    definition.insert(definition.end(), {0x92, 0x8B});
    AppendF64(definition, width);
  }
  append_class("moWeldmentTableRow_c");
  AppendF64(definition, 0.005);
  append_class("moWeldmentTableGrid_c");
  for (const auto [number, y] :
       std::array<std::pair<std::string_view, double>, 3>{
           {{"3", 0.100}, {"2", 0.095}, {"1", 0.090}}}) {
    AppendWeldmentCellText(definition, number, 0.325, y, 0.009);
  }
  AppendWeldmentCellText(definition, "Top", 0.3305, 0.100, 0.049);
  AppendWeldmentCellText(definition, "Description", 0.3305, 0.0835, 0.049);
  AppendWeldmentCellText(definition, "Hidden", 0.3305, 0.099994, 0.049, 0.0025,
                         1.0e-6);
  append_class("moDisplayWeldmentTable_c");
  definition.resize(definition.size() + 32u, 0);
  AppendF64(definition, 0.400);
  AppendF64(definition, 0.080);
  const std::vector<std::string> sheets = {"List1"};

  auto result = DrawingGeometryReader::ReadLegacyWeldmentTableLines(
      definition, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 9u);
  EXPECT_DOUBLE_EQ(result.value().front().start.x, 0.32);
  EXPECT_DOUBLE_EQ(result.value().front().start.y, 0.08);
  EXPECT_DOUBLE_EQ(result.value().front().end.y, 0.1025);
  EXPECT_DOUBLE_EQ(result.value().back().start.y, 0.0875);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/Definition/moDisplayWeldmentTable_c/grid");

  auto text_result = DrawingGeometryReader::ReadLegacyWeldmentTableText(
      definition, sheets, "List1");
  ASSERT_TRUE(text_result.ok()) << text_result.error();
  ASSERT_EQ(text_result.value().size(), 5u);
  EXPECT_EQ(text_result.value()[3].text, "Top");
  EXPECT_EQ(text_result.value()[3].anchor, TextAnchor::kStart);
  EXPECT_EQ(text_result.value()[4].text, "Description");
  EXPECT_TRUE(std::none_of(text_result.value().begin(),
                           text_result.value().end(),
                           [](const TextAnnotation &annotation) {
                             return annotation.text == "Hidden";
                           }));
}

TEST(DrawingViewGeometryTest, ReadsCachedWeldmentTableWithDisplayBeforeGrid) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  const auto append_class = [&](std::string_view value) {
    definition.insert(definition.end(), value.begin(), value.end());
  };
  append_class("moDisplayWeldmentTable_c");
  definition.resize(definition.size() + 400u, 0);
  AppendF64(definition, 0.400);
  AppendF64(definition, 0.080);
  append_class("moWeldmentTableColumn_c");
  AppendF64(definition, 0.01);
  for (double width : {0.05, 0.02}) {
    definition.insert(definition.end(), {0x7E, 0x88});
    AppendF64(definition, width);
  }
  append_class("moWeldmentTableRow_c");
  AppendF64(definition, 0.005);
  append_class("moWeldmentTableGrid_c");
  definition.resize(definition.size() + 700u, 0);
  for (const auto [number, y] :
       std::array<std::pair<std::string_view, double>, 3>{
           {{"3", 0.100}, {"2", 0.095}, {"1", 0.090}}}) {
    AppendWeldmentCellText(definition, number, 0.325, y, 0.009);
  }
  AppendWeldmentCellText(definition, "Description", 0.3305, 0.0835, 0.049);
  const std::vector<std::string> sheets = {"List1"};

  auto lines = DrawingGeometryReader::ReadLegacyWeldmentTableLines(
      definition, sheets, "List1");

  ASSERT_TRUE(lines.ok()) << lines.error();
  ASSERT_EQ(lines.value().size(), 9u);
  EXPECT_DOUBLE_EQ(lines.value().front().start.x, 0.32);
  EXPECT_DOUBLE_EQ(lines.value().front().start.y, 0.08);

  auto text = DrawingGeometryReader::ReadLegacyWeldmentTableText(
      definition, sheets, "List1");
  ASSERT_TRUE(text.ok()) << text.error();
  ASSERT_EQ(text.value().size(), 4u);
  EXPECT_EQ(text.value().back().text, "Description");
}

TEST(DrawingViewGeometryTest, ReadsIndexedSheetFormatSegment) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  AppendSheetSketchPoint(definition, 0.005, 0.005);
  AppendSheetSketchPoint(definition, 0.415, 0.005, true, 5u);
  AppendSheetSketchPoint(definition, 0.415, 0.292, false, 4u, 0x0012u);
  AppendSheetSketchLine(definition, 0u, 1u);
  AppendSheetSketchLine(definition, 1u, 2u);
  AppendSheetSketchLine(definition, 0u, 2u, 5u);

  const std::vector<std::string> sheets = {"List1"};

  auto result = DrawingGeometryReader::ReadSheetFormatLines(
      definition, sheets, "List1", 0.420, 0.297);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 2u);
  EXPECT_DOUBLE_EQ(result.value().front().start.x, 0.005);
  EXPECT_DOUBLE_EQ(result.value().front().start.y, 0.005);
  EXPECT_DOUBLE_EQ(result.value().front().end.x, 0.415);
  EXPECT_DOUBLE_EQ(result.value().front().end.y, 0.005);
  EXPECT_EQ(result.value().front().source_table,
            "Contents/Definition/sgSketch/sheet-format/List1");
}

TEST(DrawingViewGeometryTest, SuppressesStateFiveHelpersWithSketchBitmap) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  AppendSheetSketchPoint(definition, 0.005, 0.005);
  AppendSheetSketchPoint(definition, 0.415, 0.005, true, 5u);
  AppendSheetSketchPoint(definition, 0.415, 0.292, false, 4u, 0x0012u);
  AppendSheetSketchPoint(definition, 0.010, 0.010);
  AppendSheetSketchLine(definition, 0u, 1u);
  AppendSheetSketchLine(definition, 1u, 2u);
  AppendSheetSketchLine(definition, 0u, 3u, 5u);
  const std::vector<std::string> sheets = {"List1"};

  auto fallback = DrawingGeometryReader::ReadSheetFormatLines(
      definition, sheets, "List1", 0.420, 0.297);
  auto with_bitmap = DrawingGeometryReader::ReadSheetFormatLines(
      definition, sheets, "List1", 0.420, 0.297, true);

  ASSERT_TRUE(fallback.ok()) << fallback.error();
  ASSERT_TRUE(with_bitmap.ok()) << with_bitmap.error();
  EXPECT_EQ(fallback.value().size(), 3u);
  EXPECT_EQ(with_bitmap.value().size(), 2u);
}

TEST(DrawingViewGeometryTest, ReadsStaticSheetFormatNote) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  AppendSheetFormatNote(definition, "TITLE:", 0.2475, 0.0168, "Century Gothic",
                        0.0015);
  AppendSheetFormatNote(definition, "$PRPSHEET:\"Title\"", 0.300, 0.0168,
                        "Century Gothic", 0.0025);
  const std::vector<std::string> sheets = {"List1"};

  auto result =
      DrawingGeometryReader::ReadSheetFormatText(definition, sheets, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front().text, "TITLE:");
  EXPECT_DOUBLE_EQ(result.value().front().origin.x, 0.2475);
  EXPECT_DOUBLE_EQ(result.value().front().origin.y, 0.0168);
  EXPECT_DOUBLE_EQ(result.value().front().height, 0.0015);
  EXPECT_EQ(result.value().front().font_family, "Century Gothic");
}

TEST(DrawingViewGeometryTest, SelectsSheetAndAcceptsSlddrwSerializerTag) {
  std::vector<uint8_t> definition;
  AppendUtf16(definition, "List1");
  AppendSheetFormatNote(definition, "FIRST", 0.25, 0.02, "Century Gothic",
                        0.0015, 0x8AFDu);
  AppendUtf16(definition, "List1(2)");
  AppendSheetFormatNote(definition, "SECOND", 0.26, 0.03, "Century Gothic",
                        0.0015, 0x85DBu);
  const std::vector<std::string> sheets = {"List1", "List1(2)"};

  auto result = DrawingGeometryReader::ReadSheetFormatText(definition, sheets,
                                                           "List1(2)");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  EXPECT_EQ(result.value().front().text, "SECOND");
  EXPECT_EQ(result.value().front().source_table,
            "Contents/Definition/moDisplayNote_c/List1(2)");
}

TEST(DrawingViewGeometryTest, ReadsCachedFaceTriangleStrip) {
  std::vector<uint8_t> data(7u, 0x55u);
  const std::size_t class_offset = data.size();
  const std::string_view class_name = "uoTempFaceTessData_c";
  data.insert(data.end(), class_name.begin(), class_name.end());
  AppendU32(data, 1u); // triangle count
  AppendU32(data, 1u); // strip count

  AppendTessellationDescriptor(data, 4u, 8u, 1u);
  AppendU32(data, 3u);
  AppendTessellationDescriptor(data, 12u, 100u, 3u);
  for (const auto &vertex : std::array<Vec3, 3>{
           Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}}) {
    AppendF32(data, static_cast<float>(vertex.x));
    AppendF32(data, static_cast<float>(vertex.y));
    AppendF32(data, static_cast<float>(vertex.z));
  }
  AppendTessellationDescriptor(data, 12u, 100u, 3u);
  for (std::size_t index = 0; index < 3u; ++index) {
    AppendF32(data, 0.0F);
    AppendF32(data, 0.0F);
    AppendF32(data, 1.0F);
  }
  AppendTessellationDescriptor(data, 4u, 8u, 4u);
  for (std::size_t index = 0; index < 4u; ++index)
    AppendF32(data, 0.0F);
  AppendTessellationDescriptor(data, 4u, 8u, 1u);
  AppendU32(data, 4u);
  AppendTessellationDescriptor(data, 1u, 8u, 4u);
  data.insert(data.end(), {0u, 0u, 0u, 0u});

  auto result = DrawingGeometryReader::ReadFaceTessellations(data);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().size(), 1u);
  const FaceTessellation &mesh = result.value().front();
  EXPECT_EQ(mesh.source_offset, class_offset);
  ASSERT_EQ(mesh.vertices.size(), 3u);
  ASSERT_EQ(mesh.normals.size(), 3u);
  ASSERT_EQ(mesh.strip_lengths, std::vector<uint32_t>({3u}));
  ASSERT_EQ(mesh.triangles.size(), 1u);
  EXPECT_EQ(mesh.triangles.front(), (std::array<uint32_t, 3>{0u, 1u, 2u}));
  EXPECT_DOUBLE_EQ(mesh.vertices[1].x, 1.0);
}

TEST(DrawingViewGeometryTest, ReadsCachedVisualPropertiesColorAndName) {
  std::vector<uint8_t> data(5u, 0x55u);
  const std::size_t class_offset = data.size();
  const std::string_view class_name = "moVisualProperties_c";
  data.insert(data.end(), class_name.begin(), class_name.end());
  AppendU32(data, 0x00785634u);
  data.resize(data.size() + 12u, 0u);
  AppendUtf16(data, "polishedcarbonsteel");

  const auto appearances = DrawingGeometryReader::ReadAppearances(data);

  ASSERT_EQ(appearances.size(), 1u);
  EXPECT_EQ(appearances.front().name, "polishedcarbonsteel");
  EXPECT_EQ(appearances.front().rgb, 0x00345678u);
  EXPECT_DOUBLE_EQ(appearances.front().opacity, 1.0);
  EXPECT_EQ(appearances.front().source_offset, class_offset);
  EXPECT_EQ(appearances.front().scope,
            DrawingAppearanceScope::kClassDefinition);
}

TEST(DrawingViewGeometryTest, ReadsInlineVisualPropertiesRecord) {
  std::vector<uint8_t> data(9u, 0x55u);
  const std::size_t record_offset = data.size();
  data.insert(data.end(), {0x33u, 0x80u});
  AppendU32(data, 0x00030201u);
  data.resize(data.size() + 8u, 0u);
  AppendUtf16(data, "Face override");

  const auto appearances = DrawingGeometryReader::ReadAppearances(data);

  ASSERT_EQ(appearances.size(), 1u);
  EXPECT_EQ(appearances.front().name, "Face override");
  EXPECT_EQ(appearances.front().rgb, 0x00010203u);
  EXPECT_EQ(appearances.front().source_offset, record_offset);
  EXPECT_EQ(appearances.front().scope,
            DrawingAppearanceScope::kInlineUnresolved);
}

TEST(DrawingViewGeometryTest, ReadsAmbientAndInlineDirectionalLights) {
  std::vector<uint8_t> data;
  const auto append_light =
      [&](std::string_view class_name, std::array<double, 3> coefficients,
          uint32_t rgb, std::string_view name, Vec3 direction) {
        if (!class_name.empty()) {
          data.insert(data.end(), class_name.begin(), class_name.end());
        } else {
          data.insert(data.end(), {0x07u, 0x80u});
        }
        const std::size_t payload = data.size();
        for (const double value : coefficients)
          AppendF64(data, value);
        AppendU32(data, rgb);
        data.resize(payload + 40u, 0u);
        AppendU32(data, 1u);
        AppendUtf16(data, name);
        data.resize(payload + 128u, 0u);
        AppendF64(data, direction.x);
        AppendF64(data, direction.y);
        AppendF64(data, direction.z);
      };
  append_light("moAmbientLight_c", {0.29, 1.0, 0.0}, 0x00FFFFFFu, "Ambient",
               {});
  data.insert(data.end(), {0xFFu, 0xFFu, 0x01u, 0x00u});
  append_light("moDirectionLight_c", {0.2, 0.45, 0.69}, 0x00FFFFFFu,
               "Direction1", {-2.5, 2.5, 7.5});
  append_light({}, {0.13, 0.88, 0.85}, 0x00C08080u, "Direction2",
               {17.0, -2.7, -1.8});
  data.insert(data.end(), {0xFFu, 0xFFu, 0x01u, 0x00u});

  const auto lights = DrawingGeometryReader::ReadSceneLights(data);

  ASSERT_EQ(lights.size(), 3u);
  EXPECT_EQ(lights[0].kind, DrawingLightKind::kAmbient);
  EXPECT_EQ(lights[0].name, "Ambient");
  EXPECT_DOUBLE_EQ(lights[0].ambient, 0.29);
  EXPECT_DOUBLE_EQ(lights[1].diffuse, 0.45);
  EXPECT_DOUBLE_EQ(lights[2].specular, 0.85);
  EXPECT_EQ(lights[1].kind, DrawingLightKind::kDirectional);
  EXPECT_EQ(lights[1].name, "Direction1");
  EXPECT_DOUBLE_EQ(lights[1].direction.z, 7.5);
  EXPECT_EQ(lights[2].name, "Direction2");
  EXPECT_EQ(lights[2].rgb, 0x008080C0u);
  EXPECT_DOUBLE_EQ(lights[2].direction.x, 17.0);
}

TEST(DrawingViewGeometryTest, ReadsSlddrw19000VisualPropertiesPrefix) {
  std::vector<uint8_t> data;
  const std::string_view class_name = "moVisualProperties_c";
  data.insert(data.end(), class_name.begin(), class_name.end());
  AppendU32(data, 0x00C0C0C0u);
  data.resize(data.size() + 8u, 0u);
  AppendUtf16(data, "polishedcarbonsteel");

  const auto appearances = DrawingGeometryReader::ReadAppearances(data);

  ASSERT_EQ(appearances.size(), 1u);
  EXPECT_EQ(appearances.front().name, "polishedcarbonsteel");
  EXPECT_EQ(appearances.front().rgb, 0x00C0C0C0u);
}

TEST(DrawingViewGeometryTest, RejectsIncidentalVisualPropertiesClassName) {
  const std::string value = "moVisualProperties_c without a record";
  const std::vector<uint8_t> data(value.begin(), value.end());

  EXPECT_TRUE(DrawingGeometryReader::ReadAppearances(data).empty());
}

TEST(DrawingViewGeometryTest, ClassifiesLineBehindCachedFace) {
  FaceTessellation face;
  face.vertices = {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}};
  face.triangles = {{0u, 1u, 2u}};
  const std::array<double, 9> identity = {1.0, 0.0, 0.0, 0.0, 1.0,
                                          0.0, 0.0, 0.0, 1.0};
  const std::array<FaceTessellation, 1> faces = {face};

  EXPECT_TRUE(DrawingGeometryReader::IsLineHiddenByTessellation(
      Line3D{{0.2, 0.2, 0.0}, {0.6, 0.2, 0.0}}, faces, identity));
  EXPECT_FALSE(DrawingGeometryReader::IsLineHiddenByTessellation(
      Line3D{{0.2, 0.2, 2.0}, {0.6, 0.2, 2.0}}, faces, identity));
  EXPECT_FALSE(DrawingGeometryReader::IsLineHiddenByTessellation(
      Line3D{{1.2, 1.2, 0.0}, {1.6, 1.2, 0.0}}, faces, identity));
}

TEST(DrawingViewGeometryTest, ClipsProjectedTriangleToViewRectangle) {
  const std::array<Vec2, 3> triangle = {Vec2{-1.0, 0.0}, Vec2{2.0, 0.0},
                                        Vec2{0.5, 2.0}};
  const auto clipped = DrawingGeometryReader::ClipTriangleToRect(
      triangle, Rect2{{0.0, 0.0}, {1.0, 1.0}});

  ASSERT_EQ(clipped.size(), 2u);
  double area = 0.0;
  for (const auto &part : clipped) {
    for (const Vec2 point : part) {
      EXPECT_GE(point.x, 0.0);
      EXPECT_LE(point.x, 1.0);
      EXPECT_GE(point.y, 0.0);
      EXPECT_LE(point.y, 1.0);
    }
    area += std::abs((part[1].x - part[0].x) * (part[2].y - part[0].y) -
                     (part[1].y - part[0].y) * (part[2].x - part[0].x)) /
            2.0;
  }
  EXPECT_NEAR(area, 1.0, 1e-12);
}

TEST(DrawingViewGeometryTest, SplitsPartiallyOccludedCachedLine) {
  FaceTessellation face;
  face.vertices = {
      {0.5, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.5, 1.0, 1.0}};
  face.triangles = {{0u, 1u, 2u}, {0u, 2u, 3u}};
  const std::array<double, 9> identity = {1.0, 0.0, 0.0, 0.0, 1.0,
                                          0.0, 0.0, 0.0, 1.0};
  const std::array<FaceTessellation, 1> faces = {face};

  const auto spans = DrawingGeometryReader::SplitLineByTessellation(
      Line3D{{0.0, 0.5, 0.0}, {1.0, 0.5, 0.0}}, faces, identity);

  ASSERT_EQ(spans.size(), 2u);
  EXPECT_FALSE(spans[0].hidden);
  EXPECT_NEAR(spans[0].start_fraction, 0.0, 1e-8);
  EXPECT_NEAR(spans[0].end_fraction, 0.5, 1e-8);
  EXPECT_TRUE(spans[1].hidden);
  EXPECT_NEAR(spans[1].start_fraction, 0.5, 1e-8);
  EXPECT_NEAR(spans[1].end_fraction, 1.0, 1e-8);
}

TEST(DrawingViewGeometryTest, IgnoresOccludingFaceOwnedByAnotherViewModel) {
  const std::string keywords = "<Keywords><Sheet Name=\"List1\" Type=\"Sheet\">"
                               "<View Name=\"View1\"/></Sheet></Keywords>";
  const auto bytes = [](const std::string &value) {
    return std::span<const uint8_t>(
        reinterpret_cast<const uint8_t *>(value.data()), value.size());
  };

  constexpr std::size_t placement_offset = 400u;
  constexpr std::size_t matrix_offset = placement_offset - 317u;
  std::vector<uint8_t> definition(placement_offset + 120u, 0u);
  const std::array<uint8_t, 5> name_marker = {0x09, 0x89, 0xFF, 0xFE, 0xFF};
  std::copy(name_marker.begin(), name_marker.end(), definition.begin());
  definition[name_marker.size()] = 5u;
  for (std::size_t index = 0; index < 5u; ++index) {
    definition[name_marker.size() + 1u + index * 2u] =
        static_cast<uint8_t>("View1"[index]);
  }
  const std::array<double, 9> identity = {1.0, 0.0, 0.0, 0.0, 1.0,
                                          0.0, 0.0, 0.0, 1.0};
  for (std::size_t index = 0; index < identity.size(); ++index) {
    SetF64(definition, matrix_offset + index * 8u, identity[index]);
  }
  SetF64(definition, matrix_offset + 72u, 0.05);
  SetF64(definition, matrix_offset + 80u, 0.05);
  SetF64(definition, matrix_offset + 88u, 0.0);
  SetF64(definition, matrix_offset + 96u, 0.1);
  const std::array<double, 11> placement = {0.04, 0.04, 0.20, 0.20, 0.05, 0.05,
                                            0.19, 0.19, 0.12, 0.12, 0.0};
  for (std::size_t index = 0; index < placement.size(); ++index) {
    SetF64(definition, placement_offset + index * 8u, placement[index]);
  }

  const std::string_view bucket_class = "uiViewBucket_c";
  std::vector<uint8_t> vb_lists = {
      0x01, 0x00, 0x00,
      0x00, 0xFF, 0xFF,
      0x01, 0x00, static_cast<uint8_t>(bucket_class.size()),
      0x00};
  vb_lists.insert(vb_lists.end(), bucket_class.begin(), bucket_class.end());
  AppendF64(vb_lists, 0.0);
  AppendF64(vb_lists, 0.0);
  AppendF64(vb_lists, 1.0);
  AppendF64(vb_lists, 1.0);
  AppendF64(vb_lists, 1.0);
  vb_lists.insert(vb_lists.end(), {0u, 0u});
  vb_lists.insert(vb_lists.end(),
                  {0x21, 0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00});
  for (const double value : {0.2, 0.2, 0.0, 0.6, 0.2, 0.0}) {
    AppendF64(vb_lists, value);
  }
  AppendUtf16(vb_lists, "part@View1");

  AppendUtf16(vb_lists, "C:\\models\\part.SLDPRT");
  const std::string_view face_class = "uoTempFaceTessData_c";
  vb_lists.insert(
      vb_lists.end(),
      {0xFF, 0xFF, 0x01, 0x00, static_cast<uint8_t>(face_class.size()), 0x00});
  vb_lists.insert(vb_lists.end(), face_class.begin(), face_class.end());
  AppendU32(vb_lists, 1u);
  AppendU32(vb_lists, 1u);
  const auto append_face = [&](float z) {
    AppendTessellationDescriptor(vb_lists, 4u, 8u, 1u);
    AppendU32(vb_lists, 3u);
    AppendTessellationDescriptor(vb_lists, 12u, 100u, 3u);
    for (const Vec3 vertex :
         {Vec3{0.0, 0.0, z}, Vec3{1.0, 0.0, z}, Vec3{0.0, 1.0, z}}) {
      AppendF32(vb_lists, static_cast<float>(vertex.x));
      AppendF32(vb_lists, static_cast<float>(vertex.y));
      AppendF32(vb_lists, static_cast<float>(vertex.z));
    }
    AppendTessellationDescriptor(vb_lists, 12u, 100u, 3u);
    for (std::size_t index = 0; index < 3u; ++index) {
      AppendF32(vb_lists, 0.0F);
      AppendF32(vb_lists, 0.0F);
      AppendF32(vb_lists, 1.0F);
    }
    AppendTessellationDescriptor(vb_lists, 4u, 8u, 0u);
    AppendTessellationDescriptor(vb_lists, 4u, 8u, 1u);
    AppendU32(vb_lists, 4u);
    AppendTessellationDescriptor(vb_lists, 1u, 8u, 0u);
  };
  append_face(-1.0F);
  AppendUtf16(vb_lists, "C:\\models\\other.SLDPRT");
  append_face(1.0F);

  auto parsed_faces = DrawingGeometryReader::ReadFaceTessellations(vb_lists);
  ASSERT_TRUE(parsed_faces.ok()) << parsed_faces.error();
  ASSERT_EQ(parsed_faces.value().size(), 2u);
  EXPECT_EQ(parsed_faces.value()[0].referenced_document, "part");
  EXPECT_EQ(parsed_faces.value()[1].referenced_document, "other");

  auto result = DrawingGeometryReader::ReadCachedSheet(
      bytes(keywords), definition, vb_lists, {}, "List1");

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().entities.size(), 1u);
  const auto *line = std::get_if<Line2D>(&result.value().entities.front());
  ASSERT_NE(line, nullptr);
  EXPECT_EQ(line->style.pattern, LinePattern::kSolid);
}

TEST(DrawingViewGeometryTest, RejectsMalformedCachedFaceDescriptorTable) {
  std::vector<uint8_t> data;
  const std::string_view class_name = "uoTempFaceTessData_c";
  data.insert(data.end(), class_name.begin(), class_name.end());
  AppendU32(data, 1u);
  AppendU32(data, 1u);
  AppendTessellationDescriptor(data, 8u, 8u, 1u); // wrong item size
  AppendU64(data, 3u);

  auto result = DrawingGeometryReader::ReadFaceTessellations(data);

  EXPECT_FALSE(result.ok());
}

} // namespace
} // namespace openswx
