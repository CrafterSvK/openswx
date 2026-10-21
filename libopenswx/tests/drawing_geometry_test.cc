// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <span>
#include <string>
#include <variant>

#include "openswx/drawing_geometry.h"

namespace openswx {
namespace {

TEST(DrawingGeometryTest, ReadsTableLinesAndDeduplicatesMergedCellText) {
  const std::string xml =
      "junk<Tables><Table Display_Name=\"Revision table\" "
      "SheetName=\"Sheet1\"><Charformat TypeFace=\"Arial\" Size=\"0.002\"/>"
      "<Row><Cell><Text_Element Text=\"A &amp; B\" Angle=\"0.5\" "
      "CellWidthArray=\"0.001 0.002 0.003\"><Point X=\"0.1\" "
      "Y=\"0.2\"/></Text_Element><Text_Element Text=\"A &amp; B\" "
      "Angle=\"0.5\" CellWidthArray=\"0.001 0.002 0.003\"><Point "
      "X=\"0.1\" Y=\"0.2\"/></Text_Element></Cell></Row><GridLines>"
      "<Pen ColorRef=\"255\" LineStyle=\"2\" LineWeight=\"3\"/>"
      "<Line><StartPt X=\"0.01\" Y=\"0.02\"/><EndPt X=\"0.03\" "
      "Y=\"0.04\"/></Line></GridLines></Table></Tables>";
  const auto bytes = std::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(xml.data()), xml.size());

  auto result = DrawingGeometryReader::ReadTablesXml(bytes);

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().sheets.size(), 1u);
  const DrawingSheetGeometry& sheet = result.value().sheets[0];
  EXPECT_EQ(sheet.name, "Sheet1");
  ASSERT_EQ(sheet.entities.size(), 2u);

  const auto* text = std::get_if<TextAnnotation>(&sheet.entities[0]);
  ASSERT_NE(text, nullptr);
  EXPECT_EQ(text->text, "A & B");
  EXPECT_DOUBLE_EQ(text->origin.x, 0.1);
  EXPECT_DOUBLE_EQ(text->origin.y, 0.2);
  EXPECT_DOUBLE_EQ(text->rotation_radians, 0.5);
  EXPECT_DOUBLE_EQ(text->height, 0.002);
  EXPECT_NEAR(text->width, 0.006, 1e-12);
  EXPECT_EQ(text->font_family, "Arial");
  EXPECT_FALSE(text->height_is_cap_height);

  const auto* line = std::get_if<Line2D>(&sheet.entities[1]);
  ASSERT_NE(line, nullptr);
  EXPECT_DOUBLE_EQ(line->start.x, 0.01);
  EXPECT_DOUBLE_EQ(line->end.y, 0.04);
  EXPECT_EQ(line->style.rgb, 0xFF0000u);
  EXPECT_EQ(line->style.source_line_style, 2);
  EXPECT_EQ(line->style.source_line_weight, 3);
}

TEST(DrawingGeometryTest, RejectsMalformedXml) {
  const std::string xml = "<Tables><Table>";
  const auto bytes = std::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(xml.data()), xml.size());
  auto result = DrawingGeometryReader::ReadTablesXml(bytes);
  EXPECT_FALSE(result.ok());
}

}  // namespace
}  // namespace openswx
