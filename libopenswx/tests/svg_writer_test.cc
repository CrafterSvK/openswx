// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <string>

#include "openswx/svg_writer.h"

namespace openswx {
namespace {

TEST(SvgWriterTest, WritesAutoCroppedUprightSvg) {
  DrawingSheetGeometry sheet{"List & 1", {}};
  sheet.entities.emplace_back(
      Line2D{{0.01, 0.02}, {0.03, 0.04}, {}, "Revision & table"});
  sheet.entities.emplace_back(TextAnnotation{
      "A & B", {0.01, 0.02}, 0.5, 0.002, 0.006, "Arial", "Notes"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  const std::string& svg = result.value();
  EXPECT_NE(svg.find("<svg xmlns="), std::string::npos);
  EXPECT_NE(svg.find("<title>List &amp; 1</title>"), std::string::npos);
  EXPECT_NE(svg.find("<line x1=\"10\" y1=\"-20\""), std::string::npos);
  EXPECT_NE(svg.find(">A &amp; B</text>"), std::string::npos);
  EXPECT_NE(svg.find("rotate(-28.647"), std::string::npos);
  EXPECT_NE(svg.find("Revision &amp; table"), std::string::npos);
}

TEST(SvgWriterTest, RejectsEmptySheet) {
  auto result = SvgWriter::WriteSheet({"Empty", {}});
  EXPECT_FALSE(result.ok());
}

TEST(SvgWriterTest, WritesExplicitTextAnchor) {
  DrawingSheetGeometry sheet{"Anchors", {}};
  sheet.entities.emplace_back(TextAnnotation{
      "Middle", {0.01, 0.02}, 0.0, 0.002, 0.006, "Arial", "Table",
      TextAnchor::kMiddle});
  sheet.entities.emplace_back(TextAnnotation{
      "End", {0.02, 0.02}, 0.0, 0.002, 0.006, "Arial", "Table",
      TextAnchor::kEnd});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find("text-anchor=\"middle\""),
            std::string::npos);
  EXPECT_NE(result.value().find("text-anchor=\"end\""),
            std::string::npos);
}

TEST(SvgWriterTest, UsesPhysicalSheetSizeWhenAvailable) {
  DrawingSheetGeometry sheet{"List1", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  sheet.entities.emplace_back(Line2D{{0.0, 0.0}, {0.1, 0.1}, {}, "view"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find("width=\"420mm\" height=\"297mm\""),
            std::string::npos);
  EXPECT_NE(result.value().find("viewBox=\"0 -297 420 297\""),
            std::string::npos);
}

TEST(SvgWriterTest, WritesCenterLineDashPattern) {
  DrawingSheetGeometry sheet{"List2", {}};
  LineStyle style;
  style.pattern = LinePattern::kCenter;
  sheet.entities.emplace_back(
      Line2D{{0.01, 0.02}, {0.10, 0.02}, style, "DisplayLists"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find("stroke-dasharray=\"4 1 0.7 1\""),
            std::string::npos);
}

TEST(SvgWriterTest, WritesCircularDisplayListArc) {
  DrawingSheetGeometry sheet{"Arc", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  sheet.entities.emplace_back(Arc2D{
      {0.08, 0.12}, {0.07, 0.16}, {0.06, 0.14}, 1.0, LineStyle{},
      "Contents/DisplayLists/arc/objID=42"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find("<path d=\"M 80 -120 A"),
            std::string::npos);
  EXPECT_NE(result.value().find("0 0 70 -160\" fill=\"none\""),
            std::string::npos);
  EXPECT_NE(result.value().find("DisplayLists/arc/objID=42"),
            std::string::npos);
}

TEST(SvgWriterTest, WritesFilledTriangleWithoutTessellationEdges) {
  DrawingSheetGeometry sheet{"Shaded", {}};
  sheet.entities.emplace_back(FilledTriangle2D{
      {{{0.01, 0.02}, {0.03, 0.02}, {0.02, 0.04}}}, 0xC0D0E0u, 0.75,
      "Contents/VBLists/uoTempFaceTessData_c"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find(
                "<polygon points=\"10,-20 30,-20 20,-40 \""),
            std::string::npos);
  EXPECT_NE(result.value().find("fill=\"#c0d0e0\" stroke=\"none\""),
            std::string::npos);
  EXPECT_NE(result.value().find("fill-opacity=\"0.75\""),
            std::string::npos);
}

TEST(SvgWriterTest, EmbedsRgbaSketchBitmapAsPng) {
  DrawingSheetGeometry sheet{"Bitmap", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  sheet.entities.emplace_back(RasterImage2D{
      {0.38, 0.03}, 0.02, 0.01, 1u, 1u, {0, 0, 0, 255},
      "Contents/SketchBitmap0"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find("<image x=\"380\" y=\"-40\" width=\"20\"") ,
             std::string::npos);
  EXPECT_NE(result.value().find("href=\"data:image/png;base64,iVBORw0KGgo"),
            std::string::npos);
  EXPECT_NE(result.value().find("Contents/SketchBitmap0"),
            std::string::npos);
}

TEST(SvgWriterTest, WritesLinearDimensionWithRotatedText) {
  DrawingSheetGeometry sheet{"List1", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  sheet.entities.emplace_back(Dimension{
      "1462", {0.03, 0.12}, 1.462, 0.1462,
      DimensionOrientation::kVertical, 123u,
      "Contents/Definition/moDimBackupDef_c/RD1"});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find(">1462</text>"), std::string::npos);
  EXPECT_NE(result.value().find("rotate(-90 30 -120)"), std::string::npos);
  EXPECT_NE(result.value().find(
                "font-family=\"osifont, 'DejaVu Sans', Arial, sans-serif\""),
            std::string::npos);
  EXPECT_NE(result.value().find("<path d=\"M 30 -193.1"),
            std::string::npos);
  EXPECT_NE(result.value().find("moDimBackupDef_c/RD1"),
            std::string::npos);
}

TEST(SvgWriterTest, WritesBomBalloonTextLeaderAndArrow) {
  DrawingSheetGeometry sheet{"List1", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  sheet.entities.emplace_back(BomBalloon{
      "3", {0.349, 0.248}, {0.334, 0.223}, 456u,
      "Contents/Definition/moDisplayBalloonNote_c", false, {}});

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_NE(result.value().find(">3</text>"), std::string::npos);
  EXPECT_NE(result.value().find("<line x1=\"334\" y1=\"-223\""),
            std::string::npos);
  EXPECT_NE(result.value().find("<path d=\"M 334 -223"),
            std::string::npos);
  EXPECT_NE(result.value().find(
                "font-family=\"'Century Gothic', 'DejaVu Sans', Arial, sans-serif\""),
            std::string::npos);
  EXPECT_NE(result.value().find("moDisplayBalloonNote_c"),
            std::string::npos);
}

TEST(SvgWriterTest, DoesNotDuplicateCachedBomBalloonLeader) {
  DrawingSheetGeometry sheet{"List1", {}};
  sheet.width_m = 0.420;
  sheet.height_m = 0.297;
  BomBalloon balloon{"3", {0.349, 0.248}, {0.334, 0.223}, 456u,
                     "Contents/Definition/moDisplayBalloonNote_c", false, {}};
  balloon.has_cached_leader = true;
  balloon.cached_leader_point = {0.342, 0.230};
  sheet.entities.emplace_back(balloon);

  auto result = SvgWriter::WriteSheet(sheet);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().find("<line x1=\"334\" y1=\"-223\""),
            std::string::npos);
  EXPECT_NE(result.value().find("<path d=\"M 334 -223"),
            std::string::npos);
  EXPECT_NE(result.value().find(">3</text>"), std::string::npos);
}

}  // namespace
}  // namespace openswx
