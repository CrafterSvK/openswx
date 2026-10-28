// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <tuple>

#include <pugixml.hpp>

namespace openswx {
namespace {

std::optional<double> ReadDouble(const pugi::xml_attribute& attribute) {
  if (!attribute) return std::nullopt;
  char* end = nullptr;
  const double value = std::strtod(attribute.value(), &end);
  if (end == attribute.value() || *end != '\0' || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

int ReadInt(const pugi::xml_attribute& attribute, int fallback = 0) {
  if (!attribute) return fallback;
  char* end = nullptr;
  const long value = std::strtol(attribute.value(), &end, 10);
  if (end == attribute.value() || *end != '\0') return fallback;
  return static_cast<int>(value);
}

uint32_t ColorRefToRgb(const pugi::xml_attribute& attribute) {
  const int color_ref = ReadInt(attribute, -1);
  if (color_ref < 0) return 0;
  const auto value = static_cast<uint32_t>(color_ref);
  const uint32_t red = value & 0xFFu;
  const uint32_t green = (value >> 8u) & 0xFFu;
  const uint32_t blue = (value >> 16u) & 0xFFu;
  return (red << 16u) | (green << 8u) | blue;
}

double SumWidths(std::string_view widths) {
  std::istringstream input{std::string(widths)};
  double total = 0.0;
  double value = 0.0;
  while (input >> value) {
    if (std::isfinite(value) && value > 0.0) total += value;
  }
  return total;
}

void VisitDescendants(const pugi::xml_node& node,
                      const std::function<void(pugi::xml_node)>& visitor) {
  for (const pugi::xml_node child : node.children()) {
    visitor(child);
    VisitDescendants(child, visitor);
  }
}

DrawingSheetGeometry& FindSheet(DrawingGeometry& drawing,
                                std::string_view name) {
  for (auto& sheet : drawing.sheets) {
    if (sheet.name == name) return sheet;
  }
  DrawingSheetGeometry sheet;
  sheet.name = std::string(name);
  drawing.sheets.push_back(std::move(sheet));
  return drawing.sheets.back();
}

pugi::xml_node FindCharFormat(pugi::xml_node node,
                              const pugi::xml_node& table) {
  for (pugi::xml_node parent = node.parent(); parent && parent != table;
       parent = parent.parent()) {
    if (const pugi::xml_node format = parent.child("Charformat")) return format;
  }
  return table.child("Charformat");
}

}  // namespace

Result<DrawingGeometry> DrawingGeometryReader::ReadTablesXml(
    std::span<const uint8_t> xml) {
  const auto first_xml = std::find(xml.begin(), xml.end(), uint8_t{'<'});
  if (first_xml == xml.end()) {
    return Result<DrawingGeometry>::Err("Tables stream does not contain XML");
  }

  pugi::xml_document document;
  const pugi::xml_parse_result parsed = document.load_buffer(
      &*first_xml, static_cast<std::size_t>(xml.end() - first_xml),
      pugi::parse_default, pugi::encoding_utf8);
  if (!parsed) {
    return Result<DrawingGeometry>::Err(
        std::string("Cannot parse Tables XML: ") + parsed.description());
  }

  const pugi::xml_node root = document.child("Tables");
  if (!root) {
    return Result<DrawingGeometry>::Err("Tables XML has no Tables root");
  }

  DrawingGeometry drawing;
  std::set<std::tuple<std::string, double, double, double, std::string,
                      std::string>>
      seen_text;

  for (const pugi::xml_node table : root.children("Table")) {
    const std::string table_name = table.attribute("Display_Name").as_string();
    const std::string sheet_name = table.attribute("SheetName").as_string(
        "(unspecified)");
    DrawingSheetGeometry& sheet = FindSheet(drawing, sheet_name);

    LineStyle table_style;
    if (const pugi::xml_node pen = table.child("GridLines").child("Pen")) {
      table_style.rgb = ColorRefToRgb(pen.attribute("ColorRef"));
      table_style.source_line_style = ReadInt(pen.attribute("LineStyle"));
      table_style.source_line_weight = ReadInt(pen.attribute("LineWeight"));
    }

    VisitDescendants(table, [&](const pugi::xml_node node) {
      const std::string_view name = node.name();
      if (name == "Line") {
        const pugi::xml_node start = node.child("StartPt");
        const pugi::xml_node end = node.child("EndPt");
        const auto x1 = ReadDouble(start.attribute("X"));
        const auto y1 = ReadDouble(start.attribute("Y"));
        const auto x2 = ReadDouble(end.attribute("X"));
        const auto y2 = ReadDouble(end.attribute("Y"));
        if (x1 && y1 && x2 && y2) {
          sheet.entities.emplace_back(Line2D{{*x1, *y1}, {*x2, *y2},
                                             table_style, table_name, {}});
        }
        return;
      }

      if (name != "Text_Element") return;
      const pugi::xml_node point = node.child("Point");
      const auto x = ReadDouble(point.attribute("X"));
      const auto y = ReadDouble(point.attribute("Y"));
      if (!x || !y) return;
      const double angle = ReadDouble(node.attribute("Angle")).value_or(0.0);
      const std::string value = node.attribute("Text").as_string();
      const pugi::xml_node format = FindCharFormat(node, table);
      double height = ReadDouble(format.attribute("Size")).value_or(0.0);
      if (height <= 0.0) {
        const double points =
            ReadDouble(format.attribute("PtSize")).value_or(0.0);
        if (points > 0.0 && points <= 72.0)
          height = points * 0.0254 / 72.0 * 0.75;
      }
      const std::string font = format.attribute("TypeFace").as_string();
      const auto key =
          std::make_tuple(value, *x, *y, angle, sheet_name, table_name);
      if (!seen_text.insert(key).second) return;
      sheet.entities.emplace_back(TextAnnotation{
          value,
          {*x, *y},
          angle,
          height,
          SumWidths(node.attribute("CellWidthArray").as_string()),
          font,
          table_name,
          TextAnchor::kStart,
          0.0,
          false});
    });
  }

  return Result<DrawingGeometry>::Ok(std::move(drawing));
}

}  // namespace openswx
