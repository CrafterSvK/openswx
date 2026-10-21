// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};
constexpr std::string_view kColumnClass = "moWeldmentTableColumn_c";
constexpr std::string_view kRowClass = "moWeldmentTableRow_c";
constexpr std::string_view kGridClass = "moWeldmentTableGrid_c";
constexpr std::string_view kDisplayClass = "moDisplayWeldmentTable_c";
constexpr std::size_t kAnchorSearchBytes = 1024u;
constexpr std::size_t kMaximumCachedTableBytes = 1024u * 1024u;
constexpr double kCoordinateTolerance = 5.0e-4;
constexpr double kTextPositionTolerance = 1.0e-5;
constexpr double kBaselineTolerance = 1.0e-6;

double ReadF64Le(std::span<const uint8_t> data, std::size_t offset) {
  std::array<uint8_t, sizeof(double)> bytes{};
  std::copy_n(data.begin() + static_cast<std::ptrdiff_t>(offset), bytes.size(),
              bytes.begin());
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  double value = 0.0;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

std::optional<std::size_t> FindBytes(std::span<const uint8_t> data,
                                     std::string_view value,
                                     std::size_t first = 0u) {
  if (first >= data.size()) return std::nullopt;
  const auto found = std::search(
      data.begin() + static_cast<std::ptrdiff_t>(first), data.end(),
      value.begin(), value.end());
  if (found == data.end()) return std::nullopt;
  return static_cast<std::size_t>(std::distance(data.begin(), found));
}

struct EncodedString {
  std::size_t offset = 0;
  std::size_t end = 0;
  std::string value;
};

void AppendUtf8(std::string& output, uint16_t codepoint) {
  if (codepoint <= 0x7Fu) {
    output.push_back(static_cast<char>(codepoint));
  } else if (codepoint <= 0x7FFu) {
    output.push_back(static_cast<char>(0xC0u | (codepoint >> 6u)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  } else {
    output.push_back(static_cast<char>(0xE0u | (codepoint >> 12u)));
    output.push_back(
        static_cast<char>(0x80u | ((codepoint >> 6u) & 0x3Fu)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  }
}

std::vector<EncodedString> ReadUtf16Strings(
    std::span<const uint8_t> data) {
  std::vector<EncodedString> strings;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    if (!std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const std::size_t units = data[offset + 3u];
    const std::size_t start = offset + 4u;
    if (units == 0u || units > 127u ||
        units > (data.size() - start) / 2u) {
      continue;
    }
    std::string value;
    value.reserve(units);
    bool valid = true;
    for (std::size_t index = 0; index < units; ++index) {
      const uint16_t codepoint =
          static_cast<uint16_t>(data[start + index * 2u]) |
          static_cast<uint16_t>(data[start + index * 2u + 1u] << 8u);
      if (codepoint == 0u ||
          (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
        valid = false;
        break;
      }
      AppendUtf8(value, codepoint);
    }
    if (valid) strings.push_back({offset, start + units * 2u,
                                  std::move(value)});
  }
  return strings;
}

std::string OwnerSheetAt(const std::vector<EncodedString>& strings,
                         const std::set<std::string>& known_sheets,
                         std::size_t offset) {
  for (auto item = strings.rbegin(); item != strings.rend(); ++item) {
    if (item->offset >= offset) continue;
    if (known_sheets.contains(item->value)) return item->value;
  }
  return {};
}

bool IsSheetPoint(double x, double y) {
  return std::isfinite(x) && std::isfinite(y) &&
      x >= 1.0e-4 && x <= 2.0 && y >= 1.0e-4 && y <= 2.0;
}

std::optional<Vec2> FindUniqueAnchor(std::span<const uint8_t> definition,
                                     std::size_t display_offset) {
  const std::size_t first = display_offset + kDisplayClass.size();
  const std::size_t last = std::min(definition.size(),
                                    first + kAnchorSearchBytes);
  std::optional<Vec2> anchor;
  for (std::size_t offset = first; offset + 16u <= last; ++offset) {
    const double x = ReadF64Le(definition, offset);
    const double y = ReadF64Le(definition, offset + 8u);
    if (!IsSheetPoint(x, y)) continue;
    if (anchor) return std::nullopt;
    anchor = Vec2{x, y};
  }
  return anchor;
}

}  // namespace

Result<std::vector<Line2D>>
DrawingGeometryReader::ReadLegacyWeldmentTableLines(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<Line2D>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }

  const auto column_offset = FindBytes(definition, kColumnClass);
  const auto row_offset = FindBytes(definition, kRowClass);
  const auto grid_offset = FindBytes(definition, kGridClass);
  const auto display_offset = FindBytes(definition, kDisplayClass);
  if (!column_offset || !row_offset || !grid_offset || !display_offset) {
    return Result<std::vector<Line2D>>::Ok({});
  }
  if (!(*column_offset < *row_offset && *row_offset < *grid_offset)) {
    return Result<std::vector<Line2D>>::Ok({});
  }

  const std::vector<EncodedString> strings =
      ReadUtf16Strings(definition);
  if (OwnerSheetAt(strings, known_sheets, *display_offset) != sheet_name) {
    return Result<std::vector<Line2D>>::Ok({});
  }

  const std::size_t first_width_offset = *column_offset + kColumnClass.size();
  const std::size_t row_height_offset = *row_offset + kRowClass.size();
  if (first_width_offset + 8u > definition.size() ||
      row_height_offset + 8u > definition.size()) {
    return Result<std::vector<Line2D>>::Ok({});
  }
  std::vector<double> widths = {ReadF64Le(definition, first_width_offset)};
  for (std::size_t offset = first_width_offset + 8u;
       offset + 10u <= *row_offset; ++offset) {
    const bool legacy_width_marker =
        definition[offset] == 0x92u && definition[offset + 1u] == 0x8Bu;
    const bool cached_width_marker =
        definition[offset] == 0x7Eu && definition[offset + 1u] == 0x88u;
    if (!legacy_width_marker && !cached_width_marker) {
      continue;
    }
    const double width = ReadF64Le(definition, offset + 2u);
    if (std::isfinite(width) && width > 1.0e-4 && width < 0.5) {
      widths.push_back(width);
    }
  }
  const double row_height = ReadF64Le(definition, row_height_offset);
  const auto anchor = FindUniqueAnchor(definition, *display_offset);
  if (!anchor || widths.size() < 2u || widths.size() > 32u ||
      !std::isfinite(row_height) || row_height <= 1.0e-4 ||
      row_height >= 0.1) {
    return Result<std::vector<Line2D>>::Ok({});
  }

  const double total_width =
      std::accumulate(widths.begin(), widths.end(), 0.0);
  const double left = anchor->x - total_width;
  const double item_number_x = left + widths.front() / 2.0;
  std::set<int> item_numbers;
  double top_item_y = 0.0;
  const std::size_t cell_content_end =
      *display_offset > *grid_offset
          ? *display_offset
          : std::min(definition.size(),
                     *grid_offset + kMaximumCachedTableBytes);
  for (const EncodedString& item : strings) {
    if (item.offset <= *grid_offset || item.offset >= cell_content_end ||
        item.end + 16u > definition.size()) {
      continue;
    }
    int number = 0;
    const auto parsed = std::from_chars(
        item.value.data(), item.value.data() + item.value.size(), number);
    if (parsed.ec != std::errc{} ||
        parsed.ptr != item.value.data() + item.value.size() ||
        number <= 0 || number > 10000) {
      continue;
    }
    const double x = ReadF64Le(definition, item.end);
    const double y = ReadF64Le(definition, item.end + 8u);
    if (std::abs(x - item_number_x) > kCoordinateTolerance ||
        !std::isfinite(y) || y <= anchor->y || y > 2.0) {
      continue;
    }
    item_numbers.insert(number);
    top_item_y = std::max(top_item_y, y);
  }
  if (item_numbers.empty()) {
    return Result<std::vector<Line2D>>::Ok({});
  }
  const int row_count = *item_numbers.rbegin();
  for (int number = 1; number <= row_count; ++number) {
    if (!item_numbers.contains(number)) {
      return Result<std::vector<Line2D>>::Ok({});
    }
  }

  const double top = top_item_y + row_height / 2.0;
  const double header_top = top - row_count * row_height;
  if (!(left >= 0.0 && left < anchor->x && anchor->y < header_top &&
        header_top < top && top <= 2.0)) {
    return Result<std::vector<Line2D>>::Ok({});
  }

  constexpr std::string_view kSource =
      "Contents/Definition/moDisplayWeldmentTable_c/grid";
  std::vector<Line2D> lines;
  double x = left;
  lines.push_back(Line2D{{x, anchor->y}, {x, top}, LineStyle{},
                         std::string(kSource)});
  for (double width : widths) {
    x += width;
    lines.push_back(Line2D{{x, anchor->y}, {x, top}, LineStyle{},
                           std::string(kSource)});
  }
  lines.push_back(Line2D{{left, anchor->y}, {anchor->x, anchor->y},
                         LineStyle{}, std::string(kSource)});
  for (int boundary = 0; boundary <= row_count; ++boundary) {
    const double y = top - boundary * row_height;
    lines.push_back(Line2D{{left, y}, {anchor->x, y}, LineStyle{},
                           std::string(kSource)});
  }
  return Result<std::vector<Line2D>>::Ok(std::move(lines));
}

Result<std::vector<TextAnnotation>>
DrawingGeometryReader::ReadLegacyWeldmentTableText(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name) {
  auto grid_result = ReadLegacyWeldmentTableLines(
      definition, sheet_names, sheet_name);
  if (!grid_result.ok()) {
    return Result<std::vector<TextAnnotation>>::Err(grid_result.error());
  }
  const std::vector<Line2D>& grid = grid_result.value();
  if (grid.empty()) return Result<std::vector<TextAnnotation>>::Ok({});

  std::vector<double> columns;
  std::vector<double> rows;
  for (const Line2D& line : grid) {
    if (std::abs(line.start.x - line.end.x) < 1.0e-12) {
      columns.push_back(line.start.x);
    } else if (std::abs(line.start.y - line.end.y) < 1.0e-12) {
      rows.push_back(line.start.y);
    }
  }
  std::sort(columns.begin(), columns.end());
  std::sort(rows.begin(), rows.end());
  if (columns.size() < 3u || rows.size() < 3u) {
    return Result<std::vector<TextAnnotation>>::Ok({});
  }
  const double bottom = rows.front();
  const double header_top = rows[1u];
  const double top = rows.back();
  const double row_height = rows.back() - rows[rows.size() - 2u];
  const std::size_t row_count = rows.size() - 2u;

  struct ColumnPosition {
    double x = 0.0;
    double width = 0.0;
    TextAnchor anchor = TextAnchor::kStart;
  };
  std::vector<ColumnPosition> positions = {
      {(columns[0] + columns[1]) / 2.0,
       columns[1] - columns[0] - 0.001, TextAnchor::kMiddle},
      {columns[1] + 0.0005, columns[2] - columns[1] - 0.001,
       TextAnchor::kStart}};
  for (std::size_t column = 2u; column + 1u < columns.size() - 1u;
       ++column) {
    positions.push_back(
        {(columns[column] + columns[column + 1u]) / 2.0,
         columns[column + 1u] - columns[column] - 0.001,
         TextAnchor::kMiddle});
  }
  positions.push_back({columns.back() - 0.0005,
                       columns.back() - columns[columns.size() - 2u] - 0.001,
                       TextAnchor::kEnd});

  const auto grid_offset = FindBytes(definition, kGridClass);
  const auto display_offset = FindBytes(definition, kDisplayClass);
  if (!grid_offset || !display_offset) {
    return Result<std::vector<TextAnnotation>>::Ok({});
  }
  const std::size_t cell_content_end =
      *display_offset > *grid_offset
          ? *display_offset
          : std::min(definition.size(),
                     *grid_offset + kMaximumCachedTableBytes);
  const std::vector<EncodedString> strings = ReadUtf16Strings(definition);
  const auto find_column = [&](double x) -> std::optional<ColumnPosition> {
    for (const ColumnPosition& position : positions) {
      if (std::abs(x - position.x) <= kTextPositionTolerance) {
        return position;
      }
    }
    return std::nullopt;
  };

  double raised_baseline = 0.0;
  const double top_center = top - row_height / 2.0;
  for (const EncodedString& item : strings) {
    if (item.offset <= *grid_offset || item.offset >= cell_content_end ||
        item.end + 24u > definition.size()) {
      continue;
    }
    const double x = ReadF64Le(definition, item.end);
    const double y = ReadF64Le(definition, item.end + 8u);
    if (!find_column(x)) continue;
    const double delta = y - top_center;
    if (delta > 5.0e-5 && delta < row_height / 2.0 &&
        (raised_baseline == 0.0 || delta < raised_baseline)) {
      raised_baseline = delta;
    }
  }

  constexpr std::string_view kFont = "Century Gothic";
  constexpr std::size_t kCellWidthAfterFont = 120u;
  constexpr std::size_t kCellHeightAfterFont = 128u;
  constexpr std::string_view kSource =
      "Contents/Definition/moDisplayWeldmentTable_c/cell";
  std::vector<TextAnnotation> annotations;
  for (std::size_t index = 0; index < strings.size(); ++index) {
    const EncodedString& item = strings[index];
    if (item.offset <= *grid_offset || item.offset >= cell_content_end ||
        item.end + 24u > definition.size()) {
      continue;
    }
    const double x = ReadF64Le(definition, item.end);
    const double y = ReadF64Le(definition, item.end + 8u);
    const double rotation = ReadF64Le(definition, item.end + 16u);
    const auto column = find_column(x);
    if (!column || !std::isfinite(y) || !std::isfinite(rotation)) continue;

    bool visible_baseline = y > bottom && y < header_top;
    for (std::size_t row = 0u; row < row_count && !visible_baseline; ++row) {
      const double centre = top - (static_cast<double>(row) + 0.5) *
                                      row_height;
      visible_baseline = std::abs(y - centre) <= kBaselineTolerance ||
          (raised_baseline > 0.0 &&
           std::abs(y - centre - raised_baseline) <= kBaselineTolerance);
    }
    if (!visible_baseline) continue;

    const EncodedString* font = nullptr;
    for (std::size_t following = index + 1u; following < strings.size();
         ++following) {
      if (strings[following].offset > item.end + 256u) break;
      if (strings[following].value == kFont) {
        font = &strings[following];
        break;
      }
    }
    double width = column->width;
    double char_format_height = 0.0;
    double measured_height = 0.0;
    if (font != nullptr &&
        font->end + kCellHeightAfterFont + 8u <= definition.size()) {
      const double serialized_width =
          ReadF64Le(definition, font->end + kCellWidthAfterFont);
      if (std::isfinite(serialized_width) && serialized_width > 0.0 &&
          serialized_width <= 0.5) {
        width = serialized_width;
      }
      char_format_height = ReadF64Le(definition, font->end);
      measured_height =
          ReadF64Le(definition, font->end + kCellHeightAfterFont);
    }
    const auto valid_height = [](double value) {
      return std::isfinite(value) && value >= 5.0e-4 && value <= 0.01;
    };
    const std::size_t glyph_count = static_cast<std::size_t>(std::count_if(
        item.value.begin(), item.value.end(), [](unsigned char byte) {
          return (byte & 0xC0u) != 0x80u;
        }));
    const bool numeric = !item.value.empty() &&
        std::all_of(item.value.begin(), item.value.end(), [](char character) {
          return (character >= '0' && character <= '9') ||
              character == '.' || character == ',' || character == '-';
        });
    const double fitted_height =
        numeric ? 0.0025
                : (glyph_count == 0u
                       ? row_height * 0.6
                       : std::min(row_height * 0.6,
                                  width /
                                      (0.62 * static_cast<double>(
                                                  glyph_count))));
    const double height = valid_height(char_format_height)
                              ? char_format_height
                              : (valid_height(measured_height)
                                     ? measured_height
                                     : fitted_height);
    if (!std::isfinite(width) || !std::isfinite(height) ||
        width <= 0.0 || width > 0.5 || height < 5.0e-4 || height > 0.01) {
      continue;
    }
    annotations.push_back(TextAnnotation{
        item.value, {x, y}, rotation, height, width, std::string(kFont),
        std::string(kSource), column->anchor});
  }
  return Result<std::vector<TextAnnotation>>::Ok(std::move(annotations));
}

}  // namespace openswx
