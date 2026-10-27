// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <set>
#include <string_view>
#include <tuple>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 5> kCurrentSketchMarker = {
    0xFF, 0xFF, 0x1F, 0x00, 0x03};
constexpr std::array<uint8_t, 5> kLegacySketchMarker = {
    0xFF, 0xFF, 0x07, 0x00, 0x01};
constexpr std::array<uint8_t, 5> kExtendedSketchMarker = {
    0xFF, 0xFF, 0x1F, 0x00, 0x01};
constexpr std::array<uint8_t, 4> kGeometryLocus = {
    0x05, 0x00, 0x01, 0x00};
constexpr std::array<uint8_t, 4> kGeometryStatePrefix = {
    0x00, 0x00, 0x80, 0xBF};
constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};

bool Matches(std::span<const uint8_t> data, std::size_t offset,
             auto marker) {
  return offset <= data.size() && marker.size() <= data.size() - offset &&
         std::equal(marker.begin(), marker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset));
}

bool IsSketchMarker(std::span<const uint8_t> data, std::size_t offset) {
  return Matches(data, offset, kCurrentSketchMarker) ||
         Matches(data, offset, kLegacySketchMarker) ||
         Matches(data, offset, kExtendedSketchMarker);
}

uint16_t ReadU16Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8u);
}

uint32_t ReadU32Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1u]) << 8u) |
         (static_cast<uint32_t>(data[offset + 2u]) << 16u) |
         (static_cast<uint32_t>(data[offset + 3u]) << 24u);
}

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

bool IsPointRecord(std::span<const uint8_t> data, std::size_t offset,
                   std::size_t record_size) {
  if (record_size != 142u && record_size != 146u) return false;
  if (!Matches(data, offset, kCurrentSketchMarker) &&
      !Matches(data, offset, kLegacySketchMarker)) {
    return false;
  }
  const uint16_t coordinate_tag = ReadU16Le(data, offset + 64u);
  return Matches(data, offset + 23u, kGeometryLocus) &&
         ReadU16Le(data, offset + 27u) == 1u &&
         Matches(data, offset + 31u, kGeometryStatePrefix) &&
         ReadF64Le(data, offset + 48u) == 1.0 &&
         (coordinate_tag == 0x0012u || coordinate_tag == 0x001Eu);
}

bool IsIndexedSegmentRecord(std::span<const uint8_t> data,
                            std::size_t offset, std::size_t record_size) {
  if (record_size != 92u ||
      (!Matches(data, offset, kCurrentSketchMarker) &&
       !Matches(data, offset, kLegacySketchMarker))) {
    return false;
  }
  static constexpr std::array<uint8_t, 8> kMinusOne = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xBF};
  static constexpr std::array<uint8_t, 8> kZero = {};
  const uint16_t role = ReadU16Le(data, offset + 27u);
  const uint16_t state = ReadU16Le(data, offset + 37u);
  const bool confirmed_state =
      (role == 1u && (state == 4u || state == 5u)) ||
      (role == 2u && state == 12u);
  return Matches(data, offset + 23u, kGeometryLocus) &&
         confirmed_state &&
         Matches(data, offset + 31u, kGeometryStatePrefix) &&
         ReadF64Le(data, offset + 48u) == 1.0 &&
         Matches(data, offset + 56u, kZero) &&
         Matches(data, offset + 72u, kMinusOne);
}

bool IsPlausibleSheetPoint(Vec2 point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         point.x >= -0.1 && point.x <= 2.0 &&
         point.y >= -0.1 && point.y <= 2.0;
}

bool IsRenderableSheetPoint(Vec2 point) {
  return point.x >= 0.0 && point.y >= 0.0;
}

void AppendUtf8(std::string& output, uint16_t codepoint) {
  if (codepoint <= 0x7Fu) {
    output.push_back(static_cast<char>(codepoint));
  } else if (codepoint <= 0x7FFu) {
    output.push_back(static_cast<char>(0xC0u | (codepoint >> 6u)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  } else {
    output.push_back(static_cast<char>(0xE0u | (codepoint >> 12u)));
    output.push_back(static_cast<char>(0x80u | ((codepoint >> 6u) & 0x3Fu)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  }
}

struct Utf16Field {
  std::string value;
  std::size_t end = 0;
};

std::optional<Utf16Field> ReadUtf16Field(std::span<const uint8_t> data,
                                         std::size_t marker_offset) {
  if (!Matches(data, marker_offset, kUtf16Marker)) return std::nullopt;
  std::size_t length_offset = marker_offset + kUtf16Marker.size();
  if (length_offset >= data.size()) return std::nullopt;
  std::size_t units = data[length_offset++];
  if (units == 0xFFu) {
    if (length_offset + 2u > data.size()) return std::nullopt;
    units = ReadU16Le(data, length_offset);
    length_offset += 2u;
  }
  if (units == 0u || units > 4096u ||
      units > (data.size() - length_offset) / 2u) {
    return std::nullopt;
  }
  Utf16Field result;
  result.value.reserve(units);
  for (std::size_t index = 0; index < units; ++index) {
    const uint16_t codepoint = ReadU16Le(data, length_offset + index * 2u);
    if (codepoint == 0u || (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
      return std::nullopt;
    }
    AppendUtf8(result.value, codepoint);
  }
  result.end = length_offset + units * 2u;
  return result;
}

std::string StripRichTextTags(std::string_view input) {
  std::string result;
  result.reserve(input.size());
  bool in_tag = false;
  for (const char character : input) {
    if (character == '<') {
      in_tag = true;
    } else if (character == '>' && in_tag) {
      in_tag = false;
    } else if (!in_tag) {
      result.push_back(character);
    }
  }
  return result;
}

std::optional<Utf16Field> FindFontField(std::span<const uint8_t> definition,
                                        std::size_t start) {
  const std::size_t end = std::min(definition.size(), start + 80u);
  for (std::size_t offset = start;
       offset + kUtf16Marker.size() < end; ++offset) {
    if (auto field = ReadUtf16Field(definition, offset)) return field;
  }
  return std::nullopt;
}

bool HasNoteFieldPrefix(std::span<const uint8_t> definition,
                        std::size_t marker_offset) {
  if (marker_offset < 12u || definition[marker_offset - 12u] != 0x05u) {
    return false;
  }
  // The two bytes directly before FF FE FF are a version/object-map tag
  // (observed as 0D 83 in SLDDRT, FD 8A and DB 85 in SLDDRW). The preceding
  // 05 + nine zero bytes are stable across the confirmed corpus.
  return std::all_of(
      definition.begin() + static_cast<std::ptrdiff_t>(marker_offset - 11u),
      definition.begin() + static_cast<std::ptrdiff_t>(marker_offset - 2u),
      [](uint8_t byte) { return byte == 0u; });
}

struct NamedOffset {
  std::size_t offset = 0;
  std::string name;
};

std::vector<NamedOffset> FindSheetNames(
    std::span<const uint8_t> definition,
    const std::set<std::string>& known_sheets) {
  std::vector<NamedOffset> result;
  for (std::size_t offset = 0;
       offset + kUtf16Marker.size() < definition.size(); ++offset) {
    if (!Matches(definition, offset, kUtf16Marker)) continue;
    auto field = ReadUtf16Field(definition, offset);
    if (field && known_sheets.contains(field->value)) {
      result.push_back({offset, std::move(field->value)});
    }
    offset += kUtf16Marker.size() - 1u;
  }
  return result;
}

std::string OwnerSheetAt(std::span<const NamedOffset> sheets,
                         std::size_t offset) {
  const auto owner = std::upper_bound(
      sheets.begin(), sheets.end(), offset,
      [](std::size_t value, const NamedOffset& item) {
        return value < item.offset;
      });
  return owner == sheets.begin() ? std::string{} : std::prev(owner)->name;
}

}  // namespace

Result<std::vector<Line2D>> DrawingGeometryReader::ReadSheetFormatLines(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name,
    double sheet_width_m,
    double sheet_height_m,
    bool suppress_bitmap_helpers) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<Line2D>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }
  const std::vector<NamedOffset> sheets =
      FindSheetNames(definition, known_sheets);
  std::vector<std::size_t> markers;
  for (std::size_t offset = 0; offset + kCurrentSketchMarker.size() <=
                                   definition.size(); ++offset) {
    if (!IsSketchMarker(definition, offset)) continue;
    markers.push_back(offset);
    offset += kCurrentSketchMarker.size() - 1u;
  }

  std::vector<Vec2> points;
  std::vector<std::vector<Line2D>> runs;
  std::vector<Line2D> current_run;
  std::set<std::tuple<double, double, double, double>> seen;
  std::string current_owner;
  const auto finish_run = [&]() {
    if (!current_run.empty()) runs.push_back(std::move(current_run));
    current_run.clear();
    points.clear();
    seen.clear();
  };
  for (std::size_t index = 0; index < markers.size(); ++index) {
    const std::size_t offset = markers[index];
    const std::string owner = OwnerSheetAt(sheets, offset);
    if (owner != current_owner) {
      finish_run();
      current_owner = owner;
    }
    if (owner != sheet_name) continue;
    const std::size_t end = index + 1u < markers.size()
                                ? markers[index + 1u]
                                : definition.size();
    const std::size_t record_size = end - offset;
    if (record_size > 4096u) {
      finish_run();
      continue;
    }
    if (IsPointRecord(definition, offset, record_size)) {
      const Vec2 point = {ReadF64Le(definition, offset + 66u),
                          ReadF64Le(definition, offset + 74u)};
      if (IsPlausibleSheetPoint(point)) {
        points.push_back(point);
      } else {
        finish_run();
      }
      continue;
    }
    if (!IsIndexedSegmentRecord(definition, offset, record_size)) continue;
    const std::size_t start_index = ReadU16Le(definition, offset + 64u);
    const std::size_t end_index = ReadU16Le(definition, offset + 66u);
    if (start_index == end_index || start_index >= points.size() ||
        end_index >= points.size()) {
      continue;
    }
    const Vec2 start = points[start_index];
    const Vec2 finish = points[end_index];
    // Keep off-sheet points in the roster so all following zero-based
    // references retain their meaning, but do not render construction
    // geometry whose endpoint lies outside the physical sheet quadrant.
    if (!IsRenderableSheetPoint(start) || !IsRenderableSheetPoint(finish)) {
      continue;
    }
    const double length = std::hypot(finish.x - start.x, finish.y - start.y);
    if (!std::isfinite(length) || length <= 1e-10 || length > 2.0) continue;
    // State 5 also serializes helper geometry belonging to embedded sketch
    // bitmaps/blocks. Without a decoded bitmap, retain its short strokes as a
    // useful logo fallback. Once the real bitmap is available, suppress the
    // complete helper layer; none of it appears in SolidWorks' preview.
    const bool state_five = ReadU16Le(definition, offset + 37u) == 5u;
    if (state_five && (suppress_bitmap_helpers || length > 0.02)) {
      continue;
    }
    const auto key = std::make_tuple(start.x, start.y, finish.x, finish.y);
    const auto reverse = std::make_tuple(finish.x, finish.y, start.x, start.y);
    if (seen.contains(key) || seen.contains(reverse)) continue;
    seen.insert(key);
    current_run.push_back(Line2D{
        start, finish, LineStyle{},
        "Contents/Definition/sgSketch/sheet-format/" +
            std::string(sheet_name),
        {}});
  }
  finish_run();

  const bool has_sheet_size =
      std::isfinite(sheet_width_m) && std::isfinite(sheet_height_m) &&
      sheet_width_m > 0.0 && sheet_height_m > 0.0;
  const auto physical = [&](const Line2D& line) {
    constexpr double kTolerance = 1e-6;
    return !has_sheet_size ||
           (line.start.x <= sheet_width_m + kTolerance &&
            line.end.x <= sheet_width_m + kTolerance &&
            line.start.y <= sheet_height_m + kTolerance &&
            line.end.y <= sheet_height_m + kTolerance);
  };
  const auto border_score = [&](const std::vector<Line2D>& run) {
    if (!has_sheet_size) return std::size_t{0};
    constexpr double kAxisTolerance = 1e-6;
    constexpr double kMarginLimit = 0.02;
    std::size_t score = 0;
    for (const Line2D& line : run) {
      if (!physical(line)) continue;
      const double dx = std::abs(line.end.x - line.start.x);
      const double dy = std::abs(line.end.y - line.start.y);
      const double centre_x = (line.start.x + line.end.x) * 0.5;
      const double centre_y = (line.start.y + line.end.y) * 0.5;
      const bool horizontal_border =
          dy < kAxisTolerance && dx > sheet_width_m * 0.8 &&
          (centre_y < kMarginLimit ||
           sheet_height_m - centre_y < kMarginLimit);
      const bool vertical_border =
          dx < kAxisTolerance && dy > sheet_height_m * 0.8 &&
          (centre_x < kMarginLimit ||
           sheet_width_m - centre_x < kMarginLimit);
      if (horizontal_border || vertical_border) ++score;
    }
    return score;
  };

  auto selected = runs.end();
  std::pair<std::size_t, std::size_t> selected_score{};
  for (auto run = runs.begin(); run != runs.end(); ++run) {
    const std::size_t borders = border_score(*run);
    const std::size_t physical_count =
        static_cast<std::size_t>(std::count_if(run->begin(), run->end(), physical));
    const auto score = std::make_pair(borders, physical_count);
    if (selected == runs.end() || score > selected_score) {
      selected = run;
      selected_score = score;
    }
  }
  // When a physical sheet size is known, require at least two independent
  // border edges. This rejects a larger user sketch from being guessed as the
  // sheet format merely because it contains more segments.
  if (selected == runs.end() || (has_sheet_size && selected_score.first < 2u)) {
    return Result<std::vector<Line2D>>::Ok({});
  }
  std::vector<Line2D> lines;
  for (Line2D& line : *selected) {
    if (physical(line)) lines.push_back(std::move(line));
  }
  return Result<std::vector<Line2D>>::Ok(std::move(lines));
}

Result<std::vector<TextAnnotation>>
DrawingGeometryReader::ReadSheetFormatText(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<TextAnnotation>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }
  const std::vector<NamedOffset> sheets =
      FindSheetNames(definition, known_sheets);
  std::vector<TextAnnotation> annotations;
  std::set<std::tuple<std::string, double, double>> seen;
  for (std::size_t offset = 0;
       offset + kUtf16Marker.size() < definition.size(); ++offset) {
    if (!Matches(definition, offset, kUtf16Marker) ||
        !HasNoteFieldPrefix(definition, offset) ||
        OwnerSheetAt(sheets, offset) != sheet_name) {
      continue;
    }
    auto field = ReadUtf16Field(definition, offset);
    if (!field || field->end + 24u > definition.size()) continue;

    std::string text = StripRichTextTags(field->value);
    // Dynamic property expressions need a separate property resolver. Long
    // paragraphs need SVG tspans and line-spacing metadata; neither should be
    // rendered as a misleading single raw line in this first decoder.
    if (text.empty() || text.size() > 160u ||
        text.find("$PRP") != std::string::npos ||
        text.find_first_of("\r\n") != std::string::npos) {
      continue;
    }
    const Vec2 origin = {ReadF64Le(definition, field->end),
                         ReadF64Le(definition, field->end + 8u)};
    const double rotation = ReadF64Le(definition, field->end + 16u);
    if (!IsPlausibleSheetPoint(origin) || origin.x < 1e-4 ||
        origin.y < 1e-4 ||
        !std::isfinite(rotation) || std::abs(rotation) > 7.0) {
      continue;
    }
    const auto key = std::make_tuple(text, origin.x, origin.y);
    if (!seen.insert(key).second) continue;

    TextAnnotation annotation;
    annotation.text = std::move(text);
    annotation.origin = origin;
    annotation.rotation_radians = std::abs(rotation) < 1e-12 ? 0.0 : rotation;
    annotation.height = 0.0015;
    if (auto font = FindFontField(definition, field->end + 24u)) {
      annotation.font_family = font->value;
      if (font->end + sizeof(double) <= definition.size()) {
        const double height = ReadF64Le(definition, font->end);
        if (std::isfinite(height) && height >= 0.0001 && height <= 0.02) {
          annotation.height = height;
        }
      }
    }
    annotation.source_table = "Contents/Definition/moDisplayNote_c/" +
                              std::string(sheet_name);
    annotations.push_back(std::move(annotation));
  }
  return Result<std::vector<TextAnnotation>>::Ok(std::move(annotations));
}

Result<std::vector<RasterImage2D>> DrawingGeometryReader::ReadSketchBitmap(
    std::span<const uint8_t> definition,
    std::span<const uint8_t> sketch_bitmap) {
  constexpr std::string_view kBitmapDataClass = "moSketchBitmapData_c";
  const auto found = std::search(definition.begin(), definition.end(),
                                 kBitmapDataClass.begin(),
                                 kBitmapDataClass.end());
  if (found == definition.end() || sketch_bitmap.size() < 6u) {
    return Result<std::vector<RasterImage2D>>::Ok({});
  }
  const std::size_t data_offset =
      static_cast<std::size_t>(std::distance(definition.begin(), found)) +
      kBitmapDataClass.size();
  if (data_offset + 56u > definition.size() ||
      sketch_bitmap[0] != 0xFFu || sketch_bitmap[1] != 0xFFu) {
    return Result<std::vector<RasterImage2D>>::Ok({});
  }

  const Vec2 bottom_left = {ReadF64Le(definition, data_offset),
                            ReadF64Le(definition, data_offset + 8u)};
  const double rotation = ReadF64Le(definition, data_offset + 16u);
  const double pixel_width_m = ReadF64Le(definition, data_offset + 24u);
  const double pixel_height_m = ReadF64Le(definition, data_offset + 32u);
  const uint32_t pixel_width = ReadU32Le(definition, data_offset + 48u);
  const uint32_t pixel_height = ReadU32Le(definition, data_offset + 52u);
  const uint32_t declared_bytes = ReadU32Le(sketch_bitmap, 2u);
  if (!IsPlausibleSheetPoint(bottom_left) || !std::isfinite(rotation) ||
      std::abs(rotation) > 1e-10 || !std::isfinite(pixel_width_m) ||
      !std::isfinite(pixel_height_m) || pixel_width_m <= 0.0 ||
      pixel_height_m <= 0.0 || pixel_width_m > 0.001 ||
      pixel_height_m > 0.001 || pixel_width == 0u || pixel_height == 0u ||
      pixel_width > 8192u || pixel_height > 8192u ||
      declared_bytes != sketch_bitmap.size() - 6u) {
    return Result<std::vector<RasterImage2D>>::Ok({});
  }
  const std::size_t expected_bytes =
      static_cast<std::size_t>(pixel_width) * pixel_height * 4u;
  if (expected_bytes != declared_bytes) {
    return Result<std::vector<RasterImage2D>>::Ok({});
  }

  RasterImage2D image;
  image.bottom_left = bottom_left;
  image.width = pixel_width_m * static_cast<double>(pixel_width);
  image.height = pixel_height_m * static_cast<double>(pixel_height);
  image.pixel_width = pixel_width;
  image.pixel_height = pixel_height;
  image.rgba.assign(sketch_bitmap.begin() + 6, sketch_bitmap.end());
  image.source_table =
      "Contents/Definition/moSketchBitmapData_c+Contents/SketchBitmapN";
  return Result<std::vector<RasterImage2D>>::Ok({std::move(image)});
}

}  // namespace openswx
