// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string_view>

#include <pugixml.hpp>

namespace openswx {
namespace {

constexpr double kDisplayListBalloonLabelToleranceM = 0.008;

std::string_view DisplayObjectId(std::string_view source_table) {
  constexpr std::string_view marker = "objID=";
  const std::size_t begin = source_table.find(marker);
  if (begin == std::string_view::npos)
    return {};
  const std::size_t end = source_table.find('/', begin);
  return source_table.substr(begin, end - begin);
}

void AlignLeaderLabel(TextAnnotation &text,
                      std::span<const DrawingEntity> entities) {
  if (text.text.empty() || text.text.size() > 6u || text.height <= 0.0 ||
      !std::all_of(text.text.begin(), text.text.end(), [](char character) {
        return character >= '0' && character <= '9';
      })) {
    return;
  }
  const std::string_view owner = DisplayObjectId(text.source_table);
  const Vec2 tangent{std::cos(text.rotation_radians),
                     std::sin(text.rotation_radians)};
  const Vec2 normal{-tangent.y, tangent.x};
  std::optional<Vec2> landing;
  for (const DrawingEntity &entity : entities) {
    const auto *line = std::get_if<Line2D>(&entity);
    if (line == nullptr ||
        !line->source_table.starts_with("Contents/DisplayLists/line/") ||
        DisplayObjectId(line->source_table) != owner) {
      continue;
    }
    const Vec2 delta{line->end.x - line->start.x, line->end.y - line->start.y};
    const double length = std::hypot(delta.x, delta.y);
    if (length < text.height || length > 3.0 * text.height ||
        std::abs(delta.x * normal.x + delta.y * normal.y) > 1e-8) {
      continue;
    }
    const Vec2 middle{(line->start.x + line->end.x) * 0.5,
                      (line->start.y + line->end.y) * 0.5};
    const Vec2 offset{text.origin.x - middle.x, text.origin.y - middle.y};
    if (std::abs(offset.x * tangent.x + offset.y * tangent.y) >
            length * 0.5 + text.height ||
        std::abs(offset.x * normal.x + offset.y * normal.y) > text.height) {
      continue;
    }
    // Only an unambiguous nearby landing belonging to this label is usable.
    if (landing)
      return;
    landing = middle;
  }
  if (!landing)
    return;
  const double clearance = (text.origin.x - landing->x) * normal.x +
                           (text.origin.y - landing->y) * normal.y;
  const double gap = text.height * 0.2;
  if (clearance >= gap)
    return;
  // Cached BOM labels can place their text origin below the landing (item
  // 8497/List2: 0.597 mm for a 4 mm font). Treating it as an SVG baseline
  // strikes through the digits. Lay out the numeric label above its own
  // landing, independently of the substituted font's advance widths.
  text.origin = {landing->x + normal.x * gap, landing->y + normal.y * gap};
  text.anchor = TextAnchor::kMiddle;
}

void AlignAngularDimensionLabel(TextAnnotation &text,
                                std::span<const DrawingEntity> entities) {
  constexpr std::string_view degree = "\xC2\xB0";
  if (!text.text.ends_with(degree) || text.text.size() <= degree.size() ||
      text.height <= 0.0) {
    return;
  }
  const std::string_view number(text.text.data(),
                                text.text.size() - degree.size());
  if (!std::all_of(number.begin(), number.end(), [](char character) {
        return (character >= '0' && character <= '9') || character == '.' ||
               character == ',';
      })) {
    return;
  }
  const auto owner = DisplayObjectId(text.source_table);
  const Arc2D *circle = nullptr;
  double radius = 0.0;
  for (const auto &entity : entities) {
    const auto *arc = std::get_if<Arc2D>(&entity);
    if (!arc || !arc->source_table.starts_with("Contents/DisplayLists/arc/") ||
        DisplayObjectId(arc->source_table) != owner) {
      continue;
    }
    const double candidate_radius =
        std::hypot(arc->start.x - arc->center.x, arc->start.y - arc->center.y);
    // A dimension may split its arc at the arrows; all pieces must describe
    // the same circle before using it to lay out the label.
    if (circle && (std::hypot(circle->center.x - arc->center.x,
                              circle->center.y - arc->center.y) > 1e-8 ||
                   std::abs(radius - candidate_radius) > 1e-8)) {
      return;
    }
    circle = arc;
    radius = candidate_radius;
  }
  if (!circle || radius <= text.height)
    return;
  const Vec2 normal{-std::sin(text.rotation_radians),
                    std::cos(text.rotation_radians)};
  const Vec2 offset{text.origin.x - circle->center.x,
                    text.origin.y - circle->center.y};
  if (std::abs(std::hypot(offset.x, offset.y) - radius) > 3.0 * text.height) {
    return;
  }
  const double side =
      offset.x * normal.x + offset.y * normal.y >= 0.0 ? 1.0 : -1.0;
  // On the inside of an arc, its ends rise towards the glyphs. Allow for
  // that curvature as well as the baseline gap. Width is a conservative
  // fallback for the substituted technical font, not a decoded font metric.
  const double half_width =
      text.height * 0.35 * static_cast<double>(number.size() + 1u);
  if (half_width >= radius)
    return;
  const double curvature =
      side < 0.0 ? radius - std::sqrt(radius * radius - half_width * half_width)
                 : 0.0;
  const double distance = side * radius + text.height * 0.2 + curvature;
  text.origin = {circle->center.x + normal.x * distance,
                 circle->center.y + normal.y * distance};
  text.anchor = TextAnchor::kMiddle;
}

std::vector<FilledTriangle2D>
FindDisplayListArrowheads(std::span<const Line2D> lines) {
  constexpr double kPointToleranceM = 1e-8;
  constexpr double kMinimumArmLengthM = 5e-4;
  constexpr double kMaximumArmLengthM = 5e-3;
  constexpr double kMinimumBaseWidthM = 2e-4;
  constexpr double kMaximumBaseWidthM = 3e-3;
  std::vector<FilledTriangle2D> triangles;
  std::set<std::tuple<std::string, double, double>> seen;
  const auto same_point = [](Vec2 left, Vec2 right) {
    return std::hypot(left.x - right.x, left.y - right.y) < kPointToleranceM;
  };
  for (std::size_t first = 0; first < lines.size(); ++first) {
    const Line2D &left = lines[first];
    if (!left.source_table.starts_with("Contents/DisplayLists/line/objID=")) {
      continue;
    }
    for (std::size_t second = first + 1u; second < lines.size(); ++second) {
      const Line2D &right = lines[second];
      if (left.source_table != right.source_table)
        continue;
      Vec2 tip;
      Vec2 left_base;
      Vec2 right_base;
      if (same_point(left.start, right.start)) {
        tip = left.start;
        left_base = left.end;
        right_base = right.end;
      } else if (same_point(left.start, right.end)) {
        tip = left.start;
        left_base = left.end;
        right_base = right.start;
      } else if (same_point(left.end, right.start)) {
        tip = left.end;
        left_base = left.start;
        right_base = right.end;
      } else if (same_point(left.end, right.end)) {
        tip = left.end;
        left_base = left.start;
        right_base = right.start;
      } else {
        continue;
      }
      const double left_length =
          std::hypot(left_base.x - tip.x, left_base.y - tip.y);
      const double right_length =
          std::hypot(right_base.x - tip.x, right_base.y - tip.y);
      const double base_width =
          std::hypot(left_base.x - right_base.x, left_base.y - right_base.y);
      if (left_length < kMinimumArmLengthM ||
          left_length > kMaximumArmLengthM ||
          right_length < kMinimumArmLengthM ||
          right_length > kMaximumArmLengthM ||
          base_width < kMinimumBaseWidthM || base_width > kMaximumBaseWidthM) {
        continue;
      }
      const auto point_on_line = [&](Vec2 point, const Line2D &line) {
        const Vec2 direction{line.end.x - line.start.x,
                             line.end.y - line.start.y};
        const double length_squared =
            direction.x * direction.x + direction.y * direction.y;
        if (length_squared <= kPointToleranceM * kPointToleranceM)
          return false;
        const Vec2 offset{point.x - line.start.x, point.y - line.start.y};
        const double cross =
            std::abs(offset.x * direction.y - offset.y * direction.x);
        const double parameter =
            (offset.x * direction.x + offset.y * direction.y) / length_squared;
        return cross <= kPointToleranceM * std::sqrt(length_squared) &&
               parameter >= -kPointToleranceM &&
               parameter <= 1.0 + kPointToleranceM;
      };
      const bool has_outline_base =
          std::any_of(lines.begin(), lines.end(), [&](const Line2D &line) {
            return line.source_table == left.source_table &&
                   point_on_line(left_base, line) &&
                   point_on_line(right_base, line);
          });
      // A third side closes an outlined weld/surface symbol. Only an open pair
      // of short arms is an arrowhead that needs a filled SVG triangle.
      if (has_outline_base)
        continue;
      const double dot = ((left_base.x - tip.x) * (right_base.x - tip.x) +
                          (left_base.y - tip.y) * (right_base.y - tip.y)) /
                         (left_length * right_length);
      // Cached arrowheads are narrow. Wider open V shapes are engineering
      // symbols and must retain their two-line outline.
      if (!std::isfinite(dot) || dot < 0.9)
        continue;
      if (!seen.emplace(left.source_table, tip.x, tip.y).second)
        continue;
      triangles.push_back(FilledTriangle2D{{tip, left_base, right_base},
                                           0x000000u,
                                           1.0,
                                           left.source_table + "/arrowhead"});
    }
  }
  return triangles;
}

constexpr std::array<uint8_t, 5> kObjectNameMarker = {0x09, 0x89, 0xFF, 0xFE,
                                                      0xFF};
constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};
constexpr std::array<uint8_t, 4> kClassMarker = {0xFF, 0xFF, 0x01, 0x00};
constexpr std::size_t kMatrixBeforePlacement = 317u;

uint16_t ReadU16Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8u);
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

void AppendUtf8(std::string &output, uint16_t codepoint) {
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

std::optional<std::string> ReadUtf16Name(std::span<const uint8_t> data,
                                         std::size_t marker_offset,
                                         std::size_t marker_size) {
  const std::size_t length_offset = marker_offset + marker_size;
  if (length_offset >= data.size())
    return std::nullopt;
  const std::size_t units = data[length_offset];
  const std::size_t start = length_offset + 1u;
  if (units == 0u || units > 127u || start > data.size() ||
      units > (data.size() - start) / 2u) {
    return std::nullopt;
  }
  std::string value;
  value.reserve(units);
  for (std::size_t index = 0; index < units; ++index) {
    const uint16_t codepoint = ReadU16Le(data, start + index * 2u);
    if (codepoint == 0u || (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
      return std::nullopt;
    }
    AppendUtf8(value, codepoint);
  }
  return value;
}

std::optional<int> FindConfiguredLineWeight(std::span<const uint8_t> definition,
                                            std::string_view category) {
  constexpr std::string_view kLineFontConfigClass = "uiLineFontMgr_c";
  const auto class_position =
      std::search(definition.begin(), definition.end(),
                  kLineFontConfigClass.begin(), kLineFontConfigClass.end());
  if (class_position == definition.end()) return std::nullopt;
  const std::size_t first = static_cast<std::size_t>(
      std::distance(definition.begin(), class_position)) +
                            kLineFontConfigClass.size();
  const std::size_t limit = std::min(definition.size(), first + 16384u);
  for (std::size_t offset = first; offset + 4u <= limit; ++offset) {
    if (!std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                    definition.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const auto name = ReadUtf16Name(definition, offset, kUtf16Marker.size());
    if (!name || *name != category) continue;
    const std::size_t category_end = offset + 4u + definition[offset + 3u] * 2u;
    const std::size_t style_limit = std::min(limit, category_end + 32u);
    for (std::size_t style_offset = category_end;
         style_offset + 4u <= style_limit; ++style_offset) {
      if (!std::equal(
              kUtf16Marker.begin(), kUtf16Marker.end(),
              definition.begin() + static_cast<std::ptrdiff_t>(style_offset))) {
        continue;
      }
      const auto style_name =
          ReadUtf16Name(definition, style_offset, kUtf16Marker.size());
      if (!style_name) continue;
      const std::size_t style_end =
          style_offset + 4u + definition[style_offset + 3u] * 2u;
      if (style_end + 2u > definition.size()) return std::nullopt;
      const int weight = ReadU16Le(definition, style_end);
      if (weight >= 0 && weight <= 10) return weight;
      return std::nullopt;
    }
  }
  return std::nullopt;
}

struct NamedOffset {
  std::size_t offset = 0;
  std::string name;
};

std::vector<NamedOffset>
FindKnownViewNames(std::span<const uint8_t> data,
                   const std::set<std::string> &known_names) {
  const auto scan = [&](auto marker) {
    std::vector<NamedOffset> result;
    if (data.size() < marker.size() + 1u)
      return result;
    for (std::size_t offset = 0; offset + marker.size() + 1u <= data.size();
         ++offset) {
      if (!std::equal(marker.begin(), marker.end(),
                      data.begin() + static_cast<std::ptrdiff_t>(offset))) {
        continue;
      }
      auto name = ReadUtf16Name(data, offset, marker.size());
      if (name && known_names.contains(*name)) {
        result.push_back({offset, std::move(*name)});
      }
      offset += marker.size() - 1u;
    }
    return result;
  };

  // Current files tag object names with 09 89 before the common UTF-16
  // marker. SolidWorks 19000 stores the same length-prefixed names without
  // that object prefix. Only use the broader scan when no current marker was
  // found, so references elsewhere in newer object payloads cannot steal
  // ownership from a confirmed object name.
  auto result = scan(kObjectNameMarker);
  return result.empty() ? scan(kUtf16Marker) : result;
}

std::vector<NamedOffset>
FindDrawingSheetObjects(std::span<const uint8_t> definition,
                        std::span<const std::string> sheet_names) {
  constexpr std::string_view kSheetClass = "moDrSheet";
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  std::optional<std::array<uint8_t, 2>> sheet_token;
  for (std::size_t offset = 0;
       offset + kClassMarker.size() + 2u + kSheetClass.size() + 2u <=
       definition.size();
       ++offset) {
    if (!std::equal(kClassMarker.begin(), kClassMarker.end(),
                    definition.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const uint16_t length = ReadU16Le(definition, offset + kClassMarker.size());
    const std::size_t name_offset = offset + kClassMarker.size() + 2u;
    if (length != kSheetClass.size() ||
        !std::equal(kSheetClass.begin(), kSheetClass.end(),
                    definition.begin() +
                        static_cast<std::ptrdiff_t>(name_offset))) {
      continue;
    }
    sheet_token = {definition[name_offset + length],
                   definition[name_offset + length + 1u]};
    break;
  }
  if (!sheet_token)
    return {};

  std::vector<NamedOffset> sheets;
  for (std::size_t offset = 0;
       offset + sheet_token->size() + kUtf16Marker.size() + 1u <=
       definition.size();
       ++offset) {
    if (!std::equal(sheet_token->begin(), sheet_token->end(),
                    definition.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const std::size_t name_offset = offset + sheet_token->size();
    auto name = ReadUtf16Name(definition, name_offset, kUtf16Marker.size());
    if (name && known_sheets.contains(*name))
      sheets.push_back({name_offset, std::move(*name)});
  }
  return sheets;
}

std::optional<std::pair<std::size_t, std::size_t>>
FindDrawingSheetRange(std::span<const uint8_t> definition,
                      std::span<const std::string> sheet_names,
                      std::string_view sheet_name) {
  const std::vector<NamedOffset> sheets =
      FindDrawingSheetObjects(definition, sheet_names);
  const auto selected = std::find_if(
      sheets.rbegin(), sheets.rend(),
      [&](const NamedOffset &item) { return item.name == sheet_name; });
  if (selected == sheets.rend())
    return std::nullopt;
  const auto following =
      std::find_if(selected.base(), sheets.end(), [&](const NamedOffset &item) {
        return item.name != sheet_name;
      });
  return std::pair{selected->offset, following == sheets.end()
                                         ? definition.size()
                                         : following->offset};
}

bool IsSectionAssemblyName(std::string_view name) {
  constexpr std::string_view kMarker = "-SectionAssembly-";
  const std::size_t marker = name.rfind(kMarker);
  return marker != std::string_view::npos &&
         marker + kMarker.size() < name.size() &&
         std::all_of(
             name.begin() +
                 static_cast<std::ptrdiff_t>(marker + kMarker.size()),
             name.end(),
             [](char character) {
               return character >= '0' && character <= '9';
             });
}

std::vector<std::string>
FindSectionAssemblyNames(std::span<const uint8_t> definition,
                         std::span<const std::string> sheet_names,
                         std::string_view sheet_name) {
  const auto range = FindDrawingSheetRange(definition, sheet_names, sheet_name);
  if (!range)
    return {};
  std::vector<std::string> names;
  for (std::size_t offset = range->first;
       offset + kUtf16Marker.size() + 1u <= range->second; ++offset) {
    if (!std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                    definition.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    auto name = ReadUtf16Name(definition, offset, kUtf16Marker.size());
    if (name && IsSectionAssemblyName(*name) &&
        std::find(names.begin(), names.end(), *name) == names.end()) {
      names.push_back(std::move(*name));
    }
    offset += kUtf16Marker.size() - 1u;
  }
  return names;
}

bool IsPlacementRecord(std::span<const uint8_t> data, std::size_t offset,
                       std::array<double, 11> &values) {
  if (offset > data.size() || data.size() - offset < 88u)
    return false;
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = ReadF64Le(data, offset + index * 8u);
  }
  if (!std::all_of(values.begin(), values.begin() + 10,
                   [](double value) {
                     return std::isfinite(value) && value >= -0.1 &&
                            value <= 0.6;
                   }) ||
      !std::isfinite(values[10]) || std::abs(values[10]) > 1e-12) {
    return false;
  }
  // The clip rectangle is normally inset from the displayed view bounds, but
  // cropped/detail views may use a different inset on each side.  The final
  // two coordinates are a serialized view reference point whose exact role
  // is still unresolved, not necessarily the rectangle midpoint (and they
  // may even lie outside the displayed bounds).
  if (values[0] >= values[2] || values[1] >= values[3] ||
      values[4] >= values[6] || values[5] >= values[7]) {
    return false;
  }
  const std::array<double, 4> insets = {
      values[4] - values[0], values[5] - values[1], values[2] - values[6],
      values[3] - values[7]};
  return std::all_of(insets.begin(), insets.end(), [](double value) {
    return value >= 0.0001 && value <= 0.02;
  });
}

double ColumnDot(const std::array<double, 9> &matrix, std::size_t column,
                 Vec3 point) {
  return matrix[column] * point.x + matrix[column + 3u] * point.y +
         matrix[column + 6u] * point.z;
}

bool ValidMatrix(const std::array<double, 9> &matrix) {
  for (std::size_t row = 0; row < 3u; ++row) {
    double norm = 0.0;
    for (std::size_t column = 0; column < 3u; ++column) {
      const double value = matrix[row * 3u + column];
      if (!std::isfinite(value))
        return false;
      norm += value * value;
    }
    if (std::abs(norm - 1.0) > 1e-8)
      return false;
  }
  for (std::size_t left = 0; left < 3u; ++left) {
    for (std::size_t right = left + 1u; right < 3u; ++right) {
      double dot = 0.0;
      for (std::size_t column = 0; column < 3u; ++column) {
        dot += matrix[left * 3u + column] * matrix[right * 3u + column];
      }
      if (std::abs(dot) > 1e-8)
        return false;
    }
  }
  return true;
}

bool ClipLine(Rect2 rectangle, Vec2 &start, Vec2 &end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const std::array<double, 4> p = {-dx, dx, -dy, dy};
  const std::array<double, 4> q = {
      start.x - rectangle.min.x, rectangle.max.x - start.x,
      start.y - rectangle.min.y, rectangle.max.y - start.y};
  double first = 0.0;
  double last = 1.0;
  for (std::size_t index = 0; index < p.size(); ++index) {
    if (std::abs(p[index]) < 1e-15) {
      if (q[index] < 0.0)
        return false;
      continue;
    }
    const double ratio = q[index] / p[index];
    if (p[index] < 0.0) {
      first = std::max(first, ratio);
    } else {
      last = std::min(last, ratio);
    }
    if (first > last)
      return false;
  }
  const Vec2 original = start;
  start = {original.x + first * dx, original.y + first * dy};
  end = {original.x + last * dx, original.y + last * dy};
  return true;
}

struct Bounds3 {
  Vec3 min;
  Vec3 max;
  bool valid = false;
};

void ExtendBounds(Bounds3 &bounds, Vec3 point) {
  if (!bounds.valid) {
    bounds.min = point;
    bounds.max = point;
    bounds.valid = true;
    return;
  }
  bounds.min.x = std::min(bounds.min.x, point.x);
  bounds.min.y = std::min(bounds.min.y, point.y);
  bounds.min.z = std::min(bounds.min.z, point.z);
  bounds.max.x = std::max(bounds.max.x, point.x);
  bounds.max.y = std::max(bounds.max.y, point.y);
  bounds.max.z = std::max(bounds.max.z, point.z);
}

std::vector<FaceTessellation>
FacesForView(std::span<const FaceTessellation> faces,
             const CachedDrawingView &view,
             std::span<const CachedDrawingView> all_views) {
  const auto references_document = [](std::string_view reference,
                                      std::string_view document) {
    if (reference.size() < document.size() ||
        !std::equal(document.begin(), document.end(), reference.begin(),
                    [](char left, char right) {
                      const auto lower = [](char value) {
                        return value >= 'A' && value <= 'Z'
                                   ? static_cast<char>(value - 'A' + 'a')
                                   : value;
                      };
                      return lower(left) == lower(right);
                    })) {
      return false;
    }
    const std::string_view suffix = reference.substr(document.size());
    return suffix.empty() ||
           (suffix.size() > 1u && suffix.front() == '-' &&
            std::all_of(suffix.begin() + 1u, suffix.end(), [](char value) {
              return value >= '0' && value <= '9';
            }));
  };

  std::vector<FaceTessellation> native_owned;
  native_owned.reserve(faces.size());
  for (const FaceTessellation &face : faces) {
    if (face.owning_view_name == view.name)
      native_owned.push_back(face);
  }
  if (!native_owned.empty()) return native_owned;

  const bool has_document_ownership =
      std::any_of(faces.begin(), faces.end(), [](const FaceTessellation &face) {
        return !face.referenced_document.empty();
      });
  if (has_document_ownership) {
    std::vector<FaceTessellation> selected;
    selected.reserve(faces.size());
    for (const FaceTessellation &face : faces) {
      if (references_document(view.referenced_document,
                              face.referenced_document)) {
        selected.push_back(face);
      }
    }
    if (selected.empty())
      return selected;

    // One uoTempFaceTessData_c declaration can contain several complete
    // model-reference groups for different drawing views/configurations. The
    // document stem alone therefore does not establish ownership. Select the
    // group whose model-space envelope best matches this view's active edge
    // cache; retaining every same-document group mixes section/configuration
    // faces into shaded views.
    std::map<uint64_t, Bounds3> group_bounds;
    for (const FaceTessellation &face : selected) {
      for (const Vec3 vertex : face.vertices)
        ExtendBounds(group_bounds[face.source_group_offset], vertex);
    }
    if (group_bounds.size() > 1u) {
      Bounds3 view_bounds;
      for (const Line3D &line : view.source_lines) {
        ExtendBounds(view_bounds, line.start);
        ExtendBounds(view_bounds, line.end);
      }
      const auto mismatch = [&](const Bounds3 &bounds) {
        if (!bounds.valid || !view_bounds.valid)
          return std::numeric_limits<double>::infinity();
        return std::abs(bounds.min.x - view_bounds.min.x) +
               std::abs(bounds.min.y - view_bounds.min.y) +
               std::abs(bounds.min.z - view_bounds.min.z) +
               std::abs(bounds.max.x - view_bounds.max.x) +
               std::abs(bounds.max.y - view_bounds.max.y) +
               std::abs(bounds.max.z - view_bounds.max.z);
      };
      uint64_t best_group = group_bounds.begin()->first;
      double best_mismatch = std::numeric_limits<double>::infinity();
      for (const auto &[group, bounds] : group_bounds) {
        const double candidate = mismatch(bounds);
        if (candidate < best_mismatch) {
          best_mismatch = candidate;
          best_group = group;
        }
      }
      selected.erase(
          std::remove_if(selected.begin(), selected.end(),
                         [&](const FaceTessellation &face) {
                           return face.source_group_offset != best_group;
                         }),
          selected.end());
    }

    // A document may occur in several drawing views with different
    // configurations. VBLists stores the model path on the tessellation but
    // not the view's numeric instance suffix. Resolve that ambiguity by the
    // aggregate model-space envelope: views of the same configuration have
    // the same envelope apart from small omissions in the cached edge list.
    const std::string &document = selected.front().referenced_document;
    const std::size_t sibling_count = std::count_if(
        all_views.begin(), all_views.end(), [&](const CachedDrawingView &item) {
          return references_document(item.referenced_document, document);
        });
    if (sibling_count > 1u && selected.size() >= 16u) {
      Bounds3 face_bounds;
      for (const FaceTessellation &face : selected) {
        for (const Vec3 vertex : face.vertices)
          ExtendBounds(face_bounds, vertex);
      }
      const auto bounds_for_view = [](const CachedDrawingView &item) {
        Bounds3 bounds;
        for (const Line3D &line : item.source_lines) {
          ExtendBounds(bounds, line.start);
          ExtendBounds(bounds, line.end);
        }
        return bounds;
      };
      const auto mismatch = [&](const Bounds3 &bounds) {
        if (!bounds.valid || !face_bounds.valid) {
          return std::numeric_limits<double>::infinity();
        }
        return std::abs(bounds.min.x - face_bounds.min.x) +
               std::abs(bounds.min.y - face_bounds.min.y) +
               std::abs(bounds.min.z - face_bounds.min.z) +
               std::abs(bounds.max.x - face_bounds.max.x) +
               std::abs(bounds.max.y - face_bounds.max.y) +
               std::abs(bounds.max.z - face_bounds.max.z);
      };
      double best_mismatch = std::numeric_limits<double>::infinity();
      for (const CachedDrawingView &item : all_views) {
        if (references_document(item.referenced_document, document)) {
          best_mismatch =
              std::min(best_mismatch, mismatch(bounds_for_view(item)));
        }
      }
      const double extent = std::max({face_bounds.max.x - face_bounds.min.x,
                                      face_bounds.max.y - face_bounds.min.y,
                                      face_bounds.max.z - face_bounds.min.z});
      const double configuration_tolerance = std::max(1e-6, extent * 0.005);
      if (mismatch(bounds_for_view(view)) >
          best_mismatch + configuration_tolerance) {
        // A small tail of alternate/cache coordinates can stretch the raw
        // edge envelope far beyond the actual model (item 8556/List2 has 28
        // such lines among 3683). Do not discard an otherwise matching face
        // group when it contains the overwhelming majority of complete
        // source edges. This keeps the configuration guard for genuinely
        // different smaller views while allowing HLR to classify the main
        // drawing view.
        const auto inside_face_bounds = [&](Vec3 point) {
          return point.x >= face_bounds.min.x - configuration_tolerance &&
                 point.x <= face_bounds.max.x + configuration_tolerance &&
                 point.y >= face_bounds.min.y - configuration_tolerance &&
                 point.y <= face_bounds.max.y + configuration_tolerance &&
                 point.z >= face_bounds.min.z - configuration_tolerance &&
                 point.z <= face_bounds.max.z + configuration_tolerance;
        };
        const std::size_t contained_edges = std::count_if(
            view.source_lines.begin(), view.source_lines.end(),
            [&](const Line3D &line) {
              return inside_face_bounds(line.start) &&
                     inside_face_bounds(line.end);
            });
        constexpr double kDominantEdgeCoverage = 0.98;
        Bounds3 contained_bounds;
        for (const Line3D &line : view.source_lines) {
          if (inside_face_bounds(line.start) &&
              inside_face_bounds(line.end)) {
            ExtendBounds(contained_bounds, line.start);
            ExtendBounds(contained_bounds, line.end);
          }
        }
        constexpr double kMaximumContainedBoundsMismatch = 3.5;
        if (static_cast<double>(contained_edges) <
            static_cast<double>(view.source_lines.size()) *
                kDominantEdgeCoverage ||
            mismatch(contained_bounds) >
                configuration_tolerance * kMaximumContainedBoundsMismatch) {
          return {};
        }
      }
    }
    return selected;
  }

  Bounds3 view_bounds;
  for (const Line3D &line : view.source_lines) {
    ExtendBounds(view_bounds, line.start);
    ExtendBounds(view_bounds, line.end);
  }
  if (!view_bounds.valid)
    return {};

  const double extent = std::max({view_bounds.max.x - view_bounds.min.x,
                                  view_bounds.max.y - view_bounds.min.y,
                                  view_bounds.max.z - view_bounds.min.z});
  const double tolerance = std::max(1e-9, extent * 1e-6);
  const auto overlaps = [&](double face_min, double face_max, double view_min,
                            double view_max) {
    return face_max + tolerance >= view_min && face_min - tolerance <= view_max;
  };

  std::vector<FaceTessellation> selected;
  selected.reserve(faces.size());
  for (const FaceTessellation &face : faces) {
    Bounds3 face_bounds;
    for (const Vec3 &vertex : face.vertices) {
      ExtendBounds(face_bounds, vertex);
    }
    if (!face_bounds.valid ||
        !overlaps(face_bounds.min.x, face_bounds.max.x, view_bounds.min.x,
                  view_bounds.max.x) ||
        !overlaps(face_bounds.min.y, face_bounds.max.y, view_bounds.min.y,
                  view_bounds.max.y) ||
        !overlaps(face_bounds.min.z, face_bounds.max.z, view_bounds.min.z,
                  view_bounds.max.z)) {
      continue;
    }
    selected.push_back(face);
  }
  return selected;
}

struct KeywordViews {
  std::string sheet_name;
  std::vector<std::string> sheet_names;
  std::set<std::string> all;
  std::vector<std::string> selected;
};

std::optional<KeywordViews> ParseKeywordViews(std::span<const uint8_t> xml,
                                              std::string_view sheet_name) {
  const auto start = std::find(xml.begin(), xml.end(), uint8_t{'<'});
  if (start == xml.end())
    return std::nullopt;
  pugi::xml_document document;
  if (!document.load_buffer(&*start,
                            static_cast<std::size_t>(xml.end() - start))) {
    return std::nullopt;
  }
  KeywordViews result;
  bool found_sheet = false;
  for (const pugi::xml_node sheet :
       document.document_element().children("Sheet")) {
    // A drawing template or empty drawing has a real sheet but no model view.
    // SolidWorks also serializes a separate sheet-format node; keep excluding
    // that node while accepting Type="List" sheets with zero View children.
    const std::string_view type = sheet.attribute("Type").as_string();
    const bool real_sheet =
        type == "List" || type == "Sheet" || sheet.child("View");
    if (!real_sheet)
      continue;
    const std::string current_name = sheet.attribute("Name").as_string();
    result.sheet_names.push_back(current_name);
    const bool selected =
        current_name == sheet_name || (sheet_name.empty() && !found_sheet);
    found_sheet = found_sheet || selected;
    if (selected)
      result.sheet_name = current_name;
    for (const pugi::xml_node view : sheet.children("View")) {
      const std::string name = view.attribute("Name").as_string();
      if (name.empty())
        continue;
      result.all.insert(name);
      if (selected)
        result.selected.push_back(name);
    }
  }
  if (!found_sheet)
    return std::nullopt;
  return result;
}

std::pair<double, double>
FindStandardSheetSize(std::span<const uint8_t> definition) {
  static constexpr std::array<std::pair<double, double>, 5> kLandscapeSizes = {
      {{1.189, 0.841},
       {0.841, 0.594},
       {0.594, 0.420},
       {0.420, 0.297},
       {0.297, 0.210}}};
  if (definition.size() < 16u)
    return {0.0, 0.0};
  for (std::size_t offset = 0; offset + 16u <= definition.size(); ++offset) {
    const double first = ReadF64Le(definition, offset);
    const double second = ReadF64Le(definition, offset + 8u);
    if (!std::isfinite(first) || !std::isfinite(second))
      continue;
    for (const auto [width, height] : kLandscapeSizes) {
      if ((std::abs(first - width) < 1e-9 &&
           std::abs(second - height) < 1e-9) ||
          (std::abs(first - height) < 1e-9 &&
           std::abs(second - width) < 1e-9)) {
        return {width, height};
      }
    }
  }
  return {0.0, 0.0};
}

} // namespace

std::vector<std::array<Vec2, 3>>
DrawingGeometryReader::ClipTriangleToRect(const std::array<Vec2, 3> &triangle,
                                          const Rect2 &bounds) {
  if (bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y)
    return {};
  std::vector<Vec2> polygon(triangle.begin(), triangle.end());
  const auto clip_edge = [&](bool x_axis, double boundary, bool keep_less) {
    if (polygon.empty())
      return;
    std::vector<Vec2> output;
    output.reserve(polygon.size() + 1u);
    const auto coordinate = [&](const Vec2 &point) {
      return x_axis ? point.x : point.y;
    };
    const auto inside = [&](const Vec2 &point) {
      return keep_less ? coordinate(point) <= boundary
                       : coordinate(point) >= boundary;
    };
    Vec2 previous = polygon.back();
    bool previous_inside = inside(previous);
    for (const Vec2 current : polygon) {
      const bool current_inside = inside(current);
      if (current_inside != previous_inside) {
        const double previous_coordinate = coordinate(previous);
        const double difference = coordinate(current) - previous_coordinate;
        if (std::abs(difference) > 1e-15) {
          const double fraction = (boundary - previous_coordinate) / difference;
          output.push_back(
              Vec2{previous.x + (current.x - previous.x) * fraction,
                   previous.y + (current.y - previous.y) * fraction});
        }
      }
      if (current_inside)
        output.push_back(current);
      previous = current;
      previous_inside = current_inside;
    }
    polygon = std::move(output);
  };
  clip_edge(true, bounds.min.x, false);
  clip_edge(true, bounds.max.x, true);
  clip_edge(false, bounds.min.y, false);
  clip_edge(false, bounds.max.y, true);

  std::vector<std::array<Vec2, 3>> result;
  if (polygon.size() < 3u)
    return result;
  result.reserve(polygon.size() - 2u);
  for (std::size_t index = 1u; index + 1u < polygon.size(); ++index) {
    const std::array<Vec2, 3> clipped = {polygon.front(), polygon[index],
                                         polygon[index + 1u]};
    const double area =
        (clipped[1].x - clipped[0].x) * (clipped[2].y - clipped[0].y) -
        (clipped[1].y - clipped[0].y) * (clipped[2].x - clipped[0].x);
    if (std::abs(area) >= 1e-14)
      result.push_back(clipped);
  }
  return result;
}

Result<std::vector<DrawingViewPlacement>>
DrawingGeometryReader::ReadViewPlacements(
    std::span<const uint8_t> definition,
    std::span<const std::string> view_names) {
  const std::set<std::string> known_names(view_names.begin(), view_names.end());
  const std::vector<NamedOffset> names =
      FindKnownViewNames(definition, known_names);
  if (names.empty()) {
    return Result<std::vector<DrawingViewPlacement>>::Err(
        "No requested drawing view object names found in Definition");
  }

  std::vector<DrawingViewPlacement> placements;
  std::set<std::string> seen;
  for (std::size_t offset = kMatrixBeforePlacement;
       offset + 88u <= definition.size(); ++offset) {
    std::array<double, 11> values{};
    if (!IsPlacementRecord(definition, offset, values))
      continue;
    const auto name =
        std::upper_bound(names.begin(), names.end(), offset,
                         [](std::size_t value, const NamedOffset &item) {
                           return value < item.offset;
                         });
    if (name == names.begin())
      continue;
    const NamedOffset &owner = *std::prev(name);
    if (offset - owner.offset > 100000u || seen.contains(owner.name))
      continue;

    const std::size_t matrix_offset = offset - kMatrixBeforePlacement;
    DrawingViewPlacement placement;
    placement.source_offset = offset;
    placement.name = owner.name;
    for (std::size_t index = 0; index < placement.model_to_view.size();
         ++index) {
      placement.model_to_view[index] =
          ReadF64Le(definition, matrix_offset + index * 8u);
    }
    placement.translation = {ReadF64Le(definition, matrix_offset + 72u),
                             ReadF64Le(definition, matrix_offset + 80u),
                             ReadF64Le(definition, matrix_offset + 88u)};
    placement.scale = ReadF64Le(definition, matrix_offset + 96u);
    if (!ValidMatrix(placement.model_to_view) ||
        !std::isfinite(placement.scale) || placement.scale <= 0.0 ||
        placement.scale > 1000.0) {
      continue;
    }
    placement.outer_bounds = {{values[0], values[1]}, {values[2], values[3]}};
    placement.clip_bounds = {{values[4], values[5]}, {values[6], values[7]}};
    placement.center = {values[8], values[9]};
    seen.insert(placement.name);
    placements.push_back(std::move(placement));
    offset += 87u;
  }
  if (placements.empty()) {
    return Result<std::vector<DrawingViewPlacement>>::Err(
        "No confirmed drawing view placement records found in Definition");
  }
  return Result<std::vector<DrawingViewPlacement>>::Ok(std::move(placements));
}

Result<std::array<double, 8>> DrawingGeometryReader::ReadPrintLineWeights(
    std::span<const uint8_t> printing) {
  constexpr std::string_view kPrintDataClass = "uoPrintMainData_c";
  constexpr std::size_t kWeightsAfterPrinterName = 52u;
  const auto class_position =
      std::search(printing.begin(), printing.end(), kPrintDataClass.begin(),
                  kPrintDataClass.end());
  if (class_position == printing.end()) {
    return Result<std::array<double, 8>>::Err(
        "Printing does not declare uoPrintMainData_c");
  }
  const std::size_t printer_marker = static_cast<std::size_t>(
      std::distance(printing.begin(), class_position)) +
                                     kPrintDataClass.size();
  if (printer_marker + 4u > printing.size() ||
      !std::equal(
          kUtf16Marker.begin(), kUtf16Marker.end(),
          printing.begin() + static_cast<std::ptrdiff_t>(printer_marker))) {
    return Result<std::array<double, 8>>::Err(
        "Printing printer-name field is malformed");
  }
  const std::size_t printer_units = printing[printer_marker + 3u];
  const std::size_t printer_end = printer_marker + 4u + printer_units * 2u;
  const std::size_t weights_offset = printer_end + kWeightsAfterPrinterName;
  if (printer_units > 127u || weights_offset > printing.size() ||
      printing.size() - weights_offset < 8u * sizeof(double)) {
    return Result<std::array<double, 8>>::Err(
        "Printing line-weight table is truncated");
  }
  std::array<double, 8> weights{};
  for (std::size_t index = 0u; index < weights.size(); ++index) {
    weights[index] = ReadF64Le(printing, weights_offset + index * sizeof(double));
    if (!std::isfinite(weights[index]) || weights[index] < 1e-5 ||
        weights[index] > 0.01 ||
        (index > 0u && weights[index] <= weights[index - 1u])) {
      return Result<std::array<double, 8>>::Err(
          "Printing line-weight table is invalid");
    }
  }
  return Result<std::array<double, 8>>::Ok(weights);
}

Result<DrawingSheetGeometry> DrawingGeometryReader::ReadCachedSheet(
    std::span<const uint8_t> keywords, std::span<const uint8_t> definition,
    std::span<const uint8_t> vb_lists, std::span<const uint8_t> tables,
    std::string_view sheet_name, std::span<const uint8_t> display_lists,
    std::span<const uint8_t> sketch_bitmap,
    std::span<const uint8_t> printing) {
  const auto keyword_views = ParseKeywordViews(keywords, sheet_name);
  if (!keyword_views) {
    return Result<DrawingSheetGeometry>::Err(
        "Requested sheet was not found in KeyWords XML");
  }
  const std::vector<std::string> all_names(keyword_views->all.begin(),
                                           keyword_views->all.end());
  std::vector<std::string> section_assembly_names = FindSectionAssemblyNames(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  const auto sheet_object_range = FindDrawingSheetRange(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  std::vector<DrawingViewPlacement> placements;
  std::vector<CachedDrawingView> buckets;
  std::vector<FaceTessellation> face_tessellations;
  std::vector<DrawingAppearance> appearances;
  std::vector<DrawingSceneLight> scene_lights;
  std::multiset<std::string> display_arrowhead_object_ids;
  if (!all_names.empty()) {
    auto parsed_buckets = ReadViewBuckets(vb_lists);
    if (!parsed_buckets.ok()) {
      return Result<DrawingSheetGeometry>::Err(parsed_buckets.error());
    }
    buckets = std::move(parsed_buckets).value();
    // A sheet object can contain names of nested or referenced section
    // assemblies. Only a name backed by an actual uiViewBucket_c is a
    // drawing-view cache owner. Keeping an unrelated nested name can make it
    // steal the following placement merely because it is serialized closer.
    std::erase_if(section_assembly_names, [&](const std::string &name) {
      return std::none_of(
          buckets.begin(), buckets.end(),
          [&](const CachedDrawingView &bucket) { return bucket.name == name; });
    });
    std::vector<std::string> placement_names = all_names;
    placement_names.insert(placement_names.end(),
                           section_assembly_names.begin(),
                           section_assembly_names.end());
    auto parsed_placements = ReadViewPlacements(definition, placement_names);
    if (!parsed_placements.ok()) {
      return Result<DrawingSheetGeometry>::Err(parsed_placements.error());
    }
    placements = std::move(parsed_placements).value();
    auto parsed_faces = ReadFaceTessellations(vb_lists);
    if (parsed_faces.ok()) {
      face_tessellations = std::move(parsed_faces).value();
    }
    appearances = ReadAppearances(vb_lists);
    scene_lights = ReadSceneLights(vb_lists);
  }

  DrawingSheetGeometry sheet;
  sheet.name = keyword_views->sheet_name;
  const auto [width, height] = FindStandardSheetSize(definition);
  sheet.width_m = width;
  sheet.height_m = height;
  if (!printing.empty()) {
    auto parsed_line_weights = ReadPrintLineWeights(printing);
    if (parsed_line_weights.ok()) {
      sheet.line_weights_m = std::move(parsed_line_weights).value();
    }
  }
  const int visible_line_weight =
      FindConfiguredLineWeight(definition, "Visible").value_or(2);
  const int hidden_line_weight =
      FindConfiguredLineWeight(definition, "Hidden").value_or(0);
  const int tangent_line_weight =
      FindConfiguredLineWeight(definition, "TanVisible").value_or(0);
  if (!sketch_bitmap.empty()) {
    auto parsed_bitmap = ReadSketchBitmap(definition, sketch_bitmap);
    if (parsed_bitmap.ok()) {
      std::vector<RasterImage2D> images = std::move(parsed_bitmap).value();
      for (auto image = images.rbegin(); image != images.rend(); ++image) {
        sheet.entities.insert(sheet.entities.begin(), std::move(*image));
      }
    }
  }
  const std::set<std::string> selected(keyword_views->selected.begin(),
                                       keyword_views->selected.end());
  std::set<std::string> selected_placement_names(selected.begin(),
                                                 selected.end());
  selected_placement_names.insert(section_assembly_names.begin(),
                                  section_assembly_names.end());
  std::vector<TextAnnotation> display_text;
  std::vector<uint32_t> display_object_ids;
  auto parsed_object_ids = ReadDisplayObjectIds(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  if (parsed_object_ids.ok()) {
    display_object_ids = std::move(parsed_object_ids).value();
  }
  for (const DrawingViewPlacement &placement : placements) {
    if (sheet_object_range &&
        (placement.source_offset < sheet_object_range->first ||
         placement.source_offset >= sheet_object_range->second)) {
      continue;
    }
    if (!selected_placement_names.contains(placement.name))
      continue;
    auto bucket = std::find_if(buckets.begin(), buckets.end(),
                               [&](const CachedDrawingView &item) {
                                 return item.name == placement.name;
                               });
    // Array order is not a relationship: omitted, suppressed, or section
    // views make the placement and cache arrays diverge. Emit a bucket only
    // when its serialized reference equals the owning placement name.
    if (bucket == buckets.end() || bucket->source_lines.empty())
      continue;
    const std::vector<FaceTessellation> view_faces =
        FacesForView(face_tessellations, *bucket, buckets);
    uint64_t native_face_group_offset = 0;
    std::string native_face_component;
    for (const FaceTessellation &face : face_tessellations) {
      if (face.owning_view_name != placement.name) continue;
      if (native_face_group_offset == 0) {
        native_face_group_offset = face.source_group_offset;
        native_face_component = face.owning_component_name;
      } else if (face.source_group_offset != native_face_group_offset ||
                 face.owning_component_name != native_face_component) {
        native_face_group_offset = 0;
        native_face_component.clear();
        break;
      }
    }
    uint64_t face_group_offset = 0;
    if (!view_faces.empty()) {
      face_group_offset = view_faces.front().source_group_offset;
      if (std::any_of(view_faces.begin(), view_faces.end(),
                      [&](const FaceTessellation &face) {
                        return face.source_group_offset != face_group_offset;
                      })) {
        face_group_offset = 0;
      }
    }
    const bool face_group_native =
        face_group_offset != 0 &&
        face_group_offset == native_face_group_offset;
    const bool is_section_view =
        placement.name.find("-SectionAssembly-") != std::string::npos;
    const bool shaded_hlr_edges = bucket->has_suppressed_edge_cache &&
                                  !view_faces.empty() && !is_section_view;
    std::set<uint64_t> face_body_groups;
    for (const FaceTessellation &face : view_faces) {
      if (face.source_body_group_offset != 0)
        face_body_groups.insert(face.source_body_group_offset);
    }
    const uint32_t view_ownership_index =
        static_cast<uint32_t>(sheet.view_ownership.size());
    sheet.view_ownership.push_back(
        DrawingViewOwnership{placement.name,
                             bucket->referenced_document,
                             bucket->source_offset,
                             placement.source_offset,
                             native_face_group_offset,
                             face_group_offset,
                             face_group_native,
                             face_group_offset != 0 && !face_group_native,
                             is_section_view ? DrawingViewKind::kSection
                                             : DrawingViewKind::kStandard,
                             std::move(native_face_component),
                             shaded_hlr_edges,
                             {face_body_groups.begin(), face_body_groups.end()}});
    // A bucket containing both the suppressed edge cache and owned face
    // tessellation is a shaded drawing view. Its active edges use HLR: hidden
    // spans are removed instead of being drawn with the hidden-line font.
    const std::size_t view_entity_begin = sheet.entities.size();

    // Definition stores the sheet-space translation of the projected model
    // origin. Do not recenter the cached-edge bounding box: asymmetric views
    // (notably isometric views) then move away from their SolidWorks position
    // and may overlap tables even though the scale and matrix are correct.
    auto project = [&](Vec3 point) {
      return Vec2{
          placement.scale * ColumnDot(placement.model_to_view, 0u, point) +
              placement.translation.x,
          placement.scale * ColumnDot(placement.model_to_view, 1u, point) +
              placement.translation.y};
    };
    if (shaded_hlr_edges) {
      Rect2 shaded_bounds = placement.clip_bounds;
      struct ShadedTriangle {
        std::array<Vec2, 3> points;
        std::array<double, 3> depths;
        uint32_t rgb = 0;
        double opacity = 1.0;
      };
      std::vector<ShadedTriangle> projected_faces;
      Rect2 projected_face_bounds;
      bool has_projected_face_bounds = false;
      for (const FaceTessellation &face : view_faces) {
        for (const std::array<uint32_t, 3> &indices : face.triangles) {
          if (indices[0] >= face.vertices.size() ||
              indices[1] >= face.vertices.size() ||
              indices[2] >= face.vertices.size()) {
            continue;
          }
          const Vec3 a = face.vertices[indices[0]];
          const Vec3 b = face.vertices[indices[1]];
          const Vec3 c = face.vertices[indices[2]];
          const std::array<Vec2, 3> points = {project(a), project(b),
                                              project(c)};
          const double signed_area =
              (points[1].x - points[0].x) * (points[2].y - points[0].y) -
              (points[1].y - points[0].y) * (points[2].x - points[0].x);
          if (std::abs(signed_area) < 1e-14)
            continue;
          const auto clipped_triangles =
              ClipTriangleToRect(points, placement.clip_bounds);
          if (clipped_triangles.empty())
            continue;
          for (const auto &clipped : clipped_triangles) {
            for (const Vec2 point : clipped) {
              if (!has_projected_face_bounds) {
                projected_face_bounds = {point, point};
                has_projected_face_bounds = true;
              }
              projected_face_bounds.min.x =
                  std::min(projected_face_bounds.min.x, point.x);
              projected_face_bounds.min.y =
                  std::min(projected_face_bounds.min.y, point.y);
              projected_face_bounds.max.x =
                  std::max(projected_face_bounds.max.x, point.x);
              projected_face_bounds.max.y =
                  std::max(projected_face_bounds.max.y, point.y);
            }
          }

          const Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
          const Vec3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
          const Vec3 normal{ab.y * ac.z - ab.z * ac.y,
                            ab.z * ac.x - ab.x * ac.z,
                            ab.x * ac.y - ab.y * ac.x};
          const double normal_length = std::sqrt(
              normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
          const uint32_t base_rgb =
              appearances.size() == 1u ? appearances.front().rgb : 0xD9D9D9u;
          std::array<double, 3> illumination = {0.0, 0.0, 0.0};
          bool used_scene_lighting = false;
          if (normal_length > 0.0) {
            Vec3 view_normal{
                ColumnDot(placement.model_to_view, 0u, normal) / normal_length,
                ColumnDot(placement.model_to_view, 1u, normal) / normal_length,
                ColumnDot(placement.model_to_view, 2u, normal) / normal_length};
            // SolidWorks' shaded cache contains closed-body back faces as well
            // as the sheet-facing surface. SVG has no depth buffer, so drawing
            // both makes rear panels bleed through the model even after an
            // average-depth sort. Cull triangles facing away from the sheet,
            // matching the back-face stage of the native 3D renderer.
            if (view_normal.z < 0.0)
              continue;
            const auto add_light = [&](const DrawingSceneLight &light,
                                       double factor) {
              for (uint32_t channel = 0u; channel < 3u; ++channel) {
                const uint32_t shift = (2u - channel) * 8u;
                const double colour =
                    static_cast<double>((light.rgb >> shift) & 0xFFu) / 255.0;
                illumination[channel] += factor * colour;
              }
            };
            // RGB alone does not define how a material responds to the cached
            // scene lights. Until those coefficients are decoded, retain the
            // neutral facing fallback; otherwise valid named appearances such
            // as item 8556's aluminium become much too dark.
            if (appearances.size() == 1u &&
                appearances.front().has_material_response) {
              for (const DrawingSceneLight &light : scene_lights) {
                if (!light.enabled)
                  continue;
                used_scene_lighting = true;
                if (light.kind == DrawingLightKind::kAmbient) {
                  add_light(light, light.ambient);
                  continue;
                }
                const double direction_length =
                    std::sqrt(light.direction.x * light.direction.x +
                              light.direction.y * light.direction.y +
                              light.direction.z * light.direction.z);
                if (direction_length <= 0.0)
                  continue;
                const double lambert =
                    std::max(0.0, (view_normal.x * light.direction.x +
                                   view_normal.y * light.direction.y +
                                   view_normal.z * light.direction.z) /
                                      direction_length);
                add_light(light, light.ambient + light.diffuse * lambert);
              }
            }
          }
          if (!used_scene_lighting) {
            const double facing =
                normal_length > 0.0
                    ? std::abs(ColumnDot(placement.model_to_view, 2u, normal)) /
                          normal_length
                    : 0.0;
            illumination.fill(0.88 + 0.28 * facing);
          }
          const auto shade_channel = [&](uint32_t shift) {
            const std::size_t channel = 2u - shift / 8u;
            return static_cast<uint32_t>(std::clamp(
                std::round(static_cast<double>((base_rgb >> shift) & 0xFFu) *
                           std::clamp(illumination[channel], 0.0, 1.35)),
                0.0, 255.0));
          };
          const uint32_t rgb = (shade_channel(16u) << 16u) |
                               (shade_channel(8u) << 8u) | shade_channel(0u);
          projected_faces.push_back(ShadedTriangle{
              points,
              {ColumnDot(placement.model_to_view, 2u, a),
               ColumnDot(placement.model_to_view, 2u, b),
               ColumnDot(placement.model_to_view, 2u, c)},
              rgb,
              appearances.size() == 1u ? appearances.front().opacity : 1.0});
        }
      }

      // Face caches can include body-local geometry whose component transform
      // has not been decoded yet. The authoritative active edge cache still
      // gives us the visible silhouette. Select the edge cluster intersecting
      // the projected face region and constrain shaded pixels to its bounds;
      // the Definition clip rectangle is intentionally much looser.
      if (has_projected_face_bounds) {
        Rect2 edge_bounds;
        bool has_edge_bounds = false;
        for (const Line3D &source : bucket->source_lines) {
          Vec2 start = project(source.start);
          Vec2 end = project(source.end);
          if (!ClipLine(projected_face_bounds, start, end))
            continue;
          if (!has_edge_bounds) {
            edge_bounds = {start, start};
            has_edge_bounds = true;
          }
          edge_bounds.min.x = std::min({edge_bounds.min.x, start.x, end.x});
          edge_bounds.min.y = std::min({edge_bounds.min.y, start.y, end.y});
          edge_bounds.max.x = std::max({edge_bounds.max.x, start.x, end.x});
          edge_bounds.max.y = std::max({edge_bounds.max.y, start.y, end.y});
        }
        if (has_edge_bounds) {
          constexpr double kSilhouettePaddingM = 5e-4;
          shaded_bounds.min.x =
              std::max(placement.clip_bounds.min.x,
                       edge_bounds.min.x - kSilhouettePaddingM);
          shaded_bounds.min.y =
              std::max(placement.clip_bounds.min.y,
                       edge_bounds.min.y - kSilhouettePaddingM);
          shaded_bounds.max.x =
              std::min(placement.clip_bounds.max.x,
                       edge_bounds.max.x + kSilhouettePaddingM);
          shaded_bounds.max.y =
              std::min(placement.clip_bounds.max.y,
                       edge_bounds.max.y + kSilhouettePaddingM);
        }
      }

      // SVG has no per-pixel depth buffer. Average-depth painter sorting leaks
      // large rear faces through intersecting assembly parts, so rasterise only
      // the shaded fill into one transparent image. Cached edges, annotations
      // and dimensions remain vector entities drawn above this layer.
      const double raster_width_m = shaded_bounds.max.x - shaded_bounds.min.x;
      const double raster_height_m = shaded_bounds.max.y - shaded_bounds.min.y;
      if (!projected_faces.empty() && raster_width_m > 0.0 &&
          raster_height_m > 0.0) {
        constexpr double kPixelsPerMetre = 4000.0;
        constexpr uint32_t kMaximumRasterSide = 2048u;
        const uint32_t pixel_width = std::clamp(
            static_cast<uint32_t>(std::ceil(raster_width_m * kPixelsPerMetre)),
            1u, kMaximumRasterSide);
        const uint32_t pixel_height = std::clamp(
            static_cast<uint32_t>(std::ceil(raster_height_m * kPixelsPerMetre)),
            1u, kMaximumRasterSide);
        const std::size_t pixel_count =
            static_cast<std::size_t>(pixel_width) * pixel_height;
        std::vector<double> depth_buffer(
            pixel_count, -std::numeric_limits<double>::infinity());
        std::vector<uint8_t> rgba(pixel_count * 4u, 0u);
        bool wrote_pixel = false;
        const auto to_pixel = [&](Vec2 point) {
          return Vec2{
              (point.x - shaded_bounds.min.x) / raster_width_m * pixel_width,
              (shaded_bounds.max.y - point.y) / raster_height_m * pixel_height};
        };
        for (const ShadedTriangle &triangle : projected_faces) {
          const std::array<Vec2, 3> pixels = {to_pixel(triangle.points[0]),
                                              to_pixel(triangle.points[1]),
                                              to_pixel(triangle.points[2])};
          const double denominator =
              (pixels[1].y - pixels[2].y) * (pixels[0].x - pixels[2].x) +
              (pixels[2].x - pixels[1].x) * (pixels[0].y - pixels[2].y);
          if (std::abs(denominator) < 1e-12)
            continue;
          const int min_x =
              std::max(0, static_cast<int>(std::floor(std::min(
                              {pixels[0].x, pixels[1].x, pixels[2].x}))));
          const int max_x =
              std::min(static_cast<int>(pixel_width) - 1,
                       static_cast<int>(std::ceil(
                           std::max({pixels[0].x, pixels[1].x, pixels[2].x}))));
          const int min_y =
              std::max(0, static_cast<int>(std::floor(std::min(
                              {pixels[0].y, pixels[1].y, pixels[2].y}))));
          const int max_y =
              std::min(static_cast<int>(pixel_height) - 1,
                       static_cast<int>(std::ceil(
                           std::max({pixels[0].y, pixels[1].y, pixels[2].y}))));
          for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
              const double sample_x = static_cast<double>(x) + 0.5;
              const double sample_y = static_cast<double>(y) + 0.5;
              const double weight0 =
                  ((pixels[1].y - pixels[2].y) * (sample_x - pixels[2].x) +
                   (pixels[2].x - pixels[1].x) * (sample_y - pixels[2].y)) /
                  denominator;
              const double weight1 =
                  ((pixels[2].y - pixels[0].y) * (sample_x - pixels[2].x) +
                   (pixels[0].x - pixels[2].x) * (sample_y - pixels[2].y)) /
                  denominator;
              const double weight2 = 1.0 - weight0 - weight1;
              if (weight0 < -1e-9 || weight1 < -1e-9 || weight2 < -1e-9) {
                continue;
              }
              const double depth = weight0 * triangle.depths[0] +
                                   weight1 * triangle.depths[1] +
                                   weight2 * triangle.depths[2];
              const std::size_t pixel =
                  static_cast<std::size_t>(y) * pixel_width +
                  static_cast<std::size_t>(x);
              if (depth <= depth_buffer[pixel])
                continue;
              depth_buffer[pixel] = depth;
              const std::size_t rgba_offset = pixel * 4u;
              rgba[rgba_offset] =
                  static_cast<uint8_t>((triangle.rgb >> 16u) & 0xFFu);
              rgba[rgba_offset + 1u] =
                  static_cast<uint8_t>((triangle.rgb >> 8u) & 0xFFu);
              rgba[rgba_offset + 2u] =
                  static_cast<uint8_t>(triangle.rgb & 0xFFu);
              rgba[rgba_offset + 3u] = static_cast<uint8_t>(
                  std::clamp(std::round(triangle.opacity * 255.0), 0.0, 255.0));
              wrote_pixel = true;
            }
          }
        }
        if (wrote_pixel) {
          sheet.entities.emplace_back(
              RasterImage2D{shaded_bounds.min, raster_width_m, raster_height_m,
                            pixel_width, pixel_height, std::move(rgba),
                            "Contents/VBLists/uoTempFaceTessData_c/z-buffer/" +
                                placement.name});
        }
      }
    }
    // A visible tangent transition can occur once on each side of a thin
    // member. In a normal projection the two model edges collapse to the same
    // short paper-space segment. SolidWorks emits that connector once with
    // the TanVisible line font; emitting both cached 0x23 records makes it a
    // heavy solid stroke. Keep this deliberately conservative: long
    // coincident outlines are ordinary silhouette geometry.
    constexpr double kMinimumVisibilitySpanM = 1e-5;
    constexpr double kCoincidentEdgeToleranceM = 1e-8;
    constexpr double kMaximumTangentConnectorLengthM = 5e-4;
    std::map<std::array<int64_t, 4>, std::vector<std::size_t>>
        short_coincident_edges;
    const double largest_direction_component =
        std::max({std::abs(bucket->direction.x),
                  std::abs(bucket->direction.y),
                  std::abs(bucket->direction.z)});
    if (largest_direction_component < 0.999) {
      const auto quantize = [](double value) {
        return static_cast<int64_t>(
            std::llround(value / kCoincidentEdgeToleranceM));
      };
      for (std::size_t source_index = 0u;
           source_index < bucket->source_lines.size(); ++source_index) {
        const Line3D &source = bucket->source_lines[source_index];
        if (source.source_record_type != 0x23u)
          continue;
        Vec2 start = project(source.start);
        Vec2 end = project(source.end);
        const double length = std::hypot(end.x - start.x, end.y - start.y);
        if (length < kMinimumVisibilitySpanM ||
            length > kMaximumTangentConnectorLengthM) {
          continue;
        }
        std::array<int64_t, 2> start_key = {quantize(start.x),
                                             quantize(start.y)};
        std::array<int64_t, 2> end_key = {quantize(end.x), quantize(end.y)};
        if (end_key < start_key)
          std::swap(start_key, end_key);
        short_coincident_edges[{start_key[0], start_key[1], end_key[0],
                                end_key[1]}]
            .push_back(source_index);
      }
    }
    std::set<std::size_t> tangent_edge_representatives;
    std::set<std::size_t> duplicate_tangent_edges;
    for (const auto &[key, indices] : short_coincident_edges) {
      (void)key;
      if (indices.size() != 2u)
        continue;
      tangent_edge_representatives.insert(indices.front());
      duplicate_tangent_edges.insert(indices.back());
    }

    // Face tables expose native uoBodyPropInfo_c groups, while cached edges do
    // not carry a direct body token. Recover a conservative edge-to-body link
    // from the cached edge topology and matching 3D body envelopes:
    // edges whose 3D endpoints join belong to one closed member. A projected
    // convex-boundary edge of such a member is its local silhouette. Preserve
    // it when the unowned face pool incorrectly hides the complete edge.
    std::vector<std::size_t> edge_parents(bucket->source_lines.size());
    for (std::size_t index = 0u; index < edge_parents.size(); ++index)
      edge_parents[index] = index;
    const auto find_parent = [&](std::size_t index) {
      while (edge_parents[index] != index) {
        edge_parents[index] = edge_parents[edge_parents[index]];
        index = edge_parents[index];
      }
      return index;
    };
    const auto join_edges = [&](std::size_t left, std::size_t right) {
      const std::size_t left_root = find_parent(left);
      const std::size_t right_root = find_parent(right);
      if (left_root != right_root)
        edge_parents[right_root] = left_root;
    };
    constexpr double kTopologyEndpointToleranceM = 1e-7;
    const auto endpoint_key = [](Vec3 point) {
      return std::array<int64_t, 3>{
          static_cast<int64_t>(
              std::llround(point.x / kTopologyEndpointToleranceM)),
          static_cast<int64_t>(
              std::llround(point.y / kTopologyEndpointToleranceM)),
          static_cast<int64_t>(
              std::llround(point.z / kTopologyEndpointToleranceM))};
    };
    std::map<std::array<int64_t, 3>, std::size_t> endpoint_owner;
    for (std::size_t index = 0u; index < bucket->source_lines.size(); ++index) {
      const Line3D &line = bucket->source_lines[index];
      for (const Vec3 endpoint : {line.start, line.end}) {
        const auto [iterator, inserted] =
            endpoint_owner.emplace(endpoint_key(endpoint), index);
        if (!inserted)
          join_edges(index, iterator->second);
      }
    }
    std::map<std::size_t, std::vector<std::size_t>> connected_edges;
    for (std::size_t index = 0u; index < bucket->source_lines.size(); ++index)
      connected_edges[find_parent(index)].push_back(index);
    std::map<uint64_t, Bounds3> face_body_bounds;
    for (const FaceTessellation &face : view_faces) {
      for (const Vec3 vertex : face.vertices)
        ExtendBounds(face_body_bounds[face.source_body_group_offset], vertex);
    }
    std::map<std::size_t, uint64_t> topology_body_groups;
    for (const auto &[root, indices] : connected_edges) {
      Bounds3 edge_bounds;
      for (const std::size_t index : indices) {
        ExtendBounds(edge_bounds, bucket->source_lines[index].start);
        ExtendBounds(edge_bounds, bucket->source_lines[index].end);
      }
      const auto mismatch = [&](const Bounds3 &body_bounds) {
        return std::abs(edge_bounds.min.x - body_bounds.min.x) +
               std::abs(edge_bounds.min.y - body_bounds.min.y) +
               std::abs(edge_bounds.min.z - body_bounds.min.z) +
               std::abs(edge_bounds.max.x - body_bounds.max.x) +
               std::abs(edge_bounds.max.y - body_bounds.max.y) +
               std::abs(edge_bounds.max.z - body_bounds.max.z);
      };
      uint64_t best_body = 0;
      double best_mismatch = std::numeric_limits<double>::infinity();
      double second_mismatch = std::numeric_limits<double>::infinity();
      double best_extent = 0.0;
      for (const auto &[body_group, body_bounds] : face_body_bounds) {
        const double candidate = mismatch(body_bounds);
        if (candidate < best_mismatch) {
          second_mismatch = best_mismatch;
          best_mismatch = candidate;
          best_body = body_group;
          best_extent = std::max(
              {body_bounds.max.x - body_bounds.min.x,
               body_bounds.max.y - body_bounds.min.y,
               body_bounds.max.z - body_bounds.min.z});
        } else if (candidate < second_mismatch) {
          second_mismatch = candidate;
        }
      }
      const double tolerance = std::max(1e-5, best_extent * 0.01);
      if (best_body != 0 && best_mismatch <= tolerance &&
          second_mismatch > best_mismatch + tolerance) {
        topology_body_groups.emplace(root, best_body);
      }
    }
    const auto projected_turn = [](Vec2 origin, Vec2 first, Vec2 second) {
      return (first.x - origin.x) * (second.y - origin.y) -
             (first.y - origin.y) * (second.x - origin.x);
    };
    const auto point_on_segment = [](Vec2 point, Vec2 start, Vec2 end) {
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= 1e-24)
        return std::hypot(point.x - start.x, point.y - start.y);
      const double fraction = std::clamp(
          ((point.x - start.x) * dx + (point.y - start.y) * dy) /
              length_squared,
          0.0, 1.0);
      return std::hypot(point.x - (start.x + fraction * dx),
                        point.y - (start.y + fraction * dy));
    };
    std::set<std::size_t> local_silhouette_edges;
    for (const auto &[root, indices] : connected_edges) {
      (void)root;
      // A standalone polyline has no closed-body silhouette. Requiring several
      // joined edges also avoids promoting isolated construction geometry.
      if (indices.size() < 8u)
        continue;
      std::vector<Vec2> points;
      for (const std::size_t index : indices) {
        points.push_back(project(bucket->source_lines[index].start));
        points.push_back(project(bucket->source_lines[index].end));
      }
      std::sort(points.begin(), points.end(), [](Vec2 left, Vec2 right) {
        return left.x < right.x || (left.x == right.x && left.y < right.y);
      });
      points.erase(std::unique(points.begin(), points.end(), [](Vec2 left,
                                                                Vec2 right) {
                     return left.x == right.x && left.y == right.y;
                   }),
                   points.end());
      if (points.size() < 3u)
        continue;
      std::vector<Vec2> hull;
      hull.reserve(points.size() * 2u);
      for (const Vec2 point : points) {
        while (hull.size() >= 2u &&
               projected_turn(hull[hull.size() - 2u], hull.back(), point) <=
                   0.0) {
          hull.pop_back();
        }
        hull.push_back(point);
      }
      const std::size_t lower_size = hull.size();
      for (auto iterator = points.rbegin() + 1u; iterator != points.rend();
           ++iterator) {
        while (hull.size() > lower_size &&
               projected_turn(hull[hull.size() - 2u], hull.back(), *iterator) <=
                   0.0) {
          hull.pop_back();
        }
        hull.push_back(*iterator);
      }
      hull.pop_back();
      for (const std::size_t index : indices) {
        const Vec2 start = project(bucket->source_lines[index].start);
        const Vec2 end = project(bucket->source_lines[index].end);
        for (std::size_t side = 0u; side < hull.size(); ++side) {
          const Vec2 side_start = hull[side];
          const Vec2 side_end = hull[(side + 1u) % hull.size()];
          if (point_on_segment(start, side_start, side_end) <=
                  kCoincidentEdgeToleranceM &&
              point_on_segment(end, side_start, side_end) <=
                  kCoincidentEdgeToleranceM) {
            local_silhouette_edges.insert(index);
            break;
          }
        }
      }
    }

    std::vector<std::vector<LineVisibilitySpan>> visibility_spans(
        bucket->source_lines.size());
    std::vector<bool> fully_hidden_active_edge(bucket->source_lines.size(),
                                               false);
    std::vector<std::size_t> fully_hidden_active_indices;
    for (std::size_t source_index = 0u;
         source_index < bucket->source_lines.size(); ++source_index) {
      const Line3D &source = bucket->source_lines[source_index];
      const bool cached_hidden = source.source_record_type == 0x31u ||
                                 source.source_record_type == 0x33u;
      visibility_spans[source_index] = {{0.0, 1.0, cached_hidden}};
      if (!view_faces.empty()) {
        visibility_spans[source_index] = SplitLineByTessellation(
            source, view_faces, placement.model_to_view);
      }
      fully_hidden_active_edge[source_index] =
          (source.source_record_type == 0x21u ||
           source.source_record_type == 0x23u) &&
          visibility_spans[source_index].size() == 1u &&
          visibility_spans[source_index].front().hidden;
      if (fully_hidden_active_edge[source_index])
        fully_hidden_active_indices.push_back(source_index);
    }

    // A thin member often contributes two almost coincident projected edges.
    // When one edge lies exactly on a face boundary, the triangle clip can
    // leave it visible even though the parallel edge immediately behind it is
    // fully occluded. SolidWorks resolves both as hidden. Limit that inference
    // to short topology fragments in an orthographic view; closed-body
    // contours and their local silhouettes keep their independent visibility.
    const auto has_hidden_parallel_companion = [&](std::size_t source_index) {
      if (largest_direction_component < 0.999 ||
          connected_edges[find_parent(source_index)].size() >= 8u) {
        return false;
      }
      const Line3D &source = bucket->source_lines[source_index];
      if ((source.source_record_type != 0x21u &&
           source.source_record_type != 0x23u) ||
          fully_hidden_active_edge[source_index]) {
        return false;
      }
      const Vec2 source_start = project(source.start);
      const Vec2 source_end = project(source.end);
      const double source_dx = source_end.x - source_start.x;
      const double source_dy = source_end.y - source_start.y;
      const double source_length = std::hypot(source_dx, source_dy);
      constexpr double kMinimumPairedEdgeLengthM = 0.005;
      constexpr double kPairedEdgeEndpointToleranceM = 1e-7;
      constexpr double kPairedEdgeSeparationM = 0.0002;
      constexpr double kMaximumPairedEdgeDepthM = 0.01;
      if (source_length < kMinimumPairedEdgeLengthM)
        return false;
      const double tangent_x = source_dx / source_length;
      const double tangent_y = source_dy / source_length;
      for (const std::size_t candidate_index : fully_hidden_active_indices) {
        const Line3D &candidate = bucket->source_lines[candidate_index];
        if (candidate.source_record_type != source.source_record_type)
          continue;
        Vec2 candidate_start = project(candidate.start);
        Vec2 candidate_end = project(candidate.end);
        double candidate_dx = candidate_end.x - candidate_start.x;
        double candidate_dy = candidate_end.y - candidate_start.y;
        if (source_dx * candidate_dx + source_dy * candidate_dy < 0.0) {
          std::swap(candidate_start, candidate_end);
          candidate_dx = -candidate_dx;
          candidate_dy = -candidate_dy;
        }
        const double candidate_length =
            std::hypot(candidate_dx, candidate_dy);
        if (std::abs(candidate_length - source_length) >
            kPairedEdgeEndpointToleranceM) {
          continue;
        }
        const Vec2 start_delta{candidate_start.x - source_start.x,
                               candidate_start.y - source_start.y};
        const Vec2 end_delta{candidate_end.x - source_end.x,
                             candidate_end.y - source_end.y};
        const double start_along =
            start_delta.x * tangent_x + start_delta.y * tangent_y;
        const double end_along =
            end_delta.x * tangent_x + end_delta.y * tangent_y;
        if (std::abs(start_along) > kPairedEdgeEndpointToleranceM ||
            std::abs(end_along) > kPairedEdgeEndpointToleranceM) {
          continue;
        }
        const double start_across =
            start_delta.x * -tangent_y + start_delta.y * tangent_x;
        const double end_across =
            end_delta.x * -tangent_y + end_delta.y * tangent_x;
        if (std::abs(start_across - end_across) <=
                kPairedEdgeEndpointToleranceM &&
            std::abs(start_across) > kPairedEdgeEndpointToleranceM &&
            std::abs(start_across) <= kPairedEdgeSeparationM &&
            std::abs(ColumnDot(placement.model_to_view, 2u, candidate.start) -
                     ColumnDot(placement.model_to_view, 2u, source.start)) <=
                kMaximumPairedEdgeDepthM &&
            std::abs(ColumnDot(placement.model_to_view, 2u, candidate.end) -
                     ColumnDot(placement.model_to_view, 2u, source.end)) <=
                kMaximumPairedEdgeDepthM) {
          return true;
        }
      }
      return false;
    };

    for (std::size_t source_index = 0u;
         source_index < bucket->source_lines.size(); ++source_index) {
      if (duplicate_tangent_edges.contains(source_index))
        continue;
      const Line3D &source = bucket->source_lines[source_index];
      const bool cached_hidden = source.source_record_type == 0x31u ||
                                 source.source_record_type == 0x33u;
      const std::vector<LineVisibilitySpan> &spans =
          visibility_spans[source_index];
      const bool shadowed_by_hidden_companion =
          has_hidden_parallel_companion(source_index);
      for (std::size_t span_index = 0u; span_index < spans.size();
           ++span_index) {
        const LineVisibilitySpan &span = spans[span_index];
        const bool complete_computed_occlusion =
            spans.size() == 1u && spans.front().hidden;
        const Vec2 projected_source_start = project(source.start);
        const Vec2 projected_source_end = project(source.end);
        constexpr double kMinimumLocalSilhouetteLengthM = 0.005;
        const bool substantial_local_silhouette =
            local_silhouette_edges.contains(source_index) &&
            std::hypot(projected_source_end.x - projected_source_start.x,
                       projected_source_end.y - projected_source_start.y) >=
                kMinimumLocalSilhouetteLengthM;
        const bool hidden =
            cached_hidden || shadowed_by_hidden_companion ||
            (span.hidden &&
             !(complete_computed_occlusion && substantial_local_silhouette));
        if (shaded_hlr_edges && hidden)
          continue;
        const auto interpolate = [&](double fraction) {
          return Vec3{
              source.start.x + (source.end.x - source.start.x) * fraction,
              source.start.y + (source.end.y - source.start.y) * fraction,
              source.start.z + (source.end.z - source.start.z) * fraction};
        };
        Vec2 start = project(interpolate(span.start_fraction));
        Vec2 end = project(interpolate(span.end_fraction));
        if (!ClipLine(placement.clip_bounds, start, end))
          continue;
        if (std::hypot(end.x - start.x, end.y - start.y) <
            kMinimumVisibilitySpanM) {
          continue;
        }
        LineStyle style;
        if (hidden) {
          style.pattern = LinePattern::kHidden;
          style.source_line_weight = hidden_line_weight;
        } else if (tangent_edge_representatives.contains(source_index)) {
          style.pattern = LinePattern::kPhantom;
          style.source_line_weight = tangent_line_weight;
        } else {
          style.source_line_weight = visible_line_weight;
        }
        sheet.entities.emplace_back(
            Line2D{start, end, style,
                   "Contents/VBLists/uiViewBucket_c/" + placement.name +
                       "/offset=" + std::to_string(source.source_offset) +
                       "/record=" + std::to_string(source.source_record_type) +
                       "/span=" + std::to_string(span_index),
                   DrawingEntitySource{
                       view_ownership_index,
                       source.source_offset,
                       bucket->source_lines[find_parent(source_index)]
                           .source_offset,
                       source.source_record_type,
                       topology_body_groups.contains(find_parent(source_index))
                           ? topology_body_groups.at(find_parent(source_index))
                           : 0,
                       topology_body_groups.contains(
                           find_parent(source_index))}});
      }
    }

    const auto point_segment_distance = [](Vec2 point, const Line2D &line) {
      const double dx = line.end.x - line.start.x;
      const double dy = line.end.y - line.start.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= 1e-24)
        return std::hypot(point.x - line.start.x, point.y - line.start.y);
      const double fraction = std::clamp(
          ((point.x - line.start.x) * dx + (point.y - line.start.y) * dy) /
              length_squared,
          0.0, 1.0);
      return std::hypot(point.x - (line.start.x + fraction * dx),
                        point.y - (line.start.y + fraction * dy));
    };
    const auto computed_hidden = [](const Line2D &line) {
      return line.style.pattern == LinePattern::kHidden &&
             (line.source.record_type == 0x21u ||
              line.source.record_type == 0x23u);
    };

    // A line on the convex boundary of the complete projected cache is a
    // drawing silhouette. No face can correctly occlude it from outside the
    // view, even when ambiguous assembly ownership makes the depth test choose
    // a coincident face from another component.
    std::vector<Vec2> projected_points;
    for (auto iterator =
             sheet.entities.begin() +
             static_cast<std::ptrdiff_t>(view_entity_begin);
         iterator != sheet.entities.end(); ++iterator) {
      const Line2D *line = std::get_if<Line2D>(&*iterator);
      if (line == nullptr)
        continue;
      projected_points.push_back(line->start);
      projected_points.push_back(line->end);
    }
    std::sort(projected_points.begin(), projected_points.end(),
              [](Vec2 left, Vec2 right) {
                return left.x < right.x ||
                       (left.x == right.x && left.y < right.y);
              });
    projected_points.erase(
        std::unique(projected_points.begin(), projected_points.end(),
                    [](Vec2 left, Vec2 right) {
                      return left.x == right.x && left.y == right.y;
                    }),
        projected_points.end());
    const auto turn = [](Vec2 origin, Vec2 first, Vec2 second) {
      return (first.x - origin.x) * (second.y - origin.y) -
             (first.y - origin.y) * (second.x - origin.x);
    };
    std::vector<Vec2> projected_hull;
    if (projected_points.size() >= 3u) {
      projected_hull.reserve(projected_points.size() * 2u);
      for (const Vec2 point : projected_points) {
        while (projected_hull.size() >= 2u &&
               turn(projected_hull[projected_hull.size() - 2u],
                    projected_hull.back(), point) <= 0.0) {
          projected_hull.pop_back();
        }
        projected_hull.push_back(point);
      }
      const std::size_t lower_size = projected_hull.size();
      for (auto iterator = projected_points.rbegin() + 1u;
           iterator != projected_points.rend(); ++iterator) {
        while (projected_hull.size() > lower_size &&
               turn(projected_hull[projected_hull.size() - 2u],
                    projected_hull.back(), *iterator) <= 0.0) {
          projected_hull.pop_back();
        }
        projected_hull.push_back(*iterator);
      }
      projected_hull.pop_back();
    }
    const auto on_projected_hull = [&](const Line2D &line) {
      for (std::size_t index = 0u; index < projected_hull.size(); ++index) {
        const Line2D boundary{projected_hull[index],
                              projected_hull[(index + 1u) %
                                             projected_hull.size()],
                              {}, {}, {}};
        if (point_segment_distance(line.start, boundary) <=
                kCoincidentEdgeToleranceM &&
            point_segment_distance(line.end, boundary) <=
                kCoincidentEdgeToleranceM) {
          return true;
        }
      }
      return false;
    };
    for (auto iterator =
             sheet.entities.begin() +
             static_cast<std::ptrdiff_t>(view_entity_begin);
         iterator != sheet.entities.end(); ++iterator) {
      Line2D *line = std::get_if<Line2D>(&*iterator);
      if (line != nullptr && computed_hidden(*line) &&
          on_projected_hull(*line)) {
        line->style.pattern = LinePattern::kSolid;
        line->style.source_line_weight = visible_line_weight;
      }
    }

    // Assembly caches can also repeat a projected contour for two component
    // instances with sub-millimetre coordinate drift. SolidWorks resolves the
    // overlap in favour of the visible contour.
    constexpr double kOutlineOverlapToleranceM = 0.0002;
    constexpr double kParallelDirectionTolerance = 0.005;
    std::vector<Line2D> visible_outlines;
    for (auto iterator =
             sheet.entities.begin() +
             static_cast<std::ptrdiff_t>(view_entity_begin);
         iterator != sheet.entities.end(); ++iterator) {
      const Line2D *line = std::get_if<Line2D>(&*iterator);
      if (line != nullptr && line->style.pattern == LinePattern::kSolid)
        visible_outlines.push_back(*line);
    }
    const auto projected_outline_duplicate = [&](const Line2D &hidden) {
      const double hidden_dx = hidden.end.x - hidden.start.x;
      const double hidden_dy = hidden.end.y - hidden.start.y;
      const double hidden_length = std::hypot(hidden_dx, hidden_dy);
      if (hidden_length <= kMinimumVisibilitySpanM)
        return false;
      for (const Line2D &visible : visible_outlines) {
        const double visible_dx = visible.end.x - visible.start.x;
        const double visible_dy = visible.end.y - visible.start.y;
        const double visible_length = std::hypot(visible_dx, visible_dy);
        if (visible_length <= kMinimumVisibilitySpanM ||
            std::abs(hidden_dx * visible_dy - hidden_dy * visible_dx) >
                kParallelDirectionTolerance * hidden_length * visible_length) {
          continue;
        }
        if (point_segment_distance(hidden.start, visible) <=
                kOutlineOverlapToleranceM &&
            point_segment_distance(hidden.end, visible) <=
                kOutlineOverlapToleranceM) {
          return true;
        }
      }
      return false;
    };
    auto erase_begin = sheet.entities.begin() +
                       static_cast<std::ptrdiff_t>(view_entity_begin);
    sheet.entities.erase(
        std::remove_if(erase_begin, sheet.entities.end(),
                       [&](const DrawingEntity &entity) {
                         const Line2D *line = std::get_if<Line2D>(&entity);
                         if (line == nullptr || !computed_hidden(*line)) {
                           return false;
                         }
                         return projected_outline_duplicate(*line);
                       }),
        sheet.entities.end());
  }

  if (!display_lists.empty()) {
    auto display_lines = ReadDisplayListLines(
        display_lists, all_names, keyword_views->selected, sheet.width_m,
        sheet.height_m, display_object_ids, keyword_views->sheet_names,
        keyword_views->sheet_name);
    if (display_lines.ok()) {
      std::vector<Line2D> parsed_lines = std::move(display_lines).value();
      for (FilledTriangle2D &triangle :
           FindDisplayListArrowheads(parsed_lines)) {
        const std::string_view object_id =
            DisplayObjectId(triangle.source_table);
        if (!object_id.empty()) {
          display_arrowhead_object_ids.emplace(object_id);
        }
        sheet.entities.emplace_back(std::move(triangle));
      }
      for (Line2D &line : parsed_lines) {
        sheet.entities.emplace_back(std::move(line));
      }
    }
    auto display_arcs = ReadDisplayListArcs(
        display_lists, all_names, keyword_views->selected, sheet.width_m,
        sheet.height_m, display_object_ids, keyword_views->sheet_names,
        keyword_views->sheet_name);
    if (display_arcs.ok()) {
      for (Arc2D &arc : std::move(display_arcs).value()) {
        sheet.entities.emplace_back(std::move(arc));
      }
    }
    auto parsed_text = ReadDisplayListText(
        display_lists, all_names, keyword_views->selected, sheet.width_m,
        sheet.height_m, display_object_ids, keyword_views->sheet_names,
        keyword_views->sheet_name);
    if (parsed_text.ok())
      display_text = std::move(parsed_text).value();
  }
  const bool has_cached_sheet_text = std::any_of(
      display_text.begin(), display_text.end(),
      [](const TextAnnotation &annotation) {
        return annotation.source_table.find("/sheet=") != std::string::npos;
      });

  auto format_lines = ReadSheetFormatLines(
      definition, keyword_views->sheet_names, keyword_views->sheet_name,
      sheet.width_m, sheet.height_m,
      std::any_of(sheet.entities.begin(), sheet.entities.end(),
                  [](const DrawingEntity &entity) {
                    return std::holds_alternative<RasterImage2D>(entity);
                  }));
  if (format_lines.ok()) {
    std::vector<Line2D> parsed_lines = std::move(format_lines).value();
    for (Line2D &line : parsed_lines) {
      const auto same_point = [](Vec2 left, Vec2 right) {
        return std::abs(left.x - right.x) < 1e-10 &&
               std::abs(left.y - right.y) < 1e-10;
      };
      const bool duplicate =
          std::any_of(sheet.entities.begin(), sheet.entities.end(),
                      [&](const auto &entity) {
                        const Line2D *existing = std::get_if<Line2D>(&entity);
                        return existing != nullptr &&
                               ((same_point(existing->start, line.start) &&
                                 same_point(existing->end, line.end)) ||
                                (same_point(existing->start, line.end) &&
                                 same_point(existing->end, line.start)));
                      });
      if (duplicate)
        continue;
      sheet.entities.emplace_back(std::move(line));
    }
  }

  auto format_text = ReadSheetFormatText(definition, keyword_views->sheet_names,
                                         keyword_views->sheet_name);
  if (format_text.ok() && !has_cached_sheet_text) {
    std::vector<TextAnnotation> parsed_text = std::move(format_text).value();
    for (TextAnnotation &annotation : parsed_text) {
      const bool cached = std::any_of(
          display_text.begin(), display_text.end(),
          [&](const TextAnnotation &candidate) {
            return candidate.text == annotation.text &&
                   std::hypot(candidate.origin.x - annotation.origin.x,
                              candidate.origin.y - annotation.origin.y) < 0.004;
          });
      if (cached)
        continue;
      sheet.entities.emplace_back(std::move(annotation));
    }
  }

  auto weldment_lines = ReadLegacyWeldmentTableLines(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  if (weldment_lines.ok()) {
    std::vector<Line2D> parsed_lines = std::move(weldment_lines).value();
    for (Line2D &line : parsed_lines) {
      sheet.entities.emplace_back(std::move(line));
    }
  }
  auto weldment_text = ReadLegacyWeldmentTableText(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  if (weldment_text.ok()) {
    std::vector<TextAnnotation> parsed_text = std::move(weldment_text).value();
    const std::size_t cached_matches = static_cast<std::size_t>(std::count_if(
        parsed_text.begin(), parsed_text.end(),
        [&](const TextAnnotation &annotation) {
          return std::any_of(
              display_text.begin(), display_text.end(),
              [&](const TextAnnotation &candidate) {
                return candidate.text == annotation.text &&
                       std::hypot(candidate.origin.x - annotation.origin.x,
                                  candidate.origin.y - annotation.origin.y) <
                           0.006;
              });
        }));
    // Three matching cells establish that DisplayLists contains the complete
    // cached table, whose baselines and text styles are more faithful than the
    // reconstructed Definition cells. The cache does not always repeat a
    // usable font-height record, so retain its exact origins but transfer the
    // validated per-cell height from Definition. Also discard the reconstructed
    // grid: its inferred row origin can differ from the cached grid by about a
    // millimetre and would otherwise create doubled horizontal rules.
    if (cached_matches < 3u) {
      for (TextAnnotation &annotation : parsed_text) {
        sheet.entities.emplace_back(std::move(annotation));
      }
    } else {
      for (TextAnnotation &cached : display_text) {
        const TextAnnotation *closest = nullptr;
        double closest_distance = 0.006;
        for (const TextAnnotation &reconstructed : parsed_text) {
          if (cached.text != reconstructed.text)
            continue;
          const double distance =
              std::hypot(cached.origin.x - reconstructed.origin.x,
                         cached.origin.y - reconstructed.origin.y);
          if (distance < closest_distance) {
            closest = &reconstructed;
            closest_distance = distance;
          }
        }
        if (closest != nullptr)
          cached.height = closest->height;
      }
      std::erase_if(sheet.entities, [](const DrawingEntity &entity) {
        const Line2D *line = std::get_if<Line2D>(&entity);
        return line != nullptr &&
               line->source_table ==
                   "Contents/Definition/moDisplayWeldmentTable_c/grid";
      });
    }
  }

  if (!tables.empty()) {
    auto table_geometry = ReadTablesXml(tables);
    if (table_geometry.ok()) {
      const auto source = std::find_if(
          table_geometry.value().sheets.begin(),
          table_geometry.value().sheets.end(),
          [&](const auto &item) { return item.name == sheet.name; });
      if (source != table_geometry.value().sheets.end()) {
        sheet.entities.insert(sheet.entities.end(), source->entities.begin(),
                              source->entities.end());
      }
    }
  }

  auto dimensions = ReadLegacyDimensions(definition, keyword_views->sheet_names,
                                         keyword_views->sheet_name);
  if (dimensions.ok()) {
    std::vector<const DrawingViewPlacement *> selected_placements;
    for (const DrawingViewPlacement &placement : placements) {
      if (selected.contains(placement.name)) {
        selected_placements.push_back(&placement);
      }
    }
    // The legacy record does not yet expose its owning view reference. A
    // single-view sheet is unambiguous; retain text-only dimensions on more
    // complex sheets until that relationship is decoded.
    const double paper_scale = selected_placements.size() == 1u
                                   ? selected_placements.front()->scale
                                   : 0.0;
    std::vector<Dimension> parsed_dimensions = std::move(dimensions).value();
    for (Dimension &dimension : parsed_dimensions) {
      dimension.paper_length_m = dimension.measured_value_m * paper_scale;
      sheet.entities.emplace_back(std::move(dimension));
    }
  }
  auto balloons = ReadLegacyBomBalloons(definition, keyword_views->sheet_names,
                                        keyword_views->sheet_name);
  std::vector<BomBalloon> parsed_balloons;
  if (balloons.ok()) {
    parsed_balloons = std::move(balloons).value();
  }
  // The richer Definition record gives us the balloon label and arrow anchor,
  // while DisplayLists often already carries the exact bent leader.  Mark a
  // matching cached segment so SvgWriter does not draw a second straight
  // leader over it.  Keep the other endpoint to orient the arrowhead along the
  // native final segment.
  for (BomBalloon &balloon : parsed_balloons) {
    constexpr double kLeaderEndpointToleranceM = 1e-7;
    const auto same_point = [&](Vec2 point) {
      return std::hypot(point.x - balloon.leader_anchor.x,
                        point.y - balloon.leader_anchor.y) <
             kLeaderEndpointToleranceM;
    };
    for (const DrawingEntity &entity : sheet.entities) {
      const Line2D *line = std::get_if<Line2D>(&entity);
      if (line == nullptr ||
          !line->source_table.starts_with("Contents/DisplayLists/line/")) {
        continue;
      }
      if (same_point(line->start)) {
        balloon.has_cached_leader = true;
        balloon.cached_leader_point = line->end;
        break;
      }
      if (same_point(line->end)) {
        balloon.has_cached_leader = true;
        balloon.cached_leader_point = line->start;
        break;
      }
    }
  }
  // DisplayLists repeats text already represented by the richer legacy BOM
  // balloon entity. Match both label and nearby centre so ordinary dimensions
  // containing the same number are retained.
  std::erase_if(display_text, [&](const TextAnnotation &annotation) {
    return std::any_of(
        parsed_balloons.begin(), parsed_balloons.end(),
        [&](const BomBalloon &balloon) {
          return annotation.text == balloon.text &&
                 std::hypot(annotation.origin.x - balloon.text_origin.x,
                            annotation.origin.y - balloon.text_origin.y) <
                     kDisplayListBalloonLabelToleranceM;
        });
  });
  std::multiset<std::string> display_text_object_ids;
  for (const TextAnnotation &annotation : display_text) {
    display_text_object_ids.emplace(DisplayObjectId(annotation.source_table));
  }
  for (TextAnnotation &annotation : display_text) {
    // A cached dimension owns a pair of arrowheads under the same DisplayLists
    // objID as its text. BOM balloons have only one arrowhead and therefore
    // keep their drawing/template font. This also covers linear dimensions,
    // which do not have the circular arc used by angular dimensions.
    const std::string_view object_id = DisplayObjectId(annotation.source_table);
    if (!object_id.empty() &&
        display_arrowhead_object_ids.count(std::string(object_id)) >= 2u) {
      annotation.font_family = "osifont";
      if (display_text_object_ids.count(std::string(object_id)) == 1u) {
        AlignAngularDimensionLabel(annotation, sheet.entities);
      }
    } else if (!object_id.empty() &&
               display_arrowhead_object_ids.count(std::string(object_id)) ==
                   1u &&
               display_text_object_ids.count(std::string(object_id)) == 1u) {
      AlignLeaderLabel(annotation, sheet.entities);
    }
    const bool duplicate = std::any_of(
        sheet.entities.begin(), sheet.entities.end(),
        [&](const DrawingEntity &entity) {
          const TextAnnotation *existing = std::get_if<TextAnnotation>(&entity);
          return existing != nullptr && existing->text == annotation.text &&
                 std::hypot(existing->origin.x - annotation.origin.x,
                            existing->origin.y - annotation.origin.y) < 0.004;
        });
    if (duplicate)
      continue;
    sheet.entities.emplace_back(std::move(annotation));
  }
  for (BomBalloon &balloon : parsed_balloons) {
    sheet.entities.emplace_back(std::move(balloon));
  }
  if (sheet.entities.empty()) {
    return Result<DrawingSheetGeometry>::Err(
        "No cached view or table geometry found for requested sheet");
  }
  return Result<DrawingSheetGeometry>::Ok(std::move(sheet));
}

} // namespace openswx
