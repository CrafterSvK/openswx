// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
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
      const double dot = ((left_base.x - tip.x) * (right_base.x - tip.x) +
                          (left_base.y - tip.y) * (right_base.y - tip.y)) /
                         (left_length * right_length);
      if (!std::isfinite(dot) || dot < 0.7)
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
        return {};
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

Result<DrawingSheetGeometry> DrawingGeometryReader::ReadCachedSheet(
    std::span<const uint8_t> keywords, std::span<const uint8_t> definition,
    std::span<const uint8_t> vb_lists, std::span<const uint8_t> tables,
    std::string_view sheet_name, std::span<const uint8_t> display_lists,
    std::span<const uint8_t> sketch_bitmap) {
  const auto keyword_views = ParseKeywordViews(keywords, sheet_name);
  if (!keyword_views) {
    return Result<DrawingSheetGeometry>::Err(
        "Requested sheet was not found in KeyWords XML");
  }
  const std::vector<std::string> all_names(keyword_views->all.begin(),
                                           keyword_views->all.end());
  std::vector<DrawingViewPlacement> placements;
  std::vector<CachedDrawingView> buckets;
  std::vector<FaceTessellation> face_tessellations;
  std::vector<DrawingAppearance> appearances;
  std::vector<DrawingSceneLight> scene_lights;
  std::multiset<std::string> display_arrowhead_object_ids;
  if (!all_names.empty()) {
    auto parsed_placements = ReadViewPlacements(definition, all_names);
    if (!parsed_placements.ok()) {
      return Result<DrawingSheetGeometry>::Err(parsed_placements.error());
    }
    placements = std::move(parsed_placements).value();
    auto parsed_buckets = ReadViewBuckets(vb_lists);
    if (!parsed_buckets.ok()) {
      return Result<DrawingSheetGeometry>::Err(parsed_buckets.error());
    }
    buckets = std::move(parsed_buckets).value();
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
  std::vector<TextAnnotation> display_text;
  std::vector<uint32_t> display_object_ids;
  auto parsed_object_ids = ReadDisplayObjectIds(
      definition, keyword_views->sheet_names, keyword_views->sheet_name);
  if (parsed_object_ids.ok()) {
    display_object_ids = std::move(parsed_object_ids).value();
  }
  for (std::size_t placement_index = 0; placement_index < placements.size();
       ++placement_index) {
    const DrawingViewPlacement &placement = placements[placement_index];
    if (!selected.contains(placement.name))
      continue;
    auto bucket = std::find_if(buckets.begin(), buckets.end(),
                               [&](const CachedDrawingView &item) {
                                 return item.name == placement.name;
                               });
    // SolidWorks 19000 VBLists references the component/assembly rather than
    // the drawing-view name. In the confirmed layout the bucket and placement
    // arrays are parallel. Restrict this fallback to an exact cardinality
    // match; otherwise a guessed bucket would be worse than no geometry.
    if (bucket == buckets.end() && buckets.size() == placements.size()) {
      bucket = buckets.begin() + static_cast<std::ptrdiff_t>(placement_index);
    }
    if (bucket == buckets.end() || bucket->source_lines.empty())
      continue;
    const std::vector<FaceTessellation> view_faces =
        FacesForView(face_tessellations, *bucket, buckets);

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
    if (bucket->has_suppressed_edge_cache && !view_faces.empty()) {
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
    for (const Line3D &source : bucket->source_lines) {
      std::vector<LineVisibilitySpan> spans = {{0.0, 1.0, false}};
      if (!view_faces.empty()) {
        spans = SplitLineByTessellation(source, view_faces,
                                        placement.model_to_view);
      }
      for (std::size_t span_index = 0u; span_index < spans.size();
           ++span_index) {
        constexpr double kMinimumVisibilitySpanM = 1e-5;
        const LineVisibilitySpan &span = spans[span_index];
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
        if (span.hidden)
          style.pattern = LinePattern::kHidden;
        sheet.entities.emplace_back(
            Line2D{start, end, style,
                   "Contents/VBLists/uiViewBucket_c/" + placement.name +
                       "/offset=" + std::to_string(source.source_offset) +
                       "/record=" + std::to_string(source.source_record_type) +
                       "/span=" + std::to_string(span_index)});
      }
    }
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
  for (TextAnnotation &annotation : display_text) {
    // A cached dimension owns a pair of arrowheads under the same DisplayLists
    // objID as its text. BOM balloons have only one arrowhead and therefore
    // keep their drawing/template font. This also covers linear dimensions,
    // which do not have the circular arc used by angular dimensions.
    const std::string_view object_id = DisplayObjectId(annotation.source_table);
    if (!object_id.empty() &&
        display_arrowhead_object_ids.count(std::string(object_id)) >= 2u) {
      annotation.font_family = "osifont";
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
