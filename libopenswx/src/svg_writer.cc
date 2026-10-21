// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/svg_writer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <vector>

#include <zlib.h>

namespace openswx {
namespace {

constexpr double kMetresToMillimetres = 1000.0;
constexpr double kRadiansToDegrees = 57.2957795130823208768;

std::string EscapeXml(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
    case '&':
      escaped += "&amp;";
      break;
    case '<':
      escaped += "&lt;";
      break;
    case '>':
      escaped += "&gt;";
      break;
    case '"':
      escaped += "&quot;";
      break;
    case '\'':
      escaped += "&apos;";
      break;
    default:
      escaped += character;
      break;
    }
  }
  return escaped;
}

struct Bounds {
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  void Add(Vec2 point) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  [[nodiscard]] bool Valid() const {
    return std::isfinite(min_x) && std::isfinite(min_y) &&
           std::isfinite(max_x) && std::isfinite(max_y);
  }
};

std::string ColorHex(uint32_t rgb) {
  std::ostringstream output;
  output << '#' << std::hex << std::setfill('0') << std::setw(6)
         << (rgb & 0xFFFFFFu);
  return output.str();
}

std::string_view DashArray(LinePattern pattern) {
  // SolidWorks' PDF export expresses these values in PostScript points. The
  // SVG uses millimetres, so the confirmed 3.6 pt base unit becomes 1.27 mm.
  switch (pattern) {
  case LinePattern::kHidden:
    return "1.27 0.635";
  case LinePattern::kCenter:
    return "15.24 1.27 1.27 1.27";
  case LinePattern::kChain:
  case LinePattern::kChainThick:
    return "6.35 1.27 1.27 1.27";
  case LinePattern::kPhantom:
    return "6.35 1.27 1.27 1.27 1.27 1.27";
  case LinePattern::kStitch:
    return "0.7 1.2";
  case LinePattern::kSolid:
    return {};
  }
  return {};
}

std::string_view DashLineCap(LinePattern pattern) {
  switch (pattern) {
  case LinePattern::kHidden:
  case LinePattern::kCenter:
  case LinePattern::kChain:
  case LinePattern::kChainThick:
    return "butt";
  case LinePattern::kPhantom:
    return "round";
  case LinePattern::kStitch:
  case LinePattern::kSolid:
    return {};
  }
  return {};
}

std::optional<double> CenteredDashOffset(LinePattern pattern,
                                         double length_mm) {
  if (pattern != LinePattern::kChain && pattern != LinePattern::kChainThick)
    return std::nullopt;

  // SolidWorks centres cutting-plane patterns so both labelled ends land in
  // the same long dash. Offsets repeat after half of this symmetric cycle.
  constexpr double kDashMm = 6.35;
  constexpr double kPeriodMm = 10.16;
  constexpr double kSymmetricPeriodMm = kPeriodMm / 2.0;
  const double remainder = std::fmod(length_mm, kPeriodMm);
  double offset = std::fmod((kDashMm - remainder) / 2.0,
                            kSymmetricPeriodMm);
  if (offset < 0.0)
    offset += kSymmetricPeriodMm;
  return offset;
}

void AppendU32Be(std::vector<uint8_t> &output, uint32_t value) {
  output.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
  output.push_back(static_cast<uint8_t>(value & 0xFFu));
}

void AppendPngChunk(std::vector<uint8_t> &png, std::string_view type,
                    std::span<const uint8_t> payload) {
  AppendU32Be(png, static_cast<uint32_t>(payload.size()));
  const std::size_t crc_start = png.size();
  png.insert(png.end(), type.begin(), type.end());
  png.insert(png.end(), payload.begin(), payload.end());
  const uLong checksum =
      crc32(0u, reinterpret_cast<const Bytef *>(png.data() + crc_start),
            static_cast<uInt>(type.size() + payload.size()));
  AppendU32Be(png, static_cast<uint32_t>(checksum));
}

std::optional<std::vector<uint8_t>> EncodePng(const RasterImage2D &image) {
  if (image.pixel_width == 0u || image.pixel_height == 0u ||
      image.rgba.size() != static_cast<std::size_t>(image.pixel_width) *
                               image.pixel_height * 4u) {
    return std::nullopt;
  }
  const std::size_t row_bytes =
      static_cast<std::size_t>(image.pixel_width) * 4u;
  std::vector<uint8_t> filtered((row_bytes + 1u) *
                                static_cast<std::size_t>(image.pixel_height));
  for (std::size_t row = 0; row < image.pixel_height; ++row) {
    const std::size_t output_offset = row * (row_bytes + 1u);
    filtered[output_offset] = 0u;
    std::copy_n(
        image.rgba.begin() + static_cast<std::ptrdiff_t>(row * row_bytes),
        row_bytes,
        filtered.begin() + static_cast<std::ptrdiff_t>(output_offset + 1u));
  }
  uLongf compressed_size = compressBound(filtered.size());
  std::vector<uint8_t> compressed(compressed_size);
  if (compress2(compressed.data(), &compressed_size, filtered.data(),
                filtered.size(), Z_BEST_COMPRESSION) != Z_OK) {
    return std::nullopt;
  }
  compressed.resize(compressed_size);

  std::vector<uint8_t> png = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
  std::vector<uint8_t> header;
  header.reserve(13u);
  AppendU32Be(header, image.pixel_width);
  AppendU32Be(header, image.pixel_height);
  header.insert(header.end(), {8u, 6u, 0u, 0u, 0u});
  AppendPngChunk(png, "IHDR", header);
  AppendPngChunk(png, "IDAT", compressed);
  AppendPngChunk(png, "IEND", {});
  return png;
}

std::string Base64Encode(std::span<const uint8_t> data) {
  static constexpr std::string_view kAlphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((data.size() + 2u) / 3u) * 4u);
  for (std::size_t offset = 0; offset < data.size(); offset += 3u) {
    uint32_t value = static_cast<uint32_t>(data[offset]) << 16u;
    if (offset + 1u < data.size()) {
      value |= static_cast<uint32_t>(data[offset + 1u]) << 8u;
    }
    if (offset + 2u < data.size()) {
      value |= static_cast<uint32_t>(data[offset + 2u]);
    }
    output.push_back(kAlphabet[(value >> 18u) & 0x3Fu]);
    output.push_back(kAlphabet[(value >> 12u) & 0x3Fu]);
    output.push_back(
        offset + 1u < data.size() ? kAlphabet[(value >> 6u) & 0x3Fu] : '=');
    output.push_back(offset + 2u < data.size() ? kAlphabet[value & 0x3Fu]
                                               : '=');
  }
  return output;
}

double StrokeWidthMm(const LineStyle &style,
                     const DrawingSheetGeometry &sheet,
                     const SvgOptions &options) {
  if (style.source_line_weight >= 0 && style.source_line_weight < 8) {
    const double width_m =
        sheet.line_weights_m[static_cast<std::size_t>(style.source_line_weight)];
    if (std::isfinite(width_m) && width_m > 0.0) {
      // Serialized values commonly contain float-sized noise even though the
      // table uses doubles (for example 0.3499999875 mm for nominal 0.35 mm).
      // SVG needs no precision below one micrometre for printed line widths.
      const double width_mm = width_m * kMetresToMillimetres;
      return std::round(width_mm * 1000.0) / 1000.0;
    }
  }
  return options.default_stroke_width_mm;
}

} // namespace

Result<std::string> SvgWriter::WriteSheet(const DrawingSheetGeometry &sheet,
                                          const SvgOptions &options) {
  if (!std::isfinite(options.margin_mm) || options.margin_mm < 0.0 ||
      !std::isfinite(options.default_stroke_width_mm) ||
      options.default_stroke_width_mm <= 0.0) {
    return Result<std::string>::Err("Invalid SVG options");
  }

  Bounds bounds;
  for (const DrawingEntity &entity : sheet.entities) {
    std::visit(
        [&](const auto &item) {
          using T = std::decay_t<decltype(item)>;
          if constexpr (std::is_same_v<T, Line2D>) {
            bounds.Add(item.start);
            bounds.Add(item.end);
          } else if constexpr (std::is_same_v<T, Arc2D>) {
            const double radius = std::hypot(item.start.x - item.center.x,
                                             item.start.y - item.center.y);
            bounds.Add({item.center.x - radius, item.center.y - radius});
            bounds.Add({item.center.x + radius, item.center.y + radius});
          } else if constexpr (std::is_same_v<T, FilledTriangle2D>) {
            for (const Vec2 point : item.points)
              bounds.Add(point);
          } else if constexpr (std::is_same_v<T, RasterImage2D>) {
            bounds.Add(item.bottom_left);
            bounds.Add({item.bottom_left.x + item.width,
                        item.bottom_left.y + item.height});
          } else if constexpr (std::is_same_v<T, TextAnnotation>) {
            const double height = item.height > 0.0 ? item.height : 0.003;
            const double width =
                item.width > 0.0
                    ? item.width
                    : height * 0.6 * static_cast<double>(item.text.size());
            const double anchor_offset =
                item.anchor == TextAnchor::kMiddle
                    ? -width / 2.0
                    : (item.anchor == TextAnchor::kEnd ? -width : 0.0);
            const double cosine = std::cos(item.rotation_radians);
            const double sine = std::sin(item.rotation_radians);
            for (const Vec2 local :
                 {Vec2{anchor_offset, -height * 0.25},
                  Vec2{anchor_offset + width, -height * 0.25},
                  Vec2{anchor_offset, height},
                  Vec2{anchor_offset + width, height}}) {
              bounds.Add({item.origin.x + local.x * cosine - local.y * sine,
                          item.origin.y + local.x * sine + local.y * cosine});
            }
          } else if constexpr (std::is_same_v<T, BomBalloon>) {
            bounds.Add(item.leader_anchor);
            bounds.Add(item.text_origin);
          } else if constexpr (std::is_same_v<T, Dimension>) {
            bounds.Add(item.text_origin);
            const double half = item.paper_length_m * 0.5;
            if (item.orientation == DimensionOrientation::kHorizontal) {
              bounds.Add({item.text_origin.x - half, item.text_origin.y});
              bounds.Add({item.text_origin.x + half, item.text_origin.y});
            } else {
              bounds.Add({item.text_origin.x, item.text_origin.y - half});
              bounds.Add({item.text_origin.x, item.text_origin.y + half});
            }
          }
        },
        entity);
  }
  if (!bounds.Valid()) {
    return Result<std::string>::Err("Cannot render an empty drawing sheet");
  }

  const bool fixed_sheet = std::isfinite(sheet.width_m) &&
                           std::isfinite(sheet.height_m) &&
                           sheet.width_m > 0.0 && sheet.height_m > 0.0;
  const double view_x =
      fixed_sheet ? 0.0
                  : bounds.min_x * kMetresToMillimetres - options.margin_mm;
  const double view_y =
      fixed_sheet ? -sheet.height_m * kMetresToMillimetres
                  : -bounds.max_y * kMetresToMillimetres - options.margin_mm;
  const double width =
      fixed_sheet ? sheet.width_m * kMetresToMillimetres
                  : (bounds.max_x - bounds.min_x) * kMetresToMillimetres +
                        2.0 * options.margin_mm;
  const double height =
      fixed_sheet ? sheet.height_m * kMetresToMillimetres
                  : (bounds.max_y - bounds.min_y) * kMetresToMillimetres +
                        2.0 * options.margin_mm;

  std::ostringstream svg;
  svg << std::setprecision(15);
  svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "mm\" height=\"" << height << "mm\" viewBox=\"" << view_x << ' '
      << view_y << ' ' << width << ' ' << height << "\">\n"
      << "  <title>" << EscapeXml(sheet.name) << "</title>\n"
      << "  <rect x=\"" << view_x << "\" y=\"" << view_y << "\" width=\""
      << width << "\" height=\"" << height << "\" fill=\"#ffffff\"/>\n"
      << "  <g fill=\"none\" stroke-linecap=\"square\">\n";

  const Line2D *previous_pattern_line = nullptr;
  double pattern_distance_mm = 0.0;
  for (const DrawingEntity &entity : sheet.entities) {
    if (!std::holds_alternative<Line2D>(entity)) {
      previous_pattern_line = nullptr;
    }
    std::visit(
        [&](const auto &item) {
          using T = std::decay_t<decltype(item)>;
          if constexpr (std::is_same_v<T, Line2D>) {
            svg << "    <line x1=\"" << item.start.x * kMetresToMillimetres
                << "\" y1=\"" << -item.start.y * kMetresToMillimetres
                << "\" x2=\"" << item.end.x * kMetresToMillimetres << "\" y2=\""
                << -item.end.y * kMetresToMillimetres << "\" stroke=\""
                << ColorHex(item.style.rgb) << "\" stroke-width=\""
                << StrokeWidthMm(item.style, sheet, options) << "\"";
            const std::string_view dash_array = DashArray(item.style.pattern);
            if (!dash_array.empty()) {
              svg << " stroke-dasharray=\"" << dash_array << "\"";
              const std::string_view line_cap =
                  DashLineCap(item.style.pattern);
              if (!line_cap.empty())
                svg << " stroke-linecap=\"" << line_cap << "\"";
            }
            if (item.style.pattern != LinePattern::kSolid) {
              const Vec2 direction{item.end.x - item.start.x,
                                   item.end.y - item.start.y};
              const double length = std::hypot(direction.x, direction.y);
              if (const auto offset = CenteredDashOffset(
                      item.style.pattern, length * kMetresToMillimetres)) {
                svg << " stroke-dashoffset=\"" << *offset << "\"";
                previous_pattern_line = nullptr;
                pattern_distance_mm = 0.0;
                svg << " data-source-table=\""
                    << EscapeXml(item.source_table) << "\"/>\n";
                if (item.style.pattern == LinePattern::kChainThick &&
                    length > 0.0) {
                  constexpr double kEndLengthM = 0.010;
                  const double end_length = std::min(kEndLengthM, length / 2.0);
                  const Vec2 unit{direction.x / length, direction.y / length};
                  const Vec2 start_inner{item.start.x + unit.x * end_length,
                                         item.start.y + unit.y * end_length};
                  const Vec2 end_inner{item.end.x - unit.x * end_length,
                                       item.end.y - unit.y * end_length};
                  const double thick_width =
                      3.0 * StrokeWidthMm(item.style, sheet, options);
                  svg << "    <line x1=\""
                      << item.start.x * kMetresToMillimetres << "\" y1=\""
                      << -item.start.y * kMetresToMillimetres << "\" x2=\""
                      << start_inner.x * kMetresToMillimetres << "\" y2=\""
                      << -start_inner.y * kMetresToMillimetres
                      << "\" stroke=\"" << ColorHex(item.style.rgb)
                      << "\" stroke-width=\"" << thick_width
                      << "\" stroke-linecap=\"butt\" data-source-table=\""
                      << EscapeXml(item.source_table)
                      << "\" data-role=\"cutting-plane-end\"/>\n"
                      << "    <line x1=\""
                      << item.end.x * kMetresToMillimetres << "\" y1=\""
                      << -item.end.y * kMetresToMillimetres << "\" x2=\""
                      << end_inner.x * kMetresToMillimetres << "\" y2=\""
                      << -end_inner.y * kMetresToMillimetres
                      << "\" stroke=\"" << ColorHex(item.style.rgb)
                      << "\" stroke-width=\"" << thick_width
                      << "\" stroke-linecap=\"butt\" data-source-table=\""
                      << EscapeXml(item.source_table)
                      << "\" data-role=\"cutting-plane-end\"/>\n";
                }
                return;
              }
              bool continues = false;
              if (previous_pattern_line && length > 0.0) {
                const auto &previous = *previous_pattern_line;
                const Vec2 previous_direction{previous.end.x - previous.start.x,
                                              previous.end.y -
                                                  previous.start.y};
                continues = previous.style.pattern == item.style.pattern &&
                            previous.style.rgb == item.style.rgb &&
                            previous.source_table == item.source_table &&
                            std::hypot(previous.end.x - item.start.x,
                                       previous.end.y - item.start.y) < 1e-8 &&
                            std::abs(previous_direction.x * direction.y -
                                     previous_direction.y * direction.x) /
                                    length <
                                1e-8 &&
                            previous_direction.x * direction.x +
                                    previous_direction.y * direction.y >
                                0.0;
              }
              if (!continues)
                pattern_distance_mm = 0.0;
              // Cached centre lines can be split into pieces shorter than one
              // dash cycle. Restarting each piece loses the short dash
              // entirely.
              if (pattern_distance_mm > 0.0) {
                svg << " stroke-dashoffset=\"" << pattern_distance_mm << "\"";
              }
              pattern_distance_mm += length * kMetresToMillimetres;
              previous_pattern_line = &item;
            } else {
              previous_pattern_line = nullptr;
            }
            svg << " data-source-table=\"" << EscapeXml(item.source_table)
                << "\"/>\n";
          } else if constexpr (std::is_same_v<T, Arc2D>) {
            const double radius = std::hypot(item.start.x - item.center.x,
                                             item.start.y - item.center.y);
            const bool full_circle =
                radius > 0.0 && std::hypot(item.end.x - item.start.x,
                                           item.end.y - item.start.y) <= 1e-9;
            if (full_circle) {
              svg << "    <circle cx=\"" << item.center.x * kMetresToMillimetres
                  << "\" cy=\"" << -item.center.y * kMetresToMillimetres
                  << "\" r=\"" << radius * kMetresToMillimetres;
            } else {
              double delta = std::atan2(item.end.y - item.center.y,
                                        item.end.x - item.center.x) -
                             std::atan2(item.start.y - item.center.y,
                                        item.start.x - item.center.x);
              if (item.normal_z > 0.0) {
                while (delta < 0.0)
                  delta += 2.0 * std::numbers::pi;
              } else {
                while (delta > 0.0)
                  delta -= 2.0 * std::numbers::pi;
              }
              const int large_arc = std::abs(delta) > std::numbers::pi ? 1 : 0;
              const int sweep = item.normal_z > 0.0 ? 0 : 1;
              svg << "    <path d=\"M " << item.start.x * kMetresToMillimetres
                  << ' ' << -item.start.y * kMetresToMillimetres << " A "
                  << radius * kMetresToMillimetres << ' '
                  << radius * kMetresToMillimetres << " 0 " << large_arc << ' '
                  << sweep << ' ' << item.end.x * kMetresToMillimetres << ' '
                  << -item.end.y * kMetresToMillimetres;
            }
            svg << "\" fill=\"none\" stroke=\"" << ColorHex(item.style.rgb)
                << "\" stroke-width=\""
                << StrokeWidthMm(item.style, sheet, options)
                << "\"";
            const std::string_view dash_array = DashArray(item.style.pattern);
            if (!dash_array.empty()) {
              svg << " stroke-dasharray=\"" << dash_array << "\"";
              const std::string_view line_cap =
                  DashLineCap(item.style.pattern);
              if (!line_cap.empty())
                svg << " stroke-linecap=\"" << line_cap << "\"";
            }
            svg << " data-source-table=\"" << EscapeXml(item.source_table)
                << "\"/>\n";
          } else if constexpr (std::is_same_v<T, FilledTriangle2D>) {
            svg << "    <polygon points=\"";
            for (const Vec2 point : item.points) {
              svg << point.x * kMetresToMillimetres << ','
                  << -point.y * kMetresToMillimetres << ' ';
            }
            svg << "\" fill=\"" << ColorHex(item.rgb) << "\" stroke=\"none\"";
            if (item.opacity < 1.0) {
              svg << " fill-opacity=\"" << item.opacity << "\"";
            }
            svg << " data-source-table=\"" << EscapeXml(item.source_table)
                << "\"/>\n";
          } else if constexpr (std::is_same_v<T, RasterImage2D>) {
            const auto png = EncodePng(item);
            if (png) {
              svg << "    <image x=\""
                  << item.bottom_left.x * kMetresToMillimetres << "\" y=\""
                  << -(item.bottom_left.y + item.height) * kMetresToMillimetres
                  << "\" width=\"" << item.width * kMetresToMillimetres
                  << "\" height=\"" << item.height * kMetresToMillimetres
                  << "\" preserveAspectRatio=\"none\" "
                     "href=\"data:image/png;base64,"
                  << Base64Encode(*png) << "\" data-source-table=\""
                  << EscapeXml(item.source_table) << "\"/>\n";
            }
          } else if constexpr (std::is_same_v<T, TextAnnotation>) {
            // SolidWorks stores the cap height for ordinary drawing text,
            // while SVG font-size is an em square. Its cached runs use a
            // 3:4 cap-height-to-em ratio.
            constexpr double kCapHeightToEm = 4.0 / 3.0;
            const double font_size =
                (item.height > 0.0 ? item.height : 0.003) *
                kMetresToMillimetres *
                (item.height_is_cap_height ? kCapHeightToEm : 1.0);
            const double x = item.origin.x * kMetresToMillimetres;
            const double y = -item.origin.y * kMetresToMillimetres;
            svg << "    <text x=\"" << x << "\" y=\"" << y
                << "\" fill=\"#000000\" stroke=\"none\" font-size=\""
                << font_size << "\"";
            if (!item.font_family.empty()) {
              svg << " font-family=\"" << EscapeXml(item.font_family);
              if (item.font_family.find("sans-serif") == std::string::npos)
                svg << ", sans-serif";
              svg << "\"";
            }
            if (item.rendered_width > 0.0) {
              svg << " textLength=\""
                  << item.rendered_width * kMetresToMillimetres
                  << "\" lengthAdjust=\"spacing\"";
            }
            if (item.bold)
              svg << " font-weight=\"bold\"";
            if (item.anchor == TextAnchor::kMiddle) {
              svg << " text-anchor=\"middle\"";
            } else if (item.anchor == TextAnchor::kEnd) {
              svg << " text-anchor=\"end\"";
            }
            if (item.rotation_radians != 0.0) {
              svg << " transform=\"rotate("
                  << -item.rotation_radians * kRadiansToDegrees << ' ' << x
                  << ' ' << y << ")\"";
            }
            svg << " data-source-table=\"" << EscapeXml(item.source_table)
                << "\">" << EscapeXml(item.text) << "</text>\n";
          } else if constexpr (std::is_same_v<T, BomBalloon>) {
            const double font_size =
                (item.text_height_m > 0.0 ? item.text_height_m : 0.0035) *
                kMetresToMillimetres;
            constexpr std::string_view technical_font =
                "SWTxt, 'Century Gothic', 'DejaVu Sans', Arial, sans-serif";
            constexpr double text_gap = 2.2;
            constexpr double arrow_length = 2.4;
            constexpr double arrow_half_width = 0.8;
            const double anchor_x = item.leader_anchor.x * kMetresToMillimetres;
            const double anchor_y =
                -item.leader_anchor.y * kMetresToMillimetres;
            const double text_x = item.text_origin.x * kMetresToMillimetres;
            const double text_y = -item.text_origin.y * kMetresToMillimetres;
            const double direction_x =
                item.has_cached_leader
                    ? item.cached_leader_point.x * kMetresToMillimetres
                    : text_x;
            const double direction_y =
                item.has_cached_leader
                    ? -item.cached_leader_point.y * kMetresToMillimetres
                    : text_y;
            const double dx = direction_x - anchor_x;
            const double dy = direction_y - anchor_y;
            const double length = std::hypot(dx, dy);
            const double ux = length > 0.0 ? dx / length : 1.0;
            const double uy = length > 0.0 ? dy / length : 0.0;
            const double leader_end_x = text_x - ux * text_gap;
            const double leader_end_y = text_y - uy * text_gap;
            const double base_x = anchor_x + ux * arrow_length;
            const double base_y = anchor_y + uy * arrow_length;
            svg << "    <g stroke=\"#000000\" stroke-width=\""
                << options.default_stroke_width_mm << "\" data-source-table=\""
                << EscapeXml(item.source_table) << "\">\n";
            if (!item.has_cached_leader) {
              svg << "      <line x1=\"" << anchor_x << "\" y1=\"" << anchor_y
                  << "\" x2=\"" << leader_end_x << "\" y2=\"" << leader_end_y
                  << "\"/>\n";
            }
            svg << "      <path d=\"M " << anchor_x << ' ' << anchor_y << " L "
                << base_x - uy * arrow_half_width << ' '
                << base_y + ux * arrow_half_width << " L "
                << base_x + uy * arrow_half_width << ' '
                << base_y - ux * arrow_half_width << " Z\" fill=\"#000000\"/>\n"
                << "      <text x=\"" << text_x << "\" y=\"" << text_y
                << "\" fill=\"#000000\" stroke=\"none\" font-size=\""
                << font_size << "\" font-family=\"" << technical_font
                << "\" text-anchor=\"middle\" dominant-baseline=\"central\">"
                << EscapeXml(item.text) << "</text>\n"
                << "    </g>\n";
          } else if constexpr (std::is_same_v<T, Dimension>) {
            constexpr double font_size = 4.0;
            constexpr std::string_view technical_font =
                "osifont, 'DejaVu Sans', Arial, sans-serif";
            const double x = item.text_origin.x * kMetresToMillimetres;
            const double y = -item.text_origin.y * kMetresToMillimetres;
            const double half =
                item.paper_length_m * kMetresToMillimetres * 0.5;
            const double text_width =
                font_size * 0.62 * static_cast<double>(item.text.size());
            const double gap = std::min(text_width * 0.5 + 1.0, half * 0.4);
            const double arrow = std::min(3.0, half * 0.2);
            svg << "    <g stroke=\"#000000\" stroke-width=\""
                << options.default_stroke_width_mm << "\" data-source-table=\""
                << EscapeXml(item.source_table) << "\">\n";
            if (item.paper_length_m > 0.0 &&
                item.orientation == DimensionOrientation::kHorizontal) {
              const double left = x - half;
              const double right = x + half;
              svg << "      <line x1=\"" << left << "\" y1=\"" << y
                  << "\" x2=\"" << x - gap << "\" y2=\"" << y << "\"/>\n"
                  << "      <line x1=\"" << x + gap << "\" y1=\"" << y
                  << "\" x2=\"" << right << "\" y2=\"" << y << "\"/>\n"
                  << "      <path d=\"M " << left << ' ' << y << " L "
                  << left + arrow << ' ' << y - arrow * 0.4 << " L "
                  << left + arrow << ' ' << y + arrow * 0.4
                  << " Z\" fill=\"#000000\"/>\n"
                  << "      <path d=\"M " << right << ' ' << y << " L "
                  << right - arrow << ' ' << y - arrow * 0.4 << " L "
                  << right - arrow << ' ' << y + arrow * 0.4
                  << " Z\" fill=\"#000000\"/>\n";
            } else if (item.paper_length_m > 0.0) {
              const double top = y - half;
              const double bottom = y + half;
              svg << "      <line x1=\"" << x << "\" y1=\"" << top << "\" x2=\""
                  << x << "\" y2=\"" << y - gap << "\"/>\n"
                  << "      <line x1=\"" << x << "\" y1=\"" << y + gap
                  << "\" x2=\"" << x << "\" y2=\"" << bottom << "\"/>\n"
                  << "      <path d=\"M " << x << ' ' << top << " L "
                  << x - arrow * 0.4 << ' ' << top + arrow << " L "
                  << x + arrow * 0.4 << ' ' << top + arrow
                  << " Z\" fill=\"#000000\"/>\n"
                  << "      <path d=\"M " << x << ' ' << bottom << " L "
                  << x - arrow * 0.4 << ' ' << bottom - arrow << " L "
                  << x + arrow * 0.4 << ' ' << bottom - arrow
                  << " Z\" fill=\"#000000\"/>\n";
            }
            svg << "      <text x=\"" << x << "\" y=\"" << y
                << "\" fill=\"#000000\" stroke=\"none\" font-size=\""
                << font_size << "\" font-family=\"" << technical_font
                << "\" text-anchor=\"middle\" dominant-baseline=\"central\"";
            if (item.orientation == DimensionOrientation::kVertical) {
              svg << " transform=\"rotate(-90 " << x << ' ' << y << ")\"";
            }
            svg << ">" << EscapeXml(item.text) << "</text>\n"
                << "    </g>\n";
          }
        },
        entity);
  }
  svg << "  </g>\n</svg>\n";
  return Result<std::string>::Ok(svg.str());
}

} // namespace openswx
