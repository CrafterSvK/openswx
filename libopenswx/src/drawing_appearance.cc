// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <string_view>

namespace openswx {
namespace {

constexpr std::string_view kVisualPropertiesClass = "moVisualProperties_c";
constexpr std::string_view kAmbientLightClass = "moAmbientLight_c";
constexpr std::string_view kDirectionLightClass = "moDirectionLight_c";
constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFFu, 0xFEu, 0xFFu};
constexpr std::array<uint8_t, 4> kClassMarker = {0xFFu, 0xFFu, 0x01u, 0x00u};

uint32_t ReadU32Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1u]) << 8u) |
         (static_cast<uint32_t>(data[offset + 2u]) << 16u) |
         (static_cast<uint32_t>(data[offset + 3u]) << 24u);
}

uint32_t ColorRefToRgb(uint32_t packed) {
  return ((packed & 0x0000FFu) << 16u) | (packed & 0x00FF00u) |
         ((packed & 0xFF0000u) >> 16u);
}

bool Has(std::span<const uint8_t> data, std::size_t offset,
         std::size_t length) {
  return offset <= data.size() && length <= data.size() - offset;
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

bool AppendUtf8(std::string& output, uint32_t code_point) {
  if (code_point <= 0x7Fu) {
    output.push_back(static_cast<char>(code_point));
  } else if (code_point <= 0x7FFu) {
    output.push_back(static_cast<char>(0xC0u | (code_point >> 6u)));
    output.push_back(static_cast<char>(0x80u | (code_point & 0x3Fu)));
  } else if (code_point <= 0xFFFFu) {
    output.push_back(static_cast<char>(0xE0u | (code_point >> 12u)));
    output.push_back(
        static_cast<char>(0x80u | ((code_point >> 6u) & 0x3Fu)));
    output.push_back(static_cast<char>(0x80u | (code_point & 0x3Fu)));
  } else if (code_point <= 0x10FFFFu) {
    output.push_back(static_cast<char>(0xF0u | (code_point >> 18u)));
    output.push_back(
        static_cast<char>(0x80u | ((code_point >> 12u) & 0x3Fu)));
    output.push_back(
        static_cast<char>(0x80u | ((code_point >> 6u) & 0x3Fu)));
    output.push_back(static_cast<char>(0x80u | (code_point & 0x3Fu)));
  } else {
    return false;
  }
  return true;
}

std::optional<std::string> ReadUtf16Name(
    std::span<const uint8_t> data, std::size_t marker_offset) {
  if (!Has(data, marker_offset, 4u) ||
      !std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                  data.begin() + static_cast<std::ptrdiff_t>(marker_offset))) {
    return std::nullopt;
  }
  const std::size_t count = data[marker_offset + 3u];
  const std::size_t start = marker_offset + 4u;
  if (count == 0u || count > 127u || !Has(data, start, count * 2u)) {
    return std::nullopt;
  }
  std::string name;
  name.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const uint16_t unit = static_cast<uint16_t>(data[start + index * 2u]) |
                          (static_cast<uint16_t>(
                               data[start + index * 2u + 1u])
                           << 8u);
    uint32_t code_point = unit;
    if (unit >= 0xD800u && unit <= 0xDBFFu) {
      if (++index >= count) return std::nullopt;
      const uint16_t low =
          static_cast<uint16_t>(data[start + index * 2u]) |
          (static_cast<uint16_t>(data[start + index * 2u + 1u]) << 8u);
      if (low < 0xDC00u || low > 0xDFFFu) return std::nullopt;
      code_point = 0x10000u +
                   ((static_cast<uint32_t>(unit) - 0xD800u) << 10u) +
                   (static_cast<uint32_t>(low) - 0xDC00u);
    } else if (unit >= 0xDC00u && unit <= 0xDFFFu) {
      return std::nullopt;
    }
    if (code_point < 0x20u || !AppendUtf8(name, code_point))
      return std::nullopt;
  }
  return name;
}

std::optional<DrawingSceneLight> ParseLightRecord(
    std::span<const uint8_t> data, std::size_t payload_offset,
    std::size_t source_offset, DrawingLightKind kind) {
  constexpr std::size_t kNameOffset = 44u;
  constexpr std::size_t kDirectionOffset = 128u;
  const std::size_t required = kind == DrawingLightKind::kDirectional
                                   ? kDirectionOffset + 24u
                                   : kNameOffset + 4u;
  if (!Has(data, payload_offset, required)) return std::nullopt;
  const Vec3 coefficients{ReadF64Le(data, payload_offset),
                          ReadF64Le(data, payload_offset + 8u),
                          ReadF64Le(data, payload_offset + 16u)};
  if (!std::isfinite(coefficients.x) || !std::isfinite(coefficients.y) ||
      !std::isfinite(coefficients.z) || std::abs(coefficients.x) > 10.0 ||
      std::abs(coefficients.y) > 10.0 || std::abs(coefficients.z) > 10.0) {
    return std::nullopt;
  }
  auto name = ReadUtf16Name(data, payload_offset + kNameOffset);
  if (!name) return std::nullopt;
  DrawingSceneLight light;
  light.kind = kind;
  light.name = std::move(*name);
  light.ambient = coefficients.x;
  light.diffuse = coefficients.y;
  light.specular = coefficients.z;
  light.rgb = ColorRefToRgb(ReadU32Le(data, payload_offset + 24u));
  light.enabled = ReadU32Le(data, payload_offset + 40u) != 0u;
  light.source_offset = source_offset;
  if (kind == DrawingLightKind::kDirectional) {
    light.direction = {ReadF64Le(data, payload_offset + kDirectionOffset),
                       ReadF64Le(data, payload_offset + kDirectionOffset + 8u),
                       ReadF64Le(data, payload_offset + kDirectionOffset + 16u)};
    const double length_squared =
        light.direction.x * light.direction.x +
        light.direction.y * light.direction.y +
        light.direction.z * light.direction.z;
    if (!std::isfinite(length_squared) || length_squared < 1e-20 ||
        length_squared > 1e12) {
      return std::nullopt;
    }
  }
  return light;
}

}  // namespace

std::vector<DrawingAppearance> DrawingGeometryReader::ReadAppearances(
    std::span<const uint8_t> data) {
  std::vector<DrawingAppearance> appearances;
  const auto begin = data.begin();
  auto cursor = begin;
  while (cursor != data.end()) {
    cursor = std::search(cursor, data.end(), kVisualPropertiesClass.begin(),
                         kVisualPropertiesClass.end());
    if (cursor == data.end()) break;
    const std::size_t class_offset =
        static_cast<std::size_t>(cursor - begin);
    const std::size_t packed_offset =
        class_offset + kVisualPropertiesClass.size();
    // Two confirmed serializer variants place the framed UTF-16LE name after
    // either 8 or 12 uninterpreted bytes following packed RGB. SLDDRW 19000
    // uses +12; newer CADMPEG fixtures use +16. Requiring the complete framed
    // name still rejects incidental class-name strings.
    auto name = ReadUtf16Name(data, packed_offset + 12u);
    if (!name) name = ReadUtf16Name(data, packed_offset + 16u);
    if (Has(data, packed_offset, 20u) && name) {
      appearances.push_back(DrawingAppearance{
          *name, ColorRefToRgb(ReadU32Le(data, packed_offset)), 1.0,
          class_offset, DrawingAppearanceScope::kClassDefinition});
    }
    cursor += static_cast<std::ptrdiff_t>(kVisualPropertiesClass.size());
  }

  // Body- and face-local records omit the class name and start with 33 80.
  // As with class definitions, observed serializers use two prefix lengths.
  for (std::size_t offset = 0u; offset + 18u <= data.size(); ++offset) {
    if (data[offset] != 0x33u || data[offset + 1u] != 0x80u) continue;
    auto name = ReadUtf16Name(data, offset + 14u);
    if (!name) name = ReadUtf16Name(data, offset + 18u);
    if (!name) continue;
    appearances.push_back(DrawingAppearance{
        *name, ColorRefToRgb(ReadU32Le(data, offset + 2u)), 1.0,
        offset, DrawingAppearanceScope::kInlineUnresolved});
  }
  std::sort(appearances.begin(), appearances.end(),
            [](const DrawingAppearance& left,
               const DrawingAppearance& right) {
              return left.source_offset < right.source_offset;
            });
  return appearances;
}

std::vector<DrawingSceneLight> DrawingGeometryReader::ReadSceneLights(
    std::span<const uint8_t> data) {
  std::vector<DrawingSceneLight> lights;
  for (const auto [class_name, kind] :
       std::array<std::pair<std::string_view, DrawingLightKind>, 2>{
           {{kAmbientLightClass, DrawingLightKind::kAmbient},
            {kDirectionLightClass, DrawingLightKind::kDirectional}}}) {
    auto cursor = data.begin();
    while (cursor != data.end()) {
      cursor = std::search(cursor, data.end(), class_name.begin(),
                           class_name.end());
      if (cursor == data.end()) break;
      const std::size_t class_offset =
          static_cast<std::size_t>(cursor - data.begin());
      const std::size_t payload_offset = class_offset + class_name.size();
      const auto next_class =
          std::search(data.begin() + static_cast<std::ptrdiff_t>(payload_offset),
                      data.end(), kClassMarker.begin(), kClassMarker.end());
      const std::size_t limit = static_cast<std::size_t>(next_class - data.begin());
      if (auto light = ParseLightRecord(data, payload_offset, class_offset, kind)) {
        lights.push_back(std::move(*light));
      }
      if (kind == DrawingLightKind::kDirectional) {
        for (std::size_t offset = payload_offset;
             offset + 2u <= limit; ++offset) {
          if (data[offset] != 0x07u || data[offset + 1u] != 0x80u) continue;
          if (auto light = ParseLightRecord(data, offset + 2u, offset, kind)) {
            lights.push_back(std::move(*light));
          }
        }
      }
      cursor += static_cast<std::ptrdiff_t>(class_name.size());
    }
  }
  std::sort(lights.begin(), lights.end(),
            [](const DrawingSceneLight& left, const DrawingSceneLight& right) {
              return left.source_offset < right.source_offset;
            });
  return lights;
}

}  // namespace openswx
