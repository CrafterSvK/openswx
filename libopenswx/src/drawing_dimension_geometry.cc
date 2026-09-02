// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};
constexpr std::size_t kPositionSearchBytes = 4096u;
constexpr std::size_t kMeasurementSearchBytes = 512u;
constexpr std::size_t kOrientationSearchBytes = 400u;
constexpr double kPositionPlaneZ = -0.08295;

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

uint64_t ReadU64Le(std::span<const uint8_t> data, std::size_t offset) {
  uint64_t value = 0;
  for (std::size_t index = 0; index < 8u; ++index) {
    value |= static_cast<uint64_t>(data[offset + index]) << (index * 8u);
  }
  return value;
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

struct EncodedString {
  std::size_t offset = 0;
  std::string value;
};

std::vector<EncodedString> ReadEncodedStrings(std::span<const uint8_t> data) {
  std::vector<EncodedString> strings;
  if (data.size() < 4u) return strings;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    if (!std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const std::size_t units = data[offset + 3u];
    const std::size_t start = offset + 4u;
    if (units == 0u || units > 127u || units > (data.size() - start) / 2u) {
      offset += 2u;
      continue;
    }
    std::string value;
    value.reserve(units);
    bool valid = true;
    for (std::size_t index = 0; index < units; ++index) {
      const std::size_t character = start + index * 2u;
      const uint16_t codepoint = static_cast<uint16_t>(data[character]) |
          static_cast<uint16_t>(static_cast<uint16_t>(data[character + 1u])
                                << 8u);
      if (codepoint == 0u || (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
        valid = false;
        break;
      }
      AppendUtf8(value, codepoint);
    }
    if (valid) strings.push_back({offset, std::move(value)});
    offset += 2u;
  }
  return strings;
}

bool IsRdName(std::string_view value) {
  return value.size() >= 3u && value.starts_with("RD") &&
      std::all_of(value.begin() + 2, value.end(), [](char character) {
        return character >= '0' && character <= '9';
      });
}

bool IsUnsignedInteger(std::string_view value) {
  return !value.empty() &&
      std::all_of(value.begin(), value.end(), [](char character) {
        return character >= '0' && character <= '9';
      });
}

std::optional<Vec2> FindTextPosition(std::span<const uint8_t> data,
                                     std::size_t before) {
  const std::size_t first = before > kPositionSearchBytes
                                ? before - kPositionSearchBytes
                                : 0u;
  std::optional<Vec2> result;
  for (std::size_t offset = first; offset + 36u <= before; ++offset) {
    if (!std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset)) ||
        data[offset + 3u] != 0u) {
      continue;
    }
    const double x = ReadF64Le(data, offset + 4u);
    const double y = ReadF64Le(data, offset + 12u);
    const double z = ReadF64Le(data, offset + 20u);
    if (!std::isfinite(x) || !std::isfinite(y) ||
        x < 0.0 || x > 2.0 || y < 0.0 || y > 2.0 ||
        std::abs(z - kPositionPlaneZ) > 1e-9 ||
        ReadU64Le(data, offset + 28u) != 18u) {
      continue;
    }
    result = Vec2{x, y};
  }
  return result;
}

std::optional<double> FindMeasurement(std::span<const uint8_t> data,
                                      std::size_t rd_offset,
                                      std::size_t text_offset,
                                      int displayed_millimetres,
                                      std::size_t& value_offset) {
  const std::size_t first = std::max(
      rd_offset, text_offset > kMeasurementSearchBytes
                     ? text_offset - kMeasurementSearchBytes
                     : 0u);
  for (std::size_t offset = first; offset + 40u <= text_offset; ++offset) {
    const double value = ReadF64Le(data, offset);
    if (!std::isfinite(value) || value <= 0.0 || value > 100.0 ||
        std::llround(value * 1000.0) != displayed_millimetres) {
      continue;
    }
    if (ReadF64Le(data, offset + 8u) != 0.0 ||
        ReadF64Le(data, offset + 16u) != value ||
        ReadF64Le(data, offset + 24u) != 0.0 ||
        ReadF64Le(data, offset + 32u) != 0.0) {
      continue;
    }
    value_offset = offset;
    return value;
  }
  return std::nullopt;
}

DimensionOrientation FindOrientation(std::span<const uint8_t> data,
                                     std::size_t measurement_offset,
                                     double measurement) {
  struct Point {
    double horizontal = 0.0;
    double vertical = 0.0;
  };
  std::vector<Point> points;
  const std::size_t first = measurement_offset > kOrientationSearchBytes
                                ? measurement_offset - kOrientationSearchBytes
                                : 0u;
  for (std::size_t offset = first; offset + 24u <= measurement_offset;
       ++offset) {
    const double leading = ReadF64Le(data, offset);
    const double horizontal = ReadF64Le(data, offset + 8u);
    const double vertical = ReadF64Le(data, offset + 16u);
    if (leading != 0.0 || !std::isfinite(horizontal) ||
        !std::isfinite(vertical) || std::abs(horizontal) > 100.0 ||
        std::abs(vertical) > 100.0) {
      continue;
    }
    points.push_back({horizontal, vertical});
  }

  double horizontal_error = std::numeric_limits<double>::infinity();
  double vertical_error = std::numeric_limits<double>::infinity();
  for (std::size_t left = 0; left < points.size(); ++left) {
    for (std::size_t right = left + 1u; right < points.size(); ++right) {
      horizontal_error = std::min(
          horizontal_error,
          std::abs(std::abs(points[right].horizontal -
                            points[left].horizontal) - measurement));
      vertical_error = std::min(
          vertical_error,
          std::abs(std::abs(points[right].vertical - points[left].vertical) -
                   measurement));
    }
  }
  return vertical_error < horizontal_error
             ? DimensionOrientation::kVertical
             : DimensionOrientation::kHorizontal;
}

}  // namespace

Result<std::vector<Dimension>> DrawingGeometryReader::ReadLegacyDimensions(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<Dimension>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }
  const std::vector<EncodedString> strings = ReadEncodedStrings(definition);
  std::vector<Dimension> dimensions;
  for (std::size_t index = 0; index < strings.size(); ++index) {
    const EncodedString& rd = strings[index];
    if (!IsRdName(rd.value)) continue;

    std::string owner_sheet;
    for (std::size_t previous = index; previous > 0u; --previous) {
      if (known_sheets.contains(strings[previous - 1u].value)) {
        owner_sheet = strings[previous - 1u].value;
        break;
      }
    }
    if (owner_sheet != sheet_name) continue;

    const auto position = FindTextPosition(definition, rd.offset);
    if (!position) continue;
    const EncodedString* displayed = nullptr;
    for (std::size_t following = index + 1u; following < strings.size();
         ++following) {
      if (IsRdName(strings[following].value) ||
          known_sheets.contains(strings[following].value)) {
        break;
      }
      if (IsUnsignedInteger(strings[following].value)) {
        displayed = &strings[following];
        break;
      }
    }
    if (displayed == nullptr) continue;
    int displayed_millimetres = 0;
    const auto parsed = std::from_chars(displayed->value.data(),
                                        displayed->value.data() +
                                            displayed->value.size(),
                                        displayed_millimetres);
    if (parsed.ec != std::errc{} ||
        parsed.ptr != displayed->value.data() + displayed->value.size()) {
      continue;
    }
    std::size_t measurement_offset = 0;
    const auto measurement =
        FindMeasurement(definition, rd.offset, displayed->offset,
                        displayed_millimetres, measurement_offset);
    if (!measurement) continue;

    dimensions.push_back(Dimension{
        displayed->value, *position, *measurement, 0.0,
        FindOrientation(definition, measurement_offset, *measurement),
        static_cast<uint64_t>(rd.offset),
        "Contents/Definition/moDimBackupDef_c/" + rd.value});
  }
  return Result<std::vector<Dimension>>::Ok(std::move(dimensions));
}

}  // namespace openswx
