// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};
constexpr std::string_view kMagnetLineClass = "moDisplayMagnetLine_c";
constexpr std::size_t kAnchorSearchBytes = 4096u;
constexpr std::size_t kRepeatedAnchorSearchBytes = 2048u;
constexpr std::size_t kTextHeightOffsetAfterString = 82u;
constexpr std::size_t kTextCompanionMetricOffsetAfterString = 90u;

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

bool IsUnsignedInteger(std::string_view value) {
  return !value.empty() && value.size() <= 6u &&
      std::all_of(value.begin(), value.end(), [](char character) {
        return character >= '0' && character <= '9';
      });
}

struct EncodedString {
  std::size_t offset = 0;
  std::size_t end = 0;
  std::string value;
};

std::vector<EncodedString> ReadAsciiUtf16Strings(
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
      const uint8_t low = data[start + index * 2u];
      const uint8_t high = data[start + index * 2u + 1u];
      if (high != 0u || low == 0u || low > 0x7Fu) {
        valid = false;
        break;
      }
      value.push_back(static_cast<char>(low));
    }
    if (valid) strings.push_back({offset, start + units * 2u,
                                  std::move(value)});
  }
  return strings;
}

bool IsSheetPoint(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x >= 1.0e-4 && x <= 2.0 && y >= 1.0e-4 && y <= 2.0 &&
         std::abs(z) <= 2.0;
}

double ReadBalloonTextHeight(std::span<const uint8_t> definition,
                             std::size_t string_end) {
  if (string_end > definition.size() ||
      definition.size() - string_end <
          kTextCompanionMetricOffsetAfterString + 8u) {
    return 0.0;
  }
  const double height =
      ReadF64Le(definition, string_end + kTextHeightOffsetAfterString);
  const double companion_metric = ReadF64Le(
      definition, string_end + kTextCompanionMetricOffsetAfterString);
  if (!std::isfinite(height) || !std::isfinite(companion_metric) ||
      height < 0.001 || height > 0.02 || companion_metric < 0.001 ||
      companion_metric > 0.05) {
    return 0.0;
  }
  return height;
}

bool SameBytes(std::span<const uint8_t> data, std::size_t left,
               std::size_t right, std::size_t count) {
  return left <= data.size() && right <= data.size() &&
         count <= data.size() - left && count <= data.size() - right &&
         std::equal(data.begin() + static_cast<std::ptrdiff_t>(left),
                    data.begin() + static_cast<std::ptrdiff_t>(left + count),
                    data.begin() + static_cast<std::ptrdiff_t>(right));
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

}  // namespace

Result<std::vector<BomBalloon>>
DrawingGeometryReader::ReadLegacyBomBalloons(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names,
    std::string_view sheet_name) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<BomBalloon>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }
  const std::vector<EncodedString> strings =
      ReadAsciiUtf16Strings(definition);
  for (auto marker = std::search(definition.begin(), definition.end(),
                                 kMagnetLineClass.begin(),
                                 kMagnetLineClass.end());
       marker != definition.end();
       marker = std::search(marker + 1, definition.end(),
                            kMagnetLineClass.begin(),
                            kMagnetLineClass.end())) {
    const std::size_t offset = static_cast<std::size_t>(
        std::distance(definition.begin(), marker));
    if (OwnerSheetAt(strings, known_sheets, offset) == sheet_name) {
      return Result<std::vector<BomBalloon>>::Ok({});
    }
  }
  std::vector<BomBalloon> balloons;
  for (std::size_t index = 0; index < strings.size(); ++index) {
    const EncodedString& candidate = strings[index];
    if (!IsUnsignedInteger(candidate.value) ||
        candidate.end > definition.size() ||
        definition.size() - candidate.end < 24u) {
      continue;
    }
    if (OwnerSheetAt(strings, known_sheets, candidate.offset) != sheet_name) {
      continue;
    }
    const double text_height = ReadBalloonTextHeight(definition, candidate.end);
    if (text_height == 0.0)
      continue;
    const double text_x = ReadF64Le(definition, candidate.end);
    const double text_y = ReadF64Le(definition, candidate.end + 8u);
    const double plane_z = ReadF64Le(definition, candidate.end + 16u);
    if (!IsSheetPoint(text_x, text_y, plane_z)) continue;

    const std::size_t first = candidate.offset > kAnchorSearchBytes
                                  ? candidate.offset - kAnchorSearchBytes
                                  : 0u;
    const std::size_t repeated_first = candidate.end + 24u;
    const std::size_t repeated_end = std::min(
        definition.size(), repeated_first + kRepeatedAnchorSearchBytes);
    std::size_t anchor_offset = definition.size();
    for (std::size_t offset = first; offset + 24u <= candidate.offset;
         ++offset) {
      const double anchor_x = ReadF64Le(definition, offset);
      const double anchor_y = ReadF64Le(definition, offset + 8u);
      const double anchor_z = ReadF64Le(definition, offset + 16u);
      if (!IsSheetPoint(anchor_x, anchor_y, anchor_z) ||
          std::abs(anchor_z - plane_z) > 1e-12) {
        continue;
      }
      const double distance = std::hypot(text_x - anchor_x,
                                         text_y - anchor_y);
      if (distance < 0.002 || distance > 0.25) continue;
      bool repeated = false;
      for (std::size_t after = repeated_first; after + 24u <= repeated_end;
           ++after) {
        if (SameBytes(definition, offset, after, 24u)) {
          repeated = true;
          break;
        }
      }
      if (repeated) anchor_offset = offset;
    }
    if (anchor_offset == definition.size()) continue;

    balloons.push_back(BomBalloon{candidate.value,
                                  {text_x, text_y},
                                  {ReadF64Le(definition, anchor_offset),
                                   ReadF64Le(definition, anchor_offset + 8u)},
                                  static_cast<uint64_t>(candidate.offset),
                                  "Contents/Definition/moDisplayBalloonNote_c",
                                  false,
                                  {},
                                  text_height});
  }
  return Result<std::vector<BomBalloon>>::Ok(std::move(balloons));
}

}  // namespace openswx
