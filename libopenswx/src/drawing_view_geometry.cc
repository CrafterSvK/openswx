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

namespace openswx {
namespace {

constexpr std::array<uint8_t, 4> kClassMarker = {0xFF, 0xFF, 0x01, 0x00};
constexpr std::array<uint8_t, 2> kRepeatedClassToken = {0x01, 0x80};
constexpr std::array<uint8_t, 7> kCachedSegmentMarkerSuffix = {
    0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
constexpr std::string_view kViewBucketClass = "uiViewBucket_c";

bool HasCachedSegmentMarkerSuffix(std::span<const uint8_t> data,
                                  std::size_t offset) {
  if (offset >= data.size() || data.size() - offset < 8u)
    return false;
  return std::equal(kCachedSegmentMarkerSuffix.begin(),
                    kCachedSegmentMarkerSuffix.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset + 1u));
}

bool IsCachedSegmentMarker(std::span<const uint8_t> data, std::size_t offset) {
  if (!HasCachedSegmentMarkerSuffix(data, offset))
    return false;
  const uint8_t record_type = data[offset];
  // The high 0x20 bit distinguishes active display edges. The otherwise
  // identical 0x01/0x03 records form a suppressed cache in shaded views and
  // must not be emitted as dashed wireframe geometry.
  return record_type == 0x21u || record_type == 0x23u;
}

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

bool ValidBucketHeader(std::span<const uint8_t> data, std::size_t offset) {
  if (offset > data.size() || data.size() - offset < 42u)
    return false;
  const double x = ReadF64Le(data, offset);
  const double y = ReadF64Le(data, offset + 8u);
  const double z = ReadF64Le(data, offset + 16u);
  const double scale = ReadF64Le(data, offset + 32u);
  const double norm = x * x + y * y + z * z;
  return std::isfinite(norm) && std::abs(norm - 1.0) < 1e-8 &&
         std::isfinite(scale) && scale > 0.0 && scale < 1.0e6;
}

void AppendUtf8(std::string &output, uint32_t codepoint) {
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

std::optional<std::string> FindViewReference(std::span<const uint8_t> data) {
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    if (data[offset] != 0xFFu || data[offset + 1u] != 0xFEu ||
        data[offset + 2u] != 0xFFu) {
      continue;
    }
    const std::size_t units = data[offset + 3u];
    const std::size_t start = offset + 4u;
    if (units == 0u || units > 127u || start > data.size() ||
        units > (data.size() - start) / 2u) {
      continue;
    }
    std::string value;
    value.reserve(units);
    bool valid = true;
    for (std::size_t index = 0; index < units; ++index) {
      const uint16_t codepoint = ReadU16Le(data, start + index * 2u);
      if (codepoint == 0u || (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
        valid = false;
        break;
      }
      AppendUtf8(value, codepoint);
    }
    if (valid && value.find('@') != std::string::npos)
      return value;
  }
  return std::nullopt;
}

Vec3 Cross(Vec3 left, Vec3 right) {
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

Vec3 Normalize(Vec3 value) {
  const double length =
      std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  return {value.x / length, value.y / length, value.z / length};
}

double Dot(Vec3 left, Vec3 right) {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec2 Project(Vec3 point, Vec3 horizontal, Vec3 vertical, double scale) {
  return {Dot(point, horizontal) / scale, Dot(point, vertical) / scale};
}

} // namespace

Result<std::vector<CachedDrawingView>>
DrawingGeometryReader::ReadViewBuckets(std::span<const uint8_t> data) {
  if (data.size() < 24u ||
      !std::equal(kClassMarker.begin(), kClassMarker.end(), data.begin() + 4)) {
    return Result<std::vector<CachedDrawingView>>::Err(
        "VBLists does not start with a class array");
  }
  const uint16_t class_length = ReadU16Le(data, 8u);
  if (class_length != kViewBucketClass.size() ||
      10u + class_length > data.size() ||
      !std::equal(kViewBucketClass.begin(), kViewBucketClass.end(),
                  data.begin() + 10)) {
    return Result<std::vector<CachedDrawingView>>::Err(
        "VBLists does not declare uiViewBucket_c first");
  }

  std::vector<std::size_t> starts = {10u + class_length};
  for (std::size_t offset = starts.front(); offset + 44u <= data.size();
       ++offset) {
    if (data[offset] == kRepeatedClassToken[0] &&
        data[offset + 1u] == kRepeatedClassToken[1] &&
        ValidBucketHeader(data, offset + 2u)) {
      starts.push_back(offset + 2u);
      ++offset;
    }
  }
  if (!ValidBucketHeader(data, starts.front())) {
    return Result<std::vector<CachedDrawingView>>::Err(
        "First uiViewBucket_c header is invalid");
  }

  std::size_t bucket_section_end = data.size();
  for (std::size_t offset = starts.back() + 42u;
       offset + kClassMarker.size() + 2u <= data.size(); ++offset) {
    if (std::equal(kClassMarker.begin(), kClassMarker.end(),
                   data.begin() + static_cast<std::ptrdiff_t>(offset))) {
      bucket_section_end = offset;
      break;
    }
  }

  std::vector<CachedDrawingView> views;
  views.reserve(starts.size());
  for (std::size_t index = 0; index < starts.size(); ++index) {
    const std::size_t start = starts[index];
    const std::size_t end = index + 1u < starts.size() ? starts[index + 1u] - 2u
                                                       : bucket_section_end;
    if (end <= start + 42u)
      continue;

    CachedDrawingView view;
    view.source_offset = start;
    view.direction = {ReadF64Le(data, start), ReadF64Le(data, start + 8u),
                      ReadF64Le(data, start + 16u)};
    view.scale_denominator = ReadF64Le(data, start + 32u);
    const auto reference = FindViewReference(data.subspan(start, end - start));
    if (reference) {
      const std::size_t separator = reference->find('@');
      view.referenced_document = reference->substr(0, separator);
      view.name = reference->substr(separator + 1u);
    } else {
      view.name = "view@" + std::to_string(start);
    }
    view.geometry.name = view.name;

    const Vec3 preferred_up = std::abs(view.direction.z) > 0.95
                                  ? Vec3{0.0, 1.0, 0.0}
                                  : Vec3{0.0, 0.0, 1.0};
    const Vec3 horizontal = Normalize(Cross(preferred_up, view.direction));
    const Vec3 vertical = Normalize(Cross(view.direction, horizontal));
    // Shared topological edges are serialized once per adjacent face. The
    // two copies can differ by sub-nanometre floating-point noise.
    constexpr double kDuplicateCoordinateToleranceM = 1e-8;
    std::set<std::array<int64_t, 6>> emitted_segments;

    for (std::size_t offset = start + 42u; offset + 8u + 48u <= end; ++offset) {
      if (HasCachedSegmentMarkerSuffix(data, offset) &&
          (data[offset] == 0x01u || data[offset] == 0x03u)) {
        view.has_suppressed_edge_cache = true;
        continue;
      }
      if (!IsCachedSegmentMarker(data, offset))
        continue;
      const std::size_t coordinates = offset + 8u;
      const Vec3 first{ReadF64Le(data, coordinates),
                       ReadF64Le(data, coordinates + 8u),
                       ReadF64Le(data, coordinates + 16u)};
      const Vec3 second{ReadF64Le(data, coordinates + 24u),
                        ReadF64Le(data, coordinates + 32u),
                        ReadF64Le(data, coordinates + 40u)};
      const std::array<double, 6> values = {first.x,  first.y,  first.z,
                                            second.x, second.y, second.z};
      if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && std::abs(value) <= 1.0e6;
          })) {
        continue;
      }
      const auto quantize = [](double value) {
        return static_cast<int64_t>(
            std::llround(value / kDuplicateCoordinateToleranceM));
      };
      std::array<int64_t, 3> first_key = {quantize(first.x), quantize(first.y),
                                          quantize(first.z)};
      std::array<int64_t, 3> second_key = {
          quantize(second.x), quantize(second.y), quantize(second.z)};
      if (second_key < first_key)
        std::swap(first_key, second_key);
      const std::array<int64_t, 6> segment_key = {first_key[0],  first_key[1],
                                                  first_key[2],  second_key[0],
                                                  second_key[1], second_key[2]};
      if (!emitted_segments.insert(segment_key).second) {
        offset += 8u + 48u - 1u;
        continue;
      }
      view.geometry.entities.emplace_back(
          Line2D{Project(first, horizontal, vertical, view.scale_denominator),
                 Project(second, horizontal, vertical, view.scale_denominator),
                 LineStyle{}, "Contents/VBLists/uiViewBucket_c"});
      view.source_lines.push_back(
          {first, second, static_cast<uint64_t>(offset), data[offset]});
      offset += 8u + 48u - 1u;
    }
    if (!view.geometry.entities.empty())
      views.push_back(std::move(view));
  }
  if (views.empty()) {
    return Result<std::vector<CachedDrawingView>>::Err(
        "No confirmed cached segment records found in uiViewBucket_c");
  }
  return Result<std::vector<CachedDrawingView>>::Ok(std::move(views));
}

} // namespace openswx
