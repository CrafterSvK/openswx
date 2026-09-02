// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_probe.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 4> kClassMarker = {0xFF, 0xFF, 0x01, 0x00};
constexpr std::array<uint8_t, 5> kSketchMarker = {
    0xFF, 0xFF, 0x1F, 0x00, 0x03};
constexpr std::array<uint8_t, 5> kLegacySketchMarker = {
    0xFF, 0xFF, 0x07, 0x00, 0x01};
constexpr std::array<uint8_t, 5> kLegacyExtendedSketchMarker = {
    0xFF, 0xFF, 0x1F, 0x00, 0x01};
constexpr uint16_t kMaximumClassNameLength = 128;

uint16_t ReadU16Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8u);
}

template <std::size_t N>
uint64_t CountMarkers(std::span<const uint8_t> data,
                      const std::array<uint8_t, N>& marker) {
  uint64_t count = 0;
  auto cursor = data.begin();
  while (cursor != data.end()) {
    cursor = std::search(cursor, data.end(), marker.begin(), marker.end());
    if (cursor == data.end()) break;
    ++count;
    ++cursor;
  }
  return count;
}

bool IsClassNameByte(uint8_t byte) {
  return std::isalnum(static_cast<unsigned char>(byte)) != 0 ||
         byte == '_' || byte == '-';
}

double ReadF64Le(std::span<const uint8_t> data, std::size_t offset) {
  std::array<uint8_t, sizeof(double)> bytes{};
  std::copy_n(data.begin() + static_cast<std::ptrdiff_t>(offset),
              bytes.size(), bytes.begin());
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  double value = 0.0;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

void AddCoordinateCandidate(DrawingBinaryProbe& probe, double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  constexpr double kMaximumPlausibleCoordinateMetres = 1000000.0;
  if (std::abs(x) > kMaximumPlausibleCoordinateMetres ||
      std::abs(y) > kMaximumPlausibleCoordinateMetres) {
    return;
  }
  ++probe.coordinate_pair_candidates;
  if (!probe.has_coordinate_bounds) {
    probe.coordinate_min_x = probe.coordinate_max_x = x;
    probe.coordinate_min_y = probe.coordinate_max_y = y;
    probe.has_coordinate_bounds = true;
    return;
  }
  probe.coordinate_min_x = std::min(probe.coordinate_min_x, x);
  probe.coordinate_min_y = std::min(probe.coordinate_min_y, y);
  probe.coordinate_max_x = std::max(probe.coordinate_max_x, x);
  probe.coordinate_max_y = std::max(probe.coordinate_max_y, y);
}

template <std::size_t N>
void ProbeCoordinates(std::span<const uint8_t> data,
                      const std::array<uint8_t, N>& marker,
                      DrawingBinaryProbe& probe) {
  auto cursor = data.begin();
  while (cursor != data.end()) {
    cursor = std::search(cursor, data.end(), marker.begin(), marker.end());
    if (cursor == data.end()) break;
    const std::size_t offset =
        static_cast<std::size_t>(cursor - data.begin());
    // Known coordinate-bearing sketch-marker layouts store tag 0x001e at
    // one of these fixed offsets, followed by two little-endian f64 values
    // in metres. This is a structural probe; it does not classify the point.
    static constexpr std::array<std::size_t, 4> kCoordinateTags = {
        56u, 64u, 42u, 48u};
    for (const std::size_t tag : kCoordinateTags) {
      if (data.size() - offset >= tag + 18u &&
          data[offset + tag] == 0x1Eu && data[offset + tag + 1u] == 0x00u) {
        AddCoordinateCandidate(probe, ReadF64Le(data, offset + tag + 2u),
                               ReadF64Le(data, offset + tag + 10u));
        break;
      }
    }
    ++cursor;
  }
}

}  // namespace

DrawingBinaryProbe ProbeDrawingBinary(std::span<const uint8_t> data) {
  DrawingBinaryProbe probe;
  probe.byte_size = data.size();
  probe.current_sketch_markers = CountMarkers(data, kSketchMarker);
  probe.legacy_sketch_markers = CountMarkers(data, kLegacySketchMarker);
  probe.legacy_extended_sketch_markers =
      CountMarkers(data, kLegacyExtendedSketchMarker);
  ProbeCoordinates(data, kSketchMarker, probe);
  ProbeCoordinates(data, kLegacySketchMarker, probe);
  ProbeCoordinates(data, kLegacyExtendedSketchMarker, probe);

  if (data.size() < kClassMarker.size() + 2u) return probe;
  for (std::size_t offset = 0;
       offset <= data.size() - kClassMarker.size() - 2u; ++offset) {
    if (!std::equal(kClassMarker.begin(), kClassMarker.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset))) {
      continue;
    }
    const std::size_t length_offset = offset + kClassMarker.size();
    const uint16_t length = ReadU16Le(data, length_offset);
    if (length == 0 || length > kMaximumClassNameLength) continue;
    const std::size_t name_offset = length_offset + 2u;
    if (name_offset > data.size() || length > data.size() - name_offset) {
      continue;
    }
    const auto name = data.subspan(name_offset, length);
    if (!std::all_of(name.begin(), name.end(), IsClassNameByte)) continue;
    probe.classes.push_back({
        static_cast<uint64_t>(offset),
        std::string(reinterpret_cast<const char*>(name.data()), name.size())});
    offset = name_offset + length - 1u;
  }
  return probe;
}

}  // namespace openswx
