// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace openparasolid::test {
inline void U16(std::vector<uint8_t> &out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value >> 8u));
  out.push_back(static_cast<uint8_t>(value));
}
inline void U32(std::vector<uint8_t> &out, uint32_t value) {
  for (int shift : {24, 16, 8, 0})
    out.push_back(static_cast<uint8_t>(value >> shift));
}
inline void F64(std::vector<uint8_t> &out, double value) {
  const auto bits = std::bit_cast<uint64_t>(value);
  for (int shift = 56; shift >= 0; shift -= 8)
    out.push_back(static_cast<uint8_t>(bits >> shift));
}
inline std::vector<uint8_t> Header(
    std::string_view form = "",
    std::string_view schema = "SCH_1234567_12345_13006",
    std::span<const uint8_t> padding = {}) {
  const std::string description = ": TRANSMIT FILE " + std::string(form) +
                                  "created by modeller version 1234567";
  std::vector<uint8_t> out{'P', 'S', 0, 0};
  U16(out, static_cast<uint16_t>(description.size()));
  out.insert(out.end(), description.begin(), description.end());
  out.insert(out.end(), padding.begin(), padding.end());
  out.push_back(static_cast<uint8_t>(schema.size()));
  out.insert(out.end(), schema.begin(), schema.end());
  if (schema == "SCH_1234567_12345_13006" || schema == "SCH_1_2_13006") {
    U16(out, 204u);
    U32(out, 0u);
  } else if (schema == "SCH_1300000_13006") {
    U32(out, 0u);
  }
  return out;
}
inline void CompactIndex(std::vector<uint8_t> &out, uint32_t value) {
  if (value < 32767)
    U16(out, static_cast<uint16_t>(value + 1));
  else {
    U16(out, static_cast<uint16_t>(-static_cast<int32_t>(value % 32767 + 1)));
    U16(out, static_cast<uint16_t>(value / 32767));
  }
}
inline std::vector<uint8_t> DeltaPrefixBytes() {
  auto out = Header("(deltas) ");
  U16(out, 3);
  out.push_back(255);
  CompactIndex(out, 1);
  CompactIndex(out, 2);
  U32(out, 10);
  U16(out, 4);
  out.push_back(255);
  CompactIndex(out, 2);
  for (int i = 0; i < 4; ++i) CompactIndex(out, 0);
  for (int i = 0; i < 4; ++i) U32(out, 0);
  out.push_back(0);
  U32(out, 1);
  return out;
}
inline void Marker(std::vector<uint8_t> &out) {
  out.insert(out.end(), {0xc2, 0xbc, 0x92, 0x8f, 0x99, 0x6e, 0, 0});
}
inline void Refs(std::vector<uint8_t> &out, std::span<const uint16_t> values) {
  for (auto value : values) U16(out, value);
}
inline void Tag(std::vector<uint8_t> &out, uint8_t tag,
                bool schema_marker = false) {
  out.insert(out.end(), {0, tag});
  if (schema_marker) out.push_back(0xff);
}
inline void Point(std::vector<uint8_t> &out, uint16_t id = 60,
                  std::array<double, 3> xyz = {.1, .2, .3},
                  bool schema_marker = false) {
  Tag(out, 0x1d, schema_marker);
  U16(out, id);
  U32(out, 1);
  Refs(out, std::array<uint16_t, 4>{0, 0, 0, 0});
  for (auto value : xyz) F64(out, value);
}
inline std::vector<uint8_t> LinkedRecords() {
  auto out = Header("(partition) ");
  Tag(out, 0x0e);
  U16(out, 10);
  U32(out, 1);
  U16(out, 0);
  Marker(out);
  Refs(out, std::array<uint16_t, 5>{0, 0, 20, 0, 100});
  out.push_back('+');
  out.resize(out.size() + 10u);
  Tag(out, 0x0f);
  U16(out, 20);
  U32(out, 0);
  Refs(out, std::array<uint16_t, 4>{0, 30, 10, 0});
  Tag(out, 0x10);
  U16(out, 40);
  U32(out, 2);
  U16(out, 0);
  Marker(out);
  Refs(out, std::array<uint16_t, 6>{30, 0, 0, 110, 0, 0});
  Tag(out, 0x11);
  U16(out, 30);
  Refs(out, std::array<uint16_t, 9>{0, 20, 30, 30, 50, 0, 40, 0, 0});
  out.push_back('+');
  Tag(out, 0x12);
  U16(out, 50);
  U32(out, 3);
  Refs(out, std::array<uint16_t, 5>{0, 0, 0, 0, 60});
  Marker(out);
  Point(out);
  return out;
}
}  // namespace openparasolid::test
