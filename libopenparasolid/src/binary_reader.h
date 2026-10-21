// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <bit>
#include <span>
#include <string>
#include <utility>

#include "openparasolid/result.h"

namespace openparasolid::internal {
[[noreturn]] inline void Fail(ErrorCode code, uint64_t offset,
                              std::string message) {
  throw ParseError{code, offset, std::move(message)};
}
class Cursor {
 public:
  std::span<const uint8_t> data;
  uint64_t pos;
  uint64_t Read(unsigned width) {
    if (pos > data.size() || width > data.size() - pos)
      Fail(ErrorCode::kInvalidRecord, pos, "Truncated binary value");
    uint64_t value = 0;
    for (unsigned i = 0; i < width; ++i) value = (value << 8u) | data[pos++];
    return value;
  }
  uint32_t Index() {
    const auto offset = pos;
    const int32_t remainder =
        std::bit_cast<int16_t>(static_cast<uint16_t>(Read(2)));
    if (remainder > 0) return static_cast<uint32_t>(remainder - 1);
    if (remainder == 0 || remainder == -32768)
      Fail(ErrorCode::kInvalidRecord, offset,
           "Invalid compact index remainder");
    const int32_t quotient =
        std::bit_cast<int16_t>(static_cast<uint16_t>(Read(2)));
    if (quotient <= 0)
      Fail(ErrorCode::kInvalidRecord, offset, "Invalid compact index quotient");
    return static_cast<uint32_t>(quotient * 32767 - remainder - 1);
  }
  std::string Text(bool empty = false) {
    const auto offset = pos;
    const auto size = Read(1);
    if (!empty && size == 0)
      Fail(ErrorCode::kInvalidSchema, offset, "Empty schema name");
    std::string text;
    for (uint64_t i = 0; i < size; ++i) {
      const auto ch = Read(1);
      if (ch < 32 || ch > 126)
        Fail(ErrorCode::kInvalidSchema, pos - 1, "Non-ASCII schema text");
      text.push_back(static_cast<char>(ch));
    }
    return text;
  }
  bool Logical() {
    const auto value = Read(1);
    if (value > 1)
      Fail(ErrorCode::kInvalidRecord, pos - 1, "Invalid logical value");
    return value != 0;
  }
};
}  // namespace openparasolid::internal
