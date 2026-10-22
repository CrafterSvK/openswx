// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_BINARY_READER_H_
#define OPENPARASOLID_BINARY_READER_H_

#include <bit>
#include <span>
#include <string>
#include <utility>

#include "openparasolid/result.h"

namespace openparasolid::internal {
// Reads big-endian values and reports the first failed operation explicitly.
// A failed read never advances past the available input.
class Cursor {
 public:
  std::span<const uint8_t> data;
  uint64_t pos;
  [[nodiscard]] Result<uint64_t> Read(unsigned width) {
    if (width > sizeof(uint64_t))
      return Result<uint64_t>::Err(
          {ErrorCode::kInvalidOptions, pos, "Invalid binary value width"});
    if (pos > data.size() || width > data.size() - pos)
      return Result<uint64_t>::Err(
          {ErrorCode::kInvalidRecord, pos, "Truncated binary value"});
    uint64_t value = 0;
    for (unsigned i = 0; i < width; ++i) value = (value << 8u) | data[pos++];
    return Result<uint64_t>::Ok(value);
  }
  [[nodiscard]] Result<uint32_t> Index() {
    const auto offset = pos;
    const auto first = Read(2);
    if (!first.ok()) return Result<uint32_t>::Err(first.error());
    const int32_t remainder =
        std::bit_cast<int16_t>(static_cast<uint16_t>(first.value()));
    if (remainder > 0)
      return Result<uint32_t>::Ok(static_cast<uint32_t>(remainder - 1));
    if (remainder == 0 || remainder == -32768)
      return Result<uint32_t>::Err({ErrorCode::kInvalidRecord, offset,
                                    "Invalid compact index remainder"});
    const auto second = Read(2);
    if (!second.ok()) return Result<uint32_t>::Err(second.error());
    const int32_t quotient =
        std::bit_cast<int16_t>(static_cast<uint16_t>(second.value()));
    if (quotient <= 0)
      return Result<uint32_t>::Err({ErrorCode::kInvalidRecord, offset,
                                    "Invalid compact index quotient"});
    return Result<uint32_t>::Ok(
        static_cast<uint32_t>(quotient * 32767 - remainder - 1));
  }
  [[nodiscard]] Result<std::string> Text(bool empty = false) {
    const auto offset = pos;
    const auto count = Read(1);
    if (!count.ok()) return Result<std::string>::Err(count.error());
    if (!empty && count.value() == 0)
      return Result<std::string>::Err(
          {ErrorCode::kInvalidSchema, offset, "Empty schema name"});
    std::string text;
    for (uint64_t i = 0; i < count.value(); ++i) {
      const auto ch = Read(1);
      if (!ch.ok()) return Result<std::string>::Err(ch.error());
      if (ch.value() < 32 || ch.value() > 126)
        return Result<std::string>::Err(
            {ErrorCode::kInvalidSchema, pos - 1, "Non-ASCII schema text"});
      text.push_back(static_cast<char>(ch.value()));
    }
    return Result<std::string>::Ok(std::move(text));
  }
  [[nodiscard]] Result<bool> Logical() {
    const auto value = Read(1);
    if (!value.ok()) return Result<bool>::Err(value.error());
    if (value.value() > 1)
      return Result<bool>::Err(
          {ErrorCode::kInvalidRecord, pos - 1, "Invalid logical value"});
    return Result<bool>::Ok(value.value() != 0);
  }
};
}  // namespace openparasolid::internal
#endif  // OPENPARASOLID_BINARY_READER_H_
