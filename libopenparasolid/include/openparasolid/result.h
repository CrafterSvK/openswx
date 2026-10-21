// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <variant>

namespace openparasolid {

enum class ErrorCode {
  kIo,
  kInvalidHeader,
  kUnsupportedFormat,
  kResourceLimit,
  kInvalidOptions,
  kInvalidSchema,
  kInvalidRecord,
  kUnsupportedSchema
};

struct ParseError {
  ErrorCode code = ErrorCode::kInvalidHeader;
  uint64_t offset = 0;
  std::string message;
};

/// Lightweight value-or-error type, following the openswx Result API.
template <typename T>
class [[nodiscard]] Result {
 public:
  [[nodiscard]] static Result Ok(T value) {
    return Result(std::in_place_index<0>, std::move(value));
  }
  [[nodiscard]] static Result Err(ParseError error) {
    return Result(std::in_place_index<1>, std::move(error));
  }
  [[nodiscard]] bool ok() const noexcept { return data_.index() == 0; }
  [[nodiscard]] const T &value() const & { return std::get<0>(data_); }
  [[nodiscard]] T &&value() && { return std::get<0>(std::move(data_)); }
  [[nodiscard]] const ParseError &error() const { return std::get<1>(data_); }

 private:
  template <std::size_t I, typename... Args>
  explicit Result(std::in_place_index_t<I> tag, Args &&...args)
      : data_(tag, std::forward<Args>(args)...) {}
  std::variant<T, ParseError> data_;
};

[[nodiscard]] const char *ToString(ErrorCode code) noexcept;

}  // namespace openparasolid
