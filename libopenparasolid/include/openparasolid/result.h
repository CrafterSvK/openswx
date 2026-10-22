// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

/**
 * @file
 * @brief Explicit value-or-error results for the public API.
 * @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
 */
#ifndef OPENPARASOLID_RESULT_H_
#define OPENPARASOLID_RESULT_H_

#include <cstdint>
#include <string>
#include <utility>
#include <variant>

namespace openparasolid {

/// @brief Library failure categories, not transmitted XT status codes.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
enum class ErrorCode {
  /// Input/output failure.
  kIo,
  /// Malformed or incomplete supported prologue.
  kInvalidHeader,
  /// Recognized request or data whose semantics are unsupported.
  kUnsupportedFormat,
  /// Explicit work, storage or numeric representation bound exceeded.
  kResourceLimit,
  /// Invalid caller parameters or resource limits.
  kInvalidOptions,
  /// Malformed schema definition or inconsistent catalog.
  kInvalidSchema,
  /// Invalid record, connectivity, geometry or evaluation result.
  kInvalidRecord,
  /// Schema layout is not supported by this operation.
  kUnsupportedSchema,
};

/// @brief Failure with an input-relative byte offset when one is available.
///
/// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
struct ParseError {
  /// Library failure category.
  ErrorCode code = ErrorCode::kInvalidHeader;
  /// Byte offset relative to the beginning of the inspected input.
  uint64_t offset = 0;
  /// Human-readable diagnostic explanation.
  std::string message;
};

/// @brief Owned success value or ParseError; check ok() before accessing
/// either.
///
/// Lightweight value-or-error type, following the openswx Result API.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
template <typename T>
class [[nodiscard]] Result {
 public:
  /// @brief Constructs an owned success result.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @param value Success value moved into this result.
  /// @return Owned result containing the supplied value.
  [[nodiscard]] static Result Ok(T value) {
    return Result(std::in_place_index<0>, std::move(value));
  }
  /// @brief Constructs an owned error result.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @param error Failure moved into this result.
  /// @return Owned result containing the supplied error.
  [[nodiscard]] static Result Err(ParseError error) {
    return Result(std::in_place_index<1>, std::move(error));
  }
  /// @brief Reports whether this result contains a success value.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @return True for a success value, false for an error.
  [[nodiscard]] bool ok() const noexcept { return data_.index() == 0; }
  /// @brief Returns the success value. @pre ok() is true.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @return Borrowed const value or movable value reference, according to the
  /// overload.
  [[nodiscard]] const T &value() const & { return std::get<0>(data_); }
  /// @brief Returns the success value. @pre ok() is true.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @return Borrowed const value or movable value reference, according to the
  /// overload.
  [[nodiscard]] T &&value() && { return std::get<0>(std::move(data_)); }
  /// @brief Returns the error. @pre ok() is false.
  ///
  /// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p.
  /// 7)"
  ///
  /// @return Borrowed error, valid until this result is moved, assigned or
  /// destroyed.
  [[nodiscard]] const ParseError &error() const { return std::get<1>(data_); }

 private:
  template <std::size_t I, typename... Args>
  explicit Result(std::in_place_index_t<I> tag, Args &&...args)
      : data_(tag, std::forward<Args>(args)...) {}
  std::variant<T, ParseError> data_;
};

/// @brief Returns a static diagnostic name; the caller does not own the string.
///
/// @see @ref xt_terminology "XT 1.1.1: Parasolid XT format terminology (p. 7)"
///
/// @param code Enum value to describe.
/// @return Static diagnostic name, including "unknown" for unrecognized enum
/// values.
[[nodiscard]] const char *ToString(ErrorCode code) noexcept;

}  // namespace openparasolid

#endif  // OPENPARASOLID_RESULT_H_
