// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

/**
 * @file
 * @brief Binary prologue reading and conservative stream inspection.
 * @see @ref xt_header "XT 3.1: Common header (p. 19)", @ref xt_binary "XT 3.3:
 * Binary (p. 22)"
 */
#ifndef OPENPARASOLID_PARSER_H_
#define OPENPARASOLID_PARSER_H_

#include <filesystem>
#include <span>

#include "openparasolid/result.h"
#include "openparasolid/types.h"

namespace openparasolid {

/// @brief Reads supported binary stream headers without decoding model nodes.
///
/// Reads a binary PS\0\0 prologue, including variable schema padding.
/// The input must begin with a single extracted transmit stream.
///
/// @see @ref xt_header "XT 3.1: Common header (p. 19)", @ref xt_binary "XT 3.3:
/// Binary (p. 22)"
class ParasolidReader {
 public:
  /// @brief Reads the supported internal header after the common prologue.
  ///
  /// @see @ref xt_header "XT 3.1: Common header (p. 19)", @ref xt_binary
  /// "XT 3.3: Binary (p. 22)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<StreamHeader> ReadHeader(
      std::span<const uint8_t> data);
  /// Reads only the common signature/description/schema prologue. Useful for
  /// container extraction without requiring a complete internal header.
  ///
  /// @see @ref xt_header "XT 3.1: Common header (p. 19)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<StreamHeader> ReadCommonHeader(
      std::span<const uint8_t> data);
};

/// @brief Conservatively scans extracted binary streams without a CAD kernel.
///
/// Inspects binary transmit, partition and delta streams without a CAD kernel.
/// Reports a limited set of fixed-layout topology candidates and leaves every
/// other body byte in an uninterpreted range. No delta application, complete
/// schema decoding, or geometry/topology validity is implied by a result.
/// Calls are independent and may run concurrently on separate inputs.
///
/// @see @ref xt_binary "XT 3.3: Binary (p. 22)", @ref xt_topology "XT 4.1:
/// Topology (p. 25)"
class ParasolidInspector {
 public:
  /// @brief Inspects one input span using the supplied per-call budgets.
  ///
  /// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
  ///
  /// @param data Borrowed bytes beginning at the extracted stream prologue.
  /// @param options Per-call settings and resource limits; see the associated
  /// options type.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<StreamInspection> Inspect(
      std::span<const uint8_t> data, const InspectOptions &options = {});
  /// @brief Reads and inspects a file subject to the input-size budget.
  ///
  /// @see @ref xt_binary "XT 3.3: Binary (p. 22)"
  ///
  /// @param path Input file to read under the same limits as the span overload.
  /// @param options Per-call settings and resource limits; see the associated
  /// options type.
  /// @return The requested value on success, or an explicit ParseError on
  /// failure.
  [[nodiscard]] static Result<StreamInspection> InspectFile(
      const std::filesystem::path &path, const InspectOptions &options = {});
};

}  // namespace openparasolid

#endif  // OPENPARASOLID_PARSER_H_
