// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <filesystem>
#include <span>

#include "openparasolid/result.h"
#include "openparasolid/types.h"

namespace openparasolid {

/// Reads a binary PS\0\0 prologue, including variable schema padding.
/// The input must begin with a single extracted transmit stream.
class ParasolidReader {
 public:
  [[nodiscard]] static Result<StreamHeader> ReadHeader(
      std::span<const uint8_t> data);
  /// Reads only the common signature/description/schema prologue. Useful for
  /// container extraction without requiring a complete internal header.
  [[nodiscard]] static Result<StreamHeader> ReadCommonHeader(
      std::span<const uint8_t> data);
};

/// Inspects binary transmit, partition and delta streams without a CAD kernel.
/// Reports a limited set of fixed-layout topology candidates and leaves every
/// other body byte in an uninterpreted range. No delta application, complete
/// schema decoding, or geometry/topology validity is implied by a result.
/// Calls are independent and may run concurrently on separate inputs.
class ParasolidInspector {
 public:
  [[nodiscard]] static Result<StreamInspection> Inspect(
      std::span<const uint8_t> data, const InspectOptions &options = {});
  [[nodiscard]] static Result<StreamInspection> InspectFile(
      const std::filesystem::path &path, const InspectOptions &options = {});
};

}  // namespace openparasolid
