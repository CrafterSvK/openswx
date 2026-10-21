// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace openswx {

struct BinaryClassDeclaration {
  uint64_t offset = 0;
  std::string name;
};

struct DrawingBinaryProbe {
  uint64_t byte_size = 0;
  uint64_t current_sketch_markers = 0;
  uint64_t legacy_sketch_markers = 0;
  uint64_t legacy_extended_sketch_markers = 0;
  uint64_t coordinate_pair_candidates = 0;
  bool has_coordinate_bounds = false;
  double coordinate_min_x = 0.0;
  double coordinate_min_y = 0.0;
  double coordinate_max_x = 0.0;
  double coordinate_max_y = 0.0;
  std::vector<BinaryClassDeclaration> classes;
};

/// Performs a structure-only scan of a decoded SolidWorks binary stream.
/// It intentionally does not assign geometric meaning to marker payloads.
[[nodiscard]] DrawingBinaryProbe ProbeDrawingBinary(
    std::span<const uint8_t> data);

}  // namespace openswx
