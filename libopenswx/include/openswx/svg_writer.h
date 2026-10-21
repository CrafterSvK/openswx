// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <string>

#include "openswx/drawing_geometry.h"

namespace openswx {

struct SvgOptions {
  double margin_mm = 2.0;
  double default_stroke_width_mm = 0.18;
};

class SvgWriter {
 public:
  [[nodiscard]] static Result<std::string> WriteSheet(
      const DrawingSheetGeometry& sheet,
      const SvgOptions& options = {});
};

}  // namespace openswx
