// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWX_SRC_SAVED_GEOMETRY_HISTORY_H_
#define OPENSWX_SRC_SAVED_GEOMETRY_HISTORY_H_

#include <optional>
#include <vector>

#include "openparasolid/schema.h"
#include "openswx/saved_geometry.h"

namespace openswx::internal {
struct SavedHistoryData {
  bool complete = false;
  std::optional<openparasolid::DeltaPrefix> prefix;
  std::vector<openparasolid::DeltaMark> marks;
};
void InspectSavedState(SavedGeometryResolution& result,
                       const std::vector<SavedHistoryData>& histories);
}  // namespace openswx::internal
#endif  // OPENSWX_SRC_SAVED_GEOMETRY_HISTORY_H_
