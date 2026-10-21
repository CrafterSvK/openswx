// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "openswx/drawing_probe.h"

namespace openswx {
namespace {

TEST(DrawingProbeTest, FindsClassesAndSketchMarkerFamilies) {
  std::vector<uint8_t> data = {
      0xFF, 0xFF, 0x01, 0x00, 0x08, 0x00,
      's',  'g',  'S',  'k',  'e',  't',  'c',  'h',
      0xFF, 0xFF, 0x1F, 0x00, 0x03,
      0xFF, 0xFF, 0x07, 0x00, 0x01,
      0xFF, 0xFF, 0x1F, 0x00, 0x01,
  };
  data.resize(14u + 74u, 0);
  const std::size_t marker = 14u;
  data[marker + 56u] = 0x1E;
  const double x = 0.25;
  const double y = -0.5;
  std::memcpy(data.data() + marker + 58u, &x, sizeof(x));
  std::memcpy(data.data() + marker + 66u, &y, sizeof(y));

  const DrawingBinaryProbe probe = ProbeDrawingBinary(data);

  EXPECT_EQ(probe.byte_size, data.size());
  EXPECT_EQ(probe.current_sketch_markers, 1u);
  EXPECT_EQ(probe.legacy_sketch_markers, 1u);
  EXPECT_EQ(probe.legacy_extended_sketch_markers, 1u);
  EXPECT_EQ(probe.coordinate_pair_candidates, 1u);
  ASSERT_TRUE(probe.has_coordinate_bounds);
  EXPECT_DOUBLE_EQ(probe.coordinate_min_x, x);
  EXPECT_DOUBLE_EQ(probe.coordinate_max_y, y);
  ASSERT_EQ(probe.classes.size(), 1u);
  EXPECT_EQ(probe.classes[0].offset, 0u);
  EXPECT_EQ(probe.classes[0].name, "sgSketch");
}

TEST(DrawingProbeTest, IgnoresInvalidOrTruncatedClassDeclarations) {
  const std::vector<uint8_t> data = {
      0xFF, 0xFF, 0x01, 0x00, 0x03, 0x00, 'a', '/', 'b',
      0xFF, 0xFF, 0x01, 0x00, 0x20, 0x00, 'x',
  };
  EXPECT_TRUE(ProbeDrawingBinary(data).classes.empty());
}

}  // namespace
}  // namespace openswx
