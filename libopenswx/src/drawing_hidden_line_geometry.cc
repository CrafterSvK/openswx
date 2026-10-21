// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace openswx {
namespace {

struct ViewPoint {
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;
};

struct ParameterInterval {
  double start = 0.0;
  double end = 1.0;
};

double ColumnDot(const std::array<double, 9> &matrix, std::size_t column,
                 Vec3 point) {
  return matrix[column] * point.x + matrix[3u + column] * point.y +
         matrix[6u + column] * point.z;
}

ViewPoint Project(const std::array<double, 9> &matrix, Vec3 point) {
  return {ColumnDot(matrix, 0u, point), ColumnDot(matrix, 1u, point),
          ColumnDot(matrix, 2u, point)};
}

bool ClipLinearLowerBound(double value_at_start, double value_at_end,
                          double lower_bound, ParameterInterval &interval) {
  constexpr double kLinearTolerance = 1e-18;
  const double delta = value_at_end - value_at_start;
  if (std::abs(delta) < kLinearTolerance) {
    return value_at_start >= lower_bound;
  }
  const double crossing = (lower_bound - value_at_start) / delta;
  if (delta > 0.0) {
    interval.start = std::max(interval.start, crossing);
  } else {
    interval.end = std::min(interval.end, crossing);
  }
  return interval.start < interval.end;
}

} // namespace

std::vector<LineVisibilitySpan> DrawingGeometryReader::SplitLineByTessellation(
    const Line3D &line, std::span<const FaceTessellation> faces,
    const std::array<double, 9> &model_to_view) {
  constexpr double kBarycentricTolerance = 1e-9;
  constexpr double kDepthToleranceM = 1e-7;
  constexpr double kParameterMergeTolerance = 1e-8;
  const ViewPoint line_start = Project(model_to_view, line.start);
  const ViewPoint line_end = Project(model_to_view, line.end);
  std::vector<ParameterInterval> hidden_intervals;

  for (const FaceTessellation &face : faces) {
    for (const std::array<uint32_t, 3> &indices : face.triangles) {
      if (indices[0] >= face.vertices.size() ||
          indices[1] >= face.vertices.size() ||
          indices[2] >= face.vertices.size()) {
        continue;
      }
      const ViewPoint a = Project(model_to_view, face.vertices[indices[0]]);
      const ViewPoint b = Project(model_to_view, face.vertices[indices[1]]);
      const ViewPoint c = Project(model_to_view, face.vertices[indices[2]]);
      const double denominator =
          (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
      if (std::abs(denominator) < 1e-18)
        continue;
      const auto weights = [&](ViewPoint point) {
        const double weight_a =
            ((b.y - c.y) * (point.x - c.x) + (c.x - b.x) * (point.y - c.y)) /
            denominator;
        const double weight_b =
            ((c.y - a.y) * (point.x - c.x) + (a.x - c.x) * (point.y - c.y)) /
            denominator;
        return std::array<double, 3>{weight_a, weight_b,
                                     1.0 - weight_a - weight_b};
      };
      const std::array<double, 3> start_weights = weights(line_start);
      const std::array<double, 3> end_weights = weights(line_end);
      ParameterInterval interval;
      bool intersects = true;
      for (std::size_t index = 0u; index < 3u; ++index) {
        if (!ClipLinearLowerBound(start_weights[index], end_weights[index],
                                  -kBarycentricTolerance, interval)) {
          intersects = false;
          break;
        }
      }
      if (!intersects)
        continue;

      const double face_depth_at_start = start_weights[0] * a.depth +
                                         start_weights[1] * b.depth +
                                         start_weights[2] * c.depth;
      const double face_depth_at_end = end_weights[0] * a.depth +
                                       end_weights[1] * b.depth +
                                       end_weights[2] * c.depth;
      const double depth_difference_at_start =
          face_depth_at_start - line_start.depth;
      const double depth_difference_at_end = face_depth_at_end - line_end.depth;
      if (!ClipLinearLowerBound(depth_difference_at_start,
                                depth_difference_at_end, kDepthToleranceM,
                                interval)) {
        continue;
      }
      interval.start = std::clamp(interval.start, 0.0, 1.0);
      interval.end = std::clamp(interval.end, 0.0, 1.0);
      if (interval.end - interval.start > kParameterMergeTolerance) {
        hidden_intervals.push_back(interval);
      }
    }
  }

  if (hidden_intervals.empty())
    return {{0.0, 1.0, false}};
  std::sort(hidden_intervals.begin(), hidden_intervals.end(),
            [](const ParameterInterval &left, const ParameterInterval &right) {
              return left.start < right.start;
            });
  std::vector<ParameterInterval> merged;
  for (const ParameterInterval interval : hidden_intervals) {
    if (merged.empty() ||
        interval.start > merged.back().end + kParameterMergeTolerance) {
      merged.push_back(interval);
    } else {
      merged.back().end = std::max(merged.back().end, interval.end);
    }
  }

  std::vector<LineVisibilitySpan> spans;
  double cursor = 0.0;
  for (const ParameterInterval interval : merged) {
    if (interval.start > cursor + kParameterMergeTolerance) {
      spans.push_back({cursor, interval.start, false});
    }
    spans.push_back({std::max(cursor, interval.start), interval.end, true});
    cursor = interval.end;
  }
  if (cursor < 1.0 - kParameterMergeTolerance) {
    spans.push_back({cursor, 1.0, false});
  }
  return spans;
}

bool DrawingGeometryReader::IsLineHiddenByTessellation(
    const Line3D &line, std::span<const FaceTessellation> faces,
    const std::array<double, 9> &model_to_view) {
  double hidden_fraction = 0.0;
  for (const LineVisibilitySpan span :
       SplitLineByTessellation(line, faces, model_to_view)) {
    if (span.hidden)
      hidden_fraction += span.end_fraction - span.start_fraction;
  }
  return hidden_fraction >= 0.5;
}

} // namespace openswx
