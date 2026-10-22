// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <cmath>
#include <utility>

#include "model_internal.h"

namespace openparasolid::internal {
namespace {
bool Direction(const Vector3& v) { return v.x != 0 || v.y != 0 || v.z != 0; }
bool Frame(Vector3 a, Vector3 b) {
  if (!Direction(a) || !Direction(b)) return false;
  const double as = std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z)});
  const double bs = std::max({std::abs(b.x), std::abs(b.y), std::abs(b.z)});
  a = {a.x / as, a.y / as, a.z / as};
  b = {b.x / bs, b.y / bs, b.z / bs};
  return a.y * b.z != a.z * b.y || a.z * b.x != a.x * b.z ||
         a.x * b.y != a.y * b.x;
}
Result<BuiltGeometry> Invalid(const DecodedNode& node) {
  return Result<BuiltGeometry>::Err(
      {ErrorCode::kInvalidRecord, node.offset,
       "Invalid analytic radius, direction or coordinate frame"});
}
template <typename T>
Result<BuiltGeometry> Built(T object, uint64_t count) {
  return Result<BuiltGeometry>::Ok({GeometryPayload{std::move(object)}, count});
}
}  // namespace
Result<BuiltGeometry> BuildAnalytic(const DecodedNode& node, ModelFields& f) {
  switch (node.node_type) {
    case 60: {
      OffsetSurfaceGeometry p;
      if (!f.Reference("surface", p.surface) || !f.Number("offset", p.distance))
        return Result<BuiltGeometry>::Err(f.Error());
      if (!p.surface.value)
        return Result<BuiltGeometry>::Err(
            {ErrorCode::kInvalidRecord, node.offset,
             "Offset surface requires a non-null basis"});
      return Built(p, 1);
    }
    case 29: {
      PointGeometry p;
      if (!f.Vector("position", p.position))
        return Result<BuiltGeometry>::Err(f.Error());
      return Built(p, 3);
    }
    case 30: {
      LineGeometry p;
      if (!f.Vector("origin", p.origin) || !f.Vector("direction", p.direction))
        return Result<BuiltGeometry>::Err(f.Error());
      if (!Direction(p.direction)) return Invalid(node);
      return Built(p, 6);
    }
    case 31: {
      CircleGeometry p;
      if (!f.Vector("center", p.center) || !f.Vector("normal", p.normal) ||
          !f.Vector("x_axis", p.x_axis) || !f.Number("radius", p.radius))
        return Result<BuiltGeometry>::Err(f.Error());
      if (p.radius <= 0 || !Frame(p.normal, p.x_axis)) return Invalid(node);
      return Built(p, 10);
    }
    case 32: {
      EllipseGeometry p;
      if (!f.Vector("center", p.center) || !f.Vector("normal", p.normal) ||
          !f.Vector("x_axis", p.x_axis) ||
          !f.Number("major_radius", p.major_radius) ||
          !f.Number("minor_radius", p.minor_radius))
        return Result<BuiltGeometry>::Err(f.Error());
      if (p.major_radius <= 0 || p.minor_radius <= 0 ||
          !Frame(p.normal, p.x_axis))
        return Invalid(node);
      return Built(p, 11);
    }
    case 50: {
      PlaneGeometry p;
      if (!f.Vector("origin", p.origin) || !f.Vector("normal", p.normal) ||
          !f.Vector("x_axis", p.x_axis))
        return Result<BuiltGeometry>::Err(f.Error());
      if (!Frame(p.normal, p.x_axis)) return Invalid(node);
      return Built(p, 9);
    }
    case 51: {
      CylinderGeometry p;
      if (!f.Vector("origin", p.origin) || !f.Vector("axis", p.axis) ||
          !f.Number("radius", p.radius) || !f.Vector("x_axis", p.x_axis))
        return Result<BuiltGeometry>::Err(f.Error());
      if (p.radius <= 0 || !Frame(p.axis, p.x_axis)) return Invalid(node);
      return Built(p, 10);
    }
    case 52: {
      ConeGeometry p;
      if (!f.Vector("origin", p.origin) || !f.Vector("axis", p.axis) ||
          !f.Number("radius", p.radius) ||
          !f.Number("sin_half_angle", p.sin_half_angle) ||
          !f.Number("cos_half_angle", p.cos_half_angle) ||
          !f.Vector("x_axis", p.x_axis))
        return Result<BuiltGeometry>::Err(f.Error());
      if (p.radius < 0 || std::abs(p.sin_half_angle) > 1 ||
          std::abs(p.cos_half_angle) > 1 || !Frame(p.axis, p.x_axis))
        return Invalid(node);
      return Built(p, 12);
    }
    case 53: {
      SphereGeometry p;
      if (!f.Vector("center", p.center) || !f.Number("radius", p.radius) ||
          !f.Vector("axis", p.axis) || !f.Vector("x_axis", p.x_axis))
        return Result<BuiltGeometry>::Err(f.Error());
      if (p.radius <= 0 || !Frame(p.axis, p.x_axis)) return Invalid(node);
      return Built(p, 10);
    }
    case 54: {
      TorusGeometry p;
      if (!f.Vector("center", p.center) || !f.Vector("axis", p.axis) ||
          !f.Number("major_radius", p.major_radius) ||
          !f.Number("minor_radius", p.minor_radius) ||
          !f.Vector("x_axis", p.x_axis))
        return Result<BuiltGeometry>::Err(f.Error());
      // XT distinguishes the inner lemon surface by a negative major radius.
      if (p.major_radius == 0 || p.minor_radius <= 0 ||
          (p.major_radius < 0 && -p.major_radius >= p.minor_radius) ||
          !Frame(p.axis, p.x_axis))
        return Invalid(node);
      return Built(p, 11);
    }
    default:
      return Result<BuiltGeometry>::Err(
          {ErrorCode::kUnsupportedFormat, node.offset,
           "Typed geometry is not implemented for node type " +
               std::to_string(node.node_type)});
  }
}
}  // namespace openparasolid::internal
