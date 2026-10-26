// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SWEEP_BOUNDS_H_
#define OPENSWSTEP_SRC_SWEEP_BOUNDS_H_
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include "openparasolid/model.h"
namespace openswstep::internal::sweep_bounds {
constexpr auto kInfinity = std::numeric_limits<long double>::infinity();
struct Interval {
  long double low = 0, high = 0;
};
inline Interval Exact(long double x) { return {x, x}; }
inline bool Valid(Interval x) {
  return std::isfinite(x.low) && std::isfinite(x.high) && x.low <= x.high;
}
inline Interval Enclose(long double low, long double high) {
  return {std::nextafter(low, -kInfinity), std::nextafter(high, kInfinity)};
}
inline Interval Unbounded() { return {-kInfinity, kInfinity}; }
inline Interval Add(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  return Enclose(a.low + b.low, a.high + b.high);
}
inline Interval Sub(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  return Enclose(a.low - b.high, a.high - b.low);
}
inline Interval Mul(Interval a, Interval b) {
  if (!Valid(a) || !Valid(b)) return Unbounded();
  const std::array<long double, 4> v{a.low * b.low, a.low * b.high,
                                     a.high * b.low, a.high * b.high};
  for (auto x : v)
    if (!std::isfinite(x)) return Unbounded();
  return Enclose(*std::min_element(v.begin(), v.end()),
                 *std::max_element(v.begin(), v.end()));
}
inline Interval Div(Interval a, Interval b) {
  if (b.high < 0) return Div(Mul(a, Exact(-1)), Mul(b, Exact(-1)));
  if (!Valid(a) || !Valid(b) || b.low <= 0) return Unbounded();
  return Mul(a, Enclose(1 / b.high, 1 / b.low));
}
using Vector = std::array<Interval, 3>;
inline Vector Vec(openparasolid::Vector3 p) {
  return {Exact(p.x), Exact(p.y), Exact(p.z)};
}
inline Vector Diff(openparasolid::Vector3 a, openparasolid::Vector3 b) {
  return {Sub(Exact(a.x), Exact(b.x)), Sub(Exact(a.y), Exact(b.y)),
          Sub(Exact(a.z), Exact(b.z))};
}
inline Vector Cross(Vector a, Vector b) {
  Vector out{};
  for (unsigned k = 0; k < 3; ++k)
    out[k] = Sub(Mul(a[(k + 1) % 3], b[(k + 2) % 3]),
                 Mul(a[(k + 2) % 3], b[(k + 1) % 3]));
  return out;
}
inline Interval Dot(Vector a, Vector b) {
  Interval out{};
  for (unsigned k = 0; k < 3; ++k) out = Add(out, Mul(a[k], b[k]));
  return out;
}
inline long double Length(Vector a) {
  long double sum = 0;
  for (auto v : a) {
    if (!Valid(v)) return kInfinity;
    const auto x = std::max(std::abs(v.low), std::abs(v.high));
    sum = std::nextafter(sum + std::nextafter(x * x, kInfinity), kInfinity);
  }
  return std::nextafter(std::sqrt(sum), kInfinity);
}
inline bool Spend(uint64_t& remaining, uint64_t count) {
  if (count > remaining) return false;
  remaining -= count;
  return true;
}
}  // namespace openswstep::internal::sweep_bounds
#endif  // OPENSWSTEP_SRC_SWEEP_BOUNDS_H_
