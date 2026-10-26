// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "strip_pcurve.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include "nurbs_wire.h"
#include "sweep_bounds.h"
namespace openswstep::internal {
namespace {
namespace bounds = sweep_bounds;
using namespace openparasolid;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
}  // namespace
Result<NurbsCurveGeometry> BuildRationalStripPcurve(
    const SplineSweepPatch& patch, const NurbsCurveGeometry& boundary,
    double tolerance, uint64_t& checks) {
  using Return = Result<NurbsCurveGeometry>;
  const auto& source = patch.profile;
  const auto p = source.axis.degree;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0, "Invalid strip pcurve tolerance"});
  if (!source.controls.rational || source.controls.dimension != 4 ||
      (p != 2 && p != 3) || source.axis.knots.size() != 2 ||
      source.axis.control_count != p + 1 || !boundary.controls.rational ||
      boundary.controls.dimension != 4 || boundary.axis.degree != p)
    return Return::Err(Unsupported(
        "Strip pcurve requires matching single-span rational profiles"));
  const auto first = source.axis.knots.front(), last = source.axis.knots.back();
  const auto validated = RestrictNurbsWire(source, first, last, checks);
  if (!validated.ok()) return Return::Err(validated.error());
  const auto selected = RestrictNurbsWire(boundary, first, last, checks);
  if (!selected.ok()) return Return::Err(selected.error());
  const auto& edge = selected.value();
  if (edge.axis.knots != source.axis.knots ||
      edge.axis.multiplicities != source.axis.multiplicities ||
      edge.axis.control_count != source.axis.control_count)
    return Return::Err(Unsupported("Strip pcurve bases disagree"));
  if (!bounds::Spend(checks, 256 * (p + 2)))
    return Return::Err(
        {ErrorCode::kResourceLimit, 0, "Strip pcurve work budget exhausted"});
  const auto direction = bounds::Vec(patch.sweep);
  const auto norm2 = bounds::Dot(direction, direction);
  if (!bounds::Valid(norm2) || norm2.low <= 0)
    return Return::Err(Unsupported("Strip pcurve sweep is singular"));
  long double scale = 0;
  for (unsigned i = 0; i <= p; ++i)
    scale = std::max(
        scale, static_cast<long double>(source.controls.values[4 * i + 3]));
  std::array<bounds::Interval, 4> weights{}, height{};
  std::array<bounds::Vector, 4> cartesian{};
  long double minimum_weight = bounds::kInfinity, maximum_weight = 0,
              maximum_height = 0, membership_error = 0;
  for (unsigned i = 0; i <= p; ++i) {
    if (source.controls.values[4 * i + 3] != edge.controls.values[4 * i + 3])
      return Return::Err(Unsupported("Strip pcurve weights disagree"));
    weights[i] = bounds::Div(bounds::Exact(source.controls.values[4 * i + 3]),
                             bounds::Exact(scale));
    minimum_weight = std::min(minimum_weight, weights[i].low);
    maximum_weight = std::max(maximum_weight, weights[i].high);
    bounds::Vector delta{};
    for (unsigned k = 0; k < 3; ++k) {
      delta[k] = bounds::Div(
          bounds::Sub(bounds::Exact(edge.controls.values[4 * i + k]),
                      bounds::Exact(source.controls.values[4 * i + k])),
          bounds::Exact(scale));
      cartesian[i][k] =
          bounds::Div(bounds::Exact(source.controls.values[4 * i + k]),
                      bounds::Exact(source.controls.values[4 * i + 3]));
    }
    height[i] = bounds::Div(bounds::Dot(delta, direction), norm2);
    const auto v = bounds::Div(height[i], weights[i]);
    if (!bounds::Valid(v))
      return Return::Err(
          Unsupported("Strip pcurve ordinate is unrepresentable"));
    maximum_height =
        std::max(maximum_height, std::max(std::abs(v.low), std::abs(v.high)));
    bounds::Vector residual{};
    for (unsigned k = 0; k < 3; ++k)
      residual[k] = bounds::Div(
          bounds::Sub(delta[k], bounds::Mul(height[i], direction[k])),
          weights[i]);
    membership_error = std::max(membership_error, bounds::Length(residual));
  }
  NurbsCurveGeometry out;
  out.axis.degree = static_cast<uint16_t>(p + 1);
  out.axis.control_count = p + 2;
  out.axis.knots = {first, last};
  out.axis.multiplicities = {static_cast<uint16_t>(p + 2),
                             static_cast<uint16_t>(p + 2)};
  out.controls.rational = true;
  out.controls.dimension = 3;
  const auto width = bounds::Sub(bounds::Exact(last), bounds::Exact(first));
  long double min_output_weight = bounds::kInfinity;
  std::array<long double, 3> perturbation{};
  for (unsigned i = 0; i <= p + 1; ++i) {
    const auto a = bounds::Div(bounds::Exact(p + 1 - i), bounds::Exact(p + 1)),
               b = bounds::Div(bounds::Exact(i), bounds::Exact(p + 1));
    bounds::Interval w{}, v{};
    if (i <= p) {
      w = bounds::Mul(a, weights[i]);
      v = bounds::Mul(a, height[i]);
    }
    if (i) {
      w = bounds::Add(w, bounds::Mul(b, weights[i - 1]));
      v = bounds::Add(v, bounds::Mul(b, height[i - 1]));
    }
    auto u = bounds::Mul(bounds::Exact(first), w);
    if (i)
      u = bounds::Add(u, bounds::Mul(width, bounds::Mul(b, weights[i - 1])));
    const std::array<bounds::Interval, 3> exact{u, v, w};
    std::array<double, 3> rounded{};
    for (unsigned k = 0; k < 3; ++k) {
      if (!bounds::Valid(exact[k]))
        return Return::Err(
            Unsupported("Strip pcurve controls are unrepresentable"));
      rounded[k] =
          static_cast<double>(std::midpoint(exact[k].low, exact[k].high));
      out.controls.values.push_back(rounded[k]);
    }
    if (!std::isfinite(rounded[2]) || rounded[2] <= 0)
      return Return::Err(
          Unsupported("Strip pcurve output weight is not positive"));
    min_output_weight =
        std::min(min_output_weight, static_cast<long double>(rounded[2]));
    for (unsigned k = 0; k < 3; ++k) {
      auto emitted = bounds::Exact(rounded[2]);
      if (k < 2) {
        const auto point = static_cast<double>(
            static_cast<long double>(rounded[k]) / rounded[2]);
        emitted = bounds::Mul(bounds::Exact(point), bounds::Exact(rounded[2]));
      }
      const auto difference = bounds::Sub(exact[k], emitted);
      perturbation[k] = std::max(
          perturbation[k],
          std::max(std::abs(difference.low), std::abs(difference.high)));
    }
  }
  const auto u_error = bounds::Div(
      bounds::Add(
          bounds::Exact(perturbation[0]),
          bounds::Mul(bounds::Exact(std::max(std::abs(first), std::abs(last))),
                      bounds::Exact(perturbation[2]))),
      bounds::Exact(min_output_weight));
  const auto v_error =
      bounds::Div(bounds::Add(bounds::Exact(perturbation[1]),
                              bounds::Mul(bounds::Exact(maximum_height),
                                          bounds::Exact(perturbation[2]))),
                  bounds::Exact(min_output_weight));
  long double radius = 0;
  for (unsigned i = 0; i <= p; ++i) {
    bounds::Vector difference{};
    for (unsigned k = 0; k < 3; ++k)
      difference[k] = bounds::Sub(cartesian[i][k], cartesian[0][k]);
    radius = std::max(radius, bounds::Length(difference));
  }
  // Translation gives |A|<=Wmax*radius, |A'|<=2p*Wmax*radius,
  // |W'|<=p*Wmax. Four p leaves outward slack for the rational quotient.
  const auto speed = bounds::Div(
      bounds::Mul(bounds::Exact(4 * p * radius),
                  bounds::Mul(bounds::Exact(maximum_weight),
                              bounds::Exact(maximum_weight))),
      bounds::Mul(width, bounds::Mul(bounds::Exact(minimum_weight),
                                     bounds::Exact(minimum_weight))));
  const auto lifted = bounds::Add(
      bounds::Exact(membership_error),
      bounds::Add(
          bounds::Mul(speed, u_error),
          bounds::Mul(bounds::Exact(bounds::Length(direction)), v_error)));
  if (!bounds::Valid(lifted) || lifted.high > tolerance / 2)
    return Return::Err(
        Unsupported("Strip pcurve lift exceeds the source allowance"));
  return Return::Ok(std::move(out));
}
}  // namespace openswstep::internal
