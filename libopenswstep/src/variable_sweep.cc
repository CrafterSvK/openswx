// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include "nurbs_wire.h"
#include "openparasolid/evaluate.h"
#include "spline_sweep.h"
#include "sweep_bounds.h"
namespace openswstep::internal {
namespace {
using namespace openparasolid;
namespace bounds = sweep_bounds;
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Variable sweep proof budget exhausted"};
}
Vector3 Pole(const NurbsCurveGeometry& c, unsigned i) {
  return {c.controls.values[3 * i], c.controls.values[3 * i + 1],
          c.controls.values[3 * i + 2]};
}
bool SameBasis(const NurbsCurveGeometry& a, const NurbsCurveGeometry& b) {
  return !b.controls.rational && b.controls.dimension == 3 &&
         a.axis.degree == b.axis.degree &&
         a.axis.control_count == b.axis.control_count &&
         a.axis.knots == b.axis.knots &&
         a.axis.multiplicities == b.axis.multiplicities && !b.axis.closed &&
         !b.axis.periodic;
}
unsigned Choose(unsigned n, unsigned k) {
  unsigned out = 1;
  for (unsigned i = 1; i <= k; ++i) out = out * (n + 1 - i) / i;
  return out;
}
}  // namespace
Result<SplineSweepPatch> RecognizeSplineSweepStrip(
    const Model& model, NodeId surface, std::span<const PlanarCurve> curves,
    bool natural_sense, double tolerance, uint64_t& checks) {
  using Return = Result<SplineSweepPatch>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0, "Invalid variable sweep tolerance"});
  const auto* node = model.FindNode(surface);
  const auto* swept = node && node->geometry
                          ? std::get_if<SweptSurfaceGeometry>(
                                &model.geometry()[*node->geometry].payload)
                          : nullptr;
  const auto* profile_node = swept ? model.FindNode(swept->section) : nullptr;
  const auto* section =
      profile_node && profile_node->geometry
          ? std::get_if<NurbsCurveGeometry>(
                &model.geometry()[*profile_node->geometry].payload)
          : nullptr;
  if (!section || section->controls.rational ||
      section->controls.dimension != 3 ||
      (section->axis.degree != 2 && section->axis.degree != 3) ||
      section->axis.closed || section->axis.periodic ||
      section->axis.knots.size() != 2 || curves.size() != 4)
    return Return::Err(
        Unsupported("Variable sweep strips require one polynomial "
                    "quadratic/cubic span and four edges"));
  std::array<const NurbsCurveGeometry*, 4> profiles{};
  unsigned count = 0;
  double first = 0, last = 0;
  for (unsigned i = 0; i < 4; ++i) {
    if (!curves[i].geometry || !std::isfinite(curves[i].start) ||
        !std::isfinite(curves[i].end) || curves[i].start == curves[i].end)
      return Return::Err(
          Unsupported("Variable sweep boundary interval is invalid"));
    profiles[i] = std::get_if<NurbsCurveGeometry>(curves[i].geometry);
    if (profiles[i]) {
      const auto a = std::min(curves[i].start, curves[i].end),
                 b = std::max(curves[i].start, curves[i].end);
      if (!SameBasis(*section, *profiles[i]) ||
          (count && (a != first || b != last)))
        return Return::Err(
            Unsupported("Variable sweep profiles require matching original "
                        "bases and intervals"));
      first = a;
      last = b;
      ++count;
    } else if (!std::holds_alternative<LineGeometry>(*curves[i].geometry))
      return Return::Err(
          Unsupported("Variable sweep strip needs polynomial profiles and "
                      "straight generators"));
  }
  if (count != 2 || (profiles[0] != nullptr) == (profiles[1] != nullptr) ||
      (profiles[0] != nullptr) != (profiles[2] != nullptr))
    return Return::Err(Unsupported(
        "Variable sweep boundary must alternate profiles and generators"));
  const auto restricted = RestrictNurbsWire(*section, first, last, checks);
  if (!restricted.ok()) return Return::Err(restricted.error());
  SplineSweepPatch result;
  result.profile = restricted.value();
  result.sweep = swept->sweep;
  const auto sample = EvaluateSurface(model, surface, first, 0);
  if (!sample.ok()) return Return::Err(sample.error());
  const auto inverse =
      InvertSurface(model, surface, sample.value().position, tolerance);
  if (!inverse.ok()) return Return::Err(inverse.error());
  const auto direction = bounds::Vec(swept->sweep);
  const auto norm2 = bounds::Dot(direction, direction);
  if (!bounds::Valid(norm2) || norm2.low <= 0)
    return Return::Err(Unsupported("Variable sweep direction is singular"));
  std::array<std::array<bounds::Interval, 4>, 2> displacement{};
  std::array<NurbsCurveGeometry, 2> selected{};
  unsigned profile_count = 0;
  for (unsigned i = 0; i < 4; ++i) {
    if (!profiles[i]) continue;
    const auto curve = RestrictNurbsWire(*profiles[i], first, last, checks);
    if (!curve.ok()) return Return::Err(curve.error());
    selected[profile_count] = curve.value();
    for (unsigned j = 0; j < result.profile.axis.control_count; ++j) {
      if (!bounds::Spend(checks, 64)) return Return::Err(Limit());
      const auto delta =
          bounds::Diff(Pole(curve.value(), j), Pole(result.profile, j));
      const auto v = bounds::Div(bounds::Dot(delta, direction), norm2);
      displacement[profile_count][j] = v;
      bounds::Vector residual{};
      for (unsigned k = 0; k < 3; ++k)
        residual[k] = bounds::Sub(delta[k], bounds::Mul(v, direction[k]));
      if (!bounds::Valid(v) || bounds::Length(residual) > tolerance / 4)
        return Return::Err(Unsupported(
            "Variable sweep profile leaves its supporting generators"));
    }
    ++profile_count;
  }
  const unsigned low = displacement[0][0].low > displacement[1][0].high ? 1 : 0,
                 high = 1 - low;
  result.lower = std::numeric_limits<double>::infinity();
  result.upper = -result.lower;
  const auto direction_length = std::sqrt(norm2.low);
  for (unsigned j = 0; j < result.profile.axis.control_count; ++j) {
    const auto gap = bounds::Sub(displacement[high][j], displacement[low][j]);
    // Equal bases make these coefficients a whole-domain separation bound.
    // No shortest-arc or endpoint-only ordering supplies the missing region.
    if (!bounds::Valid(gap) || gap.low * direction_length <= 2 * tolerance)
      return Return::Err(Unsupported(
          "Variable sweep profiles cross or have no certified positive gap"));
    result.height_controls.push_back({gap.low, gap.high});
    result.lower =
        std::min(result.lower,
                 std::nextafter(static_cast<double>(displacement[low][j].low),
                                -std::numeric_limits<double>::infinity()));
    result.upper =
        std::max(result.upper,
                 std::nextafter(static_cast<double>(displacement[high][j].high),
                                std::numeric_limits<double>::infinity()));
  }
  if (!std::isfinite(result.lower) || !std::isfinite(result.upper) ||
      !(result.upper > result.lower))
    return Return::Err(
        Unsupported("Variable sweep support extent is unrepresentable"));
  // The finite tensor support covers both graphs. Certify every emitted
  // Cartesian pole as well, including cancellation and FMA/narrowing at its
  // bounding extents. A finite trim alone does not guarantee a finite box.
  for (unsigned i = 0; i < result.profile.axis.control_count; ++i) {
    for (const double v : {result.lower, result.upper}) {
      if (!bounds::Spend(checks, 32)) return Return::Err(Limit());
      const auto point = Pole(result.profile, i);
      const auto input = bounds::Vec(point);
      const std::array<double, 3> coordinates{point.x, point.y, point.z};
      const std::array<double, 3> d{swept->sweep.x, swept->sweep.y,
                                    swept->sweep.z};
      bounds::Vector rounding{};
      for (unsigned k = 0; k < 3; ++k) {
        const auto actual =
            bounds::Add(input[k], bounds::Mul(bounds::Exact(v), direction[k]));
        const auto emitted = static_cast<double>(
            std::fma(static_cast<long double>(v), d[k], coordinates[k]));
        rounding[k] = bounds::Sub(actual, bounds::Exact(emitted));
      }
      if (bounds::Length(rounding) > tolerance / 4)
        return Return::Err(Unsupported(
            "Variable sweep support rounding exceeds its allowance"));
    }
  }
  std::array<Vector3, 4> corners{};
  for (unsigned i = 0; i < 4; ++i) {
    const auto point = EvaluateCurve(selected[i < 2 ? low : high],
                                     i == 0 || i == 3 ? first : last);
    if (!point.ok()) return Return::Err(point.error());
    corners[i] = point.value().position;
  }
  std::array<unsigned, 4> starts{}, ends{};
  for (unsigned i = 0; i < 4; ++i) {
    for (unsigned end = 0; end < 2; ++end) {
      if (!bounds::Spend(checks, 4)) return Return::Err(Limit());
      const auto point = EvaluateCurve(*curves[i].geometry,
                                       end ? curves[i].end : curves[i].start);
      if (!point.ok()) return Return::Err(point.error());
      unsigned matches = 0, index = 0;
      for (unsigned k = 0; k < 4; ++k)
        if (bounds::Length(bounds::Diff(point.value().position, corners[k])) <=
            tolerance) {
          ++matches;
          index = k;
        }
      if (matches != 1)
        return Return::Err(Unsupported("Variable sweep corners are ambiguous"));
      (end ? ends : starts)[i] = index;
    }
    const auto delta = (ends[i] + 4 - starts[i]) % 4;
    if (delta != (natural_sense ? 1u : 3u) ||
        (profiles[i] && starts[i] / 2 != ends[i] / 2) ||
        (!profiles[i] && starts[i] + ends[i] != 3))
      return Return::Err(Unsupported(
          "Variable sweep loop has inconsistent parameter orientation"));
  }
  for (unsigned i = 0; i < 4; ++i)
    if (ends[i] != starts[(i + 1) % 4])
      return Return::Err(
          Unsupported("Variable sweep parameter loop is not closed"));
  return Return::Ok(std::move(result));
}
Result<BoundedIntegral> SplineSweepStripVolume6(const SplineSweepPatch& patch,
                                                Vector3 origin,
                                                uint64_t& checks) {
  using Return = Result<BoundedIntegral>;
  const auto& curve = patch.profile;
  const auto p = curve.axis.degree;
  if (curve.controls.rational || curve.controls.dimension != 3 ||
      (p != 2 && p != 3) || curve.axis.knots.size() != 2 ||
      curve.axis.control_count != p + 1 ||
      patch.height_controls.size() != p + 1)
    return Return::Err(Unsupported(
        "Variable sweep flux requires a certified polynomial strip"));
  const auto valid =
      EvaluateCurve(GeometryPayload{curve}, curve.axis.knots.front());
  if (!valid.ok()) return Return::Err(valid.error());
  const uint64_t products = uint64_t{p + 1} * p * (p + 1);
  if (!bounds::Spend(checks, 32 * products)) return Return::Err(Limit());
  bounds::Interval integral{};
  for (unsigned i = 0; i <= p; ++i)
    for (unsigned j = 0; j < p; ++j)
      for (unsigned k = 0; k <= p; ++k) {
        const auto& h = patch.height_controls[k];
        if (!std::isfinite(h.lower) || !std::isfinite(h.upper) ||
            h.lower <= 0 || h.upper < h.lower)
          return Return::Err(
              Unsupported("Variable sweep height bound is invalid"));
        const auto factor =
            bounds::Div(bounds::Exact(2u * p * Choose(p, i) * Choose(p - 1, j) *
                                      Choose(p, k)),
                        bounds::Exact(3u * p * Choose(3u * p - 1, i + j + k)));
        const auto cross = bounds::Dot(
            bounds::Cross(bounds::Diff(Pole(curve, i), origin),
                          bounds::Diff(Pole(curve, j + 1), Pole(curve, j))),
            bounds::Vec(patch.sweep));
        integral = bounds::Add(
            integral,
            bounds::Mul(factor, bounds::Mul({h.lower, h.upper}, cross)));
      }
  if (!bounds::Valid(integral))
    return Return::Err(Unsupported("Variable sweep flux is unrepresentable"));
  const auto value = std::midpoint(integral.low, integral.high);
  return Return::Ok({value, std::nextafter(std::max(value - integral.low,
                                                    integral.high - value),
                                           bounds::kInfinity)});
}
}  // namespace openswstep::internal
