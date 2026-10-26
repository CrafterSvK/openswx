// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "spline_sweep.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "nurbs_wire.h"
#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
using Vector = std::array<long double, 3>;
Vector Difference(Vector3 a, Vector3 b) {
  return {static_cast<long double>(a.x) - b.x,
          static_cast<long double>(a.y) - b.y,
          static_cast<long double>(a.z) - b.z};
}
long double Dot(const Vector& a, Vector3 b) {
  return a[0] * b.x + a[1] * b.y + a[2] * b.z;
}
long double Distance(Vector3 a, Vector3 b) {
  const auto q = Difference(a, b);
  return std::hypot(q[0], q[1], q[2]);
}
Vector3 Control(const NurbsCurveGeometry& c, std::size_t i) {
  const auto at = i * c.controls.dimension;
  const long double w = c.controls.rational ? c.controls.values[at + 3] : 1;
  return {static_cast<double>(c.controls.values[at] / w),
          static_cast<double>(c.controls.values[at + 1] / w),
          static_cast<double>(c.controls.values[at + 2] / w)};
}
bool SameBasis(const NurbsCurveGeometry& a, const NurbsCurveGeometry& b) {
  return a.axis.degree == b.axis.degree &&
         a.axis.control_count == b.axis.control_count &&
         a.axis.knots == b.axis.knots &&
         a.axis.multiplicities == b.axis.multiplicities &&
         a.controls.rational == b.controls.rational &&
         a.controls.dimension == (a.controls.rational ? 4 : 3) &&
         b.controls.dimension == a.controls.dimension;
}
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Spline sweep control check limit exceeded"};
}
bool Spend(uint64_t& remaining) {
  if (!remaining) return false;
  --remaining;
  return true;
}
Vector3 Shift(Vector3 p, Vector3 d, double v) {
  return {static_cast<double>(std::fma(static_cast<long double>(v), d.x, p.x)),
          static_cast<double>(std::fma(static_cast<long double>(v), d.y, p.y)),
          static_cast<double>(std::fma(static_cast<long double>(v), d.z, p.z))};
}
}  // namespace
static Result<SplineSweepPatch> RecognizeRectangularSplineSweepPatch(
    const Model& model, NodeId surface, std::span<const PlanarCurve> curves,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<SplineSweepPatch>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return Return::Err({ErrorCode::kInvalidOptions, 0,
                        "Spline sweep requires a positive finite tolerance"});
  const auto* node = model.FindNode(surface);
  const auto* swept = node && node->geometry
                          ? std::get_if<SweptSurfaceGeometry>(
                                &model.geometry()[*node->geometry].payload)
                          : nullptr;
  const auto* section_node = swept ? model.FindNode(swept->section) : nullptr;
  const auto* section =
      section_node && section_node->geometry
          ? std::get_if<NurbsCurveGeometry>(
                &model.geometry()[*section_node->geometry].payload)
          : nullptr;
  if (!section || curves.size() != 4 ||
      section->controls.dimension != (section->controls.rational ? 4 : 3) ||
      section->axis.closed || section->axis.periodic ||
      (section->axis.degree != 2 && section->axis.degree != 3))
    return Return::Err(
        Unsupported("Spline sweeps require an open "
                    "quadratic/cubic section and four edges"));
  std::array<const NurbsCurveGeometry*, 4> profiles{};
  std::array<double, 4> displacement{};
  double low_u = 0, high_u = 0;
  unsigned count = 0;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    const auto& c = curves[i];
    if (!c.geometry || !std::isfinite(c.start) || !std::isfinite(c.end) ||
        c.start == c.end)
      return Return::Err(
          Unsupported("Spline sweep boundary interval is invalid"));
    profiles[i] = std::get_if<NurbsCurveGeometry>(c.geometry);
    if (!profiles[i]) {
      if (!std::holds_alternative<LineGeometry>(*c.geometry))
        return Return::Err(
            Unsupported("Spline sweep boundaries require LINE/B_CURVE"));
      continue;
    }
    const auto first = std::min(c.start, c.end),
               last = std::max(c.start, c.end);
    if (!SameBasis(*section, *profiles[i]) ||
        (count && (first != low_u || last != high_u)))
      return Return::Err(
          Unsupported("Spline sweep profiles require the same "
                      "source knot basis and interval"));
    low_u = first;
    high_u = last;
    ++count;
  }
  if (count != 2 || (profiles[0] != nullptr) == (profiles[1] != nullptr) ||
      (profiles[0] != nullptr) != (profiles[2] != nullptr))
    return Return::Err(Unsupported(
        "Spline sweep boundary must alternate profiles and generators"));
  const auto source =
      RestrictNurbsWire(*section, low_u, high_u, remaining_checks);
  if (!source.ok()) return Return::Err(source.error());
  SplineSweepPatch result;
  result.profile = source.value();
  result.sweep = swept->sweep;
  if (section->axis.multiplicities.front() != section->axis.degree + 1 ||
      section->axis.multiplicities.back() != section->axis.degree + 1)
    return Return::Err(Unsupported(
        "Spline sweep inversion requires a clamped source profile"));
  const auto valid =
      EvaluateSurface(model, surface, section->axis.knots.front(), 0);
  if (!valid.ok()) return Return::Err(valid.error());
  // The inverse's transverse monotonicity certificate excludes a folded or
  // singular support over the source domain. Binding has its own sample checks.
  const auto inverse =
      InvertSurface(model, surface, valid.value().position, tolerance);
  if (!inverse.ok()) return Return::Err(inverse.error());
  const long double norm2 =
      Dot({swept->sweep.x, swept->sweep.y, swept->sweep.z}, swept->sweep);
  result.lower = std::numeric_limits<double>::infinity();
  result.upper = -result.lower;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    if (!profiles[i]) continue;
    const auto selected =
        RestrictNurbsWire(*profiles[i], low_u, high_u, remaining_checks);
    if (!selected.ok()) return Return::Err(selected.error());
    if (!SameBasis(result.profile, selected.value()))
      return Return::Err(
          Unsupported("Restricted sweep profile bases disagree"));
    const long double v = Dot(Difference(Control(selected.value(), 0),
                                         Control(result.profile, 0)),
                              swept->sweep) /
                          norm2;
    if (!std::isfinite(v) || std::abs(v) > std::numeric_limits<double>::max())
      return Return::Err(
          Unsupported("Spline sweep displacement is unrepresentable"));
    displacement[i] = static_cast<double>(v);
    for (std::size_t j = 0; j < result.profile.axis.control_count; ++j) {
      if (!Spend(remaining_checks)) return Return::Err(Limit());
      if (result.profile.controls.rational &&
          selected.value().controls.values[4 * j + 3] !=
              result.profile.controls.values[4 * j + 3])
        return Return::Err(
            Unsupported("Rational sweep profiles require identical weights"));
      long double translation_error = Distance(
          Control(selected.value(), j),
          Shift(Control(result.profile, j), swept->sweep, displacement[i]));
      if (result.profile.controls.rational) {
        const auto a = Control(selected.value(), j),
                   b = Control(result.profile, j);
        const auto scale =
            std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z),
                      std::abs(b.x), std::abs(b.y), std::abs(b.z)});
        // Cover dehomogenization, shifted STEP control rounding and the
        // displacement inferred from rounded Cartesian controls.
        translation_error +=
            16 * std::numeric_limits<double>::epsilon() * scale;
      }
      if (translation_error > tolerance)
        return Return::Err(Unsupported(
            "Spline sweep profile is not a constant source translation"));
    }
    result.lower = std::min(result.lower, displacement[i]);
    result.upper = std::max(result.upper, displacement[i]);
  }
  if (!std::isfinite(result.upper - result.lower) ||
      result.upper - result.lower <= tolerance)
    return Return::Err(Unsupported("Spline sweep height is unresolved"));
  const auto first = EvaluateCurve(result.profile, low_u);
  const auto last = EvaluateCurve(result.profile, high_u);
  if (!first.ok()) return Return::Err(first.error());
  if (!last.ok()) return Return::Err(last.error());
  const std::array<Vector3, 4> corners{
      Shift(first.value().position, swept->sweep, result.lower),
      Shift(last.value().position, swept->sweep, result.lower),
      Shift(last.value().position, swept->sweep, result.upper),
      Shift(first.value().position, swept->sweep, result.upper)};
  std::array<unsigned, 4> starts{}, ends{};
  for (std::size_t i = 0; i < curves.size(); ++i) {
    for (unsigned end = 0; end < 2; ++end) {
      if (!Spend(remaining_checks)) return Return::Err(Limit());
      const auto sample = EvaluateCurve(*curves[i].geometry,
                                        end ? curves[i].end : curves[i].start);
      if (!sample.ok()) return Return::Err(sample.error());
      unsigned matches = 0, index = 0;
      for (unsigned k = 0; k < 4; ++k)
        if (Distance(sample.value().position, corners[k]) <= tolerance) {
          ++matches;
          index = k;
        }
      if (matches != 1)
        return Return::Err(
            Unsupported("Spline sweep corners are ambiguous or inconsistent"));
      (end ? ends : starts)[i] = index;
    }
    const auto delta = (ends[i] + 4 - starts[i]) % 4;
    if (delta != (natural_sense ? 1u : 3u) ||
        (profiles[i] && (starts[i] / 2 != ends[i] / 2)) ||
        (!profiles[i] && ((starts[i] + ends[i]) != 3)))
      return Return::Err(Unsupported(
          "Spline sweep loop does not bound one oriented parameter rectangle"));
  }
  for (std::size_t i = 0; i < curves.size(); ++i)
    if (ends[i] != starts[(i + 1) % 4])
      return Return::Err(
          Unsupported("Spline sweep parameter boundary is not closed"));
  return Return::Ok(std::move(result));
}
Result<SplineSweepPatch> RecognizeSplineSweepPatch(
    const Model& model, NodeId surface, std::span<const PlanarCurve> curves,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  auto rectangle = RecognizeRectangularSplineSweepPatch(
      model, surface, curves, natural_sense, tolerance, remaining_checks);
  if (rectangle.ok() || rectangle.error().code != ErrorCode::kUnsupportedFormat)
    return rectangle;
  return RecognizeSplineSweepStrip(model, surface, curves, natural_sense,
                                   tolerance, remaining_checks);
}
Result<long double> SplineSweepVolume6(const SplineSweepPatch& patch,
                                       Vector3 origin,
                                       uint64_t& remaining_checks,
                                       long double* error) {
  using Return = Result<long double>;
  if (error) *error = 0;
  if (!patch.height_controls.empty()) {
    if (!error)
      return Return::Err(
          Unsupported("Variable sweep flux requires its error bound"));
    const auto flux = SplineSweepStripVolume6(patch, origin, remaining_checks);
    if (!flux.ok()) return Return::Err(flux.error());
    *error = flux.value().error;
    return Return::Ok(flux.value().value);
  }
  if (patch.profile.controls.rational) {
    if (!error)
      return Return::Err(Unsupported(
          "Rational sweep flux requires its certified error bound"));
    const auto integral = RationalCurveIntegral(patch.profile, origin,
                                                patch.sweep, remaining_checks);
    if (!integral.ok()) return Return::Err(integral.error());
    const long double height =
        2 * (static_cast<long double>(patch.upper) - patch.lower);
    if (!std::isfinite(height) || height <= 0)
      return Return::Err(Unsupported("Rational sweep height is invalid"));
    *error = std::nextafter(height * integral.value().error,
                            std::numeric_limits<long double>::infinity());
    const auto value = height * integral.value().value;
    *error += 8 * std::numeric_limits<long double>::epsilon() * std::abs(value);
    if (!std::isfinite(value) || !std::isfinite(*error))
      return Return::Err(Unsupported("Rational sweep flux is unrepresentable"));
    return Return::Ok(value);
  }
  const auto& profile = patch.profile;
  if ((profile.axis.degree != 2 && profile.axis.degree != 3) ||
      profile.controls.rational || profile.controls.dimension != 3 ||
      profile.axis.knots.size() < 2 || !std::isfinite(patch.lower) ||
      !std::isfinite(patch.upper) || patch.upper <= patch.lower)
    return Return::Err(
        Unsupported("Spline sweep flux requires a certified polynomial patch"));
  long double integral = 0;
  for (std::size_t i = 0; i + 1 < profile.axis.knots.size(); ++i) {
    const auto span =
        RestrictNurbsWire(profile, profile.axis.knots[i],
                          profile.axis.knots[i + 1], remaining_checks);
    if (!span.ok()) return Return::Err(span.error());
    std::array<Vector, 4> p;
    const auto degree = span.value().axis.degree;
    for (unsigned j = 0; j <= degree; ++j)
      p[j] = Difference(Control(span.value(), j), origin);
    const uint64_t cost = degree == 2 ? 3 : 6;
    if (remaining_checks < cost) return Return::Err(Limit());
    remaining_checks -= cost;
    const auto cross = [&](unsigned a, unsigned b) {
      return Dot({p[a][1] * p[b][2] - p[a][2] * p[b][1],
                  p[a][2] * p[b][0] - p[a][0] * p[b][2],
                  p[a][0] * p[b][1] - p[a][1] * p[b][0]},
                 patch.sweep);
    };
    if (degree == 2)
      integral += (2 * cross(0, 1) + cross(0, 2) + 2 * cross(1, 2)) / 3;
    else
      integral += (6 * cross(0, 1) + 3 * cross(0, 2) + cross(0, 3) +
                   3 * cross(1, 2) + 3 * cross(1, 3) + 6 * cross(2, 3)) /
                  10;
    if (!std::isfinite(integral))
      return Return::Err(
          Unsupported("Spline sweep polynomial integral is unrepresentable"));
  }
  const long double value =
      2 * (static_cast<long double>(patch.upper) - patch.lower) * integral;
  if (!std::isfinite(value))
    return Return::Err(
        Unsupported("Spline sweep surface flux is unrepresentable"));
  return Return::Ok(value);
}
}  // namespace openswstep::internal
