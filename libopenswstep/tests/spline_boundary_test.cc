// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "../src/simple_boundary.h"
#include "../src/spline_sweep.h"
#include "fixtures.h"

namespace openswstep {
namespace {
using namespace openparasolid;
struct Contour {
  std::vector<GeometryPayload> geometry;
  std::vector<double> ends;
  std::vector<internal::PlanarCurve> Curves(bool reverse = false) const {
    std::vector<internal::PlanarCurve> result;
    for (std::size_t i = 0; i < geometry.size(); ++i)
      result.push_back({&geometry[i], i == 4 ? 2. : 0., ends[i]});
    if (reverse) {
      for (auto& curve : result) std::swap(curve.start, curve.end);
      std::reverse(result.begin(), result.end());
    }
    return result;
  }
};
Contour Notch(Vector3 origin = {}, double scale = 1) {
  const std::array<Vector3, 8> points{{{0, 0, 0},
                                       {4, 0, 0},
                                       {4, 4, 0},
                                       {3, 4, 0},
                                       {3, 1, 0},
                                       {1, 1, 0},
                                       {1, 4, 0},
                                       {0, 4, 0}}};
  const auto transform = [&](Vector3 p) {
    return Vector3{origin.x + scale * p.x, origin.y + scale * p.y,
                   origin.z + scale * p.z};
  };
  Contour result;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto a = transform(points[i]),
               b = transform(points[(i + 1) % points.size()]);
    if (i == 4) {
      NurbsCurveGeometry curve;
      curve.axis.degree = 3;
      curve.axis.control_count = 4;
      curve.axis.knots = {2, 5};
      curve.axis.multiplicities = {4, 4};
      curve.controls.dimension = 3;
      // Monotone in X, with an actual curvature sign change and concave joins.
      for (Vector3 p : {Vector3{3, 1, 0}, Vector3{2.5, .5, 0},
                        Vector3{1.5, 1.5, 0}, Vector3{1, 1, 0}}) {
        p = transform(p);
        for (double v : {p.x, p.y, p.z}) curve.controls.values.push_back(v);
      }
      result.geometry.push_back(curve);
      result.ends.push_back(5);
    } else {
      const Vector3 d{b.x - a.x, b.y - a.y, b.z - a.z};
      const double length = std::hypot(d.x, d.y, d.z);
      result.geometry.push_back(
          LineGeometry{a, {d.x / length, d.y / length, d.z / length}});
      result.ends.push_back(length);
    }
  }
  return result;
}
PlaneGeometry Plane() { return {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}}; }
TEST(OpenSwStepSplineBoundary, CertifiesInflectedConcaveLoopsAndCollinearGaps) {
  const auto fixture = Notch();
  for (bool reverse : {false, true}) {
    uint64_t checks = 100000;
    const auto curves = fixture.Curves(reverse);
    const auto result = internal::ValidateSimplePlanarLoop(
        curves, Plane(), !reverse, 1e-7, checks);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_NEAR(result.value().area, 10, 1e-12);
    EXPECT_GT(checks, 0u);
    EXPECT_LT(checks, 100000u);
  }
}
TEST(OpenSwStepSplineBoundary,
     RejectsWrongWindingOutOfPlaneAndCrossingControls) {
  for (int bad = 0; bad < 4; ++bad) {
    auto fixture = Notch();
    auto& controls =
        std::get<NurbsCurveGeometry>(fixture.geometry[4]).controls.values;
    if (bad == 1) controls[5] = 1e-3;
    if (bad == 2) {
      controls[3] = 10;
      controls[6] = 10;
    }
    if (bad == 3) {
      controls[3] = controls[0];
      controls[4] = controls[1];
    }
    uint64_t checks = 100000;
    const auto result = internal::ValidateSimplePlanarLoop(
        fixture.Curves(bad == 0), Plane(), true, 1e-7, checks);
    EXPECT_FALSE(result.ok());
  }
}
TEST(OpenSwStepSplineBoundary,
     ClassifiesCurvedOuterAndMixedHolesInSourceOrder) {
  const auto outer = Notch();
  const auto hole = Notch({.1, .1, 0}, .1);
  GeometryPayload circle =
      CircleGeometry{{.8, .5, 0}, {0, 0, 1}, {1, 0, 0}, .1};
  const std::vector<std::vector<internal::PlanarCurve>> loops{
      hole.Curves(true), outer.Curves(), {{&circle, 2 * std::numbers::pi, 0}}};
  uint64_t checks = 100000;
  const auto result =
      internal::ValidateSimplePlanarRegion(loops, Plane(), true, 1e-7, checks);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().outer_loop, 1u);
  EXPECT_NEAR(result.value().area, 9.9 - std::numbers::pi * .01, 1e-12);
}
TEST(OpenSwStepSplineBoundary, RejectsOverlappingNestedAndOutsideHoles) {
  const auto outer = Notch();
  const auto first = Notch({.1, .1, 0}, .1);
  for (Vector3 p :
       {Vector3{.15, .15, 0}, Vector3{.4, .2, 0}, Vector3{6, 6, 0}}) {
    const auto second = Notch(p, p.x == .15 ? .01 : .1);
    const std::vector<std::vector<internal::PlanarCurve>> loops{
        outer.Curves(), first.Curves(true), second.Curves(true)};
    uint64_t checks = 10000;
    EXPECT_FALSE(
        internal::ValidateSimplePlanarRegion(loops, Plane(), true, 1e-7, checks)
            .ok());
  }
}
TEST(OpenSwStepSplineBoundary, ClosesRoundedJoinsDuringContainmentTests) {
  // Two separated clockwise square holes in a rectangular outer loop. The
  // right hole's lower-right join straddles the left hole's horizontal ray.
  // Independent source edge evaluation leaves a valid sub-tolerance gap.
  const auto rectangle = [](double left, double bottom, double right,
                            double top, double gap) {
    Contour result;
    result.geometry = {LineGeometry{{left, bottom, 0}, {0, 1, 0}},
                       LineGeometry{{left, top, 0}, {1, 0, 0}},
                       LineGeometry{{right, top, 0}, {0, -1, 0}},
                       LineGeometry{{right, bottom, 0}, {-1, 0, 0}}};
    result.ends = {top - bottom, right - left, top - bottom - gap,
                   right - left};
    return result;
  };
  const auto outer = rectangle(0, 0, 6, 3, 0);
  const auto left = rectangle(1, 1, 2, 2, 0);
  for (const double gap : {std::nextafter(1., 2.) - 1., 2.5e-8}) {
    const auto right = rectangle(3, 1, 4, 2, gap);
    for (const bool reverse : {false, true})
      for (const bool closing_join : {false, true}) {
        auto right_curves = right.Curves(reverse);
        if (closing_join)
          std::rotate(right_curves.begin(),
                      right_curves.begin() + (reverse ? 1 : 3),
                      right_curves.end());
        const std::vector<std::vector<internal::PlanarCurve>> loops{
            left.Curves(reverse), right_curves, outer.Curves(!reverse)};
        uint64_t checks = 10000;
        const auto result = internal::ValidateSimplePlanarRegion(
            loops, Plane(), !reverse, 1e-7, checks);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().outer_loop, 2u);
        EXPECT_NEAR(result.value().area, 16, 1e-7);
      }
  }
  // Missing source joins larger than the caller tolerance remain unsupported.
  const auto broken = rectangle(3, 1, 4, 2, 1e-4);
  const std::vector<std::vector<internal::PlanarCurve>> loops{
      left.Curves(), broken.Curves(), outer.Curves(true)};
  uint64_t checks = 10000;
  EXPECT_FALSE(
      internal::ValidateSimplePlanarRegion(loops, Plane(), true, 1e-7, checks)
          .ok());
}
TEST(OpenSwStepSplineBoundary, EnforcesExplicitToleranceAndWorkBudget) {
  const auto fixture = Notch();
  uint64_t checks = 0;
  const auto limited = internal::ValidateSimplePlanarLoop(
      fixture.Curves(), Plane(), true, 1e-7, checks);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  checks = 100000;
  EXPECT_FALSE(internal::ValidateSimplePlanarLoop(fixture.Curves(), Plane(),
                                                  true, 0, checks)
                   .ok());
}
Model SweepModel(bool shifted_knots = false, bool rational = false) {
  openparasolid::test::ModelFixture f;
  f.Spline(rational);
  if (shifted_knots)
    for (auto& node : f.report.nodes)
      if (node.index == 4) node.fields[0].values = {2., 5.};
  f.Add(67, 20,
        {test::Pointer("section", 1, 1008), test::Numbers("scale", {42}),
         test::Numbers("sweep", {0, 0, 1}, FieldType::kVector),
         test::ByteValue("sense", '+', FieldType::kCharacter)});
  return ModelBuilder::Build(f.report).value();
}
std::vector<GeometryPayload> SweepBoundary(const Model& model, double first = 0,
                                           double last = 1, double low = 0,
                                           double high = 2) {
  auto curve = std::get<NurbsCurveGeometry>(
      model.geometry()[*model.FindNode(NodeId{1})->geometry].payload);
  auto lower = curve, upper = curve;
  for (std::size_t i = 2; i < curve.controls.values.size();
       i += curve.controls.dimension) {
    const auto w = curve.controls.rational ? curve.controls.values[i + 1] : 1;
    lower.controls.values[i] += low * w;
    upper.controls.values[i] += high * w;
  }
  const auto a = EvaluateCurve(curve, first).value().position;
  const auto b = EvaluateCurve(curve, last).value().position;
  return {lower, LineGeometry{{b.x, b.y, b.z + low}, {0, 0, 1}}, upper,
          LineGeometry{{a.x, a.y, a.z + high}, {0, 0, -1}}};
}
std::vector<internal::PlanarCurve> SweepCurves(
    const std::vector<GeometryPayload>& curves, double first = 0,
    double last = 1, double height = 2) {
  return {{&curves[0], first, last},
          {&curves[1], 0, height},
          {&curves[2], last, first},
          {&curves[3], 0, height}};
}
TEST(OpenSwStepSplineSweep, CertifiesControlTranslationsAndExactSurfaceFlux) {
  const auto model = SweepModel();
  const auto geometry = SweepBoundary(model);
  uint64_t checks = 10000;
  const auto patch = internal::RecognizeSplineSweepPatch(
      model, NodeId{20}, SweepCurves(geometry), true, 1e-7, checks);
  ASSERT_TRUE(patch.ok()) << patch.error().message;
  EXPECT_DOUBLE_EQ(patch.value().lower, 0);
  EXPECT_DOUBLE_EQ(patch.value().upper, 2);
  const auto flux = internal::SplineSweepVolume6(patch.value(), {}, checks);
  ASSERT_TRUE(flux.ok());
  EXPECT_NEAR(flux.value(), -8, 1e-12);
}
TEST(OpenSwStepSplineSweep,
     PreservesPartialNonzeroDomainsAndSignedDisplacements) {
  const auto model = SweepModel(true);
  const auto geometry = SweepBoundary(model, 2.6, 4.4, 3, 5);
  uint64_t checks = 10000;
  const auto patch = internal::RecognizeSplineSweepPatch(
      model, NodeId{20}, SweepCurves(geometry, 2.6, 4.4), true, 1e-7, checks);
  ASSERT_TRUE(patch.ok()) << patch.error().message;
  EXPECT_DOUBLE_EQ(patch.value().profile.axis.knots.front(), 2.6);
  EXPECT_DOUBLE_EQ(patch.value().profile.axis.knots.back(), 4.4);
  EXPECT_NEAR(patch.value().lower, 3, 1e-12);
  EXPECT_NEAR(patch.value().upper, 5, 1e-12);
  const auto flux = internal::SplineSweepVolume6(patch.value(), {}, checks);
  ASSERT_TRUE(flux.ok());
  EXPECT_NEAR(flux.value(), -4.8, 1e-12);
  const auto shifted =
      internal::SplineSweepVolume6(patch.value(), {2, 0, 0}, checks);
  ASSERT_TRUE(shifted.ok());
  EXPECT_NEAR(shifted.value(), -14.4, 1e-12);
  checks = 0;
  const auto exhausted =
      internal::SplineSweepVolume6(patch.value(), {}, checks);
  ASSERT_FALSE(exhausted.ok());
  EXPECT_EQ(exhausted.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStepSplineSweep, CertifiesWeightedProfilesAndBoundedFlux) {
  const auto model = SweepModel(false, true);
  const auto geometry = SweepBoundary(model);
  uint64_t checks = 100000;
  const auto patch = internal::RecognizeSplineSweepPatch(
      model, NodeId{20}, SweepCurves(geometry), true, 1e-7, checks);
  ASSERT_TRUE(patch.ok()) << patch.error().message;
  long double error = 0;
  const auto flux =
      internal::SplineSweepVolume6(patch.value(), {}, checks, &error);
  ASSERT_TRUE(flux.ok()) << flux.error().message;
  // This source profile has Y=1, X=2t/(1+2t), so integral(X dY-Y dX)=-2/3.
  EXPECT_LE(std::abs(flux.value() + 8.L / 3), error);
  EXPECT_GT(error, 0);
  EXPECT_LT(error, .1);
  EXPECT_FALSE(internal::SplineSweepVolume6(patch.value(), {}, checks).ok());
  auto wrong = geometry;
  auto& upper = std::get<NurbsCurveGeometry>(wrong[2]);
  upper.controls.values[7] *= 1.01;
  checks = 100000;
  EXPECT_FALSE(internal::RecognizeSplineSweepPatch(
                   model, NodeId{20}, SweepCurves(wrong), true, 1e-7, checks)
                   .ok());
  checks = 0;
  const auto limited =
      internal::SplineSweepVolume6(patch.value(), {}, checks, &error);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStepSplineSweep, EnclosesRationalQuarterCircleGreenIntegral) {
  NurbsCurveGeometry c;
  c.axis.degree = 2;
  c.axis.control_count = 3;
  c.axis.knots = {0, 1};
  c.axis.multiplicities = {3, 3};
  c.controls.rational = true;
  c.controls.dimension = 4;
  const double w = std::sqrt(.5);
  c.controls.values = {1, 0, 0, 1, w, w, 0, w, 0, 1, 0, 1};
  for (const double scale : {1e-100, 1., 1e100}) {
    auto source = c;
    for (auto& value : source.controls.values) value *= scale;
    uint64_t checks = 100000;
    const auto value =
        internal::RationalCurveIntegral(source, {}, {0, 0, 1}, checks);
    ASSERT_TRUE(value.ok()) << value.error().message;
    EXPECT_LE(std::abs(value.value().value - std::acos(-1.L) / 2),
              value.value().error);
    EXPECT_LT(value.value().error, .02);
  }
  uint64_t finite_checks=100000;
  const auto invalid=internal::RationalCurveIntegral(c,{},
      {0,0,std::numeric_limits<double>::infinity()},finite_checks);
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code,ErrorCode::kInvalidOptions);
  c.controls.values[3] = 0;
  uint64_t checks = 100000;
  EXPECT_FALSE(internal::RationalCurveIntegral(c, {}, {0, 0, 1}, checks).ok());
}
TEST(OpenSwStepSplineSweep, CertifiesVariableHeightAndItsPolynomialFlux) {
  const auto model = SweepModel();
  auto geometry = SweepBoundary(model);
  auto& upper = std::get<NurbsCurveGeometry>(geometry[2]);
  upper.controls.values[5] += .5;
  upper.controls.values[8] += 1;
  auto curves = SweepCurves(geometry);
  curves[1].end = 3;
  uint64_t checks = 100000;
  const auto patch = internal::RecognizeSplineSweepPatch(
      model, NodeId{20}, curves, true, 1e-7, checks);
  ASSERT_TRUE(patch.ok()) << patch.error().message;
  ASSERT_EQ(patch.value().height_controls.size(), 3u);
  long double error = 0;
  const auto flux = internal::SplineSweepVolume6(patch.value(), {}, checks, &error);
  ASSERT_TRUE(flux.ok()) << flux.error().message;
  EXPECT_LE(std::abs(flux.value() + 10), error);
  EXPECT_LT(error, 1e-10);
  EXPECT_FALSE(internal::SplineSweepVolume6(patch.value(), {}, checks).ok());
  checks = 0;
  const auto exhausted = internal::SplineSweepVolume6(patch.value(), {}, checks, &error);
  ASSERT_FALSE(exhausted.ok());
  EXPECT_EQ(exhausted.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStepSplineSweep, EnclosesCubicVariableFluxAgainstExactMoments) {
  internal::SplineSweepPatch patch;
  patch.profile.axis.degree = 3;
  patch.profile.axis.control_count = 4;
  patch.profile.axis.knots = {0, 1};
  patch.profile.axis.multiplicities = {4, 4};
  patch.profile.controls.dimension = 3;
  patch.profile.controls.values = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 3, 0};
  patch.sweep = {0, 0, 1};
  patch.lower = 0;
  patch.upper = 6;
  patch.height_controls = {{3, 3}, {4, 4}, {5, 5}, {6, 6}};
  // P=(3t,3t^3,0), H=3+3t. Twice integral H*(P cross P')_z is 243/5.
  for (const bool shifted : {false, true}) {
    uint64_t checks = 100000;
    const auto flux = internal::SplineSweepStripVolume6(
        patch, shifted ? Vector3{1, 0, 0} : Vector3{}, checks);
    ASSERT_TRUE(flux.ok()) << flux.error().message;
    const auto expected = shifted ? 171.L / 10 : 243.L / 5;
    EXPECT_LE(std::abs(flux.value().value - expected), flux.value().error);
    EXPECT_LT(flux.value().error, 1e-10);
  }
}
TEST(OpenSwStepSplineSweep, RejectsCrossingAndUnresolvedStripHeights) {
  const auto model = SweepModel();
  for (unsigned bad = 0; bad < 4; ++bad) {
    auto geometry = SweepBoundary(model);
    auto& upper = std::get<NurbsCurveGeometry>(geometry[2]);
    if (bad == 0) upper.controls.values[5] -= 4;
    if (bad == 1) upper.controls.values[5] -= 2;
    if (bad == 2) upper.controls.values[4] += .01;
    auto curves = SweepCurves(geometry);
    if (bad == 3) std::swap(curves[1], curves[2]);
    uint64_t checks = 100000;
    EXPECT_FALSE(internal::RecognizeSplineSweepStrip(
        model, NodeId{20}, curves, true, 1e-7, checks).ok()) << bad;
  }
}
TEST(OpenSwStepSplineSweep, RejectsUnmatchedControlsBasisWindingAndBudgets) {
  const auto model = SweepModel();
  for (int bad = 0; bad < 5; ++bad) {
    auto geometry = SweepBoundary(model);
    auto& upper = std::get<NurbsCurveGeometry>(geometry[2]);
    if (bad == 0) upper.controls.values[4] += .01;
    if (bad == 1) upper.axis.knots.back() = 2;
    if (bad == 2)
      geometry[1] = CircleGeometry{{2, 3, 4}, {0, 0, 1}, {1, 0, 0}, 2};
    uint64_t checks = bad == 4 ? 0 : 10000;
    const auto result = internal::RecognizeSplineSweepPatch(
        model, NodeId{20}, SweepCurves(geometry), bad != 3, 1e-7, checks);
    ASSERT_FALSE(result.ok());
    if (bad == 4) EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
}
}  // namespace
}  // namespace openswstep
