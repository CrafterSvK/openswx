// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "../src/rational_graph.h"
#include "../src/strip_pcurve.h"
#include "openparasolid/evaluate.h"
#include "openswstep/export.h"
#include "rational_strip_fixture.h"
namespace openswstep {
namespace {
using namespace openparasolid;
NurbsCurveGeometry RationalSection() {
  NurbsCurveGeometry c;
  c.axis.degree = 2;
  c.axis.control_count = 3;
  c.axis.knots = {0, 1};
  c.axis.multiplicities = {3, 3};
  c.controls.rational = true;
  c.controls.dimension = 4;
  c.controls.values = {0, 1, 2, 1, 1, 2, 3, 2, 2, 3, 4, 3};
  return c;
}
TEST(OpenSwStepRationalStrip,
     ExportsWholeCurvedWedgeWithRationalParameterCurves) {
  const auto f = test::RationalVariableWedge();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok()) << model.error().message;
  ExportOptions options;
  options.length_unit = LengthUnit::kMillimetre;
  options.binding.intersection = IntersectionEvaluationOptions{1e-7, 1e-11};
  const auto result = ExportBody(model.value(), NodeId{1}, options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 6u);
  EXPECT_EQ(result.value().statistics.edges, 12u);
  EXPECT_EQ(result.value().statistics.vertices, 8u);
  EXPECT_EQ(result.value().statistics.solids, 1u);
  EXPECT_NE(result.value().text.find("SURFACE_CURVE"), std::string::npos);
  EXPECT_NE(result.value().text.find("PCURVE"), std::string::npos);
  EXPECT_NE(result.value().text.find("RATIONAL_B_SPLINE_CURVE"),
            std::string::npos);
}
TEST(OpenSwStepRationalStrip, EnclosesWeightedFluxAgainstAnalyticLogarithm) {
  const auto source = RationalSection();
  for (const double scale : {1e-150, 1., 1e150}) {
    internal::SplineSweepPatch patch;
    patch.profile = source;
    patch.sweep = {0, 0, 1};
    patch.height_controls = {{2, 2}, {2.5, 2.5}, {3, 3}};
    for (auto& value : patch.profile.controls.values) value *= scale;
    uint64_t checks = 100000;
    const auto integral = internal::RationalWeightedCurveIntegral(
        patch.profile, patch.height_controls, {}, patch.sweep, checks);
    ASSERT_TRUE(integral.ok()) << integral.error().message;
    const long double exact = -4.L / 3 - std::log(3.L) / 4;
    EXPECT_LE(std::abs(integral.value().value - exact), integral.value().error);
    EXPECT_LT(integral.value().error, .1);
    const auto flux = internal::SplineSweepStripVolume6(patch, {}, checks);
    ASSERT_TRUE(flux.ok()) << flux.error().message;
    EXPECT_LE(std::abs(flux.value().value - 2 * exact), flux.value().error);
  }
}
TEST(OpenSwStepRationalStrip,
     EnclosesCubicWeightedFluxAndRejectsInvalidBounds) {
  internal::SplineSweepPatch patch;
  patch.profile.axis.degree = 3;
  patch.profile.axis.control_count = 4;
  patch.profile.axis.knots = {-2, 3};
  patch.profile.axis.multiplicities = {4, 4};
  patch.profile.controls.dimension = 4;
  patch.profile.controls.rational = true;
  patch.profile.controls.values = {0, 0, 0, 1, 1, 0, 0, 1,
                                   2, 0, 0, 1, 3, 3, 0, 1};
  patch.sweep = {0, 0, 1};
  patch.height_controls = {{3, 3}, {4, 4}, {5, 5}, {6, 6}};
  uint64_t checks = 100000;
  const auto flux = internal::SplineSweepStripVolume6(patch, {}, checks);
  ASSERT_TRUE(flux.ok()) << flux.error().message;
  EXPECT_LE(std::abs(flux.value().value - 243.L / 5), flux.value().error);
  EXPECT_LT(flux.value().error, 5);
  checks = 0;
  const auto limited = internal::SplineSweepStripVolume6(patch, {}, checks);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  for (unsigned bad = 0; bad < 4; ++bad) {
    auto changed = patch;
    if (bad == 0) changed.height_controls[1] = {-1, 1};
    if (bad == 1) changed.height_controls[1] = {2, 1};
    if (bad == 2) changed.height_controls.pop_back();
    if (bad == 3) changed.profile.controls.values[3] = 0;
    checks = 100000;
    EXPECT_FALSE(internal::SplineSweepStripVolume6(changed, {}, checks).ok())
        << bad;
  }
}
TEST(OpenSwStepRationalStrip,
     ConstructsExactHomogeneousUvMapInOriginalKnotUnits) {
  for (const bool shifted : {false, true})
    for (const double scale : {1e-150, 1., 1e150}) {
      internal::SplineSweepPatch patch;
      patch.profile = RationalSection();
      patch.sweep = {0, 0, 1};
      if (shifted) patch.profile.axis.knots = {2, 5};
      for (auto& value : patch.profile.controls.values) value *= scale;
      auto boundary = patch.profile;
      for (unsigned i = 0; i < 3; ++i)
        boundary.controls.values[4 * i + 2] +=
            (2 + .5 * i) * boundary.controls.values[4 * i + 3];
      const auto before = boundary.controls.values;
      uint64_t checks = 100000;
      const auto pc =
          internal::BuildRationalStripPcurve(patch, boundary, 1e-8, checks);
      ASSERT_TRUE(pc.ok()) << pc.error().message;
      EXPECT_EQ(pc.value().axis.degree, 3u);
      EXPECT_EQ(pc.value().controls.dimension, 3u);
      EXPECT_TRUE(pc.value().controls.rational);
      EXPECT_EQ(pc.value().axis.knots, patch.profile.axis.knots);
      for (const double t : {0., .125, .5, .875, 1.}) {
        const double u = shifted ? 2 + 3 * t : t;
        const auto uv = EvaluateCurve(GeometryPayload{pc.value()}, u);
        ASSERT_TRUE(uv.ok()) << uv.error().message;
        EXPECT_NEAR(uv.value().position.x, u, 1e-12);
        EXPECT_NEAR(uv.value().position.y, (2 + 6 * t + t * t) / (1 + 2 * t),
                    1e-12);
        const auto a = EvaluateCurve(GeometryPayload{patch.profile},
                                     uv.value().position.x);
        const auto b = EvaluateCurve(GeometryPayload{boundary}, u);
        ASSERT_TRUE(a.ok());
        ASSERT_TRUE(b.ok());
        EXPECT_NEAR(a.value().position.z + uv.value().position.y,
                    b.value().position.z, 1e-12);
      }
      EXPECT_EQ(boundary.controls.values, before);
      checks = 0;
      const auto limit =
          internal::BuildRationalStripPcurve(patch, boundary, 1e-8, checks);
      ASSERT_FALSE(limit.ok());
      EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
    }
}
TEST(OpenSwStepRationalStrip,
     RejectsUnprovenUvMappingsAndFiniteAllowanceFailures) {
  internal::SplineSweepPatch patch;
  patch.profile = RationalSection();
  patch.sweep = {0, 0, 1};
  for (unsigned bad = 0; bad < 4; ++bad) {
    auto boundary = patch.profile;
    for (unsigned i = 0; i < 3; ++i)
      boundary.controls.values[4 * i + 2] +=
          (2 + .5 * i) * boundary.controls.values[4 * i + 3];
    if (bad == 0) boundary.controls.values[7] *= 1.01;
    if (bad == 1) boundary.controls.values[0] += .01;
    if (bad == 2) boundary.controls.values[3] = 0;
    uint64_t checks = 100000;
    EXPECT_FALSE(internal::BuildRationalStripPcurve(
                     patch, boundary, bad == 3 ? 1e-25 : 1e-8, checks)
                     .ok())
        << bad;
  }
}
TEST(OpenSwStepRationalStrip,
     SubdividesUncertainNumeratorsAndRejectsTrueFolds) {
  namespace b = internal::sweep_bounds;
  const std::array<double, 4> weights{1, 1, 1, 1};
  for (const bool folded : {false, true}) {
    const std::array<double, 4> x = folded
                                        ? std::array<double, 4>{0, 2, -1, 1}
                                        : std::array<double, 4>{0, 1, .2, 1.2};
    std::array<b::Vector, 4> controls{};
    for (unsigned i = 0; i < 4; ++i) controls[i] = b::Vec({x[i], 0, 0});
    uint64_t checks = 100000;
    const auto result = internal::RationalGraphChordCondition(
        controls, weights, {}, b::Vec({1, 0, 0}), 1, checks);
    if (folded)
      EXPECT_FALSE(result.ok());
    else {
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_LT(result.value(), 20);
    }
  }
}
TEST(OpenSwStepRationalStrip,
     CertifiesRationalChordDirectionsAndFiniteBudgets) {
  const auto c = RationalSection();
  for (const double scale : {1e-150, 1., 1e150}) {
    std::array<internal::sweep_bounds::Vector, 3> controls{};
    std::array<double, 3> weights{};
    for (unsigned i = 0; i < 3; ++i) {
      weights[i] = c.controls.values[4 * i + 3] * scale;
      for (unsigned k = 0; k < 3; ++k)
        controls[i][k] =
            internal::sweep_bounds::Exact(c.controls.values[4 * i + k] * scale);
    }
    for (const int sign : {1, -1}) {
      uint64_t checks = 100000;
      const auto result = internal::RationalGraphChordCondition(
          controls, weights, {0, 1, 2}, internal::sweep_bounds::Vec({1, 0, 0}),
          sign, checks);
      if (sign > 0) {
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_GE(result.value(), 1);
      } else
        EXPECT_FALSE(result.ok());
    }
    uint64_t checks = 0;
    const auto limit = internal::RationalGraphChordCondition(
        controls, weights, {0, 1, 2}, internal::sweep_bounds::Vec({1, 0, 0}), 1,
        checks);
    ASSERT_FALSE(limit.ok());
    EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
  }
}
}  // namespace
}  // namespace openswstep
