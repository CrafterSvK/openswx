// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
NurbsSurfaceGeometry Patch() {
  NurbsSurfaceGeometry surface;
  surface.u.degree = 1;
  surface.u.control_count = 2;
  surface.u.knots = {0, 1};
  surface.u.multiplicities = {2, 2};
  surface.v.degree = 2;
  surface.v.control_count = 3;
  surface.v.knots = {0, 1};
  surface.v.multiplicities = {3, 3};
  surface.controls.dimension = 3;
  // P(u,v) = (2u, 3v, 2uv+v^2).
  surface.controls.values = {0, 0, 0, 0, 1.5, 0, 0, 3, 1,
                             2, 0, 0, 2, 1.5, 1, 2, 3, 3};
  return surface;
}
TEST(ParasolidNurbsInvert, RecoversCurvedPatchIncludingBoundariesAndBadSeed) {
  for (double u : {0., .13, .61, 1.}) {
    for (double v : {0., .27, .82, 1.}) {
      const Vector3 point{2 * u, 3 * v, 2 * u * v + v * v};
      for (bool seeded : {false, true}) {
        const auto result =
            seeded ? InvertSurface(Patch(), point, 1e-12, {1 - u, 1 - v})
                   : InvertSurface(Patch(), point, 1e-12);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_NEAR(result.value().u, u, 1e-12);
        EXPECT_NEAR(result.value().v, v, 1e-12);
        EXPECT_LE(result.value().residual, 1e-12);
      }
    }
  }
}
TEST(ParasolidNurbsInvert, RationalWeightsAndIndependentDomainScales) {
  auto surface = Patch();
  std::swap(surface.u, surface.v);
  surface.u.knots = {-3, 17};
  surface.v.knots = {10, 10.001};
  surface.controls.rational = true;
  surface.controls.dimension = 4;
  const double w = std::sqrt(.5);
  surface.controls.values = {1, 0, 0,     1, 1, 0, 2, 1, w, w, 0, w,
                             w, w, 2 * w, w, 0, 1, 0, 1, 0, 1, 2, 1};
  for (double scale : {1e-200, 1., 1e200}) {
    auto scaled = surface;
    for (auto& value : scaled.controls.values) value *= scale;
    for (double u : {-3., 7., 17.}) {
      // These are exact corner or mid-arc positions, independent of evaluator.
      const Vector3 point = u == -3   ? Vector3{1, 0, 1}
                            : u == 17 ? Vector3{0, 1, 1}
                                      : Vector3{w, w, 1};
      const auto result = InvertSurface(scaled, point, 1e-10);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_NEAR(result.value().u, u, 1e-9);
      EXPECT_NEAR(result.value().v, 10.0005, 1e-12);
    }
  }
}
TEST(ParasolidNurbsInvert, SingularSeedFallsBackToOtherStarts) {
  auto surface = Patch();
  surface.u = surface.v;
  // P(u,v)=(u^2, v, 0): du vanishes at the supplied starting seed.
  surface.v.degree = 1;
  surface.v.control_count = 2;
  surface.v.multiplicities = {2, 2};
  surface.controls.values = {0, 0, 0, 0, 1, 0, 0, 0, 0,
                             0, 1, 0, 1, 0, 0, 1, 1, 0};
  const auto result = InvertSurface(surface, {.36, .3, 0}, 1e-12, {0, .3});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_NEAR(result.value().u, .6, 1e-12);
  EXPECT_NEAR(result.value().v, .3, 1e-12);
}
TEST(ParasolidNurbsInvert, MembershipUsesExplicitDistanceAndDomainOnly) {
  auto surface = Patch();
  for (std::size_t i = 2; i < surface.controls.values.size(); i += 3)
    surface.controls.values[i] = 0;
  ASSERT_TRUE(InvertSurface(surface, {1, 1.5, 0}, 0).ok());
  const auto outside = InvertSurface(surface, {1, 1.5, .1}, .09);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  const auto within = InvertSurface(surface, {1, 1.5, .1}, .1);
  ASSERT_TRUE(within.ok());
  EXPECT_NEAR(within.value().residual, .1, 1e-15);
  EXPECT_FALSE(InvertSurface(surface, {2.1, 1, 0}, 1e-12).ok());
  surface.controls.values.assign(18, 0);
  EXPECT_TRUE(InvertSurface(surface, {}, 0).ok());
  EXPECT_FALSE(InvertSurface(surface, {1, 0, 0}, 0).ok());
}
TEST(ParasolidNurbsInvert, RejectsMalformedGeometryAndInvalidHints) {
  for (double bad : {-1., 2., std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    for (const auto seed :
         {SurfaceParameterSeed{bad, .5}, SurfaceParameterSeed{.5, bad}}) {
      const auto result = InvertSurface(Patch(), {}, 1e-10, seed);
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
    }
  }
  const auto empty = InvertSurface(NurbsSurfaceGeometry{}, {}, 0);
  ASSERT_FALSE(empty.ok());
  EXPECT_EQ(empty.error().code, ErrorCode::kInvalidRecord);
  for (int bad = 0; bad < 4; ++bad) {
    auto surface = Patch();
    if (bad == 0) surface.controls.values.pop_back();
    if (bad == 1) surface.v.knots[1] = 0;
    if (bad == 2)
      surface.controls.values[0] = std::numeric_limits<double>::quiet_NaN();
    if (bad == 3) surface.u.degree = 65;
    const auto result = InvertSurface(surface, {}, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code,
              bad == 3 ? ErrorCode::kResourceLimit : ErrorCode::kInvalidRecord);
  }
  const GeometryPayload plane = PlaneGeometry{{}, {0, 0, 1}, {1, 0, 0}};
  EXPECT_TRUE(InvertSurface(plane, {2, 3, 0}, 0, {-100, 100}).ok());
  EXPECT_FALSE(
      InvertSurface(plane, {}, 0, {std::numeric_limits<double>::infinity(), 0})
          .ok());
}
TEST(ParasolidNurbsInvert, WorkBudgetBoundsHighDegreeUnsuccessfulSearches) {
  auto surface = Patch();
  surface.u.degree = surface.v.degree = 64;
  surface.u.control_count = surface.v.control_count = 512;
  for (auto* axis : {&surface.u, &surface.v}) {
    axis->knots.clear();
    for (int i = 0; i <= 448; ++i) axis->knots.push_back(i);
    axis->multiplicities.assign(449, 1);
    axis->multiplicities.front() = axis->multiplicities.back() = 65;
  }
  surface.controls.values.clear();
  surface.controls.values.reserve(512 * 512 * 3);
  for (int u = 0; u < 512; ++u)
    for (int v = 0; v < 512; ++v)
      surface.controls.values.insert(surface.controls.values.end(),
                                     {u / 511., v / 511., 0});
  const auto result = InvertSurface(surface, {.3, .7, 1}, 1e-12);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidNurbsInvert, PeriodicFlagsDoNotExtendTheSearchDomain) {
  auto surface = Patch();
  surface.u.closed = surface.v.closed = true;
  surface.u.periodic = surface.v.periodic = true;
  const auto point = EvaluateSurface(surface, .2, .7);
  ASSERT_TRUE(point.ok());
  EXPECT_TRUE(InvertSurface(surface, point.value().position, 1e-12).ok());
  EXPECT_FALSE(InvertSurface(surface, {}, 1e-12, {1.2, .7}).ok());
}
TEST(ParasolidNurbsInvert, UnclampedDomainAndModelOverload) {
  auto surface = Patch();
  surface.u.knots = {0, 1, 2, 3};
  surface.u.multiplicities = {1, 1, 1, 1};
  const auto point = EvaluateSurface(surface, 1.3, .4);
  ASSERT_TRUE(point.ok());
  const auto result = InvertSurface(surface, point.value().position, 1e-12);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().u, 1.3, 1e-12);
  EXPECT_FALSE(InvertSurface(surface, {}, 0, {.5, .5}).ok());
  test::ModelFixture f;
  f.Spline(false, true);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  // The fixture's collinear net is singular, but an exact UV hint is valid.
  const auto value = EvaluateSurface(model.value(), NodeId{1}, .25, .5);
  ASSERT_TRUE(value.ok());
  const auto inverse = InvertSurface(model.value(), NodeId{1},
                                     value.value().position, 0, {.25, .5});
  ASSERT_TRUE(inverse.ok());
  EXPECT_EQ(inverse.value().u, .25);
  EXPECT_EQ(inverse.value().v, .5);
}
}  // namespace
}  // namespace openparasolid
