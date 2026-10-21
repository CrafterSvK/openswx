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
  // P(u,v) = (2u, 3v, 2uv+v^2), with contiguous v rows.
  surface.controls.values = {0, 0, 0, 0, 1.5, 0, 0, 3, 1,
                             2, 0, 0, 2, 1.5, 1, 2, 3, 3};
  return surface;
}

void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

TEST(ParasolidNurbsSurfaceEvaluate,
     NonsquareNetUsesVFastestAndSeparateDegrees) {
  const auto surface = Patch();
  for (double u : {0., .25, 1.}) {
    for (double v : {0., .6, 1.}) {
      const auto result = EvaluateSurface(surface, u, v);
      ASSERT_TRUE(result.ok());
      ExpectVector(result.value().position, {2 * u, 3 * v, 2 * u * v + v * v});
      ExpectVector(result.value().u_derivative, {2, 0, 2 * v});
      ExpectVector(result.value().v_derivative, {0, 3, 2 * u + 2 * v});
    }
  }
}

TEST(ParasolidNurbsSurfaceEvaluate,
     RationalQuarterCylinderKeepsHomogeneousWeights) {
  auto surface = Patch();
  std::swap(surface.u, surface.v);
  surface.controls.rational = true;
  surface.controls.dimension = 4;
  const double w = std::sqrt(.5);
  surface.controls.values = {1, 0, 0,     1, 1, 0, 2, 1, w, w, 0, w,
                             w, w, 2 * w, w, 0, 1, 0, 1, 0, 1, 2, 1};
  const auto reference = EvaluateSurface(surface, .5, .3);
  ASSERT_TRUE(reference.ok());
  ExpectVector(reference.value().position, {w, w, .6});
  ExpectVector(reference.value().u_derivative, {-2 / (1 + w), 2 / (1 + w), 0});
  ExpectVector(reference.value().v_derivative, {0, 0, 2});
  for (double scale : {1e-200, 1e200}) {
    auto scaled = surface;
    for (auto& value : scaled.controls.values) value *= scale;
    const auto result = EvaluateSurface(scaled, .5, .3);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, reference.value().position);
    ExpectVector(result.value().u_derivative, reference.value().u_derivative);
    ExpectVector(result.value().v_derivative, reference.value().v_derivative);
  }
}

TEST(ParasolidNurbsSurfaceEvaluate,
     RationalPartialsIncludeBothWeightDerivatives) {
  auto surface = Patch();
  surface.v = surface.u;
  surface.controls.rational = true;
  surface.controls.dimension = 4;
  surface.controls.values = {0, 0, 0, 1, 0, 2, 0, 2, 3, 0, 0, 3, 5, 5, 5, 5};
  const auto result = EvaluateSurface(surface, .5, .5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {8. / 11, 7. / 11, 5. / 11});
  ExpectVector(result.value().u_derivative, {96. / 121, -4. / 121, 60. / 121});
  ExpectVector(result.value().v_derivative, {-4. / 121, 112. / 121, 80. / 121});
}

TEST(ParasolidNurbsSurfaceEvaluate,
     RepeatedKnotsUseTheCorrectSideInEitherAxis) {
  for (bool transpose : {false, true}) {
    auto surface = Patch();
    surface.v = surface.u;
    surface.u.control_count = 3;
    surface.u.knots = {0, 1, 2};
    surface.u.multiplicities = {2, 1, 2};
    surface.controls.values = {0, 0, 0, 0, 1, 0, 2, 0, 0,
                               2, 1, 0, 2, 0, 3, 2, 1, 3};
    if (transpose) {
      std::swap(surface.u, surface.v);
      surface.controls.values = {0, 0, 0, 2, 0, 0, 2, 0, 3,
                                 0, 1, 0, 2, 1, 0, 2, 1, 3};
    }
    const auto at =
        EvaluateSurface(surface, transpose ? .4 : 1, transpose ? 1 : .4);
    const double before_knot = std::nextafter(1., 0.);
    const auto before = EvaluateSurface(surface, transpose ? .4 : before_knot,
                                        transpose ? before_knot : .4);
    ASSERT_TRUE(at.ok());
    ASSERT_TRUE(before.ok());
    ExpectVector(at.value().position, {2, .4, 0});
    ExpectVector(transpose ? at.value().v_derivative : at.value().u_derivative,
                 {0, 0, 3});
    ExpectVector(
        transpose ? before.value().v_derivative : before.value().u_derivative,
        {2, 0, 0});
    ExpectVector(transpose ? at.value().u_derivative : at.value().v_derivative,
                 {0, 1, 0});
    EXPECT_TRUE(
        EvaluateSurface(surface, transpose ? 1 : 2, transpose ? 2 : 1).ok());
  }
}

TEST(ParasolidNurbsSurfaceEvaluate,
     ScaledDomainsRetainOriginalParameterDerivatives) {
  auto surface = Patch();
  surface.u.knots = {-3, 1};
  surface.v.knots = {10, 12};
  const auto result = EvaluateSurface(surface, -2, 11.2);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {.5, 1.8, .66});
  ExpectVector(result.value().u_derivative, {.5, 0, .3});
  ExpectVector(result.value().v_derivative, {0, 1.5, .85});
}

TEST(ParasolidNurbsSurfaceEvaluate, UnclampedAxesRejectImaginaryKnotIntervals) {
  auto surface = Patch();
  surface.u.knots = {0, 1, 2, 3};
  surface.u.multiplicities = {1, 1, 1, 1};
  surface.v.knots = {0, 1, 2, 3, 4, 5};
  surface.v.multiplicities = {1, 1, 1, 1, 1, 1};
  for (double u : {1., 2.}) {
    for (double v : {2., 3.}) EXPECT_TRUE(EvaluateSurface(surface, u, v).ok());
  }
  for (const auto& uv : {std::pair{.5, 2.5}, std::pair{1.5, 1.5},
                         std::pair{2.5, 2.5}, std::pair{1.5, 3.5}}) {
    const auto result = EvaluateSurface(surface, uv.first, uv.second);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}

TEST(ParasolidNurbsSurfaceEvaluate, MaximumDegreesRemainBoundedAndEvaluable) {
  auto surface = Patch();
  surface.u.degree = surface.v.degree = 64;
  surface.u.control_count = surface.v.control_count = 65;
  surface.u.multiplicities = surface.v.multiplicities = {65, 65};
  surface.controls.values.clear();
  for (int u = 0; u < 65; ++u) {
    for (int v = 0; v < 65; ++v) {
      surface.controls.values.insert(surface.controls.values.end(),
                                     {u / 64., v / 64., 0});
    }
  }
  const auto result = EvaluateSurface(surface, .2, .7);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {.2, .7, 0});
  ExpectVector(result.value().u_derivative, {1, 0, 0});
  ExpectVector(result.value().v_derivative, {0, 1, 0});
}

TEST(ParasolidNurbsSurfaceEvaluate, RejectsMalformedAxesAndControlsAtomically) {
  for (int bad = 0; bad < 13; ++bad) {
    auto surface = Patch();
    switch (bad) {
      case 0:
        surface.u.degree = 0;
        break;
      case 1:
        surface.v.degree = surface.v.control_count;
        break;
      case 2:
        surface.v.knots.clear();
        break;
      case 3:
        surface.v.multiplicities.pop_back();
        break;
      case 4:
        surface.v.knots[1] = 0;
        break;
      case 5:
        surface.controls.dimension = 2;
        break;
      case 6:
        surface.controls.values.pop_back();
        break;
      case 7:
        surface.controls.values[0] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 8:
        surface.v.periodic = true;
        break;
      case 9:
        surface.controls.rational = true;
        break;
      case 10:
        surface.u.multiplicities[0] = 0;
        break;
      case 11:
        surface.controls.rational = true;
        surface.controls.dimension = 4;
        surface.controls.values.assign(24, 1);
        surface.controls.values[7] = -1;
        break;
      case 12:
        surface.v.knots = {0, 1, 2};
        surface.v.multiplicities = {2, 2, 2};  // Empty v domain.
        break;
    }
    const auto result = EvaluateSurface(surface, .3, .4);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}

TEST(ParasolidNurbsSurfaceEvaluate, RejectsExcessiveDegreesAndControlProducts) {
  for (int bad = 0; bad < 3; ++bad) {
    auto surface = Patch();
    if (bad == 0) {
      surface.v.degree = 65;
    } else if (bad == 1) {
      surface.controls.values.resize(1000001, 1);
    } else {
      for (auto* axis : {&surface.u, &surface.v}) {
        axis->degree = 1;
        axis->control_count = 1001;
        axis->knots.clear();
        for (int i = 0; i < 1003; ++i) axis->knots.push_back(i);
        axis->multiplicities.assign(1003, 1);
      }
    }
    const auto result = EvaluateSurface(surface, .3, .4);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
}

TEST(ParasolidNurbsSurfaceEvaluate,
     RejectsNonfiniteParametersAndUnrepresentablePartials) {
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(EvaluateSurface(Patch(), value, .3).ok());
    EXPECT_FALSE(EvaluateSurface(Patch(), .3, value).ok());
  }
  for (bool v : {false, true}) {
    auto surface = Patch();
    (v ? surface.v : surface.u).knots[1] =
        std::numeric_limits<double>::denorm_min();
    const auto result = EvaluateSurface(surface, 0, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}

TEST(ParasolidNurbsSurfaceEvaluate, PeriodicFlagsDoNotWrapEitherParameter) {
  auto surface = Patch();
  surface.u.closed = surface.v.closed = true;
  surface.u.periodic = surface.v.periodic = true;
  EXPECT_TRUE(EvaluateSurface(surface, .3, .4).ok());
  EXPECT_FALSE(EvaluateSurface(surface, 1.3, .4).ok());
  EXPECT_FALSE(EvaluateSurface(surface, .3, 1.4).ok());
}

TEST(ParasolidNurbsSurfaceEvaluate,
     ModelPayloadPreservesSourceSenseAndControlOrder) {
  for (bool rational : {false, true}) {
    test::ModelFixture fixture;
    fixture.Spline(rational, true);
    const auto model = ModelBuilder::Build(fixture.report);
    ASSERT_TRUE(model.ok());
    ASSERT_EQ(model.value().geometry().size(), 1);
    const auto& geometry = model.value().geometry().front();
    ASSERT_EQ(geometry.sense, '-');
    const auto source_controls =
        std::get<NurbsSurfaceGeometry>(geometry.payload).controls.values;
    const auto result = EvaluateSurface(geometry.payload, .25, .5);
    ASSERT_TRUE(result.ok());
    if (rational) {
      ExpectVector(result.value().position, {.5, 1, 1.5});
      ExpectVector(result.value().u_derivative, {.5, 0, -.5});
      ExpectVector(result.value().v_derivative, {.25, 0, -.25});
    } else {
      ExpectVector(result.value().position, {1, 2, 3});
      ExpectVector(result.value().u_derivative, {2, 2, 2});
      ExpectVector(result.value().v_derivative, {1, 1, 1});
    }
    EXPECT_EQ(std::get<NurbsSurfaceGeometry>(geometry.payload).controls.values,
              source_controls);
  }
}
TEST(ParasolidNurbsSurfaceEvaluate, SingularPatchesCanReturnZeroPartials) {
  auto surface = Patch();
  surface.controls.values.assign(18, 0);
  const auto result = EvaluateSurface(surface, .3, .4);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {0, 0, 0});
  ExpectVector(result.value().u_derivative, {0, 0, 0});
  ExpectVector(result.value().v_derivative, {0, 0, 0});
}
}  // namespace
}  // namespace openparasolid
