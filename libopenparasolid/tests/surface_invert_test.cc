// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
std::vector<GeometryPayload> Surfaces() {
  const Vector3 origin{1, -2, 3}, axis{0, .6, .8}, x{1, 0, 0};
  return {PlaneGeometry{origin, axis, x}, CylinderGeometry{origin, axis, 2, x},
          ConeGeometry{origin, axis, 2, .6, .8, x},
          SphereGeometry{origin, 2, axis, x},
          TorusGeometry{origin, axis, 3, 1, x}};
}
TEST(ParasolidSurfaceInvert,
     RecoversNaturalParametersInTranslatedObliqueFrames) {
  const auto surfaces = Surfaces();
  for (std::size_t i = 0; i < surfaces.size(); ++i) {
    for (double u : {-2.4, -.7, 0., 1.4, 3.}) {
      for (double v : {-1., 0., .8}) {
        const auto point = EvaluateSurface(surfaces[i], u, v);
        ASSERT_TRUE(point.ok());
        const auto result =
            InvertSurface(surfaces[i], point.value().position, 1e-12);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_NEAR(result.value().u, u, 1e-12);
        EXPECT_NEAR(result.value().v, v, 1e-12);
        EXPECT_LE(result.value().residual, 1e-12);
      }
    }
  }
}
TEST(ParasolidSurfaceInvert, MembershipUsesOnlyTheExplicitAbsoluteTolerance) {
  const GeometryPayload plane = PlaneGeometry{{}, {0, 0, 1}, {1, 0, 0}};
  const auto exact = InvertSurface(plane, {2, 3, 0}, 0);
  ASSERT_TRUE(exact.ok());
  EXPECT_EQ(exact.value().u, 2);
  EXPECT_EQ(exact.value().v, 3);
  EXPECT_EQ(exact.value().residual, 0);
  const auto outside = InvertSurface(plane, {2, 3, .1}, .09);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  const auto within = InvertSurface(plane, {2, 3, .1}, .1);
  ASSERT_TRUE(within.ok());
  EXPECT_EQ(within.value().residual, .1);
}
TEST(ParasolidSurfaceInvert, InvertsActualAcceptedFrameWithoutNormalizingAxes) {
  const Vector3 axis{0, 0, 1 + 4e-11}, x{1, 0, 3e-11};
  for (const GeometryPayload geometry :
       {GeometryPayload{PlaneGeometry{{}, axis, x}},
        GeometryPayload{CylinderGeometry{{}, axis, 2, x}}}) {
    const auto point = EvaluateSurface(geometry, .4, 3);
    ASSERT_TRUE(point.ok());
    const auto result = InvertSurface(geometry, point.value().position, 1e-14);
    ASSERT_TRUE(result.ok());
    EXPECT_NEAR(result.value().u, .4, 1e-14);
    EXPECT_NEAR(result.value().v, 3, 1e-14);
  }
}
TEST(ParasolidSurfaceInvert, AngularSeamsUsePrincipalParameters) {
  const GeometryPayload cylinder =
      CylinderGeometry{{}, {0, 0, 1}, 2, {1, 0, 0}};
  const auto point = EvaluateSurface(cylinder, 4, -2);
  ASSERT_TRUE(point.ok());
  const auto result = InvertSurface(cylinder, point.value().position, 1e-14);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().u, 4 - 2 * std::numbers::pi, 1e-14);
  EXPECT_EQ(result.value().v, -2);
  const auto seam = InvertSurface(cylinder, {-2, 0, 1}, 1e-14);
  ASSERT_TRUE(seam.ok());
  EXPECT_NEAR(std::abs(seam.value().u), std::numbers::pi, 1e-14);
}
TEST(ParasolidSurfaceInvert, SignedConeSlopeKeepsTheWriterAxialConvention) {
  for (double sine : {-.6, .6}) {
    const GeometryPayload cone =
        ConeGeometry{{}, {0, 0, 1}, 2, sine, .8, {1, 0, 0}};
    const auto point = EvaluateSurface(cone, .3, 1);
    ASSERT_TRUE(point.ok());
    const auto result = InvertSurface(cone, point.value().position, 1e-14);
    ASSERT_TRUE(result.ok());
    EXPECT_NEAR(result.value().u, .3, 1e-14);
    EXPECT_EQ(result.value().v, 1);
    const auto beyond = InvertSurface(cone, {0, 0, sine > 0 ? -10. : 10.}, 0);
    ASSERT_FALSE(beyond.ok());
    EXPECT_EQ(beyond.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSurfaceInvert, PolesAndApexHaveDeterministicUndefinedAzimuth) {
  const GeometryPayload sphere = SphereGeometry{{}, 2, {0, 0, 1}, {1, 0, 0}};
  for (double z : {-2., 2.}) {
    const auto result = InvertSurface(sphere, {0, 0, z}, 1e-14);
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.value().u, 0);
    EXPECT_EQ(result.value().v, std::copysign(std::numbers::pi / 2, z));
  }
  const GeometryPayload cone =
      ConeGeometry{{}, {0, 0, 1}, 0, .6, .8, {1, 0, 0}};
  const auto apex = InvertSurface(cone, {}, 0);
  ASSERT_TRUE(apex.ok());
  EXPECT_EQ(apex.value().u, 0);
  EXPECT_EQ(apex.value().v, 0);
}
TEST(ParasolidSurfaceInvert, RingAppleLemonAndHornToriUsePrincipalSheet) {
  for (double major : {3., .5, -.5, 1.}) {
    const GeometryPayload torus =
        TorusGeometry{{}, {0, 0, 1}, major, 1, {1, 0, 0}};
    const double bound = major > 1 ? std::numbers::pi : std::acos(-major);
    for (double v : {-.8 * bound, 0., .8 * bound, bound}) {
      const auto point = EvaluateSurface(torus, .4, v);
      ASSERT_TRUE(point.ok());
      const auto result = InvertSurface(torus, point.value().position, 1e-13);
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto checked =
          EvaluateSurface(torus, result.value().u, result.value().v);
      ASSERT_TRUE(checked.ok());
      EXPECT_LE(result.value().residual, 1e-13);
      if (major <= 1) EXPECT_LE(std::abs(result.value().v), bound);
    }
  }
}
TEST(ParasolidSurfaceInvert,
     ExtremeRadiiAndWideCoordinateDifferencesAreChecked) {
  for (double radius : {1e-300, 1e300}) {
    const GeometryPayload sphere =
        SphereGeometry{{}, radius, {0, 0, 1}, {1, 0, 0}};
    const auto result = InvertSurface(sphere, {radius, 0, 0}, 0);
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.value().u, 0);
    EXPECT_EQ(result.value().v, 0);
  }
  const double largest = std::numeric_limits<double>::max();
  const GeometryPayload plane =
      PlaneGeometry{{-largest, 0, 0}, {0, 0, 1}, {1, 0, 0}};
  const auto result = InvertSurface(plane, {largest, 0, 0}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidSurfaceInvert, InvalidRequestsAndAnalyticDefinitionsAreExplicit) {
  const GeometryPayload plane = PlaneGeometry{{}, {0, 0, 1}, {1, 0, 0}};
  for (double tolerance : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    const auto result = InvertSurface(plane, {}, tolerance);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_FALSE(
      InvertSurface(plane, {std::numeric_limits<double>::infinity(), 0, 0}, 0)
          .ok());
  for (const GeometryPayload invalid :
       {GeometryPayload{PlaneGeometry{{}, {0, 0, 0}, {1, 0, 0}}},
        GeometryPayload{CylinderGeometry{{}, {0, 0, 2}, 1, {1, 0, 0}}},
        GeometryPayload{SphereGeometry{{}, -1, {0, 0, 1}, {1, 0, 0}}}}) {
    const auto result = InvertSurface(invalid, {1, 0, 0}, 1);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSurfaceInvert,
     ModelLookupPreservesSenseAndRejectsUnavailableKinds) {
  using namespace test;
  ModelFixture f;
  f.Add(50, 1,
        {Numbers("origin", {0., 0., 0.}, FieldType::kVector),
         Numbers("normal", {0., 0., 1.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         ByteValue("sense", '?', FieldType::kCharacter)});
  f.Point(2);
  f.Add(60, 3, {ByteValue("sense", '+', FieldType::kCharacter)});
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto result = InvertSurface(built.value(), NodeId{1}, {2, 3, 0}, 0);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().u, 2);
  EXPECT_EQ(result.value().v, 3);
  for (uint32_t id : {2u, 3u}) {
    const auto unavailable = InvertSurface(built.value(), NodeId{id}, {}, 0);
    ASSERT_FALSE(unavailable.ok());
    EXPECT_EQ(unavailable.error().code, ErrorCode::kUnsupportedFormat);
  }
  const auto absent = InvertSurface(built.value(), NodeId{99}, {}, 0);
  ASSERT_FALSE(absent.ok());
  EXPECT_EQ(absent.error().code, ErrorCode::kInvalidRecord);
  for (const GeometryPayload other :
       {GeometryPayload{NurbsSurfaceGeometry{}},
        GeometryPayload{SpunSurfaceGeometry{}},
        GeometryPayload{SweptSurfaceGeometry{}}}) {
    const auto unsupported = InvertSurface(other, {}, 0);
    ASSERT_FALSE(unsupported.ok());
    EXPECT_EQ(unsupported.error().code, ErrorCode::kUnsupportedFormat);
  }
}
}  // namespace
}  // namespace openparasolid
