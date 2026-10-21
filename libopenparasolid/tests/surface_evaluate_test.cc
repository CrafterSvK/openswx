// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
constexpr double kPi = std::numbers::pi;

void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

ConeGeometry Cone() {
  return {{0, 0, 0}, {0, 0, 1}, 2, std::sqrt(0.5), std::sqrt(0.5), {1, 0, 0}};
}

TEST(ParasolidSurfaceEvaluate, PlaneUsesAnObliqueRightHandedFrame) {
  const PlaneGeometry plane{{1, 2, 3}, {0, 0.6, 0.8}, {1, 0, 0}};
  const auto result = EvaluateSurface(plane, -2, 5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {-1, 6, 0});
  ExpectVector(result.value().u_derivative, {1, 0, 0});
  ExpectVector(result.value().v_derivative, {0, 0.8, -0.6});
}

TEST(ParasolidSurfaceEvaluate, CylinderUsesAngularUAndSignedAxialV) {
  const CylinderGeometry cylinder{{1, 2, 3}, {1, 0, 0}, 2, {0, 1, 0}};
  const auto result = EvaluateSurface(cylinder, kPi / 2, -4);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {-3, 2, 5});
  ExpectVector(result.value().u_derivative, {0, -2, 0});
  ExpectVector(result.value().v_derivative, {1, 0, 0});
}

TEST(ParasolidSurfaceEvaluate, ConeUsesSourceAxisForAxialParameter) {
  const auto result = EvaluateSurface(Cone(), 0, 3);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {5, 0, 3});
  ExpectVector(result.value().u_derivative, {0, 5, 0});
  ExpectVector(result.value().v_derivative, {1, 0, 1});
  auto cone = Cone();
  cone.sin_half_angle = 0.6;
  cone.cos_half_angle = 0.8;
  const auto acute = EvaluateSurface(cone, 0, 4);
  ASSERT_TRUE(acute.ok());
  ExpectVector(acute.value().position, {5, 0, 4});
  ExpectVector(acute.value().v_derivative, {0.75, 0, 1});
}

TEST(ParasolidSurfaceEvaluate, ConeAllowsTheApexAndRejectsTheOtherHalf) {
  const auto apex = EvaluateSurface(Cone(), 0, -2);
  ASSERT_TRUE(apex.ok());
  ExpectVector(apex.value().position, {0, 0, -2});
  ExpectVector(apex.value().u_derivative, {0, 0, 0});
  ExpectVector(apex.value().v_derivative, {1, 0, 1});
  const auto outside = EvaluateSurface(Cone(), 0, std::nextafter(-2., -3.));
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  auto at_origin = Cone();
  at_origin.radius = 0;
  EXPECT_TRUE(EvaluateSurface(at_origin, 0, 0).ok());
}

TEST(ParasolidSurfaceEvaluate, ConePreservesSignedHalfAngles) {
  auto cone = Cone();
  cone.sin_half_angle = -cone.sin_half_angle;
  const auto result = EvaluateSurface(cone, 0, 1);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1, 0, 1});
  ExpectVector(result.value().u_derivative, {0, 1, 0});
  ExpectVector(result.value().v_derivative, {-1, 0, 1});
  EXPECT_TRUE(EvaluateSurface(cone, 0, 2).ok());
  const auto beyond = EvaluateSurface(cone, 0, std::nextafter(2., 3.));
  ASSERT_FALSE(beyond.ok());
  EXPECT_EQ(beyond.error().code, ErrorCode::kInvalidOptions);
}

TEST(ParasolidSurfaceEvaluate, SphereUsesLongitudeLatitudeAndAllowsPoles) {
  const SphereGeometry sphere{{1, 2, 3}, 3, {0, 0, 1}, {1, 0, 0}};
  const auto equator = EvaluateSurface(sphere, kPi / 2, 0);
  ASSERT_TRUE(equator.ok());
  ExpectVector(equator.value().position, {1, 5, 3});
  ExpectVector(equator.value().u_derivative, {-3, 0, 0});
  ExpectVector(equator.value().v_derivative, {0, 0, 3});
  const auto pole = EvaluateSurface(sphere, kPi / 2, kPi / 2);
  ASSERT_TRUE(pole.ok());
  ExpectVector(pole.value().position, {1, 2, 6});
  ExpectVector(pole.value().u_derivative, {0, 0, 0});
  ExpectVector(pole.value().v_derivative, {0, -3, 0});
  EXPECT_TRUE(EvaluateSurface(sphere, 0, -kPi / 2).ok());
  for (double v : {std::nextafter(kPi / 2, kPi), -kPi}) {
    const auto outside = EvaluateSurface(sphere, 0, v);
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  }
}

TEST(ParasolidSurfaceEvaluate, RingTorusRetainsBothRadii) {
  const TorusGeometry torus{{0, 0, 0}, {0, 0, 1}, 5, 2, {1, 0, 0}};
  const auto result = EvaluateSurface(torus, 0, kPi / 2);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {5, 0, 2});
  ExpectVector(result.value().u_derivative, {0, 5, 0});
  ExpectVector(result.value().v_derivative, {-2, 0, 0});
  EXPECT_TRUE(EvaluateSurface(torus, -9 * kPi, 7 * kPi).ok());
}

TEST(ParasolidSurfaceEvaluate, AppleAndLemonToriKeepTheirPrincipalSheets) {
  for (double major : {1., -1., 2.}) {
    const TorusGeometry torus{{0, 0, 0}, {0, 0, 1}, major, 2, {1, 0, 0}};
    const auto middle = EvaluateSurface(torus, 0, 0);
    ASSERT_TRUE(middle.ok());
    ExpectVector(middle.value().position, {major + 2, 0, 0});
    ExpectVector(middle.value().u_derivative, {0, major + 2, 0});
    ExpectVector(middle.value().v_derivative, {0, 0, 2});
    const double limit = std::acos(-major / 2);
    for (double v : {-limit, limit}) {
      const auto endpoint = EvaluateSurface(torus, 0, v);
      ASSERT_TRUE(endpoint.ok());
      EXPECT_NEAR(endpoint.value().position.x, 0, 1e-12);
      ExpectVector(endpoint.value().u_derivative, {0, 0, 0});
    }
    const auto outside = EvaluateSurface(torus, 0, std::nextafter(limit, 10.));
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
    EXPECT_FALSE(EvaluateSurface(torus, 0, 2 * kPi).ok());
  }
}

TEST(ParasolidSurfaceEvaluate, PartialsAgreeWithCentralDifferences) {
  const GeometryPayload surfaces[] = {
      PlaneGeometry{{1, 2, 3}, {0, 0.6, 0.8}, {1, 0, 0}},
      CylinderGeometry{{1, 2, 3}, {0, 0.6, 0.8}, 3, {1, 0, 0}},
      Cone(),
      SphereGeometry{{1, 2, 3}, 4, {0, 0.6, 0.8}, {1, 0, 0}},
      TorusGeometry{{1, 2, 3}, {0, 0.6, 0.8}, 5, 2, {1, 0, 0}},
      TorusGeometry{{1, 2, 3}, {0, 0.6, 0.8}, -1, 2, {1, 0, 0}}};
  constexpr double h = 1e-5;
  for (const auto& surface : surfaces) {
    for (double u : {-1.3, 0.4, 2.2}) {
      const double v = 0.3;
      const auto middle = EvaluateSurface(surface, u, v);
      ASSERT_TRUE(middle.ok());
      for (bool vary_u : {false, true}) {
        const auto before = EvaluateSurface(surface, u - (vary_u ? h : 0),
                                            v - (vary_u ? 0 : h));
        const auto after = EvaluateSurface(surface, u + (vary_u ? h : 0),
                                           v + (vary_u ? 0 : h));
        ASSERT_TRUE(before.ok());
        ASSERT_TRUE(after.ok());
        const auto& a = before.value().position;
        const auto& b = after.value().position;
        ExpectVector(
            vary_u ? middle.value().u_derivative : middle.value().v_derivative,
            {(b.x - a.x) / (2 * h), (b.y - a.y) / (2 * h),
             (b.z - a.z) / (2 * h)},
            1e-8);
      }
    }
  }
}

TEST(ParasolidSurfaceEvaluate, RejectsNonfiniteParameters) {
  const PlaneGeometry plane{{}, {0, 0, 1}, {1, 0, 0}};
  for (double t : {std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::quiet_NaN()}) {
    for (bool use_u : {false, true}) {
      const auto result = EvaluateSurface(plane, use_u ? t : 0, use_u ? 0 : t);
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
    }
  }
}

TEST(ParasolidSurfaceEvaluate, RejectsMalformedPublicPayloads) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const GeometryPayload invalid[] = {
      PlaneGeometry{{nan, 0, 0}, {0, 0, 1}, {1, 0, 0}},
      PlaneGeometry{{}, {}, {1, 0, 0}},
      PlaneGeometry{{}, {0, 0, 1}, {2, 0, 0}},
      PlaneGeometry{{}, {0, 0, 1}, {0.6, 0, 0.8}},
      PlaneGeometry{{}, {0, 0, 1}, {nan, 0, 0}},
      CylinderGeometry{{}, {0, 0, 1}, 0, {1, 0, 0}},
      CylinderGeometry{{}, {0, 0, 1}, inf, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, -1, 0.6, 0.8, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, 1, 0, 1, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, 1, 1, 0, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, 1, 0.6, 0.6, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, 1, nan, 0.8, {1, 0, 0}},
      ConeGeometry{{}, {0, 0, 1}, 1, 0.6, -0.8, {1, 0, 0}},
      SphereGeometry{{}, -1, {0, 0, 1}, {1, 0, 0}},
      TorusGeometry{{}, {0, 0, 1}, 0, 2, {1, 0, 0}},
      TorusGeometry{{}, {0, 0, 1}, -2, 2, {1, 0, 0}},
      TorusGeometry{{}, {0, 0, 1}, -3, 2, {1, 0, 0}},
      TorusGeometry{{}, {0, 0, 1}, 3, 0, {1, 0, 0}},
      TorusGeometry{{}, {0, 0, 1}, inf, 2, {1, 0, 0}}};
  for (const auto& geometry : invalid) {
    const auto result = EvaluateSurface(geometry, 0, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}

TEST(ParasolidSurfaceEvaluate, FrameToleranceDoesNotNormalizeAcceptedAxes) {
  const double x = 1 + 5e-11;
  const auto result =
      EvaluateSurface(PlaneGeometry{{}, {0, 0, 1}, {x, 0, 0}}, 2, 3);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().position.x, 2 * x);
  EXPECT_EQ(result.value().u_derivative.x, x);
  EXPECT_EQ(result.value().v_derivative.y, x);
  EXPECT_FALSE(
      EvaluateSurface(PlaneGeometry{{}, {0, 0, 1}, {1 + 5e-9, 0, 0}}, 0, 0)
          .ok());
}

TEST(ParasolidSurfaceEvaluate, ChecksDoubleRangeBeforeReturningResults) {
  const double limit = std::numeric_limits<double>::max();
  const PlaneGeometry plane{{limit, 0, 0}, {0, 0, 1}, {1, 0, 0}};
  const auto cancelled = EvaluateSurface(plane, -limit, 0);
  ASSERT_TRUE(cancelled.ok());
  ExpectVector(cancelled.value().position, {0, 0, 0});
  EXPECT_FALSE(EvaluateSurface(plane, limit, 0).ok());
  EXPECT_FALSE(EvaluateSurface(
                   TorusGeometry{{}, {0, 0, 1}, limit, limit, {1, 0, 0}}, 0, 0)
                   .ok());
}

TEST(ParasolidSurfaceEvaluate, RejectsUnsupportedPayloadsExplicitly) {
  const GeometryPayload unsupported[] = {PointGeometry{}, LineGeometry{},
                                         CircleGeometry{}, EllipseGeometry{},
                                         NurbsCurveGeometry{}};
  for (const auto& geometry : unsupported) {
    const auto result = EvaluateSurface(geometry, 0, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}

TEST(ParasolidSurfaceEvaluate, ModelPayloadWorksWithoutApplyingSurfaceSense) {
  test::ModelFixture fixture;
  fixture.Add(50, 1,
              {test::ByteValue("sense", '-', FieldType::kCharacter),
               test::Numbers("origin", {1, 2, 3}, FieldType::kVector),
               test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
               test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector)});
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  ASSERT_EQ(model.value().geometry().size(), 1);
  const auto& geometry = model.value().geometry().front();
  ASSERT_EQ(geometry.sense, '-');
  const auto result = EvaluateSurface(geometry.payload, 4, -2);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {5, 0, 3});
  ExpectVector(result.value().u_derivative, {1, 0, 0});
  ExpectVector(result.value().v_derivative, {0, 1, 0});
}
}  // namespace
}  // namespace openparasolid
