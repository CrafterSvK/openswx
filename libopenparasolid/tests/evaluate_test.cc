// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/evaluate.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "model_test_helpers.h"

namespace openparasolid {
namespace {
constexpr double kTolerance = 1e-12;
constexpr double kPi = std::numbers::pi;

void ExpectVector(const Vector3& actual, Vector3 expected) {
  EXPECT_NEAR(actual.x, expected.x, kTolerance);
  EXPECT_NEAR(actual.y, expected.y, kTolerance);
  EXPECT_NEAR(actual.z, expected.z, kTolerance);
}

TEST(ParasolidEvaluate, LineUsesSignedUnboundedParameterAndSourceUnits) {
  const LineGeometry line{{10, -2, 7}, {0, 0.6, 0.8}};
  const auto result = EvaluateCurve(line, -5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {10, -5, 3});
  ExpectVector(result.value().first_derivative, line.direction);
}

TEST(ParasolidEvaluate, CircleUsesRightHandedFrameAndRadians) {
  // Normal +X and X axis +Y give the in-plane Y axis +Z.
  const CircleGeometry circle{{1, 2, 3}, {1, 0, 0}, {0, 1, 0}, 4};
  for (const double t : {kPi / 2, -3 * kPi / 2, 5 * kPi / 2}) {
    const auto result = EvaluateCurve(circle, t);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {1, 2, 7});
    ExpectVector(result.value().first_derivative, {0, -4, 0});
  }
}

TEST(ParasolidEvaluate, EllipseRetainsSeparateRadiiAndNaturalDerivative) {
  const EllipseGeometry ellipse{{1, 2, 3}, {0, 0, -1}, {1, 0, 0}, 5, 2};
  const auto start = EvaluateCurve(ellipse, 0);
  ASSERT_TRUE(start.ok());
  ExpectVector(start.value().position, {6, 2, 3});
  ExpectVector(start.value().first_derivative, {0, -2, 0});
  const auto quarter = EvaluateCurve(ellipse, kPi / 2);
  ASSERT_TRUE(quarter.ok());
  ExpectVector(quarter.value().position, {1, 0, 3});
  ExpectVector(quarter.value().first_derivative, {-5, 0, 0});
}

TEST(ParasolidEvaluate, ConicSupportsAnObliqueCoordinateFrame) {
  const CircleGeometry circle{{3, 2, 1}, {0, 0.6, 0.8}, {1, 0, 0}, 5};
  const auto result = EvaluateCurve(circle, kPi / 2);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {3, 6, -2});
  ExpectVector(result.value().first_derivative, {-5, 0, 0});
}

TEST(ParasolidEvaluate, FrameTolerancePreservesAcceptedSourceCoefficients) {
  const double direction = 1 + 5e-11;
  const auto accepted = EvaluateCurve(LineGeometry{{}, {direction, 0, 0}}, 2);
  ASSERT_TRUE(accepted.ok());
  EXPECT_EQ(accepted.value().position.x, 2 * direction);
  EXPECT_EQ(accepted.value().first_derivative.x, direction);
  EXPECT_FALSE(EvaluateCurve(LineGeometry{{}, {1 + 5e-9, 0, 0}}, 2).ok());
  EXPECT_TRUE(
      EvaluateCurve(CircleGeometry{{}, {0, 0, 1}, {1, 0, 5e-11}, 1}, 0).ok());
  EXPECT_FALSE(
      EvaluateCurve(CircleGeometry{{}, {0, 0, 1}, {1, 0, 5e-9}, 1}, 0).ok());
}

TEST(ParasolidEvaluate, DerivativesAgreeWithCentralDifferences) {
  const GeometryPayload curves[] = {
      LineGeometry{{1, 2, 3}, {0, 0.6, 0.8}},
      CircleGeometry{{-2, 3, 1}, {0, 1, 0}, {0, 0, 1}, 3},
      EllipseGeometry{{-2, 3, 1}, {0, 1, 0}, {0, 0, 1}, 5, 2}};
  constexpr double h = 1e-5;
  for (const auto& curve : curves) {
    for (const double t : {-1.7, 0.3, 2.4}) {
      const auto middle = EvaluateCurve(curve, t);
      const auto before = EvaluateCurve(curve, t - h);
      const auto after = EvaluateCurve(curve, t + h);
      ASSERT_TRUE(middle.ok());
      ASSERT_TRUE(before.ok());
      ASSERT_TRUE(after.ok());
      const auto& a = before.value().position;
      const auto& b = after.value().position;
      const auto& d = middle.value().first_derivative;
      EXPECT_NEAR(d.x, (b.x - a.x) / (2 * h), 1e-8);
      EXPECT_NEAR(d.y, (b.y - a.y) / (2 * h), 1e-8);
      EXPECT_NEAR(d.z, (b.z - a.z) / (2 * h), 1e-8);
    }
  }
}

TEST(ParasolidEvaluate, RejectsNonfiniteParameters) {
  for (const double t : {std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    const auto result = EvaluateCurve(LineGeometry{{}, {1, 0, 0}}, t);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}

TEST(ParasolidEvaluate, RejectsMalformedGeometryWithoutNormalizingIt) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const GeometryPayload invalid[] = {
      LineGeometry{{}, {}},
      LineGeometry{{}, {2, 0, 0}},
      LineGeometry{{inf, 0, 0}, {1, 0, 0}},
      LineGeometry{{}, {nan, 0, 0}},
      CircleGeometry{{}, {0, 0, 1}, {1, 0, 0}, 0},
      CircleGeometry{{}, {0, 0, 1}, {1, 0, 0}, -1},
      CircleGeometry{{}, {0, 0, 1}, {1, 0, 0}, inf},
      CircleGeometry{{nan, 0, 0}, {0, 0, 1}, {1, 0, 0}, 1},
      CircleGeometry{{}, {1, 0, 0}, {1, 0, 0}, 1},
      CircleGeometry{{}, {0, 0, 1}, {0.6, 0, 0.8}, 1},
      CircleGeometry{{}, {0, 0, 2}, {1, 0, 0}, 1},
      EllipseGeometry{{}, {0, 0, 1}, {1, 0, 0}, 2, nan},
      EllipseGeometry{{}, {0, 0, 1}, {1, 0, 0}, 2, -1}};
  for (const auto& geometry : invalid) {
    const auto result = EvaluateCurve(geometry, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}

TEST(ParasolidEvaluate, RejectsOverflowAndAllowsFiniteExtremeResults) {
  const double largest = std::numeric_limits<double>::max();
  EXPECT_FALSE(
      EvaluateCurve(LineGeometry{{largest, 0, 0}, {1, 0, 0}}, largest).ok());
  const auto cancelled =
      EvaluateCurve(LineGeometry{{-largest, 0, 0}, {1, 0, 0}}, largest);
  ASSERT_TRUE(cancelled.ok());
  EXPECT_EQ(cancelled.value().position.x, 0);
  EXPECT_FALSE(
      EvaluateCurve(
          CircleGeometry{{largest, 0, 0}, {0, 0, 1}, {1, 0, 0}, largest}, 0)
          .ok());
}

TEST(ParasolidEvaluate, RejectsUnsupportedPayloadsExplicitly) {
  const GeometryPayload unsupported[] = {
      PointGeometry{},      PlaneGeometry{},       CylinderGeometry{},
      ConeGeometry{},       SphereGeometry{},      TorusGeometry{},
      NurbsCurveGeometry{}, NurbsSurfaceGeometry{}};
  for (const auto& geometry : unsupported) {
    const auto result = EvaluateCurve(geometry, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}

TEST(ParasolidEvaluate, ModelPayloadEvaluatesIndependentlyOfCurveSense) {
  test::ModelFixture fixture;
  fixture.Add(30, 1,
              {test::ByteValue("sense", '-', FieldType::kCharacter),
               test::Numbers("origin", {1, 2, 3}, FieldType::kVector),
               test::Numbers("direction", {0, 0, 1}, FieldType::kVector)});
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  ASSERT_EQ(model.value().geometry().size(), 1);
  const auto& geometry = model.value().geometry().front();
  ASSERT_EQ(geometry.sense, '-');
  const auto evaluated = EvaluateCurve(geometry.payload, 4);
  ASSERT_TRUE(evaluated.ok());
  ExpectVector(evaluated.value().position, {1, 2, 7});
  ExpectVector(evaluated.value().first_derivative, {0, 0, 1});
}
}  // namespace
}  // namespace openparasolid
