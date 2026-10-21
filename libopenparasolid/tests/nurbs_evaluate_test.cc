// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
NurbsCurveGeometry Quadratic() {
  NurbsCurveGeometry curve;
  curve.axis.degree = 2;
  curve.axis.control_count = 3;
  curve.axis.knots = {0, 1};
  curve.axis.multiplicities = {3, 3};
  curve.controls.dimension = 3;
  curve.controls.values = {0, 0, 1, 1, 2, 1, 2, 0, 1};
  return curve;
}

void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

TEST(ParasolidNurbsEvaluate, QuadraticBezierIncludesBothDomainEndpoints) {
  const auto curve = Quadratic();
  for (double t : {0., 0.25, 0.5, 0.75, 1.}) {
    const auto result = EvaluateCurve(curve, t);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {2 * t, 4 * t * (1 - t), 1});
    ExpectVector(result.value().first_derivative, {2, 4 - 8 * t, 0});
  }
}

TEST(ParasolidNurbsEvaluate, RationalQuarterCircleUsesHomogeneousCoordinates) {
  auto curve = Quadratic();
  const double w = std::sqrt(0.5);
  curve.controls.dimension = 4;
  curve.controls.rational = true;
  curve.controls.values = {1, 0, 0, 1, w, w, 0, w, 0, 1, 0, 1};
  const auto result = EvaluateCurve(curve, 0.5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {w, w, 0});
  const double tangent = 2 / (1 + w);
  ExpectVector(result.value().first_derivative, {-tangent, tangent, 0});
  const auto start = EvaluateCurve(curve, 0);
  const auto end = EvaluateCurve(curve, 1);
  ASSERT_TRUE(start.ok());
  ASSERT_TRUE(end.ok());
  ExpectVector(start.value().first_derivative, {0, 2 * w, 0});
  ExpectVector(end.value().first_derivative, {-2 * w, 0, 0});
  // A common homogeneous scale must not change position or derivative.
  for (double scale : {1e-200, 1e200}) {
    auto scaled = curve;
    for (auto& value : scaled.controls.values) value *= scale;
    const auto sample = EvaluateCurve(scaled, 0.5);
    ASSERT_TRUE(sample.ok());
    ExpectVector(sample.value().position, result.value().position);
    ExpectVector(sample.value().first_derivative,
                 result.value().first_derivative);
  }
}

TEST(ParasolidNurbsEvaluate, TwoDimensionalControlsEmbedInTheXYPlane) {
  for (bool rational : {false, true}) {
    auto curve = Quadratic();
    curve.controls.dimension = rational ? 3 : 2;
    curve.controls.rational = rational;
    curve.controls.values = rational
                                ? std::vector<double>{0, 0, 1, 1, 2, 1, 2, 0, 1}
                                : std::vector<double>{0, 0, 1, 2, 2, 0};
    const auto result = EvaluateCurve(curve, 0.25);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {0.5, 0.75, 0});
    ExpectVector(result.value().first_derivative, {2, 2, 0});
  }
}

TEST(ParasolidNurbsEvaluate, RepeatedInteriorKnotUsesRightHandDerivative) {
  auto curve = Quadratic();
  curve.axis.control_count = 5;
  curve.axis.knots = {0, 1, 2};
  curve.axis.multiplicities = {3, 2, 3};
  curve.controls.values = {0, 0, 0, 1, 0, 0, 2, 0, 0, 2, 1, 0, 2, 2, 0};
  const auto result = EvaluateCurve(curve, 1);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {2, 0, 0});
  ExpectVector(result.value().first_derivative, {0, 2, 0});
  const auto before = EvaluateCurve(curve, std::nextafter(1., 0.));
  ASSERT_TRUE(before.ok());
  ExpectVector(before.value().first_derivative, {2, 0, 0});
}

TEST(ParasolidNurbsEvaluate, UnclampedKnotsUseTheDefinedDomain) {
  auto curve = Quadratic();
  curve.axis.knots = {-2, -1, 0, 1, 2, 3};
  curve.axis.multiplicities = {1, 1, 1, 1, 1, 1};
  const auto start = EvaluateCurve(curve, 0);
  const auto end = EvaluateCurve(curve, 1);
  ASSERT_TRUE(start.ok());
  ASSERT_TRUE(end.ok());
  ExpectVector(start.value().position, {0.5, 1, 1});
  ExpectVector(start.value().first_derivative, {1, 2, 0});
  ExpectVector(end.value().position, {1.5, 1, 1});
  ExpectVector(end.value().first_derivative, {1, -2, 0});
  for (double t : {-1., std::nextafter(0., -1.), std::nextafter(1., 2.), 2.}) {
    const auto outside = EvaluateCurve(curve, t);
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  }
}

TEST(ParasolidNurbsEvaluate, NonuniformKnotScaleChangesDerivative) {
  auto curve = Quadratic();
  curve.axis.knots = {10, 14};
  const auto result = EvaluateCurve(curve, 11);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {0.5, 0.75, 1});
  ExpectVector(result.value().first_derivative, {0.5, 0.5, 0});
}

TEST(ParasolidNurbsEvaluate, FirstDegreeAndMaximumSupportedDegree) {
  for (uint32_t degree : {1u, 64u}) {
    auto curve = Quadratic();
    curve.axis.degree = degree;
    curve.axis.control_count = degree + 1;
    curve.axis.multiplicities.assign(2, static_cast<uint16_t>(degree + 1));
    curve.controls.values.clear();
    for (uint32_t i = 0; i <= degree; ++i) {
      curve.controls.values.insert(curve.controls.values.end(),
                                   {double(i) / degree, 0, 0});
    }
    const auto result = EvaluateCurve(curve, 0.3);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {0.3, 0, 0});
    ExpectVector(result.value().first_derivative, {1, 0, 0});
  }
}

TEST(ParasolidNurbsEvaluate, PeriodicFlagsDoNotEnableExtrapolation) {
  auto curve = Quadratic();
  curve.axis.periodic = true;
  curve.axis.closed = true;
  EXPECT_TRUE(EvaluateCurve(curve, 0.5).ok());
  const auto outside = EvaluateCurve(curve, 1.5);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
}

TEST(ParasolidNurbsEvaluate, RejectsMalformedPublicPayloads) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int bad = 0; bad < 17; ++bad) {
    auto curve = Quadratic();
    switch (bad) {
      case 0:
        curve.axis.degree = 0;
        break;
      case 1:
        curve.axis.control_count = 2;
        break;
      case 2:
        curve.axis.knots.clear();
        break;
      case 3:
        curve.axis.multiplicities.pop_back();
        break;
      case 4:
        curve.axis.knots[1] = 0;
        break;
      case 5:
        curve.axis.knots[1] = nan;
        break;
      case 6:
        curve.axis.multiplicities[0] = 0;
        break;
      case 7:
        curve.axis.multiplicities[0] = 4;
        break;
      case 8:
        curve.axis.multiplicities[0] = 2;
        break;
      case 9:
        curve.controls.dimension = 1;
        break;
      case 10:
        curve.controls.values.pop_back();
        break;
      case 11:
        curve.controls.values[1] = nan;
        break;
      case 12:
        curve.axis.periodic = true;
        break;
      case 13:
        curve.controls.rational = true;
        curve.controls.values[2] = 0;
        break;
      case 14:
        curve.controls.rational = true;
        curve.controls.values[2] = -1;
        break;
      case 15:
        curve.axis.knots = {0, 1, 2};
        curve.axis.multiplicities = {2, 2, 2};  // Empty domain.
        break;
      case 16:
        curve.axis.control_count = 6;
        curve.axis.knots = {0, 1, 2};
        curve.axis.multiplicities = {3, 3, 3};  // Excess interior multiplicity.
        curve.controls.values.resize(18, 1);
        break;
    }
    const auto result = EvaluateCurve(curve, 0.5);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}

TEST(ParasolidNurbsEvaluate, RejectsExcessiveEvaluationResources) {
  for (int bad = 0; bad < 4; ++bad) {
    auto curve = Quadratic();
    switch (bad) {
      case 0:
        curve.axis.degree = 65;
        break;
      case 1:
        curve.controls.values.resize(1000001, 1);
        break;
      case 2:
        curve.axis.knots.resize(1000001, 1);
        break;
      case 3:
        curve.axis.control_count = UINT32_MAX;
        break;
    }
    const auto result = EvaluateCurve(curve, 0.5);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
}

TEST(ParasolidNurbsEvaluate, RejectsDerivativeOutsideDoubleRange) {
  auto curve = Quadratic();
  curve.axis.knots[1] = std::numeric_limits<double>::denorm_min();
  const auto result = EvaluateCurve(curve, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}

TEST(ParasolidNurbsEvaluate,
     ModelBuilderPayloadWorksWithoutChangingSourceData) {
  test::ModelFixture fixture;
  fixture.Spline(true);
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  ASSERT_EQ(model.value().geometry().size(), 1);
  const auto& geometry = model.value().geometry().front();
  const auto result = EvaluateCurve(geometry.payload, 0.5);
  ASSERT_TRUE(result.ok());
  // Homogeneous Bezier coefficients are (0,1,2,1),
  // (1,2,3,2), (2,3,4,3) in the fixture's original array order.
  ExpectVector(result.value().position, {0.5, 1, 1.5});
  ExpectVector(result.value().first_derivative, {0.5, 0, -0.5});
  const auto& curve = std::get<NurbsCurveGeometry>(geometry.payload);
  EXPECT_EQ(curve.controls.values[4], 1);
  EXPECT_EQ(curve.controls.values[7], 2);
}
}  // namespace
}  // namespace openparasolid
