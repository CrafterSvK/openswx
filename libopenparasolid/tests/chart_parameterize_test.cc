// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
IntersectionChart Straight() {
  IntersectionChart chart;
  chart.base_parameter = -3;
  chart.base_scale = 2;
  chart.positions = {{0, 0, 0}, {1, 0, 0}, {4, 0, 0}};
  return chart;
}
TEST(ParasolidChartParameters, StraightChartPreservesOriginalBaseAndScale) {
  const auto chart = Straight();
  const std::array<Vector3, 3> tangents{{{1, 0, 0}, {2, 0, 0}, {3, 0, 0}}};
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().parameters, (std::vector<double>{-3, -1, 5}));
  EXPECT_EQ(result.value().chord_scales, (std::vector<double>{2, 2}));
  EXPECT_EQ(chart.positions[2].x, 4);
  EXPECT_EQ(tangents[1].x, 2);
}
TEST(ParasolidChartParameters, UnequalChordsUseTangentRatiosForC1Projection) {
  IntersectionChart chart;
  chart.base_parameter = 5;
  chart.base_scale = 2;
  chart.positions = {{0, 0, 0}, {1, 0, 0}, {2, 1, 0}, {2, 3, 0}};
  const std::array<Vector3, 4> tangents{
      {{1, 0, 0}, {2, 1, 0}, {1, 2, 0}, {0, 1, 0}}};
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().parameters.size(), 4);
  ASSERT_EQ(result.value().chord_scales.size(), 3);
  EXPECT_DOUBLE_EQ(result.value().parameters[0], 5);
  EXPECT_DOUBLE_EQ(result.value().parameters[1], 7);
  EXPECT_NEAR(result.value().parameters[2], 29. / 3, 1e-14);
  EXPECT_NEAR(result.value().parameters[3], 41. / 3, 1e-14);
  EXPECT_NEAR(result.value().chord_scales[1], 4 * std::sqrt(2.) / 3, 1e-14);
  EXPECT_NEAR(result.value().chord_scales[2], 2, 1e-14);
  // The implied dP/dt agrees from either chord at the interior point.
  const double incoming = result.value().chord_scales[0] * 2 / std::sqrt(5.);
  const double outgoing = result.value().chord_scales[1] * 3 / std::sqrt(10.);
  EXPECT_NEAR(incoming, outgoing, 1e-14);
}
TEST(ParasolidChartParameters, ClosedChartMayRepeatItsFirstPointAtTheEnd) {
  IntersectionChart chart;
  chart.base_scale = 1;
  chart.positions = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {1, 0, 0}};
  const std::array<Vector3, 5> tangents{
      {{0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {1, 0, 0}, {0, 1, 0}}};
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().parameters.back(), 4 * std::sqrt(2.), 1e-14);
  for (double scale : result.value().chord_scales) EXPECT_DOUBLE_EQ(scale, 1);
}
TEST(ParasolidChartParameters, OrientedSurfaceNormalsSupplyCircleTangents) {
  using namespace test;
  ModelFixture fixture;
  fixture.Add(50, 1,
              {Numbers("origin", {0., 0., 0.}, FieldType::kVector),
               Numbers("normal", {0., 0., 1.}, FieldType::kVector),
               Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
               ByteValue("sense", '+', FieldType::kCharacter)});
  fixture.Add(53, 2,
              {Numbers("center", {0., 0., 0.}, FieldType::kVector),
               Numbers("axis", {0., 0., 1.}, FieldType::kVector),
               Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
               Numbers("radius", {1.}),
               ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  IntersectionChart chart;
  chart.base_scale = 2;
  std::vector<Vector3> tangents;
  for (double angle : {0., .3, .6, .9}) {
    const auto point = EvaluateSurface(model.value(), NodeId{2}, angle, 0);
    const auto a = EvaluateSurfaceNormal(model.value(), NodeId{1}, 0, 0);
    const auto b = EvaluateSurfaceNormal(model.value(), NodeId{2}, angle, 0);
    ASSERT_TRUE(point.ok());
    ASSERT_TRUE(a.ok());
    ASSERT_TRUE(b.ok());
    chart.positions.push_back(point.value().position);
    const auto& n = a.value();
    const auto& m = b.value();
    tangents.push_back(
        {n.y * m.z - n.z * m.y, n.z * m.x - n.x * m.z, n.x * m.y - n.y * m.x});
  }
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  for (std::size_t i = 0; i < chart.positions.size(); ++i)
    EXPECT_NEAR(result.value().parameters[i],
                static_cast<double>(i) * 4 * std::sin(.15), 1e-14);
}
TEST(ParasolidChartParameters, RejectsMismatchedCountsAndShortCharts) {
  const std::array<Vector3, 1> tangent{{{1, 0, 0}}};
  const auto mismatch = ParameterizeIntersectionChart(Straight(), tangent);
  ASSERT_FALSE(mismatch.ok());
  EXPECT_EQ(mismatch.error().code, ErrorCode::kInvalidOptions);
  IntersectionChart chart;
  chart.base_scale = 1;
  EXPECT_FALSE(ParameterizeIntersectionChart(chart, {}).ok());
  chart.positions.push_back({});
  EXPECT_FALSE(ParameterizeIntersectionChart(chart, tangent).ok());
}
TEST(ParasolidChartParameters, RejectsNonfinitePositionsTangentsAndMetadata) {
  for (int bad = 0; bad < 6; ++bad) {
    auto chart = Straight();
    std::array<Vector3, 3> tangents{{{1, 0, 0}, {1, 0, 0}, {1, 0, 0}}};
    const double infinity = std::numeric_limits<double>::infinity();
    if (bad == 0) chart.base_parameter = infinity;
    if (bad == 1) chart.base_scale = infinity;
    if (bad == 2) chart.base_scale = -1;
    if (bad == 3) chart.positions[1].z = infinity;
    if (bad == 4) tangents[1].y = std::numeric_limits<double>::quiet_NaN();
    if (bad == 5) tangents[1] = {};
    const auto result = ParameterizeIntersectionChart(chart, tangents);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidChartParameters,
     RejectsZeroChordsAndWrongOrPerpendicularTangents) {
  for (int bad = 0; bad < 5; ++bad) {
    auto chart = Straight();
    std::array<Vector3, 3> tangents{{{1, 0, 0}, {1, 0, 0}, {1, 0, 0}}};
    if (bad == 0) chart.positions[1] = chart.positions[0];
    if (bad == 1) tangents[0] = {-1, 0, 0};
    if (bad == 2) tangents[1] = {0, 1, 0};
    if (bad == 3) tangents[2] = {-1, 0, 0};
    if (bad == 4) chart.positions[2] = {-1, 0, 0};
    const auto result = ParameterizeIntersectionChart(chart, tangents);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidChartParameters,
     RejectsOverflowAndNonIncreasingRoundedParameters) {
  const std::array<Vector3, 3> tangents{{{1, 0, 0}, {1, 0, 0}, {1, 0, 0}}};
  auto chart = Straight();
  chart.base_parameter = std::numeric_limits<double>::max();
  EXPECT_FALSE(ParameterizeIntersectionChart(chart, tangents).ok());
  chart.base_parameter = 0;
  chart.base_scale = std::numeric_limits<double>::max();
  EXPECT_FALSE(ParameterizeIntersectionChart(chart, tangents).ok());
  chart.base_parameter = 1e20;
  chart.base_scale = 1;
  EXPECT_FALSE(ParameterizeIntersectionChart(chart, tangents).ok());
}
TEST(ParasolidChartParameters, TangentRatioOverflowAndUnderflowAreExplicit) {
  for (bool overflow : {false, true}) {
    IntersectionChart chart;
    chart.base_scale = overflow ? 1e300 : 1e-300;
    chart.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    const double tiny = 1e-300;
    const std::array<Vector3, 3> tangents{
        {{1, 0, 0},
         overflow ? Vector3{1, tiny, 0} : Vector3{tiny, 1, 0},
         {0, 1, 0}}};
    const auto result = ParameterizeIntersectionChart(chart, tangents);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidChartParameters,
     TinyChordsAndExtremeTangentMagnitudesAreSupported) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  IntersectionChart chart;
  chart.base_parameter = -0.;
  chart.base_scale = 1;
  chart.positions = {{}, {tiny, 0, 0}};
  const std::array<Vector3, 2> tangents{{{tiny, 0, 0}, {1e300, 0, 0}}};
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(std::signbit(result.value().parameters[0]));
  EXPECT_EQ(result.value().parameters[1], tiny);
}
TEST(ParasolidChartParameters, WideChordsCanCancelAgainstNegativeBase) {
  if (std::numeric_limits<long double>::max_exponent <=
      std::numeric_limits<double>::max_exponent)
    GTEST_SKIP() << "Requires wider intermediate exponent range";
  const double large = std::numeric_limits<double>::max();
  IntersectionChart chart;
  chart.base_parameter = -large;
  chart.base_scale = .5;
  chart.positions = {{-large, 0, 0}, {large, 0, 0}};
  const std::array<Vector3, 2> tangents{{{1, 0, 0}, {1, 0, 0}}};
  const auto result = ParameterizeIntersectionChart(chart, tangents);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().parameters[1], 0);
}
TEST(ParasolidChartParameters, ResourceLimitPrecedesResultAllocation) {
  IntersectionChart chart;
  chart.positions.resize(1000000 / 3 + 1);
  const auto result = ParameterizeIntersectionChart(chart, {});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
