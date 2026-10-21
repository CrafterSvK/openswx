// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
std::vector<Vector3> Circle() {
  std::vector<Vector3> result;
  for (double angle : {0., .3, .6, .9})
    result.push_back({std::cos(angle), std::sin(angle), 0});
  return result;
}
void Definition(ModelFixture& f, const std::vector<Vector3>& points = Circle(),
                bool swap = false) {
  f.Add(38, 1,
        {{{"surface", FieldType::kPointer, 1006, 2, true},
          {uint64_t{swap ? 51u : 50u}, uint64_t{swap ? 50u : 51u}}},
         Pointer("chart", 2, 40),
         Pointer("start", 3, 41),
         Pointer("end", 4, 41),
         ByteValue("sense", '-', FieldType::kCharacter)});
  std::vector<ScalarValue> positions;
  for (auto p : points)
    for (double value : {p.x, p.y, p.z}) positions.emplace_back(value);
  f.Add(40, 2,
        {Numbers("base_parameter", {-2.}),
         Numbers("base_scale", {3.}),
         Integer("chart_count", static_cast<int64_t>(points.size())),
         Numbers("chordal_error", {100.}),
         Numbers("angular_error", {1.}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               std::move(positions))});
  for (uint32_t id : {3u, 4u}) {
    const auto p = id == 3 ? points.front() : points.back();
    f.Add(41, id,
          {ByteValue("type", 'L', FieldType::kCharacter),
           Array("positions", FieldType::kIntersectionPoint, {p.x, p.y, p.z})});
  }
}
void Surfaces(ModelFixture& f, uint8_t plane_sense = '+',
              uint8_t sphere_sense = '+', double plane_z = 0) {
  f.Add(50, 50,
        {Numbers("origin", {0., 0., plane_z}, FieldType::kVector),
         Numbers("normal", {0., 0., 1.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         ByteValue("sense", plane_sense, FieldType::kCharacter)});
  f.Add(53, 51,
        {Numbers("center", {0., 0., 0.}, FieldType::kVector),
         Numbers("axis", {0., 0., 1.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         Numbers("radius", {1.}),
         ByteValue("sense", sphere_sense, FieldType::kCharacter)});
}
TEST(ParasolidIntersectionChart,
     ResolvesForwardSurfacesAndReturnsNaturalCircleData) {
  ModelFixture f;
  Definition(f);
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  ASSERT_TRUE(built.value().references_verified());
  const auto result =
      EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
  ASSERT_TRUE(result.ok());
  const auto& chart = result.value();
  ASSERT_EQ(chart.surface_parameters.size(), 4);
  ASSERT_EQ(chart.tangents.size(), 4);
  ASSERT_EQ(chart.parameterization.parameters.size(), 4);
  ASSERT_EQ(chart.parameterization.chord_scales.size(), 3);
  for (std::size_t i = 0; i < 4; ++i) {
    const double angle = static_cast<double>(i) * .3;
    const auto& parameters = chart.surface_parameters[i];
    EXPECT_NEAR(parameters[0].u, std::cos(angle), 1e-14);
    EXPECT_NEAR(parameters[0].v, std::sin(angle), 1e-14);
    EXPECT_NEAR(parameters[1].u, angle, 1e-14);
    EXPECT_EQ(parameters[1].v, 0);
    EXPECT_LE(parameters[0].residual, 1e-14);
    EXPECT_LE(parameters[1].residual, 1e-14);
    EXPECT_NEAR(chart.tangents[i].x, -std::sin(angle), 1e-14);
    EXPECT_NEAR(chart.tangents[i].y, std::cos(angle), 1e-14);
    EXPECT_EQ(chart.tangents[i].z, 0);
    EXPECT_NEAR(chart.parameterization.parameters[i],
                -2 + static_cast<double>(i) * 6 * std::sin(.15), 1e-14);
  }
  auto copy = built.value();
  auto moved = std::move(copy);
  EXPECT_TRUE(EvaluateIntersectionChart(moved, NodeId{1}, 1e-14).ok());
  // This prepares chart data, not exact curve evaluation between its points.
  const auto curve = EvaluateCurve(built.value(), NodeId{1}, -1);
  ASSERT_FALSE(curve.ok());
  EXPECT_EQ(curve.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidIntersectionChart,
     SurfaceSensesAndOrderDetermineTangentOrientation) {
  for (int configuration = 0; configuration < 4; ++configuration) {
    auto points = Circle();
    const bool reversed = configuration >= 2;
    if (reversed) std::reverse(points.begin(), points.end());
    ModelFixture f;
    Definition(f, points, configuration == 3);
    Surfaces(f, configuration == 1 || configuration == 2 ? '-' : '+',
             configuration == 1 ? '-' : '+');
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    const auto result =
        EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
    ASSERT_TRUE(result.ok());
    const double sign = reversed ? -1 : 1;
    EXPECT_NEAR(result.value().tangents[0].x, -sign * points[0].y, 1e-14);
    EXPECT_NEAR(result.value().tangents[0].y, sign * points[0].x, 1e-14);
  }
}
TEST(ParasolidIntersectionChart,
     MembershipToleranceIsExplicitAndErrorsLocatePoint) {
  auto points = Circle();
  points[1].z = 1e-4;
  ModelFixture f;
  Definition(f, points);
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto outside =
      EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-5);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(outside.error().message.find("point 1, surface 0"),
            std::string::npos);
  const auto within = EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-3);
  ASSERT_TRUE(within.ok());
  EXPECT_DOUBLE_EQ(within.value().surface_parameters[1][0].residual, 1e-4);
}
TEST(ParasolidIntersectionChart,
     MissingWrongUntypedAndUnsupportedSurfacesStayExplicit) {
  for (int bad = 0; bad < 5; ++bad) {
    ModelFixture f;
    Definition(f);
    Surfaces(f, bad == 4 ? '?' : '+');
    if (bad == 0) f.report.nodes[0].fields[0].values[0] = uint64_t{99};
    if (bad == 1) f.report.nodes[0].fields[0].values[0] = uint64_t{2};
    if (bad == 2) f.report.nodes[4].fields[1].values = {0., 0., 0.};
    if (bad == 3) {
      f.Add(124, 70, {ByteValue("sense", '+', FieldType::kCharacter)});
      f.report.nodes[0].fields[0].values[0] = uint64_t{70};
    }
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    const auto result =
        EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, bad < 2 ? ErrorCode::kInvalidRecord
                                           : ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidIntersectionChart, InvalidToleranceAndWrongRootRolesAreRejected) {
  ModelFixture f;
  Definition(f);
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  for (double tolerance : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    const auto result =
        EvaluateIntersectionChart(built.value(), NodeId{1}, tolerance);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  for (uint32_t id : {2u, 50u, 99u}) {
    const auto result = EvaluateIntersectionChart(built.value(), NodeId{id}, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  f.report.nodes[0].fields[1].values[0] = uint64_t{99};
  const auto invalid = ModelBuilder::Build(f.report);
  ASSERT_TRUE(invalid.ok());
  const auto unavailable =
      EvaluateIntersectionChart(invalid.value(), NodeId{1}, 0);
  ASSERT_FALSE(unavailable.ok());
  EXPECT_EQ(unavailable.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidIntersectionChart, ParallelAndSingularSurfaceNormalsAreRejected) {
  ModelFixture parallel;
  Definition(parallel);
  Surfaces(parallel);
  parallel.report.nodes[0].fields[0].values[1] = uint64_t{50};
  const auto a = ModelBuilder::Build(parallel.report);
  ASSERT_TRUE(a.ok());
  const auto tangent = EvaluateIntersectionChart(a.value(), NodeId{1}, 1e-14);
  ASSERT_FALSE(tangent.ok());
  EXPECT_EQ(tangent.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(tangent.error().message.find("parallel"), std::string::npos);
  ModelFixture singular;
  Definition(singular, {{0, 0, 1}, {0, 0, 1}});
  Surfaces(singular, '+', '+', 1);
  const auto b = ModelBuilder::Build(singular.report);
  ASSERT_TRUE(b.ok());
  const auto normal = EvaluateIntersectionChart(b.value(), NodeId{1}, 1e-14);
  ASSERT_FALSE(normal.ok());
  EXPECT_EQ(normal.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(normal.error().message.find("point 0, surface 1"),
            std::string::npos);
}
TEST(ParasolidIntersectionChart, ReversedChartOrderIsNotSilentlyCorrected) {
  ModelFixture f;
  auto points = Circle();
  std::reverse(points.begin(), points.end());
  Definition(f, points);
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto result =
      EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidIntersectionChart,
     ClosedChartsPreserveIncreasingUnwrappedParameters) {
  ModelFixture f;
  Definition(f, {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {1, 0, 0}});
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto result =
      EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().parameterization.parameters.back(),
              -2 + 12 * std::sqrt(2.), 1e-14);
  EXPECT_NEAR(result.value().surface_parameters[2][1].u, std::acos(-1.), 1e-14);
  EXPECT_LT(result.value().surface_parameters[3][1].u, 0);
}
TEST(ParasolidIntersectionChart,
     RequiresTwoChartPointsWithoutTerminatorExtension) {
  ModelFixture f;
  Definition(f, {{1, 0, 0}});
  Surfaces(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto result =
      EvaluateIntersectionChart(built.value(), NodeId{1}, 1e-14);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidIntersectionChart,
     BoundsChartBeforeSurfaceWorkOrOutputAllocation) {
  ModelFixture f;
  Definition(f, std::vector<Vector3>(1000000 / 3 + 1, {1, 0, 0}));
  // Missing surfaces must not hide the evaluator's stricter resource bound.
  ModelOptions options;
  options.max_values = 5000000;
  const auto built = ModelBuilder::Build(f.report, options);
  ASSERT_TRUE(built.ok());
  const auto result = EvaluateIntersectionChart(built.value(), NodeId{1}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
