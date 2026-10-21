// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
void Definition(ModelFixture& f, const std::vector<Vector3>& points) {
  f.Add(38, 1,
        {{{"surface", FieldType::kPointer, 1006, 2, true},
          {uint64_t{50}, uint64_t{51}}},
         Pointer("chart", 2, 40),
         Pointer("start", 3, 41),
         Pointer("end", 4, 41),
         ByteValue("sense", '-', FieldType::kCharacter)});
  std::vector<ScalarValue> positions;
  for (auto point : points)
    for (double value : {point.x, point.y, point.z})
      positions.emplace_back(value);
  f.Add(40, 2,
        {Numbers("base_parameter", {-2.}),
         Numbers("base_scale", {3.}),
         Integer("chart_count", static_cast<int64_t>(points.size())),
         Numbers("chordal_error", {.1}),
         Numbers("angular_error", {1.}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               std::move(positions))});
  for (uint32_t id : {3u, 4u}) {
    const auto point = id == 3 ? points.front() : points.back();
    f.Add(41, id,
          {ByteValue("type", 'L', FieldType::kCharacter),
           Array("positions", FieldType::kIntersectionPoint,
                 {point.x, point.y, point.z})});
  }
}
void Plane(ModelFixture& f, double height = 0) {
  f.Add(50, 50,
        {Numbers("origin", {0., 0., height}, FieldType::kVector),
         Numbers("normal", {0., 0., 1.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
}
std::vector<Vector3> CirclePoints(const std::vector<double>& angles,
                                  double radius = 1, double height = 0) {
  std::vector<Vector3> points;
  for (double angle : angles)
    points.push_back(
        {radius * std::cos(angle), radius * std::sin(angle), height});
  return points;
}
void Sphere(ModelFixture& f) {
  f.Add(53, 51,
        {Numbers("center", {0., 0., 0.}, FieldType::kVector),
         Numbers("axis", {0., 0., 1.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         Numbers("radius", {1.}),
         ByteValue("sense", '+', FieldType::kCharacter)});
}
TEST(ParasolidIntersectionEvaluate,
     SolvesCircleAndNaturalDerivativeUsingChordPlane) {
  const double height = .3, radius = std::sqrt(1 - height * height);
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}, radius, height));
  Plane(f, height);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  for (double fraction : {.1, .3, .5, .9}) {
    const double t = std::lerp(times[0], times[1], fraction);
    const auto sample =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {1e-13, 1e-13});
    ASSERT_TRUE(sample.ok()) << sample.error().message;
    const double angle = .4 + std::asin((2 * fraction - 1) * std::sin(.4));
    const auto& p = sample.value().position;
    EXPECT_NEAR(p.x, radius * std::cos(angle), 1e-12);
    EXPECT_NEAR(p.y, radius * std::sin(angle), 1e-12);
    EXPECT_NEAR(p.z, height, 1e-13);
    EXPECT_NEAR(std::hypot(p.x, p.y, p.z), 1, 1e-13);
    const double factor = 1 / (3 * std::cos(angle - .4));
    EXPECT_NEAR(sample.value().first_derivative.x, -std::sin(angle) * factor,
                1e-12);
    EXPECT_NEAR(sample.value().first_derivative.y, std::cos(angle) * factor,
                1e-12);
    EXPECT_NEAR(sample.value().first_derivative.z, 0, 1e-13);
    const double h = 1e-5;
    const auto a = EvaluateIntersectionCurve(model.value(), NodeId{1}, t - h,
                                             {1e-13, 1e-13});
    const auto b = EvaluateIntersectionCurve(model.value(), NodeId{1}, t + h,
                                             {1e-13, 1e-13});
    ASSERT_TRUE(a.ok());
    ASSERT_TRUE(b.ok());
    EXPECT_NEAR((b.value().position.x - a.value().position.x) / (2 * h),
                sample.value().first_derivative.x, 1e-8);
    EXPECT_NEAR((b.value().position.y - a.value().position.y) / (2 * h),
                sample.value().first_derivative.y, 1e-8);
  }
}
TEST(ParasolidIntersectionEvaluate,
     ChartKnotsRetainStoredPointsAndC1Derivatives) {
  const auto points = CirclePoints({0, .4, 1.1});
  ModelFixture f;
  Definition(f, points);
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto sample = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                  times[i], {1e-13, 1e-13});
    ASSERT_TRUE(sample.ok());
    EXPECT_EQ(sample.value().position.x, points[i].x);
    EXPECT_EQ(sample.value().position.y, points[i].y);
  }
  const auto left = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                              times[1] - 1e-7, {1e-13, 1e-13});
  const auto right = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                               times[1] + 1e-7, {1e-13, 1e-13});
  ASSERT_TRUE(left.ok());
  ASSERT_TRUE(right.ok());
  EXPECT_NEAR(left.value().first_derivative.x, right.value().first_derivative.x,
              1e-7);
  EXPECT_NEAR(left.value().first_derivative.y, right.value().first_derivative.y,
              1e-7);
}
TEST(ParasolidIntersectionEvaluate, AngularSeamUsesLocalBranchForInitialGuess) {
  ModelFixture f;
  Definition(f, CirclePoints({2.9, 3.4}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  const auto sample = EvaluateIntersectionCurve(
      model.value(), NodeId{1}, std::midpoint(times[0], times[1]),
      {1e-13, 1e-13});
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().position.x, std::cos(3.15), 1e-12);
  EXPECT_NEAR(sample.value().position.y, std::sin(3.15), 1e-12);
}
TEST(ParasolidIntersectionEvaluate,
     PlanePlaneIntersectionIsLinearWithoutNewtonUpdates) {
  ModelFixture f;
  Definition(f, {{0, 0, 0}, {2, 0, 0}});
  Plane(f);
  f.Add(50, 51,
        {Numbers("origin", {0., 0., 0.}, FieldType::kVector),
         Numbers("normal", {0., -1., 0.}, FieldType::kVector),
         Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample =
      EvaluateIntersectionCurve(model.value(), NodeId{1}, 1, {1e-14, 1e-14});
  ASSERT_TRUE(sample.ok());
  EXPECT_EQ(sample.value().position.x, 1);
  EXPECT_EQ(sample.value().position.y, 0);
  EXPECT_EQ(sample.value().first_derivative.x, 1. / 3);
}
TEST(ParasolidIntersectionEvaluate, CylinderConeAndTorusSectionsAreSolved) {
  for (uint16_t type : {51, 52, 54}) {
    const bool torus = type == 54;
    const double radius = torus ? 4 : type == 52 ? 2.75 : 1;
    const double height = type == 52 ? 1 : 0;
    ModelFixture f;
    Definition(f, CirclePoints({.2, .9}, radius, height));
    Plane(f, height);
    std::vector<ModelField> fields{
        Numbers(torus ? "center" : "origin", {0., 0., 0.}, FieldType::kVector),
        Numbers("axis", {0., 0., 1.}, FieldType::kVector),
        Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
        ByteValue("sense", '+', FieldType::kCharacter)};
    if (torus) {
      fields.push_back(Numbers("major_radius", {3.}));
      fields.push_back(Numbers("minor_radius", {1.}));
    } else {
      fields.push_back(Numbers("radius", {type == 52 ? 2. : 1.}));
      if (type == 52) {
        fields.push_back(Numbers("sin_half_angle", {.6}));
        fields.push_back(Numbers("cos_half_angle", {.8}));
      }
    }
    f.Add(type, 51, std::move(fields));
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto chart =
        EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
    ASSERT_TRUE(chart.ok());
    const auto& times = chart.value().parameterization.parameters;
    const auto sample = EvaluateIntersectionCurve(
        model.value(), NodeId{1}, std::midpoint(times[0], times[1]),
        {1e-13, 1e-13});
    ASSERT_TRUE(sample.ok());
    EXPECT_NEAR(sample.value().position.x, radius * std::cos(.55), 1e-12);
    EXPECT_NEAR(sample.value().position.y, radius * std::sin(.55), 1e-12);
  }
}
TEST(ParasolidIntersectionEvaluate,
     UnattainableResidualToleranceFailsExplicitly) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-30);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  const auto result = EvaluateIntersectionCurve(
      model.value(), NodeId{1}, std::lerp(times[0], times[1], .37),
      {1e-30, 1e-30});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("Newton"), std::string::npos);
}
TEST(ParasolidIntersectionEvaluate,
     RejectsOutsideChartAndInvalidToleranceWithoutExtension) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double tolerance : {0., -1., std::numeric_limits<double>::infinity()}) {
    const auto sample = EvaluateIntersectionCurve(model.value(), NodeId{1}, -1,
                                                  {tolerance, tolerance});
    ASSERT_FALSE(sample.ok());
    EXPECT_EQ(sample.error().code, ErrorCode::kInvalidOptions);
  }
  for (double parameter :
       {-3., 100., std::numeric_limits<double>::quiet_NaN()}) {
    const auto sample = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                  parameter, {1e-13, 1e-13});
    ASSERT_FALSE(sample.ok());
    EXPECT_EQ(sample.error().code, ErrorCode::kInvalidOptions);
  }
  const auto absent =
      EvaluateIntersectionCurve(model.value(), NodeId{99}, -1, {1e-13, 1e-13});
  ASSERT_FALSE(absent.ok());
}
TEST(ParasolidIntersectionEvaluate,
     StoredPointToleranceDoesNotRelaxInteriorSolve) {
  const auto points = CirclePoints({0, .8}, 1 + 1e-8);
  ModelFixture f;
  Definition(f, points);
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 2e-8);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  const double t = std::midpoint(times.front(), times.back());
  const auto sample =
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {2e-8, 1e-13});
  ASSERT_TRUE(sample.ok());
  const auto& p = sample.value().position;
  EXPECT_NEAR(std::hypot(p.x, p.y, p.z), 1, 1e-13);
  const auto stored = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                times.front(), {2e-8, 1e-13});
  ASSERT_TRUE(stored.ok());
  EXPECT_EQ(stored.value().position.x, points.front().x);
  EXPECT_FALSE(
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {1e-10, 1e-13})
          .ok());
}
void Trim(ModelFixture& f, uint32_t id, uint32_t basis, double first,
          double second) {
  f.Add(133, id,
        {Pointer("basis_curve", basis, 1008),
         Numbers("point_1", {0., 0., 0.}, FieldType::kVector),
         Numbers("point_2", {0., 0., 0.}, FieldType::kVector),
         Numbers("parm_1", {first}), Numbers("parm_2", {second}),
         ByteValue("sense", '+', FieldType::kCharacter)});
}
TEST(ParasolidIntersectionEvaluate,
     CurveDispatcherHandlesNestedTrimmedIntersections) {
  ModelFixture f;
  // Forward references, descending intersection sense, then a positive trim.
  Trim(f, 11, 10, -1.5, -.4);
  Trim(f, 10, 1, 0, -1.8);
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const IntersectionEvaluationOptions options{1e-13, 1e-13};
  for (double t : {-1.5, -1., -.4}) {
    const auto expected =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, t, options);
    const auto direct = EvaluateCurve(model.value(), NodeId{1}, t, options);
    const auto wrapped = EvaluateCurve(model.value(), NodeId{11}, t, options);
    ASSERT_TRUE(expected.ok());
    ASSERT_TRUE(direct.ok());
    ASSERT_TRUE(wrapped.ok());
    EXPECT_EQ(wrapped.value().position.x, expected.value().position.x);
    EXPECT_EQ(wrapped.value().position.y, expected.value().position.y);
    EXPECT_EQ(wrapped.value().first_derivative.x,
              expected.value().first_derivative.x);
    EXPECT_EQ(wrapped.value().first_derivative.y,
              expected.value().first_derivative.y);
    EXPECT_EQ(direct.value().position.x, expected.value().position.x);
    const auto legacy = EvaluateCurve(model.value(), NodeId{11}, t);
    ASSERT_FALSE(legacy.ok());
    EXPECT_EQ(legacy.error().code, ErrorCode::kUnsupportedFormat);
  }
  for (double t : {-1.6, -.3}) {
    const auto outside = EvaluateCurve(model.value(), NodeId{11}, t, options);
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  }
  const auto inner_endpoint =
      EvaluateCurve(model.value(), NodeId{10}, -1.8, options);
  EXPECT_TRUE(inner_endpoint.ok());
}
TEST(ParasolidIntersectionEvaluate, TrimAndChartBoundsBothRemainMandatory) {
  ModelFixture f;
  Trim(f, 10, 1, 1, -3);
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto outside =
      EvaluateCurve(model.value(), NodeId{10}, -2.5, {1e-13, 1e-13});
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  EXPECT_NE(outside.error().message.find("chart"), std::string::npos);
}
TEST(ParasolidIntersectionEvaluate,
     OptionsDoNotBypassTrimSenseCycleOrDepthChecks) {
  for (int bad = 0; bad < 3; ++bad) {
    ModelFixture f;
    Trim(f, 10, bad == 2 ? 10 : 1, bad == 1 ? 0 : -1.8, bad == 1 ? -1.8 : 0);
    Definition(f, CirclePoints({0, .8}));
    if (bad == 1) f.report.nodes[1].fields.back().values[0] = uint64_t{'?'};
    Plane(f);
    Sphere(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        EvaluateCurve(model.value(), NodeId{10}, -1, {1e-13, 1e-13});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, bad == 1 ? ErrorCode::kUnsupportedFormat
                                            : ErrorCode::kInvalidRecord);
  }
  for (uint32_t depth : {64u, 65u}) {
    ModelFixture f;
    for (uint32_t i = 0; i < depth; ++i) {
      const bool last = i + 1 == depth;
      Trim(f, 100 + i, last ? 1 : 101 + i, last ? -.1 : -1.8,
           last ? -1.9 : -.2);
    }
    Definition(f, CirclePoints({0, .8}));
    Plane(f);
    Sphere(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        EvaluateCurve(model.value(), NodeId{100}, -1, {1e-13, 1e-13});
    if (depth == 64)
      ASSERT_TRUE(result.ok());
    else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
    }
  }
}
TEST(ParasolidIntersectionEvaluate,
     OptionsPreserveMissingAndUnsupportedBasisErrors) {
  for (uint32_t basis : {50u, 99u, 70u}) {
    ModelFixture f;
    Trim(f, 10, basis, -1.5, -.4);
    Definition(f, CirclePoints({0, .8}));
    Plane(f);
    Sphere(f);
    f.Curve(70, 999);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        EvaluateCurve(model.value(), NodeId{10}, -1, {1e-13, 1e-13});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, basis == 70 ? ErrorCode::kUnsupportedFormat
                                               : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidIntersectionEvaluate,
     OptionsValidateForOrdinaryCurvesAndRetainNurbsBehavior) {
  ModelFixture f;
  f.Spline();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto old = EvaluateCurve(model.value(), NodeId{1}, .5);
  const auto explicit_options =
      EvaluateCurve(model.value(), NodeId{1}, .5, {0, 1e-13});
  ASSERT_TRUE(old.ok());
  ASSERT_TRUE(explicit_options.ok());
  EXPECT_EQ(explicit_options.value().position.z, old.value().position.z);
  EXPECT_EQ(explicit_options.value().first_derivative.z,
            old.value().first_derivative.z);
  for (const IntersectionEvaluationOptions options :
       {IntersectionEvaluationOptions{-1, 1e-13},
        IntersectionEvaluationOptions{0, 0},
        IntersectionEvaluationOptions{0,
                                      std::numeric_limits<double>::infinity()},
        IntersectionEvaluationOptions{std::numeric_limits<double>::quiet_NaN(),
                                      1e-13}}) {
    const auto result = EvaluateCurve(model.value(), NodeId{1}, .5, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_FALSE(EvaluateCurve(model.value(), NodeId{1}, 2, {0, 1e-13}).ok());
}
}  // namespace
}  // namespace openparasolid
