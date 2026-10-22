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
void Sweep(ModelFixture& f, Vector3 direction = {0, 0, 1},
           uint32_t section = 60) {
  f.Add(67, 51,
        {Pointer("section", section, 1008),
         Numbers("sweep", {direction.x, direction.y, direction.z},
                 FieldType::kVector),
         {{"scale", FieldType::kDouble}, {std::monostate{}}},
         ByteValue("sense", '+', FieldType::kCharacter)});
}
TEST(ParasolidIntersectionEvaluate, SweptLinesKeepBothLinearParameterUnits) {
  for (bool along_sweep : {false, true}) {
    ModelFixture f;
    Definition(
        f, {{0, 0, 0}, along_sweep ? Vector3{0, 0, 10} : Vector3{10, 0, 0}});
    f.Add(50, 50,
          {Numbers("origin", {0, 0, 0}, FieldType::kVector),
           Numbers("normal", {along_sweep ? -1. : 0., 0, along_sweep ? 0. : 1.},
                   FieldType::kVector),
           Numbers("x_axis", {along_sweep ? 0. : 1., along_sweep ? 1. : 0., 0},
                   FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
    Sweep(f);
    f.Add(30, 60,
          {Numbers("origin", {0, 0, 0}, FieldType::kVector),
           Numbers("direction", {1, 0, 0}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 0);
    ASSERT_TRUE(chart.ok()) << chart.error().message;
    EXPECT_EQ(chart.value().surface_parameters.back()[1].u,
              along_sweep ? 0 : 10);
    EXPECT_EQ(chart.value().surface_parameters.back()[1].v,
              along_sweep ? 10 : 0);
    for (double fraction : {0., .2, .5, .8, 1.}) {
      const double parameter = -2 + 30 * fraction;
      const auto result = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                    parameter, {0, 1e-12});
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_NEAR(result.value().position.x, along_sweep ? 0 : 10 * fraction,
                  1e-12);
      EXPECT_EQ(result.value().position.y, 0);
      EXPECT_NEAR(result.value().position.z, along_sweep ? 10 * fraction : 0,
                  1e-12);
      EXPECT_NEAR(result.value().first_derivative.x, along_sweep ? 0 : 1. / 3,
                  1e-12);
      EXPECT_NEAR(result.value().first_derivative.z, along_sweep ? 1. / 3 : 0,
                  1e-12);
    }
  }
}
TEST(ParasolidIntersectionEvaluate,
     SweptConicsCrossSeamsWithObliqueDisplacement) {
  for (bool ellipse : {false, true}) {
    for (bool reverse : {false, true}) {
      ModelFixture f;
      const double minor = ellipse ? 1 : 2;
      std::vector<Vector3> points;
      for (double angle : {reverse ? 3.3 : 3., reverse ? 3. : 3.3})
        points.push_back(
            {.75 + 2 * std::cos(angle), minor * std::sin(angle), 1});
      Definition(f, points);
      f.Add(50, 50,
            {Numbers("origin", {0, 0, 1}, FieldType::kVector),
             Numbers("normal", {0, 0, 1}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             ByteValue("sense", reverse ? '-' : '+', FieldType::kCharacter)});
      Sweep(f, {.6, 0, .8});
      std::vector<ModelField> section{
          Numbers("center", {0, 0, 0}, FieldType::kVector),
          Numbers("normal", {0, 0, 1}, FieldType::kVector),
          Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
          ByteValue("sense", '+', FieldType::kCharacter)};
      if (ellipse) {
        section.push_back(Numbers("major_radius", {2}));
        section.push_back(Numbers("minor_radius", {minor}));
      } else
        section.push_back(Numbers("radius", {2}));
      f.Add(ellipse ? 32 : 31, 60, std::move(section));
      const auto model = ModelBuilder::Build(f.report);
      ASSERT_TRUE(model.ok());
      const auto chart =
          EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
      ASSERT_TRUE(chart.ok()) << chart.error().message;
      for (const auto& uv : chart.value().surface_parameters)
        EXPECT_NEAR(uv[1].v, 1.25, 1e-12);
      const auto& times = chart.value().parameterization.parameters;
      for (double fraction : {.2, .5, .8}) {
        const double t = std::lerp(times.front(), times.back(), fraction);
        const auto result = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                      t, {1e-12, 1e-12});
        ASSERT_TRUE(result.ok()) << result.error().message;
        const auto& p = result.value().position;
        EXPECT_NEAR(p.z, 1, 1e-12);
        EXPECT_NEAR(std::pow((p.x - .75) / 2, 2) + std::pow(p.y / minor, 2), 1,
                    1e-12);
        EXPECT_LT(p.x, -1.);
        const double dx = points[1].x - points[0].x;
        const double dy = points[1].y - points[0].y;
        const double px = std::lerp(points[0].x, points[1].x, fraction);
        const double py = std::lerp(points[0].y, points[1].y, fraction);
        EXPECT_NEAR(((p.x - px) * dx + (p.y - py) * dy) / std::hypot(dx, dy), 0,
                    1e-12);
        const double h = (times.back() - times.front()) * 1e-5;
        const auto before = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                      t - h, {1e-12, 1e-12});
        const auto after = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                     t + h, {1e-12, 1e-12});
        ASSERT_TRUE(before.ok());
        ASSERT_TRUE(after.ok());
        EXPECT_NEAR(
            result.value().first_derivative.x,
            (after.value().position.x - before.value().position.x) / (2 * h),
            1e-8);
        EXPECT_NEAR(
            result.value().first_derivative.y,
            (after.value().position.y - before.value().position.y) / (2 * h),
            1e-8);
      }
    }
  }
}
TEST(ParasolidIntersectionEvaluate,
     SweptChartsReportUnsupportedAndInvalidSections) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    Definition(f, {{1, 0, 0}, {0, 1, 0}});
    Plane(f);
    Sweep(f, bad == 3 ? Vector3{1, 0, 0} : Vector3{0, 0, 1}, bad == 0 ? 99 : bad == 1 ? 50 : 60);
    if (bad == 2)
      f.Add(134, 60, {ByteValue("sense", '+', FieldType::kCharacter)});
    if (bad == 3)
      f.Add(31, 60,
            {Numbers("center", {0, 0, 0}, FieldType::kVector),
             Numbers("normal", {0, 0, 1}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             Numbers("radius", {1}),
             ByteValue("sense", '+', FieldType::kCharacter)});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto chart =
        EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
    ASSERT_FALSE(chart.ok());
    EXPECT_EQ(chart.error().code, bad >= 2 ? ErrorCode::kUnsupportedFormat
                                           : ErrorCode::kInvalidRecord);
    EXPECT_NE(chart.error().message.find("point 0, surface 1"),
              std::string::npos);
  }
}
void SpunRing(ModelFixture& f, bool meridian, bool bounded = false,
              bool reverse = false, double major_radius = 3) {
  const double first = reverse ? 3.3 : 3., last = reverse ? 3. : 3.3;
  std::vector<Vector3> points;
  for (double angle : {first, last})
    points.push_back(
        meridian ? Vector3{major_radius + std::cos(angle), 0, -std::sin(angle)}
                 : Vector3{(major_radius + 1) * std::cos(angle),
                           (major_radius + 1) * std::sin(angle), 0});
  Definition(f, points);
  f.Add(50, 50,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, meridian ? 1. : 0., meridian ? 0. : 1.},
                 FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", reverse ? '-' : '+', FieldType::kCharacter)});
  const ModelField null_vector{
      {"start", FieldType::kVector},
      {std::monostate{}, std::monostate{}, std::monostate{}}};
  f.Add(68, 51,
        {Pointer("profile", 60, 1008),
         Numbers("base", {0, 0, 0}, FieldType::kVector),
         Numbers("axis", {0, 0, 1}, FieldType::kVector),
         null_vector,
         {{"end", FieldType::kVector}, null_vector.values},
         {{"x_axis", FieldType::kVector}, null_vector.values},
         {{"start_param", FieldType::kDouble},
          {bounded ? ScalarValue{3.} : ScalarValue{std::monostate{}}}},
         {{"end_param", FieldType::kDouble},
          {bounded ? ScalarValue{3.4} : ScalarValue{std::monostate{}}}},
         {{"scale", FieldType::kDouble}, {std::monostate{}}},
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(
      31, 60,
      {Numbers("center", {major_radius, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 1, 0}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {1}),
       ByteValue("sense", '+', FieldType::kCharacter)});
}
TEST(ParasolidIntersectionEvaluate, CircularSpunSupportsCrossAngularSeams) {
  for (bool meridian : {false, true}) {
    for (bool reverse : {false, true}) {
      for (bool bounded : {false, true}) {
        if (bounded && !meridian) continue;
        ModelFixture f;
        SpunRing(f, meridian, bounded, reverse);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto chart =
            EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
        ASSERT_TRUE(chart.ok()) << chart.error().message;
        const auto& times = chart.value().parameterization.parameters;
        for (double fraction : {0., .2, .5, .8, 1.}) {
          const double parameter =
              std::lerp(times.front(), times.back(), fraction);
          const auto result = EvaluateIntersectionCurveWithDiagnostics(
              model.value(), NodeId{1}, parameter, {1e-12, 1e-12});
          ASSERT_TRUE(result.ok()) << result.error().message;
          const double direction = reverse ? -1 : 1;
          const double delta = std::asin((2 * fraction - 1) * std::sin(.15));
          const double angle = 3.15 + direction * delta;
          const auto& point = result.value().curve.position;
          EXPECT_NEAR(point.x,
                      meridian ? 3 + std::cos(angle) : 4 * std::cos(angle),
                      1e-11);
          EXPECT_NEAR(point.y, meridian ? 0 : 4 * std::sin(angle), 1e-11);
          EXPECT_NEAR(point.z, meridian ? -std::sin(angle) : 0, 1e-11);
          const auto& tangent = result.value().curve.first_derivative;
          const double scale = direction / (3 * std::cos(delta));
          EXPECT_NEAR(tangent.x, -scale * std::sin(angle), 1e-11);
          EXPECT_NEAR(tangent.y, meridian ? 0 : scale * std::cos(angle), 1e-11);
          EXPECT_NEAR(tangent.z, meridian ? -scale * std::cos(angle) : 0,
                      1e-11);
          EXPECT_NEAR(result.value().diagnostics.surface_normal_sine, 1, 1e-12);
        }
        if (bounded) {
          for (const auto& uv : chart.value().surface_parameters) {
            EXPECT_GE(uv[1].u, 3);
            EXPECT_LE(uv[1].u, 3.4);
          }
        }
      }
    }
  }
}
TEST(ParasolidIntersectionEvaluate,
     SpunSupportsRetainSurfaceOrderAndTrimBounds) {
  ModelFixture f;
  SpunRing(f, false);
  // Swap the supports and reverse the plane sense, retaining chart direction.
  f.report.nodes[0].fields[0].values = {uint64_t{51}, uint64_t{50}};
  for (auto& node : f.report.nodes) {
    if (node.index != 50) continue;
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (auto& field : node.fields)
      if (fields[field.definition_index].name == "sense")
        field.values = {uint64_t{'-'}};
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
  ASSERT_TRUE(chart.ok()) << chart.error().message;
  const auto& times = chart.value().parameterization.parameters;
  EXPECT_NEAR(chart.value().surface_parameters.front()[0].v, 3, 1e-12);
  EXPECT_NEAR(chart.value().surface_parameters.front()[1].u, 4 * std::cos(3.),
              1e-12);
  f.Add(133, 70,
        {Pointer("basis_curve", 1, 1008),
         Numbers("point_1", {4 * std::cos(3.3), 4 * std::sin(3.3), 0},
                 FieldType::kVector),
         Numbers("point_2", {4 * std::cos(3.), 4 * std::sin(3.), 0},
                 FieldType::kVector),
         Numbers("parm_1", {times.back()}), Numbers("parm_2", {times.front()}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto trimmed = ModelBuilder::Build(f.report);
  ASSERT_TRUE(trimmed.ok());
  const IntersectionEvaluationOptions options{1e-12, 1e-12};
  const double parameter = std::lerp(times.front(), times.back(), .2);
  const auto direct =
      EvaluateIntersectionCurve(model.value(), NodeId{1}, parameter, options);
  const auto sample =
      EvaluateCurve(trimmed.value(), NodeId{70}, parameter, options);
  ASSERT_TRUE(direct.ok());
  ASSERT_TRUE(sample.ok()) << sample.error().message;
  EXPECT_DOUBLE_EQ(sample.value().position.x, direct.value().position.x);
  EXPECT_DOUBLE_EQ(sample.value().first_derivative.y,
                   direct.value().first_derivative.y);
  const auto outside =
      EvaluateCurve(trimmed.value(), NodeId{70}, times.back() + 1, options);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidIntersectionEvaluate, SpunChartsRetainExplicitProfileFailures) {
  for (int bad = 0; bad < 3; ++bad) {
    ModelFixture f;
    SpunRing(f, false, false, false, bad == 0 ? 1 : 3);
    for (auto& node : f.report.nodes) {
      const auto& fields =
          f.report.schemas[node.schema_index].definition.fields;
      for (auto& field : node.fields) {
        const auto& name = fields[field.definition_index].name;
        if (bad == 1 && node.index == 51 && name == "profile")
          field.values = {uint64_t{99}};
        if (bad == 2 && node.index == 60 && name == "normal")
          field.values = {0., 0., 1.};
      }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, bad == 1 ? ErrorCode::kInvalidRecord
                                            : ErrorCode::kUnsupportedFormat);
    EXPECT_NE(result.error().message.find("point 0, surface 1"),
              std::string::npos);
  }
}
TEST(ParasolidIntersectionEvaluate,
     RationalNurbsSupportsKeepOriginalKnotUnits) {
  ModelFixture f;
  f.Spline(true, true);
  // Move spline fixture nodes away from the chart IDs. Its knot interval is
  // wider than 2*pi: treating NURBS parameters as angles would leave the
  // domain.
  for (auto& node : f.report.nodes) {
    node.index += 50;
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < node.fields.size(); ++i) {
      auto& values = node.fields[i].values;
      if (fields[i].type == FieldType::kPointer)
        values[0] = std::get<uint64_t>(values[0]) + 50;
      if (fields[i].name == "u_degree") values[0] = int64_t{2};
      if (fields[i].name == "n_u_vertices") values[0] = int64_t{3};
      if (fields[i].name == "v_knot_mult") values[0] = uint64_t{56};
      if (fields[i].name == "knots") values = {0., 10.};
      if (fields[i].name == "multiplicities") values = {int64_t{3}, int64_t{3}};
      if (fields[i].name == "vertices") {
        const double w = std::sqrt(.5);
        values = {1., 0., 0.,    1., 1., 0., 2., 1., w,  w,  0., w,
                  w,  w,  2 * w, w,  0., 1., 0., 1., 0., 1., 2., 1.};
      }
    }
  }
  ModelFixture rebuilt;
  for (const auto& node : f.report.nodes) {
    const auto& definitions =
        f.report.schemas[node.schema_index].definition.fields;
    std::vector<ModelField> fields;
    for (std::size_t i = 0; i < node.fields.size(); ++i)
      fields.push_back({definitions[i], node.fields[i].values});
    rebuilt.Add(node.node_type, node.index, std::move(fields));
  }
  f = std::move(rebuilt);
  f.Add(127, 56,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
  Definition(f, {{0, 1, 1}, {1, 0, 1}});
  Plane(f, 1);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok()) << model.error().message;
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-12);
  ASSERT_TRUE(chart.ok()) << chart.error().message;
  EXPECT_NEAR(chart.value().surface_parameters.front()[1].u, 10, 1e-12);
  EXPECT_NEAR(chart.value().surface_parameters.back()[1].u, 0, 1e-12);
  const auto& times = chart.value().parameterization.parameters;
  const double t = std::midpoint(times.front(), times.back());
  const auto result =
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {1e-12, 1e-12});
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto details = EvaluateIntersectionCurveWithDiagnostics(
      model.value(), NodeId{1}, t, {1e-12, 1e-12, 0, 1, .99});
  ASSERT_TRUE(details.ok()) << details.error().message;
  EXPECT_EQ(details.value().diagnostics.surface_normal_sine, 1);
  EXPECT_NEAR(details.value().diagnostics.tangent_chord_projection, 1, 1e-14);
  const double w = std::sqrt(.5);
  EXPECT_NEAR(result.value().position.x, w, 1e-12);
  EXPECT_NEAR(result.value().position.y, w, 1e-12);
  EXPECT_NEAR(result.value().position.z, 1, 1e-12);
  EXPECT_NEAR(result.value().first_derivative.x, w / 3, 1e-12);
  EXPECT_NEAR(result.value().first_derivative.y, -w / 3, 1e-12);
  EXPECT_NEAR(result.value().first_derivative.z, 0, 1e-12);
}
TEST(ParasolidIntersectionEvaluate,
     DiagnosticsMatchCircleGeometryAndLegacySample) {
  const double height = .6, radius = .8;
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}, radius, height));
  Plane(f, height);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  for (double fraction : {0., .2, .5, .8, 1.}) {
    const double t = std::lerp(times.front(), times.back(), fraction);
    const auto result = EvaluateIntersectionCurveWithDiagnostics(
        model.value(), NodeId{1}, t, {1e-13, 1e-13});
    const auto legacy =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {1e-13, 1e-13});
    ASSERT_TRUE(result.ok()) << result.error().message;
    ASSERT_TRUE(legacy.ok());
    const auto& diagnostic = result.value().diagnostics;
    const double angle = .4 + std::asin((2 * fraction - 1) * std::sin(.4));
    EXPECT_NEAR(diagnostic.surface_normal_sine, radius, 1e-12);
    EXPECT_NEAR(diagnostic.tangent_chord_projection, std::cos(angle - .4),
                1e-12);
    EXPECT_EQ(result.value().curve.position.x, legacy.value().position.x);
    EXPECT_EQ(result.value().curve.first_derivative.y,
              legacy.value().first_derivative.y);
  }
}
TEST(ParasolidIntersectionEvaluate,
     NormalSineDetectsNearParallelAndAntiparallelSupports) {
  for (double sine : {1e-8, 1e-150}) {
    for (double sign : {-1., 1.}) {
      ModelFixture f;
      Definition(f, {{1, 0, 0}, {0, 0, 0}});
      Plane(f);
      f.Add(50, 51,
            {Numbers("origin", {0., 0., 0.}, FieldType::kVector),
             Numbers("normal", {0., sine, sign * std::sqrt(1 - sine * sine)},
                     FieldType::kVector),
             Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
             ByteValue("sense", '+', FieldType::kCharacter)});
      const auto model = ModelBuilder::Build(f.report);
      ASSERT_TRUE(model.ok());
      IntersectionEvaluationOptions options{0, 1e-13};
      options.minimum_normal_sine = sine / 2;
      options.minimum_chord_projection = 1;
      for (double t : {-2., -.5, 1.}) {
        const auto result = EvaluateIntersectionCurveWithDiagnostics(
            model.value(), NodeId{1}, t, options);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_NEAR(result.value().diagnostics.surface_normal_sine, sine,
                    sine * 1e-14);
        EXPECT_EQ(result.value().diagnostics.tangent_chord_projection, 1);
        EXPECT_NEAR(result.value().curve.first_derivative.x, -1. / 3, 1e-15);
        auto strict = options;
        strict.minimum_normal_sine = sine * 2;
        const auto rejected =
            EvaluateIntersectionCurve(model.value(), NodeId{1}, t, strict);
        ASSERT_FALSE(rejected.ok());
        EXPECT_EQ(rejected.error().code, ErrorCode::kInvalidRecord);
        EXPECT_NE(rejected.error().message.find("normal sine"),
                  std::string::npos);
      }
    }
  }
}
TEST(ParasolidIntersectionEvaluate, ReportedMeasuresDefineInclusiveMinimums) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}, .8, .6));
  Plane(f, .6);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  IntersectionEvaluationOptions options{1e-13, 1e-13};
  const double t = -1.9;
  const auto result = EvaluateIntersectionCurveWithDiagnostics(
      model.value(), NodeId{1}, t, options);
  ASSERT_TRUE(result.ok());
  options.minimum_normal_sine = result.value().diagnostics.surface_normal_sine;
  options.minimum_chord_projection =
      result.value().diagnostics.tangent_chord_projection;
  EXPECT_TRUE(
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, options).ok());
  auto strict = options;
  strict.minimum_normal_sine = std::nextafter(options.minimum_normal_sine, 1.);
  EXPECT_FALSE(
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, strict).ok());
  strict = options;
  strict.minimum_chord_projection =
      std::nextafter(options.minimum_chord_projection, 1.);
  EXPECT_FALSE(
      EvaluateIntersectionCurve(model.value(), NodeId{1}, t, strict).ok());
}
TEST(ParasolidIntersectionEvaluate,
     ChordMinimumAppliesAtQueryAndSnappedEndpoints) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  IntersectionEvaluationOptions options{1e-13, 1e-13, 1e-8};
  options.minimum_normal_sine = 1;
  options.minimum_chord_projection = .99;
  EXPECT_TRUE(EvaluateIntersectionCurve(
                  model.value(), NodeId{1},
                  std::midpoint(times.front(), times.back()), options)
                  .ok());
  for (double t : {times.front(), times.back(), times.front() - 1e-9,
                   times.back() + 1e-9}) {
    const auto rejected = EvaluateIntersectionCurveWithDiagnostics(
        model.value(), NodeId{1}, t, options);
    ASSERT_FALSE(rejected.ok());
    EXPECT_EQ(rejected.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(rejected.error().message.find("chord projection"),
              std::string::npos);
    auto loose = options;
    loose.minimum_chord_projection = .9;
    const auto accepted = EvaluateIntersectionCurveWithDiagnostics(
        model.value(), NodeId{1}, t, loose);
    ASSERT_TRUE(accepted.ok());
    EXPECT_NEAR(accepted.value().diagnostics.tangent_chord_projection,
                std::cos(.4), 1e-14);
  }
}
TEST(ParasolidIntersectionEvaluate,
     RegularityOptionsValidateBeforeModelLookup) {
  ModelFixture f;
  f.Spline();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double bad : {-1., 1.01, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    for (bool normal : {false, true}) {
      IntersectionEvaluationOptions options{0, 1e-13};
      (normal ? options.minimum_normal_sine
              : options.minimum_chord_projection) = bad;
      const auto ordinary =
          EvaluateCurve(model.value(), NodeId{1}, .5, options);
      const auto absent =
          EvaluateIntersectionCurve(model.value(), NodeId{99}, 0, options);
      const auto details = EvaluateIntersectionCurveWithDiagnostics(
          model.value(), NodeId{99}, 0, options);
      ASSERT_FALSE(ordinary.ok());
      ASSERT_FALSE(absent.ok());
      ASSERT_FALSE(details.ok());
      EXPECT_EQ(ordinary.error().code, ErrorCode::kInvalidOptions);
      EXPECT_EQ(absent.error().code, ErrorCode::kInvalidOptions);
      EXPECT_EQ(details.error().code, ErrorCode::kInvalidOptions);
    }
  }
  // Valid quality minimums have no effect on ordinary curves.
  IntersectionEvaluationOptions options{0, 1e-13, 0, 1, 1};
  EXPECT_TRUE(EvaluateCurve(model.value(), NodeId{1}, .5, options).ok());
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
TEST(ParasolidIntersectionEvaluate, TrimmedDispatcherRetainsRegularityChecks) {
  ModelFixture f;
  Trim(f, 11, 10, -1.99, -1.4);
  Trim(f, 10, 1, 0, -2);
  Definition(f, CirclePoints({0, .8}, .8, .6));
  Plane(f, .6);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  IntersectionEvaluationOptions options{1e-13, 1e-13};
  options.minimum_normal_sine = .9;
  auto rejected = EvaluateCurve(model.value(), NodeId{11}, -1.9, options);
  ASSERT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(rejected.error().message.find("normal sine"), std::string::npos);
  options.minimum_normal_sine = .7;
  EXPECT_TRUE(EvaluateCurve(model.value(), NodeId{11}, -1.9, options).ok());
  options.minimum_chord_projection = .99;
  rejected = EvaluateCurve(model.value(), NodeId{11}, -1.9, options);
  ASSERT_FALSE(rejected.ok());
  EXPECT_NE(rejected.error().message.find("chord projection"),
            std::string::npos);
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
TEST(ParasolidIntersectionEvaluate,
     EndpointToleranceIsExplicitAndReturnsNaturalEndpointData) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .8}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  for (bool upper : {false, true}) {
    const double endpoint = upper ? times.back() : times.front();
    const double outside = std::nextafter(
        endpoint, upper ? std::numeric_limits<double>::infinity()
                        : -std::numeric_limits<double>::infinity());
    const auto strict = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                  outside, {1e-13, 1e-13});
    ASSERT_FALSE(strict.ok());
    EXPECT_EQ(strict.error().code, ErrorCode::kInvalidOptions);
    const auto exact = EvaluateIntersectionCurve(model.value(), NodeId{1},
                                                 endpoint, {1e-13, 1e-13});
    const auto snapped = EvaluateIntersectionCurve(
        model.value(), NodeId{1}, outside, {1e-13, 1e-13, 1e-12});
    ASSERT_TRUE(exact.ok());
    ASSERT_TRUE(snapped.ok());
    EXPECT_EQ(snapped.value().position.x, exact.value().position.x);
    EXPECT_EQ(snapped.value().position.y, exact.value().position.y);
    EXPECT_EQ(snapped.value().first_derivative.x,
              exact.value().first_derivative.x);
    EXPECT_EQ(snapped.value().first_derivative.y,
              exact.value().first_derivative.y);
    const auto too_far = EvaluateIntersectionCurve(
        model.value(), NodeId{1}, endpoint + (upper ? 1e-4 : -1e-4),
        {1e-13, 1e-13, 1e-8});
    ASSERT_FALSE(too_far.ok());
    EXPECT_EQ(too_far.error().code, ErrorCode::kInvalidOptions);
  }
  // Inclusive tolerance comparison, using exactly representable values.
  const auto boundary = EvaluateIntersectionCurve(
      model.value(), NodeId{1}, times.front() - .125, {1e-13, 1e-13, .125});
  EXPECT_TRUE(boundary.ok());
}
TEST(ParasolidIntersectionEvaluate,
     EndpointToleranceDoesNotMoveInteriorQueriesOrKnots) {
  ModelFixture f;
  Definition(f, CirclePoints({0, .4, 1.1}));
  Plane(f);
  Sphere(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const auto& times = chart.value().parameterization.parameters;
  for (double t :
       {times[1], times[1] + 1e-5, std::midpoint(times[1], times[2])}) {
    const auto strict =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, t, {1e-13, 1e-13});
    const auto tolerant = EvaluateIntersectionCurve(model.value(), NodeId{1}, t,
                                                    {1e-13, 1e-13, 1000});
    ASSERT_TRUE(strict.ok());
    ASSERT_TRUE(tolerant.ok());
    EXPECT_EQ(strict.value().position.x, tolerant.value().position.x);
    EXPECT_EQ(strict.value().position.y, tolerant.value().position.y);
    EXPECT_EQ(strict.value().first_derivative.y,
              tolerant.value().first_derivative.y);
  }
}
TEST(ParasolidIntersectionEvaluate,
     EndpointToleranceLeavesEveryTrimIntervalStrict) {
  ModelFixture f;
  const auto points = CirclePoints({0, .8});
  // The upper stored trim parameter is one representable step beyond the
  // reconstructed chart end. It remains an inclusive trim endpoint.
  Definition(f, points);
  Plane(f);
  Sphere(f);
  auto first = ModelBuilder::Build(f.report);
  ASSERT_TRUE(first.ok());
  const auto chart = EvaluateIntersectionChart(first.value(), NodeId{1}, 1e-13);
  ASSERT_TRUE(chart.ok());
  const double chart_end = chart.value().parameterization.parameters.back();
  const double trim_end =
      std::nextafter(chart_end, std::numeric_limits<double>::infinity());
  Trim(f, 10, 1, trim_end, -1.8);
  Trim(f, 11, 1, .1, -1.8);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const IntersectionEvaluationOptions options{1e-13, 1e-13, 1e-12};
  const auto accepted =
      EvaluateCurve(model.value(), NodeId{10}, trim_end, options);
  ASSERT_TRUE(accepted.ok());
  EXPECT_EQ(accepted.value().position.x, points.back().x);
  EXPECT_FALSE(
      EvaluateCurve(model.value(), NodeId{10}, trim_end, {1e-13, 1e-13}).ok());
  for (auto [id, t] :
       {std::pair{10u, std::nextafter(trim_end,
                                      std::numeric_limits<double>::infinity())},
        std::pair{11u, std::nextafter(
                           .1, std::numeric_limits<double>::infinity())}}) {
    const auto direct =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, t, options);
    ASSERT_TRUE(direct.ok());
    const auto wrapped = EvaluateCurve(model.value(), NodeId{id}, t, options);
    ASSERT_FALSE(wrapped.ok());
    EXPECT_EQ(wrapped.error().code, ErrorCode::kInvalidOptions);
    EXPECT_NE(wrapped.error().message.find("trim interval"), std::string::npos);
  }
}
TEST(ParasolidIntersectionEvaluate,
     EndpointToleranceMustBeFiniteAndNonnegativeForAllCurves) {
  ModelFixture f;
  f.Spline();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double tolerance : {-1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    const IntersectionEvaluationOptions options{0, 1e-13, tolerance};
    const auto general = EvaluateCurve(model.value(), NodeId{1}, .5, options);
    const auto direct =
        EvaluateIntersectionCurve(model.value(), NodeId{1}, .5, options);
    ASSERT_FALSE(general.ok());
    ASSERT_FALSE(direct.ok());
    EXPECT_EQ(general.error().code, ErrorCode::kInvalidOptions);
    EXPECT_EQ(direct.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_TRUE(EvaluateCurve(model.value(), NodeId{1}, .5, {0, 1e-13, 0}).ok());
}
}  // namespace
}  // namespace openparasolid
