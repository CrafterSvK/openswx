// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include "../src/evaluate_internal.h"
#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
void Surface(ModelFixture& f, uint16_t type, uint8_t sense = '+',
             double radius = 2, double major = 3) {
  std::vector<ModelField> fields;
  const bool centered = type == 53 || type == 54;
  fields.push_back(Numbers(centered ? "center" : "origin", {0., 0., 0.},
                           FieldType::kVector));
  fields.push_back(Numbers(type == 50 ? "normal" : "axis", {0., 0., 1.},
                           FieldType::kVector));
  fields.push_back(Numbers("x_axis", {1., 0., 0.}, FieldType::kVector));
  if (type == 54) {
    fields.push_back(Numbers("major_radius", {major}));
    fields.push_back(Numbers("minor_radius", {radius}));
  } else if (type != 50) {
    fields.push_back(Numbers("radius", {radius}));
  }
  if (type == 52) {
    fields.push_back(Numbers("sin_half_angle", {.6}));
    fields.push_back(Numbers("cos_half_angle", {.8}));
  }
  fields.push_back(ByteValue("sense", sense, FieldType::kCharacter));
  f.Add(type, 1, std::move(fields));
}
void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
  EXPECT_NEAR(std::hypot(actual.x, actual.y, actual.z), 1, tolerance);
}
TEST(ParasolidSurfaceNormal, AnalyticSurfacesApplyTheirOwnSense) {
  for (uint16_t type : {50, 51, 52, 53, 54}) {
    for (uint8_t sense : {'+', '-'}) {
      ModelFixture f;
      Surface(f, type, sense);
      const auto built = ModelBuilder::Build(f.report);
      ASSERT_TRUE(built.ok());
      const auto result = EvaluateSurfaceNormal(built.value(), NodeId{1}, 0, 0);
      ASSERT_TRUE(result.ok()) << result.error().message;
      const double sign = sense == '+' ? 1 : -1;
      const Vector3 expected = type == 50   ? Vector3{0, 0, sign}
                               : type == 52 ? Vector3{.8 * sign, 0, -.6 * sign}
                                            : Vector3{sign, 0, 0};
      ExpectVector(result.value(), expected);
      // Normal orientation does not change natural surface partials.
      const auto sample = EvaluateSurface(built.value(), NodeId{1}, 0, 0);
      ASSERT_TRUE(sample.ok());
      EXPECT_GT(type == 50 ? sample.value().u_derivative.x
                           : sample.value().u_derivative.y,
                0);
    }
  }
}
TEST(ParasolidSurfaceNormal, RotatedPlaneUsesSourceFrame) {
  ModelFixture f;
  Surface(f, 50);
  f.report.nodes[0].fields[1].values = {0., .6, .8};
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{1}, 2, -3);
  ASSERT_TRUE(normal.ok());
  ExpectVector(normal.value(), {0, .6, .8});
}
TEST(ParasolidSurfaceNormal, RationalAndNonrationalPatchesUseNaturalPartials) {
  for (bool rational : {false, true}) {
    ModelFixture f;
    f.Spline(rational, true);
    std::vector<ScalarValue> controls;
    for (double u : {0., 1.}) {
      for (double v : {0., 1.}) {
        const double weight = rational ? 2 : 1;
        for (double value : {u, v, u * v})
          controls.emplace_back(value * weight);
        if (rational) controls.emplace_back(weight);
      }
    }
    f.report.nodes[2].fields[0].values = std::move(controls);
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{1}, .2, .7);
    ASSERT_TRUE(normal.ok());
    const double length = std::hypot(.7, .2, 1.);
    ExpectVector(normal.value(), {.7 / length, .2 / length, -1 / length});
    EXPECT_FALSE(EvaluateSurfaceNormal(built.value(), NodeId{1}, -1, 0).ok());
  }
}
void Line(ModelFixture& f, Vector3 origin, Vector3 direction) {
  f.Add(30, 1,
        {Numbers("origin", {origin.x, origin.y, origin.z}, FieldType::kVector),
         Numbers("direction", {direction.x, direction.y, direction.z},
                 FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
TEST(ParasolidSurfaceNormal, SweptSectionSenseDoesNotFlipSurfaceNormal) {
  ModelFixture f;
  Line(f, {}, {1, 0, 0});
  f.Add(67, 20,
        {Pointer("section", 1, 1008),
         Numbers("sweep", {0., 0., 1.}, FieldType::kVector),
         Numbers("scale", {42.}),
         ByteValue("sense", '-', FieldType::kCharacter)});
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{20}, -2, 3);
  ASSERT_TRUE(normal.ok());
  ExpectVector(normal.value(), {0, 1, 0});
}
TEST(ParasolidSurfaceNormal, SpunGeneratorUsesSurfaceOrientation) {
  ModelFixture f;
  Line(f, {2, 0, 0}, {0, 0, 1});
  auto null_vector = [](const char* name) -> ModelField {
    return {{name, FieldType::kVector},
            {std::monostate{}, std::monostate{}, std::monostate{}}};
  };
  auto null_number = [](const char* name) -> ModelField {
    return {{name, FieldType::kDouble}, {std::monostate{}}};
  };
  f.Add(68, 20,
        {Pointer("profile", 1, 1008),
         Numbers("base", {0., 0., 0.}, FieldType::kVector),
         Numbers("axis", {0., 0., 1.}, FieldType::kVector),
         null_vector("start"), null_vector("end"), null_vector("x_axis"),
         null_number("start_param"), null_number("end_param"),
         null_number("scale"), ByteValue("sense", '-', FieldType::kCharacter)});
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto normal =
      EvaluateSurfaceNormal(built.value(), NodeId{20}, 3, std::numbers::pi / 2);
  ASSERT_TRUE(normal.ok());
  ExpectVector(normal.value(), {0, 1, 0});
}
TEST(ParasolidSurfaceNormal, ExtremeRadiiDoNotOverflowOrUnderflowCrossProduct) {
  for (double radius : {1e-300, 1e300}) {
    ModelFixture f;
    Surface(f, 53, '+', radius);
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{1}, 0, 0);
    ASSERT_TRUE(normal.ok());
    ExpectVector(normal.value(), {1, 0, 0});
  }
}
TEST(ParasolidSurfaceNormal,
     IndependentPartialScalesAndSubnormalsAreSupported) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  for (double scale : {tiny, 1e-300, 1e300}) {
    const SurfaceEvaluation sample{{}, {scale, 0, 0}, {0, 0, 1e300}};
    const auto normal = internal::UnitSurfaceNormal(sample, false);
    ASSERT_TRUE(normal.ok());
    ExpectVector(normal.value(), {0, -1, 0});
  }
  // Nonparallel partials need no arbitrary angle threshold.
  const auto small_angle =
      internal::UnitSurfaceNormal({{}, {1, 0, 0}, {1, 1e-200, 0}}, false);
  ASSERT_TRUE(small_angle.ok());
  ExpectVector(small_angle.value(), {0, 0, 1});
}
TEST(ParasolidSurfaceNormal, RejectsZeroParallelAndNonfinitePartials) {
  for (Vector3 bad : {Vector3{}, Vector3{2, 0, 0},
                      Vector3{std::numeric_limits<double>::infinity(), 1, 0}}) {
    const auto normal =
        internal::UnitSurfaceNormal({{}, {1, 0, 0}, bad}, false);
    ASSERT_FALSE(normal.ok());
    EXPECT_EQ(normal.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSurfaceNormal, SingularParametersDoNotProduceInventedNormals) {
  for (uint16_t type : {52, 53, 54}) {
    ModelFixture f;
    Surface(f, type, '+', type == 52 ? 0 : 2, -1);
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    const double v = type == 53   ? std::numbers::pi / 2
                     : type == 54 ? std::acos(.5)
                                  : 0;
    ASSERT_TRUE(EvaluateSurface(built.value(), NodeId{1}, 0, v).ok());
    const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{1}, 0, v);
    ASSERT_FALSE(normal.ok());
    EXPECT_EQ(normal.error().code, ErrorCode::kInvalidRecord);
  }
  ModelFixture f;
  f.Spline(false, true);  // Collinear control net, with dependent partials.
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  EXPECT_FALSE(EvaluateSurfaceNormal(built.value(), NodeId{1}, .5, .5).ok());
}
TEST(ParasolidSurfaceNormal, UnknownSenseAndUnavailableTargetsAreExplicit) {
  ModelFixture f;
  Surface(f, 50, '?');
  f.Point(2);
  f.Add(60, 3, {ByteValue("sense", '+', FieldType::kCharacter)});
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  for (uint32_t id : {1u, 2u, 3u}) {
    const auto normal = EvaluateSurfaceNormal(built.value(), NodeId{id}, 0, 0);
    ASSERT_FALSE(normal.ok());
    EXPECT_EQ(normal.error().code, ErrorCode::kUnsupportedFormat);
  }
  EXPECT_FALSE(EvaluateSurfaceNormal(built.value(), NodeId{99}, 0, 0).ok());
  const auto nonfinite = EvaluateSurfaceNormal(
      built.value(), NodeId{1}, std::numeric_limits<double>::quiet_NaN(), 0);
  ASSERT_FALSE(nonfinite.ok());
  EXPECT_EQ(nonfinite.error().code, ErrorCode::kInvalidOptions);
}
}  // namespace
}  // namespace openparasolid
