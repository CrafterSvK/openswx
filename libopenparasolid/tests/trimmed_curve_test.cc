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
using namespace test;
void Trim(ModelFixture& f, uint32_t id = 10, uint32_t basis = 1,
          double first = 1, double second = 3, char sense = '+') {
  f.Add(
      133, id,
      {Pointer("basis_curve", basis, 1008),
       Numbers("point_1", {first, 0, 0}, FieldType::kVector),
       Numbers("point_2", {second, 0, 0}, FieldType::kVector),
       Numbers("parm_1", {first}), Numbers("parm_2", {second}),
       ByteValue("sense", static_cast<uint8_t>(sense), FieldType::kCharacter)});
}
void Line(ModelFixture& f, char sense = '+') {
  f.Add(
      30, 1,
      {Numbers("origin", {0, 0, 0}, FieldType::kVector),
       Numbers("direction", {1, 0, 0}, FieldType::kVector),
       ByteValue("sense", static_cast<uint8_t>(sense), FieldType::kCharacter)});
}
void UVLine(ModelFixture& f,
            std::vector<ScalarValue> values = {0., 0., 1., 2.}) {
  f.Curve();
  f.Add(136, 2,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim", 2, FieldType::kShort), Integer("n_knots", 2),
         ByteValue("knot_type", 2), Logical("periodic", false),
         Logical("closed", false), Logical("rational", false),
         ByteValue("curve_form", 1), Pointer("bspline_vertices", 3, 45),
         Pointer("knots", 4, 128), Pointer("knot_mult", 5, 127)});
  f.Add(45, 3, {Array("vertices", FieldType::kDouble, std::move(values))});
  f.Add(128, 4, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 5,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
}
void ExpectVector(Vector3 actual, Vector3 expected) {
  EXPECT_NEAR(actual.x, expected.x, 1e-12);
  EXPECT_NEAR(actual.y, expected.y, 1e-12);
  EXPECT_NEAR(actual.z, expected.z, 1e-12);
}
TEST(ParasolidTrimmedCurve,
     BuildsTypedDefinitionAndEvaluatesInclusiveInterval) {
  ModelFixture f;
  Trim(f);
  Line(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto& model = built.value();
  EXPECT_TRUE(model.references_verified());
  EXPECT_TRUE(model.typed_geometry_complete());
  const auto* node = model.FindNode(NodeId{10});
  ASSERT_NE(node, nullptr);
  ASSERT_TRUE(node->geometry);
  const auto& geometry = model.geometry()[*node->geometry];
  EXPECT_STREQ(GeometryName(geometry.payload), "trimmed_curve");
  const auto& trim = std::get<TrimmedCurveGeometry>(geometry.payload);
  EXPECT_EQ(trim.basis_curve.value, 1);
  EXPECT_EQ(trim.parameter_1, 1);
  EXPECT_EQ(trim.parameter_2, 3);
  EXPECT_EQ(trim.point_2.x, 3);
  for (double t : {1., 2., 3.}) {
    const auto result = EvaluateCurve(model, NodeId{10}, t);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {t, 0, 0});
    ExpectVector(result.value().first_derivative, {1, 0, 0});
  }
  for (double t : {std::nextafter(1., 0.), std::nextafter(3., 4.)}) {
    const auto result = EvaluateCurve(model, NodeId{10}, t);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_FALSE(EvaluateCurve(geometry.payload, 2).ok());
  auto copy = model;
  auto moved = std::move(copy);
  EXPECT_TRUE(EvaluateCurve(moved, NodeId{10}, 2).ok());
}
TEST(ParasolidTrimmedCurve,
     NegativeBasisSenseUsesDescendingBoundsWithoutReversingDerivative) {
  ModelFixture f;
  Trim(f, 10, 1, 3, 1);
  Line(f, '-');
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, 2);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {2, 0, 0});
  ExpectVector(result.value().first_derivative, {1, 0, 0});
}
TEST(ParasolidTrimmedCurve, NestedTrimsIntersectTheirParameterIntervals) {
  ModelFixture f;
  Trim(f, 10, 11, 0, 4);
  Trim(f, 11, 1, 1, 3);
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(EvaluateCurve(model.value(), NodeId{10}, 2).ok());
  const auto outside = EvaluateCurve(model.value(), NodeId{10}, .5);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidTrimmedCurve, NurbsBasisKeepsItsOwnDomainAndDerivative) {
  ModelFixture f;
  Trim(f, 10, 1, .2, .8);
  f.Spline();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1, 2, 3});
  ExpectVector(result.value().first_derivative, {2, 2, 2});
  EXPECT_FALSE(EvaluateCurve(model.value(), NodeId{10}, .1).ok());
}
TEST(ParasolidTrimmedCurve, AnalyticCircleCanCrossItsAngularSeam) {
  ModelFixture f;
  Trim(f, 10, 1, 5.5, 7.0);
  f.Add(
      31, 1,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {2}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      EvaluateCurve(model.value(), NodeId{10}, 2 * std::numbers::pi);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {2, 0, 0});
  ExpectVector(result.value().first_derivative, {0, 2, 0});
}
TEST(ParasolidTrimmedCurve, TrimsCanSelectASurfaceCurve) {
  ModelFixture f;
  Trim(f, 10, 11, .2, .8);
  f.Add(137, 11,
        {Pointer("surface", 20, 1006), Pointer("b_curve", 1, 134),
         ByteValue("sense", '+', FieldType::kCharacter)});
  UVLine(f);
  f.Add(50, 20,
        {Numbers("origin", {1, 2, 3}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1.5, 3, 3});
  ExpectVector(result.value().first_derivative, {1, 2, 0});
}
TEST(ParasolidTrimmedCurve,
     RejectsMalformedDefinitionsWithoutPublishingPayload) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    if (bad == 0) Trim(f, 10, 0);
    if (bad == 1) Trim(f, 10, 1, 2, 2);
    if (bad == 2) Trim(f, 10, 1, 1, std::numeric_limits<double>::quiet_NaN());
    if (bad == 3) Trim(f, 10, 1, 1, 3, '-');
    Line(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_EQ(model.value().statistics().invalid_geometry, 1);
    EXPECT_FALSE(model.value().FindNode(NodeId{10})->geometry);
  }
}
TEST(ParasolidTrimmedCurve, RejectsIncompatibleOrderingAndUnknownBasisSense) {
  for (char sense : {'+', '-', '?'}) {
    ModelFixture f;
    Trim(f, 10, 1, sense == '-' ? 1 : 3, sense == '-' ? 3 : 1);
    Line(f, sense);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = EvaluateCurve(model.value(), NodeId{10}, 2);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, sense == '?' ? ErrorCode::kUnsupportedFormat
                                                : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidTrimmedCurve, MissingWrongAndUnsupportedBasesHaveExplicitErrors) {
  for (int kind = 0; kind < 3; ++kind) {
    ModelFixture f;
    Trim(f);
    if (kind == 1) f.Point(1);
    if (kind == 2) f.Add(38, 1, {});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = EvaluateCurve(model.value(), NodeId{10}, 2);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, kind == 2 ? ErrorCode::kUnsupportedFormat
                                             : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidTrimmedCurve, DetectsCyclesWithoutRecursion) {
  for (bool direct : {false, true}) {
    ModelFixture f;
    Trim(f, 10, direct ? 10 : 11);
    if (!direct) Trim(f, 11, 10);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = EvaluateCurve(model.value(), NodeId{10}, 2);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidTrimmedCurve, DepthLimitAccepts64WrappersAndRejects65) {
  for (uint32_t count : {64u, 65u}) {
    ModelFixture f;
    Line(f);
    for (uint32_t i = 0; i < count; ++i)
      Trim(f, 10 + i, i + 1 == count ? 1 : 11 + i);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = EvaluateCurve(model.value(), NodeId{10}, 2);
    if (count == 64)
      EXPECT_TRUE(result.ok());
    else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
    }
  }
}
TEST(ParasolidTrimmedCurve, StoredConeEndpointsUseTheWriterAxialConvention) {
  ModelFixture f;
  f.Add(133, 10,
        {Pointer("basis_curve", 11, 1008),
         Numbers("point_1", {.25, .25, 0}, FieldType::kVector),
         Numbers("point_2", {.5, .5, 0}, FieldType::kVector),
         Numbers("parm_1", {0}), Numbers("parm_2", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(137, 11,
        {Pointer("surface", 20, 1006), Pointer("b_curve", 1, 134),
         ByteValue("sense", '+', FieldType::kCharacter)});
  UVLine(f, {0., -.25, 0., 0.});
  f.Add(52, 20,
        {Numbers("origin", {0, .5, 0}, FieldType::kVector),
         Numbers("axis", {0, 1, 0}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         Numbers("radius", {.5}), Numbers("sin_half_angle", {std::sqrt(.5)}),
         Numbers("cos_half_angle", {std::sqrt(.5)}),
         ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double t : {0., 1.}) {
    const auto result = EvaluateCurve(model.value(), NodeId{10}, t);
    ASSERT_TRUE(result.ok());
    ExpectVector(result.value().position, {.25 + .25 * t, .25 + .25 * t, 0});
    ExpectVector(result.value().first_derivative, {.25, .25, 0});
  }
}
}  // namespace
}  // namespace openparasolid
