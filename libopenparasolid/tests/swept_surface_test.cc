// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
void Swept(ModelFixture& f, uint32_t section = 1, Vector3 direction = {0, 0, 1},
           bool scale = false) {
  ModelField metadata{
      {"scale", FieldType::kDouble},
      {scale ? ScalarValue{42.} : ScalarValue{std::monostate{}}}};
  f.Add(67, 20,
        {Pointer("section", section, 1008),
         Numbers("sweep", {direction.x, direction.y, direction.z},
                 FieldType::kVector),
         std::move(metadata), ByteValue("sense", '-', FieldType::kCharacter)});
}
void Line(ModelFixture& f, Vector3 origin = {}) {
  f.Add(30, 1,
        {Numbers("origin", {origin.x, origin.y, origin.z}, FieldType::kVector),
         Numbers("direction", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}
TEST(ParasolidSweptSurface,
     ResolvesForwardSectionWithOriginalParametersAndNaturalPartials) {
  ModelFixture f;
  Swept(f);
  Line(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto& model = built.value();
  EXPECT_TRUE(model.references_verified());
  EXPECT_TRUE(model.typed_geometry_complete());
  const auto* node = model.FindNode(NodeId{20});
  ASSERT_NE(node, nullptr);
  ASSERT_TRUE(node->geometry);
  const auto& geometry = model.geometry()[*node->geometry];
  EXPECT_STREQ(GeometryName(geometry.payload), "swept_surface");
  const auto& swept = std::get<SweptSurfaceGeometry>(geometry.payload);
  EXPECT_EQ(swept.section.value, 1);
  EXPECT_FALSE(swept.scale);
  const auto sample = EvaluateSurface(model, NodeId{20}, -3, -2);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {-3, 0, -2});
  ExpectVector(sample.value().u_derivative, {1, 0, 0});
  ExpectVector(sample.value().v_derivative, {0, 0, 1});
  EXPECT_FALSE(EvaluateSurface(geometry.payload, 0, 0).ok());
  auto copy = model;
  auto moved = std::move(copy);
  EXPECT_TRUE(EvaluateSurface(moved, NodeId{20}, 1, 2).ok());
}
TEST(ParasolidSweptSurface, ObliqueSweepPreservesSourceScaleAsUnusedMetadata) {
  ModelFixture f;
  Swept(f, 1, {0, .6, .8}, true);
  Line(f, {1, 2, 3});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 2, -5);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {3, -1, -1});
  ExpectVector(sample.value().v_derivative, {0, .6, .8});
  const auto& geometry =
      model.value().geometry()[*model.value().FindNode(NodeId{20})->geometry];
  EXPECT_EQ(std::get<SweptSurfaceGeometry>(geometry.payload).scale, 42);
}
TEST(ParasolidSweptSurface, SupportsRationalAndNonrationalNurbsSections) {
  for (bool rational : {false, true}) {
    ModelFixture f;
    Swept(f);
    f.Spline(rational);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto sample = EvaluateSurface(model.value(), NodeId{20}, .5, 2);
    ASSERT_TRUE(sample.ok());
    ExpectVector(sample.value().position,
                 rational ? Vector3{.5, 1, 3.5} : Vector3{1, 2, 5});
    ExpectVector(sample.value().u_derivative,
                 rational ? Vector3{.5, 0, -.5} : Vector3{2, 2, 2});
    ExpectVector(sample.value().v_derivative, {0, 0, 1});
    const auto outside = EvaluateSurface(model.value(), NodeId{20}, 1.5, 2);
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidSweptSurface,
     CircleSectionAndBothPartialsAgreeWithCentralDifferences) {
  ModelFixture f;
  Swept(f);
  f.Add(
      31, 1,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {2}),
       ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const double u = .4, v = -1, h = 1e-5;
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, u, v);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {2 * std::cos(u), 2 * std::sin(u), v});
  for (bool vary_u : {false, true}) {
    const auto a = EvaluateSurface(model.value(), NodeId{20},
                                   u - (vary_u ? h : 0), v - (vary_u ? 0 : h));
    const auto b = EvaluateSurface(model.value(), NodeId{20},
                                   u + (vary_u ? h : 0), v + (vary_u ? 0 : h));
    ASSERT_TRUE(a.ok());
    ASSERT_TRUE(b.ok());
    const auto& p = a.value().position;
    const auto& q = b.value().position;
    ExpectVector(
        vary_u ? sample.value().u_derivative : sample.value().v_derivative,
        {(q.x - p.x) / (2 * h), (q.y - p.y) / (2 * h), (q.z - p.z) / (2 * h)},
        1e-8);
  }
}
TEST(ParasolidSweptSurface,
     RejectsNullSectionsZeroDirectionsAndNonfiniteFields) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    Swept(f, bad == 0 ? 0 : 1,
          bad == 1   ? Vector3{}
          : bad == 2 ? Vector3{0, 0, std::numeric_limits<double>::quiet_NaN()}
                     : Vector3{0, 0, 1});
    if (bad == 3)
      f.report.nodes[0].fields[2].values = {
          std::numeric_limits<double>::infinity()};
    Line(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_EQ(model.value().statistics().invalid_geometry, 1);
    EXPECT_FALSE(model.value().FindNode(NodeId{20})->geometry);
  }
}
TEST(ParasolidSweptSurface, RejectsMissingWrongCyclicAndUntypedSections) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    Swept(f, bad == 0 ? 99 : bad == 1 ? 20 : 1);
    if (bad == 2) f.Point(1);
    if (bad == 3) f.Curve(1, 99);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto sample = EvaluateSurface(model.value(), NodeId{20}, 0, 0);
    ASSERT_FALSE(sample.ok());
    EXPECT_EQ(sample.error().code, bad == 3 ? ErrorCode::kUnsupportedFormat
                                            : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSweptSurface,
     RequiresUnitSweepWithoutNormalizingWithinTolerance) {
  const double direction = 1 + 5e-11;
  ModelFixture f;
  Swept(f, 1, {0, 0, direction});
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 0, 2);
  ASSERT_TRUE(sample.ok());
  EXPECT_EQ(sample.value().position.z, 2 * direction);
  EXPECT_EQ(sample.value().v_derivative.z, direction);
  ModelFixture invalid;
  Swept(invalid, 1, {0, 0, 2});
  Line(invalid);
  const auto bad_model = ModelBuilder::Build(invalid.report);
  ASSERT_TRUE(bad_model.ok());
  EXPECT_FALSE(EvaluateSurface(bad_model.value(), NodeId{20}, 0, 0).ok());
}
TEST(ParasolidSweptSurface,
     RejectsNonfiniteParametersAndChecksRangeBeforeNarrowing) {
  const double largest = std::numeric_limits<double>::max();
  for (double sign : {-1., 1.}) {
    ModelFixture f;
    Swept(f);
    Line(f, {0, 0, sign * largest});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = EvaluateSurface(model.value(), NodeId{20}, 0, largest);
    if (sign < 0) {
      ASSERT_TRUE(result.ok());
      EXPECT_EQ(result.value().position.z, 0);
    } else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    }
    EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{20},
                                 std::numeric_limits<double>::infinity(), 0)
                     .ok());
    EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{20}, 0,
                                 std::numeric_limits<double>::quiet_NaN())
                     .ok());
  }
}
TEST(ParasolidSweptSurface,
     ParallelSectionAndSweepAreEvaluableWithoutNormalCertification) {
  ModelFixture f;
  Swept(f, 1, {1, 0, 0});
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 2, 3);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {5, 0, 0});
  ExpectVector(sample.value().u_derivative, {1, 0, 0});
  ExpectVector(sample.value().v_derivative, {1, 0, 0});
}
TEST(ParasolidSweptSurface, SurfaceAndTrimmedCurvesComposeWithSweptSections) {
  ModelFixture f;
  Swept(f);
  Line(f);
  f.Curve(30, 31);
  f.Add(136, 31,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim", 2, FieldType::kShort), Integer("n_knots", 2),
         ByteValue("knot_type", 2), Logical("periodic", false),
         Logical("closed", false), Logical("rational", false),
         ByteValue("curve_form", 1), Pointer("bspline_vertices", 32, 45),
         Pointer("knots", 33, 128), Pointer("knot_mult", 34, 127)});
  f.Add(45, 32, {Array("vertices", FieldType::kDouble, {0., 0., 1., 2.})});
  f.Add(128, 33, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 34,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
  f.Add(137, 40,
        {Pointer("surface", 20, 1006), Pointer("b_curve", 30, 134),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(133, 50,
        {Pointer("basis_curve", 40, 1008),
         Numbers("point_1", {0, 0, 0}, FieldType::kVector),
         Numbers("point_2", {0, 0, 0}, FieldType::kVector),
         Numbers("parm_1", {.2}), Numbers("parm_2", {.8}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (uint32_t id : {40u, 50u}) {
    const auto sample = EvaluateCurve(model.value(), NodeId{id}, .5);
    ASSERT_TRUE(sample.ok());
    ExpectVector(sample.value().position, {.5, 0, 1});
    ExpectVector(sample.value().first_derivative, {1, 0, 2});
  }
}
}  // namespace
}  // namespace openparasolid
