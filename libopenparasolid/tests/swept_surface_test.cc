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
void Conic(ModelFixture& f, bool ellipse, double radius = 2) {
  std::vector<ModelField> fields{
      Numbers("center", {1, -2, 3}, FieldType::kVector),
      Numbers("normal", {0, 0, 1}, FieldType::kVector),
      Numbers("x_axis", {.6, .8, 0}, FieldType::kVector),
      ByteValue("sense", '-', FieldType::kCharacter)};
  if (ellipse) {
    fields.push_back(Numbers("major_radius", {radius}));
    fields.push_back(Numbers("minor_radius", {radius / 2}));
  } else {
    fields.push_back(Numbers("radius", {radius}));
  }
  f.Add(ellipse ? 32 : 31, 1, std::move(fields));
}
void Spline(ModelFixture& f, std::vector<ScalarValue> controls,
            std::vector<ScalarValue> knots = {2., 5.},
            std::vector<ScalarValue> multiplicities = {int64_t{4},
                                                       int64_t{4}}) {
  const auto count = static_cast<int64_t>(controls.size() / 3);
  const auto knot_count = static_cast<int64_t>(knots.size());
  f.Curve();
  f.Add(136, 2,
        {Integer("degree", 3, FieldType::kShort), Integer("n_vertices", count),
         Integer("vertex_dim", 3, FieldType::kShort),
         Integer("n_knots", knot_count), ByteValue("knot_type", 2),
         Logical("periodic", false), Logical("closed", false),
         Logical("rational", false), ByteValue("curve_form", 1),
         Pointer("bspline_vertices", 3, 45), Pointer("knots", 4, 128),
         Pointer("knot_mult", 5, 127)});
  f.Add(45, 3, {Array("vertices", FieldType::kDouble, std::move(controls))});
  f.Add(128, 4, {Array("knots", FieldType::kDouble, std::move(knots))});
  f.Add(
      127, 5,
      {Array("multiplicities", FieldType::kShort, std::move(multiplicities))});
}
TEST(ParasolidSweptSurface, InvertsClampedCubicInSourceKnotUnits) {
  ModelFixture f;
  Swept(f, 1, {1, 0, 0}, true);
  Spline(f,
         {-.008, 0., 0., -.008, .004, 0., -.008, .008, 0., -.008, .012, .009});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double t : {0., .01, .3, .75, .999, 1.}) {
    for (double v : {-.3, 0., .008, 3.}) {
      const double u = 2 + 3 * t;
      // Independently constructed analytic cubic, including both endpoints.
      const Vector3 point{-.008 + v, .012 * t, .009 * t * t * t};
      const auto inverse =
          InvertSurface(model.value(), NodeId{20}, point, 1e-13, {4, 100});
      ASSERT_TRUE(inverse.ok()) << inverse.error().message;
      EXPECT_NEAR(inverse.value().u, u, 1e-12);
      EXPECT_NEAR(inverse.value().v, v, 1e-13);
      EXPECT_LE(inverse.value().residual, 1e-13);
    }
  }
}
TEST(ParasolidSweptSurface, InvertsNonplanarObliqueMultiSpanSection) {
  ModelFixture f;
  Swept(f, 1, {.6, 0, .8});
  Spline(
      f,
      {0., 0., 0., 1., 1., 0., 0., 2., 1., 2., 3., 2., 0., 4., 3., 3., 5., 4.},
      {2., 2.9, 4.25, 5.}, {int64_t{4}, int64_t{1}, int64_t{1}, int64_t{4}});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (double u : {2., 2.4, 2.9, 3.7, 4.25, 4.8, 5.}) {
    const auto sample = EvaluateSurface(model.value(), NodeId{20}, u, -3);
    ASSERT_TRUE(sample.ok());
    const auto inverse = InvertSurface(model.value(), NodeId{20},
                                       sample.value().position, 1e-12);
    ASSERT_TRUE(inverse.ok()) << inverse.error().message;
    EXPECT_NEAR(inverse.value().u, u, 1e-12);
    EXPECT_NEAR(inverse.value().v, -3, 1e-12);
  }
}
TEST(ParasolidSweptSurface, SplineMembershipDoesNotExpandToleranceOrDomain) {
  ModelFixture f;
  Swept(f, 1, {1, 0, 0});
  Spline(f, {0., 0., 0., 0., 1., 0., 0., 2., 0., 0., 3., 0.});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, 0).ok());
  EXPECT_TRUE(InvertSurface(model.value(), NodeId{20}, {4, 3, 0}, 0).ok());
  EXPECT_FALSE(InvertSurface(model.value(), NodeId{20}, {4, -1, 0}, 1e-5).ok());
  EXPECT_FALSE(InvertSurface(model.value(), NodeId{20}, {4, 4, 0}, 1e-5).ok());
  EXPECT_FALSE(
      InvertSurface(model.value(), NodeId{20}, {4, 1.5, 1e-4}, 1e-5).ok());
  const auto accepted =
      InvertSurface(model.value(), NodeId{20}, {4, 1.5, 1e-4}, 1e-3);
  ASSERT_TRUE(accepted.ok());
  EXPECT_DOUBLE_EQ(accepted.value().residual, 1e-4);
}
TEST(ParasolidSweptSurface, SplineRejectsUncertifiedOrSingularProjections) {
  for (auto controls : {std::vector<ScalarValue>{0., 0., 0., 0., 2., 0., 0.,
                                                 -1., 0., 0., 1., 0.},
                        std::vector<ScalarValue>{0., 0., 0., 0., 0., 0., 0., 1.,
                                                 0., 0., 2., 0.},
                        std::vector<ScalarValue>{0., 0., 0., 1., 0., 0., 2., 0.,
                                                 0., 3., 0., 0.}}) {
    ModelFixture f;
    Swept(f, 1, {1, 0, 0});
    Spline(f, std::move(controls));
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto inverse = InvertSurface(model.value(), NodeId{20}, {}, 1e-6);
    ASSERT_FALSE(inverse.ok());
    EXPECT_EQ(inverse.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidSweptSurface,
     CertifiesPositiveBezierDerivativeDespiteControlReversal) {
  ModelFixture f;
  Swept(f, 1, {1, 0, 0});
  Spline(f, {0., 0., 0., 0., 1., 0., 0., .9, 0., 0., 2., 0.});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (const double t : {.125, .25, .5, .75, .875}) {
    const double y = 3 * t - 3.3 * t * t + 2.3 * t * t * t;
    const auto result =
        InvertSurface(model.value(), NodeId{20}, {.3, y, 0}, 1e-12);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_NEAR(result.value().u, 2 + 3 * t, 1e-12);
    EXPECT_NEAR(result.value().v, .3, 1e-12);
  }
}
TEST(ParasolidSweptSurface, SplineInversionPreservesSmallAndLargeSourceUnits) {
  for (double scale : {1e-150, 1e150}) {
    ModelFixture f;
    Swept(f, 1, {-1, 0, 0});
    Spline(f, {0., 0., 0., 0., scale, 0., 0., 2 * scale, 0., 0., 3 * scale,
               scale});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto inverse =
        InvertSurface(model.value(), NodeId{20},
                      {-2 * scale, 1.5 * scale, .125 * scale}, scale * 1e-12);
    ASSERT_TRUE(inverse.ok()) << inverse.error().message;
    EXPECT_NEAR(inverse.value().u, 3.5, 1e-12);
    EXPECT_NEAR(inverse.value().v / scale, 2, 1e-12);
  }
}
TEST(ParasolidSweptSurface, SplineInversionHasAnExplicitSearchBudget) {
  ModelFixture f;
  Swept(f, 1, {1, 0, 0});
  Spline(f, {0., 0., 0., 0., 1., 0., 0., 2., 0., 0., 3., 0.}, {0., 1.});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto inverse =
      InvertSurface(model.value(), NodeId{20}, {0, 1e-200, 0}, 0);
  ASSERT_FALSE(inverse.ok());
  EXPECT_EQ(inverse.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidSweptSurface, InvertsAnalyticSectionsInOriginalCoordinates) {
  for (int kind = 0; kind < 3; ++kind) {
    ModelFixture f;
    Swept(f, 1, {.36, .48, .8}, true);
    if (kind == 0)
      Line(f, {1, -2, 3});
    else
      Conic(f, kind == 2);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    for (double u : {-2.9, -.5, 0., .8, 2.9}) {
      for (double v : {-10., 0., 3.}) {
        const auto sample = EvaluateSurface(model.value(), NodeId{20}, u, v);
        ASSERT_TRUE(sample.ok());
        const auto inverse = InvertSurface(model.value(), NodeId{20},
                                           sample.value().position, 1e-12);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        EXPECT_NEAR(inverse.value().u, u, 1e-12);
        EXPECT_NEAR(inverse.value().v, v, 1e-12);
        EXPECT_LE(inverse.value().residual, 1e-12);
      }
    }
  }
}
TEST(ParasolidSweptSurface,
     InversionUsesAcceptedSourceFramesAndPrincipalAngles) {
  ModelFixture f;
  Swept(f, 1, {.36, .48, .8 * (1 + 1e-11)});
  Conic(f, true);
  for (auto& node : f.report.nodes) {
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (auto& field : node.fields) {
      const auto& name = fields[field.definition_index].name;
      if (node.index == 1 && name == "normal")
        field.values = {0., 0., 1 + 1e-11};
      if (node.index == 1 && name == "x_axis")
        field.values = {.6 * (1 + 1e-11), .8, 0.};
    }
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 8, -3);
  ASSERT_TRUE(sample.ok());
  const auto inverse = InvertSurface(model.value(), NodeId{20},
                                     sample.value().position, 1e-14, {8, -3});
  ASSERT_TRUE(inverse.ok()) << inverse.error().message;
  EXPECT_NEAR(inverse.value().u, std::remainder(8., 2 * std::acos(-1.)), 1e-14);
  EXPECT_NEAR(inverse.value().v, -3, 1e-14);
  EXPECT_LE(inverse.value().residual, 1e-14);
}
TEST(ParasolidSweptSurface,
     InversionPreservesExactAndExplicitMembershipTolerance) {
  ModelFixture f;
  Swept(f);
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto exact = InvertSurface(model.value(), NodeId{20}, {2, 0, -3}, 0);
  ASSERT_TRUE(exact.ok());
  EXPECT_EQ(exact.value().u, 2);
  EXPECT_EQ(exact.value().v, -3);
  EXPECT_EQ(exact.value().residual, 0);
  const auto outside =
      InvertSurface(model.value(), NodeId{20}, {2, 1e-4, -3}, 1e-5);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  const auto within =
      InvertSurface(model.value(), NodeId{20}, {2, 1e-4, -3}, 1e-3);
  ASSERT_TRUE(within.ok());
  EXPECT_EQ(within.value().residual, 1e-4);
  ModelFixture conic;
  Swept(conic);
  Conic(conic, false);
  const auto cylinder = ModelBuilder::Build(conic.report);
  ASSERT_TRUE(cylinder.ok());
  EXPECT_FALSE(
      InvertSurface(cylinder.value(), NodeId{20}, {1, -2, 3}, 1e-6).ok());
}
TEST(ParasolidSweptSurface, InversionHandlesSmallAndLargeSourceUnits) {
  for (double radius : {1e-150, 1e150}) {
    ModelFixture f;
    Swept(f, 1, {.36, .48, .8});
    Conic(f, true, radius);
    for (auto& node : f.report.nodes) {
      if (node.index != 1) continue;
      const auto& fields =
          f.report.schemas[node.schema_index].definition.fields;
      for (auto& field : node.fields)
        if (fields[field.definition_index].name == "center")
          field.values = {0., 0., 0.};
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto point =
        EvaluateSurface(model.value(), NodeId{20}, .8, -.3 * radius);
    ASSERT_TRUE(point.ok());
    const auto inverse = InvertSurface(model.value(), NodeId{20},
                                       point.value().position, radius * 1e-12);
    ASSERT_TRUE(inverse.ok()) << inverse.error().message;
    EXPECT_NEAR(inverse.value().u, .8, 1e-12);
    EXPECT_NEAR(inverse.value().v / radius, -.3, 1e-12);
    EXPECT_LE(inverse.value().residual, radius * 1e-12);
  }
}
TEST(ParasolidSweptSurface,
     InversionRejectsSingularUnsupportedAndInvalidSections) {
  for (int bad = 0; bad < 5; ++bad) {
    ModelFixture f;
    Swept(f, bad == 3 ? 99 : 1, bad == 4 ? Vector3{0, 0, 2} : Vector3{1, 0, 0});
    if (bad == 0 || bad == 3 || bad == 4) Line(f);
    if (bad == 1) Conic(f, false);
    if (bad == 2) {
      f.Spline(true);
      for (auto& node : f.report.nodes) {
        if (node.index != 2) continue;
        const auto& fields =
            f.report.schemas[node.schema_index].definition.fields;
        for (auto& field : node.fields)
          if (fields[field.definition_index].name == "closed")
            field.values = {true};
      }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto inverse =
        InvertSurface(model.value(), NodeId{20}, {0, 0, 0}, 1e-6);
    ASSERT_FALSE(inverse.ok());
    EXPECT_EQ(inverse.error().code, bad == 1 || bad == 2
                                        ? ErrorCode::kUnsupportedFormat
                                        : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSweptSurface,
     InvertsCertifiedRationalSectionsWithoutChangingWeights) {
  for (const auto direction : {Vector3{0, 0, 1}, Vector3{0, .6, .8}}) {
    ModelFixture f;
    Swept(f, 1, direction);
    f.Spline(true);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    for (double u : {0., .125, .5, .875, 1.})
      for (double v : {-2., 0., .75}) {
        const Vector3 point{2 * u / (1 + 2 * u), 1 + direction.y * v,
                            (2 + 2 * u) / (1 + 2 * u) + direction.z * v};
        const auto inverse =
            InvertSurface(model.value(), NodeId{20}, point, 1e-12);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        EXPECT_NEAR(inverse.value().u, u, 1e-12);
        EXPECT_NEAR(inverse.value().v, v, 1e-12);
      }
    EXPECT_FALSE(
        InvertSurface(model.value(), NodeId{20}, {.5, 1.01, 1.5}, 1e-12).ok());
  }
}
TEST(ParasolidSweptSurface, InversionRejectsNonfiniteRequestsAndParameters) {
  ModelFixture f;
  Swept(f);
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto result :
       {InvertSurface(model.value(), NodeId{20}, {nan, 0, 0}, 1e-6),
        InvertSurface(model.value(), NodeId{20}, {}, -1),
        InvertSurface(model.value(), NodeId{20}, {}, nan),
        InvertSurface(model.value(), NodeId{20}, {}, 1e-6, {0, nan})}) {
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_FALSE(InvertSurface(model.value(), NodeId{99}, {}, 1e-6).ok());
  ModelFixture overflow;
  Swept(overflow, 1, {1, 0, 1e-300});
  Line(overflow);
  const auto thin = ModelBuilder::Build(overflow.report);
  ASSERT_TRUE(thin.ok());
  const auto unrepresentable =
      InvertSurface(thin.value(), NodeId{20}, {0, 0, 1e100}, 0);
  ASSERT_FALSE(unrepresentable.ok());
  EXPECT_EQ(unrepresentable.error().code, ErrorCode::kInvalidRecord);
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
