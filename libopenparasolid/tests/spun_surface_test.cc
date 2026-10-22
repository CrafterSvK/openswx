// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
ModelField NullVector(const char* name) {
  return {{name, FieldType::kVector},
          {std::monostate{}, std::monostate{}, std::monostate{}}};
}
ModelField Nullable(const char* name, std::optional<double> value) {
  return {{name, FieldType::kDouble},
          {value ? ScalarValue{*value} : ScalarValue{std::monostate{}}}};
}
void Spun(ModelFixture& f, uint32_t profile = 1,
          std::optional<double> start = {}, std::optional<double> end = {},
          Vector3 base = {}, Vector3 axis = {0, 0, 1}) {
  f.Add(
      68, 20,
      {Pointer("profile", profile, 1008),
       Numbers("base", {base.x, base.y, base.z}, FieldType::kVector),
       Numbers("axis", {axis.x, axis.y, axis.z}, FieldType::kVector),
       NullVector("start"), NullVector("end"), NullVector("x_axis"),
       Nullable("start_param", start), Nullable("end_param", end),
       Nullable("scale", {}), ByteValue("sense", '-', FieldType::kCharacter)});
}
void Line(ModelFixture& f, Vector3 origin = {2, 0, 0},
          Vector3 direction = {0, 0, 1}) {
  f.Add(30, 1,
        {Numbers("origin", {origin.x, origin.y, origin.z}, FieldType::kVector),
         Numbers("direction", {direction.x, direction.y, direction.z},
                 FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
DecodedField& Field(ModelFixture& f, uint32_t id, const char* name) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    const auto& schema = f.report.schemas[node.schema_index].definition;
    for (auto& field : node.fields)
      if (schema.fields[field.definition_index].name == name) return field;
  }
  ADD_FAILURE() << "Missing fixture field";
  return f.report.nodes.front().fields.front();
}
void ExpectVector(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}
void Circle(ModelFixture& f, Vector3 center = {3, 0, 0},
            Vector3 normal = {0, 1, 0}, Vector3 x = {1, 0, 0},
            double radius = 1) {
  f.Add(31, 1,
        {Numbers("center", {center.x, center.y, center.z}, FieldType::kVector),
         Numbers("normal", {normal.x, normal.y, normal.z}, FieldType::kVector),
         Numbers("x_axis", {x.x, x.y, x.z}, FieldType::kVector),
         Numbers("radius", {radius}),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
TEST(ParasolidSpunSurface, InvertsCircularRingsInSourceCoordinates) {
  for (bool oblique : {false, true}) {
    ModelFixture f;
    if (oblique) {
      Spun(f, 1, {}, {}, {1, 2, 3}, {0, .6, .8});
      Circle(f, {4, 2, 3}, {0, .8, -.6}, {.8, .36, .48});
    } else {
      Spun(f);
      Circle(f);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    for (double u : {-3., -.8, 0., 1.7, 3.}) {
      for (double v : {-2.7, 0., .6, 2.9}) {
        const auto point = EvaluateSurface(model.value(), NodeId{20}, u, v);
        ASSERT_TRUE(point.ok());
        const auto inverse = InvertSurface(model.value(), NodeId{20},
                                           point.value().position, 1e-12);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        EXPECT_NEAR(inverse.value().u, u, 1e-12);
        EXPECT_NEAR(inverse.value().v, v, 1e-12);
        EXPECT_LE(inverse.value().residual, 1e-12);
      }
    }
  }
}
TEST(ParasolidSpunSurface, InversionHonorsLiftedAndOneSidedProfileBounds) {
  for (int bounds = 0; bounds < 3; ++bounds) {
    ModelFixture f;
    Spun(f, 1, bounds == 2 ? std::nullopt : std::optional<double>{4},
         bounds == 1 ? std::nullopt : std::optional<double>{7});
    Circle(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    for (double u : {4., 5.5, 7.}) {
      const auto point = EvaluateSurface(model.value(), NodeId{20}, u, .3);
      ASSERT_TRUE(point.ok());
      const auto inverse = InvertSurface(model.value(), NodeId{20},
                                         point.value().position, 1e-12);
      ASSERT_TRUE(inverse.ok()) << inverse.error().message;
      if (bounds != 2) EXPECT_GE(inverse.value().u, 4);
      if (bounds != 1) EXPECT_LE(inverse.value().u, 7);
      EXPECT_LE(inverse.value().residual, 1e-12);
    }
  }
  ModelFixture f;
  Spun(f, 1, .1, .2);
  Circle(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, 1e-12).ok());
}
TEST(ParasolidSpunSurface, InversionPreservesPreferredPeriodsAndFallsBack) {
  ModelFixture f;
  Spun(f);
  Circle(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const SurfaceParameterSeed seed{8., -9.};
  const auto point = EvaluateSurface(model.value(), NodeId{20}, seed.u, seed.v);
  ASSERT_TRUE(point.ok());
  const auto exact =
      InvertSurface(model.value(), NodeId{20}, point.value().position, 0, seed);
  ASSERT_TRUE(exact.ok());
  EXPECT_EQ(exact.value().u, seed.u);
  EXPECT_EQ(exact.value().v, seed.v);
  EXPECT_EQ(exact.value().residual, 0);
  // At the opposite meridian the local residual is normal to both partials.
  const auto fallback = InvertSurface(model.value(), NodeId{20}, {4, 0, 0},
                                      1e-12, {std::numbers::pi, 0});
  ASSERT_TRUE(fallback.ok()) << fallback.error().message;
  EXPECT_LE(fallback.value().residual, 1e-12);
}
TEST(ParasolidSpunSurface, InversionRefinesAcceptedSourceFrameResiduals) {
  ModelFixture f;
  Spun(f, 1, {}, {}, {}, {0, 0, 1 + 2e-11});
  Circle(f, {3, 0, 0}, {0, 1, 0}, {1 + 2e-11, 0, 0});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto point = EvaluateSurface(model.value(), NodeId{20}, .8, 1.2);
  ASSERT_TRUE(point.ok());
  const auto inverse =
      InvertSurface(model.value(), NodeId{20}, point.value().position, 1e-14);
  ASSERT_TRUE(inverse.ok()) << inverse.error().message;
  EXPECT_LE(inverse.value().residual, 1e-14);
}
TEST(ParasolidSpunSurface, InversionRejectsOffSurfaceAndInvalidRequests) {
  ModelFixture f;
  Spun(f);
  Circle(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (Vector3 point : {Vector3{0, 0, 0}, Vector3{4.01, 0, 0}}) {
    const auto inverse = InvertSurface(model.value(), NodeId{20}, point, 1e-6);
    ASSERT_FALSE(inverse.ok());
    EXPECT_EQ(inverse.error().code, ErrorCode::kInvalidRecord);
  }
  EXPECT_TRUE(InvertSurface(model.value(), NodeId{20}, {4.01, 0, 0}, .02).ok());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto result :
       {InvertSurface(model.value(), NodeId{20}, {nan, 0, 0}, 1e-6),
        InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, -1),
        InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, nan),
        InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, 1e-6, {nan, 0})}) {
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidSpunSurface, InversionRestrictsProfilesToRegularMeridianRings) {
  for (int kind = 0; kind < 4; ++kind) {
    ModelFixture f;
    Spun(f);
    if (kind == 0) Line(f);
    if (kind == 1) Circle(f, {1, 0, 0});
    if (kind == 2) Circle(f, {.5, 0, 0});
    if (kind == 3) Circle(f, {3, 0, 0}, {0, 0, 1});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto inverse = InvertSurface(model.value(), NodeId{20}, {4, 0, 0}, 1);
    ASSERT_FALSE(inverse.ok());
    EXPECT_EQ(inverse.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidSpunSurface, ResolvesForwardProfileAndPreservesNullableMetadata) {
  ModelFixture f;
  Spun(f);
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
  EXPECT_STREQ(GeometryName(geometry.payload), "spun_surface");
  const auto& spun = std::get<SpunSurfaceGeometry>(geometry.payload);
  EXPECT_EQ(spun.profile.value, 1);
  EXPECT_FALSE(spun.start);
  EXPECT_FALSE(spun.end);
  EXPECT_FALSE(spun.start_parameter);
  EXPECT_FALSE(spun.end_parameter);
  EXPECT_FALSE(spun.x_axis);
  EXPECT_FALSE(spun.scale);
  const auto sample =
      EvaluateSurface(model, NodeId{20}, 3, std::numbers::pi / 2);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {0, 2, 3});
  ExpectVector(sample.value().u_derivative, {0, 0, 1});
  ExpectVector(sample.value().v_derivative, {-2, 0, 0});
  EXPECT_FALSE(EvaluateSurface(geometry.payload, 3, 0).ok());
  auto copy = model;
  auto moved = std::move(copy);
  EXPECT_TRUE(EvaluateSurface(moved, NodeId{20}, 3, 0).ok());
}
TEST(ParasolidSpunSurface, RotatesAboutTranslatedAndObliqueWorldAxes) {
  ModelFixture f;
  Spun(f, 1, {}, {}, {1, 2, 3}, {1, 0, 0});
  Line(f, {1, 4, 3}, {1, 0, 0});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample =
      EvaluateSurface(model.value(), NodeId{20}, .5, std::numbers::pi / 2);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {1.5, 2, 5});
  ExpectVector(sample.value().u_derivative, {1, 0, 0});
  ExpectVector(sample.value().v_derivative, {0, -2, 0});
}
TEST(ParasolidSpunSurface, SupportsRationalAndNonrationalNurbsProfiles) {
  for (bool rational : {false, true}) {
    ModelFixture f;
    Spun(f);
    f.Spline(rational);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto sample =
        EvaluateSurface(model.value(), NodeId{20}, .5, std::numbers::pi / 2);
    ASSERT_TRUE(sample.ok());
    if (rational) {
      ExpectVector(sample.value().position, {-1, .5, 1.5});
      ExpectVector(sample.value().u_derivative, {0, .5, -.5});
      ExpectVector(sample.value().v_derivative, {-.5, -1, 0});
    } else {
      ExpectVector(sample.value().position, {-2, 1, 3});
      ExpectVector(sample.value().u_derivative, {-2, 2, 2});
      ExpectVector(sample.value().v_derivative, {-1, -2, 0});
    }
    EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{20}, 1.5, 0).ok());
  }
}
TEST(ParasolidSpunSurface, CircleProfileCanGenerateATorus) {
  ModelFixture f;
  Spun(f);
  f.Add(
      31, 1,
      {Numbers("center", {3, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 1, 0}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {1}),
       ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample =
      EvaluateSurface(model.value(), NodeId{20}, 0, std::numbers::pi / 2);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {0, 4, 0});
  ExpectVector(sample.value().u_derivative, {0, 0, -1});
  ExpectVector(sample.value().v_derivative, {-4, 0, 0});
}
TEST(ParasolidSpunSurface, InclusiveAndOneSidedProfileBoundsAreEnforced) {
  for (bool both : {false, true}) {
    ModelFixture f;
    Spun(f, 1, 1, both ? std::optional<double>{3} : std::nullopt);
    Line(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_TRUE(EvaluateSurface(model.value(), NodeId{20}, 1, 0).ok());
    EXPECT_TRUE(
        EvaluateSurface(model.value(), NodeId{20}, 3, 7 * std::numbers::pi)
            .ok());
    const auto before =
        EvaluateSurface(model.value(), NodeId{20}, std::nextafter(1., 0.), 0);
    ASSERT_FALSE(before.ok());
    EXPECT_EQ(before.error().code, ErrorCode::kInvalidOptions);
    const auto after = EvaluateSurface(model.value(), NodeId{20}, 4, 0);
    EXPECT_EQ(after.ok(), !both);
  }
}
TEST(ParasolidSpunSurface, AuxiliaryMetadataDoesNotScaleOrRotateTheProfile) {
  ModelFixture f;
  Spun(f);
  Line(f);
  Field(f, 20, "scale").values = {42.};
  Field(f, 20, "start").values = {1., 2., 3.};
  Field(f, 20, "end").values = {4., 5., 6.};
  Field(f, 20, "x_axis").values = {0., 1., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 1, 0);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {2, 0, 1});
  const auto& geometry =
      model.value().geometry()[*model.value().FindNode(NodeId{20})->geometry];
  const auto& spun = std::get<SpunSurfaceGeometry>(geometry.payload);
  EXPECT_EQ(spun.scale, 42);
  ASSERT_TRUE(spun.start);
  EXPECT_EQ(spun.start->y, 2);
}
TEST(ParasolidSpunSurface, PartialsAgreeWithCentralDifferences) {
  ModelFixture f;
  Spun(f, 1, {}, {}, {1, -2, 3}, {0, .6, .8});
  f.Spline(true);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  constexpr double h = 1e-5;
  const double u = .4, v = -1.3;
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, u, v);
  ASSERT_TRUE(sample.ok());
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
TEST(ParasolidSpunSurface,
     RejectsMalformedDefinitionsAndPartiallyUnsetVectors) {
  for (int bad = 0; bad < 5; ++bad) {
    ModelFixture f;
    Spun(f, bad == 0 ? 0 : 1);
    Line(f);
    if (bad == 1) Field(f, 20, "axis").values = {0., 0., 0.};
    if (bad == 2) Field(f, 20, "start").values[1] = 1.;
    if (bad == 3)
      Field(f, 20, "scale").values = {std::numeric_limits<double>::infinity()};
    if (bad == 4) {
      Field(f, 20, "start_param").values = {3.};
      Field(f, 20, "end_param").values = {1.};
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_EQ(model.value().statistics().invalid_geometry, 1);
    EXPECT_FALSE(model.value().FindNode(NodeId{20})->geometry);
  }
}
TEST(ParasolidSpunSurface, RejectsNonunitAxesAndMissingWrongOrCyclicProfiles) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    Spun(f, bad == 1 ? 99 : bad == 2 ? 20 : 1);
    Line(f);
    if (bad == 0) Field(f, 20, "axis").values = {0., 0., 2.};
    if (bad == 3) {
      f.report.nodes[1].node_type = 137;
      f.report.schemas[f.report.nodes[1].schema_index].definition.node_type =
          137;
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto sample = EvaluateSurface(model.value(), NodeId{20}, 0, 0);
    ASSERT_FALSE(sample.ok());
    EXPECT_EQ(sample.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidSpunSurface, ProfileMustBeThreeDimensional) {
  ModelFixture f;
  Spun(f);
  f.Spline();
  Field(f, 2, "vertex_dim").values = {int64_t{2}};
  auto& vertices = Field(f, 3, "vertices");
  vertices.values = {0., 0., 1., 1., 2., 2.};
  vertices.elements = 6;
  f.report.nodes[3].variable_length = 6;
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, .5, 0);
  ASSERT_FALSE(sample.ok());
  EXPECT_EQ(sample.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidSpunSurface, RejectsNonfiniteParametersAndNumericalOverflow) {
  ModelFixture f;
  Spun(f);
  Line(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{20},
                               std::numeric_limits<double>::quiet_NaN(), 0)
                   .ok());
  EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{20}, 0,
                               std::numeric_limits<double>::infinity())
                   .ok());
  const double largest = std::numeric_limits<double>::max();
  ModelFixture huge;
  Spun(huge, 1, {}, {}, {-largest, 0, 0});
  Line(huge, {largest, 0, 0});
  const auto huge_model = ModelBuilder::Build(huge.report);
  ASSERT_TRUE(huge_model.ok());
  const auto overflow =
      EvaluateSurface(huge_model.value(), NodeId{20}, 0, std::numbers::pi);
  ASSERT_FALSE(overflow.ok());
  EXPECT_EQ(overflow.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidSpunSurface, SingularAxisPointsRemainEvaluable) {
  ModelFixture f;
  Spun(f);
  Line(f, {0, 0, 0});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 1, .4);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {0, 0, 1});
  ExpectVector(sample.value().v_derivative, {0, 0, 0});
}
TEST(ParasolidSpunSurface,
     SurfaceAndTrimmedCurveCompositionUseTheModelSurface) {
  ModelFixture f;
  Spun(f);
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
    ExpectVector(sample.value().position,
                 {2 * std::cos(1.), 2 * std::sin(1.), .5});
    ExpectVector(sample.value().first_derivative,
                 {-4 * std::sin(1.), 4 * std::cos(1.), 1});
  }
}
TEST(ParasolidSpunSurface, ModelEntryPointAlsoSupportsStandaloneSurfaces) {
  ModelFixture f;
  f.Add(50, 20,
        {Numbers("origin", {1, 2, 3}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto sample = EvaluateSurface(model.value(), NodeId{20}, 2, 3);
  ASSERT_TRUE(sample.ok());
  ExpectVector(sample.value().position, {3, 5, 3});
  const auto missing = EvaluateSurface(model.value(), NodeId{99}, 0, 0);
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.error().code, ErrorCode::kInvalidRecord);
}
}  // namespace
}  // namespace openparasolid
