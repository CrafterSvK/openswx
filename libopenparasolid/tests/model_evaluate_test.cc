// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
void SurfaceCurve(ModelFixture& f, uint32_t surface = 20, uint32_t uv = 1) {
  f.Add(137, 10,
        {Pointer("surface", surface, 1006), Pointer("b_curve", uv, 134),
         Pointer("original", 10, 1008), Numbers("tolerance_to_original", {0}),
         ByteValue("sense", '?', FieldType::kCharacter)});
}
void ParameterCurve(ModelFixture& f, bool rational = false,
                    bool spatial = false) {
  f.Curve();
  f.Add(136, 2,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim",
                 spatial    ? 3
                 : rational ? 3
                            : 2,
                 FieldType::kShort),
         Integer("n_knots", 2), ByteValue("knot_type", 2),
         Logical("periodic", false), Logical("closed", false),
         Logical("rational", rational), ByteValue("curve_form", 1),
         Pointer("bspline_vertices", 3, 45), Pointer("knots", 4, 128),
         Pointer("knot_mult", 5, 127)});
  const std::vector<ScalarValue> values =
      spatial    ? std::vector<ScalarValue>{0., 0., 0., 1., 2., 3.}
      : rational ? std::vector<ScalarValue>{0., 0., 1., 2., 4., 2.}
                 : std::vector<ScalarValue>{0., 0., 1., 2.};
  f.Add(45, 3, {Array("vertices", FieldType::kDouble, values)});
  f.Add(128, 4, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 5,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
}
void Plane(ModelFixture& f) {
  f.Add(50, 20,
        {Numbers("origin", {1, 2, 3}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
void Cylinder(ModelFixture& f) {
  f.Add(
      51, 20,
      {Numbers("origin", {0, 0, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {4}),
       ByteValue("sense", '-', FieldType::kCharacter)});
}
void NurbsSurface(ModelFixture& f) {
  ModelFixture source;
  source.Spline(false, true);
  for (const auto& node : source.report.nodes) {
    std::vector<ModelField> fields;
    const auto& schema = source.report.schemas[node.schema_index].definition;
    for (const auto& field : node.fields) {
      auto values = field.values;
      const auto& def = schema.fields[field.definition_index];
      if (def.type == FieldType::kPointer) {
        for (auto& value : values) {
          auto& target = std::get<uint64_t>(value);
          if (target) target += 19;
        }
      }
      fields.push_back({def, std::move(values)});
    }
    f.Add(node.node_type, node.index + 19, std::move(fields));
  }
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
TEST(ParasolidModelEvaluate,
     BuildsForwardReferencedSurfaceCurveWithoutRecursingOriginal) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  Plane(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(model.value().references_verified());
  EXPECT_TRUE(model.value().typed_geometry_complete());
  const auto* node = model.value().FindNode(NodeId{10});
  ASSERT_NE(node, nullptr);
  ASSERT_TRUE(node->geometry);
  const auto& geometry = model.value().geometry()[*node->geometry];
  EXPECT_STREQ(GeometryName(geometry.payload), "surface_curve");
  EXPECT_EQ(geometry.sense, '?');
  const auto& sp = std::get<SurfaceCurveGeometry>(geometry.payload);
  EXPECT_EQ(sp.surface.value, 20);
  EXPECT_EQ(sp.parameter_curve.value, 1);
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .4);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1.4, 2.8, 3});
  ExpectVector(result.value().first_derivative, {1, 2, 0});
  EXPECT_FALSE(EvaluateCurve(geometry.payload, .4).ok());
  auto copied = model.value();
  auto moved = std::move(copied);
  EXPECT_TRUE(EvaluateCurve(moved, NodeId{10}, .4).ok());
}
TEST(ParasolidModelEvaluate, ComposesRationalUVWithAnalyticSurface) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f, true);
  Plane(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1 + 2. / 3, 2 + 4. / 3, 3});
  ExpectVector(result.value().first_derivative, {8. / 9, 16. / 9, 0});
}
TEST(ParasolidModelEvaluate, CylinderCompositionUsesBothNaturalPartials) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  Cylinder(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .5);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position,
               {4 * std::cos(.5), 4 * std::sin(.5), 1});
  ExpectVector(result.value().first_derivative,
               {-4 * std::sin(.5), 4 * std::cos(.5), 2});
  constexpr double h = 1e-5;
  const auto before = EvaluateCurve(model.value(), NodeId{10}, .5 - h);
  const auto after = EvaluateCurve(model.value(), NodeId{10}, .5 + h);
  ASSERT_TRUE(before.ok());
  ASSERT_TRUE(after.ok());
  const auto& a = before.value().position;
  const auto& b = after.value().position;
  ExpectVector(
      result.value().first_derivative,
      {(b.x - a.x) / (2 * h), (b.y - a.y) / (2 * h), (b.z - a.z) / (2 * h)},
      1e-8);
}
TEST(ParasolidModelEvaluate, ComposesNurbsSurfaceAndPropagatesBothDomains) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  NurbsSurface(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .25);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1, 2, 3});
  ExpectVector(result.value().first_derivative, {4, 4, 4});
  for (double t : {.75, 1.5}) {
    const auto outside = EvaluateCurve(model.value(), NodeId{10}, t);
    ASSERT_FALSE(outside.ok());
    EXPECT_EQ(outside.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidModelEvaluate,
     PreservesReferenceIssuesAndRejectsWrongOrCyclicTargets) {
  for (const auto& targets :
       {std::pair{99u, 1u}, std::pair{20u, 99u}, std::pair{10u, 1u},
        std::pair{20u, 10u}, std::pair{20u, 20u}}) {
    ModelFixture f;
    SurfaceCurve(f, targets.first, targets.second);
    ParameterCurve(f);
    Plane(f);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_FALSE(model.value().references_verified());
    const auto result = EvaluateCurve(model.value(), NodeId{10}, .4);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidModelEvaluate,
     RejectsNullDefinitionsAndThreeDimensionalParameterCurves) {
  ModelFixture f;
  SurfaceCurve(f, 0);
  ParameterCurve(f);
  Plane(f);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_EQ(model.value().statistics().invalid_geometry, 1);
  EXPECT_FALSE(model.value().typed_geometry_complete());
  ModelFixture spatial;
  SurfaceCurve(spatial);
  ParameterCurve(spatial, false, true);
  Plane(spatial);
  const auto spatial_model = ModelBuilder::Build(spatial.report);
  ASSERT_TRUE(spatial_model.ok());
  const auto result = EvaluateCurve(spatial_model.value(), NodeId{10}, .4);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidModelEvaluate,
     UnsupportedSupportingSurfaceHasNoApproximationFallback) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  f.Add(56, 20, {});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .4);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidModelEvaluate,
     PlainCurvesAndMissingIdsUseTheSameModelEntryPoint) {
  ModelFixture f;
  f.Add(30, 1,
        {Numbers("origin", {1, 2, 3}, FieldType::kVector),
         Numbers("direction", {0, 0, 1}, FieldType::kVector),
         ByteValue("sense", '-', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{1}, 4);
  ASSERT_TRUE(result.ok());
  ExpectVector(result.value().position, {1, 2, 7});
  EXPECT_FALSE(EvaluateCurve(model.value(), NodeId{99}, 0).ok());
  const auto invalid = EvaluateCurve(model.value(), NodeId{1},
                                     std::numeric_limits<double>::quiet_NaN());
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidModelEvaluate, RejectsOverflowInTheComposedDerivative) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  Cylinder(f);
  Field(f, 3, "vertices").values = {0., 0.,
                                    std::numeric_limits<double>::max() / 2, 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidModelEvaluate, PropagatesParameterCurveResourceLimits) {
  ModelFixture f;
  SurfaceCurve(f);
  ParameterCurve(f);
  Plane(f);
  Field(f, 2, "degree").values = {int64_t{65}};
  Field(f, 2, "n_vertices").values = {int64_t{66}};
  auto& controls = Field(f, 3, "vertices");
  controls.values.assign(132, 0.);
  controls.elements = 132;
  Field(f, 5, "multiplicities").values = {int64_t{66}, int64_t{66}};
  // The synthetic variable-array count is part of the decoded report contract.
  for (auto& node : f.report.nodes)
    if (node.index == 3) node.variable_length = 132;
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = EvaluateCurve(model.value(), NodeId{10}, .4);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
