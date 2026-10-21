// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/evaluate.h"

namespace openparasolid {
namespace {
using namespace test;
ModelField NullNumber(const char* name) {
  return {{name, FieldType::kDouble}, {std::monostate{}}};
}
void Intersection(ModelFixture& f, uint32_t id = 1, bool modern = false) {
  std::vector<ModelField> fields{
      {{"surface", FieldType::kPointer, 1006, 2, true},
       {uint64_t{50}, uint64_t{51}}},
      Pointer("chart", 2, 40),
      Pointer("start", 3, 41),
      Pointer("end", 4, 41),
      ByteValue("sense", '?', FieldType::kCharacter)};
  if (modern) fields.push_back(Pointer("intersection_data", 0, 204));
  f.Add(38, id, std::move(fields));
}
void Chart(ModelFixture& f) {
  f.Add(40, 2,
        {Numbers("base_parameter", {-0.}),
         Numbers("base_scale", {2.}),
         Integer("chart_count", 2),
         NullNumber("chordal_error"),
         Numbers("angular_error", {.1}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               {0., 0., -0., 1., 2., 3.})});
}
void Limit(ModelFixture& f, uint32_t id, uint8_t type = 'L',
           bool modern = false) {
  std::vector<ModelField> fields{
      ByteValue("type", type, FieldType::kCharacter)};
  if (modern)
    fields.push_back(ByteValue("term_use", 'S', FieldType::kCharacter));
  auto positions =
      Array("positions", FieldType::kIntersectionPoint, {0., 0., 0.});
  if (type == 'T')
    positions.values.insert(positions.values.end(), {1., 2., 3.});
  fields.push_back(std::move(positions));
  f.Add(41, id, std::move(fields));
}
void Sources(ModelFixture& f, bool modern = false) {
  Chart(f);
  Limit(f, 3, 'T', modern);
  Limit(f, 4, 'H', modern);
}
const IntersectionCurveGeometry& Curve(const Model& model) {
  return std::get<IntersectionCurveGeometry>(
      model.geometry()[*model.FindNode(NodeId{1})->geometry].payload);
}
TEST(ParasolidIntersectionModel, ResolvesChartAndLimitsWithForwardReferences) {
  ModelFixture f;
  Intersection(f);
  Sources(f);
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto& model = built.value();
  EXPECT_TRUE(model.typed_geometry_complete());
  EXPECT_FALSE(model.references_verified());
  const auto& curve = Curve(model);
  EXPECT_EQ(curve.surfaces[0].value, 50);
  EXPECT_EQ(curve.surfaces[1].value, 51);
  EXPECT_EQ(curve.chart.source.value, 2);
  EXPECT_EQ(curve.chart.base_scale, 2);
  EXPECT_TRUE(std::signbit(curve.chart.base_parameter));
  EXPECT_FALSE(curve.chart.chordal_error);
  EXPECT_EQ(curve.chart.angular_error, .1);
  EXPECT_FALSE(curve.chart.parameter_error[0]);
  ASSERT_EQ(curve.chart.positions.size(), 2);
  EXPECT_TRUE(std::signbit(curve.chart.positions[0].z));
  EXPECT_EQ(curve.chart.positions[1].y, 2);
  EXPECT_EQ(curve.start.type, 'T');
  EXPECT_EQ(curve.start.positions.size(), 2);
  EXPECT_EQ(curve.end.type, 'H');
  EXPECT_EQ(curve.end.positions.size(), 1);
  EXPECT_FALSE(curve.start.term_use);
  EXPECT_FALSE(curve.intersection_data);
  EXPECT_STREQ(GeometryName(model.geometry()[0].payload), "intersection_curve");
  auto copy = model;
  auto moved = std::move(copy);
  EXPECT_EQ(Curve(moved).chart.positions[1].z, 3);
}
TEST(ParasolidIntersectionModel, PreservesOptionalFieldsFromResolvedSchema) {
  ModelFixture f;
  Intersection(f, 1, true);
  Sources(f, true);
  // Reorder a fixed-field definition and its decoded fields together.
  auto& node = f.report.nodes[0];
  auto& schema = f.report.schemas[node.schema_index].definition;
  std::swap(schema.fields[0], schema.fields[1]);
  std::swap(node.fields[0], node.fields[1]);
  node.fields[0].definition_index = 0;
  node.fields[1].definition_index = 1;
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  ASSERT_TRUE(built.value().typed_geometry_complete());
  EXPECT_EQ(Curve(built.value()).intersection_data, NodeId{0});
  EXPECT_EQ(Curve(built.value()).start.term_use, 'S');
}
TEST(ParasolidIntersectionModel, MissingWrongAndNullAuxiliariesStayInvalid) {
  for (int bad = 0; bad < 3; ++bad) {
    ModelFixture f;
    Intersection(f);
    if (bad == 1) f.Point(2);
    if (bad == 2) f.report.nodes[0].fields[1].values[0] = uint64_t{0};
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    EXPECT_EQ(built.value().statistics().invalid_geometry, 1);
    EXPECT_FALSE(built.value().FindNode(NodeId{1})->geometry);
  }
}
TEST(ParasolidIntersectionModel, RejectsInvalidChartMetadataAndPositionValues) {
  for (int bad = 0; bad < 7; ++bad) {
    ModelFixture f;
    Intersection(f);
    Sources(f);
    auto& fields = f.report.nodes[1].fields;
    if (bad == 0) fields[0].values[0] = std::monostate{};
    if (bad == 1) fields[1].values[0] = 0.;
    if (bad == 2) fields[2].values[0] = int64_t{3};
    if (bad == 3) fields[3].values[0] = -1.;
    if (bad == 4) fields[4].values[0] = std::numeric_limits<double>::infinity();
    if (bad == 5)
      fields[5].values[0] = std::numeric_limits<double>::quiet_NaN();
    if (bad == 6) fields[6].values[0] = std::monostate{};
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    EXPECT_EQ(built.value().statistics().invalid_geometry, 1) << bad;
    EXPECT_FALSE(built.value().FindNode(NodeId{1})->geometry);
  }
}
TEST(ParasolidIntersectionModel, RejectsNullSurfaceReferences) {
  ModelFixture f;
  Intersection(f);
  Sources(f);
  f.report.nodes[0].fields[0].values[0] = uint64_t{0};
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  EXPECT_EQ(built.value().statistics().invalid_geometry, 1);
}
TEST(ParasolidIntersectionModel, LimitCardinalityAndUnknownKindsAreExplicit) {
  for (uint8_t type : {'H', 'L', 'B', '?'}) {
    ModelFixture f;
    Intersection(f);
    Sources(f);
    f.report.nodes[2].fields[0].values[0] = uint64_t{type};
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    EXPECT_FALSE(built.value().FindNode(NodeId{1})->geometry);
    if (type == 'H' || type == 'L')
      EXPECT_EQ(built.value().statistics().invalid_geometry, 1);
    else
      EXPECT_EQ(built.value().statistics().unsupported_geometry, 1);
  }
}
TEST(ParasolidIntersectionModel, DoesNotIgnoreWrongCodecOfOptionalFields) {
  for (bool chart_field : {false, true}) {
    ModelFixture f;
    Intersection(f, 1, true);
    Sources(f, true);
    const auto index = chart_field ? 2u : 0u;
    const auto field = chart_field ? 1u : 5u;
    auto& node = f.report.nodes[index];
    auto& definition =
        f.report.schemas[node.schema_index].definition.fields[field];
    definition.type = chart_field ? FieldType::kByte : FieldType::kInteger;
    definition.pointer_class = 0;
    node.fields[field].values[0] =
        chart_field ? ScalarValue{uint64_t{'S'}} : ScalarValue{int64_t{0}};
    const auto built = ModelBuilder::Build(f.report);
    ASSERT_TRUE(built.ok());
    EXPECT_EQ(built.value().statistics().invalid_geometry, 1);
  }
}
TEST(ParasolidIntersectionModel,
     SharedChartsAndLimitsRespectCopiedValueBudget) {
  ModelFixture f;
  // Each raw intersection costs 6 scalars, each typed copy 32 scalars.
  for (uint32_t id = 10; id < 20; ++id) Intersection(f, id);
  Sources(f);
  uint64_t raw = 0;
  for (const auto& node : f.report.nodes)
    for (const auto& field : node.fields) raw += field.values.size();
  ModelOptions options;
  options.max_values = 319;
  ASSERT_LT(raw, options.max_values);
  const auto limited = ModelBuilder::Build(f.report, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  options.max_values = 320;
  const auto built = ModelBuilder::Build(f.report, options);
  ASSERT_TRUE(built.ok());
  EXPECT_EQ(built.value().geometry().size(), 10);
}
TEST(ParasolidIntersectionModel, TypedChartDoesNotPretendToEvaluateExactCurve) {
  ModelFixture f;
  Intersection(f);
  Sources(f);
  f.report.nodes[0].fields[4].values[0] = uint64_t{'+'};
  f.Add(133, 10,
        {Pointer("basis_curve", 1, 1008),
         Numbers("point_1", {0., 0., 0.}, FieldType::kVector),
         Numbers("point_2", {1., 2., 3.}, FieldType::kVector),
         Numbers("parm_1", {0.}), Numbers("parm_2", {1.}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto built = ModelBuilder::Build(f.report);
  ASSERT_TRUE(built.ok());
  const auto sample = EvaluateCurve(built.value(), NodeId{1}, 0.);
  ASSERT_FALSE(sample.ok());
  EXPECT_EQ(sample.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(sample.error().message.find("solver"), std::string::npos);
  const auto trimmed = EvaluateCurve(built.value(), NodeId{10}, .5);
  ASSERT_FALSE(trimmed.ok());
  EXPECT_EQ(trimmed.error().code, ErrorCode::kUnsupportedFormat);
  const auto standalone =
      EvaluateCurve(built.value().geometry()[0].payload, 0.);
  ASSERT_FALSE(standalone.ok());
  EXPECT_EQ(standalone.error().code, ErrorCode::kUnsupportedFormat);
}
}  // namespace
}  // namespace openparasolid
