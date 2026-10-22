// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <limits>

#include "model_test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
TEST(ParasolidModel, RetainsTransmittedEdgeTolerancesWithoutInferringDefaults) {
  ModelFixture f;
  f.Add(16, 1, {Numbers("tolerance", {2.5e-7})});
  f.Add(16, 2, {});
  f.Add(14, 3, {Numbers("tolerance", {1.})});
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().FindNode(NodeId{1})->edge_tolerance.has_value());
  EXPECT_DOUBLE_EQ(*result.value().FindNode(NodeId{1})->edge_tolerance, 2.5e-7);
  EXPECT_FALSE(result.value().FindNode(NodeId{2})->edge_tolerance.has_value());
  EXPECT_FALSE(result.value().FindNode(NodeId{3})->edge_tolerance.has_value());
}
TEST(ParasolidModel, ResolvesForwardNullCyclicAndTypedReferences) {
  ModelFixture f;
  f.Add(74, 8,
        {Pointer("next_block", 8, 74), Pointer("entry", 9, 29),
         Pointer("empty", 0, 29)});
  f.Point(9);
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  const auto& model = result.value();
  ASSERT_EQ(model.nodes().size(), 2);
  EXPECT_TRUE(model.references_verified());
  EXPECT_TRUE(model.typed_geometry_complete());
  EXPECT_EQ(model.statistics().resolved_references, 2);
  EXPECT_EQ(model.statistics().null_references, 1);
  EXPECT_EQ(model.nodes()[0].references[0].target_node, 0);
  EXPECT_EQ(model.nodes()[0].references[1].target_node, 1);
  EXPECT_EQ(model.FindNode(NodeId{9}), &model.nodes()[1]);
  EXPECT_EQ(model.FindNode(NodeId{10}), nullptr);
  ASSERT_EQ(model.geometry().size(), 1);
  EXPECT_EQ(std::get<PointGeometry>(model.geometry()[0].payload).position.y,
            -2.);
}
TEST(ParasolidModel, DistinguishesMissingWrongAndUnreviewedClasses) {
  ModelFixture f;
  f.Add(74, 8,
        {Pointer("missing", 100, 29), Pointer("wrong", 9, 50),
         Pointer("unchecked", 9, 60000), Pointer("geometry", 9, 1003)});
  f.Point(9);
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  const auto& m = result.value();
  EXPECT_FALSE(m.references_verified());
  EXPECT_EQ(m.statistics().missing_references, 1);
  EXPECT_EQ(m.statistics().wrong_class_references, 1);
  EXPECT_EQ(m.statistics().unverified_references, 1);
  EXPECT_EQ(m.statistics().resolved_references, 1);
  EXPECT_EQ(m.issues().size(), 3);
  EXPECT_EQ(m.nodes()[0].references[0].status,
            ModelReferenceStatus::kMissingTarget);
  EXPECT_EQ(m.nodes()[0].references[1].status,
            ModelReferenceStatus::kWrongClass);
  EXPECT_EQ(m.nodes()[0].references[2].status,
            ModelReferenceStatus::kUnverifiedClass);
}
TEST(ParasolidModel, BodyOwnerUnionAcceptsWorldAndBodyButRejectsOtherTypes) {
  ModelFixture f;
  f.Add(12, 1,
        {Pointer("world", 2, 1040), Pointer("parent", 3, 1040),
         Pointer("wrong", 4, 1040), Pointer("future", 5, 1040)});
  f.Add(101, 2, {});
  f.Add(12, 3, {});
  f.Add(13, 4, {});
  f.Add(250, 5, {});
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  const auto& refs = result.value().nodes()[0].references;
  EXPECT_EQ(refs[0].status, ModelReferenceStatus::kResolved);
  EXPECT_EQ(refs[1].status, ModelReferenceStatus::kResolved);
  EXPECT_EQ(refs[2].status, ModelReferenceStatus::kWrongClass);
  EXPECT_EQ(refs[3].status, ModelReferenceStatus::kUnverifiedClass);
}
TEST(ParasolidModel, DoesNotGuessMembershipForFutureTargetTypes) {
  ModelFixture f;
  f.Add(74, 1, {Pointer("future_curve", 2, 1008)});
  f.Add(250, 2, {});
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().statistics().unverified_references, 1);
  EXPECT_EQ(result.value().statistics().wrong_class_references, 0);
}
TEST(ParasolidModel, RejectsPartialDeltaAndMalformedReports) {
  ModelFixture fixture;
  fixture.Point(1);
  for (int change = 0; change < 11; ++change) {
    auto r = fixture.report;
    switch (change) {
      case 0:
        r.complete = false;
        break;
      case 1:
        r.header.kind = StreamKind::kDeltas;
        break;
      case 2:
        r.nodes[0].schema_index = 100;
        break;
      case 3:
        r.nodes[0].index = 0;
        break;
      case 4:
        r.nodes.push_back(r.nodes[0]);
        break;
      case 5:
        r.nodes[0].fields[0].components = 1;
        break;
      case 6:
        r.nodes[0].fields[0].values[0] = uint64_t{1};
        break;
      case 7:
        r.nodes[0].fields[0].definition_index = 100;
        break;
      case 8:
        r.nodes[0].fields[0].offset = r.byte_size;
        break;
      case 9:
        r.delta_nodes.push_back(r.nodes[0]);
        break;
      case 10:
        r.nodes[0].node_type = UINT16_MAX;
        break;
    }
    EXPECT_FALSE(ModelBuilder::Build(r).ok()) << change;
  }
}
TEST(ParasolidModel, LimitsBoundNodesFieldsReferencesValuesAndDiagnostics) {
  ModelFixture f;
  f.Point(9);
  f.Add(74, 8,
        {Pointer("one", 10, 29), Pointer("two", 11, 29),
         Pointer("three", 12, 29)});
  for (int cap = 0; cap < 4; ++cap) {
    ModelOptions o;
    if (cap == 0) o.max_nodes = 1;
    if (cap == 1) o.max_fields = 2;
    if (cap == 2) o.max_references = 1;
    if (cap == 3) o.max_values = 2;
    const auto result = ModelBuilder::Build(f.report, o);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
  ModelOptions o;
  o.max_diagnostics = 1;
  const auto result = ModelBuilder::Build(f.report, o);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().issues().size(), 1);
  EXPECT_EQ(result.value().statistics().total_issues, 3);
  EXPECT_TRUE(result.value().issues_truncated());
  o.max_values = 0;
  EXPECT_FALSE(ModelBuilder::Build(f.report, o).ok());
}
TEST(ParasolidModel,
     OwnsGeometryAndIndexAcrossReportDestructionCopiesAndMoves) {
  auto build = [] {
    ModelFixture f;
    f.Point(7);
    return ModelBuilder::Build(f.report);
  };
  const auto result = build();
  ASSERT_TRUE(result.ok());
  Model copy = result.value();
  Model moved = std::move(copy);
  ASSERT_NE(moved.FindNode(NodeId{7}), nullptr);
  EXPECT_EQ(moved.FindNode(NodeId{7})->geometry, 0);
  EXPECT_EQ(std::get<PointGeometry>(moved.geometry()[0].payload).position.z,
            3.);
}
TEST(ParasolidModel, ReorderedFieldsAndInsertedValuesDoNotChangeRoles) {
  ModelFixture f;
  f.Add(30, 1,
        {Numbers("unused", {99.}),
         Numbers("direction", {0., 2., 0.}, FieldType::kVector),
         ByteValue("sense", '?', FieldType::kCharacter),
         Numbers("origin", {-4., 5., 6.}, FieldType::kVector)});
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().geometry().size(), 1);
  const auto& g = result.value().geometry()[0];
  EXPECT_EQ(g.sense, '?');
  const auto& line = std::get<LineGeometry>(g.payload);
  EXPECT_EQ(line.origin.x, -4.);
  EXPECT_EQ(line.direction.y, 2.);  // No normalization.
}
TEST(ParasolidModel,
     InvalidAndUnsupportedGeometryRemainExplicitWithoutPartialObjects) {
  for (int bad = 0; bad < 5; ++bad) {
    ModelFixture f;
    const double value = bad == 1   ? std::numeric_limits<double>::infinity()
                         : bad == 0 ? 0.
                                    : 1.;
    f.Add(bad == 4 ? 130 : 31, 1,
          {Numbers("center", {0., 0., 0.}, FieldType::kVector),
           Numbers("normal", {0., 0., bad == 2 ? 0. : 1.}, FieldType::kVector),
           Numbers("x_axis", {bad == 3 ? 0. : 1., 0., bad == 3 ? 1. : 0.},
                   FieldType::kVector),
           Numbers("radius", {value}),
           ByteValue("sense", '+', FieldType::kCharacter)});
    const auto result = ModelBuilder::Build(f.report);
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().geometry().empty());
    EXPECT_FALSE(result.value().typed_geometry_complete());
    EXPECT_EQ(result.value().statistics().geometry_candidates, 1);
    ASSERT_EQ(result.value().issues().size(), 1);
    EXPECT_EQ(result.value().issues()[0].code,
              bad == 4 ? ModelIssueCode::kUnsupportedGeometry
                       : ModelIssueCode::kInvalidGeometry);
  }
}
TEST(ParasolidModel, PreservesLemonTorusSignAndRejectsItsDegenerateLimits) {
  for (double major : {-1., -2., 0., 2.}) {
    ModelFixture f;
    f.Add(54, 1,
          {Numbers("center", {0., 0., 0.}, FieldType::kVector),
           Numbers("axis", {0., 0., 1.}, FieldType::kVector),
           Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
           Numbers("major_radius", {major}), Numbers("minor_radius", {2.}),
           ByteValue("sense", '-', FieldType::kCharacter)});
    const auto result = ModelBuilder::Build(f.report);
    ASSERT_TRUE(result.ok());
    const bool valid = major == -1. || major == 2.;
    EXPECT_EQ(result.value().typed_geometry_complete(), valid);
    if (valid) {
      ASSERT_EQ(result.value().geometry().size(), 1);
      EXPECT_EQ(std::get<TorusGeometry>(result.value().geometry()[0].payload)
                    .major_radius,
                major);
    } else
      EXPECT_TRUE(result.value().geometry().empty());
  }
}
TEST(ParasolidModel, BuildsAnalyticFamiliesAndPreservesCoefficients) {
  for (uint16_t type : {uint16_t{31}, uint16_t{32}, uint16_t{50}, uint16_t{51},
                        uint16_t{52}, uint16_t{53}, uint16_t{54}}) {
    ModelFixture f;
    std::vector<ModelField> fields{
        Numbers(type == 50 || type == 51 || type == 52 ? "origin" : "center",
                {1., 2., 3.}, FieldType::kVector),
        Numbers(type == 31 || type == 32 || type == 50 ? "normal" : "axis",
                {0., 0., 1.}, FieldType::kVector),
        Numbers("x_axis", {1., 0., 0.}, FieldType::kVector),
        ByteValue("sense", '-', FieldType::kCharacter)};
    if (type == 32 || type == 54) {
      fields.push_back(Numbers("major_radius", {4.}));
      fields.push_back(Numbers("minor_radius", {2.}));
    } else if (type != 50)
      fields.push_back(Numbers("radius", {3.}));
    if (type == 52) {
      fields.push_back(Numbers("sin_half_angle", {.6}));
      fields.push_back(Numbers("cos_half_angle", {.8}));
    }
    f.Add(type, 1, std::move(fields));
    const auto result = ModelBuilder::Build(f.report);
    ASSERT_TRUE(result.ok());
    ASSERT_EQ(result.value().geometry().size(), 1) << type;
    EXPECT_TRUE(result.value().typed_geometry_complete());
    EXPECT_EQ(result.value().geometry()[0].sense, '-');
  }
}
}  // namespace
}  // namespace openparasolid
