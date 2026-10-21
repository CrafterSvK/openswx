// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openparasolid/topology.h"

#include <gtest/gtest.h>

#include <utility>

#include "model_test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
void Fin(ModelFixture& f, uint32_t id, uint32_t loop, uint32_t forward,
         uint32_t backward, uint32_t vertex, uint32_t other, uint32_t edge,
         char sense, uint32_t curve = 0) {
  f.Add(
      17, id,
      {Pointer("loop", loop, 15), Pointer("forward", forward, 17),
       Pointer("backward", backward, 17), Pointer("vertex", vertex, 18),
       Pointer("other", other, 17), Pointer("edge", edge, 16),
       Pointer("curve", curve, 1008), Pointer("next_at_vx", 0, 17),
       ByteValue("sense", static_cast<uint8_t>(sense), FieldType::kCharacter)});
}
void Edge(ModelFixture& f, uint32_t id, uint32_t fin, uint32_t curve = 0) {
  f.Add(16, id, {Pointer("halfedge", fin, 17), Pointer("curve", curve, 1008)});
}
void LoopAndFace(ModelFixture& f, uint32_t fin = 20) {
  f.Add(15, 1, {Pointer("halfedge", fin, 17), Pointer("face", 2, 14)});
  f.Add(14, 2,
        {Pointer("surface", 0, 1006),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
ModelFixture Triangle() {
  ModelFixture f;
  LoopAndFace(f);
  Fin(f, 20, 1, 21, 22, 101, 30, 40, '+');
  Fin(f, 21, 1, 22, 20, 102, 31, 41, '-');
  Fin(f, 22, 1, 20, 21, 100, 32, 42, '+');
  Fin(f, 30, 0, 0, 0, 100, 20, 40, '-');
  Fin(f, 31, 0, 0, 0, 101, 21, 41, '+');
  Fin(f, 32, 0, 0, 0, 102, 22, 42, '-');
  Edge(f, 40, 20);
  Edge(f, 41, 31);
  Edge(f, 42, 22);
  for (uint32_t id : {100u, 101u, 102u}) f.Add(18, id, {});
  return f;
}
DecodedField& Field(ModelFixture& f, uint32_t id, const char* name) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    const auto& definitions =
        f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < definitions.size(); ++i)
      if (definitions[i].name == name) return node.fields[i];
  }
  ADD_FAILURE() << "Missing test field";
  return f.report.nodes.front().fields.front();
}
void Set(ModelFixture& f, uint32_t id, const char* field, uint32_t value) {
  Field(f, id, field).values[0] = uint64_t{value};
}
void Rename(ModelFixture& f, uint32_t id, const char* field, const char* name) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    for (auto& definition :
         f.report.schemas[node.schema_index].definition.fields)
      if (definition.name == field) {
        definition.name = name;
        return;
      }
  }
  ADD_FAILURE() << "Missing test field definition";
}
void Rebuild(ModelFixture& f) {
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
}
void ChangeDefinition(ModelFixture& f, uint32_t id, const char* field,
                      FieldType type, uint32_t elements = 0) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    for (auto& definition :
         f.report.schemas[node.schema_index].definition.fields) {
      if (definition.name != field) continue;
      definition.type = type;
      definition.elements = elements;
      Rebuild(f);
      return;
    }
  }
  ADD_FAILURE() << "Missing test field definition";
}
TEST(ParasolidTopology, TriangleUsesLoopOrderAndBothEdgeDirections) {
  auto fixture = Triangle();
  // A rotated head is retained, not sorted by IDs or fin sense.
  Set(fixture, 1, "halfedge", 21);
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto boundary = TraverseLoopBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(boundary.ok()) << boundary.error().message;
  const auto& result = boundary.value();
  ASSERT_EQ(result.fins.size(), 3);
  EXPECT_EQ(result.face, NodeId{2});
  EXPECT_EQ(result.surface, NodeId{0});
  EXPECT_EQ(result.face_sense, '-');
  EXPECT_FALSE(result.isolated_vertex);
  EXPECT_EQ(result.fins[0].fin, NodeId{21});
  EXPECT_EQ(result.fins[0].edge, NodeId{41});
  EXPECT_EQ(result.fins[0].start_vertex, NodeId{101});
  EXPECT_EQ(result.fins[0].end_vertex, NodeId{102});
  EXPECT_EQ(result.fins[0].sense, TopologySense::kReverse);
  EXPECT_EQ(result.fins[1].fin, NodeId{22});
  EXPECT_EQ(result.fins[2].fin, NodeId{20});
  EXPECT_EQ(result.fins[2].sense, TopologySense::kForward);
}
TEST(ParasolidTopology, RetainsSurfaceAndSeparateCurveRolesWithoutEvaluation) {
  auto f = Triangle();
  Set(f, 2, "surface", 200);
  Set(f, 40, "curve", 300);
  Set(f, 20, "curve", 301);
  f.Add(124, 200, {});  // Geometry may be untyped without blocking topology.
  f.Add(38, 300, {});
  f.Add(133, 301, {});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().typed_geometry_complete());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().surface, NodeId{200});
  EXPECT_EQ(result.value().fins[0].edge_curve, NodeId{300});
  EXPECT_EQ(result.value().fins[0].fin_curve, NodeId{301});
}
TEST(ParasolidTopology, VertexFreeRingRetainsItsSingleEdgeUse) {
  ModelFixture f;
  LoopAndFace(f);
  Fin(f, 20, 1, 20, 20, 0, 30, 40, '+');
  Fin(f, 30, 0, 0, 0, 0, 20, 40, '-');
  Edge(f, 40, 20);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1}, {1});
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().fins.size(), 1);
  EXPECT_EQ(result.value().fins[0].start_vertex, NodeId{0});
  EXPECT_EQ(result.value().fins[0].end_vertex, NodeId{0});
}
TEST(ParasolidTopology, IsolatedVertexIgnoresItsInsignificantFinSense) {
  ModelFixture f;
  LoopAndFace(f);
  Fin(f, 20, 1, 20, 20, 100, 0, 0, '?');
  f.Add(18, 100, {});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1}, {1});
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().isolated_vertex);
  EXPECT_EQ(result.value().isolated_vertex->fin, NodeId{20});
  EXPECT_EQ(result.value().isolated_vertex->vertex, NodeId{100});
  EXPECT_TRUE(result.value().fins.empty());
}
TEST(ParasolidTopology, SameEdgeMayAppearOnceInEachDirection) {
  ModelFixture f;
  LoopAndFace(f);
  Fin(f, 20, 1, 30, 30, 101, 30, 40, '+');
  Fin(f, 30, 1, 20, 20, 100, 20, 40, '-');
  Edge(f, 40, 20);
  f.Add(18, 100, {});
  f.Add(18, 101, {});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().fins.size(), 2);
  EXPECT_EQ(result.value().fins[0].edge, result.value().fins[1].edge);
  EXPECT_NE(result.value().fins[0].sense, result.value().fins[1].sense);
}
TEST(ParasolidTopology, RejectsBrokenLinksRolesAndVertexContinuity) {
  for (int bad = 0; bad < 18; ++bad) {
    auto f = Triangle();
    switch (bad) {
      case 0:
        Set(f, 20, "loop", 0);
        break;
      case 1:
        Set(f, 20, "forward", 0);
        break;
      case 2:
        Set(f, 21, "backward", 22);
        break;
      case 3:
        Set(f, 20, "backward", 21);
        break;
      case 4:
        Set(f, 20, "forward", 999);
        break;
      case 5:
        Set(f, 20, "forward", 40);
        break;
      case 6:
        Set(f, 30, "vertex", 0);
        break;
      case 7:
        Set(f, 20, "vertex", 102);
        break;
      case 8:
        Set(f, 30, "edge", 41);
        break;
      case 9:
        Set(f, 40, "halfedge", 30);
        break;
      case 10:
        Set(f, 20, "other", 20);
        break;
      case 11:
        Set(f, 21, "forward", 21);
        break;
      case 12:
        Set(f, 20, "edge", 0);
        break;
      case 13:
        Set(f, 1, "halfedge", 0);
        break;
      case 14:
        Set(f, 1, "face", 0);
        break;
      case 15:
        Set(f, 30, "vertex", 102);
        break;
      case 16:
        Field(f, 20, "edge").values.push_back(uint64_t{40});
        ChangeDefinition(f, 20, "edge", FieldType::kPointer, 2);
        break;
      case 17:
        Set(f, 21, "other", 32);
        break;
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok()) << bad;
    const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidTopology, UnknownAndMissingRolesRemainExplicit) {
  for (int bad = 0; bad < 4; ++bad) {
    auto f = Triangle();
    if (bad == 0) Set(f, 20, "sense", '?');
    if (bad == 1) Rename(f, 20, "sense", "unavailable");
    if (bad == 2) ChangeDefinition(f, 20, "sense", FieldType::kByte);
    if (bad == 3) Rename(f, 20, "curve", "unavailable");
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat) << bad;
  }
}
TEST(ParasolidTopology, RepeatedDirectionAndMultiFinRingAreRejected) {
  for (bool ring : {false, true}) {
    ModelFixture f;
    LoopAndFace(f);
    const uint32_t vertex = ring ? 0 : 100;
    Fin(f, 20, 1, 21, 21, vertex, 30, 40, '+');
    Fin(f, 21, 1, 20, 20, vertex, 20, 40, '+');
    Fin(f, 30, 0, 0, 0, vertex, 21, 40, '-');
    Edge(f, 40, 20);
    if (vertex) f.Add(18, vertex, {});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidTopology, IsolatedFinRequiresOnlyItsSelfLinksAndVertex) {
  for (int bad = 0; bad < 3; ++bad) {
    ModelFixture f;
    LoopAndFace(f);
    Fin(f, 20, 1, 20, 20, bad == 0 ? 0 : 100, bad == 1 ? 20 : 0, 0, '?',
        bad == 2 ? 300 : 0);
    f.Add(18, 100, {});
    f.Add(30, 300, {});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidTopology, FaceSenseMetadataIsNeverInvented) {
  for (int kind = 0; kind < 4; ++kind) {
    auto f = Triangle();
    if (kind == 0) Set(f, 2, "sense", '?');
    if (kind == 1) Rename(f, 2, "sense", "unavailable");
    if (kind == 2) ChangeDefinition(f, 2, "sense", FieldType::kByte);
    if (kind == 3) {
      Field(f, 2, "sense").values.push_back(uint64_t{'+'});
      ChangeDefinition(f, 2, "sense", FieldType::kCharacter, 2);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
    ASSERT_TRUE(result.ok());
    if (kind == 0)
      EXPECT_EQ(result.value().face_sense, '?');
    else
      EXPECT_FALSE(result.value().face_sense);
  }
}
TEST(ParasolidTopology, UnverifiedRoleClassesAreNotGuessedFromTargetType) {
  auto f = Triangle();
  for (auto& node : f.report.nodes) {
    if (node.index != 20) continue;
    for (auto& definition :
         f.report.schemas[node.schema_index].definition.fields)
      if (definition.name == "forward") definition.pointer_class = 60000;
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().references_verified());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidTopology, NonPrimaryFinUsesTheDocumentedEdgeAnchors) {
  ModelFixture f;
  LoopAndFace(f, 21);
  Fin(f, 20, 3, 20, 20, 0, 30, 40, '+');
  Fin(f, 30, 4, 30, 30, 0, 21, 40, '-');
  Fin(f, 21, 1, 21, 21, 0, 20, 40, '+');
  Edge(f, 40, 20);
  f.Add(15, 3, {});
  f.Add(15, 4, {});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseLoopBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().fins.size(), 1);
  EXPECT_EQ(result.value().fins[0].fin, NodeId{21});
  EXPECT_EQ(result.value().fins[0].sense, TopologySense::kForward);
}
TEST(ParasolidTopology, UnrelatedReferenceIssuesDoNotBlockLocalTraversal) {
  auto f = Triangle();
  f.Add(74, 999, {Pointer("unrelated", 777, 29)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().references_verified());
  EXPECT_TRUE(TraverseLoopBoundary(model.value(), NodeId{1}).ok());
}
TEST(ParasolidTopology, LimitsAreCheckedBeforeReturningAnyBoundary) {
  auto f = Triangle();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto zero = TraverseLoopBoundary(model.value(), NodeId{999}, {0});
  ASSERT_FALSE(zero.ok());
  EXPECT_EQ(zero.error().code, ErrorCode::kInvalidOptions);
  const auto small = TraverseLoopBoundary(model.value(), NodeId{1}, {2});
  ASSERT_FALSE(small.ok());
  EXPECT_EQ(small.error().code, ErrorCode::kResourceLimit);
  EXPECT_TRUE(TraverseLoopBoundary(model.value(), NodeId{1}, {3}).ok());
  EXPECT_FALSE(TraverseLoopBoundary(model.value(), NodeId{999}).ok());
  EXPECT_FALSE(TraverseLoopBoundary(model.value(), NodeId{40}).ok());
}
TEST(ParasolidTopology, ModelAndBoundaryOwnTheirDataAcrossCopiesAndMoves) {
  auto build = [] {
    auto f = Triangle();
    return ModelBuilder::Build(f.report);
  };
  auto built = build();
  ASSERT_TRUE(built.ok());
  auto model = built.value();
  Model moved = std::move(model);
  auto boundary = TraverseLoopBoundary(moved, NodeId{1});
  ASSERT_TRUE(boundary.ok());
  moved = Model{};
  EXPECT_EQ(boundary.value().fins[0].end_vertex, NodeId{101});
  EXPECT_EQ(built.value().FindNode(NodeId{20})->topology_sense, '+');
  EXPECT_EQ(built.value().FindNode(NodeId{2})->topology_sense, '-');
}
}  // namespace
}  // namespace openparasolid
