// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/topology.h"

namespace openparasolid {
namespace {
using namespace test;
void Set(ModelFixture& f, uint32_t id, const char* role, uint32_t value) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    const auto& defs = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < defs.size(); ++i)
      if (defs[i].name == role) {
        node.fields[i].values[0] = uint64_t{value};
        return;
      }
  }
  ADD_FAILURE() << "Missing test field";
}
void Vertex(ModelFixture& f, uint32_t id, uint32_t head, uint32_t owner,
            uint32_t point) {
  f.Add(18, id,
        {Pointer("owner", owner, 1029), Pointer("point", point, 29),
         Pointer("halfedge", head, 17)});
}
void Fin(ModelFixture& f, uint32_t id, uint32_t edge, uint32_t other,
         uint32_t vertex, uint32_t loop, uint32_t next, char sense) {
  f.Add(17, id,
        {Pointer("edge", edge, 16), Pointer("other", other, 17),
         Pointer("vertex", vertex, 18), Pointer("loop", loop, 15),
         Pointer("forward", loop ? id : 0, 17),
         Pointer("backward", loop ? id : 0, 17), Pointer("curve", 0, 1008),
         Pointer("next_at_vx", next, 17),
         ByteValue("sense", sense, FieldType::kCharacter)});
}
ModelFixture Fixture() {
  ModelFixture f;
  f.Add(12, 1, {});
  Vertex(f, 100, 10, 1, 200);
  Vertex(f, 101, 11, 1, 201);
  f.Point(200);
  f.Point(201);
  f.Add(16, 2, {Pointer("halfedge", 10, 17), Pointer("curve", 0, 1008)});
  f.Add(16, 3, {Pointer("halfedge", 12, 17), Pointer("curve", 0, 1008)});
  Fin(f, 10, 2, 11, 100, 20, 13, '+');
  Fin(f, 11, 2, 10, 101, 0, 12, '-');
  Fin(f, 12, 3, 13, 101, 0, 0, '+');
  Fin(f, 13, 3, 12, 100, 0, 30, '-');
  Fin(f, 30, 0, 0, 100, 40, 0, '?');
  f.Add(15, 20, {Pointer("halfedge", 10, 17)});
  f.Add(15, 40, {Pointer("halfedge", 30, 17)});
  return f;
}
TEST(ParasolidVertexTopology,
     OwnsOrderedAnchorsAndIsolatedFinsAfterSourceDestruction) {
  const auto result = [] {
    const auto f = Fixture();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<VertexIncidence>::Err(model.error());
    return TraverseVertexIncidence(model.value(), NodeId{100}, {3});
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto copy = result.value();
  auto moved = std::move(copy);
  EXPECT_EQ(moved.vertex, NodeId{100});
  EXPECT_EQ(moved.owner, NodeId{1});
  EXPECT_EQ(moved.point, NodeId{200});
  ASSERT_EQ(moved.fins.size(), 3);
  EXPECT_EQ(moved.fins[0].fin, NodeId{10});
  EXPECT_EQ(moved.fins[0].loop, NodeId{20});
  ASSERT_TRUE(moved.fins[0].edge_use);
  EXPECT_EQ(moved.fins[0].edge_use->edge, NodeId{2});
  EXPECT_EQ(moved.fins[0].edge_use->start_vertex, NodeId{101});
  EXPECT_EQ(moved.fins[0].edge_use->end_vertex, NodeId{100});
  EXPECT_EQ(moved.fins[1].fin, NodeId{13});
  EXPECT_EQ(moved.fins[1].loop, NodeId{0});
  ASSERT_TRUE(moved.fins[1].edge_use);
  EXPECT_EQ(moved.fins[1].edge_use->sense, TopologySense::kReverse);
  EXPECT_EQ(moved.fins[2].fin, NodeId{30});
  EXPECT_EQ(moved.fins[2].loop, NodeId{40});
  EXPECT_FALSE(moved.fins[2].edge_use);
}
TEST(ParasolidVertexTopology, DummyVertexLinksMayLeadToIsolatedLoopFins) {
  auto f = Fixture();
  auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(TraverseEdgeBoundary(model.value(), NodeId{3}).ok());
  Set(f, 30, "forward", 10);
  model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto invalid = TraverseEdgeBoundary(model.value(), NodeId{3});
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidVertexTopology, ClosedEdgeMayListBothAnchorsAtOneVertex) {
  auto f = Fixture();
  Set(f, 12, "vertex", 100);
  Set(f, 10, "next_at_vx", 12);
  Set(f, 12, "next_at_vx", 13);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseVertexIncidence(model.value(), NodeId{100}, {4});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().fins.size(), 4);
  EXPECT_EQ(result.value().fins[1].edge_use->edge, NodeId{3});
  EXPECT_EQ(result.value().fins[2].edge_use->edge, NodeId{3});
}
TEST(ParasolidVertexTopology, AcornAndBodyOwnedNullHeadsRemainExplicit) {
  for (bool acorn : {false, true}) {
    ModelFixture f;
    f.Add(acorn ? 13 : 12, 1, {});
    Vertex(f, 100, 0, 1, 200);
    f.Point(200);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        TraverseVertexIncidence(model.value(), NodeId{100}, {1});
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().fins.empty());
    EXPECT_EQ(result.value().owner, NodeId{1});
  }
}
TEST(ParasolidVertexTopology, AllEntriesCountAndCyclesFailAtExactLimits) {
  auto f = Fixture();
  auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto limit = TraverseVertexIncidence(model.value(), NodeId{100}, {2});
  ASSERT_FALSE(limit.ok());
  EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
  const auto zero = TraverseVertexIncidence(model.value(), NodeId{999}, {0});
  ASSERT_FALSE(zero.ok());
  EXPECT_EQ(zero.error().code, ErrorCode::kInvalidOptions);
  Set(f, 30, "next_at_vx", 10);
  model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto cycle = TraverseVertexIncidence(model.value(), NodeId{100}, {3});
  ASSERT_FALSE(cycle.ok());
  EXPECT_EQ(cycle.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(cycle.error().message.find("cycle"), std::string::npos);
}
TEST(ParasolidVertexTopology, MalformedOwnersPointsHeadsAndListedVerticesFail) {
  for (int bad = 0; bad < 9; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 100, "owner", 0);
    if (bad == 1) Set(f, 100, "owner", 200);
    if (bad == 2) Set(f, 100, "point", 0);
    if (bad == 3) Set(f, 100, "point", 999);
    if (bad == 4) Set(f, 100, "point", 10);
    if (bad == 5) Set(f, 100, "halfedge", 200);
    if (bad == 6) Set(f, 13, "vertex", 101);
    if (bad == 7) Set(f, 30, "next_at_vx", 999);
    if (bad == 8) {
      f.Add(13, 5, {});
      Set(f, 100, "owner", 5);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseVertexIncidence(model.value(), NodeId{100});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidVertexTopology, NonAnchorFinsAndBrokenIsolatedShapesAreRejected) {
  for (int bad = 0; bad < 8; ++bad) {
    auto f = Fixture();
    if (bad == 0) {
      Fin(f, 14, 2, 10, 100, 20, 0, '+');
      Set(f, 100, "halfedge", 14);
    }
    if (bad == 1) Set(f, 30, "other", 10);
    if (bad == 2) Set(f, 30, "curve", 50);
    if (bad == 2) f.Add(30, 50, {});
    if (bad == 3) Set(f, 30, "loop", 0);
    if (bad == 4) Set(f, 30, "forward", 10);
    if (bad == 5) Set(f, 40, "halfedge", 10);
    if (bad == 6) Set(f, 10, "sense", '-');
    if (bad == 7) Set(f, 13, "forward", 13);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseVertexIncidence(model.value(), NodeId{100});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidVertexTopology,
     MissingRolesUnverifiedClassesAndAnchorSensesStayUnsupported) {
  for (int bad = 0; bad < 6; ++bad) {
    auto f = Fixture();
    if (bad == 5)
      Set(f, 10, "sense", '?');
    else {
      const uint32_t id = bad < 3 ? 100 : 10;
      const std::string role = bad == 0   ? "owner"
                               : bad == 1 ? "point"
                               : bad == 2 ? "halfedge"
                               : bad == 3 ? "next_at_vx"
                                          : "edge";
      for (auto& node : f.report.nodes)
        if (node.index == id)
          for (auto& field :
               f.report.schemas[node.schema_index].definition.fields)
            if (field.name == role) {
              if (bad == 4)
                field.pointer_class = 60000;
              else
                field.name = "unavailable";
            }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseVertexIncidence(model.value(), NodeId{100});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat) << bad;
  }
}
TEST(ParasolidVertexTopology,
     UnlistedFinsAndUnrelatedFindingsDoNotBlockTheLocalQuery) {
  auto f = Fixture();
  f.Add(17, 999, {Pointer("vertex", 100, 18), Pointer("edge", 888, 16)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().references_verified());
  EXPECT_TRUE(TraverseVertexIncidence(model.value(), NodeId{100}).ok());
  EXPECT_FALSE(TraverseVertexIncidence(model.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseVertexIncidence(model.value(), NodeId{777}).ok());
}
TEST(ParasolidVertexTopology,
     IsolatedFinsDoNotRequireSenseOrTypedPointGeometry) {
  ModelFixture f;
  f.Add(12, 1, {});
  Vertex(f, 100, 30, 1, 200);
  f.Add(29, 200, {});  // Missing point coordinates remain a model finding.
  Fin(f, 30, 0, 0, 100, 40, 0, '?');
  f.Add(15, 40, {Pointer("halfedge", 30, 17)});
  for (auto& node : f.report.nodes)
    if (node.index == 30)
      for (auto& field : f.report.schemas[node.schema_index].definition.fields)
        if (field.name == "sense") field.name = "unavailable";
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().typed_geometry_complete());
  const auto result = TraverseVertexIncidence(model.value(), NodeId{100});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().point, NodeId{200});
  ASSERT_EQ(result.value().fins.size(), 1);
  EXPECT_FALSE(result.value().fins[0].edge_use);
}

}  // namespace
}  // namespace openparasolid
