// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/topology.h"

namespace openparasolid {
namespace {
using namespace test;
ModelFixture Fixture(uint32_t count = 3, uint32_t dummy_mask = 0) {
  ModelFixture f;
  f.Add(16, 1, {Pointer("halfedge", 10, 17), Pointer("curve", 50, 1008)});
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t id = 10 + i;
    const bool dummy = (dummy_mask & (1u << i)) != 0;
    f.Add(17, id,
          {Pointer("loop", dummy ? 0 : 20 + i, 15),
           Pointer("forward", dummy ? 0 : id, 17),
           Pointer("backward", dummy ? 0 : id, 17), Pointer("vertex", 0, 18),
           Pointer("other", 10 + (i + 1) % count, 17), Pointer("edge", 1, 16),
           Pointer("curve", 0, 1008), Pointer("next_at_vx", 0, 17),
           ByteValue("sense", i == 1 ? '-' : '+', FieldType::kCharacter)});
    if (!dummy) f.Add(15, 20 + i, {});
  }
  f.Add(30, 50, {});  // Untyped geometry does not block topology.
  f.Add(18, 100, {});
  f.Add(18, 101, {});
  return f;
}
void Set(ModelFixture& f, uint32_t id, const char* role, uint32_t value) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < fields.size(); ++i)
      if (fields[i].name == role) {
        node.fields[i].values[0] = uint64_t{value};
        return;
      }
  }
  ADD_FAILURE() << "Missing test field";
}
TEST(ParasolidEdgeTopology, OwnsFullRadialOrderIncludingRepeatedSenses) {
  const auto result = [] {
    const auto f = Fixture();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<EdgeBoundary>::Err(model.error());
    return TraverseEdgeBoundary(model.value(), NodeId{1}, {3});
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto copy = result.value();
  auto moved = std::move(copy);
  EXPECT_EQ(moved.edge, NodeId{1});
  EXPECT_EQ(moved.curve, NodeId{50});
  EXPECT_EQ(moved.start_vertex, NodeId{0});
  EXPECT_EQ(moved.end_vertex, NodeId{0});
  ASSERT_EQ(moved.fins.size(), 3);
  for (uint32_t i = 0; i < 3; ++i) {
    EXPECT_EQ(moved.fins[i].use.fin, NodeId{10 + i});
    EXPECT_EQ(moved.fins[i].use.edge, NodeId{1});
    EXPECT_EQ(moved.fins[i].loop, NodeId{20 + i});
  }
  EXPECT_EQ(moved.fins[0].use.sense, TopologySense::kForward);
  EXPECT_EQ(moved.fins[1].use.sense, TopologySense::kReverse);
  EXPECT_EQ(moved.fins[2].use.sense, TopologySense::kForward);
}
TEST(ParasolidEdgeTopology, OrientedVerticesAndNullableCurvesRemainExplicit) {
  auto f = Fixture();
  Set(f, 10, "vertex", 101);
  Set(f, 11, "vertex", 100);
  Set(f, 12, "vertex", 101);
  Set(f, 1, "curve", 0);
  f.Add(31, 51, {});
  Set(f, 12, "curve", 51);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseEdgeBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().start_vertex, NodeId{100});
  EXPECT_EQ(result.value().end_vertex, NodeId{101});
  EXPECT_EQ(result.value().curve, NodeId{0});
  EXPECT_EQ(result.value().fins[0].use.start_vertex, NodeId{100});
  EXPECT_EQ(result.value().fins[1].use.start_vertex, NodeId{101});
  EXPECT_EQ(result.value().fins[1].use.end_vertex, NodeId{100});
  EXPECT_EQ(result.value().fins[2].use.fin_curve, NodeId{51});
}
TEST(ParasolidEdgeTopology, DummyAnchorsSupportSheetAndWireRings) {
  for (uint32_t mask : {1, 2, 3}) {
    const auto f = Fixture(2, mask);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseEdgeBoundary(model.value(), NodeId{1}, {2});
    ASSERT_TRUE(result.ok()) << result.error().message;
    ASSERT_EQ(result.value().fins.size(), 2);
    for (uint32_t i = 0; i < 2; ++i)
      EXPECT_EQ(result.value().fins[i].loop.value == 0,
                (mask & (1u << i)) != 0);
  }
}
TEST(ParasolidEdgeTopology, WriterDummyVertexLinksCanJoinTheTwoAnchors) {
  auto f = Fixture(2, 3);
  Set(f, 10, "vertex", 100);
  Set(f, 11, "vertex", 100);
  Set(f, 10, "next_at_vx", 11);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseEdgeBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().start_vertex, NodeId{100});
  EXPECT_EQ(result.value().end_vertex, NodeId{100});
}
TEST(ParasolidEdgeTopology, LimitsIncludeEveryDummyAndPrecedeTargetLookup) {
  for (uint32_t mask : {0, 3}) {
    const auto f = Fixture(2, mask);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto limited = TraverseEdgeBoundary(model.value(), NodeId{1}, {1});
    ASSERT_FALSE(limited.ok());
    EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
    const auto zero = TraverseEdgeBoundary(model.value(), NodeId{999}, {0});
    ASSERT_FALSE(zero.ok());
    EXPECT_EQ(zero.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidEdgeTopology, RadialCyclesAndWrongEdgeOwnersAreRejected) {
  for (int bad = 0; bad < 7; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 12, "other", 11);
    if (bad == 1) Set(f, 10, "other", 10);
    if (bad == 2) Set(f, 12, "other", 0);
    if (bad == 3) Set(f, 12, "other", 999);
    if (bad == 4) {
      Set(f, 12, "edge", 2);
      f.Add(16, 2, {});
    }
    if (bad == 5) Set(f, 12, "other", 50);
    if (bad == 6) Set(f, 1, "halfedge", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseEdgeBoundary(model.value(), NodeId{1}, {3});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
    if (bad == 0)
      EXPECT_NE(result.error().message.find("cycle"), std::string::npos);
  }
}
TEST(ParasolidEdgeTopology, AnchorsEndpointsAndLocalLoopLinksAreChecked) {
  for (int bad = 0; bad < 8; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 10, "sense", '-');
    if (bad == 1) Set(f, 11, "sense", '+');
    if (bad == 2) Set(f, 10, "vertex", 101);
    if (bad == 3) {
      Set(f, 10, "vertex", 101);
      Set(f, 11, "vertex", 100);
      Set(f, 12, "vertex", 100);
    }
    if (bad == 4) Set(f, 10, "forward", 11);
    if (bad == 5) Set(f, 10, "backward", 12);
    if (bad == 6) Set(f, 10, "forward", 0);
    if (bad == 7) {
      Set(f, 10, "forward", 11);
      Set(f, 11, "loop", 20);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseEdgeBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidEdgeTopology, DummyShapeAndNonAnchorDummiesAreRejected) {
  for (int bad = 0; bad < 5; ++bad) {
    auto f = Fixture(3, 1);
    if (bad == 0) Set(f, 10, "forward", 10);
    if (bad == 1) Set(f, 10, "backward", 10);
    if (bad == 2) Set(f, 10, "curve", 50);
    if (bad == 3) Set(f, 10, "next_at_vx", 10);
    if (bad == 4) f = Fixture(3, 4);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseEdgeBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidEdgeTopology,
     MissingRolesUnverifiedClassesAndUnknownSensesStayUnsupported) {
  for (int bad = 0; bad < 6; ++bad) {
    auto f = Fixture();
    if (bad == 5)
      Set(f, 12, "sense", '?');
    else {
      const uint32_t id = bad == 0 ? 1 : 10;
      const std::string role = bad == 0   ? "halfedge"
                               : bad == 1 ? "other"
                               : bad == 2 ? "forward"
                               : bad == 3 ? "sense"
                                          : "loop";
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
    const auto result = TraverseEdgeBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat) << bad;
  }
}
TEST(ParasolidEdgeTopology,
     UnlistedFinsAndUnrelatedModelIssuesDoNotBlockQuery) {
  auto f = Fixture();
  f.Add(17, 999, {Pointer("edge", 1, 16), Pointer("loop", 888, 15)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().references_verified());
  EXPECT_TRUE(TraverseEdgeBoundary(model.value(), NodeId{1}).ok());
  EXPECT_FALSE(TraverseEdgeBoundary(model.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseEdgeBoundary(model.value(), NodeId{888}).ok());
}
}  // namespace
}  // namespace openparasolid
