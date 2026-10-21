// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <utility>

#include "model_test_helpers.h"
#include "openparasolid/topology.h"

namespace openparasolid {
namespace {
using namespace test;
void Shell(ModelFixture& f, uint32_t back = 10, uint32_t front = 10,
           uint32_t edge = 20, uint32_t vertex = 0) {
  f.Add(13, 1,
        {Pointer("region", 2, 19), Pointer("face", back, 14),
         Pointer("front_face", front, 14), Pointer("edge", edge, 16),
         Pointer("vertex", vertex, 18), Pointer("body", 0, 12)});
  f.Add(19, 2, {});
}
void Face(ModelFixture& f, uint32_t id, uint32_t loop, uint32_t previous,
          uint32_t next) {
  f.Add(14, id,
        {Pointer("surface", 0, 1006), Pointer("loop", loop, 15),
         Pointer("shell", 1, 13), Pointer("previous", previous, 14),
         Pointer("next", next, 14), Pointer("front_shell", 1, 13),
         Pointer("previous_front", 0, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '-', FieldType::kCharacter)});
}
void Fin(ModelFixture& f, uint32_t id, uint32_t vertex, uint32_t other,
         uint32_t edge, char sense) {
  f.Add(
      17, id,
      {Pointer("loop", 0, 15), Pointer("forward", 0, 17),
       Pointer("backward", 0, 17), Pointer("vertex", vertex, 18),
       Pointer("other", other, 17), Pointer("edge", edge, 16),
       Pointer("curve", 0, 1008), Pointer("next_at_vx", 0, 17),
       ByteValue("sense", static_cast<uint8_t>(sense), FieldType::kCharacter)});
}
void IsolatedLoop(ModelFixture& f, uint32_t loop, uint32_t fin, uint32_t vertex,
                  uint32_t face) {
  f.Add(15, loop,
        {Pointer("face", face, 14), Pointer("halfedge", fin, 17),
         Pointer("next", 0, 15)});
  f.Add(17, fin,
        {Pointer("loop", loop, 15), Pointer("forward", fin, 17),
         Pointer("backward", fin, 17), Pointer("vertex", vertex, 18),
         Pointer("other", 0, 17), Pointer("edge", 0, 16),
         Pointer("curve", 0, 1008),
         ByteValue("sense", '?', FieldType::kCharacter)});
  f.Add(18, vertex, {});
}
ModelFixture Fixture() {
  ModelFixture f;
  Shell(f);
  Face(f, 10, 11, 0, 14);
  Face(f, 14, 0, 10, 0);
  IsolatedLoop(f, 11, 12, 13, 10);
  f.Add(16, 20,
        {Pointer("owner", 1, 1029), Pointer("previous", 0, 16),
         Pointer("next", 21, 16), Pointer("halfedge", 30, 17),
         Pointer("curve", 50, 1008)});
  f.Add(16, 21,
        {Pointer("owner", 1, 1029), Pointer("previous", 20, 16),
         Pointer("next", 0, 16), Pointer("halfedge", 32, 17),
         Pointer("curve", 0, 1008)});
  Fin(f, 30, 41, 31, 20, '+');
  Fin(f, 31, 40, 30, 20, '-');
  Fin(f, 32, 0, 33, 21, '+');
  Fin(f, 33, 0, 32, 21, '-');
  f.Add(18, 40, {});
  f.Add(18, 41, {});
  f.Add(30, 50, {});  // Untyped geometry does not block topology.
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
void Rename(ModelFixture& f, uint32_t id, const char* role) {
  for (auto& node : f.report.nodes) {
    if (node.index != id) continue;
    for (auto& field : f.report.schemas[node.schema_index].definition.fields)
      if (field.name == role) {
        field.name = "unavailable";
        return;
      }
  }
  ADD_FAILURE() << "Missing test role";
}
ModelFixture Acorn() {
  ModelFixture f;
  Shell(f, 0, 0, 0, 40);
  f.Add(18, 40,
        {Pointer("owner", 1, 1029), Pointer("halfedge", 0, 17),
         Pointer("point", 50, 29)});
  f.Point(50);
  return f;
}
TEST(ParasolidShellTopology, SharesOneBoundaryForFrontAndBackUses) {
  const auto f = Fixture();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      TraverseShellBoundary(model.value(), NodeId{1}, {3, 1, 1, 2});
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& shell = result.value();
  EXPECT_EQ(shell.shell, NodeId{1});
  EXPECT_EQ(shell.region, NodeId{2});
  ASSERT_EQ(shell.faces.size(), 2);
  ASSERT_EQ(shell.face_uses.size(), 3);
  EXPECT_EQ(shell.faces[0].face, NodeId{10});
  EXPECT_EQ(shell.faces[1].face, NodeId{14});
  EXPECT_EQ(shell.faces[0].face_sense, '-');
  EXPECT_EQ(shell.face_uses[0].boundary, 0);
  EXPECT_EQ(shell.face_uses[1].boundary, 1);
  EXPECT_EQ(shell.face_uses[2].boundary, 0);
  EXPECT_EQ(shell.face_uses[0].side, ShellFaceSide::kBack);
  EXPECT_EQ(shell.face_uses[1].side, ShellFaceSide::kBack);
  EXPECT_EQ(shell.face_uses[2].side, ShellFaceSide::kFront);
  ASSERT_TRUE(shell.faces[0].loops[0].isolated_vertex);
  EXPECT_TRUE(
      shell.faces[1]
          .loops.empty());  // Accepted after loop/fin budget is exhausted.
  ASSERT_EQ(shell.wire_edges.size(), 2);
  EXPECT_EQ(shell.wire_edges[0].edge, NodeId{20});
  EXPECT_EQ(shell.wire_edges[0].start_vertex, NodeId{40});
  EXPECT_EQ(shell.wire_edges[0].end_vertex, NodeId{41});
  EXPECT_EQ(shell.wire_edges[0].curve, NodeId{50});
  EXPECT_EQ(shell.wire_edges[1].start_vertex, NodeId{0});
  EXPECT_EQ(shell.wire_edges[1].end_vertex, NodeId{0});
  EXPECT_FALSE(shell.isolated_vertex);
}
TEST(ParasolidShellTopology, FrontOnlyAndWireOnlyShellsKeepTheirOwnRoles) {
  for (bool front : {false, true}) {
    auto f = Fixture();
    Set(f, 1, "face", 0);
    Set(f, 1, front ? "edge" : "front_face", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_TRUE(result.ok());
    if (front) {
      ASSERT_EQ(result.value().face_uses.size(), 1);
      EXPECT_EQ(result.value().face_uses[0].side, ShellFaceSide::kFront);
      EXPECT_TRUE(result.value().wire_edges.empty());
    } else {
      EXPECT_TRUE(result.value().faces.empty());
      EXPECT_EQ(result.value().wire_edges.size(), 2);
    }
  }
}
TEST(ParasolidShellTopology, AcornVertexRetainsItsPointReference) {
  const auto f = Acorn();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      TraverseShellBoundary(model.value(), NodeId{1}, {1, 1, 1, 1});
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().isolated_vertex);
  EXPECT_EQ(result.value().isolated_vertex->vertex, NodeId{40});
  EXPECT_EQ(result.value().isolated_vertex->point, NodeId{50});
  EXPECT_TRUE(result.value().faces.empty());
  EXPECT_TRUE(result.value().wire_edges.empty());
}
TEST(ParasolidShellTopology, AggregateBudgetsCountDistinctFaceDataAndEveryUse) {
  auto f = Fixture();
  Set(f, 14, "loop", 60);
  IsolatedLoop(f, 60, 61, 62, 14);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(
      TraverseShellBoundary(model.value(), NodeId{1}, {3, 2, 2, 2}).ok());
  for (auto options :
       {ShellTraversalOptions{2, 2, 2, 2}, ShellTraversalOptions{3, 1, 2, 2},
        ShellTraversalOptions{3, 2, 1, 2}, ShellTraversalOptions{3, 2, 2, 1}}) {
    const auto result =
        TraverseShellBoundary(model.value(), NodeId{1}, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
  for (int bad = 0; bad < 4; ++bad) {
    ShellTraversalOptions options;
    if (bad == 0) options.max_face_uses = 0;
    if (bad == 1) options.max_loops = 0;
    if (bad == 2) options.max_fins = 0;
    if (bad == 3) options.max_wire_edges = 0;
    const auto result =
        TraverseShellBoundary(model.value(), NodeId{999}, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidShellTopology, SourceOrderCanStartWithAnEmptyFace) {
  auto f = Fixture();
  Set(f, 1, "face", 14);
  Set(f, 14, "previous", 0);
  Set(f, 14, "next", 10);
  Set(f, 10, "previous", 14);
  Set(f, 10, "next", 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      TraverseShellBoundary(model.value(), NodeId{1}, {3, 1, 1, 2});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().faces[0].face, NodeId{14});
  EXPECT_EQ(result.value().faces[1].face, NodeId{10});
  EXPECT_EQ(result.value().face_uses.back().boundary, 1);
}
TEST(ParasolidShellTopology, FaceAndWireCyclesFailAtExactLimits) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 14, "next", 10);
    if (bad == 1) Set(f, 10, "next_front", 10);
    if (bad == 2) Set(f, 21, "next", 20);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        TraverseShellBoundary(model.value(), NodeId{1}, {3, 1, 1, 2});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().message.find("cycle"), std::string::npos);
  }
}
TEST(ParasolidShellTopology, OwnershipAndReciprocalLinksAreCheckedPerSide) {
  for (int bad = 0; bad < 8; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 10, "previous", 14);
    if (bad == 1) Set(f, 10, "previous_front", 14);
    if (bad == 2) {
      Set(f, 14, "shell", 3);
      f.Add(13, 3, {});
    }
    if (bad == 3) {
      Set(f, 10, "front_shell", 3);
      f.Add(13, 3, {});
    }
    if (bad == 4) {
      Set(f, 20, "owner", 3);
      f.Add(12, 3, {});
    }
    if (bad == 5) Set(f, 1, "region", 0);
    if (bad == 6) Set(f, 1, "face", 20);
    if (bad == 7) Set(f, 20, "previous", 21);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidShellTopology,
     WireEdgesRequireClosedDummyAnchorsAndPairedVertices) {
  for (int bad = 0; bad < 7; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 30, "sense", '-');
    if (bad == 1) Set(f, 31, "vertex", 0);
    if (bad == 2) Set(f, 30, "loop", 11);
    if (bad == 3) Set(f, 31, "next_at_vx", 12);
    if (bad == 4) Set(f, 31, "other", 33);
    if (bad == 5) Set(f, 30, "edge", 21);
    if (bad == 6) Set(f, 30, "other", 30);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidShellTopology, WireAnchorsMayLinkAtSharedVertices) {
  for (bool closed : {false, true}) {
    auto f = Fixture();
    if (closed) {
      Set(f, 30, "vertex", 40);
      Set(f, 30, "next_at_vx", 31);
    } else {
      Set(f, 33, "vertex", 41);
      Set(f, 32, "vertex", 100);
      Set(f, 30, "next_at_vx", 33);
      f.Add(18, 100, {});
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().wire_edges[0].end_vertex,
              NodeId{closed ? 40u : 41u});
  }
}
TEST(ParasolidShellTopology, InvalidWireVertexLinksAreRejected) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 30, "next_at_vx", 30);
    if (bad == 1) Set(f, 32, "next_at_vx", 33);
    if (bad == 2) {
      Fin(f, 34, 41, 32, 21, '+');
      Set(f, 30, "next_at_vx", 34);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidShellTopology, EmptyAndConflictingAcornShellsAreInvalid) {
  for (int bad = 0; bad < 6; ++bad) {
    auto f = Acorn();
    if (bad == 0) Set(f, 1, "vertex", 0);
    if (bad == 1) {
      Set(f, 1, "face", 10);
      f.Add(14, 10, {});
    }
    if (bad == 2) {
      Set(f, 40, "owner", 3);
      f.Add(13, 3, {});
    }
    if (bad == 3) Set(f, 40, "point", 0);
    if (bad == 4) Set(f, 40, "point", 999);
    if (bad == 5) {
      Set(f, 40, "halfedge", 30);
      f.Add(17, 30, {});
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidShellTopology, MissingAndUnverifiedRolesRemainUnsupported) {
  for (int bad = 0; bad < 5; ++bad) {
    auto f = Fixture();
    if (bad == 0) Rename(f, 1, "front_face");
    if (bad == 1) Rename(f, 10, "next_front");
    if (bad == 2) Rename(f, 31, "next_at_vx");
    if (bad == 3) Rename(f, 1, "region");
    if (bad == 4) {
      for (auto& node : f.report.nodes) {
        if (node.index != 1) continue;
        for (auto& field :
             f.report.schemas[node.schema_index].definition.fields)
          if (field.name == "region") field.pointer_class = 60000;
      }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidShellTopology, IgnoresObsoleteBodyAndUnlistedEntities) {
  auto f = Fixture();
  Set(f, 1, "body", 999);
  f.Add(14, 888, {Pointer("shell", 1, 13)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(model.value().references_verified());
  EXPECT_TRUE(TraverseShellBoundary(model.value(), NodeId{1}).ok());
  EXPECT_FALSE(TraverseShellBoundary(model.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseShellBoundary(model.value(), NodeId{999}).ok());
}
TEST(ParasolidShellTopology, NestedErrorsKeepTheirSourceAndShellContext) {
  auto f = Fixture();
  Set(f, 12, "loop", 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto face = TraverseFaceBoundary(model.value(), NodeId{10});
  const auto shell = TraverseShellBoundary(model.value(), NodeId{1});
  ASSERT_FALSE(face.ok());
  ASSERT_FALSE(shell.ok());
  EXPECT_EQ(shell.error().code, face.error().code);
  EXPECT_EQ(shell.error().offset, face.error().offset);
  EXPECT_NE(shell.error().message.find("Shell 1, face 10"), std::string::npos);
  EXPECT_NE(shell.error().message.find(face.error().message),
            std::string::npos);
}
TEST(ParasolidShellTopology, OwnsCachedFaceIndicesAcrossCopiesAndMoves) {
  auto read = [] {
    const auto f = Fixture();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<ShellBoundary>::Err(model.error());
    return TraverseShellBoundary(model.value(), NodeId{1});
  };
  const auto result = read();
  ASSERT_TRUE(result.ok());
  auto copy = result.value();
  auto moved = std::move(copy);
  ASSERT_EQ(moved.face_uses.size(), 3);
  ASSERT_EQ(moved.faces.size(), 2);
  EXPECT_EQ(moved.faces[moved.face_uses.back().boundary].face, NodeId{10});
  EXPECT_EQ(moved.wire_edges[0].end_vertex, NodeId{41});
}
}  // namespace
}  // namespace openparasolid
