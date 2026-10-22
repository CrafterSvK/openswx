// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <initializer_list>
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
ShellBoundary IncidenceShell() {
  ShellBoundary shell;
  shell.shell = NodeId{1};
  FaceBoundary first, second;
  first.face = NodeId{2};
  second.face = NodeId{3};
  LoopBoundary loop;
  for (uint32_t edge : {9u, 3u, 3u, 7u, 7u, 7u}) {
    FinUse fin;
    fin.fin = NodeId{static_cast<uint32_t>(loop.fins.size() + 10)};
    fin.edge = NodeId{edge};
    fin.sense = loop.fins.size() == 2 || loop.fins.size() == 5
                    ? TopologySense::kReverse
                    : TopologySense::kForward;
    loop.fins.push_back(fin);
  }
  first.loops.push_back(loop);
  loop.fins = {loop.fins.back()};
  loop.fins[0].fin = NodeId{30};
  second.loops.push_back(loop);
  LoopBoundary isolated;
  isolated.isolated_vertex = IsolatedLoopVertex{NodeId{31}, NodeId{32}};
  second.loops.push_back(isolated);
  shell.faces = {first, second};
  shell.face_uses = {{0, ShellFaceSide::kFront},
                     {0, ShellFaceSide::kBack},
                     {1, ShellFaceSide::kFront}};
  shell.wire_edges.push_back({NodeId{100}, {}, {}, {}});
  return shell;
}
TEST(ParasolidShellIncidence, CountsSeamsAndValenceWithoutDoublingFaceSides) {
  const auto result = AnalyzeShellEdgeIncidence(IncidenceShell(), {2, 3, 8, 3});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().shell, NodeId{1});
  EXPECT_EQ(result.value().boundary_edges, 1);
  EXPECT_EQ(result.value().paired_edges, 1);
  EXPECT_EQ(result.value().higher_valence_edges, 1);
  EXPECT_EQ(result.value().face_fins, 7);
  EXPECT_EQ(result.value().isolated_loops, 1);
  const auto& edges = result.value().edges;
  ASSERT_EQ(edges.size(), 3);
  EXPECT_EQ(edges[0].edge, NodeId{3});
  EXPECT_EQ(edges[0].forward_fins, 1);
  EXPECT_EQ(edges[0].reverse_fins, 1);
  EXPECT_EQ(edges[1].edge, NodeId{7});
  EXPECT_EQ(edges[1].forward_fins, 2);
  EXPECT_EQ(edges[1].reverse_fins, 2);
  EXPECT_EQ(edges[2].edge, NodeId{9});
}
TEST(ParasolidShellIncidence, BoundsAllVisitsAndStorageBeforeReturningResults) {
  const auto shell = IncidenceShell();
  for (int field = 0; field < 4; ++field) {
    ShellEdgeIncidenceOptions options{2, 3, 8, 3};
    uint64_t* selected = field == 0   ? &options.max_faces
                         : field == 1 ? &options.max_loops
                         : field == 2 ? &options.max_fins
                                      : &options.max_edges;
    --*selected;
    auto result = AnalyzeShellEdgeIncidence(shell, options);
    ASSERT_FALSE(result.ok()) << field;
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
    *selected = 0;
    result = AnalyzeShellEdgeIncidence(ShellBoundary{}, options);
    ASSERT_FALSE(result.ok()) << field;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidShellIncidence, TraversedWireAndAcornShellsHaveNoFaceEdges) {
  for (const auto& fixture : {Fixture(), Acorn()}) {
    const auto model = ModelBuilder::Build(fixture.report);
    ASSERT_TRUE(model.ok());
    const auto shell = TraverseShellBoundary(model.value(), NodeId{1});
    ASSERT_TRUE(shell.ok()) << shell.error().message;
    const auto result = AnalyzeShellEdgeIncidence(shell.value());
    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.value().edges.empty());
    EXPECT_EQ(result.value().face_fins, 0);
    EXPECT_EQ(result.value().boundary_edges, 0);
    EXPECT_EQ(result.value().paired_edges, 0);
    EXPECT_EQ(result.value().face_components, shell.value().faces.size());
  }
}
TEST(ParasolidShellIncidence, RejectsMalformedEdgeUsesAndMixedIsolatedLoops) {
  for (int bad = 0; bad < 5; ++bad) {
    auto shell = IncidenceShell();
    auto& fin = shell.faces[0].loops[0].fins[0];
    if (bad == 0) fin.edge = NodeId{};
    if (bad == 1) fin.sense = static_cast<TopologySense>(99);
    if (bad == 2)
      shell.faces[0].loops[0].isolated_vertex = IsolatedLoopVertex{};
    if (bad == 3) shell.faces[0].face = NodeId{};
    if (bad == 4) shell.faces[1].face = shell.faces[0].face;
    const auto result = AnalyzeShellEdgeIncidence(shell);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
void ComponentFace(ShellBoundary& shell, uint32_t id,
                   std::initializer_list<uint32_t> edges) {
  FaceBoundary face;
  face.face = NodeId{id};
  LoopBoundary loop;
  for (uint32_t edge : edges) {
    FinUse fin;
    fin.fin = NodeId{static_cast<uint32_t>(shell.faces.size() * 1000 +
                                           loop.fins.size() + 1)};
    fin.edge = NodeId{edge};
    // All test faces share a vertex, which must not connect edge-disjoint
    // faces.
    fin.start_vertex = NodeId{99};
    fin.end_vertex = NodeId{99};
    loop.fins.push_back(fin);
  }
  if (!loop.fins.empty()) face.loops.push_back(loop);
  shell.faces.push_back(face);
}
TEST(ParasolidShellIncidence, GroupsTransitiveEdgeConnectionsInStableIdOrder) {
  ShellBoundary shell;
  ComponentFace(shell, 90, {5});
  ComponentFace(shell, 10, {6});
  ComponentFace(shell, 40, {5, 6});
  ComponentFace(shell, 3, {7, 7});
  ComponentFace(shell, 80, {});
  ComponentFace(shell, 2, {});
  LoopBoundary isolated;
  isolated.isolated_vertex = IsolatedLoopVertex{NodeId{90}, NodeId{99}};
  shell.faces.back().loops.push_back(isolated);
  shell.face_uses = {{0, ShellFaceSide::kFront}, {0, ShellFaceSide::kBack}};
  for (int order = 0; order < 2; ++order) {
    const auto result = AnalyzeShellEdgeIncidence(shell);
    ASSERT_TRUE(result.ok()) << result.error().message;
    const auto& groups = result.value().components;
    ASSERT_EQ(groups.size(), 4);
    EXPECT_EQ(result.value().face_components, 4);
    EXPECT_EQ(groups[0].faces, (std::vector<NodeId>{{2}}));
    EXPECT_EQ(groups[1].faces, (std::vector<NodeId>{{3}}));
    EXPECT_EQ(groups[2].faces, (std::vector<NodeId>{{10}, {40}, {90}}));
    EXPECT_EQ(groups[3].faces, (std::vector<NodeId>{{80}}));
    EXPECT_TRUE(groups[0].edges.empty());
    EXPECT_EQ(groups[0].isolated_loops, 1);
    EXPECT_EQ(groups[0].face_fins, 0);
    EXPECT_EQ(groups[1].edges, (std::vector<NodeId>{{7}}));
    EXPECT_EQ(groups[1].paired_edges, 1);
    EXPECT_EQ(groups[1].face_fins, 2);
    EXPECT_EQ(groups[2].edges, (std::vector<NodeId>{{5}, {6}}));
    EXPECT_EQ(groups[2].paired_edges, 2);
    EXPECT_EQ(groups[2].face_fins, 4);
    EXPECT_TRUE(groups[3].edges.empty());
    EXPECT_EQ(groups[3].isolated_loops, 0);

    std::reverse(shell.faces.begin(), shell.faces.end());
  }
}
TEST(ParasolidShellIncidence, PairedEdgesCanBelongToSeparateComponents) {
  ShellBoundary shell;
  ComponentFace(shell, 1, {10, 10});
  ComponentFace(shell, 2, {20, 20});
  const auto result = AnalyzeShellEdgeIncidence(shell);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().boundary_edges, 0);
  EXPECT_EQ(result.value().paired_edges, 2);
  EXPECT_EQ(result.value().face_components, 2);
}
TEST(ParasolidShellIncidence, HigherValenceEdgesConnectAllTheirFaces) {
  ShellBoundary shell;
  for (uint32_t id = 300; id > 0; --id) ComponentFace(shell, id, {1});
  const auto result = AnalyzeShellEdgeIncidence(shell, {300, 300, 300, 1});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().higher_valence_edges, 1);
  EXPECT_EQ(result.value().face_components, 1);
  ASSERT_EQ(result.value().components.size(), 1);
  const auto& faces = result.value().components[0].faces;
  ASSERT_EQ(faces.size(), 300);
  EXPECT_EQ(faces.front(), NodeId{1});
  EXPECT_EQ(faces.back(), NodeId{300});
  EXPECT_EQ(result.value().components[0].edges, (std::vector<NodeId>{{1}}));
  EXPECT_EQ(result.value().components[0].higher_valence_edges, 1);
  EXPECT_EQ(result.value().components[0].face_fins, 300);

  const auto empty = AnalyzeShellEdgeIncidence(ShellBoundary{});
  ASSERT_TRUE(empty.ok());
  EXPECT_EQ(empty.value().face_components, 0);
  EXPECT_TRUE(empty.value().components.empty());
}
TEST(ParasolidShellIncidence, LocalizesDifferentValencesToTheirComponents) {
  ShellBoundary shell;
  ComponentFace(shell, 50, {9});
  ComponentFace(shell, 10, {3, 3});
  ComponentFace(shell, 30, {7, 7, 7});
  ComponentFace(shell, 20, {});
  LoopBoundary isolated;
  isolated.isolated_vertex = IsolatedLoopVertex{NodeId{90}, NodeId{99}};
  shell.faces.back().loops.push_back(isolated);
  const auto result = AnalyzeShellEdgeIncidence(shell);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& groups = result.value().components;
  ASSERT_EQ(groups.size(), 4);
  EXPECT_EQ(groups[0].faces, (std::vector<NodeId>{{10}}));
  EXPECT_EQ(groups[0].edges, (std::vector<NodeId>{{3}}));
  EXPECT_EQ(groups[0].paired_edges, 1);
  EXPECT_EQ(groups[1].faces, (std::vector<NodeId>{{20}}));
  EXPECT_TRUE(groups[1].edges.empty());
  EXPECT_EQ(groups[1].isolated_loops, 1);
  EXPECT_EQ(groups[2].faces, (std::vector<NodeId>{{30}}));
  EXPECT_EQ(groups[2].edges, (std::vector<NodeId>{{7}}));
  EXPECT_EQ(groups[2].higher_valence_edges, 1);
  EXPECT_EQ(groups[3].faces, (std::vector<NodeId>{{50}}));
  EXPECT_EQ(groups[3].edges, (std::vector<NodeId>{{9}}));
  EXPECT_EQ(groups[3].boundary_edges, 1);
  uint64_t boundary = 0, paired = 0, higher = 0, fins = 0, isolated_count = 0;
  std::size_t edges = 0;
  for (const auto& group : groups) {
    boundary += group.boundary_edges;
    paired += group.paired_edges;
    higher += group.higher_valence_edges;
    fins += group.face_fins;
    isolated_count += group.isolated_loops;
    edges += group.edges.size();
  }
  EXPECT_EQ(boundary, result.value().boundary_edges);
  EXPECT_EQ(paired, result.value().paired_edges);
  EXPECT_EQ(higher, result.value().higher_valence_edges);
  EXPECT_EQ(fins, result.value().face_fins);
  EXPECT_EQ(isolated_count, result.value().isolated_loops);
  EXPECT_EQ(edges, result.value().edges.size());
}
TEST(ParasolidShellIncidence, ClassifiesBoundaryGroupsByRecordedVertexDegrees) {
  ShellBoundary shell;
  ComponentFace(shell, 1, {10, 11, 12, 20, 21, 30, 31, 32, 40, 50, 60, 61});
  auto& fins = shell.faces[0].loops[0].fins;
  const std::array<std::array<uint32_t, 2>, 12> endpoints{{
      {1, 2}, {2, 3}, {3, 1}, {4, 5}, {5, 6}, {7, 8},
      {7, 9}, {7, 10}, {0, 0}, {11, 11}, {12, 13}, {12, 13}}};
  for (std::size_t i = 0; i < fins.size(); ++i) {
    fins[i].start_vertex = NodeId{endpoints[i][0]};
    fins[i].end_vertex = NodeId{endpoints[i][1]};
  }
  for (int order = 0; order < 2; ++order) {
    const auto result = AnalyzeShellEdgeIncidence(shell, {1, 1, 12, 12});
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().boundary_cycles, 4);
    EXPECT_EQ(result.value().boundary_chains, 1);
    EXPECT_EQ(result.value().branched_boundaries, 1);
    const auto& component = result.value().components[0];
    EXPECT_EQ(component.boundary_cycles, 4);
    EXPECT_EQ(component.boundary_chains, 1);
    EXPECT_EQ(component.branched_boundaries, 1);
    const auto& groups = component.boundary_groups;
    ASSERT_EQ(groups.size(), 6);
    for (const auto& group : groups) {
      if (group.kind == BoundaryGroupKind::kBranched) {
        EXPECT_TRUE(group.path.empty());
        continue;
      }
      ASSERT_EQ(group.path.size(), group.edges.size());
      for (std::size_t i = 1; i < group.path.size(); ++i)
        EXPECT_EQ(group.path[i - 1].end_vertex, group.path[i].start_vertex);
      if (group.kind == BoundaryGroupKind::kCycle)
        EXPECT_EQ(group.path.front().start_vertex, group.path.back().end_vertex);
      else {
        EXPECT_EQ(group.path.front().start_vertex, group.end_vertices.front());
        EXPECT_EQ(group.path.back().end_vertex, group.end_vertices.back());
      }
    }

    EXPECT_EQ(groups[0].kind, BoundaryGroupKind::kCycle);
    EXPECT_EQ(groups[0].edges, (std::vector<NodeId>{{10}, {11}, {12}}));
    EXPECT_EQ(groups[1].kind, BoundaryGroupKind::kChain);
    EXPECT_EQ(groups[1].end_vertices, (std::vector<NodeId>{{4}, {6}}));
    EXPECT_EQ(groups[2].kind, BoundaryGroupKind::kBranched);
    EXPECT_EQ(groups[2].branch_vertices, (std::vector<NodeId>{{7}}));
    EXPECT_EQ(groups[2].end_vertices, (std::vector<NodeId>{{8}, {9}, {10}}));
    EXPECT_EQ(groups[3].kind, BoundaryGroupKind::kCycle);
    EXPECT_EQ(groups[3].edges, (std::vector<NodeId>{{40}}));
    EXPECT_TRUE(groups[3].vertices.empty());
    EXPECT_EQ(groups[4].kind, BoundaryGroupKind::kCycle);
    EXPECT_EQ(groups[4].vertices, (std::vector<NodeId>{{11}}));
    EXPECT_EQ(groups[5].kind, BoundaryGroupKind::kCycle);
    EXPECT_EQ(groups[5].vertices, (std::vector<NodeId>{{12}, {13}}));
    EXPECT_EQ(groups[3].path[0].sense, TopologySense::kForward);
    EXPECT_EQ(groups[3].path[0].start_vertex, NodeId{});
    EXPECT_EQ(groups[4].path[0].sense, TopologySense::kForward);
    EXPECT_EQ(groups[5].path[0].edge, NodeId{60});
    EXPECT_EQ(groups[5].path[0].sense, TopologySense::kForward);
    EXPECT_EQ(groups[5].path[1].edge, NodeId{61});
    EXPECT_EQ(groups[5].path[1].sense, TopologySense::kReverse);

    std::reverse(fins.begin(), fins.end());
  }
}
TEST(ParasolidShellIncidence, BoundaryGroupsStayWithinTheirFaceComponent) {
  ShellBoundary shell;
  ComponentFace(shell, 1, {10});
  ComponentFace(shell, 2, {20});
  shell.faces[0].loops[0].fins[0].start_vertex = NodeId{1};
  shell.faces[0].loops[0].fins[0].end_vertex = NodeId{2};
  shell.faces[1].loops[0].fins[0].start_vertex = NodeId{1};
  shell.faces[1].loops[0].fins[0].end_vertex = NodeId{3};
  const auto result = AnalyzeShellEdgeIncidence(shell);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().boundary_chains, 2);
  ASSERT_EQ(result.value().components.size(), 2);
  for (const auto& component : result.value().components) {
    ASSERT_EQ(component.boundary_groups.size(), 1);
    EXPECT_EQ(component.boundary_groups[0].edges.size(), 1);
    EXPECT_EQ(component.boundary_groups[0].kind, BoundaryGroupKind::kChain);
  }
}
TEST(ParasolidShellIncidence, RejectsPartiallyMissingBoundaryVertices) {
  ShellBoundary shell;
  ComponentFace(shell, 1, {10});
  auto& fin = shell.faces[0].loops[0].fins[0];
  fin.start_vertex = NodeId{};
  const auto result = AnalyzeShellEdgeIncidence(shell);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidShellIncidence, OrdersChainsIndependentlyOfFinAndEdgeIdOrder) {
  ShellBoundary shell;
  ComponentFace(shell, 1, {30, 10, 20});
  auto& fins = shell.faces[0].loops[0].fins;
  fins[0].start_vertex = NodeId{2};
  fins[0].end_vertex = NodeId{1};
  fins[1].start_vertex = NodeId{2};
  fins[1].end_vertex = NodeId{3};
  fins[1].sense = TopologySense::kReverse;
  fins[2].start_vertex = NodeId{3};
  fins[2].end_vertex = NodeId{4};
  for (int order = 0; order < 3; ++order) {
    const auto result = AnalyzeShellEdgeIncidence(shell);
    ASSERT_TRUE(result.ok()) << result.error().message;
    const auto& path = result.value().components[0].boundary_groups[0].path;
    ASSERT_EQ(path.size(), 3);
    const std::array<uint32_t, 3> edge_ids{30, 10, 20};
    for (std::size_t i = 0; i < path.size(); ++i) {
      EXPECT_EQ(path[i].edge, NodeId{edge_ids[i]});
      EXPECT_EQ(path[i].fin, NodeId{static_cast<uint32_t>(i + 1)});
      EXPECT_EQ(path[i].start_vertex, NodeId{static_cast<uint32_t>(i + 1)});
      EXPECT_EQ(path[i].end_vertex, NodeId{static_cast<uint32_t>(i + 2)});
      EXPECT_EQ(path[i].sense,
                i < 2 ? TopologySense::kReverse : TopologySense::kForward);
    }
    std::reverse(fins.begin(), fins.end());
    if (order == 1)
      for (auto& fin : fins) {
        std::swap(fin.start_vertex, fin.end_vertex);
        fin.sense = fin.sense == TopologySense::kForward
                        ? TopologySense::kReverse
                        : TopologySense::kForward;
      }
  }
}
TEST(ParasolidShellIncidence, TracesLongChainsWithinExactBudgets) {
  ShellBoundary shell;
  FaceBoundary face;
  face.face = NodeId{1};
  LoopBoundary loop;
  constexpr uint32_t kEdges = 2048;
  for (uint32_t i = 0; i < kEdges; ++i) {
    FinUse fin;
    fin.fin = NodeId{i + 1};
    fin.edge = NodeId{kEdges - i};
    fin.start_vertex = NodeId{i + 1};
    fin.end_vertex = NodeId{i + 2};
    loop.fins.push_back(fin);
  }
  face.loops.push_back(std::move(loop));
  shell.faces.push_back(std::move(face));
  const auto result = AnalyzeShellEdgeIncidence(shell, {1, 1, kEdges, kEdges});
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& path = result.value().components[0].boundary_groups[0].path;
  ASSERT_EQ(path.size(), kEdges);
  EXPECT_EQ(path.front().edge, NodeId{kEdges});
  EXPECT_EQ(path.back().edge, NodeId{1});
  EXPECT_EQ(path.front().start_vertex, NodeId{1});
  EXPECT_EQ(path.back().end_vertex, NodeId{kEdges + 1});
}
}  // namespace
}  // namespace openparasolid
