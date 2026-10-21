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
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < fields.size(); ++i)
      if (fields[i].name == role) {
        node.fields[i].values[0] = uint64_t{value};
        return;
      }
  }
  ADD_FAILURE() << "Missing test field";
}
void Region(ModelFixture& f, uint32_t shell, char kind = 'S',
            uint32_t owner = 0) {
  f.Add(19, 1,
        {Pointer("body", 2, 12), Pointer("shell", shell, 13),
         Pointer("owner", owner, 12),
         ByteValue("type", kind, FieldType::kCharacter)});
  f.Add(12, 2, {});
}
void Shell(ModelFixture& f, uint32_t id, uint32_t next, uint32_t face,
           uint32_t front, uint32_t wire, uint32_t vertex) {
  f.Add(13, id,
        {Pointer("region", 1, 19), Pointer("next", next, 13),
         Pointer("face", face, 14), Pointer("front_face", front, 14),
         Pointer("edge", wire, 16), Pointer("vertex", vertex, 18)});
}
void Face(ModelFixture& f, uint32_t id, uint32_t shell, uint32_t loop,
          uint32_t front = 0) {
  f.Add(14, id,
        {Pointer("surface", 0, 1006), Pointer("loop", loop, 15),
         Pointer("shell", shell, 13), Pointer("previous", 0, 14),
         Pointer("next", 0, 14), Pointer("front_shell", front, 13),
         Pointer("previous_front", 0, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '+', FieldType::kCharacter)});
}
void Loop(ModelFixture& f, uint32_t face, uint32_t loop, uint32_t fin) {
  f.Add(15, loop,
        {Pointer("face", face, 14), Pointer("halfedge", fin, 17),
         Pointer("next", 0, 15)});
}
void Fin(ModelFixture& f, uint32_t id, uint32_t loop, uint32_t vertex,
         uint32_t other, uint32_t edge, char sense) {
  f.Add(17, id,
        {Pointer("loop", loop, 15), Pointer("forward", loop ? id : 0, 17),
         Pointer("backward", loop ? id : 0, 17), Pointer("vertex", vertex, 18),
         Pointer("other", other, 17), Pointer("edge", edge, 16),
         Pointer("curve", 0, 1008), Pointer("next_at_vx", 0, 17),
         ByteValue("sense", sense, FieldType::kCharacter)});
}
void IsolatedFace(ModelFixture& f, uint32_t face, uint32_t shell,
                  uint32_t front = 0) {
  Face(f, face, shell, face + 1, front);
  Loop(f, face, face + 1, face + 2);
  Fin(f, face + 2, face + 1, face + 3, 0, 0, '?');
  f.Add(18, face + 3, {});
}
void Wire(ModelFixture& f, uint32_t id, uint32_t shell) {
  f.Add(16, id,
        {Pointer("owner", shell, 1029), Pointer("previous", 0, 16),
         Pointer("next", 0, 16), Pointer("halfedge", id + 1, 17),
         Pointer("curve", 0, 1008)});
  Fin(f, id + 1, 0, id + 4, id + 2, id, '+');
  Fin(f, id + 2, 0, id + 3, id + 1, id, '-');
  f.Add(18, id + 3, {});
  f.Add(18, id + 4, {});
}
ModelFixture Fixture(bool extra = false) {
  ModelFixture f;
  Region(f, 10);
  Shell(f, 10, 11, 100, 100, 0, 0);
  IsolatedFace(f, 100, 10, 10);
  Shell(f, 11, 12, 0, 0, 0, 200);
  f.Add(18, 200,
        {Pointer("owner", 11, 1029), Pointer("halfedge", 0, 17),
         Pointer("point", 201, 29)});
  f.Point(201);
  Shell(f, 12, extra ? 13 : 0, 0, 0, 300, 0);
  Wire(f, 300, 12);
  if (extra) {
    Shell(f, 13, 0, 110, 0, 310, 0);
    IsolatedFace(f, 110, 13);
    Wire(f, 310, 13);
  }
  return f;
}
TEST(ParasolidRegionTopology, OwnsOrderedBoundariesAfterExactBudgetsExpire) {
  auto read = [] {
    const auto f = Fixture();
    const auto m = ModelBuilder::Build(f.report);
    if (!m.ok()) return Result<RegionBoundary>::Err(m.error());
    return TraverseRegionBoundary(m.value(), NodeId{1}, {3, 2, 1, 1, 1});
  };
  const auto result = read();
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto copy = result.value();
  const auto moved = std::move(copy);
  EXPECT_EQ(moved.region, NodeId{1});
  EXPECT_EQ(moved.body, NodeId{2});
  EXPECT_EQ(moved.kind, RegionKind::kSolid);
  ASSERT_EQ(moved.shells.size(), 3);
  EXPECT_EQ(moved.shells[0].shell, NodeId{10});
  ASSERT_EQ(moved.shells[0].face_uses.size(), 2);
  EXPECT_EQ(moved.shells[0].face_uses[1].boundary, 0);
  EXPECT_EQ(moved.shells[1].isolated_vertex->vertex, NodeId{200});
  EXPECT_EQ(moved.shells[2].wire_edges[0].edge, NodeId{300});
}
TEST(ParasolidRegionTopology, AggregateLimitsApplyAcrossAllShells) {
  const auto f = Fixture(true);
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  ASSERT_TRUE(
      TraverseRegionBoundary(m.value(), NodeId{1}, {4, 3, 2, 2, 2}).ok());
  for (const auto options : {RegionTraversalOptions{3, 3, 2, 2, 2},
                             RegionTraversalOptions{4, 2, 2, 2, 2},
                             RegionTraversalOptions{4, 3, 1, 2, 2},
                             RegionTraversalOptions{4, 3, 2, 1, 2},
                             RegionTraversalOptions{4, 3, 2, 2, 1}}) {
    const auto r = TraverseRegionBoundary(m.value(), NodeId{1}, options);
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kResourceLimit);
  }
  for (int i = 0; i < 5; ++i) {
    RegionTraversalOptions o;
    if (i == 0) o.max_shells = 0;
    if (i == 1) o.max_face_uses = 0;
    if (i == 2) o.max_loops = 0;
    if (i == 3) o.max_fins = 0;
    if (i == 4) o.max_wire_edges = 0;
    const auto r = TraverseRegionBoundary(m.value(), NodeId{999}, o);
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidRegionTopology, EmptyRegionsRetainKindAndRecordedParentBody) {
  for (char kind : {'S', 'V'}) {
    ModelFixture f;
    Region(f, 0, kind, 3);
    f.Add(12, 3, {});
    // A child-body owner does not replace the recorded compound parent.
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value().body, NodeId{2});
    EXPECT_EQ(r.value().kind,
              kind == 'S' ? RegionKind::kSolid : RegionKind::kVoid);
    EXPECT_TRUE(r.value().shells.empty());
  }
}
TEST(ParasolidRegionTopology, UnavailableAndUnknownTypesRemainUnsupported) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    auto field = ByteValue(bad == 1 ? "unavailable" : "type", '?',
                           bad == 2 ? FieldType::kByte : FieldType::kCharacter);
    if (bad == 3)
      field =
          Array("type", FieldType::kCharacter, {uint64_t{'S'}, uint64_t{'V'}});
    f.Add(19, 1, {Pointer("body", 2, 12), Pointer("shell", 0, 13), field});
    f.Add(12, 2, {});
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto* n = m.value().FindNode(NodeId{1});
    if (bad == 0)
      EXPECT_EQ(n->region_type, '?');
    else
      EXPECT_FALSE(n->region_type);
    const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidRegionTopology, CyclesOwnersAndRequiredTargetsAreRejected) {
  for (int bad = 0; bad < 8; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 12, "next", 10);
    if (bad == 1) {
      Set(f, 10, "region", 3);
      f.Add(19, 3, {});
    }
    if (bad == 2) Set(f, 12, "next", 999);
    if (bad == 3) Set(f, 12, "next", 100);
    if (bad == 4) Set(f, 1, "body", 0);
    if (bad == 5) Set(f, 1, "body", 999);
    if (bad == 6) Set(f, 1, "body", 100);
    if (bad == 7) Set(f, 1, "shell", 100);
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r =
        TraverseRegionBoundary(m.value(), NodeId{1}, {3, 2, 1, 1, 1});
    ASSERT_FALSE(r.ok()) << bad;
    EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord) << bad;
    if (bad == 0) EXPECT_NE(r.error().message.find("cycle"), std::string::npos);
  }
}
TEST(ParasolidRegionTopology, MissingAndUnverifiedRolesRemainUnsupported) {
  for (int bad = 0; bad < 4; ++bad) {
    auto f = Fixture();
    const uint32_t id = bad == 2 ? 10 : 1;
    const std::string role = bad == 0 ? "body" : bad == 2 ? "next" : "shell";
    for (auto& n : f.report.nodes)
      if (n.index == id)
        for (auto& field : f.report.schemas[n.schema_index].definition.fields)
          if (field.name == role) {
            if (bad == 3)
              field.pointer_class = 60000;
            else
              field.name = "unavailable";
          }
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidRegionTopology, UnlistedEntitiesDoNotBlockLocalQueries) {
  auto f = Fixture();
  f.Add(13, 999, {Pointer("region", 1, 19)});
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  EXPECT_TRUE(TraverseRegionBoundary(m.value(), NodeId{1}).ok());
  EXPECT_FALSE(TraverseRegionBoundary(m.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseRegionBoundary(m.value(), NodeId{888}).ok());
}
TEST(ParasolidRegionTopology, NestedErrorsRetainSourceAndRegionContext) {
  auto f = Fixture();
  Set(f, 102, "loop", 0);
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  const auto child = TraverseShellBoundary(m.value(), NodeId{10});
  const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
  ASSERT_FALSE(child.ok());
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, child.error().code);
  EXPECT_EQ(r.error().offset, child.error().offset);
  EXPECT_NE(r.error().message.find("Region 1, shell 10"), std::string::npos);
  EXPECT_NE(r.error().message.find(child.error().message), std::string::npos);
}
TEST(ParasolidRegionTopology, DifferentShellsCannotShareFacesOrVertices) {
  for (bool face : {false, true}) {
    auto f = Fixture();
    if (face) {
      Set(f, 10, "front_face", 0);
      Set(f, 12, "front_face", 100);
      Set(f, 100, "front_shell", 12);
    } else
      Set(f, 102, "vertex", 200);
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    for (uint32_t shell : {10, 11, 12})
      ASSERT_TRUE(TraverseShellBoundary(m.value(), NodeId{shell}).ok());
    const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(r.error().message.find("shared"), std::string::npos);
  }
}
TEST(ParasolidRegionTopology, DifferentShellsCannotShareVertexFreeRingEdges) {
  ModelFixture f;
  Region(f, 10);
  Shell(f, 10, 11, 100, 0, 0, 0);
  Shell(f, 11, 0, 110, 0, 0, 0);
  Face(f, 100, 10, 101);
  Face(f, 110, 11, 111);
  Loop(f, 100, 101, 333);
  Loop(f, 110, 111, 334);
  Fin(f, 333, 101, 0, 334, 300, '+');
  Fin(f, 334, 111, 0, 333, 300, '-');
  f.Add(16, 300, {Pointer("halfedge", 333, 17), Pointer("curve", 0, 1008)});
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  ASSERT_TRUE(TraverseShellBoundary(m.value(), NodeId{10}).ok());
  ASSERT_TRUE(TraverseShellBoundary(m.value(), NodeId{11}).ok());
  const auto r = TraverseRegionBoundary(m.value(), NodeId{1});
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(r.error().message.find("shared"), std::string::npos);
}
}  // namespace
}  // namespace openparasolid
