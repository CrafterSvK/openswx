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

void Body(ModelFixture& f, uint32_t id, uint32_t region, uint32_t owner = 0,
          uint8_t kind = 6, uint32_t child = 0) {
  f.Add(12, id,
        {Pointer("owner", owner, 1040), Pointer("region", region, 19),
         Pointer("child", child, 12), Pointer("previous", 0, 12),
         Pointer("next", 0, 12), Pointer("shell", 0, 13),
         ByteValue("body_type", kind)});
}
void Region(ModelFixture& f, uint32_t id, uint32_t previous, uint32_t next,
            uint32_t shell, char kind) {
  f.Add(19, id,
        {Pointer("body", 1, 12), Pointer("previous", previous, 19),
         Pointer("next", next, 19), Pointer("shell", shell, 13),
         Pointer("owner", 0, 12),
         ByteValue("type", kind, FieldType::kCharacter)});
}
void Shell(ModelFixture& f, uint32_t id, uint32_t region, uint32_t next,
           uint32_t back, uint32_t front, uint32_t edge, uint32_t vertex = 0) {
  f.Add(13, id,
        {Pointer("region", region, 19), Pointer("next", next, 13),
         Pointer("face", back, 14), Pointer("front_face", front, 14),
         Pointer("edge", edge, 16), Pointer("vertex", vertex, 18)});
}
ModelFixture Fixture() {
  ModelFixture f;
  Body(f, 1, 10);
  Region(f, 10, 0, 11, 20, 'V');
  Region(f, 11, 10, 12, 21, 'S');
  Region(f, 12, 11, 0, 0, 'V');
  Shell(f, 20, 10, 22, 100, 0, 0);
  Shell(f, 22, 10, 0, 0, 0, 300);
  Shell(f, 21, 11, 23, 0, 100, 310);
  Shell(f, 23, 11, 0, 0, 0, 0, 200);
  IsolatedFace(f, 100, 20, 21);
  Wire(f, 300, 22);
  Wire(f, 310, 21);
  f.Add(18, 200,
        {Pointer("owner", 23, 1029), Pointer("halfedge", 0, 17),
         Pointer("point", 201, 29)});
  f.Point(201);
  return f;
}
ModelFixture CompoundFixture() {
  auto f = Fixture();
  Body(f, 3, 10, 1, 1);
  Body(f, 4, 12, 1, 2);
  Set(f, 3, "next", 4);
  Set(f, 4, "previous", 3);
  Set(f, 1, "child", 3);
  Set(f, 10, "owner", 3);
  Set(f, 11, "owner", 3);
  Set(f, 12, "owner", 4);
  return f;
}
constexpr BodyTraversalOptions kExact{3, 4, 2, 2, 2, 2};
TEST(ParasolidBodyTopology, OwnsRegionsAndSharedFacesWithAggregateBudgets) {
  const auto result = [] {
    const auto f = Fixture();
    const auto m = ModelBuilder::Build(f.report);
    if (!m.ok()) return Result<BodyBoundary>::Err(m.error());
    return TraverseBodyBoundary(m.value(), NodeId{1}, kExact);
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto copy = result.value();
  auto moved = std::move(copy);
  EXPECT_EQ(moved.body, NodeId{1});
  EXPECT_EQ(moved.kind, BodyKind::kGeneral);
  ASSERT_EQ(moved.regions.size(), 3);
  EXPECT_EQ(moved.regions[0].region, NodeId{10});
  EXPECT_EQ(moved.regions[1].region, NodeId{11});
  EXPECT_EQ(moved.regions[2].region, NodeId{12});
  EXPECT_EQ(moved.regions[0].kind, RegionKind::kVoid);
  EXPECT_EQ(moved.regions[1].kind, RegionKind::kSolid);
  EXPECT_TRUE(moved.regions[2].shells.empty());
  const auto& back = moved.regions[0].shells[0];
  const auto& front = moved.regions[1].shells[0];
  EXPECT_EQ(back.faces[0].face, front.faces[0].face);
  EXPECT_EQ(back.face_uses[0].side, ShellFaceSide::kBack);
  EXPECT_EQ(front.face_uses[0].side, ShellFaceSide::kFront);
  EXPECT_EQ(moved.regions[1].shells[1].isolated_vertex->point, NodeId{201});
}
TEST(ParasolidBodyTopology, AllSixLimitsApplyAcrossTheWholeBody) {
  const auto f = Fixture();
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  for (int bad = 0; bad < 6; ++bad) {
    auto o = kExact;
    if (bad == 0) --o.max_regions;
    if (bad == 1) --o.max_shells;
    if (bad == 2) --o.max_face_uses;
    if (bad == 3) --o.max_loops;
    if (bad == 4) --o.max_fins;
    if (bad == 5) --o.max_wire_edges;
    const auto r = TraverseBodyBoundary(m.value(), NodeId{1}, o);
    ASSERT_FALSE(r.ok()) << bad;
    EXPECT_EQ(r.error().code, ErrorCode::kResourceLimit) << bad;
    if (bad == 0) o.max_regions = 0;
    if (bad == 1) o.max_shells = 0;
    if (bad == 2) o.max_face_uses = 0;
    if (bad == 3) o.max_loops = 0;
    if (bad == 4) o.max_fins = 0;
    if (bad == 5) o.max_wire_edges = 0;
    const auto zero = TraverseBodyBoundary(m.value(), NodeId{999}, o);
    ASSERT_FALSE(zero.ok());
    EXPECT_EQ(zero.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidBodyTopology, KnownKindsAndNullRegionHeadsRemainExplicit) {
  for (uint8_t kind : {1, 2, 3, 6}) {
    ModelFixture f;
    Body(f, 1, 0, 0, kind);
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r = TraverseBodyBoundary(m.value(), NodeId{1});
    ASSERT_TRUE(r.ok());
    EXPECT_TRUE(r.value().regions.empty());
    EXPECT_EQ(r.value().kind, kind == 1   ? BodyKind::kSolid
                              : kind == 2 ? BodyKind::kWire
                              : kind == 3 ? BodyKind::kSheet
                                          : BodyKind::kGeneral);
  }
}
TEST(ParasolidBodyTopology, RawUnknownAndIncompatibleBodyTypesStayUnsupported) {
  for (int bad = 0; bad < 4; ++bad) {
    ModelFixture f;
    auto field = ByteValue(bad == 1 ? "unavailable" : "body_type", 99,
                           bad == 2 ? FieldType::kCharacter : FieldType::kByte);
    if (bad == 3)
      field = Array("body_type", FieldType::kByte, {uint64_t{1}, uint64_t{2}});
    f.Add(12, 1, {Pointer("owner", 0, 101), Pointer("region", 0, 19), field});
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    if (bad == 0)
      EXPECT_EQ(m.value().FindNode(NodeId{1})->body_type, 99);
    else
      EXPECT_FALSE(m.value().FindNode(NodeId{1})->body_type);
    const auto r = TraverseBodyBoundary(m.value(), NodeId{1});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidBodyTopology, CyclesOwnershipAndReciprocalLinksAreChecked) {
  for (int bad = 0; bad < 9; ++bad) {
    auto f = Fixture();
    if (bad == 0) Set(f, 12, "next", 10);
    if (bad == 1) Set(f, 10, "previous", 12);
    if (bad == 2) Set(f, 11, "previous", 0);
    if (bad == 3) {
      Set(f, 11, "body", 2);
      Body(f, 2, 0);
    }
    if (bad == 4) Set(f, 12, "next", 999);
    if (bad == 5) Set(f, 12, "next", 100);
    if (bad == 6) Set(f, 1, "region", 100);
    if (bad == 7) Set(f, 1, "owner", 100);
    if (bad == 8) Set(f, 1, "owner", 999);
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r = TraverseBodyBoundary(m.value(), NodeId{1}, kExact);
    ASSERT_FALSE(r.ok()) << bad;
    EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord) << bad;
    if (bad == 0) EXPECT_NE(r.error().message.find("cycle"), std::string::npos);
  }
}
TEST(ParasolidBodyTopology, MissingAndUnverifiedRolesRemainUnsupported) {
  for (int bad = 0; bad < 5; ++bad) {
    auto f = Fixture();
    const uint32_t id = bad == 2 || bad == 3 ? 10 : 1;
    const std::string role = bad == 0   ? "owner"
                             : bad == 2 ? "previous"
                             : bad == 3 ? "next"
                                        : "region";
    for (auto& n : f.report.nodes)
      if (n.index == id)
        for (auto& field : f.report.schemas[n.schema_index].definition.fields)
          if (field.name == role) {
            if (bad == 4)
              field.pointer_class = 60000;
            else
              field.name = "unavailable";
          }
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    const auto r = TraverseBodyBoundary(m.value(), NodeId{1});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidBodyTopology,
     CompoundParentReadsCombinedChainAndRejectsChildQueries) {
  auto f = Fixture();
  Body(f, 3, 10, 1, 1);
  Body(f, 4, 12, 1, 2);
  Set(f, 3, "next", 4);
  Set(f, 4, "previous", 3);
  Set(f, 1, "child", 3);
  Set(f, 10, "owner", 3);
  Set(f, 11, "owner", 3);
  Set(f, 12, "owner", 4);
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  const auto parent = TraverseBodyBoundary(m.value(), NodeId{1}, kExact);
  ASSERT_TRUE(parent.ok());
  EXPECT_EQ(parent.value().regions.size(), 3);
  EXPECT_EQ(parent.value().regions[1].body, NodeId{1});
  for (uint32_t child : {3, 4}) {
    const auto r = TraverseBodyBoundary(m.value(), NodeId{child});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_NE(r.error().message.find("compound parent"), std::string::npos);
  }
}
TEST(ParasolidBodyTopology, EmptyChildStillRequiresItsParent) {
  ModelFixture f;
  Body(f, 1, 0, 2);
  Body(f, 2, 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseBodyBoundary(model.value(), NodeId{1});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidBodyTopology, AmbiguousRegionPointerArrayIsRejected) {
  ModelFixture f;
  auto regions =
      Array("region", FieldType::kPointer, {uint64_t{10}, uint64_t{10}});
  regions.definition.pointer_class = 19;
  f.Add(12, 1, {ByteValue("body_type", 1), Pointer("owner", 0, 101), regions});
  Region(f, 10, 0, 0, 0, 'V');
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseBodyBoundary(model.value(), NodeId{1});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidBodyTopology, WorldOwnerSupportsConcreteAndUnionClasses) {
  for (uint16_t pointer_class : {101, 1040}) {
    auto f = Fixture();
    for (auto& n : f.report.nodes)
      if (n.index == 1)
        for (auto& field : f.report.schemas[n.schema_index].definition.fields)
          if (field.name == "owner") field.pointer_class = pointer_class;
    Set(f, 1, "owner", 400);
    f.Add(101, 400, {});
    const auto m = ModelBuilder::Build(f.report);
    ASSERT_TRUE(m.ok());
    EXPECT_TRUE(TraverseBodyBoundary(m.value(), NodeId{1}).ok());
  }
}
TEST(ParasolidBodyTopology, UnlistedEntitiesAndObsoleteShellDoNotBlockQuery) {
  auto f = Fixture();
  f.Add(19, 999, {Pointer("body", 1, 12)});
  Set(f, 1, "shell", 888);
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  EXPECT_FALSE(m.value().references_verified());
  EXPECT_TRUE(TraverseBodyBoundary(m.value(), NodeId{1}).ok());
  EXPECT_FALSE(TraverseBodyBoundary(m.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseBodyBoundary(m.value(), NodeId{777}).ok());
}
TEST(ParasolidBodyTopology, NestedErrorsKeepTheirCodeOffsetAndBodyContext) {
  auto f = Fixture();
  Set(f, 102, "loop", 0);
  const auto m = ModelBuilder::Build(f.report);
  ASSERT_TRUE(m.ok());
  const auto region = TraverseRegionBoundary(m.value(), NodeId{10});
  const auto r = TraverseBodyBoundary(m.value(), NodeId{1});
  ASSERT_FALSE(region.ok());
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.error().code, region.error().code);
  EXPECT_EQ(r.error().offset, region.error().offset);
  EXPECT_NE(r.error().message.find("Body 1, region 10"), std::string::npos);
  EXPECT_NE(r.error().message.find(region.error().message), std::string::npos);
}
TEST(ParasolidCompoundTopology, OwnsOrderedChildRangesWithoutCopyingRegions) {
  const auto result = [] {
    const auto f = CompoundFixture();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<CompoundBodyBoundary>::Err(model.error());
    return TraverseCompoundBodyBoundary(model.value(), NodeId{1}, {kExact, 2});
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto copy = result.value();
  auto moved = std::move(copy);
  EXPECT_EQ(moved.boundary.body, NodeId{1});
  EXPECT_EQ(moved.boundary.kind, BodyKind::kGeneral);
  ASSERT_EQ(moved.boundary.regions.size(), 3);
  ASSERT_EQ(moved.children.size(), 2);
  EXPECT_EQ(moved.children[0].body, NodeId{3});
  EXPECT_EQ(moved.children[0].kind, BodyKind::kSolid);
  EXPECT_EQ(moved.children[0].first_region, 0);
  EXPECT_EQ(moved.children[0].region_count, 2);
  EXPECT_EQ(moved.children[1].body, NodeId{4});
  EXPECT_EQ(moved.children[1].kind, BodyKind::kWire);
  EXPECT_EQ(moved.children[1].first_region, 2);
  EXPECT_EQ(moved.children[1].region_count, 1);
  EXPECT_EQ(moved.boundary.regions[moved.children[1].first_region].region,
            NodeId{12});
  EXPECT_EQ(moved.boundary.regions[1].shells[1].isolated_vertex->point,
            NodeId{201});
}
TEST(ParasolidCompoundTopology, EmptyCompoundRequiresBothHeadsToBeNull) {
  ModelFixture f;
  Body(f, 1, 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = TraverseCompoundBodyBoundary(model.value(), NodeId{1});
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().boundary.regions.empty());
  EXPECT_TRUE(result.value().children.empty());
  Body(f, 3, 0, 1);
  Set(f, 1, "child", 3);
  const auto invalid = ModelBuilder::Build(f.report);
  ASSERT_TRUE(invalid.ok());
  const auto child = TraverseCompoundBodyBoundary(invalid.value(), NodeId{1});
  ASSERT_FALSE(child.ok());
  EXPECT_EQ(child.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidCompoundTopology, CommonAndMixedChildKindsDetermineParentKind) {
  for (bool mixed : {false, true}) {
    auto f = CompoundFixture();
    if (!mixed) Set(f, 4, "body_type", 1);
    Set(f, 1, "body_type", mixed ? 6 : 1);
    auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_TRUE(TraverseCompoundBodyBoundary(model.value(), NodeId{1}).ok());
    Set(f, 1, "body_type", mixed ? 1 : 6);
    model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto bad = TraverseCompoundBodyBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(bad.ok());
    EXPECT_EQ(bad.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidCompoundTopology,
     ListAndOwnershipFailuresNeverReturnPartialData) {
  for (int bad = 0; bad < 20; ++bad) {
    auto f = CompoundFixture();
    if (bad == 0) Set(f, 3, "next", 3);
    if (bad == 1) Set(f, 4, "next", 3);
    if (bad == 2) Set(f, 4, "previous", 0);
    if (bad == 3) Set(f, 3, "previous", 4);
    if (bad == 4) Set(f, 3, "owner", 4);
    if (bad == 5) Set(f, 3, "child", 4);
    if (bad == 6) Set(f, 3, "region", 11);
    if (bad == 7) Set(f, 4, "region", 0);
    if (bad == 8) Set(f, 4, "region", 11);
    if (bad == 9) Set(f, 11, "owner", 4);
    if (bad == 10) Set(f, 12, "owner", 3);
    if (bad == 11) Set(f, 11, "owner", 0);
    if (bad == 12) Set(f, 11, "owner", 999);
    if (bad == 13) Set(f, 1, "child", 0);
    if (bad == 14) Set(f, 1, "child", 100);
    if (bad == 15) Set(f, 3, "next", 999);
    if (bad == 16) Set(f, 3, "next", 100);
    if (bad == 17) Set(f, 1, "child", 1);
    if (bad == 18) Set(f, 10, "owner", 4);
    if (bad == 19) Set(f, 3, "next", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        TraverseCompoundBodyBoundary(model.value(), NodeId{1}, {kExact, 2});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord) << bad;
    if (bad == 0 || bad == 1 || bad == 17)
      EXPECT_NE(result.error().message.find("cycle"), std::string::npos);
  }
}
TEST(ParasolidCompoundTopology,
     MissingUnverifiedAndUnknownChildDataStayUnsupported) {
  for (int bad = 0; bad < 10; ++bad) {
    auto f = CompoundFixture();
    if (bad == 9)
      Set(f, 3, "body_type", 99);
    else {
      const uint32_t id = bad == 0 ? 1 : bad == 7 ? 10 : 3;
      const std::string role = bad == 0 || bad == 4   ? "child"
                               : bad == 1 || bad == 7 ? "owner"
                               : bad == 2             ? "previous"
                               : bad == 3 || bad == 8 ? "next"
                               : bad == 5             ? "region"
                                                      : "body_type";
      for (auto& node : f.report.nodes)
        if (node.index == id)
          for (auto& field :
               f.report.schemas[node.schema_index].definition.fields)
            if (field.name == role) {
              if (bad == 8)
                field.pointer_class = 60000;
              else
                field.name = "unavailable";
            }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    // The baseline boundary query does not require child grouping metadata.
    EXPECT_TRUE(TraverseBodyBoundary(model.value(), NodeId{1}).ok());
    const auto result = TraverseCompoundBodyBoundary(model.value(), NodeId{1});
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat) << bad;
  }
}
TEST(ParasolidCompoundTopology, ChildAndNestedBodyLimitsRemainIndependent) {
  const auto f = CompoundFixture();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto limited =
      TraverseCompoundBodyBoundary(model.value(), NodeId{1}, {kExact, 1});
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  auto options = CompoundBodyTraversalOptions{kExact, 2};
  --options.body.max_fins;
  const auto nested =
      TraverseCompoundBodyBoundary(model.value(), NodeId{1}, options);
  ASSERT_FALSE(nested.ok());
  EXPECT_EQ(nested.error().code, ErrorCode::kResourceLimit);
  for (int bad = 0; bad < 7; ++bad) {
    options = {kExact, 2};
    if (bad == 0) options.max_children = 0;
    if (bad == 1) options.body.max_regions = 0;
    if (bad == 2) options.body.max_shells = 0;
    if (bad == 3) options.body.max_face_uses = 0;
    if (bad == 4) options.body.max_loops = 0;
    if (bad == 5) options.body.max_fins = 0;
    if (bad == 6) options.body.max_wire_edges = 0;
    const auto invalid =
        TraverseCompoundBodyBoundary(model.value(), NodeId{999}, options);
    ASSERT_FALSE(invalid.ok());
    EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidCompoundTopology, NestedErrorsRetainBodyContextAndSource) {
  auto f = CompoundFixture();
  Set(f, 102, "loop", 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto body = TraverseBodyBoundary(model.value(), NodeId{1});
  const auto result = TraverseCompoundBodyBoundary(model.value(), NodeId{1});
  ASSERT_FALSE(body.ok());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, body.error().code);
  EXPECT_EQ(result.error().offset, body.error().offset);
  EXPECT_EQ(result.error().message, body.error().message);
}
TEST(ParasolidCompoundTopology, UnlistedChildrenAreOutsideTheLocalQuery) {
  auto f = CompoundFixture();
  Body(f, 999, 0, 1);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(TraverseCompoundBodyBoundary(model.value(), NodeId{1}).ok());
  EXPECT_FALSE(TraverseCompoundBodyBoundary(model.value(), NodeId{3}).ok());
  EXPECT_FALSE(TraverseCompoundBodyBoundary(model.value(), NodeId{10}).ok());
  EXPECT_FALSE(TraverseCompoundBodyBoundary(model.value(), NodeId{888}).ok());
}

}  // namespace
}  // namespace openparasolid
