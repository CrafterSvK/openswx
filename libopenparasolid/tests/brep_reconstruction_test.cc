// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "model_test_helpers.h"
#include "openparasolid/brep.h"
#include "openparasolid/delta.h"
namespace openparasolid {
namespace {
using namespace test;
void Set(ModelFixture& f, uint32_t id, const char* name, uint32_t value) {
  for (auto& n : f.report.nodes)
    if (n.index == id) {
      const auto& defs = f.report.schemas[n.schema_index].definition.fields;
      for (std::size_t i = 0; i < defs.size(); ++i)
        if (defs[i].name == name) {
          n.fields[i].values[0] = uint64_t{value};
          return;
        }
    }
  ADD_FAILURE() << "Missing fixture field";
}
void Fin(ModelFixture& f, uint32_t id, uint32_t vertex, uint32_t other,
         uint32_t loop = 0, uint32_t neighbor = 0) {
  f.Add(17, id,
        {Pointer("vertex", vertex, 18), Pointer("edge", 2, 16),
         Pointer("other", other, 17), Pointer("loop", loop, 15),
         Pointer("forward", neighbor, 17), Pointer("backward", neighbor, 17),
         Pointer("curve", 0, 1008), Pointer("next_at_vx", 0, 17),
         ByteValue("sense", id == 10 ? '+' : '-', FieldType::kCharacter)});
}
void Vertex(ModelFixture& f, uint32_t id, uint32_t head, uint32_t previous,
            uint32_t next, uint32_t point) {
  f.Add(18, id,
        {Pointer("owner", 1, 1029), Pointer("halfedge", head, 17),
         Pointer("previous", previous, 18), Pointer("next", next, 18),
         Pointer("point", point, 29)});
}
ModelFixture Wire() {
  ModelFixture f;
  f.Add(12, 1,
        {Pointer("owner", 0, 1040), Pointer("region", 5, 19),
         Pointer("child", 0, 12), Pointer("edge", 0, 16),
         Pointer("vertex", 100, 18), ByteValue("body_type", 2)});
  f.Add(19, 5,
        {Pointer("body", 1, 12), Pointer("previous", 0, 19),
         Pointer("next", 0, 19), Pointer("shell", 6, 13),
         ByteValue("type", 'V', FieldType::kCharacter)});
  f.Add(13, 6,
        {Pointer("region", 5, 19), Pointer("next", 0, 13),
         Pointer("face", 0, 14), Pointer("front_face", 0, 14),
         Pointer("edge", 2, 16), Pointer("vertex", 0, 18)});
  f.Add(16, 2,
        {Pointer("owner", 6, 1029), Pointer("previous", 0, 16),
         Pointer("next", 0, 16), Pointer("halfedge", 10, 17),
         Pointer("curve", 400, 1008)});
  Fin(f, 10, 101, 11);
  Fin(f, 11, 100, 10);
  Vertex(f, 100, 11, 0, 101, 200);
  Vertex(f, 101, 10, 100, 0, 201);
  f.Add(29, 200, {Numbers("position", {0, 0, 0}, FieldType::kVector)});
  f.Add(29, 201, {Numbers("position", {1, 0, 0}, FieldType::kVector)});
  f.Add(30, 400,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("direction", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  return f;
}
ModelFixture Face() {
  auto f = Wire();
  Set(f, 10, "loop", 20);
  Set(f, 11, "loop", 20);
  Set(f, 10, "forward", 11);
  Set(f, 10, "backward", 11);
  Set(f, 11, "forward", 10);
  Set(f, 11, "backward", 10);
  f.Add(15, 20,
        {Pointer("face", 30, 14), Pointer("halfedge", 10, 17),
         Pointer("next", 0, 15)});
  f.Add(14, 30,
        {Pointer("loop", 20, 15), Pointer("surface", 300, 1006),
         Pointer("shell", 6, 13), Pointer("front_shell", 6, 13),
         Pointer("previous", 0, 14), Pointer("next", 0, 14),
         Pointer("previous_front", 0, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(50, 300,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  return f;
}
TEST(ParasolidBrepValidation, OwnsCompleteWireBodyAndChecksAggregateLimits) {
  const auto result = [] {
    const auto f = Wire();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<ValidatedBodyTopology>::Err(model.error());
    return ValidateBodyTopology(model.value(), NodeId{1});
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().edges.size(), 1);
  EXPECT_EQ(result.value().vertices.size(), 2);
  const auto f = Wire();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 4; ++bad) {
    BodyValidationOptions o;
    if (bad == 0) o.max_model_nodes = 1;
    if (bad == 1) o.max_radial_fins = 1;
    if (bad == 2) o.max_vertex_fins = 1;
    if (bad == 3) o.max_model_references = 1;
    auto r = ValidateBodyTopology(model.value(), NodeId{1}, o);
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error().code, ErrorCode::kResourceLimit);
  }
}
TEST(ParasolidBrepValidation, FindsMissingOwnerListsAndUnlistedIncidence) {
  for (int bad = 0; bad < 7; ++bad) {
    auto f = Wire();
    if (bad == 0) Set(f, 100, "halfedge", 0);
    if (bad == 1) Set(f, 100, "next", 0);
    if (bad == 2) Vertex(f, 102, 0, 0, 0, 200);
    if (bad == 3) Fin(f, 12, 101, 10);
    if (bad == 4) {
      Set(f, 101, "owner", 9);
      f.Add(12, 9, {});
    }
    if (bad == 5) f.Add(17, 12, {Pointer("vertex", 100, 18)});
    if (bad == 6) f.Add(12, 9, {Pointer("owner", 1, 1040)});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_TRUE(TraverseBodyBoundary(model.value(), NodeId{1}).ok());
    const auto r = ValidateBodyTopology(model.value(), NodeId{1});
    ASSERT_FALSE(r.ok()) << bad;
    EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord) << bad;
  }
}
TEST(ParasolidFaceBinding, BindsDirectedIntervalsAndPreservesExactCurves) {
  const auto f = Face();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, p, 1e-12);
  ASSERT_TRUE(face.ok()) << face.error().message;
  ASSERT_EQ(face.value().loops.size(), 1);
  ASSERT_EQ(face.value().loops[0].fins.size(), 2);
  const auto sample = EvaluateBoundFin(
      model.value(), face.value().loops[0].fins[1], 0.25, 1e-12);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().curve.position.x, 0.75, 1e-12);
  EXPECT_NEAR(sample.value().curve.first_derivative.x, -1, 1e-12);
  EXPECT_NEAR(sample.value().uv.u, 0.75, 1e-12);
}

TEST(ParasolidFaceBinding, AutomaticallyBindsLinesAndSourceTrimmedIntervals) {
  for (bool trimmed : {false, true}) {
    auto f = Face();
    if (trimmed) {
      f.Add(133, 401,
            {Pointer("basis_curve", 400, 1008),
             Numbers("point_1", {0, 0, 0}, FieldType::kVector),
             Numbers("point_2", {1, 0, 0}, FieldType::kVector),
             Numbers("parm_1", {0}), Numbers("parm_2", {1}),
             ByteValue("sense", '+', FieldType::kCharacter)});
      Set(f, 2, "curve", 401);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().loops[0].fins[0].start, 0);
    EXPECT_EQ(result.value().loops[0].fins[0].end, 1);
    EXPECT_EQ(result.value().loops[0].fins[1].start, 1);
    EXPECT_EQ(result.value().loops[0].fins[1].end, 0);
  }
}
TEST(ParasolidFaceBinding,
     RejectsMissingDuplicateWrongDirectionAndSurfaceIntervals) {
  const auto f = Face();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 4; ++bad) {
    std::vector<FinParameterInterval> p{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
    if (bad == 0) p.pop_back();
    if (bad == 1) p[1].fin = NodeId{10};
    if (bad == 2) p[0].end = 2;
    if (bad == 3) p[1].fin = NodeId{999};
    EXPECT_FALSE(BindFaceTrimming(model.value(), NodeId{30}, p, 1e-12).ok());
  }
  const FinParameterInterval p[]{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
  EXPECT_FALSE(BindFaceTrimming(model.value(), NodeId{30}, p, -1).ok());
}

TEST(ParasolidBrepValidation, ChecksBothFaceSidesAndCompleteLoopMembership) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = Face();
    Set(f, 6, "edge", 0);
    Set(f, 6, "face", 30);
    Set(f, 6, "front_face", 30);
    Set(f, 2, "owner", 1);
    Set(f, 1, "edge", 2);
    Set(f, 1, "body_type", 3);
    if (bad == 1) Set(f, 6, "front_face", 0);
    if (bad == 2) f.Add(15, 21, {Pointer("face", 30, 14)});
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto r = ValidateBodyTopology(model.value(), NodeId{1});
    if (!bad) {
      ASSERT_TRUE(r.ok()) << r.error().message;
    } else {
      ASSERT_FALSE(r.ok());
      EXPECT_EQ(r.error().code, ErrorCode::kInvalidRecord);
    }
  }
}
void ParameterCurve(ModelFixture& f) {
  f.Add(134, 410,
        {Pointer("nurbs", 411, 136),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(136, 411,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim", 2, FieldType::kShort), Integer("n_knots", 2),
         ByteValue("knot_type", 2), Logical("periodic", false),
         Logical("closed", false), Logical("rational", false),
         ByteValue("curve_form", 1), Pointer("bspline_vertices", 412, 45),
         Pointer("knots", 413, 128), Pointer("knot_mult", 414, 127)});
  f.Add(45, 412, {Array("vertices", FieldType::kDouble, {0., 0., 1., 0.})});
  f.Add(128, 413, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 414,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
}
TEST(ParasolidFaceBinding,
     UsesTransmittedUvAndRejectsAnotherSupportingSurface) {
  for (bool wrong : {false, true}) {
    auto f = Face();
    ParameterCurve(f);
    f.Add(137, 500,
          {Pointer("surface", wrong ? 301 : 300, 1006),
           Pointer("b_curve", 410, 134),
           ByteValue("sense", '+', FieldType::kCharacter)});
    if (wrong)
      f.Add(50, 301,
            {Numbers("origin", {0, 0, 0}, FieldType::kVector),
             Numbers("normal", {0, 0, 1}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             ByteValue("sense", '+', FieldType::kCharacter)});
    Set(f, 10, "curve", 500);
    Set(f, 11, "curve", 500);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const FinParameterInterval p[]{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, p, 1e-12);
    if (wrong) {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    } else {
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto sample = EvaluateBoundFin(
          model.value(), result.value().loops[0].fins[0], 0.4, 1e-12);
      ASSERT_TRUE(sample.ok());
      EXPECT_NEAR(sample.value().uv.u, 0.4, 1e-12);
      EXPECT_NEAR(sample.value().uv.v, 0, 1e-12);
    }
  }
}
TEST(ParasolidFaceBinding, AcceptsExplicitUnwrappedPeriodicInterval) {
  auto f = Face();
  Set(f, 10, "vertex", 0);
  Set(f, 11, "vertex", 0);
  Set(f, 10, "forward", 10);
  Set(f, 10, "backward", 10);
  Set(f, 11, "loop", 0);
  Set(f, 11, "forward", 0);
  Set(f, 11, "backward", 0);
  Set(f, 2, "curve", 401);
  f.Add(
      31, 401,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {1}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{{NodeId{10}, 0, 2 * std::numbers::pi}};
  const auto result = BindFaceTrimming(model.value(), NodeId{30}, p, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto sample = EvaluateBoundFin(
      model.value(), result.value().loops[0].fins[0], 0.25, 1e-12);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().curve.position.x, 0, 1e-12);
  EXPECT_NEAR(sample.value().curve.position.y, 1, 1e-12);
  const FinParameterInterval reversed[]{{NodeId{10}, 2 * std::numbers::pi, 0}};
  EXPECT_FALSE(
      BindFaceTrimming(model.value(), NodeId{30}, reversed, 1e-12).ok());
}
void Offset(ModelFixture& f, uint32_t id, uint32_t basis, double d,
            char sense = '+') {
  f.Add(60, id,
        {Pointer("surface", basis, 1006), Numbers("offset", {d}),
         ByteValue("sense", sense, FieldType::kCharacter)});
}
TEST(ParasolidOffsetSurface,
     AppliesBasisSenseAndNestedOffsetsWithoutChangingUv) {
  auto f = Face();
  Offset(f, 301, 300, 2, '-');
  Offset(f, 302, 301, 0.5);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto r = EvaluateSurface(model.value(), NodeId{302}, 0.25, 0.5);
  ASSERT_TRUE(r.ok()) << r.error().message;
  EXPECT_NEAR(r.value().position.x, 0.25, 1e-12);
  EXPECT_NEAR(r.value().position.y, 0.5, 1e-12);
  EXPECT_NEAR(r.value().position.z, 1.5, 1e-12);
  EXPECT_EQ(r.value().u_derivative.x, 1);
  EXPECT_EQ(r.value().v_derivative.y, 1);
}
TEST(ParasolidOffsetSurface, RejectsCyclesAndRadiusCollapse) {
  auto f = Face();
  Offset(f, 301, 302, 1);
  Offset(f, 302, 301, 1);
  auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{301}, 0, 0).ok());
  f = ModelFixture();
  f.Add(
      53, 1,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {1}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  Offset(f, 2, 1, -2);
  model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{2}, 0, 0).ok());
}

TEST(ParasolidOffsetSurface,
     EvaluatesAndInvertsCylinderSphereConeAndRingTorus) {
  for (uint16_t type : {51, 52, 53, 54}) {
    ModelFixture f;
    std::vector<ModelField> fields{
        Numbers(type == 51 || type == 52 ? "origin" : "center", {0, 0, 0},
                FieldType::kVector),
        Numbers("axis", {0, 0, 1}, FieldType::kVector),
        Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
        ByteValue("sense", '+', FieldType::kCharacter)};
    if (type == 54) {
      fields.push_back(Numbers("major_radius", {5}));
      fields.push_back(Numbers("minor_radius", {1}));
    } else
      fields.push_back(Numbers("radius", {2}));
    if (type == 52) {
      fields.push_back(Numbers("sin_half_angle", {std::sin(0.3)}));
      fields.push_back(Numbers("cos_half_angle", {std::cos(0.3)}));
    }
    f.Add(type, 1, std::move(fields));
    Offset(f, 2, 1, 0.2);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto basis = EvaluateSurface(model.value(), NodeId{1}, 0.4, 0.2);
    const auto normal =
        EvaluateSurfaceNormal(model.value(), NodeId{1}, 0.4, 0.2);
    const auto offset = EvaluateSurface(model.value(), NodeId{2}, 0.4, 0.2);
    ASSERT_TRUE(basis.ok());
    ASSERT_TRUE(normal.ok());
    ASSERT_TRUE(offset.ok()) << offset.error().message;
    const auto a = basis.value().position, b = offset.value().position,
               n = normal.value();
    EXPECT_NEAR(b.x, a.x + 0.2 * n.x, 1e-12);
    EXPECT_NEAR(b.y, a.y + 0.2 * n.y, 1e-12);
    EXPECT_NEAR(b.z, a.z + 0.2 * n.z, 1e-12);
    const auto uv = InvertSurface(model.value(), NodeId{2}, b, 1e-11);
    ASSERT_TRUE(uv.ok());
    EXPECT_NEAR(uv.value().u, 0.4, 1e-10);
    EXPECT_NEAR(uv.value().v, 0.2, 1e-10);
    if (type == 52) {
      EXPECT_FALSE(EvaluateSurface(model.value(), NodeId{2}, 0, -7).ok());
      ConeGeometry extended{{0, 0, -0.2 * std::sin(0.3)},
                            {0, 0, 1},
                            2 + 0.2 * std::cos(0.3),
                            std::sin(0.3),
                            std::cos(0.3),
                            {1, 0, 0}};
      const auto outside = EvaluateSurface(GeometryPayload{extended}, 0, -6.6);
      ASSERT_TRUE(outside.ok());
      EXPECT_FALSE(InvertSurface(model.value(), NodeId{2},
                                 outside.value().position, 1e-11)
                       .ok());
    }
    const double h = 1e-5;
    const auto plus = EvaluateSurface(model.value(), NodeId{2}, 0.4 + h, 0.2);
    const auto minus = EvaluateSurface(model.value(), NodeId{2}, 0.4 - h, 0.2);
    ASSERT_TRUE(plus.ok());
    ASSERT_TRUE(minus.ok());
    EXPECT_NEAR(offset.value().u_derivative.x,
                (plus.value().position.x - minus.value().position.x) / (2 * h),
                1e-8);
  }
}
SchemaParseReport Delta() {
  ModelFixture f;
  f.Add(74, 7, {Pointer("entry", 9, 29), Integer("word", -1)});
  f.Add(29, 8, {Numbers("position", {2, 3, 4}, FieldType::kVector)});
  auto r = f.report;
  r.header.kind = StreamKind::kDeltas;
  r.delta_prefix = DeltaPrefix{};
  r.delta_nodes = std::move(r.nodes);
  r.nodes.clear();
  for (auto& n : r.delta_nodes) {
    n.status_framed = true;
    for (std::size_t i = 0; i < n.fields.size(); ++i) {
      if (r.schemas[n.schema_index].definition.fields[i].type ==
          FieldType::kPointer)
        n.fields[i].reference_statuses = {1};
    }
  }
  r.delta_nodes[0].fields[1].values[0] = uint64_t{UINT32_MAX};
  return r;
}
TEST(ParasolidDeltaApply,
     AppliesCreateReplaceDeleteAndRemapsStatusQualifiedIds) {
  ModelFixture f;
  f.Point(1);
  f.Point(2);
  const auto base = CreateDeltaState(f.report, 10);
  ASSERT_TRUE(base.ok());
  const auto delta = Delta();
  DeltaApplicationPlan plan{10,
                            11,
                            {{DeltaOperationKind::kErase, NodeId{2}, 0},
                             {DeltaOperationKind::kCreate, NodeId{3}, 0},
                             {DeltaOperationKind::kReplace, NodeId{1}, 1}},
                            {{9, 1, NodeId{1}}}};
  const auto applied = ApplyDelta(base.value(), delta, plan);
  ASSERT_TRUE(applied.ok()) << applied.error().message;
  const auto model = ModelBuilder::Build(applied.value().snapshot);
  ASSERT_TRUE(model.ok());
  EXPECT_EQ(model.value().nodes().size(), 2);
  EXPECT_TRUE(model.value().references_verified());
  EXPECT_EQ(model.value().FindNode(NodeId{2}), nullptr);
  EXPECT_EQ(model.value().FindNode(NodeId{3})->references[0].target, NodeId{1});
  EXPECT_EQ(
      std::get<PointGeometry>(model.value().geometry()[0].payload).position.x,
      2);
  EXPECT_EQ(
      std::get<int64_t>(applied.value().snapshot.nodes[1].fields[1].values[0]),
      -1);
  EXPECT_EQ(base.value().snapshot.nodes.size(), 2);
  EXPECT_EQ(base.value().revision, 10);
}
TEST(ParasolidDeltaApply, FailsAtomicallyOnStaleAmbiguousAndDanglingChanges) {
  ModelFixture f;
  f.Point(1);
  f.Point(2);
  const auto base = CreateDeltaState(f.report, 10);
  ASSERT_TRUE(base.ok());
  for (int bad = 0; bad < 13; ++bad) {
    auto delta = Delta();
    DeltaApplicationPlan plan{10,
                              11,
                              {{DeltaOperationKind::kCreate, NodeId{3}, 0}},
                              {{9, 1, NodeId{1}}}};
    if (bad == 0) plan.base_revision = 9;
    if (bad == 1) plan.references.clear();
    if (bad == 2) plan.references.push_back(plan.references[0]);
    if (bad == 3)
      plan.operations.push_back({DeltaOperationKind::kErase, NodeId{1}, 0});
    if (bad == 4) delta.delta_nodes[0].fields[0].reference_statuses = {2};
    if (bad == 5) plan.operations.push_back(plan.operations[0]);
    if (bad == 6) delta.complete = false;
    if (bad == 7) plan.operations[0].occurrence = 999;
    if (bad == 8)
      plan.operations.push_back({DeltaOperationKind::kCreate, NodeId{4}, 0});
    if (bad == 9)
      plan.operations[0] = {DeltaOperationKind::kReplace, NodeId{1}, 0};
    if (bad == 10) delta.delta_nodes[0].fields[0].elements = 2;
    if (bad == 11) delta.delta_nodes[0].fields[0].offset = UINT64_MAX;
    if (bad == 12) delta.delta_nodes[0].status_framed = false;
    EXPECT_FALSE(ApplyDelta(base.value(), delta, plan).ok()) << bad;
    EXPECT_EQ(base.value().snapshot.nodes.size(), 2);
    EXPECT_EQ(base.value().revision, 10);
  }
}
TEST(ParasolidDeltaApply, SeparatesStatusNamespacesAndEnforcesStateLimits) {
  ModelFixture f;
  f.Point(1);
  f.Point(2);
  const auto base = CreateDeltaState(f.report, 10);
  ASSERT_TRUE(base.ok());
  ModelFixture source;
  auto array = Array("entry", FieldType::kPointer, {uint64_t{9}, uint64_t{9}});
  array.definition.pointer_class = 29;
  source.Add(74, 7, {array});
  auto delta = source.report;
  delta.header.kind = StreamKind::kDeltas;
  delta.delta_prefix = DeltaPrefix{};
  delta.delta_nodes = std::move(delta.nodes);
  delta.nodes.clear();
  delta.delta_nodes[0].status_framed = true;
  delta.delta_nodes[0].fields[0].reference_statuses = {0, 1};
  DeltaApplicationPlan plan{10,
                            11,
                            {{DeltaOperationKind::kCreate, NodeId{3}, 0}},
                            {{9, 0, NodeId{1}}, {9, 1, NodeId{2}}}};
  const auto result = ApplyDelta(base.value(), delta, plan);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto model = ModelBuilder::Build(result.value().snapshot);
  ASSERT_TRUE(model.ok());
  const auto* node = model.value().FindNode(NodeId{3});
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->references[0].target, NodeId{1});
  EXPECT_EQ(node->references[1].target, NodeId{2});
  ModelOptions limits;
  limits.max_nodes = 2;
  const auto limited = ApplyDelta(base.value(), delta, plan, limits);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  limits.max_values = 0;
  const auto invalid = ApplyDelta(base.value(), delta, plan, limits);
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
}

}  // namespace
}  // namespace openparasolid
