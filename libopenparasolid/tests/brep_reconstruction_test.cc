// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
ModelFixture IntersectionFace(bool trimmed, bool curved = false,
                              const std::vector<Vector3>& chart_points = {}) {
  auto f = Face();
  const Vector3 a = curved ? Vector3{1, 0, 0} : Vector3{0, 0, 0};
  const Vector3 b = curved ? Vector3{0, 1, 0} : Vector3{1, 0, 0};
  for (auto& n : f.report.nodes) {
    if (n.index == 200) n.fields[0].values = {a.x, a.y, a.z};
    if (n.index == 201) n.fields[0].values = {b.x, b.y, b.z};
  }
  if (curved) {
    f.Add(53, 301,
          {Numbers("center", {0, 0, 0}, FieldType::kVector),
           Numbers("axis", {0, 0, 1}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           Numbers("radius", {1}),
           ByteValue("sense", '+', FieldType::kCharacter)});
  } else {
    f.Add(50, 301,
          {Numbers("origin", {0, 0, 0}, FieldType::kVector),
           Numbers("normal", {0, -1, 0}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
  }
  f.Add(38, 401,
        {{{"surface", FieldType::kPointer, 1006, 2, true},
          {uint64_t{300}, uint64_t{301}}},
         Pointer("chart", 402, 40),
         Pointer("start", 403, 41),
         Pointer("end", 404, 41),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto points =
      chart_points.empty() ? std::vector<Vector3>{a, b} : chart_points;
  std::vector<ScalarValue> positions;
  for (const auto point : points)
    for (double value : {point.x, point.y, point.z})
      positions.emplace_back(value);
  f.Add(40, 402,
        {Numbers("base_parameter", {0}),
         Numbers("base_scale", {1}),
         Integer("chart_count", static_cast<int64_t>(points.size())),
         Numbers("chordal_error", {.1}),
         Numbers("angular_error", {1}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               std::move(positions))});
  f.Add(41, 403,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {a.x, a.y, a.z})});
  f.Add(41, 404,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {b.x, b.y, b.z})});
  if (trimmed) {
    f.Add(133, 410,
          {Pointer("basis_curve", 401, 1008),
           Numbers("point_1", {a.x, a.y, a.z}, FieldType::kVector),
           Numbers("point_2", {b.x, b.y, b.z}, FieldType::kVector),
           Numbers("parm_1", {0}),
           Numbers("parm_2", {curved ? std::sqrt(2.) : 1}),
           ByteValue("sense", '+', FieldType::kCharacter)});
  }
  Set(f, 2, "curve", trimmed ? 410 : 401);
  return f;
}
TEST(ParasolidFaceBinding, BindsIntersectionSolvesAndOwnsSolverSettings) {
  for (bool curved : {false, true}) {
    const auto f = IntersectionFace(false, curved);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const double end = curved ? std::sqrt(2.) : 1;
    const FinParameterInterval intervals[]{{NodeId{10}, 0, end},
                                           {NodeId{11}, end, 0}};
    const auto no_options =
        BindFaceTrimming(model.value(), NodeId{30}, intervals, 1e-12);
    ASSERT_FALSE(no_options.ok());
    EXPECT_EQ(no_options.error().code, ErrorCode::kUnsupportedFormat);
    auto options = IntersectionEvaluationOptions{1e-12, 1e-12};
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, intervals,
                                         1e-12, {}, options);
    ASSERT_TRUE(result.ok()) << result.error().message;
    options.solve_tolerance = 0;
    for (std::size_t i = 0; i < 2; ++i) {
      const auto& fin = result.value().loops[0].fins[i];
      ASSERT_TRUE(fin.intersection.has_value());
      EXPECT_EQ(fin.intersection->solve_tolerance, 1e-12);
      const auto sample = EvaluateBoundFin(model.value(), fin, .5, 1e-12);
      ASSERT_TRUE(sample.ok()) << sample.error().message;
      const double coordinate = curved ? std::sqrt(.5) : .5;
      EXPECT_NEAR(sample.value().curve.position.x, coordinate, 1e-12);
      EXPECT_NEAR(sample.value().curve.position.y, curved ? coordinate : 0,
                  1e-12);
      EXPECT_NEAR(sample.value().uv.u, coordinate, 1e-12);
      const double sign = i == 0 ? 1 : -1;
      EXPECT_NEAR(sample.value().curve.first_derivative.x,
                  curved ? -sign : sign, 1e-12);
      EXPECT_NEAR(sample.value().curve.first_derivative.y, curved ? sign : 0,
                  1e-12);
    }
  }
}
TEST(ParasolidChartFinIntervals, BindsDirectIntersectionsInBothFinDirections) {
  for (bool curved : {false, true}) {
    const auto f = IntersectionFace(false, curved);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto no_options = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
    ASSERT_FALSE(no_options.ok());
    EXPECT_EQ(no_options.error().code, ErrorCode::kUnsupportedFormat);
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    ASSERT_TRUE(result.ok()) << result.error().message;
    const auto& fins = result.value().loops[0].fins;
    const double end = curved ? std::sqrt(2.) : 1;
    EXPECT_EQ(fins[0].start, 0);
    EXPECT_DOUBLE_EQ(fins[0].end, end);
    EXPECT_DOUBLE_EQ(fins[1].start, end);
    EXPECT_EQ(fins[1].end, 0);
    const auto midpoint = EvaluateBoundFin(model.value(), fins[0], .5, 1e-12);
    ASSERT_TRUE(midpoint.ok());
    EXPECT_NEAR(midpoint.value().curve.position.x, curved ? std::sqrt(.5) : .5,
                1e-12);
  }
}
TEST(ParasolidChartFinIntervals,
     SelectsInteriorChartPointsInOriginalParameterUnits) {
  auto f = IntersectionFace(
      false, false,
      {{0, 0, 0}, {.25, 0, 0}, {.5, 0, 0}, {.75, 0, 0}, {1, 0, 0}});
  for (auto& n : f.report.nodes) {
    if (n.index == 200) n.fields[0].values = {.25, 0., 0.};
    if (n.index == 201) n.fields[0].values = {.75, 0., 0.};
    if (n.index == 402) {
      n.fields[0].values[0] = -4.;
      n.fields[1].values[0] = 2.5;
    }
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                       IntersectionEvaluationOptions{1e-12, 1e-12});
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& fin = result.value().loops[0].fins[0];
  EXPECT_DOUBLE_EQ(fin.start, -3.375);
  EXPECT_DOUBLE_EQ(fin.end, -2.125);
  const auto midpoint = EvaluateBoundFin(model.value(), fin, .5, 1e-12);
  ASSERT_TRUE(midpoint.ok());
  EXPECT_NEAR(midpoint.value().curve.position.x, .5, 1e-12);
  EXPECT_NEAR(midpoint.value().curve.first_derivative.x, .5, 1e-12);
}
TEST(ParasolidChartFinIntervals,
     RecoversUnstoredPointsAndRejectsAmbiguousOrCollapsedCorrespondence) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = IntersectionFace(false);
    for (auto& n : f.report.nodes) {
      if (n.index == 200 && bad == 0) n.fields[0].values = {.25, 0., 0.};
      if (n.index == 200 && bad == 2) n.fields[0].values = {.1, 0., 0.};
      if (n.index == 201 && bad == 2) n.fields[0].values = {.2, 0., 0.};
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30},
                         bad == 0   ? 1e-12
                         : bad == 1 ? 1
                                    : .25,
                         {}, IntersectionEvaluationOptions{1e-12, 1e-12});
    if (bad == 0) {
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_NEAR(result.value().loops[0].fins[0].start, .25, 1e-12);
    } else {
      ASSERT_FALSE(result.ok()) << bad;
      EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
    }
  }
}
TEST(ParasolidChartFinIntervals,
     HonorsEdgeSenseAndRejectsUnknownOrContradictoryOrder) {
  for (int mode = 0; mode < 3; ++mode) {
    auto f = IntersectionFace(false);
    Set(f, 401, "sense", mode == 0 ? '?' : '-');
    if (mode == 2) {
      Set(f, 10, "vertex", 100);
      Set(f, 11, "vertex", 101);
      Set(f, 100, "halfedge", 10);
      Set(f, 101, "halfedge", 11);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    if (mode == 2) {
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().loops[0].fins[0].start, 1);
      EXPECT_EQ(result.value().loops[0].fins[0].end, 0);
    } else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, mode == 0 ? ErrorCode::kUnsupportedFormat
                                               : ErrorCode::kInvalidRecord);
    }
  }
}
TEST(ParasolidChartFinIntervals,
     FinCurveUsesUniqueOrientedVertexCorrespondence) {
  auto f = IntersectionFace(false);
  Set(f, 2, "curve", 400);
  Set(f, 10, "curve", 401);
  Set(f, 401, "sense", '?');
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                       IntersectionEvaluationOptions{1e-12, 1e-12});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().loops[0].fins[0].curve.value, 401);
  EXPECT_EQ(result.value().loops[0].fins[0].start, 0);
  EXPECT_EQ(result.value().loops[0].fins[0].end, 1);
  EXPECT_EQ(result.value().loops[0].fins[1].curve.value, 400);
}
TEST(ParasolidChartFinIntervals,
     ClosedAndVertexFreeEdgesStillNeedExplicitIntervals) {
  for (bool vertex_free : {false, true}) {
    auto f = IntersectionFace(false);
    Set(f, 10, "vertex", vertex_free ? 0 : 100);
    Set(f, 11, "vertex", vertex_free ? 0 : 100);
    Set(f, 10, "forward", 10);
    Set(f, 10, "backward", 10);
    Set(f, 11, "loop", 0);
    Set(f, 11, "forward", 0);
    Set(f, 11, "backward", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}
ModelFixture HelpClosedIntersectionFace(bool vertex_free, bool negative_basis,
                                        bool reverse_fin) {
  constexpr std::size_t kSegments = 16;
  std::vector<Vector3> points;
  for (std::size_t i = 0; i < kSegments; ++i) {
    const double angle = 2 * std::numbers::pi * static_cast<double>(i) /
                         static_cast<double>(kSegments);
    points.push_back({std::cos(angle), std::sin(angle), 0});
  }
  points.push_back(points.front());
  auto f = IntersectionFace(false, true, points);
  Set(f, 401, "end", 403);
  Set(f, 403, "type", 'H');
  Set(f, 401, "sense", negative_basis ? '-' : '+');
  Set(f, 10, "sense", reverse_fin ? '-' : '+');
  Set(f, 11, "sense", reverse_fin ? '+' : '-');
  Set(f, 2, "halfedge", reverse_fin ? 11 : 10);
  Set(f, 10, "vertex", vertex_free ? 0 : 100);
  Set(f, 11, "vertex", vertex_free ? 0 : 100);
  Set(f, 10, "forward", 10);
  Set(f, 10, "backward", 10);
  Set(f, 11, "loop", 0);
  Set(f, 11, "forward", 0);
  Set(f, 11, "backward", 0);
  for (auto& node : f.report.nodes) {
    if (node.index != 402) continue;
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < fields.size(); ++i) {
      if (fields[i].name == "base_parameter") node.fields[i].values[0] = 5.25;
      if (fields[i].name == "base_scale") node.fields[i].values[0] = 2.4;
    }
  }
  return f;
}
TEST(ParasolidChartFinIntervals, BindsCompleteHelpClosedChartInSourceUnits) {
  for (bool vertex_free : {false, true}) {
    for (bool negative : {false, true}) {
      for (bool reverse : {false, true}) {
        const auto fixture =
            HelpClosedIntersectionFace(vertex_free, negative, reverse);
        const auto model = ModelBuilder::Build(fixture.report);
        ASSERT_TRUE(model.ok()) << model.error().message;
        const auto result =
            BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                             IntersectionEvaluationOptions{1e-12, 1e-12});
        ASSERT_TRUE(result.ok()) << result.error().message;
        const auto& fin = result.value().loops[0].fins[0];
        const double start = 5.25;
        const double end =
            start + 16 * 2 * std::sin(std::numbers::pi / 16) * 2.4;
        const bool increasing = negative == reverse;
        EXPECT_NEAR(fin.start, increasing ? start : end, 1e-12);
        EXPECT_NEAR(fin.end, increasing ? end : start, 1e-12);
        for (double fraction : {0., .25, .5, .75, 1.}) {
          const auto sample =
              EvaluateBoundFin(model.value(), fin, fraction, 1e-12);
          ASSERT_TRUE(sample.ok()) << sample.error().message;
          const double angle =
              (increasing ? 1 : -1) * 2 * std::numbers::pi * fraction;
          EXPECT_NEAR(sample.value().curve.position.x, std::cos(angle), 1e-12);
          EXPECT_NEAR(sample.value().curve.position.y, std::sin(angle), 1e-12);
          EXPECT_NEAR(sample.value().curve.position.z, 0, 1e-12);
        }
      }
    }
  }
}
TEST(ParasolidChartFinIntervals, BindsPartialEdgesEndingAtProvenHelpSeams) {
  for (const bool negative : {false, true})
    for (const bool reverse : {false, true})
      for (const bool seam_at_start : {false, true}) {
        auto f = HelpClosedIntersectionFace(false, negative, reverse);
        Set(f, 10, "forward", 11);
        Set(f, 10, "backward", 11);
        Set(f, 11, "loop", 20);
        Set(f, 11, "forward", 10);
        Set(f, 11, "backward", 10);
        Set(f, 10, "vertex", seam_at_start ? 101 : 100);
        Set(f, 11, "vertex", seam_at_start ? 100 : 101);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto result =
            BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                             IntersectionEvaluationOptions{1e-12, 1e-12});
        ASSERT_TRUE(result.ok()) << result.error().message;
        const auto& fins = result.value().loops.front().fins;
        ASSERT_EQ(fins.size(), 2u);
        for (const auto& fin : fins) {
          EXPECT_EQ(fin.end > fin.start,
                    negative == (fin.use.sense == TopologySense::kReverse));
          for (const double fraction : {0., .5, 1.}) {
            const auto sample =
                EvaluateBoundFin(model.value(), fin, fraction, 1e-12);
            ASSERT_TRUE(sample.ok()) << sample.error().message;
            EXPECT_NEAR(std::hypot(sample.value().curve.position.x,
                                   sample.value().curve.position.y),
                        1, 1e-12);
          }
        }
      }
}
TEST(ParasolidChartFinIntervals, RejectsUnprovenClosedChartPeriods) {
  for (int mode = 0; mode < 7; ++mode) {
    auto fixture = HelpClosedIntersectionFace(true, false, false);
    if (mode == 0) Set(fixture, 401, "end", 404);
    if (mode == 1) Set(fixture, 403, "type", 'L');
    if (mode == 2) Set(fixture, 401, "sense", '?');
    if (mode == 3) Set(fixture, 10, "curve", 401);
    for (auto& node : fixture.report.nodes) {
      if (node.index != 402) continue;
      const auto& fields =
          fixture.report.schemas[node.schema_index].definition.fields;
      for (std::size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name != "positions") continue;
        auto& field = node.fields[i];
        if (mode == 4) field.values[field.values.size() - 3] = .99;
        if (mode == 5) {
          field.values[3] = 1.;
          field.values[4] = 0.;
          field.values[5] = 0.;
        }
        if (mode == 6) field.values[2] = .01;
      }
    }
    const auto model = ModelBuilder::Build(fixture.report);
    ASSERT_TRUE(model.ok()) << model.error().message;
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    ASSERT_FALSE(result.ok()) << mode;
  }
}
TEST(ParasolidChordFinIntervals,
     RecoversCurvedInteriorPointsInOriginalChartUnits) {
  auto f = IntersectionFace(false, true);
  for (auto& n : f.report.nodes) {
    if (n.index == 200) n.fields[0].values = {std::cos(.2), std::sin(.2), 0.};
    if (n.index == 201) n.fields[0].values = {std::cos(.9), std::sin(.9), 0.};
    if (n.index == 402) {
      n.fields[0].values[0] = -4.;
      n.fields[1].values[0] = 2.5;
    }
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-10, {},
                       IntersectionEvaluationOptions{1e-12, 1e-13});
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto parameter = [](double angle) {
    return -4 + 2.5 * (1 - std::cos(angle) + std::sin(angle)) / std::sqrt(2.);
  };
  const auto& fins = result.value().loops[0].fins;
  EXPECT_NEAR(fins[0].start, parameter(.2), 1e-12);
  EXPECT_NEAR(fins[0].end, parameter(.9), 1e-12);
  EXPECT_NEAR(fins[1].start, fins[0].end, 1e-12);
  EXPECT_NEAR(fins[1].end, fins[0].start, 1e-12);
  const auto sample = EvaluateBoundFin(model.value(), fins[0], 0, 1e-10);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().curve.position.x, std::cos(.2), 1e-10);
  EXPECT_NEAR(sample.value().curve.position.y, std::sin(.2), 1e-10);
}
TEST(ParasolidChordFinIntervals,
     ProjectionRequiresSolvedMembershipAndStaysInsideChart) {
  for (bool outside : {false, true}) {
    auto f = IntersectionFace(false);
    for (auto& n : f.report.nodes)
      if (n.index == 200)
        n.fields[0].values = outside ? std::vector<ScalarValue>{-1., 0., 0.}
                                     : std::vector<ScalarValue>{.25, 0., .01};
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-8, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidChordFinIntervals,
     RejectsMultipleSolvedParametersOnRepeatedBranches) {
  std::vector<Vector3> points;
  for (int i = 0; i <= 30; ++i) {
    const double angle = i * .3;
    points.push_back({std::cos(angle), std::sin(angle), 0});
  }
  auto f = IntersectionFace(false, true, points);
  for (auto& n : f.report.nodes)
    if (n.index == 200) n.fields[0].values = {std::cos(.15), std::sin(.15), 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-10, {},
                       IntersectionEvaluationOptions{1e-12, 1e-13});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("Ambiguous solved"), std::string::npos);
}
TEST(ParasolidChordFinIntervals,
     RecoveryQueryBudgetIsFiniteEvenWithoutAnyMatch) {
  std::vector<Vector3> points;
  for (int i = 0; i <= 70; ++i) {
    const double angle = i * .01;
    points.push_back({std::cos(angle), std::sin(angle), 0});
  }
  auto f = IntersectionFace(false, true, points);
  for (auto& n : f.report.nodes)
    if (n.index == 200) n.fields[0].values = {0., 0., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-10, {},
                       IntersectionEvaluationOptions{1e-12, 1e-13});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidFaceBinding, AutomaticallyBindsTrimmedIntersectionsWithOptions) {
  for (bool curved : {false, true}) {
    const auto f = IntersectionFace(true, curved);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_FALSE(BindFaceTrimming(model.value(), NodeId{30}, 1e-12).ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {},
                         IntersectionEvaluationOptions{1e-12, 1e-12});
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().loops[0].fins[0].curve.value, 410);
    EXPECT_DOUBLE_EQ(result.value().loops[0].fins[0].end,
                     curved ? std::sqrt(2.) : 1);
  }
}
TEST(ParasolidFaceBinding,
     IntersectionSettingsDoNotRelaxFaceMembershipOrChartBounds) {
  for (bool outside_chart : {false, true}) {
    auto f = IntersectionFace(false);
    if (!outside_chart) {
      for (auto& n : f.report.nodes)
        if (n.index == 402) {
          n.fields.back().values[2] = 1e-6;
          n.fields.back().values[5] = 1e-6;
        }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const double end = outside_chart ? 1.1 : 1;
    const FinParameterInterval intervals[]{{NodeId{10}, 0, end},
                                           {NodeId{11}, end, 0}};
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, intervals, 1e-12, {},
                         IntersectionEvaluationOptions{1e-3, 1e-3});
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, outside_chart ? ErrorCode::kInvalidOptions
                                                 : ErrorCode::kInvalidRecord);
  }
}
TEST(ParasolidFaceBinding,
     ValidatesIntersectionOptionsEvenWithoutCurveSamples) {
  auto f = Face();
  Set(f, 30, "loop", 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const IntersectionEvaluationOptions invalid;
  const auto automatic =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-12, {}, invalid);
  ASSERT_FALSE(automatic.ok());
  EXPECT_EQ(automatic.error().code, ErrorCode::kInvalidOptions);
  const auto explicit_intervals = BindFaceTrimming(
      model.value(), NodeId{30}, std::span<const FinParameterInterval>{}, 1e-12,
      {}, invalid);
  ASSERT_FALSE(explicit_intervals.ok());
  EXPECT_EQ(explicit_intervals.error().code, ErrorCode::kInvalidOptions);
}
ModelFixture ConicFace(bool ellipse, char sense = '+') {
  auto f = Face();
  Set(f, 2, "curve", 401);
  std::vector<ModelField> fields{
      Numbers("center", {2, -3, 0}, FieldType::kVector),
      Numbers("normal", {0, 0, -1}, FieldType::kVector),
      Numbers("x_axis", {0, 1, 0}, FieldType::kVector),
      ByteValue("sense", sense, FieldType::kCharacter)};
  if (ellipse) {
    fields.push_back(Numbers("major_radius", {3}));
    fields.push_back(Numbers("minor_radius", {2}));
  } else {
    fields.push_back(Numbers("radius", {3}));
  }
  f.Add(ellipse ? 32 : 31, 401, std::move(fields));
  for (auto& n : f.report.nodes) {
    if (n.index == 200 || n.index == 201) {
      const double angle = n.index == 200 ? -0.25 : 0.25;
      n.fields[0].values = {2 + (ellipse ? 2 : 3) * std::sin(angle),
                            -3 + 3 * std::cos(angle), 0.};
    }
  }
  return f;
}
TEST(ParasolidFaceBinding, AutomaticallyBindsOrientedConicsAcrossSeams) {
  for (bool ellipse : {false, true}) {
    for (char sense : {'+', '-'}) {
      const auto f = ConicFace(ellipse, sense);
      const auto model = ModelBuilder::Build(f.report);
      ASSERT_TRUE(model.ok());
      const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto& fins = result.value().loops[0].fins;
      ASSERT_EQ(fins.size(), 2);
      const double span = sense == '+' ? 0.5 : 0.5 - 2 * std::numbers::pi;
      EXPECT_NEAR(fins[0].start, -0.25, 1e-12);
      EXPECT_NEAR(fins[0].end - fins[0].start, span, 1e-12);
      EXPECT_NEAR(fins[1].end - fins[1].start, -span, 1e-12);
      const auto midpoint =
          EvaluateBoundFin(model.value(), fins[0], 0.5, 1e-12);
      ASSERT_TRUE(midpoint.ok());
      EXPECT_NEAR(midpoint.value().curve.position.x, 2, 1e-12);
      EXPECT_NEAR(midpoint.value().curve.position.y, sense == '+' ? 0 : -6,
                  1e-12);
      EXPECT_NEAR(midpoint.value().curve.first_derivative.x,
                  (ellipse ? 2 : 3) * span * (sense == '+' ? 1 : -1), 1e-12);
    }
  }
}
TEST(ParasolidFaceBinding, AutomaticallyBindsClosedAndVertexFreeConics) {
  for (bool ellipse : {false, true}) {
    for (bool vertex_free : {false, true}) {
      for (char sense : {'+', '-'}) {
        auto f = ConicFace(ellipse, sense);
        Set(f, 10, "vertex", vertex_free ? 0 : 100);
        Set(f, 11, "vertex", vertex_free ? 0 : 100);
        Set(f, 10, "forward", 10);
        Set(f, 10, "backward", 10);
        Set(f, 11, "loop", 0);
        Set(f, 11, "forward", 0);
        Set(f, 11, "backward", 0);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
        ASSERT_TRUE(result.ok()) << result.error().message;
        const auto& fin = result.value().loops[0].fins[0];
        EXPECT_NEAR(fin.start, vertex_free ? 0 : -0.25, 1e-12);
        EXPECT_NEAR(fin.end - fin.start,
                    (sense == '+' ? 2 : -2) * std::numbers::pi, 1e-12);
      }
    }
  }
}
TEST(ParasolidFaceBinding, BindsIsolatedConeApexWithinExplicitTolerance) {
  auto f = ConicFace(false);
  Set(f, 30, "surface", 301);
  f.Add(
      52, 301,
      {Numbers("origin", {2, -3, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {3}),
       Numbers("sin_half_angle", {.6}), Numbers("cos_half_angle", {.8}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 20, "next", 21);
  f.Add(15, 21,
        {Pointer("face", 30, 14), Pointer("halfedge", 12, 17),
         Pointer("next", 0, 15)});
  Fin(f, 12, 102, 0, 21, 12);
  Set(f, 12, "edge", 0);
  Vertex(f, 102, 12, 0, 0, 202);
  const double outside = std::nextafter(-4., -5.);
  f.Add(29, 202, {Numbers("position", {2, -3, outside}, FieldType::kVector)});
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().loops.size(), 2);
  ASSERT_TRUE(result.value().loops[1].isolated_vertex.has_value());
  const auto& uv = *result.value().loops[1].isolated_vertex;
  EXPECT_NEAR(uv.v, -4, 1e-14);
  EXPECT_GT(uv.residual, 0);
  EXPECT_LE(uv.residual, 1e-12);
}
TEST(ParasolidFaceBinding,
     RejectsUnresolvedDistinctConicEndpointsBeforeWrapping) {
  for (const bool ellipse : {false, true})
    for (const char sense : {'+', '-'})
      for (const double gap : {0., 1e-13}) {
        auto f = ConicFace(ellipse, sense);
        auto point = std::vector<ScalarValue>{};
        for (const auto& node : f.report.nodes)
          if (node.index == 200) point = node.fields[0].values;
        ASSERT_EQ(point.size(), 3u);
        point[0] = std::get<double>(point[0]) + gap;
        for (auto& node : f.report.nodes)
          if (node.index == 201) node.fields[0].values = point;
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
        ASSERT_FALSE(result.ok());
        EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
      }
}
TEST(ParasolidFaceBinding, RejectsAmbiguousOrInvalidAutomaticConics) {
  for (int bad = 0; bad < 6; ++bad) {
    auto f = ConicFace(false);
    if (bad == 0) Set(f, 401, "sense", '?');
    if (bad == 1) Set(f, 10, "vertex", 0);
    if (bad == 2) Set(f, 10, "curve", 401);
    if (bad >= 3) {
      for (auto& n : f.report.nodes)
        if (n.index == 201) {
          if (bad == 3)
            n.fields[0].values = {2 + 3 * std::sin(-0.25),
                                  -3 + 3 * std::cos(-0.25), 0.};
          if (bad == 4) n.fields[0].values = {2., -3., 0.};
          if (bad == 5) n.fields[0].values = {2., 0., 1.};
        }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, (bad < 4 && bad != 1)
                                       ? ErrorCode::kUnsupportedFormat
                                       : ErrorCode::kInvalidRecord)
        << bad;
  }
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
ModelFixture ClosedTrimFace(bool vertex_free, bool negative_basis,
                            bool fin_curve = false) {
  auto f = ConicFace(false, negative_basis ? '-' : '+');
  Set(f, 10, "vertex", vertex_free ? 0 : 100);
  Set(f, 11, "vertex", vertex_free ? 0 : 100);
  Set(f, 10, "forward", 10);
  Set(f, 10, "backward", 10);
  Set(f, 11, "forward", 11);
  Set(f, 11, "backward", 11);
  Set(f, 11, "loop", 21);
  Set(f, 20, "next", 21);
  f.Add(15, 21,
        {Pointer("face", 30, 14), Pointer("halfedge", 11, 17),
         Pointer("next", 0, 15)});
  const double first = -.25;
  const double second = first + (negative_basis ? -2 : 2) * std::numbers::pi;
  const double x = 2 + 3 * std::sin(first), y = -3 + 3 * std::cos(first);
  f.Add(133, 402,
        {Pointer("basis_curve", 401, 1008),
         Numbers("point_1", {x, y, 0}, FieldType::kVector),
         Numbers("point_2", {x, y, 0}, FieldType::kVector),
         Numbers("parm_1", {first}), Numbers("parm_2", {second}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  if (fin_curve) {
    Set(f, 10, "curve", 402);
    Set(f, 11, "curve", 402);
  } else {
    Set(f, 2, "curve", 402);
  }
  return f;
}
ModelFixture Sheet(ModelFixture f) {
  Set(f, 6, "edge", 0);
  Set(f, 6, "face", 30);
  Set(f, 6, "front_face", 30);
  Set(f, 2, "owner", 1);
  Set(f, 1, "edge", 2);
  Set(f, 1, "body_type", 3);
  return f;
}
ModelFixture TwoFaceBody() {
  auto f = Sheet(ClosedTrimFace(false, false));
  Set(f, 20, "next", 0);
  Set(f, 21, "face", 31);
  Set(f, 30, "next", 31);
  Set(f, 30, "next_front", 31);
  f.Add(14, 31,
        {Pointer("loop", 21, 15), Pointer("surface", 300, 1006),
         Pointer("shell", 6, 13), Pointer("front_shell", 6, 13),
         Pointer("previous", 30, 14), Pointer("next", 0, 14),
         Pointer("previous_front", 30, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '+', FieldType::kCharacter)});
  std::erase_if(f.report.nodes, [](const auto& n) { return n.index == 101; });
  Set(f, 100, "next", 0);
  Set(f, 11, "next_at_vx", 10);
  return f;
}
ModelFixture WireFromFace(ModelFixture f) {
  std::erase_if(f.report.nodes,
                [](const auto& n) { return n.index == 20 || n.index == 30; });
  Set(f, 1, "body_type", 2);
  Set(f, 1, "edge", 0);
  Set(f, 2, "owner", 6);
  Set(f, 6, "edge", 2);
  Set(f, 6, "face", 0);
  Set(f, 6, "front_face", 0);
  for (uint32_t id : {10u, 11u}) {
    Set(f, id, "loop", 0);
    Set(f, id, "forward", 0);
    Set(f, id, "backward", 0);
    Set(f, id, "curve", 0);
  }
  return f;
}
ModelFixture TwoWireBody(ModelFixture f = Wire()) {
  Set(f, 2, "next", 3);
  f.Add(16, 3,
        {Pointer("owner", 6, 1029), Pointer("previous", 2, 16),
         Pointer("next", 0, 16), Pointer("halfedge", 12, 17),
         Pointer("curve", 400, 1008)});
  Fin(f, 12, 103, 13);
  Fin(f, 13, 102, 12);
  Set(f, 12, "sense", '+');
  Set(f, 12, "edge", 3);
  Set(f, 13, "edge", 3);
  Set(f, 101, "next", 102);
  Vertex(f, 102, 13, 101, 103, 202);
  Vertex(f, 103, 12, 102, 0, 203);
  f.Add(29, 202, {Numbers("position", {2, 0, 0}, FieldType::kVector)});
  f.Add(29, 203, {Numbers("position", {3, 0, 0}, FieldType::kVector)});
  return f;
}
ModelFixture MixedGeometryBody() {
  auto f = TwoWireBody(Face());
  Set(f, 1, "body_type", 6);
  Set(f, 1, "edge", 2);
  Set(f, 2, "owner", 1);
  Set(f, 2, "next", 0);
  Set(f, 3, "previous", 0);
  Set(f, 6, "edge", 3);
  Set(f, 6, "face", 30);
  Set(f, 6, "front_face", 30);
  return f;
}
ModelFixture AcornBody() {
  auto f = Wire();
  std::erase_if(f.report.nodes, [](const auto& n) {
    return n.index == 2 || n.index == 10 || n.index == 11 || n.index == 101;
  });
  Set(f, 6, "edge", 0);
  Set(f, 6, "vertex", 100);
  Set(f, 100, "owner", 6);
  Set(f, 100, "halfedge", 0);
  Set(f, 100, "next", 0);
  for (auto& node : f.report.nodes)
    if (node.index == 200) node.fields[0].values = {1., -2., 3.};
  return f;
}
TEST(ParasolidBodyGeometryBinding, CombinesFacesWiresAndSortedVertexPoints) {
  const auto result = [] {
    const auto f = MixedGeometryBody();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<BodyGeometryBinding>::Err(model.error());
    return BindBodyGeometry(model.value(), NodeId{1}, 0);
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().faces.size(), 1);
  EXPECT_EQ(result.value().faces[0].face.value, 30);
  ASSERT_EQ(result.value().wires.size(), 1);
  EXPECT_EQ(result.value().wires[0].use.edge.value, 3);
  ASSERT_EQ(result.value().vertices.size(), 4);
  EXPECT_EQ(result.value().topology.edges.size(), 2);
  EXPECT_EQ(
      result.value().topology.boundary.regions[0].shells[0].face_uses.size(),
      2);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(result.value().vertices[i].vertex.value, 100 + i);
    EXPECT_EQ(result.value().vertices[i].point.value, 200 + i);
    EXPECT_EQ(result.value().vertices[i].position.x, i);
  }
}
TEST(ParasolidBodyGeometryBinding, AcornsRequireTypedPointGeometry) {
  for (bool invalid : {false, true}) {
    auto f = AcornBody();
    if (invalid)
      for (auto& node : f.report.nodes)
        if (node.index == 200)
          node.fields[0].values = {std::numeric_limits<double>::quiet_NaN(), 0.,
                                   0.};
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_TRUE(BindBodyFaces(model.value(), NodeId{1}, 0).ok());
    EXPECT_TRUE(BindBodyWires(model.value(), NodeId{1}, 0).ok());
    const auto result = BindBodyGeometry(model.value(), NodeId{1}, 0);
    if (invalid) {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
      EXPECT_NE(result.error().message.find("vertex 100"), std::string::npos);
    } else {
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_TRUE(result.value().faces.empty());
      EXPECT_TRUE(result.value().wires.empty());
      ASSERT_EQ(result.value().vertices.size(), 1);
      EXPECT_EQ(result.value().vertices[0].vertex.value, 100);
      EXPECT_EQ(result.value().vertices[0].position.x, 1);
      EXPECT_EQ(result.value().vertices[0].position.y, -2);
      EXPECT_EQ(result.value().vertices[0].position.z, 3);
    }
  }
}
TEST(ParasolidBodyGeometryBinding, AppliesBothOverrideNamespaces) {
  const auto f = MixedGeometryBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval fins[]{{NodeId{10}, 0, 1}};
  const WireParameterInterval wires[]{{NodeId{3}, 2, 3}};
  const auto result =
      BindBodyGeometryWithOverrides(model.value(), NodeId{1}, fins, wires, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().faces[0].loops[0].fins[0].end, 1);
  EXPECT_EQ(result.value().wires[0].start, 2);
  EXPECT_EQ(result.value().wires[0].end, 3);
  const FinParameterInterval wire_fin[]{{NodeId{12}, 2, 3}};
  const auto foreign_fin =
      BindBodyGeometryWithOverrides(model.value(), NodeId{1}, wire_fin, {}, 0);
  ASSERT_FALSE(foreign_fin.ok());
  EXPECT_EQ(foreign_fin.error().code, ErrorCode::kInvalidOptions);
  const WireParameterInterval face_edge[]{{NodeId{2}, 0, 1}};
  const auto foreign_wire =
      BindBodyGeometryWithOverrides(model.value(), NodeId{1}, {}, face_edge, 0);
  ASSERT_FALSE(foreign_wire.ok());
  EXPECT_EQ(foreign_wire.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidBodyGeometryBinding, RejectsLateWireFailureAtomically) {
  auto f = MixedGeometryBody();
  Set(f, 3, "curve", 0);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(BindBodyFaces(model.value(), NodeId{1}, 0).ok());
  const auto result = BindBodyGeometry(model.value(), NodeId{1}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("wire edge 3"), std::string::npos);
}
TEST(ParasolidBodyGeometryBinding,
     SharedSolverSettingsAreOwnedByBothCurvePaths) {
  for (bool face : {false, true}) {
    const auto f = face ? Sheet(IntersectionFace(false, true))
                        : WireFromFace(IntersectionFace(false, true));
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    BodyGeometryBindingOptions options;
    options.intersection = IntersectionEvaluationOptions{1e-12, 1e-12};
    const auto result =
        BindBodyGeometry(model.value(), NodeId{1}, 1e-12, options);
    ASSERT_TRUE(result.ok()) << result.error().message;
    options.intersection.reset();
    if (face) {
      const auto sample = EvaluateBoundFin(
          model.value(), result.value().faces[0].loops[0].fins[0], .5, 1e-12);
      ASSERT_TRUE(sample.ok());
      EXPECT_NEAR(sample.value().curve.position.x, std::sqrt(.5), 1e-12);
    } else {
      const auto sample =
          EvaluateBoundWire(model.value(), result.value().wires[0], .5);
      ASSERT_TRUE(sample.ok());
      EXPECT_NEAR(sample.value().position.x, std::sqrt(.5), 1e-12);
    }
  }
}
TEST(ParasolidBodyGeometryBinding, EnforcesAggregateGeometryBudgets) {
  for (int limited = 0; limited < 4; ++limited) {
    const auto f = limited == 0   ? TwoFaceBody()
                   : limited == 2 ? TwoWireBody()
                                  : MixedGeometryBody();
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    BodyGeometryBindingOptions options;
    if (limited == 0) options.max_faces = 1;
    if (limited == 1) options.max_fins = 1;
    if (limited == 2) options.max_wire_edges = 1;
    if (limited == 3) options.max_vertices = 3;
    const auto result =
        BindBodyGeometry(model.value(), NodeId{1}, 1e-12, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
}
TEST(ParasolidBodyGeometryBinding, OverrideBudgetsApplyBeforeModelLookup) {
  const auto f = MixedGeometryBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BodyGeometryBindingOptions options;
  options.max_fins = 1;
  options.max_wire_edges = 1;
  const FinParameterInterval fins[]{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
  const WireParameterInterval wires[]{{NodeId{2}, 0, 1}, {NodeId{3}, 2, 3}};
  const auto fin_limit = BindBodyGeometryWithOverrides(
      model.value(), NodeId{99}, fins, {}, 0, options);
  ASSERT_FALSE(fin_limit.ok());
  EXPECT_EQ(fin_limit.error().code, ErrorCode::kResourceLimit);
  const auto wire_limit = BindBodyGeometryWithOverrides(
      model.value(), NodeId{99}, {}, wires, 0, options);
  ASSERT_FALSE(wire_limit.ok());
  EXPECT_EQ(wire_limit.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidBodyGeometryBinding, ValidatesAllOptionsBeforeTopologyWork) {
  const auto f = AcornBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 8; ++bad) {
    BodyGeometryBindingOptions options;
    if (bad == 0) options.max_faces = 0;
    if (bad == 1) options.max_fins = 0;
    if (bad == 2) options.max_wire_edges = 0;
    if (bad == 3) options.max_vertices = 0;
    if (bad == 4) options.face.max_loops = 0;
    if (bad == 5) options.face.max_fins = 0;
    if (bad == 6) options.intersection = IntersectionEvaluationOptions{};
    const auto result =
        BindBodyGeometry(model.value(), NodeId{99}, bad == 7 ? -1 : 0, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidAutomaticWireBinding, ConicArcsPreserveDirectedSeamBranches) {
  for (bool ellipse : {false, true}) {
    for (bool negative : {false, true}) {
      auto f = Wire();
      const double minor = ellipse ? 1 : 2;
      for (auto& node : f.report.nodes) {
        const double angle = node.index == 200 ? 3 : -3;
        if (node.index == 200 || node.index == 201)
          node.fields[0].values = {2 * std::cos(angle), minor * std::sin(angle),
                                   0.};
      }
      std::vector<ModelField> fields{
          Numbers("center", {0, 0, 0}, FieldType::kVector),
          Numbers("normal", {0, 0, 1}, FieldType::kVector),
          Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
          ByteValue("sense", negative ? '-' : '+', FieldType::kCharacter)};
      if (ellipse) {
        fields.push_back(Numbers("major_radius", {2}));
        fields.push_back(Numbers("minor_radius", {minor}));
      } else
        fields.push_back(Numbers("radius", {2}));
      f.Add(ellipse ? 32 : 31, 401, std::move(fields));
      Set(f, 2, "curve", 401);
      const auto model = ModelBuilder::Build(f.report);
      ASSERT_TRUE(model.ok());
      const auto result = BindBodyWires(model.value(), NodeId{1}, 1e-12);
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto& wire = result.value().wires[0];
      EXPECT_NEAR(wire.start, 3, 1e-12);
      EXPECT_NEAR(wire.end, negative ? -3 : 2 * std::numbers::pi - 3, 1e-12);
    }
  }
}
TEST(ParasolidAutomaticWireBinding, TrimsRequireMatchingTransmittedPoints) {
  auto f = Wire();
  f.Add(133, 401,
        {Pointer("basis_curve", 400, 1008),
         Numbers("point_1", {0, 0, 0}, FieldType::kVector),
         Numbers("point_2", {2, 0, 0}, FieldType::kVector),
         Numbers("parm_1", {0}), Numbers("parm_2", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 2, "curve", 401);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyWires(model.value(), NodeId{1}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("wire edge 2"), std::string::npos);
}
TEST(ParasolidAutomaticWireBinding,
     OptionsAndAggregateLimitsApplyBeforeInference) {
  const auto f = TwoWireBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BodyWireBindingOptions options;
  options.max_edges = 1;
  const auto limit = BindBodyWires(model.value(), NodeId{1}, 0, options);
  ASSERT_FALSE(limit.ok());
  EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
  const auto sheet = Sheet(Face());
  const auto empty_model = ModelBuilder::Build(sheet.report);
  ASSERT_TRUE(empty_model.ok());
  EXPECT_TRUE(BindBodyWires(empty_model.value(), NodeId{1}, 0).ok());
  options.intersection = IntersectionEvaluationOptions{};
  const auto invalid =
      BindBodyWires(empty_model.value(), NodeId{1}, 0, options);
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
  EXPECT_FALSE(BindBodyWires(empty_model.value(), NodeId{1}, -1).ok());
}
TEST(ParasolidAutomaticWireBinding, RejectsUnrepresentableLineParameters) {
  auto f = Wire();
  const double largest = std::numeric_limits<double>::max();
  for (auto& node : f.report.nodes) {
    if (node.index == 400) node.fields[0].values = {-largest, 0., 0.};
    if (node.index == 201) node.fields[0].values = {largest, 0., 0.};
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyWires(model.value(), NodeId{1}, 0);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidBodyWireBinding,
     BindsSortedWiresAndPreservesNaturalEdgeDirection) {
  const auto f = TwoWireBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto automatic = BindBodyWires(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(automatic.ok()) << automatic.error().message;
  ASSERT_EQ(automatic.value().wires.size(), 2);
  EXPECT_EQ(automatic.value().wires[0].start, 0);
  EXPECT_EQ(automatic.value().wires[0].end, 1);
  EXPECT_EQ(automatic.value().wires[1].start, 2);
  EXPECT_EQ(automatic.value().wires[1].end, 3);
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, {}, 0).ok());
  const WireParameterInterval intervals[]{{NodeId{3}, 2, 3}, {NodeId{2}, 0, 1}};
  const auto result = BindBodyWires(model.value(), NodeId{1}, intervals, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().wires.size(), 2);
  EXPECT_EQ(result.value().topology.edges.size(), 2);
  EXPECT_EQ(result.value().wires[0].use.edge.value, 2);
  EXPECT_EQ(result.value().wires[1].use.edge.value, 3);
  for (const auto& wire : result.value().wires) {
    for (double fraction : {0., .25, .5, .75, 1.}) {
      const auto point = EvaluateBoundWire(model.value(), wire, fraction);
      ASSERT_TRUE(point.ok());
      EXPECT_DOUBLE_EQ(point.value().position.x, wire.start + fraction);
      EXPECT_DOUBLE_EQ(point.value().first_derivative.x, 1);
    }
  }
}
TEST(ParasolidBodyWireBinding, ReversedSourceCurveRetainsFractionTangents) {
  auto f = Wire();
  Set(f, 400, "sense", '-');
  for (auto& node : f.report.nodes)
    if (node.index == 400) node.fields[1].values = {-1., 0., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto automatic = BindBodyWires(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(automatic.ok()) << automatic.error().message;
  ASSERT_EQ(automatic.value().wires.size(), 1);
  EXPECT_EQ(automatic.value().wires[0].end, -1);
  const WireParameterInterval intervals[]{{NodeId{2}, 0, -1}};
  const auto result = BindBodyWires(model.value(), NodeId{1}, intervals, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto point =
      EvaluateBoundWire(model.value(), result.value().wires[0], .25);
  ASSERT_TRUE(point.ok());
  EXPECT_DOUBLE_EQ(point.value().position.x, .25);
  EXPECT_DOUBLE_EQ(point.value().first_derivative.x, 1);
}
TEST(ParasolidBodyWireBinding,
     VertexFreeConicsKeepExplicitTurnCountsAndClosure) {
  auto f = Wire();
  Set(f, 1, "vertex", 0);
  Set(f, 10, "vertex", 0);
  Set(f, 11, "vertex", 0);
  std::erase_if(f.report.nodes,
                [](const auto& n) { return n.index == 100 || n.index == 101; });
  f.Add(
      31, 401,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("normal", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {2}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 2, "curve", 401);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto automatic = BindBodyWires(model.value(), NodeId{1}, 1e-12);
  ASSERT_TRUE(automatic.ok()) << automatic.error().message;
  EXPECT_NEAR(automatic.value().wires[0].end, 2 * std::numbers::pi, 1e-12);
  for (double turns : {1., 2.}) {
    const WireParameterInterval intervals[]{
        {NodeId{2}, 0, turns * 2 * std::numbers::pi}};
    const auto result =
        BindBodyWires(model.value(), NodeId{1}, intervals, 1e-12);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_DOUBLE_EQ(result.value().wires[0].end, intervals[0].end);
    const auto partial = BindBodyWiresWithOverrides(model.value(), NodeId{1}, intervals, 1e-12);
    ASSERT_TRUE(partial.ok()) << partial.error().message;
    EXPECT_DOUBLE_EQ(partial.value().wires[0].end, intervals[0].end);

    const auto point =
        EvaluateBoundWire(model.value(), result.value().wires[0], 0);
    ASSERT_TRUE(point.ok());
    EXPECT_NEAR(point.value().first_derivative.y, 2 * intervals[0].end, 1e-12);
    EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, intervals, 0).ok());
  }
  const WireParameterInterval open[]{{NodeId{2}, 0, std::numbers::pi}};
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, open, 1e-12).ok());
}
TEST(ParasolidBodyWireBinding,
     RejectsMissingDuplicateForeignAndInvalidIntervals) {
  const auto f = Wire();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 6; ++bad) {
    std::vector<WireParameterInterval> intervals{{NodeId{2}, 0, 1}};
    if (bad == 0) intervals.clear();
    if (bad == 1) intervals.push_back(intervals[0]);
    if (bad == 2) intervals[0].edge = NodeId{99};
    if (bad == 3) intervals[0].end = 0;
    if (bad == 4) intervals[0].start = std::numeric_limits<double>::quiet_NaN();
    if (bad == 5) intervals[0].end = std::numeric_limits<double>::infinity();
    const auto result = BindBodyWires(model.value(), NodeId{1}, intervals, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidBodyWireBinding,
     PreservesTopologyAndLateGeometryFailuresAtomically) {
  const WireParameterInterval intervals[]{{NodeId{2}, 0, 1}, {NodeId{3}, 2, 3}};
  for (int bad = 0; bad < 3; ++bad) {
    auto f = TwoWireBody();
    if (bad == 0) Set(f, 2, "next", 0);
    if (bad == 1)
      for (auto& node : f.report.nodes)
        if (node.index == 203) node.fields[0].values = {4., 0., 0.};
    if (bad == 2) Set(f, 400, "sense", '-');
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindBodyWires(model.value(), NodeId{1}, intervals, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    if (bad == 1)
      EXPECT_NE(result.error().message.find("wire edge 3"), std::string::npos);
  }
}
TEST(ParasolidBodyWireBinding, ValidatesOptionsLimitsAndEmptyWireSets) {
  const auto f = Sheet(Face());
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto empty = BindBodyWires(model.value(), NodeId{1}, {}, 0);
  ASSERT_TRUE(empty.ok()) << empty.error().message;
  EXPECT_TRUE(empty.value().wires.empty());
  EXPECT_EQ(empty.value().topology.edges.size(), 1);
  for (int bad = 0; bad < 3; ++bad) {
    BodyWireBindingOptions options;
    if (bad == 0) options.max_edges = 0;
    if (bad == 1) options.intersection = IntersectionEvaluationOptions{};
    const auto result =
        BindBodyWires(model.value(), NodeId{1}, {}, bad == 2 ? -1 : 0, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  const WireParameterInterval foreign[]{{NodeId{2}, 0, 1}};
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, foreign, 0).ok());
  const auto two = TwoWireBody();
  const auto wires = ModelBuilder::Build(two.report);
  ASSERT_TRUE(wires.ok());
  BodyWireBindingOptions options;
  options.max_edges = 1;
  const auto limited =
      BindBodyWires(wires.value(), NodeId{1}, foreign, 0, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const WireParameterInterval both[]{{NodeId{2}, 0, 1}, {NodeId{3}, 2, 3}};
  const auto input_limit = BindBodyWires(wires.value(), NodeId{1}, both, 0, options);
  ASSERT_FALSE(input_limit.ok());
  EXPECT_EQ(input_limit.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidBodyWireBinding, RetainsOwnedIntersectionSettings) {
  auto f = IntersectionFace(false, true);
  std::erase_if(f.report.nodes,
                [](const auto& n) { return n.index == 20 || n.index == 30; });
  for (uint32_t id : {10u, 11u}) {
    Set(f, id, "loop", 0);
    Set(f, id, "forward", 0);
    Set(f, id, "backward", 0);
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const WireParameterInterval intervals[]{{NodeId{2}, 0, std::sqrt(2.)}};
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, intervals, 1e-12).ok());
  BodyWireBindingOptions options;
  options.intersection = IntersectionEvaluationOptions{1e-12, 1e-12};
  const auto automatic = BindBodyWires(model.value(), NodeId{1}, 1e-12, options);
  ASSERT_TRUE(automatic.ok()) << automatic.error().message;
  EXPECT_NEAR(automatic.value().wires[0].end, std::sqrt(2.), 1e-12);
  EXPECT_TRUE(automatic.value().wires[0].intersection.has_value());

  const auto result =
      BindBodyWires(model.value(), NodeId{1}, intervals, 1e-12, options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  options.intersection.reset();
  const auto sample =
      EvaluateBoundWire(model.value(), result.value().wires[0], .5);
  ASSERT_TRUE(sample.ok()) << sample.error().message;
  EXPECT_NEAR(sample.value().position.x, std::sqrt(.5), 1e-12);
  EXPECT_NEAR(sample.value().first_derivative.x, -1, 1e-12);
  EXPECT_NEAR(sample.value().first_derivative.y, 1, 1e-12);
}
TEST(ParasolidBodyWireBinding, SourceTrimsKeepDomainsAndExplicitEndpointTolerance) {
  auto f = Wire();
  f.Add(133, 401,
        {Pointer("basis_curve", 400, 1008),
         Numbers("point_1", {0, 0, 0}, FieldType::kVector),
         Numbers("point_2", {1, 0, 0}, FieldType::kVector),
         Numbers("parm_1", {0}), Numbers("parm_2", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 2, "curve", 401);
  for (auto& node : f.report.nodes)
    if (node.index == 201) node.fields[0].values = {1., 1e-4, 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  EXPECT_TRUE(BindBodyWires(model.value(), NodeId{1}, 1e-3).ok());
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, 1e-5).ok());
  const WireParameterInterval intervals[]{{NodeId{2}, 0, 1}};
  const auto result = BindBodyWires(model.value(), NodeId{1}, intervals, 1e-3);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto outside = BindBodyWires(model.value(), NodeId{1}, intervals, 1e-5);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  const WireParameterInterval beyond[]{{NodeId{2}, -.1, 1}};
  const auto domain = BindBodyWires(model.value(), NodeId{1}, beyond, 1e-3);
  ASSERT_FALSE(domain.ok());
  EXPECT_EQ(domain.error().code, ErrorCode::kInvalidOptions);
  const auto partial_domain = BindBodyWiresWithOverrides(
      model.value(), NodeId{1}, beyond, 1e-3);
  ASSERT_FALSE(partial_domain.ok());
  EXPECT_EQ(partial_domain.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidBodyWireBinding, EvaluatorRejectsBadFractionsAndTangentOverflow) {
  const auto f = Wire();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BoundWireGeometry wire{
      {NodeId{2}, NodeId{100}, NodeId{101}, NodeId{400}}, 0, 1};
  for (double fraction : {-1., 2., std::numeric_limits<double>::quiet_NaN()}) {
    const auto result = EvaluateBoundWire(model.value(), wire, fraction);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  wire.start = -std::numeric_limits<double>::max();
  wire.end = std::numeric_limits<double>::max();
  const auto overflow = EvaluateBoundWire(model.value(), wire, .5);
  ASSERT_FALSE(overflow.ok());
  EXPECT_EQ(overflow.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidBodyFaceBinding, MapsExactFinGeometryOntoAnalyticSweptFaces) {
  auto f = Sheet(Face());
  f.Add(67, 301,
        {Pointer("section", 400, 1008),
         Numbers("sweep", {0, 1, 0}, FieldType::kVector),
         {{"scale", FieldType::kDouble}, {std::monostate{}}},
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 30, "surface", 301);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyFaces(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().faces.size(), 1);
  for (const auto& fin : result.value().faces[0].loops[0].fins) {
    for (double fraction : {0., .25, .5, .75, 1.}) {
      const auto sample = EvaluateBoundFin(model.value(), fin, fraction, 0);
      ASSERT_TRUE(sample.ok()) << sample.error().message;
      EXPECT_DOUBLE_EQ(sample.value().uv.u, sample.value().curve.position.x);
      EXPECT_EQ(sample.value().uv.v, 0);
      EXPECT_EQ(sample.value().uv.residual, 0);
    }
  }
}
TEST(ParasolidBodyFaceBinding, BindsUniqueFacesAndPreservesBothSourceSides) {
  const auto result = [] {
    const auto f = TwoFaceBody();
    const auto model = ModelBuilder::Build(f.report);
    if (!model.ok()) return Result<BodyFaceBinding>::Err(model.error());
    return BindBodyFaces(model.value(), NodeId{1}, 1e-12);
  }();
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().faces.size(), 2);
  EXPECT_EQ(result.value().faces[0].face.value, 30);
  EXPECT_EQ(result.value().faces[1].face.value, 31);
  EXPECT_EQ(
      result.value().topology.boundary.regions[0].shells[0].face_uses.size(),
      4);
  EXPECT_EQ(result.value().topology.edges.size(), 1);
  EXPECT_EQ(result.value().topology.vertices.size(), 1);
  const auto incidence = AnalyzeShellEdgeIncidence(
      result.value().topology.boundary.regions[0].shells[0]);
  ASSERT_TRUE(incidence.ok());
  EXPECT_EQ(incidence.value().paired_edges, 1);
  EXPECT_EQ(incidence.value().face_fins, 2);
  EXPECT_EQ(incidence.value().face_components, 1);

  EXPECT_LT(result.value().faces[1].loops[0].fins[0].end,
            result.value().faces[1].loops[0].fins[0].start);
}
TEST(ParasolidBodyFaceBinding, AppliesExplicitIntervalsAndOwnedSolverOptions) {
  const auto f = Sheet(IntersectionFace(false, true));
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const double end = std::sqrt(2.);
  const FinParameterInterval p[]{{NodeId{11}, end, 0}, {NodeId{10}, 0, end}};
  BodyFaceBindingOptions options;
  options.intersection = IntersectionEvaluationOptions{1e-12, 1e-12};
  auto result = BindBodyFaces(model.value(), NodeId{1}, p, 1e-12, options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().faces.size(), 1);
  const auto automatic =
      BindBodyFaces(model.value(), NodeId{1}, 1e-12, options);
  ASSERT_TRUE(automatic.ok()) << automatic.error().message;
  options.intersection.reset();
  const auto sample = EvaluateBoundFin(
      model.value(), result.value().faces[0].loops[0].fins[0], .5, 1e-12);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().curve.position.x, std::sqrt(.5), 1e-12);
  EXPECT_FALSE(BindBodyFaces(model.value(), NodeId{1}, 1e-12, options).ok());
}
TEST(ParasolidBodyFaceBinding, RejectsSharedEndpointGapsAcrossClosedFaces) {
  auto f = TwoFaceBody();
  // Both single-fin loops close and each lies within 0.09375 of the
  // recorded vertex, but corresponding fin endpoints are 0.1875 apart.
  f.Add(
      31, 403,
      {Numbers("center", {2.1875, -3, 0}, FieldType::kVector),
       Numbers("normal", {0, 0, -1}, FieldType::kVector),
       Numbers("x_axis", {0, 1, 0}, FieldType::kVector), Numbers("radius", {3}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 11, "curve", 403);
  for (auto& node : f.report.nodes)
    if (node.index == 200)
      node.fields[0].values[0] =
          std::get<double>(node.fields[0].values[0]) + .09375;
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const double start = -.25, end = start + 2 * std::numbers::pi;
  const FinParameterInterval intervals[]{{NodeId{10}, start, end},
                                         {NodeId{11}, end, start}};
  for (uint32_t face : {30u, 31u}) {
    const auto index = face == 30 ? 0 : 1;
    const auto bound =
        BindFaceTrimming(model.value(), NodeId{face},
                         std::span(intervals).subspan(index, 1), .125);
    ASSERT_TRUE(bound.ok()) << bound.error().message;
  }
  const auto result = BindBodyFaces(model.value(), NodeId{1}, intervals, .125);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(
      result.error().message,
      "Body 1, edge 2, fins 10/11: Shared edge fin endpoints exceed tolerance");
  EXPECT_TRUE(BindBodyFaces(model.value(), NodeId{1}, intervals, .1875).ok());
  const auto partial = std::span(intervals).subspan(1);
  EXPECT_FALSE(
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, partial, .125).ok());
  EXPECT_FALSE(
      BindBodyGeometryWithOverrides(model.value(), NodeId{1}, partial, {}, .125)
          .ok());
}
TEST(ParasolidBodyFaceBinding, MatchesOppositeFinDirectionsOnOpenEdges) {
  const auto f = Sheet(Face());
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyFaces(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& fins = result.value().faces[0].loops[0].fins;
  ASSERT_EQ(fins.size(), 2);
  EXPECT_EQ(fins[0].start, fins[1].end);
  EXPECT_EQ(fins[0].end, fins[1].start);
}
TEST(ParasolidBodyFaceBinding, VertexFreeRingsMayUseDifferentParameterSeams) {
  auto f = TwoFaceBody();
  Set(f, 1, "vertex", 0);
  std::erase_if(f.report.nodes,
                [](const auto& node) { return node.index == 100; });
  for (uint32_t fin : {10u, 11u}) {
    Set(f, fin, "vertex", 0);
    Set(f, fin, "next_at_vx", 0);
    Set(f, fin, "curve", 401);
  }
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval intervals[]{
      {NodeId{10}, 0, 2 * std::numbers::pi},
      {NodeId{11}, 3 * std::numbers::pi, std::numbers::pi}};
  const auto result = BindBodyFaces(model.value(), NodeId{1}, intervals, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().faces.size(), 2);
}
TEST(ParasolidBodyFaceBinding, RejectsMissingDuplicateAndForeignIntervals) {
  const auto f = Sheet(Face());
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 4; ++bad) {
    std::vector<FinParameterInterval> p{{NodeId{10}, 0, 1}, {NodeId{11}, 1, 0}};
    if (bad == 0) p.pop_back();
    if (bad == 1) p.push_back(p[0]);
    if (bad == 2) p.push_back({NodeId{999}, 0, 1});
    if (bad == 3) p[0].end = 0;
    const auto result = BindBodyFaces(model.value(), NodeId{1}, p, 1e-12);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
}
TEST(ParasolidBodyFaceBinding, EnforcesAggregateLimitsAcrossFacesAndSides) {
  const auto f = TwoFaceBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BodyFaceBindingOptions exact;
  exact.max_faces = 2;
  exact.max_fins = 2;
  exact.face.max_fins = 1;
  const auto good = BindBodyFaces(model.value(), NodeId{1}, 1e-12, exact);
  ASSERT_TRUE(good.ok()) << good.error().message;
  for (int bad = 0; bad < 4; ++bad) {
    auto options = exact;
    if (bad == 0) options.max_faces = 1;
    if (bad == 1) options.max_fins = 1;
    if (bad == 2) options.topology.max_radial_fins = 1;
    if (bad == 3) options.topology.traversal.body.max_face_uses = 3;
    const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12, options);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
}
TEST(ParasolidBodyFaceBinding, CountsIsolatedVertexLoopsInAggregateBudget) {
  auto f = TwoFaceBody();
  Set(f, 21, "next", 22);
  f.Add(15, 22,
        {Pointer("face", 31, 14), Pointer("halfedge", 12, 17),
         Pointer("next", 0, 15)});
  Fin(f, 12, 100, 0, 22, 12);
  Set(f, 12, "edge", 0);
  Set(f, 10, "next_at_vx", 12);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BodyFaceBindingOptions options;
  options.max_fins = 3;
  const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12, options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().faces[1].loops.size(), 2);
  EXPECT_TRUE(result.value().faces[1].loops[1].isolated_vertex.has_value());
  options.max_fins = 2;
  const auto limited = BindBodyFaces(model.value(), NodeId{1}, 1e-12, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidBodyFaceBinding, RejectsTopologyOrLateGeometryFailureAtomically) {
  for (bool geometry : {false, true}) {
    auto f = TwoFaceBody();
    if (geometry) {
      Set(f, 31, "surface", 399);
      f.Add(50, 399,
            {Numbers("origin", {0, 0, 1}, FieldType::kVector),
             Numbers("normal", {0, 0, 1}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             ByteValue("sense", '+', FieldType::kCharacter)});
    } else {
      Set(f, 1, "edge", 0);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    if (geometry)
      EXPECT_NE(result.error().message.find("face 31"), std::string::npos);
  }
}
TEST(ParasolidBodyFaceBinding,
     KeepsWireTopologyAndValidatesEmptyBindingOptions) {
  const auto f = Wire();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyFaces(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_TRUE(result.value().faces.empty());
  EXPECT_EQ(result.value().topology.edges.size(), 1);
  for (int bad = 0; bad < 5; ++bad) {
    BodyFaceBindingOptions options;
    if (bad == 0) options.max_faces = 0;
    if (bad == 1) options.max_fins = 0;
    if (bad == 2) options.face.max_fins = 0;
    if (bad == 3) options.face.max_loops = 0;
    if (bad == 4) options.intersection = IntersectionEvaluationOptions{};
    const auto rejected = BindBodyFaces(model.value(), NodeId{1}, 0, options);
    ASSERT_FALSE(rejected.ok());
    EXPECT_EQ(rejected.error().code, ErrorCode::kInvalidOptions);
  }
  const FinParameterInterval foreign[]{{NodeId{10}, 0, 1}};
  const auto rejected = BindBodyFaces(model.value(), NodeId{1}, foreign, 0);
  ASSERT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidFaceBinding, SourceTrimsPreserveClosedBranchesAndFinDirection) {
  for (bool vertex_free : {false, true}) {
    for (bool negative_basis : {false, true}) {
      auto f = ClosedTrimFace(vertex_free, negative_basis);
      const auto model = ModelBuilder::Build(f.report);
      ASSERT_TRUE(model.ok());
      const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
      ASSERT_TRUE(result.ok()) << result.error().message;
      ASSERT_EQ(result.value().loops.size(), 2);
      const auto& a_fin = result.value().loops[0].fins[0];
      const auto& b_fin = result.value().loops[1].fins[0];
      const BoundFinGeometry fins[]{a_fin, b_fin};
      const double span = (negative_basis ? -2 : 2) * std::numbers::pi;
      EXPECT_DOUBLE_EQ(fins[0].start, -.25);
      EXPECT_DOUBLE_EQ(fins[0].end, -.25 + span);
      EXPECT_DOUBLE_EQ(fins[1].start, fins[0].end);
      EXPECT_DOUBLE_EQ(fins[1].end, fins[0].start);
      const auto a = EvaluateBoundFin(model.value(), fins[0], .25, 1e-12);
      const auto b = EvaluateBoundFin(model.value(), fins[1], .75, 1e-12);
      ASSERT_TRUE(a.ok());
      ASSERT_TRUE(b.ok());
      EXPECT_NEAR(a.value().curve.position.x, b.value().curve.position.x,
                  1e-12);
      EXPECT_NEAR(a.value().curve.first_derivative.x,
                  -b.value().curve.first_derivative.x, 1e-12);
    }
  }
}
TEST(ParasolidFaceBinding, SourceTrimsRejectBadPointsAndUnorientedFinBranches) {
  for (int bad = 0; bad < 5; ++bad) {
    auto f = ClosedTrimFace(bad == 1 || bad == 4, false, bad == 3 || bad == 4);
    if (bad == 0 || bad == 1) {
      for (auto& n : f.report.nodes)
        if (n.index == 402) n.fields[2].values = {0., 0., 0.};
    }
    if (bad == 2) Set(f, 401, "sense", '-');
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, bad >= 3 ? ErrorCode::kUnsupportedFormat
                                            : ErrorCode::kInvalidRecord)
        << bad;
  }
}
TEST(ParasolidFaceBinding, FinTrimsRetainUnambiguousEndpointCorrespondence) {
  auto f = Face();
  f.Add(133, 402,
        {Pointer("basis_curve", 400, 1008),
         Numbers("point_1", {0, 0, 0}, FieldType::kVector),
         Numbers("point_2", {1, 0, 0}, FieldType::kVector),
         Numbers("parm_1", {0}), Numbers("parm_2", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 10, "curve", 402);
  Set(f, 11, "curve", 402);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& fins = result.value().loops[0].fins;
  EXPECT_EQ(fins[0].start, 0);
  EXPECT_EQ(fins[0].end, 1);
  EXPECT_EQ(fins[1].start, 1);
  EXPECT_EQ(fins[1].end, 0);
}
TEST(ParasolidFaceBinding, VertexFreeTrimMustCloseWithinCallerTolerance) {
  auto f = Face();
  Set(f, 10, "forward", 10);
  Set(f, 10, "backward", 10);
  Set(f, 11, "loop", 0);
  Set(f, 11, "forward", 0);
  Set(f, 11, "backward", 0);
  Set(f, 10, "vertex", 0);
  Set(f, 11, "vertex", 0);
  f.Add(133, 402,
        {Pointer("basis_curve", 400, 1008),
         Numbers("point_1", {0, 0, 0}, FieldType::kVector),
         Numbers("point_2", {1, 0, 0}, FieldType::kVector),
         Numbers("parm_1", {0}), Numbers("parm_2", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 2, "curve", 402);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidFaceBinding, ExplicitVertexFreeSingleFinLoopsMustClose) {
  const auto f = ClosedTrimFace(true, false);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const double start = -.25, end = start + 2 * std::numbers::pi;
  std::vector<FinParameterInterval> intervals{{NodeId{10}, start, end},
                                              {NodeId{11}, end, start}};
  ASSERT_TRUE(
      BindFaceTrimming(model.value(), NodeId{30}, intervals, 1e-12).ok());
  intervals[0].end = start + std::numbers::pi;
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, intervals, 1e-12);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(result.error().message,
            "Face loop does not close within tolerance");
  EXPECT_FALSE(
      BindFaceTrimmingWithOverrides(model.value(), NodeId{30}, intervals, 1e-12)
          .ok());
}
TEST(ParasolidFaceBinding, AdjacentFinsMustJoinEvenWhenEachMatchesItsVertex) {
  auto f = Face();
  // Each fin is within 0.75 of the recorded vertex; the two curves are 1.5
  // apart. Endpoint-to-vertex checks alone would accept both with tolerance 1.
  for (auto& node : f.report.nodes)
    if (node.index == 200 || node.index == 201) node.fields[0].values[1] = .75;
  f.Add(30, 401,
        {Numbers("origin", {0, 1.5, 0}, FieldType::kVector),
         Numbers("direction", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 10, "curve", 400);
  Set(f, 11, "curve", 401);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval intervals[]{{NodeId{10}, 0, 1},
                                         {NodeId{11}, 1, 0}};
  const auto result = BindFaceTrimming(model.value(), NodeId{30}, intervals, 1);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(result.error().message,
            "Fin endpoints do not join within face loop tolerance");
  // At tolerance 0.75 automatic endpoint correspondence remains unambiguous.
  EXPECT_FALSE(BindFaceTrimming(model.value(), NodeId{30}, .75).ok());
  EXPECT_TRUE(BindFaceTrimming(model.value(), NodeId{30}, intervals, 1.5).ok());
}
TEST(ParasolidFaceBinding,
     ClosingPairUsesCallerToleranceWithoutAccumulatingIt) {
  const auto f = Face();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  // The internal pair meets at 1. The closing pair has a gap of 0.75,
  // although each endpoint separately lies within 0.375 of vertex 100.
  const FinParameterInterval intervals[]{{NodeId{10}, -.375, 1},
                                         {NodeId{11}, 1, .375}};
  const auto result =
      BindFaceTrimming(model.value(), NodeId{30}, intervals, .5);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().message,
            "Face loop does not close within tolerance");
  EXPECT_TRUE(BindFaceTrimming(model.value(), NodeId{30}, intervals, .75).ok());
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
ModelFixture SplineFace(bool on_surface, bool rational = false,
                        bool unclamped = false) {
  auto f = Sheet(Face());
  f.Add(134, 410,
        {Pointer("nurbs", 411, 136),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const uint16_t dimension =
      on_surface ? (rational ? 3 : 2) : (rational ? 4 : 3);
  f.Add(136, 411,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim", dimension, FieldType::kShort),
         Integer("n_knots", unclamped ? 4 : 2), ByteValue("knot_type", 2),
         Logical("periodic", false), Logical("closed", false),
         Logical("rational", rational), ByteValue("curve_form", 1),
         Pointer("bspline_vertices", 412, 45), Pointer("knots", 413, 128),
         Pointer("knot_mult", 414, 127)});
  std::vector<ScalarValue> controls;
  for (int i = 0; i < 2; ++i) {
    const double weight = rational ? (i == 0 ? 2 : 5) : 1;
    controls.emplace_back(i * weight);
    controls.emplace_back(0.);
    if (!on_surface) controls.emplace_back(0.);
    if (rational) controls.emplace_back(weight);
  }
  f.Add(45, 412, {Array("vertices", FieldType::kDouble, std::move(controls))});
  f.Add(128, 413,
        {unclamped ? Array("knots", FieldType::kDouble, {-5., -2., 3., 7.})
                   : Array("knots", FieldType::kDouble, {-2., 3.})});
  f.Add(127, 414,
        {unclamped ? Array("multiplicities", FieldType::kShort,
                           {int64_t{1}, int64_t{1}, int64_t{1}, int64_t{1}})
                   : Array("multiplicities", FieldType::kShort,
                           {int64_t{2}, int64_t{2}})});
  if (on_surface) {
    f.Add(137, 500,
          {Pointer("surface", 300, 1006), Pointer("b_curve", 410, 134),
           ByteValue("sense", '+', FieldType::kCharacter)});
    Set(f, 10, "curve", 500);
    Set(f, 11, "curve", 500);
  } else {
    Set(f, 2, "curve", 410);
  }
  return f;
}
TEST(ParasolidPartialWireBinding, OverridesPartialSplineAndInfersOtherWires) {
  auto f = TwoWireBody(WireFromFace(SplineFace(false)));
  for (auto& node : f.report.nodes)
    if (node.index == 201) node.fields[0].values = {.5, 0., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const WireParameterInterval overrides[]{{NodeId{2}, -2, .5}};
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, 1e-12).ok());
  EXPECT_FALSE(BindBodyWires(model.value(), NodeId{1}, overrides, 1e-12).ok());
  EXPECT_FALSE(BindBodyWiresWithOverrides(model.value(), NodeId{1}, {}, 1e-12).ok());
  const auto result = BindBodyWiresWithOverrides(model.value(), NodeId{1}, overrides, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  ASSERT_EQ(result.value().wires.size(), 2);
  EXPECT_EQ(result.value().wires[0].use.edge.value, 2);
  EXPECT_EQ(result.value().wires[0].start, -2);
  EXPECT_EQ(result.value().wires[0].end, .5);
  EXPECT_EQ(result.value().wires[1].use.edge.value, 3);
  EXPECT_EQ(result.value().wires[1].start, 2);
  EXPECT_EQ(result.value().wires[1].end, 3);
  const auto sample =
      EvaluateBoundWire(model.value(), result.value().wires[0], .5);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().position.x, .25, 1e-12);
  EXPECT_NEAR(sample.value().first_derivative.x, .5, 1e-12);
}
TEST(ParasolidPartialWireBinding,
     EmptyAndCompleteOverridesRetainAutomaticResults) {
  const auto f = TwoWireBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto automatic = BindBodyWires(model.value(), NodeId{1}, 0);
  ASSERT_TRUE(automatic.ok());
  const WireParameterInterval all[]{{NodeId{3}, 2, 3}, {NodeId{2}, 0, 1}};
  for (bool complete : {false, true}) {
    const auto result = BindBodyWiresWithOverrides(
        model.value(), NodeId{1},
        complete ? std::span<const WireParameterInterval>{all}
                 : std::span<const WireParameterInterval>{},
        0);
    ASSERT_TRUE(result.ok()) << result.error().message;
    ASSERT_EQ(result.value().wires.size(), automatic.value().wires.size());
    for (std::size_t i = 0; i < result.value().wires.size(); ++i) {
      EXPECT_EQ(result.value().wires[i].use.edge,
                automatic.value().wires[i].use.edge);
      EXPECT_EQ(result.value().wires[i].start,
                automatic.value().wires[i].start);
      EXPECT_EQ(result.value().wires[i].end, automatic.value().wires[i].end);
    }
  }
}
TEST(ParasolidPartialWireBinding, RejectsInvalidForeignAndFaceOwnedOverrides) {
  const auto f = TwoWireBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 5; ++bad) {
    std::vector<WireParameterInterval> overrides{{NodeId{2}, 0, 1}};
    if (bad == 0) overrides.push_back(overrides.front());
    if (bad == 1) overrides[0].edge = NodeId{99};
    if (bad == 2) overrides[0].start = 1;
    if (bad == 3) overrides[0].end = std::numeric_limits<double>::infinity();
    if (bad == 4) overrides[0].edge = NodeId{};
    const auto result =
        BindBodyWiresWithOverrides(model.value(), NodeId{1}, overrides, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  const auto sheet = Sheet(Face());
  const auto face_model = ModelBuilder::Build(sheet.report);
  ASSERT_TRUE(face_model.ok());
  const WireParameterInterval face_edge[]{{NodeId{2}, 0, 1}};
  const auto foreign =
      BindBodyWiresWithOverrides(face_model.value(), NodeId{1}, face_edge, 0);
  ASSERT_FALSE(foreign.ok());
  EXPECT_EQ(foreign.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidPartialWireBinding, OverridesKeepMembershipAndLateFailureChecks) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = TwoWireBody();
    if (bad == 2) Set(f, 3, "curve", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const WireParameterInterval overrides[]{{NodeId{2}, bad == 0 ? 1. : 0.,
                                             bad == 0   ? 0.
                                             : bad == 1 ? 2.
                                                        : 1.}};
    const auto result =
        BindBodyWiresWithOverrides(model.value(), NodeId{1}, overrides, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, bad == 2 ? ErrorCode::kUnsupportedFormat
                                            : ErrorCode::kInvalidRecord);
    EXPECT_NE(
        result.error().message.find(bad == 2 ? "wire edge 3" : "wire edge 2"),
        std::string::npos);
  }
}
TEST(ParasolidPartialWireBinding, LimitsAndEmptySelectionOptionsRemainStrict) {
  const auto f = TwoWireBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  BodyWireBindingOptions options;
  options.max_edges = 1;
  const WireParameterInterval one[]{{NodeId{2}, 0, 1}};
  const WireParameterInterval two[]{{NodeId{2}, 0, 1}, {NodeId{3}, 2, 3}};
  for (bool both : {false, true}) {
    const auto result = BindBodyWiresWithOverrides(
        model.value(), NodeId{1},
        both ? std::span<const WireParameterInterval>{two}
             : std::span<const WireParameterInterval>{one},
        0, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  }
  const auto sheet = Sheet(Face());
  const auto empty_model = ModelBuilder::Build(sheet.report);
  ASSERT_TRUE(empty_model.ok());
  EXPECT_TRUE(
      BindBodyWiresWithOverrides(empty_model.value(), NodeId{1}, {}, 0).ok());
  options.intersection = IntersectionEvaluationOptions{};
  const auto result = BindBodyWiresWithOverrides(empty_model.value(), NodeId{1},
                                                 {}, 0, options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidAutomaticWireBinding,
     SplinesAndSurfaceCurvesKeepCompleteKnotDomains) {
  for (bool on_surface : {false, true}) {
    for (bool rational : {false, true}) {
      for (bool unclamped : {false, true}) {
        auto f = WireFromFace(SplineFace(on_surface, rational, unclamped));
        if (on_surface) Set(f, 2, "curve", 500);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto result = BindBodyWires(model.value(), NodeId{1}, 1e-12);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().wires[0].start, -2);
        EXPECT_EQ(result.value().wires[0].end, 3);
      }
    }
  }
}
TEST(ParasolidAutomaticWireBinding,
     PartialSplineDomainsStillRequireExplicitIntervals) {
  auto f = WireFromFace(SplineFace(false));
  for (auto& node : f.report.nodes)
    if (node.index == 201) node.fields[0].values = {.5, 0., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyWires(model.value(), NodeId{1}, 1e-12);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("wire edge 2"), std::string::npos);
  const WireParameterInterval intervals[]{{NodeId{2}, -2, .5}};
  EXPECT_TRUE(BindBodyWires(model.value(), NodeId{1}, intervals, 1e-12).ok());
}
TEST(ParasolidSplineFinIntervals,
     BindsCompleteDomainsWithRationalAndUnclampedAxes) {
  for (bool on_surface : {false, true})
    for (bool rational : {false, true})
      for (bool unclamped : {false, true}) {
        const auto f = SplineFace(on_surface, rational, unclamped);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12);
        ASSERT_TRUE(result.ok()) << result.error().message;
        const auto& fins = result.value().faces[0].loops[0].fins;
        EXPECT_DOUBLE_EQ(fins[0].start, -2);
        EXPECT_DOUBLE_EQ(fins[0].end, 3);
        EXPECT_DOUBLE_EQ(fins[1].start, 3);
        EXPECT_DOUBLE_EQ(fins[1].end, -2);
        const auto sample = EvaluateBoundFin(model.value(), fins[0], .5, 1e-12);
        ASSERT_TRUE(sample.ok());
        EXPECT_NEAR(sample.value().curve.position.x, rational ? 5. / 7 : .5,
                    1e-12);
        EXPECT_NEAR(sample.value().curve.first_derivative.x,
                    rational ? 40. / 49 : 1, 1e-12);
        EXPECT_NEAR(sample.value().uv.u, sample.value().curve.position.x,
                    1e-12);
      }
}
TEST(ParasolidSplineFinIntervals, InteriorEndpointsRemainExplicit) {
  for (bool on_surface : {false, true}) {
    auto f = SplineFace(on_surface);
    for (auto& n : f.report.nodes)
      if (n.index == 200) n.fields[0].values = {.25, 0., 0.};
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto automatic = BindBodyFaces(model.value(), NodeId{1}, 1e-12);
    ASSERT_FALSE(automatic.ok());
    EXPECT_EQ(automatic.error().code, ErrorCode::kUnsupportedFormat);
    const FinParameterInterval p[]{{NodeId{10}, -.75, 3},
                                   {NodeId{11}, 3, -.75}};
    const auto explicit_result =
        BindBodyFaces(model.value(), NodeId{1}, p, 1e-12);
    ASSERT_TRUE(explicit_result.ok()) << explicit_result.error().message;
  }
}
TEST(ParasolidSplineFinIntervals,
     RejectsAmbiguousEndpointsAndUnknownEdgeSense) {
  for (int bad = 0; bad < 3; ++bad) {
    auto f = SplineFace(false);
    if (bad == 0) Set(f, 410, "sense", '?');
    if (bad == 1) {
      for (auto& n : f.report.nodes) {
        if (n.index == 412) n.fields[0].values = {0., 0., 0., 0., 0., 0.};
        if (n.index == 201) n.fields[0].values = {0., 0., 0.};
      }
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result =
        BindFaceTrimming(model.value(), NodeId{30}, bad == 2 ? 1 : 1e-12);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidSplineFinIntervals,
     PreservesAccurateEdgeDirectionAndSourceGeometry) {
  for (bool reverse_vertices : {false, true}) {
    auto f = SplineFace(false);
    Set(f, 410, "sense", '-');
    if (reverse_vertices) {
      Set(f, 10, "vertex", 100);
      Set(f, 11, "vertex", 101);
      Set(f, 100, "halfedge", 10);
      Set(f, 101, "halfedge", 11);
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12);
    if (reverse_vertices) {
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto& fin = result.value().faces[0].loops[0].fins[0];
      EXPECT_EQ(fin.curve.value, 410);
      EXPECT_EQ(fin.start, 3);
      EXPECT_EQ(fin.end, -2);
    } else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    }
  }
}
TEST(ParasolidSplineFinIntervals, InvalidKnotsAndWeightsDoNotProduceBindings) {
  for (bool invalid_weight : {false, true}) {
    auto f = SplineFace(false, true);
    for (auto& n : f.report.nodes) {
      if (!invalid_weight && n.index == 413) n.fields[0].values = {3., -2.};
      if (invalid_weight && n.index == 412) n.fields[0].values.back() = 0.;
    }
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    EXPECT_FALSE(BindBodyFaces(model.value(), NodeId{1}, 1e-12).ok());
  }
}
TEST(ParasolidSplineFinIntervals,
     RequiresVerticesForAutomaticDomainCorrespondence) {
  for (bool on_surface : {false, true}) {
    auto f = SplineFace(on_surface);
    Set(f, 10, "vertex", 0);
    Set(f, 11, "vertex", 0);
    Set(f, 10, "forward", 10);
    Set(f, 10, "backward", 10);
    Set(f, 11, "loop", 0);
    Set(f, 11, "forward", 0);
    Set(f, 11, "backward", 0);
    Set(f, 11, "curve", 0);
    const auto model = ModelBuilder::Build(f.report);
    ASSERT_TRUE(model.ok());
    const auto result = BindFaceTrimming(model.value(), NodeId{30}, 1e-12);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  }
}
TEST(ParasolidSplineFinIntervals,
     MapsSpCurvesFromAnotherSurfaceThroughTargetInversion) {
  auto f = SplineFace(true);
  f.Add(50, 301,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {0, 1, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 500, "surface", 301);
  for (auto& n : f.report.nodes)
    if (n.index == 201) n.fields[0].values = {0., 1., 0.};
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto result = BindBodyFaces(model.value(), NodeId{1}, 1e-12);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto sample = EvaluateBoundFin(
      model.value(), result.value().faces[0].loops[0].fins[0], .5, 1e-12);
  ASSERT_TRUE(sample.ok());
  EXPECT_NEAR(sample.value().curve.position.y, .5, 1e-12);
  EXPECT_NEAR(sample.value().uv.u, 0, 1e-12);
  EXPECT_NEAR(sample.value().uv.v, .5, 1e-12);
}
ModelFixture PartialIntervalBody() {
  auto f = Sheet(Face());
  ParameterCurve(f);
  // The vertex interval [0,1] is strictly inside this curve's natural domain.
  for (auto& node : f.report.nodes) {
    if (node.index == 412) node.fields[0].values = {-1., 0., 2., 0.};
    if (node.index == 413) node.fields[0].values = {-1., 2.};
  }
  f.Add(137, 500,
        {Pointer("surface", 300, 1006), Pointer("b_curve", 410, 134),
         ByteValue("sense", '+', FieldType::kCharacter)});
  Set(f, 10, "curve", 500);
  return f;
}
TEST(ParasolidPartialIntervals, SuppliesOnlyTheFinThatNeedsAnInterval) {
  const auto f = PartialIntervalBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{{NodeId{10}, 0, 1}};
  EXPECT_FALSE(BindFaceTrimming(model.value(), NodeId{30}, 1e-12).ok());
  const auto complete = BindFaceTrimming(model.value(), NodeId{30}, p, 1e-12);
  ASSERT_FALSE(complete.ok());
  EXPECT_EQ(complete.error().code, ErrorCode::kInvalidOptions);
  const auto face =
      BindFaceTrimmingWithOverrides(model.value(), NodeId{30}, p, 1e-12);
  ASSERT_TRUE(face.ok()) << face.error().message;
  const auto body =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12);
  ASSERT_TRUE(body.ok()) << body.error().message;
  const auto& fins = body.value().faces[0].loops[0].fins;
  ASSERT_EQ(fins.size(), 2);
  EXPECT_EQ(fins[0].curve.value, 500);
  EXPECT_EQ(fins[1].curve.value, 400);
  EXPECT_EQ(fins[1].start, 1);
  EXPECT_EQ(fins[1].end, 0);
  const auto point = EvaluateBoundFin(model.value(), fins[0], .25, 1e-12);
  ASSERT_TRUE(point.ok());
  EXPECT_NEAR(point.value().uv.u, .25, 1e-12);
}
TEST(ParasolidPartialIntervals,
     RejectsInvalidOrForeignOverridesBeforeInference) {
  const auto f = PartialIntervalBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  for (int bad = 0; bad < 5; ++bad) {
    std::vector<FinParameterInterval> p{{NodeId{10}, 0, 1}};
    if (bad == 0) p.push_back(p[0]);
    if (bad == 1) p[0].fin = NodeId{999};
    if (bad == 2) p[0].start = p[0].end;
    if (bad == 3) p[0].start = std::numeric_limits<double>::quiet_NaN();
    if (bad == 4) p[0].end = .9;
    const auto face =
        BindFaceTrimmingWithOverrides(model.value(), NodeId{30}, p, 1e-12);
    const auto body =
        BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12);
    ASSERT_FALSE(face.ok()) << bad;
    ASSERT_FALSE(body.ok()) << bad;
    const auto code =
        bad == 4 ? ErrorCode::kInvalidRecord : ErrorCode::kInvalidOptions;
    EXPECT_EQ(face.error().code, code);
    EXPECT_EQ(body.error().code, code);
  }
}
TEST(ParasolidPartialIntervals, KeepsUnspecifiedBranchesAndDirectionsStrict) {
  auto f = PartialIntervalBody();
  Set(f, 11, "curve", 500);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval one[]{{NodeId{10}, 0, 1}};
  const auto missing =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, one, 1e-12);
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.error().code, ErrorCode::kUnsupportedFormat);
  const auto plain = ModelBuilder::Build(Sheet(Face()).report);
  ASSERT_TRUE(plain.ok());
  const FinParameterInterval wrong[]{{NodeId{11}, 0, 1}};
  const auto direction =
      BindBodyFacesWithOverrides(plain.value(), NodeId{1}, wrong, 1e-12);
  ASSERT_FALSE(direction.ok());
  EXPECT_EQ(direction.error().code, ErrorCode::kInvalidRecord);
}
TEST(ParasolidPartialIntervals,
     GroupsOverridesAcrossUniqueBodyFacesAndBoundsInput) {
  const auto f = TwoFaceBody();
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{
      {NodeId{11}, -.25 + 2 * std::numbers::pi, -.25}};
  BodyFaceBindingOptions options;
  options.max_fins = 2;
  options.face.max_fins = 1;
  const auto body =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12, options);
  ASSERT_TRUE(body.ok()) << body.error().message;
  EXPECT_EQ(body.value().faces.size(), 2);
  options.max_fins = 1;
  const auto aggregate =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12, options);
  ASSERT_FALSE(aggregate.ok());
  EXPECT_EQ(aggregate.error().code, ErrorCode::kResourceLimit);
  const FinParameterInterval excessive[]{{NodeId{10}, 0, 1},
                                         {NodeId{11}, 1, 0}};
  const auto body_input = BindBodyFacesWithOverrides(model.value(), NodeId{1},
                                                     excessive, 1e-12, options);
  const auto face_input = BindFaceTrimmingWithOverrides(
      model.value(), NodeId{30}, excessive, 1e-12, {1, 1});
  ASSERT_FALSE(body_input.ok());
  ASSERT_FALSE(face_input.ok());
  EXPECT_EQ(body_input.error().code, ErrorCode::kResourceLimit);
  EXPECT_EQ(face_input.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidPartialIntervals, CarriesIntersectionSettingsIntoOverriddenFins) {
  auto f = Sheet(IntersectionFace(false));
  Set(f, 2, "curve", 400);
  Set(f, 10, "curve", 401);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{{NodeId{10}, 0, 1}};
  BodyFaceBindingOptions options;
  options.intersection = IntersectionEvaluationOptions{1e-12, 1e-12};
  const auto result =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12, options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto& fin = result.value().faces[0].loops[0].fins[0];
  ASSERT_TRUE(fin.intersection.has_value());
  const auto point = EvaluateBoundFin(model.value(), fin, .5, 1e-12);
  ASSERT_TRUE(point.ok());
  EXPECT_NEAR(point.value().curve.position.x, .5, 1e-12);
}
TEST(ParasolidPartialIntervals, IsolatedLoopFinsCannotHaveIntervals) {
  auto f = TwoFaceBody();
  Set(f, 21, "next", 22);
  f.Add(15, 22,
        {Pointer("face", 31, 14), Pointer("halfedge", 12, 17),
         Pointer("next", 0, 15)});
  Fin(f, 12, 100, 0, 22, 12);
  Set(f, 12, "edge", 0);
  Set(f, 10, "next_at_vx", 12);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval p[]{{NodeId{12}, 0, 1}};
  const auto face =
      BindFaceTrimmingWithOverrides(model.value(), NodeId{31}, p, 1e-12);
  const auto body =
      BindBodyFacesWithOverrides(model.value(), NodeId{1}, p, 1e-12);
  ASSERT_FALSE(face.ok());
  ASSERT_FALSE(body.ok());
  EXPECT_EQ(face.error().code, ErrorCode::kInvalidOptions);
  EXPECT_EQ(body.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidFaceBinding,
     UsesTransmittedUvAndRejectsOffFaceSupportingSurface) {
  for (bool wrong : {false, true}) {
    auto f = Face();
    ParameterCurve(f);
    f.Add(137, 500,
          {Pointer("surface", wrong ? 301 : 300, 1006),
           Pointer("b_curve", 410, 134),
           ByteValue("sense", '+', FieldType::kCharacter)});
    if (wrong)
      f.Add(50, 301,
            {Numbers("origin", {0, 0, 1}, FieldType::kVector),
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

ModelFixture BoundaryGeometryFace(bool reverse, bool negative_basis = false) {
  auto f = ClosedTrimFace(true, negative_basis);
  const uint32_t loop = reverse ? 21 : 20;
  const uint32_t dummy = reverse ? 10 : 11;
  Set(f, 30, "loop", loop);
  Set(f, loop, "next", 0);
  Set(f, dummy, "loop", 0);
  Set(f, dummy, "forward", 0);
  Set(f, dummy, "backward", 0);
  std::erase_if(f.report.nodes, [loop](const auto& node) {
    return node.node_type == 15 && node.index != loop;
  });
  return f;
}
std::vector<ShellBoundaryGroup> BoundaryGroups(const Model& model) {
  const auto face = TraverseFaceBoundary(model, NodeId{30});
  EXPECT_TRUE(face.ok());
  if (!face.ok()) return {};
  ShellBoundary shell;
  shell.faces.push_back(face.value());
  const auto incidence = AnalyzeShellEdgeIncidence(shell);
  EXPECT_TRUE(incidence.ok());
  if (!incidence.ok()) return {};
  return incidence.value().components[0].boundary_groups;
}
TEST(ParasolidBoundaryGeometry, OwnsSourceBindingsAndReversesPathTangents) {
  for (bool reverse : {false, true}) {
    for (bool negative : {false, true}) {
      const auto fixture = BoundaryGeometryFace(reverse, negative);
      const auto model = ModelBuilder::Build(fixture.report);
      ASSERT_TRUE(model.ok());
      const auto face = BindFaceTrimming(model.value(), NodeId{30}, 1e-10);
      ASSERT_TRUE(face.ok()) << face.error().message;
      auto groups = BoundaryGroups(model.value());
      ASSERT_EQ(groups.size(), 1);
      std::vector<FaceTrimming> faces{face.value()};
      const auto result = BindBoundaryPathsGeometry(
          model.value(), groups, faces, 1e-10, {1, 1, 1, 1, 1});
      ASSERT_TRUE(result.ok()) << result.error().message;
      ASSERT_EQ(result.value().size(), 1);
      ASSERT_EQ(result.value()[0].edges.size(), 1);
      const auto& edge = result.value()[0].edges[0];
      EXPECT_EQ(edge.use.fin, NodeId{reverse ? 11u : 10u});
      EXPECT_EQ(edge.use.sense, TopologySense::kForward);
      EXPECT_EQ(edge.source.start, faces[0].loops[0].fins[0].start);
      faces.clear();
      groups.clear();
      for (double fraction : {0., .25, .75, 1.}) {
        const auto value =
            EvaluateBoundBoundaryPathEdge(model.value(), edge, fraction, 1e-10);
        const auto source =
            EvaluateBoundFin(model.value(), edge.source,
                             reverse ? 1 - fraction : fraction, 1e-10);
        ASSERT_TRUE(value.ok()) << value.error().message;
        ASSERT_TRUE(source.ok());
        EXPECT_NEAR(value.value().curve.position.x,
                    source.value().curve.position.x, 1e-10);
        EXPECT_NEAR(value.value().curve.position.y,
                    source.value().curve.position.y, 1e-10);
        EXPECT_NEAR(
            value.value().curve.first_derivative.x,
            (reverse ? -1 : 1) * source.value().curve.first_derivative.x,
            1e-10);
        EXPECT_NEAR(
            value.value().curve.first_derivative.y,
            (reverse ? -1 : 1) * source.value().curve.first_derivative.y,
            1e-10);
        EXPECT_NEAR(value.value().uv.u, source.value().uv.u, 1e-10);
      }
    }
  }
}
TEST(ParasolidBoundaryGeometry, RepeatedPathsHonorAggregateBudgets) {
  const auto fixture = BoundaryGeometryFace(false);
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, 1e-10);
  ASSERT_TRUE(face.ok());
  const std::vector<FaceTrimming> faces{face.value()};
  auto groups = BoundaryGroups(model.value());
  ASSERT_EQ(groups.size(), 1);
  groups.push_back(groups.front());
  auto result = BindBoundaryPathsGeometry(model.value(), groups, faces, 1e-10,
                                          {1, 1, 1, 2, 2});
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().size(), 2);
  result = BindBoundaryPathsGeometry(model.value(), groups, faces, 1e-10,
                                     {1, 1, 1, 1, 2});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  result = BindBoundaryPathsGeometry(model.value(), groups, faces, 1e-10,
                                     {1, 1, 1, 2, 1});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  for (int field = 0; field < 5; ++field) {
    BoundaryPathBindingOptions options;
    uint64_t* limit = field == 0   ? &options.max_faces
                      : field == 1 ? &options.max_loops
                      : field == 2 ? &options.max_fins
                      : field == 3 ? &options.max_paths
                                   : &options.max_edges;
    *limit = 0;
    result = BindBoundaryPathsGeometry(model.value(), {}, {}, 0, options);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  for (int field = 0; field < 3; ++field) {
    auto extra_faces = faces;
    if (field == 0) extra_faces.push_back(faces[0]);
    if (field == 1) extra_faces[0].loops.push_back(faces[0].loops[0]);
    if (field == 2)
      extra_faces[0].loops[0].fins.push_back(faces[0].loops[0].fins[0]);
    const auto limited =
        BindBoundaryPathsGeometry(model.value(), std::span(groups).first(1),
                                  extra_faces, 1e-10, {1, 1, 1, 2, 2});
    ASSERT_FALSE(limited.ok());
    EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  }
  EXPECT_TRUE(BindBoundaryPathsGeometry(model.value(), {}, {}, 0).ok());
}
TEST(ParasolidBoundaryGeometry,
     RejectsForeignUsesBranchesAndOpenCyclesAtomically) {
  const auto fixture = BoundaryGeometryFace(false);
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, 1e-10);
  ASSERT_TRUE(face.ok());
  for (int bad = 0; bad < 7; ++bad) {
    std::vector<FaceTrimming> faces{face.value()};
    auto groups = BoundaryGroups(model.value());
    ASSERT_EQ(groups.size(), 1);
    if (bad == 0) groups[0].path[0].fin = NodeId{999};
    if (bad == 1) groups[0].path[0].edge = NodeId{999};
    if (bad == 2) groups[0].path[0].start_vertex = NodeId{100};
    if (bad == 3) faces[0].loops[0].fins.push_back(faces[0].loops[0].fins[0]);
    if (bad == 4) groups[0].kind = BoundaryGroupKind::kBranched;
    if (bad == 5) faces[0].loops[0].fins[0].end = .5;
    if (bad == 6) groups[0].path.clear();
    const auto result =
        BindBoundaryPathsGeometry(model.value(), groups, faces, 1e-10);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, bad == 4   ? ErrorCode::kUnsupportedFormat
                                   : bad == 5 ? ErrorCode::kInvalidRecord
                                              : ErrorCode::kInvalidOptions);
  }
  const std::vector<FaceTrimming> faces{face.value()};
  const auto groups = BoundaryGroups(model.value());
  EXPECT_FALSE(
      BindBoundaryPathsGeometry(model.value(), groups, faces, -1).ok());
  EXPECT_FALSE(
      BindBoundaryPathsGeometry(model.value(), groups, faces,
                                std::numeric_limits<double>::quiet_NaN())
          .ok());
}
TEST(ParasolidBoundaryGeometry, RetainsOwnedIntersectionSolverSettings) {
  const auto fixture = IntersectionFace(false, true);
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto face =
      BindFaceTrimming(model.value(), NodeId{30}, 1e-10, {},
                       IntersectionEvaluationOptions{1e-10, 1e-12});
  ASSERT_TRUE(face.ok()) << face.error().message;
  const auto use = face.value().loops[0].fins[0].use;
  ShellBoundaryGroup group;
  group.kind = BoundaryGroupKind::kChain;
  group.edges = {use.edge};
  group.end_vertices = {use.start_vertex, use.end_vertex};
  group.path = {
      {use.edge, use.fin, use.start_vertex, use.end_vertex, use.sense}};
  std::vector<FaceTrimming> faces{face.value()};
  const auto result = BindBoundaryPathsGeometry(
      model.value(), std::span(&group, 1), faces, 1e-10);
  ASSERT_TRUE(result.ok()) << result.error().message;
  faces.clear();
  const auto& edge = result.value()[0].edges[0];
  ASSERT_TRUE(edge.source.intersection);
  const auto midpoint =
      EvaluateBoundBoundaryPathEdge(model.value(), edge, .5, 1e-10);
  ASSERT_TRUE(midpoint.ok()) << midpoint.error().message;
  EXPECT_NEAR(midpoint.value().curve.position.x, std::sqrt(.5), 1e-10);
  EXPECT_NEAR(midpoint.value().curve.position.y, std::sqrt(.5), 1e-10);
  const auto whole =
      EvaluateBoundBoundaryPath(model.value(), result.value()[0], .5, 1e-10);
  ASSERT_TRUE(whole.ok()) << whole.error().message;
  EXPECT_EQ(whole.value().edge_index, 0);
  EXPECT_EQ(whole.value().use.fin, use.fin);
  EXPECT_EQ(whole.value().curve, edge.source.curve);
  EXPECT_NEAR(whole.value().geometry.curve.position.x, std::sqrt(.5), 1e-10);

  auto unsupported = edge;
  unsupported.source.intersection.reset();
  EXPECT_FALSE(
      EvaluateBoundBoundaryPathEdge(model.value(), unsupported, .5, 1e-10)
          .ok());
}
ModelFixture TwoEdgeBoundaryFixture() {
  auto fixture = Face();
  Set(fixture, 10, "other", 12);
  Set(fixture, 11, "other", 13);
  Set(fixture, 11, "edge", 3);
  Fin(fixture, 12, 100, 10);
  Fin(fixture, 13, 101, 11);
  Set(fixture, 13, "sense", '+');
  Set(fixture, 13, "edge", 3);
  fixture.Add(16, 3,
              {Pointer("owner", 6, 1029), Pointer("previous", 0, 16),
               Pointer("next", 0, 16), Pointer("halfedge", 13, 17),
               Pointer("curve", 400, 1008)});
  return fixture;
}
TEST(ParasolidBoundaryGeometry, ChecksJoinGapsWithTheRequestedPathTolerance) {
  const auto fixture = TwoEdgeBoundaryFixture();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const FinParameterInterval intervals[]{{NodeId{10}, 0, 1.375},
                                         {NodeId{11}, .625, 0}};
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, intervals, .75);
  ASSERT_TRUE(face.ok()) << face.error().message;
  const auto groups = BoundaryGroups(model.value());
  ASSERT_EQ(groups.size(), 1);
  const std::vector<FaceTrimming> faces{face.value()};
  const auto result =
      BindBoundaryPathsGeometry(model.value(), groups, faces, .5);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_EQ(result.error().message, "Boundary path endpoints do not join");
  EXPECT_TRUE(
      BindBoundaryPathsGeometry(model.value(), groups, faces, .75).ok());
}
TEST(ParasolidBoundaryGeometry, EvaluatesWholePathsWithExplicitJoinSides) {
  const auto fixture = TwoEdgeBoundaryFixture();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, 0);
  ASSERT_TRUE(face.ok()) << face.error().message;
  const auto groups = BoundaryGroups(model.value());
  const std::vector<FaceTrimming> faces{face.value()};
  const auto bound = BindBoundaryPathsGeometry(model.value(), groups, faces, 0);
  ASSERT_TRUE(bound.ok()) << bound.error().message;
  const auto& path = bound.value()[0];
  ASSERT_EQ(path.edges.size(), 2);
  const auto incoming = EvaluateBoundBoundaryPath(
      model.value(), path, 1, 0, {2, BoundaryPathJoinSide::kIncoming});
  const auto outgoing = EvaluateBoundBoundaryPath(model.value(), path, 1, 0);
  ASSERT_TRUE(incoming.ok());
  ASSERT_TRUE(outgoing.ok());
  EXPECT_EQ(incoming.value().edge_index, 0);
  EXPECT_EQ(incoming.value().edge_fraction, 1);
  EXPECT_EQ(incoming.value().use.fin, NodeId{10});
  EXPECT_EQ(outgoing.value().edge_index, 1);
  EXPECT_EQ(outgoing.value().edge_fraction, 0);
  EXPECT_EQ(outgoing.value().use.fin, NodeId{11});
  EXPECT_EQ(incoming.value().geometry.curve.position.x, 1);
  EXPECT_EQ(outgoing.value().geometry.curve.position.x, 1);
  EXPECT_EQ(incoming.value().geometry.curve.first_derivative.x, 1);
  EXPECT_EQ(outgoing.value().geometry.curve.first_derivative.x, -1);
  EXPECT_EQ(outgoing.value().face, NodeId{30});
  EXPECT_EQ(outgoing.value().surface, NodeId{300});
  EXPECT_EQ(outgoing.value().curve, NodeId{400});
  const auto interior = EvaluateBoundBoundaryPath(model.value(), path, 1.25, 0);
  ASSERT_TRUE(interior.ok());
  EXPECT_EQ(interior.value().edge_fraction, .25);
  EXPECT_EQ(interior.value().geometry.curve.position.x, .75);
  for (auto side :
       {BoundaryPathJoinSide::kIncoming, BoundaryPathJoinSide::kOutgoing}) {
    const auto first =
        EvaluateBoundBoundaryPath(model.value(), path, 0, 0, {2, side});
    const auto last =
        EvaluateBoundBoundaryPath(model.value(), path, 2, 0, {2, side});
    ASSERT_TRUE(first.ok());
    ASSERT_TRUE(last.ok());
    EXPECT_EQ(first.value().edge_index, 0);
    EXPECT_EQ(first.value().edge_fraction, 0);
    EXPECT_EQ(last.value().edge_index, 1);
    EXPECT_EQ(last.value().edge_fraction, 1);
  }
  const auto before =
      EvaluateBoundBoundaryPath(model.value(), path, std::nextafter(1., 0.), 0);
  const auto after =
      EvaluateBoundBoundaryPath(model.value(), path, std::nextafter(1., 2.), 0,
                                {2, BoundaryPathJoinSide::kIncoming});
  ASSERT_TRUE(before.ok());
  ASSERT_TRUE(after.ok());
  EXPECT_EQ(before.value().edge_index, 0);
  EXPECT_EQ(after.value().edge_index, 1);
}
TEST(ParasolidBoundaryGeometry,
     WholePathQueriesValidateDomainsBudgetsAndSourceErrors) {
  const auto fixture = TwoEdgeBoundaryFixture();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto face = BindFaceTrimming(model.value(), NodeId{30}, 0);
  ASSERT_TRUE(face.ok());
  const auto groups = BoundaryGroups(model.value());
  const std::vector<FaceTrimming> faces{face.value()};
  const auto bound = BindBoundaryPathsGeometry(model.value(), groups, faces, 0);
  ASSERT_TRUE(bound.ok());
  const auto& path = bound.value()[0];
  for (double parameter : {-1., 2.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    const auto result =
        EvaluateBoundBoundaryPath(model.value(), path, parameter, 0);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  EXPECT_FALSE(EvaluateBoundBoundaryPath(model.value(), path, 0, -1).ok());
  EXPECT_FALSE(
      EvaluateBoundBoundaryPath(model.value(), path, 0,
                                std::numeric_limits<double>::quiet_NaN())
          .ok());
  EXPECT_FALSE(EvaluateBoundBoundaryPath(model.value(), {}, 0, 0).ok());
  const auto limited = EvaluateBoundBoundaryPath(
      model.value(), path, 0, 0, {1, BoundaryPathJoinSide::kOutgoing});
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  EXPECT_FALSE(EvaluateBoundBoundaryPath(model.value(), path, 0, 0,
                                         {0, BoundaryPathJoinSide::kOutgoing})
                   .ok());
  EXPECT_FALSE(
      EvaluateBoundBoundaryPath(model.value(), path, 0, 0,
                                {2, static_cast<BoundaryPathJoinSide>(99)})
          .ok());
  auto changed = path;
  changed.kind = BoundaryGroupKind::kBranched;
  const auto branched = EvaluateBoundBoundaryPath(model.value(), changed, 0, 0);
  ASSERT_FALSE(branched.ok());
  EXPECT_EQ(branched.error().code, ErrorCode::kUnsupportedFormat);
  changed = path;
  changed.edges[0].source.curve = NodeId{999};
  const auto direct =
      EvaluateBoundBoundaryPathEdge(model.value(), changed.edges[0], 0, 0);
  const auto whole = EvaluateBoundBoundaryPath(model.value(), changed, 0, 0);
  ASSERT_FALSE(direct.ok());
  ASSERT_FALSE(whole.ok());
  EXPECT_EQ(whole.error().code, direct.error().code);
  EXPECT_EQ(whole.error().message, direct.error().message);
}
}  // namespace
}  // namespace openparasolid
