// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "../src/swept_intersection.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "../../libopenparasolid/tests/model_test_helpers.h"
namespace openswstep {
namespace {
using namespace openparasolid;
using namespace openparasolid::test;
ModelFixture SweptPlaneFixture(bool cubic = false) {
  ModelFixture f;
  f.Spline();
  for (auto& node : f.report.nodes)
    if (node.index == 3)
      node.fields[0].values = {0., 0., 0., .5, 0., 0., 1., 1., 0.};
  f.Add(67, 20,
        {Pointer("section", 1, 1008), Numbers("scale", {42}),
         Numbers("sweep", {0, 0, 1}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const double w = std::sqrt(.5);
  f.Add(50, 21,
        {Numbers("origin", {0, 0, 2}, FieldType::kVector),
         Numbers("normal", {-w, 0, w}, FieldType::kVector),
         Numbers("x_axis", {w, 0, w}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(38, 30,
        {{{"surface", FieldType::kPointer, 1006, 2, true},
          {uint64_t{21}, uint64_t{20}}},
         Pointer("chart", 31, 40),
         Pointer("start", 32, 41),
         Pointer("end", 33, 41),
         ByteValue("sense", '-', FieldType::kCharacter)});
  f.Add(40, 31,
        {Numbers("base_parameter", {0}),
         Numbers("base_scale", {1}),
         Integer("chart_count", 3),
         Numbers("chordal_error", {.1}),
         Numbers("angular_error", {1}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               {0., 0., 2., .5, .25, 2.5, 1., 1., 3.})});
  f.Add(41, 32,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {0., 0., 2.})});
  f.Add(41, 33,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {1., 1., 3.})});
  if (cubic) {
    for (auto& node : f.report.nodes) {
      if (node.index == 2) {
        node.fields[0].values = {int64_t{3}};
        node.fields[1].values = {int64_t{4}};
      }
      if (node.index == 5) node.fields[0].values = {int64_t{4}, int64_t{4}};
      if (node.index == 31)
        node.fields.back().values = {0., 0., 2., .5, .125, 2.5, 1., 1., 3.};
    }
    std::erase_if(f.report.nodes, [](const auto& n) { return n.index == 3; });
    f.Add(45, 3,
          {Array("vertices", FieldType::kDouble,
                 {0., 0., 0., 1. / 3, 0., 0., 2. / 3, 0., 0., 1., 1., 0.})});
  }
  return f;
}
Result<internal::IntersectionCurveRecovery> Recover(const ModelFixture& fixture,
                                                    uint64_t& checks,
                                                    unsigned bad = 0,
                                                    bool partial = false) {
  using Return = Result<internal::IntersectionCurveRecovery>;
  const auto model = ModelBuilder::Build(fixture.report);
  if (!model.ok()) return Return::Err(model.error());
  const IntersectionEvaluationOptions options{1e-7, 1e-10};
  const auto chart = EvaluateIntersectionChart(model.value(), NodeId{30},
                                               options.chart_tolerance,
                                               options.solve_tolerance);
  double first = 0, last = 1;
  if (chart.ok()) {
    first = chart.value().parameterization.parameters.front();
    last = chart.value().parameterization.parameters.back();
  }
  if (partial) {
    const auto width = last - first;
    first += .2 * width;
    last -= .1 * width;
  }
  std::array<BoundFinGeometry, 2> fins{
      BoundFinGeometry{{NodeId{50},
                        NodeId{40},
                        NodeId{60},
                        NodeId{61},
                        NodeId{30},
                        {},
                        TopologySense::kForward},
                       NodeId{70},
                       NodeId{20},
                       NodeId{30},
                       first,
                       last,
                       options},
      BoundFinGeometry{{NodeId{51},
                        NodeId{40},
                        NodeId{61},
                        NodeId{60},
                        NodeId{30},
                        {},
                        TopologySense::kReverse},
                       NodeId{71},
                       NodeId{21},
                       NodeId{30},
                       last,
                       first,
                       options}};
  std::array<BoundBodyVertex, 2> vertices{
      {{NodeId{60}, {}, {0, 0, 2}}, {NodeId{61}, {}, {1, 1, 3}}}};
  if (partial) {
    for (unsigned i = 0; i < 2; ++i) {
      const auto point = EvaluateIntersectionCurve(model.value(), NodeId{30},
                                                   i ? last : first, options);
      if (!point.ok()) return Return::Err(point.error());
      vertices[i].position = point.value().position;
    }
  }
  if (bad == 1) fins[1].intersection.reset();
  if (bad == 2) fins[1].use.sense = TopologySense::kForward;
  if (bad == 3) fins[1].use.fin_curve = NodeId{1};
  if (bad == 4) vertices[0].position.y += .001;
  if (bad == 5) fins[0].start -= 1;
  if (bad == 6) fins[1].surface = NodeId{20};
  if (bad == 7) {
    fins[0].intersection->solve_tolerance = 1e-5;
    fins[1].intersection->solve_tolerance = 1e-5;
  }
  if (bad == 8) fins[1].intersection->solve_tolerance *= 2;
  const std::array<const BoundFinGeometry*, 2> uses{&fins[0], &fins[1]};
  const auto& source =
      model.value().geometry()[*model.value().FindNode(NodeId{30})->geometry];
  return internal::RecoverSweptPlaneIntersection(model.value(), source, uses,
                                                 vertices, 1e-8, checks);
}
TEST(OpenSwStepSweptIntersection,
     DerivesExactPolynomialGraphAndPartialIntervals) {
  for (const bool cubic : {false, true})
    for (const bool partial : {false, true}) {
      uint64_t checks = 100000;
      const auto result = Recover(SweptPlaneFixture(cubic), checks, 0, partial);
      ASSERT_TRUE(result.ok()) << result.error().message;
      const auto& curve = std::get<NurbsCurveGeometry>(result.value().geometry);
      const std::vector<double> expected =
          cubic ? std::vector<double>{0,          0,          2,      1. / 3,
                                      0,          2 + 1. / 3, 2. / 3, 0,
                                      2 + 2. / 3, 1,          1,      3}
                : std::vector<double>{0, 0, 2, .5, 0, 2.5, 1, 1, 3};
      ASSERT_EQ(curve.controls.values.size(), expected.size());
      for (std::size_t i = 0; i < expected.size(); ++i)
        EXPECT_NEAR(curve.controls.values[i], expected[i], 1e-15);
      ASSERT_EQ(result.value().uses.size(), 2u);
      EXPECT_EQ(result.value().uses[0].start, result.value().uses[1].end);
      EXPECT_EQ(result.value().uses[0].end, result.value().uses[1].start);
      EXPECT_GT(result.value().uses[0].end, result.value().uses[0].start);
      if (partial) {
        EXPECT_GT(result.value().uses[0].start, 0);
        EXPECT_LT(result.value().uses[0].end, 1);
      }
      EXPECT_LT(checks, uint64_t{100000});
    }
}
TEST(OpenSwStepSweptIntersection,
     RejectsUnsupportedTopologyAndEndpointChanges) {
  for (unsigned bad = 1; bad <= 8; ++bad) {
    uint64_t checks = 100000;
    EXPECT_FALSE(Recover(SweptPlaneFixture(), checks, bad).ok()) << bad;
  }
  uint64_t checks = 0;
  const auto limit = Recover(SweptPlaneFixture(), checks);
  ASSERT_FALSE(limit.ok());
  EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStepSweptIntersection, RejectsChartsWithoutGlobalChordUniqueness) {
  auto fixture = SweptPlaneFixture();
  for (auto& n : fixture.report.nodes) {
    if (n.index == 3) n.fields[0].values = {0., 0., 0., .5, 2., 0., 1., 0., 0.};
    if (n.index == 31)
      n.fields.back().values = {0., 0., 2., .5, 1., 2.5, 1., 0., 3.};
    if (n.index == 33) n.fields.back().values = {1., 0., 3.};
  }
  uint64_t checks = 100000;
  const auto result = Recover(fixture, checks);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("globally unique"), std::string::npos);
}
TEST(OpenSwStepSweptIntersection, RejectsParallelAndUnrepresentablePlaneCuts) {
  for (const double z : {0., 1e-16}) {
    auto fixture = SweptPlaneFixture();
    for (auto& n : fixture.report.nodes)
      if (n.index == 21) {
        n.fields[1].values = {1., 0., z};
        n.fields[2].values = {-z, 0., 1.};
      }
    uint64_t checks = 100000;
    EXPECT_FALSE(Recover(fixture, checks).ok());
  }
}
}  // namespace
}  // namespace openswstep
