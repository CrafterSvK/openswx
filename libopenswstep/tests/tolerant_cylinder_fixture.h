// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_TESTS_TOLERANT_CYLINDER_FIXTURE_H_
#define OPENSWSTEP_TESTS_TOLERANT_CYLINDER_FIXTURE_H_
#include <array>
#include <cmath>
#include <map>

#include "../../libopenparasolid/tests/model_test_helpers.h"
#include "../src/tolerant_cylinder.h"
namespace openswstep::test {
using namespace openparasolid;
using namespace openparasolid::test;
inline Vector3 CarrierCircle(double v) {
  const double w = std::sqrt(.5), a = (1 - v) * (1 - v),
               b = 2 * w * v * (1 - v), c = v * v;
  return {.010 * (a + b) / (a + b + c), .010 * (b + c) / (a + b + c), 0};
}
struct TolerantCylinderFixture {
  ModelFixture source;
  std::array<BoundFinGeometry, 2> fins;
  std::map<uint32_t, Vector3> vertices;
  double first = 0, last = 0;
};
inline void AddAffinePcurve(ModelFixture& f, uint32_t id, uint32_t surface,
                            std::array<double, 2> a, std::array<double, 2> b) {
  f.Add(137, id,
        {Pointer("surface", surface, 1006), Pointer("b_curve", id + 1, 134),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(134, id + 1,
        {Pointer("nurbs", id + 2, 136),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(136, id + 2,
        {Integer("degree", 1, FieldType::kShort), Integer("n_vertices", 2),
         Integer("vertex_dim", 2, FieldType::kShort), Integer("n_knots", 2),
         ByteValue("knot_type", 0), Logical("periodic", false),
         Logical("closed", false), Logical("rational", false),
         ByteValue("curve_form", 0), Pointer("bspline_vertices", id + 3, 45),
         Pointer("knots", 14, 128), Pointer("knot_mult", 15, 127)});
  f.Add(45, id + 3,
        {Array("vertices", FieldType::kDouble, {a[0], a[1], b[0], b[1]})});
}
inline TolerantCylinderFixture CylinderNurbsFixture(bool existing_master = true,
                                                    bool circle_master = false,
                                                    double bow = 3e-7) {
  TolerantCylinderFixture out;
  auto& f = out.source;
  constexpr double a = .45, b = .50, r = .010, height = .020;
  const auto p = CarrierCircle(a), q = CarrierCircle(b);
  out.first = std::atan2(p.y, p.x);
  out.last = std::atan2(q.y, q.x);
  f.Add(
      51, 10,
      {Numbers("origin", {0, 0, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {r}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(124, 11,
        {Pointer("nurbs", 12, 126),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(126, 12,
        {Integer("u_degree", 1, FieldType::kShort),
         Integer("v_degree", 2, FieldType::kShort),
         Integer("n_u_vertices", 2),
         Integer("n_v_vertices", 3),
         Integer("n_u_knots", 2),
         Integer("n_v_knots", 2),
         Logical("u_periodic", false),
         Logical("v_periodic", false),
         Logical("u_closed", false),
         Logical("v_closed", false),
         ByteValue("u_knot_type", 0),
         ByteValue("v_knot_type", 0),
         ByteValue("surface_form", 0),
         Logical("rational", true),
         Integer("vertex_dim", 4, FieldType::kShort),
         Pointer("bspline_vertices", 13, 45),
         Pointer("u_knots", 14, 128),
         Pointer("v_knots", 14, 128),
         Pointer("u_knot_mult", 15, 127),
         Pointer("v_knot_mult", 16, 127)});
  const double w = std::sqrt(.5), middle = (a + b) / 2,
               wm = (1 - middle) * (1 - middle) +
                    2 * w * middle * (1 - middle) + middle * middle;
  const double factor = -4 * bow * wm / ((b - a) * (b - a));
  const std::array<double, 3> z{factor * a * b, factor * (a * b - (a + b) / 2),
                                factor * (1 - a - b + a * b)};
  std::vector<ScalarValue> controls;
  for (int u = 0; u < 2; ++u)
    for (unsigned v = 0; v < 3; ++v) {
      const double weight = v == 1 ? w : 1;
      controls.emplace_back(v == 2 ? 0 : r * weight);
      controls.emplace_back(v == 0 ? 0 : r * weight);
      controls.emplace_back((u ? height / 2 : -height / 2) * weight + z[v]);
      controls.emplace_back(weight);
    }
  f.Add(45, 13, {Array("vertices", FieldType::kDouble, std::move(controls))});
  f.Add(128, 14, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 15,
        {Array("multiplicities", FieldType::kShort, {int64_t{2}, int64_t{2}})});
  f.Add(127, 16,
        {Array("multiplicities", FieldType::kShort, {int64_t{3}, int64_t{3}})});
  f.Add(50, 30,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const auto chart_a = CarrierCircle(.40), chart_b = CarrierCircle(.60);
  if (circle_master)
    f.Add(31, 31,
          {Numbers("center", {0, 0, 0}, FieldType::kVector),
           Numbers("normal", {0, 0, 1}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           Numbers("radius", {r}),
           ByteValue("sense", '+', FieldType::kCharacter)});
  else {
    f.Add(38, 31,
          {{{"surface", FieldType::kPointer, 1006, 2, true},
            {uint64_t{11}, uint64_t{30}}},
           Pointer("chart", 32, 40),
           Pointer("start", 33, 41),
           Pointer("end", 34, 41),
           ByteValue("sense", '+', FieldType::kCharacter)});
    f.Add(40, 32,
          {Numbers("base_parameter", {0}),
           Numbers("base_scale", {1}),
           Integer("chart_count", 2),
           Numbers("chordal_error", {1e-4}),
           Numbers("angular_error", {1}),
           {{"parameter_error", FieldType::kDouble, 0, 2, true},
            {std::monostate{}, std::monostate{}}},
           Array("positions", FieldType::kIntersectionPoint,
                 {chart_a.x, chart_a.y, chart_a.z, chart_b.x, chart_b.y,
                  chart_b.z})});
    for (uint32_t id : {33u, 34u}) {
      const auto point = id == 33 ? chart_a : chart_b;
      f.Add(41, id,
            {ByteValue("type", 'L', FieldType::kCharacter),
             Array("positions", FieldType::kIntersectionPoint,
                   {point.x, point.y, point.z})});
    }
  }
  const auto parameter = [&](Vector3 point) {
    const auto x = chart_b.x - chart_a.x, y = chart_b.y - chart_a.y;
    return ((point.x - chart_a.x) * x + (point.y - chart_a.y) * y) /
           std::hypot(x, y);
  };
  f.Add(133, 50,
        {Pointer("basis_curve", 31, 1008),
         Numbers("point_1", {p.x, p.y, p.z}, FieldType::kVector),
         Numbers("point_2", {q.x, q.y, q.z}, FieldType::kVector),
         Numbers("parm_1", {circle_master ? out.first : parameter(p)}),
         Numbers("parm_2", {circle_master ? out.last : parameter(q)}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(16, 20,
        {Numbers("tolerance", {2e-6}),
         Pointer("curve", existing_master ? 50 : 0, 1008)});
  AddAffinePcurve(f, 100, 10, {out.first, 0}, {out.last, 0});
  AddAffinePcurve(f, 110, 11, {.5, b}, {.5, a});
  const NodeId master{existing_master ? 50u : 0u};
  out.fins[0] = {{NodeId{21}, NodeId{20}, NodeId{23}, NodeId{24}, master,
                  NodeId{100}, TopologySense::kForward},
                 NodeId{40},
                 NodeId{10},
                 NodeId{100},
                 0,
                 1,
                 IntersectionEvaluationOptions{1e-9, 1e-11}};
  out.fins[1] = {{NodeId{22}, NodeId{20}, NodeId{24}, NodeId{23}, master,
                  NodeId{110}, TopologySense::kReverse},
                 NodeId{41},
                 NodeId{11},
                 NodeId{110},
                 0,
                 1,
                 IntersectionEvaluationOptions{1e-9, 1e-11}};
  out.vertices = {{23, p}, {24, q}};
  return out;
}
inline Result<internal::CircularEdgeRecovery> RecoverFixture(
    const TolerantCylinderFixture& f, uint64_t& checks,
    bool source_tolerances = true, double tolerance = 1e-7) {
  const auto model = ModelBuilder::Build(f.source.report);
  if (!model.ok())
    return Result<internal::CircularEdgeRecovery>::Err(model.error());
  const std::array<const BoundFinGeometry*, 2> fins{&f.fins[0], &f.fins[1]};
  return internal::RecoverCylinderNurbsEdge(
      model.value(), fins, f.vertices, tolerance, source_tolerances, checks);
}
}  // namespace openswstep::test
#endif  // OPENSWSTEP_TESTS_TOLERANT_CYLINDER_FIXTURE_H_
