// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_TESTS_RATIONAL_STRIP_FIXTURE_H_
#define OPENSWSTEP_TESTS_RATIONAL_STRIP_FIXTURE_H_
#include "fixtures.h"
namespace openswstep::test {
// Original curved wedge. Its circular front footprint adds pi/8-1/4 to the
// unit square. Symmetry makes the X centroid 1/2, so V=(9/4)*(3/4+pi/8).
// The slanted top is a direct plane/swept intersection, not a stored spline.
inline ModelFixture RationalVariableWedge() {
  auto f = Polyhedron({{0, 0, 0},
                       {1, 0, 0},
                       {1, 1, 0},
                       {0, 1, 0},
                       {0, 0, 2},
                       {1, 0, 2.5},
                       {1, 1, 2.5},
                       {0, 1, 2}},
                      {{0, 3, 2, 1},
                       {4, 5, 6, 7},
                       {0, 1, 5, 4},
                       {1, 2, 6, 5},
                       {2, 3, 7, 6},
                       {3, 0, 4, 7}},
                      true);
  RemoveNode(f, 4003);
  RemoveNode(f, 4004);
  RemoveNode(f, 702);
  f.Add(134, 4003,
        {Pointer("nurbs", 12000, 136),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(136, 12000,
        {Integer("degree", 2, FieldType::kShort), Integer("n_vertices", 3),
         Integer("vertex_dim", 4, FieldType::kShort), Integer("n_knots", 2),
         ByteValue("knot_type", 0), Logical("periodic", false),
         Logical("closed", false), Logical("rational", true),
         ByteValue("curve_form", 0), Pointer("bspline_vertices", 12001, 45),
         Pointer("knots", 12002, 128), Pointer("knot_mult", 12003, 127)});
  const double w = std::sqrt(.5);
  f.Add(45, 12001,
        {Array("vertices", FieldType::kDouble,
               {0., 0., 0., 1., .5 * w, -.5 * w, 0., w, 1., 0., 0., 1.})});
  f.Add(128, 12002, {Array("knots", FieldType::kDouble, {0., 1.})});
  f.Add(127, 12003,
        {Array("multiplicities", FieldType::kShort, {int64_t{3}, int64_t{3}})});
  f.Add(67, 702,
        {Pointer("section", 4003, 1008), Numbers("scale", {42}),
         Numbers("sweep", {0, 0, 1}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(38, 4004,
        {{{"surface", FieldType::kPointer, 1006, 2, true},
          {uint64_t{701}, uint64_t{702}}},
         Pointer("chart", 12010, 40),
         Pointer("start", 12011, 41),
         Pointer("end", 12012, 41),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(40, 12010,
        {Numbers("base_parameter", {0}),
         Numbers("base_scale", {1}),
         Integer("chart_count", 3),
         Numbers("chordal_error", {.1}),
         Numbers("angular_error", {1}),
         {{"parameter_error", FieldType::kDouble, 0, 2, true},
          {std::monostate{}, std::monostate{}}},
         Array("positions", FieldType::kIntersectionPoint,
               {0., 0., 2., .5, -.5 * w / (1 + w), 2.25, 1., 0., 2.5})});
  f.Add(41, 12011,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {0., 0., 2.})});
  f.Add(41, 12012,
        {ByteValue("type", 'L', FieldType::kCharacter),
         Array("positions", FieldType::kIntersectionPoint, {1., 0., 2.5})});
  return f;
}
}  // namespace openswstep::test
#endif  // OPENSWSTEP_TESTS_RATIONAL_STRIP_FIXTURE_H_
