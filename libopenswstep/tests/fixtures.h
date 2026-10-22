// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_TESTS_FIXTURES_H_
#define OPENSWSTEP_TESTS_FIXTURES_H_
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "../../libopenparasolid/tests/model_test_helpers.h"
#include "../../libopenparasolid/tests/test_helpers.h"

namespace openswstep::test {
using namespace openparasolid;
using namespace openparasolid::test;
inline Vector3 Sub(Vector3 a, Vector3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vector3 Unit(Vector3 a) {
  const auto n = std::hypot(a.x, a.y, a.z);
  return {a.x / n, a.y / n, a.z / n};
}
inline Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
struct FixtureFin {
  uint32_t id;
  uint32_t edge;
  uint32_t vertex;
  bool forward;
  int face = -1;
  uint32_t next = 0;
  uint32_t previous = 0;
  uint32_t other = 0;
  uint32_t at_vertex = 0;
};
inline ModelFixture Polyhedron(
    std::vector<Vector3> points,
    const std::vector<std::vector<unsigned>>& polygons, bool solid = false) {
  ModelFixture f;
  struct EdgeData {
    unsigned a, b;
    uint32_t id;
    std::vector<uint32_t> fins;
    bool positive = false, negative = false;
  };
  std::map<std::pair<unsigned, unsigned>, std::size_t> edge_indices;
  std::vector<EdgeData> edges;
  std::map<uint32_t, FixtureFin> fins;
  std::vector<std::vector<uint32_t>> loops;
  for (std::size_t face = 0; face < polygons.size(); ++face) {
    std::vector<uint32_t> loop;
    const auto& polygon = polygons[face];
    for (std::size_t j = 0; j < polygon.size(); ++j) {
      const auto a = polygon[j], b = polygon[(j + 1) % polygon.size()];
      const std::pair<unsigned, unsigned> key{std::min(a, b), std::max(a, b)};
      auto [entry, inserted] = edge_indices.emplace(key, edges.size());
      if (inserted)
        edges.push_back({key.first,
                         key.second,
                         static_cast<uint32_t>(100 + edges.size()),
                         {},
                         false,
                         false});
      auto& edge = edges[entry->second];
      const bool forward = a == edge.a;
      uint32_t id = static_cast<uint32_t>(1000 + entry->second * 4);
      if (forward) {
        if (edge.positive)
          id += 2;
        else
          edge.positive = true;
      } else {
        if (edge.negative)
          id += 2;
        else {
          id += 1;
          edge.negative = true;
        }
      }
      fins.emplace(id, FixtureFin{id, edge.id, 200 + b, forward,
                                  static_cast<int>(face)});
      edge.fins.push_back(id);
      loop.push_back(id);
    }
    for (std::size_t j = 0; j < loop.size(); ++j) {
      fins[loop[j]].next = loop[(j + 1) % loop.size()];
      fins[loop[j]].previous = loop[(j + loop.size() - 1) % loop.size()];
    }
    loops.push_back(std::move(loop));
  }
  if (polygons.empty()) {
    edges.push_back({0, 1, 100, {}, false, false});
  }
  std::vector<std::vector<uint32_t>> vertex_fins(points.size());
  for (std::size_t i = 0; i < edges.size(); ++i) {
    auto& edge = edges[i];
    const auto primary = static_cast<uint32_t>(1000 + i * 4),
               secondary = primary + 1;
    if (!edge.positive)
      fins.emplace(primary, FixtureFin{primary, edge.id, 200 + edge.b, true});
    if (!edge.negative)
      fins.emplace(secondary,
                   FixtureFin{secondary, edge.id, 200 + edge.a, false});
    std::vector<uint32_t> radial{primary, secondary};
    for (const auto id : edge.fins)
      if (id != primary && id != secondary) radial.push_back(id);
    for (std::size_t j = 0; j < radial.size(); ++j)
      fins[radial[j]].other = radial[(j + 1) % radial.size()];
    vertex_fins[edge.b].push_back(primary);
    vertex_fins[edge.a].push_back(secondary);
  }
  for (const auto& list : vertex_fins)
    for (std::size_t i = 0; i < list.size(); ++i)
      fins[list[i]].at_vertex = i + 1 < list.size() ? list[i + 1] : 0;
  const auto count = static_cast<uint32_t>(polygons.size());
  f.Add(
      12, 1,
      {Pointer("owner", 0, 1040), Pointer("region", 2, 19),
       Pointer("child", 0, 12), Pointer("edge", polygons.empty() ? 0 : 100, 16),
       Pointer("vertex", 200, 18),
       ByteValue("body_type", solid              ? 1
                              : polygons.empty() ? 2
                                                 : 3)});
  f.Add(19, 2,
        {Pointer("body", 1, 12), Pointer("previous", 0, 19),
         Pointer("next", solid ? 3 : 0, 19), Pointer("shell", 4, 13),
         ByteValue("type", 'V', FieldType::kCharacter)});
  f.Add(13, 4,
        {Pointer("region", 2, 19), Pointer("next", 0, 13),
         Pointer("face",
                 solid   ? 0
                 : count ? 300
                         : 0,
                 14),
         Pointer("front_face", count ? 300 : 0, 14),
         Pointer("edge", polygons.empty() ? 100 : 0, 16),
         Pointer("vertex", 0, 18)});
  if (solid) {
    f.Add(19, 3,
          {Pointer("body", 1, 12), Pointer("previous", 2, 19),
           Pointer("next", 0, 19), Pointer("shell", 5, 13),
           ByteValue("type", 'S', FieldType::kCharacter)});
    f.Add(13, 5,
          {Pointer("region", 3, 19), Pointer("next", 0, 13),
           Pointer("face", 300, 14), Pointer("front_face", 0, 14),
           Pointer("edge", 0, 16), Pointer("vertex", 0, 18)});
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    f.Add(18, static_cast<uint32_t>(200 + i),
          {Pointer("owner", 1, 1029),
           Pointer("halfedge", vertex_fins[i].empty() ? 0 : vertex_fins[i][0],
                   17),
           Pointer("previous", i ? static_cast<uint32_t>(199 + i) : 0, 18),
           Pointer("next",
                   i + 1 < points.size() ? static_cast<uint32_t>(201 + i) : 0,
                   18),
           Pointer("point", static_cast<uint32_t>(500 + i), 29)});
    f.Add(29, static_cast<uint32_t>(500 + i),
          {Numbers("position", {points[i].x, points[i].y, points[i].z},
                   FieldType::kVector)});
  }
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const auto& edge = edges[i];
    f.Add(16, edge.id,
          {Pointer("owner", polygons.empty() ? 4 : 1, 1029),
           Pointer("previous", i ? edges[i - 1].id : 0, 16),
           Pointer("next", i + 1 < edges.size() ? edges[i + 1].id : 0, 16),
           Pointer("halfedge", static_cast<uint32_t>(1000 + i * 4), 17),
           Pointer("curve", static_cast<uint32_t>(4000 + i), 1008)});
    const auto p = points[edge.a], d = Unit(Sub(points[edge.b], p));
    f.Add(30, static_cast<uint32_t>(4000 + i),
          {Numbers("origin", {p.x, p.y, p.z}, FieldType::kVector),
           Numbers("direction", {d.x, d.y, d.z}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
  }
  for (const auto& [id, fin] : fins) {
    f.Add(
        17, id,
        {Pointer("edge", fin.edge, 16), Pointer("vertex", fin.vertex, 18),
         Pointer("other", fin.other, 17),
         Pointer("loop",
                 fin.face >= 0 ? static_cast<uint32_t>(600 + fin.face) : 0, 15),
         Pointer("forward", fin.next, 17),
         Pointer("backward", fin.previous, 17), Pointer("curve", 0, 1008),
         Pointer("next_at_vx", fin.at_vertex, 17),
         ByteValue("sense", fin.forward ? '+' : '-', FieldType::kCharacter)});
  }
  for (uint32_t i = 0; i < count; ++i) {
    const auto& polygon = polygons[i];
    const auto p = points[polygon[0]], x = Unit(Sub(points[polygon[1]], p)),
               n = Unit(Cross(x, Sub(points[polygon[2]], p)));
    f.Add(15, 600 + i,
          {Pointer("face", 300 + i, 14), Pointer("halfedge", loops[i][0], 17),
           Pointer("next", 0, 15)});
    f.Add(14, 300 + i,
          {Pointer("loop", 600 + i, 15), Pointer("surface", 700 + i, 1006),
           Pointer("shell", solid ? 5 : 4, 13), Pointer("front_shell", 4, 13),
           Pointer("previous", i ? 299 + i : 0, 14),
           Pointer("next", i + 1 < count ? 301 + i : 0, 14),
           Pointer("previous_front", i ? 299 + i : 0, 14),
           Pointer("next_front", i + 1 < count ? 301 + i : 0, 14),
           ByteValue("sense", '+', FieldType::kCharacter)});
    f.Add(50, 700 + i,
          {Numbers("origin", {p.x, p.y, p.z}, FieldType::kVector),
           Numbers("normal", {n.x, n.y, n.z}, FieldType::kVector),
           Numbers("x_axis", {x.x, x.y, x.z}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
  }
  return f;
}
inline ModelFixture Square() {
  return Polyhedron({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
                    {{0, 1, 2, 3}});
}
inline ModelFixture Tetrahedron() {
  return Polyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                    {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}, true);
}
inline void Set(ModelFixture& f, uint32_t id, const char* name,
                uint64_t value) {
  for (auto& node : f.report.nodes)
    if (node.index == id) {
      const auto& fields =
          f.report.schemas[node.schema_index].definition.fields;
      for (std::size_t i = 0; i < fields.size(); ++i)
        if (fields[i].name == name) {
          node.fields[i].values[0] = value;
          return;
        }
    }
}
inline void RemoveNode(ModelFixture& f, uint32_t id) {
  std::erase_if(f.report.nodes,
                [id](const auto& node) { return node.index == id; });
}
// Groups source faces into one face's ordered loop list, retaining fin/edge
// identities. The first face in each group supplies the surface and sense.
inline void MergeFaceLoops(ModelFixture& f,
                           const std::vector<std::vector<uint32_t>>& groups) {
  for (const auto& group : groups) {
    for (std::size_t i = 0; i < group.size(); ++i) {
      Set(f, 600 + group[i], "face", 300 + group.front());
      Set(f, 600 + group[i], "next",
          i + 1 < group.size() ? 600 + group[i + 1] : 0);
      if (i != 0) {
        RemoveNode(f, 300 + group[i]);
        RemoveNode(f, 700 + group[i]);
      }
    }
  }
  std::vector<uint32_t> faces;
  for (const auto& node : f.report.nodes)
    if (node.node_type == 14) faces.push_back(node.index);
  std::sort(faces.begin(), faces.end());
  for (std::size_t i = 0; i < faces.size(); ++i) {
    for (const char* field : {"previous", "previous_front"})
      Set(f, faces[i], field, i ? faces[i - 1] : 0);
    for (const char* field : {"next", "next_front"})
      Set(f, faces[i], field, i + 1 < faces.size() ? faces[i + 1] : 0);
  }
}
inline ModelFixture PlanarFace(const std::vector<Vector3>& points,
                               const std::vector<std::vector<unsigned>>& loops,
                               bool hole_first = false) {
  auto f = Polyhedron(points, loops);
  std::vector<uint32_t> group;
  for (std::size_t i = 0; i < loops.size(); ++i)
    group.push_back(static_cast<uint32_t>(i));
  MergeFaceLoops(f, {group});
  for (auto& node : f.report.nodes)
    if (node.index == 700) {
      node.fields[0].values = {0., 0., 0.};
      node.fields[1].values = {0., 0., 1.};
      node.fields[2].values = {1., 0., 0.};
    }
  if (hole_first && loops.size() > 1) {
    Set(f, 300, "loop", 601);
    Set(f, 600, "next", loops.size() > 2 ? 602 : 0);
    Set(f, 601, "next", 600);
  }
  return f;
}
inline ModelFixture PerforatedSheet(bool hole_first = false) {
  return PlanarFace({{0, 0, 0},
                     {4, 0, 0},
                     {4, 4, 0},
                     {0, 4, 0},
                     {1, 1, 0},
                     {1, 3, 0},
                     {3, 3, 0},
                     {3, 1, 0}},
                    {{0, 1, 2, 3}, {4, 5, 6, 7}}, hole_first);
}
inline ModelFixture TwoHoleSheet() {
  return PlanarFace({{0, 0, 0},
                     {6, 0, 0},
                     {6, 4, 0},
                     {0, 4, 0},
                     {1, 1, 0},
                     {1, 2, 0},
                     {2, 2, 0},
                     {2, 1, 0},
                     {3, 1, 0},
                     {3, 2, 0},
                     {5, 2, 0},
                     {5, 1, 0}},
                    {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}}, true);
}
inline ModelFixture PerforatedPrism() {
  auto f = Polyhedron({{0, 0, 0},
                       {4, 0, 0},
                       {4, 4, 0},
                       {0, 4, 0},
                       {1, 1, 0},
                       {3, 1, 0},
                       {3, 3, 0},
                       {1, 3, 0},
                       {0, 0, 1},
                       {4, 0, 1},
                       {4, 4, 1},
                       {0, 4, 1},
                       {1, 1, 1},
                       {3, 1, 1},
                       {3, 3, 1},
                       {1, 3, 1}},
                      {{0, 3, 2, 1},
                       {4, 5, 6, 7},
                       {8, 9, 10, 11},
                       {12, 15, 14, 13},
                       {0, 1, 9, 8},
                       {1, 2, 10, 9},
                       {2, 3, 11, 10},
                       {3, 0, 8, 11},
                       {4, 12, 13, 5},
                       {5, 13, 14, 6},
                       {6, 14, 15, 7},
                       {7, 15, 12, 4}},
                      true);
  MergeFaceLoops(f, {{0, 1}, {2, 3}});
  return f;
}
inline ModelFixture NegativePerforatedSheet() {
  auto f = PerforatedSheet(true);
  Set(f, 300, "sense", '-');
  for (auto& node : f.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  return f;
}
inline ModelFixture RotateAndTranslate(ModelFixture f) {
  for (auto& node : f.report.nodes) {
    if (node.node_type != 29 && node.node_type != 30 && node.node_type != 31 &&
        node.node_type != 32 && node.node_type != 50 && node.node_type != 51 &&
        node.node_type != 52 && node.node_type != 53 && node.node_type != 54)
      continue;
    const auto& definitions =
        f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < definitions.size(); ++i) {
      if (definitions[i].type != FieldType::kVector) continue;
      const auto old = node.fields[i].values;
      const bool point = definitions[i].name == "position" ||
                         definitions[i].name == "origin" ||
                         definitions[i].name == "center";
      node.fields[i].values = {std::get<double>(old[1]) + (point ? 1e6 : 0),
                               std::get<double>(old[2]) + (point ? -2e6 : 0),
                               std::get<double>(old[0]) + (point ? 3e6 : 0)};
    }
  }
  return f;
}
inline ModelFixture ObliquePerforatedSheet() {
  return RotateAndTranslate(PerforatedSheet(true));
}
inline ModelFixture Conic(bool ellipse = false, bool ring = false,
                          bool reverse = false) {
  auto f = Polyhedron({{ellipse ? 2. : 1., 0, 0}, {0, 1, 0}}, {});
  RemoveNode(f, 4000);
  if (ellipse)
    f.Add(32, 4000,
          {Numbers("center", {0, 0, 0}, FieldType::kVector),
           Numbers("normal", {0, 0, 1}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           Numbers("major_radius", {2}), Numbers("minor_radius", {1}),
           ByteValue("sense", reverse ? '-' : '+', FieldType::kCharacter)});
  else
    f.Add(31, 4000,
          {Numbers("center", {0, 0, 0}, FieldType::kVector),
           Numbers("normal", {0, 0, 1}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           Numbers("radius", {1}),
           ByteValue("sense", reverse ? '-' : '+', FieldType::kCharacter)});
  if (ring) {
    Set(f, 1, "vertex", 0);
    Set(f, 1000, "vertex", 0);
    Set(f, 1001, "vertex", 0);
    RemoveNode(f, 200);
    RemoveNode(f, 201);
  }
  return f;
}
struct ConicLoopSpec {
  Vector3 center{0, 0, 0};
  double major = 1;
  double minor = 1;
  bool ellipse = false;
  bool reverse_curve = false;
  bool reverse_fin = false;
  bool reverse_normal = false;
};
inline ModelFixture ConicSheet(
    const std::vector<ConicLoopSpec>& loops = {ConicLoopSpec{}},
    bool named_seams = false) {
  ModelFixture f;
  f.Add(12, 1,
        {Pointer("owner", 0, 1040), Pointer("region", 2, 19),
         Pointer("child", 0, 12), Pointer("edge", 100, 16),
         Pointer("vertex", named_seams ? 200 : 0, 18),
         ByteValue("body_type", 3)});
  f.Add(19, 2,
        {Pointer("body", 1, 12), Pointer("previous", 0, 19),
         Pointer("next", 0, 19), Pointer("shell", 4, 13),
         ByteValue("type", 'V', FieldType::kCharacter)});
  f.Add(13, 4,
        {Pointer("region", 2, 19), Pointer("next", 0, 13),
         Pointer("face", 300, 14), Pointer("front_face", 300, 14),
         Pointer("edge", 0, 16), Pointer("vertex", 0, 18)});
  f.Add(14, 300,
        {Pointer("loop", 600, 15), Pointer("surface", 700, 1006),
         Pointer("shell", 4, 13), Pointer("front_shell", 4, 13),
         Pointer("previous", 0, 14), Pointer("next", 0, 14),
         Pointer("previous_front", 0, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '+', FieldType::kCharacter)});
  f.Add(50, 700,
        {Numbers("origin", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  for (uint32_t i = 0; i < loops.size(); ++i) {
    const auto& spec = loops[i];
    const uint32_t positive = 1000 + 2 * i, negative = positive + 1;
    const uint32_t used = spec.reverse_fin ? negative : positive;
    f.Add(15, 600 + i,
          {Pointer("face", 300, 14), Pointer("halfedge", used, 17),
           Pointer("next", i + 1 < loops.size() ? 601 + i : 0, 15)});
    f.Add(
        16, 100 + i,
        {Pointer("owner", 1, 1029), Pointer("previous", i ? 99 + i : 0, 16),
         Pointer("next", i + 1 < loops.size() ? 101 + i : 0, 16),
         Pointer("halfedge", positive, 17), Pointer("curve", 4000 + i, 1008)});
    for (const uint32_t id : {positive, negative}) {
      const bool in_loop = id == used;
      f.Add(
          17, id,
          {Pointer("edge", 100 + i, 16),
           Pointer("vertex", named_seams ? 200 + i : 0, 18),
           Pointer("other", id == positive ? negative : positive, 17),
           Pointer("loop", in_loop ? 600 + i : 0, 15),
           Pointer("forward", in_loop ? id : 0, 17),
           Pointer("backward", in_loop ? id : 0, 17), Pointer("curve", 0, 1008),
           Pointer("next_at_vx", named_seams && id == positive ? negative : 0,
                   17),
           ByteValue("sense", id == positive ? '+' : '-',
                     FieldType::kCharacter)});
    }
    if (named_seams) {
      f.Add(18, 200 + i,
            {Pointer("owner", 1, 1029), Pointer("halfedge", positive, 17),
             Pointer("previous", i ? 199 + i : 0, 18),
             Pointer("next", i + 1 < loops.size() ? 201 + i : 0, 18),
             Pointer("point", 500 + i, 29)});
      f.Add(29, 500 + i,
            {Numbers("position",
                     {spec.center.x + spec.major, spec.center.y, spec.center.z},
                     FieldType::kVector)});
    }
    const auto normal = spec.reverse_normal ? -1. : 1.;
    if (spec.ellipse)
      f.Add(32, 4000 + i,
            {Numbers("center", {spec.center.x, spec.center.y, spec.center.z},
                     FieldType::kVector),
             Numbers("normal", {0, 0, normal}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             Numbers("major_radius", {spec.major}),
             Numbers("minor_radius", {spec.minor}),
             ByteValue("sense", spec.reverse_curve ? '-' : '+',
                       FieldType::kCharacter)});
    else
      f.Add(31, 4000 + i,
            {Numbers("center", {spec.center.x, spec.center.y, spec.center.z},
                     FieldType::kVector),
             Numbers("normal", {0, 0, normal}, FieldType::kVector),
             Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
             Numbers("radius", {spec.major}),
             ByteValue("sense", spec.reverse_curve ? '-' : '+',
                       FieldType::kCharacter)});
  }
  return f;
}
inline uint32_t PointerValue(const ModelFixture& f, uint32_t id,
                             const char* field) {
  for (const auto& node : f.report.nodes) {
    if (node.index != id) continue;
    const auto& fields = f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < fields.size(); ++i)
      if (fields[i].name == field)
        return static_cast<uint32_t>(
            std::get<uint64_t>(node.fields[i].values[0]));
  }
  return 0;
}
// Append isolated conic source loops to a face while retaining source chains.
// Each call uses a disjoint offset for the borrowed ConicSheet identities.
inline void AddConicFaceLoops(ModelFixture& f, uint32_t face,
                              const std::vector<ConicLoopSpec>& specs,
                              bool named_seams = false, bool hole_first = false,
                              uint32_t offset = 10000) {
  uint32_t last_edge = 0, last_vertex = 0;
  for (const auto& node : f.report.nodes) {
    if (node.node_type == 16 && !PointerValue(f, node.index, "next"))
      last_edge = node.index;
    if (node.node_type == 18 && !PointerValue(f, node.index, "next"))
      last_vertex = node.index;
  }
  const auto first_loop = PointerValue(f, face, "loop");
  auto last_loop = first_loop;
  while (const auto next = PointerValue(f, last_loop, "next")) last_loop = next;
  const auto conics = ConicSheet(specs, named_seams);
  for (const auto& node : conics.report.nodes) {
    if (node.node_type != 15 && node.node_type != 16 && node.node_type != 17 &&
        node.node_type != 18 && node.node_type != 29 && node.node_type != 31 &&
        node.node_type != 32)
      continue;
    const auto& definitions =
        conics.report.schemas[node.schema_index].definition.fields;
    std::vector<ModelField> fields;
    for (std::size_t i = 0; i < definitions.size(); ++i) {
      auto values = node.fields[i].values;
      if (definitions[i].type == FieldType::kPointer) {
        auto& value = std::get<uint64_t>(values[0]);
        if (value == 300)
          value = face;
        else if (value >= 100)
          value += offset;
      }
      fields.push_back({definitions[i], std::move(values)});
    }
    f.Add(node.node_type, node.index + offset, std::move(fields));
  }
  Set(f, last_edge, "next", 100 + offset);
  Set(f, 100 + offset, "previous", last_edge);
  if (named_seams) {
    Set(f, last_vertex, "next", 200 + offset);
    Set(f, 200 + offset, "previous", last_vertex);
  }
  if (hole_first) {
    Set(f, face, "loop", 600 + offset);
    Set(f, 600 + offset + static_cast<uint32_t>(specs.size()) - 1, "next",
        first_loop);
  } else {
    Set(f, last_loop, "next", 600 + offset);
  }
}
inline ModelFixture PolygonCircularHoles(bool polygon_hole = false,
                                         bool hole_first = false,
                                         bool named_seams = false,
                                         bool concave = false) {
  std::vector<Vector3> points{{0, 0, 0}, {12, 0, 0}, {12, 8, 0}, {0, 8, 0}};
  std::vector<std::vector<unsigned>> loops{{0, 1, 2, 3}};
  if (concave) {
    points = {{0, 0, 0}, {12, 0, 0}, {12, 8, 0}, {8, 8, 0},
              {8, 6, 0}, {4, 6, 0},  {4, 8, 0},  {0, 8, 0}};
    loops = {{0, 1, 2, 3, 4, 5, 6, 7}};
  }
  if (polygon_hole) {
    const auto first = static_cast<unsigned>(points.size());
    points.insert(points.end(), {{5, 3, 0}, {5, 5, 0}, {7, 5, 0}, {7, 3, 0}});
    loops.push_back({first, first + 1, first + 2, first + 3});
  }
  auto f = PlanarFace(points, loops, polygon_hole);
  ConicLoopSpec hole;
  hole.center = {3, 4, 0};
  hole.reverse_curve = true;
  auto second = hole;
  second.center = {9, 4, 0};
  AddConicFaceLoops(f, 300, {hole, second}, named_seams, hole_first);
  return f;
}
inline ModelFixture CircularPolygonHoles(bool two_polygons = false,
                                         bool circular_hole = false,
                                         bool outer_first = false,
                                         bool named_seams = false,
                                         bool concave_hole = false) {
  std::vector<Vector3> points{{-1, -1, 0}, {-1, 1, 0}, {1, 1, 0}, {1, -1, 0}};
  std::vector<std::vector<unsigned>> loops{{0, 1, 2, 3}};
  if (two_polygons) {
    points = {{-3, -1, 0}, {-3, 1, 0}, {-1, 1, 0}, {-1, -1, 0},
              {1, -1, 0},  {1, 1, 0},  {3, 1, 0},  {3, -1, 0}};
    loops = {{0, 1, 2, 3}, {4, 5, 6, 7}};
  } else if (concave_hole) {
    points = {{-1, -1, 0}, {-1, 1, 0}, {0, 1, 0},
              {0, 0, 0},   {1, 0, 0},  {1, -1, 0}};
    loops = {{0, 1, 2, 3, 4, 5}};
  }
  auto f = PlanarFace(points, loops);
  ConicLoopSpec outer;
  outer.major = 6;
  std::vector<ConicLoopSpec> circles{outer};
  if (circular_hole) {
    ConicLoopSpec inner;
    inner.center = {0, 3, 0};
    inner.reverse_curve = true;
    circles.push_back(inner);
  }
  AddConicFaceLoops(f, 300, circles, named_seams, outer_first);
  return f;
}
inline ModelFixture ReorderedCircularPolygonHoles() {
  auto f = CircularPolygonHoles(false, true);
  Set(f, 600, "next", 10601);
  Set(f, 10601, "next", 10600);
  Set(f, 10600, "next", 0);
  return f;
}
inline ModelFixture CircularAnnulus(bool hole_first = false,
                                    bool named_seams = false) {
  ConicLoopSpec outer;
  outer.major = 3;
  ConicLoopSpec inner;
  inner.reverse_curve = true;
  return ConicSheet(
      hole_first ? std::vector{inner, outer} : std::vector{outer, inner},
      named_seams);
}
inline ModelFixture CircularHoles() {
  ConicLoopSpec outer;
  outer.major = 4;
  ConicLoopSpec inner;
  inner.center = {-2, 0, 0};
  inner.reverse_curve = true;
  auto second = inner;
  second.center = {2, 0, 0};
  return ConicSheet({inner, second, outer});
}
// Builds circular caps and one or two complete cylindrical bands. Every
// boundary edge is shared by its cap and mantle using opposite source fins.
inline ModelFixture Cylinder(bool tube = false, bool named_seams = false,
                             bool solid = true, bool mantle_only = false) {
  ConicLoopSpec bottom;
  bottom.major = 3;
  bottom.reverse_fin = true;
  auto top = bottom;
  top.center.z = 2;
  top.reverse_fin = false;
  std::vector<ConicLoopSpec> circles{bottom, top};
  if (tube) {
    auto inner_bottom = bottom;
    inner_bottom.major = 1;
    inner_bottom.reverse_fin = false;
    auto inner_top = inner_bottom;
    inner_top.center.z = 2;
    inner_top.reverse_fin = true;
    circles.insert(circles.end(), {inner_bottom, inner_top});
  }
  auto f = ConicSheet(circles, named_seams);
  Set(f, 1, "body_type", solid ? 1 : 3);
  if (solid) {
    Set(f, 2, "next", 3);
    Set(f, 4, "face", 0);
    f.Add(19, 3,
          {Pointer("body", 1, 12), Pointer("previous", 2, 19),
           Pointer("next", 0, 19), Pointer("shell", 5, 13),
           ByteValue("type", 'S', FieldType::kCharacter)});
    f.Add(13, 5,
          {Pointer("region", 3, 19), Pointer("next", 0, 13),
           Pointer("face", 300, 14), Pointer("front_face", 0, 14),
           Pointer("edge", 0, 16), Pointer("vertex", 0, 18)});
  }
  const uint32_t count = tube ? 4 : 3;
  for (uint32_t i = 0; i < count; ++i) {
    if (i == 0) RemoveNode(f, 300);
    f.Add(14, 300 + i,
          {Pointer("loop",
                   i < 2    ? 600 + i
                   : i == 2 ? 604
                            : 606,
                   15),
           Pointer("surface", 700 + i, 1006),
           Pointer("shell", solid ? 5 : 4, 13), Pointer("front_shell", 4, 13),
           Pointer("previous", i ? 299 + i : 0, 14),
           Pointer("next", i + 1 < count ? 301 + i : 0, 14),
           Pointer("previous_front", i ? 299 + i : 0, 14),
           Pointer("next_front", i + 1 < count ? 301 + i : 0, 14),
           ByteValue("sense", i == 0 || i == 3 ? '-' : '+',
                     FieldType::kCharacter)});
  }
  Set(f, 600, "next", tube ? 602 : 0);
  Set(f, 601, "face", 301);
  Set(f, 601, "next", tube ? 603 : 0);
  if (tube) {
    Set(f, 602, "next", 0);
    Set(f, 603, "next", 0);
    Set(f, 603, "face", 301);
  }
  f.Add(50, 701,
        {Numbers("origin", {0, 0, 2}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  for (uint32_t mantle = 0; mantle < (tube ? 2u : 1u); ++mantle) {
    f.Add(51, 702 + mantle,
          {Numbers("origin", {0, 0, 0}, FieldType::kVector),
           Numbers("axis", {0, 0, 1}, FieldType::kVector),
           Numbers("radius", {mantle ? 1. : 3.}),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
    const uint32_t first_loop = 604 + mantle * 2;
    const uint32_t first_fin = mantle ? 1005 : 1000;
    const uint32_t second_fin = mantle ? 1006 : 1003;
    for (uint32_t j = 0; j < 2; ++j) {
      const uint32_t fin = j ? second_fin : first_fin;
      f.Add(15, first_loop + j,
            {Pointer("face", 302 + mantle, 14), Pointer("halfedge", fin, 17),
             Pointer("next", j ? 0 : first_loop + 1, 15)});
      Set(f, fin, "loop", first_loop + j);
      Set(f, fin, "forward", fin);
      Set(f, fin, "backward", fin);
    }
  }
  if (mantle_only) {
    for (const auto id : {300u, 301u, 600u, 601u, 700u, 701u})
      RemoveNode(f, id);
    Set(f, 4, "face", 302);
    Set(f, 4, "front_face", 302);
    Set(f, 302, "previous", 0);
    Set(f, 302, "previous_front", 0);
    for (const uint32_t fin : {1001u, 1002u}) {
      Set(f, fin, "loop", 0);
      Set(f, fin, "forward", 0);
      Set(f, fin, "backward", 0);
    }
  }
  return f;
}
inline ModelFixture SphericalBand(double lower = 0, double upper = 3,
                                  bool tube = false, bool named_seams = false,
                                  bool solid = true, bool mantle_only = false) {
  auto f = Cylinder(tube, named_seams, solid, mantle_only);
  const double radii[]{std::sqrt(25 - lower * lower),
                       std::sqrt(25 - upper * upper)};
  for (auto& node : f.report.nodes) {
    if (node.index >= 4000 && node.index <= 4003) {
      const auto i = node.index - 4000;
      node.fields[0].values = {0., 0., i % 2 ? upper : lower};
      if (i < 2) node.fields[3].values = {radii[i]};
    }
    if (node.index == 700 || node.index == 701)
      node.fields[0].values = {0., 0., node.index == 700 ? lower : upper};
    if (node.index >= 500 && node.index <= 503) {
      const auto i = node.index - 500;
      node.fields[0].values = {i < 2 ? radii[i] : 1., 0.,
                               i % 2 ? upper : lower};
    }
  }
  RemoveNode(f, 702);
  f.Add(
      53, 702,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {5}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  return f;
}
inline ModelFixture SphericalCap(double level = 3, bool north = true,
                                 bool named_seams = false, bool solid = true) {
  ConicLoopSpec circle;
  circle.center.z = level;
  circle.major = std::sqrt(25 - level * level);
  circle.reverse_fin = solid ? north : !north;
  auto f = ConicSheet({circle}, named_seams);
  if (!solid) RemoveNode(f, 700);
  f.Add(
      53, solid ? 701 : 700,
      {Numbers("center", {0, 0, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector), Numbers("radius", {5}),
       ByteValue("sense", '+', FieldType::kCharacter)});
  if (!solid) return f;
  Set(f, 1, "body_type", 1);
  Set(f, 2, "next", 3);
  Set(f, 4, "face", 0);
  Set(f, 300, "shell", 5);
  Set(f, 300, "next", 301);
  Set(f, 300, "next_front", 301);
  Set(f, 300, "sense", north ? '-' : '+');
  for (auto& node : f.report.nodes)
    if (node.index == 700) node.fields[0].values = {0., 0., level};
  f.Add(19, 3,
        {Pointer("body", 1, 12), Pointer("previous", 2, 19),
         Pointer("next", 0, 19), Pointer("shell", 5, 13),
         ByteValue("type", 'S', FieldType::kCharacter)});
  f.Add(13, 5,
        {Pointer("region", 3, 19), Pointer("next", 0, 13),
         Pointer("face", 300, 14), Pointer("front_face", 0, 14),
         Pointer("edge", 0, 16), Pointer("vertex", 0, 18)});
  f.Add(14, 301,
        {Pointer("loop", 601, 15), Pointer("surface", 701, 1006),
         Pointer("shell", 5, 13), Pointer("front_shell", 4, 13),
         Pointer("previous", 300, 14), Pointer("next", 0, 14),
         Pointer("previous_front", 300, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '+', FieldType::kCharacter)});
  const uint32_t fin = north ? 1000 : 1001;
  f.Add(15, 601,
        {Pointer("face", 301, 14), Pointer("halfedge", fin, 17),
         Pointer("next", 0, 15)});
  Set(f, fin, "loop", 601);
  Set(f, fin, "forward", fin);
  Set(f, fin, "backward", fin);
  return f;
}
inline ModelFixture PartitionedSphere(double level = 0,
                                      bool named_seams = false) {
  auto f = SphericalCap(level, true, named_seams);
  // The former closing disk becomes the complementary outward sphere cap.
  Set(f, 300, "surface", 701);
  Set(f, 300, "sense", '+');
  RemoveNode(f, 700);
  return f;
}
inline ModelFixture ReversedSphericalCap() {
  auto f = SphericalCap();
  for (auto& node : f.report.nodes)
    if (node.index == 701) node.fields[1].values = {0., 0., -1.};
  return f;
}
inline ModelFixture InwardSphericalCap() {
  auto f = SphericalCap(3, true, false, false);
  Set(f, 300, "sense", '-');
  Set(f, 4000, "sense", '-');
  return f;
}
inline ModelFixture ReversedSphericalBand(bool axis = false) {
  auto f = SphericalBand();
  if (axis) {
    for (auto& node : f.report.nodes)
      if (node.index == 702) node.fields[1].values = {0., 0., -1.};
  } else {
    Set(f, 302, "loop", 605);
    Set(f, 605, "next", 604);
    Set(f, 604, "next", 0);
  }
  return f;
}
inline ModelFixture InwardSphericalBand() {
  auto f = SphericalBand(0, 3, false, false, false, true);
  Set(f, 302, "sense", '-');
  Set(f, 4000, "sense", '-');
  Set(f, 4001, "sense", '-');
  return f;
}
inline ModelFixture ReversedCylinder(bool axis = false) {
  auto f = Cylinder();
  if (axis) {
    for (auto& node : f.report.nodes)
      if (node.index == 702) node.fields[1].values = {0., 0., -1.};
  } else {
    Set(f, 302, "loop", 605);
    Set(f, 605, "next", 604);
    Set(f, 604, "next", 0);
  }
  return f;
}
inline ModelFixture ShiftedCylinderSeams(bool named_seams = true) {
  auto f = Cylinder(false, named_seams);
  for (auto& node : f.report.nodes) {
    if (named_seams && (node.index == 500 || node.index == 501)) {
      const auto phase = node.index == 500 ? 0.3 : 1.1;
      node.fields[0].values = {3 * std::cos(phase), 3 * std::sin(phase),
                               node.index == 500 ? 0. : 2.};
    }
    if (!named_seams && (node.index == 4000 || node.index == 4001)) {
      const auto phase = node.index == 4000 ? 0.3 : 1.1;
      node.fields[2].values = {std::cos(phase), std::sin(phase), 0.};
    }
  }
  return f;
}
inline ModelFixture Frustum(bool narrowing = false, bool tube = false,
                            bool named_seams = false, bool solid = true,
                            bool mantle_only = false) {
  auto f = Cylinder(tube, named_seams, solid, mantle_only);
  const double slope = narrowing ? -1. : 1.;
  for (uint32_t mantle = 0; mantle < (tube ? 2u : 1u); ++mantle) {
    const double base = mantle ? 1. : 3.;
    RemoveNode(f, 702 + mantle);
    f.Add(52, 702 + mantle,
          {Numbers("origin", {0, 0, 0}, FieldType::kVector),
           Numbers("axis", {0, 0, 1}, FieldType::kVector),
           Numbers("radius", {base}),
           Numbers("sin_half_angle", {slope / std::sqrt(2.)}),
           Numbers("cos_half_angle", {1 / std::sqrt(2.)}),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           ByteValue("sense", '+', FieldType::kCharacter)});
    for (auto& node : f.report.nodes) {
      if (node.index == 4001 + 2 * mantle)
        node.fields[3].values = {base + 2 * slope};
      if (named_seams && node.index == 501 + 2 * mantle)
        node.fields[0].values = {base + 2 * slope, 0., 2.};
    }
  }
  return f;
}
inline ModelFixture ReversedFrustum(bool axis = false) {
  auto f = Frustum();
  if (axis) {
    for (auto& node : f.report.nodes)
      if (node.index == 702) {
        node.fields[1].values = {0., 0., -1.};
        node.fields[3].values = {-1 / std::sqrt(2.)};
      }
  } else {
    Set(f, 302, "loop", 605);
    Set(f, 605, "next", 604);
    Set(f, 604, "next", 0);
  }
  return f;
}
inline ModelFixture FrustumAtApexOrigin() {
  auto f = Frustum();
  for (auto& node : f.report.nodes)
    if (node.index == 702) {
      node.fields[0].values = {0., 0., -3.};
      node.fields[2].values = {0.};
    }
  return f;
}
inline ModelFixture ToroidalBand(bool solid = true, bool named_seams = false,
                                 bool complement = false, bool tube = false) {
  auto f = Cylinder(tube, named_seams, solid, !solid);
  for (auto& node : f.report.nodes) {
    if (node.index == 4000) node.fields[3].values = {4.};
    if (node.index == 4001 || (tube && node.index == 4003))
      node.fields[0].values = {0., 0., 1.};
    if (node.index == 701) node.fields[0].values = {0., 0., 1.};
    if (named_seams && node.index == 500) node.fields[0].values = {4., 0., 0.};
    if (named_seams && node.index == 501) node.fields[0].values = {3., 0., 1.};
    if (named_seams && tube && node.index == 503)
      node.fields[0].values = {1., 0., 1.};
  }
  RemoveNode(f, 702);
  f.Add(54, 702,
        {Numbers("center", {0, 0, 0}, FieldType::kVector),
         Numbers("axis", {0, 0, 1}, FieldType::kVector),
         Numbers("major_radius", {3}), Numbers("minor_radius", {1}),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         ByteValue("sense", '+', FieldType::kCharacter)});
  if (complement) {
    Set(f, 4000, "sense", '-');
    Set(f, 4001, "sense", '-');
  }
  return f;
}
inline ModelFixture ReversedToroidalBand(bool axis = false) {
  auto f = ToroidalBand();
  if (axis) {
    for (auto& node : f.report.nodes)
      if (node.index == 702) node.fields[1].values = {0., 0., -1.};
  } else {
    Set(f, 302, "loop", 605);
    Set(f, 605, "next", 604);
    Set(f, 604, "next", 0);
  }
  return f;
}
inline ModelFixture ToroidalTube() {
  return ToroidalBand(true, true, false, true);
}
inline ModelFixture InwardToroidalBand() {
  auto f = ToroidalBand(false, false, true);
  Set(f, 302, "sense", '-');
  return f;
}
inline ModelFixture ToroidalProfileBand(bool across_seam) {
  auto f = ToroidalBand(false);
  const auto q = std::sqrt(0.5);
  for (auto& node : f.report.nodes) {
    if (node.index == 4000 && across_seam) {
      node.fields[0].values = {0., 0., q};
      node.fields[3].values = {3 - q};
    }
    if (node.index == 4001) {
      node.fields[0].values = {0., 0., across_seam ? -q : 0.};
      node.fields[3].values = {across_seam ? 3 - q : 2.};
    }
  }
  return f;
}
inline void CircleBasis(ModelFixture& f, uint32_t id, Vector3 center,
                        double radius = 1, char sense = '+') {
  RemoveNode(f, id);
  f.Add(31, id,
        {Numbers("center", {center.x, center.y, center.z}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         Numbers("radius", {radius}),
         ByteValue("sense", sense, FieldType::kCharacter)});
}
inline ModelFixture QuarterDisk() {
  auto f = Polyhedron({{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}, {{0, 1, 2}});
  CircleBasis(f, 4000, {0, 0, 0});
  return f;
}
inline ModelFixture RoundedRectangle(bool reverse = false) {
  std::vector<unsigned> polygon{0, 1, 2, 3, 4, 5, 6, 7};
  if (reverse) std::reverse(polygon.begin(), polygon.end());
  auto f = Polyhedron({{1, 0, 0},
                       {3, 0, 0},
                       {4, 1, 0},
                       {4, 2, 0},
                       {3, 3, 0},
                       {1, 3, 0},
                       {0, 2, 0},
                       {0, 1, 0}},
                      {polygon});
  struct Arc {
    unsigned first, last;
    Vector3 center;
  };
  for (const auto arc : {Arc{1, 2, {3, 1, 0}}, Arc{3, 4, {3, 2, 0}},
                         Arc{5, 6, {1, 2, 0}}, Arc{7, 0, {1, 1, 0}}}) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto a = polygon[i], b = polygon[(i + 1) % polygon.size()];
      if (std::min(a, b) == std::min(arc.first, arc.last) &&
          std::max(a, b) == std::max(arc.first, arc.last))
        CircleBasis(f, static_cast<uint32_t>(4000 + i), arc.center, 1,
                    arc.first < arc.last ? '+' : '-');
    }
  }
  return f;
}
inline ModelFixture HalfDisk() {
  auto f = Polyhedron({{-1, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}});
  // Keep the chord and one upper semicircle; reconnect the two source vertices
  // and their anchor fin lists after removing the triangle's third edge.
  for (const auto id : {102u, 1008u, 1009u, 4002u, 202u, 502u})
    RemoveNode(f, id);
  Set(f, 101, "next", 0);
  Set(f, 201, "next", 0);
  Set(f, 1000, "forward", 1004);
  Set(f, 1000, "backward", 1004);
  Set(f, 1004, "forward", 1000);
  Set(f, 1004, "backward", 1000);
  Set(f, 1004, "vertex", 200);
  Set(f, 1001, "next_at_vx", 1004);
  Set(f, 1004, "next_at_vx", 0);
  CircleBasis(f, 4001, {0, 0, 0});
  return f;
}
inline ModelFixture HalfEllipse() {
  auto f = HalfDisk();
  for (auto& node : f.report.nodes) {
    if (node.index == 500 || node.index == 4000)
      node.fields[0].values = {-2., 0., 0.};
    if (node.index == 501) node.fields[0].values = {2., 0., 0.};
  }
  RemoveNode(f, 4001);
  f.Add(32, 4001,
        {Numbers("center", {0, 0, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         Numbers("major_radius", {2}), Numbers("minor_radius", {1}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  return f;
}
inline ModelFixture CircularLens() {
  auto f = HalfDisk();
  CircleBasis(f, 4000, {0, 1, 0}, std::sqrt(2.));
  CircleBasis(f, 4001, {0, -1, 0}, std::sqrt(2.));
  return f;
}
inline ModelFixture MajorCircularSegment() {
  auto f = HalfDisk();
  const auto q = std::sqrt(0.5);
  for (auto& node : f.report.nodes) {
    if (node.index == 500) node.fields[0].values = {1., 0., 0.};
    if (node.index == 501) node.fields[0].values = {0., 1., 0.};
    if (node.index == 4000) {
      node.fields[0].values = {1., 0., 0.};
      node.fields[1].values = {-q, q, 0.};
    }
  }
  return f;
}
inline ModelFixture SplitCircle(bool double_turn = false) {
  std::vector<Vector3> points;
  const unsigned count = double_turn ? 5 : 4;
  const auto turn = 2 * std::acos(-1.);
  for (unsigned i = 0; i < count; ++i)
    points.push_back(
        {std::cos(turn * i / count), std::sin(turn * i / count), 0});
  const std::vector<unsigned> polygon =
      double_turn ? std::vector<unsigned>{0, 2, 4, 1, 3}
                  : std::vector<unsigned>{0, 1, 2, 3};
  auto f = Polyhedron(points, {polygon});
  for (unsigned i = 0; i < count; ++i)
    CircleBasis(f, 4000 + i, {0, 0, 0}, 1,
                polygon[i] < polygon[(i + 1) % count] ? '+' : '-');
  return f;
}
inline ModelFixture ConcaveArcFace() {
  auto f = Polyhedron({{0, 0, 0},
                       {4, 0, 0},
                       {4, 4, 0},
                       {3, 4, 0},
                       {3, 2, 0},
                       {1, 2, 0},
                       {1, 4, 0},
                       {0, 4, 0}},
                      {{0, 1, 2, 3, 4, 5, 6, 7}});
  CircleBasis(f, 4004, {2, 2, 0}, 1, '-');
  return f;
}
// Affinely stretch XY-plane LINE/CIRCLE fixtures into exact ellipse profiles.
// Source circle frames in these fixtures have normal +Z and reference X +X.
inline ModelFixture EllipticProfile(ModelFixture f, double stretch = 2) {
  struct Replacement {
    uint32_t id;
    Vector3 center;
    double radius;
    uint64_t sense;
  };
  std::vector<Replacement> replacements;
  for (auto& node : f.report.nodes) {
    if (node.node_type == 31) {
      replacements.push_back(
          {node.index,
           {stretch * std::get<double>(node.fields[0].values[0]),
            std::get<double>(node.fields[0].values[1]),
            std::get<double>(node.fields[0].values[2])},
           std::get<double>(node.fields[3].values[0]),
           std::get<uint64_t>(node.fields[4].values[0])});
    } else if (node.node_type == 29 || node.node_type == 30) {
      node.fields[0].values[0] =
          stretch * std::get<double>(node.fields[0].values[0]);
      if (node.node_type == 30) {
        auto& direction = node.fields[1].values;
        const auto unit = Unit({stretch * std::get<double>(direction[0]),
                                std::get<double>(direction[1]),
                                std::get<double>(direction[2])});
        direction = {unit.x, unit.y, unit.z};
      }
    }
  }
  for (const auto& replacement : replacements) {
    RemoveNode(f, replacement.id);
    f.Add(32, replacement.id,
          {Numbers("center",
                   {replacement.center.x, replacement.center.y,
                    replacement.center.z},
                   FieldType::kVector),
           Numbers("normal", {0, 0, 1}, FieldType::kVector),
           Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
           Numbers("major_radius", {stretch * replacement.radius}),
           Numbers("minor_radius", {replacement.radius}),
           ByteValue("sense", replacement.sense, FieldType::kCharacter)});
  }
  return f;
}
inline ModelFixture MixedConicLens() {
  auto f = CircularLens();
  RemoveNode(f, 4001);
  f.Add(32, 4001,
        {Numbers("center", {0, -1, 0}, FieldType::kVector),
         Numbers("normal", {0, 0, 1}, FieldType::kVector),
         Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         Numbers("major_radius", {2}),
         Numbers("minor_radius", {2 / std::sqrt(3.)}),
         ByteValue("sense", '+', FieldType::kCharacter)});
  return f;
}
inline ModelFixture Acorn() {
  auto f = Polyhedron({{0, 0, 0}, {1, 0, 0}}, {});
  Set(f, 4, "edge", 0);
  Set(f, 4, "vertex", 200);
  Set(f, 200, "owner", 4);
  Set(f, 200, "halfedge", 0);
  Set(f, 200, "next", 0);
  for (const auto id : {100u, 1000u, 1001u, 201u, 4000u}) RemoveNode(f, id);
  return f;
}
inline ModelFixture Cube() {
  return Polyhedron({{0, 0, 0},
                     {1, 0, 0},
                     {1, 1, 0},
                     {0, 1, 0},
                     {0, 0, 1},
                     {1, 0, 1},
                     {1, 1, 1},
                     {0, 1, 1}},
                    {{0, 3, 2, 1},
                     {4, 5, 6, 7},
                     {0, 1, 5, 4},
                     {1, 2, 6, 5},
                     {2, 3, 7, 6},
                     {3, 0, 4, 7}},
                    true);
}
inline ModelFixture CubeWithCircularBore(bool named_seams = false) {
  auto f = Cube();
  ConicLoopSpec bottom;
  bottom.center = {.5, .5, 0};
  bottom.major = .25;
  auto top = bottom;
  top.center.z = 1;
  top.reverse_curve = true;
  AddConicFaceLoops(f, 300, {bottom}, named_seams, false, 10000);
  AddConicFaceLoops(f, 301, {top}, named_seams, true, 20000);
  Set(f, 305, "next", 306);
  Set(f, 305, "next_front", 306);
  f.Add(14, 306,
        {Pointer("loop", 606, 15), Pointer("surface", 706, 1006),
         Pointer("shell", 5, 13), Pointer("front_shell", 4, 13),
         Pointer("previous", 305, 14), Pointer("next", 0, 14),
         Pointer("previous_front", 305, 14), Pointer("next_front", 0, 14),
         ByteValue("sense", '-', FieldType::kCharacter)});
  f.Add(
      51, 706,
      {Numbers("origin", {.5, .5, 0}, FieldType::kVector),
       Numbers("axis", {0, 0, 1}, FieldType::kVector), Numbers("radius", {.25}),
       Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
       ByteValue("sense", '+', FieldType::kCharacter)});
  for (uint32_t i = 0; i < 2; ++i) {
    const auto fin = 1001 + (i ? 20000u : 10000u);
    f.Add(15, 606 + i,
          {Pointer("face", 306, 14), Pointer("halfedge", fin, 17),
           Pointer("next", i ? 0 : 607, 15)});
    Set(f, fin, "loop", 606 + i);
    Set(f, fin, "forward", fin);
    Set(f, fin, "backward", fin);
  }
  return f;
}
struct Serialized {
  std::vector<uint8_t> bytes;
  SchemaCatalog catalog;
};
inline Serialized Serialize(const ModelFixture& f) {
  Serialized out{Header(),
                 {"SCH_1234567_12345_13006", "synthetic fixture", {}, {}}};
  std::set<uint16_t> defined;
  for (const auto& node : f.report.nodes) {
    U16(out.bytes, node.node_type);
    if (defined.insert(node.node_type).second) {
      out.catalog.base_types.push_back(
          f.report.schemas[node.schema_index].definition);
      out.bytes.push_back(255);
    }
    CompactIndex(out.bytes, node.index);
    const auto& definitions =
        f.report.schemas[node.schema_index].definition.fields;
    for (std::size_t i = 0; i < node.fields.size(); ++i) {
      const auto type = definitions[i].type;
      for (const auto& value : node.fields[i].values) {
        if (type == FieldType::kPointer)
          CompactIndex(out.bytes,
                       static_cast<uint32_t>(std::get<uint64_t>(value)));
        else if (type == FieldType::kByte || type == FieldType::kCharacter)
          out.bytes.push_back(static_cast<uint8_t>(std::get<uint64_t>(value)));
        else
          F64(out.bytes, std::get<double>(value));
      }
    }
  }
  U16(out.bytes, 1);
  CompactIndex(out.bytes, 0);
  return out;
}
}  // namespace openswstep::test
#endif  // OPENSWSTEP_TESTS_FIXTURES_H_
