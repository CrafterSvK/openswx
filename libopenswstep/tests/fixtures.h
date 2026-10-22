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
