// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "tolerant_cylinder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <set>

namespace openswstep::internal {
namespace {
using namespace openparasolid;
constexpr auto kInfinity = std::numeric_limits<long double>::infinity();
struct I {
  long double lo = 0, hi = 0;
};
long double Down(long double x) { return std::nextafter(x, -kInfinity); }
long double Up(long double x) { return std::nextafter(x, kInfinity); }
bool Valid(I a) {
  return std::isfinite(a.lo) && std::isfinite(a.hi) && a.lo <= a.hi;
}
I Bad() { return {-kInfinity, kInfinity}; }
I E(long double x) { return {x, x}; }
I Add(I a, I b) {
  if (!Valid(a) || !Valid(b)) return Bad();
  return {Down(a.lo + b.lo), Up(a.hi + b.hi)};
}
I Sub(I a, I b) {
  if (!Valid(a) || !Valid(b)) return Bad();
  return {Down(a.lo - b.hi), Up(a.hi - b.lo)};
}
I Mul(I a, I b) {
  if (!Valid(a) || !Valid(b)) return Bad();
  const std::array<long double, 4> q{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo,
                                     a.hi * b.hi};
  for (auto v : q)
    if (!std::isfinite(v)) return Bad();
  return {Down(*std::min_element(q.begin(), q.end())),
          Up(*std::max_element(q.begin(), q.end()))};
}
I Div(I a, I b) {
  if (!Valid(a) || !Valid(b) || b.lo <= 0) return Bad();
  return Mul(a, {Down(1 / b.hi), Up(1 / b.lo)});
}
I SignedDiv(I a, I b) {
  if (b.hi < 0) return Div(Mul(a, E(-1)), Mul(b, E(-1)));
  return Div(a, b);
}
long double Abs(I a) { return std::max(std::abs(a.lo), std::abs(a.hi)); }
using H = std::array<I, 4>;
using V = std::array<I, 3>;
V Vec(Vector3 p) { return {E(p.x), E(p.y), E(p.z)}; }
V Diff(Vector3 a, Vector3 b) {
  return {Sub(E(a.x), E(b.x)), Sub(E(a.y), E(b.y)), Sub(E(a.z), E(b.z))};
}
I Dot(V a, V b) {
  I r{};
  for (unsigned k = 0; k < 3; ++k) r = Add(r, Mul(a[k], b[k]));
  return r;
}
long double Length(V a) {
  long double sum = 0;
  for (auto c : a) {
    if (!Valid(c)) return kInfinity;
    sum = Up(sum + Up(Abs(c) * Abs(c)));
  }
  return Up(std::sqrt(sum));
}
Vector3 Cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
ParseError Unsupported(const char* m) {
  return {ErrorCode::kUnsupportedFormat, 0, m};
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Cylinder/NURBS edge certificate exhausted its work limit"};
}
bool Spend(uint64_t& n, uint64_t cost = 1) {
  if (n < cost) return false;
  n -= cost;
  return true;
}
Result<const GeometryObject*> Object(const Model& m, NodeId id) {
  const auto* n = m.FindNode(id);
  if (!n || !n->geometry)
    return Result<const GeometryObject*>::Err(
        Unsupported("Tolerant edge dependency is not typed"));
  return Result<const GeometryObject*>::Ok(&m.geometry()[*n->geometry]);
}
V PlaneNormal(const PlaneGeometry& plane) {
  const auto x = Vec(plane.x_axis), y = Vec(Cross(plane.normal, plane.x_axis));
  V n{};
  for (unsigned k = 0; k < 3; ++k)
    n[k] = Sub(Mul(x[(k + 1) % 3], y[(k + 2) % 3]),
               Mul(x[(k + 2) % 3], y[(k + 1) % 3]));
  return n;
}
struct Linear {
  NurbsCurveGeometry curve;
  std::array<double, 2> first, last;
};
Result<Linear> ReadLinear(const Model& model, const BoundFinGeometry& fin) {
  using R = Result<Linear>;
  NodeId id = fin.curve;
  std::set<uint32_t> seen;
  const GeometryObject* object = nullptr;
  for (unsigned depth = 0; depth < 64; ++depth) {
    if (!seen.insert(id.value).second)
      return R::Err(
          {ErrorCode::kInvalidRecord, 0, "Cyclic tolerant pcurve trim"});
    auto g = Object(model, id);
    if (!g.ok()) return R::Err(g.error());
    object = g.value();
    if (object->sense != '+')
      return R::Err(
          Unsupported("Cylinder/NURBS pcurves require positive source senses"));
    auto* t = std::get_if<TrimmedCurveGeometry>(&object->payload);
    if (!t) break;
    id = t->basis_curve;
    object = nullptr;
  }
  if (!object)
    return R::Err({ErrorCode::kResourceLimit, 0,
                   "Tolerant pcurve trim depth exceeds 64"});
  const auto* sp = std::get_if<SurfaceCurveGeometry>(&object->payload);
  if (!sp || sp->surface != fin.surface)
    return R::Err(
        Unsupported("Tolerant pcurve must reference its own carrier"));
  auto basis = Object(model, sp->parameter_curve);
  if (!basis.ok()) return R::Err(basis.error());
  const auto* c = std::get_if<NurbsCurveGeometry>(&basis.value()->payload);
  if (!c || basis.value()->sense != '+' || c->controls.rational ||
      c->controls.dimension != 2 || c->axis.degree != 1 ||
      c->axis.control_count != 2 || c->axis.periodic || c->axis.closed ||
      c->axis.knots.size() != 2 ||
      c->axis.multiplicities != std::vector<uint16_t>{2, 2} ||
      c->controls.values.size() != 4)
    return R::Err(Unsupported(
        "Cylinder/NURBS recovery requires affine clamped 2D pcurves"));
  if (std::min(fin.start, fin.end) != c->axis.knots.front() ||
      std::max(fin.start, fin.end) != c->axis.knots.back())
    return R::Err(
        Unsupported("Tolerant affine pcurve must use its complete domain"));
  const auto a = EvaluateCurve(model, fin.curve, fin.start),
             b = EvaluateCurve(model, fin.curve, fin.end);
  if (!a.ok()) return R::Err(a.error());
  if (!b.ok()) return R::Err(b.error());
  Linear out{*c,
             {c->controls.values[0], c->controls.values[1]},
             {c->controls.values[2], c->controls.values[3]}};
  if (fin.end < fin.start) std::swap(out.first, out.last);
  if (fin.use.sense == TopologySense::kReverse) std::swap(out.first, out.last);
  return R::Ok(std::move(out));
}
// Homogeneous interval patches use V-fast order and coordinates relative to
// the candidate circle center. Every tensor restriction is de Casteljau;
// these controls are proof enclosures and never replace source geometry.
struct Net {
  unsigned p = 0, q = 0;
  std::array<H, 16> h{};
};
H Mix(const H& a, const H& b, I t) {
  H r;
  for (unsigned k = 0; k < 4; ++k)
    r[k] = Add(Mul(Sub(E(1), t), a[k]), Mul(t, b[k]));
  return r;
}
std::array<H, 4> Section(std::array<H, 4> row, unsigned degree, I a, I b) {
  std::array<H, 4> left{}, right{};
  if (b.lo == 1 && b.hi == 1)
    left = row;
  else {
    for (unsigned l = 0; l <= degree; ++l) {
      left[l] = row[0];
      for (unsigned j = 0; j + l < degree; ++j)
        row[j] = Mix(row[j], row[j + 1], b);
    }
  }
  if (a.lo == 0 && a.hi == 0) return left;
  row = left;
  const auto ratio = Div(a, b);
  for (unsigned l = 0; l <= degree; ++l) {
    right[degree - l] = row[degree - l];
    for (unsigned j = 0; j + l < degree; ++j)
      row[j] = Mix(row[j], row[j + 1], ratio);
  }
  return right;
}
Net Restrict(Net n, I u0, I u1, I v0, I v1) {
  for (unsigned j = 0; j <= n.q; ++j) {
    std::array<H, 4> row{};
    for (unsigned i = 0; i <= n.p; ++i) row[i] = n.h[i * (n.q + 1) + j];
    row = Section(row, n.p, u0, u1);
    for (unsigned i = 0; i <= n.p; ++i) n.h[i * (n.q + 1) + j] = row[i];
  }
  for (unsigned i = 0; i <= n.p; ++i) {
    std::array<H, 4> row{};
    for (unsigned j = 0; j <= n.q; ++j) row[j] = n.h[i * (n.q + 1) + j];
    row = Section(row, n.q, v0, v1);
    for (unsigned j = 0; j <= n.q; ++j) n.h[i * (n.q + 1) + j] = row[j];
  }
  return n;
}
struct Rect {
  long double u0, u1, v0, v1;
};
struct Bound {
  V box{}, du{}, dv{};
  I phi{}, fu{}, fv{};
  long double max_weight = 0;
};
void Union(I& a, I b) { a = {std::min(a.lo, b.lo), std::max(a.hi, b.hi)}; }
I PointParameter(long double x, double a, double b) {
  if (x == a) return E(0);
  if (x == b) return E(1);
  return Div(Sub(E(x), E(a)), Sub(E(b), E(a)));
}
struct SurfaceBounds {
  const NurbsSurfaceGeometry& surface;
  Vector3 origin;
  const PlaneGeometry* plane;
  long double weight_scale;
  uint64_t& work;
  Result<Bound> Get(Rect r) const {
    using R = Result<Bound>;
    const auto& u = surface.u;
    const auto& v = surface.v;
    if (r.u0 < u.knots.front() || r.u1 > u.knots.back() ||
        r.v0 < v.knots.front() || r.v1 > v.knots.back() || r.u0 > r.u1 ||
        r.v0 > r.v1)
      return R::Err(
          Unsupported("Tolerant edge proof leaves the original NURBS domain"));
    Bound out;
    bool first = true;
    const auto np = plane ? PlaneNormal(*plane) : V{},
               offset = plane ? Diff(origin, plane->origin) : V{};
    const auto begin = [](const auto& knots, long double x) {
      auto it = std::lower_bound(knots.begin(), knots.end(), x);
      return it == knots.begin()
                 ? std::size_t{0}
                 : static_cast<std::size_t>(it - knots.begin() - 1);
    };
    for (std::size_t a = begin(u.knots, r.u0);
         a + 1 < u.knots.size() && u.knots[a] <= r.u1; ++a) {
      if (u.knots[a + 1] < r.u0 || u.knots[a] > r.u1) continue;
      for (std::size_t b = begin(v.knots, r.v0);
           b + 1 < v.knots.size() && v.knots[b] <= r.v1; ++b) {
        if (v.knots[b + 1] < r.v0 || v.knots[b] > r.v1) continue;
        if (!Spend(work, 128 * (u.degree + 1) * (v.degree + 1)))
          return R::Err(Limit());
        Net n;
        n.p = u.degree;
        n.q = v.degree;
        for (unsigned i = 0; i <= n.p; ++i)
          for (unsigned j = 0; j <= n.q; ++j) {
            const auto at =
                4 * ((a * u.degree + i) * v.control_count + b * v.degree + j);
            auto& h = n.h[i * (n.q + 1) + j];
            h[3] = Div(E(surface.controls.values[at + 3]), E(weight_scale));
            const std::array<double, 3> o{origin.x, origin.y, origin.z};
            for (unsigned k = 0; k < 3; ++k)
              h[k] =
                  Sub(Div(E(surface.controls.values[at + k]), E(weight_scale)),
                      Mul(h[3], E(o[k])));
          }
        Net du, dv;
        du.p = n.p - 1;
        du.q = n.q;
        dv.p = n.p;
        dv.q = n.q - 1;
        const auto us = Sub(E(u.knots[a + 1]), E(u.knots[a])),
                   vs = Sub(E(v.knots[b + 1]), E(v.knots[b]));
        for (unsigned i = 0; i <= du.p; ++i)
          for (unsigned j = 0; j <= du.q; ++j)
            for (unsigned k = 0; k < 4; ++k)
              du.h[i * (du.q + 1) + j][k] =
                  Div(Mul(E(n.p), Sub(n.h[(i + 1) * (n.q + 1) + j][k],
                                      n.h[i * (n.q + 1) + j][k])),
                      us);
        for (unsigned i = 0; i <= dv.p; ++i)
          for (unsigned j = 0; j <= dv.q; ++j)
            for (unsigned k = 0; k < 4; ++k)
              dv.h[i * (dv.q + 1) + j][k] =
                  Div(Mul(E(n.q), Sub(n.h[i * (n.q + 1) + j + 1][k],
                                      n.h[i * (n.q + 1) + j][k])),
                      vs);
        const auto u0 =
            PointParameter(std::max(r.u0, static_cast<long double>(u.knots[a])),
                           u.knots[a], u.knots[a + 1]);
        const auto u1 = PointParameter(
            std::min(r.u1, static_cast<long double>(u.knots[a + 1])),
            u.knots[a], u.knots[a + 1]);
        const auto v0 =
            PointParameter(std::max(r.v0, static_cast<long double>(v.knots[b])),
                           v.knots[b], v.knots[b + 1]);
        const auto v1 = PointParameter(
            std::min(r.v1, static_cast<long double>(v.knots[b + 1])),
            v.knots[b], v.knots[b + 1]);
        n = Restrict(n, u0, u1, v0, v1);
        du = Restrict(du, u0, u1, v0, v1);
        dv = Restrict(dv, u0, u1, v0, v1);
        const auto bounds = [](const Net& data) {
          H range;
          for (auto& c : range) c = {kInfinity, -kInfinity};
          for (unsigned i = 0; i < (data.p + 1) * (data.q + 1); ++i)
            for (unsigned k = 0; k < 4; ++k) Union(range[k], data.h[i][k]);
          return range;
        };
        const auto h = bounds(n), hu = bounds(du), hv = bounds(dv);
        Bound one;
        for (auto& c : one.box) c = {kInfinity, -kInfinity};
        one.phi = {kInfinity, -kInfinity};
        for (unsigned i = 0; i < (n.p + 1) * (n.q + 1); ++i) {
          V point{}, f{};
          for (unsigned k = 0; k < 3; ++k) {
            point[k] = Div(n.h[i][k], n.h[i][3]);
            f[k] = Add(n.h[i][k], Mul(offset[k], n.h[i][3]));
            Union(one.box[k], point[k]);
          }
          Union(one.phi, Dot(f, np));
        }
        V fu{}, fv{};
        for (unsigned k = 0; k < 3; ++k) {
          one.du[k] = Div(Sub(hu[k], Mul(one.box[k], hu[3])), h[3]);
          one.dv[k] = Div(Sub(hv[k], Mul(one.box[k], hv[3])), h[3]);
          fu[k] = Add(hu[k], Mul(offset[k], hu[3]));
          fv[k] = Add(hv[k], Mul(offset[k], hv[3]));
        }
        one.fu = Dot(fu, np);
        one.fv = Dot(fv, np);
        one.max_weight = h[3].hi;
        if (!Valid(h[3]) || h[3].lo <= 0 || !Valid(one.phi) || !Valid(one.fu) ||
            !Valid(one.fv) || !std::isfinite(Length(one.du)) ||
            !std::isfinite(Length(one.dv)))
          return R::Err(
              Unsupported("Tolerant NURBS bounds are unrepresentable"));
        if (first) {
          out = one;
          first = false;
        } else {
          for (unsigned k = 0; k < 3; ++k) {
            Union(out.box[k], one.box[k]);
            Union(out.du[k], one.du[k]);
            Union(out.dv[k], one.dv[k]);
          }
          Union(out.phi, one.phi);
          Union(out.fu, one.fu);
          Union(out.fv, one.fv);
          out.max_weight = std::max(out.max_weight, one.max_weight);
        }
      }
    }
    if (first)
      return R::Err(Unsupported("Tolerant NURBS range contains no knot cell"));
    return R::Ok(out);
  }
};
struct TraceProof {
  long double error = 0;
  Rect graph{};
  bool initialized = false;
};
I LinearAt(double a, double b, long double t) {
  if (t == 0) return E(a);
  if (t == 1) return E(b);
  return Add(Mul(E(a), E(1 - t)), Mul(E(b), E(t)));
}
Rect Range(const Linear& line, long double a, long double b) {
  auto u0 = LinearAt(line.first[0], line.last[0], a),
       u1 = LinearAt(line.first[0], line.last[0], b),
       v0 = LinearAt(line.first[1], line.last[1], a),
       v1 = LinearAt(line.first[1], line.last[1], b);
  return {std::min(u0.lo, u1.lo), std::max(u0.hi, u1.hi),
          std::min(v0.lo, v1.lo), std::max(v0.hi, v1.hi)};
}
Result<bool> CheckTrace(const SurfaceBounds& surface, const Linear& line,
                        const CircleGeometry& circle, double first, double last,
                        long double allowance, long double a, long double b,
                        unsigned depth, TraceProof& proof) {
  using R = Result<bool>;
  if (!Spend(surface.work)) return R::Err(Limit());
  auto r = Range(line, a, b);
  r.u0 =
      std::max(r.u0, static_cast<long double>(surface.surface.u.knots.front()));
  r.u1 =
      std::min(r.u1, static_cast<long double>(surface.surface.u.knots.back()));
  r.v0 =
      std::max(r.v0, static_cast<long double>(surface.surface.v.knots.front()));
  r.v1 =
      std::min(r.v1, static_cast<long double>(surface.surface.v.knots.back()));
  auto bound = surface.Get(r);
  if (!bound.ok()) return R::Err(bound.error());
  const auto mid = (a + b) / 2;
  const auto angle_bound = Add(E(first), Mul(Sub(E(last), E(first)), E(mid)));
  const auto angle = std::midpoint(angle_bound.lo, angle_bound.hi);
  const auto angle_error = Abs(Sub(angle_bound, E(angle)));
  const auto y = Cross(circle.normal, circle.x_axis);
  const auto c = std::cos(angle), s = std::sin(angle);
  const Vector3 target{
      static_cast<double>(circle.radius * (circle.x_axis.x * c + y.x * s)),
      static_cast<double>(circle.radius * (circle.x_axis.y * c + y.y * s)),
      static_cast<double>(circle.radius * (circle.x_axis.z * c + y.z * s))};
  V difference{};
  const auto t = Vec(target);
  for (unsigned k = 0; k < 3; ++k)
    difference[k] = Sub(bound.value().box[k], t[k]);
  long double error =
      Length(difference) +
      circle.radius *
          (std::abs(static_cast<long double>(last) - first) * (b - a) / 2 +
           angle_error) *
          (Length(Vec(circle.x_axis)) + Length(Vec(y)));
  Rect graph = r;
  if (surface.plane) {
    auto data = bound.value();
    long double delta = 0;
    bool isolated = false;
    for (unsigned iteration = 0; iteration < 8; ++iteration) {
      const auto m = data.fu.lo > 0   ? data.fu.lo
                     : data.fu.hi < 0 ? -data.fu.hi
                                      : 0;
      if (!(m > 0)) break;
      const auto needed = Up(Abs(bound.value().phi) / m);
      if (delta >= needed) {
        error = Up(error + needed * Length(data.du));
        isolated = true;
        break;
      }
      delta = Up(std::max(needed * 1.25L, delta * 2));
      graph = r;
      graph.u0 = Down(r.u0 - delta);
      graph.u1 = Up(r.u1 + delta);
      const auto enlarged = surface.Get(graph);
      if (!enlarged.ok()) return R::Err(enlarged.error());
      data = enlarged.value();
    }
    if (!isolated)
      return R::Err(
          Unsupported("Tolerant intersection has no isolated plane graph"));
  }
  if (std::isfinite(error) && error <= allowance) {
    proof.error = std::max(proof.error, error);
    if (!proof.initialized) {
      proof.graph = graph;
      proof.initialized = true;
    } else {
      proof.graph.u0 = std::min(proof.graph.u0, graph.u0);
      proof.graph.u1 = std::max(proof.graph.u1, graph.u1);
      proof.graph.v0 = std::min(proof.graph.v0, graph.v0);
      proof.graph.v1 = std::max(proof.graph.v1, graph.v1);
    }
    return R::Ok(true);
  }
  if (depth == 16)
    return R::Err(Unsupported(
        "Whole tolerant pcurve/master trace exceeds the edge tolerance"));
  const auto left = CheckTrace(surface, line, circle, first, last, allowance, a,
                               mid, depth + 1, proof);
  if (!left.ok()) return left;
  return CheckTrace(surface, line, circle, first, last, allowance, mid, b,
                    depth + 1, proof);
}
V GraphVelocity(const Bound& p) {
  V result{};
  const auto ratio = SignedDiv(Mul(p.fv, E(-1)), p.fu);
  for (unsigned k = 0; k < 3; ++k)
    result[k] = Add(p.dv[k], Mul(p.du[k], ratio));
  return result;
}
Result<bool> IsolateActiveChartChord(const SurfaceBounds& surface,
                                     const IntersectionChartEvaluation& chart,
                                     unsigned side, std::size_t index,
                                     const IntersectionCurveGeometry& source,
                                     Rect selected) {
  using R = Result<bool>;
  const auto a = chart.surface_parameters[index][side],
             b = chart.surface_parameters[index + 1][side];
  Rect range{std::min({selected.u0, static_cast<long double>(a.u),
                       static_cast<long double>(b.u)}),
             std::max({selected.u1, static_cast<long double>(a.u),
                       static_cast<long double>(b.u)}),
             std::min({selected.v0, static_cast<long double>(a.v),
                       static_cast<long double>(b.v)}),
             std::max({selected.v1, static_cast<long double>(a.v),
                       static_cast<long double>(b.v)})};
  auto padding =
      std::max((range.u1 - range.u0) / 8,
               256 * std::numeric_limits<double>::epsilon() *
                   std::max({1.L, std::abs(range.u0), std::abs(range.u1)}));
  const auto chord =
      Diff(source.chart.positions[index + 1], source.chart.positions[index]);
  const int direction = b.v > a.v ? 1 : b.v < a.v ? -1 : 0;
  if (!direction)
    return R::Err(Unsupported(
        "Active master chart chord has an unresolved graph direction"));
  for (unsigned attempt = 0; attempt < 8; ++attempt) {
    auto expanded = range;
    expanded.u0 =
        std::max(static_cast<long double>(surface.surface.u.knots.front()),
                 Down(range.u0 - padding));
    expanded.u1 =
        std::min(static_cast<long double>(surface.surface.u.knots.back()),
                 Up(range.u1 + padding));
    const auto all = surface.Get(expanded);
    if (!all.ok()) return R::Err(all.error());
    const auto f = all.value().fu;
    if (!(f.lo > 0 || f.hi < 0))
      return R::Err(Unsupported(
          "Active master chart does not isolate one NURBS plane graph"));
    auto left = expanded, right = expanded;
    left.u1 = left.u0;
    right.u0 = right.u1;
    const auto l = surface.Get(left), r = surface.Get(right);
    if (!l.ok()) return R::Err(l.error());
    if (!r.ok()) return R::Err(r.error());
    if ((f.lo > 0 && l.value().phi.hi < 0 && r.value().phi.lo > 0) ||
        (f.hi < 0 && l.value().phi.lo > 0 && r.value().phi.hi < 0)) {
      const auto projection = Dot(GraphVelocity(all.value()), chord);
      if ((direction > 0 && projection.lo > 0) ||
          (direction < 0 && projection.hi < 0))
        return R::Ok(true);
      return R::Err(Unsupported(
          "Active chart projection is not one-to-one on the isolated graph"));
    }
    padding *= 2;
  }
  return R::Err(
      Unsupported("Active chart graph cannot be bracketed within the source "
                  "surface domain"));
}
Result<long double> CheckMaster(
    const Model& model, const TrimmedCurveGeometry& trim,
    const GeometryObject& master, const SurfaceBounds& surface,
    const Linear& line, const CircleGeometry& circle, double first, double last,
    TraceProof& proof, const BoundFinGeometry& fin, double tolerance) {
  using R = Result<long double>;
  if (const auto* c = std::get_if<CircleGeometry>(&master.payload)) {
    if (c->center.x != circle.center.x || c->center.y != circle.center.y ||
        c->center.z != circle.center.z || c->normal.x != circle.normal.x ||
        c->normal.y != circle.normal.y || c->normal.z != circle.normal.z ||
        c->x_axis.x != circle.x_axis.x || c->x_axis.y != circle.x_axis.y ||
        c->x_axis.z != circle.x_axis.z || c->radius != circle.radius ||
        !((trim.parameter_1 == first && trim.parameter_2 == last) ||
          (trim.parameter_1 == last && trim.parameter_2 == first)))
      return R::Err(
          Unsupported("Existing circle master must retain the exact cylinder "
                      "frame and minor interval"));
    return R::Ok(0);
  }
  const auto* intersection =
      std::get_if<IntersectionCurveGeometry>(&master.payload);
  if (!intersection || !surface.plane || !fin.intersection)
    return R::Err(
        Unsupported("Tolerant master requires a certified circle or bounded "
                    "plane/NURBS intersection with explicit solver settings"));
  if (intersection->start.type != 'L' || intersection->end.type != 'L' ||
      intersection->chart.positions.size() < 2)
    return R::Err(Unsupported(
        "Tolerant intersection master needs two ordinary chart limits"));
  const auto chart = EvaluateIntersectionChart(
      model, master.source, fin.intersection->chart_tolerance,
      fin.intersection->solve_tolerance);
  if (!chart.ok()) return R::Err(chart.error());
  const auto& times = chart.value().parameterization.parameters;
  const auto low = std::min(trim.parameter_1, trim.parameter_2),
             high = std::max(trim.parameter_1, trim.parameter_2);
  if (low < times.front() || high > times.back() || low == high)
    return R::Err(
        Unsupported("Existing master trim is outside its reconstructed chart"));
  const auto after = std::upper_bound(times.begin(), times.end(), low);
  const auto chord_index =
      after == times.end()
          ? times.size() - 2
          : static_cast<std::size_t>(after - times.begin() - 1);
  if (high > times[chord_index + 1])
    return R::Err(
        Unsupported("Tolerant master trim must lie in one chart chord"));
  const auto continuation = IsolateActiveChartChord(
      surface, chart.value(), intersection->surfaces[0] == fin.surface ? 0 : 1,
      chord_index, *intersection, proof.graph);
  if (!continuation.ok()) return R::Err(continuation.error());
  std::array<SurfaceParameters, 2> selected{};
  std::array<Vector3, 2> selected_points{};
  long double residual = 0;
  const auto c0 = EvaluateCurve(GeometryPayload{circle}, first),
             c1 = EvaluateCurve(GeometryPayload{circle}, last);
  if (!c0.ok()) return R::Err(c0.error());
  if (!c1.ok()) return R::Err(c1.error());
  const bool reversed = Length(Diff(trim.point_1, c1.value().position)) <
                        Length(Diff(trim.point_1, c0.value().position));
  const std::array<std::array<double, 2>, 2> seeds{line.first, line.last};
  for (unsigned i = 0; i < 2; ++i) {
    const auto point = EvaluateIntersectionCurve(
        model, master.source, i ? trim.parameter_2 : trim.parameter_1,
        *fin.intersection);
    if (!point.ok()) return R::Err(point.error());
    selected_points[i] = point.value().position;
    const auto seed = seeds[reversed ? 1 - i : i];
    const auto uv = InvertSurface(model, fin.surface, selected_points[i],
                                  std::min(fin.intersection->chart_tolerance,
                                           fin.intersection->solve_tolerance),
                                  SurfaceParameterSeed{seed[0], seed[1]});
    if (!uv.ok()) return R::Err(uv.error());
    selected[i] = uv.value();
    residual =
        std::max(residual, static_cast<long double>(uv.value().residual));
  }
  auto bounds = surface.Get(proof.graph);
  if (!bounds.ok()) return R::Err(bounds.error());
  auto data = bounds.value();
  const auto direction = Diff(trim.point_2, trim.point_1);
  const auto len = Length(direction);
  if (!std::isfinite(len) || len <= 2 * tolerance)
    return R::Err(Unsupported("Tolerant master endpoints are unresolved"));
  V unit{};
  for (unsigned k = 0; k < 3; ++k) unit[k] = Div(direction[k], E(len));
  auto velocity = GraphVelocity(data);
  auto projected = Dot(velocity, unit);
  auto minimum = projected.lo > 0   ? projected.lo
                 : projected.hi < 0 ? -projected.hi
                                    : 0;
  if (!(minimum > 0))
    return R::Err(Unsupported(
        "Tolerant intersection graph is not globally ordered along its chord"));
  const long double numerical =
      256 * std::numeric_limits<double>::epsilon() *
      std::max({1.L, std::abs(proof.graph.u0), std::abs(proof.graph.u1),
                std::abs(proof.graph.v0), std::abs(proof.graph.v1)});
  long double plane_error = 0;
  for (const auto& point : selected_points)
    plane_error =
        std::max(plane_error, Abs(Dot(Diff(point, surface.plane->origin),
                                      PlaneNormal(*surface.plane))));
  const auto m = data.fu.lo > 0 ? data.fu.lo : data.fu.hi < 0 ? -data.fu.hi : 0;
  if (!(m > 0))
    return R::Err(
        Unsupported("Tolerant plane graph has no signed derivative bound"));
  const auto normal_length = Length(PlaneNormal(*surface.plane));
  const auto mapped_error =
      residual + (plane_error + normal_length * residual) * Length(data.du) *
                     data.max_weight / m;
  const auto dv = Up(4 * mapped_error / minimum + numerical);
  const auto du =
      Up(4 * (plane_error + normal_length * residual) * data.max_weight / m +
         numerical);
  auto expanded = proof.graph;
  expanded.u0 = Down(expanded.u0 - du);
  expanded.u1 = Up(expanded.u1 + du);
  expanded.v0 = Down(expanded.v0 - dv);
  expanded.v1 = Up(expanded.v1 + dv);
  bounds = surface.Get(expanded);
  if (!bounds.ok()) return R::Err(bounds.error());
  data = bounds.value();
  velocity = GraphVelocity(data);
  projected = Dot(velocity, unit);
  const auto final_m = data.fu.lo > 0   ? data.fu.lo
                       : data.fu.hi < 0 ? -data.fu.hi
                                        : 0;
  const auto final_min = projected.lo > 0   ? projected.lo
                         : projected.hi < 0 ? -projected.hi
                                            : 0;
  if (!(final_m > 0) || !(final_min > 0))
    return R::Err(
        Unsupported("Tolerant graph ordering is ambiguous at chart endpoints"));
  const auto final_error = residual + (plane_error + normal_length * residual) *
                                          Length(data.du) * data.max_weight /
                                          final_m;
  if (final_error / final_min > dv ||
      (plane_error + normal_length * residual) * data.max_weight / final_m > du)
    return R::Err(
        Unsupported("Tolerant chart endpoint neighbourhood is not certified"));
  int chart_direction = 0;
  long double previous = 0;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (!Spend(surface.work)) return R::Err(Limit());
    const auto uv = selected[i];
    if (uv.u < expanded.u0 || uv.u > expanded.u1 || uv.v < expanded.v0 ||
        uv.v > expanded.v1)
      return R::Err(Unsupported(
          "Existing intersection chart selects another NURBS branch"));
    if (i) {
      const int sign = uv.v > previous ? 1 : uv.v < previous ? -1 : 0;
      if (!sign || (chart_direction && sign != chart_direction))
        return R::Err(Unsupported(
            "Existing intersection chart reverses its graph coordinate"));
      chart_direction = sign;
      auto chord = Diff(intersection->chart.positions[chord_index + 1],
                        intersection->chart.positions[chord_index]);
      if (trim.parameter_2 < trim.parameter_1)
        for (auto& coordinate : chord) coordinate = Mul(coordinate, E(-1));
      const auto speed = Dot(velocity, chord);
      if ((sign > 0 && speed.lo <= 0) || (sign < 0 && speed.hi >= 0))
        return R::Err(
            Unsupported("Intersection chart chord does not isolate the ordered "
                        "plane graph"));
    }
    previous = uv.v;
  }
  const auto end_gap = std::max(Length(Diff(trim.point_1, selected_points[0])),
                                Length(Diff(trim.point_2, selected_points[1])));
  if (end_gap > tolerance)
    return R::Err(Unsupported(
        "Existing master trim points disagree with their parameters"));
  // An isolated graph with strictly ordered free V cannot leave the strip and
  // return between these chart limits. Inversion residuals reserve endpoint
  // neighbourhoods; they never enlarge the caller/source edge allowance.
  const auto correction = Up(Length(velocity) * dv + final_error + end_gap);
  if (!std::isfinite(correction))
    return R::Err(
        Unsupported("Tolerant chart endpoint bounds are unrepresentable"));
  return R::Ok(correction);
}
}  // namespace
Result<CircularEdgeRecovery> RecoverCylinderNurbsEdge(
    const Model& model, std::span<const BoundFinGeometry* const> fins,
    const std::map<uint32_t, Vector3>& vertices, double tolerance,
    bool use_source_tolerances, uint64_t& work) {
  using R = Result<CircularEdgeRecovery>;
  if (!std::isfinite(tolerance) || tolerance <= 0)
    return R::Err(
        {ErrorCode::kInvalidOptions, 0, "Invalid tolerant edge allowance"});
  if (fins.size() != 2 || !fins[0]->use.fin_curve.value ||
      !fins[1]->use.fin_curve.value || fins[0]->use.edge != fins[1]->use.edge ||
      fins[0]->use.edge_curve != fins[1]->use.edge_curve)
    return R::Err(Unsupported(
        "Cylinder/NURBS recovery needs two consistent fin-specific curves"));
  for (const auto* fin : fins)
    if (fin->curve != fin->use.fin_curve ||
        (fin->use.sense != TopologySense::kForward &&
         fin->use.sense != TopologySense::kReverse))
      return R::Err(
          Unsupported("Tolerant mapping needs the recorded fin curve and a "
                      "known direction"));
  if (fins[0]->use.sense == fins[1]->use.sense ||
      fins[0]->use.start_vertex != fins[1]->use.end_vertex ||
      fins[0]->use.end_vertex != fins[1]->use.start_vertex)
    return R::Err(Unsupported(
        "Tolerant paired fins require opposite endpoint orientations"));
  const BoundFinGeometry* cyl_fin = nullptr;
  const BoundFinGeometry* nurbs_fin = nullptr;
  const CylinderGeometry* cylinder = nullptr;
  const NurbsSurfaceGeometry* nurbs = nullptr;
  for (auto* f : fins) {
    auto s = Object(model, f->surface);
    if (!s.ok()) return R::Err(s.error());
    if (const auto* c = std::get_if<CylinderGeometry>(&s.value()->payload)) {
      cylinder = c;
      cyl_fin = f;
    }
    if (const auto* n =
            std::get_if<NurbsSurfaceGeometry>(&s.value()->payload)) {
      nurbs = n;
      nurbs_fin = f;
    }
    if (!f->use.start_vertex.value || !f->use.end_vertex.value ||
        f->use.start_vertex == f->use.end_vertex)
      return R::Err(Unsupported(
          "Cylinder/NURBS recovery supports vertex-bounded open arcs"));
  }
  if (!cylinder || !nurbs)
    return R::Err(
        Unsupported("Tolerant recovery requires a CYLINDER/B_SURF pair"));
  auto cp = ReadLinear(model, *cyl_fin), np = ReadLinear(model, *nurbs_fin);
  if (!cp.ok()) return R::Err(cp.error());
  if (!np.ok()) return R::Err(np.error());
  const auto* edge = model.FindNode(cyl_fin->use.edge);
  if (!edge || edge->node_type != 16)
    return R::Err(
        {ErrorCode::kInvalidRecord, 0, "Tolerant source edge is absent"});
  const auto reference =
      std::find_if(edge->references.begin(), edge->references.end(),
                   [](const auto& ref) { return ref.field == "curve"; });
  if (reference == edge->references.end() ||
      reference->target != cyl_fin->use.edge_curve)
    return R::Err({ErrorCode::kInvalidRecord, 0,
                   "Tolerant master reference disagrees with its edge"});
  if (use_source_tolerances && edge && edge->edge_tolerance) {
    if (!std::isfinite(*edge->edge_tolerance) || *edge->edge_tolerance <= 0)
      return R::Err(
          {ErrorCode::kInvalidRecord, 0, "Invalid source edge tolerance"});
    tolerance = std::max(tolerance, *edge->edge_tolerance);
  }
  const auto valid =
      EvaluateSurface(GeometryPayload{*nurbs},
                      nurbs->u.knots.empty() ? 0 : nurbs->u.knots.front(),
                      nurbs->v.knots.empty() ? 0 : nurbs->v.knots.front());
  if (!valid.ok()) return R::Err(valid.error());
  for (const auto* axis : {&nurbs->u, &nurbs->v}) {
    if (!axis->degree || axis->degree > 3 || axis->closed || axis->periodic ||
        axis->knots.size() < 2 ||
        axis->multiplicities.front() != axis->degree + 1 ||
        axis->multiplicities.back() != axis->degree + 1)
      return R::Err(
          Unsupported("Tolerant NURBS carriers need open clamped degree-one to "
                      "degree-three axes"));
    for (std::size_t i = 1; i + 1 < axis->knots.size(); ++i)
      if (axis->multiplicities[i] != axis->degree)
        return R::Err(
            Unsupported("Tolerant NURBS carriers need piecewise-Bezier knot "
                        "multiplicities"));
  }
  if (!nurbs->controls.rational || nurbs->controls.dimension != 4 ||
      np.value().first[1] == np.value().last[1])
    return R::Err(
        Unsupported("Tolerant NURBS recovery needs positive homogeneous "
                    "controls and a changing V coordinate"));
  for (unsigned k = 0; k < 2; ++k) {
    const auto& axis = k == 0 ? nurbs->u : nurbs->v;
    if (std::min(np.value().first[k], np.value().last[k]) <
            axis.knots.front() ||
        std::max(np.value().first[k], np.value().last[k]) > axis.knots.back())
      return R::Err(Unsupported(
          "Tolerant source pcurve leaves the original carrier domain"));
  }
  CircleGeometry circle;
  circle.normal = cylinder->axis;
  circle.x_axis = cylinder->x_axis;
  circle.radius = cylinder->radius;
  const auto height = cp.value().first[1];
  if (std::abs(cp.value().last[1] - height) >
      64 * std::numeric_limits<double>::epsilon() *
          std::max({1., std::abs(height), std::abs(cp.value().last[1])}))
    return R::Err(
        Unsupported("Cylinder pcurve must have constant source height up to "
                    "arithmetic rounding"));
  circle.center = {
      static_cast<double>(std::fma(static_cast<long double>(height),
                                   cylinder->axis.x, cylinder->origin.x)),
      static_cast<double>(std::fma(static_cast<long double>(height),
                                   cylinder->axis.y, cylinder->origin.y)),
      static_cast<double>(std::fma(static_cast<long double>(height),
                                   cylinder->axis.z, cylinder->origin.z))};
  const double first = cp.value().first[0], last = cp.value().last[0];
  const auto cv = EvaluateCurve(GeometryPayload{circle}, first);
  if (!cv.ok()) return R::Err(cv.error());
  if (!std::isfinite(last - first) ||
      std::abs(last - first) >= std::numbers::pi ||
      circle.radius * std::abs(last - first) <= 2 * tolerance)
    return R::Err(Unsupported(
        "Cylinder/NURBS recovery needs one resolved minor circle arc"));
  const auto cylinder_error =
      std::abs(cp.value().last[1] - height) * Length(Vec(cylinder->axis)) +
      Length(Diff(
          circle.center,
          {static_cast<double>(cylinder->origin.x + height * cylinder->axis.x),
           static_cast<double>(cylinder->origin.y + height * cylinder->axis.y),
           static_cast<double>(cylinder->origin.z +
                               height * cylinder->axis.z)}));
  const auto scale =
      std::max({1., std::abs(circle.center.x), std::abs(circle.center.y),
                std::abs(circle.center.z), circle.radius});
  const long double numerical =
      256 * std::numeric_limits<double>::epsilon() * scale;
  const auto frame_error = std::abs(Length(Vec(circle.normal)) - 1) +
                           std::abs(Length(Vec(circle.x_axis)) - 1) +
                           Abs(Dot(Vec(circle.normal), Vec(circle.x_axis)));
  const auto available = static_cast<long double>(tolerance) - cylinder_error -
                         numerical -
                         8 * (circle.radius + std::abs(height)) * frame_error;
  if (!(available > 0))
    return R::Err(
        Unsupported("Cylinder pcurve departure exhausts the edge tolerance"));
  const TrimmedCurveGeometry* trim = nullptr;
  const GeometryObject* master = nullptr;
  const PlaneGeometry* plane = nullptr;
  if (cyl_fin->use.edge_curve.value) {
    auto original = Object(model, cyl_fin->use.edge_curve);
    if (!original.ok()) return R::Err(original.error());
    if (original.value()->sense != '+' && original.value()->sense != '-')
      return R::Err(Unsupported("Existing master trim sense is unknown"));
    trim = std::get_if<TrimmedCurveGeometry>(&original.value()->payload);
    if (!trim)
      return R::Err(Unsupported(
          "Existing tolerant master must retain explicit trim endpoints"));
    auto basis = Object(model, trim->basis_curve);
    if (!basis.ok()) return R::Err(basis.error());
    master = basis.value();
    if (master->sense != '+' && master->sense != '-')
      return R::Err(Unsupported("Existing master basis sense is unknown"));
    if (const auto* intersection =
            std::get_if<IntersectionCurveGeometry>(&master->payload)) {
      NodeId other{};
      if (intersection->surfaces[0] == nurbs_fin->surface)
        other = intersection->surfaces[1];
      else if (intersection->surfaces[1] == nurbs_fin->surface)
        other = intersection->surfaces[0];
      auto support = Object(model, other);
      if (!support.ok()) return R::Err(support.error());
      plane = std::get_if<PlaneGeometry>(&support.value()->payload);
      if (!plane)
        return R::Err(
            Unsupported("Existing intersection master must use the same NURBS "
                        "carrier and a plane"));
    } else if (!std::holds_alternative<CircleGeometry>(master->payload))
      return R::Err(
          Unsupported("Existing tolerant master is outside the "
                      "circle/plane-intersection subset"));
  }
  long double master_endpoint_error = 0;
  if (trim) {
    const auto a =
        nurbs_fin->intersection
            ? EvaluateCurve(model, cyl_fin->use.edge_curve, trim->parameter_1,
                            *nurbs_fin->intersection)
            : EvaluateCurve(model, cyl_fin->use.edge_curve, trim->parameter_1);
    const auto b =
        nurbs_fin->intersection
            ? EvaluateCurve(model, cyl_fin->use.edge_curve, trim->parameter_2,
                            *nurbs_fin->intersection)
            : EvaluateCurve(model, cyl_fin->use.edge_curve, trim->parameter_2);
    if (!a.ok()) return R::Err(a.error());
    if (!b.ok()) return R::Err(b.error());
    const auto p = EvaluateCurve(GeometryPayload{circle}, first),
               q = EvaluateCurve(GeometryPayload{circle}, last);
    if (!p.ok()) return R::Err(p.error());
    if (!q.ok()) return R::Err(q.error());
    const auto matched = std::min(
        std::max(Length(Diff(a.value().position, p.value().position)),
                 Length(Diff(b.value().position, q.value().position))),
        std::max(Length(Diff(a.value().position, q.value().position)),
                 Length(Diff(b.value().position, p.value().position))));
    master_endpoint_error =
        matched + std::max(Length(Diff(a.value().position, trim->point_1)),
                           Length(Diff(b.value().position, trim->point_2)));
    if (master_endpoint_error > available)
      return R::Err(Unsupported(
          "Existing master trim endpoints select another circle arc"));
  }
  long double weights = 0;
  if (!Spend(work, nurbs->controls.values.size())) return R::Err(Limit());
  for (std::size_t i = 3; i < nurbs->controls.values.size(); i += 4)
    weights =
        std::max(weights, static_cast<long double>(nurbs->controls.values[i]));
  SurfaceBounds surface{*nurbs, circle.center, plane, weights, work};
  TraceProof proof;
  const auto checked = CheckTrace(surface, np.value(), circle, first, last,
                                  available * .85L, 0, 1, 0, proof);
  if (!checked.ok()) return R::Err(checked.error());
  long double extra = master_endpoint_error;
  if (master) {
    const auto matched =
        CheckMaster(model, *trim, *master, surface, np.value(), circle, first,
                    last, proof, *nurbs_fin, tolerance);
    if (!matched.ok()) return R::Err(matched.error());
    extra += matched.value();
  }
  if (proof.error + extra > available)
    return R::Err(Unsupported(
        "Stored master and pcurve bounds exceed the source edge allowance"));
  CircularEdgeRecovery result{
      cyl_fin->use.edge, cyl_fin->use.edge, circle, tolerance, {}};
  for (auto* f : fins) {
    const auto a = f->use.sense == TopologySense::kForward ? first : last,
               b = f->use.sense == TopologySense::kForward ? last : first;
    for (unsigned end = 0; end < 2; ++end) {
      const auto pos = EvaluateCurve(GeometryPayload{circle}, end ? b : a);
      if (!pos.ok()) return R::Err(pos.error());
      const auto id = end ? f->use.end_vertex : f->use.start_vertex;
      const auto vertex = vertices.find(id.value);
      if (vertex == vertices.end())
        return R::Err(
            {ErrorCode::kInvalidRecord, 0, "Tolerant vertex is missing"});
      if (Length(Diff(pos.value().position, vertex->second)) > tolerance)
        return R::Err(
            Unsupported("Tolerant circle does not match the source vertex"));
      const auto source =
          EvaluateCurve(model, f->curve, end ? f->end : f->start);
      if (!source.ok()) return R::Err(source.error());
      if (Length(Diff(pos.value().position, source.value().position)) >
          tolerance)
        return R::Err(
            Unsupported("Tolerant pcurve endpoints select different arcs"));
    }
    result.uses.push_back({f->use.fin, f->surface, a, b,
                           f == cyl_fin ? cp.value().curve : np.value().curve});
  }
  return R::Ok(std::move(result));
}
}  // namespace openswstep::internal
