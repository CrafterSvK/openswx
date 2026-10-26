// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "simple_boundary.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nurbs_wire.h"
#include "openparasolid/evaluate.h"

namespace openswstep::internal {
namespace {
using namespace openparasolid;
struct Point {
  long double x = 0, y = 0;
};
Point operator+(Point a, Point b) { return {a.x + b.x, a.y + b.y}; }
Point operator-(Point a, Point b) { return {a.x - b.x, a.y - b.y}; }
Point operator*(Point a, long double s) { return {a.x * s, a.y * s}; }
long double Dot(Point a, Point b) { return a.x * b.x + a.y * b.y; }
long double Cross(Point a, Point b) { return a.x * b.y - a.y * b.x; }
long double Length(Point p) { return std::hypot(p.x, p.y); }
ParseError Unsupported(const char* message) {
  return {ErrorCode::kUnsupportedFormat, 0, message};
}
struct Piece {
  std::array<Point, 4> p;
  std::array<long double, 4> w{1, 1, 1, 1};
  unsigned degree = 1, depth = 0;
  std::size_t parent = 0;
};
struct Certificate {
  std::vector<Piece> pieces;
  long double area2 = 0, perimeter = 0;
};
struct Frame {
  Vector3 origin, x, y, normal, plane_origin;
  Point Project(Vector3 p) const {
    const std::array<long double, 3> d{
        static_cast<long double>(p.x) - origin.x,
        static_cast<long double>(p.y) - origin.y,
        static_cast<long double>(p.z) - origin.z};
    return {d[0] * x.x + d[1] * x.y + d[2] * x.z,
            d[0] * y.x + d[1] * y.y + d[2] * y.z};
  }
  Point Direction(Vector3 p) const {
    return {static_cast<long double>(p.x) * x.x +
                static_cast<long double>(p.y) * x.y +
                static_cast<long double>(p.z) * x.z,
            static_cast<long double>(p.x) * y.x +
                static_cast<long double>(p.y) * y.y +
                static_cast<long double>(p.z) * y.z};
  }
  bool Coplanar(Vector3 p, double tolerance) const {
    const long double d =
        (static_cast<long double>(p.x) - plane_origin.x) * normal.x +
        (static_cast<long double>(p.y) - plane_origin.y) * normal.y +
        (static_cast<long double>(p.z) - plane_origin.z) * normal.z;
    return std::isfinite(d) && std::abs(d) <= tolerance;
  }
};
bool Spend(uint64_t& remaining) {
  if (!remaining) return false;
  --remaining;
  return true;
}
ParseError Limit() {
  return {ErrorCode::kResourceLimit, 0,
          "Simple planar control-hull certificate exhausted its budget"};
}
long double PolynomialArea(const Piece& s) {
  const auto& p = s.p;
  if (s.degree == 1) return Cross(p[0], p[1]);
  if (s.degree == 2)
    return (2 * Cross(p[0], p[1]) + Cross(p[0], p[2]) + 2 * Cross(p[1], p[2])) /
           3;
  return (6 * Cross(p[0], p[1]) + 3 * Cross(p[0], p[2]) + Cross(p[0], p[3]) +
          3 * Cross(p[1], p[2]) + 3 * Cross(p[1], p[3]) +
          6 * Cross(p[2], p[3])) /
         10;
}
Result<bool> AddPolynomial(const NurbsCurveGeometry& source, double start,
                           double end, const Frame& frame, double tolerance,
                           uint64_t& remaining, Certificate& result) {
  using Return = Result<bool>;
  if (source.controls.rational || source.controls.dimension != 3 ||
      (source.axis.degree != 2 && source.axis.degree != 3))
    return Return::Err(
        Unsupported("Simple planar splines require nonrational "
                    "quadratic/cubic 3D controls"));
  const auto restricted = RestrictNurbsWire(source, start, end, remaining);
  if (!restricted.ok()) return Return::Err(restricted.error());
  const auto& selected = restricted.value();
  const auto count = selected.axis.knots.size() - 1;
  for (std::size_t i = 0; i < count; ++i) {
    const auto k = end > start ? i : count - 1 - i;
    const auto span = RestrictNurbsWire(selected, selected.axis.knots[k],
                                        selected.axis.knots[k + 1], remaining);
    if (!span.ok()) return Return::Err(span.error());
    const auto& c = span.value().controls.values;
    Piece piece;
    piece.degree = selected.axis.degree;
    for (unsigned j = 0; j <= piece.degree; ++j) {
      if (!Spend(remaining)) return Return::Err(Limit());
      const auto index = 3 * (end > start ? j : piece.degree - j);
      const Vector3 p{c[index], c[index + 1], c[index + 2]};
      if (!frame.Coplanar(p, tolerance))
        return Return::Err(
            Unsupported("Simple spline hull is outside the plane"));
      piece.p[j] = frame.Project(p);
      if (j) result.perimeter += Length(piece.p[j] - piece.p[j - 1]);
    }
    result.area2 += PolynomialArea(piece);
    result.pieces.push_back(piece);
  }
  return Return::Ok(true);
}
Result<bool> AddCurve(const PlanarCurve& curve, const Frame& frame,
                      double tolerance, uint64_t& remaining,
                      Certificate& result) {
  using Return = Result<bool>;
  if (!curve.geometry || !std::isfinite(curve.start) ||
      !std::isfinite(curve.end) || curve.start == curve.end)
    return Return::Err(
        Unsupported("Simple planar curve needs a finite interval"));
  if (const auto* spline = std::get_if<NurbsCurveGeometry>(curve.geometry))
    return AddPolynomial(*spline, curve.start, curve.end, frame, tolerance,
                         remaining, result);
  const auto first = EvaluateCurve(*curve.geometry, curve.start);
  const auto last = EvaluateCurve(*curve.geometry, curve.end);
  if (!first.ok()) return Return::Err(first.error());
  if (!last.ok()) return Return::Err(last.error());
  if (std::holds_alternative<LineGeometry>(*curve.geometry)) {
    Piece piece;
    for (unsigned i = 0; i < 2; ++i) {
      const auto p = i ? last.value().position : first.value().position;
      if (!frame.Coplanar(p, tolerance))
        return Return::Err(Unsupported("Simple line is outside the plane"));
      piece.p[i] = frame.Project(p);
    }
    result.area2 += PolynomialArea(piece);
    result.perimeter += Length(piece.p[1] - piece.p[0]);
    result.pieces.push_back(piece);
    return Return::Ok(true);
  }
  const auto* circle = std::get_if<CircleGeometry>(curve.geometry);
  const auto* ellipse = std::get_if<EllipseGeometry>(curve.geometry);
  if (!circle && !ellipse)
    return Return::Err(Unsupported("Simple planar curve basis is unsupported"));
  const auto center = circle ? circle->center : ellipse->center;
  const auto x = circle ? circle->x_axis : ellipse->x_axis;
  const auto n = circle ? circle->normal : ellipse->normal;
  const Vector3 y{n.y * x.z - n.z * x.y, n.z * x.x - n.x * x.z,
                  n.x * x.y - n.y * x.x};
  const double a = circle ? circle->radius : ellipse->major_radius;
  const double b = circle ? circle->radius : ellipse->minor_radius;
  const double span = curve.end - curve.start;
  if (!std::isfinite(span) || std::abs(span) > 2 * std::numbers::pi)
    return Return::Err(Unsupported("Simple conic exceeds one turn"));
  const auto steps = static_cast<unsigned>(
      std::max(1., std::ceil(std::abs(span) / (std::numbers::pi / 2))));
  for (unsigned i = 0; i < steps; ++i) {
    if (!Spend(remaining)) return Return::Err(Limit());
    const double t0 = curve.start + span * i / steps;
    const double t1 = curve.start + span * (i + 1) / steps;
    const double middle = (t0 + t1) / 2;
    Piece piece;
    piece.degree = 2;
    piece.w[1] = std::cos((t1 - t0) / 2);
    for (unsigned j = 0; j < 3; ++j) {
      const double t = j == 0 ? t0 : j == 2 ? t1 : middle;
      const double weight = static_cast<double>(piece.w[j]);
      const Vector3 p{
          center.x + (a * x.x * std::cos(t) + b * y.x * std::sin(t)) / weight,
          center.y + (a * x.y * std::cos(t) + b * y.y * std::sin(t)) / weight,
          center.z + (a * x.z * std::cos(t) + b * y.z * std::sin(t)) / weight};
      if (!frame.Coplanar(p, tolerance))
        return Return::Err(
            Unsupported("Simple conic hull is outside the plane"));
      piece.p[j] = frame.Project(p);
    }
    result.pieces.push_back(piece);
  }
  const auto c = frame.Project(center);
  const auto px = frame.Direction(x), py = frame.Direction(y);
  result.area2 += Cross(c, frame.Project(last.value().position) -
                               frame.Project(first.value().position)) +
                  a * b * Cross(px, py) * span;
  result.perimeter += std::max(a, b) * std::abs(span);
  return Return::Ok(true);
}
bool Monotone(const Piece& p) {
  const auto chord = p.p[p.degree] - p.p[0];
  const auto length = Length(chord);
  if (!std::isfinite(length) || length == 0) return false;
  for (unsigned i = 1; i <= p.degree; ++i) {
    const auto step = p.p[i] - p.p[i - 1];
    const auto value = Dot(step, chord);
    if (value < 0 || ((i == 1 || i == p.degree) && value == 0)) return false;
  }
  return true;
}
bool Join(const Piece& a, const Piece& b, double tolerance) {
  const auto end = a.p[a.degree], start = b.p[0];
  const auto da = end - a.p[0], db = b.p[b.degree] - start;
  const auto la = Length(da), lb = Length(db);
  if (la == 0 || lb == 0 || Length(start - end) > tolerance) return false;
  auto n = da * (1 / la) + db * (1 / lb);
  const auto length = Length(n);
  if (length == 0) return false;
  n = n * (1 / length);
  long double ratio_a = 0, ratio_b = 0;
  for (unsigned i = 0; i < a.degree; ++i) {
    const auto d = end - a.p[i];
    const auto projection = Dot(d, n);
    if (projection <= 0) return false;
    ratio_a = std::max(ratio_a, Length(d) / projection);
  }
  for (unsigned i = 1; i <= b.degree; ++i) {
    const auto d = b.p[i] - start;
    const auto projection = Dot(d, n);
    if (projection <= 0) return false;
    ratio_b = std::max(ratio_b, Length(d) / projection);
  }
  // With exact shared endpoints this is strict half-plane separation. Rounded
  // endpoints can introduce a narrow overlap strip; convex combinations bound
  // that entire hull intersection inside both endpoint tolerance balls.
  const auto overlap = std::max(0.L, -Dot(start - end, n));
  return overlap * ratio_a <= tolerance && overlap * ratio_b <= tolerance;
}
bool Separate(const Piece& a, const Piece& b, double tolerance) {
  for (const auto* hull : {&a, &b}) {
    for (unsigned i = 0; i <= hull->degree; ++i) {
      for (unsigned j = i + 1; j <= hull->degree; ++j) {
        const auto edge = hull->p[j] - hull->p[i];
        const auto length = Length(edge);
        if (length == 0) continue;
        // Tangential axes also separate collinear disjoint line segments,
        // whose zero-area hulls have no transverse separating edge normal.
        for (Point normal : {Point{-edge.y / length, edge.x / length},
                             Point{edge.x / length, edge.y / length}}) {
          long double low_a = std::numeric_limits<long double>::infinity();
          long double low_b = low_a, high_a = -low_a, high_b = -low_a;
          for (unsigned k = 0; k <= a.degree; ++k) {
            const auto value = Dot(a.p[k] - a.p[0], normal);
            low_a = std::min(low_a, value);
            high_a = std::max(high_a, value);
          }
          for (unsigned k = 0; k <= b.degree; ++k) {
            const auto value = Dot(b.p[k] - a.p[0], normal);
            low_b = std::min(low_b, value);
            high_b = std::max(high_b, value);
          }
          if (low_b - high_a > tolerance || low_a - high_b > tolerance)
            return true;
        }
      }
    }
  }
  return false;
}
bool Split(std::vector<Piece>& pieces, std::size_t i, uint64_t& remaining) {
  constexpr unsigned kMaxDepth = 24;
  const auto p = pieces[i];
  if (p.degree == 1 || p.depth == kMaxDepth || !Spend(remaining)) return false;
  struct Homogeneous {
    Point p;
    long double w;
  };
  std::array<Homogeneous, 4> row;
  for (unsigned j = 0; j <= p.degree; ++j) row[j] = {p.p[j] * p.w[j], p.w[j]};
  Piece left, right;
  left.degree = right.degree = p.degree;
  left.parent = right.parent = p.parent;
  left.depth = right.depth = p.depth + 1;
  for (unsigned level = 0; level <= p.degree; ++level) {
    const auto a = row[0], b = row[p.degree - level];
    left.p[level] = a.p * (1 / a.w);
    left.w[level] = a.w;
    right.p[p.degree - level] = b.p * (1 / b.w);
    right.w[p.degree - level] = b.w;
    for (unsigned j = 0; j < p.degree - level; ++j)
      row[j] = {(row[j].p + row[j + 1].p) * .5L,
                (row[j].w + row[j + 1].w) * .5L};
  }
  pieces[i] = left;
  pieces.insert(pieces.begin() + static_cast<std::ptrdiff_t>(i + 1), right);
  return true;
}
Result<Certificate> CertifyLoop(std::span<const PlanarCurve> curves,
                                const PlaneGeometry& plane, bool natural_sense,
                                double tolerance, uint64_t& remaining_checks,
                                const Frame* shared_frame = nullptr) {
  using Return = Result<Certificate>;
  if (!std::isfinite(tolerance) || tolerance <= 0 || curves.empty() ||
      !curves.front().geometry)
    return Return::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Simple planar loop needs curves and positive tolerance"});
  const auto first =
      EvaluateCurve(*curves.front().geometry, curves.front().start);
  if (!first.ok()) return Return::Err(first.error());
  const auto n = plane.normal, x = plane.x_axis;
  const double sign = natural_sense ? 1 : -1;
  const Frame own_frame{
      first.value().position,
      x,
      {sign * (n.y * x.z - n.z * x.y), sign * (n.z * x.x - n.x * x.z),
       sign * (n.x * x.y - n.y * x.x)},
      n,
      plane.origin};
  const Frame& frame = shared_frame ? *shared_frame : own_frame;
  Certificate certificate;
  for (const auto& curve : curves) {
    if (!Spend(remaining_checks)) return Return::Err(Limit());
    const auto added =
        AddCurve(curve, frame, tolerance, remaining_checks, certificate);
    if (!added.ok()) return Return::Err(added.error());
  }
  if (!std::isfinite(certificate.area2) ||
      !std::isfinite(certificate.perimeter) ||
      std::abs(certificate.area2) <= tolerance * certificate.perimeter)
    return Return::Err(
        Unsupported("Simple planar loop has unresolved area or winding"));
  auto& pieces = certificate.pieces;
  for (std::size_t i = 0; i < pieces.size(); ++i) pieces[i].parent = i;
  const auto originals = pieces;
  const auto adjacent_proven = [&](const Piece& a, const Piece& b) {
    const auto& oa = originals[a.parent];
    const auto& ob = originals[b.parent];
    if (Join(oa, ob, tolerance)) return true;
    if (Length(oa.p[oa.degree] - ob.p[0]) > tolerance) return false;
    const auto joint = (oa.p[oa.degree] + ob.p[0]) * .5L;
    const auto within_joint = [&](const Piece& piece) {
      for (unsigned i = 0; i <= piece.degree; ++i)
        if (Length(piece.p[i] - joint) > tolerance) return false;
      return true;
    };
    // Descendants of two adjacent source pieces may approach the common
    // vertex arbitrarily closely, including a tangential cusp. Their full
    // intersection must stay in the admitted join ball; away from it a
    // strict separating axis suffices without demanding a nonzero gap at
    // that very same vertex. Unrelated pieces still require full clearance.
    return within_joint(a) || within_joint(b) || Separate(a, b, 0);
  };
  const auto proven = [&](const Piece& a, const Piece& b) {
    if (a.parent == b.parent) return Monotone(originals[a.parent]);
    if ((a.parent + 1) % originals.size() == b.parent)
      return adjacent_proven(a, b);
    if ((b.parent + 1) % originals.size() == a.parent)
      return adjacent_proven(b, a);
    return false;
  };
  while (true) {
    std::optional<std::size_t> refine;
    std::string stage = "monotonicity";
    for (std::size_t i = 0; i < pieces.size() && !refine; ++i) {
      if (!Spend(remaining_checks)) return Return::Err(Limit());
      if (!Monotone(pieces[i])) {
        refine = i;
        break;
      }
      const auto j = (i + 1) % pieces.size();
      if (!proven(pieces[i], pieces[j]) &&
          !Join(pieces[i], pieces[j], tolerance)) {
        stage = "endpoint separation";
        refine = pieces[i].degree > 1 && (pieces[j].degree == 1 ||
                                          pieces[i].depth <= pieces[j].depth)
                     ? i
                     : j;
        break;
      }
      for (std::size_t k = i + 2; k < pieces.size(); ++k) {
        if (i == 0 && k + 1 == pieces.size()) continue;
        if (!Spend(remaining_checks)) return Return::Err(Limit());
        if (!proven(pieces[i], pieces[k]) &&
            !Separate(pieces[i], pieces[k], tolerance)) {
          stage = "nonadjacent separation (parents " +
                  std::to_string(pieces[i].parent) + "," +
                  std::to_string(pieces[k].parent) + ")";
          refine = pieces[i].degree > 1 && (pieces[k].degree == 1 ||
                                            pieces[i].depth <= pieces[k].depth)
                       ? i
                       : k;
          break;
        }
      }
    }
    if (!refine) break;
    if (!Split(pieces, *refine, remaining_checks))
      return Return::Err(
          remaining_checks
              ? ParseError{ErrorCode::kUnsupportedFormat, 0,
                           std::string(
                               "Simple planar hull certificate failed at ") +
                               stage + ", piece " + std::to_string(*refine) +
                               ", degree " +
                               std::to_string(pieces[*refine].degree) +
                               ", depth " +
                               std::to_string(pieces[*refine].depth)}
              : Limit());
  }
  return Return::Ok(std::move(certificate));
}
bool Contains(const Certificate& loop, Point p, uint64_t& remaining) {
  bool inside = false;
  const auto cross = [&](Point a, Point b) {
    if ((a.y > p.y) != (b.y > p.y) &&
        p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
      inside = !inside;
  };
  for (std::size_t i = 0; i < loop.pieces.size(); ++i) {
    if (!Spend(remaining)) return false;
    const auto& piece = loop.pieces[i];
    const auto end = piece.p[piece.degree];
    cross(piece.p[0], end);
    // Independently evaluated source endpoints can differ within tolerance.
    // Close those certified joins for winding, including the last/first join.
    // Otherwise a ray through the tiny gap can produce odd parity outside a
    // loop. Cross-loop hull clearance keeps each connector away from the
    // tested loop. These connectors are never serialized as source geometry.
    const auto next = loop.pieces[(i + 1) % loop.pieces.size()].p[0];
    if (!Spend(remaining)) return false;
    cross(end, next);
  }
  return inside;
}
}  // namespace
Result<ConvexBoundary> ValidateSimplePlanarLoop(
    std::span<const PlanarCurve> curves, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<ConvexBoundary>;
  const auto certificate =
      CertifyLoop(curves, plane, natural_sense, tolerance, remaining_checks);
  if (!certificate.ok()) return Return::Err(certificate.error());
  if (certificate.value().area2 <= 0)
    return Return::Err(
        Unsupported("Simple planar loop winding opposes its normal"));
  const auto first =
      EvaluateCurve(*curves.front().geometry, curves.front().start);
  if (!first.ok()) return Return::Err(first.error());
  return Return::Ok({certificate.value().area2 / 2, first.value().position});
}
Result<PlanarRegion> ValidateSimplePlanarRegion(
    std::span<const std::vector<PlanarCurve>> loops, const PlaneGeometry& plane,
    bool natural_sense, double tolerance, uint64_t& remaining_checks) {
  using Return = Result<PlanarRegion>;
  if (loops.empty() || loops.front().empty() || !loops.front().front().geometry)
    return Return::Err(
        Unsupported("Simple planar region needs nonempty loops"));
  const auto& first_curve = loops.front().front();
  const auto first = EvaluateCurve(*first_curve.geometry, first_curve.start);
  if (!first.ok()) return Return::Err(first.error());
  const auto n = plane.normal, x = plane.x_axis;
  const double sign = natural_sense ? 1 : -1;
  const Frame frame{
      first.value().position,
      x,
      {sign * (n.y * x.z - n.z * x.y), sign * (n.z * x.x - n.x * x.z),
       sign * (n.x * x.y - n.y * x.x)},
      n,
      plane.origin};
  std::vector<Certificate> certificates;
  std::optional<std::size_t> outer;
  long double area = 0;
  for (const auto& loop : loops) {
    auto result = CertifyLoop(loop, plane, natural_sense, tolerance,
                              remaining_checks, &frame);
    if (!result.ok()) return Return::Err(result.error());
    if (result.value().area2 > 0) {
      if (outer)
        return Return::Err(
            Unsupported("Simple planar region has multiple outer loops"));
      outer = certificates.size();
    }
    area += result.value().area2 / 2;
    certificates.push_back(std::move(result).value());
  }
  if (!outer || !std::isfinite(area) || area <= 0)
    return Return::Err(
        Unsupported("Simple planar region has no positive outer area"));
  // Separate complete control hulls across loops before testing containment.
  // Replacing a Bezier arc by its chord is then a homotopy avoiding every
  // other loop. Chord winding gives exact containment for these certificates.
  bool refined;
  do {
    refined = false;
    for (std::size_t i = 0; i < certificates.size() && !refined; ++i)
      for (std::size_t j = i + 1; j < certificates.size() && !refined; ++j)
        for (std::size_t a = 0; a < certificates[i].pieces.size() && !refined;
             ++a)
          for (std::size_t b = 0; b < certificates[j].pieces.size(); ++b) {
            if (!Spend(remaining_checks)) return Return::Err(Limit());
            auto& left = certificates[i].pieces;
            auto& right = certificates[j].pieces;
            if (Separate(left[a], right[b], tolerance)) continue;
            const bool split_left =
                left[a].degree > 1 &&
                (right[b].degree == 1 || left[a].depth <= right[b].depth);
            if (!(split_left ? Split(left, a, remaining_checks)
                             : Split(right, b, remaining_checks)))
              return Return::Err(
                  remaining_checks ? Unsupported("Simple planar loop hulls "
                                                 "touch or cannot be separated")
                                   : Limit());
            refined = true;
            break;
          }
  } while (refined);
  for (std::size_t i = 0; i < certificates.size(); ++i) {
    if (i == *outer) continue;
    const auto p = certificates[i].pieces.front().p[0];
    if (!Contains(certificates[*outer], p, remaining_checks))
      return Return::Err(
          remaining_checks
              ? Unsupported("Simple planar hole is outside its outer loop")
              : Limit());
    for (std::size_t j = 0; j < certificates.size(); ++j) {
      if (j == *outer || i == j) continue;
      if (Contains(certificates[j], p, remaining_checks))
        return Return::Err(Unsupported("Simple planar holes are nested"));
      if (!remaining_checks) return Return::Err(Limit());
    }
  }
  return Return::Ok({*outer, area, false});
}
}  // namespace openswstep::internal
