// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "../src/evaluate_internal.h"

namespace openparasolid {
namespace {
NurbsCurveGeometry Parabola() {
  NurbsCurveGeometry c;
  c.axis.degree = 2;
  c.axis.control_count = 3;
  c.axis.knots = {2, 7};
  c.axis.multiplicities = {3, 3};
  c.controls.dimension = 3;
  c.controls.values = {0, 0, 0, .5, 0, 1.5, 1, 2, 3};
  return c;
}
TEST(ParasolidSplineCurveInverse,
     RecoversAnalyticInteriorAndBoundaryParameters) {
  const auto curve = Parabola();
  for (const double t : {0., .125, .25, .5, .875, 1.}) {
    const Vector3 point{t, 2 * t * t, 3 * t};
    const auto result = internal::InvertMonotoneNurbsCurve(curve, point, 1e-12);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_NEAR(result.value(), 2 + 5 * t, 1e-12);
  }
  const auto exact =
      internal::InvertMonotoneNurbsCurve(curve, {.5, .5, 1.5}, 0);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  EXPECT_EQ(exact.value(), 4.5);
}
TEST(ParasolidSplineCurveInverse, PreservesMultispanDomainsAndSpatialControls) {
  auto curve = Parabola();
  curve.axis.control_count = 4;
  curve.axis.knots = {-3, -1, 2};
  curve.axis.multiplicities = {3, 1, 3};
  curve.controls.values = {0, 0, 0, .3, .1, .2, .7, .4, .6, 1, 1, 1};
  for (const double u : {-3., -2., -1., 0., 1.5, 2.}) {
    const auto point = internal::EvaluateNurbsCurve(curve, u);
    ASSERT_TRUE(point.ok());
    const auto inverse = internal::InvertMonotoneNurbsCurve(
        curve, point.value().position, 1e-12);
    ASSERT_TRUE(inverse.ok()) << inverse.error().message;
    EXPECT_NEAR(inverse.value(), u, 1e-12);
  }
}
TEST(ParasolidSplineCurveInverse,
     CertifiesPositiveBezierDerivativesBySubdivision) {
  auto curve = Parabola();
  curve.axis.degree = 3;
  curve.axis.control_count = 4;
  curve.axis.knots = {-2, 3};
  curve.axis.multiplicities = {4, 4};
  curve.controls.values = {0, 0, 0, 1, 0, 0, .9, 0, 0, 2, 0, 0};
  for (const double t : {.125, .25, .5, .75, .875}) {
    const double x = 3 * t - 3.3 * t * t + 2.3 * t * t * t;
    const auto inverse =
        internal::InvertMonotoneNurbsCurve(curve, {x, 0, 0}, 1e-12);
    ASSERT_TRUE(inverse.ok()) << inverse.error().message;
    EXPECT_NEAR(inverse.value(), -2 + 5 * t, 1e-12);
  }
  curve.controls.values = {0, 0, 0, 2, 0, 0, -1, 0, 0, 1, 0, 0};
  const auto folded =
      internal::InvertMonotoneNurbsCurve(curve, {.5, 0, 0}, 1e-12);
  ASSERT_FALSE(folded.ok());
  EXPECT_EQ(folded.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidSplineCurveInverse, RejectsUncertifiedBranchesAndOffCurvePoints) {
  const auto base = Parabola();
  for (unsigned bad = 0; bad < 6; ++bad) {
    auto curve = base;
    if (bad == 0) curve.axis.closed = true;
    if (bad == 1) curve.axis.periodic = true;
    if (bad == 2) {
      curve.controls.rational = true;
      curve.controls.dimension = 3;
      curve.controls.values = {0, 0, 1, .5, 0, 1, 1, 2, 1};
    }
    if (bad == 3) curve.controls.values = {0, 0, 0, 2, 2, 3, 1, 2, 3};
    if (bad == 4) {
      curve.axis.knots = {-3, 2, 7, 12};
      curve.axis.multiplicities = {1, 2, 2, 1};
    }
    const Vector3 point =
        bad == 5 ? Vector3{.5, .6, 1.5} : Vector3{.5, .5, 1.5};
    const auto result = internal::InvertMonotoneNurbsCurve(curve, point, 1e-8);
    ASSERT_FALSE(result.ok()) << bad;
    EXPECT_EQ(result.error().code, bad == 1 ? ErrorCode::kInvalidRecord
                                            : ErrorCode::kUnsupportedFormat)
        << bad;
  }
  const auto outside =
      internal::InvertMonotoneNurbsCurve(base, {2, 8, 6}, 1e-8);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kUnsupportedFormat);
}
NurbsCurveGeometry RisingRationalCoils() {
  NurbsCurveGeometry curve;
  curve.axis.degree = 2;
  curve.axis.control_count = 17;
  for (unsigned i = 0; i <= 8; ++i) {
    curve.axis.knots.push_back(2 + .75 * i);
    curve.axis.multiplicities.push_back(i == 0 || i == 8 ? 3 : 2);
  }
  curve.controls.rational = true;
  curve.controls.dimension = 4;
  curve.controls.values = {1, 0, 0, 1};
  // Two turns with exactly circular XY projection and monotone rational Z.
  // These original rational quadratic arcs are not an exact constant-pitch
  // helix: their axial coordinate follows the rational parameter, not angle.
  double x = 1, y = 0;
  const double weight = std::sqrt(.5);
  for (unsigned i = 0; i < 8; ++i) {
    curve.controls.values.insert(
        curve.controls.values.end(),
        {(x - y) * weight, (x + y) * weight, (i + .5) * weight, weight, -y, x,
         static_cast<double>(i + 1), 1});
    const double next = -y;
    y = x;
    x = next;
  }
  return curve;
}
TEST(ParasolidSplineCurveInverse,
     RecoversPartialRationalSpatialCoilsInOriginalKnotUnits) {
  const auto source = RisingRationalCoils();
  for (const double weight_scale : {1e-150, 1., 1e150}) {
    auto curve = source;
    for (auto& value : curve.controls.values) value *= weight_scale;
    for (unsigned quarter = 0; quarter < 8; ++quarter)
      for (const double t : {0., .125, .5, .875, 1.}) {
        const double w = std::sqrt(.5), a = (1 - t) * (1 - t),
                     b = 2 * w * t * (1 - t), c = t * t;
        const double denominator = a + b + c;
        Vector3 point{(a + b) / denominator, (b + c) / denominator,
                      quarter + (.5 * b + c) / denominator};
        for (unsigned rotation = 0; rotation < quarter; ++rotation) {
          const auto next_x = -point.y;
          point.y = point.x;
          point.x = next_x;
        }
        const auto inverse =
            internal::InvertMonotoneNurbsCurve(curve, point, 1e-12);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        EXPECT_NEAR(inverse.value(), 2 + .75 * (quarter + t), 1e-12);
      }
  }
  auto reversed = source;
  for (std::size_t i = 2; i < reversed.controls.values.size(); i += 4)
    reversed.controls.values[i] = -reversed.controls.values[i];
  const auto inverse =
      internal::InvertMonotoneNurbsCurve(reversed, {0, 1, -1}, 1e-12);
  ASSERT_TRUE(inverse.ok()) << inverse.error().message;
  EXPECT_NEAR(inverse.value(), 2.75, 1e-12);
  EXPECT_FALSE(
      internal::InvertMonotoneNurbsCurve(source, {0, 1.001, 1}, 1e-7).ok());
}
TEST(ParasolidSplineCurveInverse, RejectsRationalFoldsAndUncertifiedDomains) {
  // The advertised maximum degree must certify a regular rational branch;
  // the next degree stays unsupported even though this particular curve is
  // monotone. For weights 1+i and Cartesian X=i, binomial moments give
  // X(t)=[2*n*t+n*(n-1)*t^2]/[1+n*t] independently of the certificate.
  for (unsigned degree : {7u, 8u}) {
    auto boundary = Parabola();
    boundary.axis.degree = degree;
    boundary.axis.control_count = degree + 1;
    const auto endpoint_multiplicity = static_cast<uint16_t>(degree + 1);
    boundary.axis.multiplicities = {endpoint_multiplicity,
                                    endpoint_multiplicity};
    boundary.controls.rational = true;
    boundary.controls.dimension = 4;
    boundary.controls.values.clear();
    for (unsigned i = 0; i <= degree; ++i)
      boundary.controls.values.insert(
          boundary.controls.values.end(),
          {double(i * (i + 1)), 0, 0, double(i + 1)});
    const double x = (degree + degree * (degree - 1) * .25) / (1 + degree * .5);
    const auto result =
        internal::InvertMonotoneNurbsCurve(boundary, {x, 0, 0}, 1e-12);
    if (degree == 7) {
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_NEAR(result.value(), 4.5, 1e-12);
    } else {
      ASSERT_FALSE(result.ok());
      EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
    }
  }
  auto source = Parabola();
  source.controls.rational = true;
  source.controls.dimension = 4;
  // Homogeneous xyz,w, with a reversing projected derivative on this span.
  source.controls.values = {0, 0, 0, 1, 4, 0, 0, 2, 1, 0, 0, 1};
  EXPECT_FALSE(
      internal::InvertMonotoneNurbsCurve(source, {.5, 0, 0}, 1e-12).ok());
  for (unsigned bad = 0; bad < 4; ++bad) {
    auto changed = RisingRationalCoils();
    if (bad == 0) changed.axis.closed = true;
    if (bad == 1) changed.controls.values[3] = 0;
    if (bad == 2) {
      changed.axis.control_count = 4;
      changed.axis.knots = {0, .5, 1};
      changed.axis.multiplicities = {3, 1, 3};
      changed.controls.values = {0, 0, 0, 1, 1, 0, 1, 1,
                                 2, 0, 2, 1, 3, 0, 3, 1};
    }
    if (bad == 3) {
      // Positive weights alone do not establish a monotone axial branch.
      for (std::size_t i = 2; i < changed.controls.values.size(); i += 4)
        changed.controls.values[i] = 0;
    }
    EXPECT_FALSE(
        internal::InvertMonotoneNurbsCurve(changed, {0, 1, 1}, 1e-12).ok())
        << bad;
  }
}
// This original cubic has a reversing control-polygon step, but its two
// polynomial pieces have strictly positive derivatives. Rational weights keep
// the source simple knot; the certificate must not change its representation.
NurbsCurveGeometry SimpleKnotCurve(bool rational) {
  NurbsCurveGeometry curve;
  curve.axis.degree = 3;
  curve.axis.control_count = 5;
  curve.axis.knots = {-2, 0, 3};
  curve.axis.multiplicities = {4, 1, 4};
  curve.controls.dimension = rational ? 4 : 3;
  curve.controls.rational = rational;
  const std::array<double, 5> x{0, 1, .9, 2, 3};
  const std::array<double, 5> weights{1, .8, 1.3, .9, 1.1};
  for (unsigned i = 0; i < 5; ++i) {
    const auto w = rational ? weights[i] : 1;
    curve.controls.values.insert(curve.controls.values.end(), {x[i] * w, 0, 0});
    if (rational) curve.controls.values.push_back(w);
  }
  return curve;
}
TEST(ParasolidSplineCurveInverse,
     CertifiesSimpleKnotSpansAndPreservesOriginalData) {
  for (const bool rational : {false, true}) {
    const auto original = SimpleKnotCurve(rational);
    for (const double scale : {1e-150, 1., 1e150}) {
      auto curve = original;
      if (rational)
        for (auto& value : curve.controls.values) value *= scale;
      const auto before = curve.controls.values;
      const auto certificate =
          internal::CertifyMonotoneNurbsProjection(curve, {1, 0, 0});
      ASSERT_TRUE(certificate.ok()) << certificate.error().message;
      ASSERT_TRUE(certificate.value()) << rational;
      for (const double u : {-2., -1.75, -.1, 0., .001, 1., 2.9, 3.}) {
        const auto sample = internal::EvaluateNurbsCurve(curve, u);
        ASSERT_TRUE(sample.ok());
        const auto inverse = internal::InvertMonotoneNurbsCurve(
            curve, sample.value().position, 1e-12);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        EXPECT_NEAR(inverse.value(), u, 1e-12);
      }
      EXPECT_EQ(curve.controls.values, before);
      EXPECT_EQ(curve.axis.knots, original.axis.knots);
      EXPECT_EQ(curve.axis.multiplicities, original.axis.multiplicities);
      EXPECT_FALSE(
          internal::InvertMonotoneNurbsCurve(curve, {1, 1e-4, 0}, 1e-7).ok());
    }
  }
}
TEST(ParasolidSplineCurveInverse,
     RecoversEverySupportedDegreeAgainstIndependentRationalMoments) {
  for (unsigned degree = 1; degree <= 7; ++degree) {
    NurbsCurveGeometry curve;
    curve.axis.degree = degree;
    curve.axis.control_count = degree + 2;
    curve.axis.knots = {2, 3.5, 7};
    const auto end = static_cast<uint16_t>(degree + 1);
    curve.axis.multiplicities = {end, 1, end};
    curve.controls.rational = true;
    curve.controls.dimension = 4;
    // Start with homogeneous poles (i*(i+1), 0, 0, i+1). Insert t=.3 once;
    // binomial moments still give the original rational function independently
    // of the B-spline evaluator or certificate's blossom arithmetic.
    for (unsigned i = 0; i <= degree + 1; ++i) {
      const double x = i == 0 ? 0
                       : i == degree + 1
                           ? degree * (degree + 1.)
                           : .7 * (i - 1.) * i + .3 * i * (i + 1.);
      const double w = i == 0            ? 1
                       : i == degree + 1 ? degree + 1.
                                         : .7 * i + .3 * (i + 1.);
      curve.controls.values.insert(curve.controls.values.end(), {x, 0, 0, w});
    }
    const auto before = curve.controls.values;
    for (const double t : {.05, .3, .75, .95}) {
      const double x =
          (2 * degree * t + degree * (degree - 1) * t * t) / (1 + degree * t);
      const auto inverse =
          internal::InvertMonotoneNurbsCurve(curve, {x, 0, 0}, 1e-12);
      ASSERT_TRUE(inverse.ok()) << degree << ": " << inverse.error().message;
      EXPECT_NEAR(inverse.value(), 2 + 5 * t, 1e-12);
    }
    EXPECT_EQ(curve.controls.values, before);
  }
}
TEST(ParasolidSplineCurveInverse,
     RejectsNarrowInteriorFoldsBetweenOrdinarySamples) {
  auto curve = SimpleKnotCurve(true);
  curve.axis.knots = {0, .4, 1};
  // X'(t)=(t-a)^2-d^2 is negative only within a 20-microparameter interval.
  // The cubic is represented after one exact simple-knot insertion. Positive
  // weights and a nonzero endpoint chord do not prove injectivity.
  constexpr double a = .5003, d = .00001;
  const std::array<double, 4> bezier{0, (a * a - d * d) / 3,
                                     (2 * (a * a - d * d) - a) / 3,
                                     1. / 3 - a + a * a - d * d};
  const std::array<double, 5> poles{bezier[0], .6 * bezier[0] + .4 * bezier[1],
                                    .6 * bezier[1] + .4 * bezier[2],
                                    .6 * bezier[2] + .4 * bezier[3], bezier[3]};
  curve.controls.values.clear();
  for (auto x : poles)
    curve.controls.values.insert(curve.controls.values.end(), {x, 0, 0, 1});
  const auto certificate =
      internal::CertifyMonotoneNurbsProjection(curve, {1, 0, 0});
  ASSERT_TRUE(certificate.ok()) << certificate.error().message;
  EXPECT_FALSE(certificate.value());
  const auto inverse =
      internal::InvertMonotoneNurbsCurve(curve, {.04, 0, 0}, 1e-12);
  ASSERT_FALSE(inverse.ok());
  EXPECT_EQ(inverse.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(ParasolidSplineCurveInverse,
     HandlesMixedMultiplicitiesAndRejectsAmbiguousDirections) {
  auto curve = SimpleKnotCurve(true);
  curve.axis.control_count = 7;
  curve.axis.knots = {-7, -2, .125, 11};
  curve.axis.multiplicities = {4, 1, 2, 4};
  curve.controls.values.clear();
  for (unsigned i = 0; i < 7; ++i) {
    const auto w = 1. + .1 * i;
    curve.controls.values.insert(curve.controls.values.end(),
                                 {i * w, 0, 2 * i * w, w});
  }
  for (const auto direction :
       {std::array<long double, 3>{1, 0, 2}, {0, 0, 1}}) {
    const auto certificate =
        internal::CertifyMonotoneNurbsProjection(curve, direction);
    ASSERT_TRUE(certificate.ok()) << certificate.error().message;
    EXPECT_TRUE(certificate.value());
  }
  for (const auto direction :
       {std::array<long double, 3>{-1, 0, -2}, {0, 1, 0}}) {
    const auto certificate =
        internal::CertifyMonotoneNurbsProjection(curve, direction);
    ASSERT_TRUE(certificate.ok());
    EXPECT_FALSE(certificate.value());
  }
  const auto invalid =
      internal::CertifyMonotoneNurbsProjection(curve, {0, 0, 0});
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
}
TEST(ParasolidSplineCurveInverse,
     BoundsRationalCertificateWorkBeforeSpanProcessing) {
  auto curve = RisingRationalCoils();
  constexpr unsigned spans = 120000;
  curve.axis.control_count = 2 * spans + 1;
  curve.axis.knots.clear();
  curve.axis.multiplicities.clear();
  for (unsigned i = 0; i <= spans; ++i) {
    curve.axis.knots.push_back(i);
    curve.axis.multiplicities.push_back(i == 0 || i == spans ? 3 : 2);
  }
  curve.controls.values.assign(4 * curve.axis.control_count, 1);
  const auto result =
      internal::CertifyMonotoneNurbsProjection(curve, {0, 0, 1});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidSplineCurveInverse, EnforcesFiniteOptionsWorkAndSearchLimits) {
  auto curve = Parabola();
  for (const double tolerance : {-1., std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN()}) {
    const auto result =
        internal::InvertMonotoneNurbsCurve(curve, {}, tolerance);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  }
  curve.axis.degree = 1;
  curve.axis.control_count = 2;
  curve.axis.knots = {0, 1e200};
  curve.axis.multiplicities = {2, 2};
  curve.controls.values = {0, 0, 0, 1, 0, 0};
  const auto search =
      internal::InvertMonotoneNurbsCurve(curve, {1e-150, 0, 0}, 0);
  ASSERT_FALSE(search.ok());
  EXPECT_EQ(search.error().code, ErrorCode::kResourceLimit);
  constexpr unsigned kControls = 100000;
  curve.axis.control_count = kControls;
  curve.axis.knots.clear();
  curve.axis.multiplicities.assign(kControls, 1);
  curve.axis.multiplicities.front() = curve.axis.multiplicities.back() = 2;
  curve.controls.values.clear();
  for (unsigned i = 0; i < kControls; ++i) {
    curve.axis.knots.push_back(i);
    curve.controls.values.insert(curve.controls.values.end(),
                                 {static_cast<double>(i), 0., 0.});
  }
  const auto work = internal::InvertMonotoneNurbsCurve(curve, {}, 1e-8);
  ASSERT_FALSE(work.ok());
  EXPECT_EQ(work.error().code, ErrorCode::kResourceLimit);
}
}  // namespace
}  // namespace openparasolid
