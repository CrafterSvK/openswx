// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

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
      curve.controls.dimension = 4;
      curve.controls.values = {0, 0, 0, 1, .5, 0, 1.5, 1, 1, 2, 3, 1};
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
