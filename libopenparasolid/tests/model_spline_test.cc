// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include "model_test_helpers.h"

namespace openparasolid {
namespace {
using namespace test;
DecodedField& Field(ModelFixture& fixture, std::size_t node, const char* name) {
  const auto& schema =
      fixture.report.schemas[fixture.report.nodes[node].schema_index]
          .definition;
  for (std::size_t i = 0; i < schema.fields.size(); ++i)
    if (schema.fields[i].name == name)
      return fixture.report.nodes[node].fields[i];
  ADD_FAILURE() << "Missing fixture field " << name;
  return fixture.report.nodes[node].fields[0];
}
TEST(ParasolidModelSpline,
     ResolvesSharedArraysAndPreservesHomogeneousCoordinates) {
  for (bool rational : {false, true}) {
    ModelFixture f;
    f.Spline(rational);
    const auto result = ModelBuilder::Build(f.report);
    ASSERT_TRUE(result.ok());
    const auto& m = result.value();
    EXPECT_TRUE(m.references_verified());
    EXPECT_TRUE(m.typed_geometry_complete());
    ASSERT_EQ(m.geometry().size(), 1);
    const auto& curve = std::get<NurbsCurveGeometry>(m.geometry()[0].payload);
    EXPECT_EQ(curve.descriptor.value, 2);
    EXPECT_EQ(curve.axis.degree, 2);
    EXPECT_EQ(curve.axis.control_count, 3);
    EXPECT_EQ(curve.axis.knots, (std::vector<double>{0., 1.}));
    EXPECT_EQ(curve.axis.multiplicities, (std::vector<uint16_t>{3, 3}));
    EXPECT_EQ(curve.controls.dimension, rational ? 4 : 3);
    EXPECT_EQ(curve.controls.rational, rational);
    EXPECT_EQ(curve.controls.values.size(), rational ? 12 : 9);
    if (rational) {
      EXPECT_EQ(curve.controls.values[7], 2.);
      EXPECT_EQ(curve.controls.values[4], 1.);  // No division by weight 2.
    }
  }
}
TEST(ParasolidModelSpline, SurfaceUsesSeparateAxesAndProductControlCount) {
  ModelFixture f;
  f.Spline(true, true);
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().geometry().size(), 1);
  const auto& surface =
      std::get<NurbsSurfaceGeometry>(result.value().geometry()[0].payload);
  EXPECT_EQ(surface.u.control_count, 2);
  EXPECT_EQ(surface.v.control_count, 2);
  EXPECT_EQ(surface.u.degree, 1);
  EXPECT_EQ(surface.controls.dimension, 4);
  EXPECT_EQ(surface.controls.values.size(), 16);
  EXPECT_EQ(surface.u.knots, surface.v.knots);
  EXPECT_EQ(result.value().geometry()[0].sense, '-');
}
TEST(ParasolidModelSpline, RejectsBadDescriptorsKnotsControlsAndWeights) {
  for (int bad = 0; bad < 9; ++bad) {
    ModelFixture f;
    f.Spline(true);
    switch (bad) {
      case 0:
        Field(f, 1, "n_vertices").values[0] = int64_t{4};
        break;
      case 1:
        Field(f, 1, "degree").values[0] = int64_t{3};
        break;
      case 2:
        Field(f, 2, "vertices").values[3] = 0.;
        break;
      case 3:
        Field(f, 2, "vertices").values[0] = std::monostate{};
        break;
      case 4:
        Field(f, 3, "knots").values[1] = -1.;
        break;
      case 5:
        Field(f, 4, "multiplicities").values[0] = int64_t{-1};
        break;
      case 6:
        Field(f, 4, "multiplicities").values[0] = int64_t{2};
        break;
      case 7:
        Field(f, 1, "knots").values[0] = uint64_t{0};
        break;
      case 8:
        Field(f, 0, "nurbs").values[0] = uint64_t{100};
        break;
    }
    const auto result = ModelBuilder::Build(f.report);
    ASSERT_TRUE(result.ok()) << bad;
    EXPECT_TRUE(result.value().geometry().empty()) << bad;
    EXPECT_EQ(result.value().statistics().invalid_geometry, 1) << bad;
    EXPECT_FALSE(result.value().typed_geometry_complete());
  }
}
TEST(ParasolidModelSpline, SharedDataCopiesCannotBypassModelValueBudget) {
  ModelFixture f;
  f.Spline();
  for (uint32_t id : {6u, 7u, 8u}) f.Curve(id);
  ModelOptions options;
  options.max_values =
      40;  // Input uses 33 values; four geometry copies need 52.
  const auto result = ModelBuilder::Build(f.report, options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
TEST(ParasolidModelSpline, DoesNotPublishPartiallyBuiltSurfaceOnBadSecondAxis) {
  ModelFixture f;
  f.Spline(false, true);
  Field(f, 1, "v_degree").values[0] = int64_t{3};
  const auto result = ModelBuilder::Build(f.report);
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().geometry().empty());
  EXPECT_FALSE(result.value().nodes()[0].geometry.has_value());
  EXPECT_EQ(result.value().statistics().invalid_geometry, 1);
}
}  // namespace
}  // namespace openparasolid
