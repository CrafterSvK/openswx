// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <gtest/gtest.h>

#include "tolerant_cylinder_fixture.h"
namespace openswstep {
namespace {
using namespace openparasolid;
TEST(OpenSwStepTolerantCylinder,
     CertifiesFinitePlaneIntersectionMastersAndRetainsBothPcurves) {
  for (bool master : {false, true})
    for (bool circle : {false, true}) {
      const auto f = test::CylinderNurbsFixture(master, circle);
      uint64_t checks = 10000000;
      const auto result = test::RecoverFixture(f, checks);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().edge.value, 20u);
      EXPECT_DOUBLE_EQ(result.value().tolerance, 2e-6);
      ASSERT_EQ(result.value().uses.size(), 2u);
      EXPECT_DOUBLE_EQ(result.value().circle.radius, .010);
      EXPECT_DOUBLE_EQ(result.value().circle.center.z, 0);
      EXPECT_EQ(result.value().uses[0].pcurve.controls.values,
                (std::vector<double>{f.first, 0, f.last, 0}));
      EXPECT_EQ(result.value().uses[1].pcurve.controls.values,
                (std::vector<double>{.5, .50, .5, .45}));
      EXPECT_LT(checks, uint64_t{10000000});
    }
}
TEST(OpenSwStepTolerantCylinder,
     RequiresExplicitSourceAllowanceAndEnforcesFiniteWork) {
  const auto f = test::CylinderNurbsFixture();
  uint64_t checks = 10000000;
  auto strict = test::RecoverFixture(f, checks, false);
  ASSERT_FALSE(strict.ok());
  EXPECT_EQ(strict.error().code, ErrorCode::kUnsupportedFormat);
  checks = 0;
  auto limit = test::RecoverFixture(f, checks);
  ASSERT_FALSE(limit.ok());
  EXPECT_EQ(limit.error().code, ErrorCode::kResourceLimit);
  checks = 10000000;
  EXPECT_FALSE(test::RecoverFixture(f, checks, true, 0).ok());
}
TEST(OpenSwStepTolerantCylinder,
     RejectsUnprovenMasterBranchesAndSourceDepartures) {
  for (unsigned bad = 0; bad < 12; ++bad) {
    auto f = test::CylinderNurbsFixture();
    if (bad == 0) f.fins[1].use.sense = TopologySense::kForward;
    if (bad == 1) f.fins[1].use.edge_curve = {};
    if (bad == 2) f.vertices[23].x += .001;
    if (bad == 5) f.fins[1].curve = NodeId{100};
    if (bad == 8) f.fins[1].intersection.reset();
    for (auto& n : f.source.report.nodes) {
      if (bad == 3 && n.index == 30) n.fields[0].values = {0., 0., .001};
      if (bad == 4 && n.index == 50) n.fields[3].values = {-1.};
      if (bad == 6 && n.index == 103) n.fields[0].values[3] = 1e-7;
      if (bad == 9 && n.index == 13) n.fields[0].values[3] = -1.;
      if (bad == 10 && n.index == 113) n.fields[0].values[0] = -1.;
      if (bad == 11 && n.index == 101) {
        const auto& defs =
            f.source.report.schemas[n.schema_index].definition.fields;
        for (auto& field : n.fields)
          if (defs[field.definition_index].name == "sense")
            field.values = {uint64_t{'-'}};
      }
      if (bad == 7 && n.index == 12) {
        const auto& defs =
            f.source.report.schemas[n.schema_index].definition.fields;
        for (auto& field : n.fields)
          if (defs[field.definition_index].name == "u_closed")
            field.values = {true};
      }
    }
    uint64_t checks = 10000000;
    EXPECT_FALSE(test::RecoverFixture(f, checks).ok()) << bad;
  }
  auto bent = test::CylinderNurbsFixture(true, false, 1e-4);
  uint64_t checks = 10000000;
  EXPECT_FALSE(test::RecoverFixture(bent, checks).ok());
}
}  // namespace
}  // namespace openswstep
