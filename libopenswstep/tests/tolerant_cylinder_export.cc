// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <iostream>

#include "fixtures.h"
#include "openswstep/export.h"
#include "tolerant_cylinder_fixture.h"

// Independent validation of the recovered curve, not whole B_SURF face export.
int main(int argc, char** argv) {
  if (argc != 2) return 1;
  using namespace openswstep;
  using namespace openparasolid::test;
  const auto fixture = test::CylinderNurbsFixture();
  uint64_t checks = 10000000;
  const auto recovered = test::RecoverFixture(fixture, checks);
  if (!recovered.ok()) {
    std::cerr << recovered.error().message << '\n';
    return 2;
  }
  auto wire =
      test::Polyhedron({fixture.vertices.at(23), fixture.vertices.at(24)}, {});
  test::RemoveNode(wire, 4000);
  const auto& c = recovered.value().circle;
  wire.Add(31, 4000,
           {Numbers("center", {c.center.x, c.center.y, c.center.z},
                    openparasolid::FieldType::kVector),
            Numbers("normal", {c.normal.x, c.normal.y, c.normal.z},
                    openparasolid::FieldType::kVector),
            Numbers("x_axis", {c.x_axis.x, c.x_axis.y, c.x_axis.z},
                    openparasolid::FieldType::kVector),
            Numbers("radius", {c.radius}),
            ByteValue("sense", '+', openparasolid::FieldType::kCharacter)});
  const auto model = openparasolid::ModelBuilder::Build(wire.report);
  if (!model.ok()) return 3;
  ExportOptions options;
  options.length_unit = LengthUnit::kMetre;
  options.tolerance = recovered.value().tolerance;
  const auto step =
      ExportBody(model.value(), openparasolid::NodeId{1}, options);
  if (!step.ok()) {
    std::cerr << step.error().message << '\n';
    return 4;
  }
  if (!WriteStepFile(step.value(), argv[1]).ok()) return 5;
  std::cout << "Certified cylinder/NURBS master; checks consumed: "
            << 10000000 - checks << '\n';
}
