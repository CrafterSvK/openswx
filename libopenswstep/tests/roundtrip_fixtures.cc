// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "fixtures.h"
#include "openswstep/export.h"

int main(int argc, char** argv) {
  if (argc != 2) return 1;
  using namespace openswstep;
  using openparasolid::NodeId;
  const std::vector<std::pair<std::string, openparasolid::test::ModelFixture>>
      fixtures{
          {"tetra", test::Tetrahedron()},
          {"frustum", test::Frustum()},
          {"frustum-apex-origin", test::FrustumAtApexOrigin()},
          {"frustum-narrowing", test::Frustum(true)},
          {"frustum-seams", test::Frustum(false, false, true)},
          {"conical-tube", test::Frustum(false, true)},
          {"conical-tube-seams", test::Frustum(false, true, true)},
          {"conical-mantle", test::Frustum(false, false, false, false, true)},
          {"oblique-frustum", test::RotateAndTranslate(test::Frustum())},
          {"frustum-loop-order", test::ReversedFrustum()},
          {"frustum-axis", test::ReversedFrustum(true)},
          {"cylinder", test::Cylinder()},
          {"cylinder-shifted-seams", test::ShiftedCylinderSeams()},
          {"cylinder-shifted-frames", test::ShiftedCylinderSeams(false)},
          {"cylinder-loop-order", test::ReversedCylinder()},
          {"cylinder-axis", test::ReversedCylinder(true)},
          {"cylinder-seams", test::Cylinder(false, true)},
          {"tube", test::Cylinder(true)},
          {"tube-seams", test::Cylinder(true, true)},
          {"cylinder-mantle", test::Cylinder(false, false, false, true)},
          {"cylinder-sheet", test::Cylinder(false, false, false)},
          {"oblique-cylinder", test::RotateAndTranslate(test::Cylinder())},
          {"oblique-tube", test::RotateAndTranslate(test::Cylinder(true))},
          {"disk", test::ConicSheet()},
          {"disk-seam", test::ConicSheet({test::ConicLoopSpec{}}, true)},
          {"ellipse-disk", test::ConicSheet({{{0, 0, 0}, 2, 1, true}})},
          {"circular-annulus", test::CircularAnnulus()},
          {"circular-annulus-hole-first", test::CircularAnnulus(true, true)},
          {"circular-holes", test::CircularHoles()},
          {"oblique-circular-holes",
           test::RotateAndTranslate(test::CircularHoles())},
          {"disk-reversed",
           test::ConicSheet({{{0, 0, 0}, 1, 1, false, true, true}})},
          {"disk-negative-normal",
           test::ConicSheet({{{0, 0, 0}, 1, 1, false, true, false, true}})},
          {"cube", test::Cube()},
          {"perforated-sheet", test::PerforatedSheet()},
          {"hole-first-sheet", test::PerforatedSheet(true)},
          {"two-hole-sheet", test::TwoHoleSheet()},
          {"perforated-prism", test::PerforatedPrism()},
          {"negative-hole-sheet", test::NegativePerforatedSheet()},
          {"oblique-hole-sheet", test::ObliquePerforatedSheet()},
          {"square", test::Square()},
          {"concave",
           test::Polyhedron(
               {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {1, 1, 0}, {0, 2, 0}},
               {{0, 1, 2, 3, 4}})},
          {"line", test::Polyhedron({{0, 0, 0}, {2, 0, 0}}, {})},
          {"circle", test::Conic()},
          {"circle-reverse", test::Conic(false, false, true)},
          {"ring", test::Conic(false, true)},
          {"ellipse", test::Conic(true)},
          {"ellipse-ring", test::Conic(true, true)},
          {"acorn", test::Acorn()},
          {"sheet-pair",
           test::Polyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                            {{0, 1, 2}, {0, 1, 3}})}};
  for (const auto& [name, fixture] : fixtures) {
    const auto model = openparasolid::ModelBuilder::Build(fixture.report);
    if (!model.ok()) {
      std::cerr << model.error().message << '\n';
      return 2;
    }
    ExportOptions options;
    options.length_unit = LengthUnit::kMillimetre;
    const auto step = ExportBody(model.value(), NodeId{1}, options);
    if (!step.ok()) {
      std::cerr << name << ": " << step.error().message << '\n';
      return 3;
    }
    const auto written = WriteStepFile(
        step.value(), std::filesystem::path(argv[1]) / (name + ".step"));
    if (!written.ok()) {
      std::cerr << written.error().message << '\n';
      return 4;
    }
  }
}
