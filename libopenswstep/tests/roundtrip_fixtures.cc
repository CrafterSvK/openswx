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
      fixtures{{"tetra", test::Tetrahedron()},
               {"cube", test::Cube()},
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
