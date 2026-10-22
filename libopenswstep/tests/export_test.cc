// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openswstep/export.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <fstream>
#include <limits>
#include <numbers>
#include <regex>
#include <set>
#include <utility>

#include "../../libopenswx/tests/test_helpers.h"
#include "../src/annular_boundary.h"
#include "../src/cone_patch.h"
#include "../src/cylinder_band.h"
#include "../src/cylinder_patch.h"
#include "../src/spun_torus.h"
#include "../src/tolerant_circle.h"
#include "../src/torus_boundary.h"
#include "fixtures.h"

namespace openswstep {
namespace {
using namespace openparasolid;
ExportOptions Options() {
  ExportOptions options;
  options.length_unit = LengthUnit::kMetre;
  return options;
}
Result<StepDocument> Export(const test::ModelFixture& f,
                            ExportOptions options = Options()) {
  const auto model = ModelBuilder::Build(f.report);
  if (!model.ok()) return Result<StepDocument>::Err(model.error());
  return ExportBody(model.value(), NodeId{1}, options);
}
void CheckReferences(const StepDocument& document) {
  const std::regex definitions("#([0-9]+)="), references("#([0-9]+)");
  std::set<std::string> ids;
  for (std::sregex_iterator
           i(document.text.begin(), document.text.end(), definitions),
       end;
       i != end; ++i)
    EXPECT_TRUE(ids.insert((*i)[1]).second);
  EXPECT_EQ(ids.size(), document.statistics.entities);
  for (std::sregex_iterator
           i(document.text.begin(), document.text.end(), references),
       end;
       i != end; ++i)
    EXPECT_TRUE(ids.contains((*i)[1]));
  EXPECT_TRUE(document.text.starts_with("ISO-10303-21;\nHEADER;"));
  EXPECT_TRUE(document.text.ends_with("END-ISO-10303-21;\n"));
}
TEST(OpenSwStep, ExportsClosedSolidWithSharedEdgesAndVertices) {
  const auto result = Export(test::Tetrahedron());
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
  EXPECT_EQ(result.value().statistics.shells, 1);
  EXPECT_EQ(result.value().statistics.faces, 4);
  EXPECT_EQ(result.value().statistics.edges, 6);
  EXPECT_EQ(result.value().statistics.vertices, 4);
  EXPECT_NE(result.value().text.find("MANIFOLD_SOLID_BREP("),
            std::string::npos);
  CheckReferences(result.value());
}
TEST(OpenSwStep, ExportsSheetAndLineWireDeterministically) {
  const auto f = test::Square();
  const auto a = Export(f), b = Export(f);
  ASSERT_TRUE(a.ok()) << a.error().message;
  ASSERT_TRUE(b.ok());
  EXPECT_EQ(a.value().text, b.value().text);
  EXPECT_EQ(a.value().statistics.faces, 1);
  EXPECT_EQ(a.value().statistics.solids, 0);
  EXPECT_NE(a.value().text.find("OPEN_SHELL("), std::string::npos);
  CheckReferences(a.value());
  const auto wire = Export(test::Polyhedron({{0, 0, 0}, {2, 0, 0}}, {}));
  ASSERT_TRUE(wire.ok()) << wire.error().message;
  EXPECT_EQ(wire.value().statistics.edges, 1);
  EXPECT_NE(
      wire.value().text.find("EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION("),
      std::string::npos);
  CheckReferences(wire.value());
}
TEST(OpenSwStep, RequiresUnitsAndBoundsOutput) {
  auto options = Options();
  options.length_unit = LengthUnit::kUnspecified;
  auto result = Export(test::Square(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
  options = Options();
  options.max_entities = 1;
  result = Export(test::Square(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  options = Options();
  options.max_output_bytes = 32;
  result = Export(test::Square(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  options = Options();
  options.tolerance = std::numeric_limits<double>::quiet_NaN();
  result = Export(test::Square(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
}
TEST(OpenSwStep, AcceptsExactEntityAndByteBudgets) {
  const auto fixture = test::Square();
  const auto baseline = Export(fixture);
  ASSERT_TRUE(baseline.ok());
  auto options = Options();
  options.max_entities = baseline.value().statistics.entities;
  options.max_output_bytes = baseline.value().text.size();
  const auto exact = Export(fixture, options);
  ASSERT_TRUE(exact.ok());
  EXPECT_EQ(exact.value().text, baseline.value().text);
  --options.max_output_bytes;
  const auto bytes = Export(fixture, options);
  ASSERT_FALSE(bytes.ok());
  EXPECT_EQ(bytes.error().code, ErrorCode::kResourceLimit);
  options.max_output_bytes = baseline.value().text.size();
  --options.max_entities;
  const auto entities = Export(fixture, options);
  ASSERT_FALSE(entities.ok());
  EXPECT_EQ(entities.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, ExportsCircularSectorsWithoutApproximation) {
  const auto result = Export(test::QuarterDisk());
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.vertices, 3);
  EXPECT_EQ(result.value().statistics.edges, 3);
  EXPECT_EQ(result.value().statistics.faces, 1);
  EXPECT_NE(result.value().text.find("=CIRCLE("), std::string::npos);
  CheckReferences(result.value());
}
TEST(OpenSwStep, RejectsCrossingPolygonsAndBoundsPolygonWork) {
  const auto crossing = Export(
      test::Polyhedron({{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}, {3, 1, 0}},
                       {{0, 1, 2, 3, 4}}));
  ASSERT_FALSE(crossing.ok());
  EXPECT_EQ(crossing.error().code, ErrorCode::kInvalidRecord);
  auto options = Options();
  options.max_polygon_checks = 1;
  const auto result = Export(test::Square(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, SerializesExplicitUnitsAndEscapesProductNames) {
  auto options = Options();
  options.length_unit = LengthUnit::kMillimetre;
  options.product_name = "O'Brien\\part";
  const auto result = Export(test::Square(), options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_NE(result.value().text.find("SI_UNIT(.MILLI.,.METRE.)"),
            std::string::npos);
  EXPECT_NE(result.value().text.find("O''Brien\\\\part"), std::string::npos);
  options.product_name = "bad\nname";
  EXPECT_FALSE(Export(test::Square(), options).ok());
}
TEST(OpenSwStep, ParsesSelectedPayloadAndPreservesTruncationDiagnostics) {
  auto serialized = test::Serialize(test::Square());
  auto result = ExportParasolid(serialized.bytes, NodeId{1}, Options(),
                                &serialized.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  CheckReferences(result.value());
  openswx::ParasolidPayload payload;
  payload.header_valid = true;
  payload.kind = openswx::ParasolidPayloadKind::kTransmit;
  payload.payload.data = serialized.bytes;
  result = ExportPayload(payload, NodeId{1}, Options(), &serialized.catalog);
  ASSERT_TRUE(result.ok());
  payload.ghost_partition = true;
  EXPECT_FALSE(
      ExportPayload(payload, NodeId{1}, Options(), &serialized.catalog).ok());
  serialized.bytes.pop_back();
  const auto parsed =
      SchemaParser::Parse(serialized.bytes, &serialized.catalog);
  ASSERT_TRUE(parsed.ok());
  ASSERT_FALSE(parsed.value().complete);
  result = ExportParasolid(serialized.bytes, NodeId{1}, Options(),
                           &serialized.catalog);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().offset, parsed.value().diagnostics[0].error.offset);
  EXPECT_EQ(result.error().code, parsed.value().diagnostics[0].error.code);
}
TEST(OpenSwStep, ExportsConicArcsRingsAndPointOnlyAcorns) {
  for (const bool ellipse : {false, true})
    for (const bool ring : {false, true})
      for (const bool reverse : {false, true}) {
        const auto result = Export(test::Conic(ellipse, ring, reverse));
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().statistics.edges, 1);
        EXPECT_EQ(result.value().statistics.vertices, ring ? 1 : 2);
        EXPECT_NE(result.value().text.find(ellipse ? "ELLIPSE(" : "CIRCLE("),
                  std::string::npos);
        CheckReferences(result.value());
      }
  const auto acorn = Export(test::Acorn());
  ASSERT_TRUE(acorn.ok()) << acorn.error().message;
  EXPECT_NE(acorn.value().text.find("GEOMETRIC_CURVE_SET("), std::string::npos);
  CheckReferences(acorn.value());
}
TEST(OpenSwStep, RejectsUnrepresentableConicTurns) {
  auto options = Options();
  options.wire_intervals = {{{100}, 0, 4 * std::numbers::pi}};
  const auto result = Export(test::Conic(false, true), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, PropagatesSheetOrientationAndChecksVertexLinks) {
  // Both faces traverse their common edge in the same source direction.
  const auto sheet = Export(test::Polyhedron(
      {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {{0, 1, 2}, {0, 1, 3}}));
  ASSERT_TRUE(sheet.ok()) << sheet.error().message;
  EXPECT_EQ(sheet.value().statistics.shells, 1);
  EXPECT_EQ(sheet.value().statistics.edges, 5);
  const auto pinched = Export(test::Polyhedron(
      {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}},
      {{0, 1, 2}, {0, 3, 4}}));
  ASSERT_FALSE(pinched.ok());
  EXPECT_EQ(pinched.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, HandlesNegativeSurfaceAndFaceSenses) {
  auto f = test::Square();
  test::Set(f, 300, "sense", '-');
  for (auto& node : f.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  const auto result = Export(f);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_NE(result.value().text.find("ADVANCED_FACE("), std::string::npos);
}
TEST(OpenSwStep, RejectsInvertedSolidAndUnclosedSolidTopology) {
  auto inverted = test::Tetrahedron();
  // Reverse which side of each source face is material, preserving all links.
  for (uint32_t i = 0; i < 4; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto negative = Export(inverted);
  ASSERT_FALSE(negative.ok());
  EXPECT_EQ(negative.error().code, ErrorCode::kInvalidRecord);
  const auto open = Export(
      test::Polyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}}, true));
  ASSERT_FALSE(open.ok());
  EXPECT_EQ(open.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, RejectsGeneralBodiesAndUnsupportedFaces) {
  auto f = test::Square();
  test::Set(f, 1, "body_type", 6);
  const auto general = Export(f);
  ASSERT_FALSE(general.ok());
  EXPECT_EQ(general.error().code, ErrorCode::kUnsupportedFormat);
  f = test::Square();
  test::RemoveNode(f, 700);
  f.Add(51, 700,
        {test::Numbers("origin", {0, 0, 0}, FieldType::kVector),
         test::Numbers("axis", {0, 0, 1}, FieldType::kVector),
         test::Numbers("radius", {1}),
         test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         test::ByteValue("sense", '+', FieldType::kCharacter)});
  EXPECT_FALSE(Export(f).ok());
}
TEST(OpenSwStep, ExportsPolygonHolesWithoutAssumingSourceLoopOrder) {
  for (const bool hole_first : {false, true}) {
    const auto result = Export(test::PerforatedSheet(hole_first));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.vertices, 8);
    EXPECT_EQ(result.value().statistics.edges, 8);
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_EQ(result.value().statistics.solids, 0);
    const auto& text = result.value().text;
    const auto outer = text.find("=FACE_OUTER_BOUND(");
    const auto inner = text.find("=FACE_BOUND(");
    ASSERT_NE(outer, std::string::npos);
    ASSERT_NE(inner, std::string::npos);
    EXPECT_EQ(inner < outer, hole_first);
    CheckReferences(result.value());
  }
  const auto multiple = Export(test::TwoHoleSheet());
  ASSERT_TRUE(multiple.ok()) << multiple.error().message;
  EXPECT_EQ(multiple.value().statistics.edges, 12);
  CheckReferences(multiple.value());
}
TEST(OpenSwStep, PreservesThroughHolesInClosedPlanarSolids) {
  const auto result = Export(test::PerforatedPrism());
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.vertices, 16);
  EXPECT_EQ(result.value().statistics.edges, 24);
  EXPECT_EQ(result.value().statistics.faces, 10);
  EXPECT_EQ(result.value().statistics.solids, 1);
  EXPECT_NE(result.value().text.find("=FACE_BOUND("), std::string::npos);
  CheckReferences(result.value());
}
TEST(OpenSwStep, ChecksHoleContainmentWindingNestingAndIntersections) {
  const std::vector<Vector3> outer{{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}};
  for (const auto& hole : std::vector<std::vector<Vector3>>{
           {{5, 1, 0}, {5, 2, 0}, {6, 2, 0}, {6, 1, 0}},  // outside
           {{3, 1, 0},
            {3, 2, 0},
            {5, 2, 0},
            {5, 1, 0}},  // crosses outer boundary
           {{0, 1, 0}, {0, 2, 0}, {1, 2, 0}, {1, 1, 0}},  // touches outer edge
           {{1, 1, 0}, {3, 1, 0}, {3, 3, 0}, {1, 3, 0}},  // wrong winding
           {{-1, -1, 0},
            {-1, 5, 0},
            {5, 5, 0},
            {5, -1, 0}}}) {  // surrounds outer
    auto points = outer;
    points.insert(points.end(), hole.begin(), hole.end());
    const auto result =
        Export(test::PlanarFace(points, {{0, 1, 2, 3}, {4, 5, 6, 7}}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().offset, 0);
  }
  for (const auto& second :
       std::vector<std::vector<Vector3>>{{{1.5, 1.5, 0},
                                          {1.5, 2.5, 0},
                                          {2.5, 2.5, 0},
                                          {2.5, 1.5, 0}},  // nested
                                         {{2, 2, 0},
                                          {2, 3.5, 0},
                                          {3.5, 3.5, 0},
                                          {3.5, 2, 0}},  // crossing holes
                                         {{3, 1, 0},
                                          {3, 2, 0},
                                          {3.5, 2, 0},
                                          {3.5, 1, 0}}}) {  // touching holes
    auto points = outer;
    const std::vector<Vector3> first{
        {1, 1, 0}, {1, 3, 0}, {3, 3, 0}, {3, 1, 0}};
    points.insert(points.end(), first.begin(), first.end());
    points.insert(points.end(), second.begin(), second.end());
    const auto result = Export(
        test::PlanarFace(points, {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(OpenSwStep, BoundsHoleComparisonsAndContainmentTogether) {
  auto options = Options();
  options.max_polygon_checks = 24;
  const auto exact = Export(test::PerforatedSheet(true), options);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  options.max_polygon_checks = 23;
  const auto limited = Export(test::PerforatedSheet(true), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsToleranceAmbiguousHoleGaps) {
  for (const auto gap : {1e-9, 1e-5}) {
    const auto f = test::PlanarFace({{0, 0, 0},
                                     {4, 0, 0},
                                     {4, 4, 0},
                                     {0, 4, 0},
                                     {gap, 1, 0},
                                     {gap, 2, 0},
                                     {1, 2, 0},
                                     {1, 1, 0}},
                                    {{0, 1, 2, 3}, {4, 5, 6, 7}});
    const auto result = Export(f);
    EXPECT_EQ(result.ok(), gap > Options().tolerance);
    if (!result.ok()) EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(OpenSwStep, HandlesHolesWithNegativeSensesAndObliqueTranslatedPlanes) {
  for (const auto& f :
       {test::NegativePerforatedSheet(), test::ObliquePerforatedSheet()}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.edges, 8);
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, ParsesHoleLoopsThroughTheBinaryExportEntryPoint) {
  const auto fixture = test::Serialize(test::PerforatedPrism());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 10);
  CheckReferences(result.value());
}
TEST(OpenSwStep, ExportsExactConicDisksWithSyntheticOrSourceSeams) {
  for (const bool ellipse : {false, true}) {
    for (const bool named_seam : {false, true}) {
      test::ConicLoopSpec loop;
      loop.ellipse = ellipse;
      loop.major = ellipse ? 2 : 1;
      const auto result = Export(test::ConicSheet({loop}, named_seam));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.vertices, 1);
      EXPECT_EQ(result.value().statistics.edges, 1);
      EXPECT_EQ(result.value().statistics.faces, 1);
      EXPECT_NE(result.value().text.find(ellipse ? "=ELLIPSE(" : "=CIRCLE("),
                std::string::npos);
      CheckReferences(result.value());
    }
  }
}
TEST(OpenSwStep, PreservesCircularHolesAndSourceLoopOrder) {
  for (const bool hole_first : {false, true}) {
    for (const bool named_seams : {false, true}) {
      const auto result =
          Export(test::CircularAnnulus(hole_first, named_seams));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.vertices, 2);
      EXPECT_EQ(result.value().statistics.edges, 2);
      EXPECT_EQ(result.value().statistics.faces, 1);
      const auto& text = result.value().text;
      EXPECT_EQ(text.find("=FACE_BOUND(") < text.find("=FACE_OUTER_BOUND("),
                hole_first);
      CheckReferences(result.value());
    }
  }
  const auto result = Export(test::CircularHoles());
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.edges, 3);
  CheckReferences(result.value());
}
TEST(OpenSwStep, ChecksConicLoopDirectionAgainstTheFaceNormal) {
  for (const bool reverse_curve : {false, true})
    for (const bool reverse_fin : {false, true})
      for (const bool reverse_normal : {false, true}) {
        test::ConicLoopSpec loop;
        loop.reverse_curve = reverse_curve;
        loop.reverse_fin = reverse_fin;
        loop.reverse_normal = reverse_normal;
        const auto result = Export(test::ConicSheet({loop}));
        const bool positive = (reverse_curve == reverse_fin) != reverse_normal;
        EXPECT_EQ(result.ok(), positive);
        if (!result.ok())
          EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
      }
  auto negative = test::ConicSheet();
  test::Set(negative, 300, "sense", '-');
  test::Set(negative, 700, "sense", '-');
  const auto result = Export(negative);
  ASSERT_TRUE(result.ok()) << result.error().message;
}
TEST(OpenSwStep, RejectsInvalidCircularHolesAndUnimplementedEllipseHoles) {
  test::ConicLoopSpec outer;
  outer.major = 3;
  test::ConicLoopSpec inner;
  inner.reverse_curve = true;
  for (const double x : {2., 2.5, 4., 2. - 1e-9}) {
    inner.center.x = x;
    const auto result = Export(test::ConicSheet({outer, inner}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().offset, 0);
  }
  inner.center = {0, 0, 0};
  for (const double x : {0., 0.5, 2.}) {
    auto second = inner;
    second.center.x = x;
    const auto result = Export(test::ConicSheet({outer, inner, second}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  inner.ellipse = true;
  const auto ellipse = Export(test::ConicSheet({outer, inner}));
  ASSERT_FALSE(ellipse.ok());
  EXPECT_EQ(ellipse.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, BoundsCircularHoleComparisonsAcrossAllLoops) {
  auto options = Options();
  options.max_polygon_checks = 3;
  const auto exact = Export(test::CircularHoles(), options);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  options.max_polygon_checks = 2;
  const auto limited = Export(test::CircularHoles(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsPartialMultipleTurnAndNonplanarConicFaces) {
  for (const double span : {std::numbers::pi, 4 * std::numbers::pi}) {
    auto options = Options();
    options.fin_intervals = {{NodeId{1000}, 0, span}};
    const auto result = Export(test::ConicSheet(), options);
    EXPECT_FALSE(result.ok());
  }
  test::ConicLoopSpec offset;
  offset.center.z = 1e-5;
  EXPECT_FALSE(Export(test::ConicSheet({offset})).ok());
  test::ConicLoopSpec small;
  small.major = 1e-8;
  EXPECT_FALSE(Export(test::ConicSheet({small})).ok());
}
TEST(OpenSwStep, ChecksTheWholeConicAgainstItsPlaneBeyondBindingSamples) {
  auto tilted = test::ConicSheet();
  constexpr double angle = 1e-6;
  for (auto& node : tilted.report.nodes)
    if (node.index == 4000)
      node.fields[1].values = {0., std::sin(angle), std::cos(angle)};
  // The three binding samples at 0, pi and 2*pi lie on the X axis in the
  // plane. The extrema between them exceed the plane tolerance.
  const auto result = Export(tilted);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("Complete conic boundary"),
            std::string::npos);
  const auto oblique = Export(test::RotateAndTranslate(test::CircularHoles()));
  ASSERT_TRUE(oblique.ok()) << oblique.error().message;
  EXPECT_EQ(oblique.value().statistics.edges, 3);
}
TEST(OpenSwStep, ParsesCircularFaceLoopsThroughTheBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::CircularHoles());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 1);
  EXPECT_EQ(result.value().statistics.edges, 3);
  CheckReferences(result.value());
}
TEST(OpenSwStep, ExportsExactCylindersAndTubesWithSharedCircularEdges) {
  for (const bool tube : {false, true}) {
    for (const bool named_seams : {false, true}) {
      const auto result = Export(test::Cylinder(tube, named_seams));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.faces, tube ? 4 : 3);
      EXPECT_EQ(result.value().statistics.edges, tube ? 4 : 2);
      EXPECT_EQ(result.value().statistics.vertices, tube ? 4 : 2);
      EXPECT_EQ(result.value().statistics.solids, 1);
      EXPECT_NE(result.value().text.find("=CYLINDRICAL_SURFACE("),
                std::string::npos);
      CheckReferences(result.value());
    }
  }
}
TEST(OpenSwStep, ExportsFullCylinderMantlesAndClosedSheets) {
  for (const bool mantle_only : {false, true}) {
    const auto result =
        Export(test::Cylinder(false, false, false, mantle_only));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, mantle_only ? 1 : 3);
    EXPECT_EQ(result.value().statistics.solids, 0);
    EXPECT_NE(result.value().text.find(mantle_only ? "=OPEN_SHELL("
                                                   : "=CLOSED_SHELL("),
              std::string::npos);
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, HandlesCylinderLoopOrderAxisAndNegativeSenses) {
  auto reversed_loops = test::Cylinder();
  test::Set(reversed_loops, 302, "loop", 605);
  test::Set(reversed_loops, 605, "next", 604);
  test::Set(reversed_loops, 604, "next", 0);
  auto negative_senses = test::Cylinder();
  test::Set(negative_senses, 302, "sense", '-');
  test::Set(negative_senses, 702, "sense", '-');
  auto negative_axis = test::Cylinder();
  for (auto& node : negative_axis.report.nodes)
    if (node.index == 702) node.fields[1].values = {0., 0., -1.};
  for (const auto& fixture : {reversed_loops, negative_senses, negative_axis}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
  }
}
TEST(OpenSwStep, ChecksCompleteCylinderCutsBeyondBindingSamples) {
  auto tilted = test::Cylinder(false, false, false, true);
  constexpr double angle = 1e-6;
  for (auto& node : tilted.report.nodes)
    if (node.index == 4001)
      node.fields[1].values = {0., std::sin(angle), std::cos(angle)};
  const auto result = Export(tilted);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("tilted"), std::string::npos);
  for (const double height : {0., 1e-8, 1e-5}) {
    auto short_band = test::Cylinder(false, false, false, true);
    for (auto& node : short_band.report.nodes)
      if (node.index == 4001) node.fields[0].values = {0., 0., height};
    const auto band = Export(short_band);
    EXPECT_EQ(band.ok(), height > 2 * Options().tolerance);
    if (!band.ok()) EXPECT_EQ(band.error().code, ErrorCode::kInvalidRecord);
  }
  auto wrong_winding = test::Cylinder(false, false, false, true);
  test::Set(wrong_winding, 4001, "sense", '-');
  const auto winding = Export(wrong_winding);
  ASSERT_FALSE(winding.ok());
  EXPECT_NE(winding.error().message.find("winding"), std::string::npos);
}
TEST(OpenSwStep, BoundsCylinderAndCapComparisonsAcrossTheBody) {
  auto options = Options();
  // Two annular cap containment checks plus one cut-pair check per mantle.
  options.max_polygon_checks = 4;
  const auto exact = Export(test::Cylinder(true), options);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  options.max_polygon_checks = 3;
  const auto limited = Export(test::Cylinder(true), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsInvertedCurvedSolidsUsingAnalyticVolume) {
  auto inverted = test::Cylinder(true);
  for (uint32_t i = 0; i < 4; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, ParsesCylindricalSolidsThroughTheBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::Cylinder(true, true));
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 4);
  EXPECT_EQ(result.value().statistics.edges, 4);
  EXPECT_EQ(result.value().statistics.solids, 1);
  CheckReferences(result.value());
}
TEST(OpenSwStep, PreservesDistinctSeamPhasesOnCylinderCuts) {
  for (const bool named_seams : {false, true}) {
    const auto result = Export(test::ShiftedCylinderSeams(named_seams));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.vertices, 2);
    EXPECT_EQ(result.value().statistics.edges, 2);
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, ExportsExactExpandingAndNarrowingConicalFrustums) {
  for (const bool narrowing : {false, true})
    for (const bool named_seams : {false, true}) {
      const auto result = Export(test::Frustum(narrowing, false, named_seams));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.solids, 1);
      EXPECT_EQ(result.value().statistics.faces, 3);
      EXPECT_EQ(result.value().statistics.edges, 2);
      EXPECT_NE(result.value().text.find("=CONICAL_SURFACE("),
                std::string::npos);
      CheckReferences(result.value());
    }
  const auto tube = Export(test::Frustum(false, true, true));
  ASSERT_TRUE(tube.ok()) << tube.error().message;
  EXPECT_EQ(tube.value().statistics.faces, 4);
  EXPECT_EQ(tube.value().statistics.edges, 4);
}
TEST(OpenSwStep, HandlesConicalSheetsAndReversedAxesAndLoops) {
  for (const auto& f : {test::Frustum(false, false, false, false, true),
                        test::ReversedFrustum(), test::ReversedFrustum(true),
                        test::FrustumAtApexOrigin(),
                        test::RotateAndTranslate(test::Frustum())}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, RejectsConicalApexCutsAndInvalidProfiles) {
  for (const double radius : {0., 1e-8, 4.}) {
    auto f = test::Frustum(false, false, false, false, true);
    for (auto& node : f.report.nodes)
      if (node.index == 4001) node.fields[3].values = {radius};
    EXPECT_FALSE(Export(f).ok());
  }
  auto tilted = test::Frustum(false, false, false, false, true);
  for (auto& node : tilted.report.nodes)
    if (node.index == 4001)
      node.fields[1].values = {0., std::sin(1e-6), std::cos(1e-6)};
  const auto result = Export(tilted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("tilted"), std::string::npos);
}
TEST(OpenSwStep, BoundsConicalBandsAndRejectsInvertedConicalSolids) {
  auto options = Options();
  options.max_polygon_checks = 4;
  ASSERT_TRUE(Export(test::Frustum(false, true), options).ok());
  options.max_polygon_checks = 3;
  const auto limited = Export(test::Frustum(false, true), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  auto inverted = test::Frustum(false, true);
  for (uint32_t i = 0; i < 4; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto negative = Export(inverted);
  ASSERT_FALSE(negative.ok());
  EXPECT_NE(negative.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, ParsesConicalSolidsThroughTheBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::Frustum(false, true, true));
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
  EXPECT_EQ(result.value().statistics.faces, 4);
}
TEST(OpenSwStep, ExportsExactToroidalBandsAndClosedSolids) {
  for (const bool named_seams : {false, true}) {
    const auto result = Export(test::ToroidalBand(true, named_seams));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 3);
    EXPECT_EQ(result.value().statistics.edges, 2);
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_NE(result.value().text.find("=TOROIDAL_SURFACE("),
              std::string::npos);
    CheckReferences(result.value());
  }
  for (const bool complement : {false, true}) {
    const auto result = Export(test::ToroidalBand(false, false, complement));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_EQ(result.value().statistics.solids, 0);
  }
}
TEST(OpenSwStep, ExportsClosedTolerantRingsWithSourceOrSyntheticSeams) {
  for (const bool exterior : {false, true})
    for (const bool named : {false, true}) {
      const auto fixture = test::ClosedTolerantToroidalBand(exterior, named);
      const auto result = Export(fixture);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.faces, 3);
      EXPECT_EQ(result.value().statistics.edges, 2);
      EXPECT_EQ(result.value().statistics.vertices, 2);
      EXPECT_EQ(result.value().statistics.solids, 1);
      EXPECT_DOUBLE_EQ(result.value().statistics.length_uncertainty, .001);
      CheckReferences(result.value());
      const auto serialized = test::Serialize(fixture);
      const auto binary = ExportParasolid(serialized.bytes, NodeId{1},
                                          Options(), &serialized.catalog);
      ASSERT_TRUE(binary.ok()) << binary.error().message;
      EXPECT_EQ(binary.value().text, result.value().text);
    }
  for (const bool profile : {false, true})
    for (const bool axis : {false, true}) {
      const auto fixture = test::RotateAndTranslate(
          test::ClosedTolerantToroidalBand(true, true, profile, axis));
      const auto result = Export(fixture);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.solids, 1);
    }
}
TEST(OpenSwStep, PreservesClosedTolerantRingSeamPhasesAcrossParameterTurns) {
  for (const double phase : {std::numbers::pi / 2, 3 * std::numbers::pi / 2,
                             -std::numbers::pi / 3, 8 * std::numbers::pi + .4})
    for (const bool named : {false, true}) {
      const auto result = Export(
          test::ClosedTolerantToroidalBand(true, named, false, false, phase));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.edges, 2);
      EXPECT_EQ(result.value().statistics.vertices, 2);
      EXPECT_EQ(result.value().statistics.solids, 1);
    }
}
TEST(OpenSwStep, RecoversRingIntervalsOnlyAfterTheCompletePairedCertificate) {
  const auto model =
      ModelBuilder::Build(test::ClosedTolerantToroidalBand().report);
  ASSERT_TRUE(model.ok());
  const auto direct = BindBodyGeometry(model.value(), NodeId{1}, 1e-7);
  ASSERT_FALSE(direct.ok());
  EXPECT_EQ(direct.error().code, ErrorCode::kUnsupportedFormat);
  auto options = Options();
  constexpr uint64_t kBudget = 100000;
  uint64_t remaining = kBudget;
  const auto inferred = internal::ResolveClosedTolerantRingIntervals(
      model.value(), NodeId{1}, {}, options.tolerance, true, options.binding,
      remaining);
  ASSERT_TRUE(inferred.ok()) << inferred.error().message;
  ASSERT_EQ(inferred.value().size(), 4);
  const auto used = kBudget - remaining;
  EXPECT_GT(used, 0);
  const auto bound = BindBodyGeometryWithOverrides(model.value(), NodeId{1},
                                                   inferred.value(), {}, 1e-7);
  ASSERT_TRUE(bound.ok()) << bound.error().message;
  // Automatic recovery charges both its preflight and its final certificate,
  // plus the torus cut-pair check. Explicit intervals still need final proof.
  options.max_polygon_checks = 2 * used + 1;
  auto exported = ExportBody(model.value(), NodeId{1}, options);
  ASSERT_TRUE(exported.ok()) << exported.error().message;
  --options.max_polygon_checks;
  exported = ExportBody(model.value(), NodeId{1}, options);
  ASSERT_FALSE(exported.ok());
  EXPECT_EQ(exported.error().code, ErrorCode::kResourceLimit);
  options.max_polygon_checks = used + 1;
  options.fin_intervals = inferred.value();
  exported = ExportBody(model.value(), NodeId{1}, options);
  ASSERT_TRUE(exported.ok()) << exported.error().message;
  remaining = kBudget;
  const auto partial = internal::ResolveClosedTolerantRingIntervals(
      model.value(), NodeId{1}, std::span(inferred.value()).first(1), 1e-7,
      true, options.binding, remaining);
  ASSERT_TRUE(partial.ok()) << partial.error().message;
  EXPECT_EQ(partial.value().size(), 3);
  for (const auto& interval : partial.value())
    EXPECT_NE(interval.fin, inferred.value().front().fin);
  EXPECT_FALSE(BindBodyGeometry(model.value(), NodeId{1}, 1e-7).ok());
}
TEST(OpenSwStep, RestrictsPeriodicRingKnotsWithoutChangingSourcePolynomials) {
  const auto fixture = test::ClosedTolerantToroidalBand();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto options = Options();
  uint64_t remaining = 100000;
  const auto inferred = internal::ResolveClosedTolerantRingIntervals(
      model.value(), NodeId{1}, {}, 1e-7, true, options.binding, remaining);
  ASSERT_TRUE(inferred.ok()) << inferred.error().message;
  const auto bound = BindBodyGeometryWithOverrides(model.value(), NodeId{1},
                                                   inferred.value(), {}, 1e-7);
  ASSERT_TRUE(bound.ok());
  const auto recovered = internal::RecoverTolerantCircles(
      model.value(), bound.value(), 1e-7, true, remaining);
  ASSERT_TRUE(recovered.ok()) << recovered.error().message;
  ASSERT_EQ(recovered.value().size(), 2);
  unsigned checked = 0;
  for (const auto& edge : recovered.value())
    for (const auto& use : edge.uses) {
      if (use.pcurve.axis.degree != 2) continue;
      const auto spline_id = test::PointerValue(
          fixture, 10000 + 10 * use.fin.value + 6, "b_curve");
      const auto* object = model.value().FindNode(NodeId{spline_id});
      ASSERT_NE(object, nullptr);
      ASSERT_TRUE(object->geometry.has_value());
      const auto& original = std::get<NurbsCurveGeometry>(
          model.value().geometry()[*object->geometry].payload);
      ASSERT_TRUE(original.axis.periodic);
      ASSERT_TRUE(original.axis.closed);
      EXPECT_EQ(original.controls.values, use.pcurve.controls.values);
      EXPECT_EQ(original.axis.knots.size(), use.pcurve.axis.knots.size() + 2);
      EXPECT_FALSE(use.pcurve.axis.periodic);
      EXPECT_TRUE(use.pcurve.axis.closed);
      for (double parameter : {0., .001, .11, .25, .57, .999, 1.}) {
        const auto a = EvaluateCurve(GeometryPayload{original}, parameter);
        const auto b = EvaluateCurve(GeometryPayload{use.pcurve}, parameter);
        ASSERT_TRUE(a.ok());
        ASSERT_TRUE(b.ok());
        EXPECT_NEAR(a.value().position.x, b.value().position.x, 1e-12);
        EXPECT_NEAR(a.value().position.y, b.value().position.y, 1e-12);
        EXPECT_NEAR(a.value().first_derivative.x, b.value().first_derivative.x,
                    1e-10);
        EXPECT_NEAR(a.value().first_derivative.y, b.value().first_derivative.y,
                    1e-10);
      }
      ++checked;
    }
  EXPECT_EQ(checked, 2);
}
TEST(OpenSwStep, RejectsClosedTolerantRingTurnsSeamsAndContradictoryOverrides) {
  auto invalid = test::ClosedTolerantToroidalBand();
  const auto trim_id = test::PointerValue(invalid, 1000, "curve");
  for (auto& node : invalid.report.nodes)
    if (node.index == trim_id)
      for (auto& field : node.fields)
        if (invalid.report.schemas[node.schema_index]
                .definition.fields[field.definition_index]
                .name == "parm_2")
          field.values = {.5};
  auto result = Export(invalid);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(result.error().message.find("Stored closed-ring trim endpoint"),
            std::string::npos);
  auto multiple = test::ClosedTolerantToroidalBand();
  const uint32_t controls_id = 10000 + 10 * 1000 + 3;
  for (auto& node : multiple.report.nodes)
    if (node.index == controls_id)
      node.fields[0].values[3] = 4 * std::numbers::pi;
  result = Export(multiple);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("resolved arc or ring turn"),
            std::string::npos);
  auto options = Options();
  options.use_source_edge_tolerances = false;
  result = Export(test::ClosedTolerantToroidalBand(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  options = Options();
  options.fin_intervals = {{NodeId{1000}, 1, 0}};
  result = Export(test::ClosedTolerantToroidalBand(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  options.fin_intervals = {{NodeId{1000}, 0, 1}, {NodeId{1000}, 0, 1}};
  result = Export(test::ClosedTolerantToroidalBand(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidOptions);
}
TEST(OpenSwStep, ExportsCertifiedTolerantCircularEdgesWithSourcePcurves) {
  const auto fixture = test::TolerantToroidalBand();
  const auto result = Export(fixture);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 3);
  EXPECT_EQ(result.value().statistics.edges, 8);
  EXPECT_EQ(result.value().statistics.vertices, 8);
  EXPECT_EQ(result.value().statistics.solids, 1);
  EXPECT_DOUBLE_EQ(result.value().statistics.length_uncertainty, .001);
  EXPECT_NE(result.value().text.find("=SURFACE_CURVE("), std::string::npos);
  EXPECT_NE(result.value().text.find("=PCURVE("), std::string::npos);
  EXPECT_NE(result.value().text.find("=B_SPLINE_CURVE_WITH_KNOTS("),
            std::string::npos);
  for (const auto& [type, count] : std::vector<std::pair<std::string, int>>{
           {"=SURFACE_CURVE", 8},
           {"=PCURVE", 16},
           {"=B_SPLINE_CURVE_WITH_KNOTS", 16}}) {
    std::size_t position = 0;
    int found = 0;
    while ((position = result.value().text.find(type, position)) !=
           std::string::npos) {
      ++found;
      ++position;
    }
    EXPECT_EQ(found, count) << type;
  }
  EXPECT_NE(result.value().text.find("LENGTH_MEASURE(0.001)"),
            std::string::npos);
  CheckReferences(result.value());
  const auto serialized = test::Serialize(fixture);
  const auto parsed = ExportParasolid(serialized.bytes, NodeId{1}, Options(),
                                      &serialized.catalog);
  ASSERT_TRUE(parsed.ok()) << parsed.error().message;
  EXPECT_EQ(parsed.value().text, result.value().text);
}
TEST(OpenSwStep,
     RetainsTolerantEdgeOrientationUnderReversedAndTranslatedFrames) {
  for (const bool profile : {false, true})
    for (const bool axis : {false, true}) {
      const auto fixture = test::TolerantToroidalBand(.001, profile, axis);
      for (const auto& input : {fixture, test::RotateAndTranslate(fixture)}) {
        const auto result = Export(input);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().statistics.solids, 1);
        EXPECT_EQ(result.value().statistics.edges, 8);
        EXPECT_DOUBLE_EQ(result.value().statistics.length_uncertainty, .001);
      }
    }
}
TEST(OpenSwStep, RejectsUnmappedTolerantBranchesAndInvalidVertexAgreement) {
  const auto fixture = test::TolerantToroidalBand();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto bound = BindBodyGeometry(model.value(), NodeId{1}, 1e-7);
  ASSERT_TRUE(bound.ok());
  auto incomplete = bound.value();
  incomplete.faces.erase(incomplete.faces.begin());
  uint64_t remaining = 10000;
  const auto partial = internal::RecoverTolerantCircles(
      model.value(), incomplete, 1e-7, true, remaining);
  ASSERT_FALSE(partial.ok());
  EXPECT_EQ(partial.error().code, ErrorCode::kUnsupportedFormat);
  auto moved = bound.value();
  moved.vertices.front().position.x += .01;
  remaining = 10000;
  const auto displaced = internal::RecoverTolerantCircles(
      model.value(), moved, 1e-7, true, remaining);
  ASSERT_FALSE(displaced.ok());
  EXPECT_NE(displaced.error().message.find("vertex tolerance"),
            std::string::npos);
  auto truncated = bound.value();
  truncated.faces.front().loops.front().fins.front().end = .5;
  remaining = 10000;
  const auto domain = internal::RecoverTolerantCircles(model.value(), truncated,
                                                       1e-7, true, remaining);
  ASSERT_FALSE(domain.ok());
  EXPECT_NE(domain.error().message.find("full clamped spline domain"),
            std::string::npos);
}
TEST(OpenSwStep, EnforcesSourceTolerancePolicyAndWholeSplineCertificate) {
  auto options = Options();
  options.use_source_edge_tolerances = false;
  auto result = Export(test::TolerantToroidalBand(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  options.tolerance = .001;
  result = Export(test::TolerantToroidalBand(), options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  result = Export(test::TolerantToroidalBand(1e-6));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  // Tolerance violations occur inside the Bezier spans: merely matching the
  // source endpoints and midpoints must not certify the complete edge.
  EXPECT_NE(result.error().message.find("exceeds the circular edge tolerance"),
            std::string::npos);
  options = Options();
  options.max_polygon_checks = 1;
  result = Export(test::TolerantToroidalBand(), options);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kResourceLimit);
  result = Export(test::TolerantToroidalBand(-1));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
}
TEST(OpenSwStep, ExportsCircularSpunSurfacesWithCorrectNormals) {
  for (bool reverse_profile : {false, true})
    for (bool reverse_axis : {false, true}) {
      for (const auto& fixture :
           {test::SpunToroidalBand(test::ToroidalBand(), reverse_profile,
                                   reverse_axis),
            test::SpunToroidalBand(test::ToroidalBand(true, true),
                                   reverse_profile, reverse_axis),
            test::SpunToroidalBand(test::InwardToroidalBand(), reverse_profile,
                                   reverse_axis),
            test::RotateAndTranslate(test::SpunToroidalBand(
                test::ToroidalBand(), reverse_profile, reverse_axis))}) {
        const auto result = Export(fixture);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_NE(result.value().text.find("=TOROIDAL_SURFACE("),
                  std::string::npos);
        EXPECT_EQ(result.value().statistics.edges, 2);
        CheckReferences(result.value());
      }
    }
  const auto fixture = test::Serialize(test::SpunToroidalBand());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, RecognizesSpunTorusFramesIndependentlyOfParameterPhase) {
  const auto canonical_model = ModelBuilder::Build(test::ToroidalBand().report);
  ASSERT_TRUE(canonical_model.ok());
  for (const bool reversed : {false, true}) {
    const auto fixture = test::SpunToroidalBand(test::ToroidalBand(), reversed);
    const auto model = ModelBuilder::Build(fixture.report);
    ASSERT_TRUE(model.ok());
    const auto& objects = model.value().geometry();
    const SpunSurfaceGeometry* spun = nullptr;
    const CircleGeometry* profile = nullptr;
    for (const auto& object : objects) {
      if (object.source.value == 702)
        spun = std::get_if<SpunSurfaceGeometry>(&object.payload);
      if (object.source.value == 9000)
        profile = std::get_if<CircleGeometry>(&object.payload);
    }
    ASSERT_NE(spun, nullptr);
    ASSERT_NE(profile, nullptr);
    const auto result = internal::RecognizeSpunTorus(*spun, *profile, 1e-7);
    ASSERT_TRUE(result.ok()) << result.error().message;
    const auto& torus = result.value().geometry;
    EXPECT_EQ(result.value().same_normal, !reversed);
    EXPECT_DOUBLE_EQ(torus.major_radius, 3);
    for (double u : {0., .7, 2.1, 4.8})
      for (double v : {0., .6, 3.2}) {
        const auto sample = EvaluateSurface(model.value(), NodeId{702}, u, v);
        ASSERT_TRUE(sample.ok());
        const auto inverse = InvertSurface(GeometryPayload{torus},
                                           sample.value().position, 1e-10);
        ASSERT_TRUE(inverse.ok()) << inverse.error().message;
        const auto n = EvaluateSurfaceNormal(model.value(), NodeId{702}, u, v);
        ASSERT_TRUE(n.ok());
        const auto canonical =
            EvaluateSurfaceNormal(canonical_model.value(), NodeId{702},
                                  inverse.value().u, inverse.value().v);
        ASSERT_TRUE(canonical.ok());
        EXPECT_NEAR(n.value().x, canonical.value().x, 1e-12);
        EXPECT_NEAR(n.value().y, canonical.value().y, 1e-12);
        EXPECT_NEAR(n.value().z, canonical.value().z, 1e-12);
      }
  }
}
TEST(OpenSwStep, RejectsNonmeridionalAndBoundedSpunProfiles) {
  SpunSurfaceGeometry spun;
  spun.axis = {0, 0, 1};
  CircleGeometry circle{{3, 0, 0}, {0, 1, 0}, {1, 0, 0}, 1};
  const auto require_unsupported = [&](const SpunSurfaceGeometry& s,
                                       const CircleGeometry& c) {
    const auto result = internal::RecognizeSpunTorus(s, c, 1e-7);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  };
  spun.start_parameter = 0.;
  require_unsupported(spun, circle);
  spun.start_parameter.reset();
  spun.end_parameter = 7.;
  require_unsupported(spun, circle);
  spun.end_parameter.reset();
  auto bad = circle;
  bad.normal = {0, 0, 1};
  require_unsupported(spun, bad);
  bad = circle;
  bad.center = {3, 1, 0};
  require_unsupported(spun, bad);
  for (double radius : {3., 4.}) {
    bad = circle;
    bad.radius = radius;
    require_unsupported(spun, bad);
  }
  auto axis = spun;
  axis.axis.z = 2;
  EXPECT_FALSE(internal::RecognizeSpunTorus(axis, circle, 1e-7).ok());
  EXPECT_FALSE(internal::RecognizeSpunTorus(spun, circle, 0).ok());
}
TEST(OpenSwStep, PreservesSplitCircularTorusCutsAndSourceTopology) {
  for (const auto& fixture : {test::SplitToroidalBand(),
                              test::SpunToroidalBand(test::SplitToroidalBand()),
                              test::RotateAndTranslate(test::SpunToroidalBand(
                                  test::SplitToroidalBand()))}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 3);
    EXPECT_EQ(result.value().statistics.edges, 8);
    EXPECT_EQ(result.value().statistics.vertices, 8);
    EXPECT_EQ(result.value().statistics.solids, 1);
    CheckReferences(result.value());
  }
  auto options = Options();
  options.max_polygon_checks = 17;
  EXPECT_TRUE(Export(test::SplitToroidalBand(), options).ok());
  options.max_polygon_checks = 16;
  const auto limited = Export(test::SplitToroidalBand(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsUncertifiedSplitCircularCuts) {
  const double pi = std::numbers::pi;
  std::vector<internal::ConicBoundary> arcs{
      {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}, 3, 3, 0, pi, true},
      {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}, 3, 3, pi, 2 * pi, true}};
  uint64_t remaining = 2;
  auto result = internal::CombineCircularCut(arcs, 1e-7, remaining);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(remaining, 0);
  auto invalid = arcs;
  invalid[1].end = 0;
  remaining = 2;
  EXPECT_FALSE(internal::CombineCircularCut(invalid, 1e-7, remaining).ok());
  invalid = arcs;
  invalid[1].major_radius += 1e-5;
  remaining = 2;
  EXPECT_FALSE(internal::CombineCircularCut(invalid, 1e-7, remaining).ok());
  invalid = arcs;
  invalid[1].end = 1.5 * pi;
  remaining = 2;
  EXPECT_FALSE(internal::CombineCircularCut(invalid, 1e-7, remaining).ok());
  // Reversing a circle's normal and interval preserves its geometric direction.
  invalid = arcs;
  invalid[1].normal.z = -1;
  invalid[1].start = pi;
  invalid[1].end = 0;
  remaining = 2;
  EXPECT_TRUE(internal::CombineCircularCut(invalid, 1e-7, remaining).ok());
}
TEST(OpenSwStep, HandlesToroidalCutOrderAndReversedAxes) {
  for (const auto& f :
       {test::ReversedToroidalBand(), test::ReversedToroidalBand(true),
        test::RotateAndTranslate(test::ToroidalBand())}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, ParsesToroidalSolidsThroughTheBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::ToroidalBand(true, true));
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, ChecksToroidalCutsWindingAndRegularRingRestrictions) {
  auto tilted = test::ToroidalBand(false);
  for (auto& node : tilted.report.nodes)
    if (node.index == 4001)
      node.fields[1].values = {0., std::sin(1e-6), std::cos(1e-6)};
  const auto tilt = Export(tilted);
  ASSERT_FALSE(tilt.ok());
  EXPECT_NE(tilt.error().message.find("tilted"), std::string::npos);
  auto coincident = test::ToroidalBand(false);
  for (auto& node : coincident.report.nodes)
    if (node.index == 4001) {
      node.fields[0].values = {0., 0., 0.};
      node.fields[3].values = {4.};
    }
  const auto cuts = Export(coincident);
  ASSERT_FALSE(cuts.ok());
  EXPECT_NE(cuts.error().message.find("coincide"), std::string::npos);
  auto winding = test::ToroidalBand(false);
  test::Set(winding, 4001, "sense", '-');
  const auto direction = Export(winding);
  ASSERT_FALSE(direction.ok());
  EXPECT_NE(direction.error().message.find("winding"), std::string::npos);
  for (const double major : {0.5, 1.}) {
    auto nonring = test::ToroidalBand(false);
    for (auto& node : nonring.report.nodes) {
      if (node.index == 702) node.fields[2].values = {major};
      if (node.index == 4000) node.fields[3].values = {major + 1};
      if (node.index == 4001) node.fields[3].values = {major};
    }
    const auto result = Export(nonring);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_NE(result.error().message.find("regular ring"), std::string::npos);
  }
}
TEST(OpenSwStep, BoundsToroidalCutComparisonsAcrossMixedSupportedFaces) {
  auto options = Options();
  options.max_polygon_checks = 4;
  const auto exact = Export(test::ToroidalTube(), options);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  EXPECT_EQ(exact.value().statistics.faces, 4);
  options.max_polygon_checks = 3;
  const auto limited = Export(test::ToroidalTube(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsInvertedToroidalSolidsUsingAnalyticVolume) {
  auto inverted = test::ToroidalBand();
  for (uint32_t i = 0; i < 3; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
  for (const auto& f :
       {test::InwardToroidalBand(), test::ToroidalProfileBand(false),
        test::ToroidalProfileBand(true)}) {
    const auto sheet = Export(f);
    ASSERT_TRUE(sheet.ok()) << sheet.error().message;
  }
}
TEST(OpenSwStep, ExportsExactAnnularSectorsAndClosedRingSectors) {
  for (const double span :
       {std::numbers::pi / 2, std::numbers::pi, 1.5 * std::numbers::pi})
    for (const bool inward : {false, true}) {
      const auto result = Export(test::AnnularSector(span, inward));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.faces, 1);
      EXPECT_EQ(result.value().statistics.edges, 4);
      CheckReferences(result.value());
    }
  for (const auto& fixture :
       {test::Stadium(), test::AnnularSectorSolid(),
        test::AnnularSectorSolid(1.5 * std::numbers::pi),
        test::PhaseShiftedAnnularSector(),
        test::RotateAndTranslate(test::AnnularSector()),
        test::RotateAndTranslate(test::AnnularSectorSolid())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
  }
}
TEST(OpenSwStep, BoundsAnnularChecksAndRejectsUnimplementedContours) {
  auto options = Options();
  options.max_polygon_checks = 5;
  ASSERT_TRUE(Export(test::AnnularSector(), options).ok());
  options.max_polygon_checks = 4;
  // Two nonconcentric, positively curved stadium arcs retain convex support.
  ASSERT_TRUE(Export(test::Stadium(), options).ok());
  const auto limited = Export(test::AnnularSector(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto offset =
      Export(test::AnnularSector(std::numbers::pi / 2, false, 0, .2));
  ASSERT_FALSE(offset.ok());
  EXPECT_EQ(offset.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(offset.error().message.find("concentric"), std::string::npos);
  const auto twisted =
      Export(test::AnnularSector(std::numbers::pi / 2, false, .1));
  ASSERT_FALSE(twisted.ok());
  EXPECT_EQ(twisted.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(twisted.error().message.find("radial"), std::string::npos);
  EXPECT_FALSE(Export(test::ConcaveArcFace()).ok());
  EXPECT_FALSE(Export(test::AnnularSector(1.5e-7)).ok());
  EXPECT_FALSE(
      Export(test::AnnularSector(std::numbers::pi / 2, false, 0, 0, 3 - 1e-7))
          .ok());
}
TEST(OpenSwStep, PreservesAnnularSourceSenseAndRejectsInvertedSolids) {
  auto negative = test::AnnularSector();
  test::Set(negative, 300, "sense", '-');
  for (auto& node : negative.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(negative).ok());
  auto inverted = test::AnnularSectorSolid();
  for (uint32_t i = 0; i < 6; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, IntegratesAnnularSourceCurveAreasInTranslatedFrames) {
  for (const double span : {std::numbers::pi / 2, 1.5 * std::numbers::pi})
    for (const Vector3 shift : {Vector3{}, Vector3{1e6, -2e6, 3e6}}) {
      const auto c = std::cos(span), s = std::sin(span);
      const std::array<GeometryPayload, 4> geometry{
          CircleGeometry{shift, {0, 0, 1}, {1, 0, 0}, 3},
          LineGeometry{{shift.x + 3 * c, shift.y + 3 * s, shift.z},
                       {-c, -s, 0}},
          CircleGeometry{shift, {0, 0, 1}, {1, 0, 0}, 1},
          LineGeometry{{shift.x + 1, shift.y, shift.z}, {1, 0, 0}}};
      const std::array<internal::PlanarCurve, 4> curves{
          {{&geometry[0], 0, span},
           {&geometry[1], 0, 2},
           {&geometry[2], span, 0},
           {&geometry[3], 0, 2}}};
      uint64_t remaining = 5;
      const auto result = internal::ValidateAnnularSector(
          curves, {shift, {0, 0, -1}, {1, 0, 0}}, false, 1e-7, remaining);
      ASSERT_TRUE(result.ok()) << result.error().message;
      // Independent sector formula: (outer_radius^2-inner_radius^2)*span/2.
      EXPECT_NEAR(result.value().area, 4 * span, 1e-9);
      EXPECT_EQ(remaining, 0);
    }
}
TEST(OpenSwStep, ParsesAnnularSectorsThroughBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::AnnularSectorSolid());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
  EXPECT_EQ(result.value().statistics.faces, 6);
}
TEST(OpenSwStep, ExportsObliqueAndSteppedCylinderBandsAsCompleteBodies) {
  for (const bool seams : {false, true})
    for (const auto slopes : {std::pair{.2, 0.}, std::pair{-.2, .15}}) {
      const auto f = test::ObliqueCylinder(slopes.first, slopes.second, seams);
      for (const auto& source : {f, test::RotateAndTranslate(f)}) {
        const auto result = Export(source);
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().statistics.solids, 1);
        EXPECT_EQ(result.value().statistics.faces, 3);
        EXPECT_EQ(result.value().statistics.edges, 2);
      }
    }
  for (const auto& f :
       {test::SteppedCylinderBand(),
        test::RotateAndTranslate(test::SteppedCylinderBand())}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_EQ(result.value().statistics.faces, 5);
    EXPECT_EQ(result.value().statistics.edges, 12);
    EXPECT_EQ(result.value().statistics.vertices, 10);
    CheckReferences(result.value());
  }
  const auto bytes = test::Serialize(test::SteppedCylinderBand());
  const auto parsed =
      ExportParasolid(bytes.bytes, NodeId{1}, Options(), &bytes.catalog);
  ASSERT_TRUE(parsed.ok()) << parsed.error().message;
  EXPECT_EQ(parsed.value().statistics.solids, 1);
}
TEST(OpenSwStep, IntegratesProfiledCylinderBandsWithTransverseOriginTerms) {
  const auto fixture = test::SteppedCylinderBand();
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const auto binding =
      BindBodyGeometry(model.value(), NodeId{1}, Options().tolerance);
  ASSERT_TRUE(binding.ok()) << binding.error().message;
  std::vector<std::vector<internal::PlanarCurve>> loops;
  for (const auto& face : binding.value().faces)
    if (face.face.value == 304)
      for (const auto& loop : face.loops) {
        std::vector<internal::PlanarCurve> curves;
        for (const auto& fin : loop.fins) {
          const auto* node = model.value().FindNode(fin.curve);
          curves.push_back({&model.value().geometry()[*node->geometry].payload,
                            fin.start, fin.end});
        }
        loops.push_back(std::move(curves));
      }
  CylinderGeometry cylinder{{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}};
  uint64_t remaining = 11;
  const auto band =
      internal::ValidateCylinderBand(loops, cylinder, true, 1e-7, remaining);
  ASSERT_TRUE(band.ok()) << band.error().message;
  EXPECT_EQ(remaining, 0);
  EXPECT_NEAR(band.value().height_integral, 5.5 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(band.value().radial_integral[0], -.2 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(band.value().radial_integral[1], 1, 1e-12);
  EXPECT_NEAR(internal::CylinderBandVolume6(cylinder, band.value(), {}),
              11 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(internal::CylinderBandVolume6(cylinder, band.value(), {1, 2, 3}),
              11.4 * std::numbers::pi - 4, 1e-12);
  remaining = 10;
  const auto limited =
      internal::ValidateCylinderBand(loops, cylinder, true, 1e-7, remaining);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, PreservesProfiledCylinderAxisSenseAndRejectsAmbiguousRanges) {
  auto reverse = test::ObliqueCylinder();
  for (auto& node : reverse.report.nodes)
    if (node.index == 702) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(reverse).ok());
  auto inward = test::ObliqueCylinder(.2, 0, false, true);
  test::Set(inward, 302, "sense", '-');
  for (const auto ids : {std::pair{1000u, 1001u}, std::pair{1003u, 1002u}}) {
    const auto loop = test::PointerValue(inward, ids.first, "loop");
    test::Set(inward, ids.first, "loop", 0);
    test::Set(inward, ids.first, "forward", 0);
    test::Set(inward, ids.first, "backward", 0);
    test::Set(inward, ids.second, "loop", loop);
    test::Set(inward, ids.second, "forward", ids.second);
    test::Set(inward, ids.second, "backward", ids.second);
    test::Set(inward, loop, "halfedge", ids.second);
  }
  const auto sheet = Export(inward);
  ASSERT_TRUE(sheet.ok()) << sheet.error().message;
  EXPECT_EQ(sheet.value().statistics.solids, 0);
  const auto uncertified = Export(test::ObliqueCylinder(.6, .6));
  ASSERT_FALSE(uncertified.ok());
  EXPECT_EQ(uncertified.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(uncertified.error().message.find("axial ranges overlap"),
            std::string::npos);
  auto options = Options();
  options.max_polygon_checks = 3;
  ASSERT_TRUE(Export(test::ObliqueCylinder(), options).ok());
  options.max_polygon_checks = 2;
  const auto limited = Export(test::ObliqueCylinder(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsNoncylindricalProfilesBeforeEmittingPartialOutput) {
  const GeometryPayload bad =
      EllipseGeometry{{0, 0, 0}, {0, 0, 1}, {1, 0, 0}, 2, 1};
  const GeometryPayload top =
      CircleGeometry{{0, 0, 3}, {0, 0, 1}, {1, 0, 0}, 1};
  std::vector<std::vector<internal::PlanarCurve>> loops{
      {{&bad, 0, 2 * std::numbers::pi}}, {{&top, 2 * std::numbers::pi, 0}}};
  uint64_t remaining = 10;
  const auto wrong = internal::ValidateCylinderBand(
      loops, {{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}}, true, 1e-7, remaining);
  ASSERT_FALSE(wrong.ok());
  EXPECT_EQ(wrong.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(wrong.error().message.find("project to the cylinder circle"),
            std::string::npos);
  const GeometryPayload foreign = IntersectionCurveGeometry{};
  loops[0][0] = {&foreign, 0, 1};
  remaining = 10;
  const auto unsupported = internal::ValidateCylinderBand(
      loops, {{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}}, true, 1e-7, remaining);
  ASSERT_FALSE(unsupported.ok());
  EXPECT_EQ(unsupported.error().code, ErrorCode::kUnsupportedFormat);
  loops[0] = {{&top, 0, std::numbers::pi}, {&top, std::numbers::pi, 0}};
  remaining = 10;
  const auto fold = internal::ValidateCylinderBand(
      loops, {{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}}, true, 1e-7, remaining);
  ASSERT_FALSE(fold.ok());
  EXPECT_NE(fold.error().message.find("not monotone"), std::string::npos);
}
TEST(OpenSwStep, ExportsCertifiedConcaveMinorArcContoursWithExactSourceArea) {
  for (const auto& f :
       {test::ConcaveMinorArcFace(), test::ConcaveMinorArcFace(1.79),
        test::RotateAndTranslate(test::ConcaveMinorArcFace())}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_EQ(result.value().statistics.edges, 8);
    EXPECT_NE(result.value().text.find("CIRCLE("), std::string::npos);
  }
  auto options = Options();
  options.max_polygon_checks = 48;
  ASSERT_TRUE(Export(test::ConcaveMinorArcFace(), options).ok());
  options.max_polygon_checks = 47;
  const auto limited = Export(test::ConcaveMinorArcFace(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  for (const double bottom : {1.8, 1.95}) {
    const auto rejected = Export(test::ConcaveMinorArcFace(bottom));
    ASSERT_FALSE(rejected.ok());
    EXPECT_EQ(rejected.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_NE(rejected.error().message.find("envelopes cannot certify"),
              std::string::npos);
  }
  const auto cones = Export(test::ConcaveMinorArcFace(0, 1));
  ASSERT_FALSE(cones.ok());
  EXPECT_EQ(cones.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, ExportsEllipticCurvedHolesWithOriginalFramesAndMixedLoops) {
  for (const bool mixed : {false, true})
    for (const bool circular_outer : {false, true})
      for (const bool rotated : {false, true}) {
        SCOPED_TRACE(::testing::Message()
                     << "mixed=" << mixed << " circular_outer="
                     << circular_outer << " rotated=" << rotated);
        const auto f =
            test::EllipticRoundedHole(mixed, circular_outer, rotated);
        const auto model = ModelBuilder::Build(f.report);
        ASSERT_TRUE(model.ok());
        const auto binding =
            BindBodyGeometry(model.value(), NodeId{1}, Options().tolerance);
        ASSERT_TRUE(binding.ok()) << binding.error().message;
        auto precise = Options();
        for (const auto& face : binding.value().faces)
          for (const auto& loop : face.loops)
            for (const auto& fin : loop.fins)
              precise.fin_intervals.push_back(
                  {fin.use.fin, fin.start, fin.end});
        // Keep the same source intervals after a world transform. Re-inferring
        // near-tangent endpoints at large coordinates can be ambiguous.
        for (const bool transformed : {false, true}) {
          const auto result = transformed
                                  ? Export(test::RotateAndTranslate(f), precise)
                                  : Export(f);
          ASSERT_TRUE(result.ok()) << result.error().message;
          EXPECT_EQ(result.value().statistics.faces, 1);
          EXPECT_EQ(result.value().statistics.edges,
                    (circular_outer ? 1 : 4) + 8 + (mixed ? 5 : 0));
          EXPECT_NE(result.value().text.find("ELLIPSE("), std::string::npos);
          CheckReferences(result.value());
        }
      }
  const auto disk = Export(test::EllipticDiskHole());
  ASSERT_TRUE(disk.ok()) << disk.error().message;
  EXPECT_EQ(disk.value().statistics.edges, 8);
}
TEST(OpenSwStep, PreservesEllipticHoleMajorArcsSeamsAndEccentricity) {
  for (const bool major : {false, true})
    for (const bool rotated : {false, true})
      for (const double stretch : {2., 100.}) {
        const auto result =
            Export(test::EllipticSegmentHole(major, rotated, stretch));
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().statistics.edges, 6);
        EXPECT_EQ(result.value().statistics.vertices, 6);
      }
  const auto fixture =
      test::Serialize(test::EllipticRoundedHole(true, true, true));
  const auto parsed =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(parsed.ok()) << parsed.error().message;
  EXPECT_EQ(parsed.value().statistics.edges, 14);
}
TEST(OpenSwStep, ChecksCompleteEllipseHoleExtremaAndConservativeSeparation) {
  // These endpoints fit, but the major elliptic arc extends to x=-2, y=-1.
  auto escaped = test::PlanarFace(
      {{-.1, -.1, 0}, {2.1, -.1, 0}, {2.1, 1.1, 0}, {-.1, 1.1, 0}},
      {{0, 1, 2, 3}});
  test::AppendFaceLoops(
      escaped,
      test::ClockwiseHole(test::EllipticProfile(test::MajorCircularSegment())),
      300, false, 10000);
  const auto outside = Export(escaped);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(outside.error().message.find("envelopes cannot certify"),
            std::string::npos);
  // The 2x1 ellipse lies inside this circle, but its rectangle does not.
  test::ConicLoopSpec circle;
  circle.major = 2.1;
  auto close = test::ConicSheet({circle});
  test::AppendFaceLoops(
      close, test::ClockwiseHole(test::EllipticProfile(test::SplitCircle())),
      300, false, 10000);
  const auto uncertified = Export(close);
  ASSERT_FALSE(uncertified.ok());
  EXPECT_EQ(uncertified.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(uncertified.error().message.find("envelopes cannot certify"),
            std::string::npos);
}
TEST(OpenSwStep, RetainsEllipseHoleProfileValidationAndWorkLimits) {
  auto options = Options();
  options.max_polygon_checks = 30;
  const auto fixture = test::EllipticSegmentHole();
  ASSERT_TRUE(Export(fixture, options).ok());
  options.max_polygon_checks = 29;
  const auto limited = Export(fixture, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  auto tilted = fixture;
  for (auto& node : tilted.report.nodes)
    if (node.node_type == 32)
      node.fields[1].values = {0., std::sin(1e-5), std::cos(1e-5)};
  const auto outside = Export(tilted);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  auto unordered = fixture;
  for (auto& node : unordered.report.nodes)
    if (node.node_type == 32) node.fields[3].values = {.5};
  ASSERT_FALSE(Export(unordered).ok());
  auto nonconvex = test::PlanarFace(
      {{-10, -10, 0}, {10, -10, 0}, {10, 10, 0}, {-10, 10, 0}}, {{0, 1, 2, 3}});
  test::AppendFaceLoops(
      nonconvex,
      test::ClockwiseHole(test::EllipticProfile(test::ConcaveArcFace())), 300,
      false, 10000);
  const auto rejected = Export(nonconvex);
  ASSERT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, ExportsConvexCurvedHolesWithPolygonalAndCircularOuterBounds) {
  for (const bool two : {false, true})
    for (const bool mixed : {false, true})
      for (const bool first : {false, true})
        for (const bool concave : {false, true}) {
          const auto f = test::ConvexCurvedHoles(two, mixed, first, concave);
          for (const auto& source : {f, test::RotateAndTranslate(f)}) {
            const auto result = Export(source);
            ASSERT_TRUE(result.ok()) << result.error().message;
            EXPECT_EQ(result.value().statistics.faces, 1);
            EXPECT_EQ(result.value().statistics.edges,
                      (concave ? 8 : 4) + (two ? 16 : 8) + (mixed ? 5 : 0));
            CheckReferences(result.value());
          }
        }
  const auto circle = Export(test::CircularOuterCurvedHole());
  ASSERT_TRUE(circle.ok()) << circle.error().message;
  EXPECT_EQ(circle.value().statistics.faces, 1);
  EXPECT_EQ(circle.value().statistics.edges, 9);
  EXPECT_EQ(circle.value().statistics.vertices, 9);
}
TEST(OpenSwStep, ExportsRoundedThroughBoresWithoutReplacingCurvedBoundaries) {
  for (const auto& source :
       {test::RoundedBorePrism(),
        test::RotateAndTranslate(test::RoundedBorePrism())}) {
    const auto result = Export(source);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_EQ(result.value().statistics.faces, 14);
    EXPECT_EQ(result.value().statistics.edges, 36);
    EXPECT_EQ(result.value().statistics.vertices, 24);
  }
}
TEST(OpenSwStep, BoundsCompleteCurvedHoleArcsIncludingMajorAndPeriodicSpans) {
  for (const bool major : {false, true})
    for (const bool phase : {false, true}) {
      const auto result = Export(test::CircularSegmentHole(major, phase));
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.edges, 6);
      EXPECT_EQ(result.value().statistics.vertices, 6);
    }
  // Endpoints fit inside this square, but the major arc extends to (-1,-1).
  auto escaped = test::PlanarFace(
      {{-.1, -.1, 0}, {1.1, -.1, 0}, {1.1, 1.1, 0}, {-.1, 1.1, 0}},
      {{0, 1, 2, 3}});
  test::AppendFaceLoops(escaped,
                        test::ClockwiseHole(test::MajorCircularSegment()), 300,
                        false, 10000);
  const auto result = Export(escaped);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("envelopes cannot certify"),
            std::string::npos);
}
TEST(OpenSwStep, RejectsUncertifiedCurvedHoleEnvelopeSeparation) {
  for (const auto translation : {Vector3{2, 0, 0}, Vector3{4, 0, 0},
                                 Vector3{.5, .5, 0}, Vector3{20, 0, 0}}) {
    auto f = test::ConvexCurvedHoles();
    test::AppendFaceLoops(
        f, test::TranslateProfile(test::RoundedRectangle(true), translation),
        300, false, 20000);
    const auto result = Export(f);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
    EXPECT_NE(result.error().message.find("envelopes cannot certify"),
              std::string::npos);
  }
  // The circles are disjoint, but the inner circle's enclosing rectangle
  // cannot fit inside the outer disk. This conservative case stays unsupported.
  test::ConicLoopSpec outer;
  outer.major = 1.2;
  auto close = test::ConicSheet({outer});
  test::AppendFaceLoops(close, test::ClockwiseHole(test::SplitCircle()), 300,
                        false, 10000);
  const auto result = Export(close);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(result.error().message.find("envelopes cannot certify"),
            std::string::npos);
  auto mixed = test::ConvexCurvedHoles();
  test::ConicLoopSpec disk;
  disk.center = {2, 1.5, 0};
  disk.major = .25;
  disk.reverse_fin = true;
  test::AddConicFaceLoops(mixed, 300, {disk}, false, false, 20000);
  const auto nested = Export(mixed);
  ASSERT_FALSE(nested.ok());
  EXPECT_EQ(nested.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, BoundsCurvedHoleEnvelopeWorkAndPreservesSourceErrors) {
  auto options = Options();
  options.max_polygon_checks = 30;
  const auto f = test::CircularSegmentHole();
  ASSERT_TRUE(Export(f, options).ok());
  options.max_polygon_checks = 29;
  const auto limited = Export(f, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  auto miswound = test::ConvexCurvedHoles();
  // Change only the effective face normal; the original outer polygon is
  // then miswound and must not become an accepted envelope-based export.
  test::Set(miswound, 300, "sense", '-');
  ASSERT_FALSE(Export(miswound).ok());
  const auto bytes =
      test::Serialize(test::ConvexCurvedHoles(true, true, true, true));
  const auto parsed =
      ExportParasolid(bytes.bytes, NodeId{1}, Options(), &bytes.catalog);
  ASSERT_TRUE(parsed.ok()) << parsed.error().message;
  EXPECT_EQ(parsed.value().statistics.faces, 1);
}
TEST(OpenSwStep, LocatesAnalyticArcExtremaWithoutLargePhaseCancellation) {
  for (const double start : {0., 7 * std::numbers::pi / 4, 1e12, -1e12}) {
    const long double mid = static_cast<long double>(start) + .5;
    const auto a = std::cos(mid), b = std::sin(mid);
    EXPECT_NEAR(internal::MaximumArcProjection(a, b, start, start + 1), 1,
                1e-15);
    EXPECT_NEAR(internal::MaximumArcProjection(a, b, start + 1, start), 1,
                1e-15);
    EXPECT_NEAR(internal::MaximumArcProjection(-a, -b, start, start + 1),
                -std::cos(.5L), 1e-15);
  }
}
TEST(OpenSwStep, ExportsPolygonAndMixedHolesInConvexCurvedContours) {
  for (const bool mixed : {false, true})
    for (const bool concave : {false, true})
      for (const bool first : {false, true})
        for (const bool inward : {false, true}) {
          const auto f =
              test::RoundedPolygonHoles(mixed, concave, first, inward, true);
          for (const auto& source : {f, test::RotateAndTranslate(f)}) {
            const auto result = Export(source);
            ASSERT_TRUE(result.ok()) << result.error().message;
            EXPECT_EQ(result.value().statistics.faces, 1);
            EXPECT_EQ(result.value().statistics.edges,
                      8 + (concave ? 6 : 8) + (mixed ? 1 : 0));
            CheckReferences(result.value());
          }
        }
}
TEST(OpenSwStep, PreservesPolygonBoresThroughCurvedSolidCaps) {
  for (const auto& f :
       {test::CylinderSectorWithPolygonBore(),
        test::RotateAndTranslate(test::CylinderSectorWithPolygonBore())}) {
    const auto result = Export(f);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_EQ(result.value().statistics.faces, 9);
    EXPECT_EQ(result.value().statistics.edges, 21);
    EXPECT_EQ(result.value().statistics.vertices, 14);
  }
  const auto fixture = test::Serialize(test::CylinderSectorWithPolygonBore());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, RejectsPolygonHolesOutsideOrTouchingCurvedContours) {
  for (const auto& hole : std::vector<std::vector<Vector3>>{
           {{3.5, .1, 0}, {3.5, .3, 0}, {3.7, .3, 0}, {3.7, .1, 0}},
           {{1, 0, 0}, {1, .2, 0}, {2, .2, 0}, {2, 0, 0}},
           {{1, 1e-7, 0}, {1, .2, 0}, {2, .2, 0}, {2, 1e-7, 0}},
           {{4.1, 1, 0}, {4.1, 2, 0}, {4.2, 2, 0}, {4.2, 1, 0}}}) {
    const auto result = Export(test::RoundedPolygonFace({hole}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().message.find("convex curved contour"),
              std::string::npos)
        << result.error().message;
  }
  ASSERT_TRUE(
      Export(test::RoundedPolygonFace(
                 {{{1, 3e-7, 0}, {1, .2, 0}, {2, .2, 0}, {2, 3e-7, 0}}}))
          .ok());
}
TEST(OpenSwStep, RejectsCrossingTouchingNestedAndMiswoundCurvedPolygonHoles) {
  const std::vector<Vector3> square{{1, 1, 0}, {1, 2, 0}, {2, 2, 0}, {2, 1, 0}};
  for (const auto& other : std::vector<std::vector<Vector3>>{
           {{1.5, 1.5, 0}, {1.5, 2.5, 0}, {2.5, 2.5, 0}, {2.5, 1.5, 0}},
           {{2, 1, 0}, {2, 2, 0}, {3, 2, 0}, {3, 1, 0}},
           {{1.2, 1.2, 0}, {1.2, 1.8, 0}, {1.8, 1.8, 0}, {1.8, 1.2, 0}},
           {{1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {1, 2, 0}}}) {
    const auto result = Export(test::RoundedPolygonFace({square, other}));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  const auto crossed = Export(test::RoundedPolygonFace(
      {{{1, 1, 0}, {1, 2, 0}, {3, 1, 0}, {3, 1.5, 0}}}));
  ASSERT_FALSE(crossed.ok());
  EXPECT_NE(crossed.error().message.find("cross, touch"), std::string::npos)
      << crossed.error().message;
}
TEST(OpenSwStep, RejectsMixedHoleIntersectionAndNestingInCurvedContours) {
  const std::vector<Vector3> square{{1, 1, 0}, {1, 2, 0}, {2, 2, 0}, {2, 1, 0}};
  for (const auto& spec :
       std::vector<std::pair<Vector3, double>>{{{1.5, 1.5, 0}, .1},
                                               {{1.5, 1.5, 0}, .8},
                                               {{2.1, 1.5, 0}, .2},
                                               {{2.25, 1.5, 0}, .25}}) {
    auto f = test::RoundedPolygonFace({square});
    test::ConicLoopSpec hole;
    hole.center = spec.first;
    hole.major = spec.second;
    hole.reverse_fin = true;
    test::AddConicFaceLoops(f, 300, {hole});
    const auto result = Export(f);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().message.find("polygon"), std::string::npos)
        << result.error().message;
  }
}
TEST(OpenSwStep, BoundsPolygonHoleChecksInsideCurvedContours) {
  const auto f =
      test::RoundedPolygonFace({{{1, 1, 0}, {1, 2, 0}, {2, 2, 0}, {2, 1, 0}}});
  auto options = Options();
  // Eight outer corners, two nonadjacent polygon pairs, 4*8 support tests.
  options.max_polygon_checks = 42;
  ASSERT_TRUE(Export(f, options).ok());
  options.max_polygon_checks = 41;
  const auto limited = Export(f, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto mixed = test::RoundedPolygonHoles(true);
  // 8 corners + 8 circle/outer + 16 disk/segment/containment + 28 polygon
  // segment/containment comparisons + 8*8 vertex/outer comparisons.
  options.max_polygon_checks = 124;
  ASSERT_TRUE(Export(mixed, options).ok());
  options.max_polygon_checks = 123;
  const auto shared = Export(mixed, options);
  ASSERT_FALSE(shared.ok());
  EXPECT_EQ(shared.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, ExportsConvexCurvedContoursWithCircularHoles) {
  for (const bool inward : {false, true})
    for (const bool first : {false, true})
      for (const bool seams : {false, true}) {
        const auto fixture =
            test::RoundedRectangleWithCircularHoles(inward, first, seams);
        for (const auto& source :
             {fixture, test::RotateAndTranslate(fixture)}) {
          const auto result = Export(source);
          ASSERT_TRUE(result.ok()) << result.error().message;
          EXPECT_EQ(result.value().statistics.faces, 1);
          EXPECT_EQ(result.value().statistics.edges, 10);
          EXPECT_EQ(result.value().statistics.vertices, 10);
          CheckReferences(result.value());
        }
      }
  for (const auto& source :
       {test::WithCircularHole(test::Stadium(), {-1, 0, 0}),
        test::WithCircularHole(test::HalfDisk(), {0, .5, 0}),
        test::WithCircularHole(test::MajorCircularSegment(), {0, 0, 0}),
        test::WithCircularHole(test::CircularLens(), {0, 0, 0}),
        test::WithCircularHole(test::SplitCircle(), {0, 0, 0}),
        test::WithCircularHole(
            test::RotateLineProfile(test::MajorCircularSegment(),
                                    7 * std::numbers::pi / 4),
            {0, 0, 0})}) {
    const auto result = Export(source);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
  }
}
TEST(OpenSwStep, ExportsCurvedCapsAndCylindricalThroughBores) {
  for (const bool seams : {false, true}) {
    const auto fixture = test::CylinderSectorWithBore(seams);
    for (const auto& source : {fixture, test::RotateAndTranslate(fixture)}) {
      const auto result = Export(source);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.solids, 1);
      EXPECT_EQ(result.value().statistics.faces, 6);
      EXPECT_EQ(result.value().statistics.edges, 11);
    }
  }
  const auto fixture = test::Serialize(test::CylinderSectorWithBore());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, RejectsCircularHolesOutsideOrTouchingCurvedContours) {
  // The first disk crosses an interior arc point while staying inside all
  // endpoint tangent half-planes. Testing only arc endpoints would accept it.
  for (const auto center : {Vector3{1.6, .6, 0}, Vector3{1.75, 0, 0},
                            Vector3{0, .75, 0}, Vector3{3, 0, 0}}) {
    const auto result = Export(test::WithCircularHole(test::Stadium(), center));
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
    EXPECT_NE(result.error().message.find("outside or touches"),
              std::string::npos)
        << result.error().message;
  }
  const auto near =
      Export(test::WithCircularHole(test::Stadium(), {1.75 - 1e-7, 0, 0}));
  ASSERT_FALSE(near.ok());
  ASSERT_TRUE(
      Export(test::WithCircularHole(test::Stadium(), {1.75 - 3e-7, 0, 0}))
          .ok());
}
TEST(OpenSwStep, RejectsOverlappingNestedOrIncorrectlyWoundCurvedHoles) {
  for (const double second_x : {1., 1.25, 1.5, 1.5 + 1e-7}) {
    auto f = test::Stadium();
    test::ConicLoopSpec a;
    a.center = {1, 0, 0};
    a.major = .25;
    a.reverse_fin = true;
    auto b = a;
    b.center.x = second_x;
    test::AddConicFaceLoops(f, 300, {a, b});
    const auto result = Export(f);
    ASSERT_FALSE(result.ok());
    EXPECT_NE(result.error().message.find("overlap, touch or are nested"),
              std::string::npos)
        << result.error().message;
  }
  auto nested = test::WithCircularHole(test::Stadium(), {0, 0, 0}, .5);
  test::ConicLoopSpec inner;
  inner.major = .1;
  inner.reverse_fin = true;
  test::AddConicFaceLoops(nested, 300, {inner}, false, false, 20000);
  ASSERT_FALSE(Export(nested).ok());
  auto wrong = test::WithCircularHole(test::Stadium(), {0, 0, 0});
  test::Set(wrong, 11001, "loop", 0);
  test::Set(wrong, 11001, "forward", 0);
  test::Set(wrong, 11001, "backward", 0);
  test::Set(wrong, 11000, "loop", 10600);
  test::Set(wrong, 11000, "forward", 11000);
  test::Set(wrong, 11000, "backward", 11000);
  test::Set(wrong, 10600, "halfedge", 11000);
  const auto result = Export(wrong);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("correctly wound"), std::string::npos);
}
TEST(OpenSwStep, BoundsCurvedHoleChecksAndRejectsUnsupportedHoleContours) {
  auto options = Options();
  options.max_polygon_checks = 8;
  const auto fixture = test::WithCircularHole(test::Stadium(), {0, 0, 0});
  ASSERT_TRUE(Export(fixture, options).ok());
  options.max_polygon_checks = 7;
  const auto limited = Export(fixture, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  options.max_polygon_checks = 25;
  ASSERT_TRUE(Export(test::RoundedRectangleWithCircularHoles(), options).ok());
  options.max_polygon_checks = 24;
  const auto paired =
      Export(test::RoundedRectangleWithCircularHoles(), options);
  ASSERT_FALSE(paired.ok());
  EXPECT_EQ(paired.error().code, ErrorCode::kResourceLimit);
  const auto elliptic_outer =
      Export(test::WithCircularHole(test::HalfEllipse(), {0, .5, 0}, .1));
  ASSERT_FALSE(elliptic_outer.ok());
  EXPECT_EQ(elliptic_outer.error().code, ErrorCode::kUnsupportedFormat);
  auto elliptic_hole = test::Stadium();
  test::ConicLoopSpec hole;
  hole.ellipse = true;
  hole.major = .25;
  hole.minor = .2;
  hole.reverse_fin = true;
  test::AddConicFaceLoops(elliptic_hole, 300, {hole});
  const auto unsupported = Export(elliptic_hole);
  ASSERT_FALSE(unsupported.ok());
  EXPECT_EQ(unsupported.error().code, ErrorCode::kUnsupportedFormat);
  const auto nonconvex =
      Export(test::WithCircularHole(test::AnnularSector(), {1.5, 1.5, 0}, .1));
  ASSERT_FALSE(nonconvex.ok());
  EXPECT_EQ(nonconvex.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, ExportsExactConePatchesAndClosedSectors) {
  for (const double slope : {1., -0.25}) {
    for (const double span :
         {std::numbers::pi / 2, std::numbers::pi, 1.5 * std::numbers::pi})
      for (const bool inward : {false, true}) {
        const auto fixture = test::ConePatchFixture(span, inward, slope);
        for (const auto& source :
             {fixture, test::RotateAndTranslate(fixture)}) {
          const auto result = Export(source);
          ASSERT_TRUE(result.ok()) << result.error().message;
          EXPECT_EQ(result.value().statistics.faces, 1);
          EXPECT_EQ(result.value().statistics.edges, 4);
          EXPECT_EQ(result.value().statistics.vertices, 4);
          CheckReferences(result.value());
        }
      }
    for (const auto& source :
         {test::ConeSector(slope),
          test::RotateAndTranslate(test::ConeSector(slope))}) {
      const auto result = Export(source);
      ASSERT_TRUE(result.ok()) << result.error().message;
      EXPECT_EQ(result.value().statistics.solids, 1);
      EXPECT_EQ(result.value().statistics.faces, 5);
      EXPECT_EQ(result.value().statistics.edges, 9);
    }
  }
  ASSERT_TRUE(Export(test::RotateLineProfile(test::ConePatchFixture(),
                                             7 * std::numbers::pi / 4))
                  .ok());
}
TEST(OpenSwStep, ChecksConePatchProfilesGeneratorsAndSharedBudget) {
  auto options = Options();
  options.max_polygon_checks = 5;
  ASSERT_TRUE(Export(test::ConePatchFixture(), options).ok());
  options.max_polygon_checks = 4;
  const auto limited = Export(test::ConePatchFixture(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto twisted =
      Export(test::ConePatchFixture(std::numbers::pi / 2, false, 1, 1e-4));
  ASSERT_FALSE(twisted.ok());
  EXPECT_NE(twisted.error().message.find("cone generator"), std::string::npos);
  auto tilted = test::ConePatchFixture(std::numbers::pi);
  for (auto& node : tilted.report.nodes)
    if (node.index == 4000)
      node.fields[1].values = {0., std::sin(1e-7), std::cos(1e-7)};
  const auto outside = Export(tilted);
  ASSERT_FALSE(outside.ok());
  EXPECT_NE(outside.error().message.find("coaxial profile"), std::string::npos)
      << outside.error().message;
}
TEST(OpenSwStep, PreservesConePatchAxisReferenceAndLoopOrder) {
  auto reverse = test::ConePatchFixture();
  for (auto& node : reverse.report.nodes)
    if (node.index == 700) {
      node.fields[1].values = {0., 0., -1.};
      node.fields[3].values[0] = -std::get<double>(node.fields[3].values[0]);
    }
  ASSERT_TRUE(Export(reverse).ok());
  auto reordered = test::ConePatchFixture();
  test::Set(reordered, 600, "halfedge", 1004);
  ASSERT_TRUE(Export(reordered).ok());
  auto apex = test::ConePatchFixture();
  for (auto& node : apex.report.nodes)
    if (node.index == 700) {
      node.fields[0].values = {0., 0., -1.};
      node.fields[2].values[0] = 0.;
    }
  ASSERT_TRUE(Export(apex).ok());
}
TEST(OpenSwStep, IntegratesConePatchFluxWithAxialAndTransverseOffsets) {
  const double q = std::sqrt(0.5);
  ConeGeometry cone{{0, 0, 0}, {0, 0, 1}, 1, q, q, {1, 0, 0}};
  internal::ConePatch patch{
      2, std::numbers::pi / 2, 1, 3, {1, 0, 0}, {0, 1, 0}, {1, 0, 0}};
  EXPECT_NEAR(internal::ConePatchVolume6(cone, patch, {}), 4 * std::numbers::pi,
              1e-12);
  EXPECT_NEAR(internal::ConePatchVolume6(cone, patch, {1, 0, 0}),
              4 * std::numbers::pi - 8, 1e-12);
  EXPECT_NEAR(internal::ConePatchVolume6(cone, patch, {0, 1, 2}),
              12 * std::numbers::pi - 8, 1e-12);
  cone.origin = {1e6, -2e6, 3e6};
  EXPECT_NEAR(internal::ConePatchVolume6(cone, patch, {1e6, -2e6 + 1, 3e6 + 2}),
              12 * std::numbers::pi - 8, 1e-12);
  cone.axis = {0, 0, -1};
  cone.sin_half_angle = -q;
  std::swap(patch.first_direction, patch.last_direction);
  std::swap(patch.lower_radius, patch.upper_radius);
  EXPECT_NEAR(internal::ConePatchVolume6(cone, patch, {1e6, -2e6 + 1, 3e6 + 2}),
              12 * std::numbers::pi - 8, 1e-12);
}
TEST(OpenSwStep, RejectsUnsupportedOrMismatchedConePatchArcs) {
  const GeometryPayload intersection = IntersectionCurveGeometry{};
  std::array<internal::PlanarCurve, 4> curves;
  curves.fill({&intersection, 0, 1});
  uint64_t remaining = 5;
  const double q = std::sqrt(0.5);
  const auto foreign = internal::ValidateConePatch(
      curves, {{0, 0, 0}, {0, 0, 1}, 1, q, q, {1, 0, 0}}, true, 1e-7,
      remaining);
  ASSERT_FALSE(foreign.ok());
  EXPECT_EQ(foreign.error().code, ErrorCode::kUnsupportedFormat);
  auto wrong = test::ConePatchFixture();
  test::Set(wrong, 4002, "sense", '+');
  const auto result = Export(wrong);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("winding"), std::string::npos);
  auto inverted = test::ConeSector();
  for (uint32_t i = 0; i < 5; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto negative = Export(inverted);
  ASSERT_FALSE(negative.ok());
  EXPECT_NE(negative.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, ParsesConePatchesThroughBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::ConeSector());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, ExportsExactCylinderPatchesAndClosedSectors) {
  for (const auto& fixture :
       {test::CylinderPatchFixture(),
        test::CylinderPatchFixture(std::numbers::pi),
        test::CylinderPatchFixture(1.5 * std::numbers::pi),
        test::CylinderPatchFixture(std::numbers::pi / 2, true),
        test::PhaseShiftedCylinderPatch(),
        test::RotateAndTranslate(test::CylinderPatchFixture())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_EQ(result.value().statistics.edges, 4);
    EXPECT_EQ(result.value().statistics.vertices, 4);
    CheckReferences(result.value());
  }
  for (const auto& fixture :
       {test::CylinderSector(),
        test::RotateAndTranslate(test::CylinderSector())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_EQ(result.value().statistics.faces, 5);
    EXPECT_EQ(result.value().statistics.edges, 9);
  }
}
TEST(OpenSwStep, ChecksCylinderPatchProfileGeneratorsAndSharedBudget) {
  auto options = Options();
  options.max_polygon_checks = 5;
  ASSERT_TRUE(Export(test::CylinderPatchFixture(), options).ok());
  options.max_polygon_checks = 4;
  const auto limited = Export(test::CylinderPatchFixture(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto twisted =
      Export(test::CylinderPatchFixture(std::numbers::pi / 2, false, 1e-4));
  ASSERT_FALSE(twisted.ok());
  EXPECT_NE(twisted.error().message.find("axial generator"), std::string::npos);
  auto tilted = test::CylinderPatchFixture(std::numbers::pi);
  for (auto& node : tilted.report.nodes)
    if (node.index == 4000)
      node.fields[1].values = {0., std::sin(1e-6), std::cos(1e-6)};
  const auto outside = Export(tilted);
  ASSERT_FALSE(outside.ok());
  EXPECT_NE(outside.error().message.find("coaxial profile"), std::string::npos);
}
TEST(OpenSwStep, PreservesCylinderPatchAxisLoopOrderAndVolumeSense) {
  auto reverse = test::CylinderPatchFixture();
  for (auto& node : reverse.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(reverse).ok());
  auto reordered = test::CylinderPatchFixture();
  test::Set(reordered, 600, "halfedge", 1004);
  ASSERT_TRUE(Export(reordered).ok());
  auto inverted = test::CylinderSector();
  for (uint32_t i = 0; i < 5; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, IntegratesPartialCylinderFluxWithTransverseTranslations) {
  CylinderGeometry cylinder{{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}};
  internal::CylinderPatch patch{
      2, std::numbers::pi / 2, {1, 0, 0}, {0, 1, 0}, {1, 0, 0}};
  EXPECT_NEAR(internal::CylinderPatchVolume6(cylinder, patch, {}),
              2 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(internal::CylinderPatchVolume6(cylinder, patch, {1, 0, 0}),
              2 * std::numbers::pi - 4, 1e-12);
  EXPECT_NEAR(internal::CylinderPatchVolume6(cylinder, patch, {0, 1, 100}),
              2 * std::numbers::pi - 4, 1e-12);
  cylinder.origin = {1e6, -2e6, 3e6};
  EXPECT_NEAR(
      internal::CylinderPatchVolume6(cylinder, patch, {1e6 + 1, -2e6, 3e6}),
      2 * std::numbers::pi - 4, 1e-12);
  cylinder.axis = {0, 0, -1};
  std::swap(patch.first_direction, patch.last_direction);
  EXPECT_NEAR(
      internal::CylinderPatchVolume6(cylinder, patch, {1e6 + 1, -2e6, 3e6}),
      2 * std::numbers::pi - 4, 1e-12);
}
TEST(OpenSwStep, RejectsUnimplementedOrMismatchedCylinderPatchArcs) {
  // Reject foreign bases before invoking evaluators that require a solver.
  const GeometryPayload intersection = IntersectionCurveGeometry{};
  std::array<internal::PlanarCurve, 4> curves;
  curves.fill({&intersection, 0, 1});
  uint64_t remaining = 5;
  const auto foreign = internal::ValidateCylinderPatch(
      curves, {{0, 0, 0}, {0, 0, 1}, 1, {1, 0, 0}}, true, 1e-7, remaining);
  ASSERT_FALSE(foreign.ok());
  EXPECT_EQ(foreign.error().code, ErrorCode::kUnsupportedFormat);
  auto ellipse = test::CylinderPatchFixture();
  test::RemoveNode(ellipse, 4000);
  ellipse.Add(
      32, 4000,
      {test::Numbers("center", {0, 0, 0}, FieldType::kVector),
       test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
       test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
       test::Numbers("major_radius", {1}), test::Numbers("minor_radius", {1}),
       test::ByteValue("sense", '+', FieldType::kCharacter)});
  const auto unsupported = Export(ellipse);
  ASSERT_FALSE(unsupported.ok());
  EXPECT_EQ(unsupported.error().code, ErrorCode::kUnsupportedFormat);
  auto wrong = test::CylinderPatchFixture();
  test::Set(wrong, 4002, "sense", '+');
  const auto result = Export(wrong);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("winding"), std::string::npos);
}
TEST(OpenSwStep, ParsesCylinderPatchesThroughBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::CylinderSector());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, ExportsSphericalCapsAndHemispheresWithoutInventingSourceCuts) {
  for (const double level : {-3., 0., 3.})
    for (const bool north : {false, true})
      for (const bool seams : {false, true})
        for (const bool solid : {false, true}) {
          const auto result =
              Export(test::SphericalCap(level, north, seams, solid));
          ASSERT_TRUE(result.ok()) << result.error().message;
          EXPECT_EQ(result.value().statistics.faces, solid ? 2 : 1);
          EXPECT_EQ(result.value().statistics.edges, 1);
          EXPECT_EQ(result.value().statistics.vertices, 1);
          EXPECT_EQ(result.value().statistics.solids, solid ? 1 : 0);
          CheckReferences(result.value());
        }
}
TEST(OpenSwStep, PreservesSphericalCapSenseAxisAndAnalyticVolume) {
  for (const auto& fixture :
       {test::InwardSphericalCap(), test::ReversedSphericalCap(),
        test::RotateAndTranslate(test::SphericalCap())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
  }
  auto inverted = test::SphericalCap();
  for (uint32_t i = 0; i < 2; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, ParsesSphericalCapsAndRetainsCutValidation) {
  auto options = Options();
  options.max_polygon_checks = 1;
  const auto fixture = test::Serialize(test::SphericalCap(-3));
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, options, &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.solids, 1);
  const auto limited = Export(test::PartitionedSphere(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  options.max_polygon_checks = 2;
  const auto sphere = Export(test::PartitionedSphere(3, true), options);
  ASSERT_TRUE(sphere.ok()) << sphere.error().message;
  EXPECT_EQ(sphere.value().statistics.faces, 2);
  EXPECT_EQ(sphere.value().statistics.edges, 1);
  EXPECT_EQ(sphere.value().statistics.solids, 1);
  auto tilted = test::SphericalCap(0, true, false, false);
  constexpr double angle = 1e-6;
  for (auto& node : tilted.report.nodes)
    if (node.index == 4000)
      node.fields[1].values = {0., std::sin(angle), std::cos(angle)};
  EXPECT_FALSE(Export(tilted).ok());
  const auto pole = Export(test::SphericalCap(5 - 1e-8, true, false, false));
  ASSERT_FALSE(pole.ok());
  EXPECT_EQ(pole.error().code, ErrorCode::kUnsupportedFormat);
}
TEST(OpenSwStep, ExportsExactSphericalBandsAndClosedTruncatedSpheres) {
  for (const auto& fixture :
       {test::SphericalBand(), test::SphericalBand(0, 3, false, true),
        test::SphericalBand(-3, 3), test::SphericalBand(-3, 0),
        test::SphericalBand(0, 3, true), test::SphericalBand(0, 3, true, true),
        test::ReversedSphericalBand(), test::ReversedSphericalBand(true),
        test::RotateAndTranslate(test::SphericalBand())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.solids, 1);
    EXPECT_NE(result.value().text.find("=SPHERICAL_SURFACE("),
              std::string::npos);
    CheckReferences(result.value());
  }
  for (const auto& fixture :
       {test::InwardSphericalBand(),
        test::SphericalBand(0, 3, false, false, false, true)}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_EQ(result.value().statistics.edges, 2);
  }
}
TEST(OpenSwStep, RejectsInvalidSphereCutsAndBoundsPairChecks) {
  auto options = Options();
  options.max_polygon_checks = 1;
  const auto result = Export(test::SphericalBand(), options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto two_bands = test::SphericalBand(0, 3, true);
  const auto limited = Export(two_bands, options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  for (const auto& fixture :
       {test::SphericalBand(0, 0, false, false, false, true),
        test::SphericalBand(0, 1e-7, false, false, false, true)}) {
    const auto invalid = Export(fixture);
    ASSERT_FALSE(invalid.ok());
    EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidRecord);
  }
  auto wrong = test::SphericalBand(0, 3, false, false, false, true);
  test::Set(wrong, 4001, "sense", '-');
  EXPECT_FALSE(Export(wrong).ok());
  auto outside = test::SphericalBand(0, 3, false, false, false, true);
  for (auto& node : outside.report.nodes)
    if (node.index == 4001) node.fields[3].values = {4.01};
  EXPECT_FALSE(Export(outside).ok());
}
TEST(OpenSwStep, ChecksEntireSphereLatitudeCutsAndRejectsPoleAndEllipseCuts) {
  // A tilted great circle still lies on the sphere; sampled surface membership
  // cannot establish that its whole supporting circle is a latitude cut.
  auto tilted = test::SphericalBand(0, 3, false, false, false, true);
  constexpr double angle = 1e-6;
  for (auto& node : tilted.report.nodes)
    if (node.index == 4000)
      node.fields[1].values = {0., std::sin(angle), std::cos(angle)};
  const auto invalid = Export(tilted);
  ASSERT_FALSE(invalid.ok());
  EXPECT_NE(invalid.error().message.find("coaxial profile"), std::string::npos);
  const auto pole =
      Export(test::SphericalBand(0, 5 - 1e-8, false, false, false, true));
  ASSERT_FALSE(pole.ok());
  EXPECT_EQ(pole.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(pole.error().message.find("pole"), std::string::npos);
  auto ellipse = test::SphericalBand(0, 3, false, false, false, true);
  test::RemoveNode(ellipse, 4000);
  ellipse.Add(
      32, 4000,
      {test::Numbers("center", {0, 0, 0}, FieldType::kVector),
       test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
       test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
       test::Numbers("major_radius", {5}), test::Numbers("minor_radius", {5}),
       test::ByteValue("sense", '+', FieldType::kCharacter)});
  const auto unsupported_ellipse = Export(ellipse);
  ASSERT_FALSE(unsupported_ellipse.ok());
  EXPECT_EQ(unsupported_ellipse.error().code, ErrorCode::kUnsupportedFormat);
  EXPECT_NE(unsupported_ellipse.error().message.find("CIRCLE"),
            std::string::npos);
}
TEST(OpenSwStep, RejectsInvertedSphericalSolidsUsingAnalyticVolume) {
  auto inverted = test::SphericalBand();
  for (uint32_t i = 0; i < 3; ++i) {
    test::Set(inverted, 300 + i, "shell", 4);
    test::Set(inverted, 300 + i, "front_shell", 5);
  }
  test::Set(inverted, 4, "face", 300);
  test::Set(inverted, 4, "front_face", 0);
  test::Set(inverted, 5, "face", 0);
  test::Set(inverted, 5, "front_face", 300);
  const auto result = Export(inverted);
  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().message.find("signed volume"), std::string::npos);
}
TEST(OpenSwStep, ParsesSphericalBandsThroughBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::SphericalBand());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.faces, 3);
  EXPECT_EQ(result.value().statistics.solids, 1);
}
TEST(OpenSwStep, ExportsCircularFacesWithPolygonalAndCircularHoles) {
  for (const bool two : {false, true})
    for (const bool circle : {false, true})
      for (const bool first : {false, true})
        for (const bool seams : {false, true}) {
          const auto result =
              Export(test::CircularPolygonHoles(two, circle, first, seams));
          ASSERT_TRUE(result.ok()) << result.error().message;
          const auto edges = (two ? 8 : 4) + (circle ? 2 : 1);
          EXPECT_EQ(result.value().statistics.faces, 1);
          EXPECT_EQ(result.value().statistics.edges, edges);
          EXPECT_EQ(result.value().statistics.vertices, edges);
          CheckReferences(result.value());
        }
  for (const auto& fixture :
       {test::CircularPolygonHoles(false, true, false, false, true),
        test::ReorderedCircularPolygonHoles(),
        test::RotateAndTranslate(test::CircularPolygonHoles(true, true))}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
  }
}
TEST(OpenSwStep, PreservesMixedCircleOuterOrientationAndSharedBudget) {
  auto negative = test::CircularPolygonHoles();
  test::Set(negative, 300, "sense", '-');
  for (auto& node : negative.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  const auto result = Export(negative);
  ASSERT_TRUE(result.ok()) << result.error().message;
  auto reversed = test::CircularPolygonHoles();
  test::Set(reversed, 14000, "sense", '-');
  for (auto& node : reversed.report.nodes)
    if (node.index == 14000) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(reversed).ok());
  auto options = Options();
  // Four vertex-to-circle tests and two nonadjacent polygon edge pairs.
  options.max_polygon_checks = 6;
  ASSERT_TRUE(Export(test::CircularPolygonHoles(), options).ok());
  options.max_polygon_checks = 5;
  const auto limited = Export(test::CircularPolygonHoles(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
}
TEST(OpenSwStep, RejectsPolygonHolesOutsideOrTouchingOuterCircle) {
  for (const double radius : {1., std::sqrt(2.), std::sqrt(2.) + 1e-7}) {
    auto fixture = test::CircularPolygonHoles();
    for (auto& node : fixture.report.nodes)
      if (node.index == 14000) node.fields[3].values = {radius};
    const auto result = Export(fixture);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
}
TEST(OpenSwStep, RejectsIntersectingOrNestedHolesInsideCircularFaces) {
  for (const Vector3 center : {Vector3{0, 0, 0}, Vector3{0, 1.5, 0},
                               Vector3{0, 2, 0}, Vector3{0, 2 + 1e-7, 0}}) {
    auto fixture = test::CircularPolygonHoles(false, true);
    for (auto& node : fixture.report.nodes)
      if (node.index == 14001)
        node.fields[0].values = {center.x, center.y, center.z};
    const auto result = Export(fixture);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  // A large circular hole enclosing a polygon hole has no edge crossings.
  auto enclosing = test::CircularPolygonHoles(false, true);
  for (auto& node : enclosing.report.nodes)
    if (node.index == 14001) {
      node.fields[0].values = {0., 0., 0.};
      node.fields[3].values = {2.};
    }
  EXPECT_FALSE(Export(enclosing).ok());
  auto nested_circle = test::CircularPolygonHoles(false, true);
  for (auto& node : nested_circle.report.nodes)
    if (node.index == 14001) {
      node.fields[0].values = {0., 0., 0.};
      node.fields[3].values = {.25};
    }
  const auto nested = Export(nested_circle);
  ASSERT_FALSE(nested.ok());
  EXPECT_NE(nested.error().message.find("nested inside"), std::string::npos);
  // Both polygon holes are strictly inside the circle, but one encloses the
  // other without any edge intersections.
  auto nested_polygons = test::PlanarFace({{-3, -3, 0},
                                           {-3, 3, 0},
                                           {3, 3, 0},
                                           {3, -3, 0},
                                           {-1, -1, 0},
                                           {-1, 1, 0},
                                           {1, 1, 0},
                                           {1, -1, 0}},
                                          {{0, 1, 2, 3}, {4, 5, 6, 7}});
  test::ConicLoopSpec outer;
  outer.major = 6;
  test::AddConicFaceLoops(nested_polygons, 300, {outer});
  const auto nested_result = Export(nested_polygons);
  ASSERT_FALSE(nested_result.ok());
  EXPECT_NE(nested_result.error().message.find("Nested hole loops"),
            std::string::npos);
  // Two valid hole profiles become touching or crossing when shifted.
  for (const double shift : {2., 3.}) {
    auto fixture = test::CircularPolygonHoles(true);
    for (auto& node : fixture.report.nodes)
      if (node.node_type == 29 && node.index >= 504 && node.index <= 507)
        node.fields[0].values[0] =
            std::get<double>(node.fields[0].values[0]) - shift;
    for (auto& node : fixture.report.nodes)
      if (node.node_type == 30 && node.index >= 4004 && node.index <= 4007)
        node.fields[0].values[0] =
            std::get<double>(node.fields[0].values[0]) - shift;
    EXPECT_FALSE(Export(fixture).ok());
  }
}
TEST(OpenSwStep, ParsesCircularPolygonHolesThroughBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::ReorderedCircularPolygonHoles());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.edges, 6);
}
TEST(OpenSwStep, ExportsPolygonFacesWithCircularAndPolygonalHoles) {
  for (const bool polygon_hole : {false, true})
    for (const bool hole_first : {false, true})
      for (const bool seams : {false, true}) {
        const auto result =
            Export(test::PolygonCircularHoles(polygon_hole, hole_first, seams));
        ASSERT_TRUE(result.ok()) << result.error().message;
        EXPECT_EQ(result.value().statistics.faces, 1);
        EXPECT_EQ(result.value().statistics.edges, polygon_hole ? 10 : 6);
        EXPECT_EQ(result.value().statistics.vertices, polygon_hole ? 10 : 6);
        EXPECT_NE(result.value().text.find("=FACE_OUTER_BOUND("),
                  std::string::npos);
        CheckReferences(result.value());
      }
  for (const auto& fixture :
       {test::PolygonCircularHoles(true, true, true, true),
        test::RotateAndTranslate(test::PolygonCircularHoles(true))}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
  }
}
TEST(OpenSwStep, ExportsCircularThroughBoresWithExactPlanarCaps) {
  for (const bool seams : {false, true}) {
    const auto result = Export(test::CubeWithCircularBore(seams));
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 7);
    EXPECT_EQ(result.value().statistics.edges, 14);
    EXPECT_EQ(result.value().statistics.solids, 1);
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, RejectsCircularHolesOutsideOrTouchingPolygonMaterial) {
  for (const Vector3 center :
       {Vector3{-2, 4, 0}, Vector3{1, 4, 0}, Vector3{1 + 1e-7, 4, 0},
        Vector3{6, 4, 0}, Vector3{4.5, 4, 0}}) {
    auto fixture = test::PolygonCircularHoles(true);
    for (auto& node : fixture.report.nodes)
      if (node.index == 14000)
        node.fields[0].values = {center.x, center.y, center.z};
    const auto result = Export(fixture);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  // A circle may enclose an entire polygon hole without crossing its edges.
  auto enclosing = test::PolygonCircularHoles(true);
  for (auto& node : enclosing.report.nodes)
    if (node.index == 14000) {
      node.fields[0].values = {6., 4., 0.};
      node.fields[3].values = {2.};
    }
  ASSERT_FALSE(Export(enclosing).ok());
}
TEST(OpenSwStep, RejectsOverlappingNestedOrIncorrectlyWoundMixedHoles) {
  for (const double radius : {1., .25}) {
    auto fixture = test::PolygonCircularHoles();
    for (auto& node : fixture.report.nodes)
      if (node.index == 14001) {
        node.fields[0].values = {3.5, 4., 0.};
        node.fields[3].values = {radius};
      }
    const auto result = Export(fixture);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidRecord);
  }
  auto wrong = test::PolygonCircularHoles();
  test::Set(wrong, 14000, "sense", '+');
  const auto outward = Export(wrong);
  ASSERT_FALSE(outward.ok());
  EXPECT_EQ(outward.error().code, ErrorCode::kInvalidRecord);
  auto negative = test::PolygonCircularHoles(true, true);
  test::Set(negative, 300, "sense", '-');
  for (auto& node : negative.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  const auto result = Export(negative);
  ASSERT_TRUE(result.ok()) << result.error().message;
}
TEST(OpenSwStep, RejectsUnsupportedAndNonplanarMixedConicHoles) {
  auto ellipse = test::PolygonCircularHoles();
  test::RemoveNode(ellipse, 14000);
  ellipse.Add(
      32, 14000,
      {test::Numbers("center", {3, 4, 0}, FieldType::kVector),
       test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
       test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
       test::Numbers("major_radius", {1}), test::Numbers("minor_radius", {.5}),
       test::ByteValue("sense", '-', FieldType::kCharacter)});
  const auto unsupported = Export(ellipse);
  ASSERT_FALSE(unsupported.ok());
  EXPECT_EQ(unsupported.error().code, ErrorCode::kUnsupportedFormat);
  auto tilted = test::PolygonCircularHoles();
  constexpr double angle = 1e-6;
  for (auto& node : tilted.report.nodes)
    if (node.index == 14000)
      node.fields[1].values = {0., std::sin(angle), std::cos(angle)};
  const auto outside = Export(tilted);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
  EXPECT_NE(outside.error().message.find("Complete conic boundary"),
            std::string::npos);
  for (const double span : {-std::numbers::pi, -4 * std::numbers::pi}) {
    auto options = Options();
    options.fin_intervals = {{NodeId{11000}, 0, span}};
    EXPECT_FALSE(Export(test::PolygonCircularHoles(), options).ok());
  }
  for (const auto center : {Vector3{6, 7.5, 0}, Vector3{3.5, 5.5, 0}}) {
    auto concave = test::PolygonCircularHoles(false, false, false, true);
    for (auto& node : concave.report.nodes)
      if (node.index == 14000)
        node.fields[0].values = {center.x, center.y, center.z};
    EXPECT_FALSE(Export(concave).ok());
  }
}
TEST(OpenSwStep, BoundsMixedHoleChecksAndParsesBinarySources) {
  auto options = Options();
  // Square self-comparisons: 2; two circles: 2*(4 distances+4 ray tests);
  // one circle-pair comparison: 1. All share the same per-body budget.
  options.max_polygon_checks = 19;
  const auto result = Export(test::PolygonCircularHoles(), options);
  ASSERT_TRUE(result.ok()) << result.error().message;
  options.max_polygon_checks = 18;
  const auto limited = Export(test::PolygonCircularHoles(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto fixture = test::Serialize(test::CubeWithCircularBore());
  const auto binary =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(binary.ok()) << binary.error().message;
  EXPECT_EQ(binary.value().statistics.solids, 1);
}
TEST(OpenSwStep, ExportsConvexLineArcAndSplitCircleBoundariesExactly) {
  for (const auto& fixture :
       {test::RoundedRectangle(), test::RoundedRectangle(true),
        test::HalfDisk(), test::CircularLens(), test::MajorCircularSegment(),
        test::SplitCircle()}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    CheckReferences(result.value());
  }
  const auto rectangle = Export(test::RoundedRectangle());
  ASSERT_TRUE(rectangle.ok());
  EXPECT_EQ(rectangle.value().statistics.vertices, 8);
  EXPECT_EQ(rectangle.value().statistics.edges, 8);
  const auto half = Export(test::HalfDisk());
  ASSERT_TRUE(half.ok());
  EXPECT_EQ(half.value().statistics.vertices, 2);
  EXPECT_EQ(half.value().statistics.edges, 2);
}
TEST(OpenSwStep, BoundsConvexCornerChecksAndRejectsConcaveOrRepeatedTurns) {
  auto options = Options();
  options.max_polygon_checks = 8;
  const auto exact = Export(test::RoundedRectangle(), options);
  ASSERT_TRUE(exact.ok()) << exact.error().message;
  options.max_polygon_checks = 7;
  const auto limited = Export(test::RoundedRectangle(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto concave = Export(test::ConcaveArcFace());
  ASSERT_FALSE(concave.ok());
  EXPECT_EQ(concave.error().code, ErrorCode::kUnsupportedFormat);
  const auto repeated = Export(test::SplitCircle(true));
  ASSERT_FALSE(repeated.ok());
  EXPECT_EQ(repeated.error().code, ErrorCode::kInvalidRecord);
}
TEST(OpenSwStep, HandlesConvexArcsOnTranslatedAndNegativelySensedPlanes) {
  auto negative = test::RoundedRectangle();
  test::Set(negative, 300, "sense", '-');
  for (auto& node : negative.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  for (const auto& fixture :
       {negative, test::RotateAndTranslate(test::RoundedRectangle())}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
  }
}
TEST(OpenSwStep, ExportsConvexEllipseArcsWithoutApproximation) {
  for (const auto& fixture :
       {test::HalfEllipse(), test::MixedConicLens(),
        test::EllipticProfile(test::RoundedRectangle()),
        test::EllipticProfile(test::RoundedRectangle(true)),
        test::EllipticProfile(test::CircularLens()),
        test::EllipticProfile(test::MajorCircularSegment()),
        test::EllipticProfile(test::SplitCircle()),
        test::RotateAndTranslate(test::HalfEllipse()),
        test::EllipticProfile(test::HalfDisk(), 100)}) {
    const auto result = Export(fixture);
    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().statistics.faces, 1);
    EXPECT_NE(result.value().text.find("=ELLIPSE("), std::string::npos);
    CheckReferences(result.value());
  }
}
TEST(OpenSwStep, ValidatesEllipseCurvaturePlaneAndSharedCornerBudget) {
  const auto concave = Export(test::EllipticProfile(test::ConcaveArcFace()));
  ASSERT_FALSE(concave.ok());
  EXPECT_EQ(concave.error().code, ErrorCode::kUnsupportedFormat);
  const auto repeated = Export(test::EllipticProfile(test::SplitCircle(true)));
  ASSERT_FALSE(repeated.ok());
  EXPECT_EQ(repeated.error().code, ErrorCode::kInvalidRecord);
  auto options = Options();
  options.max_polygon_checks = 2;
  ASSERT_TRUE(Export(test::HalfEllipse(), options).ok());
  options.max_polygon_checks = 1;
  const auto limited = Export(test::HalfEllipse(), options);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  auto negative = test::HalfEllipse();
  test::Set(negative, 300, "sense", '-');
  for (auto& node : negative.report.nodes)
    if (node.index == 700) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(negative).ok());
  auto reversed_normal = test::HalfEllipse();
  test::Set(reversed_normal, 4001, "sense", '-');
  for (auto& node : reversed_normal.report.nodes)
    if (node.index == 4001) node.fields[1].values = {0., 0., -1.};
  ASSERT_TRUE(Export(reversed_normal).ok());
  auto tilted = test::HalfEllipse();
  for (auto& node : tilted.report.nodes)
    if (node.index == 4001)
      node.fields[1].values = {0., std::sqrt(.5), std::sqrt(.5)};
  const auto outside = Export(tilted);
  ASSERT_FALSE(outside.ok());
  EXPECT_EQ(outside.error().code, ErrorCode::kInvalidRecord);
}
TEST(OpenSwStep, ParsesConvexEllipseFacesThroughTheBinaryEntryPoint) {
  const auto fixture =
      test::Serialize(test::EllipticProfile(test::CircularLens()));
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.edges, 2);
}
TEST(OpenSwStep, ParsesConvexArcFacesThroughTheBinaryEntryPoint) {
  const auto fixture = test::Serialize(test::RoundedRectangle());
  const auto result =
      ExportParasolid(fixture.bytes, NodeId{1}, Options(), &fixture.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  EXPECT_EQ(result.value().statistics.edges, 8);
}
class StepFileTest : public testing::Test {
 protected:
  std::filesystem::path root;
  void SetUp() override {
    root = std::filesystem::path(testing::TempDir()) /
           ("openswstep-" +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
  }
  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root, error);
  }
};
TEST_F(StepFileTest, ExportsModernSolidWorksContainerAndWritesStepFile) {
  const auto source = test::Serialize(test::Square());
  const auto compressed = openswx::test::DeflateZlib(source.bytes);
  std::vector<uint8_t> wrapped{0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81, 0x48, 0xa2,
                               0xa8, 0x58, 0x98, 0xb2, 0x1b, 0x89, 0xef, 0x99};
  openswx::test::AppendU32Le(&wrapped,
                             static_cast<uint32_t>(source.bytes.size()));
  openswx::test::AppendU32Le(&wrapped,
                             static_cast<uint32_t>(compressed.size()));
  wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
  const auto file =
      openswx::test::MakeModernChunkFile("SectionBodies/View-0", wrapped);
  const auto path = root / "sample.sldprt";
  {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(file.data()),
              static_cast<std::streamsize>(file.size()));
  }
  const auto result =
      ExportSolidWorksFile(path, 0, NodeId{1}, Options(), &source.catalog);
  ASSERT_TRUE(result.ok()) << result.error().message;
  const auto written = WriteStepFile(result.value(), root / "sample.step");
  ASSERT_TRUE(written.ok());
  EXPECT_EQ(written.value(), result.value().text.size());
  std::ifstream input(root / "sample.step", std::ios::binary);
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_EQ(contents, result.value().text);
  EXPECT_FALSE(
      ExportSolidWorksFile(path, 1, NodeId{1}, Options(), &source.catalog)
          .ok());
  auto options = Options();
  options.parsing.max_input_bytes = 1;
  const auto limited =
      ExportSolidWorksFile(path, 0, NodeId{1}, options, &source.catalog);
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  EXPECT_FALSE(
      WriteStepFile(result.value(), root / "missing" / "sample.step").ok());
}
}  // namespace
}  // namespace openswstep
