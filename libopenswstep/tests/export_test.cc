// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openswstep/export.h"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <limits>
#include <numbers>
#include <regex>
#include <set>

#include "../../libopenswx/tests/test_helpers.h"
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
TEST(OpenSwStep, RejectsCurvedFaceEdgesWithoutApproximation) {
  auto f = test::Polyhedron({{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}, {{0, 1, 2}});
  test::RemoveNode(f, 4000);
  f.Add(31, 4000,
        {test::Numbers("center", {0, 0, 0}, FieldType::kVector),
         test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
         test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
         test::Numbers("radius", {1}),
         test::ByteValue("sense", '+', FieldType::kCharacter)});
  const auto result = Export(f);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kUnsupportedFormat);
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
