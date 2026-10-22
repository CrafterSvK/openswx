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
