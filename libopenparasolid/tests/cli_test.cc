// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "../tools/body_report.h"
#include "../tools/boundary_geometry_report.h"
#include "../tools/model_report.h"
#include "../tools/schema_report.h"
#include "../tools/topology_report.h"
#include "model_test_helpers.h"
#include "test_helpers.h"

namespace openparasolid {
namespace {
namespace fs = std::filesystem;
using nlohmann::json;

TEST(ParasolidModelReport, RetainsTopologySenseAlongsideGeometrySense) {
  test::ModelFixture f;
  f.Add(14, 1, {test::ByteValue("sense", '?', FieldType::kCharacter)});
  f.Add(17, 2, {test::ByteValue("sense", '-', FieldType::kCharacter)});
  f.Point(3);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto report = cli::ModelReport(model.value(), {}, false);
  EXPECT_EQ(report["nodes"][0]["topology_sense"], uint64_t{'?'});
  EXPECT_EQ(report["nodes"][1]["topology_sense"], uint64_t{'-'});
  EXPECT_TRUE(report["nodes"][2]["topology_sense"].is_null());
  EXPECT_TRUE(report["geometry"][0]["sense"].is_null());
  EXPECT_FALSE(cli::ModelReport(model.value(), {}, true).contains("nodes"));
}

TEST(ParasolidModelReport, RetainsRawRegionTypesWithoutInferringOtherNodes) {
  test::ModelFixture f;
  f.Add(19, 1, {test::ByteValue("type", 'V', FieldType::kCharacter)});
  f.Add(19, 2, {test::ByteValue("type", '?', FieldType::kCharacter)});
  f.Point(3);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto report = cli::ModelReport(model.value(), {}, false);
  EXPECT_EQ(report["nodes"][0]["region_type"], uint64_t{'V'});
  EXPECT_EQ(report["nodes"][1]["region_type"], uint64_t{'?'});
  EXPECT_TRUE(report["nodes"][2]["region_type"].is_null());
  EXPECT_FALSE(cli::ModelReport(model.value(), {}, true).contains("nodes"));
}

TEST(ParasolidModelReport, RetainsRawBodyTypesWithoutClassifyingOtherNodes) {
  test::ModelFixture f;
  f.Add(12, 1, {test::ByteValue("body_type", 6)});
  f.Add(12, 2, {test::ByteValue("body_type", 99)});
  f.Point(3);
  const auto model = ModelBuilder::Build(f.report);
  ASSERT_TRUE(model.ok());
  const auto report = cli::ModelReport(model.value(), {}, false);
  EXPECT_EQ(report["nodes"][0]["body_type"], 6);
  EXPECT_EQ(report["nodes"][1]["body_type"], 99);
  EXPECT_TRUE(report["nodes"][2]["body_type"].is_null());
  EXPECT_FALSE(cli::ModelReport(model.value(), {}, true).contains("nodes"));
}

TEST(ParasolidBodyReport, KeepsCountsSettingsAndDirectedBindingData) {
  BodyGeometryBinding body;
  body.topology.boundary.body = NodeId{1};
  body.wires.push_back(
      {{NodeId{2}, NodeId{100}, NodeId{101}, NodeId{400}}, 1, -1});
  body.vertices.push_back({NodeId{100}, NodeId{200}, {1, -2, 3}});
  FaceTrimming face;
  face.face = NodeId{30};
  face.surface = NodeId{300};
  face.face_sense = '-';
  FaceTrimLoop loop;
  loop.loop = NodeId{20};
  FinUse use;
  use.fin = NodeId{10};
  use.edge = NodeId{2};
  use.edge_curve = NodeId{400};
  use.sense = TopologySense::kReverse;
  loop.fins.push_back({use, NodeId{30}, NodeId{300}, NodeId{400}, 1, -1});
  face.loops.push_back(loop);
  FaceTrimLoop isolated;
  isolated.loop = NodeId{21};
  isolated.isolated_vertex = SurfaceParameters{2, 3, 0};
  face.loops.push_back(isolated);
  body.faces.push_back(face);
  BodyGeometryBindingOptions options;
  options.intersection = IntersectionEvaluationOptions{1e-7, 1e-9};
  const auto report = cli::BodyGeometryReport(body, 1e-7, options, false);
  EXPECT_EQ(report["body"], 1);
  EXPECT_EQ(report["counts"]["faces"], 1);
  EXPECT_EQ(report["counts"]["loops"], 2);
  EXPECT_EQ(report["counts"]["fins"], 1);
  EXPECT_EQ(report["counts"]["wire_edges"], 1);
  EXPECT_EQ(report["vertices"][0]["position"], json::array({1, -2, 3}));
  EXPECT_EQ(report["wires"][0]["start"], 1);
  EXPECT_EQ(report["wires"][0]["end"], -1);
  EXPECT_EQ(report["faces"][0]["face_sense"], uint64_t{'-'});
  EXPECT_EQ(report["faces"][0]["loops"][1]["isolated_vertex_uv"]["u"], 2);
  EXPECT_EQ(report["faces"][0]["loops"][0]["fins"][0]["end"], -1);
  EXPECT_EQ(report["faces"][0]["loops"][0]["fins"][0]["sense"], "reverse");
  EXPECT_EQ(report["intersection"]["solve_tolerance"], 1e-9);
  const auto summary = cli::BodyGeometryReport(body, 1e-7, options, true);
  EXPECT_EQ(summary["counts"], report["counts"]);
  EXPECT_FALSE(summary.contains("faces"));
  EXPECT_FALSE(summary.contains("vertices"));
}
TEST(ParasolidBodyTopologyReport, RetainsRadialVertexAndShellIncidence) {
  ValidatedBodyTopology topology;
  topology.boundary.body = NodeId{1};
  topology.boundary.kind = BodyKind::kSheet;
  RegionBoundary region;
  region.region = NodeId{2};
  region.body = NodeId{1};
  ShellBoundary shell;
  shell.shell = NodeId{3};
  FaceBoundary face;
  face.face = NodeId{4};
  face.surface = NodeId{5};
  face.face_sense = '-';
  LoopBoundary loop;
  loop.loop = NodeId{6};
  const FinUse fin{NodeId{7},
                   NodeId{8},
                   NodeId{9},
                   NodeId{10},
                   NodeId{11},
                   NodeId{12},
                   TopologySense::kReverse};
  loop.fins.push_back(fin);
  face.loops.push_back(loop);
  LoopBoundary isolated;
  isolated.loop = NodeId{13};
  isolated.isolated_vertex = IsolatedLoopVertex{NodeId{14}, NodeId{9}};
  face.loops.push_back(isolated);
  shell.faces.push_back(face);
  shell.face_uses = {{0, ShellFaceSide::kBack}, {0, ShellFaceSide::kFront}};
  shell.wire_edges.push_back({NodeId{15}, NodeId{9}, NodeId{10}, NodeId{11}});
  shell.isolated_vertex = IsolatedShellVertex{NodeId{16}, NodeId{17}};
  region.shells.push_back(shell);
  topology.boundary.regions.push_back(region);
  topology.edges.push_back({NodeId{8},
                            NodeId{9},
                            NodeId{10},
                            NodeId{11},
                            {{fin, NodeId{6}}, {fin, NodeId{}}}});
  topology.vertices.push_back(
      {NodeId{9},
       NodeId{1},
       NodeId{18},
       {{NodeId{7}, NodeId{6}, fin}, {NodeId{14}, NodeId{13}, std::nullopt}}});
  const auto full_result = cli::BodyTopologyReport(topology, false);
  ASSERT_TRUE(full_result.ok()) << full_result.error().message;
  const auto& report = full_result.value();
  EXPECT_EQ(report["kind"], "sheet");
  EXPECT_EQ(report["counts"]["faces"], 1);
  EXPECT_EQ(report["counts"]["face_uses"], 2);
  EXPECT_EQ(report["counts"]["radial_fins"], 2);
  const auto s = report["regions"][0]["shells"][0];
  EXPECT_EQ(s["face_uses"][1]["side"], "front");
  EXPECT_EQ(s["faces"][0]["loops"][0]["fins"][0]["fin_curve"], 12);
  EXPECT_EQ(s["faces"][0]["loops"][1]["isolated_vertex"]["fin"], 14);
  EXPECT_EQ(s["isolated_vertex"]["point"], 17);
  EXPECT_EQ(s["wires"][0]["start_vertex"], 9);
  EXPECT_EQ(report["edges"][0]["fins"][0]["sense"], "reverse");
  EXPECT_EQ(report["edges"][0]["fins"][1]["loop"], 0);
  EXPECT_TRUE(report["vertices"][0]["fins"][1]["edge_use"].is_null());
  const auto summary_result = cli::BodyTopologyReport(topology, true);
  ASSERT_TRUE(summary_result.ok());
  const auto& summary = summary_result.value();
  EXPECT_EQ(report["shell_edge_incidence"]["boundary_edges"], 1);
  EXPECT_EQ(report["shell_edge_incidence"]["face_fins"], 1);
  EXPECT_EQ(report["shell_edge_incidence"]["face_components"], 1);
  EXPECT_EQ(s["edge_incidence"]["components"][0]["faces"], json::array({4}));
  const auto component = s["edge_incidence"]["components"][0];
  EXPECT_EQ(component["edges"], json::array({8}));
  EXPECT_EQ(component["counts"], (json{{"boundary_edges", 1},
                                       {"paired_edges", 0},
                                       {"higher_valence_edges", 0},
                                       {"face_fins", 1},
                                       {"isolated_loops", 1},
                                       {"boundary_cycles", 0},
                                       {"boundary_chains", 1},
                                       {"branched_boundaries", 0}}));
  EXPECT_EQ(component["boundary_groups"][0]["kind"], "chain");
  EXPECT_EQ(component["boundary_groups"][0]["edges"], json::array({8}));
  EXPECT_EQ(component["boundary_groups"][0]["end_vertices"],
            json::array({9, 10}));
  EXPECT_EQ(component["boundary_groups"][0]["path"],
            (json::array({{{"edge", 8},
                           {"fin", 7},
                           {"start_vertex", 9},
                           {"end_vertex", 10},
                           {"sense", "reverse"}}})));
  EXPECT_EQ(report["shell_edge_incidence"]["boundary_chains"], 1);
  EXPECT_FALSE(summary["shell_edge_incidence"].contains("components"));
  EXPECT_EQ(report["shell_edge_incidence"]["isolated_loops"], 1);
  EXPECT_EQ(s["edge_incidence"]["edges"][0]["edge"], 8);
  EXPECT_EQ(s["edge_incidence"]["edges"][0]["classification"], "boundary");
  EXPECT_EQ(summary["shell_edge_incidence"], report["shell_edge_incidence"]);

  EXPECT_EQ(summary["counts"], report["counts"]);
  EXPECT_FALSE(summary.contains("regions"));
  EXPECT_FALSE(summary.contains("edges"));
  EXPECT_FALSE(summary.contains("vertices"));
}
TEST(ParasolidBoundaryReport, RetainsDirectedIntervalsLocationsAndBudgets) {
  test::ModelFixture fixture;
  fixture.Add(30, 400,
              {test::Numbers("origin", {0, 0, 0}, FieldType::kVector),
               test::Numbers("direction", {1, 0, 0}, FieldType::kVector),
               test::ByteValue("sense", '+', FieldType::kCharacter)});
  fixture.Add(50, 300,
              {test::Numbers("origin", {0, 0, 0}, FieldType::kVector),
               test::Numbers("normal", {0, 0, 1}, FieldType::kVector),
               test::Numbers("x_axis", {1, 0, 0}, FieldType::kVector),
               test::ByteValue("sense", '+', FieldType::kCharacter)});
  const auto model = ModelBuilder::Build(fixture.report);
  ASSERT_TRUE(model.ok());
  const FinUse use{NodeId{10},
                   NodeId{2},
                   NodeId{101},
                   NodeId{100},
                   NodeId{400},
                   NodeId{},
                   TopologySense::kReverse};
  BodyGeometryBinding body;
  body.topology.boundary.body = NodeId{1};
  RegionBoundary region;
  region.region = NodeId{2};
  ShellBoundary shell;
  shell.shell = NodeId{3};
  FaceBoundary face;
  face.face = NodeId{30};
  LoopBoundary loop;
  loop.fins = {use};
  face.loops = {loop};
  shell.faces = {face};
  region.shells = {shell};
  body.topology.boundary.regions = {region};
  FaceTrimming geometry;
  geometry.face = NodeId{30};
  geometry.surface = NodeId{300};
  FaceTrimLoop trim;
  trim.fins.push_back({use, NodeId{30}, NodeId{300}, NodeId{400}, 1, 0});
  geometry.loops = {trim};
  body.faces = {geometry};
  const auto full = cli::BoundaryGeometryReport(model.value(), body, 0, false);
  ASSERT_TRUE(full.ok()) << full.error().message;
  EXPECT_EQ(full.value()["counts"],
            (json{{"paths", 1}, {"cycles", 0}, {"chains", 1}, {"edges", 1}}));
  const auto path = full.value()["paths"][0];
  EXPECT_EQ(path["region"], 2);
  EXPECT_EQ(path["shell"], 3);
  EXPECT_EQ(path["component"], 0);
  EXPECT_EQ(path["group"], 0);
  const auto edge = path["edges"][0];
  EXPECT_EQ(edge["fin"], 10);
  EXPECT_EQ(edge["face"], 30);
  EXPECT_EQ(edge["surface"], 300);
  EXPECT_EQ(edge["curve"], 400);
  EXPECT_EQ(edge["start"], 0);
  EXPECT_EQ(edge["end"], 1);
  EXPECT_EQ(edge["sense"], "forward");
  EXPECT_EQ(edge["source_sense"], "reverse");
  EXPECT_EQ(edge["reversed"], true);
  EXPECT_TRUE(edge["intersection"].is_null());
  const auto summary =
      cli::BoundaryGeometryReport(model.value(), body, 0, true);
  ASSERT_TRUE(summary.ok());
  EXPECT_EQ(summary.value()["counts"], full.value()["counts"]);
  EXPECT_FALSE(summary.value().contains("paths"));
  auto repeated = region;
  repeated.region = NodeId{4};
  repeated.shells[0].shell = NodeId{5};
  body.topology.boundary.regions.push_back(repeated);
  const auto limited = cli::BoundaryGeometryReport(model.value(), body, 0,
                                                   false, {1, 1, 1, 1, 2});
  ASSERT_FALSE(limited.ok());
  EXPECT_EQ(limited.error().code, ErrorCode::kResourceLimit);
  const auto invalid = cli::BoundaryGeometryReport(model.value(), body, 0,
                                                   false, {1, 1, 1, 0, 2});
  ASSERT_FALSE(invalid.ok());
  EXPECT_EQ(invalid.error().code, ErrorCode::kInvalidOptions);
  const auto multi = cli::BoundaryGeometryReport(model.value(), body, 0, false);
  ASSERT_TRUE(multi.ok());
  EXPECT_EQ(multi.value()["paths"][1]["region"], 4);
  EXPECT_EQ(multi.value()["paths"][1]["shell"], 5);
  auto& fins =
      body.topology.boundary.regions[0].shells[0].faces[0].loops[0].fins;
  fins[0].start_vertex = NodeId{100};
  fins[0].end_vertex = NodeId{101};
  auto extra = fins[0];
  extra.edge = NodeId{4};
  extra.end_vertex = NodeId{102};
  fins.push_back(extra);
  extra.edge = NodeId{5};
  extra.end_vertex = NodeId{103};
  fins.push_back(extra);
  const auto branched =
      cli::BoundaryGeometryReport(model.value(), body, 0, false);
  ASSERT_FALSE(branched.ok());
  EXPECT_EQ(branched.error().code, ErrorCode::kUnsupportedFormat);
}
std::string Quote(const fs::path& path) {
  std::string result = "'";
  for (char c : path.string())
    result += c == '\'' ? "'\"'\"'" : std::string(1, c);
  return result + "'";
}

class ParasolidInspectCliTest : public testing::Test {
 protected:
  fs::path root;
  void SetUp() override {
    root = fs::path(testing::TempDir()) /
           ("parasolid inspect ' " +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
    const auto bytes = test::LinkedRecords();
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(file.good());
  }
  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }
  int Run(const std::string& arguments = "") {
    const auto status = std::system(
        (Quote(PARASOLID_INSPECT_EXECUTABLE) + " " + arguments + " " +
         Quote(root / "input.x_b") + " > " + Quote(root / "report.json") +
         " 2> " + Quote(root / "stderr.txt"))
            .c_str());
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }
  json Report() {
    std::ifstream input(root / "report.json");
    return json::parse(input);
  }
};

TEST_F(ParasolidInspectCliTest,
       BodyModeRequiresCompleteStreamsAndReportsFailures) {
  ASSERT_EQ(Run("--body-geometry 1 --binding-tolerance 1e-7 --summary"), 2);
  const auto report = Report();
  EXPECT_TRUE(report["body_geometry"].is_null());
  EXPECT_EQ(report["body_geometry_error"]["body"], 1);
  EXPECT_FALSE(
      report["body_geometry_error"]["message"].get<std::string>().empty());
  ASSERT_EQ(Run("--body-topology=1 --summary"), 2);
  EXPECT_TRUE(Report()["body_topology"].is_null());
  EXPECT_EQ(Report()["body_topology_error"]["body"], 1);
  ASSERT_EQ(Run("--body-geometry=1 --binding-tolerance=0 --boundary-geometry"), 2);
  EXPECT_TRUE(Report()["boundary_geometry"].is_null());
  EXPECT_EQ(Report()["boundary_geometry_error"]["body"], 1);
  EXPECT_EQ(Run("--body-geometry 1"), 1);
  EXPECT_EQ(Run("--body-geometry 0 --binding-tolerance 0"), 1);
}
TEST_F(ParasolidInspectCliTest,
       BodyModeBindsCompleteEmptyBodyAndReportsMissingId) {
  auto bytes = test::Header();
  test::U16(bytes, 12);
  bytes.push_back(255);
  test::CompactIndex(bytes, 1);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back(2);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  test::ModelFixture fixture;
  fixture.Add(
      12, 1,
      {test::Pointer("owner", 0, 1040), test::Pointer("region", 0, 19),
       test::Pointer("child", 0, 12), test::Pointer("edge", 0, 16),
       test::Pointer("vertex", 0, 18), test::ByteValue("body_type", 2)});
  json fields = json::array();
  for (const auto& field : fixture.report.schemas[0].definition.fields)
    fields.push_back(cli::DefinitionJson(field));
  const auto path = root / "body-catalog.json";
  json catalog{{"schema_key", "SCH_1234567_12345_13006"},
               {"source", "synthetic"},
               {"base_types",
                {{{"node_type", 12}, {"name", "body"}, {"fields", fields}}}},
               {"absent_types", json::array()}};
  std::ofstream(path) << catalog;
  const auto arguments =
      "--schema-catalog " + Quote(path) + " --binding-tolerance=0";
  ASSERT_EQ(Run(arguments + " --body-geometry=1"), 0);
  EXPECT_EQ(Report()["body_geometry"]["counts"]["vertices"], 0);
  EXPECT_TRUE(Report()["body_geometry"]["faces"].empty());
  ASSERT_EQ(Run(arguments + " --body-geometry=1 --summary"), 0);
  EXPECT_FALSE(Report()["body_geometry"].contains("faces"));
  ASSERT_EQ(Run(arguments + " --body-geometry=99"), 2);
  EXPECT_TRUE(Report()["body_geometry"].is_null());
  EXPECT_EQ(Report()["body_geometry_error"]["body"], 99);
  ASSERT_EQ(Run(arguments + " --body-geometry=1 --boundary-geometry --summary"), 0);
  EXPECT_TRUE(Report()["body_geometry"].is_object());
  EXPECT_EQ(Report()["boundary_geometry"]["counts"]["paths"], 0);
  EXPECT_FALSE(Report()["boundary_geometry"].contains("paths"));
  ASSERT_EQ(Run(arguments + " --body-geometry=99 --boundary-geometry"), 2);
  EXPECT_TRUE(Report()["boundary_geometry"].is_null());
  EXPECT_EQ(Report()["boundary_geometry_error"]["body"], 99);

  // Valid syntax does not bypass body membership checks. Requests remain
  // inspectable when binding fails, with summary output retaining counts only.
  const auto overrides =
      arguments +
      " --body-geometry=1 --fin-intervals=7:1:0 --wire-intervals=8:-2:3";
  ASSERT_EQ(Run(overrides), 2);
  EXPECT_TRUE(Report()["body_geometry"].is_null());
  EXPECT_TRUE(Report().contains("body_geometry_error"));
  const auto request = Report()["body_geometry_overrides"];
  EXPECT_EQ(request["fins"][0], (json{{"fin", 7}, {"start", 1}, {"end", 0}}));
  EXPECT_EQ(request["wires"][0],
            (json{{"edge", 8}, {"start", -2}, {"end", 3}}));
  ASSERT_EQ(Run(overrides + " --summary"), 2);
  EXPECT_EQ(Report()["body_geometry_overrides"],
            (json{{"fin_count", 1}, {"wire_count", 1}}));
  EXPECT_EQ(Run(arguments + " --body-geometry=1 --fin-intervals=7:0:0"), 1);
  ASSERT_EQ(Run(arguments + " --body-topology=1 --body-geometry=99"), 2);
  EXPECT_TRUE(Report()["body_topology"].is_object());
  EXPECT_TRUE(Report()["body_geometry"].is_null());
  ASSERT_EQ(Run("--schema-catalog " + Quote(path) + " --body-topology=1"), 0);
  EXPECT_EQ(Report()["body_topology"]["kind"], "wire");
  EXPECT_TRUE(Report()["body_topology"]["edges"].empty());
  ASSERT_EQ(
      Run("--schema-catalog " + Quote(path) + " --body-topology=1 --summary"),
      0);
  EXPECT_FALSE(Report()["body_topology"].contains("edges"));
  ASSERT_EQ(Run(arguments + " --body-topology=99 --body-geometry=1"), 2);
  EXPECT_TRUE(Report()["body_geometry"].is_object());
  EXPECT_TRUE(Report()["body_topology"].is_null());
  EXPECT_EQ(Report()["body_topology_error"]["body"], 99);
  // An unrelated untyped curve keeps the model's diagnostic exit status,
  // while the selected empty body still binds successfully.
  bytes.resize(bytes.size() - 4);
  test::U16(bytes, 30);
  bytes.push_back(255);
  test::CompactIndex(bytes, 2);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  catalog["base_types"].push_back(
      {{"node_type", 30}, {"name", "untyped_line"}, {"fields", json::array()}});
  std::ofstream(path) << catalog;
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run(arguments + " --body-geometry=1 --summary"), 2);
  EXPECT_TRUE(Report()["body_geometry"].is_object());
  EXPECT_EQ(Report()["model"]["invalid_geometry_count"], 1);
  ASSERT_EQ(
      Run("--schema-catalog " + Quote(path) + " --body-topology=1 --summary"),
      2);
  EXPECT_TRUE(Report()["body_topology"].is_object());
}
TEST_F(ParasolidInspectCliTest, WritesCandidateReferencesAndUnknownRanges) {
  ASSERT_EQ(Run(), 0);
  const auto report = Report();
  EXPECT_EQ(report["record_candidate_count"], 6);
  EXPECT_EQ(report["interpretation"], "fixed_layout_candidates");
  EXPECT_EQ(report["header"]["schema_version"]["base_schema"], 13006);
  EXPECT_EQ(report["header"]["user_field_words"], 0);
  EXPECT_EQ(report["header"]["internal_header_complete"], true);
  EXPECT_EQ(report["candidates"][0]["references"][2]["target_candidate"], 1);
  EXPECT_EQ(report["uninterpreted_bytes"], 10);
  EXPECT_EQ(report["unknown_regions"][0]["preview_hex"], std::string(20, '0'));
}

TEST_F(ParasolidInspectCliTest, SupportsSummaryAndExplicitLimitedExitStatus) {
  ASSERT_EQ(Run("--summary"), 0);
  EXPECT_FALSE(Report().contains("candidates"));
  EXPECT_EQ(Report()["record_candidate_count"], 6);
  ASSERT_EQ(Run("--max-records 1"), 2);
  EXPECT_EQ(Report()["record_scan_complete"], false);
  EXPECT_EQ(Report()["record_candidate_count"], 1);
}

TEST_F(ParasolidInspectCliTest, RejectsInvalidInputAndLimits) {
  EXPECT_EQ(Run("--max-records -1"), 1);
  EXPECT_EQ(Run("--max-records 1000001"), 1);
  std::ofstream(root / "input.x_b", std::ios::binary)
      << "not a binary transmit stream";
  EXPECT_EQ(Run(), 1);
  EXPECT_EQ(fs::file_size(root / "report.json"), 0u);
}

TEST_F(ParasolidInspectCliTest, NodeModeReportsUnknownMembership) {
  ASSERT_EQ(Run("--nodes --summary"), 2);
  const auto report = Report();
  EXPECT_EQ(report["interpretation"], "schema_nodes");
  EXPECT_EQ(report["node_count"], 0);
  EXPECT_EQ(report["diagnostics"][0]["node_type"], 14);
  EXPECT_FALSE(report.contains("nodes"));
}

TEST_F(ParasolidInspectCliTest, CatalogFailuresAreExplicitResults) {
  const auto path = root / "catalog.json";
  auto result = cli::LoadCatalog(path);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.error().code, ErrorCode::kIo);
  for (const auto& text :
       {std::string("{"), std::string("{}"), std::string("[]")}) {
    std::ofstream(path) << text;
    result = cli::LoadCatalog(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidSchema);
  }
  json catalog{{"schema_key", "SCH_1234567_12345_13006"},
               {"source", "synthetic"},
               {"base_types", json::array()},
               {"absent_types", json::array()}};
  const auto valid = catalog;
  for (const auto& invalid : {json(-1), json(32768), json(1.5), json("12")}) {
    catalog = valid;
    catalog["base_types"].push_back(
        {{"node_type", invalid}, {"name", "bad"}, {"fields", json::array()}});
    std::ofstream(path) << catalog;
    result = cli::LoadCatalog(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidSchema);
    EXPECT_EQ(Run("--schema-catalog " + Quote(path)), 1);
  }
  json type{{"node_type", 12}, {"name", "sample"}, {"fields", json::array()}};
  json field{{"name", "sample"},
             {"type", "d"},
             {"pointer_class", 0},
             {"elements", 0},
             {"transmitted", true}};
  for (const auto& [key, bad_value] : std::vector<std::pair<std::string, json>>{
           {"type", "?"},
           {"name", 3},
           {"pointer_class", 65536},
           {"elements", uint64_t{UINT32_MAX} + 1},
           {"transmitted", "true"}}) {
    auto bad_field = field;
    bad_field[key] = bad_value;
    type["fields"] = json::array({bad_field});
    catalog = valid;
    catalog["base_types"].push_back(type);
    std::ofstream(path) << catalog;
    result = cli::LoadCatalog(path);
    ASSERT_FALSE(result.ok()) << key;
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidSchema);
  }
  for (const auto& invalid : {json(-1), json(32768), json("12")}) {
    catalog = valid;
    catalog["absent_types"] = json::array({invalid});
    std::ofstream(path) << catalog;
    result = cli::LoadCatalog(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error().code, ErrorCode::kInvalidSchema);
  }
  std::ofstream(path) << valid;
  result = cli::LoadCatalog(path);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().schema_key, valid["schema_key"].get<std::string>());
}

TEST_F(ParasolidInspectCliTest,
       CatalogNestingBoundsIgnoreEscapedStringContents) {
  const auto path = root / "catalog.json";
  json catalog{
      {"schema_key", "SCH_1234567_12345_13006"},
      {"source", std::string(1000, '[') + "\\\"" + std::string(1000, '}')},
      {"base_types", json::array()},
      {"absent_types", json::array()}};
  std::ofstream(path) << catalog;
  const auto result = cli::LoadCatalog(path);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().source, catalog["source"].get<std::string>());
  // The old callback permits empty containers at depth 64; a leaf there
  // would be at depth 65. Bound the input before entering recursive JSON code.
  EXPECT_TRUE(
      cli::CatalogNestingAllowed(std::string(65, '[') + std::string(65, ']')));
  EXPECT_FALSE(cli::CatalogNestingAllowed(std::string(65, '[') + "0" +
                                          std::string(65, ']')));
  std::ofstream(path) << std::string(100000, '[');
  const auto deep = cli::LoadCatalog(path);
  ASSERT_FALSE(deep.ok());
  EXPECT_EQ(deep.error().code, ErrorCode::kResourceLimit);
}

TEST_F(ParasolidInspectCliTest,
       NodeModeUsesExactCatalogAndValidatesCompletion) {
  auto bytes = test::Header();
  test::U16(bytes, 12);
  bytes.push_back(255);  // unchanged empty base definition
  test::U16(bytes, 2);   // node index 1
  test::U16(bytes, 1);   // terminator
  test::U16(bytes, 1);   // index 0
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  const auto path = root / "catalog.json";
  json catalog{
      {"schema_key", "SCH_1234567_12345_13006"},
      {"source", "synthetic"},
      {"base_types",
       {{{"node_type", 12}, {"name", "empty"}, {"fields", json::array()}}}},
      {"absent_types", json::array()}};
  std::ofstream(path) << catalog;
  ASSERT_EQ(Run("--schema-catalog " + Quote(path)), 0);
  EXPECT_EQ(Report()["complete"], true);
  EXPECT_EQ(Report()["nodes"][0]["index"], 1);
  catalog["schema_key"] = "SCH_1_2_13006";
  std::ofstream(path) << catalog;
  EXPECT_EQ(Run("--schema-catalog " + Quote(path)), 1);
  std::ofstream(path) << std::string(100, '[');
  EXPECT_EQ(Run("--schema-catalog " + Quote(path)), 1);
  EXPECT_EQ(Run("--nodes --max-records 0"), 1);
}

TEST_F(ParasolidInspectCliTest, ReportsValidatedDeltaPrefixAndUnsupportedTail) {
  auto bytes = test::Header("(deltas) ");
  test::U16(bytes, 3);
  bytes.push_back(255);
  test::U16(bytes, 3);  // partition index 2
  test::U16(bytes, 6);  // current PMARK index 5
  test::U32(bytes, 0xffffffff);
  test::U16(bytes, 4);
  bytes.push_back(255);
  test::U16(bytes, 6);
  for (int i = 0; i < 4; ++i) test::U16(bytes, 1);
  for (uint32_t value : {1u, 2u, 3u, 65535u}) test::U32(bytes, value);
  bytes.push_back(0);
  test::U32(bytes, 0xffff8004);
  const auto end = bytes.size();
  bytes.insert(bytes.end(),
               {0, 1, 0, 1, 0, 37, 0, 81, 0, 1, 0, 0, 0, 1, 0, 200});
  auto write = [&] {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  };
  write();
  ASSERT_EQ(Run("--nodes --summary"), 2);
  auto report = Report();
  EXPECT_EQ(report["node_count"], 2);
  EXPECT_EQ(report["delta_prefix"]["partition"]["index"], 2);
  EXPECT_EQ(report["delta_prefix"]["partition"]["current_pmark"], 5);
  EXPECT_EQ(report["delta_prefix"]["partition"]["highest_id"], 0xffffffffu);
  EXPECT_EQ(report["delta_prefix"]["current_mark"]["identifier"], 0xffff8004u);
  EXPECT_EQ(report["delta_prefix"]["current_mark"]["delta_is_forward"], false);
  EXPECT_EQ(report["diagnostics"][0]["code"], "unsupported_schema");
  EXPECT_EQ(report["uninterpreted_offset"], end + 14);
  EXPECT_EQ(report["delta_packet_count"], 1);
  EXPECT_EQ(report["delta_reference_count"], 1);
  EXPECT_FALSE(report.contains("delta_packets"));
  ASSERT_EQ(Run("--nodes"), 2);
  EXPECT_EQ(Report()["delta_packets"][0]["references"][0]["index"], 36);
  EXPECT_FALSE(report.contains("nodes"));
  bytes.resize(end + 4);
  write();
  ASSERT_EQ(Run("--nodes"), 0);
  report = Report();
  EXPECT_EQ(report["complete"], true);
  EXPECT_EQ(report["nodes"][0]["node_type"], 3);
  EXPECT_EQ(report["nodes"][1]["node_type"], 4);
}

TEST_F(ParasolidInspectCliTest,
       ExposesDeltaOccurrencesAndPointerStatusesInJson) {
  auto bytes = test::DeltaPrefixBytes();
  for (int occurrence = 0; occurrence < 2; ++occurrence) {
    test::U16(bytes, 15);
    if (!occurrence) bytes.push_back(255);
    test::CompactIndex(bytes, 7);
    test::U32(bytes, 0xffffffff);
    for (int i = 0; i < 4; ++i) {
      test::CompactIndex(bytes, 0);
      bytes.push_back(i % 2);
    }
  }
  test::U16(bytes, 1);
  test::U16(bytes, 1);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--nodes"), 0);
  const auto report = Report();
  EXPECT_EQ(report["node_count"], 2);
  EXPECT_EQ(report["delta_node_count"], 2);
  EXPECT_EQ(report["delta_nodes"][0]["index"], 7);
  EXPECT_EQ(report["delta_nodes"][1]["index"], 7);
  EXPECT_EQ(report["delta_nodes"][0]["fields"][0]["values"][0], 0xffffffffu);
  EXPECT_EQ(report["delta_nodes"][0]["fields"][1]["reference_statuses"][0], 0);
  EXPECT_EQ(report["delta_nodes"][0]["fields"][2]["reference_statuses"][0], 1);
  ASSERT_EQ(Run("--nodes --summary"), 0);
  EXPECT_EQ(Report()["delta_node_count"], 2);
  EXPECT_FALSE(Report().contains("delta_nodes"));
}

TEST_F(ParasolidInspectCliTest, HistoricalTerminalZeroRemainsNumericInJson) {
  auto bytes = test::DeltaPrefixBytes();
  const auto start = bytes.size();
  bytes.insert(bytes.end(), {0, 1, 0, 1});
  test::U16(bytes, 4);
  test::CompactIndex(bytes, 7);
  for (int i = 0; i < 4; ++i) test::CompactIndex(bytes, 0);
  for (int i = 0; i < 4; ++i) test::U32(bytes, 0);
  bytes.push_back(0);
  test::U32(bytes, 123);
  bytes.insert(bytes.end(), {0, 1, 0, 1, 0, 1, 0, 0, 0, 0});
  const auto end = bytes.size();
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--nodes"), 0);
  const auto report = Report();
  EXPECT_EQ(report["delta_packet_count"], 1);
  EXPECT_EQ(report["delta_historical_mark_count"], 1);
  const auto& packet = report["delta_packets"][0];
  EXPECT_EQ(packet["kind"], "historical_marks");
  EXPECT_EQ(packet["has_terminal"], true);
  EXPECT_EQ(packet["terminal_word"], 0);
  EXPECT_EQ(packet["offset"], start);
  EXPECT_EQ(packet["size"], end - start);
  EXPECT_EQ(report["terminator_offset"], end);
}

TEST_F(ParasolidInspectCliTest, ExplicitSchemaLimitsBoundAndReleaseDecoding) {
  auto bytes = test::Header();
  test::U16(bytes, 12);
  bytes.push_back(255);
  test::CompactIndex(bytes, 1);
  test::U32(bytes, 42);
  const auto second = bytes.size();
  test::U16(bytes, 12);
  test::CompactIndex(bytes, 2);
  test::U32(bytes, 43);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  const auto path = root / "catalog.json";
  json catalog{{"schema_key", "SCH_1234567_12345_13006"},
               {"source", "synthetic"},
               {"base_types",
                {{{"node_type", 12},
                  {"name", "sample"},
                  {"fields",
                   {{{"name", "value"},
                     {"type", "d"},
                     {"pointer_class", 0},
                     {"elements", 0},
                     {"transmitted", true}}}}}}},
               {"absent_types", json::array()}};
  std::ofstream(path) << catalog;
  const auto args = "--schema-catalog " + Quote(path);
  for (const auto& bound :
       {"--max-fields 1", "--max-values 1", "--max-records 1"}) {
    ASSERT_EQ(Run(args + " " + bound), 2);
    const auto r = Report();
    EXPECT_EQ(r["node_count"], 1);
    EXPECT_EQ(r["decoded_end"], second);
    EXPECT_EQ(r["diagnostics"][0]["code"], "resource_limit");
  }
  ASSERT_EQ(
      Run(args + " --max-fields 2 --max-values 2 --max-records 2 --summary"),
      0);
  const auto r = Report();
  EXPECT_EQ(r["node_count"], 2);
  EXPECT_EQ(r["limits"]["max_total_fields"], 2);
  EXPECT_EQ(r["limits"]["max_values"], 2);
  EXPECT_EQ(r["limits"]["max_nodes"], 2);
  EXPECT_EQ(r["limits"]["max_delta_entries"], 100000);
}
TEST_F(ParasolidInspectCliTest, ExplicitDeltaEntryLimitIsReportedAndEnforced) {
  auto bytes = test::DeltaPrefixBytes();
  const auto packet = bytes.size();
  bytes.insert(bytes.end(), {0,  1, 0, 1, 0, 9, 0, 81, 0, 10, 0,
                             84, 0, 1, 0, 0, 0, 1, 0,  1, 0,  1});
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--nodes --max-delta-entries 1"), 2);
  EXPECT_EQ(Report()["decoded_end"], packet);
  EXPECT_EQ(Report()["delta_packet_count"], 0);
  EXPECT_EQ(Report()["limits"]["max_delta_entries"], 1);
  ASSERT_EQ(Run("--max-delta-entries 2 --nodes"), 0);
  EXPECT_EQ(Report()["delta_reference_count"], 2);
  EXPECT_EQ(Report()["limits"]["max_delta_entries"], 2);
}
TEST_F(ParasolidInspectCliTest,
       SchemaLimitOptionsRequireModeAndPositiveIntegers) {
  for (const auto& flag :
       {"--max-fields", "--max-values", "--max-delta-entries"}) {
    EXPECT_EQ(Run(std::string(flag) + " 1"), 1);
    for (const auto& value : {"0", "-1", "1.5", "18446744073709551616", "1x"})
      EXPECT_EQ(Run(std::string("--nodes ") + flag + " " + value), 1);
    EXPECT_EQ(Run(std::string("--nodes ") + flag), 1);
  }
}

TEST_F(ParasolidInspectCliTest, GeometryModeBuildsOwnedPointAndSummary) {
  auto bytes = test::Header();
  test::U16(bytes, 29);
  bytes.push_back(255);
  test::CompactIndex(bytes, 7);
  test::U32(bytes, 1);
  for (int i = 0; i < 4; ++i) test::CompactIndex(bytes, 0);
  for (double v : {1.25, -2.5, 3.75}) test::F64(bytes, v);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 0);
  auto r = Report();
  EXPECT_EQ(r["interpretation"], "geometry_model");
  EXPECT_EQ(r["complete"], true);
  EXPECT_EQ(r["model"]["references_verified"], true);
  EXPECT_EQ(r["model"]["geometry"][0]["position"],
            json::array({1.25, -2.5, 3.75}));
  EXPECT_EQ(r["model"]["geometry"][0]["source"], 7);
  EXPECT_EQ(r["model"]["nodes"][0]["geometry"], 0);
  EXPECT_FALSE(r.contains("nodes"));
  ASSERT_EQ(Run("--geometry --summary"), 0);
  r = Report();
  EXPECT_EQ(r["model"]["geometry_count"], 1);
  EXPECT_FALSE(r["model"].contains("nodes"));
  EXPECT_FALSE(r["model"].contains("geometry"));
  EXPECT_FALSE(r["model"].contains("issues"));
}
TEST_F(ParasolidInspectCliTest,
       GeometryModeRejectsDeltasAndPreservesPartialParse) {
  auto bytes = test::DeltaPrefixBytes();
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  auto write = [&] {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  };
  write();
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["complete"], true);
  EXPECT_TRUE(Report()["model"].is_null());
  EXPECT_EQ(Report()["model_error"]["code"], "unsupported_format");
  ASSERT_EQ(Run("--body-topology=1 --summary"), 2);
  EXPECT_TRUE(Report()["body_topology"].is_null());
  EXPECT_EQ(Report()["body_topology_error"]["code"], "unsupported_format");
  bytes = test::Header();
  test::U16(bytes, 29);
  bytes.push_back(255);
  write();
  ASSERT_EQ(Run("--geometry"), 2);
  EXPECT_EQ(Report()["complete"], false);
  EXPECT_TRUE(Report()["model"].is_null());
  ASSERT_EQ(Run("--body-topology=1 --summary"), 2);
  EXPECT_EQ(Report()["complete"], false);
  EXPECT_TRUE(Report()["body_topology"].is_null());
  EXPECT_EQ(Report()["body_topology_error"]["body"], 1);

}
TEST_F(ParasolidInspectCliTest,
       GeometryModeReportsSemanticIssuesWithLimitedStatus) {
  auto bytes = test::Header();
  test::U16(bytes, 29);
  bytes.push_back(255);
  test::CompactIndex(bytes, 7);
  test::U32(bytes, 1);
  test::CompactIndex(bytes, 100);  // Missing attribute target.
  for (int i = 0; i < 3; ++i) test::CompactIndex(bytes, 0);
  for (double v : {1., 2., 3.}) test::F64(bytes, v);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  const auto r = Report();
  EXPECT_EQ(r["complete"], true);
  EXPECT_EQ(r["model"]["geometry_count"], 1);
  EXPECT_EQ(r["model"]["missing_reference_count"], 1);
  EXPECT_EQ(r["model"]["issues"][0]["code"], "missing_target");
}

TEST_F(ParasolidInspectCliTest, GeometryModeRetainsSurfaceCurveReferences) {
  auto bytes = test::Header();
  test::U16(bytes, 137);
  bytes.push_back(255);
  test::CompactIndex(bytes, 10);
  test::U32(bytes, 10);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back('-');
  test::CompactIndex(bytes, 20);
  test::CompactIndex(bytes, 1);
  test::CompactIndex(bytes, 10);  // Ignored original reference may be cyclic.
  test::F64(bytes, 0);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  auto r = Report();
  EXPECT_EQ(r["complete"], true);
  const auto& geometry = r["model"]["geometry"][0];
  EXPECT_EQ(geometry["kind"], "surface_curve");
  EXPECT_EQ(geometry["surface"], 20);
  EXPECT_EQ(geometry["parameter_curve"], 1);
  EXPECT_EQ(r["model"]["typed_geometry_complete"], true);
  EXPECT_EQ(r["model"]["missing_reference_count"], 2);
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["model"]["geometry_counts"]["surface_curve"], 1);
}

TEST_F(ParasolidInspectCliTest, GeometryModeRetainsTrimmedCurveDefinition) {
  auto bytes = test::Header();
  test::U16(bytes, 133);
  bytes.push_back(255);
  test::CompactIndex(bytes, 10);
  test::U32(bytes, 10);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back('+');
  test::CompactIndex(bytes, 1);
  for (double v : {1., 2., 3., 4., 5., 6., 1., 3.}) test::F64(bytes, v);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  const auto r = Report();
  const auto& geometry = r["model"]["geometry"][0];
  EXPECT_EQ(geometry["kind"], "trimmed_curve");
  EXPECT_EQ(geometry["basis_curve"], 1);
  EXPECT_EQ(geometry["point_1"], json::array({1., 2., 3.}));
  EXPECT_EQ(geometry["point_2"], json::array({4., 5., 6.}));
  EXPECT_EQ(geometry["parameter_1"], 1);
  EXPECT_EQ(geometry["parameter_2"], 3);
  EXPECT_EQ(r["model"]["missing_reference_count"], 1);
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["model"]["geometry_counts"]["trimmed_curve"], 1);
}
TEST_F(ParasolidInspectCliTest,
       GeometryModeRetainsSpunSurfaceProfileAndMetadata) {
  auto bytes = test::Header();
  test::U16(bytes, 68);
  bytes.push_back(255);
  test::CompactIndex(bytes, 20);
  test::U32(bytes, 20);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back('-');
  test::CompactIndex(bytes, 1);
  for (double v :
       {0., 0., 0., 0., 0., 1., 1., 0., 0., 1., 0., 1., 0., 1., 1., 0., 0., 0.})
    test::F64(bytes, v);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  const auto r = Report();
  EXPECT_EQ(r["complete"], true);
  const auto& geometry = r["model"]["geometry"][0];
  EXPECT_EQ(geometry["kind"], "spun_surface");
  EXPECT_EQ(geometry["profile"], 1);
  EXPECT_EQ(geometry["axis"], json::array({0., 0., 1.}));
  EXPECT_EQ(geometry["start_parameter"], 0);
  EXPECT_EQ(geometry["end_parameter"], 1);
  EXPECT_EQ(geometry["scale"], 0);
  EXPECT_EQ(r["model"]["missing_reference_count"], 1);
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["model"]["geometry_counts"]["spun_surface"], 1);
}
TEST_F(ParasolidInspectCliTest,
       GeometryModeRetainsSweptSurfaceSectionAndScale) {
  auto bytes = test::Header();
  test::U16(bytes, 67);
  bytes.push_back(255);
  test::CompactIndex(bytes, 20);
  test::U32(bytes, 20);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back('-');
  test::CompactIndex(bytes, 1);
  for (double v : {0., .6, .8, 0.}) test::F64(bytes, v);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  const auto r = Report();
  EXPECT_EQ(r["complete"], true);
  const auto& geometry = r["model"]["geometry"][0];
  EXPECT_EQ(geometry["kind"], "swept_surface");
  EXPECT_EQ(geometry["section"], 1);
  EXPECT_EQ(geometry["sweep"], json::array({0., .6, .8}));
  EXPECT_EQ(geometry["scale"], 0);
  EXPECT_EQ(r["model"]["missing_reference_count"], 1);
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["model"]["geometry_counts"]["swept_surface"], 1);
}
TEST_F(ParasolidInspectCliTest, GeometryModeReportsIntersectionChartAndLimits) {
  auto bytes = test::Header();
  test::U16(bytes, 38);
  bytes.push_back(255);
  test::CompactIndex(bytes, 1);
  test::U32(bytes, 1);
  for (int i = 0; i < 5; ++i) test::CompactIndex(bytes, 0);
  bytes.push_back('+');
  for (uint32_t id : {50u, 51u, 2u, 3u, 3u}) test::CompactIndex(bytes, id);
  test::U16(bytes, 40);
  bytes.push_back(255);
  test::U32(bytes, 2);
  test::CompactIndex(bytes, 2);
  test::F64(bytes, .25);
  test::F64(bytes, 1.);
  test::U32(bytes, 2);
  for (double value : {0., .1, 0., 0., 0., 0., 0., 1., 2., 3.})
    test::F64(bytes, value);
  test::U16(bytes, 41);
  bytes.push_back(255);
  test::U32(bytes, 1);
  test::CompactIndex(bytes, 3);
  bytes.push_back('L');
  for (int i = 0; i < 3; ++i) test::F64(bytes, 0.);
  test::U16(bytes, 1);
  test::CompactIndex(bytes, 0);
  {
    std::ofstream file(root / "input.x_b", std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  }
  ASSERT_EQ(Run("--geometry"), 2);
  const auto report = Report();
  EXPECT_EQ(report["complete"], true);
  const auto& geometry = report["model"]["geometry"][0];
  EXPECT_EQ(geometry["kind"], "intersection_curve");
  EXPECT_EQ(geometry["surfaces"], json::array({50, 51}));
  EXPECT_EQ(geometry["chart"]["base_parameter"], .25);
  EXPECT_EQ(geometry["chart"]["positions"][1], json::array({1., 2., 3.}));
  EXPECT_EQ(geometry["start"]["type"], 'L');
  EXPECT_FALSE(geometry["start"].contains("term_use"));
  EXPECT_FALSE(geometry.contains("intersection_data"));
  ASSERT_EQ(Run("--geometry --summary"), 2);
  EXPECT_EQ(Report()["model"]["geometry_counts"]["intersection_curve"], 1);
}
}  // namespace
}  // namespace openparasolid
