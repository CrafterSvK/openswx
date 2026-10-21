// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "test_helpers.h"

namespace openparasolid {
namespace {
namespace fs = std::filesystem;
using nlohmann::json;

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
  bytes = test::Header();
  test::U16(bytes, 29);
  bytes.push_back(255);
  write();
  ASSERT_EQ(Run("--geometry"), 2);
  EXPECT_EQ(Report()["complete"], false);
  EXPECT_TRUE(Report()["model"].is_null());
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

}  // namespace
}  // namespace openparasolid
