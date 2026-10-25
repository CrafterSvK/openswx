// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openswx/saved_geometry.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "test_helpers.h"

namespace openswx {
namespace {
using Status = GeometryResolutionStatus;
StreamInfo Xml(std::string name, std::string_view value) {
  StreamInfo s;
  s.name = std::move(name);
  s.status = StreamDecodeStatus::kDecoded;
  s.selected_as_primary = true;
  s.marker_offset = 120;
  s.decoded_data.assign(value.begin(), value.end());
  return s;
}
std::vector<uint8_t> Payload(std::string_view kind) {
  std::vector<uint8_t> data{'P', 'S'};
  for (const std::string text : {": TRANSMIT FILE (" + std::string(kind) +
                                     ") created by modeller version 1234567",
                                 std::string("SCH_1234567_12345_12345")}) {
    const auto n = static_cast<uint32_t>(text.size());
    for (const int shift : {24, 16, 8, 0})
      data.push_back(static_cast<uint8_t>(n >> shift));
    data.insert(data.end(), text.begin(), text.end());
  }
  data.insert(data.end(), {0, 1, 2, 3});
  return data;
}
void AppendBytes(StreamInfo& stream, const std::vector<uint8_t>& payload) {
  const std::array<uint8_t, 16> marker{0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81,
                                       0x48, 0xa2, 0xa8, 0x58, 0x98, 0xb2,
                                       0x1b, 0x89, 0xef, 0x99};
  const auto compressed = test::DeflateZlib(payload);
  auto& bytes = stream.decoded_data;
  bytes.insert(bytes.end(), marker.begin(), marker.end());
  test::AppendU32Le(&bytes, static_cast<uint32_t>(payload.size()));
  test::AppendU32Le(&bytes, static_cast<uint32_t>(compressed.size()));
  bytes.insert(bytes.end(), compressed.begin(), compressed.end());
}
void AppendPayload(StreamInfo& stream, std::string_view kind) {
  AppendBytes(stream, Payload(kind));
}
void Be16(std::vector<uint8_t>& data, uint16_t value) {
  data.push_back(static_cast<uint8_t>(value >> 8));
  data.push_back(static_cast<uint8_t>(value));
}
void Be32(std::vector<uint8_t>& data, uint32_t value) {
  for (int shift : {24, 16, 8, 0})
    data.push_back(static_cast<uint8_t>(value >> shift));
}
std::vector<uint8_t> Header(std::string_view kind) {
  std::vector<uint8_t> data{'P', 'S', 0, 0};
  const std::string description = ": TRANSMIT FILE (" + std::string(kind) +
                                  ") created by modeller version 1234567";
  Be16(data, static_cast<uint16_t>(description.size()));
  data.insert(data.end(), description.begin(), description.end());
  const std::string schema = "SCH_1234567_12345_13006";
  data.push_back(static_cast<uint8_t>(schema.size()));
  data.insert(data.end(), schema.begin(), schema.end());
  Be16(data, 204);
  Be32(data, 0);
  return data;
}
void End(std::vector<uint8_t>& data) {
  Be16(data, 1);
  Be16(data, 1);
}
std::vector<uint8_t> World(uint32_t current) {
  auto data = Header("partition");
  Be16(data, 101);
  data.push_back(255);
  Be16(data, 2);  // WORLD index 1.
  for (int i = 0; i < 7; ++i) Be16(data, 1);
  data.push_back(1);  // Alive.
  Be16(data, 1);
  Be32(data, 123456789);  // Deliberately unrelated allocation counter.
  Be32(data, current);
  End(data);
  return data;
}
std::vector<uint8_t> Delta(uint32_t current) {
  auto data = Header("deltas");
  Be16(data, 3);
  data.push_back(255);
  Be16(data, 3);  // PARTITION index 2.
  Be16(data, 4);  // Current PMARK index 3.
  Be32(data, current);
  Be16(data, 4);
  data.push_back(255);
  Be16(data, 4);
  for (int i = 0; i < 4; ++i) Be16(data, 1);
  for (int i = 0; i < 4; ++i) Be32(data, 0);
  data.push_back(0);
  Be32(data, current);
  End(data);
  return data;
}
StreamInfo Partition(std::string name = "Contents/Config-5-Partition") {
  auto stream = Xml(std::move(name), "");
  AppendPayload(stream, "partition");
  return stream;
}
StreamInfo Keyword(std::string_view attributes = "id='9' SourceIndex='5'",
                   std::string_view name = "Target") {
  return Xml("swXmlContents/KeyWords",
             "<Keywords><Configuration Name='" + std::string(name) + "' " +
                 std::string(attributes) + "/></Keywords>");
}
StreamInfo Manifest(uint32_t index = 5, bool active = true,
                    bool rebuild = false) {
  return Xml(
      "swXmlContents/Features",
      "<s:swSolidWorks xmlns:s='http://www.solidworks.com/sw2003/schema'>"
      "<s:swModelList><s:swModel id='model' swConfigurationName='Target' "
      "swConfigurationId='" +
          std::to_string(index) +
          "'/></s:swModelList>"
          "<s:swConfigurationList><s:swConfiguration id='24' swName='Target' "
          "swID='" +
          std::to_string(index) +
          "' swModelRef='model' swMostRecentConfiguration='" +
          (active ? "YES" : "NO") + "' swConfigurationNeedsUpdate='" +
          (rebuild ? "YES" : "NO") +
          "'/></s:swConfigurationList></s:swSolidWorks>");
}
std::size_t Selected(const SavedGeometryResolution& result) {
  return static_cast<std::size_t>(
      std::count_if(result.candidates.begin(), result.candidates.end(),
                    [](const auto& c) { return c.selected_partition; }));
}
TEST(SavedGeometry, SeparatesSelectorsFromFeatureSlotsAndStateProof) {
  ContainerInspection input;
  input.streams = {Keyword(), Manifest(), Partition(),
                   Partition("Config-9-FeatureBodies/LocalBodies"),
                   Partition("Contents/Config-5-GhostPartition")};
  const auto result = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(result.ok()) << result.error();
  const auto& r = result.value();
  EXPECT_EQ(r.configuration_status, Status::kResolved);
  EXPECT_EQ(r.partition_status, Status::kResolved);
  EXPECT_EQ(r.partition_index, 5u);
  EXPECT_EQ(r.resolved_features_index, 9u);
  EXPECT_EQ(r.was_last_active, true);
  EXPECT_EQ(r.needs_rebuild, false);
  EXPECT_EQ(r.state_status, Status::kUnresolved);
  EXPECT_EQ(r.body_membership_status, Status::kUnresolved);
  EXPECT_EQ(Selected(r), 1u);
  ASSERT_EQ(r.candidates.size(), 3u);
  EXPECT_EQ(r.candidates[1].role, GeometrySourceRole::kInternalFeature);
  EXPECT_EQ(r.candidates[1].stream_index, 9u);
  EXPECT_EQ(r.candidates[2].role, GeometrySourceRole::kGhost);
  EXPECT_FALSE(r.candidates[0].decoded_complete);
  EXPECT_TRUE(r.candidates[0].bodies.empty());
  EXPECT_FALSE(r.candidates[0].decode_issue.empty());
  EXPECT_FALSE(r.evidence.empty());
}
TEST(SavedGeometry,
     ResolvesInactiveConfigurationIndependentlyOfActiveSelection) {
  auto manifest = Manifest(7, false, true);
  std::string text(manifest.decoded_data.begin(), manifest.decoded_data.end());
  const auto at = text.find("</s:swConfigurationList>");
  text.insert(at,
              "<s:swConfiguration swName='Other' swID='3' swModelRef='another' "
              "swMostRecentConfiguration='YES'/>");
  manifest.decoded_data.assign(text.begin(), text.end());
  ContainerInspection input;
  input.streams = {manifest, Partition("Config-7-Partition"),
                   Partition("Config-3-Partition")};
  const auto result = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().partition_index, 7u);
  EXPECT_EQ(result.value().was_last_active, false);
  EXPECT_EQ(result.value().needs_rebuild, true);
  EXPECT_EQ(result.value().partition_status, Status::kResolved);
  EXPECT_EQ(Selected(result.value()), 1u);
}
TEST(SavedGeometry, ConflictingSelectorsNeverPickByPriorityOrPhysicalOrder) {
  ContainerInspection input;
  input.streams = {Keyword(), Manifest(7), Partition(),
                   Partition("Config-7-Partition")};
  for (bool reverse : {false, true}) {
    if (reverse) std::reverse(input.streams.begin(), input.streams.end());
    const auto r = ResolveSavedGeometry(input, "Target");
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value().configuration_status, Status::kUnresolved);
    EXPECT_FALSE(r.value().partition_index);
    EXPECT_EQ(Selected(r.value()), 0u);
  }
}
TEST(SavedGeometry, SolePartitionIsOnlyAHeuristicForAKnownConfiguration) {
  ContainerInspection input;
  input.streams = {Keyword("id='9'"), Partition()};
  auto result = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().partition_status, Status::kHeuristic);
  EXPECT_EQ(result.value().configuration_status, Status::kUnresolved);
  EXPECT_EQ(result.value().state_status, Status::kUnresolved);
  result = ResolveSavedGeometry(input, "Missing");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(Selected(result.value()), 0u);
  input.streams = {Keyword(), Partition("Config-7-Partition")};
  result = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(Selected(result.value()), 0u);
}
TEST(SavedGeometry,
     LocalBodiesResolvedFeaturesAndGhostsCannotReplaceAPartition) {
  ContainerInspection input;
  input.streams = {Keyword("id='0' SourceIndex='0'")};
  for (int i = 0; i < 7; ++i)
    input.streams.push_back(Partition("Config-0-FeatureBodies/LocalBodies"));
  input.streams.push_back(Partition("Contents/Config-0-ResolvedFeatures"));
  input.streams.push_back(Partition("Config-0-GhostPartition"));
  const auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().configuration_status, Status::kResolved);
  EXPECT_EQ(r.value().partition_status, Status::kUnresolved);
  EXPECT_EQ(Selected(r.value()), 0u);
  EXPECT_EQ(r.value().candidates.size(), 9u);
}
TEST(SavedGeometry, DeltaCandidatesRequireAnExplicitUnsupportedHistoryStage) {
  ContainerInspection input;
  auto partition = Partition();
  AppendPayload(partition, "deltas");
  input.streams = {Keyword(), partition};
  const auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().partition_status, Status::kResolved);
  EXPECT_EQ(r.value().state_status, Status::kUnsupported);
  EXPECT_EQ(r.value().body_membership_status, Status::kUnresolved);
  EXPECT_EQ(r.value().candidates[1].kind, ParasolidPayloadKind::kDeltas);
  EXPECT_FALSE(r.value().candidates[1].selected_partition);
}
TEST(SavedGeometry, DoesNotAssociateDeltasAcrossStreamSlotNamespaces) {
  ContainerInspection input;
  auto internal = Partition("Config-5-FeatureBodies/LocalBodies");
  internal.decoded_data.clear();
  AppendPayload(internal, "deltas");
  input.streams = {Keyword(), Partition(), internal};
  auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().partition_status, Status::kResolved);
  EXPECT_EQ(r.value().state_status, Status::kUnresolved);
  input.streams.back() = Partition("Contents/Config-6-Partition");
  AppendPayload(input.streams.back(), "deltas");
  r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().state_status, Status::kUnresolved);
}
TEST(SavedGeometry, DuplicatePartitionsDoNotUseThePrimaryChunkAsAuthority) {
  ContainerInspection input;
  auto duplicate = Partition();
  duplicate.selected_as_primary = false;
  input.streams = {Keyword(), Partition(), duplicate};
  auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(Selected(r.value()), 0u);
  input.streams.back() = Partition("Config-5-Partition");
  r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(Selected(r.value()), 0u);
}
TEST(SavedGeometry, RejectsInvalidAndDuplicateConfigurationSelectors) {
  for (const std::string value : {"-1", "4294967296", "5suffix", ""}) {
    ContainerInspection input;
    input.streams = {Keyword("SourceIndex='" + value + "'"), Manifest(),
                     Partition()};
    const auto r = ResolveSavedGeometry(input, "Target");
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(Selected(r.value()), 0u);
  }
  ContainerInspection input;
  input.streams = {Keyword(), Keyword(), Partition()};
  const auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(Selected(r.value()), 0u);
}
TEST(SavedGeometry,
     MalformedWrongNamespaceAndAmbiguousModelLinksStayUnresolved) {
  for (int kind = 0; kind < 4; ++kind) {
    auto manifest = Manifest();
    std::string text(manifest.decoded_data.begin(),
                     manifest.decoded_data.end());
    if (kind == 0) text = "<broken";
    if (kind == 1)
      text.replace(
          text.find("http://www.solidworks.com/sw2003/schema"),
          std::string("http://www.solidworks.com/sw2003/schema").size(),
          "http://example.invalid/foreign");
    if (kind == 2)
      text.replace(text.find("swModelRef='model'"),
                   std::string("swModelRef='model'").size(),
                   "swModelRef='absent'");
    if (kind == 3)
      text.insert(text.find("</s:swModelList>"),
                  "<s:swModel id='model' swConfigurationName='Target' "
                  "swConfigurationId='5'/>");
    manifest.decoded_data.assign(text.begin(), text.end());
    ContainerInspection input;
    input.streams = {Keyword(), manifest, Partition()};
    const auto r = ResolveSavedGeometry(input, "Target");
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(Selected(r.value()), 0u);
  }
}
TEST(SavedGeometry, NativeKeywordPrefixAndUnicodeNamesArePreserved) {
  auto keyword = Keyword("id='9' SourceIndex='5'", "Zwei &amp; Körper");
  keyword.decoded_data.insert(keyword.decoded_data.begin(), 0x86);
  ContainerInspection input;
  input.streams = {keyword, Partition()};
  const auto r = ResolveSavedGeometry(input, "Zwei & Körper");
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().configuration_status, Status::kResolved);
  EXPECT_EQ(Selected(r.value()), 1u);
}
TEST(SavedGeometry, MatchingStateIdentifiersDoNotCertifyEmptyBodyMembership) {
  for (uint32_t identifier : {0u, 300u, 0xffffffffu}) {
    auto stream = Xml("Config-5-Partition", "");
    AppendBytes(stream, World(identifier));
    AppendBytes(stream, Delta(identifier));
    ContainerInspection input;
    input.streams = {Keyword(), stream};
    const auto r = ResolveSavedGeometry(input, "Target");
    ASSERT_TRUE(r.ok()) << r.error();
    ASSERT_EQ(r.value().candidates.size(), 2u);
    EXPECT_TRUE(r.value().candidates[0].decoded_complete);
    EXPECT_TRUE(r.value().candidates[1].decoded_complete);
    EXPECT_TRUE(r.value().candidates[0].bodies.empty());
    EXPECT_TRUE(r.value().candidates[1].bodies.empty());
    EXPECT_EQ(r.value().candidates[0].world_current_id, identifier);
    const auto& e = r.value().state_evidence;
    EXPECT_EQ(e.baseline_current_id, identifier);
    EXPECT_EQ(e.target_identifier, identifier);
    EXPECT_EQ(e.relation, SavedStateRelation::kMatchingIdentifiers);
    EXPECT_EQ(e.history_path_status, Status::kResolved);
    EXPECT_EQ(e.mark_path, (std::vector<uint32_t>{3}));
    EXPECT_EQ(r.value().state_status, Status::kUnsupported);
    EXPECT_EQ(r.value().body_membership_status, Status::kUnresolved);
  }
}
TEST(SavedGeometry, UnsetWorldIdentifierIsNotTakenFromAllocationCounter) {
  auto stream = Xml("Config-5-Partition", "");
  AppendBytes(stream, World(0xffff8004u));  // XT unset integer sentinel.
  AppendBytes(stream, Delta(300));
  ContainerInspection input;
  input.streams = {Keyword(), stream};
  const auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r.value().candidates[0].world_current_id);
  EXPECT_EQ(r.value().state_evidence.relation,
            SavedStateRelation::kUnavailable);
}
TEST(SavedGeometry, DifferentIdentifiersDoNotApplyDeltaImages) {
  auto stream = Xml("Config-5-Partition", "");
  AppendBytes(stream, World(100));
  AppendBytes(stream, Delta(300));
  ContainerInspection input;
  input.streams = {Keyword(), stream};
  const auto r = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(r.ok());
  const auto& e = r.value().state_evidence;
  EXPECT_EQ(e.relation, SavedStateRelation::kDifferentIdentifiers);
  EXPECT_EQ(e.history_path_status, Status::kUnresolved);
  EXPECT_TRUE(e.mark_path.empty());
  EXPECT_EQ(r.value().candidates[0].world_current_id, 100u);
  EXPECT_EQ(r.value().state_status, Status::kUnsupported);
  SavedGeometryOptions options;
  options.max_nodes_per_payload = 1;
  EXPECT_FALSE(ResolveSavedGeometry(input, "Target", options).ok());
}
TEST(SavedGeometry, LargeOrdinaryPartitionsUseConsistentFiniteReadBudgets) {
  // 11,000 independent points cross both former limits (10,000 nodes and
  // 65,536 fields), without relying on any private SolidWorks geometry.
  auto data = Header("partition");
  for (uint16_t i = 1; i <= 11000; ++i) {
    Be16(data, 29);
    if (i == 1) data.push_back(255);
    Be16(data, i + 1);
    Be32(data, i);
    for (int j = 0; j < 4; ++j) Be16(data, 1);
    for (int j = 0; j < 6; ++j) Be32(data, 0);
  }
  End(data);
  auto stream = Xml("Config-5-Partition", "");
  AppendBytes(stream, data);
  ContainerInspection input;
  input.streams = {Keyword(), stream};
  const auto result = ResolveSavedGeometry(input, "Target");
  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().candidates.size(), 1u);
  EXPECT_TRUE(result.value().candidates[0].decoded_complete)
      << result.value().candidates[0].decode_issue;
  EXPECT_TRUE(result.value().candidates[0].decode_issue.empty());
  EXPECT_TRUE(result.value().candidates[0].bodies.empty());
  EXPECT_EQ(result.value().body_membership_status, Status::kUnresolved);
  for (int kind = 0; kind < 3; ++kind) {
    SavedGeometryOptions options;
    if (kind == 0) options.max_nodes_per_payload = 10000;
    if (kind == 1) options.max_fields_per_payload = 65536;
    if (kind == 2) options.max_values_per_payload = 100;
    EXPECT_FALSE(ResolveSavedGeometry(input, "Target", options).ok());
  }
}
TEST(SavedGeometry, EnforcesResourceCapsAndExplicitConfiguration) {
  ContainerInspection input;
  input.streams = {Keyword(), Manifest(), Partition()};
  EXPECT_FALSE(ResolveSavedGeometry(input, "").ok());
  for (int kind = 0; kind < 7; ++kind) {
    SavedGeometryOptions options;
    if (kind == 0) options.max_streams = 1;
    if (kind == 1) options.max_decoded_bytes = 1;
    if (kind == 2) options.max_xml_bytes = 1;
    if (kind == 3) options.max_xml_nodes = 1;
    if (kind == 4) options.max_nodes_per_payload = 0;
    if (kind == 5) options.max_fields_per_payload = 0;
    if (kind == 6) options.max_values_per_payload = 0;
    EXPECT_FALSE(ResolveSavedGeometry(input, "Target", options).ok());
  }
  input.streams.push_back(Partition("Config-6-Partition"));
  SavedGeometryOptions options;
  options.max_payloads = 1;
  EXPECT_FALSE(ResolveSavedGeometry(input, "Target", options).ok());
}
}  // namespace
}  // namespace openswx
