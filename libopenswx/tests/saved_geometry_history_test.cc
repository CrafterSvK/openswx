// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "saved_geometry_history.h"

#include <gtest/gtest.h>

#include <algorithm>

namespace openswx::internal {
namespace {
using Status = GeometryResolutionStatus;
using Relation = SavedStateRelation;
using Mark = openparasolid::DeltaMark;
Mark Node(uint32_t index, uint32_t identifier, uint32_t parent = 0,
          uint32_t child = 0) {
  Mark mark;
  mark.index = index;
  mark.identifier = identifier;
  mark.preceding = parent;
  mark.first_following = child;
  return mark;
}
SavedHistoryData History() {
  SavedHistoryData history;
  history.complete = true;
  history.prefix.emplace();
  history.prefix->partition.current_pmark = 40;
  history.prefix->current_mark = Node(40, 300, 20);
  history.marks = {Node(10, 100, 0, 20), Node(20, 200, 10, 40)};
  return history;
}
SavedGeometryResolution Input(uint32_t baseline = 300) {
  SavedGeometryResolution result;
  SavedGeometryCandidate base;
  base.payload_index = 0;
  base.kind = ParasolidPayloadKind::kPartition;
  base.role = GeometrySourceRole::kPartition;
  base.source = {"Config-0-Partition", 100, 4};
  base.selected_partition = true;
  base.decoded_complete = true;
  base.world_current_id = baseline;
  result.candidates.push_back(base);
  auto delta = base;
  delta.payload_index = 1;
  delta.selected_partition = false;
  delta.kind = ParasolidPayloadKind::kDeltas;
  delta.role = GeometrySourceRole::kDelta;
  result.candidates.push_back(delta);
  return result;
}
TEST(SavedGeometryHistory, CurrentBaselineHasOnlyTheCurrentMarkInItsPath) {
  auto result = Input();
  InspectSavedState(result, {{}, History()});
  const auto& e = result.state_evidence;
  EXPECT_EQ(e.relation, Relation::kMatchingIdentifiers);
  EXPECT_EQ(e.baseline_current_id, 300u);
  EXPECT_EQ(e.target_identifier, 300u);
  EXPECT_EQ(e.history_path_status, Status::kResolved);
  EXPECT_EQ(e.mark_path, (std::vector<uint32_t>{40}));
  EXPECT_TRUE(e.issue.empty());
  EXPECT_EQ(result.state_status, Status::kUnresolved);
  EXPECT_EQ(result.body_membership_status, Status::kUnresolved);
  EXPECT_TRUE(result.candidates.front().bodies.empty());
}
TEST(SavedGeometryHistory, DifferentBaselineSelectsOnlyItsAncestrySuffix) {
  for (uint32_t baseline : {100u, 200u}) {
    auto result = Input(baseline);
    auto history = History();
    std::reverse(history.marks.begin(), history.marks.end());
    InspectSavedState(result, {{}, history});
    const auto& e = result.state_evidence;
    EXPECT_EQ(e.relation, Relation::kDifferentIdentifiers);
    EXPECT_EQ(e.history_path_status, Status::kResolved);
    EXPECT_EQ(e.mark_path, baseline == 100 ? (std::vector<uint32_t>{10, 20, 40})
                                           : (std::vector<uint32_t>{20, 40}));
    EXPECT_EQ(result.state_status, Status::kUnresolved);
  }
}
TEST(SavedGeometryHistory, DoesNotUseLargestIdOrChooseAnotherBranch) {
  auto history = History();
  history.marks[1].next_sibling = 30;
  auto sibling = Node(30, 999, 10);
  sibling.prev_sibling = 20;
  history.marks.push_back(sibling);
  auto result = Input(999);
  InspectSavedState(result, {{}, history});
  EXPECT_EQ(result.state_evidence.target_identifier, 300u);
  EXPECT_EQ(result.state_evidence.relation, Relation::kDifferentIdentifiers);
  EXPECT_EQ(result.state_evidence.history_path_status, Status::kUnresolved);
  EXPECT_TRUE(result.state_evidence.mark_path.empty());
  EXPECT_FALSE(result.state_evidence.issue.empty());
}
TEST(SavedGeometryHistory, BrokenGraphsNeverProduceAReplayPath) {
  for (int kind = 0; kind < 9; ++kind) {
    auto history = History();
    if (kind == 0) history.marks.pop_back();  // Missing predecessor.
    if (kind == 1) history.marks[1].first_following = 20;  // Self link.
    if (kind == 2) history.marks[1].identifier = 300;  // Ambiguous identifier.
    if (kind == 3) history.marks.push_back(Node(20, 999, 10, 40));
    if (kind == 4) history.marks[1].prev_sibling = 10;
    if (kind == 5) history.marks[1].preceding = 40;  // Cycle.
    if (kind == 6) history.marks[0].first_following = 0;
    if (kind == 7) history.marks.push_back(Node(80, 800));  // Second root.
    if (kind == 8) {  // Disconnected cycle with locally reciprocal links.
      history.marks.push_back(Node(80, 800, 90, 90));
      history.marks.push_back(Node(90, 900, 80, 80));
    }
    auto result = Input();
    InspectSavedState(result, {{}, history});
    EXPECT_EQ(result.state_evidence.relation, Relation::kMatchingIdentifiers);
    EXPECT_EQ(result.state_evidence.history_path_status, Status::kUnresolved)
        << kind;
    EXPECT_TRUE(result.state_evidence.mark_path.empty()) << kind;
    EXPECT_FALSE(result.state_evidence.issue.empty()) << kind;
  }
}
TEST(SavedGeometryHistory, IdenticalRepeatedMarksAreNotExtraTransitions) {
  auto history = History();
  auto repeated = history.marks.front();
  repeated.offset = 9999;
  history.marks.push_back(repeated);
  auto result = Input();
  InspectSavedState(result, {{}, history});
  EXPECT_EQ(result.state_evidence.history_path_status, Status::kResolved);
  EXPECT_EQ(result.state_evidence.mark_path, (std::vector<uint32_t>{40}));
}
TEST(SavedGeometryHistory, DoesNotAssociateUnrelatedOrMultipleDeltaCandidates) {
  for (int kind = 0; kind < 5; ++kind) {
    auto result = Input();
    if (kind == 0) result.candidates[1].source.stream = "Config-1-Partition";
    if (kind == 1) result.candidates[1].source.container_offset = 101;
    if (kind == 2) {
      auto duplicate = result.candidates[1];
      duplicate.payload_index = 2;
      result.candidates.push_back(duplicate);
    }
    if (kind == 3) result.candidates[0].selected_partition = false;
    if (kind == 4) result.candidates[0].world_current_id.reset();
    InspectSavedState(result, {{}, History(), History()});
    EXPECT_EQ(result.state_evidence.relation, Relation::kUnavailable);
    EXPECT_EQ(result.state_evidence.history_path_status, Status::kUnresolved);
    EXPECT_TRUE(result.state_evidence.mark_path.empty());
    EXPECT_FALSE(result.state_evidence.issue.empty());
  }
}
TEST(SavedGeometryHistory, IncompleteDeltaDoesNotComparePrefixIdentifiers) {
  auto result = Input();
  auto history = History();
  history.complete = false;
  InspectSavedState(result, {{}, history});
  EXPECT_EQ(result.state_evidence.relation, Relation::kUnavailable);
  EXPECT_FALSE(result.state_evidence.target_identifier);
  EXPECT_TRUE(result.state_evidence.mark_path.empty());
}
TEST(SavedGeometryHistory, AuxiliaryBodyInventoriesDoNotBecomeFinalBodies) {
  auto result = Input();
  result.candidates[0].bodies.push_back({2, SavedBodyKind::kSolid});
  auto auxiliary = result.candidates[0];
  auxiliary.payload_index = 2;
  auxiliary.selected_partition = false;
  auxiliary.source.stream = "Config-0-ResolvedFeatures";
  auxiliary.role = GeometrySourceRole::kInternalFeature;
  auxiliary.world_current_id = 999;
  auxiliary.bodies = {{1, SavedBodyKind::kSheet}};
  result.candidates.push_back(auxiliary);
  InspectSavedState(result, {{}, History(), {}});
  EXPECT_EQ(result.state_evidence.relation, Relation::kMatchingIdentifiers);
  EXPECT_EQ(result.state_evidence.history_path_status, Status::kResolved);
  EXPECT_EQ(result.candidates[0].bodies.size(), 1u);
  EXPECT_EQ(result.candidates[2].bodies.size(), 1u);
  EXPECT_EQ(result.body_membership_status, Status::kUnresolved);
}
}  // namespace
}  // namespace openswx::internal
