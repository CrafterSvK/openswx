// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_DELTA_H_
#define OPENPARASOLID_DELTA_H_
#include <cstddef>
#include <cstdint>
#include <vector>

#include "openparasolid/model.h"
namespace openparasolid {
enum class DeltaOperationKind { kCreate, kReplace, kErase };
struct DeltaOperation {
  DeltaOperationKind kind = DeltaOperationKind::kReplace;
  NodeId target;
  // Index in SchemaParseReport.delta_nodes; ignored only for kErase.
  std::size_t occurrence = 0;
};
struct DeltaReferenceMapping {
  uint32_t wire_index = 0;
  uint8_t status = 0;
  NodeId target;
};
struct DeltaApplicationPlan {
  uint64_t base_revision = 0;
  uint64_t target_revision = 0;
  std::vector<DeltaOperation> operations;
  std::vector<DeltaReferenceMapping> references;
};
struct DeltaState {
  uint64_t revision = 0;
  // An owned logical snapshot, usable by ModelBuilder. After application byte
  // offsets are virtual addresses, not offsets in either original file.
  SchemaParseReport snapshot;
};
// Creates a checked owned baseline. Revision keys are supplied by the caller;
// no PMARK/history interpretation is guessed.
[[nodiscard]] Result<DeltaState> CreateDeltaState(
    const SchemaParseReport& base, uint64_t revision,
    const ModelOptions& limits = {});
// Atomically applies explicit create/replace/delete operations from decoded
// status-framed complete node images (replacement, not field patches). The
// caller must resolve inherited/partial-field semantics before using an image.
// Every non-null pointer requires a caller-
// resolved (wire index, status)->state ID mapping. Rejects stale revisions,
// duplicate operations, invalid shapes, missing mapping/deletion targets and
// dangling/wrong-class final references. Unknown classes remain model findings.
// No input is modified and no partial state is returned. This is the state
// application engine, NOT automatic PMARK selection or wire-status semantics:
// the caller must resolve history, identities and operation kinds explicitly.
[[nodiscard]] Result<DeltaState> ApplyDelta(const DeltaState& base,
                                            const SchemaParseReport& delta,
                                            const DeltaApplicationPlan& plan,
                                            const ModelOptions& limits = {});
}  // namespace openparasolid
#endif  // OPENPARASOLID_DELTA_H_
