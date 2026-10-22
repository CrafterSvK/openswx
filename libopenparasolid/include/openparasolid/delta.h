// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
/**
 * @file
 * @brief Caller-directed application of decoded delta node images.
 * @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)", @ref
 * xt_indices "XT 2.1.4.4: Unresolved indices (p. 17)"
 */
#ifndef OPENPARASOLID_DELTA_H_
#define OPENPARASOLID_DELTA_H_
#include <cstddef>
#include <cstdint>
#include <vector>

#include "openparasolid/model.h"
namespace openparasolid {
/// @brief Caller-selected logical operation, not an inferred wire status.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
enum class DeltaOperationKind {
  /// Create a previously absent logical node.
  kCreate,
  /// Replace an existing node with a complete decoded image.
  kReplace,
  /// Erase an existing logical node; no image is consumed.
  kErase,
};
/// @brief Explicit creation, whole-node replacement or erasure of a state node.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaOperation {
  /// Caller-selected logical operation.
  DeltaOperationKind kind = DeltaOperationKind::kReplace;
  /// State-local node identity to create, replace or erase.
  NodeId target;
  /// Index in SchemaParseReport.delta_nodes; ignored only for kErase.
  std::size_t occurrence = 0;
};
/// @brief Caller-resolved wire-index/status pair mapped to a state-local
/// NodeId.
///
/// @see @ref xt_indices "XT 2.1.4.4: Unresolved indices (p. 17)"
struct DeltaReferenceMapping {
  /// Decoded wire reference index, before mapping to a logical state identity.
  uint32_t wire_index = 0;
  /// Raw pointer status used as part of the caller-resolved mapping key.
  uint8_t status = 0;
  /// Resolved state-local target identity; zero denotes null.
  NodeId target;
};
/// @brief Explicit revision transition and complete identity mapping supplied
/// by the caller.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaApplicationPlan {
  /// Required revision of the input logical state.
  uint64_t base_revision = 0;
  /// Caller-selected revision key for the successfully applied state.
  uint64_t target_revision = 0;
  /// Explicit logical operations; targets must be unique.
  std::vector<DeltaOperation> operations;
  /// Complete caller-resolved wire-index/status to state-ID mappings.
  std::vector<DeltaReferenceMapping> references;
};
/// @brief Owned logical snapshot and caller-defined revision key.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
struct DeltaState {
  /// Caller-defined revision key, not an inferred PMARK identifier.
  uint64_t revision = 0;
  /// An owned logical snapshot, usable by ModelBuilder. After application byte
  /// offsets are virtual addresses, not offsets in either original file.
  SchemaParseReport snapshot;
};
/// Creates a checked owned baseline. Revision keys are supplied by the caller;
/// no PMARK/history interpretation is guessed.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)"
///
/// @param base Baseline snapshot/state, or reviewed base type definition as
/// documented.
/// @param revision Caller-defined baseline revision key.
/// @param limits Per-call work and storage limits.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<DeltaState> CreateDeltaState(
    const SchemaParseReport& base, uint64_t revision,
    const ModelOptions& limits = {});
/// Atomically applies explicit create/replace/delete operations from decoded
/// status-framed complete node images (replacement, not field patches). The
/// caller must resolve inherited/partial-field semantics before using an image.
/// Every non-null pointer requires a caller-
/// resolved (wire index, status)->state ID mapping. Rejects stale revisions,
/// duplicate operations, invalid shapes, missing mapping/deletion targets and
/// dangling/wrong-class final references. Unknown classes remain model
/// findings. No input is modified and no partial state is returned. This is the
/// state application engine, NOT automatic PMARK selection or wire-status
/// semantics: the caller must resolve history, identities and operation kinds
/// explicitly.
///
/// @see @ref xt_documents "XT 1.1.2: Types of file document (p. 8)", @ref
/// xt_indices "XT 2.1.4.4: Unresolved indices (p. 17)"
///
/// @param base Baseline snapshot/state, or reviewed base type definition as
/// documented.
/// @param delta Decoded internal delta occurrences supplying complete node
/// images.
/// @param plan Caller-resolved operations, identity mappings and revision
/// transition.
/// @param limits Per-call work and storage limits.
/// @return The requested value on success, or an explicit ParseError on
/// failure.
[[nodiscard]] Result<DeltaState> ApplyDelta(const DeltaState& base,
                                            const SchemaParseReport& delta,
                                            const DeltaApplicationPlan& plan,
                                            const ModelOptions& limits = {});
}  // namespace openparasolid
#endif  // OPENPARASOLID_DELTA_H_
