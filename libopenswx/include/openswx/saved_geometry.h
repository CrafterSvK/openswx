// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWX_SAVED_GEOMETRY_H_
#define OPENSWX_SAVED_GEOMETRY_H_

/// @file
/// @brief Evidence-based identities of saved SolidWorks geometry candidates.
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "openswx/container.h"

namespace openswx {
/// Independent confidence/capability result for one resolution stage.
enum class GeometryResolutionStatus {
  kResolved,     ///< This stage has an unambiguous stored association.
  kHeuristic,    ///< Candidate preference only; no stored association proved.
  kUnresolved,   ///< Missing or ambiguous evidence.
  kUnsupported,  ///< Required reconstruction capability is not implemented.
};
/// Storage provenance, not a determination of final/intermediate body role.
enum class GeometrySourceRole {
  kPartition,
  kDelta,
  kInternalFeature,
  kGhost,
  kOther
};
/// Source dimensional classification; neither validity nor final membership.
enum class SavedBodyKind { kSolid, kSheet, kWire, kGeneral, kUnknown };

/// Comparison of saved identifiers, not a certificate of model currency.
enum class SavedStateRelation {
  kUnavailable,           ///< A unique, decoded base/delta pair is unavailable.
  kMatchingIdentifiers,   ///< WORLD current_id equals current PMARK identifier.
  kDifferentIdentifiers,  ///< The two identifiers differ; no replay is implied.
};
/// Evidence for a selected partition and one same-record delta candidate.
/// WORLD current_id is a persistent mark identifier, not an XMT node index
/// (XT 5.3.1, pp. 83-84; XT 2.1.4.4, p. 17). PMARK administration is internal
/// delta metadata, outside the public XT entity catalogue. Neither equal
/// identifiers nor a valid predecessor path proves SolidWorks body membership.
struct SavedStateEvidence {
  /// Source-file-local locator of the selected baseline partition candidate.
  std::optional<std::size_t> baseline_payload_index;
  /// The unique same-record delta candidate; absent when ambiguous/missing.
  std::optional<std::size_t> delta_payload_index;
  /// Raw persistent identifier from the baseline's unique WORLD current_id.
  std::optional<uint32_t> baseline_current_id;
  /// Raw persistent identifier of the delta prefix's current PMARK.
  std::optional<uint32_t> target_identifier;
  /// Identifier comparison only; separate from history and body statuses.
  SavedStateRelation relation = SavedStateRelation::kUnavailable;
  /// Structural PMARK path only; no operation plan or base ownership proof.
  GeometryResolutionStatus history_path_status =
      GeometryResolutionStatus::kUnresolved;
  /// XMT indices from matching baseline mark through current mark, inclusive.
  /// A matching target has a one-element path, requiring no mark transition.
  /// Populated only after complete framing and consistent mark graph checks.
  std::vector<uint32_t> mark_path;
  /// Reason for unavailable comparison or unresolved path. Empty means only
  /// that this structural check succeeded, not that the model is complete.
  std::string issue;
};

/// Borrowing-free reference into one input file. Offsets and payload_index are
/// local locators; retain the input file's identity/hash alongside this result.
struct GeometrySource {
  std::string stream;             ///< Exact source stream name.
  uint64_t container_offset = 0;  ///< Stream record offset in the input file.
  uint64_t payload_offset = 0;    ///< Wrapper offset in the decoded stream.
};
/// One metadata assertion; field names identify their original namespace.
struct GeometryEvidence {
  GeometrySource source;
  std::string field;
  std::string value;
};
/// A BODY decoded in a candidate, without final-body membership certification.
/// Node identities are local to that payload/state. See XT 5.3.4 (p. 88).
struct SavedBodyCandidate {
  uint32_t body_id = 0;
  SavedBodyKind kind = SavedBodyKind::kUnknown;
};
/// One extracted primary-stream payload. Candidates remain available for
/// inspection; no successful export or payload ordering affects selection.
struct SavedGeometryCandidate {
  std::size_t payload_index = 0;
  GeometrySource source;
  GeometrySourceRole role = GeometrySourceRole::kOther;
  ParasolidPayloadKind kind = ParasolidPayloadKind::kUnknown;
  /// Numeric slot in the stream name, interpreted only within role's family.
  std::optional<uint32_t> stream_index;
  bool selected_partition = false;
  /// Framing completeness only; not a geometric or state validity certificate.
  bool decoded_complete = false;
  /// Unique WORLD current_id, when completely decoded as a 32-bit word.
  /// No value is inferred from allocation counters or BODY node identities.
  std::optional<uint32_t> world_current_id;
  std::vector<SavedBodyCandidate> bodies;
  std::string decode_issue;
};
/// Positive resource caps shared across this input's streams and candidates.
struct SavedGeometryOptions {
  uint64_t max_streams = 10000;
  uint64_t max_decoded_bytes = 128u * 1024u * 1024u;
  uint64_t max_xml_bytes = 4u * 1024u * 1024u;
  uint64_t max_xml_nodes = 100000;
  uint64_t max_payloads = 256;
  /// Per candidate; also caps delta packet/entry counts. Total work is
  /// additionally bounded by max_payloads/bytes.
  uint64_t max_nodes_per_payload = 10000;
};
/// Results for an explicitly named configuration, including inactive ones.
/// Configuration identity, history resolution and final-body membership are
/// independent. The resolver establishes metadata/partition identities and
/// checks structural PMARK ancestry; it does not derive operation plans,
/// apply deltas or certify final bodies. PARTITION/PMARK and WORLD describe
/// Parasolid state (XT 1.1.2,
/// p. 8 and 5.3.1, pp. 83-84), not SolidWorks configuration ownership by
/// themselves.
struct SavedGeometryResolution {
  std::string configuration;
  GeometryResolutionStatus configuration_status =
      GeometryResolutionStatus::kUnresolved;
  GeometryResolutionStatus partition_status =
      GeometryResolutionStatus::kUnresolved;
  GeometryResolutionStatus state_status = GeometryResolutionStatus::kUnresolved;
  GeometryResolutionStatus body_membership_status =
      GeometryResolutionStatus::kUnresolved;
  std::optional<uint32_t> partition_index;
  /// Keywords id for ResolvedFeatures; distinct from partition
  /// SourceIndex/swID.
  std::optional<uint32_t> resolved_features_index;
  /// Set only when the manifest has a unique YES row and target flag is known.
  std::optional<bool> was_last_active;
  /// Stored cache flag; false does not certify history reconstruction.
  std::optional<bool> needs_rebuild;
  SavedStateEvidence state_evidence;
  std::vector<GeometryEvidence> evidence;
  std::vector<SavedGeometryCandidate> candidates;
  std::vector<std::string> limitations;
};

/// Resolves SourceIndex and namespace-qualified Features manifest links for an
/// explicitly named configuration. Conflicts, damaged/duplicate selector
/// streams and absent selectors are reported conservatively. A sole partition
/// can be a heuristic candidate, never proof of current state. All returned
/// data is owned. A valid report can remain unresolved/unsupported; Result::Err
/// indicates bad options, I/O or a resource cap. No source bytes or model
/// states are modified.
[[nodiscard]] Result<SavedGeometryResolution> ResolveSavedGeometry(
    const ContainerInspection& inspection, std::string_view configuration,
    const SavedGeometryOptions& options = {});
/// Inspects a supported modern file and calls the same resolver. Container
/// inspection has its existing I/O/decoding behavior and precedes resolver
/// caps.
[[nodiscard]] Result<SavedGeometryResolution> ResolveSavedGeometry(
    const std::filesystem::path& path, std::string_view configuration,
    const SavedGeometryOptions& options = {});
[[nodiscard]] const char* ToString(GeometryResolutionStatus status) noexcept;
[[nodiscard]] const char* ToString(GeometrySourceRole role) noexcept;
[[nodiscard]] const char* ToString(SavedBodyKind kind) noexcept;
[[nodiscard]] const char* ToString(SavedStateRelation relation) noexcept;
}  // namespace openswx
#endif  // OPENSWX_SAVED_GEOMETRY_H_
