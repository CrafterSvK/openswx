# Saved geometry identities and unresolved model state

`openswx/saved_geometry.h` provides `ResolveSavedGeometry` for an explicitly
named SolidWorks configuration, including an inactive one. The overloads accept
a file path or an existing `ContainerInspection`. Results own all their data.
The resolver reads existing saved metadata; it does not rebuild features.

```cpp
#include "openswx/saved_geometry.h"

auto result = openswx::ResolveSavedGeometry(path, "Standard");
if (!result.ok()) {
  // Handle I/O, invalid input or resource limits.
  return;
}
const auto& resolution = result.value();
// A resolved partition identity alone is insufficient for final-body display.
```

## Independent status fields

| Field | Meaning |
| --- | --- |
| `configuration_status` | Whether stored metadata identifies a partition index for the requested configuration |
| `partition_status` | Whether one extracted configuration partition matches that identity, or is only a sole-partition heuristic |
| `state_status` | Whether the saved target state has been established; associated delta candidates currently produce `unsupported` |
| `body_membership_status` | Whether the complete final SolidWorks body set has been established; currently `unresolved` |

The vocabulary is `resolved`, `heuristic`, `unresolved`, `unsupported`.
The first implementation resolves configuration/partition identities. It never
certifies a reconstructed target state or final body membership. A successful
`Result` means a report was produced, not that the saved model was resolved.

`was_last_active` and `needs_rebuild` are independent nullable metadata fields.
A uniquely last-active configuration is not a selector for a different requested
configuration. A cache flagged for rebuild remains so, even with a proven
partition identity. Absence of a rebuild flag is not a freshness certificate.

## Stored associations

The resolver reads direct `Keywords/Configuration` definitions and their
`SourceIndex` and `id` attributes. It accepts the observed native one-byte `86`
serialization tag before UTF-8 Keywords XML, as well as plain XML. `SourceIndex`
selects the partition namespace. Keywords `id` identifies a ResolvedFeatures
slot and is returned separately as `resolved_features_index`.

The `swXmlContents/Features` manifest must use the namespace
`http://www.solidworks.com/sw2003/schema`, including when a prefix is used.
A configuration row's `swModelRef` must identify exactly one `swModel` whose
configuration name matches the request and whose `swConfigurationId` agrees
with the row's decimal `swID`. The XML row `id` is a separate identity. Matching
rows establish the requested configuration's partition index even when another
configuration has the unique `swMostRecentConfiguration="YES"` flag.

Explicit SourceIndex and manifest bindings must agree. Invalid, contradictory,
duplicate or unreadable selector records are not overridden by a precedence
rule. Stored assertions retain their stream names, container offsets, fields
and values. Missing evidence remains absent rather than being filled by an
ordinal or the payload number.

Source names with and without `Contents/` are recognized. Numeric stream slots
remain family-scoped. `FeatureBodies/LocalBodies` and `ResolvedFeatures` are
reported as `internal_feature_geometry`: this describes provenance, not proof
that each body is intermediate. Ghost partitions are retained but never selected.

A sole configuration partition is a heuristic only when the configuration is
known from metadata and has no conflicting or missing explicit target.
Duplicate partition chunks block selection even if the container's existing
primary-chunk policy kept only one payload. Physical order, maximum IDs,
export success and geometric measures do not select the current model.

## Bodies and history

Each ordinary candidate is schema-decoded with bounded work. Its BODY nodes and
source dimensional kinds (`solid`, `sheet`, `wire`, `general`, `unknown`) are
returned in that candidate's `bodies` collection. This is an inventory, without
final-body, topology, exportability or completeness certification. An incomplete
parse withholds the BODY list. Resource exhaustion returns an explicit error.
No bodies are combined across payloads, and no hidden body is silently removed.

Node IDs and payload indices are local to the source file/state. Store the input
file identity or hash alongside the report. Candidate locators include the exact
stream and container/wrapper offsets; they are not persistent model identifiers.

Deltas in the same stream record as the selected partition are recorded as
associated candidates. This does not prove a particular dependency edge or an
application order. Equal numeric indices across unrelated stream families do
not associate a delta. No delta is applied by this resolver.

The separate `openparasolid/delta.h` API can apply a caller-resolved plan.
Automatic PMARK graph traversal, wire-status/operation interpretation and
SolidWorks final-body membership remain open work. XT PARTITION/PMARK metadata
and WORLD body roots describe kernel state (XT 1.1.2, p. 8; 5.3.1, pp. 83-84);
they do not alone prove SolidWorks final/helper body roles.

## CLI and verification

```sh
swx_dump --resolve-saved-geometry "Standard" part.SLDPRT
SWX_DUMP_RESOLVE_SAVED_GEOMETRY=Standard swx_dump part.SLDPRT
```

The command-line value overrides the environment variable. The action is
exclusive with extraction, stream inspection and drawing export actions.
JSON contains separate statuses, source evidence, limitations and per-payload
`body_candidates`. Exit zero means a valid report, including unresolved results.
Callers must inspect the status fields before making currency/completeness claims.

Unit tests cover conflicting selectors, separate namespaces, inactive configs,
Unicode, native XML framing, duplicate chunks, ghost/internal candidates and
resource limits. Native SolidWorks 2025 SP5 captures in the separate
`openswx-testdata/verification/saved-geometry-001` dataset verify one/two-body
configurations and last-active changes after save and reopen. A separate native sheet case has an empty selected baseline (WORLD only) and
a sheet BODY in internal feature geometry, while SolidWorks reports one sheet.
This demonstrates why an explicit partition identity cannot certify body
completeness. The native oracle confirms these fixtures; the API still does not promote their unapplied deltas
to a generic resolved saved state. Source-less FeatureBodies cases remain
unresolved until a verified state/body-membership association is implemented.
