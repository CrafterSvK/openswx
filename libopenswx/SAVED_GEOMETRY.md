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

## Reconstruction gaps and native delta audit

The original `saved-geometry-001` fixtures were also inspected independently
with `parasolid_inspect --nodes` after extracting their Parasolid streams.
An empty reference-map-shaped packet ending in the raw word zero previously
stopped the sheet delta at offset 152. The reader now retains this bounded
ten-byte packet, without assigning operation or state semantics to its word.
Maps containing entries still require a nonzero terminal word.

| Snapshot | Partition stream slot | Delta framing | Historical PMARK occurrences | Delta node images | BODY images |
| --- | --- | --- | --- | --- | --- |
| baseline | 0 | complete | 2 | 2 | 0 |
| default-active | 0 | complete | 2 | 2 | 0 |
| default-active | 1 | complete | 3 | 19 | 1 |
| two-active | 0 | complete | 2 | 2 | 0 |
| two-active | 1 | complete | 3 | 18 | 1 |
| sheet | 0 | complete | 2 | 1 | 0 |

Occurrences exclude the leading current PMARK. Node images exclude the leading
PARTITION/current-PMARK administration nodes and are not applied operations.
These are observations on these exact saved fixtures, not format guarantees.
All six streams decode without diagnostics. Different image counts between
the two multiconfiguration snapshots do not establish a change in native body
membership: the reopened configuration queries still report one and two bodies.

The sheet delta has one status-framed WORLD image, no BODY image, and two
historical marks. Its selected base also has only WORLD. The native reopened
document has one sheet, stored separately in internal feature geometry.
Consequently this pair does not yet provide a demonstrated route to the sheet
body. Blindly replaying all images, or adding every internal BODY, would not
establish final membership. The missing SolidWorks association must be explained
alongside kernel delta semantics.

Remaining stages, in dependency order:

1. **Saved target and baseline association.** Establish which saved kernel
   mark belongs to the selected configuration and which base represents its
   starting state. A same-stream delta is a candidate association, not proof
   of that relationship. Neither the largest identifier nor serialization
   order establishes the target.
2. **History path validation.** Resolve PMARK predecessor/successor/sibling
   links, repeated occurrences and direction against that base and target.
   Detect missing, contradictory and cyclic history rather than selecting a
   branch by packet order. Reading the complete history bytes is already
   supported for these fixtures; constructing a verified replay path is not.
3. **Automatic operation planning.** Establish wire-reference/status namespaces,
   persistent identities, create/replace/delete operations and any inherited
   fields. `ApplyDelta` already checks an explicit whole-node plan transactionally;
   it does not derive that plan or reconstruct partial node images.
4. **Final body membership.** Validate the reconstructed WORLD/BODY graph and
   its SolidWorks ownership, including sheet, compound/child and helper bodies.
   Explain the sheet fixture's separate geometry before accepting FeatureBodies
   or ResolvedFeatures as final bodies. WORLD roots alone cannot establish
   SolidWorks final/helper classification.
5. **Independent native verification.** Compare complete body sets and geometry
   after save/reopen, with creation, deletion, modification, suppression,
   inactive configurations and sheet/solid transitions. A matching count or
   successful STEP export alone cannot validate identity or currency.

For a reproducible offline audit, run from the OpenSWX checkout with the corpus
as a sibling. Repeat for each snapshot and each `kind: deltas` file named by
the extraction manifest:

```sh
build/asmbox/swx_dump --extract-parasolid /tmp/sheet-state-audit \
  ../openswx-testdata/verification/saved-geometry-001/sheet.sldprt
build/libopenparasolid/parasolid_inspect --nodes \
  /tmp/sheet-state-audit/deltas/payload-2-offset-277.bin
```

The displayed filename is specific to this hashed fixture, not a stable model
identifier. This decoder improvement does not change resolver statuses:
automatic state reconstruction remains `unsupported`, and final-body membership
remains `unresolved`.
