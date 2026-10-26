# Parasolid reader and STEP exporter: open issues

This document records the remaining boundaries of `libopenparasolid` and
`libopenswstep`. The original coverage measurements below were recorded at
commit `91889db` (conical frustum export); dated reviews track later changes.
Update the relevant work item when a boundary is extended. This is a work list,
not a promise of universal Parasolid compatibility.

## Systematic trimming work, 2026-10-26

The older counts below are historical corpus measurements, not current
completion percentages. The current issue review distinguishes decoding,
automatic interval binding, exact STEP representation, and saved-state/body
membership. Advancing one stage does not establish success at the next.

| Priority | Limitation | This iteration / next verification |
| --- | --- | --- |
| P0 | Compiler-dependent coincident conic endpoints | Fixed: compare distinct vertex positions against caller tolerance before wrapping angles. The reported CI failure is reproduced with Clang 19 `-O3 -DNDEBUG`; tiny rounding gaps remain unsupported instead of becoming nearly full turns. |
| P1 | Partial intersection edges ending at a closed help seam | Fixed when the existing shared-H/full-period certificate succeeds: recorded direction selects the first/last chart occurrence. Interior duplicates, unproven periods and unknown senses remain unsupported. |
| P1 | General non-circular intersection curves | Open. Crossed cylinders, off-axis cone/cylinder pairs and off-axis sphere/torus pairs are not certified circles. The current sphere/plane circle recovery cannot represent them. The original native cross-bore coupon in `openswx-testdata/verification/cross-bore-001` is the first frozen benchmark; it binds but deliberately still fails whole-body STEP export. |
| P2 | Multiple trim regions on one analytic support | Open. Separate cone sectors and cylinder trims with additional spline loops require an explicit UV-region certificate and per-region flux. Loop count alone does not establish outer/hole or disconnected-region roles. |
| P2 | Non-rectangular or closed-profile spline sweeps | Open. The existing four-edge, translated polynomial-profile certificate does not cover full closed elliptic profiles or multiply trimmed sweeps. Exact support conversion and global trim certification are required together. |
| P2 | Partially trimmed spline edge domains | Open. Domain endpoints need not coincide with edge vertices; bounded inverse queries must recover an unambiguous directed trim without choosing a nearby parameter heuristically. |
| P2 | Oblique spherical cuts | Open for native SPHERE supports in this path. Reorienting a polar frame needs a whole-support equivalence bound and unchanged normal/branch selection, as well as native reference comparison. |
| P2 | Tolerant support pairs outside PLANE/SPUN_SURF | Open. Preserve per-edge error budgets and certify the exact support pair instead of extending approximate chart circles by sampling. |
| P3 | Final-body membership when configuration partitions contain no bodies | Open. Multiple internal payloads can correspond to multiple native bodies; neither taking the last candidate nor unioning all candidates certifies the saved target state. |

For each extension: reduce the failure to an original MIT construction, freeze
native SLDPRT/Parasolid/STEP receipts plus native topology, add positive and
rejection tests, and compare the entire supported result with independent OCCT
checks. A native STEP reference may itself approximate intersection edges;
record that discrepancy against an independent geometric oracle rather than
silently relaxing a geometric proof. Private issue CAD and derived artifacts
stay outside both repositories. Authentication headers are never captured.

## Historical corpus evidence

| Layer | Observed coverage | What success establishes |
| --- | --- | --- |
| Binary decoding | 5,976/5,976 local streams reach a supported closing marker with the larger explicit resource profile; 5,881 finish with default limits | Supported wire framing and field decoding, including captured delta occurrences |
| Recorded topology | All 163 bodies in the older 130-stream selection pass integrated topology validation | Checked ownership lists, reciprocal links and recorded incidence |
| Combined body geometry binding | 56/57 body occurrences in the separate 39-stream selection bind automatically; the remaining occurrence binds with eight explicit fin intervals | Supported curves, intervals, surfaces and vertices are connected to the selected topology, with sampled membership checks |
| STEP export | 40/57 selected body occurrences export; all 40 pass OCCT import and B-rep validity checks | The selected complete body fits the current export subset and passes those independent checks |

These selections are different and must not be combined into a global success
percentage. The 57 body queries include archival partition snapshots and bodies
without faces; they are not 57 automatic conversions of current SolidWorks
parts. Private files and generated comparison artifacts stay outside the repo.
See [reader validation](README.md#validation-and-reference-material),
[combined binding](README.md#combined-body-geometry-binding) and
[STEP validation](../libopenswstep/README.md#build-and-test) for details.

## Reader: format and state reconstruction

### Text `.x_t` input

The reader currently accepts extracted binary Parasolid streams, including
`.x_b` and binary streams embedded in SolidWorks files. It does not implement
the textual XT grammar. Text input needs a separate tokenizer and field decoder
feeding the same model semantics, with resource limits and precise diagnostics.

Completion evidence should include equivalent binary/text synthetic models,
independent text fixtures, malformed/truncated input and preserved units,
identities, senses and parameter values. Binary corpus completion does not
measure text-format support.

### Schema and binary-format coverage beyond the corpus

The built-in catalog is a reviewed subset of 54 base types for schema 13006.
Embedded schema edits and explicitly supported full declarations supply the
effective stream layouts; unknown base definitions and unsupported packet forms
remain explicit stop conditions. All observed streams decoding successfully
is not evidence that every producer version, type or extension is supported.

Extend coverage from reviewed layouts and independent field/boundary
comparisons. Keep unknown encodings explicit rather than skipping bytes or
inferring a layout from familiar-looking data. Large inputs must continue to
use explicit finite budgets; resource-limit stops are distinct from unknown
format errors.

### Automatic delta-history reconstruction

Delta packets, status-framed node occurrences and reference statuses are
preserved. `ApplyDelta` can transactionally apply caller-supplied operations
and identity mappings to a known base state. It does not infer those operations
or select an active revision from PMARK history and packet order.

The missing step is a validated automatic planner mapping wire identities,
reference statuses and history to create/replace/delete operations and the
requested final state. This requires documented semantics or matching
base/delta examples with independently known final snapshots. A successful
wire decode alone is insufficient evidence. Do not assign operation meanings
to status bytes by guesswork.

See [explicit delta application](README.md#explicit-delta-state-application).

## Reader: geometry semantics and validation

### Remaining geometry families

Reading a record is separate from constructing typed geometry, evaluating it,
inverting its surface parameters and binding it to a face. Analytic geometry,
NURBS and supported trim, SP, intersection, offset, swept and spun geometry
already have substantial implementations; these are not wholly missing areas.

General blend reconstruction and remaining procedural, foreign, mesh and
polyline geometry candidates still lack complete typed/evaluation support.
Raw fields may be available even when a typed geometry adapter is absent.
Further adapters need source-defined domains and senses, bounded evaluation,
explicit unsupported dependencies and independent numerical comparisons.

See [typed model geometry](MODEL.md#typed-geometry) and the individual evaluator
sections in the [reader README](README.md).

### Automatic intervals, branches and inverse queries

Some closed/tolerant fin trims still need explicit directed intervals. In the
57-body selection, eight overrides resolve the one remaining automatic binding
failure. This does not establish unique automatic intervals for arbitrary
inputs. Partial spline domains, ambiguous endpoints and periodic branch/turn
choices cannot generally be recovered from endpoint positions alone.

Existing NURBS inversion and intersection solving are bounded local searches.
They do not prove a global closest point, unique solution or absence of other
branches. Periodicity flags do not enable arbitrary domain wrapping.

Extend inference only where source metadata or independently verified geometry
makes the interval/branch unambiguous. Preserve explicit overrides and failures
for unresolved cases; do not substitute a shortest arc or guessed period.

### Whole-geometry B-rep certification

The reader validates recorded topology and checks geometric membership at
selected parameters. It does not certify whole-curve agreement with supporting
surfaces, global self-intersection, manifoldness, shell nesting, hole
classification, consistent material orientation or positive enclosed volume
for arbitrary models. Some of these checks exist for the exporter's restricted
geometry subset; they are not general reader guarantees.

Further certification needs whole-domain curve/surface checks, tolerances for
shared geometry, trim-region classification and shell/material consistency.
Independent OCCT comparisons should supplement targeted analytic tests and
malformed-topology regressions. OCCT accepting a generated STEP file does not
prove that every source feature was interpreted correctly; expected measures
and source correspondence must also be checked.

## STEP exporter: narrower than the reader

The exporter currently preserves exact analytic geometry for supported wire
bodies, planar polygon faces and holes, full conic disks, circular planar holes,
complete cylindrical bands and conical frustums. Closed supported shells include
cylinders and cylindrical/conical tubes. It returns an error for an unsupported
selected body instead of discarding unsupported faces.

Remaining export work includes:

- Mixed line/arc contours and multi-edge conic loops, including rounded planar
  profiles and their holes.
- Elliptical holes and more general curved trim-region validation.
- Partial cylindrical/conical faces, explicit seam fins and apex boundary loops.
- Spherical and toroidal faces, NURBS curves/surfaces and other geometry already
  available from the reader but not yet mapped to STEP.
- Tolerant fin-specific curves and their STEP pcurve/surface-curve mapping.
- Isolated face loops, general mixed-dimensional bodies, compound/assembly
  grouping, multiple material regions and separate solid void shells.
- Explicit assembly placement and configuration/history selection integration.
  Current APIs require payload/body selection and source length units; they do
  not infer these choices or apply component placement automatically.

A parser improvement is not automatically an exporter improvement. Each new
STEP subset needs preserved source topology and senses, bounded geometric
validation, atomic rejection of unsupported mixtures, and independent import,
B-rep, topology and expected-measure checks. Account explicitly for periodic
seams and boundary splits introduced by the importer.

The full current subset and its exclusions are maintained in the
[STEP exporter README](../libopenswstep/README.md#supported-geometry).

## Suggested work order

1. Extend exact STEP boundary/face mapping for geometry already present in the
   reader, using the failing private cases and synthetic examples to choose
   focused subsets. Mixed line/arc planar profiles and spherical/toroidal faces
   are useful next candidates.
2. Extend source interval and geometry adapters where concrete failures have
   sufficient metadata and independent reference results.
3. Develop broader B-rep certification alongside these subsets, keeping reader
   checks and exporter-specific guarantees distinct.
4. Implement automatic delta planning when the missing state/identity semantics
   can be validated. Text `.x_t` input is a separate format extension whose
   priority depends on actual input requirements.

Preserve the MIT implementation, explicit `Result` errors, source coordinates
and parameter units, finite resource bounds and the kernel-independent library
core. OCCT/OCP remains an optional validation dependency. Add no private CAD,
extracted streams, PDFs or generated private exports to the repository.
