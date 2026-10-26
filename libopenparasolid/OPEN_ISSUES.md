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
| P1 | Chart normals computed at coarse membership tolerance | Fixed for supported inversions: prefer the solve residual goal before the chord/tangent recurrence, retaining the source chart allowance and reporting membership fallbacks. Binding, evaluation and STEP circle recovery use the same policy. The investigated spring now binds all 15 faces without a larger tolerance. |
| P1 | General non-circular intersection curves | Open. Crossed cylinders, off-axis cone/cylinder pairs and off-axis sphere/torus pairs are not certified circles. The current sphere/plane circle recovery cannot represent them. The original native cross-bore coupon in `openswx-testdata/verification/cross-bore-001` is the first frozen benchmark; it binds but deliberately still fails whole-body STEP export. |
| P2 | Multiple trim regions on one analytic support | Open. Separate cone sectors and cylinder trims with additional spline loops require an explicit UV-region certificate and per-region flux. Loop count alone does not establish outer/hole or disconnected-region roles. |
| P2 | Non-rectangular or closed-profile spline sweeps | Open. The existing four-edge, translated polynomial-profile certificate does not cover full closed elliptic profiles or multiply trimmed sweeps. Exact support conversion and global trim certification are required together. |
| P2 | Partially trimmed spline edge domains | Extended: open nonrational clamped 3D B_CURVEs with a certified positive endpoint-chord projection now recover partial intervals. Rational piecewise-Bezier curves up to degree seven now use an additional positive Bernstein numerator certificate; an original native rational trim coupon binds completely. Other rational layouts, periodic, unclamped, SP and uncertified projections remain explicit. |
| P2 | Oblique spherical cuts | Extended for native SPHERE caps and coaxial bands: whole-support and circle-frame error bounds consume the unchanged tolerance before existing profile/winding checks. An equivalent-pole synthetic fixture is compared with an original native sphere cap; the native saved file itself was already supported. Noncoaxial cuts remain open. |
| P2 | Torus sectors bounded by complete meridians | Extended for regular ring supports: two whole-circle/profile certificates and opposite winding select minor or major azimuth sectors. Original native 90°/270° revolutions now export completely and have zero two-way solid differences against native STEP. Partial meridians and general contours remain open. |
| P2 | Negative-major-radius torus branches | Open. Valid XT lemon branches require an exact STEP support/normal/trim equivalence proof; changing the radius sign or applying the ring certificate is insufficient. |
| P2 | Tolerant support pairs outside PLANE/SPUN_SURF | Extended for vertex-bounded CYLINDER/rational B_SURF minor arcs with affine pcurves. Outward tensor bounds certify whole traces; existing circle or isolated plane-intersection masters are checked. Source allowances remain opt-in. General support pairs remain open. |
| P3 | Final-body membership when configuration partitions contain no bodies | Open. Multiple internal payloads can correspond to multiple native bodies; neither taking the last candidate nor unioning all candidates certifies the saved target state. A successfully exported internal body can also differ completely from the native final body; exporter success is not membership evidence. |

### Partial polynomial spline intervals

`openswx-testdata/verification/spline-trim-001` freezes two original native
SolidWorks 2025 SP5 prisms with Boolean cuts through cubic profiles. Source
B_CURVE domains remain `[0,1]`, while actual edge intervals begin at `5/12` and
`1/2`. Both bodies now bind and export completely with five faces, nine edges
and six vertices. The second profile has a negative projected derivative
control, but its derivative is strictly positive everywhere. Bounded Bernstein
subdivision proves that fact rather than rejecting the curve from one control
sign. The same certificate is reused for transverse spline-sweep inversion.
Neither proof changes the source controls, knots, parameter domains or STEP
geometry. Independent polynomial volume integrals and native STEP comparisons
verify both original constructions, including two-way Boolean differences.

Recovery is deliberately narrower than a general spline inverse. It certifies
one global monotone projection of an open, clamped, nonrational 3D curve, then
bisects in original knot units and checks the candidate's full 3D residual.
Each endpoint has at most 130 evaluations and a 32-million-unit work estimate.
Single-span derivative refinement is limited to degree seven, depth 16 and one
million averaging steps. Uncertified folds, off-curve vertices, exhausted
budgets and ambiguous source branches remain explicit errors. Complete-domain
rational/unclamped binding and caller-supplied intervals retain their previous
behavior; partial SP/rational/periodic/unclamped recovery is not implied.

For each extension: reduce the failure to an original MIT construction, freeze
native SLDPRT/Parasolid/STEP receipts plus native topology, add positive and
rejection tests, and compare the entire supported result with independent OCCT
checks. A native STEP reference may itself approximate intersection edges;
record that discrepancy against an independent geometric oracle rather than
silently relaxing a geometric proof. Private issue CAD and derived artifacts
stay outside both repositories. Authentication headers are never captured.

### Spring-shaped parts and rational intervals

A local confidential tension-spring investigation separates three boundaries:
its selected configuration partition contains a complete decoded solid with
four rational B-spline surfaces and rational spatial edges, rather than only
an analytic HELIX form. HELIX_SU_FORM/HELIX_CU_FORM auxiliary records are decoded,
but missing analytic helix reconstruction alone does not explain this case.
The old first failure was partial rational curve inversion. The bounded
piecewise-Bezier certificate now advances beyond that edge without changing
geometry or tolerance. The later trim discrepancy was caused by coarse chart UV inversion stopping
at the chart-membership allowance: its sampled normals changed the XT
chord/tangent parameter recurrence. Intersection evaluation and automatic
interval binding now request the solve tolerance as a preferred normal goal,
with the original chart allowance retained and fallback counts exposed.
The investigated spring now binds all 15 faces at the original tolerances.
The first tolerant CYLINDER/B_SURF edge now passes recovery when source edge
tolerances are explicitly enabled: its roughly 1.64 µm source allowance covers
a roughly 1.11 µm pcurve separation; strict 0.1 µm recovery still rejects it.
The next whole-body preflight boundary is "Intersection circle requires
complete accurate ring loops". This does not certify later non-circular
intersections or the complete body. General rational
B_SURF trimming/serialization remains an additional independent STEP limitation.
No whole-spring export success is claimed.

Private investigation files and native references stay outside the repository.
The independent MIT native rational coupon in
`openswx-testdata/verification/rational-trim-001` reproduces the reader boundary:
it binds all five faces and now exports the complete rational swept body.
`verification/rational-sweep-001` records the new whole-body native comparison
without changing the earlier frozen unsupported capture.
Original synthetic spatial rational tests also make two turns in XY while
rising in Z; these are rational arcs with monotone height, not fitted or exact
constant-pitch helices. They exercise the missing branch certificate without
copying any private spring geometry.

Next work: certify general rational/periodic surface trim regions, then serialize
original weighted B-spline surfaces and curves with independent native whole-
body references. Supporting an auxiliary helix record does not substitute for
these steps. Internal wire/sheet payloads must not be promoted to a complete
spring merely because they can be exported or displayed.

### Tolerant cylinder / rational carrier arcs

The new subset retains two full-domain affine polynomial SP pcurves between a
CYLINDER and an open rational piecewise-Bezier B_SURF (axis degrees one to
three, positive weights). The cylinder has constant source height and supplies
an exact minor circle. Outward-rounded homogeneous tensor restrictions bound
the entire other pcurve against that circle, with finite work and depth limits.
No curve fitting or sampled equivalence acceptance is used.

A recorded master must be an explicitly trimmed matching circle or an
intersection of the same B_SURF and a plane. The latter requires explicit
solver settings, ordinary limits, a selected trim within one active chart
chord, and a certified plane graph U(V) with nonzero signed U derivative.
The full active chart continuation is isolated even when it extends beyond
the trimmed edge. Endpoint residual bounds consume the same edge allowance.
Multichord trims, closed/periodic carriers, rational pcurves, arbitrary master
branches and other support pairs remain unsupported. Missing masters are also
permitted only after the complete paired trace certificate succeeds.

`verification/tolerant-cylinder-001` freezes an original MIT native rational
quarter-cylinder carrier. Synthetic tests construct the tolerant arrangement
and interior bow separately; the native file does not claim that topology.
OCC independently checks the recovered circle against the native sheet.
This extends edge mapping only; general rational face trim serialization and
whole-spring export remain open. Both source pcurves and topology are retained.

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
inputs. The certified partial polynomial subset above now extends automatic
recovery; ambiguous endpoints and periodic branch/turn choices still cannot
generally be recovered from endpoint positions alone.

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

The exporter supports reviewed analytic and polynomial subsets, including
mixed curved planar regions, selected elliptic/spline holes, partial cylinder
and cone patches, sphere caps/bands, trimmed tori and complete ring-torus faces.
Native SPHERE caps/coaxial bands can reframe the polar axis only after a
whole-support equivalence bound consumes part of the unchanged tolerance.
Regular ring-torus sectors now accept two complete meridian circles, including
major azimuth intervals selected by winding. Native quarter/three-quarter
revolutions and an independent equivalent-pole sphere reference are captured
in `openswx-testdata/verification/analytic-cuts-001`. Noncoaxial sphere cuts,
partial meridians, negative-major-radius tori and general torus contours remain
open; this extension does not establish final-body membership.
Supported polynomial and rational sweeps use exact tensor B-spline surfaces.
The rational subset retains identical corresponding profile weights and uses
bounded area/flux enclosures for convex planar caps and signed shell volume.
This does not enable arbitrary rational B_SURF faces or general trim loops. The following
are current limitations rather than the original frustum-era work list:

- General non-circular intersection curves and remaining curved trim regions
  without a whole-domain simplicity/containment certificate.
- Multiply trimmed or disconnected analytic faces, general explicit seam fins,
  and spherical pole/complete-sphere topology outside supported cap/band paths.
- Arbitrary NURBS source surfaces, closed-profile and non-rectangular sweeps
  outside the certified exact tensor-surface subset.
- Tolerant fin-specific curves without an established STEP pcurve/surface-curve
  mapping and whole-edge source error certificate.
- Isolated face loops outside the supported cone-apex subset, general
  mixed-dimensional bodies, compound/assembly grouping, multiple material
  regions and separate solid void shells.
- Explicit assembly placement and configuration/history selection integration.
  Current export APIs require payload/body selection and source length units;
  they do not infer final-body membership or component placement.

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
   focused subsets. Non-circular intersections and multiply trimmed analytic
   supports are current candidates; earlier mixed planar and analytic face
   subsets already have dedicated support.
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
