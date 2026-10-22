# Parasolid reader: open issues

This document records the remaining boundaries of `libopenparasolid`, using
commit `ec54a43` as the implementation baseline. Update this list and its
coverage figures when a boundary is extended. It is a work list, not a promise
of universal Parasolid compatibility.

## Current evidence

| Layer | Observed coverage | What success establishes |
| --- | --- | --- |
| Binary decoding | 5,976/5,976 local streams reach a supported closing marker with the larger explicit resource profile; 5,881 finish with default limits | Supported wire framing and field decoding, including captured delta occurrences |
| Recorded topology | All 163 bodies in the older 130-stream selection pass integrated topology validation | Checked ownership lists, reciprocal links and recorded incidence |
| Combined body geometry binding | 56/57 body occurrences in the separate 39-stream selection bind automatically; the remaining occurrence binds with eight explicit fin intervals | Supported curves, intervals, surfaces and vertices are connected to the selected topology, with sampled membership checks |

These selections are different and must not be combined into a global success
percentage. The 57 body queries include archival partition snapshots and bodies
without faces; they are not 57 automatic conversions of current SolidWorks
parts. Private files and generated comparison artifacts stay outside the repo.
See [reader validation](README.md#validation-and-reference-material),
[combined binding](README.md#combined-body-geometry-binding) for details.

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
for arbitrary models. Such guarantees require additional geometric validation
and are not implied by successful reader queries.

Further certification needs whole-domain curve/surface checks, tolerances for
shared geometry, trim-region classification and shell/material consistency.
Independent OCCT comparisons should supplement targeted analytic tests and
malformed-topology regressions. OCCT accepting a generated STEP file does not
prove that every source feature was interpreted correctly; expected measures
and source correspondence must also be checked.

## Suggested work order

1. Extend source interval and geometry adapters where concrete failures have
   sufficient metadata and independent reference results.
2. Develop broader B-rep certification alongside these subsets, keeping recorded
   incidence checks and whole-geometry guarantees distinct.
3. Implement automatic delta planning when the missing state/identity semantics
   can be validated. Text `.x_t` input is a separate format extension whose
   priority depends on actual input requirements.

Preserve the MIT implementation, explicit `Result` errors, source coordinates
and parameter units, finite resource bounds and the kernel-independent library
core. OCCT/OCP remains an optional validation dependency. Add no private CAD,
extracted streams, PDFs or generated private exports to the repository.
