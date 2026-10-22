# libopenparasolid

A C++20 static library and command-line inspector for extracted binary
Parasolid transmit streams. Licensed under [MIT](LICENSE).

The library depends only on the C++ standard library. The CLI uses
nlohmann/json, and the tests use GoogleTest. SolidWorks containers and
compression belong to `libopenswx`; this library reads the extracted bytes.

The [model builder](MODEL.md) adds an owned reference graph and typed analytic/
NURBS data for complete partition and transmit streams. Use
`parasolid_inspect --geometry --summary part.x_b` to inspect its findings.
Geometry mode and existing options also support `OPENPARASOLID_*` environment
settings, with command-line precedence. Integrated body incidence validation,
face parameter binding and explicit-plan delta application are described below. Automatic delta-history resolution and
full geometric BRep certification remain separate work.

## Integrated body validation and face binding

`openparasolid/brep.h` connects the previously independent topology queries.
`ValidateBodyTopology(model, body, options)` returns a `ValidatedBodyTopology`
with the owned body boundary, radial edge boundaries and vertex incidences.
It verifies body edge/vertex lists and their reciprocal links, both face
sides, complete loop/radial/vertex incidence, and source entities claiming the
selected body but missing from its lists. Compound groups are checked when
present, including unlisted children claiming the parent. Returned edges and
vertices are sorted by ID; region hierarchies retain source order.

`BodyValidationOptions.traversal` contains the existing aggregate body and
child limits. `max_model_nodes` (300,000), `max_model_references` (5,000,000),
`max_radial_fins` (100,000) and
`max_vertex_fins` (100,000) bound the whole-model scan and the additional
aggregate incidence queries. Every limit is positive. Local traversal errors
retain their codes/offsets; completeness errors return `kInvalidRecord`.
This certifies selected recorded incidence, without proving connectedness,
manifoldness, geometric membership, positive volume or tolerance validity.
Geometry/construction lists and external or omitted states remain outside it.

`BindFaceTrimming(model, face, intervals, tolerance, options)` binds the source
loop hierarchy to exact curve IDs and directed parameter intervals, without
tessellation. Supply one `FinParameterInterval` per non-isolated fin, including
explicit unwrapped periodic branches. The automatic overload omits `intervals`
and resolves lines and accurate edge trim intervals. Edge trims retain their
transmitted parameters ordered by fin sense, including closed/vertex-free
edges and decreasing basis parameters (XT section 5.2.1.6). Vertex-free edge
trims must close within the caller's tolerance. Automatic trim binding checks
both stored trim points against curve evaluations, then uses the usual vertex
and surface checks. Fin trims still require unambiguous vertex correspondence;
closed/vertex-free fin trims need explicit intervals because their edge/curve
orientation is not established here. No source interval is replaced by an
inferred shortest or full-period arc. Accurate circle/ellipse edges use curve
and fin senses (XT section 5.2.8) to select the directed arc across periodic
seams.
Closed edges with the same endpoint vertex, and vertex-free edges, use one
full turn; the latter start at parameter zero. Distinct vertices with coincident
angular parameters, conics stored on fins, unknown senses and other curve types
return `kUnsupportedFormat`. Supply explicit intervals for those cases or for
caller-selected additional turns. No shortest-arc assumption is made.

The resulting `FaceTrimming` owns its loops and `BoundFinGeometry` records.
Fin curves take precedence over edge curves. Known accurate-edge curve sense
must agree with the directed interval. Vertex points are compared with
the interval endpoints. Endpoints and midpoint must lie on the face surface
within the finite nonnegative source-unit tolerance. Isolated vertices are
mapped to UV. `EvaluateBoundFin(model, binding, fraction, tolerance)` accepts
fractions in [0,1], returns position, derivative with respect to that fraction,
and surface UV. Directed reversal therefore reverses/scales the tangent.
SP-curves on the face's surface use their transmitted 2D curve directly.
When an SP-curve references another surface, its evaluated 3D point is mapped
to the face by bounded surface inversion, as for other curve types; source UVs
are not reused across surface IDs. No periodic UV
continuation, outer/hole classification or inside/outside test is inferred.
Finite membership checks do not prove agreement at every parameter.

Synthetic regressions cover missing owner lists and incidence, both face
sides, source loops, directed/reversed intervals, UV curves, mismatched
surfaces, oriented conic arcs and closed/vertex-free periodic edges. All 163
bodies in the existing 130-stream selection pass the integrated topology
validation. Before source-trim interval binding, automatic binding succeeded
at tolerance 1e-7 for 4,505 of 4,540 selected faces whose curves are lines,
conics or trims of those curves and whose
surfaces are supported analytics. These contain 21,227 fin uses, including
6,118 direct conic uses (1,197 closed/vertex-free). The remaining 35 faces
reported 21 curves requiring explicit endpoints and 14 ambiguous/nonmatching source
trims. This is selected coverage, not evidence that all procedural surfaces
or arbitrary parameter branches are supported. Private inputs and comparison
tools stay outside the repo.

The source-trim extension has a separate before/after comparison using 39
freshly extracted streams from 20 private SolidWorks files at tolerance 1e-7:
all 542 selected analytic faces bind, versus 541 previously. The previously
unsupported face uses a vertex-free edge trim. All previous successes are
preserved. Synthetic tests cover both fin directions and basis senses, closed
trim branches, invalid stored points, open vertex-free trims and ambiguous
fin trims. This comparison is independent of the earlier 130-stream selection.

## Full spline domains in face bindings

Automatic binding supports direct NURBS curves and SP-curves when the complete
natural spline domain's endpoints match the oriented vertices unambiguously
within the caller's tolerance. For SP-curves the domain comes from the 2D
NURBS parameter curve, while endpoint matching uses the evaluated 3D curve.
Both fin directions are supported; known accurate-edge sense must agree with
the interval. Unknown accurate-edge sense, missing vertices, partial intervals
and ambiguous/coincident endpoints still require explicit intervals or
partial overrides.

Axis and control validation inherit the NURBS evaluator's validity/resource
checks. Domain bounds are expanded knots at `degree` and `control_count`,
including unclamped axes, rather than the first/last distinct knot. Original
knot units, homogeneous weights and source geometry are preserved. No curve
point inversion, periodic wrapping or extra-turn inference is performed.
All existing sampled membership and topology checks still apply.

SP-curves on another surface keep their original surface reference. Their 3D
points must invert onto the selected face within the binding tolerance, and
returned UVs belong to that face. This permits different parameter frames on
coincident surfaces without treating the source UVs as face UVs. Off-face
support geometry remains a membership error. Surfaces without supported
inversion remain unsupported for this mapping.

Synthetic tests cover rational and polynomial curves, clamped/unclamped
domains, both fin/curve directions, interior and ambiguous endpoints, invalid
knots/weights and SP mapping between different UV frames. Current real-file
coverage of the combined pipeline is listed under intersection binding below.

## Binding all faces of a body

`BindBodyFaces(model, body, tolerance, options)` first validates the complete
recorded body topology, then automatically binds the loop geometry of every
distinct face. The explicit overload adds a span of `FinParameterInterval`
before `tolerance`; it requires exactly one interval per non-isolated fin in
the selected body's faces. Input interval order is arbitrary; duplicate,
missing and foreign fin IDs are invalid options. Isolated loops need no
intervals. Both overloads return `BodyFaceBinding{topology, faces}`.

The owned result retains regions, shells and front/back uses in
`topology.boundary`. `faces` contains one binding per distinct face, sorted by
face ID, even when a face has multiple source uses. All face bindings inherit
the existing surface, endpoint, direction and sampled membership checks.
Errors preserve their code and source offset; per-face failures additionally
identify the body and face. Failure discards the entire result.

`BodyFaceBindingOptions` separates complete topology limits (`topology`),
per-face traversal limits (`face`), optional intersection solver settings
(`intersection`) and aggregate geometry limits (`max_faces`, `max_fins`).
Both aggregate limits must be positive. Fins are counted across unique faces,
including one per isolated vertex loop; front/back uses do not multiply this
allowance. Explicit interval input is also bounded by `max_fins`. Geometry
options are validated even for bodies without faces.

```cpp
BodyFaceBindingOptions options;
options.intersection = IntersectionEvaluationOptions{1e-7, 1e-9};
auto body_faces = BindBodyFaces(model, body, intervals, 1e-7, options);
```

Wire edges and acorns are retained as validated topology, without binding
their geometry. A body without faces can therefore succeed with an empty
`faces` vector. This API does not certify manifoldness, inside/outside,
positive volume or curve membership at every parameter.

In the separate 39-stream private selection, automatic body-face binding at
1e-7 succeeds for 51 of 57 body occurrences: 21 have 367 bound faces with
1,752 fin uses, and 30 have no faces. Six bodies retain explicit unsupported
results: four require directed curve intervals, one has a fin trim needing
explicit endpoints/intervals, and one needs unsupported surface inversion.
Synthetic tests cover shared front/back uses, multi-face aggregate budgets,
isolated loops, explicit intervals, stored intersection settings, topology
failure, late geometry failure and bodies without faces.

## Partial fin intervals

`BindFaceTrimmingWithOverrides` and `BindBodyFacesWithOverrides` accept a span
of directed `FinParameterInterval` values for selected fins. Each supplied
interval replaces automatic inference for that fin; all unspecified fins are
resolved automatically. This lets callers supply the few known branch choices
needed for otherwise ambiguous SP/NURBS/intersection curves. Empty overrides
retain the automatic behavior; the existing complete explicit overloads still
require every non-isolated fin interval.

```cpp
auto face_bound = BindFaceTrimmingWithOverrides(
    model, face, overrides, tolerance, face_options, solver_options);
auto body_bound = BindBodyFacesWithOverrides(
    model, body, overrides, tolerance, body_options);
```

Overrides retain the complete explicit path's direction, vertex, surface and
sampled membership checks. Invalid or duplicate intervals, foreign fin IDs and
isolated-loop fins are rejected. The body path groups overrides by unique face
and keeps complete topology validation, aggregate resource limits and atomic
results. Unspecified unsupported branches remain errors. Solver settings are
retained in the bound fins. An interval does not enable an unsupported surface
or curve evaluator, and source trim parameters are still enforced.

Synthetic tests bind a previously unsupported SP-curve fin with one override
while resolving the other fin automatically. They also cover mixed intersection
bindings, strict direction checks, foreign/isolated IDs and resource limits.
In the 39-stream private selection, 591 reversed-order overrides taken from
previously validated bindings reproduce all 1,752 fin intervals in 367 faces.
All 51 previously supported body occurrences still bind, including 30 without
faces.
The other six bodies retain their unsupported result with empty overrides.
This checks mixed binding and regressions; it does not establish missing branch
choices for those six real cases.

## Intersection curves in face bindings

Both `BindFaceTrimming` overloads accept optional `IntersectionEvaluationOptions`
after the traversal options. Supplying them enables the existing bounded local
intersection solver, including intersections reached through source trims:

```cpp
IntersectionEvaluationOptions solver{1e-7, 1e-9};
auto bound = BindFaceTrimming(model, face, intervals, 1e-7, {}, solver);
// Automatic inference uses source trims or unique chart-point matches:
auto automatic = BindFaceTrimming(model, face, 1e-7, {}, solver);
```

With solver options, direct intersections can resolve an interval when each
oriented vertex matches exactly one stored chart point within the binding
tolerance. The matches must be distinct. The chart is first validated and
parameterized by the existing evaluator; matches can select interior chart
points and retain the original parameter units. Accurate edges additionally
require known curve sense agreeing with the directed endpoint order. Fin
curves use their oriented vertex correspondence.

For an endpoint without a stored chart match, bounded recovery projects the
point onto each chart chord to obtain a candidate chord-normal plane parameter.
Candidates stay inside their chart segment and retain the original chart
parameter units. The existing intersection solver evaluates the actual curve
at each candidate; its 3D residual must meet the binding tolerance. Curve
positions are not interpolated from the chord. Repeated identical candidate
parameters are evaluated once; distinct successful parameters are ambiguous.
Query errors propagate, and an unsuccessful local search is not proof that
no curve preimage exists.

Per fin, recovery permits at most 64 candidate curve evaluations shared by
both endpoints, plus 1,000,000 estimated chart-point preparation visits
including the initial chart preparation. Exhaustion returns `kResourceLimit`,
even after a candidate matched. Subsequent membership samples retain their
existing evaluation limits. Closed/vertex-free edges, ambiguous stored matches
and unrecoverable parameters still require explicit intervals or partial
overrides. Recovery never extends the chart or guesses terminator branches.

The automatic overload also resolves source trims of intersections using their
transmitted parameters. Omitting solver options retains the explicit
unsupported result for intersections.
Invalid solver options are rejected even when the face has no curve samples.

Each `BoundFinGeometry` owns a copy of the solver settings in `intersection`.
`EvaluateBoundFin` reuses those settings without requiring the original options
object. Chart membership, Newton residual and optional regularity checks retain
the existing solver semantics. Face membership and vertex matching still use
the separate binding tolerance. Directed tangents retain their scale/reversal
with respect to the fin fraction. Chart bounds, source trim bounds, dependency
and work limits remain in force. This provides local chart evaluation, without
certifying a global intersection branch or extending to terminators. Stored
point matching precedes chord-plane recovery; all resulting intervals still
undergo the existing finite face/vertex membership checks.

Synthetic tests cover plane/plane and curved plane/sphere intersections, stored
settings, reversed fins, source trims, independent face tolerances, invalid
options and chart bounds. All 542 faces in the separate 39-stream selection
still bind with and without the supplied solver options. That selection has no
additional eligible trimmed-intersection faces, so that filtered comparison
establishes regression coverage. The expanded body comparison below exercises
direct intersections from real files.

With `IntersectionEvaluationOptions{1e-7, 1e-9}` and binding tolerance 1e-7,
the complete pipeline binds 55 of 57 body occurrences in the 39-stream
selection (25 with faces, 30 without). All 53 earlier successes remain
supported. Chord-plane recovery adds two bodies with 52 faces and 168 fin
uses; totals are now 583 bound faces and 2,652 fin uses. Two bodies remain
unsupported: an unresolved fin trim and unsupported spun-surface inversion.
Synthetic regressions cover original parameter units, straight/curved interior
points, both fin directions, off-curve and outside-chart points, multiple solved
parameters on repeated branches and finite recovery budgets.

## Analytic offset surfaces

`OFFSET_SURF` is now typed as `OffsetSurfaceGeometry{surface, distance}`;
full inspector model JSON includes its basis surface and signed distance.
Other source flags and internal scale remain in the raw report.
Model-based surface evaluation, normals and inversion support offsets of
planes, cylinders, cones, spheres and regular ring tori. Parameterisation is
preserved, including inherited basis domains, nested offsets and basis sense.
Offset dependency chains
are iterative, with cycle detection and a 64-layer bound. Singular radius or
torus transitions and unsupported bases produce explicit errors; general
NURBS/blend offsets are not approximated. Tests compare offsets to oriented
basis normals, derivatives to finite differences, and forward/inverse UVs.
The library remains MIT and STL-only.

## Explicit delta state application

`openparasolid/delta.h` adds an atomic state-application engine.
`CreateDeltaState(base_report, revision, limits)` creates an owned checked
baseline. `ApplyDelta(state, delta_report, plan, limits)` accepts a complete
decoded delta report and an explicit `DeltaApplicationPlan`: base/target
revision keys, ordered create/replace/delete operations referring to
`delta_nodes` occurrences, and `(wire index, reference status) -> state ID`
mappings. Occurrences used for replacement must represent complete node
images; inherited or partial-field semantics must be resolved beforehand.
Revision keys and operation/identity interpretation are supplied by
the caller; they are not inferred from PMARKs or packet order.

The engine preserves status namespaces, converts status-framed signed integer
words by bit pattern, and rejects stale revisions, repeated targets or source
occurrences, invalid shapes, missing mappings, absent deletion targets and
node-type-changing replacements. Final snapshots may contain unknown-class
findings; geometry issues remain model findings. Dangling or wrong-class
references are rejected. Limits bound
input operations/references, live nodes and field/value counts. Application is
transactional: errors do not modify either input or return a partial state.

`DeltaState.snapshot` can be passed directly to `ModelBuilder`. Its offsets
after application are virtual logical addresses, not positions in the original
base or delta file. Preserve the original reports for wire provenance.
Synthetic tests cover creation, replacement, deletion, namespace remapping,
signed words, stale states, malformed occurrences, limits and rollback.

**Automatic reconstruction of an arbitrary delta stream is still unavailable.**
The supplied reference material does not establish the required mapping from
PMARK history and pointer status to identity and operation semantics. The
explicit-plan engine does not guess that missing interpretation. A documented
mapping or a matching base/delta pair with known final state is needed to add
and independently validate the automatic planner. Other unsupported geometry
families likewise remain explicit; the new APIs do not claim a complete
Parasolid kernel.

## Ordered loop boundaries

`openparasolid/topology.h` introduces
`TraverseLoopBoundary(model, loop_id, options)` as a first BRep topology layer.
It follows `LOOP.halfedge` and `FIN.forward` without sorting, rotating or
reversing the source ring. The owned `LoopBoundary` result retains its loop,
face and nullable surface IDs, the raw face sense, and ordered `FinUse` values:

- FIN and EDGE IDs, with the FIN's forward/reverse sense relative to its edge.
- Start/end vertex IDs oriented in loop order.
- Separate nullable edge-curve and fin-curve references, without substitution.

The forward vertex of a fin is `FIN.vertex`. Natural edge endpoints come from
`EDGE.halfedge.vertex` (end) and `EDGE.halfedge.other.vertex` (start); these
primary and secondary fins must respectively have positive and negative sense.
Dummy fins can supply these anchors but are not appended to the loop result.
An edge may occur once in each direction, including seam uses. Vertex-free
ring edges have two null endpoints and must be the loop's only edge use.
An isolated vertex loop instead returns `isolated_vertex` with its FIN and
VERTEX IDs and no edge uses; that fin's sense has no significance.

Traversal checks selected reference roles and target types, loop ownership,
reciprocal forward/backward links, closure at the starting fin, primary and
secondary edge ownership/senses, fin endpoint agreement, vertex continuity and
repeated oriented edges. It never joins vertices by coordinate proximity.
Missing roles or unknown FIN senses and unreviewed pointer classes return
`kUnsupportedFormat`; malformed connectivity, missing/wrong targets or
ambiguous pointer roles return `kInvalidRecord`. Errors identify the source
node/field where available. Results own their data and do not modify the model.

`LoopTraversalOptions.max_fins` defaults to 100,000 and must be positive.
Exhaustion returns `kResourceLimit` without a partial boundary. The limit also
counts an isolated vertex's fin. Output and visited/edge-use sets grow with
visited fins; reference fields are scanned locally, with no recursive graph
walk or geometry evaluation. Unrelated model issues do not block this local
query. Source faces without a surface, untyped geometry and nullable
curve references remain explicit in the result.

`ModelNode.topology_sense` preserves scalar character `sense` fields on FACE
and FIN nodes, including unknown bytes. Absent or incompatible fields stay
absent; no default sense is invented. The full inspector model JSON adds the
numeric-or-null `topology_sense` field to each node. Geometry senses and summary
output are unchanged; no CLI options or dependencies are added.

This establishes a locally connected loop, not a validated body or manifold
BRep. Face-list membership, complete radial fin rings, vertex chains,
shell/body closure, geometry membership and tolerance bounds remain unchecked.
FIN-to-edge sense does not supply edge-to-curve sense or curve parameter bounds.
Face normal orientation, outer/hole classification, placements and units are
not inferred. The rules follow the XT Format Reference (July 2022, pp. 28-29
and 94-96). The implementation remains MIT and STL-only.

Synthetic tests cover rotated heads, both edge directions, dummy anchors,
vertex-free rings, isolated vertices, seam reuse, non-primary fins, malformed
links/roles/endpoints, unknown sense metadata, unverified pointer classes,
resource limits and copy/move ownership. An independent Python walk of raw
schema fields checks every one of 9,620 loops in the 90-stream selection:
9,544 edge loops (including 1,565 vertex-free rings), 76 isolated vertex loops
and 41,916 oriented edge uses. These include 20,926 reverse uses, 114 uses with
dummy anchors and 5,622 uses retaining a fin curve. All match; 50,380 FACE/FIN
sense bytes also agree with the raw fields. Private inputs and comparison
programs remain outside the repository.

## Face boundaries

`TraverseFaceBoundary(model, face_id, options)` in `openparasolid/topology.h`
returns an owned `FaceBoundary` with the source face ID, nullable surface ID,
raw/absent face sense and all boundaries listed by that face. It follows
`FACE.loop` and the singly linked, null-terminated `LOOP.next` chain in source
order. A null loop head is valid, including for faces without trimming loops;
it does not trigger an inferred boundary or a geometry check.

Every listed loop must point back to the queried face. Each boundary is read
with the same connectivity checks and orientation rules as
`TraverseLoopBoundary`, including ring edges and isolated vertex loops. Face
list cycles, incompatible ownership and broken references are explicit errors.
Missing/unverified roles remain unsupported rather than being inferred from
other nodes. Errors inside a loop retain their code and source offset, adding
face/loop context. No partial face boundary is returned on error.

`FaceTraversalOptions.max_loops` defaults to 10,000 and `max_fins` to 100,000.
Both must be positive, including for empty faces. The fin budget is shared
across the entire face, not reset for each loop; every isolated vertex loop
consumes one fin. Exceeding either bound returns `kResourceLimit`. Output and
visited sets grow only with the traversed list and bounded loop results;
there is no recursive face/shell/body walk or geometry evaluation.

The result preserves list order without classifying outer boundaries and
holes. It does not prove that loops are mutually disjoint or enclose a valid
surface patch, scan the model for unlisted loops claiming the same face, or
validate face-to-shell membership, radial rings, manifoldness or geometric
tolerances. Face normals, edge-to-curve parameters, placement and units are
unchanged. There are no new CLI settings, dependencies or license changes;
the implementation remains MIT and STL-only.

Synthetic tests cover mixed and reordered boundary kinds, empty faces,
nullable/untyped surfaces, unavailable senses, exact aggregate limits,
self/multi-loop cycles, wrong owners, missing/unverified/ambiguous references,
nested error context, unlisted loops and ownership across copies/moves.
An independent raw-field walk checks all 8,274 faces in the same 90 private
streams: every listed boundary matches the prior loop results exactly,
covering all 9,620 loops, 41,916 oriented edge uses and 76 isolated vertex
loops. There are 934 faces with multiple loops, with maxima of 20 loops and
122 fins per face; no traversal errors occur. Empty face lists are covered
synthetically. Private inputs and comparison tools remain outside the repo.

## Shell boundaries

`TraverseShellBoundary(model, shell_id, options)` in
`openparasolid/topology.h` returns an owned `ShellBoundary`: the shell and
required region IDs, distinct face boundaries, their side-specific uses,
listed wire edges, or the vertex/point IDs of an isolated (acorn) shell.
The obsolete `SHELL.body` field is ignored; region/body chains are not traversed.

Back-face uses follow `SHELL.face` and `FACE.next/previous`, with `FACE.shell`
pointing back to the queried shell. Front-face uses independently follow
`SHELL.front_face` and `FACE.next_front/previous_front`, with matching
`FACE.front_shell`. Both lists are null-terminated, acyclic and checked for
reciprocal links. The result lists back uses first, then front uses, retaining
each source chain's order. Side is relative to the face normal: a front use
places the region on the side toward which that normal points. It does not
silently change the source surface/face senses or reverse any loop.

The same face may appear once on each side of one shell. `face_uses` stores
`ShellFaceUse{boundary, side}` values, whose `boundary` indexes `faces`. Each
distinct `FaceBoundary` is read and stored once per shell, in first-encounter
order. Index relationships remain valid across copies and moves; returned
data outlives the source model.

`SHELL.edge` supplies a separate null-terminated wire-edge chain. Each edge
must belong to this shell, have reciprocal previous/next links, and retain its
natural endpoints and nullable curve reference. Its positive primary and
negative secondary fins form a closed two-fin ring with null loop, forward,
backward and curve roles. Non-null `next_at_vx` links can remain on wire
anchors: each locally checked target must be another primary/secondary edge
anchor or a valid isolated-loop fin at the same non-null vertex. This handles both shared wire vertices
and closed edges whose two endpoint IDs agree. Full vertex-chain traversal
and cycle/completeness certification remain separate work.

An acorn shell has a vertex and no face uses or wire edges. Its vertex must
belong to that shell, have no fin-chain head and retain a non-null point
reference. Geometry may remain untyped. A shell without any listed entity,
conflicting acorn contents, wrong ownership, invalid links or list cycles
returns `kInvalidRecord`; unavailable/unverified roles remain unsupported.
Nested face/loop failures retain their code and source offset and add shell
context. Errors never return a partial shell.

All four `ShellTraversalOptions` limits must be positive:
`max_face_uses = 10000`, `max_loops = 10000`, `max_fins = 100000`, and
`max_wire_edges = 100000`. Uses count both sides. Loop and fin budgets are
shared across distinct face boundaries, counting isolated loop fins once;
a second side of a cached face consumes no additional loop/fin budget. Empty
faces remain valid after these budgets are exhausted. Wire anchors are
bounded separately by the wire-edge limit. Excess returns `kResourceLimit`.

These local checks do not prove shell connectedness, full vertex/radial
incidence, manifoldness, closed volume, region-list membership or valid body
geometry. Unlisted entities, placements, units and geometric tolerances are
not inferred or repaired. The rules follow the XT Format Reference (July
2022, pp. 27-29 and 93-97), checked against actual writer vertex-chain fields.
The implementation remains MIT and STL-only, with no new CLI settings.

Synthetic tests cover shared front/back boundaries, empty and front-only
faces, wire-only and acorn shells, aggregate limits, same-vertex wire ends,
local vertex links, malformed owners/lists/anchors, nested errors and owned
indices. In the previous 90 private streams, independent raw-field checks
verify all 216 shells and 16,548 face uses (8,274 on each side). Their 15,670
stored face boundaries match the prior face results exactly; 878 faces share
one boundary between both sides of a shell. An additional 40 selected streams
without faces verify all 70 shells, including 46 acorn shells, 37 wire edges
and 16 non-null wire vertex links. No traversal errors remain. Private inputs
and comparison programs remain outside the repository.

## Region boundaries

`TraverseRegionBoundary(model, region_id, options)` in
`openparasolid/topology.h` returns an owned `RegionBoundary` with the source
region/body IDs, `RegionKind::kSolid` or `RegionKind::kVoid`, and complete nested
`ShellBoundary` results in `REGION.shell` / `SHELL.next` order. The recorded
`REGION.body` may name a compound parent; optional child ownership is not
resolved. List order does not identify a positive/outer shell.

The query requires a non-null BODY reference and a scalar character type
`'S'` or `'V'`. Unknown, missing or incompatible types return
`kUnsupportedFormat`. `ModelNode.region_type` preserves the raw character,
including unknown values, or absence; full inspector model JSON exposes it
as numeric-or-null `region_type` on each node. A null shell head is accepted
structurally without classifying the region as finite or exterior.

Shell lists must be acyclic and each listed shell must belong to this region.
Nested shell/face/loop checks still apply. Different shells of one region must
not share face, edge or vertex IDs, including wire and isolated vertices.
Reuse within one shell retains the existing rules. Sharing geometry IDs or
positions is not checked. Nested errors preserve their code and source offset
and add region/shell context; no error returns a partial boundary.

All five `RegionTraversalOptions` limits must be positive: `max_shells` and
`max_face_uses` default to 10,000, `max_loops` to 10,000, and `max_fins` and
`max_wire_edges` to 100,000. Limits apply across the whole region, rather than
resetting for each shell. Each face side consumes a face use; loop/fin data
count once per stored face boundary, and an isolated loop counts as one fin.
An acorn consumes one shell; wire anchors are bounded by the wire-edge limit.
Empty content may follow exhaustion of a corresponding budget. Exceeding a
limit returns `kResourceLimit`.

This checks selected topology, without proving body-list membership, full
vertex/radial incidence, connectedness, manifoldness, geometric disjointness,
volume or tolerances. Unlisted entities, child ownership, placements and units
are not inferred. Rules follow the XT Format Reference (July 2022, pp. 27-29
and 93-94). The implementation remains MIT and STL-only.

Synthetic tests cover source order, solid/void and unknown types, empty
regions, recorded compound-parent references, aggregate limits, invalid
owners/roles/targets, cycles, cross-shell shared faces/edges/vertices, nested
errors and result ownership. An independent raw-field walk of the same 130
selected private streams checks 267 regions (94 solid, 173 void), including
ten with multiple shells, and all 286 nested shell boundaries against the
previous shell results. No region traversal errors remain in that selection;
it contains no shell-less regions, which are covered synthetically. Private
inputs and comparison programs remain outside the repository.

## Body boundaries

`TraverseBodyBoundary(model, body_id, options)` in
`openparasolid/topology.h` reads a standard or compound-parent body's
`BODY.region` / `REGION.next` chain into an owned `BodyBoundary`. Results
contain the body ID, `BodyKind` (solid, wire, sheet or general), and nested
region/shell/face/loop boundaries in source order. Body types come from scalar
byte values 1, 2, 3 and 6; unavailable, incompatible or unknown types return
`kUnsupportedFormat`. `ModelNode.body_type` and full inspector model JSON's
numeric-or-null `body_type` retain raw values, including unknown bytes.

The body owner must be null or a WORLD. Both concrete pointers and the
reviewed `BODY_OWNER` union class 1040 are supported; that class permits WORLD
or BODY targets. A BODY owner identifies a child body, whose region chain
continues into siblings. `TraverseBodyBoundary` therefore rejects child IDs
with `kUnsupportedFormat`; query the compound parent or use
`TraverseChildBodyBoundary` for a checked child selection. The baseline body
API does not resolve child grouping or validate the child list.

Every listed region must reference the queried body and its preceding region.
The head's previous pointer must be null, next pointers must terminate without
cycles, and nested region checks still apply. Faces may be shared across
regions; their separate stored boundaries each count toward resource limits.
A null region head is accepted structurally, as needed for empty compounds,
without certifying body validity. The obsolete `BODY.shell` is ignored.

All six `BodyTraversalOptions` limits must be positive and apply across the
entire query: `max_regions`, `max_shells`, `max_face_uses` and `max_loops` each
default to 10,000; `max_fins` and `max_wire_edges` each default to 100,000.
Counting follows region traversal, with each stored region/shell boundary
consuming its aggregate allowance. Empty content can follow exhaustion of its
corresponding allowance. Exceeding a limit returns `kResourceLimit`; errors
never return partial results. Nested failures preserve code and source offset
and add body/region context. Results own their data and leave the model intact.

This traverses the recorded region hierarchy without certifying complete
incidence, connectedness, manifoldness, volume or geometric validity. It does
not classify exterior regions (compound parents combine several child
chains), validate body-type consistency against geometry, or inspect the
body's geometry/edge/vertex lists. Unlisted entities, placements, units and
tolerances remain outside this query. The rules follow the XT Format Reference
(July 2022, pp. 27-29, 88-94 and 120). The library remains MIT and STL-only.

Synthetic tests cover all four body kinds, empty bodies, compound-parent
chains and rejected child queries, shared faces across regions, aggregate
limits, cycles, ownership, malformed roles/targets, nested diagnostics and
result ownership. Reference tests check both permitted `BODY_OWNER` targets,
wrong classes and future unknown target types. An independent raw-field walk
of the same 130 selected private streams checks all 163 bodies (85 solid,
70 wire, eight sheet), 267 regions and 286 shells. All nested region
boundaries match the previous results, including 16,548 face uses, 37 wire
edges and 46 acorn shells. There are 86 bodies with multiple regions and no
body traversal errors. Empty bodies, general bodies and compound/child
behavior are covered synthetically; that private selection contains none.
Private input files and comparison programs remain outside the repository.

## Compound body children

`TraverseCompoundBodyBoundary(model, parent_id, options)` in
`openparasolid/topology.h` adds explicit child grouping above
`TraverseBodyBoundary`. Its owned `CompoundBodyBoundary` stores the existing
parent result in `boundary` and an ordered `children` vector. Each
`ChildBodyRange` contains the child BODY ID, `BodyKind`, `first_region` and
`region_count`, indexing `boundary.regions`. Nested region/shell/face/loop data
are stored only once; indices remain valid across copies and moves.

The query follows `BODY.child` and each child's `BODY.next/previous` links.
Children must belong to the parent, have reciprocal list links and no nested
children, and retain a known body type. Their `BODY.region` heads must match
nonempty contiguous ranges in the parent's combined region chain.
`REGION.owner` must identify the corresponding child. Ranges must cover all
listed parent regions in child order. A common child kind must equal the
parent kind; mixed child kinds require a general parent. Empty compounds
require both the child and region heads to be null. These are structural
checks, without certifying geometric body-type consistency.

`CompoundBodyTraversalOptions.body` holds the existing six positive aggregate
body limits. `max_children` additionally bounds child records and defaults to
10,000; it must also be positive. Child cycles are rejected even at the exact
child limit. Resource exhaustion returns `kResourceLimit`, malformed
ownership/order/ranges return `kInvalidRecord`, and unavailable roles, unknown
body types or unreviewed pointer classes return `kUnsupportedFormat`. Nested
body failures retain their code, source offset and context. Child-grouping
failures add compound-parent context. No failure returns partial results.

This API requires the transmitted `BODY.child`, child
owner/previous/next/child/region roles and `REGION.owner`. Older layouts missing
`BODY.child` or `REGION.owner` can still use `TraverseBodyBoundary` to read the
combined boundary without child grouping. For a child ID, use
`TraverseChildBodyBoundary` below, which selects its checked parent range.
The compound query does
not scan for unlisted children, classify exterior regions, read child geometry
lists or establish geometric validity, full incidence, volume or manifoldness.
The rules follow the XT Format Reference (July 2022, pp. 88-94). The library
remains MIT and STL-only, with no additional CLI settings or dependencies.

Synthetic tests cover mixed and common child kinds, empty compounds, ordered
ranges and ownership after copies/moves, aggregate body and child limits,
cycles, malformed ownership/heads/ranges, missing and unreviewed metadata,
unknown child types and nested diagnostics. The baseline body API remains
usable independently of child-grouping metadata. A raw-schema scan of all
434 multi-body streams in the existing model corpus found no non-null
`BODY.child` head, so real nonempty-compound validation remains unavailable.
The previous 130-stream body comparison is retained as regression coverage;
private inputs and comparison programs remain outside the repository.

## Child body boundaries

`TraverseChildBodyBoundary(model, child_id, options)` in
`openparasolid/topology.h` returns an owned `ChildBodyBoundary`. `parent`
identifies the compound parent; `boundary.body` and `boundary.kind` identify
the selected child, and `boundary.regions` contains only its region range.
Nested `RegionBoundary.body` IDs retain the transmitted compound-parent
reference. Source topology IDs are preserved rather than rewritten.

The child must have a non-null BODY owner and occur in that parent's checked
child list. The query first runs the full compound-parent traversal, then
moves the selected range into its result. It handles non-null previous-region
links at the beginning of a child's range and stops before sibling regions.
A claimed parent alone does not establish child membership; unlisted children,
self-ownership, null/missing/wrong-type parents and invalid root targets return
`kInvalidRecord`. Unavailable metadata and unreviewed classes remain
`kUnsupportedFormat`.

`CompoundBodyTraversalOptions` retains exactly the compound query's meaning:
all seven positive limits cover the entire parent and its children, including
siblings, before selection. A small child does not bypass parent limits or
invalid sibling topology/grouping. Nested errors retain code and source offset
and add child/parent context. No failure returns partial data. Results own
their selected boundary, can outlive the model and can be copied or moved.
The source model is unchanged; geometry, placements, units, exterior-region
classification and further body-validity claims remain outside this API.
`TraverseBodyBoundary` retains its existing standard/parent-only contract.

Synthetic tests verify first and later children, exact range selection,
retained parent IDs and nested data after source destruction, all parent-wide
limits, invalid options before lookup, malformed parent references,
unlisted children, unavailable metadata and errors in siblings. Real child
samples remain unavailable after the preceding corpus scan; the existing
130-stream body comparison remains regression coverage. No dependencies or
CLI settings are added; the library remains MIT and STL-only.

## Edge fin rings

`TraverseEdgeBoundary(model, edge_id, options)` in
`openparasolid/topology.h` returns an owned `EdgeBoundary`: the edge ID,
natural start/end vertex IDs, nullable edge curve, and all radial fins in
`EDGE.halfedge` / `FIN.other` order. Each `EdgeFinUse` retains an oriented
`FinUse` plus its source loop ID. A null loop identifies a dummy fin. Dummy
anchors are included in the output, unlike externally visible halfedge uses.
The result can outlive the model and can be copied or moved.

The first two fins must be distinct positive primary and negative secondary
anchors. Every listed fin must belong to this edge, retain a known sense and
reference the endpoint corresponding to that sense. Endpoints must either
both exist or both be null. The radial ring must close at its primary head
without repeating another fin. Additional real fins may share an orientation;
their count or senses do not establish manifoldness or geometric angular order.

Real fins retain their loop and require locally reciprocal forward/backward
links within that loop. This checks immediate neighbors without walking or
certifying the entire loop. Dummy fins must be anchors and retain null
loop/forward/backward/curve roles. As in shell traversal, actual writer data
may retain non-null dummy vertex-chain links; those links are checked locally
against another anchor or a valid isolated-loop fin at the same vertex.
Complete vertex chains are not
walked. Curve references may be null or remain untyped; tolerant-curve and
edge-to-curve orientation rules are not inferred.

`EdgeTraversalOptions.max_fins` defaults to 100,000 and must be positive. It
bounds the entire visited radial ring, including dummy anchors. Zero returns
`kInvalidOptions`, exhaustion `kResourceLimit`, malformed connectivity or
missing/wrong targets `kInvalidRecord`, and unavailable roles, unknown senses
or unreviewed classes `kUnsupportedFormat`. A cycle away from the primary head
is rejected even at the exact fin limit. No error returns a partial boundary;
the source model is unchanged.

The API does not scan for unlisted fins claiming the edge, validate body-list
membership, full loops/vertex incidence, shell closure, geometric validity or
tolerances. It remains separate from existing loop/face/shell/body queries,
whose traversal contracts and resource accounting are unchanged. The rules
follow the XT Format Reference (July 2022, pp. 29 and 95-97), together with
observed writer vertex links. The library remains MIT and STL-only.

Synthetic tests cover radial source order, repeated senses, oriented endpoints,
nullable/untyped curves, sheet/wire dummy anchors and their vertex links,
resource limits, invalid cycles/owners/anchors/loop links, unavailable metadata,
unknown senses, unlisted entities and owned results after source destruction.
An independent raw-field walk of the same 130 selected private streams checks
all 21,052 edges and 42,104 radial fins, including 188 dummy fins, 37 wire
edges, 786 vertex-free edges and 60 non-null dummy vertex links. All 41,916
real fin uses also match the previous loop results exactly. Every source fin
with an edge reference is accounted for in that selection. No radial query
errors remain. Those samples have two fins per edge; higher-arity rings are
covered synthetically. Private inputs and comparison programs remain outside
the repository.

## Vertex incidence

`TraverseVertexIncidence(model, vertex_id, options)` in
`openparasolid/topology.h` returns an owned `VertexIncidence` with vertex,
owner and point IDs and all listed fins in `VERTEX.halfedge` /
`FIN.next_at_vx` order. Each `VertexFinUse` retains fin and loop IDs. Its
optional `edge_use` contains an oriented `FinUse` for an edge anchor; absence
identifies an isolated-loop fin whose sense is insignificant.

The point must be a non-null POINT reference and the owner a BODY or SHELL.
Compound-parent BODY references are preserved without inferring child
ownership. SHELL-owned acorn vertices require null fin heads. BODY-owned
null heads are accepted structurally without claiming complete incidence.
Point geometry may remain untyped; no coordinates or tolerances are inferred.

Every listed fin must reference the queried vertex. Edge entries must be
primary or secondary anchors with consistent senses/endpoints and local
loop/dummy links. Other radial fins are excluded even when they reference
that vertex. Both anchors of one closed edge may occur at a single vertex.
Isolated-loop fins require self forward/backward links, null edge/other/curve
roles and their loop's halfedge head pointing back to them. Their sense may
be unknown or absent. Dummy vertex links may lead to these isolated-loop fins
as well as anchors; existing edge and shell local-link checks now support
that case too.

`VertexTraversalOptions.max_fins` defaults to 100,000 and must be positive.
It counts all entries, including dummy anchors and isolated fins. The chain
must terminate at null; every repeated fin is a cycle, including a return to
the head. Cycles fail even at the exact fin limit. Zero returns
`kInvalidOptions`, exhaustion `kResourceLimit`, malformed connectivity or
missing/wrong targets `kInvalidRecord`, and unavailable roles/anchor senses
or unreviewed classes `kUnsupportedFormat`. Errors preserve source context
without partial results. Results own their data and leave the model unchanged.

The query does not scan for unlisted incident fins, certify owner-list
membership, walk full radial rings or loops, or establish geometric validity,
manifoldness, shell closure or tolerance bounds. Rules follow the XT Format
Reference (July 2022, pp. 96-97), including its isolated-loop exception to the
anchor-only vertex-chain rule. The library remains MIT and STL-only; no CLI
settings or dependencies are added.

Synthetic tests cover mixed real/dummy/isolated chains, closed-edge anchors,
acorn and body-owned null heads, result ownership, limits and cycles,
malformed owners/points/vertices, non-anchor rejection, invalid isolated fins,
unavailable metadata and untyped point geometry. An independent raw-field
walk of the same 130 selected private streams checks 13,187 vertices and
40,608 chain entries: 40,532 edge anchors (including 181 dummy anchors) and
76 isolated-loop fins. All edge uses match the prior radial results. Every
expected source anchor and isolated fin is accounted for, including 46 acorn
vertices. The longest observed chain has 19 entries; no traversal errors
remain in that selection. Private files and comparison programs remain
outside the repository.

## Curve evaluation

`openparasolid/evaluate.h` evaluates an untrimmed line, circle, ellipse or NURBS
curve, returning its position and natural first derivative in source coordinates:

```cpp
openparasolid::LineGeometry line{{1, 2, 3}, {0, 0, 1}};
auto evaluated = openparasolid::EvaluateCurve(line, 4.0);
if (evaluated.ok()) {
  // Position (1, 2, 7), derivative (0, 0, 1).
  const auto& sample = evaluated.value();
}
```

For model data, pass `geometry.payload` to `EvaluateCurve`. The parameter is
signed and unbounded for analytic curves; circle and ellipse parameters are
angles in radians. NURBS parameters are restricted to the defined knot domain.
The returned derivative is `dP/dt`, without normalization or curve-sense
reversal. This follows the natural parameter direction; edge orientation and
trimming belong to topology. Units and source coefficients are preserved.

Analytic evaluation requires finite inputs, positive radii, unit directions and
orthogonal conic frames (absolute tolerance `1e-10`). It rejects malformed data
instead of normalizing it. These checks are stricter than the model builder's
initial nonzero/nonparallel checks. Nonfinite parameters, nonfinite results
and unsupported payloads return explicit `Result` errors. Evaluation errors
use offset zero because a payload does not contain a source byte offset.

NURBS evaluation uses de Boor interpolation with derivative propagation in
homogeneous coordinates. Rational controls already contain weighted spatial
coordinates followed by a positive weight; no extra multiplication by the
weight is performed. Two-dimensional controls are returned in the XY plane.
The knot domain is `[U[degree], U[control_count]]`, where `U` is the expanded
knot vector. Both endpoints are included. At repeated interior knots the
returned derivative is right-sided; at the domain end it is left-sided.
There is no automatic periodic wrapping or extrapolation, including for curves
marked periodic. Closure and periodic continuity are not certified.

Public NURBS payloads are checked again: dimensions, counts, finite controls,
positive weights, distinct knot ordering, multiplicities, expanded knot count
and a nonempty domain. Evaluation is bounded to degree 64, 1,000,000 control
scalars, and 1,000,000 entries in each knot/multiplicity array or the expanded
knot vector. Excessive input returns `kResourceLimit` before expansion.
Nonfinite or out-of-domain parameters return `kInvalidOptions`; malformed
payloads and unrepresentable numerical results return `kInvalidRecord`.
Scratch space is linear in the expanded knot count plus degree; de Boor work
is quadratic in degree. Extended-precision intermediate calculations reduce
overflow, but final positions and derivatives must fit in `double`.

The formulas and frame conventions follow the XT Format Reference (July 2022,
pp. 35–40); natural derivatives and curve sense follow section 5.2.8 (p. 82).
Ordered loop traversal is described below. Delta application and STEP export
remain separate work; curve evaluation adds no CLI options.

Synthetic tests cover rational quarter circles, 2D/3D controls, homogeneous
scaling, endpoint and repeated-knot derivatives, unclamped knots, invalid data
and resource limits. An independent local comparison with SciPy agrees on
8,565 samples from 600 synthetic curves and 192 curves in 20 private streams.
SciPy and private comparison artifacts are not library dependencies or
repository fixtures.

## Analytic surface evaluation

`EvaluateSurface(geometry.payload, u, v)` in `openparasolid/evaluate.h` returns
a `SurfaceEvaluation` with `position`, `u_derivative` and `v_derivative`.
It supports planes, cylinders, cones, spheres and tori. Results use source
coordinates and natural partial derivatives; surface/face sense, placement
transforms, length units and face trimming are not applied. The underlying
surface frame is preserved, with `Y = axis cross X` (the normal is the axis
for a plane). Frames must be orthonormal within absolute tolerance `1e-10`.

| Surface | Parameters and supported domain |
|---|---|
| Plane | Signed distances `u`, `v`, both unbounded |
| Cylinder | Angle `u` in radians and signed axial distance `v`, both unbounded |
| Cone | Angle `u` in radians; signed axial parameter `v` with `r + v*tan(a) >= 0` |
| Sphere | Longitude `u` in radians, unbounded; latitude `v` in `[-pi/2, pi/2]` |
| Ring torus (`major > minor`) | Angles `u`, `v` in radians, both unbounded |
| Apple/lemon torus | Angle `u`, unbounded; principal minor angle `abs(v) <= acos(-major/minor)` |

The cone uses `P + v*A + (r + v*tan(a))*radial(u)`, as observed in the
SolidWorks writer output and independently checked against stored trim
endpoints and the local XT-to-STEP mapping. The printed XT Format Reference
uses a minus sign for the axial term; that formula disagrees with these
endpoint correspondences. The evaluator uses the writer convention, including
`+A` in the v partial. Half-angles may be signed and must be nonzero and acute
in magnitude: the stored sine must be nonzero, cosine positive, and their
length must be one within `1e-10`. Cone radius may be zero. Parameters beyond
the apex are rejected through the signed radial condition.

Torus minor radii must be positive. Positive major radii support ring, apple
and horn (`major == minor`) tori. Negative major radii preserve the XT lemon
sheet and require `abs(major) < minor`; zero major radius is a sphere case
and is rejected. Apple/lemon parameters outside the principal branch are
rejected without automatic wrapping. Angular endpoints are compared with
the computed `acos` limit, allowing endpoint rounding without clamping radii.

Sphere poles, cone apices and torus branch endpoints are valid evaluation
points even though a partial derivative may vanish there. No unit normal or
regularity certification is returned. Floating-point trigonometric values at
these boundaries may leave derivatives close to zero rather than exactly zero.

Both parameters must be finite. Invalid parameters or domains return
`kInvalidOptions`; malformed geometry or unrepresentable numerical results
return `kInvalidRecord`; unsupported payloads return
`kUnsupportedFormat`. Error offsets are zero, as for `EvaluateCurve`.
Intermediate calculations use extended precision where available and final
values must fit in `double`. The implementation adds no dependencies or CLI
options and requires no dynamic scratch storage for analytic evaluation.

Formulas and torus types follow the XT Format Reference (July 2022, pp. 53–59).
Synthetic tests cover rotated frames, natural derivatives, the writer cone axis,
singular/domain boundaries, malformed payloads, numerical overflow and model
integration without applying surface sense.
An independent NumPy comparison checks 2,910 samples from 600 synthetic
surfaces and 370 surfaces in 25 private streams, using complex-step
differentiation for both partials. Comparison tools and data stay outside the
repository and are not dependencies of the library.

## NURBS surface evaluation

`EvaluateSurface` also evaluates tensor-product NURBS surfaces, returning the
point and natural partial derivatives in source coordinates. A nonrational
surface has three control components; a rational surface has four homogeneous
components `(w*x, w*y, w*z, w)`, with positive weights. Control `(i,j)` starts at
`(i*n_v + j)*dimension` in the flat source array: the v index varies fastest.
The evaluator preserves this order and applies no transposition, weight
premultiplication, sense reversal, placement transform or face trimming.

Each axis follows the NURBS curve rules above: its own degree, expanded knots,
closed parameter interval and one-sided derivative convention. At an interior
knot the partial is right-sided in that axis; at its upper endpoint it is
left-sided. Closed/periodic flags do not enable extrapolation or wrapping, and
continuity, regularity or closure are not certified. Singular patches can
return zero partials without implying a valid face normal.

Both axes and the complete control array are validated before allocating knot
or interpolation scratch space. The shared control net must have exactly
`n_u*n_v*dimension` scalars. The degree limit is 64 per axis, each knot array
and expanded knot vector is bounded to 1,000,000 entries, and the total control
net is bounded to 1,000,000 scalars. Excessive products return `kResourceLimit`
before multiplication by the dimension. Other errors follow the curve API.

The curve and surface evaluators share de Boor interpolation, homogeneous
projection and axis validation. Surface interpolation carries both homogeneous
partials through the tensor product before applying the quotient rule.
Scratch storage grows with both expanded knot counts plus the two degrees;
local interpolation costs `O((p+1)*q^2 + p^2)` for degrees `p` and `q`, after
input validation and knot expansion. There are no new dependencies or CLI
options. The tensor-product definition follows the XT Format Reference
(July 2022, pp. 63–65); flat control indexing was cross-checked against the
local XT-to-STEP mapping reference without incorporating its implementation.

Synthetic tests cover nonsquare nets with different degrees, homogeneous
quarter cylinders, both weight derivatives, repeated and unclamped knots,
independent parameter scaling, singular patches, maximum degrees, control
product limits, malformed input and model integration.
An independent SciPy comparison agrees on 8,401 surface samples from 400
synthetic surfaces and 120 surfaces in 25 private streams. Rechecking the
shared kernel also preserves agreement on all 8,565 earlier curve samples.
Comparison tools, data and SciPy remain outside the library and repository.

## Curves on surfaces

`SP_CURVE` now has a typed `SurfaceCurveGeometry` with `surface` and
`parameter_curve` node IDs. Model JSON reports it as `kind: "surface_curve"`;
full geometry reports retain both IDs and summaries include its count.
Non-null definitions can be represented with forward or unresolved references;
the reference graph reports missing/wrong targets independently. The raw
`original` and `tolerance_to_original` fields remain in the parse report, and
original references remain in the graph. These documented unused fields do
not provide a fallback evaluator or an approximation tolerance.

Use the model overload to evaluate an SP-curve:

```cpp
auto evaluated = openparasolid::EvaluateCurve(
    model, openparasolid::NodeId{42}, parameter);
if (!evaluated.ok()) {
  // Handle missing dependencies, unsupported geometry or parameter errors.
  return;
}
const auto& point = evaluated.value().position;
const auto& tangent = evaluated.value().first_derivative;
```

This overload also resolves ordinary supported curves by ID. A standalone
SP-curve payload has no dependency context and returns `kUnsupportedFormat`.
Targets are scoped to the supplied model; copies and moves retain their ID
relationships. No mutable caches, ownership pointers or recursive traversal
are introduced.

The parameter curve must be a `B_CURVE` with two spatial components: dimension
2 when nonrational or homogeneous dimension 3 when rational. Its evaluation
returns `(u(t), v(t))`; the supporting analytic or NURBS surface then returns
`P = S(u,v)`. The natural derivative uses the chain rule
`dP/dt = S_u * du/dt + S_v * dv/dt`. All three senses remain unchanged and are
not applied to this derivative. Source units, parameters and placements are
not converted. Curve and surface domain/resource rules remain in force;
periodic wrapping, face trimming and approximation are not added.

Missing or wrong-class dependencies and 3D parameter curves return
`kInvalidRecord`. Dependencies without typed payloads or supporting evaluators
return `kUnsupportedFormat`. Parameter/domain and resource errors propagate
from the component evaluators; an unrepresentable composed derivative is an
invalid-record error. Cyclic SP dependencies are incompatible with the required
2D B-curve/surface roles and are rejected without recursion. The unused
`original` reference is never followed, including when it is cyclic.

The builder's typed-definition count does not certify that the referenced
curve is 2D or the composition is evaluable. The model evaluator performs
those stronger checks; unrelated model findings do not prevent evaluation of
an otherwise supported curve. SP-curve definitions follow the XT Format
Reference (July 2022, pp. 51–52). The implementation is MIT code with no new
runtime dependencies or CLI options.

Synthetic tests cover forward references, rational UV curves, analytic and
NURBS supporting surfaces, natural chain-rule derivatives, copying/moving,
cyclic and wrong/missing targets, null definitions, 3D parameter-curve
rejection, domain/resource propagation and composed derivative overflow. CLI
tests check the new payload and summary count without hiding reference issues.
An independent SciPy/NumPy comparison checks 9,138 samples from 2,140 SP-curves
in 30 private streams. There are 158 curves with unavailable typed dependencies
and 772 sample-domain errors; these are retained as explicit limitations rather
than approximated. Comparison tools and private data stay outside the repository.

## Trimmed curves

`TRIMMED_CURVE` now has a typed `TrimmedCurveGeometry`: its `basis_curve` ID,
`point_1`, `point_2`, `parameter_1` and `parameter_2`. The inspector emits these
fields under `kind: "trimmed_curve"` and includes the count in summaries.
Construction requires finite endpoint vectors and distinct finite parameters,
a non-null basis and positive trim sense. Source values are preserved. Missing
and wrong-class references remain separate graph issues.

Use `EvaluateCurve(model, trimmed_id, t)` to evaluate in the **original basis
parameter**, retaining the natural `dP/dt`. The accepted interval includes both
endpoints and is bounded by `min(parameter_1, parameter_2)` and their maximum.
The basis sense must be known: `+` requires increasing endpoint parameters,
and `-` requires decreasing parameters. Descending bounds do not reverse or
rescale the natural derivative. There is no new normalized trim parameter.
Standalone trimmed payloads require model context and return
`kUnsupportedFormat` without it.

Nested trims are followed iteratively with fixed scratch storage. Every trim
interval is enforced; cycles return `kInvalidRecord`, and more than 64 wrappers
return `kResourceLimit`. The final basis can be a supported analytic curve,
NURBS curve or SP-curve. Missing/wrong bases, incompatible parameter order,
unknown basis sense and unsupported geometry remain explicit errors. Outside
intervals return `kInvalidOptions`. Existing component domain and resource
checks continue to apply. An analytic circle can cross its angular seam using
its original unbounded angular parameter; periodic NURBS wrapping is still
unsupported.

This API checks numeric intervals and basis sense, not the full validity of an
XT trim. Stored points are retained without snapping or fitting. Their agreement
with evaluated endpoints, tolerant-vertex correspondence, base-range placement
and the one-period span restriction on periodic bases are not certified.
No face clipping, approximation, unit conversion or placement transform is
introduced. Definition rules follow the XT Format Reference (July 2022,
pp. 49–50); basis-parameter interpretation was cross-checked against the local
XT-to-STEP mapping without incorporating its implementation. Code remains MIT
and adds no dependencies or CLI settings.

Tests cover inclusive and descending bounds, nested intervals, analytic seam
crossing, NURBS/SP bases, copy/move behavior, malformed definitions, missing/
wrong/unsupported bases, unknown sense, cycles and the 64-wrapper boundary.
CLI tests preserve the definition and unresolved references in full and summary
reports.

An independent comparison checks 9,703 evaluations on 2,050 typed trimmed
curves from 30 private streams. It retains 480 sample failures for unsupported
bases and 67 for NURBS domain limits. Successfully evaluated source endpoints
agree with their stored trim points within `1.5e-10` in source units. This
endpoint check exposed and verified the cone axial-sign correction above;
the correction also preserves signed half-angles. All 9,138 earlier SP-curve
samples were rechecked with the corrected cone convention. Comparison inputs
and tools remain outside the repository.

## Surfaces of revolution

`SPUN_SURF` now has a typed `SpunSurfaceGeometry`, reported by the inspector
as `kind: "spun_surface"`. It retains the profile ID, base point, spin axis,
nullable start/end points and profile parameters, nullable `x_axis`, and
internal `scale`. Entirely unset auxiliary vectors remain null; partially
unset vectors, nonfinite metadata, null profiles, zero axes and reversed
parameter bounds are rejected during model construction.

Use `EvaluateSurface(model, surface_id, u, v)` to resolve the profile and return
the point and both natural partials. This overload also accepts previously
supported standalone surfaces. A standalone spun payload requires model
context. For profile `C(u)`, axis point `B`, unit axis `A` and right-handed
rotation `R_A(v)`, evaluation is:

```text
P(u,v) = B + R_A(v) * (C(u) - B)
dP/du  = R_A(v) * C'(u)
dP/dv  = A cross (P(u,v) - B)
```

The profile must be a line, circle, ellipse or 3D NURBS curve; 2D profiles,
trimmed curves, SP-curves, intersections and foreign curves are not accepted
as generators. This keeps dependency evaluation finite without recursive
profile graphs. Missing/wrong profile roles return `kInvalidRecord`; missing
typed payloads remain explicit unsupported-format errors. No approximation
fallback is used. The model-based SP-curve evaluator uses this surface overload,
so SP-curves and their trims can now lie on supported spun surfaces.

The spin axis must have unit length within `1e-10`; it is not normalized.
The original profile parameter `u` must satisfy both its curve domain and any
inclusive `start_parameter`/`end_parameter` bounds. Null bounds add no limit on
that side. Angle `v` is in radians and may be any finite value. Profile resource
limits remain in force. Source senses, units and placement transforms are not
applied. Singular points on the axis remain evaluable without a unit normal.

The auxiliary start/end positions, profile-plane `x_axis` and internal scale
are retained without moving, rotating or scaling the generator. Endpoint
correspondence, auxiliary frame validity, self-intersections, regularity and
face trimming are not certified. A spun definition charges 19 scalar slots
against the model's copied-value budget, including nullable metadata. The
implementation follows the XT Format Reference (July 2022, pp. 68–70), adds
no runtime dependencies or CLI settings, and remains MIT code.

Synthetic tests cover translated/oblique axes, rational and nonrational
profiles, both partials, nullable metadata, bounds, singular points, overflow,
copy/move behavior, profile role errors and composition through SP/trim curves.
An independent SciPy Rotation/BSpline comparison checks 1,350 samples on all
150 spun surfaces in 30 selected private streams without evaluation errors.
On the earlier 2,050-trim comparison set, successful evaluations increase from
9,703 to 10,050; unavailable-basis sample errors decrease from 480 to 120, with
80 explicit NURBS domain errors remaining. Evaluated trim endpoints still
agree with stored points within `1.5e-10` in source units. Private inputs and
comparison tools remain outside the repository.

## Swept surfaces

`SWEPT_SURF` (type 67) has a typed `SweptSurfaceGeometry`, reported as
`kind: "swept_surface"` with its `section` ID, `sweep` vector and nullable
internal `scale`. Use `EvaluateSurface(model, surface_id, u, v)` to resolve
the section and evaluate the extruded surface:

```text
P(u,v) = C(u) + v * D
dP/du  = C'(u)
dP/dv  = D
```

The original section parameter `u` and its domain/resource limits are retained.
Parameter `v` is any finite signed displacement along the stored unit sweep
direction `D`. Unit length is checked within `1e-10`, without normalizing the
stored coefficients. Both position and partials must remain representable.

Section resolution shares the spun-surface validation: line, circle, ellipse
and 3D NURBS sections are supported, including forward references. Missing or
wrong section roles are invalid records; unavailable typed payloads return
explicit unsupported-format errors. Two-dimensional, trimmed, SP, intersection
and foreign curves are not accepted as generators. Standalone swept payloads
require model context. SP-curves and their trims can use supported swept
surfaces through the existing model-based composition API.

Internal scale is preserved without scaling the section or sweep. Source
senses, placement transforms and unit conversion are not applied. Parallel
section tangents and sweep directions remain evaluable; normals, regularity,
self-intersections, periodic wrapping and face clipping are not certified.
The copied-value budget charges five scalar slots per definition, including
nullable metadata. The implementation follows the XT Format Reference
(July 2022, pp. 67–68), remains MIT and STL-only, and adds no CLI settings.

Synthetic tests cover both partials, rational and nonrational sections,
metadata, dependency errors, domain/range checks, singular generators and
SP/trim composition. An independent NumPy/SciPy comparison verifies 1,512
samples on 168 swept surfaces in 30 selected private streams, with no
evaluation errors and a maximum normalized difference below `1.5e-16`.
The earlier 2,050-trim comparison remains unchanged at 10,050 successful
samples, 120 unavailable-payload errors and 80 explicit NURBS domain errors;
evaluated endpoints agree within `1.5e-10` in source units. Private inputs
and comparison tools remain outside the repository.

## Surface inversion

`InvertSurface(geometry, point, tolerance)` and its model/ID overload recover
natural `u/v` parameters on planes, cylinders, cones, spheres, tori and NURBS
surfaces. The
result includes the distance between the requested point and its forward
evaluation. A finite nonnegative absolute tolerance in source units is
required; success means that this measured distance does not exceed it.
Zero tolerance requires exact point agreement. There is no implicit geometric
tolerance and no general closest-point guarantee.

The actual source basis is inverted, including small frame residuals already
accepted by `EvaluateSurface`; axes are not normalized or replaced. Plane
coordinates follow the stored x-axis and `axis cross x` direction. Cylinder
and cone axial parameters retain the writer's `+v*axis` convention. Azimuth
and torus meridian angles use the principal `atan2` branch; spherical latitude
lies in `[-pi/2, pi/2]`. Undefined azimuth is represented by zero. Recovering
parameters at a sphere pole or cone/torus singularity does not imply that a
parametric normal exists there.

For cones, an axial candidate beyond the apex also tries the nearest
representable apex parameter on the valid side of the source domain. This
requires at most three parameter checks, with no domain extension or implicit
geometric tolerance. The resulting 3D residual must still satisfy the original caller tolerance, including zero.
This handles rounded isolated apex vertices in face loops and preserves
rejection of points farther than the requested tolerance. Regressions cover
both slope signs, radii from 1e-200 to 1e200 and integrated face binding. All
30 cone-domain failures in the selected face corpus now bind successfully.

Ring, apple, lemon and horn tori retain the forward evaluator's principal
sheet. An angular overshoot within eight double-precision epsilons times
`max(1, branch_bound)` may snap to the branch endpoint to accommodate rounding;
the final point must still satisfy the caller's unchanged distance tolerance.
No additional sheet or periodic continuation is searched.

The returned parameters are always checked by the existing forward evaluator,
including its geometry/frame validity, domain and representability rules.
Invalid request values return `kInvalidOptions`; nonmatching points, candidate
parameters outside the natural domain and invalid definitions/results return
`kInvalidRecord`. Missing or untyped model targets retain explicit lookup
errors. Spun, swept and other unsupported geometry kinds return
`kUnsupportedFormat`. Surface/face senses, placement and units are not applied.
The analytic inversion uses bounded stack storage. Both paths add no CLI
settings or dependencies and remain MIT and STL-only.

Together with `EvaluateSurfaceNormal`, inversion allows callers to obtain
oriented normals at analytic and NURBS chart positions.
`EvaluateIntersectionChart` combines these operations with chart
parameterisation; a local intersection evaluator is described below.

Tests cover oblique/translated and slightly imperfect source frames, explicit
distance tolerances, seams, signed cone slopes, poles/apices, torus sheets,
extreme radii, invalid data and model lookup. A separate comparison generates
19,668 points with independent NumPy forward equations from analytic surfaces
in 90 selected private streams, then checks the recovered parameters modulo
their angular periods. Maximum parameter difference is below `2e-13`, and
maximum forward residual below `4e-16` in source units. Private inputs and
comparison tools remain outside the repository.

## Oriented surface normals

`EvaluateSurfaceNormal(model, surface_id, u, v)` returns an oriented unit
normal in source coordinates for every currently evaluable surface: plane,
cylinder, cone, sphere, torus, NURBS, spun and swept surfaces. It normalizes
the natural cross product `dP/du x dP/dv` and reverses it for surface sense
`-`. Unknown sense returns `kUnsupportedFormat`. Generator-curve senses,
face sense, placement transforms and unit conversion do not alter this result.
`EvaluateSurface` continues to return natural, unoriented partials.

Both partials are scaled independently before crossing to avoid overflow or
underflow caused by their magnitudes. Zero, nonfinite or numerically
unresolvable cross products return `kInvalidRecord`. Known spherical-pole and
apple/lemon/horn torus boundary parameters are rejected explicitly rather
than normalizing trigonometric rounding residues. A surface point may remain
evaluable at such a singular parameter even when its parametric normal does
not. No arbitrary minimum angle between partials is imposed; this API does
not certify numerical conditioning, global regularity or topology.

The API propagates the complete surface evaluator's domain, dependency,
resource and representability checks, including those on the point and both
partials. It introduces no CLI settings or dependencies and remains MIT and
STL-only. Oriented normals provide a building block for intersection chart
parameterisation. The numeric chart API accepts supplied tangents; the model
API obtains them through analytic or NURBS inversion. Local intersection
evaluation is described below; procedural surface inversion remains pending.

Tests cover source orientation, rotated frames, rational/nonrational patches,
procedural surfaces, radii from `1e-300` to `1e300`, subnormal derivatives,
singular parameters and explicit errors. An independent comparison using
complex-step analytic derivatives, SciPy BSpline and Rotation verifies 24,441
normal samples across 90 selected private streams, with maximum component
difference below `9e-13`. Six further samples return explicit errors because
their generator has no typed payload. Private inputs and comparison tools
remain outside the repository.

## Intersection chart parameters

`ParameterizeIntersectionChart(chart, tangents)` reconstructs point parameters
and chord projection scales from the transmitted chart and caller-supplied
natural tangent directions. One finite nonzero tangent is required at every
point, oriented in chart order; magnitudes need not be one. For an intersection
the direction is normally the cross product of the two oriented surface
normals, in stored surface order. The function preserves the source chart,
base parameter and initial scale. For chords of length `C[i]` it implements
the XT Format Reference (July 2022, pp. 46–47) recurrence:

```text
f[0]   = base_scale
t[0]   = base_parameter
f[i]   = f[i-1] * cos(b[i]) / cos(a[i])
t[i+1] = t[i] + C[i] * f[i]
```

Here `a[i]` is the angle from the tangent to the next chord, and `b[i]` the
angle to the preceding chord in its forward direction. The result contains
one parameter per point and one scale per chord. The tangent ratio gives
matching first derivatives for the adjacent chord projection rules, provided
the supplied tangents describe the actual curve. Consecutive duplicate points
and nonpositive tangent projections are rejected; a closed chart may repeat
its first position at the end without wrapping its parameters.

At least two points, finite coordinates/base parameter and a positive finite
base scale are required. Each input position/tangent array is bounded to
1,000,000 coordinate scalars (333,333 points). Count mismatch is an invalid
option, oversized inputs exceed the resource limit, and invalid geometry or
unrepresentable results are invalid records. The recurrence uses wide
intermediates, with range checks and strictly increasing published `double`
parameters; even positive increments that round to the preceding parameter
are rejected. Tangent normalization is scaled to handle extreme magnitudes.
Runtime is linear, with only the two bounded output arrays retained.

This API does not recover surface UV coordinates or tangents from positions,
certify curve membership or chart error estimates, extend to terminators,
or solve/evaluate the exact intersection between chart points. `EvaluateCurve`
without options continues to require explicit intersection tolerances. No CLI
settings, environment variables or dependencies are added; code remains MIT
and STL-only.

Tests cover C1 projection scales, closed and uneven charts, composition with
oriented normals, malformed inputs, resource bounds, signed zero, subnormal
chords and numeric range failures. A separate 80-digit Decimal calculation
checks 3,550 parameters and 3,354 scales across 100 synthetic charts and 96
selected private charts with externally supplied analytic tangents. Maximum
absolute difference is below `5e-16`. This validates the parameter recurrence,
not automatic tangent reconstruction or a complete intersection evaluator.
Private inputs and comparison tools remain outside the repository.

## NURBS surface inversion

NURBS inversion preserves the original knot domains and homogeneous rational
controls. A damped QR least-squares iteration operates in normalized UV
coordinates to handle independent knot scales. Deterministic starts comprise
an optional caller hint, Greville parameters of the four nearest Euclidean
control points, and a 3-by-3 domain grid. All candidates remain inside the
actual domain `[U[degree], U[control_count]]` in each axis, including unclamped
axes. Closure/periodicity flags do not cause wrapping or domain extension.

`SurfaceParameterSeed{u, v}` can be supplied to either `InvertSurface`
overload. Both values must be finite; a NURBS hint must also be in-domain.
Analytic surfaces validate the finite hint and retain their direct formulas.
A singular or unsuccessful NURBS hint falls back to the other starts. Success
requires the forward point's Euclidean residual to meet the unchanged caller
tolerance. This is a bounded local membership search, not a certified global
closest-point method or proof of root uniqueness/nonexistence.

Each start permits at most 32 updates and 12 halving trials per update. A
whole inversion permits at most 512 forward evaluations and 32,000,000
estimated validation/evaluation work units; exhaustion is `kResourceLimit`.
Degree, knot and control validation precedes the search and retains the
forward evaluator's limits. An unsuccessful search is `kInvalidRecord`.
Singular patches can yield valid membership parameters while their normal
remains undefined. The surface and model are never changed.

Tests cover curved polynomial and rational patches, corners, independently
scaled domains, homogeneous scaling, singular hints, explicit membership
tolerances, invalid definitions/hints, unclamped and flagged periodic axes,
work exhaustion and model dispatch. An independent SciPy tensor-BSpline
comparison generates 4,356 points on 1,089 NURBS surfaces from the 90-stream
selection. All unseeded inversions succeed at `1e-10` tolerance; the maximum
independent forward residual is below `9.85e-11` source units. Parameter
uniqueness is not assumed for arbitrary source surfaces. Private inputs and
comparison programs remain outside the repository.

## Model-based intersection charts

`EvaluateIntersectionChart(model, curve_id, tolerance)` prepares a directly
referenced `INTERSECTION` chart without caller-supplied tangents. Both surface
targets must support analytic or NURBS inversion and have known senses. At each stored chart point it:

1. Inverts the point onto both surfaces with the caller's explicit absolute
   distance tolerance in source units. The previous recovered UV supplies the
   next point's initial hint, with the inversion's fallback starts retained.
2. Evaluates the oriented surface normals at the recovered parameters.
3. Forms the unit tangent `N0 cross N1` in stored surface order.
4. Applies the existing XT chart-parameter recurrence to these directions.

`IntersectionChartEvaluation` returns one pair of `SurfaceParameters`
(including measured residuals) and one unit tangent per chart position, plus
the point parameters and chord scales. Returned surface parameters retain
their original NURBS knot units or analytic principal angular branches,
while chart parameters increase without
periodic wrapping. Source positions, surface order, frames and metadata stay
unchanged. Curve and face senses do not flip the natural tangent. Reversed
chart order is rejected rather than silently corrected.

Inversion and normal failures identify the zero-based chart point and surface
side. Missing/wrong roles, singular/parallel normals, invalid chart order and
numerical range failures remain explicit errors. Untyped, unsupported or
unknown-sense surface dependencies return `kUnsupportedFormat`. The chart is
bounded to 1,000,000 position scalars (333,333 points) before any result arrays
are allocated. Computation is linear with bounded output arrays; no pointers
into the model are retained in the result.

Membership checks use only the supplied tolerance; auxiliary chart error
estimates do not relax it. Normals are sampled at the recovered UV values,
whose forward positions may differ from the stored point by that tolerance.
The result does not certify exact surface/curve membership, chart accuracy,
conditioning, branch topology or limit correspondence. Additional type-204
UV data is not inferred or applied. Trim wrappers and terminator extensions
are not traversed. The separate `EvaluateIntersectionCurve` API evaluates
analytic/NURBS interiors with explicit solver options. `EvaluateCurve` without
options retains its unsupported result. No CLI settings or dependencies
are added; the implementation remains MIT and STL-only.

Synthetic tests cover surface order/senses, forward references, source units,
explicit tolerances, contextual failures, singularities, closed charts,
copy/move behavior and resource limits. Independent SciPy tensor-BSpline
partials, complex-step analytic partials and an 80-digit Decimal recurrence
check 659 prepared charts and 3,516 points among 1,409 typed intersections in
90 selected private streams, compared with 419 charts before NURBS inversion.
At `1e-8` tolerance the maximum independent residual is below `9.57e-9`,
tangent component difference below `1.96e-9`, and normalized parameter/scale
difference below `5.52e-10`. Another 720 charts have unsupported dependencies;
30 fail explicit NURBS membership. Independent bounded SciPy searches from
nine starts also find residuals above tolerance for those 30, ranging from
`7.03e-8` to `6.42e-3`; this is not a proof of the global minimum. Private
inputs and comparison programs remain outside the repository.

## Local intersection evaluation

`EvaluateIntersectionCurve(model, curve_id, parameter, options)` evaluates a
direct `INTERSECTION` with analytic/NURBS supports inside its reconstructed
chart. It returns
a point and natural first derivative without flipping them for curve sense.
`IntersectionEvaluationOptions` requires two explicit finite tolerances in
source units: nonnegative `chart_tolerance` for stored-point membership and
positive `solve_tolerance` for interior residuals. Optional
`chart_parameter_tolerance` controls outer endpoint snapping in original curve
parameter units, with zero as the strict default. A coarse source chart does
not force a loose numerical solve. The `EvaluateCurve` overload without options
retains an explicit unsupported-format result and points callers to this API.

For an interior parameter, the chart defines a chord point by linear
interpolation of its endpoint parameters. The XT Format Reference (July 2022,
p. 47) selects the local intersection of the two supporting surfaces with
the plane through that point perpendicular to the chord. The implementation
solves four equations in the two surface UV pairs: three coincidence
coordinates and the first surface point's signed distance to that plane.
The returned position is the first surface's evaluated point. Its distance
to the second surface's evaluated point and its absolute plane residual must
both satisfy `solve_tolerance`.

The initial UV pairs interpolate recovered chart parameters, using local
periodic branches for analytic azimuth and ring-torus meridians. NURBS
coordinates interpolate in their original knot units without angular
wrapping. Newton updates use
natural surface partials, independently scaled Jacobian columns and partial
pivoting. At most 32 updates are attempted, each with at most 16 halving
trials that must decrease the residual. Invalid trial parameters are rejected;
singular systems, stalled or nonconvergent iterations return explicit errors.
There is no polyline fallback or global branch search.

The oriented unit tangent `T` is formed from the two surface normals at the
solution. For unit chord direction `D`, chord length `C` and parameter span
`dt`, the derivative is `T * C / (dt * dot(T,D))`. Nonpositive projections and
unrepresentable derivatives are errors. At an exact chart parameter the
stored position is returned, subject to `chart_tolerance` rather than the
interior solve tolerance. Knot derivatives use the following chord, except
at the final endpoint. Source rounding can affect numerical C1 agreement.

Chart preparation is repeated per call and retains its bounds and dependency
checks. Parameters outside the chart beyond the explicit endpoint tolerance
are invalid options; terminator extensions,
trim wrappers and other surface kinds remain unsupported by this direct API.
Residual tolerances are not bounds on point/derivative error near poorly
conditioned intersections. Global branch identity, regularity, chart accuracy
and topology are not certified. No CLI settings or dependencies are added;
the implementation remains MIT and STL-only.

Tests compare circles and lines with closed-form positions/derivatives and
central differences, including angular seams, knot behavior, signed source
orientation, cylinder/cone/torus sections, separate tolerance roles and explicit
numerical failures. An independent SciPy root solver with complex-step
Jacobians checks 5,400 interior evaluations across the 419 analytic charts
already accepted in the 90-stream selection. All queries succeed with
`chart_tolerance = 1e-9` and `solve_tolerance = 1e-13`. Maximum point component
difference is below `3e-13` in source units, and normalized derivative component
difference below `8e-8`. Private inputs and comparison tools remain outside
the repository.

With NURBS supports enabled, the same 90-stream comparison at
`chart_tolerance = 1e-9` and `solve_tolerance = 1e-13` prepares 659 charts.
Of 8,571 interior queries, 8,554 return solutions and 17 explicitly report
stalled/nonconvergent solves or incompatible tangent direction. Maximum
position component difference from the independent SciPy root solver is
below `9.35e-10` source units. Derivatives are more sensitive: 58 returned
samples exceed the earlier analytic comparison's component thresholds, with
Jacobian condition numbers above `3.9e5`; the maximum normalized derivative
component difference is `8.02e-3`. Small coincidence residuals therefore do
not certify derivative accuracy near poorly conditioned intersections.
These limitations remain visible; no tolerance relaxation or tessellation
fallback is used. The rational NURBS/plane regression separately checks
closed-form interior points and derivatives, including a knot interval wider
than `2*pi` to prevent accidental angular wrapping.

## Intersection regularity diagnostics

`EvaluateIntersectionCurveWithDiagnostics(model, curve_id, parameter, options)`
returns an `IntersectionCurveEvaluation` containing the same `curve` position
and natural derivative as `EvaluateIntersectionCurve`, plus two local
geometric measures:

- `diagnostics.surface_normal_sine` is the sine of the angle between the two
  supporting unit surface normals. Near zero indicates nearly parallel or
  antiparallel normals, where the intersection direction becomes sensitive.
- `diagnostics.tangent_chord_projection` is the dot product of the unit tangent
  and selected unit chart chord. Near zero amplifies `dP/dt` through the
  reciprocal projection in the XT parameterisation.

Both values are normalized, finite and in `(0,1]`. Normal senses do not affect
the sine. The projection retains the stored surface order and selected chord;
a reversed/nonpositive direction remains an error. Calculations use wide
intermediates, normalize the reported angular measures and clamp rounding
overshoot to one. The derivative retains its original projection formula.

`IntersectionEvaluationOptions` appends `minimum_normal_sine` and
`minimum_chord_projection`. Each must be finite and in `[0,1]`, with zero
preserving the existing singularity/direction checks. Both the diagnostic
and existing curve APIs return `kInvalidRecord` when a reported measure is
strictly below its requested minimum. Equality is accepted; there is no
implicit angular tolerance. These settings are explicit caller policy, not
new defaults or inferred error tolerances.

Checks apply at the queried UV values, including exact chart knots and
snapped outer endpoints. A stored knot still returns its stored position;
its normals and diagnostics use the recovered surface parameters. Through
`EvaluateCurve(..., options)`, the same checks also apply under nested trims.
Valid minimums have no effect on ordinary curves, while invalid options are
rejected for every geometry kind before lookup. Residual/parameter tolerances,
strict trim bounds, Newton iteration limits and resource limits are unchanged.

Passing the minimums does not certify derivative accuracy, global regularity,
branch identity, BRep topology or a bound on the full UV Jacobian condition.
A caller can inspect or reject sensitive local samples without relaxing any
geometric tolerance or converting the curve into a tessellation. The library
remains MIT and STL-only, with no new CLI settings or dependencies.

Tests compare diagnostics against closed-form circle sections and lines with
normal sines down to `1e-150`, including nearly antiparallel supports. They
cover inclusive minimums, unit upper bounds, exact/snapped endpoints, NURBS
supports, nested trim dispatch and invalid options.

A 90-stream regression checks 8,571 interior queries over 659 prepared charts.
With default minimums, all 8,554 successful positions and derivatives are
bit-identical to the previous iteration, and all 17 errors are unchanged.
An independent NumPy comparison of normalized returned derivatives with
selected chords agrees with the reported projection within `4.45e-16`.
As an example caller policy, minimum normal sine `0.001` and chord projection
`0.1` reject 232 of those successful samples, including all 58 derivative
comparison outliers identified in the preceding iteration; 8,322 remain
accepted. This selection does not establish general error bounds or prescribe
production thresholds. Private inputs and comparison tools remain outside
the repository.

## Trimmed intersections

`EvaluateCurve(model, curve_id, parameter, options)` accepts the same
`IntersectionEvaluationOptions` as the direct intersection solver. It dispatches
supported direct intersections and those reached through `TRIMMED_CURVE` chains
to that solver. Both options are validated for every geometry kind; other
curve types use their existing evaluators. The overload without options keeps
its explicit unsupported-format result for intersections, including trims.

Both overloads share the same bounded trim resolver. Every inclusive trim
interval and basis-sense check applies, with cycle detection and a maximum of
64 wrappers. Missing/wrong bases and untyped dependencies remain explicit
errors. Source basis parameters and natural derivatives are preserved without
reparameterising or reversing them; the intersection's reconstructed chart
bounds and optional chart endpoint tolerance also apply. Stored endpoint
agreement, periodic spans, branch topology
and terminator extension are not certified or inferred. No default tolerances,
CLI settings or dependencies are added; code remains MIT and STL-only.

Tests cover nested forward references, descending intersection sense, inclusive
trim bounds, chart bounds, unsupported bases, cycles, the 64-wrapper boundary,
option validation and unchanged NURBS behavior. The earlier 2,050-trim comparison
still verifies 10,050 samples in 30 streams, with 115 unsupported surface-inversion
samples, five untyped payloads and 80 explicit NURBS domain errors. An additional
targeted comparison checks 24 trimmed analytic intersections from seven private
streams using `chart_tolerance = 1e-9` and `solve_tolerance = 1e-13`.

Of their 120 queries, 118 match independent SciPy/complex-step evaluation:
maximum point component difference is below `5e-14`, normalized derivative
difference below `2e-13`, and agreement with evaluated stored trim endpoints
within `2.3e-12` in source units. Two stored endpoint parameters exceed the
reconstructed chart maximum by approximately `1.7e-17` and `2.4e-17`; the strict
default rejects them. The optional parameter tolerance below handles these
queries without changing that default. Private inputs and comparison tools
remain outside the repository.

## Chart endpoint parameter tolerance

`IntersectionEvaluationOptions::chart_parameter_tolerance` is a finite
nonnegative absolute tolerance in the **original curve parameter units**.
It defaults to zero. If a query lies outside the reconstructed chart by no
more than this tolerance, intersection evaluation returns the nearest outer
chart endpoint and its natural first derivative. It does not extrapolate a
new curve segment or return the zero derivative of a clamped continuation.

Only outside queries are adjusted. Interior queries and inner chart knots
retain their parameters, even if the configured tolerance exceeds an interval's
width. Larger outside gaps remain invalid options. Gap comparison uses wide
intermediates. Trim intervals remain strictly inclusive and are checked before
intersection evaluation; this option cannot admit a query outside any enclosing
trim. Chart membership and solver residual tolerances keep their separate
roles and source-distance units. Auxiliary chart error estimates never supply
an implicit tolerance. Invalid parameter tolerances are rejected by both the
direct intersection API and the model curve overload, including ordinary
curves evaluated with options.

Tests cover both endpoints, strict defaults, the inclusive tolerance boundary,
larger gaps, unchanged interior samples/knots, natural endpoint derivatives,
trim-bound enforcement and invalid options. The targeted seven-stream,
24-trim comparison now verifies all 120 queries with
`chart_tolerance = 1e-9`, `solve_tolerance = 1e-13` and
`chart_parameter_tolerance = 1e-16`. Independent SciPy/complex-step comparison
still gives maximum point component difference below `5e-14`, normalized
derivative difference below `2e-13`, and stored endpoint distance below
`2.3e-12` in source units. Repeating the same comparison with the zero default
retains the original 118 successes and two explicit chart-bound errors.
There are no new CLI settings or dependencies; code remains MIT and STL-only.
Private inputs and comparison tools remain outside the repository.

## Intersection curve definitions

`INTERSECTION` (type 38) now has a typed `IntersectionCurveGeometry`, reported
as `kind: "intersection_curve"`. It retains the ordered pair of surface IDs,
the chart, and both limits. Charts retain their source ID, base parameter,
positive base scale, nullable chordal/angular/parameter errors and ordered
positions. Limits retain their source ID, raw kind, optional `term_use` byte
and positions. Help (`H`) and arbitrary (`L`) limits require one position;
terminators (`T`) require the singular point and branch point in that order.
Other kinds remain explicitly unsupported.

Forward references to CHART/LIMIT records are resolved by node ID and type,
independently of node order. Definitions require a nonempty chart whose count
matches the transmitted positions, finite coordinates and metadata, and
nonnegative chordal/angular estimates when set. Surface IDs must be non-null;
surface availability and class findings remain in the reference graph rather
than requiring an evaluator during model construction. Finite parameter-error
values are retained without inferring a tolerance contract.

The resolved schema controls optional fields: absent `term_use` and
`intersection_data` fields stay absent in JSON, whereas an explicitly null
data reference is reported as zero. The optional data reference remains an ID;
type-204 UV arrays remain in the raw graph. No UV tuple layout is inferred.
Chart and limit hvecs transmit positions only in these non-delta streams;
tangents, surface parameters and individual chart parameters are not invented.

Model construction provides definitions for the separate intersection evaluator.
`EvaluateCurve` without options returns an explicit unsupported-format error for an
intersection curve, including through a trim. The chart is a chordal guide,
not the exact curve. The separate parameterisation API requires supplied
oriented tangents; interior evaluation requires solving the documented local
three-surface intersection. Terminator handling, branch consistency, chart
accuracy and endpoint agreement are not certified by the typed adapter.

Copied chart and limit occurrences each consume their full coordinate count,
plus 17 scalar slots for definition metadata. The budget is checked before
allocating position arrays, including when many curves share auxiliary nodes.
The implementation follows the XT Format Reference (July 2022, pp. 44–47),
remains MIT and STL-only, and adds no CLI settings or dependencies.

Synthetic tests cover schema variations, optional metadata, forward and
invalid references, nullable/nonfinite values, limit cardinalities, copy/move
behavior, copied-value limits and CLI output. A separate source-to-model
comparison checks all 336 intersection definitions and 3,140 copied positions
in 30 selected private streams, with no intersection-adapter issues. This is
a preservation check, not a numerical curve-evaluation comparison. Private
inputs and comparison tools remain outside the repository.

## Build and run

The top-level build includes the library and `parasolid_inspect`:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel
./build/asmbox/swx_dump --extract-parasolid /tmp/parasolid part.SLDPRT
./build/libopenparasolid/parasolid_inspect --summary /tmp/parasolid/partitions/payload-1-offset-4.x_b
```

Inspect one extracted partition, delta or other binary transmit stream:

```bash
parasolid_inspect part.x_b > inspection.json
parasolid_inspect --summary part.x_b
parasolid_inspect --max-records 100000 changes.bin
```

`--summary` omits per-candidate and unknown-range arrays. The default candidate
limit is 10,000; the maximum accepted limit is 1,000,000. CLI limits must be
positive. Input files are bounded to 128 MiB by default.

Exit status:

- `0`: the supported candidate scan reached the end of the input.
- `1`: input, header, options or resource-limit error; details go to stderr.
- `2`: a report was written, but the candidate scan was limited or the schema
  or user-field layout has no supported candidate layouts.

Status 0 does not establish a complete Parasolid document or valid B-Rep.

The library also builds separately:

```bash
cmake -S libopenparasolid -B build-parasolid -DBUILD_TESTING=OFF
cmake --build build-parasolid --parallel
```

For the standard-library-only target, add
`-DOPENPARASOLID_BUILD_TOOLS=OFF`. Standalone tests require an installed
GoogleTest package; the top-level build supplies its existing GoogleTest target.

## API

Public headers are under `include/openparasolid/`:

```cpp
#include <openparasolid/parser.h>

openparasolid::InspectOptions options;
options.max_records = 100000;
auto result = openparasolid::ParasolidInspector::InspectFile("part.x_b", options);
if (!result.ok()) {
  std::cerr << result.error().message << " at " << result.error().offset << '\n';
  return;
}
const auto& report = result.value();
std::cout << report.header.schema << '\n';
std::cout << report.candidates.size() << " record candidates\n";
```

`Inspect(span, options)` reads memory directly. Results own their strings,
metadata and previews; they retain no pointers into the supplied span.
`ParasolidReader::ReadHeader(span)` reads the common and internal headers.
`ReadCommonHeader(span)` reads only the signature, description and schema;
container extraction uses this smaller contract. Calls have no shared mutable
state.

## Header and schema reporting

The neutral binary signature is `PS 00 00`. Header reporting includes the
big-endian description length, description, schema token and its byte range.
The common-header reader permits bounded padding before the length-prefixed
schema. All offsets refer to the complete extracted input, not the original
SolidWorks file or a decompressed frame.

Numeric schema keys have two or three components:

- `SCH_<modeller>_<schema>`: a base-less key.
- `SCH_<modeller>_<schema>_<base_schema>`: an embedded-schema key.

An embedded-schema header carries a maximum schema type (`u16 BE`), followed
by the user-field word count (`i32 BE`, permitted values 0 through 16). A
base-less header carries the word count directly. `body_offset` follows these
fields when `internal_header_complete` is true; `schema_end_offset` ends the
common prologue. Unknown nonnumeric schema declarations remain inspectable
through the common header but do not enable a record scan.

A schema key identifies a layout; it does not establish complete model support.
The sequential parser below resolves embedded schemas independently of candidate inspection.

## Candidate inspection

For zero-user-field streams using schema/base schema 13006, the inspector
searches for these fixed-layout prefixes:

| Tag | Candidate kind | Additional checks |
|---|---|---|
| `00 0e` | Face-use / bridge | Topology marker and orientation |
| `00 0f` | Loop head | Bounded short-reference fields |
| `00 10` | Edge-use | Topology marker |
| `00 11` | Coedge | Bounded references and orientation |
| `00 12` | Vertex-use | Topology marker |
| `00 1d` | Point | Sentinel reference and finite coordinates |

The optional `ff` schema marker and short, bare `u16` reference cells are
recognized. Extended pointers, tripled cells, variable ownership nodes,
geometry carriers and schema edits remain outside the candidate scan.
A syntactic prefix match can occur inside other data. `candidates` and
`candidate_counts` therefore report candidates, never proven entity counts.
`prefix_size` describes the matched prefix, not a complete record boundary.
Coordinates retain source values; the inspector does not infer a length unit.

References retain their field, source offset and attribute value. A known
reference role is matched by expected candidate kind and attribute **within
this input only**. Results distinguish sentinels, unique candidates, ambiguous
candidates, unresolved references and uninterpreted target families. A unique
candidate does not prove a valid topology connection. No partition/delta
merging or saved-state reconstruction takes place.

Unknown regions preserve the complement of the matched prefixes, with offsets,
lengths and up to 16 preview bytes. Original bytes remain recoverable from the
input. Byte accounting is:

```text
body_offset + candidate_prefix_bytes + uninterpreted_bytes == byte_size
```

The JSON report includes header metadata, candidate and reference counts,
scan-limit status, per-candidate fields and unknown regions. It is an inspection
report, not a geometry interchange format. Text X_T input, general geometric evaluation,
delta application and STEP output remain future work.

## Sequential schema and node decoding

```bash
parasolid_inspect --nodes part.x_b > nodes.json
parasolid_inspect --nodes --summary part.x_b
parasolid_inspect --schema-catalog /path/to/catalog.json part.x_b
```

This mode consumes nodes in order using resolved field definitions. It never
resynchronizes by scanning for tag patterns. It supports embedded full
definitions, unchanged definitions (`ff`) and the `C/D/I/A/Z` schema edit
operations, including implicit removal of remaining base fields at `Z`.
Fixed and variable arrays, compact indices, nullable numeric scalars and
composite numeric fields are decoded. An `h` field transmits three coordinates.
Nontransmitted variable fields consume no value bytes; transmitted opaque
fields stop with an unsupported-codec diagnostic. Node indices must be positive
and unique. The terminator is type 1, index 0, at the exact end of the stream.

The schema-mode JSON report includes a `limits` object with all effective
`SchemaLimits` values, also in summary reports. Positive integer options can
adjust the following bounds; their defaults remain unchanged:

| Option | Default | Scope in schema mode |
|---|---:|---|
| `--max-records N` | 10,000 | Ordinary nodes plus delta node occurrences |
| `--max-fields N` | 65,536 | Field groups, applied separately to the catalog, resolved schemas and decoded nodes |
| `--max-values N` | 1,000,000 | Scalar values, including composite components and pointers; status bytes are excluded |
| `--max-delta-entries N` | 100,000 | Reference-map entries plus historical mark occurrences |

The last three options require `--nodes` or `--schema-catalog`; they do not
change candidate-scan mode. Limits remain independent: increasing the node
bound does not increase the field, value or packet bounds. Bounds reached
during node/packet decoding retain the completed prefix and report exit
status 2. Initial input or catalog limit errors and invalid limit arguments
report exit status 1. No automatic retry or unlimited setting is used.

A larger explicit profile used for the local corpus is:

```bash
parasolid_inspect --nodes --summary \
  --max-records 300000 --max-fields 3000000 \
  --max-values 5000000 --max-delta-entries 300000 part.x_b
```

The built-in catalog covers 54 base types for streams with base schema 13006:

| Family | Types |
|---|---|
| Administration | PARTITION 3, PMARK 4, WORLD 101 |
| Body topology | BODY 12, SHELL 13, FACE 14, LOOP 15, EDGE 16, FIN 17, VERTEX 18, REGION 19 |
| Points and analytic curves | POINT 29, LINE 30, CIRCLE 31, ELLIPSE 32 |
| Analytic surfaces | PLANE 50, CYLINDER 51, CONE 52, SPHERE 53, TORUS 54 |
| Procedural surfaces and forms | BLENDED_EDGE 56, BLEND_BOUND 59, OFFSET_SURF 60, SWEPT_SURF 67, SPUN_SURF 68, HELIX_SU_FORM 163, HELIX_CU_FORM 184 |
| Intersections | INTERSECTION 38, CHART 40, LIMIT 41, GEOMETRIC_OWNER 141 |
| Lists | LIST 70, POINTER_LIS_BLOCK 74 |
| Spline dependencies | BSPLINE_VERTICES 45, B_SURFACE 124, SURFACE_DATA 125, NURBS_SURFACE 126, KNOT_MULT 127, KNOT_SET 128, TRIMMED_CURVE 133, B_CURVE 134, CURVE_DATA 135, NURBS_CURVE 136, SP_CURVE 137 |
| Transforms | TRANSFORM 100 |
| Attributes and values | ATT_DEF_ID 79, ATTRIB_DEF 80, ATTRIBUTE 81, INT_VALUES 82, REAL_VALUES 83, CHAR_VALUES 84, VECTOR_VALUES 86, UNICODE_VALUES 98, FIELD_NAMES 99 |

The field layouts follow the public XT format and reviewed local reference
readers. They describe the 13006 base, not the effective fields of a newer
producer. BODY mesh/polyline/index fields, REGION owners and intersection
extensions must arrive through embedded schema edits. They are not guessed
members of the base. WORLD follows the public XT Format Reference (April 2008,
pp. 82-83 and 118); PARTITION/PMARK use the reviewed wire structure below.
INTERSECTION_DATA (204) is explicitly known absent from base 13006; its
complete first-use definition is read from the stream. Other unlisted types
have **unknown** base membership. The parser stops there rather
than interpreting an unknown base type as a new embedded type. This is raw
schema decoding; it does not construct or validate a B-Rep. Nonzero user-field
word counts are unsupported.

An external JSON catalog must name the **complete exact input schema key**:

```json
{
  "schema_key": "SCH_1234567_12345_13006",
  "source": "synthetic example",
  "base_types": [
    {
      "node_type": 12,
      "name": "SYNTHETIC",
      "description": "Example only; not the actual BODY definition",
      "fields": [
        {"name": "value", "type": "d", "pointer_class": 0,
         "elements": 0, "transmitted": true}
      ]
    }
  ],
  "absent_types": [204]
}
```

`base_types` supplies known complete base layouts; `absent_types` explicitly
asserts nonmembership in the base. Omitted types remain unknown. For a key
without an embedded base, definitions are used directly with no schema bytes.
Catalog files are limited to 4 MiB and JSON nesting to 64 levels. They are
provided by the caller and are not downloaded or inferred from nearby versions.
Field codes are `u c l n w d p q t f i v b h`. `elements` is 0 for a scalar,
1 for the single final variable field, or at least 2 for a fixed array.
A nonzero `pointer_class` requires codec `p`.

In C++, include `<openparasolid/schema.h>` and call
`SchemaParser::Parse(span, catalog, limits)` or `ParseFile(path, catalog, limits)`.
`ReadDefinition` exposes standalone decoding when base membership is already
known. Reports own their data and retain schemas, edit offsets, node ranges,
field ranges and values. Composite values are flattened with an explicit
component stride. Unset values become JSON `null`; nonfinite doubles retain
all bits in a `nonfinite_f64_bits` object.

Header, option and catalog failures return an error (CLI status 1). A later
unsupported type, malformed node or resource limit returns the completed
prefix plus a diagnostic, the failing node's offset and the uninterpreted tail
(CLI status 2). Schemas resolved before a failed node may still be reported.
`complete: true` and CLI status 0 require the valid final terminator; they do
not validate reference targets, topology, geometric consistency or delta state.
`--summary` omits schema and node arrays. `--max-records` sets the positive
node limit in this mode. Defaults bound nodes to 10,000, resolved types to 1,024,
fields per type to 4,096, catalog/resolved/decoded fields each to 65,536, scalar
values to 1,000,000 and input to 128 MiB; C++ callers can set `SchemaLimits`.
Byte accounting is:

```text
complete:   decoded_end + terminator_size == byte_size
incomplete: decoded_end == uninterpreted_offset
            uninterpreted_offset + uninterpreted_size == byte_size
```

## Delta administration prefix

`--nodes` reads the leading PARTITION (type 3) and its current PMARK (type 4)
in a deltas stream. Their two base layouts contain 2 and 10 fields. First-use
`ff` and schema edits that preserve the reviewed field roles are supported.
Unknown or modified roles stop with an unsupported-schema diagnostic.
An explicit caller catalog remains authoritative and must supply these layouts
with the same field names, types and pointer classes for typed interpretation.

PARTITION carries `current_pmark` (pointer class 4) and `highest_id` (32-bit word).
PMARK carries four class-4 references (`preceding`, `first_following`,
`next_sibling`, `prev_sibling`), four words (`n_new_nodes`, `n_del_nodes`,
`n_copy_mod_nodes`, `delta_key`), one logical `delta_is_forward` byte, and the
32-bit `identifier`. Its scalar tail is 21 bytes; the logical precedes the
identifier. The node index and reference indices use compact transmit encoding,
including extended indices. Decoded index zero means null.

The PARTITION index must differ from its non-null `current_pmark`, and that
reference must name the following PMARK. Allocation need not be consecutive.
The PMARK direction must be 0 or 1. No ordering or null constraint is guessed
for historical marks that have not yet been read.

`SchemaParseReport::delta_prefix` and JSON `delta_prefix` expose the validated
pair, its byte range, the partition and the current mark. This object also
appears with `--summary`; it is null until both complete nodes and their link
are valid. IDs, counts and keys retain all 32 bits here, including bit patterns
that generic schema integer values represent as negative or unset. The generic
node/field report retains its normal scalar interpretation and exact ranges.

The following supported packets are retained separately as described below.
Unknown framing keeps the whole undecoded tail and the validated prefix. A failed PMARK leaves the preceding completed PARTITION in
`nodes`, but no validated `delta_prefix`. Ending a deltas stream without the
pair is an error. A delta stream can finish with two or four null references at the exact end;
the four-reference form takes precedence. These have `terminator_size` 4 or 8.
`complete` describes framing, not applied changes.

This reads administration and bounded packets. It does not yet reconstruct
the full mark graph, apply changes, select a configuration, or merge a partition
with its changes. The next parser boundary is retained as a diagnostic.

The field structure was compared with cadmpeg's documented PARTITION/PMARK
format and checked against 811 locally extracted SolidWorks delta streams.
The C++ implementation and synthetic tests are original MIT code. No Apache
parser code or private CAD fixtures are incorporated.

## Delta reference maps and historical packets

After the validated prefix, `--nodes` consumes supported packets only at the
current sequential boundary. `delta_packets` retains each packet's kind,
offset, size, terminal and entries. Summary JSON reports `delta_packet_count`,
`delta_reference_count` and `delta_historical_mark_count`, while omitting the
packet arrays. Packet bytes contribute to `decoded_end`; a failed packet
contributes no bytes or entries and its entire span remains uninterpreted.

Two forms are supported:

- A self-delimiting reference/type map is `0001 0001`, zero or more
  `compact_reference, node_type:u16 BE` entries, then `0001 0000 word:u16 BE`.
  Entry references are non-null and the terminal word is nonzero. Empty maps,
  extended indices and repeated references are retained. Type labels are
  bounded by the header range and remain declared labels; their schema and
  target existence are not established by the map. Entries alone do not imply
  creation, deletion, replacement or final-state membership.
- A historical PMARK packet is `0001 0001` followed by one or more type-4
  instances in the already established base layout, without another first-use
  schema declaration. An optional ten-byte terminal is
  `0001 0001 0001 00000000` or `0001 0001 0001 00000001`.
  It belongs to the preceding historical packet. Its final value is retained
  in `terminal_word` as numeric 0 or 1; without a terminal the field is null.
  No state or direction meaning is assigned to that value.
  Every occurrence keeps its fields, byte range and full-width words. Repeated
  identities remain separate events in packet order; they do not replace the
  prefix's current mark or enter the current `nodes` identity table.

The standalone packet reader stops as ambiguous if both grammars complete at
the same start. It does not interpret bytes after a candidate packet.
`SchemaParser` has a complete sequential stream and additionally checks the
two-byte node type immediately after each ambiguous candidate. A type outside
`1..schema_max_type` makes that boundary impossible. If exactly one candidate
has an impossible boundary and the other is followed by an in-range type,
the latter is retained. An in-range type is only a necessary condition; normal
schema and payload validation still follows. No bytes are skipped or searched.

Two in-range boundaries remain ambiguous, including unknown base types.
Missing boundary bytes do not select a candidate. Two out-of-range boundaries
produce an invalid-record diagnostic without committing either candidate.
A resource cap also cannot select one grammar by eliminating the other. Maps
without the self-delimiting word terminal, shared-kind boundaries, the alternate
one-byte map prefix and other change-packet forms remain unsupported.
Historical packet capture does not validate reachability, sibling reciprocity,
acyclicity or a selected state. A repeated historical identity is preserved,
not asserted to be a valid unique mark in a reconstructed graph.

`DeltaPacketReader::Read(span, offset, schema_max_type, max_entries)` exposes
standalone packet framing. Callers must establish the packet boundary and the
base PMARK layout. `SchemaParser` establishes those conditions from its prefix.
`SchemaLimits` additionally bounds packets to 10,000 and the combined number
of reference entries and historical marks to 100,000 by default.

Supported status-framed nodes can follow these packets, as described below.
Missing definitions still report an unknown base type. Complete packet capture
is not delta application or saved-state reconstruction.

## Status-framed delta node occurrences

After PARTITION/current-PMARK, schema-defined delta nodes use their effective
field order and have a binary status byte after **every pointer value**.
The node's own compact index has no status byte. Variable element counts still
precede that index; fixed and variable pointer arrays each carry a status per
entry. Schema metadata has its own ordinary encoding and carries no such
extra statuses. First-use schema resolution uses the same C/D/I/A/Z decoder.

`SchemaParseReport::delta_nodes` and JSON `delta_nodes` preserve these complete
occurrences in source order. Repeated node indices, including indices present
in the administration prefix, remain separate occurrences. They do not replace
`nodes`, which retains ordinary nodes and the delta administration prefix.
`delta_node_count` is present in summary JSON. A `status_framed` node flag and
per-field `reference_statuses` distinguish the two physical encodings. Status
0 and 1 are retained without assigning create/delete/modify meanings. A status
never changes whether a pointer index is null. Invalid statuses and truncated
fields leave the entire failed node uninterpreted.

Delta integer/tag fields expose unsigned 32-bit wire words, including all-ones
and generic unset bit patterns. Ordinary node integer/tag fields keep their
signed/nullable interpretation. Short integers, nullable real values, characters,
logicals and UTF-16 code units keep their respective codecs. A delta `h` value
has eleven scalar components; an ordinary `h` value has three. Nontransmitted
variable fields consume no pointer or status bytes. Unsupported opaque codecs
remain explicit failures.

`max_nodes`, `max_total_fields` and `max_values` include all decoded delta
occurrences; status bytes do not count as additional scalar values. The packet
entry limits remain separate. Current generic nodes still require unique
indices. Plain partition/transmit streams use no pointer statuses.

The built-in base subset covers the families listed above. Unlisted
definitions remain unknown, except for the explicit absence of type 204
from base 13006. Referencing an unknown type does not require its definition,
but decoding its payload does. Full-declaration support is limited to type
204 in the built-in catalog or types explicitly known absent in a caller
catalog. Optional repeated `ff` envelopes,
shared status/tag bytes, compact tombstones and unsupported packet forms are
not inferred from lookahead. A zero terminal value is supported only as the
optional suffix of a historical packet. A standalone reference-map terminal
with a zero word remains unsupported.

The C++ field codecs and definitions are MIT code; format facts were checked
against local reference readers. Delta field values, statuses and node ranges
were compared with an independent Python reader over all 811 local streams.
Even `complete: true` only establishes supported stream framing. No changes
are applied and no final configuration, reference closure, topology or geometry
is certified.

## Topology and analytic geometry records

The additional base layouts decode source fields and preserve source units,
orientation characters, tolerance values, pointer classes and coefficient
order. FACE includes its front- and back-shell links. EDGE preserves its FIN,
curve and ownership references. BODY keeps the 23-field base before embedded
insertions; REGION keeps its seven-field base before an owner is appended.
POINT has four pointer fields and a three-component position; its owner class
is 1011.

Curve and surface records retain their common geometry header, including sense,
and their typed scalar/vector groups. Cone radius, sine and cosine remain
separate source coefficients. Spheres and tori preserve their coordinate axes
and radii. No orientation, parameter range, unit or coefficient is normalized.
INTERSECTION retains its fixed two-surface pointer array and chart/limit links.
CHART and LIMIT use their declared counts and the ordinary three- or delta
11-component `h` encoding. VECTOR_VALUES retains vector element grouping.
GEOMETRIC_OWNER preserves its four ring/geometry references.

Tests use synthetic ordinary and delta records, embedded BODY insertions,
coefficient groups, pointer-status arrays and truncations. Independent local
checks compare newly decoded field values, statuses and node byte boundaries.
Reading these fields does not establish reciprocal links, valid loops, selected
historical state, geometric evaluation or a solid model. Procedural surface definitions and remaining delta forms are separate work.

## Lists and spline dependency records

The 13006 LIST base contains twelve transmitted fields. `list_type` is a
four-byte integer; its trailing block-cursor fields and `notransmit` logical
are part of the base. A stream edit may drop any remaining base fields at `Z`.
The stored `notransmit` value is decoded, not used to suppress later records.

The POINTER_LIS_BLOCK base contains `n_entries`, `next_block` (class 74), and
the final variable pointer array `entries` (class 1001). Its array count
precedes its node index. A newer `index_map_offset` field is decoded from an
embedded insertion, not built into the 13006 layout. Every transmitted delta
pointer, including null array entries and `next_block`, retains its status.
No twenty-entry array size or populated-entry count is inferred from a part
name. List ownership, chain closure, cursor validity and live membership are
not established by raw decoding.

Spline records retain scalar control arrays (45), signed/nullable short knot
multiplicities (127), and real knot arrays (128). NURBS_CURVE and NURBS_SURFACE
keep degrees, vertex counts, dimensions, knot counts, periodic/closed/rational
flags and their control/knot references. U and V descriptor fields remain
separate. B_CURVE, B_SURFACE, TRIMMED_CURVE and SP_CURVE retain their common
geometry headers and distinct tails. CURVE_DATA's analytic-form pointer has
union class 1036; no particular concrete form is assumed. SURFACE_DATA keeps
its four intervals, state bytes and four procedural-form references.

Control coordinates and rational weights are not divided, regrouped, sampled
or approximated. Knot order and duplicates remain unchanged. A descriptor does
not yet cause another node's flat array to become a validated spline or prove
that referenced control and knot counts agree. TRANSFORM retains its nine
matrix scalars, translation, scale, flags and perspective vector without
applying them to geometry or changing matrix order.

Synthetic tests cover ordinary and delta encodings, list field widths, dropped
base tails, inserted list-block fields, extended array pointers, spline
coefficient groups, nullable multiplicities, truncation and oversized counts.
Independent checks compare the new 14 base layouts and completed stream fields.
The local comparison reader uses the documented twelve-field LIST base rather
than sldprt2step's shorter LIST table, and independently retains variable-field
transmission flags. All comparison inputs and outputs stay outside the repository.

## Embedded intersection data

INTERSECTION_DATA (204) is absent from the reviewed 13006 base schema, whose
maximum type is 185. The built-in catalog marks only type 204 as known absent;
it supplies no fixed payload layout for it. Its first occurrence must carry
a complete definition, including the name, description, fields, pointer
classes, array extents and transmission flags. Later occurrences reuse that
stream's resolved definition. An unchanged `ff` marker without a base layout
is an invalid-schema error. Caller catalogs remain authoritative and do not
inherit the built-in absence list.

The observed definitions contain a raw `uv_type` byte and a variable array
of doubles. They preserve all scalar values, unset markers and signed zero.
No UV enumeration, chart association, tuple stride or agreement with hvecs is
inferred from the node number. Full declarations with other valid fields are
handled by the same generic codecs. Empty and nontransmitted arrays retain
their declared count and transmission metadata without consuming value bytes.
Delta pointers, if declared, use the existing per-reference status encoding.
Unknown membership for other node types remains a stop condition even when
the next bytes resemble a full declaration.

Synthetic tests cover full declarations and reuse, alternate field layouts,
nullable doubles, fixed pointer arrays, delta statuses, empty arrays,
nontransmitted arrays, unchanged-marker rejection, truncation, value limits,
base-version scoping and caller catalog precedence. Independent checks over
all 5,976 local streams compare 11,251 completed type-204 nodes in 920 streams,
22,502 field groups and 282,677 values. Their first-use definitions are read
independently rather than supplied as a guessed base layout.

## Attribute definitions and names

ATTRIB_DEF (80) retains its definition chain, identifier reference, type ID,
eight action bytes, FIELD_NAMES reference, fourteen logical owner flags and
variable array of field-type bytes. The 13006 base's nontransmitted callback
pointer is excluded from this layout; it consumes neither bytes nor a delta
pointer status. Embedded schema edits remain authoritative for newer streams.
Action and field codes remain raw bytes. No attribute actions are executed,
and owner legality or consistency with an ATTRIBUTE's values is not inferred.

ATT_DEF_ID (79) retains its variable-length identifier as raw character bytes,
including embedded zero and non-ASCII values. FIELD_NAMES (99) retains a
variable pointer array with union class 1037, whose names can be CHAR_VALUES
or UNICODE_VALUES. Those two value types were already supported. Empty arrays
consume no values or status bytes; each transmitted delta pointer has its own
status, including null and extended references. String decoding, name
resolution and attribute-value interpretation remain separate work.

The three base layouts were checked against the XT Format Reference
(July 2022, pp. 101-102), the 13006 schema supplied with ps-parser, and the
corresponding cadmpeg base descriptors. Synthetic tests cover ordinary and
delta encodings, zero/one/multiple variable entries, fixed array widths,
nullable type IDs, extended references, strict logical values, embedded
insertions, truncation and resource bounds. Independent comparisons over all
5,976 local streams match 53,109 completed attribute-definition/identifier
nodes, 212,583 field groups and 1,211,292 values. No FIELD_NAMES node occurs
in the decoded corpus prefixes; its wire decoding is covered synthetically.

## Procedural geometry and forms

The 13006 base catalog includes blend surfaces and boundaries, offset, swept
and spun surfaces, and helical curve/surface forms. These layouts were checked
against the XT Format Reference (July 2022, pp. 43, 60, 62, 67, 68 and 70) and
independently compared with the corresponding 13006 base field descriptors.
Embedded schema edits still determine each stream's effective layout.

BLENDED_EDGE retains both supporting surfaces, its spine, two-element range
and weight arrays, both boundary references and start/end limits. Each delta
pointer array entry has its own status, including null references. BLEND_BOUND
uses a signed nullable short. OFFSET_SURF preserves its logical flag, signed
offset and nullable scale. SWEPT_SURF retains the section, sweep vector and
scale; SPUN_SURF retains its profile, vectors, trim parameters and scale in
wire order. Unset trim parameters and vector coordinates remain unset.
HELIX_SU_FORM contains two vectors, handedness, a turns interval, pitch, gap
and tolerance; it has no common surface header. HELIX_CU_FORM (184) instead
contains three vectors (axis point, axis direction and a representative point),
handedness, a turns interval, pitch and tolerance. It has neither a common
curve header nor the surface form's gap field. Values and null markers remain
in wire order, without generating a helix. Independent checks in the default
limit run compare 70 curve-form nodes, 490 field groups and 980 scalar values
across 18 streams. Ordinary/delta synthetic tests also cover extended indices,
negative pitch, nullable tolerance and every truncated curve-form prefix.
No surface evaluation,
normalisation, trimming or blend reconstruction is performed.

Synthetic tests cover ordinary and delta records, fixed arrays, independent
pointer statuses, signed shorts, strict logical values, nullable coefficients,
embedded edits and atomic rejection of truncated helix records. A local
independent comparison checks all completed delta fields, values, statuses
and byte ranges. Private streams, reference documents and comparison outputs
remain outside this repository.

## Validation and reference material

Repository tests use synthetic bytes only. CAD files, extracted private
streams and comparison outputs remain outside the repository.

After adding HELIX_CU_FORM, 5,881 of 5,976 local streams reach their supported
closing marker with the default limits, compared with 5,864 before this
extension. All 18 earlier stops at type 184 are passed; one of those streams
then reaches a resource limit. The remaining 95 streams stop only at configured
resource bounds. With the explicit larger profile above, all 95 also reach
their closing marker, so all 5,976 local streams can be framed. The largest
completed runs contain 221,095 nodes or 221,759 combined delta entries. Previous
decoded coverage is preserved; no format/schema corruption errors or crashes
occur in the corpus run. Completion establishes wire framing, not delta
application, a reconstructed state or a valid B-Rep.

With sequential boundary checks, 796 of 811 local delta streams reach their
supported closing marker, compared with 785 before this extension. All eleven
previously ambiguous packets resolve as reference maps; their apparent PMARK
endpoints fall inside map entries and precede out-of-range node types.
Independent checks compare all 6,642 decoded packets, their terminal values,
mark/reference fields and exact byte ownership alongside 4,251,498 decoded
node values. With default limits, the remaining 15 delta streams stop at configured field
limits (13) or node limits (2). They complete with the larger explicit profile.
This measures wire decoding, not successful delta application. Regression
checks on all 5,976 local streams show no loss of previously decoded coverage.
Synthetic tests cover both historical terminal variants, following packets,
truncated optional suffixes, unsupported terminal values, stream trailers
and numeric zero in CLI JSON. Boundary tests cover both selection directions,
two impossible boundaries, unknown schemas, truncated boundary words and entry
limits. Reference-map terminals still require a nonzero
word; zero is not accepted by broadening that grammar.

Related format research includes
[parasolid-kit](https://github.com/monozukuri-ai/parasolid-kit) and
[cadmpeg](https://github.com/cadmpeg/cadmpeg). Their complete-document and
partial-reader boundaries are distinct. This library implements its own C++
reader under MIT and does not incorporate their Apache-licensed partial readers.
The Apache-2.0 Python project `sldprt2step` is an additional local comparison
reference for schema handling and future STEP mapping; its code and schema
tables are not included.

The MIT-licensed [ps-parser](https://github.com/khoanguyen-3fc/ps-parser) by
khoanguyen-3fc provides another schema and topology traversal reference.
Its source and schema tables are not incorporated into this implementation.
