# Model builder

`ModelBuilder` adds an owned reference graph and typed geometry data above
`SchemaParser`. It is MIT code, uses only the C++ standard library, and performs
no file I/O, logging or environment access. New core code uses explicit
`Result` errors; it does not throw exceptions.

## API and ownership

```cpp
#include <openparasolid/model.h>

// parsed is a successful SchemaParser result at the application boundary.
auto built = openparasolid::ModelBuilder::Build(parsed.value());
if (!built.ok()) {
  // Handle built.error(); incomplete streams and unapplied deltas are rejected.
  return;
}
const auto& model = built.value();
const auto* node = model.FindNode(openparasolid::NodeId{42});
if (node && node->geometry) {
  const auto& geometry = model.geometry()[*node->geometry];
  // Inspect the typed alternative in geometry.payload.
}
```

The input must be a complete partition or transmit report. Delta streams must
first be applied to a base state; repeated delta indices are not a model index.
The builder checks node identities, schema/field shapes, scalar codecs, byte
ranges and resource bounds before linking data. Duplicate IDs and malformed
reports return `Result::Err`. No partially built `Model` is returned on a fatal
input or resource error.

The returned model owns its node records, field names, references and geometry
arrays. It can outlive the parse report and can be copied or moved. Relationships
use `NodeId` and vector positions rather than owning or cached raw pointers.
`FindNode` returns a non-owning pointer into that particular model; assignment,
moving or destroying the model ends that pointer's validity.

`ModelNode` retains identity, source type/offset, reference fields and an optional
geometry index. FACE and FIN nodes additionally retain a scalar character
`topology_sense`, including unknown bytes, or absence when the field is
unavailable/incompatible. It is not a copy of every raw field. Keep
`SchemaParseReport` when attribute values, procedural-form parameters or other
raw fields are also needed. `TraverseLoopBoundary`, `TraverseFaceBoundary` and
`TraverseShellBoundary` in `openparasolid/topology.h` add bounded, owned
boundary views above this graph; see [loop traversal](README.md#ordered-loop-boundaries),
[face traversal](README.md#face-boundaries) and [shell traversal](README.md#shell-boundaries).
They validate selected connectivity and list ownership without changing the
model or claiming complete body/geometric validation or a proof of manifoldness.

## Reference graph

Every transmitted pointer retains its field name, array element, target ID,
declared pointer class and resolution status:

| Status | Meaning |
|---|---|
| `null` | The wire reference is null |
| `resolved` | The target exists and satisfies a reviewed class rule |
| `unverified_class` | The target exists; this class/target combination is unreviewed |
| `missing_target` | The target is absent from this stream |
| `wrong_class` | The target exists and contradicts a reviewed class rule |

Resolved positions refer to `Model::nodes()`. A wrong-class reference keeps its
actual target position for inspection; its status remains an error. Null and
missing targets have no position. Forward references, sharing and cycles are
retained without recursive traversal. IDs are scoped to this input stream;
external or omitted targets are not invented.

Concrete classes below 1000 require the corresponding node type. Reviewed union
rules cover geometry, curves, surfaces, their documented owners, attribute/
feature links, field values, shell/body, field names and analytic-form links.
Pointer-list `void*` entries accept any existing node. List-block, list-owner
and attribute-owner checks also have explicitly reviewed positive subsets;
other combinations remain unverified. Future target types are not rejected
merely because a union's membership has not yet been reviewed for them.

`references_verified()` requires no missing, wrong-class or unverified links.
It does not establish reciprocal links, closed fin rings, shell closure,
manifoldness or a valid B-Rep. Model issues identify a node, source offset and
field where applicable. Missing targets are findings within this stream, not
proof that the enclosing SolidWorks file is malformed.

## Typed geometry

The first typed alternatives are:

- Point, line, circle and ellipse.
- Plane, cylinder, cone, sphere and torus.
- NURBS curve (`B_CURVE` + `NURBS_CURVE`) and surface (`B_SURFACE` +
  `NURBS_SURFACE`), including their referenced control and knot arrays.
- Surface curve (`SP_CURVE`), retaining surface and 2D parameter-curve IDs.
- Trimmed curve (`TRIMMED_CURVE`), retaining its basis ID, source endpoints and
  original basis parameters.
- Spun surface (`SPUN_SURF`), retaining its profile ID, axis/base and nullable
  parameter bounds and auxiliary metadata.
- Swept surface (`SWEPT_SURF`), retaining its section ID, sweep direction and
  nullable internal scale.
- Intersection curve (`INTERSECTION`), retaining its surface IDs, chart,
  limits and optional additional-data ID.

Fields are located by resolved schema name and codec, not fixed field offsets.
Inserted fields and reordered definitions therefore do not shift geometry roles.
Sense bytes remain unchanged, including `?`. Coordinates and units remain as
stored. Finite coefficients and basic radius/direction/frame constraints are
checked, without normalizing axes or changing cone conventions. A torus's
negative major radius is preserved for the documented lemon form; its limiting
sphere and invalid lemon cases are rejected as typed torus objects.

NURBS retain the original flat control array, dimension, rational flag, distinct
knots, multiplicities, degrees, closure/periodicity flags and form codes. Rational
coordinates remain homogeneous; weights are not divided out. Two- or
three-dimensional curve controls and three-dimensional surface controls are
supported, with an additional weight coordinate when rational. Surface controls
retain their transmitted order rather than being transposed.

Checks include descriptor/array lengths, finite control and knot values,
positive rational weights, strictly increasing distinct knots, multiplicity
bounds, degree/control-count consistency and the expanded-knot identity
`sum(multiplicities) == control_count + degree + 1` for each axis. Periodic axes
must also be marked closed. Periodic continuity and geometric closure are not
numerically evaluated. Oversized or mismatching arrays are not silently sliced,
padded or resampled.

Semantic problems retain the graph and other successfully built geometry while
adding `invalid_geometry` issues. No partial typed object is published. Known
geometry candidates without a typed adapter (other procedural surfaces,
foreign geometry, mesh and polyline) add
`unsupported_geometry` issues. Auxiliary raw nodes remain in the graph.
`typed_geometry_complete()` covers the recognized geometry candidates; it is
not a statement about arbitrary extension-node semantics or geometric validity.

`SurfaceCurveGeometry` retains non-null surface and parameter-curve IDs without
resolving geometry during construction. This supports forward references and
keeps missing/wrong-class findings in the reference graph. The model overload
of `EvaluateCurve` checks 2D B-curve compatibility and evaluator availability
and applies the natural chain rule. Typed-definition completeness is distinct
from composition validity; auxiliary original/tolerance fields are not used.

`TrimmedCurveGeometry` requires a non-null basis, finite endpoint vectors,
distinct finite parameters and the documented positive trim sense. Definition
construction does not resolve the basis. Model-based evaluation checks endpoint
ordering against the known basis sense and every nested numeric trim interval.
Endpoint point/parameter agreement, periodic span legality and tolerant vertex
correspondence are not certified. The model evaluator uses the original basis
parameter and natural derivative, rather than introducing a new parameter scale.

`SpunSurfaceGeometry` construction retains nullable start/end vectors,
parameters, x-axis metadata and internal scale. Partially unset vectors,
nonfinite data, reversed bounds and null profile IDs are invalid definitions.
Its model-based surface evaluator resolves only analytic or 3D NURBS profiles,
checks the unit spin axis, and preserves original parameters and natural
partials. Auxiliary metadata does not transform or scale the generator.
Forward references remain IDs rather than cached pointers. Profile validity
and evaluability are stronger checks than typed-definition completeness.

`SweptSurfaceGeometry` requires a non-null section, finite nonzero sweep vector
and nullable finite internal scale. Section references remain IDs, supporting
forward references. Model-based evaluation shares the spun surface's analytic
or 3D NURBS generator validation, requires a unit sweep direction and returns
the translated section with natural partials. Internal scale remains unused
source metadata. Definition completeness does not certify section evaluability
or surface regularity.

`IntersectionCurveGeometry` resolves CHART/LIMIT auxiliaries from the complete
raw index and copies only transmitted positions. Nullable error estimates and
schema-dependent `term_use`/`intersection_data` presence are retained. Missing
or wrong auxiliary records, inconsistent counts and nonfinite values produce
invalid-definition issues; unsupported limit kinds remain explicit. Surface
references stay IDs with graph diagnostics. Typed-definition completeness
certifies neither chart geometry nor evaluator availability. In particular,
intersection curves still require a solver and do not evaluate as polylines.

No curve/surface evaluation, derivatives, trimming, tessellation, topology
reconstruction, delta application or STEP export is performed here.

## Bounds

`ModelOptions` bounds nodes, decoded field groups, pointer occurrences, values
and retained diagnostics. The value bound applies independently to input
scalars and copied geometry arrays, so repeatedly shared spline data cannot
bypass the memory budget by being copied into many geometry objects.

The diagnostic buffer is bounded separately (1000 entries by default). Total
issue counts remain accurate; `issues_truncated()` explicitly identifies omitted
details. This cap does not mark an unverified model as verified.

## Inspector and configuration

```bash
parasolid_inspect --geometry part.x_b
parasolid_inspect --geometry --summary part.x_b
OPENPARASOLID_GEOMETRY=true parasolid_inspect --summary part.x_b
```

Geometry mode implies sequential decoding and writes a schema summary plus a
`model` object. Full output contains graph nodes, typed geometry and bounded
issues. Summary output keeps counts and flags while omitting those arrays.
The existing top-level `complete` flag continues to describe wire decoding;
model flags describe their narrower validation scope. An unavailable model
is null with a `model_error` while the parse summary remains available.

Exit 0 means the requested supported data checks completed without model issues.
Exit 2 retains a limited report for incomplete decoding, unsupported model
input, unverified references or incomplete typed geometry. Exit 1 covers input,
configuration and initial parser errors. Exit 0 is not B-Rep certification.

Configuration is parsed centrally with precedence **defaults, environment,
command line**. There is no inspector configuration file; a schema catalog is
format data, not a configuration file. Every meaningful option has a matching
environment variable:

| Option | Environment |
|---|---|
| `--geometry` | `OPENPARASOLID_GEOMETRY` |
| `--nodes` | `OPENPARASOLID_NODES` |
| `--summary` | `OPENPARASOLID_SUMMARY` |
| `--schema-catalog` | `OPENPARASOLID_SCHEMA_CATALOG` |
| `--max-records` | `OPENPARASOLID_MAX_RECORDS` |
| `--max-fields` | `OPENPARASOLID_MAX_FIELDS` |
| `--max-values` | `OPENPARASOLID_MAX_VALUES` |
| `--max-delta-entries` | `OPENPARASOLID_MAX_DELTA_ENTRIES` |

Booleans accept `0`, `1`, `false` or `true`. For example, `--geometry=false`
overrides the geometry environment setting; `--nodes=false` can separately
override the node setting. A catalog still implies node decoding. Invalid final
settings are explicit errors. All existing numeric defaults remain unchanged.
The last three limit settings require sequential mode, including when supplied
through the environment. No new dependencies or alternative build system are
introduced; CMake and the existing vcpkg-compatible dependencies are retained.

CLI model bounds follow the parser's node/field/value options. The reference
bound follows `max_values`, and the diagnostic bound remains 1000. Both sets
of effective limits appear in the JSON output. The larger explicit corpus
profile documented in README also works with `--geometry`.

## Validation

The separate `EvaluateCurve` API in `openparasolid/evaluate.h` evaluates line,
circle, ellipse and NURBS curve payloads in their natural parameter direction.
Its strict analytic unit-frame, NURBS knot-domain and finite-result checks do
not change model construction or the meaning of `typed_geometry_complete`.
See README for its supported scope.

`EvaluateSurface` evaluates the five analytic surface payloads and NURBS
surfaces, returning natural first partial derivatives. Its frame, cone-angle
and parameter-domain checks are independent of initial model construction. Singular boundary points
remain evaluable, but no face trimming, sense reversal or placement transform
is applied. NURBS surfaces use the original v-fastest control order and
independent knot domains; model flags do not certify surface regularity.

Synthetic tests cover ownership, copying/moving, forward and cyclic references,
null/missing/wrong/unreviewed classes, malformed reports, reordered fields,
finite coefficients, lemon tori, shared spline arrays, rational coordinates,
array/count mismatches, resource bounds and diagnostic truncation. CLI and
configuration tests cover output, summaries, exit status, rejected deltas,
partial input, environment precedence and invalid settings.

The earlier corpus run, before the surface-curve adapter, built 5084 non-delta
models with 5,036,695 nodes and
715,451 typed geometry objects. All 892 delta streams are explicitly rejected
for model construction. There are 146,718 unsupported geometry candidates and
100 candidates that fail the initial geometry-data checks. The graph reports
28,707 missing targets, 3671 unverified class occurrences and no wrong-class
occurrences. These are separate findings from successful wire decoding.
Independent checks on 25 selected streams cover all eleven typed alternatives,
1637 source nodes, 8793 reference occurrences and 168 geometry objects, including
NURBS arrays and signed tori. Private CAD data and comparison artifacts remain
outside the repository.
