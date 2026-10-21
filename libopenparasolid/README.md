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
settings, with command-line precedence. Delta application and B-Rep validation
remain separate work.

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
Topology traversal, delta application and STEP export remain separate work.
The inspector's JSON and command-line options are unchanged.

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
parameterisation. The separate chart API below requires supplied tangents;
automatic surface inversion and a surface-intersection solver remain pending.

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
continues to report the missing intersection solver explicitly. No CLI
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

Model construction provides definitions for a future surface-intersection solver.
`EvaluateCurve` still returns an explicit unsupported-format error for an
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
