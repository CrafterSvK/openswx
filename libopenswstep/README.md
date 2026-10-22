# libopenswstep

An MIT-licensed C++20 static library that creates ISO 10303-21 STEP AP214
(`AUTOMOTIVE_DESIGN`) documents from selected SolidWorks/Parasolid bodies.
It uses `libopenswx` for container inspection and extraction, and
`libopenparasolid` for sequential decoding, the model graph and complete body
geometry binding. It has no CAD-kernel dependency. The core builds with
`-fno-exceptions` under GCC/Clang and returns `openparasolid::Result<T>`.

## Supported geometry

| Source geometry | STEP representation |
| --- | --- |
| LINE, CIRCLE and ELLIPSE wire edges, including supported source trim wrappers | `EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION`, with shared `VERTEX_POINT` and `EDGE_CURVE` instances |
| Vertex-free full circle/ellipse wire rings | One exact conic with a synthetic seam vertex shared by both edge ends |
| Point-only acorn bodies | `GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION` containing a `GEOMETRIC_CURVE_SET` |
| Planar polygon faces bounded by LINE edges, including holes | `ADVANCED_FACE`, one `FACE_OUTER_BOUND`, interior `FACE_BOUND` instances, `EDGE_LOOP` and source-oriented edges |
| Single convex planar contour containing LINE/CIRCLE/ELLIPSE edges | Exact source `EDGE_CURVE` instances, including rounded profiles, circular/elliptic sectors, two-edge segments/lenses and split conics |
| Single-edge full CIRCLE/ELLIPSE planar disks | Exact conic `EDGE_CURVE` and one `FACE_OUTER_BOUND`; source seam vertex reused or one synthetic seam added |
| Circular planar faces with disjoint circular holes | Exact circular outer/inner bounds, including annuli and nonconcentric holes |
| Sheet bodies | `SHELL_BASED_SURFACE_MODEL` with one open/closed shell per shared-edge-connected component |
| Complete CYLINDER bands between two full coaxial circles | Exact `CYLINDRICAL_SURFACE` with two circular `FACE_BOUND` loops |
| Complete CONE bands between two full coaxial circles with positive radii | Exact `CONICAL_SURFACE`, including expanding/narrowing frustums and conical tubes |
| Regular ring TORUS bands between two full coaxial circles | Exact `TOROIDAL_SURFACE` with source-oriented circular bounds and analytic volume contributions |
| Solid bodies with one material region and one connected closed supported shell | `MANIFOLD_SOLID_BREP` inside `ADVANCED_BREP_SHAPE_REPRESENTATION` |

Coordinates remain in the source frame. **The caller must specify metres or
millimetres**; units are declared in the STEP context, without scaling source
coordinates. Binding tolerance is positive, finite, in those source length
units, and is also written as the STEP uncertainty. Natural conic parameters
remain radians. Source vertex/edge identities are reused within the exported
body. Conic intervals longer than one turn are rejected because a single
`EDGE_CURVE` cannot preserve their turn count.

Body binding validates recorded incidence, directed intervals, finite curve
membership samples, loop closure and shared-edge endpoints. The exporter also
checks polygon area/orientation, bounded segment comparisons and containment,
edge valence and connected vertex links. Sheet orientations propagate across
shared edges; nonorientable components fail explicitly. For solids, the material
shell's face sides determine outward orientation, paired edges must agree, and
signed enclosed volume must be positive. The source's void exterior region is
not emitted as an additional solid.

Each planar face has exactly one outer loop whose winding agrees with its
effective source normal. Hole loops have opposite winding and must lie strictly
inside that outer loop. Source loop order is preserved; the outer loop need not
come first. Crossings, touching or tolerance-ambiguous edges, nested holes and
disconnected islands are rejected. Through holes in closed planar solids retain
their interior walls and subtract from enclosed volume.

The mixed line/conic-arc subset supports one convex outer contour with at
least two edges and distinct non-null source vertices. It checks nonnegative
source-oriented arc curvature, convex corner turns and one full revolution of
the boundary tangent. Accepted negative corner noise is bounded by both an
angular limit and the source length tolerance divided across a conservative
perimeter bound and the corners. Every corner consumes one comparison-budget
check. Conic support planes are checked over the entire circle or ellipse;
straight endpoints retain plane membership checks. Area is integrated
analytically over each source line/arc.
No chords, fitted curves or triangulation replace the source boundary.

Elliptic tangent turning is computed analytically, rather than equating the
ellipse parameter to its tangent angle. Arcs exceeding half a revolution,
split ellipses, reversed traversal and mixed circle/ellipse lenses are supported.
Arc length is bounded by the smaller/larger radius times the absolute parameter
span; the lower bound must exceed the source tolerance, while the upper bound is used
for conservative area and corner-noise checks. Extremely eccentric or tiny arcs
whose lower bound is unresolved are rejected explicitly.

This additional subset excludes holes and nonconvex curved contours. Existing
all-line polygon faces retain their concave-polygon and polygon-hole support.
Existing full-circle hole faces retain their separate analytic containment
checks.

The separate single-edge conic-loop subset requires a complete circle or
ellipse per loop. The exporter checks the maximum plane residual over the entire analytic curve, not
just the binding samples, and validates winding against the effective face
normal. Circular outer/inner loops may appear in any source order. Every hole
must lie inside the outer circle, and holes must be mutually disjoint; each
separation must exceed two source tolerances. Elliptical holes remain
unsupported.

Complete cylindrical bands use two circular cut loops at distinct axial
levels. Their radii, axis alignment, centre positions and opposite winding are
checked analytically, including whole-circle axial variation beyond the binding
samples. Cylindrical bands have two `FACE_BOUND` instances without a distinguished
outer loop. Caps and mantles share their source circular edges and seam vertices;
vertex-free edges receive one shared synthetic seam vertex per edge. Source
closed-edge endpoints remain distinct in the manifold vertex-link check even
when they refer to the same seam vertex.

Solid cylinders, tubes and other closed supported shells use the existing
material-side and manifold checks. Signed volume includes analytic divergence
integrals for planar conic regions and complete cylinder bands. Inner mantles
and cap holes subtract from volume. This introduces no tessellation. Importers
may introduce periodic seam edges on cylindrical faces and split circular
boundaries when their seams have different phases. The exported edge statistics
count source boundary edges rather than the additional importer edges.

Complete conical bands use the same coaxial cut, winding and budget checks as
cylinders, with the linear profile `r(z) = r_ref + z * sin(a) / cos(a)` used by
libopenparasolid. Both cut radii must exceed the source tolerance; linearity
keeps the band on the positive-radius branch between them. Signed volume
includes the analytic conical flux, including inward conical tube walls.
Narrowing source cones use a positive STEP semi-angle and reversed exported
axis. The reference origin may be at the apex, provided both boundary circles
are away from it. Apex boundary loops and partial conical sectors are unsupported.

Regular ring torus bands require positive minor radius and a major radius
larger than the minor radius by more than the source tolerance. Two complete
coaxial circular cuts are checked against the meridian profile, including
whole-circle tilt and displacement bounds. Their separation must exceed two
source tolerances. The opposite cut windings and the effective face normal
select one directed meridian interval; no shortest-arc or axial-order choice is
made. Complementary bands, equal-height cuts with different radii, reversed
axes and crossing the meridian parameter seam are supported. Both bounds use
`FACE_BOUND`. A complete uncut torus and horn/apple/lemon variants remain
unsupported.

The solid volume check includes the exact divergence integral over the selected
toroidal band. Closed supported mixtures of planar caps, cylindrical/conical
mantles and toroidal bands retain shared source boundaries. As with the other
surface types, the checks do not certify intersections between separate faces.

These checks do not certify all geometric intersections, disjointness of
separate components, or full CAD-kernel validity. No triangulation or geometry
approximation is written.

NURBS, nonconvex curved face boundaries, mixed curved/polygon hole loops,
elliptical holes, partial cylindrical/conical bands, partial torus azimuths, uncut tori,
horn/apple/lemon tori, apex loops, explicit seam fins, other
non-planar surfaces, tolerant fin-specific curves/pcurves, isolated face loops,
compounds/assemblies, general
mixed-dimensional bodies, multiple material regions and solid void shells are
not supported. Unsupported selected geometry fails the entire export. No partial document is returned and no part of the selected body is
silently discarded. Unrelated source geometry is not exported.

Ghost payloads and unapplied delta streams are rejected by the source adapters.
An explicitly materialized ordinary logical snapshot can be supplied through
`ExportBody`. Assembly component placement and active configuration/payload
selection are not inferred.

The cross-library [open issues and work list](../libopenparasolid/OPEN_ISSUES.md)
distinguishes reader coverage from the narrower STEP export subset.

## API

Link the CMake target `openswstep`; both source libraries are public dependencies.
Public headers are in `include/openswstep` and use Doxygen contracts.

```cpp
#include <openswstep/export.h>

openswstep::ExportOptions options;
options.length_unit = openswstep::LengthUnit::kMetre;
options.product_name = "Example part";
options.tolerance = 1e-7;

// payload_index is the explicit zero-based index in
// openswx::ExtractParasolidStreams(inspection).payloads.
// body_id is the explicit decoded stream-local BODY index in that payload.
auto step = openswstep::ExportSolidWorksFile(
    input_path, payload_index, openparasolid::NodeId{body_id}, options);
if (!step.ok()) {
  // Handle step.error().code, offset and message.
  return;
}
auto written = openswstep::WriteStepFile(step.value(), output_path);
if (!written.ok()) {
  // Handle the output I/O failure.
  return;
}
```

`ExportPayload` takes an already selected `openswx::ParasolidPayload`.
`ExportParasolid` takes extracted binary bytes and an optional matching
`SchemaCatalog`. `ExportBody` accepts an existing model and BODY ID. All inputs
are borrowed during the call; returned STEP text and statistics own their data.
Independent inputs can be exported concurrently.

`fin_intervals` and `wire_intervals` provide partial explicit interval overrides;
unspecified entities retain the source library's automatic inference. Overrides
do not enable unsupported output geometry or bypass validation.

Positive entity, output byte and polygon comparison budgets bound serialization
and polygon checks. The shared polygon budget counts nonadjacent segments within
each loop, all segment pairs between loops, and containment edge tests across
the entire selected body. Convex line/arc contours consume one check per corner. Circular containment and hole-pair comparisons consume
one check each from the same budget; each cylindrical/conical/toroidal cut pair consumes one
check. Parser, model and body-binding budgets remain independently configurable
in `ExportOptions`. The file adapter checks source file size before container
inspection; libopenswx's per-stream decompression/extraction bounds
also apply. Any extraction issue is reported as an error.

Serialization is deterministic: numeric output is locale-independent, reals
retain round-trip double precision, IDs follow source traversal order, names
are limited to printable ASCII with STEP escaping, and the file header uses a
fixed epoch timestamp rather than reading the wall clock. Product names are at
most 1024 bytes. The timestamp is export metadata, not a source creation date.

`WriteStepFile` replaces its destination only after serialization has succeeded.
It checks open/write/close errors. File writing is not transactional: an I/O
failure may leave a partial target. Keep the previous file separately if needed.
There is no export CLI.

## Build and test

The repository build includes the library and tests automatically. A standalone
build brings in its two sibling libraries and their existing dependencies
(Zlib and pugixml):

```sh
cmake -S libopenswstep -B build-step -DBUILD_TESTING=ON
cmake --build build-step --target libopenswstep_tests
ctest --test-dir build-step -R 'OpenSwStep|StepFileTest' --output-on-failure
```

GoogleTest is required only with tests enabled. No additional runtime dependency
is introduced. The tests construct synthetic topology and a compressed modern
SolidWorks container, exercise the full extraction/decoding/export/file path,
and check sharing, ring seams, senses, polygon holes, through holes, cylindrical
bands, conical frustums, toroidal intervals, convex circular/elliptic arc
contours, invalid topology, budget boundaries and preserved source diagnostics.

An optional independent Open CASCADE round-trip test can be enabled with a
Python interpreter providing the `OCP` bindings:

```sh
cmake -S . -B build \
  -DOPENSWSTEP_OCCT_PYTHON=/path/to/python-with-OCP
cmake --build build --target openswstep_roundtrip_fixtures
ctest --test-dir build -R openswstep_occt_roundtrip --output-on-failure
```

It imports generated synthetic STEP documents, runs `BRepCheck_Analyzer`, checks
unique vertex/edge/face/solid counts and compares volumes, areas and curve
lengths, including hole areas and analytic cylinder/frustum/tube/toroidal
volumes. Different seam phases exercise importer-created periodic seams and ring
splits. OCCT is exclusively a test dependency and is never linked into the export library.
Synthetic files live in a temporary directory and are removed.
Private CAD files and generated private exports must stay outside the repository.

The private regression queries 57 explicitly selected bodies in 39
extracted streams: 42 export successfully, including twelve face-bearing bodies
with 60 faces; 30 successes contain only wire/point geometry. All 42 generated
documents pass OCCT import and B-rep validity checks. The other 15 return
unsupported-format errors. Elliptic-arc support preserves these 42 successes;
its additional geometry cases are covered by synthetic half-ellipses, lenses,
long arcs, split ellipses and rounded profiles. The remaining private bodies
require additional geometry features before their complete export can succeed. These are raw body selections, including archival
partition snapshots, rather than automatic active-configuration conversions.

Run `doxygen Doxyfile` from this directory to generate public API HTML at
`../build/openswstep-docs/html/index.html`; documentation warnings fail generation.

## Format references

- Siemens *Parasolid XT Format Reference*, July 2022: sections 4.3.3–4.3.10
  (body through vertex), 5.2.1.1–5.2.1.3 (line/circle/ellipse),
  5.2.2.1–5.2.2.3 (plane/cylinder/cone), 5.2.2.5 (torus), 5.2.8 (curve/surface senses) and 5.3.4.1 (attaching geometry to topology).
  The local reading copy is in the workspace's `xt-doc`; it is not bundled here.
- [STEP toroidal-surface attributes](https://www.steptools.com/docs/stp_aim/html/t_toroidal_surface.html).
- [STEP conical-surface attributes](https://www.steptools.com/docs/stp_aim/html/t_conical_surface.html).
- [STEP advanced-face restrictions](https://www.steptools.com/docs/stp_aim/html/t_advanced_face.html).
- [STEP edge-based wireframe representation](https://www.steptools.com/docs/stp_aim/html/t_edge_based_wireframe_shape_representation.html)
  and [permitted wireframe edge curves](https://www.steptools.com/docs/stp_aim/html/t_valid_wireframe_edge_curve.html).
- [OCCT STEP translation model](https://github.com/Open-Cascade-SAS/OCCT/blob/master/dox/user_guides/step/step.md).

The implementation is original MIT code; no third-party parser/exporter
implementation is incorporated.
