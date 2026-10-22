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
| Single-edge full CIRCLE/ELLIPSE planar sheet disks | Exact conic `EDGE_CURVE` and one `FACE_OUTER_BOUND`; source seam vertex reused or one synthetic seam added |
| Circular sheet faces with disjoint circular holes | Exact circular outer/inner bounds, including annuli and nonconcentric holes |
| Sheet bodies | `SHELL_BASED_SURFACE_MODEL` with one open/closed shell per shared-edge-connected component |
| Solid bodies with one material region and one connected closed planar shell | `MANIFOLD_SOLID_BREP` inside `ADVANCED_BREP_SHAPE_REPRESENTATION` |

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

Conic sheet loops must each contain one complete circle or ellipse edge. The
exporter checks the maximum plane residual over the entire analytic curve, not
just the binding samples, and validates winding against the effective face
normal. Circular outer/inner loops may appear in any source order. Every hole
must lie inside the outer circle, and holes must be mutually disjoint; each
separation must exceed two source tolerances. Ellipses currently support a
single disk boundary only.

These checks do not certify all geometric intersections, disjointness of
separate components, or full CAD-kernel validity. No triangulation or geometry
approximation is written.

NURBS, mixed line/conic face boundaries, multi-edge conic loops, elliptical
holes, conic faces in solids, non-planar surfaces, tolerant fin-specific
curves/pcurves, isolated face loops, compounds/assemblies, general
mixed-dimensional bodies, multiple material regions and solid void shells are
not supported. Unsupported selected geometry fails the entire
export. No partial document is returned and no part of the selected body is
silently discarded. Unrelated source geometry is not exported.

Ghost payloads and unapplied delta streams are rejected by the source adapters.
An explicitly materialized ordinary logical snapshot can be supplied through
`ExportBody`. Assembly component placement and active configuration/payload
selection are not inferred.

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
the entire selected body. Circular containment and hole-pair comparisons consume
one check each from the same budget. Parser, model and body-binding budgets
remain independently configurable in `ExportOptions`. The file adapter checks source file size before
container inspection; libopenswx's per-stream decompression/extraction bounds
also apply. Any extraction issue is reported as an error.

Serialization is deterministic: numeric output is locale-independent, reals
retain round-trip double precision, IDs follow source traversal order, names
are limited to printable ASCII with STEP escaping, and the file header uses a
fixed epoch timestamp rather than reading the wall clock. Product names are at
most 1024 bytes. The timestamp is export metadata, not a source creation date.

`WriteStepFile` replaces its destination only after serialization has succeeded.
It checks open/write/close errors. File writing is not transactional: an I/O
failure may leave a partial target. Keep the previous file separately if needed.
There is no export CLI in this iteration.

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
and check sharing, ring seams, senses, polygon holes, through holes, invalid
topology, budget boundaries and
preserved source diagnostics.

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
lengths, including hole areas and through-hole volumes. OCCT is exclusively a test dependency and is never linked into the
export library. Synthetic files live in a temporary directory and are removed.
Private CAD files and generated private exports must stay outside the repository.

The private regression queries 57 explicitly selected bodies in 39
extracted streams: 39 export successfully, including nine face-bearing bodies
with 48 faces; 30 successes contain only wire/point geometry. All 39 generated
documents pass OCCT import and B-rep validity checks. The other 18 return
unsupported-format errors. Conic-sheet support adds four successful exports
and preserves all 35 previous successes; the remaining private samples still
require other geometry features. These are raw body selections, including
archival partition snapshots, rather than automatic active-configuration
conversions.

Run `doxygen Doxyfile` from this directory to generate public API HTML at
`../build/openswstep-docs/html/index.html`; documentation warnings fail generation.

## Format references

- Siemens *Parasolid XT Format Reference*, July 2022: sections 4.3.3–4.3.10
  (body through vertex), 5.2.1.1–5.2.1.3 (line/circle/ellipse), 5.2.2.1 (plane),
  5.2.8 (curve/surface senses) and 5.3.4.1 (attaching geometry to topology).
  The local reading copy is in the workspace's `xt-doc`; it is not bundled here.
- [STEP advanced-face restrictions](https://www.steptools.com/docs/stp_aim/html/t_advanced_face.html).
- [STEP edge-based wireframe representation](https://www.steptools.com/docs/stp_aim/html/t_edge_based_wireframe_shape_representation.html)
  and [permitted wireframe edge curves](https://www.steptools.com/docs/stp_aim/html/t_valid_wireframe_edge_curve.html).
- [OCCT STEP translation model](https://github.com/Open-Cascade-SAS/OCCT/blob/master/dox/user_guides/step/step.md).

The implementation is original MIT code; no third-party parser/exporter
implementation is incorporated.
