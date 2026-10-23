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
| Open polynomial/rational B_CURVE wire edges, including unclamped source axes and certified partial intervals | Exact `B_SPLINE_CURVE_WITH_KNOTS` and, for positive homogeneous weights, `RATIONAL_B_SPLINE_CURVE` |
| Vertex-free full circle/ellipse wire rings | One exact conic with a synthetic seam vertex shared by both edge ends |
| Point-only acorn bodies | `GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION` containing a `GEOMETRIC_CURVE_SET` |
| Planar polygon faces bounded by LINE edges, with polygonal and/or full circular holes | `ADVANCED_FACE`, one `FACE_OUTER_BOUND`, interior `FACE_BOUND` instances, `EDGE_LOOP` and source-oriented edges |
| Single convex planar contour containing LINE/CIRCLE/ELLIPSE edges | Exact source `EDGE_CURVE` instances, including rounded profiles, circular/elliptic sectors, two-edge segments/lenses and split conics |
| Single convex planar contour additionally containing polynomial quadratic B_CURVE edges, including smooth multi-span bounds | Exact `B_SPLINE_CURVE_WITH_KNOTS` edge bounds, analytic polynomial area and whole-curve plane/curvature checks |
| Single hole-free concave planar LINE/CIRCLE contour with certified minor arcs | Exact source curves and analytic area; chord, departure-cone and sagitta checks certify simplicity |
| Convex planar LINE/CIRCLE contours with polygonal and/or full circular holes | Exact curved outer bound, source hole loops, analytic containment and net area |
| Polygonal or complete circular outer bounds with convex multi-edge LINE/CIRCLE/ELLIPSE holes | Exact source hole curves and analytic net area; conservative analytic envelopes certify containment |
| Planar annular sectors with two concentric circular arcs and two radial lines | Exact single-loop `ADVANCED_FACE`, including nonconvex major sectors and analytic source-curve area |
| Single-edge full CIRCLE/ELLIPSE planar disks | Exact conic `EDGE_CURVE` and one `FACE_OUTER_BOUND`; source seam vertex reused or one synthetic seam added |
| Circular planar faces with polygonal and/or circular holes | Exact circular outer bound, source polygon edges and circular inner bounds, including annuli and mixed holes |
| Sheet bodies | `SHELL_BASED_SURFACE_MODEL` with one open/closed shell per shared-edge-connected component |
| Complete CYLINDER bands between two full coaxial circles | Exact `CYLINDRICAL_SURFACE` with two circular `FACE_BOUND` loops |
| Complete CYLINDER bands between separated monotone CIRCLE/ELLIPSE profiles with isolated axial steps | Exact source bounds, oblique cuts and stepped rims; analytic variable-height volume |
| Partial CYLINDER patches with two circular arcs and two axial generators | Exact `CYLINDRICAL_SURFACE` with one four-edge outer bound, including major arcs and parameter-seam crossings |
| Complete CONE bands between two full coaxial circles with positive radii | Exact `CONICAL_SURFACE`, including expanding/narrowing frustums and conical tubes |
| Partial CONE patches with two circular arcs and two straight cone generators | Exact `CONICAL_SURFACE` with one four-edge outer bound, including expanding/narrowing sectors |
| SPHERE caps with one full latitude circle or bands with two coaxial circles | Exact `SPHERICAL_SURFACE`, source circular bounds and analytic volume contributions |
| Regular ring TORUS bands between two full coaxial circles, including split circular cuts | Exact `TOROIDAL_SURFACE` with source-oriented circular bounds and analytic volume contributions |
| SPUN_SURF with an unbounded circular generator in an axial meridian plane of a regular ring torus | Equivalent analytic `TOROIDAL_SURFACE`, preserving source normal orientation and boundary edges |
| Certified paired planar/rotational tolerant circular edges | Shared exact circular 3D master with `SURFACE_CURVE`, both source `PCURVE` representations and declared edge uncertainty |
| Solid bodies with one material region and one connected closed supported shell | `MANIFOLD_SOLID_BREP` inside `ADVANCED_BREP_SHAPE_REPRESENTATION` |

Coordinates remain in the source frame. **The caller must specify metres or
millimetres**; units are declared in the STEP context, without scaling source
coordinates. Binding tolerance is positive, finite, in those source length
units, and is the base STEP uncertainty. Certified tolerant edges may use a
larger explicitly transmitted `EDGE.tolerance`; the maximum actually used is
written to STEP and returned as `ExportStatistics::length_uncertainty`.
`use_source_edge_tolerances = false` requires the caller tolerance alone.
Neither mode widens the reader's binding or ordinary boundary checks.
Natural conic parameters remain radians. Source vertex/edge identities are reused within the exported
body. Conic intervals longer than one turn are rejected because a single
`EDGE_CURVE` cannot preserve their turn count.

Open B-spline wire edges support complete or partial natural domains, in either
direction, including nested source trim wrappers, explicit caller intervals and
nonuniform knots. Complete-domain curves retain degree, distinct knots,
multiplicities and control order unchanged when their source axis is clamped.
Rational controls contain weighted coordinates: each pole is divided by its
positive weight, and the weights are serialized separately in the STEP complex
entity. Polynomial controls are written directly. Three spatial coordinates are
required; two-dimensional parameter curves are not treated as 3D wire curves.
The source classification bytes are not assumed to be equivalent STEP enums.

Partial intervals are restricted by homogeneous knot insertion at both selected
ends. Interior cut knots reach degree multiplicity; cropping the control net and
adding one endpoint knot clamps each new end. Unselected pieces are removed,
while the degree, source parameter values and retained interior knots stay in
the source frame. Positive rational weights are interpolated along with weighted
coordinates. This is an algebraic restriction of the same B-spline, subject to
double precision; it does not fit a replacement curve. The resulting endpoints
must still meet the named source vertices within the caller's length tolerance.
Distinct selected edges on one basis receive their own restricted curves.
The reader's trim-sense checks still apply: a descending outer trim on a
positively sensed inner trim is contradictory and remains rejected.

Unclamped open source axes use the same restriction, including complete natural
domains. For the expanded source knot vector `U`, degree `p` and control count
`N`, that domain is `[U[p], U[N]]`; it is not the span between the smallest and
largest transmitted knots. Knot insertion followed by cropping removes the
inactive exterior data and clamps the selected endpoints. Nonuniform and
asymmetric exterior layouts retain their active source polynomial pieces and
parameter values. Exterior knots never authorize extrapolation beyond the
natural domain. Axes already having endpoint multiplicity `p` can lose their
inactive end knots without inserting or changing controls.

The resulting end knots are clamped with degree-plus-one multiplicity; the source
spline must be open and nonperiodic. Every retained interior Cartesian control point must lie
strictly between the two supporting planes normal to the endpoint chord, with
clearance for source tolerance and coordinate roundoff. The positive-weight
convex hull property then excludes either endpoint everywhere inside the curve,
so the STEP edge limits retain the complete selected source interval. This
conservative certificate can reject valid curves; it does not certify global
wire simplicity. Every retained interior control consumes one shared comparison
check. Restriction of partial or unclamped domains additionally charges one check per original control
for the source copy and per output control at each knot insertion. Temporary
control storage retains the reader evaluator's one-million-scalar limit.
Closed or periodic spline wires remain unsupported. Planar faces additionally
support the polynomial quadratic boundary subset described below. No fitting,
resampling or tessellation replaces the source curve.

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

Polygon faces may additionally contain complete one-edge circular holes, in
any source-loop order and with named or vertex-free seams. These circles must
be oppositely wound, have centers inside the polygon material and remain more
than two source tolerances from every polygon segment and other circular hole.
Analytic point-to-segment distances test the entire disk against concave outer
boundaries and polygon holes; containment tests also exclude circles outside
the outer polygon or nested inside a polygon hole. A circle enclosing a polygon
hole is rejected. Each source circle remains an exact shared `EDGE_CURVE`.
The net area subtracts both polygonal and circular holes and contributes
analytically to the solid-volume check. Supported closed shells therefore also
include polygonal prisms with complete cylindrical through bores.

Conversely, a complete one-edge circle may bound a face with polygonal and/or
circular holes. Every polygon vertex must lie strictly inside the outer disk,
with more than two source tolerances of radial clearance. The disk's convexity
then bounds all straight segments and the enclosed polygon, even for a concave
hole profile. Polygon self/inter-loop intersections, nesting and disk-to-polygon
clearance use the same checks as the polygon-outer subset. The outward-wound
source circle determines the outer loop regardless of list order; the original
curves, vertices and loops remain unchanged. Net area is the outer disk area
minus all polygon and circular hole areas.

Planar annular sectors support one four-edge loop alternating between two
concentric `CIRCLE` arcs and two radial `LINE` edges. The outer arc winds with
the effective face normal and the inner arc against it; both select the same
azimuth span, strictly shorter than one turn. Circle support planes, center
agreement, resolved radial width, radial edge direction and separation of the
two radial boundaries are checked analytically. Straight edges must leave the
inner circle outward. Area is integrated over the original lines and arcs,
including negative inner-arc contributions; no polygonal replacement is made.
Small and major sectors, inward sheets and parameter-seam crossings are
supported. Together with partial cylinders this also permits closed annular
sector solids. Each sector consumes five shared checks. Nonconcentric or
nonradial variants and additional hole loops remain unsupported. Existing
convex profiles, including nonconcentric stadium arcs, retain their separate
validation and corner budget.

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

A hole-free convex planar outer contour may additionally contain polynomial
quadratic B-spline edges, including multiple Bezier pieces within one source
edge. Source intervals can be partial or start from an unclamped axis;
homogeneous knot insertion and
cropping isolate the selected interval without changing its parameter frame.
For validation, each selected knot span is restricted to a quadratic Bezier
piece. Its three controls must lie in the face plane within the source length
tolerance. Their convex hull then certifies that whole piece's plane membership.
This conservative hull test can reject curves whose actual excursion is within
tolerance. The serialized edge retains the wire exporter's endpoint separation
certificate and checks its rounded endpoints against the named source vertices.

The derivative of a quadratic Bezier curve interpolates its two endpoint
derivatives. Nonnegative tangent turning shorter than pi excludes inflections
and interior derivative reversals; degenerate endpoint derivatives fail. The
projected control-polygon perimeter of each piece must exceed the source length
tolerance. Existing convex-corner and full-revolution checks apply to the contour,
including adjacent LINE/CIRCLE/ELLIPSE edges. Consecutive pieces within one
spline edge must have continuous projected tangent directions; their nonzero
parameter speeds may differ. The per-join angular noise allowance is
`min(1e-10, tolerance/perimeter)/piece_count`, in radians. Genuine internal
kinks are unsupported, even when the resulting contour would be convex.
For projected controls `P,Q,R`, the exact doubled signed area contribution is
`(2*cross(P,Q) + cross(P,R) + 2*cross(Q,R))/3`, with the source traversal sign.
The control polygon supplies only a conservative perimeter bound; it is not
serialized as the boundary. Piece areas and tangent turning are accumulated in
source traversal order. STEP retains a single selected source B-spline per edge,
including its interior knots and multiplicities; internal validation pieces
are never emitted as additional edges. Rational or cubic spline bounds,
spline holes and additional hole loops remain unsupported.
NURBS supporting surfaces are a separate remaining extension.

Each Bezier piece consumes three control-plane checks, alongside the existing
corner checks and final edge endpoint-hull check. Restrictions during both face
preparation and edge serialization share the same work budget. A full two-edge
parabolic segment consumes six checks; a partial segment of the same clamped
quadratic consumes 56, including both sets of knot insertions.
Multi-span validation additionally charges each piece restriction and one check
per internal tangent join. A clamped two-span full parabolic bound with a chord
consumes 29 checks for a simple interior knot or 22 for a double knot; partial
intervals with two new interior endpoints crossing that knot consume 89 or 92
checks, respectively. The extra work isolates polynomials for proof and does not
alter source topology.

Elliptic tangent turning is computed analytically, rather than equating the
ellipse parameter to its tangent angle. Arcs exceeding half a revolution,
split ellipses, reversed traversal and mixed circle/ellipse lenses are supported.
Arc length is bounded by the smaller/larger radius times the absolute parameter
span; the lower bound must exceed the source tolerance, while the upper bound is used
for conservative area and corner-noise checks. Extremely eccentric or tiny arcs
whose lower bound is unresolved are rejected explicitly.

Convex outer contours containing only `LINE` and `CIRCLE` edges may also
contain complete one-edge circular holes. Each hole is checked against every
supporting tangent half-plane of the outer contour. For an arc, a conservative
clearance bound is calculated over its complete directed interval using the extrema
of `A*cos(t) + B*sin(t)`, including major arcs and parameter-seam crossings.
Checking only arc endpoints would miss disks crossing the middle of a rounded
corner. The implementation uses projected source tangents and a conservative
normal-length bound, without sampling or replacing arcs by chords.

Every circular hole must wind against the effective face normal and remain
more than two source tolerances inside all support lines. Hole pairs must be
separated by more than the sum of their radii plus two tolerances; touching,
overlapping and nested holes fail explicitly. Source loop order and named or
vertex-free ring seams are preserved. Each outer corner, hole/outer-edge pair
and hole/hole pair consumes one shared check. Net area subtracts all circular
disks and contributes to the existing solid-volume integral. Closed supported
mixtures therefore include cylinder-sector solids with cylindrical through
bores.

The same curved outer subset also supports polygonal holes, including concave
hole profiles and mixtures with complete circular holes. Every polygon vertex
must remain more than two source tolerances inside all outer support lines.
Because the outer region is convex, this also contains each entire straight
edge and the polygon interior. Each vertex/outer-edge pair consumes one check.
Polygon holes must wind against the effective normal, be simple, mutually
separated and nonnested. These tests reuse the existing polygon segment and
containment checks. Mixed disk/segment distance and containment tests exclude
circle/polygon crossings, touching and nesting in either direction. Their
shared-budget accounting is the same as in the polygon/full-circle subsets.
Source loop order is retained even when polygonal or circular holes come first.
Net area subtracts both hole types. Closed supported mixtures include sector
solids with polygonal through bores and unchanged analytic outer mantles.

A polygonal or complete one-edge circular outer bound may also contain
convex multi-edge `LINE`/`CIRCLE`/`ELLIPSE` holes, alongside polygonal and full-circle
holes. The curved hole must wind against the effective face normal. The
existing convexity and exact source-curve area checks validate each complete
hole contour. Analytic directed-arc extrema produce enclosing rectangles in
the face frame, expanded for source tolerance and coordinate roundoff. These
rectangles certify strict containment and separation using the established
polygon/full-circle checks. The original outer loops and other holes are
validated before testing the additional envelopes. Ellipse extrema retain the
separate major/minor radius coefficients in each projected direction; tangent
turning and area use the existing analytic ellipse checks. Rotated frames,
major elliptic segments and split ellipses preserve their original parameters
and source edges. This support requires a multi-edge contour; single-edge
complete elliptical holes remain outside this subset.

For nearly tangent contours at large world coordinates, explicit source fin
intervals avoid ambiguity from re-inferring parameters at rounded endpoints.
The same strict convexity, planarity and area checks still apply.

Envelopes are used only for validation. They are never serialized and their
areas are never subtracted from the face: net area uses each original curved
contour's analytic integral. This preserves rounded slots, major circular
segments, split-circle holes and their source vertices/edges, including
parameter-seam crossings. Closed supported shells can therefore include
prisms with rounded through bores. Original loop order and outer-bound identity
remain intact. One shared check is charged for every curved edge's extrema,
in addition to its convex corner check. The original mixed-boundary checks
and the envelope-augmented checks both consume the same shared budget.

The envelope test is conservative: geometrically disjoint holes may have
overlapping rectangles, or a valid curved hole's rectangle may leave its outer
boundary. Such cases return an explicit unsupported-format error rather than
claiming a validity certificate. Curved outer contours with multi-edge curved
holes, nonconvex curved holes, single-edge elliptical holes and elliptic outer arcs
with holes remain unsupported. Existing hole-free elliptic contours and
separate polygon/full-circle boundary subsets retain their established checks.

The separate single-edge conic-loop subset requires a complete circle or
ellipse per loop. The exporter checks the maximum plane residual over the entire analytic curve, not
just the binding samples, and validates winding against the effective face
normal. Circular outer/inner loops may appear in any source order. Every hole
must lie inside the outer circle, and holes must be mutually disjoint; each
separation must exceed two source tolerances. Single-edge elliptical holes remain
unsupported in this conic-loop subset.

Profiled complete cylindrical bands additionally support two full-azimuth
boundaries assembled from `CIRCLE`/`ELLIPSE` arcs and isolated axial `LINE`
generators. This includes oblique planar cuts and stepped or scalloped rims.
Every complete supporting conic must project onto the cylinder's radial circle;
endpoint membership alone is insufficient. Each profile must wind monotonically
once, and the two complete axial ranges must be separated by more than two
source tolerances. Overlapping ranges remain unsupported, even for some valid
parallel oblique cuts. Source arcs and axial generators remain exact STEP edges.
The analytic volume integral retains transverse origin terms for variable-height
bands; complete azimuth alone does not make these terms vanish.

Single hole-free planar `LINE`/`CIRCLE` contours can also have concave corners
and minor circular arcs when conservative certificates establish simplicity.
The endpoint chord polygon must be simple with positive winding. Arc departure
cones separate adjacent curves outside the join tolerance; sagitta envelopes
separate nonadjacent curves. Ambiguous envelopes return unsupported-format,
including some valid close contours. These chords are validation aids only:
STEP retains the original circular arcs and analytic source-curve area.
Concave elliptic contours and nonconvex curved contours with holes remain
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

Partial cylinder patches support one four-edge loop alternating between two
`CIRCLE` arcs and two axial `LINE` generators, with four distinct source vertices.
Both complete supporting circles must be coaxial at the cylinder radius, at
levels separated by more than two source tolerances. Opposite arc windings must
agree with the effective face normal and select equal azimuth spans strictly
shorter than one turn; the two generators must remain more than two tolerances
apart. Whole-circle displacement/tilt and straight-generator drift are bounded
analytically. The source loop becomes one `FACE_OUTER_BOUND` with its four exact
curves. Major arcs, reversed axes, inward sheets and parameter-seam crossings are
supported. Closed supported mixtures include quarter-cylinder sector solids.
Partial-cylinder flux retains transverse translation terms; these cancel only
for horizontal complete azimuth bands. Five shared checks cover each patch's four edges
and cut-pair comparison. Other cylinder trimming contours, holes in a patch and
elliptic cuts in partial patches remain unsupported.

Complete conical bands use the same coaxial cut, winding and budget checks as
cylinders, with the linear profile `r(z) = r_ref + z * sin(a) / cos(a)` used by
libopenparasolid. Both cut radii must exceed the source tolerance; linearity
keeps the band on the positive-radius branch between them. Signed volume
includes the analytic conical flux, including inward conical tube walls.
Narrowing source cones use a positive STEP semi-angle and reversed exported
axis. The reference origin may be at the apex, provided both boundary circles
are away from it. Apex boundary loops remain unsupported.

Partial conical patches support one four-edge loop alternating between two
coaxial `CIRCLE` arcs and two straight `LINE` generators. Both cut radii must
exceed the source tolerance and match the source linear cone profile. The
whole-circle bound includes center displacement, tilt and slope-dependent
radius variation. The lower and upper arcs must select equal azimuth spans,
strictly shorter than one turn, and have opposite winding consistent with the
effective face normal. The two generators must stay more than two tolerances
apart even at the smaller cut radius. Each straight edge is checked against the
cone ruling over its entire length; twisted connections between different
azimuths are rejected. Source edges and vertices remain exact and shared.
Major arcs, parameter-seam crossings, inward sheets, reversed axes and narrowing
cones are supported. Together with planar caps and radial walls this permits
closed conical sector solids. Signed volume uses the exact conical divergence
integral, retaining both transverse and axial origin terms. Five shared checks
cover each patch. Other conical trimming contours, apex boundaries, elliptic
cuts and additional hole loops remain unsupported.

Spherical bands use two complete coaxial latitude circles on a positive-radius
sphere. The exporter checks circle radius, center displacement and whole-circle
tilt against `hypot(cut_radius, axial_level) = sphere_radius`. The displacement
bound includes twice the axial variation for small allowed tilts. Cut levels
must differ by more than two source tolerances and stay more than one tolerance
from either pole. The lower/upper circles must wind oppositely relative to the
effective face normal. Reversed axes, source loop order and inward sheet
orientation are supported. Both loops use `FACE_BOUND`; no seam edges are
invented by the exporter. Solid volume uses the exact spherical divergence
integral, including the sphere center's offset from the integration origin.
Closed supported shells can combine spherical bands, planar caps and cylindrical
bore walls.

A spherical cap has one complete latitude circle. Its winding relative to the
effective face normal selects the region above or below the cut; no shortest-cap
assumption is made. Small caps, hemispheres and caps larger than a hemisphere
are supported as sheets or with a shared planar closing disk. The selected pole
is interior to the face, without an invented source loop, edge or vertex. The
existing spherical divergence integral includes that pole as an integration
limit. Importers may introduce their own periodic seam and degenerate pole edge;
export statistics continue to count source boundaries and their shared seam
vertices. Whole-cut profile checks and pole clearance apply to the boundary
circle just as for bands. Two complementary source cap faces can also close a
complete sphere, sharing their original latitude edge. A single unbounded
sphere face, explicit pole cuts and partial azimuths remain unsupported.

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

Circular `SPUN_SURF` generators in an axial meridian plane of a regular ring
torus export as an equivalent analytic `TOROIDAL_SURFACE`. The center is the
generator-center projection onto the rotation axis; its radial distance and the
generator radius are the major and minor torus radii. The whole circle plane
must agree with the axial meridian plane within frame and displacement bounds.
The `du x dv` orientation is compared with the canonical outward torus normal;
reversed generator normals and rotation axes can invert it. Source surface and
face senses are then applied separately. Generator sense metadata does not
reverse its evaluated parameterization. Auxiliary source `x_axis`, endpoint and
internal `scale` metadata are not evaluation transforms, matching the reader.
Explicit generator parameter bounds remain unsupported to avoid enlarging the
source surface domain. Skew/tilted circle planes, noncircular generators and
horn/apple/lemon surfaces also remain unsupported.

Each of a torus band's two circular cuts can comprise multiple monotone
`CIRCLE` arcs on one supporting circle. Binding establishes endpoint incidence
and closure; analytic center/radius/plane comparisons certify their common
support, and signed spans must make exactly one revolution without reversals.
Only the validation cut is combined: STEP keeps every source edge and vertex.
Each arc consumes one shared comparison in addition to the band's cut-pair
check. The paired tolerant circular subset below provides one such explicit mapping.

Paired tolerant edges with no recorded 3D edge curve support one planar
`SP_CURVE` and one constant-profile `SP_CURVE` on a recognized circular spun
torus. Vertex-bounded arcs span their full clamped, nonperiodic polynomial
spline domains. Complete rings can instead be vertex-free or retain one source
seam vertex. The rotational pcurve is linear with a
constant generator parameter: its exact 3D image is a circle. The planar
pcurve comprises quadratic Bezier spans. Bernstein bounds on each complete
span's squared radius certify agreement with that circle within the edge
allowance, with axial/frame/coordinate roundoff reserved. Derivative bounds and
angular half-planes certify monotone winding and the same directed arc or
single complete turn. Endpoint samples alone cannot certify these edges. Adaptive subdivision
consumes the shared comparison budget and has an explicit depth limit.

The exact rotational circle supplies one shared 3D master curve.
`SURFACE_CURVE` retains both `PCURVE` geometries: original planar spline controls,
with exact knot-domain restriction where needed, and the exact affine change
from spun generator/azimuth parameters
to torus azimuth/meridian parameters. No circle is fitted to the approximate
planar spline. Source topology, pcurves and declared edge tolerances are
retained. The optional source allowance is applied only after this geometric
certificate; strict mode rejects source discrepancies exceeding caller tolerance.
The serialized STEP uncertainty reports any larger source allowance actually
used. These are tolerant B-reps: associated pcurves need not coincide exactly
with their 3D master. General tolerant edges, rational pcurves, arbitrary
periodic layouts, partial spline domains and other surface pairings remain
unsupported.

Single-fin closed tolerant loops with stored `TRIMMED_CURVE` intervals can be
resolved by the exporter after ordinary binding reports an unsupported case.
Stored endpoint samples and closure are checked first; the entire paired circle
certificate must pass before supplying any missing interval. Caller overrides
are retained. Complete face winding, manifold and solid-volume checks still
apply after rebinding. The generic reader continues to reject ambiguous closed
fin intervals. Both the preflight and final certificates consume the same
comparison budget; invalid input and resource failures do not trigger retries.

Quadratic periodic ring pcurves can have one inactive exterior knot at each
side of the natural domain and multiplicity two at its endpoints. On that
specific layout the exterior knots are replaced by endpoint clamps without
changing any control point or polynomial piece inside the selected domain.
This is exact restriction, not spline fitting or automatic parameter wrapping.
Other periodic layouts and incomplete-domain intervals remain unsupported.
The exact circle master retains the recorded seam's spatial phase with a zero
master parameter; both pcurves retain their own source phases.

These checks do not certify all geometric intersections, disjointness of
separate components, or full CAD-kernel validity. No triangulation or geometry
approximation is written.

NURBS curves outside the open-wire/planar-quadratic subsets and NURBS surfaces, uncertified nonconvex planar curved boundaries, holes in elliptic/nonconvex curved contours,
uncertified curved-hole envelopes, curved outer contours with curved holes,
nonconvex curved holes, single-edge elliptical holes, spherical pole cuts/unbounded sphere faces, other cylinder/cone trims, partial torus azimuths, uncut tori,
horn/apple/lemon tori, apex loops, explicit seam fins, other
non-planar surfaces, other tolerant fin-specific curves/pcurves, isolated face loops,
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
the entire selected body. Convex line/arc contours consume one check per corner.
Circular containment and hole-pair comparisons consume one check each from the
same budget. Each open B-spline wire interior control point consumes one
endpoint separation check. Partial or unclamped spline restriction also charges the original
control count and every insertion's new control count. Convex curved-hole envelopes consume one check per edge plus
one per convex corner; both original and envelope-augmented mixed-boundary
validation passes share this budget. Convex curved contours with circular holes consume one additional
check per circular-hole/outer-edge pair, polygon-vertex/outer-edge pair and
circular-hole pair. Mixed polygon/circle
boundaries also consume one check per
circle-to-segment distance and per containment edge. Circular outer boundaries
consume one radial containment check per polygon vertex. Each cylindrical/
conical/spherical/toroidal cut pair consumes one check, as does each spherical
cap selection. Profiled cylinder bands consume one check per edge and one per
cut pair. Concave minor-arc validation shares the chord-polygon, join-cone and
nonadjacent envelope-pair checks, including any preceding convex attempt.
Each partial cylinder/cone patch and planar annular sector consumes five checks. Parser, model and body-binding
budgets remain independently configurable in `ExportOptions`. The file adapter
checks source file size before container inspection; libopenswx's per-stream decompression/extraction bounds
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
and check sharing, ring seams, senses, polygon/circular holes, through bores, cylindrical
bands/patches, closed cylinder sectors, conical frustums/patches/sectors, spherical bands/caps, toroidal intervals, convex circular/elliptic arc
contours, polygonal/circular holes in LINE/CIRCLE contours, cylindrical and
polygonal through bores in curved caps, rounded through bores, convex curved
hole envelopes, invalid
topology, budget boundaries and preserved source diagnostics.

An optional independent Open CASCADE round-trip test can be enabled with a
Python interpreter providing the `OCP` 7.x bindings (`cadquery-ocp<8`). The
test harness uses the 7.x topology-container API; OCP 8 is not yet supported:

```sh
cmake -S . -B build \
  -DOPENSWSTEP_OCCT_PYTHON=/path/to/python-with-OCP
cmake --build build --target openswstep_roundtrip_fixtures
ctest --test-dir build -R openswstep_occt_roundtrip --output-on-failure
```

It imports generated synthetic STEP documents, runs `BRepCheck_Analyzer`, checks
unique vertex/edge/face/solid counts and compares volumes, areas and curve
lengths, including hole areas and analytic cylinder/frustum/tube/sphere/torus
volumes. Different seam phases exercise importer-created periodic seams and ring
splits. Oblique full-cylinder cuts expose an OCCT import limitation: rebuilt
B-spline pcurves deviate from the unchanged exact 3D ellipse. Five such synthetic
measure checks use a documented relative tolerance of 5e-6; all other cases keep
1e-9. Exact exporter flux formulas have separate analytic unit tests. Four tolerant
synthetic models additionally verify that all eight original 3D master edges
remain exact circles after import; their volume comparisons retain 1e-9
relative tolerance. Closed-ring regressions additionally cover original and
synthetic seams, shifted source phases, exact periodic-domain restriction,
reversed frames and binary source entry points. OCCT may rebuild inconsistent pcurves from the 3D master
and creates its own B-spline meridian seam.
OCCT is exclusively a test dependency and is never linked into the export library.
Synthetic files live in a temporary directory and are removed.
Private CAD files and generated private exports must stay outside the repository.

Ten additional B-spline wire round trips cover polynomial/rational Bezier and
nonuniform multi-span curves, reversed edges, full-domain trim wrappers and
translated nonplanar controls. They check OCCT validity, topology, retained
degree/poles/weights/knots and source parameter values using an independent Cox
basis recurrence. Adaptive OCCT curve-length integration checks the polynomial
parabola and exact rational quarter circle against their analytic lengths with
the existing 1e-9 relative tolerance.

Twelve partial B-spline round trips additionally check cuts inside spans and at
existing knots, reversed and nested trims, and two independent sections sharing
one source basis. Imported points are compared against the original uncut
source's Cox basis, including both selected endpoints and values around retained
knots. Polynomial and rational arc lengths keep the same analytic tolerance.

Sixteen unclamped-source round trips additionally verify nonuniform exterior
knots, inactive endpoint knots, rational/nonplanar controls, full natural domains
and partial intervals in both directions. Independent source Cox-basis checks
use the left polynomial at the natural end, even when source knots continue
beyond it. Unit tests cover degrees one through three, reflected/asymmetric
exterior axes, work-budget boundaries and rejected extrapolation.

Twelve quadratic-boundary face round trips additionally verify complete and
partial parabolic segments, reversed source curves, inward faces, unclamped
axes, translated/rotated frames and a mixed circle/parabola lens. They check
exact B-spline degree/control counts, analytic source parameter values, topology,
B-rep validity and analytic face areas with the existing 1e-9 relative tolerance.

Eighteen multi-span quadratic face round trips additionally cover simple and
double interior knots, nonuniform knot spacing, reversed and inward bounds,
partial intervals, spatial transforms and cuts excluding the original join.
The imported source B-spline remains one edge with its retained interior knot
and multiplicity. Source parameter values and analytic areas retain the same
independent checks and 1e-9 area tolerance.

The last recorded private regression queried 57 explicitly selected bodies in 39
extracted streams: 53 exported successfully, including twenty-three face-bearing
bodies with 388 faces; 30 successes contain only wire/point geometry. All 53
generated documents pass OCCT import and B-rep validity checks. Closed tolerant
ring resolution adds one complete body with 16 faces and 16 source edges,
preserving all 52 previous successes. Its closed trim intervals are recovered
only after whole-curve checks, including exact restriction of periodic planar
pcurves to their natural domains. The other four return unsupported-format
errors. Their first remaining geometry obstacles are:

| Bodies | Missing support |
| --- | --- |
| 2 | Implicit plane/cone intersection curves and faces trimmed by them |
| 2 | NURBS surfaces and implicit intersections with analytic surfaces |

These private figures precede the subsequent B-spline export extensions, which
are covered by the synthetic cases above. These counts describe this private
regression selection, not general format coverage. No private source files or
generated exports are stored here.

Synthetic OCCT round trips independently check circular/elliptic convex
curved-hole and polygonal/mixed curved-hole areas, circular/polygonal cylinder-sector
through-bore volumes and rounded through bores. They also check annular and
conical sectors, major arcs, inward sheets, loop order, translated frames and
parameter-seam crossings, rotated ellipse frames and eccentric profiles.
Existing hole-free convex profiles retain their behavior and budget.
The remaining private bodies require additional geometry features before their
complete export can succeed. These are raw body selections, including archival
partition snapshots, rather than automatic active-configuration conversions.

Run `doxygen Doxyfile` from this directory to generate public API HTML at
`../build/openswstep-docs/html/index.html`; documentation warnings fail generation.

## Format references

- Siemens *Parasolid XT Format Reference*, July 2022: sections 4.3.3–4.3.10
  (body through vertex), 5.2.1.1–5.2.1.3 (line/circle/ellipse),
  5.2.1.4 (B_curve, p. 38; degree, knots and homogeneous controls),
  5.2.2.1–5.2.2.3 (plane/cylinder/cone), 5.2.2.5 (torus), 5.2.2.11 (Spun_surf, p. 68), 5.2.8 (curve/surface senses),
  5.2.1.8 (SP_curve, p. 51), 5.3.4.1 (attaching geometry to topology) and
  5.3.11 (Edge, p. 98).
  The local reading copy is in the workspace's `xt-doc`; it is not bundled here.
- [STEP toroidal-surface attributes](https://www.steptools.com/docs/stp_aim/html/t_toroidal_surface.html).
- [STEP conical-surface attributes](https://www.steptools.com/docs/stp_aim/html/t_conical_surface.html).
- [STEP advanced-face restrictions](https://www.steptools.com/docs/stp_aim/html/t_advanced_face.html).
- [STEP edge-based wireframe representation](https://www.steptools.com/docs/stp_aim/html/t_edge_based_wireframe_shape_representation.html)
  and [permitted wireframe edge curves](https://www.steptools.com/docs/stp_aim/html/t_valid_wireframe_edge_curve.html).
- [OCCT STEP translation model](https://github.com/Open-Cascade-SAS/OCCT/blob/master/dox/user_guides/step/step.md).

The implementation is original MIT code; no third-party parser/exporter
implementation is incorporated.
