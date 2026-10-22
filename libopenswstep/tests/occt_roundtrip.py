# SPDX-License-Identifier: MIT
# Copyright (c) 2026 openswx contributors
"""Optional independent OCCT import, B-rep and geometric-measure regression."""
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedMapOfShape

# Counts are unique topology instances. Volumes/areas/lengths use millimetres.
TOROIDAL_SOLID_VOLUME = math.pi * (9 + 1.5 * math.pi + 2 / 3)
CASES = {
    "stadium": (4,4,1,0,"area",4+math.pi),
    "annular-sector": (4,4,1,0,"area",2*math.pi),
    "annular-half": (4,4,1,0,"area",4*math.pi),
    "annular-major": (4,4,1,0,"area",6*math.pi),
    "annular-inward": (4,4,1,0,"area",2*math.pi),
    "annular-seam": (4,4,1,0,"area",2*math.pi),
    "oblique-annular-sector": (4,4,1,0,"area",2*math.pi),
    "annular-solid": (8,12,6,1,"volume",4*math.pi),
    "annular-major-solid": (8,12,6,1,"volume",12*math.pi),
    "oblique-annular-solid": (8,12,6,1,"volume",4*math.pi),
    "rounded-circular-holes": (10,10,1,0,"area",8+7*math.pi/8),
    "rounded-circular-holes-first": (10,10,1,0,"area",8+7*math.pi/8),
    "rounded-circular-holes-seams": (10,10,1,0,"area",8+7*math.pi/8),
    "rounded-circular-holes-inward": (10,10,1,0,"area",8+7*math.pi/8),
    "oblique-rounded-circular-holes": (10,10,1,0,"area",8+7*math.pi/8),
    "stadium-circular-hole": (5,5,1,0,"area",4+15*math.pi/16),
    "half-disk-circular-hole": (3,3,1,0,"area",7*math.pi/16),
    "major-segment-circular-hole": (3,3,1,0,"area",11*math.pi/16+.5),
    "major-seam-circular-hole": (3,3,1,0,"area",11*math.pi/16+.5),
    "lens-circular-hole": (3,3,1,0,"area",15*math.pi/16-2),
    "split-circle-hole": (5,5,1,0,"area",15*math.pi/16),
    "cylinder-sector-bore": (8,12,6,1,"volume",12*math.pi/25),
    "cylinder-sector-bore-seams": (8,12,6,1,"volume",12*math.pi/25),
    "oblique-cylinder-sector-bore": (8,12,6,1,"volume",12*math.pi/25),
    "cone-patch": (4,4,1,0,"area",2*math.sqrt(2)*math.pi),
    "cone-half-patch": (4,4,1,0,"area",4*math.sqrt(2)*math.pi),
    "cone-major-patch": (4,4,1,0,"area",6*math.sqrt(2)*math.pi),
    "cone-inward-patch": (4,4,1,0,"area",2*math.sqrt(2)*math.pi),
    "cone-patch-seam": (4,4,1,0,"area",2*math.sqrt(2)*math.pi),
    "oblique-cone-patch": (4,4,1,0,"area",2*math.sqrt(2)*math.pi),
    "narrowing-cone-patch": (4,4,1,0,"area",3*math.sqrt(17)*math.pi/16),
    "narrowing-cone-major-patch": (4,4,1,0,"area",9*math.sqrt(17)*math.pi/16),
    "cone-sector": (6,9,5,1,"volume",13*math.pi/6),
    "oblique-cone-sector": (6,9,5,1,"volume",13*math.pi/6),
    "narrowing-cone-sector": (6,9,5,1,"volume",7*math.pi/24),
    "oblique-narrowing-cone-sector": (6,9,5,1,"volume",7*math.pi/24),
    "cylinder-patch": (4, 4, 1, 0, "area", math.pi),
    "cylinder-half-patch": (4, 4, 1, 0, "area", 2*math.pi),
    "cylinder-major-patch": (4, 4, 1, 0, "area", 3*math.pi),
    "cylinder-inward-patch": (4, 4, 1, 0, "area", math.pi),
    "cylinder-patch-seam": (4, 4, 1, 0, "area", math.pi),
    "oblique-cylinder-patch": (4, 4, 1, 0, "area", math.pi),
    "cylinder-sector": (6, 9, 5, 1, "volume", math.pi/2),
    "oblique-cylinder-sector": (6, 9, 5, 1, "volume", math.pi/2),
    "partitioned-sphere": (3, 5, 2, 1, "volume", 500 * math.pi / 3),
    "partitioned-sphere-seams": (3, 5, 2, 1, "volume", 500 * math.pi / 3),
    "spherical-cap-north": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-cap-south": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-cap-major": (2, 3, 2, 1, "volume", 448 * math.pi / 3),
    "spherical-cap-major-south": (2, 3, 2, 1, "volume", 448 * math.pi / 3),
    "spherical-cap-seams": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-hemisphere": (2, 3, 2, 1, "volume", 250 * math.pi / 3),
    "spherical-hemisphere-south": (2, 3, 2, 1, "volume", 250 * math.pi / 3),
    "spherical-cap-sheet": (2, 3, 1, 0, "area", 20 * math.pi),
    "spherical-cap-major-sheet": (2, 3, 1, 0, "area", 80 * math.pi),
    "spherical-cap-inward": (2, 3, 1, 0, "area", 20 * math.pi),
    "spherical-cap-axis": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "oblique-spherical-cap": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-solid": (2, 3, 3, 1, "volume", 66 * math.pi),
    "spherical-seams": (2, 3, 3, 1, "volume", 66 * math.pi),
    "spherical-symmetric": (2, 3, 3, 1, "volume", 132 * math.pi),
    "spherical-south": (2, 3, 3, 1, "volume", 66 * math.pi),
    "spherical-tube": (4, 6, 4, 1, "volume", 63 * math.pi),
    "spherical-tube-seams": (4, 6, 4, 1, "volume", 63 * math.pi),
    "spherical-sheet": (2, 3, 1, 0, "area", 30 * math.pi),
    "spherical-inward": (2, 3, 1, 0, "area", 30 * math.pi),
    "spherical-loop-order": (2, 3, 3, 1, "volume", 66 * math.pi),
    "spherical-axis": (2, 3, 3, 1, "volume", 66 * math.pi),
    "oblique-spherical-solid": (2, 3, 3, 1, "volume", 66 * math.pi),
    "circle-polygon-hole": (5, 5, 1, 0, "area", 36 * math.pi - 4),
    "circle-polygon-seams": (5, 5, 1, 0, "area", 36 * math.pi - 4),
    "circle-polygon-holes": (9, 9, 1, 0, "area", 36 * math.pi - 8),
    "circle-mixed-holes": (10, 10, 1, 0, "area", 35 * math.pi - 8),
    "circle-mixed-first": (10, 10, 1, 0, "area", 35 * math.pi - 8),
    "circle-concave-hole": (7, 7, 1, 0, "area", 36 * math.pi - 3),
    "circle-concave-mixed": (8, 8, 1, 0, "area", 35 * math.pi - 3),
    "circle-mixed-reordered": (6, 6, 1, 0, "area", 35 * math.pi - 4),
    "oblique-circle-mixed": (10, 10, 1, 0, "area", 35 * math.pi - 8),
    "polygon-circular-holes": (6, 6, 1, 0, "area", 96 - 2 * math.pi),
    "polygon-circular-seams": (6, 6, 1, 0, "area", 96 - 2 * math.pi),
    "polygon-mixed-holes": (10, 10, 1, 0, "area", 92 - 2 * math.pi),
    "polygon-mixed-hole-first": (10, 10, 1, 0, "area", 92 - 2 * math.pi),
    "concave-circular-holes": (10, 10, 1, 0, "area", 88 - 2 * math.pi),
    "concave-mixed-holes": (14, 14, 1, 0, "area", 84 - 2 * math.pi),
    "oblique-mixed-holes": (10, 10, 1, 0, "area", 92 - 2 * math.pi),
    "cube-circular-bore": (10, 15, 7, 1, "volume", 1 - math.pi / 16),
    "cube-circular-bore-seams": (10, 15, 7, 1, "volume", 1 - math.pi / 16),
    "oblique-circular-bore": (10, 15, 7, 1, "volume", 1 - math.pi / 16),
    "mixed-conic-lens": (2, 2, 1, 0, "area", math.pi / 2 + 2 * math.pi / (3 * math.sqrt(3)) - 2),
    "half-ellipse": (2, 2, 1, 0, "area", math.pi),
    "oblique-half-ellipse": (2, 2, 1, 0, "area", math.pi),
    "eccentric-half-ellipse": (2, 2, 1, 0, "area", 50 * math.pi),
    "elliptic-rounded-rectangle": (8, 8, 1, 0, "area", 16 + 2 * math.pi),
    "elliptic-rounded-reverse": (8, 8, 1, 0, "area", 16 + 2 * math.pi),
    "elliptic-lens": (2, 2, 1, 0, "area", 2 * math.pi - 4),
    "major-elliptic-segment": (2, 2, 1, 0, "area", 3 * math.pi / 2 + 1),
    "split-ellipse": (4, 4, 1, 0, "area", 2 * math.pi),
    "rounded-rectangle": (8, 8, 1, 0, "area", 8 + math.pi),
    "rounded-rectangle-reverse": (8, 8, 1, 0, "area", 8 + math.pi),
    "half-disk": (2, 2, 1, 0, "area", math.pi / 2),
    "quarter-disk": (3, 3, 1, 0, "area", math.pi / 4),
    "circular-lens": (2, 2, 1, 0, "area", math.pi - 2),
    "major-circular-segment": (2, 2, 1, 0, "area", 3 * math.pi / 4 + 0.5),
    "split-circle": (4, 4, 1, 0, "area", math.pi),
    "oblique-rounded-rectangle": (8, 8, 1, 0, "area", 8 + math.pi),
    "toroidal-tube": (4, 6, 4, 1, "volume", TOROIDAL_SOLID_VOLUME - math.pi),
    "toroidal-inward": (2, 3, 1, 0, "area", 3 * math.pi**2 + 2 * math.pi),
    "toroidal-half": (2, 3, 1, 0, "area", 6 * math.pi**2),
    "toroidal-profile-seam": (2, 3, 1, 0, "area", 3 * math.pi**2 - 2 * math.pi * math.sqrt(2)),
    "toroidal-solid": (2, 3, 3, 1, "volume", TOROIDAL_SOLID_VOLUME),
    "toroidal-solid-seams": (2, 3, 3, 1, "volume", TOROIDAL_SOLID_VOLUME),
    "toroidal-band": (2, 3, 1, 0, "area", 3 * math.pi**2 + 2 * math.pi),
    "toroidal-complement": (2, 3, 1, 0, "area", 9 * math.pi**2 - 2 * math.pi),
    "oblique-toroidal-solid": (2, 3, 3, 1, "volume", TOROIDAL_SOLID_VOLUME),
    "toroidal-loop-order": (2, 3, 3, 1, "volume", TOROIDAL_SOLID_VOLUME),
    "toroidal-axis": (2, 3, 3, 1, "volume", TOROIDAL_SOLID_VOLUME),
    "frustum": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    "frustum-apex-origin": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    "frustum-narrowing": (2, 3, 3, 1, "volume", 26 * math.pi / 3),
    "frustum-seams": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    "conical-tube": (4, 6, 4, 1, "volume", 24 * math.pi),
    "conical-tube-seams": (4, 6, 4, 1, "volume", 24 * math.pi),
    "conical-mantle": (2, 3, 1, 0, "area", 16 * math.pi * math.sqrt(2)),
    "oblique-frustum": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    "frustum-loop-order": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    "frustum-axis": (2, 3, 3, 1, "volume", 98 * math.pi / 3),
    # OCCT creates a seam edge on each complete periodic cylinder face,
    # splitting one circular boundary when the two source seam phases differ.
    "cylinder": (2, 3, 3, 1, "volume", 18 * math.pi),
    "cylinder-shifted-seams": (3, 4, 3, 1, "volume", 18 * math.pi),
    "cylinder-shifted-frames": (3, 4, 3, 1, "volume", 18 * math.pi),
    "cylinder-loop-order": (2, 3, 3, 1, "volume", 18 * math.pi),
    "cylinder-axis": (2, 3, 3, 1, "volume", 18 * math.pi),
    "cylinder-seams": (2, 3, 3, 1, "volume", 18 * math.pi),
    "tube": (4, 6, 4, 1, "volume", 16 * math.pi),
    "tube-seams": (4, 6, 4, 1, "volume", 16 * math.pi),
    "cylinder-mantle": (2, 3, 1, 0, "area", 12 * math.pi),
    "cylinder-sheet": (2, 3, 3, 0, "area", 30 * math.pi),
    "oblique-cylinder": (2, 3, 3, 1, "volume", 18 * math.pi),
    "oblique-tube": (4, 6, 4, 1, "volume", 16 * math.pi),
    "disk": (1, 1, 1, 0, "area", math.pi),
    "disk-seam": (1, 1, 1, 0, "area", math.pi),
    "ellipse-disk": (1, 1, 1, 0, "area", 2 * math.pi),
    "circular-annulus": (2, 2, 1, 0, "area", 8 * math.pi),
    "circular-annulus-hole-first": (2, 2, 1, 0, "area", 8 * math.pi),
    "circular-holes": (3, 3, 1, 0, "area", 14 * math.pi),
    "oblique-circular-holes": (3, 3, 1, 0, "area", 14 * math.pi),
    "disk-reversed": (1, 1, 1, 0, "area", math.pi),
    "disk-negative-normal": (1, 1, 1, 0, "area", math.pi),
    "tetra": (4, 6, 4, 1, "volume", 1 / 6),
    "cube": (8, 12, 6, 1, "volume", 1),
    "perforated-sheet": (8, 8, 1, 0, "area", 12),
    "hole-first-sheet": (8, 8, 1, 0, "area", 12),
    "two-hole-sheet": (12, 12, 1, 0, "area", 21),
    "perforated-prism": (16, 24, 10, 1, "volume", 12),
    "negative-hole-sheet": (8, 8, 1, 0, "area", 12),
    "oblique-hole-sheet": (8, 8, 1, 0, "area", 12),
    "square": (4, 4, 1, 0, "area", 1),
    "concave": (5, 5, 1, 0, "area", 3),
    "line": (2, 1, 0, 0, "length", 2),
    "circle": (2, 1, 0, 0, "length", math.pi / 2),
    "circle-reverse": (2, 1, 0, 0, "length", 3 * math.pi / 2),
    "ring": (1, 1, 0, 0, "length", 2 * math.pi),
    # Ellipse lengths are independently integrated by OCCT; counts/validity
    # still detect a truncated/full-ring or missing-topology representation.
    "ellipse": (2, 1, 0, 0, None, None),
    "ellipse-ring": (1, 1, 0, 0, None, None),
    "acorn": (1, 0, 0, 0, None, None),
    "sheet-pair": (4, 5, 2, 0, "area", 1),
}

with tempfile.TemporaryDirectory(prefix="openswstep-occt-") as directory:
    subprocess.run([sys.argv[1], directory], check=True)
    for name, expected in CASES.items():
        reader = STEPControl_Reader()
        assert reader.ReadFile(str(Path(directory) / (name + ".step"))) == IFSelect_RetDone, name
        assert reader.TransferRoots() > 0, name
        shape = reader.OneShape()
        assert not shape.IsNull(), name
        assert BRepCheck_Analyzer(shape).IsValid(), name
        for kind, count in zip((TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID), expected[:4]):
            topology = TopTools_IndexedMapOfShape()
            TopExp.MapShapes_s(shape, kind, topology)
            assert topology.Extent() == count, (name, kind, topology.Extent(), count)
        measure, target = expected[4:]
        if measure:
            properties = GProp_GProps()
            operation = {"volume": BRepGProp.VolumeProperties_s,
                         "area": BRepGProp.SurfaceProperties_s,
                         "length": BRepGProp.LinearProperties_s}[measure]
            operation(shape, properties)
            assert math.isclose(properties.Mass(), target, rel_tol=1e-9, abs_tol=1e-12), (name, properties.Mass(), target)
        print(name, "valid, topology and measures verified")
