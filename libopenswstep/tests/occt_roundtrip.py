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
