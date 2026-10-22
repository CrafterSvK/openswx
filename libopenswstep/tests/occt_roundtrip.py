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
CASES = {
    "tetra": (4, 6, 4, 1, "volume", 1 / 6),
    "cube": (8, 12, 6, 1, "volume", 1),
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
