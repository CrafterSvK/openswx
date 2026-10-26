# SPDX-License-Identifier: MIT
# Copyright (c) 2026 openswx contributors
"""Optional independent OCCT import, B-rep and geometric-measure regression."""
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GeomAbs import GeomAbs_BSplineCurve, GeomAbs_Circle
from OCP.TopoDS import TopoDS
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.GCPnts import GCPnts_AbscissaPoint
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedMapOfShape

# Counts are unique topology instances. Volumes/areas/lengths use millimetres.
TOROIDAL_SOLID_VOLUME = math.pi * (9 + 1.5 * math.pi + 2 / 3)
MINOR_ARC_SEGMENT = 25 / 2 * (2 * math.asin(.2) - math.sin(2 * math.asin(.2)))
# A quadratic parabola trimmed from u=.1 to .8, and the corresponding source
# parameter interval on a positive-weight rational quadratic quarter circle.
TRIMMED_PARABOLA_LENGTH = sum(.5*(x*math.sqrt(1+x*x)+math.asinh(x)) for x in (.8,.6))
def quarter_circle_angle(u):
    w = math.sqrt(.5)
    return math.atan2(2*w*u*(1-u)+u*u, (1-u)**2+2*w*u*(1-u))
TRIMMED_CIRCLE_LENGTH = quarter_circle_angle(.8)-quarter_circle_angle(.1)
CASES = {
    "spindle-solid": (2,3,3,1,"volume",math.pi*(.25+math.pi/4+2/3)),
    "spindle-solid-seams": (2,3,3,1,"volume",math.pi*(.25+math.pi/4+2/3)),
    "spindle-small-major": (2,3,3,1,"volume",math.pi*(.0625**2+.03125*math.pi+2/3)),
    "spindle-solid-oblique": (2,3,3,1,"volume",math.pi*(.25+math.pi/4+2/3)),
    "spindle-sheet": (2,3,1,0,"area",.5*math.pi**2+2*math.pi),
    "spindle-overhang": (2,3,3,1,"volume",math.pi*(.25*math.sin(1.8)+.5*(1.8+math.sin(1.8)*math.cos(1.8))+math.sin(1.8)-math.sin(1.8)**3/3)),
    "horn-solid": (2,3,3,1,"volume",math.pi*(1+math.pi/2+2/3)),
    "horn-solid-seams": (2,3,3,1,"volume",math.pi*(1+math.pi/2+2/3)),
    "horn-sheet": (2,3,1,0,"area",math.pi**2+2*math.pi),

    "simple-stadium-ellipse-hole": (5,5,1,0,"area",4+.95*math.pi),
    "simple-annular-circle-hole": (5,5,1,0,"area",1.99*math.pi),
    "simple-notch-spline": (8,8,1,0,"area",10),
    "simple-notch-spline-oblique": (8,8,1,0,"area",10),
    "simple-notch-spline-inward": (8,8,1,0,"area",10),
    "simple-notch-spline-hole": (12,12,1,0,"area",98),
    "simple-tangential-cusp": (3,3,1,0,"area",1/3),
    "simple-concave-circle": (8,8,1,0,"area",12-math.pi/2),
    "simple-concave-ellipse": (8,8,1,0,"area",24-math.pi),
    "simple-concave-minor-close": (8,8,1,0,"area",4.8-MINOR_ARC_SEGMENT),
    "simple-half-ellipse-hole": (3,3,1,0,"area",.99*math.pi),
    "simple-quadratic-hole": (3,3,1,0,"area",4/3-.01*math.pi),
    "simple-inflected-cubic": (2,2,1,0,"area",.99),
    "simple-kinked-quadratic-low": (2,2,1,0,"area",1.25),
    "simple-kinked-quadratic-high": (2,2,1,0,"area",17/12),
    "simple-curved-spline-region": (4,4,1,0,"area",63/40-1/75),

    # OCCT reconstructs an apex vertex, seam generator and degenerate pole edge.
    "cone-apex": (2, 3, 2, 1, "volume", 6 * math.pi),
    "cone-apex-seam": (2, 3, 2, 1, "volume", 6 * math.pi),
    "cone-apex-narrowing": (2, 3, 2, 1, "volume", 6 * math.pi),
    "cone-apex-oblique": (2, 3, 2, 1, "volume", 6 * math.pi),
    "hole-spline-quadratic": (6,6,1,0,"area",72-4/3),
    "hole-spline-quadratic-partial": (6,6,1,0,"area",72-256/375),
    "hole-spline-quadratic-reversed": (6,6,1,0,"area",72-4/3),
    "hole-spline-quadratic-partial-inward": (6,6,1,0,"area",72-256/375),
    "hole-spline-quadratic-circle": (3,3,1,0,"area",100*math.pi-4/3),
    "hole-spline-quadratic-multi-repeated": (6,6,1,0,"area",72-4/3),
    "hole-spline-quadratic-multi-simple-partial-reversed": (3,3,1,0,"area",100*math.pi-256/375),
    "hole-spline-cubic": (6,6,1,0,"area",72-63/40),
    "hole-spline-cubic-partial-reversed": (6,6,1,0,"area",72-12924/15625),
    "hole-spline-cubic-inward": (6,6,1,0,"area",72-63/40),
    "hole-spline-cubic-circle-partial": (3,3,1,0,"area",100*math.pi-12924/15625),
    "hole-spline-cubic-multi-three-partial": (6,6,1,0,"area",72-12924/15625),
    "hole-spline-cubic-multi-two-reversed": (3,3,1,0,"area",100*math.pi-63/40),
    "hole-spline-cubic-two-mixed": (13,13,1,0,"area",72-2*63/40-.5-math.pi/16),
    "hole-spline-cubic-circle-two-mixed-partial-inward": (10,10,1,0,"area",100*math.pi-2*12924/15625-.5-math.pi/16),
    "hole-spline-quadratic-two": (8,8,1,0,"area",72-2*4/3),
    "hole-spline-cubic-two-mixed-oblique": (13,13,1,0,"area",72-2*63/40-.5-math.pi/16),
    "hole-spline-quadratic-circle-partial-oblique": (3,3,1,0,"area",100*math.pi-256/375),
    "cubic-face-three-span": (2,2,1,0,"area",63/40),
    "cubic-face-three-span-partial-reversed": (2,2,1,0,"area",12924/15625),
    "cubic-face-three-span-oblique": (2,2,1,0,"area",63/40),
    "cubic-face": (2,2,1,0,"area",63/40),
    "cubic-face-partial": (2,2,1,0,"area",12924/15625),
    "cubic-face-reversed": (2,2,1,0,"area",63/40),
    "cubic-face-inward": (2,2,1,0,"area",63/40),
    "cubic-face-partial-reversed": (2,2,1,0,"area",12924/15625),
    "cubic-face-unclamped": (2,2,1,0,"area",63/40),
    "cubic-face-partial-unclamped": (2,2,1,0,"area",12924/15625),
    "cubic-face-elevated": (2,2,1,0,"area",4/3),
    "cubic-face-elevated-partial": (2,2,1,0,"area",256/375),
    "cubic-face-multi-one": (2,2,1,0,"area",63/40),
    "cubic-face-multi-two": (2,2,1,0,"area",63/40),
    "cubic-face-multi-three": (2,2,1,0,"area",63/40),
    "cubic-face-multi-one-partial": (2,2,1,0,"area",12924/15625),
    "cubic-face-multi-two-partial-reversed": (2,2,1,0,"area",12924/15625),
    "cubic-face-multi-three-partial-inward": (2,2,1,0,"area",12924/15625),
    "cubic-face-multi-two-nonuniform": (2,2,1,0,"area",63/40),
    "cubic-face-multi-one-nonuniform-partial": (2,2,1,0,"area",12924/15625),
    "cubic-face-multi-two-nonuniform-oblique": (2,2,1,0,"area",63/40),
    "cubic-face-partial-oblique": (2,2,1,0,"area",12924/15625),
    "cubic-face-middle-zero": (2,2,1,0,"area",9/10),
    "quadratic-face-multi-repeated": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-simple": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-repeated-partial": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-simple-partial": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-repeated-reversed": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-simple-reversed": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-repeated-inward": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-simple-inward": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-repeated-partial-reversed": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-simple-partial-inward": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-repeated-nonuniform": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-simple-nonuniform-reversed": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-repeated-partial-nonuniform": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-simple-partial-nonuniform-reversed": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-repeated-partial-oblique": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-simple-oblique": (2,2,1,0,"area",4/3),
    "quadratic-face-multi-repeated-partial-outside-knot": (2,2,1,0,"area",256/375),
    "quadratic-face-multi-simple-partial-outside-knot": (2,2,1,0,"area",256/375),
    "quadratic-face": (2,2,1,0,"area",4/3),
    "quadratic-face-reversed": (2,2,1,0,"area",4/3),
    "quadratic-face-inward": (2,2,1,0,"area",4/3),
    "quadratic-face-reversed-inward": (2,2,1,0,"area",4/3),
    "quadratic-face-unclamped": (2,2,1,0,"area",4/3),
    "quadratic-face-partial": (2,2,1,0,"area",256/375),
    "quadratic-face-partial-reversed": (2,2,1,0,"area",256/375),
    "quadratic-face-partial-inward": (2,2,1,0,"area",256/375),
    "quadratic-face-partial-unclamped": (2,2,1,0,"area",256/375),
    "quadratic-face-oblique": (2,2,1,0,"area",4/3),
    "quadratic-face-partial-oblique": (2,2,1,0,"area",256/375),
    "quadratic-face-circle": (2,2,1,0,"area",4/3+math.pi/2),
    "spline-polynomial-unclamped": (2,1,0,0,None,None),
    "spline-polynomial-unclamped-reversed": (2,1,0,0,None,None),
    "spline-rational-unclamped": (2,1,0,0,None,None),
    "spline-rational-unclamped-reversed": (2,1,0,0,None,None),
    "spline-polynomial-unclamped-trimmed": (2,1,0,0,None,None),
    "spline-polynomial-unclamped-trimmed-reversed": (2,1,0,0,None,None),
    "spline-rational-unclamped-trimmed": (2,1,0,0,None,None),
    "spline-rational-unclamped-trimmed-reversed": (2,1,0,0,None,None),
    "spline-polynomial-unclamped-inactive": (2,1,0,0,"length",math.sqrt(2)+math.asinh(1)),
    "spline-polynomial-unclamped-inactive-reversed": (2,1,0,0,"length",math.sqrt(2)+math.asinh(1)),
    "spline-rational-unclamped-inactive": (2,1,0,0,"length",math.pi/2),
    "spline-rational-unclamped-inactive-reversed": (2,1,0,0,"length",math.pi/2),
    "spline-polynomial-unclamped-inactive-trimmed": (2,1,0,0,None,None),
    "spline-polynomial-unclamped-inactive-trimmed-reversed": (2,1,0,0,None,None),
    "spline-rational-unclamped-inactive-trimmed": (2,1,0,0,None,None),
    "spline-rational-unclamped-inactive-trimmed-reversed": (2,1,0,0,None,None),
    "spline-polynomial-trimmed": (2,1,0,0,"length",TRIMMED_PARABOLA_LENGTH),
    "spline-polynomial-trimmed-reversed": (2,1,0,0,"length",TRIMMED_PARABOLA_LENGTH),
    "spline-rational-trimmed": (2,1,0,0,"length",TRIMMED_CIRCLE_LENGTH),
    "spline-rational-trimmed-nested": (2,1,0,0,"length",TRIMMED_CIRCLE_LENGTH),
    "spline-polynomial-multi-trimmed": (2,1,0,0,None,None),
    "spline-rational-multi-trimmed": (2,1,0,0,None,None),
    "spline-rational-multi-trimmed-reversed": (2,1,0,0,None,None),
    "spline-rational-multi-trimmed-nested": (2,1,0,0,None,None),
    "spline-polynomial-multi-trimmed-knots": (2,1,0,0,None,None),
    "spline-rational-multi-trimmed-knots": (2,1,0,0,None,None),
    "spline-polynomial-multi-trimmed-pair": (4,2,0,0,None,None),
    "spline-rational-multi-trimmed-pair": (4,2,0,0,None,None),
    "spline-polynomial": (2,1,0,0,"length",math.sqrt(2)+math.asinh(1)),
    "spline-polynomial-reversed": (2,1,0,0,"length",math.sqrt(2)+math.asinh(1)),
    "spline-polynomial-wrapped": (2,1,0,0,"length",math.sqrt(2)+math.asinh(1)),
    "spline-rational": (2,1,0,0,"length",math.pi/2),
    "spline-rational-reversed": (2,1,0,0,"length",math.pi/2),
    "spline-rational-translated": (2,1,0,0,"length",math.pi/2),
    "spline-polynomial-multi": (2,1,0,0,None,None),
    "spline-rational-multi": (2,1,0,0,None,None),
    "spline-rational-multi-reversed": (2,1,0,0,None,None),
    "spline-rational-multi-translated": (2,1,0,0,None,None),
    "closed-tolerant-torus-shifted": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-shifted-named": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-named": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-clamped": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-axis": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-profile": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "closed-tolerant-torus-translated": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "tolerant-torus": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "tolerant-torus-axis": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "tolerant-torus-profile": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "tolerant-torus-translated": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "spun-torus": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "spun-torus-profile": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "spun-torus-axis": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "spun-torus-translated": (2,3,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "spun-torus-inward": (2,3,1,0,"area",3*math.pi**2+2*math.pi),
    "split-torus": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "split-spun-torus": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "split-spun-torus-translated": (8,9,3,1,"volume",TOROIDAL_SOLID_VOLUME),
    "profiled-cylinder": (2,3,3,1,"volume",18*math.pi),
    "profiled-cylinder-seams": (2,3,3,1,"volume",18*math.pi),
    "profiled-cylinder-two-cuts": (2,3,3,1,"volume",18*math.pi),
    "profiled-cylinder-translated": (2,3,3,1,"volume",18*math.pi),
    "profiled-cylinder-mantle": (2,3,1,0,"area",12*math.pi),
    "stepped-cylinder": (10,13,5,1,"volume",2.75*math.pi),
    "stepped-cylinder-translated": (10,13,5,1,"volume",2.75*math.pi),
    "concave-minor-arc": (8,8,1,0,"area",12-MINOR_ARC_SEGMENT),
    "concave-minor-arc-close": (8,8,1,0,"area",12-4*1.79-MINOR_ARC_SEGMENT),
    "concave-minor-arc-translated": (8,8,1,0,"area",12-MINOR_ARC_SEGMENT),
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
    "elliptic-rounded-hole": (12,12,1,0,"area",144-2*math.pi),
    "elliptic-rounded-hole-rotated": (12,12,1,0,"area",144-2*math.pi),
    "circle-outer-elliptic-hole": (9,9,1,0,"area",62*math.pi-16),
    "elliptic-rounded-mixed": (17,17,1,0,"area",143-33*math.pi/16),
    "circle-outer-elliptic-mixed": (14,14,1,0,"area",991*math.pi/16-17),
    "elliptic-mixed-rotated": (17,17,1,0,"area",143-33*math.pi/16),
    "oblique-elliptic-hole": (12,12,1,0,"area",144-2*math.pi),
    "elliptic-disk-hole": (8,8,1,0,"area",15-2*math.pi),
    "oblique-elliptic-disk-hole": (8,8,1,0,"area",15-2*math.pi),
    "half-elliptic-hole": (6,6,1,0,"area",64-math.pi),
    "major-elliptic-hole": (6,6,1,0,"area",63-1.5*math.pi),
    "major-elliptic-hole-rotated": (6,6,1,0,"area",63-1.5*math.pi),
    "eccentric-elliptic-hole": (6,6,1,0,"area",160000-50*math.pi),
    "eccentric-major-elliptic-hole": (6,6,1,0,"area",159950-75*math.pi),
    "eccentric-major-elliptic-hole-rotated": (6,6,1,0,"area",159950-75*math.pi),
    "convex-curved-hole": (12,12,1,0,"area",104-math.pi),
    "convex-curved-holes": (20,20,1,0,"area",96-2*math.pi),
    "convex-curved-hole-first": (12,12,1,0,"area",104-math.pi),
    "convex-curved-mixed": (25,25,1,0,"area",95-33*math.pi/16),
    "convex-curved-mixed-first": (25,25,1,0,"area",95-33*math.pi/16),
    "concave-outer-curved-holes": (29,29,1,0,"area",94-33*math.pi/16),
    "oblique-convex-curved-holes": (25,25,1,0,"area",95-33*math.pi/16),
    "circle-outer-curved-hole": (9,9,1,0,"area",99*math.pi-8),
    "segment-hole": (6,6,1,0,"area",16-math.pi/2),
    "segment-hole-seam": (6,6,1,0,"area",16-math.pi/2),
    "major-segment-hole": (6,6,1,0,"area",15.5-3*math.pi/4),
    "major-segment-hole-seam": (6,6,1,0,"area",15.5-3*math.pi/4),
    "rounded-bore-prism": (24,36,14,1,"volume",96-2*math.pi),
    "oblique-rounded-bore-prism": (24,36,14,1,"volume",96-2*math.pi),
    "rounded-polygon-holes": (16,16,1,0,"area",7+math.pi),
    "rounded-polygon-holes-first": (16,16,1,0,"area",7+math.pi),
    "rounded-polygon-holes-inward": (16,16,1,0,"area",7+math.pi),
    "rounded-concave-polygon-hole": (14,14,1,0,"area",6.5+math.pi),
    "rounded-mixed-holes": (17,17,1,0,"area",7+15*math.pi/16),
    "rounded-mixed-holes-first": (17,17,1,0,"area",7+15*math.pi/16),
    "rounded-mixed-holes-seams": (17,17,1,0,"area",7+15*math.pi/16),
    "rounded-mixed-holes-inward": (17,17,1,0,"area",7+15*math.pi/16),
    "oblique-rounded-mixed-holes": (17,17,1,0,"area",7+15*math.pi/16),
    "rounded-concave-mixed-holes": (15,15,1,0,"area",6.5+99*math.pi/100),
    "cylinder-sector-polygon-bore": (14,21,9,1,"volume",math.pi/2-2/25),
    "oblique-cylinder-sector-polygon-bore": (14,21,9,1,"volume",math.pi/2-2/25),
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
    "spherical-spun-cap": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-spun-cap-reversed": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
    "spherical-spun-cap-major": (2, 3, 2, 1, "volume", 448 * math.pi / 3),
    "spherical-spun-cap-oblique": (2, 3, 2, 1, "volume", 52 * math.pi / 3),
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

# OCCT reconstructs an oblique ellipse's cylindrical pcurve as a B-spline.
# Sampling that imported pcurve against the unchanged exact 3D edge exposes
# about 1.5e-4 mm residual for these R=3 mm fixtures, even with read precision
# set to 1e-11. Its area/volume integration therefore needs a separate tolerance.
# All other cases retain the strict measure check. Exact exporter flux and
# source-curve area formulas are checked independently in export_test.cc.
SOURCE_TOLERANT_CIRCLE_RADII = {name: [3]*4 + [4]*4 for name in (
    "tolerant-torus", "tolerant-torus-axis", "tolerant-torus-profile",
    "tolerant-torus-translated")}
SOURCE_TOLERANT_CIRCLE_RADII.update({name: [3,4] for name in (
    "closed-tolerant-torus", "closed-tolerant-torus-named",
    "closed-tolerant-torus-clamped", "closed-tolerant-torus-axis",
    "closed-tolerant-torus-profile", "closed-tolerant-torus-translated",
    "closed-tolerant-torus-shifted", "closed-tolerant-torus-shifted-named")})
IMPORTED_PCURVE_REL_TOL = {
    "profiled-cylinder": 5e-6,
    "profiled-cylinder-seams": 5e-6,
    "profiled-cylinder-two-cuts": 5e-6,
    "profiled-cylinder-translated": 5e-6,
    "profiled-cylinder-mantle": 5e-6,
}

def check_source_spline(name, shape):
    """Check retained basis data and source parameter values independently."""
    rational = "rational" in name
    multi = "multi" in name
    if multi:
        poles = [(0,0,0), (1,1,0), (2,-1,1), (3,1,1), (4,0,0)]
        weights = [1,2,1,2,1] if rational else [1]*5
        knots, multiplicities = [2,3,5,7], [3,1,1,3]
    else:
        poles = [(1,0,0), (1,1,0), (0,1,0)] if rational else [(0,0,0), (1,1,0), (2,0,0)]
        weights = [1,math.sqrt(.5),1] if rational else [1]*3
        knots, multiplicities = [2,7], [3,3]
    unclamped = "unclamped" in name
    if unclamped:
        if "inactive" in name:
            knots, multiplicities = [-1,2,7,10], [1,2,2,1]
        else:
            poles = [(0,0,0), (1,.1,0), (2,-.1,.1), (3,0,0)]
            weights = [1,2,3,1] if rational else [1]*4
            knots, multiplicities = [-2,0,2,3,7,9,11], [1]*7
    if "translated" in name:
        poles = [(y+10,z-20,x+30) for x,y,z in poles]
    expanded = [k for k,m in zip(knots,multiplicities) for _ in range(m)]
    # Cox basis recurrence is independent of the reader's homogeneous de Boor
    # evaluator. Include interior knots and values on both adjacent spans.
    def basis(i, degree, t):
        if degree == 0:
            if t == expanded[len(poles)]:
                # The reader uses the left polynomial at the natural end,
                # even when exterior knots continue beyond that parameter.
                return float(expanded[i] < t <= expanded[i+1])
            return float(expanded[i] <= t < expanded[i+1])
        left = expanded[i+degree] - expanded[i]
        right = expanded[i+degree+1] - expanded[i+1]
        return ((t-expanded[i])/left*basis(i, degree-1, t) if left else 0) + ((expanded[i+degree+1]-t)/right*basis(i+1, degree-1, t) if right else 0)

    trimmed = "trimmed" in name
    intervals = [(2,7)]
    if trimmed:
        intervals = [(3,5)] if "knots" in name else [(2.5,6)]
        if unclamped:
            intervals = [(2.5,6.5)]
        if "pair" in name:
            intervals = [(2.25,3.5), (4.5,6.75)]
    edges = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
    observed = []
    for index in range(1, edges.Extent()+1):
        curve = BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(index)))
        assert curve.GetType() == GeomAbs_BSplineCurve, name
        spline = curve.BSpline()
        low, high = curve.FirstParameter(), curve.LastParameter()
        observed.append((low,high))
        assert (low,high) in intervals, (name,low,high)
        kept_knots = [low]+[k for k in knots if low < k < high]+[high]
        kept_counts = [3]+[m for k,m in zip(knots,multiplicities) if low < k < high]+[3]
        assert spline.Degree() == 2 and spline.NbPoles() == sum(kept_counts)-3, name
        assert spline.NbKnots() == len(kept_knots) and spline.IsRational() == rational, name
        assert not spline.IsPeriodic() and not spline.IsClosed(), name
        for i, (knot, count) in enumerate(zip(kept_knots, kept_counts), 1):
            assert spline.Knot(i) == knot and spline.Multiplicity(i) == count, name
        if not trimmed and not unclamped:
            for i, (pole, weight) in enumerate(zip(poles, weights), 1):
                actual = spline.Pole(i)
                assert math.isclose(spline.Weight(i), weight, rel_tol=1e-14), name
                assert max(abs(a-b) for a,b in zip((actual.X(), actual.Y(), actual.Z()), pole)) < 1e-13, name
        samples = {low+(high-low)*i/40 for i in range(41)}
        samples.update(t for t in (2.99,3,3.01,4.99,5,5.01) if low <= t <= high)
        for t in sorted(samples):
            coefficients = [basis(i, 2, t)*w for i,w in enumerate(weights)]
            expected = [sum(c*p[axis] for c,p in zip(coefficients, poles))/sum(coefficients) for axis in range(3)]
            actual = curve.Value(t)
            assert max(abs(a-b) for a,b in zip((actual.X(),actual.Y(),actual.Z()),expected)) < 1e-12, (name,t)
    assert sorted(observed) == sorted(intervals), (name,observed)

def check_quadratic_face(name, shape):
    edges = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
    splines = []
    for i in range(1,edges.Extent()+1):
        curve = BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(i)))
        if curve.GetType() == GeomAbs_BSplineCurve:
            splines.append(curve)
    assert len(splines) == 1, name
    curve = splines[0]
    spline = curve.BSpline()
    count = 3
    if "multi" in name and "outside-knot" not in name:
        count = 5 if "repeated" in name else 4
    assert spline.Degree() == 2 and spline.NbPoles() == count, name
    assert not spline.IsRational(), name
    low,high = (2.5,6.5) if "partial" in name else (2,7)
    assert curve.FirstParameter() == low and curve.LastParameter() == high, name
    sample_parameters = {low+(high-low)*i/40 for i in range(41)}
    if count > 3:
        split = .3 if "nonuniform" in name else .5
        knot = 2+5*(1-split if "reversed" in name else split)
        assert spline.NbKnots() == 3, name
        assert math.isclose(spline.Knot(2), knot, abs_tol=1e-14), name
        assert spline.Multiplicity(2) == (2 if "repeated" in name else 1), name
        sample_parameters.update((knot-1e-6,knot,knot+1e-6))
    else:
        assert spline.NbKnots() == 2, name
    for t in sorted(sample_parameters):
        u = (t-2)/5
        x,y,z = 1-2*u,4*u*(1-u),0
        if "reversed" in name:
            x = -x
        if "oblique" in name:
            x,y,z = y+1e6,z-2e6,x+3e6
        actual = curve.Value(t)
        tolerance = 1e-9 if "oblique" in name else 1e-12
        assert max(abs(a-b) for a,b in zip((actual.X(),actual.Y(),actual.Z()),(x,y,z))) < tolerance, (name,t)

def check_cubic_face(name,shape):
    edges=TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape,TopAbs_EDGE,edges)
    splines=[]
    for i in range(1,edges.Extent()+1):
        curve=BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(i)))
        if curve.GetType()==GeomAbs_BSplineCurve:
            splines.append(curve)
    assert len(splines)==1,name
    curve=splines[0]
    spline=curve.BSpline()
    multiplicity=next((m for word,m in (("one",1),("two",2),("three",3)) if "multi-"+word in name),0)
    pole_count=8 if "three-span" in name else 4+multiplicity
    assert spline.Degree()==3 and spline.NbPoles()==pole_count,name
    assert not spline.IsRational(),name
    low,high=(2.5,6.5) if "partial" in name else (2,7)
    assert curve.FirstParameter()==low and curve.LastParameter()==high,name
    samples={low+(high-low)*i/40 for i in range(41)}
    if "three-span" in name:
        assert spline.NbKnots()==4,name
        for i,knot in enumerate((3.25,5.75),2):
            assert spline.Knot(i)==knot and spline.Multiplicity(i)==2,name
            samples.update((knot-1e-6,knot,knot+1e-6))
    elif multiplicity:
        knot=3.5 if "nonuniform" in name else 4.5
        assert spline.NbKnots()==3 and spline.Multiplicity(2)==multiplicity,name
        assert math.isclose(spline.Knot(2),knot,abs_tol=1e-14),name
        samples.update((knot-1e-6,knot,knot+1e-6))
    else:
        assert spline.NbKnots()==2,name
    for t in sorted(samples):
        u=(t-2)/5
        if "reversed" in name:
            u=1-u
        if "elevated" in name:
            x,y=1-2*u,4*u*(1-u)
        elif "middle-zero" in name:
            x,y=1-3*u+3*u*u-2*u**3,3*u*(1-u)
        else:
            x,y=1-1.5*u-1.5*u*u+u**3,6*u-9*u*u+3*u**3
        z=0
        if "oblique" in name:
            x,y,z=y+1e6,z-2e6,x+3e6
        point=curve.Value(t)
        tolerance=1e-9 if "oblique" in name else 1e-12
        assert max(abs(a-b) for a,b in zip((point.X(),point.Y(),point.Z()),(x,y,z)))<tolerance,(name,t)

def check_spline_hole(name, shape):
    edges = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
    shifts = set()
    for index in range(1, edges.Extent()+1):
        curve = BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(index)))
        if curve.GetType() != GeomAbs_BSplineCurve:
            continue
        spline = curve.BSpline()
        degree = 3 if "cubic" in name else 2
        extra = next((m for word,m in (("one",1),("two",2),("three",3))
                      if "multi-"+word in name), 0)
        if "multi-repeated" in name:
            extra = 2
        if "multi-simple" in name:
            extra = 1
        assert spline.Degree() == degree and spline.NbPoles() == degree+1+extra, name
        assert not spline.IsRational(), name
        low,high = (2.5,6.5) if "partial" in name else (2,7)
        assert curve.FirstParameter() == low and curve.LastParameter() == high, name
        samples = {low+(high-low)*i/40 for i in range(41)}
        if extra:
            assert spline.NbKnots() == 3 and spline.Knot(2) == 4.5, name
            assert spline.Multiplicity(2) == extra, name
            samples.update((4.5-1e-6,4.5,4.5+1e-6))
        else:
            assert spline.NbKnots() == 2, name
        middle = curve.Value(4.5)
        displacement = middle.Z()-3e6 if "oblique" in name else middle.X()
        shift = 6 if displacement > 3 else 0
        assert shift not in shifts, name
        shifts.add(shift)
        for t in sorted(samples):
            u = (t-2)/5
            if "reversed" in name:
                u = 1-u
            if degree == 2:
                x,y = 1-2*u,4*u*(1-u)
            else:
                x,y = 1-1.5*u-1.5*u*u+u**3,6*u-9*u*u+3*u**3
            x += shift
            z = 0
            if "oblique" in name:
                x,y,z = y+1e6,z-2e6,x+3e6
            actual = curve.Value(t)
            tolerance = 1e-9 if "oblique" in name else 1e-12
            assert max(abs(a-b) for a,b in zip((actual.X(),actual.Y(),actual.Z()),(x,y,z))) < tolerance, (name,t)
    assert shifts == ({0,6} if "-two" in name and "multi-two" not in name else {0}), name

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
        if name.startswith("spline-"):
            check_source_spline(name, shape)
        if name.startswith("hole-spline-"):
            check_spline_hole(name, shape)
        if name.startswith("quadratic-face"):
            check_quadratic_face(name, shape)
        if name.startswith("cubic-face"):
            check_cubic_face(name,shape)
        if name in SOURCE_TOLERANT_CIRCLE_RADII:
            edges = TopTools_IndexedMapOfShape()
            TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
            # All original 3D master arcs/rings remain exact circles. The
            # additional meridian seam is generated by OCCT as a B-spline.
            radii = []
            for index in range(1, edges.Extent() + 1):
                curve = BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(index)))
                if curve.GetType() == GeomAbs_Circle:
                    radii.append(curve.Circle().Radius())
            expected_radii = SOURCE_TOLERANT_CIRCLE_RADII[name]
            assert len(radii) == len(expected_radii), (name, radii)
            for actual, target in zip(sorted(radii), expected_radii):
                assert math.isclose(actual, target, rel_tol=1e-12), (name, radii)
        measure, target = expected[4:]
        if measure:
            properties = GProp_GProps()
            operation = {"volume": BRepGProp.VolumeProperties_s,
                         "area": BRepGProp.SurfaceProperties_s,
                         "length": BRepGProp.LinearProperties_s}[measure]
            # Oblique periodic trims need adaptive quadrature; fixed-order
            # integration can miss their volume despite valid exact geometry.
            if measure in ("area", "volume"):
                operation(shape, properties, Eps=1e-12)
            else:
                operation(shape, properties)
            actual = properties.Mass()
            if name.startswith("spline-"):
                # LinearProperties uses fixed-order quadrature. Adaptive
                # curve-length integration retains the analytic 1e-9 check.
                edges = TopTools_IndexedMapOfShape()
                TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
                curve = BRepAdaptor_Curve(TopoDS.Edge_s(edges.FindKey(1)))
                actual = GCPnts_AbscissaPoint.Length_s(curve, curve.FirstParameter(), curve.LastParameter(), 1e-12)
            assert math.isclose(actual, target, rel_tol=IMPORTED_PCURVE_REL_TOL.get(name, 1e-9), abs_tol=1e-12), (name, actual, target)
        print(name, "valid, topology and measures verified")
