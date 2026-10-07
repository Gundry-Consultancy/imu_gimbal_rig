"""Build a 3D-printable swirly-grid plate. Run with freecadcmd.

    freecadcmd -c "exec(open('make_swirly_plate.py').read(), {'__file__': 'make_swirly_plate.py'})"

Writes swirly-plate-<rows>x<cols>.FCStd / .step / .stl next to this script.
"""

import os
import sys

import FreeCAD as App
import Part
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swirly_grid as sg  # noqa: E402

ROWS = int(os.environ.get("SWIRLY_ROWS", 2))
COLS = int(os.environ.get("SWIRLY_COLS", 4))
THICKNESS = 3.0
# FDM shrinks holes; 2.54 mm nominal prints too tight for M2.5.
SLOT_WIDTH = float(os.environ.get("SWIRLY_SLOT", 2.75))


def plate(rows, cols, thickness=THICKNESS, slot_width=SLOT_WIDTH):
    w, h = sg.plate_size(rows, cols)
    base = Part.makeBox(w, h, thickness)
    vertical = [e for e in base.Edges if e.BoundBox.ZLength > thickness - 0.01]
    base = base.makeFillet(sg.CORNER_RADIUS, vertical)

    r = slot_width / 2
    z0, depth = -1.0, thickness + 2.0
    cutters = []
    for (ax, ay), (bx, by) in sg.slots(rows, cols):
        cutters.append(Part.makeCylinder(r, depth, App.Vector(ax, ay, z0)))
        if (ax, ay) == (bx, by):
            continue
        cutters.append(Part.makeCylinder(r, depth, App.Vector(bx, by, z0)))
        x0, x1 = min(ax, bx), max(ax, bx)
        y0, y1 = min(ay, by), max(ay, by)
        if ay == by:
            cutters.append(Part.makeBox(x1 - x0, slot_width, depth, App.Vector(x0, ay - r, z0)))
        else:
            cutters.append(Part.makeBox(slot_width, y1 - y0, depth, App.Vector(ax - r, y0, z0)))
    return base.cut(Part.makeCompound(cutters)).removeSplitter()


if __name__ == "__main__" or True:
    name = f"swirly-plate-{ROWS}x{COLS}"
    shape = plate(ROWS, COLS)
    print(name, "valid", shape.isValid(), "solids", len(shape.Solids),
          "bbox", [round(v, 2) for v in (shape.BoundBox.XLength, shape.BoundBox.YLength, shape.BoundBox.ZLength)],
          "volume", round(shape.Volume, 1), flush=True)
    assert shape.isValid() and len(shape.Solids) == 1

    doc = App.newDocument(name.replace("-", "_"))
    obj = doc.addObject("Part::Feature", "SwirlyPlate")
    obj.Label = f"Swirly grid plate {ROWS}x{COLS} ({SLOT_WIDTH} mm slots)"
    obj.Shape = shape
    obj.Visibility = True
    base = os.path.join(HERE, name)
    doc.saveAs(base + ".FCStd")
    shape.exportStep(base + ".step")
    Mesh.export([obj], base + ".stl")
    print("wrote", base + ".{FCStd,step,stl}", flush=True)
