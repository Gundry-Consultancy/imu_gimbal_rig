"""Sensor carrier: swirly-grid plate plus one set of Feather screw bosses. Run with freecadcmd.

The bosses are placed so that the ISM330DHCX on the ISM330DHCX + LIS3MDL
FeatherWing (#4569) sits over the plate centre, which is where the gimbal axes
will cross. Slots are kept everywhere the Feather footprint doesn't cover, so
the same carrier still takes STEMMA QT boards or a swirly-grid PCB.

Bosses are solid down to the plate underside (they fill any slot beneath
them) with a blind pilot hole for M2.5 self-tapping screws or a heat-set
insert. The default 9 mm height clears male header pins, which stick out about
8.5 mm below the wing. Set CARRIER_BOSS_H=4 for a wing without headers.
"""

import os
import sys

import FreeCAD as App
import Part
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swirly_grid as sg  # noqa: E402
from make_swirly_plate import plate, THICKNESS  # noqa: E402

ROWS, COLS = 2, 4

# Feather form factor (mm, from board lower-left). Adafruit Feather spec / .brd.
FEATHER_W, FEATHER_H = 50.80, 22.86
FEATHER_HOLES = ((2.54, 2.54), (48.26, 2.54), (48.26, 20.32), (2.54, 20.32))
WING_IC = (27.94, 11.43)  # ISM330DHCX on #4569

BOSS_H = float(os.environ.get("CARRIER_BOSS_H", 9.0))  # above plate top
BOSS_OD = 6.5
PILOT_D = 2.2     # M2.5 self-tapping into PLA/PETG; use 3.6 for an M2.5 heat-set insert
PILOT_DEPTH = 7.0


def feather_origin():
    """Board lower-left on the plate, putting the wing IC at plate centre."""
    w, h = sg.plate_size(ROWS, COLS)
    return w / 2 - WING_IC[0], h / 2 - WING_IC[1]


def carrier():
    ox, oy = feather_origin()
    shape = plate(ROWS, COLS)
    top = THICKNESS + BOSS_H
    bosses, pilots = [], []
    for hx, hy in FEATHER_HOLES:
        x, y = ox + hx, oy + hy
        bosses.append(Part.makeCylinder(BOSS_OD / 2, top, App.Vector(x, y, 0)))
        pilots.append(Part.makeCylinder(PILOT_D / 2, PILOT_DEPTH + 1, App.Vector(x, y, top - PILOT_DEPTH)))
    shape = shape.fuse(bosses).cut(Part.makeCompound(pilots)).removeSplitter()
    return shape


if __name__ == "__main__":
    name = f"carrier-swirly-{ROWS}x{COLS}-feather"
    shape = carrier()
    ox, oy = feather_origin()
    print(name, "valid", shape.isValid(), "solids", len(shape.Solids),
          "bbox", [round(v, 2) for v in (shape.BoundBox.XLength, shape.BoundBox.YLength, shape.BoundBox.ZLength)],
          "feather_lower_left", (round(ox, 3), round(oy, 3)), flush=True)
    assert shape.isValid() and len(shape.Solids) == 1

    doc = App.newDocument(name.replace("-", "_"))
    obj = doc.addObject("Part::Feature", "Carrier")
    obj.Label = "Swirly carrier with Feather bosses (wing IC at plate centre)"
    obj.Shape = shape
    obj.Visibility = True
    ghost = doc.addObject("Part::Feature", "FeatherWingEnvelope")
    ghost.Label = "FeatherWing #4569 envelope (reference only)"
    ghost.Shape = Part.makeBox(FEATHER_W, FEATHER_H, 1.6, App.Vector(ox, oy, THICKNESS + BOSS_H))
    ghost.Visibility = True
    base = os.path.join(HERE, name)
    doc.saveAs(base + ".FCStd")
    shape.exportStep(base + ".step")
    Mesh.export([obj], base + ".stl")
    print("wrote", base + ".{FCStd,step,stl}", flush=True)
