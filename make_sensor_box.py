"""Sensor box: a square tube whose four long faces are swirly-grid plates, plus two end caps.

Run with freecadcmd (see README). Also imported by make_gimbal.py.

Each face is a 3 x 5 cell grid (45.72 x 76.2 mm). The roll axis runs along the
tube (+X).
  +Z face: the FeatherWing (on 10 mm standoffs, which clears header pins) + 1 portrait QT board
  -Z, +Y, -Y faces: 4 portrait QT boards each (edge to edge; connectors face the long edges)
That is 13 QT boards + 1 FeatherWing. PR 839 needs 9 breakouts + the wing.

Boards bolt through the slots with M2.5 screws and nuts on the inside, so
mount them before fitting the caps. Corner posts run the length of the
tube; each cap screws to them with 4 x M2.5. The +X cap takes the MG90S roll
horn and the -X cap has the idler axle boss. Both caps have cable windows
into the tube, which has room for a QT Py + I2C mux.
"""

import itertools
import os
import sys

import FreeCAD as App
import Part
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swirly_grid as sg  # noqa: E402
import pack_faces as pf  # noqa: E402
from make_swirly_plate import slot_cutters, SLOT_WIDTH  # noqa: E402

V = App.Vector

ROWS, COLS = 3, 5
GRID_L, GRID_W = COLS * sg.CELL, ROWS * sg.CELL      # 76.2 x 45.72
MARGIN_X = 2.54                                      # grid to tube end
MARGIN_LAT = 5.08                                    # grid to tube edge; leaves room for corner posts
WALL = 3.0
TUBE_L = GRID_L + 2 * MARGIN_X                       # 81.28
TUBE_W = GRID_W + 2 * MARGIN_LAT                     # 55.88
POST = 5.8                                           # corner post square, from the outer edges
POST_PILOT = 2.2                                     # M2.5 self-tap
CAP_T = 4.0
CAP_SCREW_CLEAR = 2.8
WINDOW = (8.0, 20.0)                                 # cable window in each cap (lateral, vertical)

QT_SPACER = 3.0
WING_STANDOFF = 10.0
PCB_T = 1.6
PARTS_H = 2.5                                        # components on top of boards
PLUG = (6.0, 8.0, 3.5)                               # QT plug envelope: width, length past edge, height

FACES = {  # name: outward normal
    "top": (0, 0, 1),
    "bottom": (0, 0, -1),
    "left": (0, 1, 0),
    "right": (0, -1, 0),
}


def face_frame(normal):
    """(origin, xdir, zdir) mapping grid-local (u, v, w) -> world. w = outward from outer surface."""
    n = V(*normal)
    x = V(1, 0, 0)
    y = n.cross(x)
    origin = n * (TUBE_W / 2) + x * (-TUBE_L / 2 + MARGIN_X) + y * (-TUBE_W / 2 + MARGIN_LAT)
    return origin, x, n


def placed(shape, origin, xdir, zdir):
    xdir, zdir = V(xdir).normalize(), V(zdir).normalize()
    ydir = zdir.cross(xdir)
    m = App.Matrix(xdir.x, ydir.x, zdir.x, origin.x,
                   xdir.y, ydir.y, zdir.y, origin.y,
                   xdir.z, ydir.z, zdir.z, origin.z,
                   0, 0, 0, 1)
    s = shape.copy()
    s.transformShape(m)
    return s


def tube():
    outer = Part.makeBox(TUBE_L, TUBE_W, TUBE_W, V(-TUBE_L / 2, -TUBE_W / 2, -TUBE_W / 2))
    inner = Part.makeBox(TUBE_L + 2, TUBE_W - 2 * WALL, TUBE_W - 2 * WALL,
                         V(-TUBE_L / 2 - 1, -TUBE_W / 2 + WALL, -TUBE_W / 2 + WALL))
    shape = outer.cut(inner)
    posts, pilots = [], []
    h = TUBE_W / 2
    for sy in (-1, 1):
        for sz in (-1, 1):
            y0 = sy * h if sy < 0 else h - POST
            z0 = sz * h if sz < 0 else h - POST
            posts.append(Part.makeBox(TUBE_L, POST, POST, V(-TUBE_L / 2, y0, z0)))
            cy, cz = sy * (h - POST / 2 - 0.5), sz * (h - POST / 2 - 0.5)
            pilots.append(Part.makeCylinder(POST_PILOT / 2, TUBE_L + 2, V(-TUBE_L / 2 - 1, cy, cz), V(1, 0, 0)))
    shape = shape.fuse(posts)
    cutters = []
    for normal in FACES.values():
        o, x, n = face_frame(normal)
        for c in slot_cutters(ROWS, COLS, -WALL - 0.01, WALL + 1.02, SLOT_WIDTH):
            cutters.append(placed(c, o, x, n))
    return shape.cut(Part.makeCompound(cutters + pilots)).removeSplitter()


def cap_screw_points():
    h = TUBE_W / 2 - POST / 2 - 0.5
    return [(sy * h, sz * h) for sy in (-1, 1) for sz in (-1, 1)]


def end_cap(sign):
    """sign=+1 for the +X cap. Returns the cap solid in world coords (outer face at x = sign*(TUBE_L/2+CAP_T))."""
    x_in = sign * TUBE_L / 2
    x0 = min(x_in, x_in + sign * CAP_T)
    cap = Part.makeBox(CAP_T, TUBE_W, TUBE_W, V(x0, -TUBE_W / 2, -TUBE_W / 2))
    cuts = [Part.makeCylinder(CAP_SCREW_CLEAR / 2, CAP_T + 2, V(x0 - 1, y, z), V(1, 0, 0))
            for y, z in cap_screw_points()]
    wy, wz = WINDOW
    for sy in (-1, 1):
        w = Part.makeBox(CAP_T + 2, wy, wz, V(x0 - 1, sy * 16 - wy / 2, -wz / 2))
        cuts.append(w.makeFillet(2.0, [e for e in w.Edges if e.BoundBox.XLength > CAP_T]))
    return cap.cut(Part.makeCompound(cuts)).removeSplitter()


def qt_layout(n=4):
    """Most centred row of n portrait QT boards on a face, as grid-local lower-left corners."""
    opts = pf.board_options(ROWS, COLS)
    xs = sorted({o[0] for o in opts})
    best = None
    for combo in itertools.combinations(xs, n):
        if all(b - a >= pf.QT_W - 1e-6 for a, b in zip(combo, combo[1:])):
            slack = abs((combo[0] + combo[-1] + pf.QT_W) / 2 - GRID_L / 2)
            if best is None or slack < best[0]:
                best = (slack, combo)
    picks = []
    for x in best[1]:
        o = min((o for o in opts if o[0] == x), key=lambda o: abs(o[1] + pf.QT_H / 2 - GRID_W / 2))
        picks.append(o)
    return picks


def feather_layout():
    """Feather lower-left on the top face, plus one portrait QT beside it."""
    holes = sg.rect_holes(45.72, 17.78)
    fits = [(dx - 2.54, dy - 2.54) for q, (dx, dy) in sg.find_fits(holes, ROWS, COLS) if q == 0]
    opts = pf.board_options(ROWS, COLS)
    best = None
    for fx, fy in fits:
        if fx < -0.01 or fy < -MARGIN_LAT + 1 or fy + 22.86 > GRID_W + MARGIN_LAT - 1:
            continue
        for o in opts:
            if o[0] >= fx + 50.8 - 1e-6 and o[0] + pf.QT_W <= GRID_L + MARGIN_X:
                span = o[0] + pf.QT_W - fx
                slack = (abs(fx + span / 2 - GRID_L / 2) + abs(fy + 11.43 - GRID_W / 2)
                         + abs(o[1] + pf.QT_H / 2 - GRID_W / 2))
                if best is None or slack < best[0]:
                    best = (slack, (round(fx, 2), round(fy, 2)), o)
    return best[1], best[2]


def board_envelope(w, h, lift, plugs_along_v=True):
    """Board on standoffs in grid-local coords at origin, plugs on the short (v) edges."""
    board = Part.makeBox(w, h, PCB_T + PARTS_H, V(0, 0, lift))
    if plugs_along_v:
        pw, pl, ph = PLUG
        for v0 in (-pl, h):
            board = board.fuse(Part.makeBox(pw, pl, ph, V(w / 2 - pw / 2, v0, lift + PCB_T)))
    return board


def boards():
    """Reference envelopes for every board position, in world coords."""
    out = []
    qt = board_envelope(pf.QT_W, pf.QT_H, QT_SPACER)
    for name, normal in FACES.items():
        o, x, n = face_frame(normal)
        if name == "top":
            (fx, fy), q = feather_layout()
            wing = Part.makeBox(50.8, 22.86, PCB_T + PARTS_H, V(fx, fy, WING_STANDOFF))
            out.append(("FeatherWing", placed(wing, o, x, n)))
            b = qt.copy()
            b.translate(V(q[0], q[1], 0))
            out.append(("QT_top_1", placed(b, o, x, n)))
            continue
        for i, (bx, by, _) in enumerate(qt_layout(), 1):
            b = qt.copy()
            b.translate(V(bx, by, 0))
            out.append((f"QT_{name}_{i}", placed(b, o, x, n)))
    return out


def sensor_box():
    return tube(), end_cap(+1), end_cap(-1)


if __name__ == "__main__":
    t, cp, cn = sensor_box()
    for name, s in (("tube", t), ("cap+X", cp), ("cap-X", cn)):
        print(name, "valid", s.isValid(), "solids", len(s.Solids),
              "bbox", [round(v, 2) for v in (s.BoundBox.XLength, s.BoundBox.YLength, s.BoundBox.ZLength)], flush=True)
    print("qt layout", qt_layout(), flush=True)
    print("feather layout", feather_layout(), flush=True)
    bs = boards()
    allb = Part.makeCompound([s for _, s in bs])
    print("boards", len(bs), "overlap with tube", round(allb.common(t).Volume, 3), flush=True)
    for (na, a), (nb, b) in itertools.combinations(bs, 2):
        v = a.common(b).Volume
        if v > 1e-3:
            print("BOARD OVERLAP", na, nb, round(v, 2), flush=True)
