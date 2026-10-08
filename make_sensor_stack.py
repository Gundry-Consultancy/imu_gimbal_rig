"""Sensor stack: layers of horizontal swirly-grid decks on corner pillars, open sides,
with a small roof in the middle of the top for the tap striker to hit.

Imported by make_gimbal.py, which also places it on the roll axis.

Decks are 4 x 4 cell swirly grids (60.96 mm square) with an 8 mm border for
the corner pillars. From the bottom:
  deck 0 (cradle)  6 portrait QT boards. Also the roll cradle: its +X / -X end
                   walls carry the MG90S horn and the idler axle.
  deck 1           FeatherWing on 10 mm standoffs + 3 portrait QT boards
  deck 2 (top)     6 portrait QT boards around a central post with a flared
                   roof for whacking on
That is 15 QT boards + 1 FeatherWing. PR 839 needs 9 breakouts + the wing.

Boards sit portrait in two rows. Each board uses the QT connector on its
outside edge, so the cables leave through the open +Y / -Y sides.

The pillars are printed onto the top of each deck below, so every part prints
upright without supports. An M3 threaded rod through each corner clamps the
stack (nuts under deck 0 and on top of deck 2).
"""

import itertools
import os
import sys

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swirly_grid as sg  # noqa: E402
import pack_faces as pf  # noqa: E402
from make_swirly_plate import slot_cutters, SLOT_WIDTH  # noqa: E402

V = App.Vector

ROWS = COLS = 4
GRID = COLS * sg.CELL                 # 60.96
BORDER = 8.0
DECK = GRID + 2 * BORDER              # 76.96 square
DECK_T = 3.0
FLOOR_T = 4.0                         # deck 0 is also the cradle floor
ROD_HOLE = 3.4
PILLAR_OD = 8.0
PILLAR_INSET = BORDER / 2             # pillar centre from deck edge

QT_SPACER = 3.0
WING_STANDOFF = 10.0
PCB_T = 1.6
PARTS_H = 2.5
PLUG = (6.0, 8.0, 3.5)               # QT plug envelope: width, length past the board edge, height
HEADROOM = 3.0                        # above the tallest thing on a deck

QT_STACK_H = QT_SPACER + PCB_T + PLUG[2]
WING_STACK_H = WING_STANDOFF + PCB_T + PARTS_H
GAPS = (QT_STACK_H + HEADROOM, WING_STACK_H + HEADROOM)   # clear height above deck 0, deck 1

POST = 9.0                            # roof post square
ROOF = 24.0                           # roof square
ROOF_T = 3.0
ROOF_H = QT_SPACER + PCB_T + PARTS_H + 1.0 + (ROOF - POST) / 2 + ROOF_T  # flare starts above the boards

WALL_T = 4.0
WALL_HALF_Y = 19.0
HUB_R = 11.0


def deck_z():
    """(bottom, top) of each deck, with the deck 0 underside at z = 0."""
    z = [(0.0, FLOOR_T)]
    for gap in GAPS:
        b = z[-1][1] + gap
        z.append((b, b + DECK_T))
    return z


def roof_top():
    return deck_z()[-1][1] + ROOF_H


def axis_z():
    """Roll axis height above the deck 0 underside: middle of the stack."""
    return roof_top() / 2


def body_half_x():
    return DECK / 2 + WALL_T


def pillar_points():
    h = DECK / 2 - PILLAR_INSET
    return [(sx * h, sy * h) for sx in (-1, 1) for sy in (-1, 1)]


def grid_origin(z_top):
    """Grid lower-left in stack coords (deck centred on x = y = 0)."""
    return V(-GRID / 2, -GRID / 2, z_top)


def deck_plate(z0, z1):
    plate = Part.makeBox(DECK, DECK, z1 - z0, V(-DECK / 2, -DECK / 2, z0))
    plate = plate.makeFillet(3.0, [e for e in plate.Edges if e.BoundBox.ZLength > (z1 - z0) - 0.01])
    cuts = []
    for c in slot_cutters(ROWS, COLS, -1.0, (z1 - z0) + 2.0, SLOT_WIDTH):
        c.translate(V(-GRID / 2, -GRID / 2, z0))
        cuts.append(c)
    for x, y in pillar_points():
        cuts.append(Part.makeCylinder(ROD_HOLE / 2, (z1 - z0) + 2, V(x, y, z0 - 1)))
    return plate.cut(Part.makeCompound(cuts))


def pillars(z0, z1):
    out = []
    for x, y in pillar_points():
        p = Part.makeCylinder(PILLAR_OD / 2, z1 - z0, V(x, y, z0))
        out.append(p.cut(Part.makeCylinder(ROD_HOLE / 2, z1 - z0 + 2, V(x, y, z0 - 1))))
    return out


def roof_post(z0):
    """Square post that flares at 45 degrees into the roof, so it prints without support."""
    z_roof_top = z0 + ROOF_H
    z_flare_top = z_roof_top - ROOF_T
    flare = (ROOF - POST) / 2
    z_flare_bot = z_flare_top - flare
    post = Part.makeBox(POST, POST, z_flare_bot - z0 + 0.01, V(-POST / 2, -POST / 2, z0))
    lo = Part.makePolygon([V(-POST / 2, -POST / 2, z_flare_bot), V(POST / 2, -POST / 2, z_flare_bot),
                           V(POST / 2, POST / 2, z_flare_bot), V(-POST / 2, POST / 2, z_flare_bot),
                           V(-POST / 2, -POST / 2, z_flare_bot)])
    hi = Part.makePolygon([V(-ROOF / 2, -ROOF / 2, z_flare_top), V(ROOF / 2, -ROOF / 2, z_flare_top),
                           V(ROOF / 2, ROOF / 2, z_flare_top), V(-ROOF / 2, ROOF / 2, z_flare_top),
                           V(-ROOF / 2, -ROOF / 2, z_flare_top)])
    taper = Part.makeLoft([lo, hi], True)
    roof = Part.makeBox(ROOF, ROOF, ROOF_T, V(-ROOF / 2, -ROOF / 2, z_flare_top))
    return post.fuse([taper, roof])


def cradle_deck(horn_cutter, axle_boss, axle_pilot):
    """Deck 0 with the roll end walls. Cutters/boss are given in stack coords by make_gimbal."""
    (z0, z1), (z1b, _) = deck_z()[0], deck_z()[1]
    shape = deck_plate(z0, z1).fuse(pillars(z1, z1b))
    za = axis_z()
    x_in = DECK / 2 - 0.01
    walls = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * x_in, sx * (DECK / 2 + WALL_T)))
        w = Part.makeBox(x1 - x0, 2 * WALL_HALF_Y, za - z0, V(x0, -WALL_HALF_Y, z0))
        hub = Part.makeCylinder(HUB_R, x1 - x0, V(x0, 0, za), V(1, 0, 0))
        walls.append(w.fuse(hub))
    shape = shape.fuse(walls + [axle_boss]).cut(Part.makeCompound([horn_cutter, axle_pilot]))
    return shape.removeSplitter()


def middle_deck():
    (z0, z1), (z2, _) = deck_z()[1], deck_z()[2]
    return deck_plate(z0, z1).fuse(pillars(z1, z2)).removeSplitter()


def top_deck():
    z0, z1 = deck_z()[2]
    return deck_plate(z0, z1).fuse(roof_post(z1)).removeSplitter()


# --- board layout ----------------------------------------------------------
def qt_row(high, n=3):
    """n portrait QT boards along the low (-Y) or high (+Y) edge, most centred in X."""
    opts = pf.board_options(ROWS, COLS, margin=BORDER - 2)
    y = max(o[1] for o in opts) if high else min(o[1] for o in opts)
    row = [o for o in opts if abs(o[1] - y) < 0.01]
    xs = sorted({o[0] for o in row})
    best = None
    for combo in itertools.combinations(xs, n):
        if all(b - a >= pf.QT_W - 1e-6 for a, b in zip(combo, combo[1:])):
            slack = abs((combo[0] + combo[-1] + pf.QT_W) / 2 - GRID / 2)
            if best is None or slack < best[0]:
                best = (slack, combo)
    return [(x, y) for x in best[1]]


def wing_spot():
    """FeatherWing lower-left (grid coords) along the low edge, all 4 holes on slots."""
    fits = [(dx - 2.54, dy - 2.54) for q, (dx, dy) in sg.find_fits(sg.rect_holes(45.72, 17.78), ROWS, COLS)
            if q == 0]
    return min(fits, key=lambda f: (round(f[1], 1), abs(f[0] + 25.4 - GRID / 2)))


def qt_envelope(plug_low):
    b = Part.makeBox(pf.QT_W, pf.QT_H, PCB_T + PARTS_H, V(0, 0, QT_SPACER))
    pw, pl, ph = PLUG
    v0 = -pl if plug_low else pf.QT_H
    return b.fuse(Part.makeBox(pw, pl, ph, V(pf.QT_W / 2 - pw / 2, v0, QT_SPACER + PCB_T)))


def boards():
    """Reference envelopes in stack coords: list of (name, shape)."""
    out = []
    tops = [z1 for _, z1 in deck_z()]
    layout = {0: ("qt", "qt"), 1: ("wing", "qt"), 2: ("qt", "qt")}
    for d, (low, high) in layout.items():
        o = grid_origin(tops[d])
        if low == "wing":
            wx, wy = wing_spot()
            wing = Part.makeBox(50.8, 22.86, PCB_T + PARTS_H, V(wx, wy, WING_STANDOFF))
            wing = wing.fuse(Part.makeBox(PLUG[1], PLUG[0], PLUG[2],
                                          V(wx + 50.8, wy + 11.43 - PLUG[0] / 2, WING_STANDOFF + PCB_T)))
            wing.translate(o)
            out.append((f"deck{d}_FeatherWing", wing))
        else:
            for i, (x, y) in enumerate(qt_row(high=False), 1):
                b = qt_envelope(plug_low=True)
                b.translate(o + V(x, y, 0))
                out.append((f"deck{d}_QT_low{i}", b))
        for i, (x, y) in enumerate(qt_row(high=True), 1):
            b = qt_envelope(plug_low=False)
            b.translate(o + V(x, y, 0))
            out.append((f"deck{d}_QT_high{i}", b))
    return out


if __name__ == "__main__":
    print("deck z", deck_z(), "roof top", roof_top(), "axis", axis_z(), "body half x", body_half_x())
    print("qt low", qt_row(False), "qt high", qt_row(True), "wing", wing_spot())
    bs = boards()
    parts = [middle_deck(), top_deck()]
    for s in parts:
        print("valid", s.isValid(), "solids", len(s.Solids))
    for (na, a), (nb, b) in itertools.combinations(bs, 2):
        if a.common(b).Volume > 1e-3:
            print("BOARD OVERLAP", na, nb)
    for name, b in bs:
        for i, s in enumerate(parts):
            v = b.common(s).Volume
            if v > 1e-3:
                print("BOARD HITS PART", name, i, round(v, 2))
    print("boards", len(bs))
