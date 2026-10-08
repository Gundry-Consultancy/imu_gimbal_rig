"""Sensor stack: three horizontal swirly-grid decks, open sides, small roof in the middle of the top.

Imported by make_gimbal.py, which also places it on the roll axis.

Only the MIDDLE deck connects to the gimbal: its +X / -X end walls carry the
MG90S roll horn and the idler axle. The top and bottom decks hang off it on
central standoff spines that sit directly under the roof, so a strike on
the roof goes straight down the middle and nothing is held at the edges.

Each deck is a 5 x 4 cell swirly grid (76.2 x 60.96 mm). The boards sit
portrait, 4 across, in two rows. A solid strip between the rows takes the
spines:
  bottom deck   8 QT
  middle deck   FeatherWing (on 10 mm standoffs) + 1 QT, and 4 QT
  top deck      8 QT around the roof post
That is 21 QT boards + 1 FeatherWing. PR 839 needs 9 breakouts + the wing.

Two M3 bolts at x = +/-14 run through top deck, upper spine, middle deck,
lower spine and bottom deck, with the heads on top (outside the roof) and
nuts underneath. Every printed part prints flat without supports.
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

ROWS, COLS = 4, 5
GRID_X, GRID_Y = COLS * sg.CELL, ROWS * sg.CELL      # 76.2 x 60.96
BORDER = 3.0
DECK_X, DECK_Y = GRID_X + 2 * BORDER, GRID_Y + 2 * BORDER
DECK_T = 3.0
MID_T = 4.0                                           # middle deck carries the walls

SPINE_HALF_X = 18.0
SPINE_HALF_Y = 3.5
STRIP_HALF_Y = 4.6                                    # solid strip between the board rows
BOLT_X = 14.0
BOLT_HOLE = 3.4

QT_SPACER = 3.0
WING_STANDOFF = 10.0
PCB_T = 1.6
PARTS_H = 2.5
PLUG = (6.0, 8.0, 3.5)
HEADROOM = 3.0

QT_STACK_H = QT_SPACER + PCB_T + PLUG[2]
WING_STACK_H = WING_STANDOFF + PCB_T + PARTS_H
GAPS = (QT_STACK_H + HEADROOM, WING_STACK_H + HEADROOM)   # above bottom deck, above middle deck

POST = 9.0
ROOF = 24.0
ROOF_T = 3.0
ROOF_H = QT_SPACER + PCB_T + PARTS_H + 1.0 + (ROOF - POST) / 2 + ROOF_T

WALL_T = 4.0
WALL_HALF_Y = 19.0
HUB_R = 11.0


def deck_z():
    """(bottom, top) of bottom, middle and top decks; bottom deck underside at z = 0."""
    b = (0.0, DECK_T)
    m0 = b[1] + GAPS[0]
    m = (m0, m0 + MID_T)
    t0 = m[1] + GAPS[1]
    return [b, m, (t0, t0 + DECK_T)]


def roof_top():
    return deck_z()[2][1] + ROOF_H


def axis_z():
    """Roll axis height: middle of the stack, but at least a hub radius above the middle deck."""
    return max(roof_top() / 2, deck_z()[1][1] + HUB_R)


def body_half_x():
    return DECK_X / 2 + WALL_T


def deck_plate(z0, z1):
    plate = Part.makeBox(DECK_X, DECK_Y, z1 - z0, V(-DECK_X / 2, -DECK_Y / 2, z0))
    plate = plate.makeFillet(3.0, [e for e in plate.Edges if e.BoundBox.ZLength > (z1 - z0) - 0.01])
    cuts = []
    for c in slot_cutters(ROWS, COLS, -1.0, (z1 - z0) + 2.0, SLOT_WIDTH):
        c.translate(V(-GRID_X / 2, -GRID_Y / 2, z0))
        cuts.append(c)
    plate = plate.cut(Part.makeCompound(cuts))
    strip = Part.makeBox(2 * SPINE_HALF_X + 6, 2 * STRIP_HALF_Y, z1 - z0, V(-SPINE_HALF_X - 3, -STRIP_HALF_Y, z0))
    plate = plate.fuse(strip)
    holes = [Part.makeCylinder(BOLT_HOLE / 2, (z1 - z0) + 2, V(x, 0, z0 - 1)) for x in (-BOLT_X, BOLT_X)]
    return plate.cut(Part.makeCompound(holes))


def spine(z0, z1):
    s = Part.makeBox(2 * SPINE_HALF_X, 2 * SPINE_HALF_Y, z1 - z0, V(-SPINE_HALF_X, -SPINE_HALF_Y, z0))
    holes = [Part.makeCylinder(BOLT_HOLE / 2, (z1 - z0) + 2, V(x, 0, z0 - 1)) for x in (-BOLT_X, BOLT_X)]
    return s.cut(Part.makeCompound(holes)).removeSplitter()


def roof_post(z0):
    """Square post flaring at 45 degrees into the roof (prints without support)."""
    z_roof_top = z0 + ROOF_H
    z_flare_top = z_roof_top - ROOF_T
    z_flare_bot = z_flare_top - (ROOF - POST) / 2

    def sq(h, z):
        return Part.makePolygon([V(-h, -h, z), V(h, -h, z), V(h, h, z), V(-h, h, z), V(-h, -h, z)])
    post = Part.makeBox(POST, POST, z_flare_bot - z0 + 0.01, V(-POST / 2, -POST / 2, z0))
    taper = Part.makeLoft([sq(POST / 2, z_flare_bot), sq(ROOF / 2, z_flare_top)], True)
    roof = Part.makeBox(ROOF, ROOF, ROOF_T, V(-ROOF / 2, -ROOF / 2, z_flare_top))
    return post.fuse([taper, roof])


def middle_deck(horn_cutter, axle_boss, axle_pilot):
    """Middle deck + roll end walls. Horn/axle features are given in stack coords by make_gimbal."""
    z0, z1 = deck_z()[1]
    za = axis_z()
    shape = deck_plate(z0, z1)
    walls = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * (DECK_X / 2 - 0.01), sx * (DECK_X / 2 + WALL_T)))
        w = Part.makeBox(x1 - x0, 2 * WALL_HALF_Y, za - z0, V(x0, -WALL_HALF_Y, z0))
        hub = Part.makeCylinder(HUB_R, x1 - x0, V(x0, 0, za), V(1, 0, 0))
        walls.append(w.fuse(hub))
    shape = shape.fuse(walls + [axle_boss]).cut(Part.makeCompound([horn_cutter, axle_pilot]))
    return shape.removeSplitter()


def bottom_deck():
    return deck_plate(*deck_z()[0]).removeSplitter()


def top_deck():
    z0, z1 = deck_z()[2]
    return deck_plate(z0, z1).fuse(roof_post(z1)).removeSplitter()


def spines():
    (_, b1), (m0, m1), (t0, _) = deck_z()
    return spine(b1, m0), spine(m1, t0)


# --- board layout ----------------------------------------------------------
def qt_row(high, n=4):
    """n portrait QT boards along the low (-Y) or high (+Y) edge, most centred in X."""
    opts = pf.board_options(ROWS, COLS, margin=BORDER)
    for y in sorted({o[1] for o in opts}, reverse=high):
        xs = sorted({o[0] for o in opts if abs(o[1] - y) < 0.01})
        best = None
        for combo in itertools.combinations(xs, n):
            if all(b - a >= pf.QT_W - 1e-6 for a, b in zip(combo, combo[1:])):
                slack = abs((combo[0] + combo[-1] + pf.QT_W) / 2 - GRID_X / 2)
                if best is None or slack < best[0]:
                    best = (slack, combo)
        if best:
            return [(x, y) for x in best[1]]
    raise ValueError(f"no row fits {n} boards")


def wing_layout():
    """FeatherWing on the low edge (all 4 holes on slots) + the portrait QT spots beside it."""
    fits = [(dx - 2.54, dy - 2.54) for q, (dx, dy) in sg.find_fits(sg.rect_holes(45.72, 17.78), ROWS, COLS)
            if q == 0 and dx - 2.54 >= -0.01]
    wx, wy = min(fits, key=lambda f: (round(f[1], 1), f[0]))
    low = [(x, y) for x, y in qt_row(high=False, n=4) if x >= wx + 50.8 - 1e-6]
    if not low:
        opts = pf.board_options(ROWS, COLS, margin=BORDER)
        y = min(o[1] for o in opts)
        xs = [o[0] for o in opts if abs(o[1] - y) < 0.01 and o[0] >= wx + 50.8 - 1e-6 and o[0] + pf.QT_W <= GRID_X + BORDER]
        low = [(min(xs), y)] if xs else []
    return (wx, wy), low


def qt_envelope(plug_low):
    b = Part.makeBox(pf.QT_W, pf.QT_H, PCB_T + PARTS_H, V(0, 0, QT_SPACER))
    pw, pl, ph = PLUG
    v0 = -pl if plug_low else pf.QT_H
    return b.fuse(Part.makeBox(pw, pl, ph, V(pf.QT_W / 2 - pw / 2, v0, QT_SPACER + PCB_T)))


def boards():
    """Reference envelopes in stack coords: list of (name, shape)."""
    out = []
    tops = [z1 for _, z1 in deck_z()]
    for d, name in enumerate(("bottom", "middle", "top")):
        o = V(-GRID_X / 2, -GRID_Y / 2, tops[d])
        if name == "middle":
            (wx, wy), low = wing_layout()
            wing = Part.makeBox(50.8, 22.86, PCB_T + PARTS_H, V(wx, wy, WING_STANDOFF))
            wing = wing.fuse(Part.makeBox(PLUG[1], PLUG[0], PLUG[2],
                                          V(wx + 50.8, wy + 11.43 - PLUG[0] / 2, WING_STANDOFF + PCB_T)))
            wing.translate(o)
            out.append((f"{name}_FeatherWing", wing))
        else:
            low = qt_row(high=False)
        for i, (x, y) in enumerate(low, 1):
            b = qt_envelope(plug_low=True)
            b.translate(o + V(x, y, 0))
            out.append((f"{name}_QT_low{i}", b))
        for i, (x, y) in enumerate(qt_row(high=True), 1):
            b = qt_envelope(plug_low=False)
            b.translate(o + V(x, y, 0))
            out.append((f"{name}_QT_high{i}", b))
    return out


if __name__ == "__main__":
    print("deck z", deck_z(), "roof top", roof_top(), "axis", axis_z(), "body half x", body_half_x())
    print("qt low", qt_row(False), "qt high", qt_row(True), "wing", wing_layout())
    bs = boards()
    parts = [bottom_deck(), top_deck()] + list(spines())
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
