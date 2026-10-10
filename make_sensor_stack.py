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
nuts underneath. The top and bottom decks have bosses round the bolt holes
(head and nut side). Every printed part prints flat without supports; the
bottom deck prints bosses-up.
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
BASE_GAPS = (QT_STACK_H + HEADROOM, WING_STACK_H + HEADROOM)   # above bottom deck, above middle deck
# Variant: a taller upper spine (top deck higher). The middle deck, its walls and the
# roll axis stay put; only the upper spine grows and the top deck + roof rise.
GAPS = (BASE_GAPS[0], float(os.environ.get("STACK_UPPER_GAP", BASE_GAPS[1])))

POST = 9.0
ROOF = 24.0
ROOF_T = 3.0
ROOF_H = QT_SPACER + PCB_T + PARTS_H + 1.0 + (ROOF - POST) / 2 + ROOF_T

# End walls (roll horn on +X, idler axle on -X). After the first print the +X wall
# snapped at its top edge, where the horn pocket and its arm pilots broke out of a
# 4 mm wall that stopped at the axis. Now: 6 mm thick (>= 3 mm behind the pocket
# floor), wider than the horn arm, carried 8 mm above the axis with 45 deg shoulders,
# with a 45 deg rib + boss behind the hub on the inner face (over the strip, clear of
# the boards) and a flared root on the +X outer face.
WALL_T = 6.0
WALL_HALF_Y = 21.0
WALL_ABOVE = 8.0          # wall top above the roll axis (the hub still rises to HUB_R)
WALL_SHOULDER = 6.0       # 45 deg chamfer on the wall's top corners
HUB_R = 11.0
RIB_HALF_Y = 4.0          # inner rib/boss behind the hub; the strip is +/-4.6, boards start at 5.08
RIB_D = 3.0               # boss thickness behind the hub (wall + boss behind the pocket)
RIB_FOOT = 8.0            # rib reach along the deck top
ROOT_FLARE = 4.0          # 45 deg flare along the +X wall's outer root

# Stack bolt bosses: the top deck (head side) and bottom deck (nut side) are doubled
# locally around the M3 holes. The bottom deck now prints bosses-up (flipped).
BOLT_BOSS_R = BOLT_HOLE / 2 + 2.4
BOLT_BOSS_H = DECK_T


def deck_z(gaps=None):
    """(bottom, top) of bottom, middle and top decks; bottom deck underside at z = 0."""
    gaps = gaps or GAPS
    b = (0.0, DECK_T)
    m0 = b[1] + gaps[0]
    m = (m0, m0 + MID_T)
    t0 = m[1] + gaps[1]
    return [b, m, (t0, t0 + DECK_T)]


def roof_top(gaps=None):
    return deck_z(gaps)[2][1] + ROOF_H


def roof_raise():
    """How much higher the roof is than in the base design."""
    return roof_top() - roof_top(BASE_GAPS)


def axis_z():
    """Roll axis height: middle of the BASE stack, but at least a hub radius above the middle deck.
    Fixed by the base design so a taller upper spine doesn't move the middle deck or the gimbal."""
    return max(roof_top(BASE_GAPS) / 2, deck_z(BASE_GAPS)[1][1] + HUB_R)


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


def _prism_yz(pts, x0, x1):
    """Polygon in (y, z) extruded along x from x0 to x1."""
    f = Part.Face(Part.makePolygon([V(x0, y, z) for y, z in pts] + [V(x0, pts[0][0], pts[0][1])]))
    return f.extrude(V(x1 - x0, 0, 0))


def _prism_xz(pts, y0, y1):
    """Polygon in (x, z) extruded along y from y0 to y1."""
    f = Part.Face(Part.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])]))
    return f.extrude(V(0, y1 - y0, 0))


def middle_deck(horn_cutter, axle_boss, axle_pilot):
    """Middle deck + roll end walls. Horn/axle features are given in stack coords by make_gimbal."""
    z0, z1 = deck_z()[1]
    za = axis_z()
    shape = deck_plate(z0, z1)
    walls = []
    zt, c, hy = za + WALL_ABOVE, WALL_SHOULDER, WALL_HALF_Y
    profile = [(-hy, z0), (hy, z0), (hy, zt - c), (hy - c, zt), (-hy + c, zt), (-hy, zt - c)]
    for sx in (-1, 1):
        x_in = sx * DECK_X / 2
        x0, x1 = sorted((sx * (DECK_X / 2 - 0.01), sx * (DECK_X / 2 + WALL_T)))
        w = _prism_yz(profile, x0, x1)
        hub = Part.makeCylinder(HUB_R, x1 - x0, V(x0, 0, za), V(1, 0, 0))
        # inner rib + boss behind the hub (45 deg faces, prints walls-up)
        h_boss = za - z1 + 2.0
        rib = [(0, z1 - 0.01), (RIB_FOOT, z1 - 0.01), (RIB_D, z1 + RIB_FOOT - RIB_D), (RIB_D, z1 + h_boss),
               (0, z1 + h_boss + RIB_D)]
        rib = _prism_xz([(x_in - sx * (d - 0.01), z) for d, z in rib], -RIB_HALF_Y, RIB_HALF_Y)
        parts = [hub, rib]
        if sx > 0:
            # flared root on the horn wall's outer face
            xo = sx * (DECK_X / 2 + WALL_T)
            parts.append(_prism_xz([(xo - 0.01, z0), (xo + ROOT_FLARE, z0), (xo - 0.01, z0 + ROOT_FLARE + 0.01)], -hy, hy))
        walls.append(w.fuse(parts))
    shape = shape.fuse(walls + [axle_boss]).cut(Part.makeCompound([horn_cutter, axle_pilot]))
    return shape.removeSplitter()


def bolt_bosses(z_face, up):
    """Bosses around the two stack bolt holes on a deck face (up = +1 grows upward)."""
    out = []
    for x in (-BOLT_X, BOLT_X):
        p = V(x, 0, z_face - up * 0.01)
        out.append(Part.makeCylinder(BOLT_BOSS_R, BOLT_BOSS_H + 0.01, p, V(0, 0, up)))
    return out


def bolt_holes(z0, z1):
    return Part.makeCompound([Part.makeCylinder(BOLT_HOLE / 2, z1 - z0 + 2, V(x, 0, z0 - 1)) for x in (-BOLT_X, BOLT_X)])


def bottom_deck():
    z0, z1 = deck_z()[0]
    shape = deck_plate(z0, z1).fuse(bolt_bosses(z0, -1))
    return shape.cut(bolt_holes(z0 - BOLT_BOSS_H, z1)).removeSplitter()


def top_deck():
    z0, z1 = deck_z()[2]
    shape = deck_plate(z0, z1).fuse([roof_post(z1)] + bolt_bosses(z1, 1))
    return shape.cut(bolt_holes(z0, z1 + BOLT_BOSS_H)).removeSplitter()


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
