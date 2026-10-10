"""Pan / tilt / roll gimbal around the sensor stack. Run with freecadcmd.

    freecadcmd -c "exec(open('make_gimbal.py').read(), {'__file__': 'make_gimbal.py', '__name__': '__main__'})"

Global frame: origin is the axis intersection (middle of the sensor stack), +X is the roll axis,
+Y is the tilt axis, +Z is the pan axis (up).

Parts (exported to parts/):
  stack_middle_deck, stack_top_deck, stack_bottom_deck, stack_spine_upper, stack_spine_lower
                                the sensor stack (see make_sensor_stack.py); only the middle deck's
                                end walls carry the MG90S horn (+X) and the 623 idler axle (-X)
  striker_bar, striker_pawl, striker_cam, striker_stand
                                cam-driven tap striker on the yoke (see make_striker.py)
  tilt_ring_mg90s                holds the roll MG90S and the -X 623 bearing;
                                standard-servo horn on +Y bar, idler axle on -Y
  pan_yoke                      holds the standard tilt servo (+Y) and the -Y 623 bearing;
                                pan horn on the underside
  base                          holds the standard pan servo

The servo, bearing and board solids are reference envelopes for clearance
checks. They are not printed.
"""

import json
import math
import os
import sys

import FreeCAD as App
import Part
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_sensor_stack as ss  # noqa: E402

V = App.Vector
# GIMBAL_OUT: output folder for a design variant (default: this folder). GIMBAL_PIN: a
# build_info.json from the base build; its ring / yoke sizes are reused so a variant
# leaves the gimbal untouched (the clearance checks still run against it).
OUT_DIR = os.environ.get("GIMBAL_OUT", HERE)
PARTS_DIR = os.path.join(OUT_DIR, "parts")
PIN = os.environ.get("GIMBAL_PIN")
PIN_MIN_CLEAR = 1.5         # a pinned variant may eat into the sweep margins, but not below this

# --- servos ---------------------------------------------------------------
# Measured by hand 2026-10-08 (see docs/servo-measurements.png). Letters match the drawing.
# Readings run 0.1-0.2 mm generous (fit allowance), so they are used as-is.
# spline_above_tab (G) sets the mount position: the directly measured G is used
# (D - E - F reads ~0.8 mm more because the generous readings compound).
# offset: shaft centre from body centre along the long axis = A/2 - H.
MG90S_TAB = 16.8                                   # E, base to tab underside
MG90S_G = 12.0                                     # G, tab top to spline top (11.7-12 measured)
MG90S = dict(L=23.0,                               # A 22.7 measured, allow 23
             W=12.3, H=32.0, case_top=4.0,         # C, D
             tab_len=32.2, tab_t=2.5,              # B, F
             holes=(27.3, 27.7), hole_across=0.0,  # J 27.5 (+/-0.2 slot), single row
             pilot=1.7, offset=22.7 / 2 - 5.9,     # M 2.0 (M2 self-tap pilot), H 5.9
             spline_r=2.4,                         # N 21T, ~4.8
             spline=dict(n=21, d=4.8, tooth=0.3, engage=3.5, screw=2.2, head=4.2),   # M2 centre screw
             pilot_wall=1.0,                       # min wall kept round a closed tab pilot
             wire_notch=dict(width=6.0, depth=None),   # see MOUSE_EAR_MIN_ARC below
             # P-S assumed (stock horn) -- or print horn_mg90s_printed, which fits this pocket exactly
             horn=dict(len=36.0, width=7.0, depth=2.0, hub=2.5, centre=6.0, pilot=1.6, arm_screw=2.2))
DS3240 = dict(L=40.5, W=20.5, H=46.2, case_top=4.5,   # A, C, D
              tab_len=54.5, tab_t=3.2,                # B, F (3.08 measured)
              holes=(48.4, 48.8), hole_across=9.75,   # J 48.5-48.6, K 9.5-10
              pilot=2.4, offset=40.5 / 2 - 9.5,       # H ~9.5
              spline_r=3.0,
              spline=dict(n=25, d=6.0, tooth=0.3, engage=4.0, screw=3.2, head=6.0),   # M3 centre screw
              spline_above_tab=14.0,                  # G measured (D - E - F reads 14.8)
              pilot_wall=1.2,
              wire_notch=dict(width=8.0, depth="tab"),
              # P-S assumed (stock horn) -- or print horn_ds3240_printed, which fits this pocket exactly
              horn=dict(len=46.0, width=8.5, depth=2.5, hub=3.5, centre=7.0, pilot=2.0, arm_screw=2.7))
STANDARD = DS3240      # tilt and pan servos (an MG995 would need its own measurements)

# --- servo plate fit tuning (after the first test print, 2026-10-10) --------
# The servos would not go in: the cable exit at the spline end hit the plate, the cut-out
# was tight, and the plates snapped at the screw pilots when flexed.
#
# Wire notch: an extension of the body cut-out at the spline (+x) end, through the whole
# plate and its bosses, centred on the servo's long axis. Per servo, in the dicts above:
#   wire_notch=dict(width=..., depth=...)   depth in mm beyond the cut-out, "tab" (to the
#   tab end) or None (as deep as MOUSE_EAR_MIN_ARC allows for a pilot on the centreline).
# Where the notch reaches a screw pilot it opens the pilot's side ("mouse ear") instead of
# leaving a paper-thin wall; at least MOUSE_EAR_MIN_ARC degrees of wall stay round it, and
# the boss below keeps it strong. Defaults:
#   DS3240  8.0 wide to the tab end. It runs between the two pilots (9.75 apart) and opens
#           each one's inner side by ~118 deg at mid-wall (~86 deg at the hole). (7 mm would leave a 0.2 mm wall; a closed
#           1.2 mm wall would cap the notch at 4.95 mm wide.)
#   MG90S   6.0 wide; its spline-end pilot sits on the centreline only ~1 mm past the
#           body, so the notch stops where the pilot keeps 240 deg of wall (~1.1 mm deep,
#           reaching ~1.5 mm past the body end). The MG90S can also go in from the other face.
SERVO_CUT_CLEAR = 0.4       # body cut-out clearance per side (was 0.25)
SERVO_LEADIN = 1.0          # 45 deg lead-in chamfer on the body cut-out, on both faces
MOUSE_EAR_MIN_ARC = 240.0   # deg of wall a wire notch must leave round a pilot it opens,
MOUSE_EAR_AT = 0.5          #   measured this far out from the pilot (mid-wall)
BOSS_WALL = 2.4             # pilot boss wall (6 perimeters at 0.4)
BOSS_FACTOR = 2.0           # local thickness at a tab pilot = BOSS_FACTOR x plate thickness
MAX_BRIDGE = 8.0            # longest flat ceiling allowed in a lightening window


def mg90s(tab_height):
    s = dict(MG90S)
    s["spline_above_tab"] = MG90S_G
    s["tab_height"] = tab_height
    return s


# --- bearing 623ZZ (3 x 10 x 4) -------------------------------------------
BRG_OD, BRG_W = 10.0, 4.0
BRG_POCKET = 10.15
LIP_HOLE = 6.5
AXLE_BOSS_D = 4.8   # touches inner race only
M3_PILOT = 2.8

# --- layout ------------------------------------------------------------------
BODY_HALF_X = ss.body_half_x()             # outer face of each cradle end wall
SWEEP_MARGIN = 4.0                          # beyond the measured sweep radius (cable slop)
RING_BAR_T = 6.0
RING_BAR_HALF_Z = 7.0
RING_BRG_PLATE_T = 6.4   # 4 mm bearing pocket + 2.4 mm lip (6 perimeters; was 2)
RING_GAP = 1.5
RING_HORN_BOSS = 9.5     # the +Y bar grows to +/- this round the tilt horn (teardrop roofs fit)

YOKE_PLATE_T = 6.0       # tilt servo plate
YOKE_BRG_T = 6.4         # idler upright: 4 mm bearing pocket + 2.4 mm lip
YOKE_HALF_X = 16.0
YOKE_GAP = 1.5
YOKE_FLOOR_T = 6.0

SERVO_PLATE_T = 5.0
STAND_BOLTS = ((-11.0, 6.0), (11.0, 6.0), (-11.0, -12.0), (11.0, -12.0))   # striker stand on the idler upright (x, z)


# --- helpers ---------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, p0, p1):
    d = p1 - p0
    return Part.makeCylinder(r, d.Length, p0, d)


def placed(shape, origin, xdir, zdir):
    """Map a local shape (x = servo long axis / horn arm, z = shaft direction) into the world."""
    xdir, zdir = V(xdir).normalize(), V(zdir).normalize()
    ydir = zdir.cross(xdir)
    m = App.Matrix(xdir.x, ydir.x, zdir.x, origin.x,
                   xdir.y, ydir.y, zdir.y, origin.y,
                   xdir.z, ydir.z, zdir.z, origin.z,
                   0, 0, 0, 1)
    s = shape.copy()
    s.transformShape(m)
    return s


def servo_envelope(s):
    """Local frame: spline top at origin, shaft along +z, long axis along x (body centre at -offset)."""
    cx = -s["offset"]
    body = box(cx - s["L"] / 2, cx + s["L"] / 2, -s["W"] / 2, s["W"] / 2, -s["H"], -s["case_top"])
    tz1 = -s["spline_above_tab"]
    tabs = box(cx - s["tab_len"] / 2, cx + s["tab_len"] / 2, -s["W"] / 2, s["W"] / 2, tz1 - s["tab_t"], tz1)
    spline = Part.makeCylinder(s["spline_r"], s["case_top"], V(0, 0, -s["case_top"]))
    return body.fuse([tabs, spline]).removeSplitter()


def prism(pts2d, origin, u, v, n, n0, n1):
    """Polygon in the (u, v) plane at origin, extruded along n from n0 to n1 (world vectors)."""
    u, v, n = V(u), V(v), V(n)
    pts = [origin + u * a + v * b + n * n0 for a, b in pts2d]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(n * (n1 - n0))


def window_pts(u0, u1, v0, v1):
    """Lightening window outline in (u, v), v = print-up. 45 deg corners top and bottom keep
    any flat ceiling under MAX_BRIDGE, so it prints without support either way up."""
    w, h = u1 - u0, v1 - v0
    c = min(h / 2, max(1.0, (w - MAX_BRIDGE) / 2))
    pts = [(u0 + c, v0), (u1 - c, v0), (u1, v0 + c), (u1, v1 - c), (u1 - c, v1), (u0 + c, v1), (u0, v1 - c), (u0, v0 + c)]
    return [p for i, p in enumerate(pts) if (abs(p[0] - pts[i - 1][0]) + abs(p[1] - pts[i - 1][1])) > 1e-6]


def row_windows(a, b, n, web=3.0):
    """Split [a, b] into n window spans separated by webs."""
    w = (b - a - (n - 1) * web) / n
    return [(a + i * (w + web), a + i * (w + web) + w) for i in range(n)]


def servo_pilots(s):
    """Tab screw pilots in servo-local coords: (x_near_body, x_far, y, r)."""
    cx = -s["offset"]
    h0, h1 = s["holes"]
    ys = [0.0] if s["hole_across"] == 0 else [-s["hole_across"] / 2, s["hole_across"] / 2]
    return [(cx + side * h0 / 2, cx + side * h1 / 2, y, s["pilot"] / 2) for side in (-1, 1) for y in ys]


def wire_notch(s):
    """(x0, x1, half_width) of the wire notch in servo-local coords, or None. Checks the mouse-ear rule."""
    n = s.get("wire_notch")
    if not n:
        return None
    cx = -s["offset"]
    x0 = cx + s["L"] / 2 + SERVO_CUT_CLEAR
    hw = n["width"] / 2
    half_open = math.radians((360.0 - MOUSE_EAR_MIN_ARC) / 2)
    x_tab = cx + s["tab_len"] / 2
    x1 = x_tab if n["depth"] in (None, "tab") else x0 + n["depth"]
    for xa, xb, y, r in servo_pilots(s):
        if xa < x0:
            continue                                   # far-end pilot
        rr = r + MOUSE_EAR_AT
        if abs(y) < hw:                                # pilot centre inside the notch band
            stop = xa - rr * math.cos(half_open)
            if n["depth"] is None:
                x1 = min(x1, stop)
            elif x1 > stop + 1e-6:
                raise ValueError(f"wire notch opens a centreline pilot past {MOUSE_EAR_MIN_ARC} deg")
        elif abs(y) - hw < rr and x1 > xa - rr:        # notch runs past, opening the pilot's side
            opened = 2 * math.degrees(math.acos((abs(y) - hw) / rr))
            if opened > 360.0 - MOUSE_EAR_MIN_ARC:
                raise ValueError(f"wire notch opens a pilot by {opened:.0f} deg")
    return (x0, x1, hw) if x1 > x0 + 0.2 else None


def _rect_wire(x0, x1, y0, y1, z):
    return Part.makePolygon([V(x0, y0, z), V(x1, y0, z), V(x1, y1, z), V(x0, y1, z), V(x0, y0, z)])


def servo_plate(s, x_ext, y_ext, thickness=SERVO_PLATE_T):
    """Mount plate in servo-local frame, tabs resting on its +z face.

    Returns (plate, cutters, bosses). The bosses double the thickness round every tab pilot on
    the -z side (away from the tabs); they are 45 deg cones so they print in any orientation.
    Fuse them after any bounding-box based layout, then apply the cutters (body cut-out with
    lead-in chamfers on both faces, wire notch, pilots)."""
    z1 = -s["spline_above_tab"] - s["tab_t"]
    z0 = z1 - thickness
    hb = (BOSS_FACTOR - 1) * thickness
    zb = z0 - hb
    plate = box(x_ext[0], x_ext[1], y_ext[0], y_ext[1], z0, z1)
    cx, c, ch = -s["offset"], SERVO_CUT_CLEAR, SERVO_LEADIN
    bx0, bx1, by0, by1 = cx - s["L"] / 2 - c, cx + s["L"] / 2 + c, -s["W"] / 2 - c, s["W"] / 2 + c
    pilots = servo_pilots(s)
    cut = [box(bx0, bx1, by0, by1, zb - 1, z1 + 1)]
    # lead-in chamfers on both faces, kept pilot_wall clear of every pilot
    lead = [Part.makeLoft([_rect_wire(bx0, bx1, by0, by1, z1 - ch), _rect_wire(bx0 - ch, bx1 + ch, by0 - ch, by1 + ch, z1)], True),
            box(bx0 - ch, bx1 + ch, by0 - ch, by1 + ch, z1 - 0.01, z1 + 1),
            Part.makeLoft([_rect_wire(bx0, bx1, by0, by1, z0 + ch), _rect_wire(bx0 - ch, bx1 + ch, by0 - ch, by1 + ch, z0)], True),
            box(bx0 - ch, bx1 + ch, by0 - ch, by1 + ch, zb - 1, z0 + 0.01)]
    keep = []
    for xa, xb, y, r in pilots:
        rk = r + s.get("pilot_wall", 1.0)
        keep += [Part.makeCylinder(rk, z1 - zb + 4, V(x, y, zb - 2)) for x in (xa, xb)]
        keep.append(box(min(xa, xb), max(xa, xb), y - rk, y + rk, zb - 2, z1 + 2))
    cut.append(lead[0].fuse(lead[1:]).cut(Part.makeCompound(keep)))
    notch = wire_notch(s)
    if notch:
        nx0, nx1, hw = notch
        cut.append(box(nx0 - 0.01, nx1, -hw, hw, zb - 1, z1 + 1))
    bosses = []
    for xa, xb, y, r in pilots:
        a, b = V(xa, y, zb - 1), V(xb, y, zb - 1)
        cut += [Part.makeCylinder(r, z1 - zb + 2, a), Part.makeCylinder(r, z1 - zb + 2, b)]
        if abs(xb - xa) > 0:
            cut.append(box(min(xa, xb), max(xa, xb), y - r, y + r, zb - 1, z1 + 1))
        rt = r + BOSS_WALL
        for x in sorted({xa, xb}):
            bosses.append(Part.makeCone(rt + hb, rt, hb + 0.01, V(x, y, z0 + 0.01), V(0, 0, -1)))
    clip = box(x_ext[0], x_ext[1], y_ext[0], y_ext[1], zb - 0.5, z0 + 0.5)
    boss = bosses[0].fuse(bosses[1:]).common(clip) if hb > 0 else None
    return plate, cut, boss


# Where each plate and horn pocket ended up (for the verification script): name -> placement.
PLATES = {}
HORNS = {}


def _teardrop(r, up, z0, z1):
    """45 deg roof over a circle of radius r centred on the local z axis, pointing along up (2D)."""
    ux, uy = up
    px, py = -uy, ux                      # perpendicular
    k = r / math.sqrt(2)
    pts = [(k * px + k * ux, k * py + k * uy), (r * math.sqrt(2) * ux, r * math.sqrt(2) * uy),
           (-k * px + k * ux, -k * py + k * uy), (0, 0)]
    return prism(pts, V(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), z0, z1)


def horn_pocket(horn, through=12.0, up=()):
    """Local frame: mounting face at z=0, part material at z<0, servo at z>0. Arm along x.

    up: the print-up directions in the face plane, as local 2D unit vectors along x or y.
    Each one gets 45 deg roofs on the pocket ceilings (arm lip, hub) and a teardrop on the
    centre access hole, so a pocket on a vertical face prints without support."""
    hl, hw, d = horn["len"] / 2, horn["width"] / 2, horn["depth"]
    rh = hw + 1.5
    arm = box(-hl, hl, -hw, hw, -d, 1)
    hub = Part.makeCylinder(rh, d + 1, V(0, 0, -d))
    centre = Part.makeCylinder(horn["centre"] / 2, through + 1, V(0, 0, -through))
    extra = []
    for ux, uy in up:
        if uy:     # roof along the arm's long edge (in the y-z plane, run along x)
            tri = [(uy * hw, -d), (uy * hw, 1), (uy * (hw + d + 1), 1)]
            extra.append(prism(tri, V(-hl, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, 2 * hl))
        else:      # roof at the arm tip
            tri = [(ux * hl, -d), (ux * hl, 1), (ux * (hl + d + 1), 1)]
            extra.append(prism(tri, V(0, -hw, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0, 2 * hw))
        extra.append(_teardrop(rh, (ux, uy), -d, 1))
        extra.append(_teardrop(horn["centre"] / 2, (ux, uy), -through, 1))
    cut = arm.fuse([hub, centre] + extra)
    if horn.get("pilot"):
        # self-tap pilots for the arm screws (match the printed horn; drill your own for a stock horn)
        for x in (-horn["len"] / 2 + HORN_SCREW_INSET, horn["len"] / 2 - HORN_SCREW_INSET):
            cut = cut.fuse(Part.makeCylinder(horn["pilot"] / 2, 5.0, V(x, 0, -horn["depth"] - 5.0 + 0.01)))
    return cut


HORN_SCREW_INSET = 3.5      # arm screw centre from the arm tip
HORN_CLEAR = 0.15           # printed horn clearance in its pocket (per side)
SPLINE_CLEAR = 0.1          # radial clearance on the printed spline socket; tune to your printer


def spline_socket(sp, length):
    """Internal spline (star polygon, triangular teeth) along +z from z=0, in local coords."""
    import math
    n, r_out = sp["n"], sp["d"] / 2 + SPLINE_CLEAR
    r_in = r_out - sp["tooth"]
    pts = []
    for i in range(2 * n):
        a = math.pi * i / n
        r = r_out if i % 2 == 0 else r_in
        pts.append(V(r * math.cos(a), r * math.sin(a), 0))
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, length))


def printed_horn(servo):
    """Horn that fits horn_pocket(servo['horn']) exactly, with a moulded spline socket.

    Local frame matches horn_pocket: part face at z=0, arm sunk to z=-depth, servo at +z,
    spline top at z=hub. Returned arm-down (z from 0) ready to print, socket facing up.
    """
    h, sp = servo["horn"], servo["spline"]
    c = HORN_CLEAR
    arm = box(-h["len"] / 2 + c, h["len"] / 2 - c, -h["width"] / 2 + c, h["width"] / 2 - c, -h["depth"], 0)
    top = h["hub"] + sp["engage"]
    hub = Part.makeCylinder(h["width"] / 2 + 1.5 - c, top + h["depth"], V(0, 0, -h["depth"]))
    shape = arm.fuse(hub)
    sock = spline_socket(sp, sp["engage"] + 1)
    sock.translate(V(0, 0, h["hub"]))
    cuts = [sock, Part.makeCylinder(sp["screw"] / 2, top + h["depth"] + 2, V(0, 0, -h["depth"] - 1))]
    for x in (-h["len"] / 2 + HORN_SCREW_INSET, h["len"] / 2 - HORN_SCREW_INSET):
        cuts.append(Part.makeCylinder(h["arm_screw"] / 2, h["depth"] + 2, V(x, 0, -h["depth"] - 1)))
    shape = shape.cut(Part.makeCompound(cuts)).removeSplitter()
    shape.translate(V(0, 0, h["depth"]))
    return shape


def bearing_housing_cut(face_out, axis, plate_t=RING_BRG_PLATE_T):
    """Cut for a 623 pocket from an outer face plus a lip hole. face_out on the axis, axis points outward."""
    a = V(axis).normalize()
    pocket = cyl(BRG_POCKET / 2, face_out - a * BRG_W, face_out + a * 1)
    lip = cyl(LIP_HOLE / 2, face_out - a * (plate_t + 1), face_out)
    return pocket.fuse(lip)


def bearing_envelope(face_out, axis):
    a = V(axis).normalize()
    outer = cyl(BRG_OD / 2, face_out - a * BRG_W, face_out)
    return outer.cut(cyl(1.5, face_out - a * (BRG_W + 1), face_out + a))


def axle_boss(face, axis, length, pilot_depth):
    """Boss on a rotating part reaching the bearing inner race; axis points from part toward bearing."""
    a = V(axis).normalize()
    boss = cyl(AXLE_BOSS_D / 2, face - a * 0.5, face + a * length)
    pilot = cyl(M3_PILOT / 2, face + a * (length - pilot_depth), face + a * (length + 1))
    return boss, pilot


# --- parts -----------------------------------------------------------------
def max_radius(shapes, axis):
    """Largest distance of any (tessellated) point from an axis through the origin."""
    a = V(*axis).normalize()
    r = 0.0
    for shp in shapes:
        for p in shp.tessellate(0.2)[0]:
            r = max(r, (p - a * p.dot(a)).Length)
    return r


def sensor_body():
    """Stack parts and board envelopes, moved so the roll axis is the X axis."""
    za = ss.axis_z()
    up = V(0, 0, za)
    # the middle deck prints flat, walls up: world +z is local +y on the horn face
    h_o, h_x, h_z = V(BODY_HALF_X, 0, 0), (0, 1, 0), (1, 0, 0)
    horn = placed(horn_pocket(MG90S["horn"], through=ss.WALL_T + ss.RIB_D + 2, up=[(0, 1)]), h_o, h_x, h_z)
    HORNS["stack_middle_deck"] = dict(horn=MG90S["horn"], origin=h_o, x=h_x, z=h_z)
    boss_len = RING_GAP + (RING_BRG_PLATE_T - BRG_W)
    boss, pilot = axle_boss(V(-BODY_HALF_X, 0, 0), (-1, 0, 0), boss_len, boss_len + ss.WALL_T - 0.5)
    for c in (horn, boss, pilot):
        c.translate(up)
    lower, upper = ss.spines()
    parts = [("stack_middle_deck", ss.middle_deck(horn, boss, pilot)),
             ("stack_top_deck", ss.top_deck()),
             ("stack_bottom_deck", ss.bottom_deck()),
             ("stack_spine_upper", upper),
             ("stack_spine_lower", lower)]
    refs = ss.boards()
    for _, shp in parts + refs:
        shp.translate(-up)
    return parts, refs


def tilt_ring(servo, ring_in_y):
    ring_out_y = ring_in_y + RING_BAR_T
    roll_spline_x = BODY_HALF_X + servo["horn"]["hub"]
    s_origin, s_x, s_z = V(roll_spline_x, 0, 0), (0, 0, 1), (-1, 0, 0)
    # servo plate in servo-local coords: local x = world z, local y = world y, local z = -world x
    tab_z = servo["holes"][1] / 2 + 3
    plate, plate_cuts, bosses = servo_plate(servo, (-servo["offset"] - tab_z, -servo["offset"] + tab_z),
                                            (-ring_out_y, ring_out_y))
    PLATES["tilt_ring_mg90s"] = dict(servo=servo, t=SERVO_PLATE_T, origin=s_origin, x=s_x, z=s_z)
    plate = placed(plate, s_origin, s_x, s_z)
    bosses = placed(bosses, s_origin, s_x, s_z)
    plate_cuts = [placed(c, s_origin, s_x, s_z) for c in plate_cuts]
    pb = plate.BoundBox
    plate_x_in, plate_x_out = pb.XMin, pb.XMax

    brg_in = -(BODY_HALF_X + RING_GAP)
    brg_out = brg_in - RING_BRG_PLATE_T
    brg_half_z = RING_BAR_HALF_Z + 5
    brg_plate = box(brg_out, brg_in, -ring_out_y, ring_out_y, -brg_half_z, brg_half_z)

    bars = [box(brg_out, plate_x_out, y0, y1, -RING_BAR_HALF_Z, RING_BAR_HALF_Z)
            for y0, y1 in ((-ring_out_y, -ring_in_y), (ring_in_y, ring_out_y))]
    # +Y bar grows round the tilt horn so the pocket's teardrop roofs stay inside it
    hx, hz, bz = STANDARD["horn"]["len"] / 2 + 4, RING_HORN_BOSS, RING_BAR_HALF_Z
    e = hz - bz
    hb = [(-hx - e, -bz), (-hx, -hz), (hx, -hz), (hx + e, -bz), (hx + e, bz), (hx, hz), (-hx, hz), (-hx - e, bz)]
    horn_boss = prism(hb, V(0, ring_in_y, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0, RING_BAR_T)
    shape = plate.fuse([brg_plate, horn_boss, bosses] + bars)

    cuts = plate_cuts + [bearing_housing_cut(V(brg_out, 0, 0), (-1, 0, 0))]
    # lightening: one window in each servo-plate wing, two in each bearing-plate wing (prints flat, z up)
    for sy in (-1, 1):
        u0, u1 = sorted((sy * 12.0, sy * (ring_in_y - 4.0)))
        cuts.append(prism(window_pts(u0, u1, pb.ZMin + 4, pb.ZMax - 4), V(plate_x_in - 1, 0, 0),
                          (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, plate_x_out - plate_x_in + 2))
        for a, b in row_windows(10.0, ring_in_y - 3.0, 2):
            u0, u1 = sorted((sy * a, sy * b))
            cuts.append(prism(window_pts(u0, u1, -brg_half_z + 3, brg_half_z - 3), V(brg_out - 1, 0, 0),
                              (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, RING_BRG_PLATE_T + 2))
    # +Y: tilt servo horn (arm along X); the ring prints flat, so world +z (local -y) is up
    h_o, h_x, h_z = V(0, ring_out_y, 0), (1, 0, 0), (0, 1, 0)
    cuts.append(placed(horn_pocket(STANDARD["horn"], through=RING_BAR_T + 2, up=[(0, -1)]), h_o, h_x, h_z))
    HORNS["tilt_ring_mg90s"] = dict(horn=STANDARD["horn"], origin=h_o, x=h_x, z=h_z)
    # -Y: axle boss into the yoke bearing
    boss_len = YOKE_GAP + (YOKE_BRG_T - BRG_W)
    boss, pilot = axle_boss(V(0, -ring_out_y, 0), (0, -1, 0), boss_len, boss_len + RING_BAR_T - 0.5)
    shape = shape.fuse(boss).cut(Part.makeCompound(cuts + [pilot]))
    servo_env = placed(servo_envelope(servo), s_origin, s_x, s_z)
    brg_env = bearing_envelope(V(brg_out, 0, 0), (-1, 0, 0))
    return shape.removeSplitter(), servo_env, brg_env


def triangle_window(p0, p1, p2, inset):
    """Triangle (2D) shrunk about its incentre so every edge moves in by inset."""
    import math as _m
    a, b, c = (_m.dist(p1, p2), _m.dist(p0, p2), _m.dist(p0, p1))
    per = a + b + c
    ix = (a * p0[0] + b * p1[0] + c * p2[0]) / per
    iy = (a * p0[1] + b * p1[1] + c * p2[1]) / per
    s_ = per / 2
    r_in = _m.sqrt(s_ * (s_ - a) * (s_ - b) * (s_ - c)) / s_
    k = (r_in - inset) / r_in
    return [(ix + (x - ix) * k, iy + (y - iy) * k) for x, y in (p0, p1, p2)]


def pan_yoke(ring_out_y, tilt_clear_r):
    s = STANDARD
    floor_top = -tilt_clear_r
    floor_bot = floor_top - YOKE_FLOOR_T
    # tilt servo on +Y
    t_origin = V(0, ring_out_y + s["horn"]["hub"], 0)
    t_x, t_z = (0, 0, 1), (0, -1, 0)
    tab_half = s["tab_len"] / 2 + 3
    plate, plate_cuts, bosses = servo_plate(s, (-s["offset"] - tab_half, -s["offset"] + tab_half),
                                            (-YOKE_HALF_X, YOKE_HALF_X), thickness=YOKE_PLATE_T)
    PLATES["pan_yoke"] = dict(servo=s, t=YOKE_PLATE_T, origin=t_origin, x=t_x, z=t_z)
    plate = placed(plate, t_origin, t_x, t_z)
    bosses = placed(bosses, t_origin, t_x, t_z)
    # extend the servo plate down to the floor
    pb = plate.BoundBox
    servo_upright = box(-YOKE_HALF_X, YOKE_HALF_X, pb.YMin, pb.YMax, floor_bot, pb.ZMax)
    plate_cuts = [placed(c, t_origin, t_x, t_z) for c in plate_cuts]

    brg_in = -(ring_out_y + YOKE_GAP)
    brg_out = brg_in - YOKE_BRG_T
    idler_upright = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out, brg_in, floor_bot, 12)
    gusset_len = 25.0
    floor = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out - gusset_len, pb.YMax + gusset_len, floor_bot, floor_top)
    gussets, cuts = [], []
    for x in (-YOKE_HALF_X, YOKE_HALF_X - 5):
        for y_face, out in ((brg_out, -1), (pb.YMax, 1)):
            tri = [(y_face, floor_top), (y_face + out * gusset_len, floor_top), (y_face, floor_top + gusset_len)]
            gussets.append(prism(tri, V(x, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, 5))
            cuts.append(prism(triangle_window(*tri, inset=4.0), V(x, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), -1, 6))
    # horn arm pilots: bosses on the floor top (45 deg cones; the pocket is underneath)
    h = s["horn"]
    rt = h["pilot"] / 2 + BOSS_WALL
    floor_bosses = [Part.makeCone(rt + 3.0, rt, 3.01, V(0, y, floor_top - 0.01), V(0, 0, 1))
                    for y in (-h["len"] / 2 + HORN_SCREW_INSET, h["len"] / 2 - HORN_SCREW_INSET)]

    shape = floor.fuse([servo_upright, idler_upright, bosses] + gussets + floor_bosses)
    cuts += plate_cuts + [bearing_housing_cut(V(0, brg_out, 0), (0, -1, 0), YOKE_BRG_T)]
    # M3 pilots for the striker stand on the idler upright's outer face
    cuts += [Part.makeCylinder(M3_PILOT / 2, YOKE_BRG_T + 2, V(x, brg_out - 1, z), V(0, 1, 0)) for x, z in STAND_BOLTS]
    # lightening windows (prints on its side, so world x is up): uprights below the servo plate and the
    # striker stand, and the floor ends between the gussets
    wx = YOKE_HALF_X - 5.5
    for a, b in row_windows(floor_top + 4, pb.ZMin - 4, 2):
        cuts.append(prism(window_pts(a, b, -wx, wx), V(0, pb.YMin - 1, 0), (0, 0, 1), (1, 0, 0), (0, 1, 0), 0, pb.YMax - pb.YMin + 2))
    stand_bottom = min(z for _, z in STAND_BOLTS) - 14
    for a, b in row_windows(floor_top + 4, stand_bottom, 3):
        cuts.append(prism(window_pts(a, b, -wx, wx), V(0, brg_out - 1, 0), (0, 0, 1), (1, 0, 0), (0, 1, 0), 0, YOKE_BRG_T + 2))
    for y0, y1 in ((brg_out - gusset_len + 3, brg_out - 3), (pb.YMax + 3, pb.YMax + gusset_len - 3)):
        cuts.append(prism(window_pts(y0, y1, -7, 7), V(0, 0, floor_bot - 1), (0, 1, 0), (1, 0, 0), (0, 0, 1), 0, YOKE_FLOOR_T + 2))
    # pan horn on the underside (arm along Y); on its side either x face may be down
    h_o, h_x, h_z = V(0, 0, floor_bot), (0, 1, 0), (0, 0, -1)
    cuts.append(placed(horn_pocket(h, through=YOKE_FLOOR_T + 2, up=[(0, 1), (0, -1)]), h_o, h_x, h_z))
    HORNS["pan_yoke"] = dict(horn=h, origin=h_o, x=h_x, z=h_z)
    shape = shape.cut(Part.makeCompound(cuts)).removeSplitter()
    servo_env = placed(servo_envelope(s), t_origin, t_x, t_z)
    brg_env = bearing_envelope(V(0, brg_out, 0), (0, -1, 0))
    return shape, servo_env, brg_env, floor_bot, brg_out


BENCH_HOLE = 3.5


def base(yoke_floor_bot):
    s = STANDARD
    p_origin = V(0, 0, yoke_floor_bot - s["horn"]["hub"])
    p_x, p_z = (1, 0, 0), (0, 0, 1)
    plate, plate_cuts, bosses = servo_plate(s, (-48, 28), (-35, 35))
    PLATES["base"] = dict(servo=s, t=SERVO_PLATE_T, origin=p_origin, x=p_x, z=p_z)
    plate = placed(plate, p_origin, p_x, p_z)
    bosses = placed(bosses, p_origin, p_x, p_z)
    plate_cuts = [placed(c, p_origin, p_x, p_z) for c in plate_cuts]
    env = placed(servo_envelope(s), p_origin, p_x, p_z)
    pb = plate.BoundBox
    foot_top = env.BoundBox.ZMin - 2.0
    foot_t = 3.0
    foot = box(pb.XMin, pb.XMax, pb.YMin, pb.YMax, foot_top - foot_t, foot_top)
    wall_t = 3.0
    walls = [box(pb.XMin, pb.XMax, pb.YMin, pb.YMin + wall_t, foot_top - foot_t, pb.ZMax),
             box(pb.XMin, pb.XMax, pb.YMax - wall_t, pb.YMax, foot_top - foot_t, pb.ZMax),
             box(pb.XMin, pb.XMin + wall_t, pb.YMin, pb.YMax, foot_top - foot_t, pb.ZMax)]
    # cable exit on the open +X end is free; bench screw holes in the foot corners, with bosses
    bench = [(x, y) for x in (pb.XMin + 8, pb.XMax - 8) for y in (pb.YMin + 8, pb.YMax - 8)]
    rt = BENCH_HOLE / 2 + BOSS_WALL
    bench_bosses = [Part.makeCone(rt + foot_t, rt, foot_t + 0.01, V(x, y, foot_top - 0.01), V(0, 0, 1)) for x, y in bench]
    shape = plate.fuse([foot, bosses] + walls + bench_bosses)
    cuts = plate_cuts
    cuts += [Part.makeCylinder(BENCH_HOLE / 2, 2 * foot_t + 2, V(x, y, foot_top - foot_t - 1)) for x, y in bench]
    # lightening windows (prints on its closed -X end, so world x is up)
    for sy in (-1, 1):
        u0, u1 = sorted((sy * 15.0, sy * (pb.YMax - wall_t - 3)))
        for a, b in row_windows(pb.XMin + wall_t + 3, pb.XMax - 6, 2, web=4.0):
            cuts.append(prism(window_pts(u0, u1, a, b), V(0, 0, pb.ZMin - 1), (0, 1, 0), (1, 0, 0), (0, 0, 1), 0, pb.ZLength + 2))
        y_wall = pb.YMin - 1 if sy < 0 else pb.YMax - wall_t - 1
        for a, b in row_windows(pb.XMin + wall_t + 4, pb.XMax - 4, 2, web=4.0):
            cuts.append(prism(window_pts(foot_top + 4, pb.ZMin - 4, a, b), V(0, y_wall, 0), (0, 0, 1), (1, 0, 0), (0, 1, 0), 0, wall_t + 2))
    cuts.append(prism(window_pts(-18, 18, pb.XMin + 10, pb.XMax - 16), V(0, 0, foot_top - foot_t - 1), (0, 1, 0), (1, 0, 0), (0, 0, 1), 0, foot_t + 2))
    # the closed end lies on the bed, so its window is a plain chamfered rectangle
    yw, z0w, z1w, c = pb.YMax - wall_t - 7, foot_top + 4, pb.ZMin - 4, 3.0
    end = [(-yw + c, z0w), (yw - c, z0w), (yw, z0w + c), (yw, z1w - c), (yw - c, z1w), (-yw + c, z1w), (-yw, z1w - c), (-yw, z0w + c)]
    cuts.append(prism(end, V(pb.XMin - 1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, wall_t + 2))
    return shape.cut(Part.makeCompound(cuts)).removeSplitter(), env


# --- clearance -------------------------------------------------------------
def rotated(shapes, axis, deg):
    out = []
    for s in shapes:
        c = s.copy()
        c.rotate(V(0, 0, 0), V(*axis), deg)
        out.append(c)
    return out


def sweep_check(moving, fixed, axis, label, step=15, tol=1e-3):
    moving_c = Part.makeCompound(moving)
    fixed_c = Part.makeCompound(fixed)
    worst = 0.0
    hits = []
    for deg in range(0, 360, step):
        m = moving_c.copy()
        m.rotate(V(0, 0, 0), V(*axis), deg)
        v = m.common(fixed_c).Volume
        if v > tol:
            hits.append((deg, round(v, 2)))
        worst = max(worst, v)
    print(f"clearance {label}: {'OK through 360 deg' if not hits else 'COLLIDES at ' + str(hits)}", flush=True)
    return hits


def _self_module():
    import types
    m = types.ModuleType("make_gimbal_self")
    m.__dict__.update(globals())
    return m


def striker_limits(roll_parts, rings, striker_parked, striker_inst, yoke_parts, tol=1e-3):
    """Report gimbal range with the striker fitted (hammer parked) and check the installed hammer."""
    parked = Part.makeCompound(striker_parked)
    roll_c = Part.makeCompound(roll_parts)
    hits = []
    for deg in range(-90, 91, 10):
        m = roll_c.copy()
        m.rotate(V(0, 0, 0), V(1, 0, 0), deg)
        if m.common(parked).Volume > tol:
            hits.append(deg)
    print(f"striker vs roll +/-90 (hammer parked): {'OK' if not hits else 'COLLIDES at ' + str(hits)}", flush=True)
    for tab, r in rings.items():
        tilt_c = Part.makeCompound(roll_parts + list(r))
        limits = []
        for sign in (1, -1):
            ok = 0
            for deg in range(5, 181, 5):
                m = tilt_c.copy()
                m.rotate(V(0, 0, 0), V(0, 1, 0), sign * deg)
                if m.common(parked).Volume > tol:
                    break
                ok = deg
            limits.append(sign * ok)
        print(f"striker fitted, MG90S tab {tab}: tilt clear from {limits[1]} to +{limits[0]} deg", flush=True)
    rest = Part.makeCompound(striker_inst)
    v = rest.common(roll_c).Volume
    gap = rest.distToShape(roll_c)[0]
    print(f"hammer at rest: overlap {v:.3f} mm^3, gap to stack {gap:.2f} mm", flush=True)
    v = parked.common(Part.makeCompound(yoke_parts)).Volume
    print(f"striker vs yoke overlap {v:.3f} mm^3", flush=True)


def build(variants=(MG90S_TAB,), check=True):
    os.makedirs(PARTS_DIR, exist_ok=True)
    stack_parts, board_refs = sensor_body()
    roll_parts = [shp for _, shp in stack_parts] + [b for _, b in board_refs]
    roll_r = max_radius(roll_parts, (1, 0, 0))
    ring_in_y = roll_r + SWEEP_MARGIN
    pinned = json.load(open(PIN)) if PIN else None
    if pinned:
        print(f"roll sweep radius {roll_r:.1f} (pinned ring inner {pinned['ring_in_y']:.1f})", flush=True)
        ring_in_y = pinned["ring_in_y"]
        print(f"  clearance to the pinned ring: {ring_in_y - roll_r:.1f} mm (design margin {SWEEP_MARGIN})", flush=True)
        assert ring_in_y - roll_r >= PIN_MIN_CLEAR, "variant stack no longer fits the base ring"
    else:
        print(f"roll sweep radius {roll_r:.1f} -> ring inner {ring_in_y:.1f}", flush=True)

    rings = {tab: tilt_ring(mg90s(tab), ring_in_y) for tab in variants}
    tilt_r = max(max_radius(roll_parts + list(r), (0, 1, 0)) for r in rings.values())
    tilt_clear_r = tilt_r + 3.0
    if pinned:
        assert pinned["tilt_clear_r"] - tilt_r >= PIN_MIN_CLEAR, "variant no longer fits the base yoke"
        tilt_clear_r = pinned["tilt_clear_r"]
    print(f"tilt sweep radius {tilt_r:.1f} -> yoke floor at z={-tilt_clear_r:.1f}", flush=True)
    with open(os.path.join(OUT_DIR, "build_info.json"), "w") as f:
        json.dump(dict(ring_in_y=ring_in_y, tilt_clear_r=tilt_clear_r, stack_gaps=list(ss.GAPS),
                       roof_raise=ss.roof_raise()), f, indent=1)

    yoke, tilt_servo, yoke_brg, yoke_floor_bot, upright_out_y = pan_yoke(ring_in_y + RING_BAR_T, tilt_clear_r)
    base_shape, pan_servo = base(yoke_floor_bot)

    import make_striker as mk
    st = mk.build(dict(upright_out_y=upright_out_y, roof_z=ss.roof_top() - ss.axis_z(), roof_raise=ss.roof_raise(),
                       upright_half_x=YOKE_HALF_X, stand_bolts=STAND_BOLTS), _self_module())
    print("striker", st["info"], flush=True)

    horns = [("horn_mg90s_printed", printed_horn(MG90S)), ("horn_ds3240_printed", printed_horn(DS3240))]
    printed = stack_parts + horns + [("pan_yoke", yoke), ("base", base_shape)] + \
              [("striker_bar", st["bar_free"]), ("striker_pawl", st["pawl_rest"]),
               ("striker_cam", st["cam"]), ("striker_cam_spline", st["cam_spline"]),
               ("striker_stand", st["stand"])] + \
              [("tilt_ring_mg90s" if len(rings) == 1 else f"tilt_ring_mg90s_tab{t:g}", r[0]) for t, r in rings.items()]
    for name, shp in printed:
        print(f"part {name}: valid {shp.isValid()} solids {len(shp.Solids)} bbox "
              f"{[round(v, 1) for v in (shp.BoundBox.XLength, shp.BoundBox.YLength, shp.BoundBox.ZLength)]}",
              flush=True)
        assert shp.isValid() and len(shp.Solids) == 1, name

    if check:
        for tab, (ring, roll_servo, ring_brg) in rings.items():
            print(f"-- MG90S tab {tab} mm", flush=True)
            sweep_check(roll_parts, [ring, roll_servo, ring_brg], (1, 0, 0), "roll: sensor stack vs ring")
            sweep_check(roll_parts + [ring, roll_servo, ring_brg], [yoke, tilt_servo, yoke_brg], (0, 1, 0),
                        "tilt: ring assembly vs yoke")
        striker_parked = [st["bar_park"], st["pawl_park"], st["cam"], st["stand"], st["servo"]]
        pan_parts = [yoke, tilt_servo, yoke_brg] + roll_parts + list(rings[variants[0]]) + striker_parked
        sweep_check(pan_parts, [base_shape, pan_servo], (0, 0, 1), "pan: yoke + striker vs base")
        striker_limits(roll_parts, rings, striker_parked, [st["bar_rest"], st["pawl_rest"]],
                       [yoke, tilt_servo, yoke_brg])

    doc = App.newDocument("imu_gimbal_rig")

    def add(name, shp, label, visible=True):
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        o.Label = label
        o.Visibility = visible
        return o

    labels = {"stack_middle_deck": "Middle deck + roll walls (wing + 5 QT)", "stack_top_deck": "Top deck (8 QT) + roof",
              "stack_bottom_deck": "Bottom deck (8 QT)", "stack_spine_upper": "Upper spine",
              "stack_spine_lower": "Lower spine"}
    for name, shp in stack_parts:
        add(name.title().replace("_", ""), shp, labels[name])
    for name, shp in board_refs:
        add(f"REF_{name}", shp, f"REF {name}")
    t0 = variants[0]
    add("TiltRing", rings[t0][0], f"Tilt ring (MG90S tab {t0} mm)")
    add("RollServo", rings[t0][1], f"REF MG90S roll servo (tab {t0} mm)")
    add("RollBearing", rings[t0][2], "REF 623ZZ roll idler")
    for t in variants[1:]:
        add(f"TiltRingTab{int(t)}", rings[t][0], f"Tilt ring (MG90S tab {t} mm) ALT", visible=False)
    add("PanYoke", yoke, "Pan yoke")
    add("TiltServo", tilt_servo, "REF standard servo, tilt (MG995 / DS3240)")
    add("TiltBearing", yoke_brg, "REF 623ZZ tilt idler")
    add("Base", base_shape, "Base")
    add("PanServo", pan_servo, "REF standard servo, pan (DS3240 270)")
    add("StrikerStand", st["stand"], "Striker stand")
    add("StrikerBar", st["bar_rest"], "Striker bar (at rest, hammer hovering)")
    add("StrikerPawl", st["pawl_rest"], "Striker pawl")
    add("StrikerCam", st["cam"], "Striker cam")
    add("StrikerServo", st["servo"], "REF MG90S-size 270 deg servo (cam)")
    add("StrikerBarParked", st["bar_park"], "REF striker bar parked (lifted)", visible=False)
    add("StrikerBarFree", st["bar_free"], "REF striker bar as printed (free shape)", visible=False)
    doc.recompute()
    path = os.path.join(OUT_DIR, "imu-gimbal-assembly.FCStd")
    doc.saveAs(path)
    print("wrote", path, flush=True)

    for name, shp in printed:
        p = os.path.join(PARTS_DIR, name)
        shp.exportStep(p + ".step")
        Mesh.Mesh(shp.tessellate(0.05)).write(p + ".stl")
    print("wrote parts to", PARTS_DIR, flush=True)


if __name__ == "__main__":
    build()
