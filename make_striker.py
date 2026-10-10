r"""Tap striker: a rigid hammer arm on a high pivot, so the gimbal can tilt all the way over.

Imported by make_gimbal.py. The tower bolts to the pan yoke's idler upright (4 x M3, the
same pilots as before) and carries everything well outside the tilt sweep.

Layout (section at x = 0; +Y toward the stack, +Z up)

                  P  pivot (M3 bolt in a printed bushing, top of the tower)
       stop -> |=|o-------------- beam ----------.------ overarm --[screw]   (nylon M3 stop)
                  |\                             '-- head leaf ----[HEAD]   (lost-motion leaf)
           tail   | |  slot                                          roof
                  |o|  <- M3 pin in the drive leaf's tip fork
                  | |
       pawl  o====|  <- drive leaf (vertical PETG, pushes the tail -Y = hammer down)
      cam ( )     |
     servo        |__ root pad, screwed to the seat on the tower

How it works
- The drive leaf is printed straight. Its root pad sits on a seat that leans back by a few
  degrees, so once its tip fork is pinned to the arm's tail it pushes the tail -Y, which
  holds the hammer arm down on its rest stop (the tail lands on a cross-bar).
  At rest the head hovers HOVER (5 mm) above the roof.
- A 270 deg servo turns a two-lobe cam (forward = -X rotation). A pawl pinned to the drive
  leaf rides up a ramp, bending the leaf +Y. Through the pin and slot this swings the arm
  PARK_DEG up, and the hammer parks far outside the tilt sweep. At the cliff the pawl
  drops off, and the leaf throws the arm down onto its rest stop.
- Let-off: the head sits on its own thin PETG leaf. A nylon M3 screw in the overarm presses
  the leaf down, preloading it (HEAD_PRELOAD) against the screw. When the arm hits its
  stop, the head's momentum bends the leaf on through the 5 mm gap into the roof, and the
  head springs back onto the screw, where the preload stops it bouncing back into the roof.
- The pawl hangs off the leaf pointing -Y and leans PAWL_LEAN down onto its stop. The cam's
  drag and gravity both hold it there. On the reverse stroke the cliffs fold it up, and
  gravity drops it back. Two lobes fit within the 270 deg travel:
      park at ~120 deg -> sweep forward through 125 and 245: double tap
      park at ~240 deg -> step past 245: single tap
  To re-arm, turn the servo back to 0.
- Keep the hammer parked whenever the gimbal moves.

Spring models: small-deflection Euler-Bernoulli cantilevers (E for printed PETG ~2 GPa),
solved in leaf_design() and head_leaf_design(); the numbers go in build()['info'].
"""

import math
import os
import sys

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

V = App.Vector

# --- targets -------------------------------------------------------------------
HOVER = 5.0            # mm, hammer face above the roof at rest
PARK_DEG = 48.0        # arm swing from rest to parked
TAP_ENERGY = 36.0      # mJ, head kinetic energy at let-off (what reaches the roof)
F_CONTACT = 2.0        # N, head-leaf force when the head touches the roof (preload + rate x HOVER)
OVERSHOOT_MIN = 3.0    # x HOVER: free let-off travel the head must have
E_PETG = 2000.0        # N/mm^2
RHO_PETG = 1.27e-6     # kg/mm^3
G = 9.81               # m/s^2 (mass in kg -> N; N x mm = mJ)
STRAIN_PARK = 0.013    # drive leaf peak strain when parked

# --- arm (2.5D profile, 12 mm thick, prints on its side) -------------------------
ARM_W = 12.0           # x thickness of the whole arm
PIVOT_BACK = 28.0      # pivot behind the upright's outer face
PIVOT_UP = 66.0        # pivot above the roof top
HUB_R = 8.0
PIVOT_HOLE = 6.3       # hub bore: rides on the printed bushing tube (6.0) round the M3 bolt
TAIL_LEVER = 18.0      # pivot to the drive pin at rest (vertical lever)
TAIL_W = 10.0
TAIL_LEN = 23.0
PIN_HOLE = 3.4         # M3 drive pin: slides in the leaf's tip slot (this wide)
TAIL_PILOT = 2.8       # ...and self-taps into the arm's tail, so it is fixed there
BEAM_H = 10.0
HEAD_L, HEAD_H = 20.0, 22.0    # head along y, height
HEAD_LEAF_L = 50.0     # head leaf, root to head centre
HEAD_LEAF_GAP = 1.5    # as printed, leaf top to overarm underside
OVERARM_T = 4.0
SCREW_PILOT = 3.4      # M3 nylon stop screw, clearance: a nylon nut each side of the boss sets the preload
STOP_BAR = 6.0         # rest-stop cross-bar section
BUMPER_T = 2.0         # the stop face is a lip this thick over a slot: a little give at landing

# --- drive leaf ----------------------------------------------------------------
LEAF_B = 20.0          # width (x)
LEAF_L = 68.0          # free length, root to pin
LEAF_PAD = 14.0        # root pad height
LEAF_PAD_T = 6.0       # root pad thickness (y)
FORK_H = 12.0          # tip block height
FORK_T = 7.0           # tip block thickness (y)
FORK_SLOT = 6.4        # half-width of the slot the tail runs in
FOLLOW_A = 40.0        # pawl lugs above the root

# --- pawl and cam (as the old striker; pin is 1.75 mm filament) -----------------
LUG_DROP = 3.5
PAWL_LEN = 14.0       # pin to tip
PAWL_T = 4.4
PAWL_W = 5.0
LUG_T = 3.0
PIN_D = 1.9
PAWL_LEAN = 16.0        # deg the pawl leans past the cam's force line, onto its stop (gravity and drag agree)
CAM_R0 = 8.0
CAM_T = 6.0
CAM_HUB = 2.5
CAM_GAP = 0.4          # pawl clear of the base circle at rest
CAM_BETA0 = 45.0       # deg, contact on the cam above its +Y side (keeps the drop clear of the lobe)
LOBES = ((15, 115, 125), (135, 235, 245))

# --- tower -----------------------------------------------------------------------
STAND_T = 5.0          # base plate on the upright
WALL_T = 5.0
WALL_X = 20.0          # wall inner face |x|
BASE_BOTTOM = -20.0
BASE_TOP = 12.0
STAND_BOLT_HOLE = 3.4
PLATE_SIDE = 4.0
WEB_FRAME = 4.0
BOSS_R = 6.0           # pivot boss radius on the walls


# --- helpers ---------------------------------------------------------------------
def rot_yz(p, c, deg):
    """Rotate (y, z) point p about c by deg (positive = about +X: +Y turns toward +Z)."""
    a = math.radians(deg)
    y, z = p[0] - c[0], p[1] - c[1]
    return (c[0] + y * math.cos(a) - z * math.sin(a), c[1] + y * math.sin(a) + z * math.cos(a))


def prism_yz(pts, x0, x1):
    """Polygon in (y, z), extruded along x from x0 to x1."""
    p = [V(x0, y, z) for y, z in pts]
    return Part.Face(Part.makePolygon(p + [p[0]])).extrude(V(x1 - x0, 0, 0))


def cyl_x(r, y, z, x0, x1):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def bar_yz(a, b, w, x0, x1):
    """Straight bar of width w between (y, z) points a and b, extruded along x."""
    dy, dz = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dy, dz)
    ny, nz = -dz / n * w / 2, dy / n * w / 2
    return prism_yz([(a[0] + ny, a[1] + nz), (b[0] + ny, b[1] + nz), (b[0] - ny, b[1] - nz), (a[0] - ny, a[1] - nz)], x0, x1)


def slot_yz(a, b, w, x0, x1):
    """Rounded slot of width w from a to b (centres), along x."""
    return bar_yz(a, b, w, x0, x1).fuse([cyl_x(w / 2, a[0], a[1], x0, x1), cyl_x(w / 2, b[0], b[1], x0, x1)])


# --- spring models ---------------------------------------------------------------------
def leaf_k(b, t, L):
    """Tip stiffness of a cantilever (N/mm)."""
    return E_PETG * b * t ** 3 / (4 * L ** 3)


def leaf_design(work, travel, L=LEAF_L, b=LEAF_B, strain=STRAIN_PARK):
    """Drive leaf: thickness t and rest deflection d_r so that the spring gives up `work` (mJ)
    as its tip runs back `travel` (mm) from park to rest, with the parked strain at `strain`.
    Strain of a tip-loaded cantilever at the root: 3 t d / (2 L^2)."""
    def solve(t):
        d_r = 2 * L ** 2 * strain / (3 * t) - travel
        return d_r, leaf_k(b, t, L) * travel * (d_r + travel / 2)
    lo, hi = 0.6, 6.0
    for _ in range(60):
        t = (lo + hi) / 2
        if solve(t)[1] < work:
            lo = t
        else:
            hi = t
    t = (lo + hi) / 2
    d_r = solve(t)[0]
    if d_r < 1.0:                       # keep some preload at rest: hold d_r = 1, ease the strain
        d_r = 1.0
        lo, hi = 0.6, 6.0
        for _ in range(60):
            t = (lo + hi) / 2
            if leaf_k(b, t, L) * travel * (d_r + travel / 2) < work:
                lo = t
            else:
                hi = t
        t = (lo + hi) / 2
    k = leaf_k(b, t, L)
    return dict(t=round(t, 2), k=k, d_rest=d_r, f_rest=k * d_r, f_park=k * (d_r + travel),
                strain_rest=3 * t * d_r / (2 * L ** 2), strain_park=3 * t * (d_r + travel) / (2 * L ** 2))


def head_leaf_design(ke, L=HEAD_LEAF_L, b=ARM_W, gap=HOVER, f_contact=F_CONTACT, factor=OVERSHOOT_MIN, margin=0.75):
    """Lost-motion head leaf. The stop screw preloads it with P; at the roof (gap below the
    stop) it pushes with P + k gap = f_contact. With head energy ke (mJ) the free let-off
    travel d solves P d + k d^2 / 2 = ke; it must be >= factor x gap. Picks k at `margin`
    of the largest stiffness that still gives that travel."""
    D = factor * gap
    k_max = (ke - f_contact * D) / (D * D / 2 - gap * D)
    if k_max <= 0:
        raise ValueError(f"head energy {ke:.1f} mJ cannot give {D} mm let-off at {f_contact} N contact")
    k = margin * k_max
    p = f_contact - k * gap
    d = (-p + math.sqrt(p * p + 2 * k * ke)) / k
    t = (12 * k * L ** 3 / (3 * E_PETG * b)) ** (1 / 3)
    return dict(t=round(t, 2), k=k, preload=p, pre_defl=p / k, overshoot=d,
                strain_rest=3 * t * (p / k) / (2 * L ** 2), strain_contact=3 * t * (p / k + gap) / (2 * L ** 2))


class DriveLeaf:
    """Vertical drive leaf, clamped at (y_root, z_root) on a seat leaning back by beta, tip at the pin.
    Shapes are small-deflection cantilever superpositions; points are (s, y, z, heading)
    stations like the old bar, heading measured from +Y (pi/2 = straight up)."""

    def __init__(self, y_root, z_root, t, d_rest):
        self.y0, self.z0, self.t = y_root, z_root, t
        self.L = LEAF_L
        self.EI = E_PETG * LEAF_B * t ** 3 / 12
        self.d_rest = d_rest
        self.beta = d_rest / self.L           # seat lean (rad): the installed tip sits over the root

    def g(self, s, a):
        """Deflection at s for a unit load at a."""
        if s <= a:
            return s * s * (3 * a - s) / (6 * self.EI)
        return a * a * (3 * s - a) / (6 * self.EI)

    def gp(self, s, a):
        """Slope at s for a unit load at a."""
        if s <= a:
            return s * (2 * a - s) / (2 * self.EI)
        return a * a / (2 * self.EI)

    def loads_for_tip(self, d_tip, f_tip=0.0, a=FOLLOW_A):
        """Cam load at a that puts the tip d_tip (from free) with f_tip pushing the tip back (-Y)."""
        return (d_tip + f_tip * self.g(self.L, self.L)) / self.g(self.L, a)

    def shape(self, p_cam=0.0, f_tip=0.0, a=FOLLOW_A, tip_prop=None, n=120):
        """Leaf stations. tip_prop: tip held at this deflection from free by an unknown tip force (rest)."""
        if tip_prop is not None:                                   # rest: only the tip reaction
            f = -tip_prop / self.g(self.L, self.L)
            loads = [(p_cam, a), (-f, self.L)] if p_cam else [(-f, self.L)]
        else:
            loads = [(p_cam, a), (-f_tip, self.L)]
        pts, z, ds = [], self.z0, self.L / n
        for i in range(n + 1):
            s = i * ds
            v = sum(P * self.g(s, aa) for P, aa in loads)
            vp = sum(P * self.gp(s, aa) for P, aa in loads)
            dy = -self.beta + vp
            y = self.y0 - self.beta * s + v
            if i:
                z += ds * (1 - dy * dy / 2)          # small-slope shortening
            pts.append((s, y, z, math.atan2(1.0, dy)))
        return pts

    def tip_force(self, d_tip):
        return d_tip / self.g(self.L, self.L)


def station(pts, s):
    ds = pts[1][0] - pts[0][0]
    return pts[min(int(round(s / ds)), len(pts) - 1)]


def _local(st, u, w):
    _, y, z, h = st
    return (y + u * math.cos(h) - w * math.sin(h), z + u * math.sin(h) + w * math.cos(h))


def _local_poly(st, uw, x0, x1):
    return prism_yz([_local(st, u, w) for u, w in uw], x0, x1)


def _local_box(st, u0, u1, w0, w1, x0, x1):
    return _local_poly(st, [(u0, w0), (u1, w0), (u1, w1), (u0, w1)], x0, x1)


def _local_cyl_x(st, u, w, r, x0, x1):
    y, z = _local(st, u, w)
    return cyl_x(r, y, z, x0, x1)


# --- hammer arm ------------------------------------------------------------------------------
def arm_layout(g, th, pre):
    """Key (y, z) positions of the arm at rest. th = head-leaf thickness, pre = its preload deflection."""
    y_uo, z_roof = g["upright_out_y"], g["roof_z"]
    P = (y_uo - PIVOT_BACK, z_roof + PIVOT_UP)
    z_face = z_roof + HOVER
    z_lc = z_face + HEAD_H + th / 2 + pre            # head-leaf root centreline
    z_oa = z_lc + th / 2 + HEAD_LEAF_GAP              # overarm underside
    return dict(P=P, pin=(P[0], P[1] - TAIL_LEVER), z_roof=z_roof, z_face=z_face, z_lc=z_lc, z_oa=z_oa,
                y_root=-HEAD_LEAF_L, th=th, pre=pre)


def head_leaf_line(lay, installed):
    """Head-leaf centreline [(y, z)] from its root to the head's far end, and the tip slope (rad)."""
    L, pre, y0, z0 = HEAD_LEAF_L, lay["pre"], lay["y_root"], lay["z_lc"]
    pts = []
    for i in range(41):
        s = L * i / 40
        pts.append((y0 + s, z0 - (pre * s * s * (3 * L - s) / (2 * L ** 3) if installed else 0.0)))
    slope = 1.5 * pre / L if installed else 0.0
    end = HEAD_L / 2
    pts.append((pts[-1][0] + end, pts[-1][1] - end * math.tan(slope)))
    return pts, slope


def _head(lay):
    """Head at rest (installed, level): rounded face down to z_face, top under the leaf."""
    zf, top = lay["z_face"], lay["z_face"] + HEAD_H
    r = HEAD_L / 2
    body = prism_yz([(-r, zf + r), (r, zf + r), (r, top + 0.01), (-r, top + 0.01)], -ARM_W / 2, ARM_W / 2)
    return body.fuse(cyl_x(r, 0.0, zf + r, -ARM_W / 2, ARM_W / 2))


def arm_parts(lay, installed=True):
    """(body, head) solids of the hammer arm at rest. installed=False gives the as-printed shape:
    head leaf straight, head tilted up by the installed tip slope."""
    x0, x1 = -ARM_W / 2, ARM_W / 2
    P, th = lay["P"], lay["th"]
    line, slope = head_leaf_line(lay, installed)
    up = [(y, z + th / 2) for y, z in line]
    dn = [(y, z - th / 2) for y, z in line]
    leaf = prism_yz(up + dn[::-1], x0, x1)
    head = _head(lay)
    if not installed:
        tip = (0.0, lay["z_lc"] - lay["pre"])
        head.translate(V(0, 0, lay["pre"]))
        head.rotate(V(0, tip[0], tip[1] + lay["pre"]), V(1, 0, 0), math.degrees(1.5 * lay["pre"] / HEAD_LEAF_L))
    head = head.fuse(leaf.common(Part.makeBox(ARM_W, HEAD_L, 100, V(x0, -HEAD_L / 2, lay["z_face"]))))
    yr, zo = lay["y_root"], lay["z_oa"]
    z_top = zo + OVERARM_T
    root = prism_yz([(yr - 8, lay["z_lc"] - th / 2 - 3), (yr + 0.01, lay["z_lc"] - th / 2 - 3),
                     (yr + 0.01, z_top), (yr - 8, z_top)], x0, x1)
    overarm = prism_yz([(yr - 1, zo), (HEAD_L / 2 - 2, zo), (HEAD_L / 2 - 2, z_top), (yr - 1, z_top)], x0, x1)
    boss = prism_yz([(-5, z_top - 0.01), (5, z_top - 0.01), (5, z_top + 4), (-5, z_top + 4)], x0, x1)
    beam = bar_yz(P, (yr - 4, (lay["z_lc"] + z_top) / 2 - 2), BEAM_H, x0, x1)
    hub = cyl_x(HUB_R, P[0], P[1], x0, x1)
    tail = bar_yz(P, (P[0], P[1] - TAIL_LEN), TAIL_W, x0, x1)
    body = hub.fuse([tail, beam, root, overarm, boss, leaf.cut(Part.makeBox(ARM_W + 2, HEAD_L, 100, V(x0 - 1, -HEAD_L / 2, lay["z_face"])))])
    cuts = [cyl_x(PIVOT_HOLE / 2, P[0], P[1], x0 - 1, x1 + 1),
            cyl_x(TAIL_PILOT / 2, P[0], P[1] - TAIL_LEVER, x0 - 1, x1 + 1),
            Part.makeCylinder(SCREW_PILOT / 2, OVERARM_T + 6, V(0, 0, zo - 1))]
    # lightening window in the beam (2.5D, prints on its side)
    a, b = P, (yr - 4, (lay["z_lc"] + z_top) / 2 - 2)
    n = math.hypot(b[0] - a[0], b[1] - a[1])
    u = ((b[0] - a[0]) / n, (b[1] - a[1]) / n)
    win = slot_yz((a[0] + u[0] * (HUB_R + 5), a[1] + u[1] * (HUB_R + 5)), (b[0] - u[0] * 9, b[1] - u[1] * 9),
                  BEAM_H - 5, x0 - 1, x1 + 1)
    cuts.append(win)
    body = body.cut(Part.makeCompound(cuts)).removeSplitter()
    return body, head.removeSplitter()


# --- drive leaf and pawl ---------------------------------------------------------------------
PIN_TRAVEL = 9.0       # slot length in the leaf's tip block for the pin's rise


def _profile(pts, t):
    left, right = [], []
    for _, y, z, h in pts:
        ny, nz = -math.sin(h), math.cos(h)
        left.append((y + ny * t / 2, z + nz * t / 2))
        right.append((y - ny * t / 2, z - nz * t / 2))
    poly = left + right[::-1]
    return [p for i, p in enumerate(poly) if i % 3 == 0 or i == len(poly) - 1]


def _cyl_w(st, u, x, r, w0, w1):
    """Cylinder along the station's w axis (through the leaf)."""
    _, y, z, h = st
    dw = V(0, -math.sin(h), math.cos(h))
    a = _local(st, u, w0)
    return Part.makeCylinder(r, w1 - w0, V(x, a[0], a[1]), dw)


def leaf_solid(leaf, pts):
    """Drive leaf with root pad, pawl lugs and stop, and the tip fork with the pin slot."""
    t, b = leaf.t, LEAF_B
    x0, x1 = -b / 2, b / 2
    body = prism_yz(_profile(pts, t), x0, x1)
    s0, sa, st = station(pts, 0), station(pts, FOLLOW_A), station(pts, leaf.L)
    # everything stands off the -Y face (w > 0): the leaf prints flat on its +Y face
    wc = FORK_T / 2 - t / 2                                                        # pin line
    parts = [_local_box(s0, -LEAF_PAD, 0.5, -t / 2, t / 2 + LEAF_PAD_T, x0, x1),        # root pad
             _local_box(st, -7.0, PIN_TRAVEL + 3.0, -t / 2, FORK_T - t / 2, x0, x1)]    # tip block
    # pawl lugs (on the -Y face, w > 0) and the stop under the pawl
    w_pin = t / 2 + LUG_DROP
    gap = PAWL_T / 2 + 0.4
    for a, c in ((-gap - LUG_T, -gap), (gap, gap + LUG_T)):
        parts.append(_local_box(sa, -3.5, 3.5, t / 2 - 0.01, w_pin + 3.0, a, c))
    parts.append(_local_poly(sa, _stop_uw(t), -gap - 0.01, gap + 0.01))
    body = body.fuse(parts)
    cuts = [_local_box(st, -7.5, PIN_TRAVEL + 4.0, -FORK_T, FORK_T, -FORK_SLOT, FORK_SLOT),   # tail runs here
            slot_yz(_local(st, 0, wc), _local(st, PIN_TRAVEL, wc), PIN_HOLE, x0 - 1, x1 + 1),
            _local_cyl_x(sa, 0, w_pin, PIN_D / 2, -gap - LUG_T - 1, gap + LUG_T + 1)]
    for u in (-4.0, -LEAF_PAD + 4.0):                     # pad screws (M3 clearance) into the seat
        for x in (-b / 4, b / 4):
            cuts.append(_cyl_w(s0, u, x, 1.7, -t / 2 - 1, t / 2 + LEAF_PAD_T + 1))
    return body.cut(Part.makeCompound(cuts)).removeSplitter()


def _pawl_dir():
    """Pawl axis in the leaf frame at the follower: along +w (-Y), tipped down (-u) so it points
    at the cam centre, plus PAWL_LEAN. The cam's push, its forward drag and gravity all turn it
    the same way, onto the stop under it; the reverse stroke folds it up and back."""
    th = math.radians(CAM_BETA0 + PAWL_LEAN)
    return (-math.sin(th), math.cos(th))


def _stop_uw(t):
    """Stop under the pawl (on its -u side), 0.3 mm off its flank, from the leaf face to 3 mm past the pin."""
    w_pin = t / 2 + LUG_DROP
    du, dw = _pawl_dir()
    nu, nw = -dw, du                                # normal toward -u
    off = PAWL_W / 2 + 0.3
    l0, l1 = -(LUG_DROP + 0.01) / dw, 3.0 / dw
    p0 = (l0 * du + off * nu, w_pin + l0 * dw + off * nw)
    p1 = (l1 * du + off * nu, w_pin + l1 * dw + off * nw)
    u_back = -PAWL_W / 2 - 4.5
    return [p0, p1, (u_back, p1[1]), (u_back, p0[1])]


def pawl_tip(leaf, pts):
    """World (y, z) of the pawl's tip-circle centre."""
    sa = station(pts, FOLLOW_A)
    du, dw = _pawl_dir()
    w_pin = leaf.t / 2 + LUG_DROP
    l = PAWL_LEN - PAWL_W / 2
    return _local(sa, l * du, w_pin + l * dw)


def pawl_solid(leaf, pts):
    sa = station(pts, FOLLOW_A)
    w_pin = leaf.t / 2 + LUG_DROP
    pin = _local(sa, 0, w_pin)
    shape = slot_yz(pin, pawl_tip(leaf, pts), PAWL_W, -PAWL_T / 2, PAWL_T / 2)
    return shape.cut(cyl_x(PIN_D / 2 + 0.05, pin[0], pin[1], -PAWL_T, PAWL_T)).removeSplitter()


# --- cam ---------------------------------------------------------------------------------------
def cam_radius(psi, b0, bp, h):
    """Cam radius at cam-frame angle psi (deg). Forward is -X rotation, so the follower, at
    world angle beta, sits over psi = beta + s. The ramps run from b0 + a0 to bp + a1."""
    psi %= 360
    for a0, a1, c in LOBES:
        p0, p1, pc = b0 + a0, bp + a1, bp + c
        if p0 <= psi < p1:
            return CAM_R0 + h * (psi - p0) / (p1 - p0)
        if p1 <= psi < pc:
            return CAM_R0 + h
    return CAM_R0


def cam_solid(K, b0, bp, h, horn):
    """Cam in the YZ plane about +X through K = (y, z), drawn at servo 0."""
    pts = []
    cliffs = [bp + c for _, _, c in LOBES]
    for i in range(720):
        psi = i / 2
        for pc in cliffs:
            if psi - 0.5 < pc <= psi:
                pts.append((pc - 1e-3, cam_radius(pc - 1e-3, b0, bp, h)))
                pts.append((pc, CAM_R0))
        pts.append((psi, cam_radius(psi, b0, bp, h)))
    poly = [(K[0] + r * math.cos(math.radians(p)), K[1] + r * math.sin(math.radians(p))) for p, r in pts]
    cam = prism_yz(poly, -CAM_T / 2, CAM_T / 2)
    cuts = [cyl_x(2.5, K[0], K[1], -CAM_T / 2 - 1, CAM_T / 2 + 1), horn]
    return cam.cut(Part.makeCompound(cuts)).removeSplitter()


def cam_spline_solid(K, b0, bp, h, spline_x, servo, mg):
    """Cam with the MG90S spline moulded in (no horn); M2 centre screw from the +X face."""
    sp = servo["spline"]
    base = cam_solid(K, b0, bp, h, Part.makeBox(0.1, 0.1, 0.1, V(1000, 0, 0)))
    base = base.fuse(cyl_x(2.6, K[0], K[1], -CAM_T / 2, CAM_T / 2))
    hub_face = spline_x - sp["engage"]
    hub = cyl_x(4.8, K[0], K[1], hub_face, -CAM_T / 2 + 0.01)
    shape = base.fuse(hub)
    sock = mg.placed(mg.spline_socket(sp, sp["engage"] + 0.01), V(hub_face, K[0], K[1]), (0, 1, 0), (1, 0, 0))
    screw = cyl_x(sp["screw"] / 2, K[0], K[1], hub_face - 1, CAM_T / 2 + 20)
    head = cyl_x(sp["head"] / 2, K[0], K[1], CAM_T / 2 - 1.8, CAM_T / 2 + 0.01)
    return shape.cut(Part.makeCompound([sock, screw, head])).removeSplitter()


# --- mechanics -------------------------------------------------------------------------------
def _mass_props(shapes, P):
    """mass (kg), CoM (y, z), inertia about the pivot axis (kg mm^2) of PETG solids."""
    m, my, mz, inertia = 0.0, 0.0, 0.0, 0.0
    for s in [so for sh in shapes for so in sh.Solids]:
        mi = s.Volume * RHO_PETG
        c = s.CenterOfMass
        d2 = (c.y - P[0]) ** 2 + (c.z - P[1]) ** 2
        inertia += RHO_PETG * s.MatrixOfInertia.A11 + mi * d2
        m, my, mz = m + mi, my + mi * c.y, mz + mi * c.z
    return m, (my / m, mz / m), inertia


def solve(g):
    """Size both leaves and place the cam. Returns (numbers, state)."""
    hl = head_leaf_design(TAP_ENERGY)
    lay = arm_layout(g, hl["t"], hl["pre_defl"])
    P = lay["P"]
    body, head = arm_parts(lay)
    m_arm, cm, i_arm = _mass_props([body, head], P)
    _, _, i_head = _mass_props([head], P)
    pin0 = lay["pin"]
    pin1 = rot_yz(pin0, P, PARK_DEG)
    travel = pin1[0] - pin0[0]
    cm1 = rot_yz(cm, P, PARK_DEG)
    w_grav = m_arm * G * (cm1[1] - cm[1])                      # mJ (kg * m/s^2 * mm)
    # the drive leaf's tip and fork move with the pin (lumped: 0.24 of the leaf + the block)
    m_leaf = RHO_PETG * (LEAF_B * 2.0 * LEAF_L * 0.24 + LEAF_B * FORK_T * 19)
    i_tot = i_arm + m_leaf * TAIL_LEVER ** 2
    share = i_head / i_tot
    ke_tot = TAP_ENERGY / share
    leaf_d = leaf_design(ke_tot - w_grav, travel)
    # the pin rides FORK_T / 2 off the leaf's +Y face, so the leaf stands that much +Y of it
    leaf = DriveLeaf(pin0[0] + FORK_T / 2 - leaf_d["t"] / 2, pin0[1] - LEAF_L, leaf_d["t"], leaf_d["d_rest"])
    # rest: tip propped by the arm on its stop; park: cam load with the arm's weight on the tip
    lever1 = P[1] - pin1[1]
    f_tip_park = max(0.0, m_arm * G * (cm1[0] - P[0]) / lever1)
    rest = leaf.shape(tip_prop=leaf.d_rest)
    p_cam = leaf.loads_for_tip(leaf.d_rest + travel, f_tip_park)
    park = leaf.shape(p_cam=p_cam, f_tip=f_tip_park)
    free = leaf.shape()
    # cam: rest contact at CAM_BETA0 on the cam, CAM_GAP clear of the base circle
    c0, c1 = pawl_tip(leaf, rest), pawl_tip(leaf, park)
    r_c = CAM_R0 + CAM_GAP + PAWL_W / 2
    b0 = math.radians(CAM_BETA0)
    K = (c0[0] - r_c * math.cos(b0), c0[1] - r_c * math.sin(b0))
    h = math.hypot(c1[0] - K[0], c1[1] - K[1]) - PAWL_W / 2 - CAM_R0
    bp = math.degrees(math.atan2(c1[1] - K[1], c1[0] - K[0]))
    assert bp < CAM_BETA0, f"pawl would land back on the lobe (beta park {bp:.1f} >= {CAM_BETA0})"
    ramp = math.radians(LOBES[0][1] - LOBES[0][0])
    du, dw = _pawl_dir()
    leans = []
    for pts, beta in ((rest, CAM_BETA0), (park, bp)):
        d = _local((0, 0.0, 0.0, station(pts, FOLLOW_A)[3]), du, dw)
        leans.append((math.degrees(math.atan2(d[1], d[0])) - (180 + beta) + 180) % 360 - 180)
    assert min(leans) > 0, f"pawl lean off the cam's force line {leans}: the load would fold it"
    t_rest = leaf.tip_force(leaf.d_rest) * TAIL_LEVER + m_arm * G * (cm[0] - P[0])
    pin_rise = (pin1[1] - pin0[1]) - (park[-1][2] - rest[-1][2])
    assert pin_rise < PIN_TRAVEL - 0.5, f"pin rises {pin_rise:.1f} mm in the leaf's slot"
    ke_h = TAP_ENERGY
    roof_e = ke_h - hl["preload"] * HOVER - hl["k"] * HOVER ** 2 / 2
    num = dict(
        head_leaf_t=hl["t"], head_leaf_k=round(hl["k"], 3), head_preload_N=round(hl["preload"], 2),
        head_contact_N=F_CONTACT, head_pre_defl=round(hl["pre_defl"], 1), let_off_overshoot_mm=round(hl["overshoot"], 1),
        head_strain_pct=round(100 * hl["strain_contact"], 2), hover=HOVER,
        arm_mass_g=round(1000 * m_arm, 1), head_share=round(share, 2), tap_energy_mJ=round(ke_h, 1),
        roof_energy_mJ=round(roof_e, 1), drop_energy_total_mJ=round(ke_tot, 1), gravity_mJ=round(w_grav, 1),
        arm_stop_energy_mJ=round(ke_tot - ke_h, 1),
        leaf_t=leaf_d["t"], leaf_b=LEAF_B, leaf_L=LEAF_L, leaf_rest_N=round(leaf_d["f_rest"], 2),
        leaf_park_N=round(leaf_d["f_park"], 2), leaf_strain_park_pct=round(100 * leaf_d["strain_park"], 2),
        seat_lean_deg=round(math.degrees(leaf.beta), 2), pin_travel_y=round(travel, 1), pin_rise=round(pin_rise, 1),
        rest_hold_Nmm=round(t_rest, 1), cam_force_N=round(p_cam, 1), cam_lift=round(h, 2),
        cam_rmax=round(CAM_R0 + h, 1), beta_park=round(bp, 1),
        cam_torque_kgcm=round(p_cam * h / ramp / 98.1, 2), park_deg=PARK_DEG,
        pawl_lean_rest=round(leans[0], 1), pawl_lean_park=round(leans[1], 1))
    state = dict(lay=lay, leaf=leaf, rest=rest, park=park, free=free, K=K, b0=CAM_BETA0, bp=bp, h=h)
    return num, state


# --- tower -------------------------------------------------------------------------------------
BOSS_IN = 3.0          # pivot bosses on the walls' inner faces (down-pointing teardrops: print upright)
HUB_PLAY = 0.2         # per side, hub between the spacers
BUSHING_OD = 6.0


def _teardrop_x(r, y, z, x0, x1):
    """Circle with a 45 deg point downward (-z), along x: no overhang when printed upright."""
    k = r / math.sqrt(2)
    pts = [(y - k, z - k), (y, z - r * math.sqrt(2)), (y + k, z - k)]
    return cyl_x(r, y, z, x0, x1).fuse(prism_yz(pts + [(y, z)], x0, x1))


def _wall(pts, side):
    x0, x1 = (WALL_X, WALL_X + WALL_T) if side > 0 else (-WALL_X - WALL_T, -WALL_X)
    return prism_yz(pts, x0, x1)


def tower(g, mg, st, plate, plate_cuts, plate_bosses):
    """Base plate on the upright, two side walls (the -X one carries the servo), the leaf seat,
    the arm's rest stops and the pivot bosses."""
    y_uo = g["upright_out_y"]
    y_f = y_uo - STAND_T
    lay, leaf, K = st["lay"], st["leaf"], st["K"]
    P = lay["P"]
    xo = WALL_X + WALL_T
    pb = plate.BoundBox
    y_back = pb.YMin - 1.0
    z_plate_top = pb.ZMax
    base = mg.box(-xo, xo, y_f, y_uo, BASE_BOTTOM, BASE_TOP)
    outline = [(y_f + 0.01, BASE_BOTTOM), (y_f + 0.01, P[1] - 24), (P[0] + 9, P[1] + 8), (P[0] - 10, P[1] + 8),
               (y_back, z_plate_top + 2), (y_back, BASE_BOTTOM)]
    walls = [_wall(outline, s) for s in (-1, 1)]
    floor = mg.box(-xo, xo, y_back, y_f + 0.01, BASE_BOTTOM, BASE_BOTTOM + 4)
    # leaf seat: leans with the leaf, attached to the base plate
    s0 = station(st["rest"], 0)
    t = leaf.t
    w_face = -t / 2                                   # the leaf's +Y face
    seat = _local_box(s0, -(s0[2] - BASE_BOTTOM) - 10, 0.0, w_face - 40, w_face, -12, 12)   # down to the floor
    seat = seat.common(mg.box(-13, 13, y_back, y_f + 0.02, BASE_BOTTOM, P[1]))
    # rest stops: wedges from each wall to over the tail, a 2 mm bumper lip on the contact face
    y1 = P[0] - TAIL_W / 2
    y0 = y1 - STOP_BAR
    zt = P[1] - 8
    stops, cuts = [], []
    for sx in (-1, 1):
        xw = sx * (WALL_X + 0.01)
        xs = sx * 3.0
        pts = [(xw, zt), (xs, zt), (xs, zt - 4), (xw, zt - 4 - (WALL_X - 3))]
        f = Part.Face(Part.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])]))
        stops.append(f.extrude(V(0, STOP_BAR, 0)))
        a, b = sorted((xs, sx * 12.0))
        cuts.append(mg.box(a - 1 if sx < 0 else a, b, y1 - BUMPER_T - 1.2, y1 - BUMPER_T, zt - 3.5, zt - 0.5))
    bosses = [_teardrop_x(6.0, P[0], P[1], WALL_X - BOSS_IN, WALL_X + 0.01),
              _teardrop_x(6.0, P[0], P[1], -WALL_X - 0.01, -WALL_X + BOSS_IN)]
    # bolt bosses round the 4 stand bolts on the back of the base plate (as before)
    hb = (mg.BOSS_FACTOR - 1) * STAND_T
    bolt_bosses = [Part.makeCylinder(STAND_BOLT_HOLE / 2 + mg.BOSS_WALL, hb + 0.01, V(x, y_f + 0.01, z), V(0, -1, 0))
                   for x, z in g["stand_bolts"]]
    shape = base.fuse(walls + [floor, seat, plate, plate_bosses] + stops + bosses + bolt_bosses)
    cuts += plate_cuts
    cuts.append(Part.makeCylinder(4.5, STAND_T + 2, V(0, y_uo + 1, 0), V(0, -1, 0)))     # tilt axle screw head
    for x, z in g["stand_bolts"]:
        cuts.append(Part.makeCylinder(STAND_BOLT_HOLE / 2, STAND_T + hb + 2, V(x, y_uo + 1, z), V(0, -1, 0)))
        cuts.append(Part.makeCylinder(3.6, 60, V(x, y_f - hb, z), V(0, -1, 0)))           # head access through the seat
    cuts.append(cyl_x(1.7, P[0], P[1], -xo - 1, xo + 1))                                  # pivot bolt
    hexa = [(P[0] + 3.35 * math.cos(math.radians(30 + 60 * i)), P[1] + 3.35 * math.sin(math.radians(30 + 60 * i)))
            for i in range(6)]
    cuts.append(prism_yz(hexa, xo - 2.6, xo + 1))                                         # M3 nut pocket, +X face
    for u in (-4.0, -LEAF_PAD + 4.0):                                                     # leaf pad screw pilots
        for x in (-LEAF_B / 4, LEAF_B / 4):
            cuts.append(_cyl_w(s0, u, x, mg.M3_PILOT / 2, w_face - 12, w_face + 0.5))
    # lightening windows: in front of the servo (both walls), under it (both), beside it (+X only)
    z_stop = zt - 4 - (WALL_X - 3)
    wins = [((pb.YMax + 4, y_f - 5, BASE_TOP + 4, z_stop - 4), (-1, 1)),
            ((y_back + 5, pb.YMax, BASE_BOTTOM + 8, pb.ZMin - 4), (-1, 1)),
            ((y_back + 5, pb.YMax, pb.ZMin, pb.ZMax - 2), (1,))]
    for (a, b, c, d), sides in wins:
        if b - a < 8 or d - c < 8:
            continue
        win = prism_yz(mg.window_pts(a, b, c, d), -xo - 1, xo + 1)
        for sx in sides:
            x0, x1 = sorted((sx * (WALL_X - 0.5), sx * (xo + 1)))
            cuts.append(win.common(mg.box(x0, x1, -500, 500, -500, 500)))
    fw = mg.window_pts(-WALL_X + 4, WALL_X - 4, y_back + 6, P[0] - 8)                  # stops short of the seat
    cuts.append(mg.prism(fw, V(0, 0, BASE_BOTTOM - 1), (1, 0, 0), (0, 1, 0), (0, 0, 1), 0, 6))
    return shape.cut(Part.makeCompound(cuts)).removeSplitter()


def bushing():
    """Pivot bushing: a tube between the two wall bosses (the M3 bolt clamps the walls onto it)."""
    L = 2 * (WALL_X - BOSS_IN)
    return Part.makeCylinder(BUSHING_OD / 2, L).cut(Part.makeCylinder(1.7, L + 2, V(0, 0, -1)))


def spacer():
    """Spacer on the bushing, either side of the arm hub."""
    L = (WALL_X - BOSS_IN) - ARM_W / 2 - HUB_PLAY
    return Part.makeCylinder(5.0, L).cut(Part.makeCylinder(BUSHING_OD / 2 + 0.2, L + 2, V(0, 0, -1)))


# --- assembly ------------------------------------------------------------------------------------
def _rot(shape, P, deg):
    s = shape.copy()
    s.rotate(V(0, P[0], P[1]), V(1, 0, 0), deg)
    return s


def build(g, mg):
    """g: upright_out_y, roof_z, stand_bolts (+ roof_raise, upright_half_x). mg: the make_gimbal module."""
    num, st = solve(g)
    lay, leaf, K = st["lay"], st["leaf"], st["K"]
    P = lay["P"]
    servo = mg.mg90s(mg.MG90S_TAB)
    spline_x = -CAM_T / 2 - CAM_HUB
    cam_horn = dict(len=14.0, width=6.0, depth=2.0, hub=CAM_HUB, centre=5.0)
    horn = mg.placed(mg.horn_pocket(cam_horn, through=CAM_T + 2), V(-CAM_T / 2, K[0], K[1]), (0, 1, 0), (-1, 0, 0))
    mg.HORNS["striker_cam"] = dict(horn=cam_horn, origin=V(-CAM_T / 2, K[0], K[1]), x=(0, 1, 0), z=(-1, 0, 0))
    cam = cam_solid(K, st["b0"], st["bp"], st["h"], horn)
    cam_spline = cam_spline_solid(K, st["b0"], st["bp"], st["h"], spline_x, servo, mg)
    # servo long axis up (spline end, and its wire notch, at the top)
    s_origin, s_x, s_z = V(spline_x, K[0], K[1]), (0, 0, 1), (1, 0, 0)
    servo_env = mg.placed(mg.servo_envelope(servo), s_origin, s_x, s_z)
    tab_half = servo["tab_len"] / 2 + 3
    plate, plate_cuts, bosses = mg.servo_plate(servo, (-servo["offset"] - tab_half, -servo["offset"] + tab_half),
                                               (-servo["W"] / 2 - PLATE_SIDE, servo["W"] / 2 + PLATE_SIDE))
    mg.PLATES["striker_stand"] = dict(servo=servo, t=mg.SERVO_PLATE_T, origin=s_origin, x=s_x, z=s_z)
    plate = mg.placed(plate, s_origin, s_x, s_z)
    bosses = mg.placed(bosses, s_origin, s_x, s_z)
    plate_cuts = [mg.placed(c, s_origin, s_x, s_z) for c in plate_cuts]
    stand = tower(g, mg, st, plate, plate_cuts, bosses)

    body, head = arm_parts(lay, installed=True)
    arm_rest = body.fuse(head).removeSplitter()
    fb, fh = arm_parts(lay, installed=False)
    arm_free = fb.fuse(fh).removeSplitter()
    arm_park = _rot(arm_rest, P, PARK_DEG)
    leaf_rest, leaf_park, leaf_free = (leaf_solid(leaf, st[k]) for k in ("rest", "park", "free"))
    pawl_rest, pawl_park = pawl_solid(leaf, st["rest"]), pawl_solid(leaf, st["park"])
    # the cam's swept disc must miss the leaf, lugs and stop in both states
    disc = cyl_x(CAM_R0 + st["h"] + 0.5, K[0], K[1], -CAM_T / 2 - 0.5, CAM_T / 2 + 0.5)
    num["cam_clear_leaf"] = round(min(disc.distToShape(s)[0] for s in (leaf_rest, leaf_park)), 2)
    # stack-side numbers: head face at the arm stop, and the free let-off travel
    num["head_gap_at_arm_stop"] = round(head.BoundBox.ZMin - lay["z_roof"], 2)
    num["pivot_yz"] = (round(P[0], 1), round(P[1], 1))
    num["cam_yz"] = (round(K[0], 1), round(K[1], 1))
    piv = []
    for shp, x0 in ((bushing(), -(WALL_X - BOSS_IN)), (spacer(), -(WALL_X - BOSS_IN)), (spacer(), ARM_W / 2 + HUB_PLAY)):
        c = shp.copy()
        c.rotate(V(0, 0, 0), V(0, 1, 0), 90)
        c.translate(V(x0, P[0], P[1]))
        piv.append(c)
    return dict(info=num, pivot_parts=piv, arm_rest=arm_rest, arm_park=arm_park, arm_free=arm_free,
                leaf_rest=leaf_rest, leaf_park=leaf_park, leaf_free=leaf_free,
                pawl_rest=pawl_rest, pawl_park=pawl_park, cam=cam, cam_spline=cam_spline,
                stand=stand, servo=servo_env, bushing=bushing(), spacer=spacer())
