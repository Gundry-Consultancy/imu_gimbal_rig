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
- An MG995 (~180 deg, on the tower's +X wall, gear face toward -X) turns a two-lobe cam.
  Forward = -X rotation = increasing pulse (the 2026-10-11 bench saw increasing pulse turn the
  cam anticlockwise seen from the gear face). A pawl pinned to the drive leaf rides up a ramp,
  bending the leaf +Y. Through the pin and slot this swings the arm PARK_DEG up, and the hammer
  parks far outside the tilt sweep. At the cliff the pawl drops off, and the leaf throws the arm
  down onto its rest stop.
- The tail leans back TAIL_TILT at rest, so its pin swings either side of straight down and only
  dips ~2 mm along the leaf's slot (it rode 7 mm up it before): the slot friction then costs ~6 mJ
  of the drop, not ~23. The leaf is sized to deliver TAP_ENERGY after that loss. The leaf's shape
  is solved with the pin on the slot's centreline at every angle (DriveLeaf.pose).
- Let-off: the head sits on its own thin PETG leaf. A nylon M3 screw in the overarm presses
  the leaf down, preloading it (HEAD_PRELOAD) against the screw. When the arm hits its
  stop, the head's momentum bends the leaf on through the 5 mm gap into the roof, and the
  head springs back onto the screw, where the preload stops it bouncing back into the roof.
- The pawl hangs off the leaf pointing -Y and leans PAWL_LEAN down onto its stop. The cam's
  drag and gravity both hold it there. On the reverse stroke the cliffs fold it up, and
  gravity drops it back. Two lobes fit the MG995's travel with >= 10 deg spare at the top
  (LOBES, angles from rest at 500 us; see docs/servo_control.md):
      park at ~69 deg -> step on to 87: single tap (the pawl leaves cliff 1 at ~75)
      park at ~69 deg -> sweep on to 165: double tap, ~0.25 s apart (cliffs at 73 and 151)
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
MU_PIN = 0.4           # friction, steel pin in the PETG slot (taken as PETG on PETG, conservative)
MU_CAM = 0.4           # friction, PETG pawl on the PETG cam

# --- arm (2.5D profile, 12 mm thick, prints on its side) -------------------------
ARM_W = 12.0           # x thickness of the whole arm
PIVOT_BACK = 28.0      # pivot behind the upright's outer face
PIVOT_UP = 66.0        # pivot above the roof top
HUB_R = 8.0
PIVOT_HOLE = 6.3       # hub bore: rides on the printed bushing tube (6.0) round the M3 bolt
TAIL_LEVER = 18.0      # pivot to the drive pin
TAIL_TILT = 28.0       # deg the tail leans back (-Y) at rest: the pin swings -TAIL_TILT .. PARK_DEG - TAIL_TILT
                       # about straight down, so it slides less along the leaf's slot (less friction)
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
FOLLOW_A = 48.0       # pawl lugs above the root (was 40: 48 halves the cam load for the same energy)

# --- pawl and cam (as the old striker; pin is 1.75 mm filament) -----------------
LUG_DROP = 3.5
PAWL_LEN = 17.0       # pin to tip (was 14: keeps the bigger cam 2 mm off the leaf)
PAWL_T = 4.4
PAWL_W = 5.0
LUG_T = 3.0
PIN_D = 1.9
PAWL_LEAN = 16.0        # deg the pawl leans past the cam's force line, onto its stop (gravity and drag agree)
CAM_R0 = 12.0         # base circle (was 8): brings the two-lobe pressure angle down to <= 38 deg
CAM_T = 6.0
CAM_HUB = 3.5          # cam face to spline top: the MG995 horn's hub (as the DS3240's; measure)
CAM_HORN_LEN = 18.0    # stock horn trimmed to this (its 8.5 mm arm stays 2 mm inside the base circle)
CAM_GAP = 0.4          # pawl clear of the base circle at rest
CAM_BETA0 = 45.0       # deg, contact on the cam above its +Y side (keeps the drop clear of the lobe)
LOBES = ((15, 65, 73), (97, 147, 151))

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

    def loads_for_tip(self, d_tip, f_tip=0.0, a=None):
        """Cam load at a that puts the tip d_tip (from free) with f_tip pushing the tip back (-Y)."""
        a = FOLLOW_A if a is None else a
        return (d_tip + f_tip * self.g(self.L, self.L)) / self.g(self.L, a)

    def shape(self, p_cam=0.0, f_tip=0.0, a=None, tip_prop=None, n=120, m_tip=0.0):
        """Leaf stations. tip_prop: tip held at this deflection from free by an unknown tip force (rest).
        m_tip: a moment at the tip (+ bends it +Y), from a load applied beyond it on the rigid tip block."""
        a = FOLLOW_A if a is None else a
        if tip_prop is not None:                                   # rest: only the tip reaction
            f = -tip_prop / self.g(self.L, self.L)
            loads = [(p_cam, a), (-f, self.L)] if p_cam else [(-f, self.L)]
        else:
            loads = [(p_cam, a), (-f_tip, self.L)]
        pts, z, ds = [], self.z0, self.L / n
        for i in range(n + 1):
            s = i * ds
            v = sum(P * self.g(s, aa) for P, aa in loads) + m_tip * s * s / (2 * self.EI)
            vp = sum(P * self.gp(s, aa) for P, aa in loads) + m_tip * s / self.EI
            dy = -self.beta + vp
            y = self.y0 - self.beta * s + v
            if i:
                z += ds * (1 - dy * dy / 2)          # small-slope shortening
            pts.append((s, y, z, math.atan2(1.0, dy)))
        return pts

    def tip_force(self, d_tip):
        return d_tip / self.g(self.L, self.L)

    def loaded_pose(self, lay, m_arm, cm, phi, lifting=True):
        """Leaf held by the cam with the arm at phi (deg), lifting (or lowering): the slot push holds
        the arm against gravity and slot friction. Returns (pts, p_cam, f_n, u)."""
        sense = slide_sense(phi, lifting)
        P = lay["P"]
        pin = rot_yz(lay["pin"], P, phi)
        cm1 = rot_yz(cm, P, phi)
        h = math.pi / 2
        for _ in range(3):
            f_n = arm_hold(P, pin, cm1, m_arm, h, sense)
            pts, p_cam, _, u = self.pose(pin, f_n)
            h = pts[-1][3]
        return pts, p_cam, f_n, u

    def pin_uw(self, pts, pin):
        """The pin's (u, w) in the tip station's frame: u along the leaf, w toward its -Y face
        (the slot's centreline is w = pin_w())."""
        _, y, z, h = pts[-1]
        dy, dz = pin[0] - y, pin[1] - z
        return dy * math.cos(h) + dz * math.sin(h), -dy * math.sin(h) + dz * math.cos(h)

    def pin_w(self):
        return FORK_T / 2 - self.t / 2

    def pose(self, pin, f_n=None, a=None):
        """Leaf shape with the drive pin ON its slot's centreline (2026-10-12 fix: before, the tip was
        put at the pin's height, and the parked slot ran up to 1.8 mm off the pin).
        f_n: the slot's push on the pin, normal to the slot (+ = toward +Y, holding the arm up); the
        cam load at a is solved. f_n=None: no cam load (rest on the stop, or falling); the pin's load
        is solved. The pin sits u along the rigid tip block, so its load also bends the tip by f u.
        Returns (pts, p_cam, f_t, u): f_t = the pin's transverse load on the leaf (+Y)."""
        wc = self.pin_w()
        u, pts, x = 0.0, None, 0.0
        for _ in range(4):
            def err(x):
                p, f = (0.0, x) if f_n is None else (x, -f_n)
                pp = self.shape(p_cam=p, f_tip=-f, a=a, m_tip=f * u)
                return self.pin_uw(pp, pin)[1] - wc, pp
            x0, x1 = x, x + 1.0
            e0, e1 = err(x0)[0], err(x1)[0]
            for _ in range(30):                          # secant: w is all but linear in the load
                if abs(e1) < 1e-6 or e1 == e0:
                    break
                x0, x1, e0 = x1, x1 - e1 * (x1 - x0) / (e1 - e0), e1
                e1 = err(x1)[0]
            x = x1
            e, pts = err(x)
            u = self.pin_uw(pts, pin)[0]
        p, f = (0.0, x) if f_n is None else (x, -f_n)
        return pts, p, f, u


def slide_sense(phi, lifting=True):
    """Which way the pin slides along the slot (+1 = +u, up the leaf) as the arm turns at phi."""
    s = 0.0 if abs(phi - TAIL_TILT) < 1e-9 else math.copysign(1.0, phi - TAIL_TILT)
    return s if lifting else -s


def arm_hold(P, pin, cm, m_arm, h, sense=1.0, mu=None):
    """Slot force on the pin (N, normal to the slot) that holds the arm against gravity, while the pin
    slides along the slot: sense +1 lifting (pin moves +u in the slot), -1 falling. h = slot heading."""
    mu = MU_PIN if mu is None else mu
    en, eu = (math.sin(h), -math.cos(h)), (math.cos(h), math.sin(h))
    d = (pin[0] - P[0], pin[1] - P[1])
    fy, fz = en[0] - sense * mu * eu[0], en[1] - sense * mu * eu[1]
    lever = d[0] * fz - d[1] * fy                       # torque about +X of a unit slot push
    return m_arm * G * (cm[0] - P[0]) / lever


def pin_torque(P, pin, f_t, h, sense=-1.0, mu=None):
    """Torque on the arm about +X (N mm) from the leaf's pin load f_t (+Y on the leaf, so the leaf
    pushes the pin with -f_t across the slot) with slot friction opposing the pin's slide (sense)."""
    mu = MU_PIN if mu is None else mu
    en, eu = (math.sin(h), -math.cos(h)), (math.cos(h), math.sin(h))
    d = (pin[0] - P[0], pin[1] - P[1])
    fy = -f_t * en[0] - sense * mu * abs(f_t) * eu[0]
    fz = -f_t * en[1] - sense * mu * abs(f_t) * eu[1]
    return d[0] * fz - d[1] * fy


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
    return dict(P=P, pin=rot_yz((P[0], P[1] - TAIL_LEVER), P, -TAIL_TILT), z_roof=z_roof, z_face=z_face, z_lc=z_lc, z_oa=z_oa,
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
    tail = bar_yz(P, rot_yz((P[0], P[1] - TAIL_LEN), P, -TAIL_TILT), TAIL_W, x0, x1)
    body = hub.fuse([tail, beam, root, overarm, boss, leaf.cut(Part.makeBox(ARM_W + 2, HEAD_L, 100, V(x0 - 1, -HEAD_L / 2, lay["z_face"])))])
    cuts = [cyl_x(PIVOT_HOLE / 2, P[0], P[1], x0 - 1, x1 + 1),
            cyl_x(TAIL_PILOT / 2, lay["pin"][0], lay["pin"][1], x0 - 1, x1 + 1),
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
PIN_U0, PIN_U1 = -3.0, 1.5   # slot in the leaf's tip block, end centres along the leaf from the tip (was 0..9):
                             # with the tail leaning back the pin dips ~2 mm and comes back, no longer rising 7


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
             _local_box(st, PIN_U0 - 4.5, PIN_U1 + 3.0, -t / 2, FORK_T - t / 2, x0, x1)]    # tip block
    # pawl lugs (on the -Y face, w > 0) and the stop under the pawl
    w_pin = t / 2 + LUG_DROP
    gap = PAWL_T / 2 + 0.4
    for a, c in ((-gap - LUG_T, -gap), (gap, gap + LUG_T)):
        parts.append(_local_box(sa, -3.5, 3.5, t / 2 - 0.01, w_pin + 3.0, a, c))
    parts.append(_local_poly(sa, _stop_uw(t), -gap - 0.01, gap + 0.01))
    body = body.fuse(parts)
    cuts = [_local_box(st, PIN_U0 - 5.0, PIN_U1 + 4.0, -FORK_T, FORK_T, -FORK_SLOT, FORK_SLOT),   # tail runs here
            slot_yz(_local(st, PIN_U0, wc), _local(st, PIN_U1, wc), PIN_HOLE, x0 - 1, x1 + 1),
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
    """Cam with the servo spline moulded in (no horn), hub toward -X (build() mirrors it to +X);
    centre screw from the other face, counterbored."""
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


def cam_profile(K, b0, bp, h, theta=0.0, step=1.0):
    """The cam's outline in (y, z) at servo angle theta (forward = -X rotation), with the cliffs."""
    pts = []
    cliffs = [bp + c for _, _, c in LOBES]
    n = int(round(360 / step))
    for i in range(n):
        psi = i * step
        for pc in cliffs:
            if psi - step < pc <= psi:
                pts.append((pc - 1e-3, cam_radius(pc - 1e-3, b0, bp, h)))
                pts.append((pc, CAM_R0))
        pts.append((psi, cam_radius(psi, b0, bp, h)))
    return [(K[0] + r * math.cos(math.radians(p - theta)), K[1] + r * math.sin(math.radians(p - theta))) for p, r in pts]


PIN_D_REAL = 3.0       # the M3 drive pin's thread crest diameter, for the slot check


def pin_slot_check(st, n=12):
    """The drive pin (an M3, PIN_D_REAL) against the leaf's slot at n + 1 arm angles from rest to
    parked, with the cam holding the leaf (lifting) and with it falling free: the worst overlap
    volume (mm^3), the worst offset from the slot centreline (mm) and the pin's run along the slot."""
    lay, leaf = st["lay"], st["leaf"]
    P = lay["P"]
    body, head = arm_parts(lay)
    m_arm, cm, _ = _mass_props([body, head], P)
    worst_v, worst_w, us = 0.0, 0.0, []
    for i in range(n + 1):
        phi = PARK_DEG * i / n
        pin = rot_yz(lay["pin"], P, phi)
        poses = [leaf.pose(pin)[0]] + ([leaf.loaded_pose(lay, m_arm, cm, phi)[0]] if i else [])
        for pts in poses:
            u, w = leaf.pin_uw(pts, pin)
            us.append(u)
            worst_w = max(worst_w, abs(w - leaf.pin_w()))
            cyl = cyl_x(PIN_D_REAL / 2, pin[0], pin[1], -LEAF_B / 2 - 1, LEAF_B / 2 + 1)
            worst_v = max(worst_v, cyl.common(leaf_solid(leaf, pts)).Volume)
    return dict(pin_slot_overlap_mm3=round(worst_v, 4), pin_slot_offset_mm=round(worst_w, 4),
                pin_slot_clear_mm=round((PIN_HOLE - PIN_D_REAL) / 2 - worst_w, 3),
                pin_u_range=(round(min(us), 2), round(max(us), 2)), slot_u=(PIN_U0, PIN_U1))


def _seg_dist(p, a, b):
    dy, dz = b[0] - a[0], b[1] - a[1]
    L2 = dy * dy + dz * dz
    f = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dy + (p[1] - a[1]) * dz) / L2))
    return math.hypot(p[0] - a[0] - f * dy, p[1] - a[1] - f * dz)


def cam_sweep_clear(st, shapes, th1=170.0, step=5.0):
    """Smallest (y, z) distance from the cam's outline, swept forward 0..th1, to the sections of
    `shapes` through the cam's thickness (x = 0 and x = +/-(CAM_T / 2 - 0.1)): (mm, theta, (y, z))."""
    K = st["K"]
    pts = []
    for shp in shapes:
        for x in (0.0, CAM_T / 2 - 0.1, -CAM_T / 2 + 0.1):
            for w in shp.slice(V(1, 0, 0), x):
                for e in w.Edges:
                    pts += [(q.y, q.z) for q in e.discretize(Distance=0.4)]
    rmax = CAM_R0 + st["h"]
    pts = [q for q in pts if math.hypot(q[0] - K[0], q[1] - K[1]) < rmax + 6]
    best = (99.0, None, None)
    th = 0.0
    while th <= th1 + 1e-9:
        prof = cam_profile(K, st["b0"], st["bp"], st["h"], th)
        segs = list(zip(prof, prof[1:] + prof[:1]))
        for q in pts:
            d = min(_seg_dist(q, a, b) for a, b in segs)
            a = math.degrees(math.atan2(q[1] - K[1], q[0] - K[0]))
            if math.hypot(q[0] - K[0], q[1] - K[1]) < cam_radius(a + th, st["b0"], st["bp"], st["h"]):
                d = -d                                   # inside the cam
            if d < best[0]:
                best = (d, th, q)
        th += step
    return best


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


def drop_energy(leaf, lay, n=48):
    """The leaf's work on the arm (mJ) as it falls free from PARK_DEG onto its stop, and the slot
    friction's share of it lost on the way: (w_leaf, w_friction). Pin on its slot at every angle."""
    P = lay["P"]
    w_leaf = w_fric = 0.0
    prev = None
    for i in range(n + 1):
        phi = PARK_DEG * i / n
        pin = rot_yz(lay["pin"], P, phi)
        pts, _, f_t, _ = leaf.pose(pin)
        h = pts[-1][3]
        t0 = pin_torque(P, pin, f_t, h, -1.0, mu=0.0)          # + = hammer up
        tf = pin_torque(P, pin, f_t, h, slide_sense(phi, False)) - t0   # friction resists the fall: + while falling
        if prev:
            dphi = math.radians(PARK_DEG / n)
            w_leaf += -(t0 + prev[0]) / 2 * dphi
            w_fric += (tf + prev[1]) / 2 * dphi
        prev = (t0, tf)
    return w_leaf, w_fric


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
    # size the leaf so the arm still reaches its stop with ke_tot after the slot friction of the drop
    target = ke_tot - w_grav
    for _ in range(8):
        leaf_d = leaf_design(target, travel)
        # the pin rides FORK_T / 2 off the leaf's +Y face, so the leaf stands that much +Y of it
        leaf = DriveLeaf(pin0[0] + FORK_T / 2 - leaf_d["t"] / 2, pin0[1] - LEAF_L, leaf_d["t"], leaf_d["d_rest"])
        w_leaf, w_fric = drop_energy(leaf, lay)
        short = ke_tot - (w_leaf - w_fric + w_grav)
        if abs(short) < 0.02:
            break
        target += short
    # rest: the pin props the leaf (arm on its stop); park: the cam holds the leaf, the pin on its slot
    rest, _, f_rest, u_rest = leaf.pose(pin0)
    park, p_cam, f_n_park, u_park = leaf.loaded_pose(lay, m_arm, cm, PARK_DEG)
    f_drop = leaf.pose(pin1)[2]                                  # the leaf's push on the pin at the cliff
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
    t_rest = -pin_torque(P, pin0, f_rest, rest[-1][3], mu=0.0) + m_arm * G * (cm[0] - P[0])
    pin_rise = u_park - u_rest
    us = [leaf.loaded_pose(lay, m_arm, cm, PARK_DEG * i / 12)[3] for i in range(1, 12)] + [u_rest, u_park]
    u_min, u_max = min(us), max(us)
    assert PIN_U0 + 0.3 < u_min and u_max < PIN_U1 - 0.3, f"pin runs {u_min:.2f}..{u_max:.2f} mm in the leaf's slot"
    ke_h = TAP_ENERGY
    roof_e = ke_h - hl["preload"] * HOVER - hl["k"] * HOVER ** 2 / 2
    num = dict(
        head_leaf_t=hl["t"], head_leaf_k=round(hl["k"], 3), head_preload_N=round(hl["preload"], 2),
        head_contact_N=F_CONTACT, head_pre_defl=round(hl["pre_defl"], 1), let_off_overshoot_mm=round(hl["overshoot"], 1),
        head_strain_pct=round(100 * hl["strain_contact"], 2), hover=HOVER,
        arm_mass_g=round(1000 * m_arm, 1), head_share=round(share, 2), tap_energy_mJ=round(ke_h, 1),
        roof_energy_mJ=round(roof_e, 1), drop_energy_total_mJ=round(ke_tot, 1), gravity_mJ=round(w_grav, 1),
        arm_stop_energy_mJ=round(ke_tot - ke_h, 1),
        leaf_t=leaf_d["t"], leaf_b=LEAF_B, leaf_L=LEAF_L, leaf_rest_N=round(f_rest, 2),
        leaf_park_N=round(f_drop, 2), leaf_strain_park_pct=round(100 * leaf_d["strain_park"], 2),
        drop_friction_mJ=round(w_fric, 1), mu_pin=MU_PIN, mu_cam=MU_CAM,
        seat_lean_deg=round(math.degrees(leaf.beta), 2), pin_travel_y=round(travel, 1), pin_rise=round(pin_rise, 1),
        tail_tilt_deg=TAIL_TILT,
        pin_u_rest=round(u_rest, 2), pin_u_park=round(u_park, 2), pin_u_min=round(u_min, 2), pin_u_max=round(u_max, 2), park_hold_N=round(f_n_park, 2),
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


def _teardrop_hole_x(r, y, z, x0, x1):
    """Hole along x with a 45 deg point upward (+z): prints without support in an upright wall."""
    k = r / math.sqrt(2)
    pts = [(y - k, z + k), (y, z + r * math.sqrt(2)), (y + k, z + k)]
    return cyl_x(r, y, z, x0, x1).fuse(prism_yz(pts + [(y, z)], x0, x1))


DRIVER_HOLE = 7.2      # driver access through the -X wall to the MG995's tab screws


def _wall(pts, side):
    x0, x1 = (WALL_X, WALL_X + WALL_T) if side > 0 else (-WALL_X - WALL_T, -WALL_X)
    return prism_yz(pts, x0, x1)


def tower(g, mg, st, plate, plate_cuts, plate_bosses):
    """Base plate on the upright, two side walls (the +X one carries the servo), the leaf seat,
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
        stops.append(_rot(f.extrude(V(0, STOP_BAR, 0)), P, -TAIL_TILT))       # square to the leaning tail
        a, b = sorted((xs, sx * 12.0))
        cuts.append(_rot(mg.box(a - 1 if sx < 0 else a, b, y1 - BUMPER_T - 1.2, y1 - BUMPER_T, zt - 3.5, zt - 0.5),
                         P, -TAIL_TILT))
    bosses = [_teardrop_x(6.0, P[0], P[1], WALL_X - BOSS_IN, WALL_X + 0.01),
              _teardrop_x(6.0, P[0], P[1], -WALL_X - 0.01, -WALL_X + BOSS_IN)]
    # bolt bosses round the 4 stand bolts on the back of the base plate (as before)
    hb = (mg.BOSS_FACTOR - 1) * STAND_T
    bolt_bosses = [Part.makeCylinder(STAND_BOLT_HOLE / 2 + mg.BOSS_WALL, hb + 0.01, V(x, y_f + 0.01, z), V(0, -1, 0))
                   for x, z in g["stand_bolts"]]
    shape = base.fuse(walls + [floor, seat, plate, plate_bosses] + stops + bosses + bolt_bosses)
    cuts += plate_cuts
    # 1 mm round the leaf's tip block at rest: the stops lean back with the tail, over the fork
    tip = station(st["rest"], leaf.L)
    cuts.append(_local_box(tip, PIN_U0 - 5.5, PIN_U1 + 4.0, -t / 2 - 1.0, FORK_T - t / 2 + 1.0, -LEAF_B / 2 - 1, LEAF_B / 2 + 1))
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
    # lightening windows: in front of the servo (both walls), under it (both), beside it (-X only:
    # the MG995 is fitted from inside the tower, and its spline end swings through this one)
    z_stop = zt - 4 - (WALL_X - 3)
    wins = [((pb.YMax + 4, y_f - 5, BASE_TOP + 4, z_stop - 4), (-1, 1)),
            ((y_back + 5, pb.YMax, BASE_BOTTOM + 8, pb.ZMin - 4), (-1, 1)),
            ((y_back + 5, pb.YMax, pb.ZMin, pb.ZMax - 2), (-1,))]
    for (a, b, c, d), sides in wins:
        if b - a < 8 or d - c < 8:
            continue
        win = prism_yz(mg.window_pts(a, b, c, d), -xo - 1, xo + 1)
        for sx in sides:
            x0, x1 = sorted((sx * (WALL_X - 0.5), sx * (xo + 1)))
            cuts.append(win.common(mg.box(x0, x1, -500, 500, -500, 500)))
    # driver access to the MG995's tab screws, straight through the -X wall
    pl = mg.PLATES["striker_stand"]
    for xa, xb, yy, _ in mg.servo_pilots(pl["servo"]):
        q = mg._to_world(pl, ((xa + xb) / 2, yy, 0.0))
        cuts.append(_teardrop_hole_x(DRIVER_HOLE / 2, q.y, q.z, -xo - 1, -WALL_X + 1))
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


def fixing_points(g, st, mg):
    """Record the striker's fixing points in mg.POINTS (see make_gimbal.point), from the same
    layout the tower, arm and leaf are cut from."""
    lay, leaf = st["lay"], st["leaf"]
    P, t = lay["P"], leaf.t
    y_uo = g["upright_out_y"]
    y_f = y_uo - STAND_T
    hb = (mg.BOSS_FACTOR - 1) * STAND_T
    xo = WALL_X + WALL_T
    for i, (x, z) in enumerate(g["stand_bolts"]):          # head on the boss, access through the seat
        mg.point(f"stand_bolt{i}", "striker_stand", "through", V(x, y_f - hb, z), (0, -1, 0), STAND_BOLT_HOLE,
                 STAND_T + hb, access_d=7.2, mate=f"yoke_stand{i}")
    mg.point("pivot", "striker_stand", "through", V(-xo, P[0], P[1]), (-1, 0, 0), 3.4, 2 * xo,
             nut=V(xo - 2.6, P[0], P[1]), nut_axis=V(1, 0, 0))
    s0 = station(st["rest"], 0)
    _, _, _, h = s0
    w_out = V(0, -math.sin(h), math.cos(h))                  # the leaf's -Y face normal (pad side)
    i = 0
    for u in (-4.0, -LEAF_PAD + 4.0):
        for x in (-LEAF_B / 4, LEAF_B / 4):
            y, z = _local(s0, u, t / 2 + LEAF_PAD_T)
            mg.point(f"leaf_pad{i}", "striker_spring", "through", V(x, y, z), w_out, 3.4, t + LEAF_PAD_T,
                     pilot_d=mg.M3_PILOT, pilot_depth=12.0)
            i += 1
    pin = lay["pin"]                                         # drive pin: head 0.3 mm off the +X fork face
    mg.point("drive_pin", "striker_arm", "pilot", V(LEAF_B / 2 + 0.3, pin[0], pin[1]), (1, 0, 0), TAIL_PILOT, ARM_W,
             slot_d=PIN_HOLE, head_gap=0.3)
    z_tip = lay["z_lc"] - lay["pre"] + lay["th"] / 2         # head-leaf top under the screw (installed)
    zo = lay["z_oa"]
    mg.point("stop_screw", "striker_arm", "through", V(0, 0, zo + OVERARM_T + 4.0), (0, 0, 1), SCREW_PILOT,
             OVERARM_T + 4.0, nut_low=V(0, 0, zo), tip_z=z_tip)
    sa = station(st["rest"], FOLLOW_A)
    y, z = _local(sa, 0, t / 2 + LUG_DROP)
    mg.point("pawl_pin", "striker_spring", "pin", V(0, y, z), (1, 0, 0), PIN_D, 2 * (PAWL_T / 2 + 0.4 + LUG_T))


# --- assembly ------------------------------------------------------------------------------------
def servo_clearance(s, mg, c=0.4):
    """Servo-local cutters: the body (where it passes the tower's inside, e.g. the leaf seat) and a
    pocket for the tabs where they sit inside the wall's inner face (the MG995's tab underside is
    1.4 mm outboard of it)."""
    cx, G = -s["offset"], s["spline_above_tab"]
    body = mg.box(cx - s["L"] / 2 - c, cx + s["L"] / 2 + c, -s["W"] / 2 - c, s["W"] / 2 + c, -s["H"] - 1, -s["case_top"] + c)
    tabs = mg.box(cx - s["tab_len"] / 2 - c, cx + s["tab_len"] / 2 + c, -s["W"] / 2 - c, s["W"] / 2 + c,
                  -G - s["tab_t"] + 0.01, -G + 3.0)
    return [body, tabs]


def _rot(shape, P, deg):
    s = shape.copy()
    s.rotate(V(0, P[0], P[1]), V(1, 0, 0), deg)
    return s


def build(g, mg):
    """g: upright_out_y, roof_z, stand_bolts (+ roof_raise, upright_half_x). mg: the make_gimbal module."""
    num, st = solve(g)
    lay, leaf, K = st["lay"], st["leaf"], st["K"]
    P = lay["P"]
    # MG995 on the +X wall, its gear face toward -X: the bench saw increasing pulse turn the cam
    # anticlockwise seen from the gear face, which from this side is -X rotation, the model's forward
    servo = dict(mg.MG995)
    spline_x = CAM_T / 2 + CAM_HUB
    cam_horn = dict(len=CAM_HORN_LEN, width=servo["horn"]["width"], depth=servo["horn"]["depth"], hub=CAM_HUB,
                    centre=servo["horn"]["centre"])
    horn = mg.placed(mg.horn_pocket(cam_horn, through=CAM_T + 2), V(CAM_T / 2, K[0], K[1]), (0, 1, 0), (1, 0, 0))
    mg.HORNS["striker_cam"] = dict(horn=cam_horn, origin=V(CAM_T / 2, K[0], K[1]), x=(0, 1, 0), z=(1, 0, 0))
    cam = cam_solid(K, st["b0"], st["bp"], st["h"], horn)
    # the spline cam is drawn with its hub on -X, then mirrored (the profile is symmetric in x)
    cam_spline = cam_spline_solid(K, st["b0"], st["bp"], st["h"], -spline_x, servo, mg).mirror(V(0, 0, 0), V(1, 0, 0))
    # servo long axis up (spline end, and its wire notch, at the top); shaft toward -X
    s_origin, s_x, s_z = V(spline_x, K[0], K[1]), (0, 0, 1), (-1, 0, 0)
    servo_env = mg.placed(mg.servo_envelope(servo), s_origin, s_x, s_z)
    tab_half = servo["tab_len"] / 2 + 3
    plate, plate_cuts, bosses = mg.servo_plate(servo, (-servo["offset"] - tab_half, -servo["offset"] + tab_half),
                                               (-servo["W"] / 2 - PLATE_SIDE, servo["W"] / 2 + PLATE_SIDE))
    mg.PLATES["striker_stand"] = dict(servo=servo, t=mg.SERVO_PLATE_T, origin=s_origin, x=s_x, z=s_z)
    plate = mg.placed(plate, s_origin, s_x, s_z)
    bosses = mg.placed(bosses, s_origin, s_x, s_z)
    plate_cuts = [mg.placed(c, s_origin, s_x, s_z) for c in plate_cuts + servo_clearance(servo, mg)]
    stand = tower(g, mg, st, plate, plate_cuts, bosses)
    mg.plate_points("striker_stand")
    mg.horn_points("striker_cam")
    fixing_points(g, st, mg)

    body, head = arm_parts(lay, installed=True)
    arm_rest = body.fuse(head).removeSplitter()
    fb, fh = arm_parts(lay, installed=False)
    arm_free = fb.fuse(fh).removeSplitter()
    arm_park = _rot(arm_rest, P, PARK_DEG)
    leaf_rest, leaf_park, leaf_free = (leaf_solid(leaf, st[k]) for k in ("rest", "park", "free"))
    pawl_rest, pawl_park = pawl_solid(leaf, st["rest"]), pawl_solid(leaf, st["park"])
    # the cam, swept through its travel, must miss the leaf, lugs and stop (rest and parked)
    d, th, _ = cam_sweep_clear(st, [leaf_rest, leaf_park])
    num["cam_clear_leaf"], num["cam_clear_leaf_at"] = round(d, 2), th
    num.update(pin_slot_check(st))
    import striker_motion as sm                               # (they import this module)
    import striker_torque as tq
    model = sm.Striker(g, state=(dict(num), st))
    tsum = tq.summary(tq.curve(model))
    num.update({f"cam_{k}": v for k, v in tsum.items()})
    num["cam_drop_deg"] = [tq.release_angles(model, i)[1] for i in range(len(LOBES))]
    num["cam_lobes"] = [list(lb) for lb in LOBES]
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
