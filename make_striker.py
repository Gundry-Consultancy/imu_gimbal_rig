"""Cam-driven tap striker: a PETG flat bar with a single hairpin curve at its clamped
base, lifted by a 270-degree servo cam and released to whack the stack's roof.

Imported by make_gimbal.py (the stand bolts to the pan yoke's idler upright).

How it works
- The bar is printed in its *free* shape, which already has the preload in it.
  Screw the base pad down and the hammer would press on the roof with F0 (2 N).
  Bend it up to fit the cam. The cam's base circle then holds the pawl so the
  hammer HOVERS just off the roof at rest, even with the servo unpowered.
- The servo turns the cam forward (+X rotation). The pawl hanging under the
  bar rides up a ramp, lifting the hammer by T, then drops off a cliff. The
  pawl lands on the base circle, the hammer's momentum flexes the long arm
  on through the hover gap into the roof (piano-style let-off), and it
  springs back to hover.
- The pawl leans PAWL_LEAN degrees into its stop. Under load at rest it is
  pushed further into the stop, not folded away.
- Two lobes fit within the 270-degree travel:
      park at ~120 deg -> sweep forward through 125 and 245: double tap
      park at ~240 deg -> step past 245: single tap
  To re-arm, turn the servo back to 0. The pawl is pinned (1.75 mm filament)
  and folds away from the cliffs on the reverse stroke, then gravity drops it
  back against its stop.
- Keep the hammer parked (lifted) whenever the gimbal moves.

Spring model: linear moment-curvature along the centreline, so large
rotations are handled. Printed PETG E is about 1.6-2.2 GPa. Tune the preload
with BAR_T (force goes as t^3) or a wedge shim under the pad.
"""

import math
import os
import sys

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

V = App.Vector

# --- design targets ----------------------------------------------------------
F0 = 2.0              # N, hammer on the roof after install
TRAVEL = 15.0         # mm, hammer lift when parked (clears the stack when it rolls)
E_PETG = 2000.0       # N/mm^2
BAR_B = 16.0          # bar width (x)
BAR_T = None          # bar thickness; None = solve for F_park = 1.5 * F0
R_CURVE = 12.0        # hairpin radius (centreline)
PAD_LEN = 12.0
PAD_EXTRA = 2.5       # pad thickening above the leaf
HAMMER_LEN = 12.0     # along the bar
LUG_DROP = 3.5        # arm underside to pawl pin
PAWL_LEN = 9.0        # pin to tip
PAWL_T = 4.4          # along x, between lugs
PAWL_W = 5.0          # along the bar
LUG_T = 3.0
PIN_D = 1.9           # 1.75 mm filament pin
HOVER = 0.5           # tip lift at rest (pawl on the cam base circle); the hammer face clears the roof by ~0.8 mm
PAWL_LEAN = 8.0       # deg, pawl tip leans toward its stop (-Y) so load at rest holds it there

CAM_R0 = 8.0
CAM_T = 6.0
CAM_HUB = 2.5         # horn hub: cam face to spline top
# servo angle (deg) breakpoints: (start_ramp, end_ramp, cliff)
LOBES = ((15, 115, 125), (135, 235, 245))

STAND_T = 5.0
RAIL_W = 7.0
RAIL_X = 18.0         # rail inner face |x|
RAIL_TOP = 12.0       # = idler upright top
PAD_SEAT_TOP = 17.0
STAND_BOTTOM = -20.0
STAND_BOLT_HOLE = 3.4  # M3 clearance through the front plate and its bosses
PLATE_SIDE = 4.0      # servo plate material beside the MG90S body (was 3)
WEB_FRAME = 3.5       # frame left round the web windows


# --- centreline --------------------------------------------------------------
class Bar:
    """Bar centreline in the YZ plane; the bar runs from the clamp (+Y end of pad) back round the hairpin and forward to the tip."""

    def __init__(self, y_c, y_tip, y_f, z_pad_top, t):
        self.t = t
        self.R = R_CURVE
        self.y_c, self.y_tip, self.y_f = y_c, y_tip, y_f
        self.z_p = z_pad_top + t / 2                      # pad centreline
        self.z_arm = self.z_p + 2 * self.R
        self.L = y_tip - y_c
        self.a = y_f - y_c
        self.EI = E_PETG * BAR_B * t ** 3 / 12
        self.ds = 0.1
        self.s_pad, self.s_curve = PAD_LEN, PAD_LEN + math.pi * self.R
        self.s_end = self.s_curve + self.L
        # installed geometry (straight arm, hammer on roof)
        self.inst = self._integrate(lambda s, y: 0.0, free=False)

    def kappa0(self, s):
        return -1.0 / self.R if self.s_pad <= s < self.s_curve else 0.0

    def _integrate(self, extra, free=True):
        """Integrate heading/position from the clamp. extra(s, y_inst) adds curvature."""
        y, z, h = self.y_c + PAD_LEN, self.z_p, math.pi
        pts = [(0.0, y, z, h)]
        n = int(round(self.s_end / self.ds))
        for i in range(n):
            s = i * self.ds
            y_inst = self.inst[i][1] if hasattr(self, "inst") else y
            k = self.kappa0(s + self.ds / 2) + (extra(s, y_inst) if free else 0.0)
            h_mid = h + k * self.ds / 2
            y += math.cos(h_mid) * self.ds
            z += math.sin(h_mid) * self.ds
            h += k * self.ds
            pts.append(((i + 1) * self.ds, y, z, h))
        return pts

    def free(self):
        return self._integrate(lambda s, y: -F0 * (self.y_tip - y) / self.EI if s >= self.s_pad else 0.0)

    def loaded(self, p_cam=0.0, f_tip=0.0):
        """Free shape + cam force at the follower + tip force (both upward)."""
        s_f = self.s_curve + self.a

        def extra(s, y):
            if s < self.s_pad:
                return 0.0
            m = -F0 * (self.y_tip - y) + f_tip * (self.y_tip - y)
            if s < s_f:
                m += p_cam * (self.y_f - y)
            return m / self.EI
        return self._integrate(extra)

    def parked_force(self):
        """Cam force that lifts the tip TRAVEL above the installed (roof) height."""
        return self.lift_force(TRAVEL)

    def lift_force(self, dz):
        """Cam force that lifts the tip dz above the installed (roof) height."""
        target = self.inst[-1][2] + dz
        # scan up from zero (tip height is not monotonic once the bar rolls over), then bisect
        lo, hi = 0.0, 0.5
        while self.loaded(p_cam=hi)[-1][2] < target:
            lo, hi = hi, hi + 0.5
            if hi > 200:
                raise ValueError("cam force runaway")
        for _ in range(30):
            mid = (lo + hi) / 2
            if self.loaded(p_cam=mid)[-1][2] < target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def station(self, pts, s):
        i = min(int(round(s / self.ds)), len(pts) - 1)
        return pts[i]


def solve_thickness(y_c, y_tip, y_f, z_pad_top):
    """Thickness giving F_park(tip) = 1.5 * F0, i.e. tip stiffness 0.5 * F0 / TRAVEL."""
    k_target = 0.5 * F0 / TRAVEL
    lo, hi = 0.8, 6.0
    for _ in range(40):
        t = (lo + hi) / 2
        bar = Bar(y_c, y_tip, y_f, z_pad_top, t)
        # tip stiffness: tip lift for a 1 N tip load from the installed state
        lift = bar.loaded(f_tip=F0 + 1.0)[-1][2] - bar.inst[-1][2]
        if 1.0 / lift < k_target:
            lo = t
        else:
            hi = t
    return round((lo + hi) / 2, 2)


# --- solids ------------------------------------------------------------------
def _profile_face(pts, t):
    left, right = [], []
    for _, y, z, h in pts:
        ny, nz = -math.sin(h), math.cos(h)
        left.append(V(0, y + ny * t / 2, z + nz * t / 2))
        right.append(V(0, y - ny * t / 2, z - nz * t / 2))
    poly = left + right[::-1] + [left[0]]
    # thin the point list for a lighter face
    poly = [p for i, p in enumerate(poly) if i % 4 == 0 or i == len(poly) - 1]
    return Part.Face(Part.makePolygon(poly))


def _local_box(st, u0, u1, w0, w1, x0, x1):
    """Box in the bar's local frame at a station (u along the bar, w normal, upwards for the arm)."""
    _, y, z, h = st
    du, dw = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))

    def P(u, w):
        return V(0, y + u * du[0] + w * dw[0], z + u * du[1] + w * dw[1])
    face = Part.Face(Part.makePolygon([P(u0, w0), P(u1, w0), P(u1, w1), P(u0, w1), P(u0, w0)]))
    f = face.extrude(V(x1 - x0, 0, 0))
    f.translate(V(x0, 0, 0))
    return f


def _local_cyl_x(st, u, w, r, x0, x1):
    _, y, z, h = st
    p = V(x0, y + u * math.cos(h) - w * math.sin(h), z + u * math.sin(h) + w * math.cos(h))
    return Part.makeCylinder(r, x1 - x0, p, V(1, 0, 0))


def _local_poly(st, uw, x0, x1):
    _, y, z, h = st
    pts = [V(0, y + u * math.cos(h) - w * math.sin(h), z + u * math.sin(h) + w * math.cos(h)) for u, w in uw]
    f = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(x1 - x0, 0, 0))
    f.translate(V(x0, 0, 0))
    return f


def _pawl_contact(st, t):
    """World (y, z) of the leaned pawl's lowest point at a bar station."""
    th = math.radians(PAWL_LEAN)
    r = PAWL_W / 2
    w_pin = -t / 2 - LUG_DROP
    cu = -(PAWL_LEN - r) * math.sin(th)
    cw = w_pin - (PAWL_LEN - r) * math.cos(th)
    _, y, z, h = st
    yc = y + cu * math.cos(h) - cw * math.sin(h)
    zc = z + cu * math.sin(h) + cw * math.cos(h) - r
    return yc, zc


def bar_solid(bar, pts, z_roof):
    t = bar.t
    body = _profile_face(pts, t).extrude(V(BAR_B, 0, 0))
    body.translate(V(-BAR_B / 2, 0, 0))
    parts = []
    # pad thickening (pad never moves)
    parts.append(Part.makeBox(BAR_B, PAD_LEN - 1.5, PAD_EXTRA,
                              V(-BAR_B / 2, bar.y_c + 1.5, bar.z_p + t / 2 - 0.01)))
    # hammer at the tip: down to the roof in the installed state, rounded bottom
    tip = bar.station(pts, bar.s_end)
    h_len = bar.inst[-1][2] - t / 2 - z_roof           # from the installed tip height
    parts.append(_local_box(tip, -HAMMER_LEN, 0, -t / 2 - h_len + HAMMER_LEN / 2, -t / 2 + 0.01, -BAR_B / 2, BAR_B / 2))
    parts.append(_local_cyl_x(tip, -HAMMER_LEN / 2, -t / 2 - h_len + HAMMER_LEN / 2, HAMMER_LEN / 2, -BAR_B / 2, BAR_B / 2))
    # pawl lugs, pin and stop under the arm at the follower station
    st = bar.station(pts, bar.s_curve + bar.a)
    w_pin = -t / 2 - LUG_DROP
    gap = PAWL_T / 2 + 0.4
    for x0, x1 in ((-gap - LUG_T, -gap), (gap, gap + LUG_T)):
        parts.append(_local_box(st, -3.5, 3.5, w_pin - 3.0, -t / 2 + 0.01, x0, x1))
    # stop: its face matches the pawl's -u side at PAWL_LEAN (0.3 mm clearance)
    th = math.radians(PAWL_LEAN)
    r = PAWL_W / 2

    def u_face(w):                     # w relative to the pin
        d = (r * math.sin(th) - w) / math.cos(th)
        return -r * math.cos(th) - d * math.sin(th) - 0.3
    w_top, w_bot = LUG_DROP + 0.01, -3.0
    u_back = -r - 4.5
    stop = [(u_face(w_top), w_pin + w_top), (u_face(w_bot), w_pin + w_bot), (u_back, w_pin + w_bot), (u_back, w_pin + w_top)]
    parts.append(_local_poly(st, stop, -gap - 0.01, gap + 0.01))
    shape = body.fuse(parts)
    pin = _local_cyl_x(st, 0, w_pin, PIN_D / 2, -gap - LUG_T - 1, gap + LUG_T + 1)
    # pad screw pilots (M3 self-tap from below)
    pilots = [Part.makeCylinder(1.3, 6, V(0, y, bar.z_p - t / 2 - 1))
              for y in (bar.y_c + 4.0, bar.y_c + PAD_LEN - 3.5)]
    return shape.cut(Part.makeCompound([pin] + pilots)).removeSplitter()


def pawl_solid(bar, pts):
    st = bar.station(pts, bar.s_curve + bar.a)
    w_pin = -bar.t / 2 - LUG_DROP
    x0, x1 = -PAWL_T / 2, PAWL_T / 2
    r = PAWL_W / 2
    body = _local_box(st, -r, r, w_pin - PAWL_LEN + r, w_pin, x0, x1)
    body = body.fuse([_local_cyl_x(st, 0, w_pin, r, x0, x1), _local_cyl_x(st, 0, w_pin - PAWL_LEN + r, r, x0, x1)])
    body = body.cut(_local_cyl_x(st, 0, w_pin, PIN_D / 2 + 0.05, x0 - 1, x1 + 1)).removeSplitter()
    _, y, z, h = st
    pin = V(0, y - w_pin * math.sin(h), z + w_pin * math.cos(h))
    body.rotate(pin, V(1, 0, 0), -PAWL_LEAN)   # tip toward -Y, into the stop
    return body


def cam_radius(s, h_tot):
    """Radius under the pawl at servo angle s (deg)."""
    s = s % 360
    for a0, a1, cliff in LOBES:
        if a0 <= s < a1:
            return CAM_R0 + h_tot * (s - a0) / (a1 - a0)
        if a1 <= s < cliff:
            return CAM_R0 + h_tot
    return CAM_R0


def cam_solid(y_cam, z_cam, h_tot, horn):
    """Cam in the YZ plane rotating about +X. Cam-frame angle psi = 90 - s."""
    pts = []
    cliffs = {c for _, _, c in LOBES}
    for psi10 in range(900, -2700, -5):
        psi = psi10 / 10
        s = 90 - psi
        if round(s, 1) in cliffs:
            pts.append((psi, cam_radius(s - 0.01, h_tot)))
        pts.append((psi, cam_radius(s, h_tot)))
    poly = [V(-CAM_T / 2, y_cam + r * math.cos(math.radians(p)), z_cam + r * math.sin(math.radians(p))) for p, r in pts]
    poly.append(poly[0])
    cam = Part.Face(Part.makePolygon(poly)).extrude(V(CAM_T, 0, 0))
    cuts = [Part.makeCylinder(2.5, CAM_T + 2, V(-CAM_T / 2 - 1, y_cam, z_cam), V(1, 0, 0))]
    cuts.append(horn)
    return cam.cut(Part.makeCompound(cuts)).removeSplitter()


def cam_spline_solid(y_cam, z_cam, h_tot, spline_x, servo, mg):
    """Cam with the MG90S spline moulded in (no horn): a hub reaches toward the servo,
    an M2 centre screw goes in from the +X face (counterbored)."""
    sp = servo["spline"]
    base = cam_solid(y_cam, z_cam, h_tot, Part.makeBox(0.1, 0.1, 0.1, V(1000, 0, 0)))
    base = base.fuse(Part.makeCylinder(2.6, CAM_T, V(-CAM_T / 2, y_cam, z_cam), V(1, 0, 0)))   # refill the horn hole
    hub_face = spline_x - sp["engage"]                    # stays clear of the servo case top
    hub = Part.makeCylinder(4.8, -CAM_T / 2 - hub_face + 0.01, V(hub_face, y_cam, z_cam), V(1, 0, 0))
    shape = base.fuse(hub)
    sock = mg.placed(mg.spline_socket(sp, sp["engage"] + 0.01), V(hub_face, y_cam, z_cam), (0, 1, 0), (1, 0, 0))
    screw = Part.makeCylinder(sp["screw"] / 2, CAM_T + 20, V(hub_face - 1, y_cam, z_cam), V(1, 0, 0))
    head = Part.makeCylinder(sp["head"] / 2, 1.8, V(CAM_T / 2 - 1.8, y_cam, z_cam), V(1, 0, 0))
    return shape.cut(Part.makeCompound([sock, screw, head])).removeSplitter()


# --- assembly ------------------------------------------------------------------
def build(g, mg):
    """g: dict with ring_out_y, upright_out_y, roof_z, upright_half_x. mg: the make_gimbal module."""
    y_uo = g["upright_out_y"]
    y_tip = 0.0
    servo = mg.mg90s(mg.MG90S_TAB)

    # cam behind the stand plate, pad behind the cam
    r_guess = CAM_R0 + 9.0
    y_cam = y_uo - STAND_T - 1.0 - r_guess
    y_pad_front = y_cam - r_guess - 2.0
    y_c = y_pad_front - PAD_LEN
    # the whole mechanism rides up with the roof: only the stand's pad seat (and servo-plate leg) grow
    pad_seat_top = PAD_SEAT_TOP + g.get("roof_raise", 0.0)
    t = BAR_T or solve_thickness(y_c, y_tip, y_cam, pad_seat_top)
    bar = Bar(y_c, y_tip, y_cam, pad_seat_top, t)

    free = bar.free()
    inst = bar.inst
    p_hover = bar.lift_force(HOVER)
    hover = bar.loaded(p_cam=p_hover)
    p_park = bar.parked_force()
    park = bar.loaded(p_cam=p_park)
    s_f = bar.s_curve + bar.a
    # cam centre straight under the leaned pawl's contact point at rest (hover)
    y_cc, z_cc = _pawl_contact(bar.station(hover, s_f), t)
    y_cam, z_cam = y_cc, z_cc - CAM_R0
    y_pk, z_pk = _pawl_contact(bar.station(park, s_f), t)
    h_tot = math.hypot(y_pk - y_cam, z_pk - z_cam) - CAM_R0
    lift_tip = park[-1][2] - inst[-1][2]
    max_strain = p_park * (bar.a + bar.R) * t / 2 / bar.EI
    preload_strain = F0 * (bar.L + bar.R) * t / 2 / bar.EI
    torque = p_park * h_tot / math.radians(LOBES[0][1] - LOBES[0][0])
    free_drop = inst[-1][2] - free[-1][2]
    # let-off: energy released by the drop vs. the stiffness of the arm beyond the follower
    k_tip = 0.5 * F0 / TRAVEL
    energy = (F0 + (F0 + k_tip * TRAVEL)) / 2 * (TRAVEL - HOVER)
    k_arm = 3 * bar.EI / (bar.L - bar.a) ** 3
    overshoot = math.sqrt(2 * energy / k_arm)
    info = dict(t=t, b=BAR_B, L=round(bar.L, 1), R=R_CURVE, a=round(bar.a, 1), free_tip_below_roof=round(free_drop, 1),
                hover=HOVER, rest_cam_force_N=round(p_hover, 1), park_cam_force_N=round(p_park, 1),
                tip_lift=round(lift_tip, 1), cam_lift=round(h_tot, 2), cam_rmax=round(CAM_R0 + h_tot, 1),
                strain_park_pct=round(100 * max_strain, 2), strain_preload_pct=round(100 * preload_strain, 2),
                cam_torque_kgcm=round(torque / 98.1, 2), drop_energy_mJ=round(energy, 1),
                let_off_overshoot_mm=round(overshoot, 1))

    spline_x = -CAM_T / 2 - CAM_HUB
    # the cam prints flat with this pocket facing up, so it needs no overhang roofs
    cam_horn = dict(len=14.0, width=6.0, depth=2.0, hub=CAM_HUB, centre=5.0)
    horn = mg.placed(mg.horn_pocket(cam_horn, through=CAM_T + 2), V(-CAM_T / 2, y_cam, z_cam), (0, 1, 0), (-1, 0, 0))
    mg.HORNS["striker_cam"] = dict(horn=cam_horn, origin=V(-CAM_T / 2, y_cam, z_cam), x=(0, 1, 0), z=(-1, 0, 0))
    cam = cam_solid(y_cam, z_cam, h_tot, horn)
    cam_spline = cam_spline_solid(y_cam, z_cam, h_tot, spline_x, servo, mg)

    s_origin, s_x, s_z = V(spline_x, y_cam, z_cam), (0, 1, 0), (1, 0, 0)
    servo_env = mg.placed(mg.servo_envelope(servo), s_origin, s_x, s_z)
    tab_half = servo["tab_len"] / 2 + 3
    plate, plate_cuts, bosses = mg.servo_plate(servo, (-servo["offset"] - tab_half, -servo["offset"] + tab_half),
                                               (-servo["W"] / 2 - PLATE_SIDE, servo["W"] / 2 + PLATE_SIDE))
    mg.PLATES["striker_stand"] = dict(servo=servo, t=mg.SERVO_PLATE_T, origin=s_origin, x=s_x, z=s_z)
    plate = mg.placed(plate, s_origin, s_x, s_z)
    bosses = mg.placed(bosses, s_origin, s_x, s_z)
    plate_cuts = [mg.placed(c, s_origin, s_x, s_z) for c in plate_cuts]

    # stand: front plate on the upright, two rails, pad seat, webs
    hx = g["upright_half_x"]
    y_back = y_c - 1.0
    front = mg.box(-RAIL_X - RAIL_W, RAIL_X + RAIL_W, y_uo - STAND_T, y_uo, STAND_BOTTOM, RAIL_TOP)
    rails = [mg.box(x0, x1, y_back, y_uo - STAND_T + 0.01, RAIL_TOP - 5, RAIL_TOP)
             for x0, x1 in ((-RAIL_X - RAIL_W, -RAIL_X), (RAIL_X, RAIL_X + RAIL_W))]
    seat = mg.box(-RAIL_X - RAIL_W, RAIL_X + RAIL_W, y_back, y_c + PAD_LEN, RAIL_TOP - 5, pad_seat_top)
    webs, cuts = [], []
    for x0 in (-RAIL_X - RAIL_W, RAIL_X):
        tri = [(y_uo - STAND_T + 0.01, STAND_BOTTOM), (y_uo - STAND_T + 0.01, RAIL_TOP - 4.99), (y_back + 4, RAIL_TOP - 4.99)]
        webs.append(mg.prism(tri, V(x0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), 0, RAIL_W))
        # lightening: triangular window (prints front-plate down; every edge is >= 45 deg or a floor)
        cuts.append(mg.prism(mg.triangle_window(*tri, inset=WEB_FRAME), V(x0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), -1, RAIL_W + 1))
    # servo plate down to the left rail
    pb = plate.BoundBox
    plate_leg = mg.box(pb.XMin, pb.XMax, pb.YMin, pb.YMax, RAIL_TOP - 5, pb.ZMin + 0.01)
    # bosses round the 4 stand bolts on the back of the front plate (bolt grip doubled)
    hb = (mg.BOSS_FACTOR - 1) * STAND_T
    bolt_bosses = [Part.makeCylinder(STAND_BOLT_HOLE / 2 + mg.BOSS_WALL, hb + 0.01, V(x, y_uo - STAND_T + 0.01, z), V(0, -1, 0))
                   for x, z in g["stand_bolts"]]
    stand = front.fuse(rails + [seat, plate, plate_leg, bosses] + webs + bolt_bosses)
    cuts += plate_cuts
    cuts.append(Part.makeCylinder(4.5, STAND_T + 2, V(0, y_uo + 1, 0), V(0, -1, 0)))   # axle screw head
    for x, z in g["stand_bolts"]:
        cuts.append(Part.makeCylinder(STAND_BOLT_HOLE / 2, STAND_T + hb + 2, V(x, y_uo + 1, z), V(0, -1, 0)))
    # keep the space above the seating face clear for the tabs (the far tab used to graze the pad seat)
    cx = -servo["offset"]
    zt = -servo["spline_above_tab"]
    cuts.append(mg.placed(mg.box(cx - servo["tab_len"] / 2 - 0.4, cx + servo["tab_len"] / 2 + 0.4, -servo["W"] / 2 - 0.4,
                                 servo["W"] / 2 + 0.4, zt - servo["tab_t"], zt + 0.5), s_origin, s_x, s_z))
    # lightening pockets through the pad seat, either side of the pad (clear of the rails)
    for sx in (-1, 1):
        u0, u1 = sorted((sx * 9.0, sx * (RAIL_X - 2.4)))
        cuts.append(mg.prism(mg.window_pts(u0, u1, -(y_c + PAD_LEN - 3.0), -(y_back + 3.0)), V(0, 0, RAIL_TOP - 6),
                             (1, 0, 0), (0, -1, 0), (0, 0, 1), 0, pad_seat_top - RAIL_TOP + 7))
    for y in (y_c + 4.0, y_c + PAD_LEN - 3.5):
        cuts.append(Part.makeCylinder(1.7, 20, V(0, y, RAIL_TOP - 6)))
    # keep the cam's swept disc clear of the stand
    cuts.append(Part.makeCylinder(CAM_R0 + h_tot + 1.5, CAM_T + 3, V(-CAM_T / 2 - 1.5, y_cam, z_cam), V(1, 0, 0)))
    stand = stand.cut(Part.makeCompound(cuts)).removeSplitter()

    z_roof = g["roof_z"]
    return dict(
        info=info,
        bar_free=bar_solid(bar, free, z_roof),
        bar_rest=bar_solid(bar, hover, z_roof),
        bar_park=bar_solid(bar, park, z_roof),
        pawl_rest=pawl_solid(bar, hover),
        pawl_park=pawl_solid(bar, park),
        cam=cam,
        cam_spline=cam_spline,
        stand=stand,
        servo=servo_env,
    )
