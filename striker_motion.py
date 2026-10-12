"""Striker kinematics and dynamics for the motion animation, computed from make_striker's model.

Nothing here is keyframed by eye. Every number comes from make_striker.solve() and the
functions it uses (the cam profile and LOBES, the drive-leaf cantilever, the pawl geometry,
the arm's pivot and lever, the masses and the head-leaf spring):

- cam angle -> arm angle (quasi-static): the smallest arm angle phi at which the pawl,
  carried by the drive leaf bent to match phi, clears the cam's profile. The pawl is a
  capsule (pin to tip, PAWL_W wide); its outline is tested against the cam radius at each
  outline point's angle, so ramps, dwell tops and cliffs are all exact.
- reverse stroke: a cam turning backwards cannot lift the leaf (a cliff meets the pawl's
  side), so phi never rises; the pawl folds about its pin just enough to clear the cam.
- cliff drop: the leaf's tip force k (d_rest + dy) and gravity drive the arm (inertia of the
  arm plus the lumped leaf tip) down onto its rest stop: RK4 on I phi'' = torque.
- let-off: the head carries its share of the energy (TAP_ENERGY, by the design) on through
  the hover gap against the preloaded head leaf (P + k x), meets the roof, rebounds with a
  coefficient of restitution E_ROOF (assumed) and comes back to rest on the stop screw.

Angles: phi = arm angle above rest (deg, + = about +X, hammer up); theta = cam servo angle
(deg from rest, forward = -X rotation = increasing pulse on the MG995, which sits on the +X wall
facing -X); alpha = pawl fold (deg, away from its stop).

The drive pin stays on its slot's centreline at every arm angle (make_striker.DriveLeaf.pose), and
the drop loses the slot friction (MU_PIN) on the way down.
"""

import bisect
import math

import make_striker as mk

E_ROOF = 0.4           # assumed coefficient of restitution, PETG head on the PETG roof
PHI_STEP = 0.25        # deg, leaf / pawl table
PHI_MAX = mk.PARK_DEG + 4.0
OUTLINE_N = 24         # points round each end of the pawl capsule


def striker_input(points):
    """make_striker.build()'s g dict, rebuilt from the generators' recorded fixing points."""
    import make_gimbal as mg
    import make_sensor_stack as ss
    p = [q for q in points if q["key"] == "stand_bolt0"][0]
    hb = (mg.BOSS_FACTOR - 1) * mk.STAND_T
    return dict(upright_out_y=p["pos"].y + hb + mk.STAND_T, roof_z=ss.roof_top() - ss.axis_z(),
                roof_raise=ss.roof_raise(), upright_half_x=mg.YOKE_HALF_X, stand_bolts=mg.STAND_BOLTS)


class Striker:
    def __init__(self, sg, state=None):
        """sg: make_striker's g dict. state: a (num, st) pair to use instead of mk.solve(sg)
        (striker_torque evaluates the edfadd7 MG90S parts this way)."""
        self.num, st = state or mk.solve(sg)
        self.st = st
        self.lay, self.leaf, self.K = st["lay"], st["leaf"], st["K"]
        self.b0, self.bp, self.h = st["b0"], st["bp"], st["h"]
        self.P, self.pin0 = self.lay["P"], self.lay["pin"]
        hl = mk.head_leaf_design(mk.TAP_ENERGY)
        self.hl = hl
        body, head = mk.arm_parts(self.lay)
        self.m_arm, self.cm, i_arm = mk._mass_props([body, head], self.P)
        _, _, self.i_head = mk._mass_props([head], self.P)
        self.m_head = sum(s.Volume for s in head.Solids) * mk.RHO_PETG
        m_leaf = mk.RHO_PETG * (mk.LEAF_B * 2.0 * mk.LEAF_L * 0.24 + mk.LEAF_B * mk.FORK_T * 19)
        self.i_tot = i_arm + m_leaf * mk.TAIL_LEVER ** 2
        self.k_leaf = mk.leaf_k(mk.LEAF_B, self.leaf.t, mk.LEAF_L)
        self.z_roof, self.z_face = self.lay["z_roof"], self.lay["z_face"]
        self.w_pin = self.leaf.t / 2 + mk.LUG_DROP
        self._table()

    # --- drive leaf and pawl as functions of the arm angle ------------------------------------
    def pin(self, phi):
        return mk.rot_yz(self.pin0, self.P, phi)

    def travel(self, phi):
        return self.pin(phi)[0] - self.pin0[0]

    def leaf_pts(self, phi):
        """Leaf stations with the arm at phi, the drive pin on its slot's centreline (as solve()):
        propped by the arm on its stop at 0, otherwise held by the cam load at the pawl."""
        if phi <= 1e-9:
            return self.leaf.pose(self.pin0)[0]
        return self.leaf.loaded_pose(self.lay, self.m_arm, self.cm, phi)[0]

    def _lerp(self, col, phi):
        phi = min(max(phi, 0.0), self.phis[-1])
        i = min(int(phi / PHI_STEP), len(self.phis) - 2)
        f = (phi - self.phis[i]) / PHI_STEP
        return col[i] + (col[i + 1] - col[i]) * f

    def leaf_force(self, phi):
        """The free leaf's push on the pin (N, across the slot) with the arm at phi, cam clear."""
        return self._lerp(self.f_free, phi)

    def cam_force(self, phi):
        """Cam load on the leaf at the pawl (N, across the leaf) holding the arm at phi while lifting
        (0 at rest: the stop holds it)."""
        if phi <= 1e-9:
            return 0.0
        return self._lerp(self.p_cam, phi)

    def _table(self):
        """phi -> pawl station (y, z, heading) at FOLLOW_A, the cam load, and the free leaf's pin load
        and slot heading, on a PHI_STEP grid."""
        self.phis, self.stations, self.p_cam, self.f_free, self.h_free, self.u_pin = [], [], [], [], [], []
        n = int(round(PHI_MAX / PHI_STEP))
        for i in range(n + 1):
            phi = i * PHI_STEP
            pin = self.pin(phi)
            free, _, f_t, _ = self.leaf.pose(pin)
            if i:
                pts, p, _, u = self.leaf.loaded_pose(self.lay, self.m_arm, self.cm, phi)
            else:
                pts, p, u = free, 0.0, self.leaf.pin_uw(free, pin)[0]
            self.phis.append(phi)
            self.stations.append(mk.station(pts, mk.FOLLOW_A))
            self.p_cam.append(p)
            self.f_free.append(f_t)
            self.h_free.append(free[-1][3])
            self.u_pin.append(u)
        self.station_rest = self.stations[0]

    def station_at(self, phi):
        phi = min(max(phi, 0.0), self.phis[-1])
        i = min(int(phi / PHI_STEP), len(self.phis) - 2)
        f = (phi - self.phis[i]) / PHI_STEP
        a, b = self.stations[i], self.stations[i + 1]
        return tuple(a[k] + (b[k] - a[k]) * f for k in range(4))

    # --- pawl against the cam ----------------------------------------------------------------
    def pawl_axis(self, phi, alpha=0.0):
        """World (y, z) of the pawl pin and tip-circle centre, with the leaf at phi and the pawl
        folded alpha (deg) about its pin, away from its stop (-alpha about +X)."""
        s = self.station_at(phi)
        pin = mk._local(s, 0.0, self.w_pin)
        du, dw = mk._pawl_dir()
        h = s[3] - math.radians(alpha)
        ln = mk.PAWL_LEN - mk.PAWL_W / 2
        d = (du * math.cos(h) - dw * math.sin(h), du * math.sin(h) + dw * math.cos(h))
        return pin, (pin[0] + ln * d[0], pin[1] + ln * d[1])

    def outline(self, phi, alpha=0.0):
        """Points on the pawl's capsule outline (the end round the tip, and its two sides)."""
        pin, tip = self.pawl_axis(phi, alpha)
        r = mk.PAWL_W / 2
        ang = math.atan2(tip[1] - pin[1], tip[0] - pin[0])
        pts = []
        for i in range(OUTLINE_N + 1):            # the half circle round the tip
            a = ang - math.pi / 2 + math.pi * i / OUTLINE_N
            pts.append((tip[0] + r * math.cos(a), tip[1] + r * math.sin(a)))
        for i in range(1, 10):                     # both sides, toward the pin
            f = i / 10
            c = (tip[0] + (pin[0] - tip[0]) * f, tip[1] + (pin[1] - tip[1]) * f)
            for sgn in (-1, 1):
                pts.append((c[0] + sgn * r * math.cos(ang + math.pi / 2), c[1] + sgn * r * math.sin(ang + math.pi / 2)))
        return pts

    def cam_r(self, psi):
        return mk.cam_radius(psi, self.b0, self.bp, self.h)

    def polar(self, q):
        dy, dz = q[0] - self.K[0], q[1] - self.K[1]
        return math.hypot(dy, dz), math.degrees(math.atan2(dz, dy))

    def clearance(self, theta, phi, alpha=0.0):
        """Smallest radial clearance (mm) of the pawl outline over the cam at servo angle theta;
        negative = the pawl would cut into the cam."""
        best = 1e9
        for q in self.outline(phi, alpha):
            r, a = self.polar(q)
            best = min(best, r - self.cam_r(a + theta))
        return best

    def contact_radius(self, theta, phi):
        """Cam radius under the pawl's tip centre (where it rides)."""
        _, tip = self.pawl_axis(phi)
        return self.cam_r(self.polar(tip)[1] + theta)

    def phi_cam(self, theta, tol=1e-4):
        """Cam-limited arm angle: the smallest phi whose (unfolded) pawl clears the cam."""
        if self.clearance(theta, 0.0) >= 0.0:
            return 0.0
        lo, hi = 0.0, PHI_MAX
        if self.clearance(theta, hi) < 0.0:
            raise ValueError(f"cam at {theta:.2f} deg lifts the arm past {PHI_MAX} deg")
        while hi - lo > tol:
            mid = (lo + hi) / 2
            if self.clearance(theta, mid) >= 0.0:
                hi = mid
            else:
                lo = mid
        return hi

    def fold(self, theta, phi, tol=1e-3, a_max=170.0):
        """Smallest pawl fold (deg) that clears the cam with the leaf held at phi."""
        if self.clearance(theta, phi, 0.0) >= 0.0:
            return 0.0
        a = 0.0
        while a < a_max and self.clearance(theta, phi, a) < 0.0:   # coarse: first clear angle
            a += 2.0
        if a >= a_max:
            raise ValueError(f"pawl cannot fold clear at cam {theta:.1f}, arm {phi:.1f}")
        lo, hi = a - 2.0, a
        while hi - lo > tol:
            mid = (lo + hi) / 2
            if self.clearance(theta, phi, mid) >= 0.0:
                hi = mid
            else:
                lo = mid
        return hi

    def head_height(self, phi, x=0.0):
        """Hammer face (its lowest point at rest) above the roof, with the arm at phi and the
        head leaf bent x further (let-off)."""
        y, z = mk.rot_yz((0.0, self.z_face), self.P, phi)
        return z - x - self.z_roof

    # --- dynamics ------------------------------------------------------------------------------
    def torque(self, phi, mu=None):
        """Torque about the pivot (N mm, + = hammer up) on the falling arm, cam clear: the leaf's
        push on the pin (with the slot friction against the fall) and gravity."""
        t_leaf = mk.pin_torque(self.P, self.pin(phi), self.leaf_force(phi), self._lerp(self.h_free, phi),
                               mk.slide_sense(phi, False), mu)
        cm1 = mk.rot_yz(self.cm, self.P, phi)
        return t_leaf - self.m_arm * mk.G * (cm1[0] - self.P[0])

    def drop(self, phi0, dt=2e-5):
        """Free fall of the arm from phi0 (deg) onto its rest stop (phi = 0), cam clear.
        Returns (times s, phis deg, omega at the stop rad/s, energy at the stop mJ)."""
        def acc(p):
            return 1000.0 * self.torque(math.degrees(p)) / self.i_tot      # rad/s^2 (N mm / kg mm^2 = 1000 /s^2)
        p, w, t = math.radians(phi0), 0.0, 0.0
        ts, ps = [0.0], [phi0]
        while p > 0.0:
            k1p, k1w = w, acc(p)
            k2p, k2w = w + dt / 2 * k1w, acc(p + dt / 2 * k1p)
            k3p, k3w = w + dt / 2 * k2w, acc(p + dt / 2 * k2p)
            k4p, k4w = w + dt * k3w, acc(p + dt * k3p)
            p_new = p + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
            w_new = w + dt / 6 * (k1w + 2 * k2w + 2 * k3w + k4w)
            if p_new <= 0.0:                          # land exactly on the stop
                f = p / (p - p_new)
                t += dt * f
                w = w + (w_new - w) * f
                p = 0.0
            else:
                p, w, t = p_new, w_new, t + dt
            ts.append(t)
            ps.append(math.degrees(p))
        return ts, ps, w, 0.5 * self.i_tot * w * w * 1e-3

    def letoff(self, ke=mk.TAP_ENERGY, dt=1e-6):
        """Head-leaf let-off after the arm stops: x = head travel below its stop screw (mm).
        Returns (times s, xs mm, events dict)."""
        m = self.m_head
        pre, k, gap = self.hl["preload"], self.hl["k"], mk.HOVER
        v = math.sqrt(2.0 * ke * 1e-3 / m) * 1000.0                     # mm/s, downward
        x, t = 0.0, 0.0
        ts, xs = [0.0], [0.0]
        ev = dict(v0=v / 1000.0, m_head_g=m * 1000.0)
        bounced = False
        while True:
            a = (m * mk.G - (pre + k * x)) / m * 1000.0                 # mm/s^2, + = down
            v += a * dt
            x += v * dt
            t += dt
            if x >= gap and v > 0 and not bounced:                      # roof contact
                x = gap
                ev.update(t_roof=t, v_roof=v / 1000.0, e_roof_mJ=0.5 * m * (v / 1000.0) ** 2 * 1000.0)
                v = -E_ROOF * v
                bounced = True
                ts.append(t)
                xs.append(x)
                continue
            if bounced and x <= 0.0:                                     # back on the stop screw
                x = 0.0
                ev.update(t_back=t)
                ts.append(t)
                xs.append(x)
                break
            if len(ts) == 0 or t - ts[-1] >= 2e-5:
                ts.append(t)
                xs.append(x)
        return ts, xs, ev

    def release(self, theta0, theta1, phi0, servo_dps=None, dt=5e-5, settle=0.006):
        """The cam turns on from theta0 to theta1 at servo speed (deg/s) with the arm at phi0 on
        the lobe: the pawl rolls off the cliff's corner (the arm follows the cam while the cam
        holds it up), then falls free onto the rest stop; then the let-off.
        Returns rows (t s, theta, phi, x) and the event numbers."""
        servo_dps = SERVO_DPS if servo_dps is None else servo_dps

        def acc(p):
            return 1000.0 * self.torque(math.degrees(p)) / self.i_tot
        t, p, w = 0.0, math.radians(phi0), 0.0
        rows, contact_until = [], 0.0
        while True:
            if t > 0.5:
                raise ValueError(f"release {theta0}->{theta1}: the arm never reaches its stop (cam still holds it)")
            th = min(theta1, theta0 + servo_dps * t)
            rows.append((t, th, math.degrees(p), 0.0))
            if p <= 0.0:
                break
            k1p, k1w = w, acc(p)
            k2p, k2w = w + dt / 2 * k1w, acc(p + dt / 2 * k1p)
            k3p, k3w = w + dt / 2 * k2w, acc(p + dt / 2 * k2p)
            k4p, k4w = w + dt * k3w, acc(p + dt * k3p)
            pn = p + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
            wn = w + dt / 6 * (k1w + 2 * k2w + 2 * k3w + k4w)
            t += dt
            pc = math.radians(self.phi_cam(min(theta1, theta0 + servo_dps * t)))
            if pc > 0.0 and pn < pc:                        # the cam still holds the pawl up
                wn = (pc - p) / dt
                pn = pc
                contact_until = t
            elif pn <= 0.0:                                 # lands on the rest stop
                f = p / (p - pn) if p != pn else 1.0
                t = t - dt + dt * f
                w = w + (wn - w) * f
                p = 0.0
            else:
                p, w = pn, wn
        t_hit, w_hit = t, w
        ke = 0.5 * self.i_tot * w_hit ** 2 * 1e-3                    # mJ in the arm + leaf tip
        ke_head = ke * self.i_head / self.i_tot
        lt, lx, ev = self.letoff(ke_head)
        for tt, xx in zip(lt[1:], lx[1:]):
            rows.append((t_hit + tt, min(theta1, theta0 + servo_dps * (t_hit + tt)), 0.0, xx))
        t_end = rows[-1][0] + settle
        rows.append((t_end, min(theta1, theta0 + servo_dps * t_end), 0.0, 0.0))
        ev.update(t_contact_ms=contact_until * 1e3, t_stop_ms=t_hit * 1e3, omega_stop=-w_hit, ke_stop_mJ=ke,
                  ke_head_mJ=ke_head, t_roof_ms=(t_hit + ev["t_roof"]) * 1e3, t_back_ms=(t_hit + ev["t_back"]) * 1e3,
                  t_end_ms=t_end * 1e3, theta0=theta0, theta1=theta1, phi0=phi0)
        return rows, ev


def smooth(f):
    f = min(1.0, max(0.0, f))
    return f * f * (3 - 2 * f)


# Cam angles (deg from rest, forward), from LOBES: lobe 1 parks the hammer for the gimbal and a single
# tap; a double tap sweeps on over lobe 2. Releases are commanded past the modelled drop with margin.
CLIFF_A, CLIFF_B = mk.LOBES[0][2], mk.LOBES[1][2]
PARK_A = (mk.LOBES[0][1] + mk.LOBES[0][2]) / 2
RELEASE_A = 87.0       # single tap: the pawl is off cliff A's corner by ~75.2; lobe 2's ramp starts at 97
RELEASE_B = 165.0      # double tap end: off cliff B by ~153.2; 15 deg short of the MG995's 180
SERVO_DPS = 375.0      # MG995 no-load speed, 0.16 s / 60 deg at 6 V (0.2 s at 4.8 V = 300 deg/s)

# The motion programme: (section, caption, kind, duration s, args). Kinds:
#   hold                       nothing moves
#   gimbal  (joint, [angles])  pan / tilt / roll through the waypoints (eased, time by angle)
#   cam     (theta0, theta1)   the cam servo turns at an animation pace; the arm follows the cam
#                              (quasi-static), the pawl folds on the reverse stroke
#   release (theta0, theta1)   the servo steps past a cliff at full speed: physical time, shown
#                              in slow motion
PROGRAMME = [
    ("Start", "At rest: hammer 5 mm over the roof (cam 0 = 500 us)", "hold", 0.6, ()),
    ("Start", f"Park the hammer before the gimbal moves (cam 0 -> {PARK_A:g})", "cam", 2.0, (0.0, PARK_A)),
    ("1 Pan", "Pan sweep, the DS3240's full +/-135 (270 deg servo)", "gimbal", 6.0, ("pan", [0.0, 135.0, -135.0, 0.0])),
    ("2 Tilt", "Tilt a full 360 with the hammer parked", "gimbal", 6.0, ("tilt", [0.0, 360.0])),
    ("3 Roll", "Roll +/-90 (MG90S)", "gimbal", 4.0, ("roll", [0.0, 90.0, -90.0, 0.0])),
    ("4 Striker", "Re-arm: cam back to 0, the arm rides the ramp down", "cam", 1.5, (PARK_A, 0.0)),
    ("4 Striker", f"Park: cam forward to {PARK_A:g} (top of lobe 1)", "cam", 1.5, (0.0, PARK_A)),
    ("4 Striker", f"Single tap: step toward the {CLIFF_A:g} cliff", "cam", 0.3, (PARK_A, CLIFF_A - 1)),
    ("4 Striker", "Single tap: the leaf throws the arm onto its stop, the head lets off to the roof", "release", 2.4,
     (CLIFF_A - 1, RELEASE_A)),
    ("4 Striker", "Re-arm: cam back to 0, the cliff folds the pawl", "cam", 2.6, (RELEASE_A, 0.0)),
    ("4 Striker", f"Park at {PARK_A:g}", "cam", 1.2, (0.0, PARK_A)),
    ("4 Striker", f"Double tap: toward the {CLIFF_A:g} cliff", "cam", 0.3, (PARK_A, CLIFF_A - 1)),
    ("4 Striker", "Double tap, first tap", "release", 2.0, (CLIFF_A - 1, RELEASE_A)),
    ("4 Striker", "Double tap: lobe 2 lifts the arm again (~0.25 s after tap 1 at full speed)", "cam", 2.0,
     (RELEASE_A, CLIFF_B - 1)),
    ("4 Striker", f"Double tap, second tap (past the {CLIFF_B:g} cliff)", "release", 2.0, (CLIFF_B - 1, RELEASE_B)),
    ("4 Striker", "Return to rest: cam back to 0, both cliffs fold the pawl", "cam", 3.0, (RELEASE_B, 0.0)),
    ("End", "At rest", "hold", 0.8, ()),
]
COLUMNS = ("T", "pan", "tilt", "roll", "theta", "phi", "alpha", "x", "seg", "phys_ms", "slow")


def _legs(angles, dur):
    """Split dur over the legs of a waypoint list in proportion to the angle each covers."""
    d = [abs(b - a) for a, b in zip(angles, angles[1:])]
    tot = sum(d) or 1.0
    return [dur * x / tot for x in d]


def build_timeline(model, rate=60, release_rate=150):
    """Sample the whole programme. Returns dict(columns, rows, segments, events). Each row is
    COLUMNS; phys_ms is the physical time into a release (else -1), slow its slow-motion factor."""
    state = dict(pan=0.0, tilt=0.0, roll=0.0, theta=0.0, phi=0.0, alpha=0.0, x=0.0)
    rows, segs, events = [], [], []
    T = 0.0

    def emit(t, seg, phys=-1.0, slow=0.0):
        s = state
        rows.append([round(t, 5), s["pan"], s["tilt"], s["roll"], s["theta"], s["phi"], s["alpha"], s["x"], seg,
                     round(phys, 4), slow])

    for i, (sec, cap, kind, dur, args) in enumerate(PROGRAMME):
        segs.append(dict(i=i, section=sec, caption=cap, kind=kind, t0=T, t1=T + dur, args=list(args)))
        if kind == "hold":
            n = max(1, int(dur * rate))
            for k in range(n + 1):
                emit(T + dur * k / n, i)
        elif kind == "gimbal":
            joint, angles = args
            t = T
            for (a, b), d in zip(zip(angles, angles[1:]), _legs(angles, dur)):
                n = max(2, int(d * rate))
                for k in range(n + 1):
                    state[joint] = a + (b - a) * smooth(k / n)
                    emit(t + d * k / n, i)
                t += d
            state[joint] = angles[-1] % 360.0 if joint == "tilt" else angles[-1]
        elif kind == "cam":
            a, b = args
            n = max(2, int(dur * rate))
            for k in range(n + 1):
                th = a + (b - a) * smooth(k / n)
                pc = model.phi_cam(th)
                if b >= a:
                    if pc < state["phi"] - 0.5 and k:
                        raise ValueError(f"segment {i}: forward cam passes a cliff at {th:.1f}; use a release")
                    phi, alpha = pc, 0.0
                else:
                    phi = min(state["phi"], pc)
                    alpha = model.fold(th, phi)
                state.update(theta=th, phi=phi, alpha=alpha, x=0.0)
                emit(T + dur * k / n, i)
        elif kind == "release":
            a, b = args
            rel, ev = model.release(a, b, state["phi"])
            ev["segment"] = i
            events.append(ev)
            t_end = rel[-1][0]
            slow = dur / t_end
            n = max(2, int(dur * release_rate))
            tps = sorted(set([t_end * k / n for k in range(n + 1)] + [ev["t_stop_ms"] / 1e3, ev["t_roof_ms"] / 1e3]))
            j = 0
            for tp in tps:
                while j < len(rel) - 2 and rel[j + 1][0] < tp:
                    j += 1
                r0, r1 = rel[j], rel[j + 1]
                f = 0.0 if r1[0] == r0[0] else min(1.0, max(0.0, (tp - r0[0]) / (r1[0] - r0[0])))
                state.update(theta=r0[1] + (r1[1] - r0[1]) * f, phi=r0[2] + (r1[2] - r0[2]) * f,
                             x=r0[3] + (r1[3] - r0[3]) * f, alpha=0.0)
                emit(T + tp * slow, i, tp * 1000.0, round(slow, 2))
            ev["rows"] = rel
        T += dur
    return dict(columns=list(COLUMNS), rows=rows, segments=segs, events=events, duration=T)
