"""Headless checks of the motion animation (docs/motion/animate_motion.FCMacro).

    freecadcmd -c "exec(open('check_motion.py').read(), {'__file__': 'check_motion.py', '__name__': '__main__'})"

Plays the macro's dry run over every frame (30 fps) and, at each frame:
- model cross-check: recomputes the arm angle from the cam angle with striker_motion (the
  cam profile and LOBES, leaf, pawl, pivot and lever, from make_striker) - independently of
  the stored timeline - and compares it with the arm angle actually shown; on the reverse
  stroke the arm may not rise and the pawl fold is recomputed; during a release the arm
  must equal the drop simulation and never sit inside the cam;
- collisions (every 10th frame while the gimbal moves, every 3rd while the cam turns, every 2nd
  of a release; results cached per relative pose): every pair of objects
  on different bodies that move relative to each other, by bounding box then distToShape
  and common().Volume. Touching or overlapping pairs are collisions unless they are
  designed contacts (DESIGNED: the joints' own interfaces, pawl on cam, arm on its rest
  stop, head on the roof at the tap);
- parked clearance: while the gimbal moves (hammer parked) the smallest distance between
  everything that tilts / rolls and the striker, as make_gimbal.striker_limits() measures it
  (the ring's -Y axle stub and its axle screw + washer, which only spin on the tilt axis
  inside the yoke's bearing, are reported separately, and so are the four tower bolts: their
  tips end flush with the yoke upright's inner face, so their gap to the ring is the yoke's).
Writes docs/motion/check-motion.json.
"""

import importlib.machinery
import importlib.util
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import FreeCAD as App  # noqa: E402
import Part  # noqa: E402

V = App.Vector
MACRO = os.path.join(HERE, "docs", "motion", "animate_motion.FCMacro")
COLLIDE_EVERY = int(os.environ.get("MOTION_COLLIDE_EVERY", "10"))   # gimbal frames; cam 3, release 2
VOL_TOL = 0.01            # mm^3: less than this common volume counts as touching, not overlapping
STRIKER_CLEAR = 5.0

# Designed contacts: (a, b) object names (prefix match on a trailing '*'), and why.
DESIGNED = [
    ("Horn_pan_yoke", "PanServo", "pan horn on its spline"), ("HornCentre_pan_yoke", "PanServo", "pan horn screw"),
    ("Horn_tilt_ring_mg90s", "TiltServo", "tilt horn on its spline"),
    ("HornCentre_tilt_ring_mg90s", "TiltServo", "tilt horn screw"),
    ("Horn_stack_middle_deck", "RollServo", "roll horn on its spline"),
    ("HornCentre_stack_middle_deck", "RollServo", "roll horn screw"),
    ("TiltRing", "BearingYoke", "tilt axle boss in the 623"), ("AxleScrewYoke", "BearingYoke", "tilt axle screw"),
    ("AxleWasherYoke", "BearingYoke", "tilt axle washer on the inner race"),
    ("StackMiddleDeck", "BearingRing", "roll axle boss in the 623"), ("AxleScrewRing", "BearingRing", "roll axle screw"),
    ("AxleWasherRing", "BearingRing", "roll axle washer on the inner race"),
    ("StrikerCam", "StrikerServo", "cam on its spline"), ("Horn_striker_cam", "StrikerServo", "cam horn on its spline"),
    ("HornCentre_striker_cam", "StrikerServo", "cam screw"),
    ("StrikerArm", "StrikerBushing", "arm hub on the bushing"), ("StrikerArm", "StrikerSpacer*", "arm hub between spacers"),
    ("DrivePin", "StrikerSpring", "drive pin in the leaf's slot"), ("StrikerArm", "StrikerSpring", "tail in the fork"),
    ("StrikerArm", "StrikerStand", "arm on its rest stop"),
    ("StrikerPawl", "PawlPin", "pawl on its pin"), ("StrikerPawl", "StrikerSpring", "pawl on its stop / lugs"),
    ("StrikerPawl", "StrikerCam", "pawl on the cam"), ("StrikerArm", "StackTopDeck", "head on the roof at the tap"),
    ("PadScrew*", "StrikerStand", "leaf pad screws in the seat"), ("StrikerSpring", "StrikerStand", "leaf pad on the seat"),
    ("PawlPin", "StrikerSpring", "pawl pin in the lugs"),
]


def _match(pat, name):
    return name.startswith(pat[:-1]) if pat.endswith("*") else name == pat


def designed(a, b):
    for x, y, why in DESIGNED:
        if (_match(x, a) and _match(y, b)) or (_match(x, b) and _match(y, a)):
            return why
    return None


def load_macro():
    loader = importlib.machinery.SourceFileLoader("animate_motion", MACRO)
    mod = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
    mod.__file__ = MACRO
    loader.exec_module(mod)
    return mod


class ModelCheck:
    """The arm angle (and pawl fold) recomputed from the cam angle at every frame."""

    def __init__(self, model, tl):
        self.m, self.tl = model, tl
        self.prev_phi = 0.0
        self.worst = dict(quasi=0.0, release=0.0, fold=0.0, inside_cam=0.0)
        self.rel_rows = {}

    def frame(self, s, shown_phi):
        g = self.tl.segments[s["seg"]]
        th = s["theta"]
        if g["kind"] == "release":
            pc = self.m.phi_cam(th)
            self.worst["inside_cam"] = max(self.worst["inside_cam"], pc - shown_phi)
            if s["seg"] not in self.rel_rows:            # the drop, simulated again from the model
                a, b = g["args"]
                self.rel_rows[s["seg"]] = self.m.release(a, b, self.prev_phi)[0]
            rows, tp = self.rel_rows[s["seg"]], max(0.0, s["phys_ms"]) / 1000.0
            j = max(0, min(len(rows) - 2, next((i for i, r in enumerate(rows) if r[0] > tp), len(rows)) - 1))
            r0, r1 = rows[j], rows[j + 1]
            f = 0.0 if r1[0] == r0[0] else min(1.0, max(0.0, (tp - r0[0]) / (r1[0] - r0[0])))
            self.worst["release"] = max(self.worst["release"], abs(shown_phi - (r0[2] + (r1[2] - r0[2]) * f)))
            self.prev_phi = shown_phi
            return
        elif g["kind"] == "cam" and g["args"][1] < g["args"][0]:
            want = min(self.prev_phi, self.m.phi_cam(th))
            self.worst["fold"] = max(self.worst["fold"], abs(self.m.fold(th, want) - s["alpha"]))
        else:
            want = self.m.phi_cam(th)
        self.worst["quasi"] = max(self.worst["quasi"], abs(shown_phi - want))
        self.prev_phi = want


def _grow(bb, m):
    b = App.BoundBox(bb)
    b.enlarge(m)
    return b


class Collider:
    """Pairwise interference between bodies in relative motion (and the flexing leaf / arm).

    Results are cached on each pair's relative placement and flex state, so a pair is only
    measured again when it has actually moved relative to the other (the striker while the
    gimbal pans, say, is measured once). Joint interfaces (ONCE: axisymmetric about the joint's
    own axis) are measured the first time they are met."""

    ONCE = ("spline", "screw", "623", "inner race", "arm hub", "pawl on its pin")

    def __init__(self, player):
        self.p = player
        self.objs = {b: [o for o in c.Group if hasattr(o, "Shape") and not o.Shape.isNull()]
                     for b, c in player.comps.items()}
        self.pairs = {}
        self.cache = {}
        self.checked = self.measured = 0

    def world(self, b):
        pl = self.p.comps[b].Placement
        out = []
        for o in self.objs[b]:
            s = o.Shape.copy()
            s.Placement = pl.multiply(o.Placement)
            out.append((o.Name, s, s.BoundBox, s.Placement))
        return out

    def _flex(self, name):
        return self.p.cur_shape["leaf"] if name == "StrikerSpring" else (
            self.p.cur_shape["arm"] if name == "StrikerArm" else "")

    def frame(self, k, section):
        bodies = list(self.objs)
        flex = {"Leaf", "Arm", "Pawl"}
        cache = {}
        for i, a in enumerate(bodies):
            for b in bodies[i + 1:]:
                rel = self.p.comps[a].Placement.inverse().multiply(self.p.comps[b].Placement)
                if rel.isSame(App.Placement(), 1e-9) and not ({a, b} & flex):
                    continue                                   # as assembled (assembly_checks covers it)
                for x in (a, b):
                    if x not in cache:
                        cache[x] = self.world(x)
                if not _grow(_bbox_all(cache[a]), 0.5).intersect(_bbox_all(cache[b])):
                    continue
                for na, sa, bba, pa in cache[a]:
                    gb = _grow(bba, 0.5)
                    for nb, sb, bbb, pb in cache[b]:
                        if not gb.intersect(bbb):
                            continue
                        self.checked += 1
                        key = tuple(sorted((na, nb)))
                        why = designed(na, nb)
                        if why and any(w in why for w in self.ONCE) and key in self.pairs:
                            continue
                        m = pa.inverse().multiply(pb).toMatrix()
                        ck = (na, nb, self._flex(na), self._flex(nb), tuple(round(v, 6) for v in m.A))
                        if ck not in self.cache:
                            self.measured += 1
                            d = _box_bound(sa, sb)
                            if d < 1.0:                   # close: measure it properly
                                d = sa.distToShape(sb)[0]
                            self.cache[ck] = (d, sa.common(sb).Volume if d < 1e-4 else 0.0)
                        d, vol = self.cache[ck]
                        r = self.pairs.setdefault(key, dict(min_d=1e9, max_vol=0.0, frame_vol=None, section=section,
                                                            designed=why))
                        if d < r["min_d"]:
                            r["min_d"], r["frame_d"] = d, k
                        if vol > r["max_vol"]:
                            r["max_vol"], r["frame_vol"], r["section"] = vol, k, section

    def report(self):
        coll = {f"{a} / {b}": r for (a, b), r in self.pairs.items() if not r["designed"] and r["max_vol"] > VOL_TOL}
        touch = {f"{a} / {b}": r for (a, b), r in self.pairs.items()
                 if not r["designed"] and r["max_vol"] <= VOL_TOL and r["min_d"] < 1e-4}
        des = {f"{a} / {b}": dict(r) for (a, b), r in self.pairs.items() if r["designed"]}
        free = [r["min_d"] for r in self.pairs.values() if not r["designed"] and r["min_d"] >= 1e-4]
        closest = min(((r["min_d"], f"{a} / {b}") for (a, b), r in self.pairs.items()
                       if not r["designed"] and r["min_d"] >= 1e-4), default=(None, None))
        near = sorted((r["min_d"], f"{a} / {b}") for (a, b), r in self.pairs.items() if not r["designed"])[:12]
        return dict(pairs_checked=self.checked, pairs_measured=self.measured, collisions=coll,
                    closest_12_non_designed=[dict(mm=round(d, 3), pair=k) for d, k in near], touching_not_designed=touch, designed=des,
                    closest_non_designed=dict(mm=closest[0], pair=closest[1]), n_free_pairs=len(free))


def _bbox_all(lst):
    b = App.BoundBox()
    for item in lst:
        b.add(item[2])
    return b


class ParkedClearance:
    """Smallest distance from everything that tilts / rolls to the striker while the gimbal moves."""

    SPIN_ONLY = ("AxleScrewYoke", "AxleWasherYoke")

    def __init__(self, player, ring_out_y):
        self.p = player
        self.stub = Part.makeCylinder(6.0, 30 - 0.15, V(0, -ring_out_y - 0.15, 0), V(0, -1, 0))
        self.min_d, self.at, self.spin_min = 1e9, None, 1e9
        self.bolt_min, self.bolt_at = 1e9, None
        self.n = 0
        self.base = {}

    def _solids(self, bodies, skip=(), cut_stub=False, only=None):
        out = []
        for b in bodies:
            pl = self.p.comps[b].Placement
            for o in self.p.comps[b].Group:
                if not hasattr(o, "Shape") or o.Name in skip or (only and o.Name not in only):
                    continue
                key = (o.Name, cut_stub)
                if key not in self.base:                    # rest-pose shape (the ring without its stub)
                    self.base[key] = o.Shape.cut(self.stub) if cut_stub and o.Name == "TiltRing" else o.Shape.copy()
                s = self.base[key].copy()
                s.Placement = pl.multiply(self.base[key].Placement)
                out += s.Solids if s.Solids else [s]
        return out

    def frame(self, k, s):
        if self.p.state is None:
            return
        moving = self._solids(("Ring", "Stack"), skip=self.SPIN_ONLY, cut_stub=True)
        bolts = [o.Name for o in self.p.comps["Tower"].Group if o.Name.startswith("StandBolt")]
        striker = self._solids(("Tower", "Arm", "Leaf", "Pawl", "Cam"), skip=bolts)
        d = _min_dist(moving, striker)
        db = _min_dist(moving, self._solids(("Tower",), only=bolts))
        self.n += 1
        at = dict(T=round(s["T"], 2), tilt=round(s["tilt"], 1), roll=round(s["roll"], 1))
        if d < self.min_d:
            self.min_d, self.at = d, at
        if db < self.bolt_min:
            self.bolt_min, self.bolt_at = db, at
        spin = self._solids(("Ring",), only=self.SPIN_ONLY)
        self.spin_min = min(self.spin_min, _min_dist(spin, striker))


def _min_dist(moving, fixed, cap=20.0):
    best = cap
    for m in moving:
        mb = m.BoundBox
        for f in fixed:
            fb = f.BoundBox
            gap = max(fb.XMin - mb.XMax, mb.XMin - fb.XMax, fb.YMin - mb.YMax, mb.YMin - fb.YMax,
                      fb.ZMin - mb.ZMax, mb.ZMin - fb.ZMax, 0.0)
            if gap < best:
                best = min(best, m.distToShape(f)[0])
    return best


def tap_instants(p):
    """At the exact instants of each release (not on the 30 fps grid): the arm landing on its
    rest stop and the head reaching the roof. Distance and overlap of the designed contacts."""
    out = []

    def world(name):
        o = p.doc.getObject(name)
        s = o.Shape.copy()
        s.Placement = p.comps[[b for b, c in p.comps.items() if o in c.Group][0]].Placement.multiply(o.Placement)
        return s

    for e in p.tl.events:
        g = p.tl.segments[e["segment"]]
        rows = [r for r in p.tl.rows if r[8] == e["segment"]]
        r = {}
        for key, ms, a, b in (("arm_on_stop", e["t_stop_ms"], "StrikerArm", "StrikerStand"),
                              ("head_on_roof", e["t_roof_ms"], "StrikerArm", "StackTopDeck")):
            row = min(rows, key=lambda q: abs(q[9] - ms))
            s = p.apply(row[0])
            sa, sb = world(a), world(b)
            r[key] = dict(T=round(row[0], 4), phys_ms=round(row[9], 3), arm_deg=round(s["phi"], 4), head_leaf_mm=round(s["x"], 3),
                          distance_mm=round(sa.distToShape(sb)[0], 4), overlap_mm3=round(sa.common(sb).Volume, 4))
        out.append(dict(segment=g["caption"], **r))
    p.restore()
    return out


def main():
    import striker_motion as sm
    mac = load_macro()
    t0 = time.time()
    doc, owned = mac.open_motion_doc(os.path.join(HERE, "docs", "motion", mac.DOC_FILE))
    data = json.loads(doc.getObject("MotionData").ModelJSON)
    sg = dict(data["striker_input"], stand_bolts=tuple(tuple(x) for x in data["striker_input"]["stand_bolts"]))
    model = sm.Striker(sg)
    taps = tap_instants(mac.Player(doc, owned))
    state = {}

    def hook(p, k):
        if "mc" not in state:
            state["mc"] = ModelCheck(model, p.tl)
            state["col"] = Collider(p)
            state["clr"] = ParkedClearance(p, data["ring_out_y"])
        s = p.state
        ang, _ = mac.arm_angle(p)
        state["mc"].frame(s, ang)
        g = p.tl.segments[s["seg"]]
        if g["kind"] == "gimbal" and g["args"][0] in ("tilt", "roll"):
            state["clr"].frame(k, s)
        every = {"release": 2, "cam": 3}.get(g["kind"], COLLIDE_EVERY)
        if k % every == 0:
            state["col"].frame(k, g["section"])
        if k % 150 == 0:
            print(f"  frame {k} T {s['T']:.1f} ({time.time() - t0:.0f} s)", flush=True)

    App.closeDocument(doc.Name)
    dry = mac.dry_run(os.path.join(HERE, "docs", "motion", mac.DOC_FILE), frame_hook=hook)
    clr = state["clr"]
    res = dict(dry_run=dry, model_check=state["mc"].worst, tap_instants=taps,
               parked_clearance=dict(min_mm=clr.min_d, at=clr.at, frames=clr.n, need=STRIKER_CLEAR,
                                     ok=clr.min_d >= STRIKER_CLEAR, spin_only_axle_screw_min_mm=clr.spin_min,
                                     tower_bolt_tips_min_mm=clr.bolt_min, tower_bolt_tips_at=clr.bolt_at),
               collisions=state["col"].report(), collide_every=COLLIDE_EVERY, seconds=round(time.time() - t0))
    out = os.path.join(HERE, "docs", "motion", "check-motion.json")
    with open(out, "w") as f:
        json.dump(res, f, indent=1, default=str)
    print(json.dumps({k: v for k, v in res.items() if k != "collisions"}, indent=1, default=str))
    c = res["collisions"]
    print("collisions:", len(c["collisions"]), "touching (not designed):", len(c["touching_not_designed"]),
          "closest non-designed:", c["closest_non_designed"], "pairs checked:", c["pairs_checked"])
    for k, r in c["collisions"].items():
        print("  COLLISION", k, r)
    for k, r in c["touching_not_designed"].items():
        print("  touching", k, r)
    for k, r in c["designed"].items():
        print(f"  designed {k}: {r['designed']}: max overlap {r['max_vol']:.3f} mm^3, min distance {r['min_d']:.3f} mm")
    return res



def _box_bound(sa, sb):
    """Lower bound on the distance between two shapes: the more complex one replaced by its
    bounding box (the faceted cam against the tower takes seconds otherwise)."""
    big, small = (sa, sb) if len(sa.Faces) >= len(sb.Faces) else (sb, sa)
    if len(big.Faces) < 200:
        return 0.0
    bb = big.BoundBox
    box = Part.makeBox(max(bb.XLength, 1e-3), max(bb.YLength, 1e-3), max(bb.ZLength, 1e-3), V(bb.XMin, bb.YMin, bb.ZMin))
    return box.distToShape(small)[0]


if __name__ == "__main__":
    main()
