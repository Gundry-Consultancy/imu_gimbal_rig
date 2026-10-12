"""Checks on the fixing plan (assembly_plan.build_plan):

  (a) counts by spec against the README fixings table / striker section
  (b) every screw coaxial with its hole: against the recorded fixing point, and against the
      actual cylindrical hole face found in the part (slots: distance to the slot centreline)
  (c) every head / washer / nut / spacer seats on its face (gap < 0.5 mm, not sunk into it)
  (d) shanks clear of printed parts outside their own hole cylinders (< 1 mm^3)
  (e) access: each fixing's insertion sweep, and a straight driver / key behind its head,
      against the printed parts already in place at that moment of the animation
"""

import math
import re

import FreeCAD as App
import Part

V = App.Vector
PRINTED = ("stack", "spine", "ring", "yoke", "base", "stand", "arm", "leaf", "cam", "pivot")
OBSTACLES = ("servo", "board", "wing", "horn")     # reference envelopes: in the way for (e), not checked in (d)
PART_ITEM = {"stack_middle_deck": "StackMiddleDeck", "stack_top_deck": "StackTopDeck",
             "stack_bottom_deck": "StackBottomDeck", "tilt_ring_mg90s": "TiltRing", "pan_yoke": "PanYoke",
             "base": "Base", "striker_stand": "StrikerStand", "striker_arm": "StrikerArm",
             "striker_spring": "StrikerSpring", "striker_cam": "StrikerCam"}

# README "Fixings" table and totals (main design, 9 QT boards + wing). regex on Spec -> (what, qty).
README_COUNTS = [
    (r"^M3x50 socket cap", "M3 x 50 (2 stack bolts + striker pivot)", 3),
    (r"^M3x55 socket cap", "M3 x 55 (tall_top stack bolts)", 0),
    (r"^M3x12 button head", "M3 x 12 button head (axles)", 2),
    (r"^M3x14 socket cap", "M3 x 14 (striker tower)", 4),
    (r"^M3x16 socket cap", "M3 x 16 (drive leaf pad)", 4),
    (r"^M3x20 socket cap", "M3 x 20 (drive pin)", 1),
    (r"^M3x30 pan head, nylon", "M3 x 30 nylon (head stop screw)", 1),
    (r"^M3 hex nut, nylon", "M3 nylon nuts (stop screw)", 2),
    (r"^M3 hex nut, A2", "M3 nuts (stack bolts + pivot)", 3),
    (r"^M3 flat washer", "M3 washers (2 axles + 2 optional stack; tall_top: none)", 4),
    (r"^M3x16 pan head", "M3 x 16 bench screws (optional)", 4),
    (r"^M3x12 pan head self-tap", "M3 x 12 self-tap (DS3240 + MG995 cam servo tabs)", 12),
    (r"^M2\.5x10 pan head", "M2.5 x 10 (QT boards)", 18),
    (r"^M2\.5x6 pan head", "M2.5 x 6 (wing)", 8),
    (r"^M2\.5x8 pan head self-tap", "M2.5 x 8 self-tap (DS3240 horn arms)", 4),
    (r"^M2\.5 hex nut", "M2.5 nuts", 22),
    (r"^M2\.5x3 round spacer", "M2.5 x 3 mm spacers", 18),
    (r"^M2\.5x10 F-F hex standoff", "M2.5 x 10 F-F standoffs", 4),
    (r"^M2x6 pan head self-tap", "M2 x 6 self-tap (MG90S horn arms)", 2),
    (r"^M2x8 pan head self-tap", "M2 x 8 self-tap (MG90S roll servo tabs)", 2),
    (r"^M3x10 socket cap", "horn centre screws, DS3240 (servo-supplied / M3 x 10)", 2),
    (r"^M2x8 pan head,", "horn centre screw, MG90S roll (servo-supplied / M2 x 8)", 1),
    (r"^M3x8 pan head,", "horn centre screw, MG995 cam servo (servo-supplied / M3 x 8)", 1),
    (r"^623ZZ", "623ZZ bearings", 2),
    (r"^1\.75 mm filament", "pawl pin (1.75 mm filament)", 1),
]


def counts(plan, variant=""):
    """(a): rows (what, README qty, model qty, ok) plus specs that matched no row."""
    specs = {}
    for it in plan["items"]:
        if it.fix:
            specs[it.spec] = specs.get(it.spec, 0) + 1
    rows, used = [], set()
    for rx, what, qty in README_COUNTS:
        if variant == "tall_top" and "M3x50" in rx:
            qty = 1
        if variant == "tall_top" and "M3x55" in rx:
            qty = 2
        if variant == "tall_top" and "washer" in rx:
            qty = 2
        n = sum(c for s, c in specs.items() if re.search(rx, s))
        used |= {s for s in specs if re.search(rx, s)}
        rows.append((what, qty, n, n == qty))
    return rows, {s: c for s, c in specs.items() if s not in used}


def print_counts(rows, extra):
    print(f"\n(a) fixing counts vs README\n  {'fixing':58s} README  model")
    for what, qty, n, ok in rows:
        print(f"  {what:58s} {qty:6d} {n:6d}  {'OK' if ok else 'DIFF'}")
    for s, c in extra.items():
        print(f"  (not in README table) {s}: {c}")


def fix_axis(it):
    return it.placement.Rotation.multVec(V(0, 0, 1))


def _line_offset(p, o, a):
    """Distance of point p from the line through o along unit a."""
    d = p - o
    return (d - a * d.dot(a)).Length


def _angle(a, b):
    c = min(1.0, abs(a.dot(b)) / (a.Length * b.Length))
    return math.degrees(math.acos(c))


def _hole_face(shape, p, r_tol=0.35):
    """Nearest cylindrical face of radius ~dia/2 coaxial-ish with point p ->
    (offset of p from its axis, angle, point on its axis nearest p) or None."""
    best = None
    pos, ax, r = V(p["pos"]), V(p["axis"]), p["dia"] / 2
    bb = App.BoundBox(pos.x, pos.y, pos.z, pos.x, pos.y, pos.z)
    bb.enlarge(p["depth"] + 3)
    for f in shape.Faces:
        srf = f.Surface
        if not isinstance(srf, Part.Cylinder) or abs(srf.Radius - r) > r_tol or not f.BoundBox.intersect(bb):
            continue
        ang = _angle(srf.Axis, ax)
        if ang > 5:
            continue
        ah = V(srf.Axis).normalize()
        off = _line_offset(pos, srf.Center, ah)
        if best is None or off < best[0]:
            best = (off, ang, srf.Center + ah * (pos - srf.Center).dot(ah))
    return best


def _slot_offset(p):
    """QT / wing holes sit in swirly-grid slots: distance from the slot centreline (mm)."""
    import make_sensor_stack as ss
    import swirly_grid as sg
    x, y = p["pos"].x + ss.GRID_X / 2, p["pos"].y + ss.GRID_Y / 2
    return min(sg._dist_to_segment((x, y), a, b) for a, b in sg.slots(ss.ROWS, ss.COLS))


def coax(plan, items_by_name):
    """(b): for every fixing with holes -> (name, worst offset, worst angle, how)."""
    out = []
    for it in plan["items"]:
        holes = it.check.get("holes") if it.fix else None
        if not holes:
            continue
        a = fix_axis(it)
        o = it.placement.Base
        worst = (0.0, 0.0, "")
        for p in holes:
            ang = _angle(a, V(p["axis"]))
            off = _line_offset(V(p["pos"]), o, a)
            how = "point"
            if p["kind"] in ("slot", "slot_wing"):
                off, how = max(off, _slot_offset(p)), "slot centreline"
            elif p["kind"] in ("pilot", "through", "pin") and p["part"] in PART_ITEM:
                part = items_by_name[PART_ITEM[p["part"]]].world_shape()
                hf = _hole_face(part, p)
                if hf is None:
                    how = "NO HOLE FACE FOUND"
                    off = 99.0
                else:
                    off = max(off, _line_offset(hf[2], o, a))       # the real hole's axis vs the screw
                    ang = max(ang, hf[1])
                    how = "hole face"
            if off > worst[0] or ang > worst[1] or how.startswith("NO"):
                worst = (max(off, worst[0]), max(ang, worst[1]), how)
        out.append((it.name, it.spec, worst[0], worst[1], worst[2]))
    return out


def _target(items_by_name, ref):
    """Seat target: an item's world shape; 'Name:pcb' = the board envelope's bottom PCB_T slab."""
    name, _, sub = ref.partition(":")
    shp = items_by_name[name].world_shape()
    if sub == "pcb":
        import make_sensor_stack as ss
        bb = shp.BoundBox
        shp = shp.common(Part.makeBox(bb.XLength + 2, bb.YLength + 2, ss.PCB_T, V(bb.XMin - 1, bb.YMin - 1, bb.ZMin)))
    return shp


def _seat_radii(fix):
    d = fix.dims
    if fix.kind == "screw":
        return d["d"] / 2 + 0.15, d["head_d"] / 2 - 0.1
    if fix.kind == "nut":
        return d["d"] / 2 + 0.15, d["af"] / 2 - 0.1
    if fix.kind == "bearing":
        return 3.95, d["od"] / 2 - 0.1                 # outer ring
    if fix.kind == "standoff":
        return d["d"] / 2 + 0.3, d["af"] / 2 - 0.1
    return d.get("id", d["d"]) / 2 + 0.1, d["od"] / 2 - 0.1   # washer, spacer


def seats(plan, items_by_name):
    """(c): -> (name, target, gap, sunk) ; sunk = probes just inside the seat face end up inside the target."""
    out = []
    for it in plan["items"]:
        if not it.fix:
            continue
        ref = it.check.get("seat")
        if not ref:
            if it.check.get("seat_exempt"):
                out.append((it.name, "-", None, False, it.check["seat_exempt"]))
            continue
        tgt = _target(items_by_name, ref)
        ri, ro = _seat_radii(it.fix)
        ring = Part.makeCylinder(ro, 0.01).cut(Part.makeCylinder(ri, 0.03, V(0, 0, -0.01)))
        ring.Placement = it.placement
        gap = ring.distToShape(tgt)[0]
        rm = (ri + ro) / 2
        sunk = 0
        for k in range(8):
            a = math.radians(45 * k)
            p = it.placement.multVec(V(rm * math.cos(a), rm * math.sin(a), 0.15))
            sunk += tgt.isInside(p, 1e-4, False)
        out.append((it.name, ref, gap, sunk > 0, it.check.get("seat_gap", "")))
    return out


def _cyl(pos, axis, r, z0, z1):
    """Cylinder radius r along unit axis from pos + axis*z0 to pos + axis*z1."""
    a = V(axis).normalize()
    return Part.makeCylinder(r, z1 - z0, V(pos) + a * z0, a)


def shanks(plan, items_by_name):
    """(d): shank overlap with printed parts outside the allowed hole cylinders -> (name, worst mm^3, part)."""
    printed = [it for it in plan["items"] if it.cat in PRINTED]
    shapes = {it.name: it.world_shape() for it in printed}
    out = []
    for it in plan["items"]:
        f = it.fix
        if not f or f.kind not in ("screw", "pin"):
            continue
        L, r = f.dims["length"], f.dims["d"] / 2
        a = fix_axis(it)
        o = it.placement.Base
        # the real shank (so a self-tapper's point counts as a point), not the head
        sh = f.shape if f.kind == "pin" else f.shape.common(Part.makeCylinder(r + 0.5, L + 1, V(0, 0, -L - 1)))
        sh = sh.copy()
        sh.Placement = it.placement
        allowed = [_cyl(c["pos"], c["axis"], c["r"], -c["start"] - c["depth"], -c["start"]) for c in it.check.get("allowed", [])]
        if it.check.get("allowed_tip"):
            allowed.append(_cyl(o, a, r + 0.05, -L - 0.01, -L + it.check["allowed_tip"]))
        if allowed:
            sh = sh.cut(Part.makeCompound(allowed))
        worst = (0.0, "")
        if not sh.Solids or sh.Volume < 1e-9:
            out.append((it.name, it.spec, 0.0, ""))
            continue
        for n, s in shapes.items():
            if not s.BoundBox.intersect(sh.BoundBox):
                continue
            v = sh.common(s).Volume
            if v > worst[0]:
                worst = (v, n)
        out.append((it.name, it.spec, worst[0], worst[1]))
    return out


# --- (e) access, on the animation's own timeline ---------------------------------------------------------
def timeline_module():
    """The macro's pure timeline functions, so the checks see exactly what the animation shows."""
    import importlib.machinery
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs", "assembly", "animate_assembly.FCMacro")
    loader = importlib.machinery.SourceFileLoader("animate_assembly_timeline", path)
    mod = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
    loader.exec_module(mod)
    return mod


def moves_json(it):
    return [dict(step=m["step"], path=[[p.x, p.y, p.z] for p in m["path"]], phase=m["phase"]) for m in it.moves]


def steps_json(plan):
    lst = [dict(key=s["key"], n=s["n"], title=s["title"], phases=s["phases"]) for s in plan["steps"]]
    return dict(list=lst, order={s["key"]: i for i, s in enumerate(lst)}, phases={s["key"]: s["phases"] for s in lst})


TOOL = {"cap": (1.4, 45.0, "hex key"), "button": (1.1, 45.0, "hex key"), "pan": (2.4, 60.0, "screwdriver"),
        "selftap": (2.4, 60.0, "screwdriver")}


def _sweep_radius(f):
    d = f.dims
    return {"screw": d.get("head_d", 0) / 2, "nut": d.get("af", 0) / 2, "washer": d.get("od", 0) / 2,
            "bearing": d.get("od", 0) / 2, "spacer": d.get("od", 0) / 2, "standoff": d.get("af", 0) / 2,
            "pin": d["d"] / 2}[f.kind] - 0.05


def _top(f):
    d = f.dims
    return d["head_h"] if f.kind == "screw" else (d["length"] / 2 if f.kind == "pin" else d["height"])


def access(plan, tl=None, samples=10):
    """(e): -> list of (name, step n, blocked mm^3, blockers, tool blocked mm^3, tool blockers, tool)."""
    tl = tl or timeline_module()
    steps = steps_json(plan)
    printed = [it for it in plan["items"] if it.cat in PRINTED + OBSTACLES]      # solid things in the way
    # a board envelope's 2.5 mm parts zone covers its own mounting holes: boards block as their PCB only
    world = {it.name: (_target({it.name: it}, it.name + ":pcb") if it.cat in ("board", "wing") else it.world_shape())
             for it in printed}
    mv = {it.name: moves_json(it) for it in plan["items"]}
    out = []
    for it in plan["items"]:
        f = it.fix
        if not f:
            continue
        m0 = it.moves[0]
        cur = steps["order"][m0["step"]]
        ph = steps["phases"][m0["step"]]
        u = ph.index(m0["phase"]) / len(ph)
        scene = []
        for p in printed:
            off, shown, _ = tl.offset_at(mv[p.name], steps, cur, u)
            busy = any(m["step"] == m0["step"] and m["phase"] == m0["phase"] for m in p.moves)
            if shown and not busy:
                s = world[p.name].copy()
                s.translate(V(*off))
                scene.append((p.name, s))
        # where this fixing is (sub-assembly carries pending) and its own path
        carry = [0.0, 0.0, 0.0]
        for m in mv[it.name][1:]:
            q = tl.path_point(m["path"], tl.phase_fraction(steps, m["step"], m["phase"], cur, u))
            carry = [carry[i] + q[i] for i in range(3)]
        pl = App.Placement(it.placement)
        pl.Base = pl.Base + V(*carry)
        r, top = _sweep_radius(f), _top(f)
        z0 = 0.15 if f.kind != "pin" else -f.dims["length"] / 2
        own = mv[it.name][0]["path"]
        a = pl.Rotation.multVec(V(0, 0, 1))
        if len(own) == 1 and _angle(V(*own[0]), a) < 0.5 and V(*own[0]).dot(a) > 0:
            # straight in along its own axis: one long cylinder is the whole sweep
            e = Part.makeCylinder(r, top - z0 + V(*own[0]).Length, V(0, 0, z0))
            e.Placement = pl
            poses = [e]
        else:
            env = Part.makeCylinder(r, top - z0, V(0, 0, z0))
            poses = []
            for k in range(samples + 1):
                q = tl.path_point(own, k / samples * 0.999)
                e = env.copy()
                e.Placement = App.Placement(pl.Base + V(*q), pl.Rotation)
                poses.append(e)
        sweep = Part.makeCompound(poses)
        tool_sh = None
        if f.kind == "screw" and f.dims["head"] in TOOL:
            tr, tlen, _ = TOOL[f.dims["head"]]
            tool_sh = Part.makeCylinder(tr, tlen, V(0, 0, top + 0.5))
            tool_sh.Placement = pl
        blk, tblk = {}, {}
        for n, s in scene:
            if s.BoundBox.intersect(sweep.BoundBox):
                v = sum(e.common(s).Volume for e in poses if e.BoundBox.intersect(s.BoundBox)) / len(poses)
                if v > 0.05:
                    blk[n] = v
            if tool_sh is not None and s.BoundBox.intersect(tool_sh.BoundBox):
                v = tool_sh.common(s).Volume
                if v > 0.05:
                    tblk[n] = v
        out.append((it.name, cur + 1, sum(blk.values()), sorted(blk), sum(tblk.values()), sorted(tblk),
                    TOOL.get(f.dims.get("head"), (0, 0, ""))[2] if f.kind == "screw" else ""))
    return out


def run_all(plan, variant="", do_access=True):
    """Run (a)-(e), print the tables, return a summary dict (worst cases)."""
    by = {it.name: it for it in plan["items"]}
    rows, extra = counts(plan, variant)
    print_counts(rows, extra)
    res = dict(counts_ok=all(r[3] for r in rows), count_diffs=[r for r in rows if not r[3]])
    cx = coax(plan, by)
    w = max(cx, key=lambda r: (r[2], r[3]))
    bad = [r for r in cx if r[2] >= 0.2 or r[3] >= 1.0]
    print(f"\n(b) coaxial: {len(cx)} fixings; worst offset {w[2]:.3f} mm / {max(r[3] for r in cx):.2f} deg ({w[0]}, {w[4]})"
          f" -> {'OK' if not bad else 'FAIL'}")
    for r in bad:
        print(f"    FAIL {r[0]} {r[1]}: {r[2]:.3f} mm {r[3]:.2f} deg ({r[4]})")
    res["coax_worst"], res["coax_bad"] = w, bad
    se = seats(plan, by)
    chk = [r for r in se if r[2] is not None]
    w = max(chk, key=lambda r: r[2])
    bad = [r for r in chk if r[2] >= 0.5 or r[3]]
    print(f"\n(c) seats: {len(chk)} checked; worst gap {w[2]:.3f} mm ({w[0]} on {w[1]}) -> {'OK' if not bad else 'FAIL'}")
    for r in bad:
        print(f"    FAIL {r[0]} on {r[1]}: gap {r[2]:.3f} sunk {r[3]}")
    for r in se:
        if r[2] is None:
            print(f"    exempt {r[0]}: {r[4]}")
        elif r[4] != "":
            print(f"    designed gap {r[0]}: {r[2]:.3f} mm (design {r[4]})")
    res["seat_worst"], res["seat_bad"] = w, bad
    sh = shanks(plan, by)
    w = max(sh, key=lambda r: r[2])
    bad = [r for r in sh if r[2] >= 1.0]
    print(f"\n(d) shanks: {len(sh)} screws/pins; worst overlap outside own hole {w[2]:.3f} mm^3 ({w[0]} {w[1]} in {w[3]})"
          f" -> {'OK' if not bad else 'FAIL'}")
    for r in sorted(sh, key=lambda r: -r[2])[:8]:
        if r[2] > 0.001:
            print(f"    {r[0]:28s} {r[1]:50s} {r[2]:8.3f} mm^3 in {r[3]}")
    res["shank_worst"], res["shank_bad"] = w, bad
    if do_access:
        ac = access(plan)
        bl = [r for r in ac if r[2] > 1.0]
        tb = [r for r in ac if r[4] > 1.0]
        print(f"\n(e) access: {len(ac)} fixings; insertion path blocked: {len(bl)}; straight tool blocked: {len(tb)}")
        for r in bl:
            print(f"    BLOCKED step {r[1]:2d} {r[0]:28s} {r[2]:8.2f} mm^3 by {r[3]}")
        for r in tb:
            print(f"    tool    step {r[1]:2d} {r[0]:28s} {r[4]:8.2f} mm^3 ({r[6]}) by {r[5]}")
        res["access_blocked"], res["tool_blocked"] = bl, tb
    return res
