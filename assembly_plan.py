"""Assembly plan for the animated assembly: steps, fixings and insertion paths.

Everything is derived from make_gimbal.geometry(): the printed parts and reference envelopes
come from the generators, and every fixing is placed on a fixing point the generators
recorded where they cut its hole (make_gimbal.POINTS). Change a parameter and the fixings
follow; the screw lengths for the stack bolts are picked from the grip.

An Item is one document object. It appears in one step (its first move) and may be carried
by later steps as part of a sub-assembly:
    moves = [dict(step=key, path=[v0, v1, ...], phase=n), ...]
Each path is a polyline of offsets from the final placement, ending at zero (implied).
At any time the displayed offset is the sum of the pending moves' offsets.
"""

import FreeCAD as App
import Part

import fixings as fx

V = App.Vector

# Colours as in docs/assembly-3d.png (RGB 0..1); fixings by material.
COLORS = {
    "stack": (0.96, 0.58, 0.18), "spine": (0.55, 0.30, 0.12), "board": (0.16, 0.30, 0.78),
    "wing": (0.08, 0.08, 0.10), "ring": (0.22, 0.62, 0.32), "yoke": (0.30, 0.46, 0.82),
    "base": (0.62, 0.63, 0.67), "stand": (0.85, 0.22, 0.60), "arm": (0.97, 0.88, 0.20),
    "leaf": (0.93, 0.76, 0.10), "cam": (0.80, 0.12, 0.12), "pivot": (0.80, 0.12, 0.12),
    "servo": (0.18, 0.18, 0.20), "horn": (0.92, 0.92, 0.90), "bearing": (0.80, 0.80, 0.85),
}
MATERIAL_COLORS = {"A2": (0.58, 0.61, 0.66), "steel": (0.42, 0.44, 0.48), "nylon": (0.96, 0.95, 0.88),
                   "brass": (0.86, 0.68, 0.28), "PETG/PLA": (0.15, 0.15, 0.15)}

# Tools per fixing head / kind (shown in the Steps sheet).
TOOLS = {"cap:M3": "2.5 mm hex key", "cap:M2.5": "2 mm hex key", "button:M3": "2 mm hex key",
         "pan:M2.5": "#1 Phillips", "pan:M3": "#1 Phillips", "pan:M2": "#0 Phillips",
         "selftap:M2": "#0 Phillips", "selftap:M2.5": "#1 Phillips", "selftap:M3": "#1 Phillips",
         "nut:M3": "5.5 mm spanner / nut driver", "nut:M2.5": "5 mm nut driver",
         "standoff:M2.5": "5 mm nut driver", "bearing": "vice or thumb press", "pin": "lighter or soldering iron"}

STD_LENGTHS = (4, 5, 6, 8, 10, 12, 14, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80)


class Item:
    """One object in the animated document."""

    def __init__(self, name, label, shape, cat, step, path=None, phase=0, placement=None, fix=None,
                 color=None, check=None, optional=False, note=""):
        self.name, self.label, self.shape, self.cat = name, label, shape, cat
        self.placement = placement or App.Placement()
        self.fix = fix                          # fx.Fixing or None
        self.moves = [dict(step=step, path=[V(p) for p in (path or [])], phase=phase)]
        self.color = color or (MATERIAL_COLORS.get(fix.material, (0.7, 0.7, 0.7)) if fix else COLORS.get(cat, (0.7, 0.7, 0.7)))
        self.check = check or {}                # what the A2 checks need (hole, seat, ...)
        self.optional, self.note = optional, note

    @property
    def step(self):
        return self.moves[0]["step"]

    def carry(self, step, path, phase=0):
        self.moves.append(dict(step=step, path=[V(p) for p in path], phase=phase))
        return self

    @property
    def spec(self):
        return self.fix.spec if self.fix else ""

    @property
    def material(self):
        return self.fix.material if self.fix else ("PETG" if self.cat not in ("servo", "board", "wing") else "")

    def world_shape(self):
        s = self.shape.copy()
        s.Placement = self.placement.multiply(s.Placement)
        return s


def pick_length(need, lengths=STD_LENGTHS):
    """Shortest standard length >= need."""
    for n in lengths:
        if n >= need - 1e-6:
            return n
    raise ValueError(f"no standard length for {need:.1f} mm")


def fixing_item(name, label, fix, pos, axis, step, travel, phase=1, check=None, spin=0.0, optional=False, note="",
                path=None):
    """A fixing in its local frame placed on (pos, axis); it slides in along -axis from
    `travel` mm out (screws come in tip first from the head side, nuts from their side),
    unless an explicit path (offsets from the final position) is given."""
    a = V(axis).normalize()
    return Item(name, label, fix.shape, "fixing", step, path=path or [a * travel], phase=phase,
                placement=fx.place(fix.shape, pos, a, spin), fix=fix, check=check, optional=optional, note=note)


# Steps in build order: README "Assembly order", tightened where the access check (assembly_checks (e))
# showed a step could not be done as written:
#  - the ring goes into the yoke before the stack goes into the ring (the tilt horn's centre screw is driven
#    from inside the ring, along the tilt axis, and the stack's spines sit on that axis);
#  - only the middle deck goes into the ring first (the roll horn's centre screw is driven along the roll
#    axis from inside, where the upper spine would be); the other decks are added in place;
#  - the stack bolts go in from below with the nuts on top: an M3 head on top clips the roof edge;
#  - the tower is bolted to the yoke before the drive leaf goes in (the leaf's root pad covers two of the
#    bolt-access holes in the seat), and the drive pin goes into the arm on the bench (between the walls
#    there is 10 mm beside the fork for a 20 mm pin).
# key, title, notes. Steps that end up with no items (e.g. a deck with no boards) are dropped.
STEP_DEFS = [
    ("base_servo", "Pan servo into the base",
     "Centre the servo first. Drop the pan DS3240 in from above, bottom first, spline end (cable) at the "
     "wire notch, cable out of the open +X end. Insertion face: top plate, tab face only."),
    ("bench", "Base to the bench (optional)",
     "4 screws through the foot corner bosses into the bench. They go in through the top plate's windows: "
     "the heads are slightly wider than the window edge, so angle each screw in, then straighten it."),
    ("yoke_servo", "Tilt servo into the yoke",
     "Tilt DS3240 into the +Y upright from the inner face (toward the ring), bottom first, outward; "
     "cable end at the notch. Tab face only."),
    ("yoke_bearing", "623 bearing into the yoke", "Press the 623ZZ into the -Y upright from the outside."),
    ("pan_horn", "Pan horn onto the yoke", "Screw the pan horn into the underside pocket (2 arm screws)."),
    ("yoke_on_base", "Yoke onto the pan spline",
     "Centre the pan servo, press the horn onto its spline, then drive the centre screw down through the "
     "floor's access hole now, while nothing is above it."),
    ("ring_servo", "Roll servo into the tilt ring",
     "Roll MG90S into the +X plate from inside the ring (the tab face), bottom first, cable end at the notch. "
     "The outer face also works (both faces chamfered)."),
    ("ring_bearing", "623 bearing into the tilt ring", "Press the 623ZZ into the -X bearing plate from the outside."),
    ("tilt_horn", "Tilt horn onto the ring", "Screw the tilt horn into the +Y bar's pocket (2 arm screws)."),
    ("ring_in_yoke", "Empty ring into the yoke (tilt spline)",
     "Centre the tilt servo. Lower the ring into the yoke with its -Y axle boss in the bearing's lip hole "
     "(spring the uprights apart a little), press the horn onto the tilt spline (+Y) and drive the centre "
     "screw through the bar's access hole, from inside the still-empty ring."),
    ("tilt_axle", "Tilt idler axle", "M3 x 12 button head + washer through the yoke's 623 into the ring's -Y boss."),
    ("boards_bottom", "Boards onto the bottom deck",
     "On the bench: spacer under each board, screw down through the board's top-edge hole pair, nut under the deck."),
    ("boards_middle", "Wing and boards onto the middle deck",
     "On the bench: FeatherWing on 4 x 10 mm F-F standoffs (screw up from under the deck, screw down through "
     "the wing), QT boards on spacers, nuts under the deck."),
    ("boards_top", "Boards onto the top deck",
     "On the bench: spacer under each board, screw down through the hole pair, nut under the deck. Spots whose "
     "inner hole is under the roof: angle the screw in and turn the nut from below."),
    ("roll_horn", "Roll horn onto the middle deck", "Screw the MG90S horn into the +X wall's pocket (2 arm screws)."),
    ("middle_in_ring", "Middle deck into the ring (roll spline)",
     "Centre the roll servo. Lower the middle deck alone into the ring, -X axle boss in the bearing's lip hole "
     "(spring the ring a little), press the horn onto the MG90S spline (+X) and drive the centre screw through "
     "the wall's access hole along the roll axis, before the upper spine is in the way."),
    ("roll_axle", "Roll idler axle", "M3 x 12 button head + washer through the ring's 623 into the middle deck's -X boss."),
    ("stack", "Stack and clamp the decks",
     "Lower spine and bottom deck (bosses down) in under the ring from the side; upper spine and top deck "
     "(bosses up) from above. Two M3 bolts up from under the bottom deck (or tilt the ring 90 deg and put them "
     "in from the side); washers and nuts on the top deck's bosses, slid in under the roof edge."),
    ("cam_servo", "Cam servo into the striker tower",
     "On the bench: the MG90S-size 270 deg servo into the tower's -X wall from inside the tower (the tab face), "
     "bottom first, spline end (cable) up."),
    ("tower", "Tower onto the yoke",
     "4 x M3 x 14 through the tower's base plate into the yoke's -Y upright, reached through the holes in the "
     "leaf seat while the seat is still empty. The tower covers the tilt axle screw, so that must be in first."),
    ("pawl", "Pawl onto the drive leaf",
     "On the bench: pin the pawl between the leaf's lugs with ~12 mm of 1.75 mm filament; melt or flare the "
     "ends. It must swing freely and fall back onto its stop."),
    ("arm_pin", "Drive leaf pinned to the arm",
     "On the bench: bend the leaf's fork onto the arm's tail; drive the M3 x 20 through one fork slot, self-tap "
     "it through the tail and out through the other slot. Leave the head ~0.3 mm off the fork so the pin slides."),
    ("stop_screw", "Head stop screw",
     "Nylon M3 x 30 down through the overarm, nylon nut under it and on top. Set the lower nut so the head "
     "leaf is pressed down 11.7 mm (head face level), then lock with the top nut."),
    ("pivot", "Arm and leaf into the tower",
     "Bushing through the arm's hub, a spacer either side. Lower the arm (leaf hanging from it) between the "
     "walls, push the M3 x 50 through the -X wall, bushing and +X wall into the nut in its hex pocket; snug "
     "only. The arm must swing freely."),
    ("leaf", "Drive leaf root pad to the seat",
     "4 x M3 x 16 through the root pad, self-tapped into the seat's pilots, lugs and pawl toward the servo."),
    ("cam", "Cam onto the servo",
     "Stock horn trimmed to 14 mm and CA-glued into the cam. With the servo at 0 deg, press the cam on (pawl "
     "tip just clear of the base circle, 45 deg up from the cam's +Y side) and fit the centre screw. Check the "
     "hover gap: head 5 mm above the roof (shim the rest-stop lip to trim)."),
]

# Sub-assembly offsets (presentation only): where each sub-assembly is built before it is carried home.
YOKE_LIFT = 60.0          # yoke built above the base
RING_LIFT = 150.0         # tilt ring built above the yoke
MID_LIFT = 90.0           # middle deck built above the ring, then lowered in
TOP_LIFT = 150.0          # top deck + upper spine wait above, then come down
BOTTOM_BESIDE = 150.0     # bottom deck + lower spine wait beside (+X), slide in under the ring, then up
BOTTOM_DROP = 30.0
TOWER_BACK = 60.0         # striker tower built behind the yoke (-Y)
ARM_LIFT = 70.0           # hammer arm + drive leaf built above the tower


def _servo_dir(plates, name):
    return V(plates[name]["z"]).normalize()


def part_items(g, variant_label):
    """Printed parts and reference envelopes, with their first step and sub-assembly carries."""
    P = dict(g["printed"])
    st = g["st"]
    items = {}

    def add(name, label, shape, cat, step, path=None, phase=0):
        it = Item(name, label, shape, cat, step, path=path, phase=phase)
        items[name] = it
        return it

    plates = g["plates"]
    add("Base", "Base", P["base"], "base", "base_servo")
    add("PanServo", "REF DS3240 pan servo (270)", g["pan_servo"], "servo", "base_servo",
        [_servo_dir(plates, "base") * 60], phase=1)
    add("PanYoke", "Pan yoke", P["pan_yoke"], "yoke", "yoke_servo")
    add("TiltServo", "REF DS3240 tilt servo", g["tilt_servo"], "servo", "yoke_servo",
        [_servo_dir(plates, "pan_yoke") * 60], phase=1)
    ring, roll_servo, _ = next(iter(g["rings"].values()))
    add("TiltRing", "Tilt ring", ring, "ring", "ring_servo")
    add("RollServo", "REF MG90S roll servo", roll_servo, "servo", "ring_servo",
        [_servo_dir(plates, "tilt_ring_mg90s") * 45], phase=1)
    labels = {"stack_middle_deck": "Middle deck + roll walls", "stack_top_deck": "Top deck + roof",
              "stack_bottom_deck": "Bottom deck", "stack_spine_upper": "Upper spine", "stack_spine_lower": "Lower spine"}
    for name, shp in g["stack_parts"]:
        cat = "spine" if "spine" in name else "stack"
        add(name.title().replace("_", ""), labels[name] + ("" if "spine_upper" not in name else variant_label),
            shp, cat, "stack" if cat == "spine" else f"boards_{name.split('_')[1]}")
    add("StrikerStand", "Striker tower" + variant_label, st["stand"], "stand", "cam_servo")
    add("StrikerServo", "REF MG90S-size 270 deg cam servo", st["servo"], "servo", "cam_servo",
        [_servo_dir(plates, "striker_stand") * 20], phase=1)
    add("StrikerSpring", "Striker drive leaf (at rest)", st["leaf_rest"], "leaf", "pawl")
    add("StrikerPawl", "Striker pawl", st["pawl_rest"], "cam", "pawl", [V(0, -25, 0)], phase=0)
    add("StrikerArm", "Striker hammer arm (at rest)", st["arm_rest"], "arm", "arm_pin")
    names = ("StrikerBushing", "StrikerSpacerA", "StrikerSpacerB")
    for i, shp in enumerate(st["pivot_parts"]):
        sx = (0, -1, 1)[i]
        add(names[i], ("Striker bushing", "Striker spacer (-X)", "Striker spacer (+X)")[i], shp, "pivot", "pivot",
            [V(sx * 25 if sx else -60, 0, 0)])
    add("StrikerCam", "Striker cam (+ trimmed horn, CA-glued)", st["cam"], "cam", "cam", [V(25, 0, 0)])
    return items


# --- fixings --------------------------------------------------------------------------------------
# Which board spots carry a board (the 9 PR 839 breakouts + the wing). Every spot is in
# make_sensor_stack.board_layout(); pass boards="all" to fill all 21.
# The top deck is left empty by default: on two of its spots (x = +10.16) the inner screw sits under the
# roof's overhang (see the access check with ASSY_BOARDS=all).
POPULATED = ("middle_FeatherWing", "middle_QT_low1", "middle_QT_high1", "middle_QT_high2", "middle_QT_high3",
             "middle_QT_high4", "bottom_QT_high1", "bottom_QT_high2", "bottom_QT_high3", "bottom_QT_high4")


def _hole(p, r=None, depth=None, start=0.0):
    """Allowed-overlap cylinder for check (d): from p['pos'] - axis*start, along -axis."""
    return dict(pos=V(p["pos"]), axis=V(p["axis"]), r=r if r is not None else p["dia"] / 2,
                depth=depth if depth is not None else p["depth"], start=start)


def board_fixings(g, pts, populated):
    """QT boards: M2.5 x 10 nylon pan head + 3 mm spacer + nut per hole. Wing: 10 mm F-F standoff,
    M2.5 x 6 from under the deck and through the wing."""
    import make_sensor_stack as ss
    out = []
    qt_screw = fx.make("pan", "M2.5", 10, material="nylon")
    wing_screw = fx.make("pan", "M2.5", 6, material="nylon")
    nut, spc = fx.make("nut", "M2.5", material="nylon"), fx.make("spacer", "M2.5", ss.QT_SPACER)
    stand = fx.make("standoff", "M2.5", ss.WING_STANDOFF)
    for p in pts.values():
        if not p["key"].startswith("board_") or p["board"] not in populated:
            continue
        deck, b = p["part"].split("_")[1], p["board"]
        step, tag = f"boards_{deck}", f"{b}_{p['key'][-1]}"
        top, bot = V(p["pos"]), V(p["pos"]) - V(0, 0, p["depth"])
        pcb_top = top + V(0, 0, p["mount_h"] + p["pcb_t"])
        slot = _hole(p, r=p["dia"] / 2 + 0.2)
        if p["board_kind"] == "qt":
            out.append(fixing_item(f"Spacer_{tag}", f"Spacer {tag}", spc, top, (0, 0, 1), step, 25, phase=1,
                                   check=dict(seat=f"Stack{deck.title()}Deck")))
            out.append(fixing_item(f"Screw_{tag}", f"Screw {tag}", qt_screw, pcb_top, (0, 0, 1), step, 25, phase=4,
                                   check=dict(holes=[p], allowed=[slot], seat=f"REF_{b}:pcb", nut_gap=p["depth"])))
            out.append(fixing_item(f"Nut_{tag}", f"Nut {tag}", nut, bot, (0, 0, -1), step, 20, phase=5,
                                   check=dict(seat=f"Stack{deck.title()}Deck")))
        else:
            out.append(fixing_item(f"Standoff_{tag}", f"Standoff {tag}", stand, top, (0, 0, 1), step, 30, phase=1,
                                   check=dict(seat=f"Stack{deck.title()}Deck")))
            out.append(fixing_item(f"ScrewLow_{tag}", f"Screw under deck {tag}", wing_screw, bot, (0, 0, -1), step, 20,
                                   phase=2, check=dict(holes=[p], allowed=[slot], seat=f"Stack{deck.title()}Deck")))
            out.append(fixing_item(f"ScrewTop_{tag}", f"Screw through wing {tag}", wing_screw, pcb_top, (0, 0, 1), step,
                                   25, phase=4, check=dict(holes=[p], allowed=[], seat=f"REF_{b}:pcb")))
    return out


def stack_fixings(g, pts, variant):
    """Two clamp bolts, up from under the bottom deck (an M3 head on the top deck's boss would clip the
    roof edge on the way in); nuts on top, slid in under the roof edge. Length from the grip; the
    optional washers (under the nuts) only if they still fit."""
    out = []
    nut, wsh = fx.make("nut", "M3", material="A2"), fx.make("washer", "M3", material="A2")
    for i in (0, 1):
        p = pts[f"stack_bolt{i}"]
        grip, top, bot = p["depth"], V(p["pos"]), V(p["nut"])
        use_w = pick_length(grip + nut.dims["height"]) >= grip + nut.dims["height"] + wsh.dims["height"] + 0.3
        w = wsh.dims["height"] if use_w else 0.0
        L = pick_length(grip + nut.dims["height"] + w)
        bolt = fx.make("cap", "M3", L, material="A2", note="brass or nylon preferred")
        sx = 1 if top.x > 0 else -1
        lift = L - grip + 1.0                                        # clear of the bolt tip
        side = [V(sx * 25, 0, lift), V(0, 0, lift)]
        out.append(fixing_item(f"StackBolt{i}", f"Stack bolt {i} (up from below)", bolt, bot, (0, 0, -1), "stack",
                               L + 1, phase=1, check=dict(holes=[p], allowed=[_hole(p, r=p["dia"] / 2 + 0.05)],
                                                          seat="StackBottomDeck")))
        if use_w:
            out.append(fixing_item(f"StackWasher{i}", f"Stack washer {i} (optional)", wsh, top, (0, 0, 1), "stack", 0,
                                   phase=2, optional=True, check=dict(seat="StackTopDeck"), path=side))
        out.append(fixing_item(f"StackNut{i}", f"Stack nut {i}", nut, top + V(0, 0, w), (0, 0, 1), "stack", 0, phase=3,
                               check=dict(seat=f"StackWasher{i}" if use_w else "StackTopDeck"), path=side))
    return out


# servo name in PLATES -> (step, screw kind, size, length, servo item)
TAB_SCREWS = {"base": ("base_servo", "selftap", "M3", 12, "PanServo"),
              "pan_yoke": ("yoke_servo", "selftap", "M3", 12, "TiltServo"),
              "tilt_ring_mg90s": ("ring_servo", "selftap", "M2", 8, "RollServo"),
              "striker_stand": ("cam_servo", "selftap", "M2", 8, "StrikerServo")}
# horn pocket in HORNS -> (screw-to-part step, press-on step, arm screw, centre screw)
HORN_STEPS = {"pan_yoke": ("pan_horn", "yoke_on_base", ("selftap", "M2.5", 8), ("cap", "M3", 10)),
              "tilt_ring_mg90s": ("tilt_horn", "ring_in_yoke", ("selftap", "M2.5", 8), ("cap", "M3", 10)),
              "stack_middle_deck": ("roll_horn", "middle_in_ring", ("selftap", "M2", 6), ("pan", "M2", 8)),
              "striker_cam": ("cam", "cam", None, ("pan", "M2", 6))}


def servo_fixings(g, pts):
    out = []
    for name, (step, kind, size, L, servo) in TAB_SCREWS.items():
        scr = fx.make(kind, size, L, material="steel", note="or servo-supplied")
        for p in [q for q in pts.values() if q["key"].startswith(name + "_tab")]:
            hole = _hole(p, r=scr.dims["d"] / 2 + 0.05)
            out.append(fixing_item(f"TabScrew_{p['key']}", f"Tab screw {p['key']}", scr, p["pos"], p["axis"], step,
                                   L + 12, phase=2, check=dict(holes=[p], allowed=[hole], seat=servo)))
    return out


def horn_model(h, top, spline_r=0.0):
    """Stock-style horn in the horn_pocket frame: arm sunk in the pocket (z in [-depth, 0]), hub up to
    `top`, with a socket over the servo spline (its top is at z = hub)."""
    c = 0.15
    arm = Part.makeBox(h["len"] - 2 * c, h["width"] - 2 * c, h["depth"], V(-h["len"] / 2 + c, -h["width"] / 2 + c, -h["depth"]))
    hub = Part.makeCylinder(h["width"] / 2 + 1.5 - c, top + h["depth"], V(0, 0, -h["depth"]))
    shape = arm.fuse(hub).cut(Part.makeCylinder(1.7, top + h["depth"] + 2, V(0, 0, -h["depth"] - 1)))
    if top > h["hub"]:
        shape = shape.cut(Part.makeCylinder(spline_r + 0.1, top - h["hub"] + 1, V(0, 0, h["hub"])))
    if h.get("pilot"):
        for x in (-h["len"] / 2 + 3.5, h["len"] / 2 - 3.5):
            shape = shape.cut(Part.makeCylinder(h.get("arm_screw", 2.2) / 2, h["depth"] + 2, V(x, 0, -h["depth"] - 1)))
    return shape.removeSplitter()


def horn_fixings(g, pts):
    """Horns (stock, or the printed ones) with arm screws (screwed to the part first) and centre screws."""
    import make_gimbal as mg
    out = []
    for name, (s_step, p_step, arm_s, ctr_s) in HORN_STEPS.items():
        p = pts[f"{name}_horn"]
        h, fr = p["horn"], p["frame"]
        top = h["hub"] + (3.5 if name != "striker_cam" else 0.0)
        servo = mg.MG90S if name in ("stack_middle_deck", "striker_cam") else mg.STANDARD
        shp = mg.placed(horn_model(h, top, servo["spline_r"]), fr["origin"], fr["x"], fr["z"])
        z = V(fr["z"]).normalize()
        label = {"striker_cam": "Cam horn, trimmed to 14 mm (CA-glued in the cam)"}.get(
            name, f"Horn on {name} (stock, or the printed horn)")
        hi = Item(f"Horn_{name}", label, shp, "horn", s_step, path=[z * 25] if name != "striker_cam" else [V(25, 0, 0)])
        out.append(hi)
        if arm_s:
            scr = fx.make(arm_s[0], arm_s[1], arm_s[2], material="steel")
            for i in (0, 1):
                q = pts[f"{name}_hornarm{i}"]
                out.append(fixing_item(f"HornScrew_{name}{i}", f"Horn arm screw {name} {i}", scr, q["pos"], q["axis"],
                                       s_step, arm_s[2] + 12, phase=1,
                                       check=dict(holes=[q], allowed=[_hole(q, r=scr.dims["d"] / 2 + 0.05)],
                                                  seat=hi.name)))
        q = pts[f"{name}_horncentre"]
        scr = fx.make(ctr_s[0], ctr_s[1], ctr_s[2], material="steel", note="printed horn; stock horn: servo-supplied")
        out.append(fixing_item(f"HornCentre_{name}", f"Horn centre screw {name}", scr, q["pos"], q["axis"], p_step,
                               ctr_s[2] + 25, phase=2, check=dict(holes=[q], allowed=[], seat=hi.name)))
    return out


def axle_fixings(g, pts):
    """623ZZ pressed in from outside, then M3 x 12 button head + washer through it into the boss pilot."""
    out = []
    for brg_key, pilot_key, b_step, a_step, carrier in (("ring_bearing", "stack_axle", "ring_bearing", "roll_axle", "TiltRing"),
                                                        ("yoke_bearing", "ring_axle", "yoke_bearing", "tilt_axle", "PanYoke")):
        p, q = pts[brg_key], pts[pilot_key]
        a = V(p["axis"])
        brg, wsh = fx.make("bearing"), fx.make("washer", "M3", material="A2")
        scr = fx.make("button", "M3", 12, material="A2")
        tag = brg_key.split("_")[0].title()
        out.append(fixing_item(f"Bearing{tag}", f"623ZZ bearing in the {brg_key.split('_')[0]}", brg,
                               V(p["pos"]) - a * brg.dims["height"], a, b_step, 30, phase=1,
                               check=dict(seat=carrier + ":lip")))
        out.append(fixing_item(f"AxleWasher{tag}", f"Axle washer ({tag.lower()})", wsh, p["pos"], a, a_step, 20, phase=0,
                               check=dict(seat=f"Bearing{tag}")))
        out.append(fixing_item(f"AxleScrew{tag}", f"Axle screw ({tag.lower()})", scr, V(p["pos"]) + a * wsh.dims["height"], a,
                               a_step, 30, phase=1,
                               check=dict(holes=[p, q], allowed=[_hole(q, r=1.55)], seat=f"AxleWasher{tag}")))
    return out


def bench_fixings(g, pts):
    out = []
    scr = fx.make("pan", "M3", 16, material="steel", note="or #4 wood screw")
    bench = [q for q in pts.values() if q["key"].startswith("base_bench")]
    c = sum((V(q["pos"]) for q in bench), V()) * (1.0 / len(bench))
    for p in bench:
        # the heads are a little wider than the top plate's windows above them: angled in, then straight
        d = V(c.x - p["pos"].x, c.y - p["pos"].y, 0).normalize()
        a = V(p["axis"])
        path = [d * 6 + a * 35, a * 6]
        out.append(fixing_item(f"BenchScrew{p['key'][-1]}", f"Bench screw {p['key'][-1]} (optional)", scr, p["pos"],
                               p["axis"], "bench", 0, phase=0, optional=True, path=path,
                               check=dict(holes=[p], allowed=[_hole(p, r=1.55)], seat="Base")))
    return out


def striker_fixings(g, pts):
    out = []
    for i in range(4):
        p = pts[f"stand_bolt{i}"]
        q = pts[p["mate"]]                                  # the pilot in the yoke's upright
        scr = fx.make("cap", "M3", 14, material="A2")
        out.append(fixing_item(f"StandBolt{i}", f"Tower bolt {i}", scr, p["pos"], p["axis"], "tower", 45, phase=1,
                               check=dict(holes=[p, q], allowed=[_hole(p, r=1.75), _hole(q, r=1.55)], seat="StrikerStand")))
    p = pts["pivot"]
    L = pick_length(p["depth"] - 0.01)
    piv = fx.make("cap", "M3", L, material="A2")
    out.append(fixing_item("PivotBolt", "Arm pivot bolt", piv, p["pos"], p["axis"], "pivot", L + 15, phase=2,
                           check=dict(holes=[p], allowed=[_hole(p, r=1.75)], seat="StrikerStand")))
    out.append(fixing_item("PivotNut", "Arm pivot nut (in the +X hex pocket)", fx.make("nut", "M3", material="A2"),
                           p["nut"], p["nut_axis"], "pivot", 20, phase=1, check=dict(seat="StrikerStand")))
    pad = fx.make("cap", "M3", 16, material="A2")
    for i in range(4):
        p = pts[f"leaf_pad{i}"]
        pil = dict(p, pos=V(p["pos"]) - V(p["axis"]) * p["depth"], dia=p["pilot_d"], depth=p["pilot_depth"])
        out.append(fixing_item(f"PadScrew{i}", f"Leaf pad screw {i}", pad, p["pos"], p["axis"], "leaf", 30, phase=0,
                               check=dict(holes=[p], allowed=[_hole(p, r=1.75), _hole(pil, r=1.55)], seat="StrikerSpring")))
    p = pts["drive_pin"]
    pin = fx.make("cap", "M3", 20, material="A2")
    out.append(fixing_item("DrivePin", "Drive pin (head ~0.3 mm off the fork)", pin, p["pos"], p["axis"], "arm_pin", 35,
                           phase=1, check=dict(holes=[p], allowed=[_hole(p, r=1.75, depth=20.0)], seat="StrikerSpring",
                                               seat_gap=p["head_gap"])))
    p = pts["stop_screw"]
    nyl = fx.make("nut", "M3", material="nylon")
    L = 30
    head = V(0, 0, p["tip_z"] + L)
    stop = fx.make("pan", "M3", L, material="nylon")
    out.append(fixing_item("StopNutLow", "Stop screw lower nut (nylon)", nyl, p["nut_low"], (0, 0, -1), "stop_screw", 20,
                           phase=0, check=dict(seat="StrikerArm"), path=[V(25, 0, 0)]))
    out.append(fixing_item("StopScrew", "Head stop screw (nylon)", stop, V(p["pos"].x, p["pos"].y, head.z), p["axis"],
                           "stop_screw", 40, phase=1,
                           check=dict(holes=[p], allowed=[_hole(p, r=1.75, depth=p["depth"] + 0.01)],
                                      allowed_tip=0.3, seat=None, seat_exempt="head stands clear; the nuts lock it")))
    out.append(fixing_item("StopNutTop", "Stop screw lock nut (nylon)", nyl, p["pos"], p["axis"], "stop_screw", 20,
                           phase=2, check=dict(seat="StrikerArm")))
    p = pts["pawl_pin"]
    fp = fx.make("pin", length=p["depth"] + 2 * fx.FLARE_H + 0.2)      # flared ends just outside the lugs
    it = fixing_item("PawlPin", "Pawl pin (1.75 mm filament)", fp, p["pos"], p["axis"], "pawl", 25, phase=1,
                     check=dict(holes=[p], allowed=[_hole(p, r=p["dia"] / 2 + 0.05, depth=p["depth"] + 1.0, start=-p["depth"] / 2 - 0.5)], seat=None,
                                seat_exempt="pin: ends melted / flared"))
    out.append(it)
    return out


# --- the plan ---------------------------------------------------------------------------------------
SUBS = {   # sub-assembly -> the steps whose items it carries home (in the step named in build_plan)
    "yoke": ("yoke_servo", "yoke_bearing", "pan_horn"),
    "ring": ("ring_servo", "ring_bearing", "tilt_horn"),
    "middle": ("boards_middle", "roll_horn"),
    "bottom": ("boards_bottom",),
    "top": ("boards_top",),
    "arm": ("pawl", "arm_pin", "stop_screw"),
    "tower": ("cam_servo",),
}


def tool_key(fix):
    if fix.kind == "screw":
        return f"{fix.dims['head']}:{fix.size}"
    if fix.kind in ("nut", "standoff"):
        return f"{fix.kind}:{fix.size}"
    return fix.kind


def build_plan(g, variant_label="", boards=POPULATED, bench=True):
    """-> dict(steps=[...], items=[Item]). boards: names of populated board spots, or 'all'."""
    import make_gimbal as mg
    import make_sensor_stack as ss
    pts = {p["key"]: p for p in g["points"]}
    layout = {b["name"]: b for b in ss.board_layout()}
    populated = tuple(layout) if boards == "all" else tuple(boards)
    parts = part_items(g, variant_label)
    items = list(parts.values())
    for name, shp in g["board_refs"]:
        if name in populated:
            b = layout[name]
            items.append(Item(f"REF_{name}", f"REF {name}", shp, "wing" if b["kind"] == "wing" else "board",
                              f"boards_{b['deck']}", path=[V(0, 0, 30)], phase=3))
    items += board_fixings(g, pts, populated) + stack_fixings(g, pts, variant_label) + servo_fixings(g, pts)
    items += horn_fixings(g, pts) + axle_fixings(g, pts) + striker_fixings(g, pts)
    if bench:
        items += bench_fixings(g, pts)
    eng_r = mg.MG90S["spline"]["engage"]
    eng_t = mg.STANDARD["spline"]["engage"]
    paths = {"bottom": [V(BOTTOM_BESIDE, 0, -BOTTOM_DROP), V(0, 0, -BOTTOM_DROP)], "top": [V(0, 0, TOP_LIFT)],
             "middle": [V(-eng_r, 0, MID_LIFT), V(-eng_r, 0, 0)]}
    # decks with no boards arrive in the step that needs them; the spines arrive with their deck's group
    for d, first in (("bottom", "stack"), ("middle", "roll_horn"), ("top", "stack")):
        deck = parts[f"Stack{d.title()}Deck"]
        if not any(it.step == f"boards_{d}" and it is not deck for it in items):
            deck.moves[0]["step"] = first
            if first == "stack":
                deck.moves[0]["path"] = list(paths[d])
    parts["StackSpineLower"].moves[0]["path"] = list(paths["bottom"])
    parts["StackSpineUpper"].moves[0]["path"] = list(paths["top"])
    # sub-assembly carries, innermost first
    arm_extra = ("StrikerBushing", "StrikerSpacerA", "StrikerSpacerB")
    for sub, path, step, phase in (
            ("arm", [V(0, 0, ARM_LIFT)], "pivot", 1),
            ("tower", [V(0, -TOWER_BACK, 0)], "tower", 0),
            ("yoke", [V(0, 0, YOKE_LIFT)], "yoke_on_base", 0),
            ("ring", [V(0, -eng_t, RING_LIFT), V(0, -eng_t, 0)], "ring_in_yoke", 0),
            ("middle", paths["middle"], "middle_in_ring", 0),
            ("bottom", paths["bottom"], "stack", 0),
            ("top", paths["top"], "stack", 0)):
        for it in items:
            if it.step in SUBS[sub] or (sub == "arm" and it.name in arm_extra):
                it.carry(step, path, phase)
    # steps that have items, numbered in order
    steps = []
    for key, title, notes in STEP_DEFS:
        own = [it for it in items if it.step == key]
        if not own:
            continue
        n = len(steps) + 1
        fixes = {}
        for it in own:
            if it.fix:
                fixes[it.spec] = fixes.get(it.spec, 0) + 1
        tools = sorted({TOOLS[tool_key(it.fix)] for it in own if it.fix and tool_key(it.fix) in TOOLS})
        phases = sorted({m["phase"] for it in items for m in it.moves if m["step"] == key})
        steps.append(dict(n=n, key=key, title=title, group=f"{n:02d} - {title}", notes=notes,
                          parts=[it.label for it in own if not it.fix],
                          fixings=fixes, tools=tools, phases=phases))
    return dict(steps=steps, items=items, populated=populated)
