"""Live design: the gimbal rig as a jointed Assembly (FreeCAD 1.1 Assembly workbench), with
native Simulations, the striker's precomputed flex states and the motion timeline.

    freecadcmd -c "exec(open('make_motion.py').read(), {'__file__': 'make_motion.py', '__name__': '__main__'})"

Writes docs/motion/imu-gimbal-motion.FCStd, then reopens it and verifies the joints headless
(the solver keeps forward-kinematics poses, pulls perturbed poses back onto the joint axes,
and the Simulations drive the joints to their formulas). Options (environment):
    ASSY_CACHE=<file>   reuse / write the generator geometry (as make_assembly_animation.py)
    MOTION_VERIFY=0     skip the verification
    MOTION_BUILD=0      verify the saved document only
    MOTION_SIMS_ONLY=1  (with MOTION_BUILD=0) rebuild only the Simulations in the saved document

Every printed part, reference envelope and fixing comes from assembly_plan.build_plan(), on the
same fixing points and in the same colours as the animated assembly (docs/assembly/). Each
moving body is an App::Part component holding its parts and the fixings attached to it.
"""

import builtins
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import FreeCAD as App  # noqa: E402
import Part  # noqa: E402

sys.path.insert(0, os.path.join(App.getHomePath(), "Mod", "Assembly"))
if not App.GuiUp:                       # CommandCreateSimulation names QtCore at class level
    from PySide import QtCore
    builtins.QtCore = QtCore
import JointObject  # noqa: E402
import UtilsAssembly  # noqa: E402
import CommandCreateSimulation as CS  # noqa: E402

V = App.Vector
OUT_DIR = os.path.join(HERE, "docs", "motion")
DOC_FILE = "imu-gimbal-motion.FCStd"

# Components (moving bodies): key -> label. Parts and fixings go to the body they move with.
BODIES = {
    "Base": "Base (grounded): base, pan servo",
    "Yoke": "Pan yoke: tilt servo, 623, pan horn",
    "Tower": "Striker tower: cam servo, pivot bolt, bushing, spacers",
    "Ring": "Tilt ring: roll servo, 623, tilt horn, tilt axle screw",
    "Stack": "Sensor stack: decks, spines, boards, roll horn, roll axle screw",
    "Cam": "Striker cam (+ glued horn)",
    "Arm": "Striker hammer arm: drive pin, stop screw",
    "Leaf": "Striker drive leaf: pad screws, pawl pin",
    "Pawl": "Striker pawl",
}
RULES = [   # (body, exact names, name prefixes)
    ("Base", ("Base", "PanServo"), ("TabScrew_base_", "BenchScrew")),
    ("Yoke", ("PanYoke", "TiltServo", "BearingYoke", "Horn_pan_yoke", "HornCentre_pan_yoke"),
     ("HornScrew_pan_yoke", "TabScrew_pan_yoke_")),
    ("Ring", ("TiltRing", "RollServo", "BearingRing", "Horn_tilt_ring_mg90s", "HornCentre_tilt_ring_mg90s",
              "AxleScrewYoke", "AxleWasherYoke"), ("HornScrew_tilt_ring", "TabScrew_tilt_ring")),
    ("Stack", ("Horn_stack_middle_deck", "HornCentre_stack_middle_deck", "AxleScrewRing", "AxleWasherRing"),
     ("Stack", "REF_", "Spacer_", "Screw_", "Nut_", "Standoff_", "ScrewLow_", "ScrewTop_", "HornScrew_stack")),
    ("Tower", ("StrikerStand", "StrikerServo", "PivotBolt", "PivotNut", "StrikerBushing", "StrikerSpacerA",
               "StrikerSpacerB"), ("TabScrew_striker_stand_", "StandBolt")),
    ("Cam", ("StrikerCam", "Horn_striker_cam", "HornCentre_striker_cam"), ()),
    ("Arm", ("StrikerArm", "DrivePin", "StopScrew", "StopNutLow", "StopNutTop"), ()),
    ("Leaf", ("StrikerSpring", "PawlPin"), ("PadScrew",)),
    ("Pawl", ("StrikerPawl",), ()),
]


def body_of(name):
    for body, exact, prefixes in RULES:
        if name in exact:
            return body
    for body, exact, prefixes in RULES:
        if any(name.startswith(p) for p in prefixes):
            return body
    raise KeyError(f"no body for {name}")


def jcs(point, axis):
    """Joint coordinate system at point with its Z along axis (world = component frame at rest)."""
    return App.Placement(V(point), App.Rotation(V(0, 0, 1), V(axis)))


def joint_defs(model, pawl_pin):
    """name -> (label, type, parent, child, point, axis, (min, max) deg or None).
    Positive angle = right-handed about the axis: hammer up (+X), cam forward (-X), pawl fold (-X)."""
    K, P = model.K, model.P
    fold_max = 120.0
    return {
        "Pan": ("Pan (base - yoke, Z)", "Revolute", "Base", "Yoke", (0, 0, 0), (0, 0, 1), (-135.0, 135.0)),
        "Tilt": ("Tilt (yoke - tilt ring, Y)", "Revolute", "Yoke", "Ring", (0, 0, 0), (0, 1, 0), None),
        "Roll": ("Roll (tilt ring - stack, X)", "Revolute", "Ring", "Stack", (0, 0, 0), (1, 0, 0), (-90.0, 90.0)),
        "TowerFix": ("Tower bolted to the yoke", "Fixed", "Yoke", "Tower", (0, 0, 0), (0, 0, 1), None),
        "Cam": ("Cam (tower - cam, -X: forward = increasing pulse)", "Revolute", "Tower", "Cam", (0, K[0], K[1]), (-1, 0, 0),
                (0.0, 180.0)),
        "ArmPivot": ("Hammer arm pivot (tower - arm, X)", "Revolute", "Tower", "Arm", (0, P[0], P[1]), (1, 0, 0),
                     (0.0, 50.0)),
        "LeafRoot": ("Drive leaf root pad on the seat", "Fixed", "Tower", "Leaf", (0, 0, 0), (0, 0, 1), None),
        "PawlPin": ("Pawl pin (drive leaf - pawl, -X: fold)", "Revolute", "Leaf", "Pawl", (0, pawl_pin.y, pawl_pin.z),
                    (-1, 0, 0), (0.0, fold_max)),
    }


# Parent chain for forward kinematics (the macro and the checks use the same).
PARENT = {"Yoke": "Base", "Tower": "Yoke", "Ring": "Yoke", "Stack": "Ring", "Cam": "Tower", "Arm": "Tower",
          "Leaf": "Tower", "Pawl": "Leaf"}
JOINT_OF = {"Yoke": "Pan", "Ring": "Tilt", "Stack": "Roll", "Cam": "Cam", "Arm": "ArmPivot", "Pawl": "PawlPin",
            "Tower": "TowerFix", "Leaf": "LeafRoot"}


def fk(defs, angles):
    """Component placements for joint angles {joint: deg} (missing = 0)."""
    out = {"Base": App.Placement()}
    for body in ("Yoke", "Tower", "Ring", "Stack", "Cam", "Arm", "Leaf", "Pawl"):
        j = JOINT_OF[body]
        d = defs[j]
        rel = App.Placement(V(), App.Rotation(V(d[5]), angles.get(j, 0.0)), V(d[4]))
        out[body] = out[PARENT[body]].multiply(rel)
    return out


# --- document -------------------------------------------------------------------------------------------
def _feature(doc, name, label, shape, placement, props=()):
    o = doc.addObject("Part::Feature", name)
    o.Shape = shape
    o.Placement = placement
    o.Label = label
    for prop, typ, val in props:
        o.addProperty(f"App::Property{typ}", prop, "Motion")
        setattr(o, prop, val)
    o.Visibility = True
    return o


def add_components(doc, asm, plan):
    """App::Part per body, each holding its parts and fixings (world placements; the components
    sit at identity in the rest pose, so their frames are the world frame there)."""
    comps = {}
    for key, label in BODIES.items():
        c = asm.newObject("App::Part", f"Body_{key}")
        c.Label = label
        c.Visibility = True
        comps[key] = c
    members = {k: [] for k in BODIES}
    for it in plan["items"]:
        body = body_of(it.name)
        pl = it.placement.multiply(it.shape.Placement)       # the shape's own placement counts too
        o = _feature(doc, it.name, it.label, it.shape.copy(), pl,
                     (("IntendedColor", "Color", tuple(it.color)), ("Spec", "String", it.spec or it.label),
                      ("Material", "String", it.material), ("Body", "String", body)))
        comps[body].addObject(o)
        members[body].append(it.name)
    return comps, members


def add_joints(asm, comps, defs):
    jg = asm.newObject("Assembly::JointGroup", "Joints")
    g = jg.newObject("App::FeaturePython", "GroundedBase")
    JointObject.GroundedJoint(g, comps["Base"])
    g.Label = "Grounded: base"
    joints = {}
    for name, (label, typ, parent, child, point, axis, lim) in defs.items():
        j = jg.newObject("App::FeaturePython", f"Joint{name}")
        JointObject.Joint(j, JointObject.JointTypes.index(typ))
        j.Label = label
        j.Offset1 = jcs(point, axis)
        j.Offset2 = jcs(point, axis)
        j.Proxy.setJointConnectors(j, [[comps[parent], ["", ""]], [comps[child], ["", ""]]])
        if lim:
            j.EnableAngleMin, j.EnableAngleMax = True, True
            j.AngleMin, j.AngleMax = lim
        joints[name] = j
    return joints


def sine_series(ts, vals, n):
    """Motion formula (radians) for samples (t, deg) over [0, T]: the straight line from the
    first value to the last plus a half-range sine series of the rest, n terms.

    FreeCAD 1.1's motion formulas accept only + - * / ^, sin and cos (the arctan 'smooth
    impulse' examples in the Simulation help do not parse, and piecewise windows built from
    powers stall the integrator), and a constant far from the start pose makes the solver
    converge to the wrong branch. A sine series is smooth, starts exactly where the saved
    pose is, and needs nothing else. Returns (formula, fitted function)."""
    T = ts[-1]
    v0, v1 = math.radians(vals[0]), math.radians(vals[-1])
    res = [math.radians(v) - (v0 + (v1 - v0) * t / T) for t, v in zip(ts, vals)]
    b = []
    for k in range(1, n + 1):
        w = k * math.pi / T
        acc = 0.0
        for i in range(len(ts) - 1):                 # trapezoid on the (uneven) samples
            acc += 0.5 * (res[i] * math.sin(w * ts[i]) + res[i + 1] * math.sin(w * ts[i + 1])) * (ts[i + 1] - ts[i])
        b.append(2.0 / T * acc)
    terms = [f"({v0:.6f})", f"({(v1 - v0) / T:.6f})*time"]
    terms += [f"({bk:.6f})*sin({(k + 1) * math.pi / T:.6f}*time)" for k, bk in enumerate(b) if abs(bk) > 1e-6]
    def fn(t):
        return v0 + (v1 - v0) * t / T + sum(bk * math.sin((k + 1) * math.pi / T * t) for k, bk in enumerate(b))
    return "+".join(terms), fn


def _motion(asm, sim, joint, formula):
    m = asm.newObject("App::FeaturePython", "Motion")
    CS.Motion(m, "Angular", joint, formula)
    m.Label = f"{joint.Name[5:]}: {formula if len(formula) < 60 else 'sine series (' + str(formula.count('sin(')) + ' terms)'}"
    sim.Group = sim.Group + [m]
    return m


SIM_DT = 1.0 / 30


def n_terms(T):
    return int(min(160, math.ceil(7 * T)))


def add_simulations(asm, joints, tl):
    """Native Simulations, one per programme section, each starting from the saved rest pose:
    the section's own timeline rows (the macro's programme), preceded by the park move when the
    section starts parked. Every revolute is driven by a sine-series fit of its timeline column
    (sine_series(); the fit error is stored in each Simulation's FitJSON). The arm and pawl
    follow the cam model's values; the Simulation cannot bend the leaves (rigid bodies only),
    and it smooths the releases."""
    sg = UtilsAssembly.getSimulationGroup(asm)
    segs = tl["segments"]
    park = [r for r in tl["rows"] if segs[r[8]]["section"] == "Start" and segs[r[8]]["kind"] == "cam"]
    specs = [("SimPan", "1 Pan sweep +/-135 (hammer parked)", "1 Pan"),
             ("SimTilt", "2 Tilt 360 (hammer parked)", "2 Tilt"),
             ("SimRoll", "3 Roll +/-90 (hammer parked)", "3 Roll"),
             ("SimStriker", "4 Striker cam cycle (arm and pawl from the cam model)", "4 Striker")]
    cols = {"Pan": 1, "Tilt": 2, "Roll": 3, "Cam": 4, "ArmPivot": 5, "PawlPin": 6}
    sims = {}
    for name, label, section in specs:
        rows = [r for r in tl["rows"] if segs[r[8]]["section"] == section]
        t0 = rows[0][0]
        sec = [(r[0] - t0, r) for r in rows]
        pre = [(r[0] - park[0][0], r) for r in park]
        shift = pre[-1][0]
        seq = pre + [(t + shift, r) for t, r in sec if t > 0]
        ts = [t for t, _ in seq]
        sim = sg.newObject("App::FeaturePython", name)
        CS.Simulation(sim)
        sim.Label = label
        sim.aTimeStart, sim.bTimeEnd, sim.cTimeStepOutput, sim.jFramesPerSecond = 0.0, round(ts[-1], 4), SIM_DT, 30
        fit = {}
        for jn, c in cols.items():
            vals = [r[c] for _, r in seq]
            if all(abs(v) < 1e-9 for v in vals):
                f = "0"
            else:
                f, fn = sine_series(ts, vals, n_terms(ts[-1]))
                fit[jn] = max(abs(math.degrees(fn(t)) - v) for t, v in zip(ts, vals))
            _motion(asm, sim, joints[jn], f)
        sim.addProperty("App::PropertyString", "FitJSON", "Motion", "worst |sine series - timeline| per joint (deg)")
        sim.FitJSON = json.dumps(fit)
        sims[name] = sim
    return sims


def rebuild_simulations(path):
    """Replace the Simulations in a saved motion document (no geometry rebuild)."""
    doc = App.openDocument(path)
    asm = doc.getObject("Assembly")
    sg = UtilsAssembly.getSimulationGroup(asm)
    for sim in list(sg.Group):
        for m in list(sim.Group):
            doc.removeObject(m.Name)
        doc.removeObject(sim.Name)
    joints = {o.Name[5:]: o for o in doc.Objects if o.Name.startswith("Joint") and hasattr(o, "JointType")}
    tl = json.loads(doc.getObject("MotionData").TimelineJSON)
    add_simulations(asm, joints, tl)
    doc.recompute()
    doc.save()
    App.closeDocument(doc.Name)


def add_motion_data(doc, model, tl, leaf_shapes, arm_shapes, defs, members, extra):
    """Group with the timeline, the model numbers and the striker's precomputed flex shapes."""
    grp = doc.addObject("App::DocumentObjectGroup", "MotionData")
    grp.Label = "Motion data (timeline, striker flex states)"
    for prop, val in (("TimelineJSON", json.dumps({k: v for k, v in tl.items() if k != "events"} |
                                                  {"events": [{k: v for k, v in e.items() if k != "rows"} for e in tl["events"]]})),
                      ("ModelJSON", json.dumps(extra))):
        grp.addProperty("App::PropertyString", prop, "Motion")
        setattr(grp, prop, val)
    states = doc.addObject("App::DocumentObjectGroup", "StrikerStates")
    states.Label = "Striker flex states (hidden; the macro swaps them in)"
    grp.addObject(states)
    for name, label, shp in leaf_shapes + arm_shapes:
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        o.Label = label
        o.Visibility = False
        states.addObject(o)
    return grp


def flex_shapes(model):
    """Drive leaf at each whole degree of arm angle (its cantilever shape solved for that arm
    angle), and the arm with its head leaf bent x mm further (let-off), 0.5 mm steps to HOVER."""
    import make_striker as mk
    import striker_motion as sm
    leaf = []
    for d in range(0, int(math.ceil(model.phi_cam(sm.PARK_A))) + 2):
        leaf.append((f"LeafPhi{d:02d}", f"Drive leaf, arm at {d} deg", mk.leaf_solid(model.leaf, model.leaf_pts(float(d)))))
    arm = []
    lay = model.lay
    n = int(round(mk.HOVER / 0.5))
    for i in range(n + 1):
        x = 0.5 * i
        lx = dict(lay, pre=lay["pre"] + x, z_face=lay["z_face"] - x)
        body, head = mk.arm_parts(lx)
        arm.append((f"ArmLetOff{int(round(x * 10)):02d}", f"Hammer arm, head leaf bent {x:.1f} mm (let-off)",
                    body.fuse(head).removeSplitter()))
    leaf.append(("LeafFree", "Drive leaf as printed (straight)", mk.leaf_solid(model.leaf, model.leaf.shape())))
    fb, fh = mk.arm_parts(lay, installed=False)
    arm.append(("ArmFree", "Hammer arm as printed (head leaf straight)", fb.fuse(fh).removeSplitter()))
    return leaf, arm


def model_extra(model, defs, members, leaf_shapes, arm_shapes, pawl_pin):
    import make_striker as mk
    import striker_motion as sm
    return dict(
        K=list(model.K), P=list(model.P), pin0=list(model.pin0), w_pin=model.w_pin, pawl_pin=[pawl_pin.y, pawl_pin.z],
        phi_step=model.phis[1] - model.phis[0], stations=[list(s[1:]) for s in model.stations],
        leaf_grid=[[float(n[7:]), n] for n, _, _ in leaf_shapes if n.startswith("LeafPhi")],
        arm_grid=[[int(n[9:]) / 10.0, n] for n, _, _ in arm_shapes if n.startswith("ArmLetOff")],
        joints={k: dict(label=v[0], type=v[1], parent=v[2], child=v[3], point=list(v[4]), axis=list(v[5]),
                        limits=v[6]) for k, v in defs.items()},
        members=members, hover=mk.HOVER, z_roof=model.z_roof, park=model.phi_cam(sm.PARK_A),
        lobes=[list(x) for x in mk.LOBES], cam_r0=mk.CAM_R0, cam_h=model.h, b0=model.b0, bp=model.bp,
        k_leaf=model.k_leaf, d_rest=model.leaf.d_rest, m_arm_g=model.m_arm * 1000, m_head_g=model.m_head * 1000,
        i_tot=model.i_tot, i_head=model.i_head, head_preload=model.hl["preload"], head_k=model.hl["k"],
        e_roof=__import__("striker_motion").E_ROOF)


def build():
    import assembly_plan as ap
    import make_assembly_animation as maa
    import striker_motion as sm
    g = maa.geometry()
    plan = ap.build_plan(g, "", boards=ap.POPULATED, bench=True)
    sg = sm.striker_input(g["points"])
    model = sm.Striker(sg)
    info = g["st"]["info"]
    err = max(abs(a - b) for a, b in zip(list(model.K) + list(model.P), list(info["cam_yz"]) + list(info["pivot_yz"])))
    assert err < 0.06, f"striker model differs from the built striker ({err:.3f} mm)"
    print(f"striker model: cam {model.K}, pivot {model.P} (matches the build to {err:.3f} mm)", flush=True)
    tl = sm.build_timeline(model)
    tl["park"] = model.phi_cam(sm.PARK_A)
    print(f"timeline: {len(tl['rows'])} rows, {tl['duration']:.1f} s, park {tl['park']:.3f} deg", flush=True)
    leaf_shapes, arm_shapes = flex_shapes(model)
    print(f"flex shapes: {len(leaf_shapes)} leaf, {len(arm_shapes)} arm", flush=True)
    pts = {p["key"]: p for p in g["points"]}
    pawl_pin = V(pts["pawl_pin"]["pos"])
    doc = App.newDocument("imu_gimbal_motion")
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
    asm.Label = "IMU gimbal rig (live design)"
    comps, members = add_components(doc, asm, plan)
    defs = joint_defs(model, pawl_pin)
    joints = add_joints(asm, comps, defs)
    add_simulations(asm, joints, tl)
    extra = model_extra(model, defs, members, leaf_shapes, arm_shapes, pawl_pin)
    import make_gimbal as mg
    extra.update(striker_input={k: (v if not isinstance(v, tuple) else [list(x) for x in v]) for k, v in sg.items()},
                 ring_out_y=g["ring_in_y"] + mg.RING_BAR_T)
    add_motion_data(doc, model, tl, leaf_shapes, arm_shapes, defs, members, extra)
    doc.recompute()
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, DOC_FILE)
    doc.saveAs(path)
    print("wrote", path, f"({os.path.getsize(path) / 1e6:.1f} MB)", flush=True)
    with open(os.path.join(OUT_DIR, "motion-model.json"), "w") as f:
        json.dump(dict(events=[{k: v for k, v in e.items() if k != "rows"} for e in tl["events"]],
                       design=model.num, park=tl["park"], duration=tl["duration"],
                       segments=tl["segments"]), f, indent=1)
    App.closeDocument(doc.Name)
    return path, model, tl



def show_rig(doc):
    """A document saved headless reopens in the GUI with everything hidden: show the assembly,
    its components and parts and the joints, hide the motion data and the flex states."""
    asm = doc.getObject("Assembly")
    keep_hidden = {o.Name for o in doc.getObject("StrikerStates").Group} | {"StrikerStates", "MotionData"}
    for o in [asm] + asm.OutListRecursive:
        if o.Name not in keep_hidden and o.TypeId != "Assembly::SimulationGroup" and not hasattr(o, "Formula") \
                and not hasattr(o, "aTimeStart"):
            o.Visibility = True
    for n in keep_hidden:
        doc.getObject(n).Visibility = False


def finish_gui(doc_or_path, save=True):
    """In the FreeCAD GUI: give the joints, simulations and motions their view providers (a
    headless build has none, so the joint markers and the Simulation panel would be missing),
    apply the colours from IntendedColor, hide the flex states, and save."""
    import FreeCADGui as Gui
    doc = doc_or_path if hasattr(doc_or_path, "Objects") else App.openDocument(doc_or_path)
    for o in doc.Objects:
        vo = o.ViewObject
        if vo is None:
            continue
        proxy = getattr(o, "Proxy", None)
        cls = type(proxy).__name__ if proxy is not None else ""
        if not hasattr(getattr(vo, "Proxy", None), "getIcon"):       # headless: None or a placeholder int
            if cls == "Joint":
                JointObject.ViewProviderJoint(vo)
            elif cls == "GroundedJoint":
                JointObject.ViewProviderGroundedJoint(vo)
            elif cls == "Simulation":
                CS.ViewProviderSimulation(vo)
            elif cls == "Motion":
                vp = CS.ViewProviderMotion.__new__(CS.ViewProviderMotion)
                vp.app_obj = o                    # its __init__ relabels the motion before attach() has run
                vo.Proxy = vp
        if hasattr(o, "IntendedColor") and hasattr(vo, "ShapeColor"):
            vo.ShapeColor = tuple(o.IntendedColor[:3])
    show_rig(doc)
    Gui.SendMsgToActiveView("ViewFit")
    if save:
        doc.save()
    return doc


# --- verification (headless) -------------------------------------------------------------------------------
def _corners(shape):
    b = shape.BoundBox
    return [V(x, y, z) for x in (b.XMin, b.XMax) for y in (b.YMin, b.YMax) for z in (b.ZMin, b.ZMax)]


def joint_state(defs, comps, name, corners):
    """(signed angle deg, residual mm): the child's placement relative to its parent, as the
    best pure rotation about the joint's axis through its point; residual = worst corner miss."""
    _, typ, parent, child, point, axis, _ = defs[name]
    rel = comps[parent].Placement.inverse().multiply(comps[child].Placement)
    a = V(axis).normalize()
    r = rel.Rotation
    ang = math.degrees(r.Angle) * (1 if r.Axis.dot(a) >= 0 else -1) if r.Angle > 1e-12 else 0.0
    pure = App.Placement(V(), App.Rotation(a, ang), V(point))
    res = max((rel.multVec(c) - pure.multVec(c)).Length for c in corners[child])
    return ang, res


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def verify(path):
    doc = App.openDocument(path)
    asm = doc.getObject("Assembly")
    data = json.loads(doc.getObject("MotionData").ModelJSON)
    defs = {k: (v["label"], v["type"], v["parent"], v["child"], tuple(v["point"]), tuple(v["axis"]), v["limits"])
            for k, v in data["joints"].items()}
    comps = {k: doc.getObject(f"Body_{k}") for k in BODIES}
    corners = {k: _corners(Part.makeCompound([o.Shape for o in c.Group if hasattr(o, "Shape")])) for k, c in comps.items()}
    out = {}
    poses = [dict(Pan=60, Tilt=30, Roll=-45, Cam=110, ArmPivot=30, PawlPin=40),
             dict(Pan=-135, Tilt=180, Roll=90, Cam=69, ArmPivot=48, PawlPin=0),
             dict(Pan=135, Tilt=275, Roll=-90, Cam=170, ArmPivot=0, PawlPin=100)]
    # (a) forward-kinematics poses: the solver must leave them where they are
    worst = 0.0
    for pose in poses:
        want = fk(defs, pose)
        for k, c in comps.items():
            c.Placement = want[k]
        r = asm.solve()
        for k, c in comps.items():
            worst = max(worst, max((c.Placement.multVec(p) - want[k].multVec(p)).Length for p in corners[k]))
        print(f"verify FK pose {pose}: solve -> {r}", flush=True)
    out["fk_pose_solver_drift_mm"] = worst
    # (b) perturbed poses: the solver pulls every body back onto a pure rotation about its joint
    import random
    rnd = random.Random(1)
    worst_res, worst_dev = 0.0, 0.0
    for pose in poses:
        want = fk(defs, pose)
        for k, c in comps.items():
            wob = App.Placement(V(rnd.uniform(-.4, .4), rnd.uniform(-.4, .4), rnd.uniform(-.4, .4)),
                                App.Rotation(V(rnd.random(), rnd.random(), rnd.random()), rnd.uniform(-1.5, 1.5)))
            c.Placement = want[k] if k == "Base" else wob.multiply(want[k])
        asm.solve()
        for jn, d in defs.items():
            ang, res = joint_state(defs, comps, jn, corners)
            worst_res = max(worst_res, res)
            worst_dev = max(worst_dev, abs(_wrap(ang - pose.get(jn, 0.0))))
    out["perturbed_residual_mm"] = worst_res
    out["perturbed_angle_dev_deg"] = worst_dev
    print(f"verify: FK drift {worst:.2e} mm; perturbed: residual {worst_res:.2e} mm off the joint axes "
          f"(angle moved {worst_dev:.2f} deg by the wobble)", flush=True)
    out["simulations"] = verify_simulations(doc, asm, defs, comps, corners)
    App.closeDocument(doc.Name)
    return out


def eval_formula(f, t):
    return eval(f.replace("^", "**"), {"__builtins__": {}}, dict(sin=math.sin, cos=math.cos, pi=math.pi, time=t))


def verify_simulations(doc, asm, defs, comps, corners, step=3):
    """Run each Simulation headless, step through its frames and compare every driven joint's
    angle with its formula at that frame's time."""
    res = {}
    sg = UtilsAssembly.getSimulationGroup(asm)
    rest = fk(defs, {})
    for sim in sg.Group:
        for k, c in comps.items():                   # start from the rest pose, as the saved document does
            c.Placement = rest[k]
        asm.solve()
        asm.generateSimulation(sim)
        n = asm.numberOfFrames()
        dt = sim.cTimeStepOutput.Value
        motions = {(m.Joint[0] if isinstance(m.Joint, tuple) else m.Joint).Name[5:]: m.Formula for m in sim.Group}
        worst_deg, worst_res = 0.0, 0.0
        for i in list(range(1, n, step)) + [n - 1]:
            asm.updateForFrame(i)
            t = (i - 1) * dt
            for jn, f in motions.items():
                ang, r = joint_state(defs, comps, jn, corners)
                worst_deg = max(worst_deg, abs(_wrap(ang - math.degrees(eval_formula(f, t)))))
                worst_res = max(worst_res, r)
        fit = json.loads(getattr(sim, "FitJSON", "{}") or "{}")
        res[sim.Label] = dict(frames=n, max_angle_err_deg=worst_deg, max_residual_mm=worst_res, fit_vs_timeline_deg=fit)
        print(f"verify simulation '{sim.Label}': {n} frames, joints vs formulas {worst_deg:.4f} deg, "
              f"off-axis {worst_res:.2e} mm; sine fit vs timeline {fit}", flush=True)
    return res


if __name__ == "__main__":
    if os.environ.get("MOTION_SIMS_ONLY") == "1":
        rebuild_simulations(os.path.join(OUT_DIR, DOC_FILE))
    if os.environ.get("MOTION_BUILD", "1") != "0":
        _path = build()[0]
    else:
        _path = os.path.join(OUT_DIR, DOC_FILE)
    if os.environ.get("MOTION_VERIFY", "1") != "0":
        _res = verify(_path)
        with open(os.path.join(OUT_DIR, "verify-joints.json"), "w") as _f:
            json.dump(_res, _f, indent=1)
        print("verify:", json.dumps(_res), flush=True)
