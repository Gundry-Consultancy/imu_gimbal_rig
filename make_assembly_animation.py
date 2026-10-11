"""Animated, step-by-step assembly document for the gimbal rig, with every fixing.

    freecadcmd -c "exec(open('make_assembly_animation.py').read(), {'__file__': 'make_assembly_animation.py', '__name__': '__main__'})"

Options (environment, since freecadcmd -c takes no argv):
    ASSY_VARIANT=tall_top   the pinned variant (upper spine 25 mm, taller tower, M3 x 55 stack bolts)
    ASSY_BOARDS=all         fill all 21 QT spots (default: the 9 PR 839 boards + the wing, assembly_plan.POPULATED)
    ASSY_NO_BENCH=1         leave out the optional bench screws
    ASSY_CACHE=<file>       reuse / write the generator geometry (BREP) to skip the ~2 min rebuild
    ASSY_RENDER=0           skip the GIF / step PNGs
    ASSY_WRITE=0            checks only: write no document (e.g. with ASSY_BOARDS=all)
    ASSY_ACCESS=0           skip the (slow) access check
    ASSY_CHECKS=0           skip all the checks

It runs the generators' geometry (make_gimbal.geometry(): the same parts as the main build,
plus the fixing points the generators record where they cut each hole), compares it with the
built imu-gimbal-assembly.FCStd, places every fixing (assembly_plan.py), runs the fixing checks
(assembly_checks.py), and writes docs/assembly/imu-gimbal-assembly-animated.FCStd (or the
variant's docs/assembly/), then renders the preview GIF and one PNG per step.
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
VARIANT = os.environ.get("ASSY_VARIANT", "")
if VARIANT == "tall_top":                     # the variant's generator settings (see variants/tall_top/README.md)
    os.environ.setdefault("STACK_UPPER_GAP", "25")
    os.environ.setdefault("GIMBAL_PIN", os.path.join(HERE, "build_info.json"))
    os.environ.setdefault("GIMBAL_OUT", os.path.join(HERE, "variants", "tall_top"))

import FreeCAD as App  # noqa: E402
import Part  # noqa: E402

V = App.Vector
OUT_ROOT = os.path.join(HERE, "variants", VARIANT) if VARIANT else HERE
OUT_DIR = os.path.join(OUT_ROOT, "docs", "assembly")
DOC_FILE = "imu-gimbal-assembly-animated.FCStd"


# --- geometry cache (dev convenience: Part shapes do not pickle) -------------------------------------
def _enc(o):
    if isinstance(o, Part.Shape):
        return {"__brep__": o.exportBrepToString()}
    if isinstance(o, App.Vector):
        return {"__vec__": [o.x, o.y, o.z]}
    if isinstance(o, dict):
        return {"__dict__": [[_enc(k), _enc(v)] for k, v in o.items()]}
    if isinstance(o, (list, tuple)):
        return {"__list__" if isinstance(o, list) else "__tuple__": [_enc(v) for v in o]}
    return o


def _dec(o):
    if isinstance(o, dict):
        if "__brep__" in o:
            s = Part.Shape()
            s.importBrepFromString(o["__brep__"])
            return s
        if "__vec__" in o:
            return V(*o["__vec__"])
        if "__dict__" in o:
            return {_dec(k): _dec(v) for k, v in o["__dict__"]}
        if "__list__" in o:
            return [_dec(v) for v in o["__list__"]]
        if "__tuple__" in o:
            return tuple(_dec(v) for v in o["__tuple__"])
    return o


def geometry():
    cache = os.environ.get("ASSY_CACHE")
    if cache and os.path.exists(cache):
        print("geometry from cache", cache, flush=True)
        g = _dec(json.load(open(cache)))
        import make_gimbal as mg          # the plan reads constants from the generators
        mg.PLATES.update(g["plates"])
        mg.HORNS.update(g["horns"])
        return g
    import make_gimbal as mg
    g = mg.geometry()
    g = {k: v for k, v in g.items() if k != "st"} | {"st": {k: v for k, v in g["st"].items()}}
    if cache:
        json.dump(_enc(g), open(cache, "w"))
        print("wrote geometry cache", cache, flush=True)
    return g


def compare_with_build(g):
    """The plan uses freshly generated geometry; warn if the built FCStd has drifted from it."""
    path = os.path.join(os.environ.get("GIMBAL_OUT", HERE), "imu-gimbal-assembly.FCStd")
    if not os.path.exists(path):
        print("no built assembly to compare with:", path)
        return
    names = {"stack_middle_deck": "StackMiddleDeck", "stack_top_deck": "StackTopDeck", "stack_bottom_deck": "StackBottomDeck",
             "stack_spine_upper": "StackSpineUpper", "stack_spine_lower": "StackSpineLower", "pan_yoke": "PanYoke",
             "base": "Base", "striker_stand": "StrikerStand", "tilt_ring_mg90s": "TiltRing", "striker_cam": "StrikerCam"}
    doc = App.openDocument(path, True)
    bad = []
    for n, s in dict(g["printed"]).items():
        if n in names and doc.getObject(names[n]):
            v0 = doc.getObject(names[n]).Shape.Volume
            if abs(v0 - s.Volume) > 1e-4 * v0:
                bad.append((n, round(v0, 1), round(s.Volume, 1)))
    App.closeDocument(doc.Name)
    print("built assembly matches the generators" if not bad else f"WARNING: built assembly differs: {bad}", flush=True)


# --- document -------------------------------------------------------------------------------------------
def _txt(s):
    """Spreadsheet text cell (a leading quote stops '=' / numbers being parsed)."""
    return "'" + str(s).replace("\n", " ")


def _insertion(it):
    """(unit vector, distance) of an item's own insertion: the direction it travels, and how far."""
    path = it.moves[0]["path"]
    if not path:
        return V(0, 0, 0), 0.0
    pts = list(path) + [V(0, 0, 0)]
    dist = sum((b - a).Length for a, b in zip(pts, pts[1:]))
    last = pts[-1] - pts[-2]
    return (V(last).normalize() if last.Length > 1e-9 else V(0, 0, 0)), dist


def write_document(plan, path, variant):
    import assembly_checks as ac
    doc = App.newDocument("imu_gimbal_assembly_animated")
    root = doc.addObject("App::DocumentObjectGroup", "AssemblySteps")
    root.Label = "Assembly steps" + (f" ({variant})" if variant else "")
    root.addProperty("App::PropertyString", "StepsJSON", "Animation", "step order, titles and phases (JSON)")
    root.addProperty("App::PropertyString", "Variant", "Animation")
    root.StepsJSON = json.dumps(ac.steps_json(plan)["list"])
    root.Variant = variant or "main"
    groups = {}
    for s in plan["steps"]:
        grp = doc.addObject("App::DocumentObjectGroup", f"Step{s['n']:02d}")
        grp.Label = s["group"]
        root.addObject(grp)
        groups[s["key"]] = grp
    nums = {s["key"]: s for s in plan["steps"]}
    for it in plan["items"]:
        o = doc.addObject("Part::Feature", it.name)
        o.Shape = it.shape
        o.Placement = it.placement.multiply(it.shape.Placement)     # the shape may carry its own placement
        o.Label = it.label
        st = nums[it.step]
        vec, dist = _insertion(it)
        for prop, typ, val in (("Spec", "String", it.spec or it.label), ("Material", "String", it.material),
                               ("Qty", "Integer", 1), ("Step", "Integer", st["n"]), ("StepTitle", "String", st["title"]),
                               ("InsertVector", "Vector", vec), ("InsertDistance", "Distance", dist),
                               ("AnimMoves", "String", json.dumps(ac.moves_json(it))),
                               ("IntendedColor", "Color", tuple(it.color)), ("Optional", "Bool", it.optional),
                               ("Note", "String", it.note)):
            o.addProperty(f"App::Property{typ}", prop, "Assembly")
            setattr(o, prop, val)
        o.Visibility = True
        if App.GuiUp:
            o.ViewObject.ShapeColor = tuple(it.color)
        groups[it.step].addObject(o)
    _sheets(doc, plan)
    doc.recompute()
    doc.saveAs(path)
    print("wrote", path, flush=True)
    return doc


def _sheets(doc, plan):
    sh = doc.addObject("Spreadsheet::Sheet", "Steps")
    heads = ("Step", "Title", "Parts", "Fixings (spec x qty)", "Tool", "Insertion (direction, distance)", "Notes")
    for c, h in zip("ABCDEFG", heads):
        sh.set(f"{c}1", _txt(h))
    for r, s in enumerate(plan["steps"], 2):
        own = [it for it in plan["items"] if it.step == s["key"]]
        ins = {}
        for it in own:
            v, d = _insertion(it)
            key = (it.spec or it.label, f"({v.x:+.2f}, {v.y:+.2f}, {v.z:+.2f}) x {d:.0f} mm")
            ins[key] = ins.get(key, 0) + 1
        sh.set(f"A{r}", str(s["n"]))
        sh.set(f"B{r}", _txt(s["title"]))
        sh.set(f"C{r}", _txt("; ".join(s["parts"])))
        sh.set(f"D{r}", _txt("; ".join(f"{k} x {n}" for k, n in s["fixings"].items())))
        sh.set(f"E{r}", _txt(", ".join(s["tools"])))
        sh.set(f"F{r}", _txt("; ".join(f"{n} x {k[0]}: {k[1]}" for k, n in ins.items())))
        sh.set(f"G{r}", _txt(s["notes"]))
    for c, w in zip("ABCDEFG", (40, 220, 300, 420, 200, 520, 600)):
        sh.setColumnWidth(c, w)
    ins = doc.addObject("Spreadsheet::Sheet", "Insertions")
    for c, h in zip("ABCDEFGH", ("Step", "Object", "Label", "Spec", "Material", "Insert vector", "Distance", "Path")):
        ins.set(f"{c}1", _txt(h))
    r = 2
    for s in plan["steps"]:
        for it in [i for i in plan["items"] if i.step == s["key"]]:
            v, d = _insertion(it)
            path = " -> ".join(f"({p.x:.0f},{p.y:.0f},{p.z:.0f})" for p in it.moves[0]["path"]) + " -> 0"
            for c, val in zip("ABCDEFGH", (str(s["n"]), _txt(it.name), _txt(it.label), _txt(it.spec), _txt(it.material),
                                            _txt(f"({v.x:+.3f}, {v.y:+.3f}, {v.z:+.3f})"), f"{d:.1f}", _txt(path))):
                ins.set(f"{c}{r}", val)
            r += 1


def main():
    import assembly_plan as ap
    import assembly_checks as ac
    g = geometry()
    compare_with_build(g)
    label = {"tall_top": " (tall_top: 25 mm)"}.get(VARIANT, "")
    boards = "all" if os.environ.get("ASSY_BOARDS") == "all" else ap.POPULATED
    plan = ap.build_plan(g, label, boards=boards, bench=not os.environ.get("ASSY_NO_BENCH"))
    print(f"\n{len(plan['steps'])} steps, {len(plan['items'])} objects "
          f"({sum(1 for i in plan['items'] if i.fix)} fixings), boards: {', '.join(plan['populated'])}")
    for s in plan["steps"]:
        print(f"  {s['group']}: {'; '.join(f'{n} x {k}' for k, n in s['fixings'].items()) or '(parts only)'}")
    res = None
    if os.environ.get("ASSY_CHECKS", "1") != "0":
        res = ac.run_all(plan, VARIANT, do_access=os.environ.get("ASSY_ACCESS", "1") != "0")
    if os.environ.get("ASSY_WRITE", "1") == "0":
        return plan, res, None
    os.makedirs(OUT_DIR, exist_ok=True)
    doc = write_document(plan, os.path.join(OUT_DIR, DOC_FILE), VARIANT)
    if os.environ.get("ASSY_RENDER", "1") != "0":
        import render_assembly_animation as ra
        ra.render(plan, OUT_DIR)
    return plan, res, doc


if __name__ == "__main__":
    main()
