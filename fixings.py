"""Simple, recognisable fixing models (pure Part API) for the assembly animation.

Every model is built in its own local frame:
  +Z is the fixing's axis, pointing OUT of the face it seats on;
  z = 0 is the seating face (under a screw head, washer, nut, spacer, standoff or bearing).
So a screw's head sits at z in [0, k] and its shank runs down z in [-L, 0]; a nut, washer,
spacer or bearing occupies z in [0, thickness]. place() maps that frame onto a world point
and outward axis.

make(kind, size, length=None, ...) returns a Fixing (shape + Spec / Material / dimensions).
Dimensions are nominal ISO values (ISO 4762, 7380, 7045, 4032, 7089) to 0.1 mm.
"""

import math

import FreeCAD as App
import Part

V = App.Vector

# d: nominal diameter; socket cap (ISO 4762) dk, k, hex key s; button (ISO 7380) dk, k;
# pan (ISO 7045) dk, k; nut (ISO 4032) s, m; washer (ISO 7089) d1, d2, h; pitch
SIZES = {
    "M2": dict(d=2.0, pitch=0.4, cap_dk=3.8, cap_k=2.0, cap_s=1.5, btn_dk=3.5, btn_k=1.3, btn_s=1.3,
               pan_dk=4.0, pan_k=1.6, nut_s=4.0, nut_m=1.6, w_d1=2.2, w_d2=5.0, w_h=0.3),
    "M2.5": dict(d=2.5, pitch=0.45, cap_dk=4.5, cap_k=2.5, cap_s=2.0, btn_dk=4.7, btn_k=1.5, btn_s=1.5,
                 pan_dk=5.0, pan_k=1.75, nut_s=5.0, nut_m=2.0, w_d1=2.7, w_d2=6.0, w_h=0.5),
    "M3": dict(d=3.0, pitch=0.5, cap_dk=5.5, cap_k=3.0, cap_s=2.5, btn_dk=5.7, btn_k=1.65, btn_s=2.0,
               pan_dk=5.6, pan_k=2.1, nut_s=5.5, nut_m=2.4, w_d1=3.2, w_d2=7.0, w_h=0.5),
}
NYLOC_M = {"M2": 2.8, "M2.5": 3.1, "M3": 4.0}

KIND_TEXT = {"cap": "socket cap", "button": "button head", "pan": "pan head", "selftap": "pan head self-tap"}


class Fixing:
    """A fixing model: shape in its local frame plus the metadata the document stores."""

    def __init__(self, kind, size, shape, spec, material, **dims):
        self.kind, self.size, self.shape = kind, size, shape
        self.spec, self.material = spec, material
        self.dims = dims

    def __repr__(self):
        return f"<Fixing {self.spec} ({self.material})>"


# --- helpers ------------------------------------------------------------------------------------
def _revolve(profile):
    """Solid of revolution about Z from an (r, z) outline (closed automatically, r >= 0)."""
    pts = [V(r, 0, z) for r, z in profile]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    return face.revolve(V(0, 0, 0), V(0, 0, 1), 360)


def _hex_prism(af, z0, z1):
    r = af / math.sqrt(3)
    pts = [V(r * math.cos(math.radians(60 * i)), r * math.sin(math.radians(60 * i)), z0) for i in range(6)]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, 0, z1 - z0))


def _shank(d, length, pitch, thread):
    """Shank from z = 0 down to z = -length; cosmetic thread = revolved V grooves (not helical)."""
    r = d / 2
    if not thread or length < 2 * pitch:
        prof = [(0, 0.01), (r, 0.01), (r, -length + 0.3), (r - 0.3, -length), (0, -length)]
        return _revolve(prof)
    rm = r - 0.6 * pitch                         # groove root
    prof = [(0, 0.01), (r, 0.01)]
    z = -min(1.0, length / 4)                    # plain neck under the head
    prof.append((r, z))
    while z - pitch > -length + 0.3:
        prof += [(rm, z - pitch / 2), (r, z - pitch)]
        z -= pitch
    prof += [(r - 0.3, -length), (0, -length)]
    return _revolve(prof)


def place(shape, pos, axis, spin=0.0):
    """Placement taking the local frame (seat at origin, +Z outward) to pos with +Z along axis."""
    a = V(axis).normalize()
    rot = App.Rotation(V(0, 0, 1), a)
    if spin:
        rot = rot.multiply(App.Rotation(V(0, 0, 1), spin))
    return App.Placement(V(pos), rot)


def placed_shape(fix, pos, axis, spin=0.0):
    s = fix.shape.copy()
    s.Placement = place(s, pos, axis, spin)
    return s


# --- screws -------------------------------------------------------------------------------------
def screw(head, size, length, material="A2", thread=False, note=""):
    """head: 'cap' (ISO 4762), 'button' (ISO 7380), 'pan' / 'selftap' (ISO 7045, cross recess)."""
    t = SIZES[size]
    d = t["d"]
    if head == "cap":
        dk, k = t["cap_dk"], t["cap_k"]
        h = _revolve([(0, 0), (dk / 2, 0), (dk / 2, k - 0.2), (dk / 2 - 0.2, k), (0, k)])
        h = h.cut(_hex_prism(t["cap_s"], k - 0.6 * k, k + 1))
    elif head == "button":
        dk, k = t["btn_dk"], t["btn_k"]
        prof = [(0, 0), (dk / 2, 0), (dk / 2, 0.25)]
        for i in range(1, 7):                                   # domed top
            a = math.radians(90 * i / 6)
            prof.append((dk / 2 * math.cos(a), 0.25 + (k - 0.25) * math.sin(a)))
        h = _revolve(prof[:-1] + [(0, k)])
        h = h.cut(_hex_prism(t["btn_s"], k * 0.35, k + 1))
    elif head in ("pan", "selftap"):
        dk, k = t["pan_dk"], t["pan_k"]
        prof = [(0, 0), (dk / 2, 0), (dk / 2, k * 0.6), (dk / 2 * 0.8, k), (0, k)]
        h = _revolve(prof)
        w, ln = d * 0.25, dk * 0.6                             # cross recess
        cross = Part.makeBox(ln, w, k, V(-ln / 2, -w / 2, k * 0.45))
        cross = cross.fuse(Part.makeBox(w, ln, k, V(-w / 2, -ln / 2, k * 0.45)))
        h = h.cut(cross)
    else:
        raise ValueError(head)
    shank = _shank(d, length, t["pitch"], thread)
    if head == "selftap":                                      # pointed tip
        tip = _revolve([(0, -length), (d / 2, -length + d * 0.6), (0, -length + d * 0.6)])
        shank = shank.cut(Part.makeCylinder(d, d * 0.6, V(0, 0, -length - 0.01))).fuse(tip)
    shape = h.fuse(shank).removeSplitter()
    spec = f"{size}x{length:g} {KIND_TEXT[head]}" + (f", {material}" if material else "") + (f" ({note})" if note else "")
    return Fixing("screw", size, shape, spec, material, d=d, length=length, head=head, head_d=dk, head_h=k)


# --- nuts, washers, spacers, standoffs -------------------------------------------------------------
def hex_nut(size, material="A2", nyloc=False):
    t = SIZES[size]
    m = NYLOC_M[size] if nyloc else t["nut_m"]
    body = _hex_prism(t["nut_s"], 0, t["nut_m"])
    body = body.common(_revolve([(0, -0.01), (t["nut_s"] * 0.58, -0.01), (t["nut_s"] * 0.58, t["nut_m"] - 0.3),
                                 (t["nut_s"] * 0.5, t["nut_m"] + 0.01), (0, t["nut_m"] + 0.01)]))   # chamfer
    if nyloc:
        body = body.fuse(Part.makeCylinder(t["nut_s"] * 0.46, m - t["nut_m"] + 0.01, V(0, 0, t["nut_m"] - 0.01)))
    body = body.cut(Part.makeCylinder(t["d"] * 0.43, m + 2, V(0, 0, -1))).removeSplitter()
    kind = "nyloc nut" if nyloc else "hex nut"
    return Fixing("nut", size, body, f"{size} {kind}, {material}", material, d=t["d"], height=m, af=t["nut_s"])


def washer(size, material="A2"):
    t = SIZES[size]
    shape = Part.makeCylinder(t["w_d2"] / 2, t["w_h"]).cut(Part.makeCylinder(t["w_d1"] / 2, t["w_h"] + 2, V(0, 0, -1)))
    return Fixing("washer", size, shape, f"{size} flat washer, {material}", material,
                  d=t["d"], height=t["w_h"], od=t["w_d2"], id=t["w_d1"])


def spacer(size, length, od=5.0, material="nylon"):
    t = SIZES[size]
    shape = Part.makeCylinder(od / 2, length).cut(Part.makeCylinder(t["d"] / 2 + 0.1, length + 2, V(0, 0, -1)))
    return Fixing("spacer", size, shape, f"{size}x{length:g} round spacer, {material}", material,
                  d=t["d"], height=length, od=od)


def standoff(size, length, male=0.0, af=5.0, material="nylon"):
    """Hex standoff. male = 0: F-F (tapped both ends). male > 0: M-F, the male stud runs down
    from the seat face (z in [-male, 0]) and the female end is on top."""
    t = SIZES[size]
    body = _hex_prism(af, 0, length)
    tap = t["d"] * 0.8 / 2
    depth = min(length / 2 - 0.5, 2.5 * t["d"])
    body = body.cut(Part.makeCylinder(tap, depth + 0.01, V(0, 0, length - depth)))
    if male:
        body = body.fuse(_shank(t["d"], male, t["pitch"], False))
        kind = f"{size}x{length:g}+{male:g} M-F hex standoff"
    else:
        body = body.cut(Part.makeCylinder(tap, depth + 0.01, V(0, 0, -0.01)))
        kind = f"{size}x{length:g} F-F hex standoff"
    return Fixing("standoff", size, body.removeSplitter(), f"{kind}, {material}", material,
                  d=t["d"], height=length, male=male, af=af)


# --- bearings and pins ---------------------------------------------------------------------------
def bearing_623zz():
    """623ZZ, 3 x 10 x 4: rings with recessed shields."""
    d, D, B = 3.0, 10.0, 4.0
    outer = _revolve([(3.9, 0), (D / 2, 0), (D / 2, B), (3.9, B)])
    inner = _revolve([(d / 2, 0), (2.6, 0), (2.6, B), (d / 2, B)])
    shield = _revolve([(2.6, 0.25), (3.9, 0.25), (3.9, B - 0.25), (2.6, B - 0.25)])
    shape = outer.fuse([inner, shield]).removeSplitter()
    return Fixing("bearing", "623", shape, "623ZZ bearing 3x10x4", "steel", d=d, od=D, height=B)


FLARE_H = 0.6      # a melted / flared filament end: a 2.4 mm button this thick


def filament_pin(length=12.0, d=1.75):
    """1.75 mm filament pin, ends melted into small buttons (FLARE_H thick, inside `length`).
    Centred: z in [-length/2, length/2]."""
    body = Part.makeCylinder(d / 2, length, V(0, 0, -length / 2))
    for s in (-1, 1):
        z0 = length / 2 - FLARE_H if s > 0 else -length / 2
        body = body.fuse(Part.makeCylinder(d * 0.7, FLARE_H, V(0, 0, z0)))
    return Fixing("pin", "1.75", body.removeSplitter(), f"1.75 mm filament pin, {length:.1f} mm, ends flared",
                  "PETG/PLA", d=d, length=length)


def make(kind, size=None, length=None, **kw):
    """Factory: make('cap', 'M3', 50), make('nut', 'M3'), make('washer', 'M3'), make('spacer', 'M2.5', 3),
    make('standoff', 'M2.5', 10), make('bearing'), make('pin', length=12)."""
    if kind in ("cap", "button", "pan", "selftap"):
        return screw(kind, size, length, **kw)
    if kind == "nut":
        return hex_nut(size, **kw)
    if kind == "washer":
        return washer(size, **kw)
    if kind == "spacer":
        return spacer(size, length, **kw)
    if kind == "standoff":
        return standoff(size, length, **kw)
    if kind == "bearing":
        return bearing_623zz()
    if kind == "pin":
        return filament_pin(length or 12.0, **kw)
    raise ValueError(kind)


if __name__ == "__main__":
    for args in (("cap", "M3", 50), ("button", "M3", 12), ("pan", "M2.5", 10), ("selftap", "M2", 8),
                 ("cap", "M2.5", 6, dict(thread=True)), ("nut", "M3"), ("nut", "M3", None, dict(nyloc=True)),
                 ("washer", "M3"), ("spacer", "M2.5", 3), ("standoff", "M2.5", 10),
                 ("standoff", "M2.5", 3, dict(male=6)), ("bearing",), ("pin", None, 12)):
        kw = args[3] if len(args) > 3 else {}
        f = make(*args[:3], **kw)
        s = f.shape
        print(f"{f.spec:45s} valid {s.isValid()} solids {len(s.Solids)} vol {s.Volume:8.2f} "
              f"z [{s.BoundBox.ZMin:.2f}, {s.BoundBox.ZMax:.2f}]")
