"""Striker figures from the assembly FCStd: docs/striker-section.png (section at x = 0: rest,
parked, as printed, the tilting parts and the sweep) and docs/striker-mg995-changes.png (two
shaded views: magenta = changed since OLD_REV, light blue = same part moved, grey = untouched).

    freecadcmd -c "exec(open('render_striker_docs.py').read(), {'__file__': 'render_striker_docs.py', '__name__': '__main__'})"
"""

import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.path import Path  # noqa: E402
from matplotlib.patches import PathPatch, Rectangle  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

import FreeCAD as App  # noqa: E402

V = App.Vector
OLD_REV = "edfadd7"
FCSTD = "imu-gimbal-assembly.FCStd"
MAGENTA, BLUE, GREY = np.array([0.85, 0.15, 0.75]), np.array([0.55, 0.78, 0.95]), np.array([0.78, 0.78, 0.78])
TILTING = ("StackMiddleDeck", "StackTopDeck", "StackBottomDeck", "StackSpineUpper", "StackSpineLower", "TiltRing")


def shapes(path):
    doc = App.openDocument(path)
    out = {o.Name: o.Shape.copy() for o in doc.Objects if hasattr(o, "Shape") and not o.Shape.isNull()}
    App.closeDocument(doc.Name)
    return out


def old_shapes(rel=FCSTD):
    blob = subprocess.run(["git", "show", f"{OLD_REV}:{rel}"], cwd=HERE, capture_output=True, check=True).stdout
    fd, path = tempfile.mkstemp(suffix=".FCStd")
    with os.fdopen(fd, "wb") as f:
        f.write(blob)
    try:
        return shapes(path)
    finally:
        os.remove(path)


# --- section ------------------------------------------------------------------------------------
def section_path(shape, x=0.0):
    verts, codes = [], []
    for w in shape.slice(V(1, 0, 0), x):
        pts = [(p.y, p.z) for p in w.discretize(Distance=0.3)]
        if len(pts) < 3:
            continue
        verts += pts + [pts[0]]
        codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
    return Path(verts, codes) if verts else None


def draw_section(ax, shp, face, edge, ls="-", lw=0.9, alpha=1.0, x=0.0, z=1):
    p = section_path(shp, x)
    if p is not None:
        ax.add_patch(PathPatch(p, facecolor=face, edgecolor=edge, lw=lw, ls=ls, alpha=alpha, zorder=z))


def section_figure(sh, info, path, title):
    fig, ax = plt.subplots(figsize=(11, 8.6), dpi=110)
    for n in TILTING:
        if n in sh:
            draw_section(ax, sh[n], "#d6e6f8", "#2a78d6", lw=0.7, z=1)
    for n in ("PanYoke", "TiltServo", "TiltBearing"):
        if n in sh:
            draw_section(ax, sh[n], "#e6e6e6", "#999999", lw=0.6, z=1)
    draw_section(ax, sh["StrikerStand"], "#d9d9de", "#333333", lw=0.8, z=2)
    draw_section(ax, sh["StrikerStand"], "none", "#777777", ls=":", lw=0.8, x=22.0, z=2)
    for n in ("StrikerArmParked", "StrikerSpringParked", "StrikerPawlParked"):
        draw_section(ax, sh[n], "#f6c9cc", "#d03030", alpha=0.75, z=3)
    for n in ("StrikerArm", "StrikerSpring", "StrikerPawl", "StrikerCam"):
        draw_section(ax, sh[n], "#bdbdbd", "#222222", z=4)
    for n in ("StrikerArmFree", "StrikerSpringFree"):
        draw_section(ax, sh[n], "none", "#1a9a2a", ls="--", lw=0.9, z=5)
    sb = sh["StrikerServo"].BoundBox
    ax.add_patch(Rectangle((sb.YMin, sb.ZMin), sb.YLength, sb.ZLength, fill=False, ec=MAGENTA, ls="-.", lw=1.0, zorder=6))
    ax.annotate(f"MG995 on the +X wall\n(outline, x {sb.XMin:.0f}..{sb.XMax:.0f})", (sb.YMin, sb.ZMax),
                xytext=(-140, 135), color=tuple(MAGENTA), fontsize=8.5, zorder=7,
                arrowprops=dict(arrowstyle="->", color=tuple(MAGENTA), lw=0.8))
    r = info.get("tilt_sweep_r", 83.3)
    t = np.linspace(0, 2 * np.pi, 400)
    ax.plot(r * np.cos(t), r * np.sin(t), color="#2a78d6", lw=1.2, ls="--", zorder=1)
    ax.plot((r + 5) * np.cos(t), (r + 5) * np.sin(t), color="#2a78d6", lw=0.8, ls=":", zorder=1)
    s = info["striker"]
    P, K = s["pivot_yz"], s["cam_yz"]
    tt = s.get("tail_tilt_deg", 28.0)
    notes = [((P[0], P[1]), "pivot (M3 on a printed bushing)", (-140, 152)),
             ((K[0], K[1]), f"cam: 2 lobes, r {s.get('cam_rmax', 0) - s.get('cam_lift', 0):.0f} -> {s.get('cam_rmax', 0):.1f}", (-140, -40)),
             ((P[0] - 12 * np.sin(np.radians(tt)), P[1] - 12 * np.cos(np.radians(tt))),
              f"tail leans back {tt:.0f} deg at rest", (-140, 120))]
    for xy, txt, xyt in notes:
        ax.annotate(txt, xy, xytext=xyt, fontsize=8.5, zorder=7, arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.plot([], [], color="#888888", lw=6, label="at rest (tilt 0)")
    ax.plot([], [], color="#f0a0a5", lw=6, label="parked")
    ax.plot([], [], color="#1a9a2a", ls="--", label="as printed")
    ax.plot([], [], color="#a8c8ee", lw=6, label="tilting parts (tilt 0)")
    ax.plot([], [], color="#777777", ls=":", label="tower wall (x = 22)")
    ax.legend(loc="upper right", fontsize=8.5)
    cl = s.get("clearance", {})
    txt = (f"head {s['hover']:g} mm over the roof at the arm stop; let-off overshoot {s['let_off_overshoot_mm']} mm; "
           f"tap {s['tap_energy_mJ']} mJ of {s['drop_energy_total_mJ']} mJ ({s.get('drop_friction_mJ', 0)} mJ lost to pin/slot friction on the way)\n"
           f"parked {s['park_deg']:g} deg: min distance through 360 deg tilt {cl.get('tilt360_16.8', '?')} mm, roll +/-90 "
           f"{cl.get('roll90_parked', '?')} mm, tilt x roll {cl.get('tilt_roll_16.8', '?')} mm; at rest {cl.get('rest', '?')} mm\n"
           f"drive leaf 20 x {s['leaf_t']} x 68 mm; cam lift {s['cam_lift']} mm; peak servo torque {s.get('cam_peak_kgcm', '?')} kg cm "
           f"with mu {s.get('mu_cam', 0.4)} (MG995 stall 8.5)")
    ax.text(0.01, 0.01, txt, transform=ax.transAxes, fontsize=8, va="bottom",
            bbox=dict(facecolor="white", edgecolor="#bbbbbb"), zorder=8)
    ax.set_xlim(-145, 100)
    ax.set_ylim(-95, 165)
    ax.set_aspect("equal")
    ax.grid(color="#eeeeee", lw=0.6)
    ax.set_xlabel("y (mm, +Y toward the tilt servo)")
    ax.set_ylabel("z (mm above the tilt axis)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# --- what changed --------------------------------------------------------------------------------
def classify(new, old, tol=0.05):
    """name -> 'changed' | 'moved' | 'same' for every object in the new document."""
    out = {}
    for n, s in new.items():
        o = old.get(n)
        if o is None:
            out[n] = "changed"
            continue
        a, b = s.BoundBox, o.BoundBox
        same_shape = abs(s.Volume - o.Volume) <= max(0.5, 0.002 * o.Volume) and \
            all(abs(x - y) < tol for x, y in ((a.XLength, b.XLength), (a.YLength, b.YLength), (a.ZLength, b.ZLength)))
        if not same_shape:
            out[n] = "changed"
        elif max(abs(a.XMin - b.XMin), abs(a.YMin - b.YMin), abs(a.ZMin - b.ZMin)) > tol:
            out[n] = "moved"
        else:
            out[n] = "same"
    return out


def tris(shape, tol=0.25):
    v, f = shape.tessellate(tol)
    if not f:
        return np.zeros((0, 3, 3))
    v = np.array([[p.x, p.y, p.z] for p in v])
    return v[np.array(f)]


def shade(t, light):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1)[:, None], 1e-12)
    return 0.45 + 0.55 * np.abs(n @ light)


SKIP = ("StrikerArmParked", "StrikerSpringParked", "StrikerPawlParked", "StrikerArmFree", "StrikerSpringFree")


def changes_figure(new, old, path, label):
    cls = classify(new, old)
    colour = {"changed": MAGENTA, "moved": BLUE, "same": GREY}
    views = [((24, -58), "striker, from the +X / back side (the MG995 on the +X wall)", 0),
             ((14, -128), "from the -X side (the -X wall cut away at x = -8)", -8.0)]
    fig = plt.figure(figsize=(14, 7.2), dpi=100)
    light = np.array([0.4, -0.5, 0.75]) / np.linalg.norm([0.4, -0.5, 0.75])
    for i, (view, title, cut) in enumerate(views):
        ax = fig.add_axes([0.5 * i, 0.06, 0.5, 0.86], projection="3d")
        allt, allc = [], []
        import Part
        clip = Part.makeBox(200, 110, 200, V(-100, -170, -30))      # the striker end of the yoke
        for n, s in new.items():
            if n in SKIP or not (n.startswith("Striker") or n in ("PanYoke", "TiltBearing")):
                continue
            if not n.startswith("Striker"):
                s = s.common(clip)
            if cut and n == "StrikerStand":
                s = s.common(Part.makeBox(200, 400, 400, V(cut, -300, -100)))
            t = tris(s)
            if not len(t):
                continue
            sh = shade(t, light)
            allt.append(t)
            allc.append(np.concatenate([np.clip(colour[cls[n]][None, :] * sh[:, None], 0, 1), np.ones((len(t), 1))], axis=1))
        T, C = np.concatenate(allt), np.concatenate(allc)
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors="none", linewidths=0))
        ctr, h = np.array([0.0, -95.0, 45.0]), 72.0
        ax.set_xlim(ctr[0] - h, ctr[0] + h)
        ax.set_ylim(ctr[1] - h, ctr[1] + h)
        ax.set_zlim(ctr[2] - h, ctr[2] + h)
        ax.set_box_aspect((1, 1, 1), zoom=1.25)
        ax.view_init(elev=view[0], azim=view[1])
        ax.set_axis_off()
        ax.set_title(title, fontsize=10)
    groups = {k: sorted(n for n, v in cls.items() if v == k and n.startswith("Striker") and n not in SKIP)
              for k in ("changed", "moved", "same")}
    txt = (f"magenta = changed since {OLD_REV}: {', '.join(groups['changed']) or '-'}\n"
           f"light blue = same part, moved: {', '.join(groups['moved']) or '-'}    "
           f"grey = untouched (striker: {', '.join(groups['same']) or '-'})")
    fig.text(0.01, 0.01, txt, fontsize=8.5, va="bottom")
    fig.suptitle(f"Striker changes for the MG995 cam servo ({label})", fontsize=12)
    fig.savefig(path)
    plt.close(fig)
    return cls


def main():
    out = os.environ.get("GIMBAL_OUT", HERE)
    label = "tall_top" if os.path.abspath(out) != os.path.abspath(HERE) else "main design"
    new = shapes(os.path.join(out, FCSTD))
    info = json.load(open(os.path.join(out, "build_info.json")))
    section_figure(new, info, os.path.join(out, "docs", "striker-section.png"),
                   f"Striker section at x = 0 ({label}, MG995 cam servo)")
    rel = os.path.relpath(os.path.join(out, FCSTD), HERE).replace(os.sep, "/")
    cls = changes_figure(new, old_shapes(rel), os.path.join(out, "docs", "striker-mg995-changes.png"), label)
    print({k: v for k, v in cls.items() if not k.startswith("REF_")})


if __name__ == "__main__":
    main()
