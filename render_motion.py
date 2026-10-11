"""Headless render of the motion programme: FreeCAD tessellation -> matplotlib Poly3DCollection
frames -> docs/motion/motion.gif (and motion.mp4 when ffmpeg is on the PATH), plus
docs/motion/striker-cycle.png (cam angle, cam radius, arm angle, head height, leaf force).

    freecadcmd -c "exec(open('render_motion.py').read(), {'__file__': 'render_motion.py', '__name__': '__main__'})"

The poses come from the macro's own Timeline and Rig (docs/motion/animate_motion.FCMacro),
so the video shows exactly what the macro plays. Left: the whole rig. Right: the striker
close-up in the yoke's frame, cut away at x = 0 (the tower's near wall removed), seen from +X.
Options (environment): MOTION_FPS (default 10); MOTION_PLOT_ONLY=1 (just the plot);
MOTION_FRAMES=a:b[:step] with MOTION_PREVIEW_DIR=<dir> renders a frame range to PNGs only (run a few
in parallel for the full set), then MOTION_ASSEMBLE=<dir> writes the GIF / MP4 from them.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from PIL import Image  # noqa: E402

import FreeCAD as App  # noqa: E402

OUT = os.path.join(HERE, "docs", "motion")
LIGHT = np.array([0.35, -0.55, 0.75]) / np.linalg.norm([0.35, -0.55, 0.75])
LIGHT_SIDE = np.array([0.8, -0.3, 0.5]) / np.linalg.norm([0.8, -0.3, 0.5])
VIEW = (22, -62)
FPS = float(os.environ.get("MOTION_FPS", "10"))
SIZE = (12.8, 6.4)
DPI = 75
CLOSE = dict(y=(-128.0, 38.0), z=(-28.0, 132.0))      # striker close-up window (yoke frame)
CAM_WIN = dict(y=(-124.0, -84.0), z=(12.0, 52.0))      # cam / pawl inset window (yoke frame)


def _cam_keep(b, name, t):
    if b not in ("Cam", "Leaf", "Pawl", "Tower"):
        return np.zeros(len(t), dtype=bool)
    return _close_keep(b, name, t, CAM_WIN)


def mesh(shape, lin, ang=0.5):
    import MeshPart
    m = MeshPart.meshFromShape(Shape=shape, LinearDeflection=lin, AngularDeflection=ang, Relative=False)
    v, f = m.Topology
    if not f:
        return np.zeros((0, 3, 3))
    v = np.array([[p.x, p.y, p.z] for p in v])
    return v[np.array(f)]


def mat(pl):
    """4x4 numpy matrix of a FreeCAD Placement."""
    m = pl.toMatrix()
    return np.array([[m.A11, m.A12, m.A13, m.A14], [m.A21, m.A22, m.A23, m.A24], [m.A31, m.A32, m.A33, m.A34],
                     [0, 0, 0, 1.0]])


def xform(t, m):
    return t @ m[:3, :3].T + m[:3, 3]


def shade(t, light):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    return 0.40 + 0.60 * np.abs((n / ln) @ light)


class Scene:
    """Meshes of every object in its rest pose, by body; the leaf and arm in each flex state."""

    def __init__(self, player):
        self.p = player
        self.items = []                                # (body, name, tris, rgb)
        for b, c in player.comps.items():
            for o in c.Group:
                if not hasattr(o, "Shape") or o.Name in ("StrikerSpring", "StrikerArm"):
                    continue
                fine = b in ("Cam", "Arm", "Leaf", "Pawl") or "Striker" in o.Name
                lin = 0.25 if fine else (0.6 if o.Shape.Volume < 400 else 1.0)
                rgb = np.array(o.IntendedColor[:3])
                if b == "Pawl":                       # same red as the cam in the document: darken it here
                    rgb = rgb * 0.55
                self.items.append((b, o.Name, mesh(o.Shape, lin), rgb))
        doc = player.doc
        self.flex = {n: mesh(doc.getObject(n).Shape, 0.2) for _, n in player.rig.leaf_grid + player.rig.arm_grid}
        self.flex_col = {"leaf": np.array(player.leaf_obj.IntendedColor[:3]), "arm": np.array(player.arm_obj.IntendedColor[:3])}
        print("render: triangles", sum(len(t) for _, _, t, _ in self.items), "+ flex", flush=True)

    def world(self, s, frame=None):
        """[(body, name, world tris, rgb)] at timeline state s; frame: optional 4x4 applied after."""
        pose = self.p.rig.pose(s)
        out = []
        for b, name, t, rgb in self.items:
            m = mat(pose[b])
            if name == "PawlPin":
                m = m @ mat(pose["PawlPin"])
            out.append((b, name, xform(t, frame @ m if frame is not None else m), rgb))
        for key, b in (("leaf", "Leaf"), ("arm", "Arm")):
            m = mat(pose[b])
            out.append((b, key, xform(self.flex[pose[key]], frame @ m if frame is not None else m), self.flex_col[key]))
        return out


def _collect(parts, light, keep=None):
    tris, cols = [], []
    for b, name, t, rgb in parts:
        if not len(t):
            continue
        if keep is not None:
            sel = keep(b, name, t)
            if sel is not None:
                t = t[sel]
                if not len(t):
                    continue
        sh = shade(t, light)
        tris.append(t)
        cols.append(np.concatenate([np.clip(rgb[None, :] * sh[:, None], 0, 1), np.ones((len(t), 1))], axis=1))
    if not tris:
        return np.zeros((0, 3, 3)), np.zeros((0, 4))
    return np.concatenate(tris), np.concatenate(cols)


def _close_keep(b, name, t, win=CLOSE):
    """Close-up: the striker cut away at x = 0 (the tower's +X half removed), plus whatever
    of the gimbal comes into the window."""
    c = t.mean(axis=1)
    inside = (c[:, 1] > win["y"][0] - 5) & (c[:, 1] < win["y"][1] + 5) & (c[:, 2] > win["z"][0] - 5) & \
             (c[:, 2] < win["z"][1] + 5)
    if b == "Tower":
        return inside & (c[:, 0] < -0.5)
    if b in ("Ring", "Stack", "Yoke", "Base"):
        return inside & (c[:, 0] < 12.0)
    return inside


def draw(scene, s, captions, bounds):
    fig = plt.figure(figsize=SIZE, dpi=DPI)
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0.0, 0.0, 0.56, 0.86], projection="3d")
    tris, cols = _collect(scene.world(s), LIGHT)
    ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolors="none", linewidths=0))
    c, h = bounds
    for i, setl in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        setl(c[i] - h, c[i] + h)
    ax.set_box_aspect((1, 1, 1), zoom=1.3)
    ax.view_init(elev=VIEW[0], azim=VIEW[1])
    ax.set_axis_off()
    # close-up in the yoke's frame (it pans with the yoke), seen from +X, orthographic
    pose = scene.p.rig.pose(s)
    inv = np.linalg.inv(mat(pose["Yoke"]))
    ax2 = fig.add_axes([0.56, 0.02, 0.44, 0.84], projection="3d", proj_type="ortho")
    t2, c2 = _collect(scene.world(s, frame=inv), LIGHT_SIDE, keep=_close_keep)
    ax2.add_collection3d(Poly3DCollection(t2, facecolors=c2, edgecolors="none", linewidths=0))
    ym, zm = sum(CLOSE["y"]) / 2, sum(CLOSE["z"]) / 2
    hh = max(CLOSE["y"][1] - CLOSE["y"][0], CLOSE["z"][1] - CLOSE["z"][0]) / 2
    ax2.set_xlim(-hh, hh)
    ax2.set_ylim(ym - hh, ym + hh)
    ax2.set_zlim(zm - hh, zm + hh)
    ax2.set_box_aspect((1, 1, 1), zoom=1.38)
    ax2.view_init(elev=0, azim=0)
    ax2.set_axis_off()
    fig.text(0.78, 0.875, "Striker close-up (yoke frame, cut at x = 0, from +X)", ha="center", fontsize=9, color="#555555")
    box = (0.01, 0.03, 0.21, 0.40)                      # inset: cam, pawl and leaf (bottom left)
    bg = fig.add_axes(box)
    bg.set_xticks([])
    bg.set_yticks([])
    bg.set_facecolor("white")
    for sp in bg.spines.values():
        sp.set_color("#bbbbbb")
    bg.set_zorder(5)
    ax3 = fig.add_axes(box, projection="3d", proj_type="ortho")
    ax3.set_zorder(6)
    t3, c3 = _collect(scene.world(s, frame=inv), LIGHT_SIDE, keep=_cam_keep)
    ax3.add_collection3d(Poly3DCollection(t3, facecolors=c3, edgecolors="none", linewidths=0))
    yc, zc = sum(CAM_WIN["y"]) / 2, sum(CAM_WIN["z"]) / 2
    h3 = (CAM_WIN["y"][1] - CAM_WIN["y"][0]) / 2
    ax3.set_xlim(-h3, h3)
    ax3.set_ylim(yc - h3, yc + h3)
    ax3.set_zlim(zc - h3, zc + h3)
    ax3.set_box_aspect((1, 1, 1), zoom=0.95)
    ax3.view_init(elev=0, azim=0)
    ax3.set_axis_off()
    ax3.patch.set_alpha(0.0)
    bg.text(0.5, 0.93, "cam, pawl and leaf", ha="center", fontsize=8, color="#555555", transform=bg.transAxes)
    fig.text(0.5, 0.965, captions[0], ha="center", va="center", fontsize=13, weight="bold")
    fig.text(0.5, 0.925, captions[1], ha="center", va="center", fontsize=8.5, color="#333333", family="monospace")
    fig.canvas.draw()
    img = Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert("RGB")
    plt.close(fig)
    return img


def overall_bounds(scene, tl):
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for T in np.linspace(0, tl.duration, 40):
        for _, _, t, _ in scene.world(tl.state(T)):
            if len(t):
                p = t.reshape(-1, 3)
                lo, hi = np.minimum(lo, p.min(0)), np.maximum(hi, p.max(0))
    return (lo + hi) / 2, max(hi - lo) / 2 * 0.98


def frame_times(tl, fps):
    n = int(tl.duration * fps)
    return [min(tl.duration, k / fps) for k in range(n + 1)]


def write_video(frames, fps):
    """GIF (Pillow, adaptive palette) and, with ffmpeg on the PATH, an H.264 MP4."""
    gif = os.path.join(OUT, "motion.gif")
    pal = [f.quantize(colors=80, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
    pal[0].save(gif, save_all=True, append_images=pal[1:], duration=int(round(1000 / fps)), loop=0, optimize=True)
    res = dict(gif=gif, gif_mb=os.path.getsize(gif) / 1e6, frames=len(frames), seconds=len(frames) / fps)
    ff = shutil.which("ffmpeg")
    if ff:
        tmp = tempfile.mkdtemp(prefix="motion_frames_")
        for i, f in enumerate(frames):
            f.save(os.path.join(tmp, f"{i:05d}.png"))
        mp4 = os.path.join(OUT, "motion.mp4")
        cmd = [ff, "-y", "-loglevel", "error", "-framerate", f"{fps:g}", "-i", os.path.join(tmp, "%05d.png"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", mp4]
        subprocess.run(cmd, check=True)
        shutil.rmtree(tmp, ignore_errors=True)
        res.update(mp4=mp4, mp4_mb=os.path.getsize(mp4) / 1e6)
    else:
        res["mp4"] = "skipped: no ffmpeg on the PATH"
    return res


def render(doc_path=None):
    import check_motion as cm
    mac = cm.load_macro()
    doc, owned = mac.open_motion_doc(doc_path or os.path.join(OUT, mac.DOC_FILE))
    player = mac.Player(doc, owned)
    scene = Scene(player)
    tl = player.tl
    bounds = overall_bounds(scene, tl)
    times = frame_times(tl, FPS)
    idx = list(range(len(times)))
    rng = os.environ.get("MOTION_FRAMES")
    if rng:
        idx = idx[slice(*(int(x) for x in rng.split(":")))]
    prev = os.environ.get("MOTION_PREVIEW_DIR")
    frames = []
    for k, i in enumerate(idx):
        s = tl.state(times[i])
        img = draw(scene, s, tl.caption(s), bounds)
        if prev:                                   # PNGs only (a preview, or one chunk of a parallel render)
            os.makedirs(prev, exist_ok=True)
            img.save(os.path.join(prev, f"frame_{i:05d}.png"))
        else:
            frames.append(img)
        if k % 25 == 0:
            print(f"render: frame {k}/{len(idx)} T {times[i]:.1f}", flush=True)
    res = dict(preview=prev, frames=len(idx)) if prev else write_video(frames, FPS)
    print("render:", res, flush=True)
    App.closeDocument(doc.Name)
    return res


INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, SPAN = "#2a78d6", "#eb6834", "#f0efec"


def striker_plot(doc_path=None, path=None):
    """docs/motion/striker-cycle.png: the striker section of the programme against animation time
    (slow-motion releases shaded), and the first tap in physical milliseconds."""
    import check_motion as cm
    import striker_motion as sm
    mac = cm.load_macro()
    doc, owned = mac.open_motion_doc(doc_path or os.path.join(OUT, mac.DOC_FILE))
    data = json.loads(doc.getObject("MotionData").ModelJSON)
    tl = mac.Timeline(json.loads(doc.getObject("MotionData").TimelineJSON))
    App.closeDocument(doc.Name)
    sg = dict(data["striker_input"], stand_bolts=tuple(tuple(x) for x in data["striker_input"]["stand_bolts"]))
    m = sm.Striker(sg)
    segs = [g for g in tl.segments if g["section"] == "4 Striker"]
    t0, t1 = segs[0]["t0"], segs[-1]["t1"]
    rows = [r for r in tl.rows if t0 <= r[0] <= t1]
    T = np.array([r[0] - t0 for r in rows])
    th, phi, x = (np.array([r[k] for r in rows]) for k in (4, 5, 7))
    series = [("Cam servo angle (deg)", th, None),
              ("Cam radius under the pawl tip (mm)", np.array([m.contact_radius(a, b) for a, b in zip(th, phi)]), None),
              ("Hammer arm angle above rest (deg)", phi, None),
              ("Hammer face above the roof (mm)", np.array([m.head_height(b, c) for b, c in zip(phi, x)]), None),
              ("Drive leaf force (N)", np.array([m.leaf_force(b) for b in phi]),
               np.array([m.cam_force(b) if b > 1e-6 and m.phi_cam(a) > 1e-6 and abs(b - m.phi_cam(a)) < 0.05 else 0.0
                         for a, b in zip(th, phi)]))]          # only while the pawl is on the cam
    rel = [g for g in segs if g["kind"] == "release"][0]
    rr, ev = m.release(*rel["args"], m.phi_cam(rel["args"][0]))
    ms = np.array([r[0] * 1000 for r in rr])
    zt, zp, zx = (np.array([r[k] for r in rr]) for k in (1, 2, 3))
    zoom = [zt, np.array([m.contact_radius(a, b) for a, b in zip(zt, zp)]), zp,
            np.array([m.head_height(b, c) for b, c in zip(zp, zx)]), np.array([m.leaf_force(b) for b in zp])]
    fig, axes = plt.subplots(5, 2, figsize=(13, 11), dpi=110, sharex="col", gridspec_kw=dict(width_ratios=(2.6, 1)))
    fig.patch.set_facecolor("#fcfcfb")
    for i, (title, y, y2) in enumerate(series):
        ax, az = axes[i]
        for g in segs:
            if g["kind"] == "release":
                ax.axvspan(g["t0"] - t0, g["t1"] - t0, color=SPAN, zorder=0)
        ax.plot(T, y, color=BLUE, lw=2, label="at the pin" if y2 is not None else None)
        az.plot(ms, zoom[i], color=BLUE, lw=2)
        if y2 is not None:
            ax.plot(T, y2, color=ORANGE, lw=2, label="cam on the pawl")
            ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK2)
        for a in (ax, az):
            a.set_facecolor("#fcfcfb")
            a.grid(color=GRID, lw=0.8)
            a.tick_params(colors=INK2, labelsize=8)
            for sp in a.spines.values():
                sp.set_visible(False)
        ax.set_title(title, loc="left", fontsize=9.5, color=INK)
        az.set_title(title.split(" (")[0] + " - first tap" + (" (zoom)" if i == 3 else ""), loc="left",
                     fontsize=8.5, color=INK2)
    for e in (ev["t_stop_ms"], ev["t_roof_ms"], ev["t_back_ms"]):
        for az in axes[:, 1]:
            az.axvline(e, color=INK2, lw=0.8, ls=":")
    lab = ["dotted lines:", f"arm on its stop {ev['t_stop_ms']:.1f} ms", f"head on the roof {ev['t_roof_ms']:.1f} ms",
           f"head back on its screw {ev['t_back_ms']:.1f} ms"]
    axes[1, 1].text(0.3, 0.6, "\n".join(lab), transform=axes[1, 1].transAxes, fontsize=7.5,
                    color=INK2, va="center")
    for g in segs:
        if g["kind"] == "release":
            axes[0, 0].text((g["t0"] + g["t1"]) / 2 - t0, axes[0, 0].get_ylim()[1], "slow-motion release", ha="center",
                            va="bottom", fontsize=7, color=INK2)
    slows = sorted({r[10] for r in rows if r[10] > 0})
    axes[-1, 0].set_xlabel(f"animation time in the striker section (s); shaded: releases, shown "
                           f"{slows[0]:.0f}-{slows[-1]:.0f}x slow", color=INK2)
    axes[3, 1].set_ylim(-0.5, 12)
    axes[-1, 1].set_xlabel("ms after the servo leaves 124 deg", color=INK2)
    fig.suptitle("Striker cycle: re-arm, park, single tap, re-arm, double tap, rest (from the cam model)",
                 x=0.01, ha="left", fontsize=12, color=INK, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = path or os.path.join(OUT, "striker-cycle.png")
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("wrote", path, "first tap:", {k: round(v, 2) for k, v in ev.items() if isinstance(v, float)}, flush=True)
    return path


def assemble(frame_dir):
    """GIF / MP4 from the PNG frames of a chunked render (MOTION_PREVIEW_DIR chunks)."""
    names = sorted(n for n in os.listdir(frame_dir) if n.startswith("frame_") and n.endswith(".png"))
    res = write_video([Image.open(os.path.join(frame_dir, n)).convert("RGB") for n in names], FPS)
    print("render:", res, flush=True)
    return res


if __name__ == "__main__":
    if os.environ.get("MOTION_ASSEMBLE"):
        assemble(os.environ["MOTION_ASSEMBLE"])
    else:
        if not os.environ.get("MOTION_FRAMES"):
            striker_plot()
        if os.environ.get("MOTION_PLOT_ONLY") != "1":
            render()
