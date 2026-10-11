"""Headless preview of the assembly animation: FreeCAD tessellation -> matplotlib Poly3DCollection
frames -> Pillow GIF, plus one PNG per step (that step's objects in colour, the rest grey).

Called by make_assembly_animation.py (render(plan, out_dir)); uses the macro's own timeline
functions (assembly_checks.timeline_module), so the GIF shows exactly what the macro plays.
"""

import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from PIL import Image  # noqa: E402

import assembly_checks as ac  # noqa: E402

LIGHT = np.array([0.35, -0.55, 0.75]) / np.linalg.norm([0.35, -0.55, 0.75])
GREY = np.array([0.88, 0.88, 0.90])
VIEW = (24, -58)                  # elev, azim: isometric-ish from front-left, striker side visible
MOTION_FRAMES = 6                 # per step (GIF)
HOLD_FRAMES = 2
FRAME_MS = 85


def mesh(it):
    """(triangles [n, 3, 3], shade [n]) of an item's final world shape."""
    import MeshPart
    lin, ang = (0.3, 0.7) if it.fix else (1.0, 0.5)      # ~80k triangles for the whole rig
    m = MeshPart.meshFromShape(Shape=it.world_shape(), LinearDeflection=lin, AngularDeflection=ang, Relative=False)
    v, f = m.Topology
    if not f:
        return np.zeros((0, 3, 3)), np.zeros(0)
    v = np.array([[p.x, p.y, p.z] for p in v])
    t = v[np.array(f)]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    return t, 0.42 + 0.58 * np.abs((n / ln) @ LIGHT)


def frame_state(plan, meshes, tl, steps, cur, u, mode="anim"):
    """Triangles and RGBA colours for step index cur at time u.
    mode 'anim': current step in colour, earlier steps dimmed; 'png': current in colour, rest grey."""
    key = steps["list"][cur]["key"]
    tris, cols = [], []
    for it in plan["items"]:
        t, sh = meshes[it.name]
        if not len(t):
            continue
        off, shown, fade = tl.offset_at(ac.moves_json(it), steps, cur, u)
        if not shown:
            continue
        current = any(m["step"] == key for m in it.moves)      # placed or carried in this step
        base = np.array(it.color)
        alpha = 1.0
        if not current:
            base = base * (1 - 0.65) + GREY * 0.65 if mode == "anim" else GREY
            alpha = 0.45 if mode == "anim" else 0.35
        else:
            alpha = max(0.15, fade)
        rgb = np.clip(base[None, :] * sh[:, None], 0, 1)
        tris.append(t + np.array(off))
        cols.append(np.concatenate([rgb, np.full((len(t), 1), alpha)], axis=1))
    if not tris:
        return np.zeros((0, 3, 3)), np.zeros((0, 4))
    return np.concatenate(tris), np.concatenate(cols)


def step_bounds(plan, meshes, tl, steps, cur):
    """Cube (centre, half) framing everything shown during step cur (start, middle, end)."""
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for u in (0.0, 0.5, 1.0):
        t, _ = frame_state(plan, meshes, tl, steps, cur, u)
        if not len(t):
            continue
        p = t.reshape(-1, 3)
        lo, hi = np.minimum(lo, p.min(0)), np.maximum(hi, p.max(0))
    c = (lo + hi) / 2
    return c, max(hi - lo) / 2 * 1.02


def draw(tris, cols, centre, half, caption, sub, size=(8.0, 6.4), dpi=100):
    fig = plt.figure(figsize=size, dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 0.9], projection="3d")
    if len(tris):
        ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolors="none", linewidths=0))
    for i, setl in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        setl(centre[i] - half, centre[i] + half)
    ax.set_box_aspect((1, 1, 1), zoom=1.25)
    ax.view_init(elev=VIEW[0], azim=VIEW[1])
    ax.set_axis_off()
    fig.text(0.5, 0.965, caption, ha="center", va="center", fontsize=14, weight="bold")
    fig.text(0.5, 0.925, sub, ha="center", va="center", fontsize=9, color="#333333")
    fig.canvas.draw()
    img = Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert("RGB")
    plt.close(fig)
    return img


def _sub(step, width=110):
    s = "; ".join(f"{n} x {k.split(' (')[0]}" for k, n in step["fixings"].items()) or "parts only"
    return s if len(s) <= width else s[:width - 3] + "..."


def render(plan, out_dir, gif_name="assembly-animation.gif"):
    tl = ac.timeline_module()
    steps = ac.steps_json(plan)
    meshes = {it.name: mesh(it) for it in plan["items"]}
    print("render: triangles", sum(len(m[0]) for m in meshes.values()), flush=True)
    os.makedirs(os.path.join(out_dir, "steps"), exist_ok=True)
    nst = len(steps["list"])
    bounds = [step_bounds(plan, meshes, tl, steps, i) for i in range(nst)]
    frames = []
    prev = bounds[0]
    for i, s in enumerate(steps["list"]):
        st = plan["steps"][i]
        cap = f"Step {s['n']}/{nst}: {s['title']}"
        for k in range(MOTION_FRAMES + HOLD_FRAMES):
            u = min(1.0, k / (MOTION_FRAMES - 1)) if MOTION_FRAMES > 1 else 1.0
            w = min(1.0, (k + 1) / 3)                       # ease the framing over the first frames
            c = prev[0] * (1 - w) + bounds[i][0] * w
            h = prev[1] * (1 - w) + bounds[i][1] * w
            tris, cols = frame_state(plan, meshes, tl, steps, i, u)
            frames.append(draw(tris, cols, c, h, cap, _sub(st)))
        prev = bounds[i]
        tris, cols = frame_state(plan, meshes, tl, steps, i, 1.0, mode="png")
        draw(tris, cols, bounds[i][0], bounds[i][1], cap, _sub(st, 140), size=(10, 8)).save(
            os.path.join(out_dir, "steps", f"{s['n']:02d}.png"), optimize=True)
        print(f"render: step {s['n']} done ({len(frames)} frames)", flush=True)
    # final: everything in colour, held
    last = plan["steps"][-1]["key"]
    full = draw(*_all_colour(plan, meshes, tl, steps), bounds[-1][0], bounds[-1][1],
                "Assembled", f"{sum(1 for i in plan['items'] if i.fix)} fixings in {nst} steps")
    frames += [full] * 8
    pal = [f.quantize(colors=96, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
    path = os.path.join(out_dir, gif_name)
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=FRAME_MS, loop=0, optimize=True)
    mb = os.path.getsize(path) / 1e6
    print(f"render: wrote {path}: {len(frames)} frames, {len(frames) * FRAME_MS / 1000:.1f} s, {mb:.1f} MB "
          f"(last step {last})", flush=True)
    return path


def _all_colour(plan, meshes, tl, steps):
    tris, cols = [], []
    for it in plan["items"]:
        t, sh = meshes[it.name]
        if len(t):
            tris.append(t)
            cols.append(np.concatenate([np.clip(np.array(it.color)[None, :] * sh[:, None], 0, 1),
                                        np.ones((len(t), 1))], axis=1))
    return np.concatenate(tris), np.concatenate(cols)
