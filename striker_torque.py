"""Cam-servo torque budget for the striker, with friction, pawl lean and pressure angle.

    freecadcmd -c "exec(open('striker_torque.py').read(), {'__file__': 'striker_torque.py', '__name__': '__main__'})"

Writes docs/striker-torque.png and prints the table. Quasi-static, forward stroke (the cam turning
forward slowly), from make_striker's model through striker_motion.Striker:
- cam angle theta -> arm angle phi: the pawl's outline against the cam profile (phi_cam).
- the leaf held by the cam at phi with the drive pin on its slot; the slot holds the arm against
  gravity and slot friction (DriveLeaf.loaded_pose). P = the cam's load across the leaf at the pawl.
- contact: the cam profile point nearest the pawl's tip circle. n = contact normal (cam -> pawl),
  t = the cam surface's forward velocity there, r = its radius. The pawl, on its stop, passes the
  leaf-normal share of F = N (n + mu t) to the leaf: N = P / ((n + mu t) . e), e = +Y.
- servo torque T = N r (n . t + mu) (power: F . v / omega). n . t is the sine of the pressure angle.
- pawl lean: the moment of F about the pawl pin must keep the pawl on its stop; reported as the force
  line's offset from the pin (mm, + = onto the stop).

The edfadd7 MG90S design (two lobes, tail straight down) is evaluated under the same assumptions:
its leaf and cam as printed, with the pin on its slot (as the real parts must be).
"""

import contextlib
import importlib.util
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import make_striker as mk  # noqa: E402
import striker_motion as sm  # noqa: E402

KGCM = 98.0665                     # N mm per kg cm
MG995_STALL = 8.5                  # kg cm at 4.8 V (datasheet; 10 at 6 V)
MG90S_STALL = 1.8                  # kg cm at 4.8 V (2.2 at 6 V)
BUDGET = 1.0 / 3.0                 # required peak <= this share of stall
OLD_REV = "edfadd7"
OLD_CONST = dict(LOBES=((15, 115, 125), (135, 235, 245)), TAIL_TILT=0.0, FOLLOW_A=40.0, CAM_R0=8.0, PAWL_LEN=14.0)


def contact(m, theta, phi):
    """(q, n, t, r): cam contact point nearest the pawl's tip circle, its normal toward the pawl,
    the forward surface direction and the radius, at cam angle theta, arm angle phi."""
    _, c = m.pawl_axis(phi)
    _, a0 = m.polar(c)
    best = None
    cliffs = [m.bp + cl for _, _, cl in mk.LOBES]
    cands = [a0 + i * 0.05 for i in range(-800, 801)]
    for a in cands:
        r = m.cam_r(a + theta)
        q = (m.K[0] + r * math.cos(math.radians(a)), m.K[1] + r * math.sin(math.radians(a)))
        d = math.hypot(c[0] - q[0], c[1] - q[1])
        if best is None or d < best[0]:
            best = (d, q)
    for pc in cliffs:                                    # the cliff faces (radial)
        a = pc - theta
        for k in range(41):
            r = mk.CAM_R0 + m.h * k / 40
            q = (m.K[0] + r * math.cos(math.radians(a)), m.K[1] + r * math.sin(math.radians(a)))
            d = math.hypot(c[0] - q[0], c[1] - q[1])
            if d < best[0]:
                best = (d, q)
    d, q = best
    n = ((c[0] - q[0]) / d, (c[1] - q[1]) / d)
    dy, dz = q[0] - m.K[0], q[1] - m.K[1]
    r = math.hypot(dy, dz)
    return q, n, (dz / r, -dy / r), r, d


def point(m, theta, mu=None):
    """Torque row at cam angle theta on the forward stroke (None while the pawl is off the cam)."""
    mu = mk.MU_CAM if mu is None else mu
    phi = m.phi_cam(theta)
    if phi <= 1e-6:
        return None
    P = m.cam_force(phi)
    q, n, t, r, d = contact(m, theta, phi)
    nt = n[0] * t[0] + n[1] * t[1]

    def load(mu_):
        N = P / (n[0] + mu_ * t[0])                       # e = +Y
        F = (N * (n[0] + mu_ * t[0]), N * (n[1] + mu_ * t[1]))
        return N, F, N * r * (nt + mu_)
    N, F, T = load(mu)
    _, _, T0 = load(0.0)
    pin, _ = m.pawl_axis(phi)
    mom = (q[0] - pin[0]) * F[1] - (q[1] - pin[1]) * F[0]
    return dict(theta=theta, phi=phi, P=P, N=N, r=r, pa=math.degrees(math.asin(max(-1, min(1, nt)))),
                T=T, T0=T0, lean_mm=mom / math.hypot(*F), gap=d - mk.PAWL_W / 2)


def curve(m, th0=0.0, th1=None, step=0.5, mu=None):
    th1 = th1 if th1 is not None else max(c for _, _, c in mk.LOBES) + 12
    rows = []
    th = th0
    while th <= th1 + 1e-9:
        p = point(m, th, mu)
        if p:
            rows.append(p)
        th += step
    return rows


@contextlib.contextmanager
def patched(**kw):
    old = {k: getattr(mk, k) for k in kw}
    for k, v in kw.items():
        setattr(mk, k, v)
    try:
        yield
    finally:
        for k, v in old.items():
            setattr(mk, k, v)


def old_module():
    """make_striker as of OLD_REV, imported under another name."""
    src = subprocess.run(["git", "show", f"{OLD_REV}:make_striker.py"], cwd=HERE, capture_output=True,
                         text=True, check=True).stdout
    spec = importlib.util.spec_from_loader("make_striker_" + OLD_REV, loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__file__ = os.path.join(HERE, "make_striker.py")
    exec(compile(src, f"make_striker@{OLD_REV}", "exec"), mod.__dict__)
    return mod


def old_model(sg):
    """The edfadd7 parts (leaf, cam, arm) with the pin-on-slot physics and the same friction."""
    mo = old_module()
    num, st = mo.solve(sg)
    lf = st["leaf"]
    st = dict(st, leaf=mk.DriveLeaf(lf.y0, lf.z0, lf.t, lf.d_rest))
    return sm.Striker(sg, state=(num, st))


def summary(rows):
    pk = max(rows, key=lambda r: r["T"])
    return dict(peak_kgcm=round(pk["T"] / KGCM, 2), peak_at=pk["theta"], peak_phi=round(pk["phi"], 1),
                frictionless_peak_kgcm=round(max(r["T0"] for r in rows) / KGCM, 2),
                max_pressure_deg=round(max(r["pa"] for r in rows), 1),
                max_cam_load_N=round(max(r["N"] for r in rows), 1),
                min_lean_mm=round(min(r["lean_mm"] for r in rows), 2))


def plot(new, old, path, new_label="MG995 cam (this design)", old_label=f"MG90S cam ({OLD_REV})"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    blue, orange, ink, muted, grid = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
    fig, ax = plt.subplots(figsize=(9, 5.2), dpi=130)
    def broken(rows, key):                       # NaN where the pawl is off the cam (no line across)
        th, ys = [], []
        for i, r in enumerate(rows):
            if i and r["theta"] - rows[i - 1]["theta"] > 0.75:
                th.append(float("nan"))
                ys.append(float("nan"))
            th.append(r["theta"])
            ys.append(r[key] / KGCM)
        return th, ys
    for rows, col, lab in ((new, blue, new_label), (old, orange, old_label)):
        ax.plot(*broken(rows, "T"), color=col, lw=2, label=f"{lab}, mu {mk.MU_CAM}")
        ax.plot(*broken(rows, "T0"), color=col, lw=1.2, ls="--", label=f"{lab}, frictionless")
        pk = max(rows, key=lambda r: r["T"])
        ax.annotate(f"{pk['T'] / KGCM:.2f} kg cm", (pk["theta"], pk["T"] / KGCM), xytext=(4, 6),
                    textcoords="offset points", color=ink, fontsize=8)
    for y, txt in ((MG995_STALL * BUDGET, f"MG995 budget: 1/3 of {MG995_STALL} kg cm stall (4.8 V)"),
                   (MG90S_STALL, f"MG90S stall {MG90S_STALL} kg cm (4.8 V)")):
        ax.axhline(y, color=muted, lw=1, ls=":")
        ax.text(1, y + 0.05, txt, color=muted, fontsize=8)
    ax.set_xlabel("cam angle from rest (deg, forward)", color=ink)
    ax.set_ylabel("servo torque needed (kg cm)", color=ink)
    ax.set_title("Striker cam torque, quasi-static forward stroke (friction at pawl/cam and pin/slot)",
                 color=ink, fontsize=10)
    ax.grid(color=grid, lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_ylim(-0.4, MG995_STALL * BUDGET + 0.4)
    ax.text(0.01, 0.02, "below 0: the pawl rolling off a cliff corner pushes the cam on", transform=ax.transAxes,
            ha="left", color=muted, fontsize=8)
    ax.legend(fontsize=8, frameon=False, loc="upper right", bbox_to_anchor=(1.0, 0.88))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def evaluate(sg, with_old=True):
    """(new rows, new summary, old rows, old summary)."""
    m = sm.Striker(sg)
    new = curve(m)
    out = [new, summary(new)]
    if with_old:
        with patched(**OLD_CONST):
            mo = old_model(sg)
            old = curve(mo)
        out += [old, summary(old)]
    return out


def release_angles(m, lobe=0, step=0.1):
    """(cliff, drop): the cam angle of the lobe's cliff, and the first angle past it where the pawl
    has rolled right over the cliff's corner and the arm is back on its rest stop (phi_cam = 0)."""
    cliff = mk.LOBES[lobe][2]
    th = cliff
    while m.phi_cam(th) > 1e-6:
        th += step
        if th > cliff + 60:
            raise ValueError("the pawl never leaves the lobe")
    return cliff, round(th, 1)


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    import json
    import make_gimbal as mg
    import make_sensor_stack as ss
    bi = json.load(open(os.path.join(HERE, "build_info.json")))
    y_uo = -(bi["ring_in_y"] + mg.RING_BAR_T + mg.YOKE_GAP) - mg.YOKE_BRG_T
    sg = dict(upright_out_y=y_uo, roof_z=ss.roof_top() - ss.axis_z(), roof_raise=ss.roof_raise(),
              upright_half_x=mg.YOKE_HALF_X, stand_bolts=mg.STAND_BOLTS)
    new, s_new, old, s_old = evaluate(sg)
    print("theta  phi    P(N)  N(N)   r    PA(deg)  T(kgcm) T0(kgcm) lean(mm)")
    for rows, name in ((new, "new"), (old, OLD_REV)):
        print("--", name)
        for r in rows:
            if abs(r["theta"] / 5 - round(r["theta"] / 5)) < 1e-6:
                print(f"{r['theta']:5.1f} {r['phi']:5.1f} {r['P']:6.2f} {r['N']:6.2f} {r['r']:5.2f} {r['pa']:7.1f} "
                      f"{r['T'] / KGCM:7.2f} {r['T0'] / KGCM:7.2f} {r['lean_mm']:7.2f}")
    print("summary new", s_new)
    print("release (cliff, pawl off the corner):", release_angles(sm.Striker(sg)))
    print("summary", OLD_REV, s_old, "-> share of MG90S stall", round(s_old["peak_kgcm"] / MG90S_STALL, 2))
    print("budget", round(MG995_STALL * BUDGET, 2), "kg cm:", "OK" if s_new["peak_kgcm"] <= MG995_STALL * BUDGET else "OVER")
    plot(new, old, os.path.join(HERE, "docs", "striker-torque.png"))
    print("wrote docs/striker-torque.png")
