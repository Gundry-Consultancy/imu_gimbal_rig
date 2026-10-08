"""Technical drawing of the servo dimensions the gimbal design depends on.

    python docs/make_servo_drawing.py

Writes docs/servo-measurements.png. The values are the ones currently in
make_gimbal.py (MG90S / STANDARD dicts). Measure your servos and fill in the
blanks, then update those dicts.
"""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

INK = "#1b2a3a"
DIM = "#c0392b"
FILL = "#dfe6ee"
TAB = "#b8c6d6"
CRIT = "#fdebd0"

SERVOS = [
    dict(
        title="MG90S micro (roll axis; also the 270° cam servo)",
        L=22.8, W=12.2, H=28.5, case_top=4.0, tab_len=32.2, tab_t=2.5,
        holes=(27.5, 28.0), across=0.0, hole_d=2.0, offset=5.5, spline_d=4.8,
        tab_under=(16.0, 21.0), boss_d=11.5,
        horn=dict(len=36.0, width=7.0, t=2.0, hub=2.5, hub_d=7.0, trimmed=14.0),
        spline="21T*",
    ),
    dict(
        title="Standard size: MG995 / DS3240 (tilt and pan axes)",
        L=40.7, W=19.9, H=42.9, case_top=4.5, tab_len=54.5, tab_t=3.0,
        holes=(48.5, 49.5), across=10.0, hole_d=4.5, offset=10.35, spline_d=6.0,
        tab_under=(27.8,), boss_d=14.0,
        horn=dict(len=46.0, width=8.5, t=2.5, hub=3.5, hub_d=10.0, trimmed=None),
        spline="25T",
    ),
]


def dim(ax, p1, p2, off, label, crit=False, text_off=1.2, rot=None, fs=8.5):
    """Linear dimension between p1 and p2, offset perpendicular by off."""
    (x1, y1), (x2, y2) = p1, p2
    horiz = abs(y2 - y1) < abs(x2 - x1)
    if horiz:
        y = max(y1, y2) + off if off > 0 else min(y1, y2) + off
        ax.plot([x1, x1], [y1, y], color=DIM, lw=0.5)
        ax.plot([x2, x2], [y2, y], color=DIM, lw=0.5)
        ax.annotate("", (x1, y), (x2, y), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.8, shrinkA=0, shrinkB=0))
        tx, ty = (x1 + x2) / 2, y + (text_off if off > 0 else -text_off)
        va = "bottom" if off > 0 else "top"
        ax.text(tx, ty, label, color=DIM, ha="center", va=va, fontsize=fs, rotation=rot or 0,
                bbox=dict(fc=CRIT, ec=DIM, lw=0.6, pad=1.2) if crit else None)
    else:
        x = max(x1, x2) + off if off > 0 else min(x1, x2) + off
        ax.plot([x1, x], [y1, y1], color=DIM, lw=0.5)
        ax.plot([x2, x], [y2, y2], color=DIM, lw=0.5)
        ax.annotate("", (x, y1), (x, y2), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.8, shrinkA=0, shrinkB=0))
        tx, ty = x + (text_off if off > 0 else -text_off), (y1 + y2) / 2
        ha = "left" if off > 0 else "right"
        ax.text(tx, ty, label, color=DIM, ha=ha, va="center", fontsize=fs, rotation=rot or 0,
                bbox=dict(fc=CRIT, ec=DIM, lw=0.6, pad=1.2) if crit else None)


def centreline(ax, x0, y0, x1, y1):
    ax.plot([x0, x1], [y0, y1], color=INK, lw=0.5, ls=(0, (8, 3, 2, 3)))


def front_view(ax, s, tab_under):
    """Long axis horizontal, shaft up. Shaft at x = 0, body centre at -offset."""
    cx = -s["offset"]
    L, H, ct = s["L"], s["H"], s["case_top"]
    case_h = H - ct
    ax.add_patch(Rectangle((cx - L / 2, 0), L, case_h, fc=FILL, ec=INK, lw=1.2))
    ax.add_patch(Rectangle((cx - s["tab_len"] / 2, tab_under), s["tab_len"], s["tab_t"], fc=TAB, ec=INK, lw=1.2))
    bd = s["boss_d"]
    ax.add_patch(Rectangle((-bd / 2, case_h), bd, ct * 0.45, fc=FILL, ec=INK, lw=1.0))
    ax.add_patch(Rectangle((-s["spline_d"] / 2, case_h + ct * 0.45), s["spline_d"], ct * 0.55, fc=TAB, ec=INK, lw=1.0))
    centreline(ax, 0, -3, 0, H + 4)
    centreline(ax, cx, -3, cx, case_h + 2)
    ax.text(cx, case_h / 2, "body", ha="center", va="center", fontsize=8, color=INK)

    left = cx - s["tab_len"] / 2
    right = cx + s["tab_len"] / 2
    dim(ax, (cx - L / 2, 0), (cx + L / 2, 0), -4, "A  body length")
    dim(ax, (left, tab_under + s["tab_t"]), (right, tab_under + s["tab_t"]), H - tab_under - s["tab_t"] + 4,
        "B  tab length")
    dim(ax, (right, 0), (right, H), 6 + (right - (cx + L / 2)) * 0 + 4, "D  base to spline top")
    dim(ax, (left, 0), (left, tab_under), -4, "E  base to\n    tab underside", crit=True)
    dim(ax, (left, tab_under), (left, tab_under + s["tab_t"]), -14, "F  tab\n    thickness")
    dim(ax, (s["boss_d"] / 2 + 1, tab_under + s["tab_t"]), (s["boss_d"] / 2 + 1, H), 2,
        "G  tab TOP to\n    spline top", crit=True)
    dim(ax, (0, -1), (cx + L / 2, -1), -9, "H  shaft centre\n    to body end", crit=True)


def top_view(ax, s):
    cx = -s["offset"]
    L, W = s["L"], s["W"]
    ax.add_patch(Rectangle((cx - s["tab_len"] / 2, -W / 2), s["tab_len"], W, fc=TAB, ec=INK, lw=1.2))
    ax.add_patch(Rectangle((cx - L / 2, -W / 2), L, W, fc=FILL, ec=INK, lw=1.2))
    ax.add_patch(Circle((0, 0), s["boss_d"] / 2, fc=FILL, ec=INK, lw=1.0))
    ax.add_patch(Circle((0, 0), s["spline_d"] / 2, fc=TAB, ec=INK, lw=1.0))
    ys = [0.0] if s["across"] == 0 else [-s["across"] / 2, s["across"] / 2]
    hp = (s["holes"][0] + s["holes"][1]) / 2
    for sx in (-1, 1):
        for y in ys:
            ax.add_patch(Circle((cx + sx * hp / 2, y), s["hole_d"] / 2, fc="white", ec=INK, lw=1.0))
    centreline(ax, cx - s["tab_len"] / 2 - 3, 0, cx + s["tab_len"] / 2 + 3, 0)
    centreline(ax, 0, -W / 2 - 3, 0, W / 2 + 3)
    y_top = max(ys)
    dim(ax, (cx - hp / 2, y_top), (cx + hp / 2, y_top), W / 2 - y_top + 4, "J  hole centres (along)", crit=True)
    dim(ax, (cx - s["tab_len"] / 2, -W / 2), (cx - s["tab_len"] / 2, W / 2), -4, "C  width")
    if s["across"]:
        dim(ax, (cx + hp / 2, -s["across"] / 2), (cx + hp / 2, s["across"] / 2), s["tab_len"] / 2 - hp / 2 + 4,
            "K  hole centres\n    (across)", crit=True)
    ax.annotate("M  hole Ø", (cx + hp / 2 + s["hole_d"] / 2, ys[0]), (cx + hp / 2 + 2, -W / 2 - 7),
                color=DIM, fontsize=8.5, arrowprops=dict(arrowstyle="-", color=DIM, lw=0.6))
    ax.annotate(f"N  spline Ø / teeth", (s["spline_d"] / 2 * 0.7, -s["spline_d"] / 2 * 0.7), (-s["boss_d"] / 2 - 10, -W / 2 - 7),
                color=DIM, fontsize=8.5, arrowprops=dict(arrowstyle="-", color=DIM, lw=0.6))


def horn_view(ax, s):
    h = s["horn"]
    ln, w = h["len"], h["width"]
    ax.add_patch(FancyBboxPatch((-ln / 2, -w / 2), ln, w, boxstyle=f"round,pad=0,rounding_size={w / 2}",
                                fc=TAB, ec=INK, lw=1.2))
    ax.add_patch(Circle((0, 0), h["hub_d"] / 2, fc=FILL, ec=INK, lw=1.0))
    ax.add_patch(Circle((0, 0), 1.2, fc="white", ec=INK, lw=0.8))
    for x in (-ln / 2 + 3, -ln / 2 + 6.5, ln / 2 - 3, ln / 2 - 6.5):
        ax.add_patch(Circle((x, 0), 0.6, fc="white", ec=INK, lw=0.6))
    if h["trimmed"]:
        for sx in (-1, 1):
            ax.plot([sx * h["trimmed"] / 2] * 2, [-w / 2 - 2, w / 2 + 2], color=DIM, lw=1, ls="--")
        ax.text(0, w / 2 + 3, f"cam: trim to {h['trimmed']:.0f}", color=DIM, ha="center", fontsize=8)
    dim(ax, (-ln / 2, -w / 2), (ln / 2, -w / 2), -3, "P  arm length (tip to tip)", crit=True)
    dim(ax, (ln / 2, -w / 2), (ln / 2, w / 2), 3, "Q  arm width\n    at hub", crit=True)
    # side view of the horn below
    y0 = -w / 2 - 14
    ax.add_patch(Rectangle((-ln / 2, y0), ln, h["t"], fc=TAB, ec=INK, lw=1.2))
    ax.add_patch(Rectangle((-h["hub_d"] / 2, y0 - h["hub"]), h["hub_d"], h["hub"] + 0.01, fc=FILL, ec=INK, lw=1.0))
    dim(ax, (ln / 2, y0), (ln / 2, y0 + h["t"]), 3, "R  arm thickness", crit=True)
    dim(ax, (-h["hub_d"] / 2, y0 - h["hub"]), (-h["hub_d"] / 2, y0 + h["t"]), -ln / 2 + h["hub_d"] / 2 - 3,
        "S  hub underside\n    to arm top", crit=True)
    ax.text(0, y0 - h["hub"] - 2.5, "side view (servo below)", ha="center", va="top", fontsize=8, color=INK)


def table(ax, s):
    tab_under = s["tab_under"]
    g = [s["H"] - t - s["tab_t"] for t in tab_under]
    hp = s["holes"]
    rows = [
        ("A", "Body length", f"{s['L']}"),
        ("B", "Tab length, end to end", f"{s['tab_len']}"),
        ("C", "Body width", f"{s['W']}"),
        ("D", "Base to spline top", f"{s['H']}"),
        ("E", "Base to tab underside", " / ".join(f"{t:g}" for t in tab_under)),
        ("F", "Tab thickness", f"{s['tab_t']}"),
        ("G", "Tab TOP to spline top  (sets mount)", " / ".join(f"{v:.1f}" for v in g)),
        ("H", "Shaft centre to near body end", f"{s['L'] / 2 - s['offset']:.1f}"),
        ("J", "Hole centres along", f"{hp[0]}–{hp[1]}"),
        ("K", "Hole centres across", f"{s['across'] or 'single row'}"),
        ("M", "Tab hole Ø / slot width", f"{s['hole_d']}"),
        ("N", "Spline Ø / teeth", f"{s['spline_d']} / {s['spline']}"),
        ("P", "Horn arm length", f"{s['horn']['len']}"),
        ("Q", "Horn arm width at hub", f"{s['horn']['width']}"),
        ("R", "Horn arm thickness", f"{s['horn']['t']}"),
        ("S", "Horn hub underside to arm top", f"{s['horn']['hub'] + s['horn']['t']:.1f}"),
    ]
    crit = {"E", "G", "H", "J", "K", "P", "Q", "R", "S"}
    ax.axis("off")
    cell = [[r[0], r[1], r[2], ""] for r in rows]
    t = ax.table(cellText=cell, colLabels=["", "Dimension (mm)", "Design value", "Measured"],
                 colWidths=[0.06, 0.52, 0.22, 0.20], loc="upper left", cellLoc="left")
    t.auto_set_font_size(False)
    t.set_fontsize(8.5)
    t.scale(1, 1.32)
    for (r, c), cl in t.get_celld().items():
        cl.set_edgecolor("#8899aa")
        if r == 0:
            cl.set_facecolor("#e8edf3")
            cl.set_text_props(weight="bold")
        elif rows[r - 1][0] in crit:
            cl.set_facecolor(CRIT if c < 3 else "white")


def main():
    fig = plt.figure(figsize=(17, 21))
    fig.suptitle("Servo dimensions to confirm / measure  —  imu_gimbal_rig", fontsize=16, weight="bold", y=0.985)
    fig.text(0.5, 0.968, "Shaded = mount geometry depends on it. MG90S figures disagree between sources, "
             "so E/G are drawn for both the 16 mm and 21 mm variants (tilt_ring_mg90s_tab16 / tab21). *some MG90S batches are 20T.",
             ha="center", fontsize=10, color="#444")
    outer = fig.add_gridspec(2, 1, hspace=0.16, top=0.93, bottom=0.03, left=0.03, right=0.97)
    for i, s in enumerate(SERVOS):
        gs = outer[i].subgridspec(2, 3, width_ratios=[1.35, 1.0, 1.05], height_ratios=[1.0, 0.85], hspace=0.25, wspace=0.12)
        fv = fig.add_subplot(gs[0, 0])
        front_view(fv, s, s["tab_under"][0])
        if len(s["tab_under"]) > 1:
            alt = s["tab_under"][1]
            cx = -s["offset"]
            fv.add_patch(Rectangle((cx - s["tab_len"] / 2, alt), s["tab_len"], s["tab_t"], fc="none", ec=DIM,
                                   lw=1.0, ls="--"))
            fv.text(cx - s["tab_len"] / 2 - 1, alt + s["tab_t"] / 2, f"alt. tab\nE = {alt:g}", color=DIM,
                    fontsize=7.5, ha="right", va="center")
        fv.set_title("Front view", fontsize=10, loc="left")
        tv = fig.add_subplot(gs[1, 0])
        top_view(tv, s)
        tv.set_title("Top view (from the spline side)", fontsize=10, loc="left")
        hv = fig.add_subplot(gs[:, 1])
        horn_view(hv, s)
        hv.set_title("Horn (double arm)", fontsize=10, loc="left")
        tb = fig.add_subplot(gs[:, 2])
        table(tb, s)
        for ax in (fv, tv, hv):
            ax.set_aspect("equal")
            ax.axis("off")
            ax.autoscale_view()
            x0, x1 = ax.get_xlim()
            y0, y1 = ax.get_ylim()
            pad = 0.12 * max(x1 - x0, y1 - y0)
            ax.set_xlim(x0 - pad, x1 + pad)
            ax.set_ylim(y0 - pad, y1 + pad)
        fig.text(0.03, outer[i].get_position(fig).y1 + 0.014, s["title"], fontsize=13, weight="bold", color=INK)
    out = os.path.join(HERE, "servo-measurements.png")
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    main()
