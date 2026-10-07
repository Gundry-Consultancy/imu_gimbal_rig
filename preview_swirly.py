"""2D preview of the swirly plate with example board placements (matplotlib)."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle
import swirly_grid as sg

rows, cols = (int(v) for v in sys.argv[1:3]) if len(sys.argv) >= 3 else (2, 4)
w, h = sg.plate_size(rows, cols)
fig, ax = plt.subplots(figsize=(10, 10 * h / w + 1))
ax.add_patch(FancyBboxPatch((0, 0), w, h, boxstyle=f"round,pad=0,rounding_size={sg.CORNER_RADIUS}",
                            fc="#d8e2ec", ec="#334"))
for (ax_, ay), (bx, by) in sg.slots(rows, cols):
    ax.plot([ax_, bx], [ay, by], lw=0, solid_capstyle="round")
    ax.add_patch(Circle((ax_, ay), sg.SLOT_WIDTH / 2, fc="white", ec="#334"))
    if (ax_, ay) != (bx, by):
        ax.add_patch(Circle((bx, by), sg.SLOT_WIDTH / 2, fc="white", ec="#334"))
        if ay == by:
            ax.add_patch(Rectangle((min(ax_, bx), ay - sg.SLOT_WIDTH / 2), abs(bx - ax_), sg.SLOT_WIDTH, fc="white", ec="none"))
        else:
            ax.add_patch(Rectangle((ax_ - sg.SLOT_WIDTH / 2, min(ay, by)), sg.SLOT_WIDTH, abs(by - ay), fc="white", ec="none"))
for c in range(1, cols):
    ax.axvline(c * sg.CELL, color="#99a", lw=0.6, ls=":")
for r in range(1, rows):
    ax.axhline(r * sg.CELL, color="#99a", lw=0.6, ls=":")
ax.set_xlim(-2, w + 2); ax.set_ylim(-2, h + 2); ax.set_aspect("equal")
ax.set_title(f"swirly grid {rows}x{cols}  ({w:.1f} x {h:.1f} mm)")
out = f"swirly-plate-{rows}x{cols}-preview.png"
fig.savefig(out, dpi=110, bbox_inches="tight")
print(out)
