"""Swirly-grid slot geometry, ported from tannewt/swirly-grid (KiCad generator).

https://github.com/tannewt/swirly-grid

The pattern is a 0.6" square cell of 3 x 3 points on a 0.2" pitch, inset 0.1"
from the cell edge. Each point is either a round 0.1" hole (cell centre) or the
end of a 0.1" wide slot joining it to a neighbour, giving a pinwheel of four
slots around the central hole. Any board whose holes sit on 0.1" multiples can
usually find a position where every screw lands in a slot.

Pure Python (no FreeCAD import) so the fit checker can run anywhere.
Coordinates are millimetres, origin at the plate's lower-left corner, +Y up,
viewed from the top (KiCad F.Silkscreen side) so the swirl handedness matches
the PCB.
"""

from __future__ import annotations

import math

INCH = 25.4
CELL = 0.6 * INCH
SLOT_WIDTH = 0.1 * INCH
CORNER_RADIUS = 0.05 * INCH

RIGHT, LEFT, UP, DOWN, FULL = "right", "left", "up", "down", "full"

# (row, col) within the 3 x 3 point cell -> which way the slot opens.
# Row 0 is the top row (KiCad +Y is down).
PATTERN = {
    (0, 0): DOWN,
    (0, 1): RIGHT,
    (0, 2): LEFT,
    (1, 0): UP,
    (1, 1): FULL,
    (1, 2): DOWN,
    (2, 0): RIGHT,
    (2, 1): LEFT,
    (2, 2): UP,
}


def plate_size(rows: int, cols: int) -> tuple[float, float]:
    return cols * CELL, rows * CELL


def _point(i: int, j: int, rows: int) -> tuple[float, float]:
    """Point column i, row j (row 0 at the top) -> mm, +Y up."""
    x_in = 0.1 + 0.2 * i
    y_in = 0.1 + 0.2 * j
    return x_in * INCH, (0.6 * rows - y_in) * INCH


def slots(rows: int, cols: int) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Slot centrelines as (start, end) pairs. Round holes have start == end."""
    out = []
    for j in range(3 * rows):
        for i in range(3 * cols):
            direction = PATTERN[(j % 3, i % 3)]
            p = _point(i, j, rows)
            if direction == FULL:
                out.append((p, p))
            elif direction == RIGHT:
                out.append((p, _point(i + 1, j, rows)))
            elif direction == DOWN:
                out.append((p, _point(i, j + 1, rows)))
            # LEFT / UP are the far ends of slots already emitted.
    return out


def _dist_to_segment(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _rotate(holes, quarter_turns):
    out = []
    for x, y in holes:
        for _ in range(quarter_turns % 4):
            x, y = -y, x
        out.append((x, y))
    return out


def find_fits(holes, rows: int, cols: int, tol: float = 0.15, step: float = INCH / 200):
    """Find placements of a board's hole pattern on the grid.

    holes: hole centres in the board's own frame (mm).
    tol: how far a hole centre may sit off a slot centreline. A 2.54 mm slot
         with an M2.5 screw has ~0.02 mm radial play, so keep this small.
    Returns a list of (quarter_turns, (dx, dy)) where board-frame hole + (dx, dy)
    lands in a slot for every hole, after rotating the pattern by quarter_turns.
    """
    grid = slots(rows, cols)
    fits = []
    seen = set()
    for q in range(4):
        pattern = _rotate(holes, q)
        x0, y0 = pattern[0]
        # Slide the first hole along every slot centreline; check the rest.
        for a, b in grid:
            n = max(1, int(round(math.dist(a, b) / step)))
            for k in range(n + 1):
                t = k / n
                cx = a[0] + t * (b[0] - a[0])
                cy = a[1] + t * (b[1] - a[1])
                dx, dy = cx - x0, cy - y0
                if all(
                    any(_dist_to_segment((hx + dx, hy + dy), sa, sb) <= tol for sa, sb in grid)
                    for hx, hy in pattern[1:]
                ):
                    key = (q, round(dx, 2), round(dy, 2))
                    if key not in seen:
                        seen.add(key)
                        fits.append((q, (dx, dy)))
    return fits


def rect_holes(spacing_x: float, spacing_y: float):
    return [(0.0, 0.0), (spacing_x, 0.0), (spacing_x, spacing_y), (0.0, spacing_y)]


if __name__ == "__main__":
    import sys

    rows, cols = (int(v) for v in sys.argv[1:3]) if len(sys.argv) >= 3 else (2, 2)
    w, h = plate_size(rows, cols)
    print(f"{rows}x{cols} plate: {w:.2f} x {h:.2f} mm, {len(slots(rows, cols))} slots/holes")
    qt = rect_holes(0.8 * INCH, 0.5 * INCH)
    fits = find_fits(qt, rows, cols)
    print(f"STEMMA QT 1.0x0.7 (0.8x0.5 in holes): {len(fits)} placements")
    for q, (dx, dy) in fits[:10]:
        print(f"  rot {q * 90:3d} deg, first hole at ({dx:.2f}, {dy:.2f})")
