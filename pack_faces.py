"""Pack portrait STEMMA QT boards along a swirly-grid face (cables exit the face's long edges).

Each board must land its top-edge hole pair (present on 2- and 4-hole QT boards)
in slots. In portrait, that pair is 20.32 mm apart along Y.
"""
import itertools
import sys

import swirly_grid as sg

QT_W, QT_H = 17.78, 25.4          # portrait outline
PAIR_DY = 20.32
GAP = 0.0                          # boards may touch edge to edge (connectors are on the short edges)


def board_options(rows, cols, margin=2.54):
    """(x_left, y_bottom, side) for portrait boards whose hole pair lands in slots."""
    fw, fh = sg.plate_size(rows, cols)
    opts = []
    for side, hx in (("left", 2.54), ("right", QT_W - 2.54)):
        for q, (dx, dy) in sg.find_fits([(0, 0), (0, PAIR_DY)], rows, cols):
            if q != 0:
                continue
            x0, y0 = dx - hx, dy - 2.54
            if -margin <= x0 and x0 + QT_W <= fw + margin and -margin <= y0 and y0 + QT_H <= fh + margin:
                opts.append((round(x0, 2), round(y0, 2), side))
    return sorted(set(opts))


def best_row(rows, cols, n):
    opts = board_options(rows, cols)
    xs = sorted({o[0] for o in opts})
    for combo in itertools.combinations(xs, n):
        if all(b - a >= QT_W + GAP for a, b in zip(combo, combo[1:])):
            picks = [min((o for o in opts if o[0] == x), key=lambda o: abs(o[1] + QT_H / 2 - sg.plate_size(rows, cols)[1] / 2))
                     for x in combo]
            return picks
    return None


if __name__ == "__main__":
    for rows, cols in ((3, 4), (3, 5)):
        fw, fh = sg.plate_size(rows, cols)
        print(f"grid {rows}x{cols} ({fw:.1f} x {fh:.1f}): {len(board_options(rows, cols))} portrait placements")
        for n in (3, 4):
            print(f"  {n} boards:", best_row(rows, cols, n))
