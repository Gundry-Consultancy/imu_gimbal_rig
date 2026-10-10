# Variant: tall_top

This is the main design with more room between the middle and top decks. The
upper spine goes from 17.1 mm to **25 mm**. The bottom deck and lower spine
are unchanged, since the bottom deck isn't attached to the gimbal.

![changes](changes-highlighted.png)

## What changed

**Modified parts (print these):**

| Part | Change |
|---|---|
| `stack_spine_upper` | 17.1 → **25.0 mm** tall |
| `striker_stand` | pad seat and servo-plate leg 7.9 mm taller (overall 52.1 → 60.0 mm) |

**Same parts, sitting 7.9 mm higher:** `stack_top_deck` (and its roof),
`striker_bar`, `striker_pawl`, `striker_cam` / `striker_cam_spline`, and the
cam servo. The STLs are the same as the main design's.

**Unchanged:**
- The middle deck and its roll walls, the roll axis, tilt ring, pan yoke, base,
  bottom deck, lower spine and horns.
- The ring and yoke sizes are pinned to the main build (`../../build_info.json`).

**Striker numbers are the same as the main design:**
- 15 mm hammer travel, hovering ~0.85 mm above the roof at rest
- 2 N preload, 14 N cam force when parked, 1.3 % peak strain

**Clearance checks:**
- All checks pass.
- **Roll through 360°:** the higher top deck's connectors clear the unchanged
  tilt ring by 2.4 mm. The main design allows 4 mm, so allow less cable slack
  on the top deck if you roll it all the way round. Within the MG90S's ±90°
  there's more room.
- **Tilt with the striker fitted:** improves from ±55° to **−70° / +65°**.

**Hardware:** the two M3 bolts through the stack need to be about 8 mm
longer (M3 x 60 instead of x 50).

## If you've already printed the main design

Print only `stack_spine_upper` and `striker_stand` from `parts/` here.
Everything else carries over.

## Regenerate

From the repo root, after a main build (which writes `build_info.json`):

```
GIMBAL_OUT=variants/tall_top GIMBAL_PIN=build_info.json STACK_UPPER_GAP=25 freecadcmd -c "exec(open('make_gimbal.py').read(), {'__file__': 'make_gimbal.py', '__name__': '__main__'})"
```
