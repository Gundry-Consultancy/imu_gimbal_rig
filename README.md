# IMU gimbal rig

Pan / tilt / roll rig that carries a whole set of Adafruit IMU, accelerometer
and magnetometer breakouts (3/6/9 DoF) at once, with a cam-driven striker for
tap and double-tap detection testing.

Target sensors: the drivers added in
[adafruit/Adafruit_Wippersnapper_Arduino#839](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/pull/839)
(ISM330DHCX, ISM330DLC, LIS2MDL, LIS3DH, LIS3MDL, LSM303AGR, LSM303DLH,
LSM6DS3, LSM6DSO32, LSM9DS1). Mostly STEMMA QT breakouts, the odd legacy board,
and one FeatherWing.

## Requirements

- Every sensor is mounted at the same time, on stacked layers of swirly-grid
  plates ([tannewt/swirly-grid](https://github.com/tannewt/swirly-grid)). The
  sides stay open, with a small roof in the middle of the top to whack on. A
  swirly-grid PCB also bolts on through the matching slots.
- Only the centre plate connects to the roll servo and axle. The other plates
  hang off it on central standoffs in line with the roof, not at the edges.
- STEMMA QT sized breakouts, 4 across, plus a Feather-sized place for one
  FeatherWing.
- Hobby servos: MG90S (roll, and an MG90S-size 270° servo for the striker
  cam) and DS3240MG (tilt and pan). Both are measured; see
  `docs/servo-measurements.png`.
- Aim for ±180° on every axis. In practice the servo travel limits it.
- Sensors do not need to sit on the rotation centre. Off-axis
  acceleration is accepted.

## Sensor stack (`make_sensor_stack.py`)

Each deck is a 5 x 4 cell swirly grid (76.2 x 61 mm) with a 3 mm border. The
sides are open.

| Deck | Boards |
|---|---|
| top | 8 portrait QT around a central post with a flared **roof** (24 mm square) for the striker |
| middle | FeatherWing on 10 mm standoffs (clears header pins) + 1 QT, and 4 QT. Its +X / -X end walls (6 mm, with a rib and boss behind the hub) carry the MG90S roll horn and the idler axle. This is the only deck connected to the gimbal. |
| bottom | 8 portrait QT |

That is room for **21 QT boards + 1 FeatherWing**. PR 839 needs 9 breakouts + the wing.

- **Board layout:** boards sit portrait, 4 across, in two rows per deck. Each
  uses the QT connector on its outer edge, so cables leave through the open
  ±Y sides.
- **Fixing:** each board bolts through its top-edge hole pair (every QT board
  has it), with M2.5 screws, 3 mm spacers and nuts.
- **Spines:** a solid strip runs between the two rows on every deck. The upper
  and lower spines (36 x 7 mm standoffs) sit on it, directly under the roof.
- **Clamping:** two M3 bolts at x = ±14 run through the whole stack. Put them
  in from below, heads under the bottom deck, with the nuts on top, just
  outside the roof. A 5.5 mm head on top would overlap the roof edge by
  0.75 mm, so it can't go in straight down. The top and bottom decks have
  bosses round the bolt holes (6 mm there instead of 3), so washers are
  optional.
- **Mux:** you'll need a TCA9548A, because of I²C address clashes (several
  LIS3MDL/LSM303/LSM9DS1 parts share addresses). It fits in a spare board spot.

`pack_faces.py` checks the board placements against the slot geometry.

## Tap striker (`make_striker.py`)

**Redesigned 2026-10-10 so the gimbal can tilt all the way over.** The old
hairpin bar hovered just above the roof and limited tilt to ±55°. Now a rigid
hammer arm pivots on top of a tower on the pan yoke. When parked, it swings
up and well clear of the whole tilt sweep.

![striker section](docs/striker-section.png)

![striker changes](docs/striker-redesign-changes.png)

**Parts:**
- **Tower** (`striker_stand`): bolts to the yoke's idler upright with the
  same 4 x M3 as before. Everything on it stays behind the ring band
  (y < −62 mm), so nothing on it can meet the ring when it tilts.
- **Hammer arm** (`striker_arm`): pivots on an M3 x 50 bolt, through a printed
  bushing and two spacers, about 66 mm above the roof.
- **Drive leaf** (`striker_spring`): a PETG strip standing on a seat at the
  bottom of the tower. Its tip fork is pinned (M3) to the arm's tail.
- **Cam:** the 270° servo's two-lobe cam bends the drive leaf through a
  folding pawl.

**How it works:**
- **Rest:** the drive leaf pushes the tail back, and the tail lands on a
  rest stop between the walls (with a 2 mm bumper lip). The hammer head then
  hovers **5 mm above the roof**. The servo carries no load at rest:
  the pawl sits 0.4 mm clear of the cam's base circle.
- **Park:** the cam bends the leaf, which swings the arm **48° up**.
  The parked arm clears everything that moves through the full 360° of tilt.
- **Tap:** at a cliff the pawl drops off the cam. The leaf throws the arm
  down onto its rest stop, and the stop takes the arm's energy.
- **Let-off:** the head sits on its own thin PETG leaf (lost motion). A nylon
  M3 screw in the overarm presses the leaf down, preloading it **1.4 N**
  against the screw. When the arm stops, the head's momentum carries it on
  through the 5 mm gap to the roof (**2.0 N** of leaf force at
  contact). Then it springs back onto the screw. The preload keeps it from
  bouncing back into the roof for a second tap.
- **Energy:** the drop releases **54.7 mJ** in all.
  - **36 mJ** is the head's share. That's what the tap carries
    (**27.5 mJ** is left at the roof after the leaf work).
  - The arm's **18.7 mJ** goes into the tower's rest stop, not the
    sensor stack.
- **Let-off margin:** the head could travel **15.5 mm** past the arm
  stop if the roof weren't there. That's ≥ 3 x the 5 mm gap, so the tap
  always lands.
- **Pawl:** pinned with 1.75 mm filament. It points at the cam centre and
  leans 16° past the cam's force line onto a stop under it. The cam's push,
  its forward drag and gravity all hold it there. On the reverse stroke the
  cliffs fold it up and back, and gravity drops it back.
- **Two lobes in the 270° travel.** Forward is −X rotation (seen from +X,
  the cam turns clockwise). If your servo runs the other way, flip the
  angles in software.

  | Action | Servo |
  |---|---|
  | rest, hammer hovering | 0° |
  | park for a double tap | ~120° |
  | park for a single tap | ~240° |
  | double tap | sweep forward through 125° and 245° (about 0.2 s apart at full MG90S speed) |
  | single tap | from 240°, step past 245° |
  | re-arm | back to 0°, then forward to a park angle |

  **Keep the hammer parked whenever the gimbal moves.**

**Computed for E = 2 GPa** (spring models in `leaf_design()` and
`head_leaf_design()`):

| | |
|---|---|
| drive leaf | 20 x 2.01 x 68 mm. 1.69 N at rest, 5.13 N parked at the pin; 1.3 % peak strain parked. The seat leans back 5.57° so a straight print is preloaded |
| head leaf | 12 x 1.36 x 50 mm, k = 0.12 N/mm, preloaded 11.7 mm by the stop screw; 1.36 % strain at contact |
| arm | 20.2 g; the head is 0.66 of its moment of inertia |
| cam | lift 5.72 mm (r 8 → 13.7). 14.5 N on the pawl when parked (the old bar: 14 N); servo torque ≤ 0.48 kg·cm |

**Clearance** (`striker_limits()` in `make_gimbal.py`: `distToShape`
against everything that moves, 5° steps). Each limit is ≥ 5 mm.

| | tilt 360° (parked) | roll ±90° (parked) | tilt 360° x roll ±45/±90° (parked, 15° steps) | at rest, tilt 0 / roll 0 |
|---|---|---|---|---|
| main | **7.9 mm** (clear through 360°) | 17.9 mm | 7.9 mm | 5.0 mm (the head over the roof) |
| `tall_top` | **7.9 mm** (clear through 360°) | 16.3 mm | 7.9 mm | 5.0 mm (the head over the roof) |

The ring's −Y axle stub is left out of these checks. It only spins inside the yoke's bearing, and its 4.5 mm to the tower's axle-screw pocket is set by the yoke (unchanged).

**At rest, apart from the roof** (main and `tall_top` alike), the nearest
moving part to each striker piece is:
- head: a top-deck QT board, 17.0 mm
- head leaf: 42.0 mm
- arm: 47.6 mm

The rest of the striker is at least 7.9 mm from anything that moves.

**Tuning:**
- The tap energy, contact force, gap and let-off margin are `TAP_ENERGY`,
  `F_CONTACT`, `HOVER` and `OVERSHOOT_MIN` at the top of `make_striker.py`.
  The leaves are re-solved from those values.
- On the bench, the stop-screw nuts trim the head preload. A thin shim on
  the rest-stop lip trims the hover gap.
- **Creep:** PETG relaxes under constant load. The drive leaf only carries
  much load while parked, and the stop screw takes up any creep in the head
  leaf.

**Printing:**
- `striker_arm` and `striker_spring` must be PETG, at 100 % infill.
  - `striker_arm` prints on its side (one 12 mm thick profile).
  - `striker_spring` prints flat, with its lugs and fork up.
- The tower prints upright, floor down. The pivot bosses are downward
  teardrops, and the walls stand vertical.

## Gimbal (`make_gimbal.py`)

Axes: roll = X, tilt = Y, pan = Z, all crossing in the stack. The script
measures the sweep radius of each stage to size the next one out. It then
rotates each stage through 360° against its neighbour, checks the striker
against the roll and tilt ranges, and reports any collision.

| Part | Holds | Servo | Idler |
|---|---|---|---|
| `stack_*` | all the boards | MG90S horn pocket on the middle deck's +X wall | M3 axle boss on the -X wall |
| `tilt_ring_mg90s` | roll servo + 623 bearing | MG90S (measured) | DS3240 horn on the +Y bar, M3 axle on -Y |
| `pan_yoke` | tilt servo + 623 bearing, outer gussets, striker stand pilots | DS3240MG on +Y | pan horn pocket underneath |
| `base` | pan servo | DS3240MG (270° version for ±135°) | open +X end for cables, 4 x M3 bench holes |
| `striker_*` | tower (stand), hammer arm, drive leaf, pawl, cam, pivot bushing + spacers | MG90S-size 270° servo | M3 pivot on a printed bushing |

**Servo mounts:** each mount sets its height from G (tab top to spline top),
taken as you measured it: MG90S 12.0, DS3240 14.0. If a servo sits a little
low, put a washer under its tabs. Every mount has a wire notch at the spline
end, 0.4 mm clearance, lead-in chamfers on both faces and bosses round the
screw pilots; see [Fit fixes](#fit-fixes-after-the-first-test-print).

The overall envelope is about 137 x 226 x 249 mm with the striker at rest
(about 300 mm tall with the hammer parked), and the printed parts weigh
about 376 g solid (305 g with the old striker, 349 g before the fit fixes).

### Fixings

Lengths come from the plate thicknesses in the model. Self-tap means a
machine or self-tapping screw cut straight into the printed pilot hole.

**Near the sensors, use non-magnetic fixings** (nylon, brass, or at least A2
stainless). Steel within a few cm of a magnetometer shows up as hard- and
soft-iron error.

| Where | Fixing | Qty | Notes |
|---|---|---|---|
| **Sensor stack** | M3 x 50 bolt (tall_top: M3 x 55) | 2 | top deck through both spines to the bottom deck at x = ±14; grip 44.3 mm (tall_top 52.2); **brass or nylon preferred** |
| | M3 nut (+ washers, optional) | 2 | heads under the bottom deck's bosses (bolts go in from below), nuts on the top deck's bosses just outside the roof, slid in under the roof edge. With tall_top's M3 x 55, leave the washers out |
| **QT boards** | M2.5 x 10 screw + nut + 3 mm spacer | 2 per board | top-edge hole pair; **nylon**. 9 boards in PR 839 = 18 sets (capacity 21 boards = 42) |
| **FeatherWing** | M2.5 x 10 standoff (F-F) + 2 x M2.5 x 6 screws + nut | 4 | nylon; 10 mm clears male headers |
| **Roll axis** | MG90S horn: supplied centre screw + 2 x M2 x 6 self-tap (arm) | 1 set | middle deck +X wall (4 mm behind the pocket floor) |
| | 623ZZ bearing + M3 x 12 button head + washer | 1 | tilt ring -X end, screw into the middle deck's -X axle boss |
| | MG90S tab screws: supplied, or 2 x M2 x 8–10 self-tap | 2 | tilt ring servo plate (1.7 mm pilots, 10 mm deep in the bosses) |
| **Tilt axis** | DS3240 horn: supplied centre screw + 2 x M2.5 x 8 self-tap (arm) | 1 set | tilt ring +Y bar |
| | 623ZZ bearing + M3 x 12 button head + washer | 1 | yoke -Y upright, screw into the ring's -Y axle boss |
| | DS3240 tab screws: supplied, or 4 x M2.6/M3 x 10–14 self-tap | 4 | yoke +Y plate (2.4 mm pilots, 12 mm deep in the bosses) |
| **Pan axis** | DS3240 horn: supplied centre screw + 2 x M2.5 x 8 self-tap (arm) | 1 set | yoke underside |
| | DS3240 tab screws: supplied, or 4 x M2.6/M3 x 10–12 self-tap | 4 | base plate (10 mm deep in the bosses) |
| | M3 x 16 (or #4 wood) screws, bench | 4 | base foot corners (3.5 mm holes through 6 mm bosses) |
| **Striker** | M3 x 14 (x 16 at most, with a washer) | 4 | tower to the yoke's idler upright (covers the tilt bearing), as before. Plate 10 mm at the bosses + upright 6.4 mm; longer pokes through toward the tilt ring. Reach the heads through the 7 mm holes in the leaf seat |
| | M3 x 50 + M3 nut | 1 | arm pivot: −X wall, `striker_bushing` (the arm and 2 spacers ride on it), +X wall. The nut drops into the hex pocket on the +X wall; tighten until snug, since the bolt clamps the walls onto the bushing, not the arm |
| | M3 x 20 | 1 | drive pin: through the drive leaf's fork slots, self-tapped into the arm's tail (2.8 mm pilot). Leave the head ~0.3 mm off the fork so the pin slides |
| | M3 x 16 | 4 | drive leaf root pad to the seat (self-tap into the 2.8 mm pilots) |
| | **M3 x 30 nylon** + 2 nylon M3 nuts | 1 | head stop screw: down through the overarm, one nut under the boss and one on top. The lower nut sets how far the screw presses the head leaf (11.7 mm preload). **Nylon:** it hangs right over the top deck |
| | MG90S-size 270° servo tab screws: supplied, or 2 x M2 x 8 self-tap | 2 | the tower's −X wall (servo plate) |
| | `striker_cam`: supplied centre screw, horn trimmed to 14 mm and **CA-glued** into the cam | 1 | the cam has no arm pilots |
| | `striker_cam_spline` instead: M2 x 10 centre screw | 1 | counterbored from the cam's front face |
| | 1.75 mm filament, ~12 mm | 1 | pawl pin; melt or flare the ends |
| **Printed horns** (if used) | MG90S: M2 x 8 centre screw; DS3240: M3 x 10 centre screw | per horn | longer than stock, because the printed hub is thicker |

**Bearings:** 2 x 623ZZ (3 x 10 x 4 mm).

**To order:** see [docs/shopping_list.md](docs/shopping_list.md), with quantities rounded to pack sizes.

**Totals for the main design, with 9 QT boards + wing:**
- **M3:**
  - M3 x 50: 3 (2 stack bolts, 1 striker pivot)
  - M3 x 12: 2 (axles)
  - M3 x 14: 4 (striker tower)
  - M3 x 16: 4 (drive leaf pad)
  - M3 x 20: 1 (drive pin)
  - M3 x 30 nylon + 2 nylon nuts: 1 (head stop screw)
  - M3 nuts: 3 (stack bolts, pivot)
  - M3 washers: ~2 (axles; the stack washers are optional)
  - bench screws (M3 x 16): 4
- **M2.5:**
  - M2.5 x 10: 18 (QT boards)
  - M2.5 x 6: 8 (wing)
  - M2.5 x 8 self-tap: 4 (horn arms)
  - M2.5 nuts: 22
  - 3 mm spacers: 18
  - 10 mm F-F standoffs: 4
- **M2:**
  - M2 x 6 self-tap: 2 (MG90S horn arms)
  - M2 x 8 self-tap: 4 (MG90S tabs, if not supplied)
  - M3 x 12 self-tap: 8 (DS3240 tabs, if not supplied)
- **Servo-supplied:** the horn centre screws and tab screws for 2 x MG90S and
  2 x DS3240.
- **Other:** 2 x 623ZZ bearings and a little 1.75 mm filament.

**Horns: two options, one set of parts.** Every horn pocket works either way.

1. **Stock horn.** The pockets are sized for an assumed double-arm horn:
   - MG90S: 36 x 7 mm, 2 mm deep, hub 2.5 mm
   - DS3240: 46 x 8.5 mm, 2.5 mm deep, hub 3.5 mm

   Each pocket has self-tap pilots 3.5 mm in from the arm tips. If your horn's
   holes land elsewhere, drill your own.
2. **Printed horn.** Print `horn_mg90s_printed` (21T) or `horn_ds3240_printed`
   (25T).
   - **Fit:** shaped to fill its pocket exactly, with a moulded spline socket,
     so it fits whatever the stock horns turn out to be.
   - **Screws:** an M2 (MG90S) or M3 (DS3240) screw through the hub into the
     spline, and M2 / M2.5 arm screws into the pocket pilots.
   - **Printing:** print arm-down, socket up, with fine layers (0.1–0.12 mm,
     a 0.25 mm nozzle if you have one).
   - **Fit tuning:** `SPLINE_CLEAR` in `make_gimbal.py` (0.1 mm radial)
     loosens or tightens the spline.

**The striker cam** has both versions too: `striker_cam` takes a stock horn
trimmed to 14 mm, and `striker_cam_spline` has the 21T socket moulded in
(M2 screw from the front, counterbored).

**Fitting a horn, either way:** screw the horn to the part, press it onto the
spline, then drive the centre screw through the part's access hole.

## Fit fixes after the first test print

![fit-fix changes](docs/fit-fixes-changes.png)

The first print failed in three ways:
- the servos would not go into their plates
- the plates and the tilt ring snapped at the screw pilots when they were
  flexed to get the servos in
- the middle deck's +X (horn) wall snapped along its top edge

Every fix is parametric. The servo-plate values are at the top of
`make_gimbal.py`, and the wall values at the top of `make_sensor_stack.py`.

| Problem | Fix | Parameters |
|---|---|---|
| The cable exit caught on the plate | A wire notch at the spline end of every body cut-out, through the plate and its bosses | `wire_notch` in the `MG90S` / `DS3240` dicts, `MOUSE_EAR_MIN_ARC` |
| The cut-out was too tight | 0.4 mm clearance per side (was 0.25), plus a 1 mm x 45° lead-in chamfer on **both** faces | `SERVO_CUT_CLEAR`, `SERVO_LEADIN` |
| Plates snapped at the pilots | 45° cone bosses on the side away from the tabs double the material round every tab pilot (the seat doesn't move). Bosses were also added at the striker-stand bolts, the stack bolt holes, the bench holes and the yoke's horn-arm pilots | `BOSS_FACTOR`, `BOSS_WALL` |
| The horn wall snapped | The wall is 6 mm thick (4 mm behind the pocket floor, was 2), wider than the horn arm and carried 8 mm above the axis with 45° shoulders. It has a rib and boss behind the hub and a flared root. The pocket has 45° roofs and the access hole is a teardrop, so nothing bridges when it prints walls-up | `WALL_T`, `WALL_ABOVE`, `RIB_*`, `ROOT_FLARE` |
| Thin bearing lips | Both 623 lips are 2.4 mm (were 2.0) | `RING_BRG_PLATE_T`, `YOKE_BRG_T` |
| Heavy parts | Windows in the low-stress webs of the ring, yoke, base and stand, with 45° tops so they print without support | `MAX_BRIDGE` |

**Wire notches and open-sided (mouse-ear) pilots.** A screw pilot sits right
where the cable leaves each servo, so the notch opens one side of that pilot
instead of leaving a paper-thin wall. At least 240° of wall stays round each
opened pilot (measured at mid-wall), and each one sits in a boss twice the
plate thickness.

| Plate | Servo | Notch | Pilots opened into the notch |
|---|---|---|---|
| `tilt_ring_mg90s` (roll) | MG90S | 6.0 mm wide, 1.1 mm past the cut-out (1.5 mm past the body end) | the one spline-end pilot (it's on the centreline) |
| `striker_stand` (cam) | MG90S | same | same |
| `pan_yoke` (tilt) | DS3240 | 8.0 mm wide, out to the tab end (7 mm past the body) | both spline-end pilots, inner side |
| `base` (pan) | DS3240 | same | same |

The MG90S notch is short because the spline-end screw sits on the centreline,
only ~1 mm past the body. If the cable still catches, there are two options:
- put the servo in from the other face (both faces are chamfered)
- lower `MOUSE_EAR_MIN_ARC` and give `wire_notch` a depth in mm. The build
  refuses a notch that opens a pilot past the limit.

**Which face each servo goes in from.** Turn each servo so the spline end (the
cable end) lines up with the notch.

| Servo | Plate | Goes in from | |
|---|---|---|---|
| roll MG90S | tilt ring, +X plate | inside the ring (the tab face), bottom first | the outer face also works (chamfered) |
| tilt DS3240 | yoke, +Y upright | the inner face (toward the ring), bottom first | tab face only |
| pan DS3240 | base, top plate | from above, bottom first | tab face only |
| cam MG90S | striker tower, −X wall | from inside the tower (the tab face), bottom first, before the leaf and cam go in; spline end (cable) up | the other face also works (chamfered) |

**Volume**, for the parts that were lightened or thickened:

| Part | Before (cm³) | After (cm³) | Change |
|---|---|---|---|
| `tilt_ring_mg90s` | 49.2 | 40.7 | −17 % |
| `pan_yoke` | 74.7 | 64.5 | −14 % |
| `base` | 54.9 | 36.0 | −34 % |
| `striker_stand` | 27.2 | 24.2 | −11 % |
| **those four** | **206.0** | **165.4** | **−20 %** |
| `stack_middle_deck` | 21.3 | 27.7 | +30 % (stronger walls) |
| `stack_top_deck` / `stack_bottom_deck` | 16.4 / 11.8 | 16.6 / 12.1 | bolt bosses |

**Envelope:** each middle-deck wall is 2 mm thicker. That makes the ring 4 mm
longer, puts the roll servo 2 mm further out and drops the yoke floor 2 mm.
The rig is now about 137 x 236 x 194 mm (was 132 x 236 x 192). The tilt range
with the striker fitted is unchanged.

**Not changed:**
- The tilt ring still needs supports: it's a closed frame, and its bars float
  above the bed.
- The tilt ring's DS3240 horn-arm pilots keep 3.5 mm behind the pocket floor.
  There's no room to thicken them: the horn is on the outside and the roll
  sweep margin is on the inside.
- The striker-stand pilots in the yoke's idler upright stay at the upright's
  6.4 mm. Only 1.5 mm separates it from the ring, so the stand side got the
  bosses instead.

## Print list

![assembly](docs/assembly-3d.png)

All the STLs are in `parts/`. PETG is recommended throughout because it takes
the taps well. **The striker arm and drive leaf must be PETG.**

| STL | Qty | Nozzle | Orientation on the bed | Notes |
|---|---|---|---|---|
| `stack_bottom_deck` | 1 | 0.4 preferred | flat, **bolt bosses up** (upside down) | slots must pass M2.5; on 0.6, regenerate with `SWIRLY_SLOT=2.95` |
| `stack_middle_deck` | 1 | 0.4 preferred | flat, end walls up | carries the roll horn and axle. The horn pocket has 45° roofs and the access hole is a teardrop, so no supports; slot note as above |
| `stack_top_deck` | 1 | 0.4 preferred | flat, roof up | the roof flare is 45°, so no supports; slot note as above |
| `stack_spine_lower` | 1 | 0.6 | on its 36 x 7 face | **11.1 mm** tall (bottom ↔ middle deck) |
| `stack_spine_upper` | 1 | 0.6 | on its 36 x 7 face | **17.1 mm** tall (middle ↔ top deck); 25 mm in `variants/tall_top` |
| `tilt_ring_mg90s` | 1 | 0.6 | flat | **still needs supports** under the side bars and bearing plate (they sit ~10–15 mm up; it's a closed frame). Windows, bosses and pocket roofs are all 45°. Drill the 1.7 mm servo pilots |
| `pan_yoke` | 1 | 0.6 | on its side (32 mm face down) | the U profile lies flat, so no supports; windows have 45° tops (either side down); ream the 623 pocket if tight |
| `base` | 1 | 0.6 | on its closed end wall | open box, no supports; windows have 45° tops |
| `striker_stand` | 1 | 0.6 | **upright, floor down** (as it sits on the yoke) | the tower: walls stand vertical, the pivot bosses are downward teardrops and the rest stops have 45° undersides, so no supports. Windows have 45° tops. Drill the small pilots |
| `striker_arm` | 1 | 0.4 preferred | **on its side** (the 12 mm thick profile flat) | **PETG**, 100 % infill. The head leaf is 1.36 mm thick (its stiffness goes as thickness cubed; the stop screw trims the preload) |
| `striker_spring` | 1 | 0.4 preferred | **flat on its plain (+Y) face**, lugs and fork up | **PETG**, 100 % infill. The drive leaf: its 2.01 mm thickness sets the drive force (thickness cubed) |
| `striker_bushing` | 1 | 0.4 | standing on end | 6 mm tube for the M3 pivot bolt |
| `striker_spacer` | 2 | 0.4 | standing on end | either side of the arm's hub |
| `striker_pawl` | 1 | **0.4 needed** | flat on a 4.4 mm face | 1.9 mm pin hole must swing freely; pin is 1.75 mm filament. **Reprint it:** it is 14 mm long now (was 9) |
| `striker_cam` | 1 | 0.4 preferred | flat, **horn pocket up** | takes a stock horn trimmed to 14 mm; a crisp cliff edge gives a clean drop |
| `striker_cam_spline` | (alt.) | **0.4 needed** | counterbored face down, hub up | 21T socket moulded in; use instead of `striker_cam` |
| `horn_mg90s_printed` | 0–1 | **0.4 needed** (0.25 better) | arm down, socket up | 0.3 mm deep spline teeth; only if the stock roll horn doesn't fit |
| `horn_ds3240_printed` | 0–2 | **0.4 needed** | arm down, socket up | only if the stock tilt/pan horns don't fit |

**Nozzle key:**
- **0.6:** resolution doesn't matter. Thicker lines make these stronger and
  faster; drill or ream the small holes.
- **0.4 preferred:** fits or force depend on accuracy. A 0.6 works with the
  adjustment noted.
- **0.4 needed:** fine features (splines, the pawl pin) that a 0.6 smears.

The core set is 15 parts (16 prints with the second spacer), plus the optional printed horns. Print the horns
with fine layers (0.1–0.12 mm). Print one horn first and test it on a servo
before printing the others (`SPLINE_CLEAR`).

**Spine names were swapped before 2026-10-10.** Older exports of
`stack_spine_lower` were really the 17.1 mm upper spine, and vice versa.
Check the height before using an older print.

## Assembly order

Centre every servo (power it at mid travel) before you fit its horn. Each
horn goes on the same way: screw the horn to the part, press it onto the
spline, then drive the centre screw through the part's access hole.

1. **Test-fit each servo in its plate** before anything else. Put it in
   bottom first, from the face in the table above, with the cable end at the
   notch. Drill the pilots if they're tight.
2. **Base:** drop the pan DS3240 into the base from above. The cable leaves
   through the open +X end. Fit the 4 tab screws, then screw the base to the
   bench.
3. **Yoke:**
   - Put the tilt DS3240 into the +Y upright from the inside (bottom first,
     outward). Fit its 4 tab screws.
   - Press a 623 into the -Y upright from the outside.
   - Screw the pan horn into the underside pocket and press it onto the pan
     spline. Drive its centre screw down through the floor's access hole now,
     while nothing is above it.
4. **Tilt ring:**
   - Put the roll MG90S into the +X plate from inside the ring. Fit its 2 tab
     screws.
   - Press a 623 into the -X bearing plate from the outside.
   - Screw the tilt horn into the +Y bar's pocket.
5. **Ring into the yoke, while the ring is still empty:**
   - Press the tilt horn onto the tilt spline and fit its centre screw through
     the bar's access hole. The driver comes from inside the ring along the
     tilt axis, where the stack's spines will be later.
   - At -Y, fit an M3 x 12 + washer through the yoke's 623 into the ring's
     axle boss.
   - The axle boss sits 2.4 mm into the bearing's lip hole, and the horn needs
     about 4 mm of travel onto the spline, so spring the uprights apart a
     little (or angle the ring in) to get both on.
6. **Boards and the middle deck:**
   - Fit the boards to the decks on the bench (nuts go under each deck).
   - Screw the roll horn to the middle deck's +X wall.
   - Lower the middle deck on its own into the ring. Press the roll horn onto
     the MG90S spline and fit its centre screw through the wall's access hole,
     along the roll axis from inside. Once the upper spine is in, it blocks
     that axis.
   - At the -X end, fit an M3 x 12 + washer through the ring's 623 into the
     middle deck's axle boss.
7. **Rest of the stack, in place:**
   - Slide the lower spine and bottom deck (bosses down) in under the ring
     from the side. Put the upper spine and top deck (bosses up) on from
     above.
   - Clamp them with the two M3 bolts, up from under the bottom deck. Below
     the bottom deck there is 54 mm down to the yoke floor, for 53 mm of bolt
     plus head. It's easier to tilt the ring 90° and put them in from the
     side. For `tall_top` (M3 x 55) you have to do it that way. Slide the
     washers and nuts in under the roof edge onto the top deck's bosses.
8. **Striker** (the tower goes on the yoke before the leaf goes in):
   - Put the cam MG90S into the tower's −X wall from inside, bottom first,
     spline end (cable) up. Fit its 2 tab screws.
   - Bolt the tower to the yoke's -Y upright (4 x M3 x 14, reached through
     the holes in the leaf seat). This covers the tilt axle screw, so that
     goes in first. Do it before the drive leaf: the leaf's root pad covers
     the upper two access holes.
   - On the bench, pin the pawl to the drive leaf's lugs with filament. It
     should swing freely and fall back onto its stop.
   - On the bench, bend the leaf's fork onto the arm's tail (the leaf leans
     back about 5.5° when free). Drive the M3 x 20 pin through one fork slot,
     self-tap it through the tail and out through the other slot. Leave the
     head ~0.3 mm clear, so the pin can slide. Do this before the arm goes
     between the walls: there the fork has only 10 mm beside it for a 20 mm
     pin.
   - Fit the nylon stop screw down through the overarm with a nut under the
     boss. Turn it until the head leaf is pressed down 11.7 mm (the head's
     face is level), then lock it with the top nut.
   - Put the bushing through the arm's hub, with a spacer either side. Lower
     the arm, with the leaf hanging from it, between the walls and push the
     M3 x 50 through the −X wall, the bushing and the +X wall, into the nut in
     its pocket. Tighten until snug. The arm should swing freely.
   - Screw the drive leaf's root pad to the seat (4 x M3 x 16), lugs and
     pawl toward the servo.
   - Fit the cam with the servo at 0°: the pawl tip just clears the base
     circle, 45° up from the cam's +Y side.
   - Check the hover gap: the head's face should sit 5 mm above the roof. To
     trim it, shim the rest-stop lip, where the tail lands.

The animated assembly (below) follows this order. Its access check found
the four changes from the earlier order: the ring goes into the yoke empty,
the stack is finished in place, the stack bolts go in from below, and the
tower goes on before the leaf.

## Assembly animation

![assembly animation](docs/assembly/assembly-animation.gif)

`docs/assembly/imu-gimbal-assembly-animated.FCStd` is the whole rig with
every fixing modelled: 119 fixings among 153 objects, in 25 steps, following
the assembly order above. The fixing count includes the optional bench
screws, the 9 PR 839 boards and the wing.

**What's in the document:**
- **Groups:** an `Assembly steps` group holds one group per step, named
  `NN - title`.
- **Spreadsheet `Steps`:** one row per step, with the title, parts,
  fixings (spec x qty), tools, each fixing's insertion direction and
  distance, and notes. This includes the face each servo goes in from.
- **Spreadsheet `Insertions`:** one row per object, with its full insertion
  path.
- **Object properties:** `Spec` (for example "M3x50 socket cap, A2"),
  `Material`, `Qty`, `Step`, `StepTitle`, `InsertVector`, `InsertDistance`,
  `IntendedColor` and `AnimMoves`.
- **Colours:** printed parts are colour-coded as in the renders. Fixings are
  coloured by material: steel / A2 grey, nylon off-white, brass gold.
  Documents built headless store the colours in `IntendedColor`, and the
  macro applies them.

**Playing it:**
1. In FreeCAD 1.1, open **Macro > Macros...** and point it at
   `docs/assembly/`.
2. Run `animate_assembly.FCMacro`. It opens a temporary copy of the
   document, so the saved file is never changed. If the document is already
   open, the macro uses it and puts every placement, visibility and colour
   back when you stop.
3. Everything starts hidden. Each step's parts and fixings fade in and slide
   along their insertion paths. Sub-assemblies (yoke, ring, decks, tower,
   arm) are built off to one side and then carried home.
4. The current step is in full colour and earlier steps are dimmed. The step
   title shows in the dialog, the status bar, the report view and the
   top-left of the 3D view.

**Controls:**
- The dialog has `|<`, `<`, Play/Pause, `>`, `>|`, a step list
  (double-click to jump) and a speed slider (0.25x to 16x).
- **Stop and restore** closes the dialog and puts the document back.
- From the Python console: `AssemblyAnim.goto(n)`, `.play()`, `.pause()`,
  `.next()`, `.prev()`, `.speed = 2`, `.stop()`.

There is one PNG per step in `docs/assembly/steps/NN.png`. Each shows that
step's objects in colour and everything else in grey.

**Rebuild** (about 2 minutes for the geometry, plus a few minutes for the
checks and the GIF):

```
freecadcmd -c "exec(open('make_assembly_animation.py').read(), {'__file__': 'make_assembly_animation.py', '__name__': '__main__'})"
```

Options are environment variables:
- `ASSY_VARIANT=tall_top` builds the variant. It writes
  `variants/tall_top/docs/assembly/`, with M3 x 55 stack bolts and no stack
  washers.
- `ASSY_BOARDS=all` fills all 21 QT spots.
- `ASSY_NO_BENCH=1` leaves out the bench screws.
- `ASSY_RENDER=0`, `ASSY_ACCESS=0` and `ASSY_WRITE=0` skip the GIF, the
  access check and the document.
- `ASSY_CACHE=file.json` reuses the generated geometry.

To check the macro without a GUI, run a dry run. It plays every frame of every
step and checks that the placements end exactly on the saved ones and are
restored afterwards:

```
freecadcmd -c "exec(open('docs/assembly/animate_assembly.FCMacro').read(), {'__name__': '__main__', 'DRY_RUN': True, '__file__': 'docs/assembly/animate_assembly.FCMacro'})"
```

**How the fixings follow the model:** each generator records a fixing point
wherever it cuts a fixing hole: its seat, axis, hole size and depth
(`make_gimbal.POINTS`, `make_sensor_stack.board_layout()` and
`stack_bolt_points()`, `make_striker.fixing_points()`).
`assembly_plan.py` puts each fixing on its point, so the fixings move with
any parameter change. The stack bolt length is picked from the grip.

**Checks** (`assembly_checks.py`, printed on every build):
- (a) counts by spec against the fixings table above
- (b) each screw is coaxial with its recorded point and with the real hole
  face found in the part. Slots are checked against the slot centreline.
- (c) each head, washer, nut, spacer and bearing seats on its face (gap
  < 0.5 mm, and not sunk into it)
- (d) no shank cuts into printed parts outside its own hole (< 1 mm³)
- (e) access: each fixing's insertion path, and a straight driver or key
  behind its head, against everything already in place at that moment of
  the animation

| # | Step | Fixings |
|---|---|---|
| 1 | Pan servo into the base | 4 x M3 x 12 self-tap (tabs) |
| 2 | Base to the bench (optional) | 4 x M3 x 16 pan head (or #4 wood screws) |
| 3 | Tilt servo into the yoke | 4 x M3 x 12 self-tap |
| 4 | 623 bearing into the yoke | 623ZZ |
| 5 | Pan horn onto the yoke | 2 x M2.5 x 8 self-tap |
| 6 | Yoke onto the pan spline | horn centre screw (supplied; M3 x 10 for a printed horn) |
| 7 | Roll servo into the tilt ring | 2 x M2 x 8 self-tap |
| 8 | 623 bearing into the tilt ring | 623ZZ |
| 9 | Tilt horn onto the ring | 2 x M2.5 x 8 self-tap |
| 10 | Empty ring into the yoke (tilt spline) | horn centre screw (supplied; M3 x 10 printed) |
| 11 | Tilt idler axle | M3 x 12 button head + washer |
| 12 | Boards onto the bottom deck | 4 boards: 8 x (M2.5 x 10 nylon + 3 mm spacer + nut) |
| 13 | Wing and boards onto the middle deck | 4 x 10 mm F-F standoff + 8 x M2.5 x 6; 5 boards: 10 x (M2.5 x 10 + spacer + nut) |
| 14 | Roll horn onto the middle deck | 2 x M2 x 6 self-tap |
| 15 | Middle deck into the ring (roll spline) | horn centre screw (supplied; M2 x 8 printed) |
| 16 | Roll idler axle | M3 x 12 button head + washer |
| 17 | Stack and clamp the decks | 2 x M3 x 50 (up from below) + 2 washers (optional) + 2 nuts |
| 18 | Cam servo into the striker tower | 2 x M2 x 8 self-tap |
| 19 | Tower onto the yoke | 4 x M3 x 14 |
| 20 | Pawl onto the drive leaf | 1.75 mm filament pin |
| 21 | Drive leaf pinned to the arm | M3 x 20 (drive pin) |
| 22 | Head stop screw | M3 x 30 nylon + 2 nylon nuts |
| 23 | Arm and leaf into the tower | M3 x 50 + nut (bushing and 2 spacers on it) |
| 24 | Drive leaf root pad to the seat | 4 x M3 x 16 |
| 25 | Cam onto the servo | centre screw (supplied), horn CA-glued in the cam |

**Check results** (main design, 9 boards + wing):
- **Counts:** match the fixings table, except the M2.5 nuts. The model
  uses 18, not 22: the wing's F-F standoffs take a screw at each end, so
  its 4 "+ nut" are not needed.
- **Coaxial:** worst 0.10 mm. That's the servo tab pilots, which are short
  slots.
- **Seats:** worst gap 0.30 mm, the drive pin's designed clearance.
- **Shanks:** worst 0.10 mm³ (the stop screw's tip on the head leaf). The
  pan-horn arm screws' self-tapping points reach 0.5 mm past the 5 mm pilots
  (0.09 mm³).
- **Access:** clear for every fixing in this order, except the bench screws.
  A straight driver can't reach them; see step 2.

**Things to check by hand:**
- **Tilt horn screws:** the arm screws on the tilt ring poke 2.0 mm through
  the +Y bar into the ring. That leaves about 2 mm to the roll sweep, so an
  M2.5 x 6 is safer there.
- **Top deck under the roof:** with `ASSY_BOARDS=all`, two top-deck spots
  (`top_QT_low3`, `top_QT_high3`, at x = +10.16) have one screw under the
  roof's overhang. The screw can't go in straight, and a driver can't reach
  it. That's why the default boards go on the bottom deck. If you use those
  spots, angle the screw in and turn the nut from below.
- **`tall_top` stack bolts:** an M3 x 55 bolt can't go up from below with the
  ring in the yoke (58 mm needed, 54 mm free). Tilt the ring 90° and put the
  bolts in from the side.
- **Idler bosses:** both sit 2.4 mm into their bearing's lip hole, and both
  horns need 3.5–4 mm of travel onto their splines. The ring (roll) and the
  yoke uprights (tilt) have to spring apart slightly for the press-on.

## Live design and motion

![motion](docs/motion/motion.gif)

`docs/motion/imu-gimbal-motion.FCStd` is the whole rig as a jointed
assembly (FreeCAD 1.1 Assembly workbench, not Assembly4), with every
printed part, reference servo, board and fixing from the animated assembly
above, on the same fixing points and in the same colours.

**What's in the document:**
- **Components:** one `App::Part` per moving body. Each holds its parts and
  the fixings attached to it:

  | Component | Holds |
  |---|---|
  | `Body_Base` (grounded) | base, pan servo, tab and bench screws |
  | `Body_Yoke` | pan yoke, tilt servo, 623, pan horn and its screws |
  | `Body_Tower` | striker tower, cam servo, tower bolts, pivot bolt, nut, bushing, spacers |
  | `Body_Ring` | tilt ring, roll servo, 623, tilt horn, tilt axle screw and washer |
  | `Body_Stack` | decks, spines, boards, board fixings, stack bolts, roll horn, roll axle screw |
  | `Body_Cam` | cam, glued horn, its centre screw |
  | `Body_Arm` | hammer arm, drive pin, nylon stop screw and nuts |
  | `Body_Leaf` | drive leaf, pad screws, pawl pin |
  | `Body_Pawl` | pawl |

- **Joints** (`Joints` group):

  | Joint | Type | Between | Axis | Limits |
  |---|---|---|---|---|
  | Pan | Revolute | base, yoke | Z through the gimbal centre | ±135° (DS3240, 270°) |
  | Tilt | Revolute | yoke, ring | Y through the centre | none: 360° (striker parked) |
  | Roll | Revolute | ring, stack | X through the centre | ±90° (MG90S) |
  | Cam | Revolute | tower, cam | −X through the cam centre (+ = forward) | 0–270° |
  | Hammer arm pivot | Revolute | tower, arm | +X through the pivot (+ = hammer up) | 0–50° |
  | Pawl pin | Revolute | leaf, pawl | −X through the pin (+ = fold) | 0–120° |
  | Tower | Fixed | yoke, tower | | |
  | Leaf root | Fixed | tower, leaf | | |

- **Simulations** (`Simulations` group): 1 pan sweep, 2 tilt 360°, 3 roll
  ±90° (each with the hammer parked first), 4 the striker cam cycle.
- **`MotionData`:** the sampled motion timeline and the striker model's
  numbers (JSON properties), and the striker's flex states (hidden).

**Using it:**
1. Open the document in FreeCAD 1.1 and switch to the Assembly workbench.
2. **Drag:** double-click the assembly to edit it, then drag any moving body.
   The solver keeps it on its joint. Pan, tilt and roll turn the gimbal; the
   cam and the hammer arm turn on their own axes; the pawl folds on its pin.
3. **Simulation:** double-click a simulation under `Simulations`, then press
   **Run kinematics** and use the player (play, step, frame slider).
4. **Mechanism animation:** **Macro > Macros...**, point it at `docs/motion/`
   and run `animate_motion.FCMacro`. It opens a temporary copy of the
   document (or uses the open one and puts it back when you stop), and plays
   the programme below. The dialog has `|<`, `<`, Play/Pause, `>`, `>|`, a
   time slider to scrub, a section list (double-click to jump) and a speed
   slider (0.1x to 4x). **Stop and restore** closes it. The caption (time,
   pan, tilt, roll, cam, arm, pawl fold, head-leaf bend, and the slow-motion
   factor) shows in the dialog, the status bar and the 3D view. From the
   Python console: `MotionAnim.goto(21.9)`, `.play()`, `.pause()`,
   `.speed = 0.5`, `.stop()`.

**The programme** (38.2 s):
1. Park the hammer (cam 0 → 120°), then pan 0 → +135 → −135 → 0. The pan
   servo is a 270° DS3240, so ±135° is its whole travel; a full 360° of pan
   isn't possible.
2. Tilt a full 360° with the hammer parked.
3. Roll +90 → −90 → 0.
4. Striker: re-arm (cam back to 0), park (120°), single tap (124 → 134°),
   re-arm (the cliff folds the pawl), park, double tap (124 → 134°, lobe 2
   lifts the arm again, 244 → 254°), and back to rest (both cliffs fold the
   pawl).

![striker cycle](docs/motion/striker-cycle.png)

**Computed, not keyframed** (`striker_motion.py`, from `make_striker`'s
model):
- **Cam → arm:** at each cam angle, the arm angle is the smallest one at
  which the pawl clears the cam profile (`cam_radius()` and `LOBES`). The
  pawl rides on the drive leaf, bent to match that arm angle by the same
  cantilever model `solve()` uses. The pawl is tested as a capsule (pin to
  tip, 5 mm wide) against the cam's radius at each point of its outline. So
  ramps, dwell tops and the roll-off over each cliff's corner come out of
  the geometry. Parked: 47.98°; cam force 14.5 N; leaf 1.71 N at rest and
  5.16 N parked at the pin (as the striker section above).
- **Reverse stroke:** a cam turning backwards can't lift the leaf, so the
  arm never rises. The pawl folds about its pin just enough to clear the
  cam, up to 51.9° over lobe 1, then drops back.
- **Release:** the servo steps from 124° to 134° at 600°/s (MG90S no-load
  speed, assumed). The pawl rolls off the cliff's corner for the first 2.2 ms,
  then the leaf's force k(d_rest + dy) and gravity throw the arm onto its
  rest stop. RK4 on the arm's inertia (arm plus the lumped leaf tip, as
  `solve()`) gives 50.3 ms to the stop, 33.0 rad/s, 55.0 mJ (the design's
  54.7 mJ).
- **Let-off:** the head's share, 36.2 mJ (design 36), carries it on against
  the preloaded head leaf (P + kx). It reaches the roof 1.6 ms after the arm
  stops, with 28.0 mJ (design 27.5). It rebounds (coefficient of
  restitution 0.4, assumed) and is back on its stop screw 4.6 ms after the
  arm stops.
- **Flexing leaves:** shapes are swapped, not morphed:
  - The drive leaf is swapped for the shape solved at the nearest whole
    degree of arm angle (50 shapes, 0–49°).
  - The hammer arm is swapped for the one with its head leaf bent the
    nearest 0.5 mm (11 shapes, 0–5 mm).
  - The pawl and its pin follow the leaf's own station at the lugs exactly
    (0.25° table).
  - The as-printed shapes are in `MotionData` too.
- **Rigid approximations:**
  - The native Simulations (rigid bodies only) leave both leaves in their
    rest shapes.
  - In the let-off shapes the head drops level while its leaf bends further.
    A real tip-loaded leaf would also tip the head by up to 8.6° (1.5 x 5 mm
    / 50 mm) at the roof.
  - The drop happens in a few tens of milliseconds, so the video and the
    macro show each release in slow motion (33–39x). The caption gives the
    physical time.

**Native Simulation limits (FreeCAD 1.1):**
- There's no cam joint, so the cam can't drive the arm in the solver. The
  arm and pawl Simulations replay the cam model's values as motions.
- Motion formulas accept only `+ - * / ^`, `sin` and `cos`. The `arctan`
  "smooth step" examples in the Simulation help don't parse, and the run
  generates no frames.
- A constant motion far from the start pose sends the solver to the wrong
  branch (120° came out as 60°).
- So each joint is driven by a sine series fitted to the timeline (straight
  line plus up to 160 sine terms). Each Simulation starts from the saved
  rest pose, and its fit error per joint is in its `FitJSON` property:

  | Joint | Worst fit error |
  |---|---|
  | pan, tilt, roll | 0.26° |
  | cam | 0.39° |
  | arm | 0.92° |
  | pawl fold | 15.3° |

  The pawl's quick folds over the cliffs get smoothed. For the exact
  motion, use the macro.
- Joint limits are stored and shown, but `solve()` does not enforce them:
  the solver kept a 30° pose on a ±10° joint in a headless test.

**Checks** (headless):
- `make_motion.py` (`verify-joints.json`):
  - Forward-kinematics poses (all six joints at several angles) are kept by
    the solver to 7.6e-14 mm.
  - Poses wobbled off their joints by up to 1.5° and 0.4 mm are pulled back
    to pure rotations about the right axes through the right points, within
    6.3e-12 mm.
  - The four Simulations run headless (242, 242, 182 and 626 frames), and
    every joint matches its formula to within 5e-8°.
- `check_motion.py` (`check-motion.json`): the macro's dry run over all
  1147 frames (30 fps), plus:
  - **Arm angle:**
    - The angle shown equals the timeline to 1.3e-11°.
    - Recomputed from the cam angle by the model at every frame (not from
      the stored timeline), it agrees to 1.7e-4°.
    - During the drops it matches a fresh drop simulation to 8.9e-5°.
    - The pawl fold agrees to 8.8e-4°.
    - The pawl is never inside the cam.
  - **Restore:** the document is put back exactly (placements and shapes).
  - **Collisions:** every pair of objects on bodies in relative motion was
    checked (11,689 pair checks, 4,258 measured) — every 10th frame while
    the gimbal moves, every 3rd while the cam turns, every 2nd in a release.
    There are no collisions and no unplanned contacts. The closest
    unplanned approach is 0.85 mm: the ring against the yoke upright and
    the middle deck, an axial gap that doesn't change with the angle.
  - **Designed contacts:** pawl on cam (worst overlap 0.93 mm³, from the
    sampled pawl outline), arm on its rest stop (touching), and head on
    the roof at the tap. The last two are checked at the exact instants
    of all three taps, not on the frame grid: the arm reaches its stop at
    50.3 ms and the head reaches the roof at 51.8 ms, each at 0.000 mm
    with 0 mm³ of overlap.
  - **Parked tilt (360°) and roll (±90°), every frame:** the striker keeps
    **7.90 mm** from everything that tilts or rolls (need ≥ 5;
    `striker_limits()` gives 7.9). Two things are reported apart:
    - The four tower bolts end flush with the yoke upright's inner face,
      so their 3.9 mm to the ring is the yoke's own gap. The upright itself
      passes the ring at 0.85 mm.
    - The tilt axle screw and washer only spin inside the yoke's bearing,
      1.0 mm from the tower's screw-head pocket.
  - **Drive pin in the leaf's slot: up to 27 mm³ of overlap when parked.**
    `make_striker.solve()` sizes the leaf so its tip centreline moves
    with the pin. But the pin rides FORK_T / 2 off that centreline and
    6.8 mm up the slot, and the tip tilts 11.6° by park. So the leaf shape
    it solves puts the slot 1.8 mm off the pin at 48° (0.2 mm clearance;
    0.3 mm off at 24°). The real leaf will bend a little differently
    (the pin sets the slot), with slightly more force when parked. Worth
    fixing in `leaf_design()` / `DriveLeaf`.

**Rebuild:**

```
freecadcmd -c "exec(open('make_motion.py').read(), {'__file__': 'make_motion.py', '__name__': '__main__'})"
freecadcmd -c "exec(open('check_motion.py').read(), {'__file__': 'check_motion.py', '__name__': '__main__'})"
freecadcmd -c "exec(open('render_motion.py').read(), {'__file__': 'render_motion.py', '__name__': '__main__'})"
```

`make_motion.py` takes about 10 minutes (`ASSY_CACHE=file.json` reuses the
generator geometry), and the checks and the render take longer.
`render_motion.py` writes `motion.gif`, `motion.mp4` (when `ffmpeg` is on the
PATH) and `striker-cycle.png`. The macro's own dry run plays every frame and
checks that the arm angle shown is the timeline's and that the document is
restored exactly:

```
freecadcmd -c "exec(open('docs/motion/animate_motion.FCMacro').read(), {'__name__': '__main__', 'DRY_RUN': True, '__file__': 'docs/motion/animate_motion.FCMacro'})"
```

**After a rebuild, finish it in the GUI.** A headless build has no view
providers for its joints, Simulations and motions. Without them:
- the joints have no markers;
- double-clicking a Simulation doesn't open its panel;
- the parts are grey and everything reopens hidden.

To fix it, open the document and run this in the Python console:

```
import sys; sys.path.insert(0, r"C:\path\to\imu_gimbal_rig")
import make_motion; make_motion.finish_gui(App.ActiveDocument)
```

That attaches the view providers, applies the colours from
`IntendedColor`, shows the rig and saves. The committed document has
already been through this. The macro shows and colours the rig by itself
either way.


## Variants

| Folder | What's different |
|---|---|
| [`variants/tall_top`](variants/tall_top/README.md) | Upper spine 25 mm. The top deck, roof and striker sit 7.9 mm higher. Only `stack_spine_upper` and `striker_stand` are new prints. |

## Files

| File | What |
| --- | --- |
| `swirly_grid.py` | Pure-Python swirly-grid geometry + hole-pattern fit checker (`python swirly_grid.py 2 4`) |
| `pack_faces.py` | Portrait QT board placement options on a swirly grid |
| `make_sensor_stack.py` | Sensor stack decks, spines, roof, board layout, reference board envelopes |
| `make_striker.py` | Striker: drive-leaf and head-leaf spring models, hammer arm (rest / parked / as printed), pawl, cam, tower |
| `make_gimbal.py` | Full gimbal + striker, per-part STEP/STL in `parts/`, clearance checks, `imu-gimbal-assembly.FCStd`. `geometry()` builds everything without writing, and records every fixing point (`POINTS`) where a hole is cut |
| `fixings.py` | Simple fixing models (Part API): socket cap / button / pan / self-tap screws, nuts, washers, spacers, F-F and M-F standoffs, 623ZZ, filament pin |
| `assembly_plan.py` | Assembly steps, every fixing placed on the generators' fixing points, insertion paths |
| `assembly_checks.py` | Fixing checks: counts vs this README, coaxial, seated, shank clearance, access on the animation timeline |
| `make_assembly_animation.py` | Builds `docs/assembly/imu-gimbal-assembly-animated.FCStd`, runs the checks, renders the GIF and step PNGs (`render_assembly_animation.py`) |
| `docs/assembly/animate_assembly.FCMacro` | Plays the animated assembly in the FreeCAD GUI (and a headless dry run) |
| `striker_motion.py` | Striker kinematics and dynamics from `make_striker`'s model: cam → pawl → leaf → arm, reverse-stroke pawl fold, drop and let-off; the motion programme and its timeline |
| `make_motion.py` | Builds `docs/motion/imu-gimbal-motion.FCStd` (components, joints, Simulations, flex states, timeline) and verifies the joints and Simulations headless |
| `docs/motion/animate_motion.FCMacro` | Plays the mechanism animation in the FreeCAD GUI (and a headless dry run) |
| `check_motion.py` | Motion checks: arm angle vs the cam model at every frame, collisions, parked-tilt clearance, restore |
| `render_motion.py` | `docs/motion/motion.gif` / `.mp4` and `striker-cycle.png` |
| `make_swirly_plate.py` | Flat printable swirly plate (env `SWIRLY_ROWS`, `SWIRLY_COLS`, `SWIRLY_SLOT`) |
| `make_carrier.py` | Older standalone flat plate with Feather bosses (not used by the gimbal) |
| `preview_swirly.py` | Matplotlib 2D preview |
| `docs/mechanical_data.md` | Board hole/IC positions for the PR 839 sensors, Feather spec, servo dimensions |

Generate with FreeCAD 1.1 (`C:\dev\software\FreeCAD\bin\freecadcmd.exe`):

```
freecadcmd -c "exec(open('make_gimbal.py').read(), {'__file__': 'make_gimbal.py', '__name__': '__main__'})"
```

## Notes

- A 1.0" x 0.7" STEMMA QT board (holes 0.8" x 0.5" apart) cannot land all four
  screws in the swirly grid at once. Three holes fit in 32 placements on a
  3x3 grid, and two holes fit in hundreds. Plan on 2–3 screws per board.
- Slots print at 2.75 mm (nominal 2.54 mm) so M2.5 screws pass on FDM.
