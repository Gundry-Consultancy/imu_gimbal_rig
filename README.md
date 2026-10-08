# IMU gimbal rig

Pan / tilt / roll rig that carries a whole set of Adafruit IMU, accelerometer
and magnetometer breakouts (3/6/9 DoF) at once, with a later add-on striker
for tap and double-tap detection testing.

Target sensors: the drivers added in
[adafruit/Adafruit_Wippersnapper_Arduino#839](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/pull/839)
(ISM330DHCX, ISM330DLC, LIS2MDL, LIS3DH, LIS3MDL, LSM303AGR, LSM303DLH,
LSM6DS3, LSM6DSO32, LSM9DS1). Mostly STEMMA QT breakouts, the odd legacy board,
and one FeatherWing.

## Requirements

- Every sensor is mounted at the same time, on swirly-grid plates
  ([tannewt/swirly-grid](https://github.com/tannewt/swirly-grid)) that form a
  box. A swirly-grid PCB also bolts on through the matching slots.
- STEMMA QT sized breakouts, plus a Feather-sized place for one FeatherWing.
- Hobby servos: MG90S (both published tab heights), DS3240, MG995.
- Aim for ±180° on every axis. In practice the servo travel limits it
  (180° or 270° servos).
- Sensors do not need to sit on the rotation centre. Off-axis
  acceleration is accepted.
- Tap / click striker (cam + sprung bar): deferred.

## Sensor box

`make_sensor_box.py` builds a square tube, 81.3 x 55.9 x 55.9 mm. Each of its
four long faces is a 3 x 5 cell swirly grid, and an end cap closes each end.

| Face | Boards |
|---|---|
| top (+Z) | FeatherWing on 10 mm standoffs (clears header pins) + 1 portrait QT |
| bottom, left, right | 4 portrait QT boards each, edge to edge |

That is room for **13 QT boards + 1 FeatherWing**. PR 839 needs 9 breakouts + the wing.

- **Orientation:** boards sit portrait so their QT connectors face the long
  edges of each face, where the cables have room.
- **Fixing:** each board bolts through its top-edge hole pair (every QT board
  has it), with M2.5 screws, 3 mm spacers and nuts inside the tube. Mount the
  boards before fitting the caps.
- **Caps:** each screws to corner posts with 4 x M2.5. The +X cap takes the
  MG90S roll horn and the -X cap has the idler axle.
- **Cables:** both caps have cable windows. The tube has room for a QT Py and
  a TCA9548A mux, which you'll need because of I²C address clashes
  (several LIS3MDL/LSM303/LSM9DS1 parts share addresses).

`pack_faces.py` checks the board placements against the slot geometry.

## Gimbal (`make_gimbal.py`)

Axes: roll = X (the tube axis), tilt = Y, pan = Z, all crossing at the box
centre. The script measures the sweep radius of each stage to size the next
one out, then rotates each stage through 360° against its neighbour and
reports any collision.

| Part | Holds | Servo | Idler |
|---|---|---|---|
| `sensor_tube`, `sensor_cap_drive`, `sensor_cap_idler` | all the boards | MG90S horn pocket on the +X cap | M3 axle boss on the -X cap |
| `tilt_ring_mg90s_tab16` / `_tab21` | roll servo + 623 bearing | MG90S (16 mm and 21 mm tab heights, since the published figures disagree; hole slots cover 27.5–28 mm) | standard-servo horn on the +Y bar, M3 axle on -Y |
| `pan_yoke` | tilt servo + 623 bearing, outer gussets | MG995 or DS3240 on +Y | pan horn pocket underneath |
| `base` | pan servo | DS3240 (270° version for ±135°) | open +X end for cables, 4 x M3 bench holes |

The overall envelope is about 128 x 191 x 180 mm, and the printed parts weigh
about 290 g solid.

**Hardware:**
- 2 x 623ZZ bearings (3 x 10 x 4)
- 2 x M3 x 12 axle screws + washers
- 8 x M2.5 x 8 for the caps
- 2 x M2.5 + nut + 3 mm spacer per QT board
- 4 x M2.5 + 10 mm standoffs for the FeatherWing
- self-tapping screws for the horn arms and servo tabs

**Assumptions to check:**
- Horn arm sizes are guesses. MG90S: 36 x 7 x 2 deep. 25T: 46 x 8.5 x 2.5 deep.
- Horn hub heights are guesses: 2.5 / 3.5 mm.

## Files

| File | What |
| --- | --- |
| `swirly_grid.py` | Pure-Python swirly-grid geometry + hole-pattern fit checker (`python swirly_grid.py 2 4`) |
| `pack_faces.py` | Portrait QT board packing along a swirly face |
| `make_sensor_box.py` | Sensor box tube + caps, board layout, reference board envelopes |
| `make_gimbal.py` | Full gimbal, per-part STEP/STL in `parts/`, clearance sweeps, `imu-gimbal-assembly.FCStd` |
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
