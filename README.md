# IMU gimbal rig

Small pan / tilt / roll rig for exercising Adafruit IMU, accelerometer and
magnetometer breakouts (3/6/9 DoF), with a later add-on striker for tap and
double-tap detection testing.

Target sensors: the drivers added in
[adafruit/Adafruit_Wippersnapper_Arduino#839](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/pull/839)
(ISM330DHCX, ISM330DLC, LIS2MDL, LIS3DH, LIS3MDL, LSM303AGR, LSM303DLH,
LSM6DS3, LSM6DSO32, LSM9DS1). Mostly STEMMA QT breakouts, the odd legacy board,
and one FeatherWing.

## Requirements

- Sensor carrier accepts:
  - the swirly grid ([tannewt/swirly-grid](https://github.com/tannewt/swirly-grid)), reproduced as a printed plate
    so a swirly-grid PCB bolts on through matching slots;
  - STEMMA QT sized breakouts (on the swirly grid);
  - one set of Feather-pattern screw bosses (for the FeatherWing).
- Hobby servos, with interchangeable mounts: MG90S, DS3240, MG995, DSC55MG.
- Aim for ±180° on every axis. In practice the servo travel limits it
  (180° or 270° servos).
- Tap / click striker (cam + sprung bar): deferred.

## Files

| File | What |
| --- | --- |
| `swirly_grid.py` | Pure-Python swirly-grid geometry + hole-pattern fit checker (`python swirly_grid.py 2 4`) |
| `make_swirly_plate.py` | FreeCAD generator for a printable swirly plate (env `SWIRLY_ROWS`, `SWIRLY_COLS`, `SWIRLY_SLOT`) |
| `preview_swirly.py` | Matplotlib 2D preview |
| `make_carrier.py` | Swirly plate + Feather bosses; the #4569 wing's ISM330DHCX sits over plate centre (env `CARRIER_BOSS_H`) |
| `make_gimbal.py` | Full pan/tilt/roll layout, per-part STEP/STL in `parts/`, 360° clearance sweeps per axis, `imu-gimbal-assembly.FCStd` |
| `docs/mechanical_data.md` | Board hole/IC positions for the PR 839 sensors, Feather spec, servo dimensions |

Generate with FreeCAD 1.1 (`C:\dev\software\FreeCAD\bin\freecadcmd.exe`):

```
freecadcmd -c "exec(open('make_swirly_plate.py').read(), {'__file__': 'make_swirly_plate.py', '__name__': '__main__'})"
```

## Gimbal layout (v1)

The origin is the sensor IC: roll = X, tilt = Y, pan = Z. Every axis passes
through the IC, and `make_gimbal.py` sweeps each moving stage through 360°
against the stage outside it and reports any collision.

| Part | Holds | Servo | Idler |
|---|---|---|---|
| `roll_cradle` | the carrier, held by 4 x M2.5 through the centre-cell round holes | MG90S horn pocket on the +X wall | M3 axle boss on -X |
| `tilt_ring_mg90s_tab16` / `_tab21` | roll servo + 623 bearing | MG90S (16 mm and 21 mm tab heights, since the published figures disagree; hole slots cover 27.5–28 mm) | standard-servo horn on the +Y bar, M3 axle on -Y |
| `pan_yoke` | tilt servo + 623 bearing | MG995 or DS3240 on +Y (near-identical mount geometry) | pan horn pocket underneath |
| `base` | pan servo | DS3240 (270° version for ±135°) | open +X end for cables, 4 x M3 bench holes |

The overall envelope is about 114 x 121 x 149 mm, and the printed parts weigh
about 185 g solid.

**Hardware:**
- 2 x 623ZZ bearings (3 x 10 x 4)
- 2 x M3 x 12 axle screws + washers
- 4 x M2.5 for carrier to cradle
- self-tapping screws for the horn arms and servo tabs

**Assumptions to check:**
- Horn arm sizes are guesses. MG90S: 36 x 7 x 2 deep. 25T: 46 x 8.5 x 2.5 deep.
- Horn hub heights are guesses: 2.5 / 3.5 mm.
- QT boards on the swirly plate need about 10 mm standoffs to bring their IC to
  the same height as the FeatherWing (on 9 mm bosses).

## Notes

- A 1.0" x 0.7" STEMMA QT board (holes 0.8" x 0.5" apart) cannot land all four
  screws in the swirly grid at once. Three holes fit in 32 placements on a
  3x3 grid, and two holes fit in hundreds. Plan on 2–3 screws per board.
- Slots print at 2.75 mm (nominal 2.54 mm) so M2.5 screws pass on FDM.
