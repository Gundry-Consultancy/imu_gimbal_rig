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

Generate with FreeCAD 1.1 (`C:\dev\software\FreeCAD\bin\freecadcmd.exe`):

```
freecadcmd -c "exec(open('make_swirly_plate.py').read(), {'__file__': 'make_swirly_plate.py', '__name__': '__main__'})"
```

## Notes

- A 1.0" x 0.7" STEMMA QT board (holes 0.8" x 0.5" apart) cannot land all four
  screws in the swirly grid at once. Three holes fit in 32 placements on a
  3x3 grid, and two holes fit in hundreds. Plan on 2–3 screws per board.
- Slots print at 2.75 mm (nominal 2.54 mm) so M2.5 screws pass on FDM.
