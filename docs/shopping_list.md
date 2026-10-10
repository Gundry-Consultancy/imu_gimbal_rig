# Shopping list

This list is for the main design with 9 QT boards and the FeatherWing, plus
the `tall_top` lengths where they differ. The "Order" column rounds up to
common pack sizes and adds spares.

**Material:** fixings marked *non-magnetic* sit near the sensors. Buy nylon or
brass for those. A2 stainless will do at a pinch, but plain or zinc steel
will not.

## Already have (2026-10-10)

- M2 self-tapping, 6 mm and 8 mm: for the MG90S horn arms and the MG90S /
  cam-servo tabs
- M2.5 x 8 self-tapping: for the DS3240 horn arms
- 623ZZ bearings (on order)
- 1.75 mm filament, for the pawl pin

## To order

| Order | Item | Material | Need | Used for |
|---|---|---|---|---|
| 4 | **M3 x 50** bolt | brass or nylon *(non-magnetic)* | 2 | stack bolts, main design |
| 4 | **M3 x 55** bolt | brass or nylon *(non-magnetic)* | 2 | stack bolts, `tall_top` (skip if not building it) |
| 10 | M3 nut | brass or nylon *(non-magnetic)* | 2 | stack bolts |
| 20 | M3 washer | nylon *(non-magnetic)* | 6 | stack bolts, axles |
| 10 | **M3 x 12** button head | A2 stainless | 2 | roll and tilt axles through the 623s |
| 20 | **M3 x 10** socket or button head | A2 / any | 14 | striker stand to yoke (4, **not longer**), DS3240 tab screws (8, cut into the 2.4 mm pilots), printed DS3240 horn centres (2) |
| 10 | **M3 x 16** socket head | any | 6 | striker bar pad from below (2), base to bench (4) |
| 10 | **M3 x 25** socket head | any | 2 | striker bar pad, `tall_top` only (tip clears the bar by a wide margin) |
| 10 | M3 nut | steel | 4 | base to bench |
| 25 | M2.5 x 10 pan head | nylon *(non-magnetic)* | 18 | QT boards (2 each) |
| 10 | M2.5 x 6 pan head | nylon *(non-magnetic)* | 8 | FeatherWing standoffs (both ends) |
| 30 | M2.5 nut | nylon *(non-magnetic)* | 18 | QT boards |
| 25 | M2.5 x 3 mm spacer | nylon *(non-magnetic)* | 18 | under each QT board |
| 8 | M2.5 x 10 F-F standoff | nylon *(non-magnetic)* | 4 | FeatherWing (clears the header pins) |
| 5 | M2 x 8 machine screw | any | 1 | printed MG90S horn centre (into the servo's threaded spline) |
| 5 | M2 x 10 machine screw | any | 1 | `striker_cam_spline` centre |
| 1 | CA glue (superglue) | - | - | the trimmed horn into `striker_cam` |

**One kit instead:** the six nylon M2.5 lines are all covered by one
**M2.5 nylon standoff / screw / nut kit** (the ~300-piece hex kits with
6–20 mm standoffs).

**The horn centre screws are machine screws.** They go into the servo
spline's tapped hole, so self-tapping screws won't do there. Use the ones
supplied with the servos for stock horns.

**Not fixings, but needed to run it:**
- a TCA9548A I2C mux
- STEMMA QT cables (one per board, 100–200 mm)
- the servos: 2 x MG90S (one 270°), 2 x DS3240MG
