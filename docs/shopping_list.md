# Shopping list

This list is for the main design with 9 QT boards and the FeatherWing. The
"Order" column rounds up to common pack sizes and adds spares. Lengths for
the `tall_top` variant are noted where they differ.

**Material:** fixings marked *non-magnetic* sit near the sensors. Buy nylon or
brass for those. A2 stainless will do at a pinch, but plain or zinc steel
will not.

## Easiest route: two kits plus a few singles

| Kit | Covers |
|---|---|
| **M2.5 nylon standoff / screw / nut kit** (the ~300-piece hex kits with 6–20 mm standoffs) | all board fixings: screws, nuts, 3 mm spacers (or use 2 nuts), 10 mm F-F standoffs |
| **M2 / M2.5 self-tapping screw assortment** (pan head, 6–10 mm) | horn arm screws and spare servo tab screws |

## Itemised

| Item | Material | Need | Order | Used for |
|---|---|---|---|---|
| **M3 x 50** bolt (tall_top: **M3 x 55**) | brass or nylon *(non-magnetic)* | 2 | 4 | clamping the stack through both spines |
| M3 nut | brass or nylon *(non-magnetic)* | 2 | 10 | stack bolts |
| M3 washer | nylon *(non-magnetic)* | 6 | 20 | stack bolts and axles |
| **M3 x 12** button head | A2 stainless | 2 | 10 | roll and tilt axles through the 623 bearings |
| **M3 x 10** socket or button head | any | 4 | 10 | striker stand to yoke (**not longer**) |
| **M3 x 14** (or x 16) (tall_top: **M3 x 22**, or x 25) | any | 2 | 10 | striker bar pad, from below |
| M3 x 16 + nut, or #4 wood screw | any | 4 | 4 | base to bench |
| **M2.5 x 10** pan head | nylon *(non-magnetic)* | 18 | 25 | QT boards (2 each) |
| M2.5 x 6 pan head | nylon *(non-magnetic)* | 8 | 10 | FeatherWing standoffs (both ends) |
| M2.5 nut | nylon *(non-magnetic)* | 18 | 30 | QT board screws |
| M2.5 x 3 mm spacer | nylon *(non-magnetic)* | 18 | 25 | under each QT board |
| M2.5 x 10 F-F standoff | nylon *(non-magnetic)* | 4 | 8 | FeatherWing (clears the header pins) |
| M2.5 x 8 self-tapping | any | 4 | 10 | DS3240 horn arms (tilt, pan) |
| M2 x 6 self-tapping | any | 2 | 10 | MG90S roll-horn arms |
| M2 x 8 self-tapping | any | 4 | 10 | MG90S and cam-servo tabs, if the servos come without screws |
| M2.6 or M3 x 10 self-tapping | any | 8 | 10 | DS3240 tabs, if not supplied (they usually are) |
| **623ZZ** bearing, 3 x 10 x 4 mm | steel | 2 | 4 (often sold in 10s) | roll and tilt idlers |
| CA glue (superglue) | - | - | 1 | the trimmed horn into `striker_cam` |

**Only if you print the horns or the spline cam:**

| Item | Need | Order | Used for |
|---|---|---|---|
| M2 x 8 machine screw | 1 | 5 | `horn_mg90s_printed` centre screw |
| M2 x 10 machine screw | 1 | 5 | `striker_cam_spline` centre screw |
| M3 x 10 machine screw | 2 | (from the M3 x 10 pack) | `horn_ds3240_printed` centre screws |

**Already on hand or supplied with the servos:** the horn centre screws, the
tab screws, and 1.75 mm filament (about 12 mm for the pawl pin).

**Not fixings, but needed to run it:**
- a TCA9548A I2C mux
- STEMMA QT cables (one per board, 100–200 mm)
- the servos: 2 x MG90S (one 270°), 2 x DS3240MG
