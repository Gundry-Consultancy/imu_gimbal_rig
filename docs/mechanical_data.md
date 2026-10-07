# Mechanical reference data

Coordinates are mm from the board's lower-left corner, in the orientation the
Eagle `.brd` draws it. Hole and IC positions were parsed from the Adafruit PCB
repos. All boards are 1.6 mm FR4 with R2.54 corners and 2.5 mm plated M2.5
holes unless a row says otherwise.

## STEMMA QT 1.0" x 0.7" (25.4 x 17.78) family

- **4-hole pattern:** (2.54, 2.54), (22.86, 2.54), (2.54, 15.24), (22.86, 15.24). The holes are 20.32 x 12.70 apart.
- **2-hole pattern:** only (2.54, 15.24) and (22.86, 15.24), along the top edge, because a header row runs along the bottom.
- **QT connectors:** centred on the left and right short edges.

| Chip (driver) | Board / PID | Holes | IC centre |
|---|---|---|---|
| ISM330DHCX | #4502 | 2 (top) * | (12.70, 8.89) * |
| LIS2MDL | #4488 | 4 (bottom pair are 2.54 non-plated) | (12.70, 8.89) |
| LIS3DH | #2809 QT, **portrait 17.78 x 25.4**, QT on top/bottom edges | 4, same pattern turned 90° | (8.89, 12.70) |
| LIS3MDL | #4479 | 4 | (12.70, 8.89) |
| LSM6DSOX+LIS3MDL | #4517 | 4 | 6DOX (12.446, 7.112), 3MDL (12.255, 10.922) |
| LSM6DS3TR-C+LIS3MDL | #5543 | 4 | 6DS3 (12.446, 7.112), 3MDL (12.255, 10.922) |
| LSM6DS33+LIS3MDL | #4485 | 4 | 6DS33 (12.70, 9.398), 3MDL (12.70, 6.160) |
| LSM303AGR | #4413 | 4 | (12.70, 8.89) |
| LSM6DS3TR-C | #4503 | 2 (top) | (12.70, 8.89) |
| LSM6DS33 | #4480 | 4 | (12.70, 8.89) |
| LSM6DSO32 | #4692 | 2 (top) * | (12.70, 8.89) * |
| LSM9DS1 | #4634 Rev C | 2 (top) | **(11.176, 8.128)** off-centre |

\* Taken from the LSM6DSOX #4438 board, which Adafruit says has an identical layout.

**Chips with no Adafruit board:** the ISM330DLC (the driver uses Adafruit_LSM6DSL) and the plain LSM6DS3 (use the TR-C).

## Legacy / non-QT

| Chip | Board / PID | Outline | Holes | IC centre |
|---|---|---|---|---|
| LIS3DH | #2809 original | 20.32 x 20.32 | (2.54, 17.78), (17.78, 17.78) | (10.16, 10.16) |
| LSM303DLHC | #1120 (no QT version exists) | 22.86 x 20.32 | (2.54, 17.78), (20.32, 17.78) | (11.684, 10.16) |
| LSM303DLHC + L3GD20H | 9-DoF #1714 / 10-DoF #1604 | 38.1 x 22.86 | 4 at 33.02 x 17.78 | LSM303 (28.461, 10.873) |
| LSM303DLHC | Flora #1247 | Ø ~16.5 round | none (sewable pads) | (8.255, 8.255) |
| LSM9DS1 | #3387 Rev A | 33.02 x 20.32 | 4 at 27.94 x 15.24 | (16.51, 10.16) |

## FeatherWing / Feather

- **Board:** probably the ISM330DHCX + LIS3MDL FeatherWing **#4569**. The #4565 LSM6DSOX version shares its layout.
- **Outline:** 50.80 x 22.86.
- **Holes:** (2.54, 2.54), (2.54, 20.32), (48.26, 2.54), (48.26, 20.32), giving 45.72 x 17.78 spacing. The USB-end pair is plated 2.5 mm and the other pair is non-plated 2.54 mm.
- **IC centres on #4569:** ISM330DHCX (27.94, 11.43) and LIS3MDL (22.86, 11.43).
- **Header rows:** y = 1.27 (x 6.35–44.45) and y = 21.59 (x 16.51–44.45).

## Swirly grid fit (2 x 4 plate, `swirly_grid.find_fits`)

| Pattern | Placements |
|---|---|
| QT 4-hole 20.32 x 12.70 | **0**. 3 of the 4 holes fit, so use 2–3 screws. |
| QT 2-hole 20.32 | 268 |
| Feather 45.72 x 17.78 (all 4) | 88 |
| LSM9DS1 legacy 27.94 x 15.24 | 176 |
| 9/10-DoF 33.02 x 17.78 | 0 |
| LIS3DH / LSM303DLHC legacy 2-hole | 3300 / 880 |

## Servos

| Servo | Body L x W x H | Tab length | Hole c-c | Tab underside from body bottom | Spline | Travel | Torque |
|---|---|---|---|---|---|---|---|
| MG90S | 22.8 x 12.2 x 28.5 | 32.2 | ~27.5–28 (single row) | 16–21: **measure** | 21T (some 20T) | 180° | 2.2 kg·cm @ 6 V |
| DS3240 | 40 x 20 x 40.4 | 54.5 | 49.5 x 10 | 27.7 | 25T | 180° **or** 270° (same body) | 36–45 kg·cm |
| MG995 | 40.7 x 19.7 x 42.9 | 54 | 48–49.5 x 9.5–10 | 28 | 25T | ~180° | 10 kg·cm @ 6 V |
| DSC55MG (9imod?) | 39.9 x 20.1 x 46.0 | ~54? | unpublished: **measure** | ? | 25T | 270° (180° variant exists) | 43–58 kg·cm, 6–8.4 V HV |

DS3240 STEP model: https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DS3240.stp
