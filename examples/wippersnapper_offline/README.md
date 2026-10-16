# WipperSnapper offline configs (PR 839 driver test)

Offline-mode `config.json` files for testing the accelerometer / magnetometer drivers in
[adafruit/Adafruit_Wippersnapper_Arduino#839](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/pull/839)
on the rig's sensor stack, from a Feather RP2040 Adalogger (SD CS pin 23).

| File | Mux | Devices |
|---|---|---|
| `config_mux71.json` | TCA9548A @ 0x71 (A0 bridged) | ism330dhcx (ch1), lis3dh + lsm6dso32 (ch3), lis2mdl + lsm6ds3 [TR-C] (ch6) |
| `config_mux77.json` | PCA9546 @ 0x77 | lsm303agr (ch0), lsm9ds1 + lis3mdl (ch2), ism330dhcx + lis3mdl FeatherWing (ch3) |

Copy one to the WIPPER drive as `config.json`. Auto-config is off everywhere, so nothing is guessed.

- WipperSnapper supports one mux per bus, hence one config per mux.
- **Power-cycle between configs.** A mux keeps its channel through an MCU reset, and the firmware never clears
  a mux it isn't configured with, so a channel left open on the other mux shares addresses (0x1E, 0x6A) and
  breaks init (seen as the LSM9DS1 failing after running `config_mux71.json`).
- `lis2mdl` / `lsm303agr` need Adafruit_LIS2MDL with `config_a` initialised
  ([adafruit/Adafruit_LIS2MDL#14](https://github.com/adafruit/Adafruit_LIS2MDL/pull/14)); 2.1.8 crashes in `begin()`.
