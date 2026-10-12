# Servo setup and later tests

**Cam servo replaced by an MG995 (12 October, model change only; not run on
hardware).** The cam now has two lobes inside the MG995's ~180°. Rest is still
**500 us** (model 0), and increasing pulse turns the cam forward (`direction=1`).
Nominal model pulses (`us = 500 + angle * 2000 / 180`) are: park 69° = 1267 us,
single tap 87° = 1467 us, double-tap end 165° = 2333 us. See
[the servo spec](../docs/servo_control.md) for the calibration steps.
`CALIBRATED` stays false until then.

**Latest bench mapping (12 October):** physical rest is **500 us / 0.500 ms**;
the opposite endpoint is estimated at **2650-2670 us / 2.650-2.670 ms**. Use
[`cam_pulse_control.py`](cam_pulse_control.py) for slow cam-only sweeps. The
[bench report](../docs/servo_bench_2026-10-11.md) records every command and
supersedes earlier assumptions that 2500 us was rest. The mechanism stalled
under load during angle-based trials. The exact click threshold and physical
angle mapping remain uncalibrated; `CALIBRATED` stays false.

```python
import cam_pulse_control as cam
cam.sweep()  # Preview only.
cam.sweep(execute=True, confirmed=True, cycles=2)  # Slow 500 -> 2650 -> 500 twice.
# An already active 50 Hz controller and verified physical setup are required.
```

The pulse controller writes only channel 1 and preserves the other three outputs.
Copy it to CIRCUITPY/lib before use; the live board was not updated during reporting.
The model-angle examples below are retained for development and dry previews.
Their reversed nominal cam angle 0 corresponds to 2500 us, **not physical rest**.

These examples implement [the model's servo spec](../docs/servo_control.md).
Run the same `gimbal_servos.py` on CircuitPython or desktop Python with Blinka.
Importing it does not initialize I2C or write PWM. Tests and CLI previews need no
Adafruit libraries or hardware.

| Joint | FeatherWing output (zero based) | Neutral pulse | Travel |
| --- | --- | --- | --- |
| Tilt, next big servo | 0 (first) | 1500 us | 180 degrees initially; verify your variant |
| Cam / spring servo (MG995) | 1 (second) | 1500 us | 180 degrees (nominal) |
| Roll (inferred remaining output) | 2 (third) | 1450 us | 180 degrees |
| Pan, bottom servo | 3 (fourth) | 1500 us | 270 degrees |

The driver is **PCA9685** (the request's PCA9865 appears to be a transposition).
Roll neutral is the spec's approximate 1450 us midpoint of 500--2400 us;
measure it on your servo. Change `SERVOS` in `gimbal_servos.py` for channels,
travel, pulse limits, neutral trims and signs. Pan/tilt/roll zero means home.
Cam neutral is mid-travel (about 90 degrees), **not rest**. Center with the
horns and cam detached. Fit the cam at its verified rest endpoint, not neutral.

## Web Serial servo console (12 October)

`circuitpython_code.py` is now an 8-channel servo console. Copy it to CIRCUITPY as `code.py`.
It needs no extra libraries: it talks to the PCA9685 registers directly and never resets the chip.

[`servo_console.html`](servo_console.html) drives it from Chrome or Edge using Web Serial. Open
the file locally, or from the CIRCUITPY drive, then click **Connect** and pick the board's serial
port.

- **Channels 0-3:**
  - Labelled with accelerometer axes: Y tilt, Tap (hammer cam), X roll, Z pan.
  - Bench-trimmed centres.
  - Software end stops on by default.
- **Channels 4-7:** spare, with end stops off.
- **Per channel:**
  - slider
  - Centre, Rest, Off
  - end-stop toggle with min/max
  - go to angle, µs or duty %
  - sweep (start, end, rate in µs/s, cycles; 0 = forever)
- **Sequences:** park, single, double, rest, home, tour, centre_all. The hammer values are the
  MG995 model pulses (uncalibrated).
- **Tilt/roll interlock** (on by default): tilt and roll can't move unless the hammer cam is
  parked.
- **PCA off at boot** (on by default, stored in NVM): every output is switched off when the board
  starts. Turn it off to keep held outputs across restarts.
- **STOP + all OFF:** stops all motion and removes holding torque.

Plain serial terminals work too. Send one command per line (`HELP` lists them); replies are
JSON lines.

## Existing CircuitPython device

The connected board was identified as `adafruit_feather_rp2040_usb_host`, running
CircuitPython 10.0.3. This route does not require U2IF or `BLINKA_U2IF`.

On CIRCUITPY, put `gimbal_servos.py` in `lib/`, together with
`adafruit_pca9685.py` (or matching `.mpy`), `adafruit_bus_device/`, and
`adafruit_register/` from the [Adafruit CircuitPython library bundle](https://circuitpython.org/libraries).
Back up/rename the existing `code.py` before copying `circuitpython_code.py` to
`code.py`. The example waits for console input and does not change PWM on startup.
Connect a serial console to the board (COM3 at setup time).

With horns/cam detached, type `CENTER` to command neutral on all four outputs.
They hold that command until `OFF` or servo power is removed. `OFF` removes
holding torque: support any fitted mechanism first. Connecting battery power
while neutral PWM is active immediately commands the servos to neutral.

The PCA9685 can retain its last PWM settings across a Python restart while it
stays powered. A passive startup is not proof that its outputs are off. Use
`OFF` explicitly before connecting servo power if movement is not intended.
Use a suitable separate servo supply on the FeatherWing servo power input,
with common ground; USB logic power is not the high-current servo supply.

For calibration, Ctrl-C to enter the REPL:

```python
from gimbal_servos import Rig
rig = Rig(dry_run=False)
rig.center(detached=True)
rig.pulse("cam", 1520, detached=True)  # Small unloaded direction check from 1500.
rig.pulse("cam", 1500, detached=True)
rig.off()
```

`pulse` is an unloaded calibration tool, not an assembled motion command.
Gradually approach endpoints in small increments; the initial 500--2500 us
(roll 500--2400 us) limits are model/spec assumptions, not measured safe limits.
Check cam direction before fitting: increasing pulse must turn the MG995's spline
anticlockwise seen from its gear face (the model's forward). `direction=1` maps
cam 0 to min_us (500 us, rest) and 180 to max_us. Fit the cam at 500 us.
For the axes, positive is anticlockwise looking toward the origin down the axis.

## Tests after calibration and assembly

Check channels, tilt travel (180 or 270), endpoints, axis signs and horn alignment;
then set `CALIBRATED = True` in the copy actually being run. This is deliberately
false initially. The script has no position feedback or persistent pose estimate.
Physically establish axes at home and cam at rest before every new session, and
after an interruption. Support the mechanism during initialization; changing the
PCA9685's frequency is chip-wide and briefly interrupts PWM. Use a dedicated wing.

```python
from gimbal_servos import Rig
rig = Rig(dry_run=False)
rig.start_home_rest(confirmed=True)  # Confirm the PHYSICAL pose, not an assumption.
rig.sweep("pan", amplitude=10)
rig.sweep("tilt", amplitude=10)      # Parks before tilt/roll; eases each axis move.
rig.sweep("roll", amplitude=10)
rig.home()
rig.tap()                          # Re-arm 0 -> 69, then 87; leaves cam at 87.
rig.tap(double=True)               # Re-arm, then 69 -> 165 in one full-speed move.
rig.tap(double=True, gap=0.5)       # 69 -> 87, ease over lobe 2 to 149, then 165.
rig.rearm()                        # 0 -> 69.
rig.rest()                         # 0, only when tilt/roll are home.
```

Only run taps after verifying clearances. Release angles 87/165 are model-derived
(the pawl leaves cliffs 73/151 by 75.2/153.2) and unverified on hardware.
`RELEASE_A` must stay 10 degrees past cliff 1 and 5 before lobe 2's ramp (97);
`RELEASE_B` must stay 10 degrees past cliff 2 and at or under 170. Changing the
geometry needs a new sequence review. Single taps wait for travel and about 0.2 s
settling. A double tap is one move through both cliffs; the drops are 78 degrees
apart, about 0.21 s at 6 V and 0.26 s at 4.8 V (MG995 no-load speed). USB, load and servo speed affect timing.
No commanded dwell is allowed inside the release-corner intervals. Motion waits
estimate travel and settling; they do not detect jams or verify actual position.

An exception or Ctrl-C does not automatically lower the hammer or cut holding
torque. The last successful PWM writes remain active. Support the mechanism,
disable servo power as appropriate, and re-establish its physical pose before
continuing. `off()` invalidates the software pose. The saved `planned_motion.py` contains the model full-motion demo, with a
calibration gate for live runs. Its command sequence completed earlier, but
physical lift/release failed with the original cam direction; do not treat that
run as mechanism validation. Increase sweep amplitude only after measuring the usable range.

## Optional desktop Blinka/U2IF route

Use this only if you later replace CircuitPython with compatible U2IF firmware.
[Adafruit's U2IF guide](https://learn.adafruit.com/circuitpython-libraries-on-any-computer-with-raspberry-pi-pico/other-rp2040-boards)
documents board-specific firmware/pin mapping and setting `BLINKA_U2IF=1` before
importing `board`. Its generic Feather RP2040 instructions do not establish
USB Host variant compatibility; verify that firmware's identity and I2C mapping.
The native CircuitPython route above was chosen for this connected board.

From the project folder, in PowerShell:

```powershell
python -m venv .venv-servo
.\.venv-servo\Scripts\python -m pip install -r examples\requirements.txt
$env:BLINKA_U2IF = "1"
python examples\servo_cli.py center --detached        # Preview only; no USB access.
python examples\servo_cli.py sweep --axis roll       # Preview default +/-10 degrees.
python examples\servo_cli.py double --gap 0.5        # Preview model sequence.
.\.venv-servo\Scripts\python examples\servo_cli.py center --detached --execute
.\.venv-servo\Scripts\python examples\servo_cli.py off --execute
# Later, with CALIBRATED=True and verified physical home/rest:
.\.venv-servo\Scripts\python examples\servo_cli.py sweep --axis pan --at-home-rest --execute
```

The CLI also supports `pulse --axis cam --us 1520 --detached`, `single`, and
`double`. Assembled CLI tests finish at home/rest, retaining PWM. Never issue
another process's motion commands concurrently with this controller.

Run the hardware-free checks with:

```powershell
python -m unittest discover -s examples -v
```

Pulse conversion uses the driver's actual prescaled frequency at nominal 50 Hz;
measure and adjust `REFERENCE_CLOCK` for precise timing, per the
[PCA9685 documentation](https://docs.circuitpython.org/projects/pca9685/en/latest/examples.html).

## Setup verification (11 October 2026)

The board was checked with servo battery power disconnected. The I2C scan
returned 0x40 and 0x70. Native CircuitPython imports, simulated sweeps and taps,
and actual neutral PWM writes succeeded. Register readback at approximately
50.0288 Hz was [4912, 4912, 4752, 4912] for channels 0--3; OFF read back
[0, 0, 0, 0]. This verifies communication and programming, not physical servo
position, measured pulse timing, direction, travel, or release angles.

On CIRCUITPY the previous UART demo is now `code_before_servo_setup.py`; the
new `code.py` console was started and its OFF command verified. The helper and
Adafruit PCA9685 driver are in `lib/`. All four outputs were left disabled.
The 13 hardware-free unit tests and Ruff checks also passed.

### Powered, detached travel check (11 October 2026)

With servo battery connected and the mechanism confirmed safely disassembled,
`sweep_unloaded.py` commanded one servo at a time from neutral to minimum,
maximum, then neutral in gradual steps. Tilt/cam/pan covered 500--2500 us;
roll covered 500--2400 us. All four sequences completed without a reported
exception. Final duty readback was [4912, 4912, 4752, 4912]. All four were left
holding neutral, and the serial console was restarted. No position feedback or
visual measurement was available, so this does not confirm actual travel, freedom
from binding, or the tilt variant. CALIBRATED remains False.

Repeat only with linkage/cam detached, from the CircuitPython REPL:

```python
import sweep_unloaded
sweep_unloaded.run(detached=True)                 # Preview only.
rig = sweep_unloaded.run(detached=True, dry_run=False)  # Powered sweep.
```

The helper is also installed as `lib/sweep_unloaded.py` on CIRCUITPY.
