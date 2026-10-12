# Servo control spec (for writing the control script)

**Cam servo changed on 2026-10-12: MG995** (standard size, ~180°, 8.5 kg·cm at
4.8 V). The MG90S-size cam servo stalled under the spring load on the bench (see
[the 11-12 October report](servo_bench_2026-10-11.md)). The redesigned cam needs a
peak of **1.53 kg·cm** with friction, which is 18 % of the MG995's stall. The
old cam needed 1.46 kg·cm under the same assumptions, which is 81 % of an MG90S's
1.8 kg·cm (see [the torque budget](striker-torque.png)).

The angles here come from the model (`make_striker.py` LOBES,
`striker_motion.py` PROGRAMME, `make_motion.py` joint limits). The main design
and `tall_top` use the same angles. **Pulses are nominal** until the MG995 is
calibrated (see below). For slow bench moves by raw pulse, use
[`cam_pulse_control.py`](../examples/cam_pulse_control.py).

## Axes and sign convention

- Model frame: +X is the roll axis, +Y the tilt axis, +Z the pan axis (up).
- Positive angles follow the right-hand rule about the positive axis.
- Home (0, 0, 0): the stack sits level and the striker is above the roof.
- The gimbal servos sit at **mid-travel** at home. Fit the horns so the servo
  centre (1500 µs) equals home.

## The four servos

| Joint | Servo | Servo travel | Joint range to command | Home = servo | Notes |
|---|---|---|---|---|---|
| **Pan** (base → yoke, Z) | DS3240MG, 270° version | 270° | **−135 … +135°** | centre | It carries the striker too, so pan is allowed at any striker state |
| **Tilt** (yoke → ring, Y) | DS3240MG (180° or 270° version) | 180° or 270° | **±90°** (180° servo) or **±135°** (270° servo) | centre | Mechanically clear for 360° **only with the hammer parked** |
| **Roll** (ring → stack, X) | MG90S | ~180° | **−90 … +90°** | centre | Clearance is verified for ±90° with the hammer parked |
| **Cam** (striker), PCA9685 channel 1 | **MG995** | ~180° | **0 … 165°** | **0 = rest = 500 µs**, the low-pulse end | Forward (increasing angle) = **increasing pulse** = anticlockwise seen from the servo's gear face |

**Typical pulse mapping** (calibrate each servo; clamp to its limits):
- **DS3240 270°:** 500–2500 µs over 270°, so `us = 1500 + angle_deg * 2000/270`.
- **DS3240 180°:** 500–2500 µs over 180°, so `us = 1500 + angle_deg * 2000/180`.
- **MG90S 180°:** about 500–2400 µs over about 180°, centre about 1450. Measure
  the end points before relying on them.
- **Cam, MG995:** `us = 500 + cam_deg * 2000/180` (11.1 µs per degree), nominal.

**Cam direction.** The MG995 sits on the tower's **+X wall** with its gear face
toward −X. On the bench, increasing pulse turned the cam anticlockwise seen
from the gear face. From −X that is a −X rotation, which is the model's forward
direction. So rest is at the low-pulse end and forward is increasing pulse, with
no software inversion (`direction=1` in `gimbal_servos.py`).

## Cam angles

Two lobes, each `(ramp start, ramp end, cliff)`: **(15, 65, 73)** and
**(97, 147, 151)**. Base radius 12 mm, lift 6.9 mm. The hammer parks 48° up
on either dwell.

| Cam angle | Pulse (µs, nominal) | State |
|---|---|---|
| **0** | **500** | **Rest.** The hammer face is 5 mm above the roof. The pawl sits 0.4 mm clear of the base circle (dwell 0–15). |
| 15 → 65 | 667 → 1222 | Lobe 1 ramp: the arm lifts (from ~19°) |
| **69** | **1267** | **Park A** (dwell 65–73). The hammer is fully up and the gimbal may move. |
| 73 | 1311 | Cliff 1. The pawl rolls over its corner and is off it by **75.2** (model). |
| **87** | **1467** | **Single-tap release command.** 11.8° past the modelled drop, 10° before lobe 2 |
| 97 → 147 | 1578 → 2133 | Lobe 2 ramp: the arm lifts again |
| 149 | 2156 | Park B (dwell 147–151), passed through during a double tap |
| 151 | 2178 | Cliff 2. The pawl is off it by **153.2** (model). |
| **165** | **2333** | **Double-tap end command.** 11.8° past the second drop, 15° short of 180° |

**Reverse moves (toward 0) never tap.** The pawl folds over the cliffs, and the
hammer comes down gently. But a reverse move also can't park: from a release
angle, go to 0 and then forward to 69. Turning back to 69 leaves the pawl
folded on the lobe with the hammer down.

## Procedures

- **Park before any tilt or roll move:** cam 0 → 69, then wait ~0.3 s. At
  rest the hammer is only 5 mm above the roof and inside the roll/tilt sweep.
  **Never move tilt or roll unless the cam was driven forward from 0 to 69.**
- **Single tap:** from Park A, cam → 87 at full speed. Allow ~0.2 s for the
  drop (~60 ms in the model) and the let-off to settle.
- **Double tap:** from Park A, cam → 165 in **one full-speed move**. The servo
  passes cliff 1, lifts the arm on lobe 2 and passes cliff 2.
  - **Interval:** the drops are 78° apart, which is **0.21 s at 6 V** (0.16 s/60°)
    and **0.26 s at 4.8 V** (0.2 s/60°). Under the ~1.5 kg·cm load, expect
    up to ~0.32 s.
  - **Both taps are full strength.** Lobe 2 lifts the arm the full 48°.
  - **Tap 1 lands first.** It hits the roof before lobe 2 meets the pawl: the arm
    lands at cam ~88–92°, and lobe 2 reaches the pawl at ~100°.
  - **Longer gap:** to space the taps further apart, stop at 87, ease over lobe 2
    to 149, then → 165 (`tap(double=True, gap=...)`).
- **Re-arm** (after a tap at 87 or 165): cam → 0 (the pawl folds over the
  cliffs), then forward → 69.
- **Return to rest:** cam → 0.
- **Don't hold the cam between 71 and 87, or between 149 and 165:** the pawl
  is on a cliff corner there, and the drop timing is unpredictable.
- **Gimbal speed:** the DS3240 does about 0.17 s/60° at 6 V and the MG90S
  about 0.1 s/60°. Ease the moves, since the stack is ~100 g on the roll axis.

## Calibrating the MG995 (before `CALIBRATED = True`)

1. **Unloaded,** with the cam off: find the pulse range. Step out from 1500 µs
   in small increments toward 500 and toward 2500, listening for the end stop.
   Check that increasing pulse turns the spline anticlockwise, seen from its gear face.
2. **Fit the cam at 500 µs.** Choose the spline tooth (25T, 14.4° each) that
   puts the pawl tip on the base circle just before lobe 1's ramp.
3. **Find the lift point:** step up slowly from 500 µs. The arm should start
   to lift near model 19° (~710 µs). The difference from the model is the angle offset.
4. **Find drop 1:** step slowly through park toward 1311 µs and note the pulse
   where the pawl drops off cliff 1 (model 75.2°, ~1336 µs). Set the single-tap
   command to that pulse **+ 12°** (~133 µs). Find drop 2 the same way (model
   153.2°) and set the double-tap end to it + 12°. Keep the double-tap end at
   least 10° short of the measured upper end.
5. Check repeatability under load: 10 single taps and 10 double taps from park.

## Example sequence (as in `docs/motion/motion.mp4`)

```
cam 0 -> 69 (park)
pan 0 -> +135 -> -135 -> 0
tilt 0 -> limit -> back      (mechanically clear 360; the servo limits it)
roll 0 -> +90 -> -90 -> 0
cam 69 -> 0 (re-arm), 0 -> 69 (park)
cam 69 -> 87 (single tap), 87 -> 0, 0 -> 69
cam 69 -> 165 (double tap)
cam 165 -> 0 (rest)
```

## Not verified on hardware

- **Release angles:** the 87 / 165 commands come from the pawl and cam
  geometry in the model, with the pawl rolling over each corner. Measure the
  drops (step 4 above).
- **Pulse map:** the MG995 mapping is nominal. Calibrate it.
- **Model assumptions:** friction is taken as μ = 0.4 (PETG on PETG) at the
  pawl and in the drive-pin slot. The torque budget and tap energy assume it.
