# Servo control spec (for writing the control script)

Bench follow-up: [11-12 October 2026 servo report](servo_bench_2026-10-11.md).
The operator confirmed physical rest at **500 us** and estimated the opposite
endpoint near **2650-2670 us**. Increasing pulse turned anticlockwise viewed from
the gear face. Stalling under load made nominal command angles unreliable.
A powered endpoint cycle clicked, but neither the exact release pulse nor the
model-to-hardware angular origin is calibrated. For the physical mechanism use
[the slow pulse-based controller](../examples/cam_pulse_control.py); the angle
sequences below remain model specifications, not bench-validated procedures.

Angles come from the model (`make_striker.py` LOBES, `striker_motion.py`
PROGRAMME, `make_motion.py` joint limits). The main design and `tall_top` use
the same angles.

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
| **Cam** (striker) | MG90S-size, 270° | 270° | **0 … 270°** | **0 = rest at one end of travel**, not centre | Forward (increasing angle) = **−X rotation**, i.e. clockwise seen from +X |

**Typical pulse mapping** (calibrate each servo; clamp to its limits):
- **DS3240 270°:** 500–2500 µs over 270°, so `us = 1500 + angle_deg * 2000/270`.
- **DS3240 180°:** 500–2500 µs over 180°, so `us = 1500 + angle_deg * 2000/180`.
- **MG90S 180°:** about 500–2400 µs over about 180°, centre about 1450. Measure
  the end points before relying on them.
- **Cam servo 270°:** `us = 500 + cam_deg * 2000/270` if a rising pulse turns
  it clockwise seen from +X. Otherwise use `us = 2500 - cam_deg * 2000/270`
  and fit the cam with rest at that end. The direction can't be fixed in
  software after the cam is fitted, so check it before fitting.

## Cam (striker) angles

The two lobes are `(start ramp, end ramp, cliff)` = (15, 115, 125) and
(135, 235, 245).

| Cam angle | State |
|---|---|
| **0** | **Rest.** The hammer face is 5 mm above the roof, and the pawl sits on the base circle (dwell 0–15). |
| 15 → 115 | Lobe 1 ramp: the arm lifts |
| **120** | **Park A** (top of lobe 1, dwell 115–125). The hammer is fully up and the gimbal may move. |
| 125 → **134** | Release 1. Command **134**, not just past 125: the pawl rolls over the cliff's corner and drops at about 134. |
| 135 → 235 | Lobe 2 ramp: the arm lifts again |
| **240** | **Park B** (top of lobe 2, dwell 235–245) |
| 245 → **254** | Release 2. Command **254**. |
| 254 … 270 | Base circle, hammer at rest |

**Reverse moves (toward 0) never tap.** The pawl folds over the cliffs, and
going down a ramp lowers the hammer gently. So "lower without tapping" is
simply cam → 0.

## Procedures

- **Park before any tilt or roll move:** cam → 120 (or 240), then wait
  ~0.2 s. At rest the hammer is only 5 mm above the roof and inside the
  roll/tilt sweep. **Never move tilt or roll while the cam is outside
  115–125 or 235–245.**
- **Single tap:** from Park A, cam → 134 at full speed. Or from Park B,
  cam → 254. Allow ~0.2 s for the drop (~50 ms) and let-off to settle.
- **Double tap:** from Park A, cam → 134 (tap 1), then → 244 (lift), then
  → 254 (tap 2), in one forward move. The tap spacing is set by how fast the
  servo covers 134 → 254, about 0.2 s at full MG90S speed (0.1 s/60°).
  Slow the 134 → 244 move to space the taps further apart.
- **Re-arm** (after a tap at 134 or 254): cam → 0 (the pawl folds over the
  cliffs), then forward → 120 to park again.
- **Return to rest:** cam → 0.
- **Don't hold the cam between 125 and 134, or between 245 and 254:** the
  pawl is on the cliff corner and the drop timing is unpredictable.
- **Gimbal speed:** the DS3240 does about 0.17 s/60° at 6 V and the MG90S
  about 0.1 s/60°. Ease the moves, since the stack is ~100 g on the roll
  axis.

## Example sequence (as in `docs/motion/motion.mp4`)

```
cam 0 -> 120 (park)
pan 0 -> +135 -> -135 -> 0
tilt 0 -> limit -> back      (mechanically clear 360; the servo limits it)
roll 0 -> +90 -> -90 -> 0
cam 120 -> 0 (re-arm), 0 -> 120 (park)
cam 120 -> 134 (single tap), 134 -> 0, 0 -> 120
cam 120 -> 134 -> 244 -> 254 (double tap)
cam 254 -> 0 (rest)
```

## Not verified on hardware

- The 134 / 254 release angles come from the pawl and cam geometry in the
  model. Fine-tune them by a few degrees on the bench.
- The pulse end points vary per servo, so calibrate them.
- The cam-servo direction must be checked before fitting the cam.
