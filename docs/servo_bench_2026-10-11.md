# Servo bench record: 11-12 October 2026

## Result and current mapping

The operator confirmed **500 us (0.500 ms) as physical rest**. The opposite
endpoint needed approximately **2650-2670 us (2.650-2.670 ms)**; during the
exploratory sweep to 2700 us the operator observed roughly two seconds without
further movement. This is an endpoint estimate, not a measured mechanical angle
or a reason to use 2700 us routinely. The repeatable pulse controller defaults
to the lower estimate, 2650 us, with slow moves and a 500 us finish.

The operator reported that a powered endpoint cycle clicked/tapped/snapped.
Earlier it had stalled under the required force and worked when manually
assisted. An exact release pulse, reliable unassisted release threshold, and
release timing have **not** been isolated. A successful PWM register write is
not proof the servo reached the requested position.

Increasing pulse was observed to turn the cam anticlockwise viewed from its
gear face; decreasing pulse turns the opposite way. Earlier statements that
2500 us was physical rest were incorrect and are superseded by the operator's
later confirmation of 500 us. The cam was detached and refitted during testing,
so early observations cannot establish the final mechanical indexing.

The intended physical cycle is now expressed in pulses: rest at 500 us, move
toward the upper/reset end, then return toward 500 us for the click. The operator
confirmed rest and estimated the upper end; the exact click point and its
repeatability still need measurement. Model 134/254-degree release positions
were not validated by this experiment.

## Hardware and wiring

- Feather RP2040 USB Host, CircuitPython 10.0.3; the existing firmware was retained.
- PCA9685 Servo FeatherWing at I2C 0x40 (0x70 also appeared in the initial scan).
- Outputs: tilt 0, cam/spring 1, roll 2, bottom pan 3.
- USB serial was COM3 during the session; separate servo battery power.
- The old UART demo was renamed `code_before_servo_setup.py` on CIRCUITPY.
- The replacement `code.py` console accepts CENTER/OFF and does not write PWM at
  startup. Centering the cam is mid-travel, not physical rest.

The oscillator setting was 25 MHz with prescale 121: calculated PWM frequency
about 50.0288 Hz, period about 19.98848 ms, and PCA9685 tick 4.88 us. These are
register-derived timing estimates, not oscilloscope measurements. The library
accepts 16-bit duty values but the PWM hardware resolves 12 bits. The conversion
used `round(pulse_us * frequency * 65535 / 1e6) >> 4`, so requested widths and
register-derived widths differ by a few microseconds. Tables below list requested
widths unless explicitly labelled otherwise. **1 ms = 1000 us.** Conversational
requests such as "700ms" were interpreted and announced as 700 us / 0.700 ms.

## Axes and final held commands

These angles remain nominal relative to the original software neutral; no
encoder or external angular measurement was available. Tilt still uses the
180-degree configuration; its actual 180/270-degree variant was not unambiguously
identified. Axis trims were retained during all subsequent cam-only tests.

| Axis | Channel | Session adjustment | Requested us | Requested ms | Final duty readback | Calculated pulse from count (us) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| Pan | 3 | 0 degrees, unchanged | 1500.000 | 1.500000 | 4912 | 1498.16 |
| Tilt | 0 | +3 degrees, then back 1: **+2 degrees** | 1522.222 | 1.522222 | 4976 | 1517.68 |
| Roll | 2 | +2 then another +2: **+4 degrees** | 1492.222 | 1.492222 | 4880 | 1488.40 |
| Cam | 1 | Final physical rest, not an established model angle | 500.000 | 0.500000 | 1632 | 497.76 |

Intermediate alignment commands: roll +2 = 1471.111 us / 1.471111 ms;
tilt +3 = 1533.333 us / 1.533333 ms. Initial neutral commands were tilt/pan/cam
1500 us and roll 1450 us. These manual axis offsets are recorded as observations;
the generic angle controller's neutral calibration has not been silently redefined.

## Initial model commands and direction correction

The first script assumed increasing pulse meant increasing cam angle:
`pulse_us = 500 + angle * 2000 / 270`. Angles were model labels only.

| Nominal cam command | us | ms | Observation/context |
| ---: | ---: | ---: | --- |
| 0 | 500.000 | 0.500000 | Initial horn-fitting endpoint, before later refit |
| 120 | 1388.889 | 1.388889 | Expected Park A; operator said hammer was flat |
| 110 | 1314.815 | 1.314815 | Reverse diagnostic step; still flat |
| 80 | 1092.593 | 1.092593 | Further reverse diagnostic step; no lift reported |
| 240 | 2277.778 | 2.277778 | Expected Park B; cam rolled backward/pawl nearly horizontal |

A full model sequence was also commanded: pan +135/-135/0, tilt +90/0 under
the configured 180-degree assumption, roll +90/-90/0, and model single/double
cam releases. The controller had first been found asleep at power-on defaults;
its PWM configuration was restored. Command completion was verified, but the
operator then reported the hammer had not been raised. **That run did not
validate the physical mechanism.** No full gimbal motion was repeated after that
report; subsequent axis movements were only the requested small alignment trims.

The operator observed anticlockwise cam rotation with increasing pulse. Testing
then used the reversed nominal formula:
`pulse_us = 2500 - angle * 2000 / 270`. The cam was detached, the servo commanded
to 2500 us, and the cam refitted before the following commands. Calling this
endpoint "rest" at that time was an incorrect assumption, corrected later.

| Reversed nominal angle | us | ms | Result reported |
| ---: | ---: | ---: | --- |
| 0 | 2500.000 | 2.500000 | Refit endpoint; later found not to be physical rest |
| 120 | 1611.111 | 1.611111 | Hammer raised; fully ready-to-release not yet established |
| 134 | 1507.407 | 1.507407 | No click; not far enough |
| 136 | 1492.593 | 1.492593 | Intermediate command before operator requested more |
| 146 | 1418.519 | 1.418519 | No release |
| 164 | 1285.185 | 1.285185 | No release |
| 184 | 1137.037 | 1.137037 | Close, but no release |
| 204 | 988.889 | 0.988889 | Operator assisted; mechanism then worked; load-related stall reported |

Do not record 204 degrees as a calibrated release angle: the servo was not
reliably tracking the commands under load. Do not extrapolate either nominal
angle formula to infer the physical angles at 2650-2670 us.

## Pulse-based endpoint investigation

The first 2500 -> 500 -> 2500 us endpoint test ran with servo battery power off,
as the operator subsequently explained. It verified only the PWM registers.
The repeat with power present produced a click, but finished at what the
operator identified as the wrong end, not physical rest. The operator requested
slower moves thereafter.

The following later cam-only setpoints were commanded in order. Other channel
registers were compared before/after and retained exactly.

| Step | us | ms | Note |
| ---: | ---: | ---: | --- |
| 1 | 2550 | 2.550 | Small extension at the wrong end; operator said no movement |
| 2 | 2500 | 2.500 | Backed off extension |
| 3 | 500 | 0.500 | Slow move to the opposite end |
| 4 | 700 | 0.700 | Operator-requested probe |
| 5 | 400 | 0.400 | Load-related sticking reported |
| 6 | 500 | 0.500 | Operator reported it shifted a little |
| 7 | 2500 | 2.500 | Slow move to the other end |
| 8 | 1600 | 1.600 | Slow return probe |
| 9 | 1400 | 1.400 | Slow return probe |
| 10 | 1200 | 1.200 | Slow return probe |
| 11 | 900 | 0.900 | Slow return probe |
| 12 | 350 | 0.350 | Exploratory low-pulse command |
| 13 | 250 | 0.250 | Exploratory low-pulse command |
| 14 | 500 | 0.500 | Return |
| 15 | 450 | 0.450 | Probe |
| 16 | 550 | 0.550 | Probe |
| 17 | 300 | 0.300 | Probe |
| 18 | 500 | 0.500 | Operator explicitly confirmed this as rest |

The 250-450 us probes and 2550 us non-movement observation do not establish
usable endpoint limits. Earlier load-related sticking confounded the inference.
No movement/torque/current sensing was available to the assistant.

From confirmed 500 us rest, slow 500 -> 2500 -> 500 sweeps were repeated,
including two-cycle runs. Each leg used about 200 increments separated by
0.05 s (about 10 seconds plus I2C overhead), with roughly one-second endpoint
pauses. "Double sweep" meant two full pulse cycles, not the model's fast double
tap over two lobes.

The last sweep went 500 -> 2500 -> 2700 -> 500 us. The extension from 2500 to
2700 used **2510, 2520, ... 2700 us**, pausing 0.3 s at each 10 us step. The
operator estimated useful travel ended near **2650 or 2670 us**, based on about
two seconds of no further movement. It finished at 500 us; final duty register
readback in channels 0..3 order was **[4976, 1632, 4880, 4912]**.

## Saved controls and remaining calibration

`examples/cam_pulse_control.py` provides slow raw-pulse moves and repeated
500 -> upper -> 500 sweeps. It attaches to an already active 50 Hz controller,
does not reset the chip or change its frequency, writes channel 1 only, and
verifies the other three outputs are unchanged. It defaults to 2650 us (the
lower end of the operator estimate); 2670/2700 require an explicit argument.
No new physical run was performed while preparing this report.

`examples/gimbal_servos.py` and `planned_motion.py` retain the model-angle
experiments with `CALIBRATED=False`. The reversed model-angle mapping is retained
for traceability; its numerical 0 is **not physical rest**. Do not enable model
motion until physical angle origin, travel, direction, horn indexing, parked
clearance and release points have been calibrated. The saved pulse-based path
is the appropriate way to reproduce this session's later tests.

The source examples and report are in the repository. The board still has the
earlier helper files installed during the session; later manual commands used
explicit pulse values and direction overrides. Repository corrections and the
new pulse controller have not been copied to the live board during reporting,
so its held output is unchanged. Copy the intended source before future scripts.

Still unresolved: exact reset endpoint inside the estimated range; release pulse
and repeatability under load; whether torque shortfall, supply sag, friction,
spring load or some combination caused sticking; actual servo angular travel;
and model/physical coordinate alignment. The operator's assistance report is
consistent with load-related stalling but does not independently diagnose its cause.
