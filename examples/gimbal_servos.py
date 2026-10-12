"""Servo examples for CircuitPython or desktop Blinka; importing never moves hardware.

Model angles/pulses are starting values, not hardware calibration. See README.md.
"""

import math
import time

# PCA9685 labels are zero based. Verify the inferred roll connection on channel 2.
SERVOS = {
    "tilt": {
        "channel": 0,
        "travel": 180,
        "min_us": 500,
        "center_us": 1500,
        "max_us": 2500,
        "direction": 1,
    },
    "cam": {
        "channel": 1,
        "travel": 180,  # MG995 (2026-10-12): nominal 180 deg over 500-2500 us; measure
        "min_us": 500,
        "center_us": 1500,
        "max_us": 2500,
        "direction": 1,  # increasing pulse = cam forward (anticlockwise seen from the gear face)
    },
    "roll": {
        "channel": 2,
        "travel": 180,
        "min_us": 500,
        "center_us": 1450,
        "max_us": 2400,
        "direction": 1,
    },
    "pan": {
        "channel": 3,
        "travel": 270,
        "min_us": 500,
        "center_us": 1500,
        "max_us": 2500,
        "direction": 1,
    },
}
# Cam: MG995 on the tower's +X wall. Rest = model 0 = 500 us (the low-pulse end); increasing pulse
# turns the cam forward, anticlockwise seen from its gear face (as observed on 11 October).
# Model angle -> pulse: us = 500 + angle * 2000 / 180, until the MG995 is calibrated.
# Bench-confirmed rest is 500 us; use cam_pulse_control.py for the physical rig.
# Full model motion and angle-based automatic taps remain uncalibrated.
# Axis trims from the session are recorded in docs/servo_bench_2026-10-11.md.
# Change only after checking channels, endpoints, horn alignment and direction.
CALIBRATED = False
ADDRESS = 0x40
FREQUENCY = 50
REFERENCE_CLOCK = 25000000
# Cam angles (deg from rest) from make_striker.LOBES = (15, 65, 73), (97, 147, 151):
PARK_A = 69  # lobe 1 dwell 65-73: hammer parked 48 deg up (gimbal moves, single and double tap)
PARK_B = 149  # lobe 2 dwell 147-151 (reached only during a double tap)
CLIFF_A, CLIFF_B = 73, 151  # the pawl rolls off these corners by ~75.2 and ~153.2 (model)
RAMP_B = 97  # lobe 2's ramp starts here: a single tap must stop short of it
RELEASE_A = 87  # single tap: 11.8 deg past the modelled drop, 10 before lobe 2
RELEASE_B = 165  # double tap end: 11.8 deg past the second drop, 15 short of 180


def number(value):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Expected a finite number")
    return value


def validate(servos):
    if set(servos) != {"pan", "tilt", "roll", "cam"}:
        raise ValueError("Need exactly pan, tilt, roll and cam")
    channels = []
    for name, spec in servos.items():
        ch = spec["channel"]
        if type(ch) is not int or not 0 <= ch < 16 or ch in channels:
            raise ValueError("Channels must be unique integers from 0 to 15")
        channels.append(ch)
        low, mid, high = [number(spec[k]) for k in ("min_us", "center_us", "max_us")]
        if not 400 <= low < mid < high <= 2600:
            raise ValueError("Invalid pulse calibration")
        expected = (
            (180, 270) if name == "tilt" else ((180,) if name in ("roll", "cam") else (270,))
        )
        if spec["travel"] not in expected or spec["direction"] not in (-1, 1):
            raise ValueError("Invalid travel or direction")


def pulse_for(name, angle, servos=SERVOS):
    number(angle)
    s = servos[name]
    if name == "cam":
        if not 0 <= angle <= s["travel"]:
            raise ValueError("Cam angle outside servo travel")
        fraction = angle / s["travel"]
        if s["direction"] == -1:
            fraction = 1 - fraction
        return s["min_us"] + fraction * (s["max_us"] - s["min_us"])
    limit = s["travel"] / 2
    if not -limit <= angle <= limit:
        raise ValueError(name + " angle outside servo travel")
    signed = angle * s["direction"]
    span = s["max_us"] - s["center_us"] if signed >= 0 else s["center_us"] - s["min_us"]
    return s["center_us"] + signed / limit * span


class Rig:
    def __init__(
        self, dry_run=True, servos=None, calibrated=None, writer=None, sleeper=None
    ):
        self.servos = SERVOS if servos is None else servos
        validate(self.servos)
        if not CLIFF_A + 10 <= RELEASE_A <= RAMP_B - 5 or not CLIFF_B + 10 <= RELEASE_B <= 170:
            raise ValueError(
                "Release angles must clear their cliffs by 10 deg, inside the travel"
            )
        self.dry_run = dry_run
        self.calibrated = CALIBRATED if calibrated is None else calibrated
        self.writer = writer
        self.sleep = sleeper or ((lambda seconds: None) if dry_run else time.sleep)
        self.pca = None
        self.i2c = None
        self.pose = {}  # Commanded state only; these servos have no feedback.

    def _write(self, name, us):
        channel = self.servos[name]["channel"]
        if self.writer is not None:
            self.writer(channel, us)
        elif self.dry_run:
            value = "OFF" if us is None else f"{us:.1f}"
            print(f"ch{channel} {name:4s} {value} us")
        else:
            if self.pca is None:
                import board
                from adafruit_pca9685 import PCA9685

                self.i2c = board.I2C()
                self.pca = PCA9685(
                    self.i2c, address=ADDRESS, reference_clock_speed=REFERENCE_CLOCK
                )
                self.pca.frequency = FREQUENCY
            # Use actual prescaled frequency, not the requested nominal frequency.
            duty = 0 if us is None else round(us * self.pca.frequency * 65535 / 1000000)
            self.pca.channels[channel].duty_cycle = duty

    def center(self, detached=False):
        """Horn fitting only. Cam neutral is NOT mechanical rest."""
        if not detached:
            raise ValueError("Center requires detached horns/cam")
        self.pose = {}
        for name in self.servos:
            self._write(name, self.servos[name]["center_us"])
        self.sleep(1)

    def pulse(self, name, us, detached=False):
        """Unloaded calibration; adjust in small increments near neutral first."""
        s = self.servos[name]
        if not detached or not s["min_us"] <= number(us) <= s["max_us"]:
            raise ValueError(
                "Pulse requires detached linkage and calibrated pulse bounds"
            )
        self.pose = {}
        self._write(name, us)
        self.sleep(1)

    def start_home_rest(self, confirmed=False):
        """Operator has physically checked axes at home and cam at rest."""
        if not confirmed or not self.calibrated:
            raise ValueError(
                "Confirm physical home/rest and set CALIBRATED after bench checks"
            )
        self.pose = {"pan": 0, "tilt": 0, "roll": 0, "cam": 0}
        self._cam(PARK_A)
        for name in ("pan", "tilt", "roll"):
            self._write(name, pulse_for(name, 0, self.servos))
        self.sleep(1)

    def _ready(self):
        if len(self.pose) != 4:
            raise ValueError("Call start_home_rest first; physical state is unknown")

    def _cam(self, angle, seconds_per_60=0.15, settle=0.2):
        self._ready()
        number(angle)
        if CLIFF_A - 2 < angle < RELEASE_A or CLIFF_B - 2 < angle < RELEASE_B:
            raise ValueError("Do not hold at a cam release corner")
        pulse = pulse_for("cam", angle, self.servos)
        old = self.pose["cam"]
        self._write("cam", pulse)
        # Wait for estimated travel PLUS settle; PWM writes are not motion feedback.
        self.sleep(abs(angle - old) / 60 * seconds_per_60 + settle)
        self.pose["cam"] = angle

    def park(self):
        self._ready()
        if self.pose["cam"] != PARK_A:
            # Turning back folds the pawl over the cliffs and lowers the hammer (no tap), so
            # always go back to rest and forward onto lobe 1.
            if self.pose["cam"] > PARK_A:
                self._cam(0)
            self._cam(PARK_A)

    def move(self, name, angle, duration=1.0):
        self._ready()
        if name not in ("pan", "tilt", "roll"):
            raise ValueError("Use the named cam procedures")
        pulse_for(name, angle, self.servos)  # Validate before any hardware write.
        if number(duration) < 0.2:
            raise ValueError("Move duration must be at least 0.2 seconds")
        if name in ("tilt", "roll"):
            self.park()
        old = self.pose[name]
        steps = max(1, int(duration * 50))
        for i in range(1, steps + 1):
            fraction = i / steps
            eased = fraction * fraction * (3 - 2 * fraction)
            current = old + (angle - old) * eased
            self._write(name, pulse_for(name, current, self.servos))
            self.pose[name] = current
            self.sleep(duration / steps)
        self.sleep(0.2)

    def home(self):
        for name in ("pan", "tilt", "roll"):
            self.move(name, 0)

    def sweep(self, name, amplitude=10):
        number(amplitude)
        if amplitude <= 0:
            raise ValueError("Amplitude must be positive")
        if name not in ("pan", "tilt", "roll"):
            raise ValueError("Sweep an axis, not the cam")
        for angle in (amplitude, -amplitude, 0):
            pulse_for(name, angle, self.servos)
        for angle in (amplitude, -amplitude, 0):
            self.move(name, angle)

    def rest(self):
        self._ready()
        if self.pose["tilt"] != 0 or self.pose["roll"] != 0:
            raise ValueError("Home tilt and roll before lowering the hammer")
        self._cam(0)

    def rearm(self):
        self.rest()
        self._cam(PARK_A)

    def tap(self, double=False, gap=0.0):
        """From rest: park on lobe 1, then a single tap (on to RELEASE_A) or a double tap (one
        full-speed move on to RELEASE_B: ~78 deg between drops, ~0.21-0.26 s at no-load speed).
        gap adds seconds between the taps, by easing over lobe 2 to its dwell first."""
        if number(gap) < 0:
            raise ValueError("Extra tap gap cannot be negative")
        self.rearm()
        if not double:
            self._cam(RELEASE_A, seconds_per_60=0.1)
            return
        if gap:
            self._cam(RELEASE_A, seconds_per_60=0.1, settle=0)
            # Traverse lobe 2 monotonically to its dwell; never hold in a release corner.
            steps = max(1, int(gap * 50))
            for i in range(1, steps + 1):
                angle = RELEASE_A + (PARK_B - RELEASE_A) * i / steps
                self._write("cam", pulse_for("cam", angle, self.servos))
                self.sleep(gap / steps)
                self.pose["cam"] = angle
        self._cam(RELEASE_B, seconds_per_60=0.1)

    def off(self):
        """Remove holding torque: support the mechanism first."""
        self.pose = {}
        for name in self.servos:
            self._write(name, None)
