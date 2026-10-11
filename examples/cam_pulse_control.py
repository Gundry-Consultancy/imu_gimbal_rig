"""Slow cam-only bench control, leaving the three aligned axes untouched.

CircuitPython REPL (or Blinka with BLINKA_U2IF=1 before importing board):
    import cam_pulse_control as cam
    cam.sweep()  # Preview, no hardware access.
    cam.sweep(execute=True, confirmed=True, cycles=2)

Physical rest: 500 us. Upper endpoint estimated at 2650--2670 us; default
2650 avoids treating the exploratory 2700 us command as a measured limit.
"""

import math
import time

from gimbal_servos import ADDRESS, REFERENCE_CLOCK

REST_US = 500
RESET_ESTIMATE_US = (2650, 2670)
DEFAULT_UPPER_US = RESET_ESTIMATE_US[0]
MAX_TESTED_US = 2700
SPEED_US_PER_SECOND = 200


def validate_target(target):
    if not isinstance(target, (int, float)) or not math.isfinite(target):
        raise ValueError("Pulse must be a finite number in microseconds")
    if not REST_US <= target <= MAX_TESTED_US:
        raise ValueError("Bench controller allows 500--2700 us only")


class CamPulse:
    """Attach to active PCA9685 settings without resetting or changing frequency."""

    def __init__(self):
        import board

        self.bus = board.I2C()
        deadline = time.monotonic() + 2
        while not self.bus.try_lock():
            if time.monotonic() >= deadline:
                raise RuntimeError("I2C bus busy")
            time.sleep(0.001)
        try:
            mode = self._read(0)
            self.frequency = REFERENCE_CLOCK / 4096 / (self._read(254) + 1)
            if mode & 0x10 or not mode & 0x20 or not 49 < self.frequency < 51:
                raise ValueError(
                    "Initialize the servo controller at 50 Hz with auto-increment first"
                )
            self.others = self._other_outputs()
            self.current = self._cam_count() / 4096 / self.frequency * 1000000
        finally:
            self.bus.unlock()

    def _read(self, register):
        data = bytearray(1)
        self.bus.writeto_then_readfrom(ADDRESS, bytes([register]), data)
        return data[0]

    def _other_outputs(self):
        return bytes(
            self._read(6 + 4 * ch + offset) for ch in (0, 2, 3) for offset in range(4)
        )

    def _cam_count(self):
        data = [self._read(register) for register in range(10, 14)]
        if data[0] or data[1] or data[3] & 0x10:
            raise ValueError("Cam output is disabled or uses an unexpected ON count")
        return data[2] | (data[3] << 8)

    def move_us(self, target):
        validate_target(target)
        if not self.bus.try_lock():
            raise RuntimeError("I2C bus busy")
        try:
            if self._other_outputs() != self.others:
                raise RuntimeError("Another controller changed the other axes")
            mode = self._read(0)
            frequency = REFERENCE_CLOCK / 4096 / (self._read(254) + 1)
            if mode & 0x10 or not mode & 0x20 or frequency != self.frequency:
                raise RuntimeError("Controller mode/frequency changed")
            start = self._cam_count() / 4096 / self.frequency * 1000000
            duration = max(0.2, abs(target - start) / SPEED_US_PER_SECOND)
            steps = max(1, math.ceil(duration / 0.05))
            for index in range(1, steps + 1):
                pulse = start + (target - start) * index / steps
                count = round(pulse * self.frequency * 65535 / 1000000) >> 4
                self.bus.writeto(ADDRESS, bytes([10, 0, 0, count & 255, count >> 8]))
                time.sleep(duration / steps)
            if self._cam_count() != count or self._other_outputs() != self.others:
                raise RuntimeError("Output verification failed")
            self.current = target
            print("Cam holding", target, "us; other axes unchanged")
        finally:
            self.bus.unlock()


def sweep(cycles=1, upper_us=DEFAULT_UPPER_US, execute=False, confirmed=False):
    if type(cycles) is not int or cycles < 1:
        raise ValueError("cycles must be a positive integer")
    validate_target(upper_us)
    if upper_us <= REST_US:
        raise ValueError("Upper endpoint must exceed rest")
    if not execute:
        print(
            "Preview: begin at 500 us, then", cycles, "cycles of", upper_us, "-> 500 us"
        )
        return
    if not confirmed:
        raise ValueError(
            "Confirm servo power, physical rest mapping and mechanism clearance"
        )
    cam = CamPulse()
    cam.move_us(REST_US)
    time.sleep(1)
    for _ in range(cycles):
        cam.move_us(upper_us)
        time.sleep(1)
        cam.move_us(REST_US)
        time.sleep(1)
    return cam
