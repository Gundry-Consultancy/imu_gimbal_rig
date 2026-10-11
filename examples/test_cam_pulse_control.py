"""Hardware-free checks for the pulse path and model-sequence gate."""

import contextlib
import io
import math
import sys
import types
import unittest
from unittest.mock import patch

import cam_pulse_control as cam
import planned_motion


class FakeBus:
    def __init__(self):
        self.reg = {0: 0xA0, 254: 121}
        self.locked = False
        self.writes = []
        for ch, count in enumerate((311, 102, 305, 307)):
            for offset, value in enumerate((0, 0, count & 255, count >> 8)):
                self.reg[6 + 4 * ch + offset] = value

    def try_lock(self):
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def writeto_then_readfrom(self, address, register, data):
        assert self.locked and address == 0x40
        for index in range(len(data)):
            data[index] = self.reg.get(register[0] + index, 0)

    def writeto(self, address, data):
        assert self.locked and address == 0x40
        self.writes.append(bytes(data))
        for offset, value in enumerate(data[1:]):
            self.reg[data[0] + offset] = value


class PulseTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()
        self.board = patch.dict(
            sys.modules, {"board": types.SimpleNamespace(I2C=lambda: self.bus)}
        )
        self.board.start()
        self.addCleanup(self.board.stop)
        self.sleep = patch.object(cam.time, "sleep")
        self.sleep.start()
        self.addCleanup(self.sleep.stop)

    def test_attach_never_writes(self):
        controller = cam.CamPulse()
        self.assertAlmostEqual(controller.current, 497.76)
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.locked)

    def test_sweep_preserves_axes_and_returns_to_rest(self):
        other = {k: v for k, v in self.bus.reg.items() if k not in range(10, 14)}
        with contextlib.redirect_stdout(io.StringIO()):
            controller = cam.sweep(execute=True, confirmed=True, cycles=2)
        self.assertEqual(controller.current, 500)
        self.assertTrue(all(write[0] == 10 for write in self.bus.writes))
        counts = [write[3] | (write[4] << 8) for write in self.bus.writes]
        self.assertEqual(max(counts), 543)  # Default 2650 us, not 2700.
        self.assertEqual(counts[-1], 102)
        self.assertEqual(
            other, {k: v for k, v in self.bus.reg.items() if k not in range(10, 14)}
        )
        self.assertFalse(self.bus.locked)

    def test_bad_target_never_writes(self):
        controller = cam.CamPulse()
        for target in (499, 2701, math.nan, math.inf):
            with self.assertRaises(ValueError):
                controller.move_us(target)
        self.assertEqual(self.bus.writes, [])

    def test_reset_does_not_initialize_or_move(self):
        self.bus.reg[0] = 0x11
        self.bus.reg[254] = 30
        with self.assertRaises(ValueError):
            cam.CamPulse()
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.locked)

    def test_auto_increment_disabled_rejected_before_writes(self):
        self.bus.reg[0] &= ~0x20
        with self.assertRaises(ValueError):
            cam.CamPulse()
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.locked)

    def test_auto_increment_change_stops_before_move(self):
        controller = cam.CamPulse()
        self.bus.reg[0] &= ~0x20
        with self.assertRaises(RuntimeError):
            controller.move_us(2650)
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.locked)

    def test_external_axis_change_stops_before_move(self):
        controller = cam.CamPulse()
        self.bus.reg[6] = 1
        with self.assertRaises(RuntimeError):
            controller.move_us(2500)
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.locked)

    def test_frequency_change_stops_before_move(self):
        controller = cam.CamPulse()
        self.bus.reg[254] = 120
        with self.assertRaises(RuntimeError):
            controller.move_us(2500)
        self.assertEqual(self.bus.writes, [])

    def test_live_sweep_requires_confirmation(self):
        with self.assertRaises(ValueError):
            cam.sweep(execute=True)
        self.assertEqual(self.bus.writes, [])

    def test_model_run_requires_calibration_before_i2c(self):
        with patch.object(planned_motion, "CALIBRATED", False):
            with patch.object(planned_motion, "check_home_rest") as preflight:
                with self.assertRaises(ValueError):
                    planned_motion.run(confirmed=True, dry_run=False)
                preflight.assert_not_called()
        self.assertEqual(self.bus.writes, [])

    def test_reset_cannot_pass_final_home_verification(self):
        self.bus.reg[0] = 0x11
        self.bus.reg[254] = 30
        for ch in range(4):
            for offset, value in enumerate((0, 0, 0, 16)):
                self.bus.reg[6 + ch * 4 + offset] = value
        with self.assertRaises(ValueError):
            planned_motion.check_home_rest()
        with contextlib.redirect_stdout(io.StringIO()):
            planned_motion.check_home_rest(allow_reset=True)
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
