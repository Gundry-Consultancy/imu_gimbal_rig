"""Hardware-free checks: python -m unittest discover -s examples -v"""

import copy
import math
import unittest

from gimbal_servos import SERVOS, Rig, pulse_for, validate


class ServoTests(unittest.TestCase):
    def rig(self, ready=False):
        writes, waits = [], []
        rig = Rig(
            calibrated=True,
            writer=lambda ch, us: writes.append((ch, us)),
            sleeper=waits.append,
        )
        if ready:
            rig.start_home_rest(confirmed=True)
            writes.clear()
            waits.clear()
        return rig, writes, waits

    def test_pulse_mapping(self):
        for name in SERVOS:
            if name != "cam":
                limit = SERVOS[name]["travel"] / 2
                self.assertEqual(pulse_for(name, -limit), SERVOS[name]["min_us"])
                self.assertEqual(pulse_for(name, 0), SERVOS[name]["center_us"])
                self.assertEqual(pulse_for(name, limit), SERVOS[name]["max_us"])
        self.assertEqual(pulse_for("cam", 0), 2500)
        self.assertEqual(pulse_for("cam", 270), 500)

    def test_reverse_and_270_tilt(self):
        config = copy.deepcopy(SERVOS)
        config["cam"]["direction"] = 1
        config["tilt"]["travel"] = 270
        config["tilt"]["direction"] = -1
        validate(config)
        self.assertEqual(pulse_for("cam", 0, config), 500)
        self.assertEqual(pulse_for("cam", 270, config), 2500)
        self.assertEqual(pulse_for("tilt", 135, config), 500)

    def test_bad_config_and_nonfinite_angles(self):
        config = copy.deepcopy(SERVOS)
        config["pan"]["channel"] = 0
        with self.assertRaises(ValueError):
            validate(config)
        for angle in (math.nan, math.inf, -math.inf, 136):
            with self.assertRaises(ValueError):
                pulse_for("pan", angle)

    def test_center_requires_detached_and_is_not_rest(self):
        rig, writes, _ = self.rig()
        with self.assertRaises(ValueError):
            rig.center()
        self.assertEqual(writes, [])
        rig.center(detached=True)
        self.assertEqual(writes, [(0, 1500), (1, 1500), (2, 1450), (3, 1500)])
        self.assertEqual(rig.pose, {})

    def test_uncalibrated_start_does_not_write(self):
        rig, writes, _ = self.rig()
        rig.calibrated = False
        with self.assertRaises(ValueError):
            rig.start_home_rest(confirmed=True)
        self.assertEqual(writes, [])

    def test_start_parks_and_waits_before_axes(self):
        rig, writes, waits = self.rig()
        rig.start_home_rest(confirmed=True)
        self.assertEqual(writes[0], (1, pulse_for("cam", 120)))
        self.assertGreaterEqual(waits[0], 0.5)
        self.assertEqual([ch for ch, _ in writes[1:]], [3, 0, 2])

    def test_tilt_parks_before_motion(self):
        rig, writes, _ = self.rig(ready=True)
        rig.rest()
        writes.clear()
        rig.move("tilt", 10)
        self.assertEqual(writes[0], (1, pulse_for("cam", 120)))
        self.assertTrue(all(ch == 0 for ch, _ in writes[1:]))

    def test_pan_does_not_park(self):
        rig, writes, _ = self.rig(ready=True)
        rig.rest()
        writes.clear()
        rig.move("pan", 10)
        self.assertTrue(all(ch == 3 for ch, _ in writes))

    def test_rest_requires_home(self):
        rig, writes, _ = self.rig(ready=True)
        rig.move("roll", 5)
        writes.clear()
        with self.assertRaises(ValueError):
            rig.rest()
        self.assertEqual(writes, [])

    def test_invalid_sweep_does_not_write(self):
        rig, writes, _ = self.rig(ready=True)
        for amplitude in (100, -10, math.nan):
            with self.assertRaises(ValueError):
                rig.sweep("roll", amplitude)
        self.assertEqual(writes, [])

    def test_double_tap_sequence_and_gap(self):
        rig, writes, waits = self.rig(ready=True)
        rig.tap(double=True)
        self.assertEqual(
            writes, [(1, pulse_for("cam", a)) for a in (0, 120, 134, 244, 254)]
        )
        # Between the two release commands: first travel + ramp travel, no 0.2s hold.
        self.assertLess(sum(waits[2:-1]), 0.25)
        rig2, writes2, waits2 = self.rig(ready=True)
        rig2.tap(double=True, gap=0.5)
        self.assertGreater(len(writes2), len(writes))
        self.assertAlmostEqual(sum(waits2) - sum(waits), 0.5)

    def test_release_corner_rejected(self):
        rig, writes, _ = self.rig(ready=True)
        for angle in (126, 133, 246, 253):
            with self.assertRaises(ValueError):
                rig._cam(angle)
        self.assertEqual(writes, [])

    def test_off_invalidates_state(self):
        rig, writes, _ = self.rig(ready=True)
        rig.off()
        self.assertEqual(writes, [(s["channel"], None) for s in SERVOS.values()])
        with self.assertRaises(ValueError):
            rig.move("pan", 1)


if __name__ == "__main__":
    unittest.main()
