"""Gradual unloaded travel check; never use with the linkage or cam fitted.

CircuitPython REPL:
    import sweep_unloaded
    sweep_unloaded.run(detached=True, dry_run=False)

Default is a no-hardware preview. Ends holding each servo at neutral.
"""

from gimbal_servos import Rig


def run(detached=False, dry_run=True):
    if not detached:
        raise ValueError("Full travel check requires detached linkage and cam")
    rig = Rig(dry_run=dry_run)
    try:
        rig.center(detached=True)
        for name in ("tilt", "cam", "roll", "pan"):
            spec = rig.servos[name]
            current = spec["center_us"]
            print("SWEEP_START", name, "channel", spec["channel"])
            for target in (spec["min_us"], spec["max_us"], spec["center_us"]):
                # 500 us/second; 10 us increments at 50 Hz. A full span takes ~4 s.
                steps = max(1, int(abs(target - current) / 10))
                start = current
                for index in range(1, steps + 1):
                    current = start + (target - start) * index / steps
                    rig._write(name, current)
                    rig.sleep(0.02)
                print("REACHED", name, target, "us")
                rig.sleep(0.25)
            print("SWEEP_DONE", name, "holding neutral")
    except BaseException:
        # This procedure is only for detached servos; remove drive on interruption.
        rig.off()
        raise
    print("ALL_SWEEPS_DONE_HOLDING_NEUTRAL")
    return rig
