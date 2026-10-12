"""Run the documented model sequence after confirming assembly and direction.

Importing does nothing. run() is a dry preview; real motion requires
run(confirmed=True, dry_run=False). Starts and ends at axes home / cam rest.
"""

from gimbal_servos import ADDRESS, CALIBRATED, REFERENCE_CLOCK, SERVOS, Rig, pulse_for


def check_home_rest(allow_reset=False):
    """Read back commands, not physical position; operator must confirm alignment."""
    import board

    bus = board.I2C()
    while not bus.try_lock():
        pass
    try:
        data = bytearray(16)
        prescale = bytearray(1)
        mode = bytearray(1)
        bus.writeto_then_readfrom(ADDRESS, bytes([0]), mode)
        bus.writeto_then_readfrom(ADDRESS, bytes([0xFE]), prescale)
        # Auto-increment is off after power-on; single-byte reads work in either state.
        value = bytearray(1)
        for index in range(16):
            bus.writeto_then_readfrom(ADDRESS, bytes([0x06 + index]), value)
            data[index] = value[0]
        if mode[0] == 0x11 and prescale[0] == 30 and data == bytes([0, 0, 0, 16]) * 4:
            if not allow_reset:
                raise ValueError("Controller reset; home/rest outputs are not active")
            print("CONTROLLER_RESET_OUTPUTS_INACTIVE_RESTORE_CONFIRMED_HOME")
            return
        frequency = REFERENCE_CLOCK / 4096 / (prescale[0] + 1)
        if mode[0] & 0x10 or not 49 < frequency < 51:
            raise ValueError("Expected existing active 50 Hz servo configuration")
        for name, spec in SERVOS.items():
            ch = spec["channel"]
            if ch not in range(4):
                raise ValueError(
                    "This preflight supports the current first-four-channel wiring"
                )
            index = ch * 4
            expected = round(pulse_for(name, 0) * frequency * 65535 / 1000000) >> 4
            actual = data[index + 2] | (data[index + 3] << 8)
            if data[index] != 0 or data[index + 1] != 0 or actual != expected:
                raise ValueError("Expected home/rest PWM before starting: " + name)
    finally:
        bus.unlock()


def run(confirmed=False, dry_run=True):
    if not dry_run:
        if not confirmed:
            raise ValueError(
                "Confirm physical home/rest, assembly, cam direction and clearance"
            )
        if not CALIBRATED:
            raise ValueError(
                "Complete hardware calibration before running the model sequence"
            )
        check_home_rest(allow_reset=True)
    rig = Rig(dry_run=dry_run, calibrated=True if dry_run else CALIBRATED)
    # Preview can exercise model angles; live runs require saved hardware calibration.
    print("INITIAL_HOME_REST")
    for name in ("cam", "pan", "tilt", "roll"):
        rig._write(name, pulse_for(name, 0))
    rig.sleep(1)
    print("PARK_CAM_LOBE1")
    rig.start_home_rest(confirmed=True)
    for name, positions in (
        ("pan", (135, -135, 0)),
        ("tilt", (rig.servos["tilt"]["travel"] / 2, 0)),
        ("roll", (90, -90, 0)),
    ):
        for angle in positions:
            print("AXIS_MOVE", name, angle)
            rig.move(name, angle, duration=2.0)
    print("SINGLE_TAP")
    rig.tap()
    print("REARM_AND_DOUBLE_TAP")
    rig.tap(double=True)
    print("RETURN_CAM_REST")
    rig.rest()
    print("PLANNED_MOTION_DONE_HOME_REST", rig.pose)
    return rig
