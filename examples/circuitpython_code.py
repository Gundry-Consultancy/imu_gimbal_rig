"""Optional code.py: no PWM writes until a serial-console command is entered.

Copy gimbal_servos.py and required Adafruit libraries to /lib first.
The CENTER command is only for disconnected horns/cam.
"""

from gimbal_servos import Rig

rig = Rig(dry_run=False)
print("Servo setup. Horns/cam must be detached for CENTER.")
print("Type CENTER to hold neutral; OFF to remove holding torque.")
print("No servo output is changed by startup. Ctrl-C gives the REPL for later tests.")
while True:
    command = input("> ").strip()
    if command == "CENTER":
        rig.center(detached=True)
        print("Neutral pulses active. Cam neutral is NOT cam rest.")
    elif command == "OFF":
        rig.off()
        print("All four configured outputs disabled.")
    else:
        print("Enter CENTER or OFF (uppercase).")
