"""Desktop example. Preview by default; --execute uses Blinka/U2IF."""

import argparse
import os

from gimbal_servos import Rig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("center", "pulse", "sweep", "single", "double", "off")
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--detached", action="store_true", help="Horns/cam disconnected"
    )
    parser.add_argument(
        "--at-home-rest", action="store_true", help="Physical axes home, cam rest"
    )
    parser.add_argument("--axis", choices=("pan", "tilt", "roll", "cam"), default="pan")
    parser.add_argument("--us", type=float)
    parser.add_argument("--amplitude", type=float, default=10)
    parser.add_argument("--gap", type=float, default=0)
    args = parser.parse_args()
    os.environ["BLINKA_U2IF"] = "1"  # Must precede import of board (lazy in Rig).
    rig = Rig(dry_run=not args.execute)
    try:
        if args.command == "center":
            rig.center(detached=args.detached)
        elif args.command == "pulse":
            rig.pulse(args.axis, args.us, detached=args.detached)
        elif args.command == "off":
            rig.off()
        else:
            # Preflight all arguments before parking or enabling output.
            from gimbal_servos import number, pulse_for

            if args.command == "sweep":
                if args.axis == "cam" or number(args.amplitude) <= 0:
                    raise ValueError("Sweep needs an axis and positive amplitude")
                pulse_for(args.axis, args.amplitude)
                pulse_for(args.axis, -args.amplitude)
            if number(args.gap) < 0:
                raise ValueError("Gap must be nonnegative")
            if not args.execute:
                rig.calibrated = True  # Simulation only.
            rig.start_home_rest(confirmed=args.at_home_rest or not args.execute)
            if args.command == "sweep":
                rig.sweep(args.axis, args.amplitude)
            else:
                rig.tap(double=args.command == "double", gap=args.gap)
            rig.home()
            rig.rest()
    except (ValueError, KeyError) as error:
        parser.exit(2, str(error) + "\n")
    except KeyboardInterrupt:
        print(
            "Interrupted. PWM retains last command; support the rig before disabling power."
        )
        return 130
    if args.execute:
        print(
            "Finished. PWM holds its last command until off() or servo power is removed."
        )
    else:
        print("Preview complete; no hardware accessed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
