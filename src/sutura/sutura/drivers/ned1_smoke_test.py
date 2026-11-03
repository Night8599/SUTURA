"""
Ned1 hardware smoke test, without ROS.

Connects, calibrates if needed, reads joints, then moves ONE joint by a small offset
and back. Every motion asks for confirmation first.

    ros2 run sutura ned1_smoke_test --ip 10.10.10.10 --joint 1 --delta 0.1
    # or, without building the package:
    python3 -m sutura.drivers.ned1_smoke_test --ip 10.10.10.10
"""

import argparse
import logging
import sys

from sutura.drivers.ned1_driver import Ned1ArmDriver


def _confirm(question: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    return input(f"{question} [y/N] ").strip().lower() == 'y'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Ned1 hardware smoke test")
    parser.add_argument('--ip', default='10.10.10.10', help="Robot IP address")
    parser.add_argument('--joint', type=int, default=1, choices=range(1, 7), help="Joint to move (1-6)")
    parser.add_argument('--delta', type=float, default=0.1, help="Offset in radians (|delta| <= 0.3)")
    parser.add_argument('--velocity', type=int, default=10, help="Max velocity percentage (1-100)")
    parser.add_argument('--home', action='store_true', help="Go to home pose before the test")
    parser.add_argument('--yes', action='store_true', help="Skip confirmations")
    args = parser.parse_args(argv)

    if abs(args.delta) > 0.3:
        parser.error("--delta is limited to 0.3 rad for a smoke test.")

    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
    driver = Ned1ArmDriver(ip_address=args.ip, velocity_percent=args.velocity, max_step_rad=0.35)

    print("Keep the power switch within reach. The arm may move during calibration.")
    if not _confirm("Connect to the robot?", args.yes):
        return 0
    if not driver.connect():
        print(f"FAILED: {driver.last_error}")
        return 1

    try:
        start = driver.get_joint_positions()
        print("Current joints (rad):", ", ".join(f"{v:.3f}" for v in start))

        if args.home and _confirm("Move to home pose?", args.yes):
            if not driver.move_to_home():
                print(f"FAILED: {driver.last_error}")
                return 1
            start = driver.get_joint_positions()

        target = list(start)
        target[args.joint - 1] += args.delta
        if not _confirm(f"Move joint_{args.joint} by {args.delta:+.3f} rad and back?", args.yes):
            return 0

        for label, goal in (("out", target), ("back", start)):
            if not driver.set_joint_positions(goal):
                print(f"FAILED ({label}): {driver.last_error}")
                return 1
            reached = driver.get_joint_positions()
            error = max(abs(a - b) for a, b in zip(reached, goal))
            print(f"{label}: max joint error {error:.4f} rad")

        print("Smoke test OK.")
        return 0
    finally:
        driver.disconnect()


if __name__ == '__main__':
    sys.exit(main())
