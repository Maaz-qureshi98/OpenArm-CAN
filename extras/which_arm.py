#!/usr/bin/env python3
"""Identify which physical arm is on a CAN bus by wiggling the shoulder.

Energizes the arm on --bus, holds, then rocks joint_1 by +/-0.08 rad twice
(small, safe on either arm), returns, and releases. Watch which arm moves.

    python3 which_arm.py --bus can0
    python3 which_arm.py --bus can1
"""
import argparse
import sys
import time

import numpy as np

from arm_driver import ArmDriver, ArmDriverError, JOINT_ORDER

J1 = JOINT_ORDER.index("joint_1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", required=True)
    ap.add_argument("--amp", type=float, default=0.08, help="rad, shoulder rock amplitude")
    ap.add_argument("--kp", type=float, default=40.0)
    args = ap.parse_args()

    try:
        d = ArmDriver(channel=args.bus, live=True, kp=args.kp, kd=2.0, max_joint_speed=0.25)
        d.start()
    except ArmDriverError as e:
        print(f"[{args.bus}] {e}")
        return 1

    try:
        home = d.positions().copy()
        print(f"[{args.bus}] shoulder now at {home[J1]:+.3f} rad -- rocking +/-{args.amp} rad. "
              f"WATCH THE ARMS.")
        for i in range(2):
            for sign in (+1, -1, +1, -1):
                tgt = home.copy()
                tgt[J1] = home[J1] + sign * args.amp
                d.goto(tgt, duration=1.2)
            print(f"    cycle {i+1} done")
        d.goto(home, duration=1.5)
        print(f"[{args.bus}] back to start. Releasing in 8 s -- hold the arm.")
        for k in range(8, 0, -1):
            print(f"   {k}", end=" ", flush=True)
            time.sleep(1)
        print()
    except KeyboardInterrupt:
        d.hold_here()
    finally:
        d.release(ramp_sec=4.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
