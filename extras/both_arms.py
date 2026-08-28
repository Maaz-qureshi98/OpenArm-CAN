#!/usr/bin/env python3
"""Bring up BOTH arms at once and put them through a short routine.

  right arm (default can0): ramp to HOME_ANGLES, cycle gripper
  left  arm (default can1): ramp to HOME_ANGLES (with --left-home) or a safe
                            shoulder rock, then rotate the end-effector
                            (joint_7) +/-0.15 rad, then cycle gripper

Both arms run on their own 50 Hz command thread at the same time. At the end
both hold, a 12 s countdown runs, then the gains on both fade out together --
keep a hand on each arm.

    python3 both_arms.py --left-home           # both go to the same pose
    python3 both_arms.py                        # left just rocks its shoulder
    python3 both_arms.py --right-bus can1 --left-bus can0
"""
import argparse
import sys
import threading
import time

import numpy as np

from arm_driver import (ArmDriver, ArmDriverError, GRIPPER_MAX_MM, HOME_ANGLES,
                        JOINT_ORDER)

J1 = JOINT_ORDER.index("joint_1")
J6 = JOINT_ORDER.index("joint_6")
J7 = JOINT_ORDER.index("joint_7")


def start_driver(bus, kp):
    d = ArmDriver(channel=bus, live=True, kp=kp, kd=2.0, gripper_kp=10.0,
                  max_joint_speed=0.30)
    d.start()
    return d


def gripper_cycle(d, tag):
    print(f"[{tag}] gripper: close 20 mm", flush=True)
    d.set_gripper(20.0, wait=True)
    time.sleep(1.5)
    print(f"[{tag}] gripper: open {GRIPPER_MAX_MM:.0f} mm  (meas {d.gripper_mm():.1f})", flush=True)
    d.set_gripper(GRIPPER_MAX_MM, wait=True)
    time.sleep(1.5)
    print(f"[{tag}] gripper open meas {d.gripper_mm():.1f} mm", flush=True)


def ee_rotate(d, tag, amp=0.15):
    base = d.positions().copy()
    print(f"[{tag}] end-effector rotate joint_7 +/-{amp} rad", flush=True)
    for sign in (+1, -1, +1, -1):
        tgt = base.copy()
        tgt[J7] = base[J7] + sign * amp
        d.goto(tgt, duration=1.2)
    d.goto(base, duration=1.2)


def right_routine(d):
    d.go_home(duration=8.0)
    gripper_cycle(d, "RIGHT")


def left_routine(d, amp, do_home):
    if do_home:
        d.go_home(duration=8.0)
    else:
        home = d.positions().copy()
        for _ in range(2):
            for sign in (+1, -1, +1, -1):
                tgt = home.copy()
                tgt[J1] = home[J1] + sign * amp
                d.goto(tgt, duration=1.2)
        d.goto(home, duration=1.5)
    ee_rotate(d, "LEFT")
    gripper_cycle(d, "LEFT")


def terr(d, ref):
    e = np.abs(d.positions() - ref)
    e[J6] = 0.0
    return e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--right-bus", default="can0")
    ap.add_argument("--left-bus", default="can1")
    ap.add_argument("--kp", type=float, default=40.0)
    ap.add_argument("--amp", type=float, default=0.08, help="left shoulder rock, rad (no --left-home)")
    ap.add_argument("--left-home", action="store_true",
                    help="Drive the LEFT arm to HOME_ANGLES too (verify that pose is safe first).")
    args = ap.parse_args()

    drivers = {}
    for tag, bus in (("RIGHT", args.right_bus), ("LEFT", args.left_bus)):
        try:
            drivers[tag] = start_driver(bus, args.kp)
            print(f"[{tag} {bus}] up. pose: "
                  f"{dict(zip(JOINT_ORDER, np.round(drivers[tag].positions(),3)))}")
        except ArmDriverError as e:
            print(f"[{tag} {bus}] FAILED to start: {e}")

    if not drivers:
        print("no arms came up.")
        return 1

    try:
        threads = []
        if "RIGHT" in drivers:
            threads.append(threading.Thread(target=right_routine, args=(drivers["RIGHT"],)))
        if "LEFT" in drivers:
            threads.append(threading.Thread(
                target=left_routine, args=(drivers["LEFT"], args.amp, args.left_home)))
        print("\nrunning both arms now ...")
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        print("\nboth arms done -- 10 s hold:")
        for _ in range(5):
            time.sleep(2.0)
            parts = []
            for tag, d in drivers.items():
                ref = HOME_ANGLES if (tag == "RIGHT" or args.left_home) else d.commanded()
                e = terr(d, ref)
                parts.append(f"{tag}: err {e.max():.3f}  send_err {d._send_errors}  "
                             f"faults {d.check_faults()}  grip {d.gripper_mm():.0f}mm")
            print("   " + "  |  ".join(parts))

        print("\nRELEASING BOTH ARMS in 12 s -- put a hand on each arm.")
        for i in range(12, 0, -1):
            print(f"   {i}", end=" ", flush=True)
            time.sleep(1.0)
        print()
    except KeyboardInterrupt:
        print("\ninterrupted -- freezing both arms.")
        for d in drivers.values():
            d.hold_here()
    finally:
        for tag, d in drivers.items():
            print(f"releasing {tag} ...")
            d.release(ramp_sec=4.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
