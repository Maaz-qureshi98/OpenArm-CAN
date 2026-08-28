#!/usr/bin/env python3
"""Exercise EVERY joint + the gripper on one arm and report per-joint.

For each live joint (joint_1..5, joint_7 -- joint_6 is dead) it moves the joint
+delta then -delta then back, from wherever the arm currently is, and checks:
  * the joint actually moved (not stuck)
  * it settled near the commanded target (tracking OK)
Then it fully closes and opens the gripper and checks the width tracks.

Works on either arm -- it perturbs from the current pose, no home needed.

    python3 check_all_joints.py --bus can0     # right arm
    python3 check_all_joints.py --bus can1     # left arm
"""
import argparse
import sys
import time

import numpy as np

from arm_driver import (ARM_JOINTS, ArmDriver, ArmDriverError, GRIPPER_MAX_MM,
                        GRIPPER_NAME, JOINT_ORDER)

LIVE = [n for n, _, _ in ARM_JOINTS]           # joint_1..5, joint_7
SETTLE_TOL = 0.12                              # rad
MIN_MOVE_FRAC = 0.4                            # must achieve >= this * delta

# Conservative per-joint soft limits (rad), kept a hair inside the URDF ones so
# the test never commands a joint into its hard stop.
SOFT_LIM = {
    "joint_1": (-1.35, 3.45), "joint_2": (-0.15, 3.30), "joint_3": (-1.55, 1.55),
    "joint_4": (0.05, 2.40), "joint_5": (-1.55, 1.55), "joint_7": (-1.55, 1.55),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", required=True)
    ap.add_argument("--delta", type=float, default=0.12, help="per-joint test move, rad")
    ap.add_argument("--kp", type=float, default=45.0)
    args = ap.parse_args()

    print(f"=== per-joint check on {args.bus}  (delta {args.delta} rad, kp {args.kp}) ===\n")
    try:
        d = ArmDriver(channel=args.bus, live=True, kp=args.kp, kd=2.0,
                      gripper_kp=10.0, max_joint_speed=0.30)
        d.start()
    except ArmDriverError as e:
        print(f"[FAIL] {e}")
        return 1

    rows = []
    try:
        # ---- probe table ----
        print(f"{'motor':9s} {'fb':4s} {'pos(rad)':>9s} {'status':>9s} {'T':>4s}")
        for name, mid, _ in ARM_JOINTS + [(GRIPPER_NAME, 8, None)]:
            st = d.motors[name].get_states()
            if st:
                print(f"{name:9s} {'OK':4s} {st['pos']:>9.3f} {st['status']:>9s} {st.get('t_rotor',0):>3.0f}C")
            else:
                print(f"{name:9s} {'--':4s}  no feedback")
        base = d.positions().copy()
        print()

        # ---- per-joint ----
        for name in LIVE:
            j = JOINT_ORDER.index(name)
            p0 = d.positions()[j]
            lo, hi = SOFT_LIM[name]
            moved = 0.0
            errs = []
            for sgn in (+1, -1):
                want = p0 + sgn * args.delta
                clamped = float(np.clip(want, lo, hi))
                if abs(clamped - p0) < 0.02:
                    print(f"  {name:9s} at {p0:+.3f} rad, near limit [{lo},{hi}] -- "
                          f"skipping {'+' if sgn > 0 else '-'} direction")
                    continue
                tgt = d.positions().copy()
                tgt[j] = clamped
                d.goto(tgt, duration=1.6)
                meas = d.positions()[j]
                errs.append(abs(meas - tgt[j]))
                moved = max(moved, abs(meas - p0))
            if not errs:
                rows.append((name, False, 0.0, 0.0, ["both dirs hit limit"]))
                print(f"  {name:9s} could not test -- sits at a joint limit")
                continue
            # return
            back = d.positions().copy(); back[j] = p0
            d.goto(back, duration=1.6)
            settled = max(errs)
            ok = settled < SETTLE_TOL and moved > MIN_MOVE_FRAC * args.delta
            rows.append((name, ok, moved, settled, d.check_faults()))
            print(f"  {name:9s} moved {moved:5.3f} rad  settle_err {settled:5.3f}  "
                  f"{'PASS' if ok else 'FAIL'}  faults={d.check_faults()}")

        # ---- gripper ----
        print()
        d.set_gripper(0.0, wait=True); time.sleep(1.5)
        g_close = d.gripper_mm()
        d.set_gripper(GRIPPER_MAX_MM, wait=True); time.sleep(1.5)
        g_open = d.gripper_mm()
        d.set_gripper(GRIPPER_MAX_MM * 0.5, wait=True); time.sleep(1.0)
        g_mid = d.gripper_mm()
        g_ok = g_close < 30.0 and g_open > 85.0 and (g_open - g_close) > 40.0
        rows.append(("gripper", g_ok, g_open - g_close, 0.0, []))
        print(f"  gripper   close->{g_close:.1f}mm  open->{g_open:.1f}mm  half->{g_mid:.1f}mm  "
              f"{'PASS' if g_ok else 'FAIL'}")

        # ---- summary ----
        npass = sum(1 for _, ok, *_ in rows if ok)
        print(f"\n=== {args.bus}: {npass}/{len(rows)} PASS ===")
        for name, ok, mv, se, fl in rows:
            if not ok:
                print(f"   FAIL {name}: moved {mv:.3f} settle {se:.3f} faults {fl}")
        print("\n  holding. RELEASING IN 15 s -- put a hand on the arm.")
        for i in range(15, 0, -1):
            print(f"   {i}", end=" ", flush=True); time.sleep(1.0)
        print()
    except KeyboardInterrupt:
        print("\ninterrupted -- freezing.")
        d.hold_here()
    finally:
        d.release(ramp_sec=4.0)
    return 0 if all(ok for _, ok, *_ in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
