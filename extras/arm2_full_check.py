#!/usr/bin/env python3
"""Full arm2 health check -- one scripted run, no interactive prompt.

Battery:
  1. zero-torque probe: all 7 motors + gripper answer, ENABLED, sane temp
  2. ramp to home (8 s), report per-joint tracking error
  3. hold 20 s: sample tracking error / send_errors / faults / temp -> catches
     gravity sag, drift, CAN frame drops under a steady command load
  4. closed-loop move test: nudge joint_1 and joint_4 +/-0.2 rad and verify the
     arm actually follows (not just holds), then back to home
  5. gripper: close to 20 mm, open to max -- does measured width track command
  6. return to home, final tracking error
  7. gradual release: 15 s "support the arm" countdown, then gains fade over 4 s

    python3 arm2_full_check.py --bus can0     # arm2 is currently on can0
    python3 arm2_full_check.py                # default bus can1
"""
import argparse
import sys
import time

import numpy as np

from arm_driver import (ARM_JOINTS, ArmDriver, ArmDriverError, GRIPPER_MAX_MM, HOME_ANGLES,
                        JOINT_ORDER)

J6 = JOINT_ORDER.index("joint_6")


def terr(driver, ref):
    e = np.abs(driver.positions() - ref)
    e[J6] = 0.0
    return e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", default="can1")
    ap.add_argument("--kp", type=float, default=50.0)
    ap.add_argument("--hold", type=float, default=20.0)
    args = ap.parse_args()

    print(f"=== arm2 full check on {args.bus}  (kp={args.kp}) ===\n")
    P = {"pass": 0, "warn": 0, "fail": 0}

    def check(name, ok, detail="", warn=False):
        tag = "PASS" if ok else ("WARN" if warn else "FAIL")
        P["pass" if ok else ("warn" if warn else "fail")] += 1
        print(f"  [{tag}] {name}{('  -- ' + detail) if detail else ''}")

    try:
        driver = ArmDriver(channel=args.bus, live=True, kp=args.kp, kd=2.0,
                           gripper_kp=10.0, max_joint_speed=0.30)
        driver.start()
    except ArmDriverError as e:
        print(f"[FAIL] driver did not start:\n{e}")
        return 1

    try:
        # ---- 1. probe / feedback ----------------------------------------
        st = driver.status()
        pos = driver.positions()
        temps = {n: (driver.motors[n].get_states() or {}).get("t_rotor", 0)
                 for n, _, _ in ARM_JOINTS}
        check("all 7 motors + gripper report", len(st) == 7 and all(v == "ENABLED" for v in st.values()),
              f"status={st}")
        check("rotor temps < 55 C", all(t < 55 for t in temps.values()), f"{temps}")
        check("start pose finite / in range", np.all(np.isfinite(pos)) and np.abs(pos).max() < 4.0,
              f"{dict(zip(JOINT_ORDER, np.round(pos,3)))}")

        # ---- 2. ramp to home ------------------------------------------
        driver.go_home(duration=8.0)
        e = terr(driver, HOME_ANGLES)
        check("reached home (max tracking err < 0.15 rad)", e.max() < 0.15,
              f"max {e.max():.3f} rad on {JOINT_ORDER[int(e.argmax())]}",
              warn=(0.15 <= e.max() < 0.30))

        # ---- 3. steady hold -----------------------------------------
        print(f"\n  holding {args.hold:.0f}s ...")
        errs, se0 = [], driver._send_errors
        t0 = time.time()
        while time.time() - t0 < args.hold:
            time.sleep(2.0)
            e = terr(driver, HOME_ANGLES)
            errs.append(e.max())
            f = driver.check_faults()
            print(f"    t={time.time()-t0:4.1f}s  max_err={e.max():.3f}  "
                  f"send_errs={driver._send_errors}  faults={f}")
            if f:
                break
        drift = max(errs) - min(errs)
        check("hold stable (no growing drift)", drift < 0.05, f"err span {drift:.3f} rad", warn=drift < 0.1)
        check("no CAN send errors during hold", driver._send_errors == se0,
              f"{driver._send_errors - se0} in {args.hold:.0f}s")
        check("no motor faults during hold", not driver.check_faults(), f"{driver.check_faults()}")

        # ---- 4. closed-loop move test -------------------------------
        print("\n  closed-loop move test ...")
        move_ok = True
        for jname, delta in [("joint_1", 0.20), ("joint_1", -0.20),
                             ("joint_4", 0.20), ("joint_4", -0.20)]:
            tgt = HOME_ANGLES.copy()
            tgt[JOINT_ORDER.index(jname)] += delta
            driver.goto(tgt, duration=3.0)
            e = terr(driver, tgt)
            good = e.max() < 0.18
            move_ok &= good
            print(f"    {jname} {delta:+.2f} -> settled err {e.max():.3f} rad "
                  f"({'ok' if good else 'OFF'})")
        driver.goto(HOME_ANGLES, duration=3.0)
        check("arm follows commanded moves (<0.18 rad)", move_ok)

        # ---- 5. gripper --------------------------------------------
        print("\n  gripper test ...")
        driver.set_gripper(20.0)
        time.sleep(2.0)
        g_close = driver.gripper_mm()
        driver.set_gripper(GRIPPER_MAX_MM)
        time.sleep(2.0)
        g_open = driver.gripper_mm()
        print(f"    cmd 20 -> meas {g_close:.1f} mm ;  cmd {GRIPPER_MAX_MM:.0f} -> meas {g_open:.1f} mm")
        check("gripper tracks command (close within 15 mm)", abs(g_close - 20.0) < 15.0,
              f"{g_close:.1f} mm", warn=abs(g_close - 20.0) < 30.0)
        check("gripper opens (> 85 mm)", g_open > 85.0, f"{g_open:.1f} mm")

        # ---- 6. return home ---------------------------------------
        driver.go_home(duration=6.0)
        e = terr(driver, HOME_ANGLES)
        check("returned to home", e.max() < 0.15, f"max {e.max():.3f} rad",
              warn=(0.15 <= e.max() < 0.30))

        print(f"\n=== RESULT: {P['pass']} pass, {P['warn']} warn, {P['fail']} fail ===")
        print("\n  Holding at home. RELEASING IN 15 s -- put a hand on the arm.")
        for i in range(15, 0, -1):
            print(f"    releasing in {i:2d}s ...", end="\r", flush=True)
            time.sleep(1.0)
        print()
    except KeyboardInterrupt:
        print("\n  interrupted -- freezing, then releasing.")
        driver.hold_here()
    finally:
        driver.release(ramp_sec=4.0)
    return 0 if P["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
