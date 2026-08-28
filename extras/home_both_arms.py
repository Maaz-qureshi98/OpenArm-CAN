#!/usr/bin/env python3
"""
A/B hardware test: probe BOTH CAN buses (can0 and can1), report which motors
answer on each, and bring every fully-present arm to its home pose.

Use it to find out which physical arm is healthy:

    python3 home_both_arms.py            # probe + home every complete arm
    python3 home_both_arms.py --probe    # probe only, never move a motor
    python3 home_both_arms.py --bus can0 # just one bus

Then swap the CAN cables and run it again -- if the "bad arm" symptom moves
with the cable it is a wiring fault; if it stays with the arm it is that arm.

Safety:
  * Each bus is handled one at a time. Nothing on the other bus is powered
    while one arm is moving.
  * An arm that is missing ANY motor is reported and LEFT ALONE -- a partial
    arm is never commanded (that would just crank the live joints while the
    rest hangs limp).
  * A complete arm is ramped to home slowly by the persistent 50 Hz command
    thread (no free-fall), then held until you press Enter, then the gains
    fade out over 4 s. Support the arm before pressing Enter.
"""
import argparse
import sys
import time

import numpy as np

from arm_driver import (ARM_JOINTS, ArmDriver, ArmDriverError, GRIPPER_ID,
                        GRIPPER_NAME, GRIPPER_TYPE, HOME_ANGLES, JOINT_ORDER,
                        make_controller)

BUSES = ["can0", "can1"]
EXPECT = [(n, mid) for n, mid, _ in ARM_JOINTS] + [(GRIPPER_NAME, GRIPPER_ID)]


def probe_bus(bus):
    """Zero-torque probe. Returns dict name->state or None. Never applies torque."""
    try:
        c = make_controller(bus)  # CAN-FD socket: sees both classic and FD feedback
    except Exception as e:  # noqa: BLE001
        print(f"  [{bus}] cannot open bus: {e}")
        return None
    motors = {}
    for name, mid, mtype in ARM_JOINTS:
        motors[name] = c.add_motor(motor_id=mid, feedback_id=mid + 0x10, motor_type=mtype)
    motors[GRIPPER_NAME] = c.add_motor(
        motor_id=GRIPPER_ID, feedback_id=GRIPPER_ID + 0x10, motor_type=GRIPPER_TYPE)
    for _ in range(50):
        for m in motors.values():
            try:
                m.send_cmd_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.02)
    result = {name: m.get_states() for name, m in motors.items()}
    try:
        c.bus.shutdown()
    except Exception:  # noqa: BLE001
        pass
    return result


def report(bus, states):
    if states is None:
        print(f"[{bus}] BUS NOT AVAILABLE (is it up?  sudo ip link set {bus} up type can "
              f"bitrate 1000000 dbitrate 5000000 fd on)")
        return "no-bus", []
    present = [n for n, st in states.items() if st]
    missing = [n for n, st in states.items() if not st]
    print(f"[{bus}] {len(present)}/{len(states)} motors answered")
    for name, mid in EXPECT:
        st = states.get(name)
        if st:
            print(f"    {name:9s} id{mid:<2d} pos={st['pos']:+.3f} rad  "
                  f"{st.get('status','?'):9s} {st.get('t_rotor',0):.0f}C")
        else:
            print(f"    {name:9s} id{mid:<2d} -- NO RESPONSE --")
    if not present:
        return "dead", missing
    if missing:
        return "partial", missing
    return "complete", []


def home_bus(bus, kp, duration, hold_only=False):
    what = "hold current pose" if hold_only else f"ramp to home over {duration:.0f}s"
    print(f"\n[{bus}] complete arm -> {what} ...")
    driver = ArmDriver(channel=bus, live=True, kp=kp, kd=2.0,
                       gripper_kp=8.0, max_joint_speed=0.30)
    try:
        driver.start()
    except ArmDriverError as e:
        print(f"[{bus}] could not start driver: {e}")
        return
    try:
        start = driver.positions()
        print(f"[{bus}] start pose (rad): {dict(zip(JOINT_ORDER, np.round(start, 3)))}")
        if hold_only:
            ref = start.copy()
            driver.hold_here()
            print(f"[{bus}] holding here 15 s -- push each joint, it should resist.")
            for _ in range(7):
                time.sleep(2.0)
                e = np.abs(driver.positions() - ref); e[JOINT_ORDER.index("joint_6")] = 0.0
                print(f"    max drift {e.max():.3f} rad  send_errs={driver._send_errors}  "
                      f"faults={driver.check_faults()}")
            driver.close(prompt=True)
            return
        driver.go_home(duration=duration)
        err = np.abs(driver.positions() - HOME_ANGLES)
        err[JOINT_ORDER.index("joint_6")] = 0.0
        print(f"[{bus}] at home. max joint tracking error {err.max():.3f} rad "
              f"(send_errors={driver._send_errors}, faults={driver.check_faults()})")
        print(f"[{bus}] holding. Push it gently -- it should resist and return.")
    except KeyboardInterrupt:
        print(f"\n[{bus}] interrupted -- freezing where it is.")
        driver.hold_here()
    finally:
        driver.close(prompt=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", choices=BUSES, help="Only handle this one bus.")
    ap.add_argument("--probe", action="store_true", help="Probe only, never move.")
    ap.add_argument("--hold-only", action="store_true",
                    help="Energize and hold the CURRENT pose (no go-home). Use for the "
                         "left arm, which has no defined home pose yet.")
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--home-time", type=float, default=8.0)
    args = ap.parse_args()

    buses = [args.bus] if args.bus else BUSES
    summary = {}

    for bus in buses:
        print("=" * 70)
        states = probe_bus(bus)
        status, missing = report(bus, states)
        summary[bus] = (status, missing)
        if args.probe:
            continue
        if status == "complete":
            home_bus(bus, args.kp, args.home_time, hold_only=args.hold_only)
        elif status == "partial":
            print(f"[{bus}] NOT moving -- missing {missing}. Fix wiring/power first.")
        else:
            print(f"[{bus}] nothing to do.")

    print("=" * 70)
    print("SUMMARY")
    for bus in buses:
        status, missing = summary[bus]
        tail = f"  (missing: {', '.join(missing)})" if missing else ""
        print(f"  {bus}: {status.upper()}{tail}")
    print("\nNow swap the CAN cables and run this again. If 'PARTIAL/DEAD' moves\n"
          "to the other bus, the fault is the cable/adapter. If it stays on the\n"
          "same arm, that arm has the power/wiring fault.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
