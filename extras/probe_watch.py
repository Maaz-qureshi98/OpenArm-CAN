#!/usr/bin/env python3
"""Probe one CAN bus ~1x/sec forever, print which motors answer. Read-only
(zero-torque). Use it while gently flexing the arm harness to find a flaky
connector -- watch the list of motors blink in and out.

    python3 probe_watch.py --bus can1
"""
import argparse, time
from arm_driver import make_controller, ARM_JOINTS, GRIPPER_ID, GRIPPER_TYPE, GRIPPER_NAME

ap = argparse.ArgumentParser()
ap.add_argument("--bus", required=True)
a = ap.parse_args()
names = [n for n, _, _ in ARM_JOINTS] + [GRIPPER_NAME]
c = make_controller(a.bus)          # ONE controller, held -- no churn
ms = {}
for n, mid, t in ARM_JOINTS:
    ms[n] = c.add_motor(motor_id=mid, feedback_id=mid + 0x10, motor_type=t)
ms[GRIPPER_NAME] = c.add_motor(motor_id=GRIPPER_ID, feedback_id=GRIPPER_ID + 0x10, motor_type=GRIPPER_TYPE)
try:
    while True:
        for _ in range(25):
            for m in ms.values():
                try: m.send_cmd_mit(0., 0., 0., 0., 0.)
                except Exception: pass
            time.sleep(0.02)
        live = [n for n in names if ms[n].get_states()]
        # clear cached state so each line reflects only THIS poll window (catches dropouts)
        for m in ms.values():
            m.state = None
        bar = "".join("#" if n in live else "." for n in names)
        print(f"{time.strftime('%H:%M:%S')}  [{bar}]  {len(live)}/7  {live}", flush=True)
        time.sleep(0.6)
except KeyboardInterrupt:
    try: c.bus.shutdown()
    except Exception: pass
