# OpenArm — dual-arm bring-up & control (Linux)

Get a bimanual **OpenArm** (Damiao motors, one PEAK PCAN-USB FD adapter per arm)
visible and moving from any Linux laptop.

## Quick start

Everything is in one file: **`openarm_standalone.py`**.

```bash
# 1. once per laptop — installs deps + configures CAN to auto-come-up forever
sudo python3 openarm_standalone.py setup

# 2. check both arms are visible
python3 openarm_standalone.py probe          # expect:  can0 7/7 [#######]   can1 7/7 [#######]

# 3. move them
python3 openarm_standalone.py hold           # energise + hold both arms; push them, they resist; Enter releases
python3 openarm_standalone.py home           # ramp both arms to HOME_ANGLES
python3 openarm_standalone.py hold --bus can1 # one arm only
```

That's it. No venv, no other files needed.

## Why `setup` needs `sudo` (one time)

A USB-CAN adapter comes up **DOWN and unconfigured** — it must be told the
bitrate / CAN-FD mode / queue length after every boot. `setup` installs a
systemd service + udev rule so that happens automatically from then on, for
every user, with no more password prompts. It applies:

| setting | why |
|---|---|
| `fd on`, dbitrate 5 Mbit | the motors reply with **CAN-FD** frames — on a plain classic socket the arm is invisible |
| `txqueuelen 1000` | the default (10) overflows with 8 motors at 50 Hz → "No buffer space [105]" |
| `restart-ms 100` | a BUS-OFF self-recovers instead of wedging the bus |

If you can't run `setup` (no sudo), bring CAN up each session with
`sudo ./can_up.sh` — same settings, not persistent.

## Buses

`can0` and `can1`, one per arm, assigned in plug order. `joint_6` (CAN id 6) is
dead on this hardware and is never commanded, so a healthy arm shows **7/7**
(joints 1-5, 7 + gripper).

## `extras/`

Fuller per-task scripts (need `arm_driver.py` from the same folder):
`home_both_arms.py`, `arm2_full_check.py`, `check_all_joints.py`, `both_arms.py`,
`which_arm.py`, `probe_watch.py`. Run `python3 extras/<script> --bus canX`.

## Requirements

Linux + socketcan, Python 3.8+, `pip install -r requirements.txt`
(`python-can`, `damiao-motor`, `numpy`). MIT licensed.

## Safety

Every moving command ends holding, then a countdown, then a gradual gain fade —
**keep a hand on the arm** for the release. `HOME_ANGLES` is the right arm's
home; the left arm has no defined home pose yet.
