#!/usr/bin/env python3
"""
OpenArm — single-file bring-up + move, for a fresh laptop.

Copy JUST THIS FILE to the other machine. No repo, no ML folder needed.

    sudo python3 openarm_standalone.py setup     # once: deps + CAN config (persistent)
    python3 openarm_standalone.py probe          # list motors on both arms
    python3 openarm_standalone.py hold           # energise + hold both arms (Enter releases)
    python3 openarm_standalone.py hold --bus can1
    python3 openarm_standalone.py home            # ramp both arms to HOME_ANGLES

Needs: Linux, a PEAK PCAN-USB FD adapter per arm, Python 3.8+, internet for the
one-time `setup` (it pip-installs `python-can` and `damiao-motor`).

WHY 'setup' needs sudo: a USB-CAN adapter comes up DOWN and unconfigured. It
must be told bitrate / CAN-FD / queue length after every boot. 'setup' installs
a tiny systemd service so that happens automatically from then on -- for every
user, no more prompts. (The LEFT arm's motors reply in CAN-FD; without `fd on`
they are invisible. `restart-ms 100` lets a BUS-OFF self-recover.)
"""
import os
import subprocess
import sys
import time

IFACES = ["can0", "can1"]
BITRATE, DBITRATE = 1_000_000, 5_000_000

# Full 7-DOF arm: CAN ids 1..7 + gripper on 8. Whichever joints actually answer
# are the ones driven -- so a complete arm reads 8/8, and an arm with a missing
# joint (e.g. one unit here has no joint_6) still works on the rest.
JOINTS = [("joint_1", 1, "8009"), ("joint_2", 2, "8009"), ("joint_3", 3, "4340"),
          ("joint_4", 4, "4340"), ("joint_5", 5, "4310"), ("joint_6", 6, "4310"),
          ("joint_7", 7, "4310")]
GRIPPER = ("gripper", 8, "4310")
HOME = {"joint_1": -1.216, "joint_2": -0.175, "joint_3": 0.553, "joint_4": 1.836,
        "joint_5": -0.127, "joint_6": 0.0, "joint_7": 1.021}
CAN_UP_SH = "/usr/local/sbin/openarm-can-up"


# ---------------------------------------------------------------- setup
def _sh(cmd, **kw):
    return subprocess.run(cmd, text=True, **kw)


def do_setup():
    if os.geteuid() != 0:
        sys.exit("run:  sudo python3 openarm_standalone.py setup")

    print("[1/3] python packages ...")
    _sh([sys.executable, "-m", "pip", "install", "-q", "python-can", "damiao-motor", "numpy"])

    print("[2/3] installing /usr/local/sbin/openarm-can-up ...")
    body = f"""#!/usr/bin/env bash
set -u
IFS=$'\\n'
for IF in ${{@:-{ ' '.join(IFACES) }}}; do
  [ -e /sys/class/net/$IF ] || {{ echo "[$IF] not present"; continue; }}
  ip link set $IF down 2>/dev/null || true
  ip link set $IF up type can bitrate {BITRATE} dbitrate {DBITRATE} fd on restart-ms 100 \\
    && ip link set $IF txqueuelen 1000 && echo "[$IF] up (CAN-FD)"
done
"""
    with open(CAN_UP_SH, "w") as f:
        f.write(body)
    os.chmod(CAN_UP_SH, 0o755)

    print("[3/3] systemd service + hotplug + sudoers ...")
    with open("/etc/systemd/system/openarm-can.service", "w") as f:
        f.write("[Unit]\nDescription=OpenArm CAN-FD bring-up\n"
                "After=systemd-udev-settle.service\nWants=systemd-udev-settle.service\n"
                "[Service]\nType=oneshot\nRemainAfterExit=yes\n"
                f"ExecStartPre=/bin/sleep 2\nExecStart={CAN_UP_SH}\n"
                "Restart=on-failure\nRestartSec=3\n[Install]\nWantedBy=multi-user.target\n")
    with open("/etc/systemd/system/openarm-can-hotplug.service", "w") as f:
        f.write("[Unit]\nDescription=OpenArm CAN re-bring-up on hotplug\n"
                f"[Service]\nType=oneshot\nExecStartPre=/bin/sleep 1\nExecStart={CAN_UP_SH}\n")
    with open("/etc/udev/rules.d/90-openarm-can.rules", "w") as f:
        f.write('ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", ENV{ID_VENDOR_ID}=="0c72", '
                'TAG+="systemd", ENV{SYSTEMD_WANTS}+="openarm-can-hotplug.service"\n')
    with open("/etc/sudoers.d/openarm-can", "w") as f:
        f.write(f"ALL ALL=(root) NOPASSWD: {CAN_UP_SH}\n")
    os.chmod("/etc/sudoers.d/openarm-can", 0o440)

    _sh(["systemctl", "daemon-reload"])
    _sh(["udevadm", "control", "--reload-rules"])
    _sh(["systemctl", "enable", "--now", "openarm-can.service"])
    time.sleep(1)
    _sh([CAN_UP_SH])
    print("\ndone. CAN will now come up automatically on every boot / replug.")
    print("verify:   python3 openarm_standalone.py probe")


def ensure_can_up():
    for ifc in IFACES:
        st = _sh(["ip", "-details", "link", "show", ifc], capture_output=True)
        if st.returncode == 0 and "state UP" in st.stdout:
            continue
        if os.path.exists(CAN_UP_SH):
            _sh(["sudo", "-n", CAN_UP_SH, ifc], capture_output=True)
        else:
            _sh(["sudo", "-n", "ip", "link", "set", ifc, "down"], capture_output=True)
            _sh(["sudo", "-n", "ip", "link", "set", ifc, "up", "type", "can",
                 "bitrate", str(BITRATE), "dbitrate", str(DBITRATE), "fd", "on",
                 "restart-ms", "100"], capture_output=True)
            _sh(["sudo", "-n", "ip", "link", "set", ifc, "txqueuelen", "1000"], capture_output=True)


# ---------------------------------------------------------------- driver
def _load_can():
    try:
        import can
        from damiao_motor import DaMiaoController
    except ImportError:
        sys.exit("missing packages -- run:  sudo python3 openarm_standalone.py setup")
    return can, DaMiaoController


class Arm:
    """One arm on one CAN-FD bus: probe, 50 Hz MIT hold, ramped goto, gentle release."""

    def __init__(self, bus):
        import threading
        can, DaMiaoController = _load_can()
        self.bus_name = bus

        class _FD(DaMiaoController):          # FD socket: sees classic AND FD feedback
            def __init__(s, ch):
                s.bus = can.interface.Bus(channel=ch, interface="socketcan", fd=True)
                s.motors, s._motors_by_feedback = {}, {}
                s._polling_thread, s._polling_active = None, False
                s._polling_lock = threading.Lock()

        self.c = _FD(bus)
        self.m = {}
        for n, i, t in JOINTS:
            self.m[n] = self.c.add_motor(motor_id=i, feedback_id=i + 0x10, motor_type=t)
        n, i, t = GRIPPER
        self.m[n] = self.c.add_motor(motor_id=i, feedback_id=i + 0x10, motor_type=t)
        self._names = [n for n, _, _ in JOINTS]
        self._stop = threading.Event()
        self._cmd = None
        self._goal = None
        self._kp = 0.0
        self._th = None

    def probe(self, tries=6):
        live = []
        for _ in range(tries):
            for _ in range(30):
                for mot in self.m.values():
                    try:
                        mot.send_cmd_mit(0., 0., 0., 0., 0.)
                    except Exception:
                        pass
                time.sleep(0.02)
            live = [n for n in self.m if self.m[n].get_states()]
            if len(live) == len(self.m):
                break
        # only drive joints that actually answered (an arm may be missing one)
        self._names = [n for n, _, _ in JOINTS if n in live]
        return live

    def _pos(self, n):
        st = self.m[n].get_states()
        return st["pos"] if st else 0.0

    def start(self, kp=35.0):
        import threading
        for mot in self.m.values():
            try:
                mot.enable()
            except Exception:
                pass
        time.sleep(0.1)
        self._cmd = {n: self._pos(n) for n in self._names}
        self._goal = dict(self._cmd)
        self._kp = kp
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        dt = 0.02
        while not self._stop.is_set():
            for n in self._names:
                g, c = self._goal[n], self._cmd[n]
                step = max(-0.006, min(0.006, g - c))          # ~0.3 rad/s
                c += step
                self._cmd[n] = c
                _, i, _ = next(j for j in JOINTS if j[0] == n)
                try:
                    self.m[n].send_cmd_mit(c, 0.0, self._kp, 2.0, 0.0)
                except Exception:
                    pass
            try:
                self.m["gripper"].send_cmd_mit(self._grip_rad, 0.0, 8.0, 0.4, 0.0)
            except Exception:
                pass
            time.sleep(dt)

    _grip_rad = -0.7854

    def goto(self, targets, secs=8.0):
        if not self._goal:
            return
        start = dict(self._cmd)
        steps = int(secs / 0.05)
        for k in range(steps + 1):
            a = k / steps
            for n, v in targets.items():
                if n in self._goal:
                    self._goal[n] = start[n] + a * (v - start[n])
            time.sleep(0.05)
        time.sleep(0.5)

    def release(self):
        for s in range(20, -1, -1):
            self._kp = 35.0 * s / 20.0
            time.sleep(0.2)
        self._stop.set()
        time.sleep(0.1)
        try:
            self.c.disable_all()
        except Exception:
            pass
        try:
            self.c.bus.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- commands
def do_probe():
    ensure_can_up()
    for bus in IFACES:
        try:
            a = Arm(bus)
        except SystemExit:
            raise
        except Exception as e:
            print(f"{bus}: cannot open ({e})")
            continue
        live = a.probe()
        bar = "".join("#" if n in live else "." for n in list(a.m))
        print(f"{bus}: {len(live)}/{len(a.m)}  [{bar}]  {live}")
        try:
            a.c.bus.shutdown()
        except Exception:
            pass


def do_hold(only=None):
    ensure_can_up()
    buses = [only] if only else IFACES
    arms = []
    for bus in buses:
        a = Arm(bus)
        live = a.probe()
        print(f"{bus}: {len(live)}/{len(a.m)}  {live}")
        if "gripper" in live and len(a._names) >= 6:
            a.start()
            arms.append(a)
        else:
            print(f"{bus}: too few motors ({len(live)}/8) -- not energising this bus")
            try:
                a.c.bus.shutdown()
            except Exception:
                pass
    if not arms:
        return
    try:
        input("\nboth arms HELD. push them -- they resist. Press Enter to release: ")
    except (EOFError, KeyboardInterrupt):
        pass
    for a in arms:
        print(f"releasing {a.bus_name} ... SUPPORT THE ARM")
        a.release()


def do_home():
    ensure_can_up()
    arms = []
    for bus in IFACES:
        a = Arm(bus)
        live = a.probe()
        if not ("gripper" in live and len(a._names) >= 6):
            print(f"{bus}: only {len(live)}/8 -- skipping")
            try:
                a.c.bus.shutdown()
            except Exception:
                pass
            continue
        a.start()
        arms.append(a)
    if not arms:
        return
    import threading
    ts = [threading.Thread(target=a.goto, args=(HOME, 8.0)) for a in arms]
    print("ramping to HOME_ANGLES ...")
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    try:
        input("\nat home, HELD. Press Enter to release: ")
    except (EOFError, KeyboardInterrupt):
        pass
    for a in arms:
        print(f"releasing {a.bus_name} ... SUPPORT THE ARM")
        a.release()


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    bus = None
    if "--bus" in sys.argv:
        bus = sys.argv[sys.argv.index("--bus") + 1]
    if cmd == "setup":
        do_setup()
    elif cmd == "probe":
        do_probe()
    elif cmd == "hold":
        do_hold(bus)
    elif cmd == "home":
        do_home()
    else:
        print(__doc__)
