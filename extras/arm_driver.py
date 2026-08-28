#!/usr/bin/env python3
"""
Persistent CAN driver for OpenArm arm2 (can1).

WHY THIS FILE EXISTS -- the free-fall bug
-----------------------------------------
The older scripts (home_arm2.py, eval_arm2.py) commanded each motor by
spawning `damiao send-cmd-mit` as a subprocess, sleeping ~0.4 s, then killing
it. Damiao motors in MIT mode only hold position while MIT frames keep
arriving. The instant those subprocesses are killed the motor stops getting
commands, its CAN-timeout alarm fires, and it goes limp -- that is exactly the
"arm falls freely after the action" problem. It also made every command take
~0.4 s and every state read ~1-15 s (`damiao scan` subprocess).

This driver replaces all of that with:
  * ONE python process, ONE CAN bus handle, opened once.
  * ONE background thread sending an MIT command to EVERY motor at 100 Hz,
    continuously, for as long as the driver is alive. The arm is therefore
    never uncommanded -> it cannot free-fall while this driver runs.
  * Joint positions read from the motors' own CAN feedback frames (already
    decoded by damiao_motor's polling thread) -- instant, no subprocess.
  * Built-in speed limiting: you give it a goal, and the command thread walks
    the commanded position toward that goal at max_joint_speed rad/s. That is
    what makes the motion smooth and slow instead of a step input.

Release is explicit and gradual: release() ramps kp/kd down to zero over a few
seconds so the arm sags instead of dropping. Nothing here ever disables a
motor without you asking for it.

joint_6 (CAN id 0x06) is dead on this arm and is never commanded (same as
during hand-eye calibration). It is treated as fixed at 0.0 rad everywhere.

Requires: damiao_motor (already in .venv_lerobot), python-can, socketcan up.
"""

import json
import os
import subprocess
import sys
import threading
import time

import can
import numpy as np

from damiao_motor import DaMiaoController


class _FDController(DaMiaoController):
    """DaMiaoController that opens its CAN socket directly in CAN-FD mode.

    WHY FD: the stock DaMiaoController opens a classic-CAN socket, and the
    kernel does NOT deliver CAN-FD frames to a classic socket. arm2's motors
    send classic-CAN feedback (so it worked), but the LEFT arm's motors 2-8
    send CAN-FD feedback -- invisible on a classic socket. That was the "only
    joint_1 answers" symptom. An FD socket receives BOTH classic and FD frames;
    MIT commands still go out as classic 8-byte frames. Safe for either arm,
    no motor reconfiguration.

    WHY A SUBCLASS (not "open then swap the .bus"): opening a classic socket
    and immediately closing it churns the interface and was making the rapid
    open/close probe tools flaky. This never opens the classic socket at all.
    """

    def __init__(self, channel, bustype="socketcan"):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, fd=True)
        self.motors = {}
        self._motors_by_feedback = {}
        self._polling_thread = None
        self._polling_active = False
        self._polling_lock = threading.Lock()


def make_controller(channel):
    """A CAN-FD DaMiaoController for `channel`. See _FDController."""
    return _FDController(channel)

# --------------------------------------------------------------------------
# Hardware map. Motor types picked to match each motor's actual VMAX/TMAX
# registers (confirmed earlier via `damiao scan` register dump):
#   8009 = [VMAX 45, TMAX 54], 4340 = [VMAX 10, TMAX 28], 4310 = [VMAX 30, TMAX 10]
# --------------------------------------------------------------------------
CAN_IFACE = "can1"

# (name, can id, motor type). joint_6 deliberately absent -- dead motor.
ARM_JOINTS = [
    ("joint_1", 1, "8009"),
    ("joint_2", 2, "8009"),
    ("joint_3", 3, "4340"),
    ("joint_4", 4, "4340"),
    ("joint_5", 5, "4310"),
    ("joint_7", 7, "4310"),
]
GRIPPER_NAME = "gripper"
GRIPPER_ID = 8
GRIPPER_TYPE = "4310"

# Full 7-joint order used by the URDF FK/IK chain. Index 5 (joint_6) is dead
# and always 0.0.
JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
DEAD_JOINT_INDEX = JOINT_ORDER.index("joint_6")
LIVE_JOINT_INDICES = [JOINT_ORDER.index(n) for n, _, _ in ARM_JOINTS]

# Gripper calibration (unchanged from the old scripts).
GRIPPER_OPEN_RAD = -0.7854
GRIPPER_WIDTH_AT_OPEN_MM = 103.0
GRIPPER_MIN_MM = 0.0
GRIPPER_MAX_MM = GRIPPER_WIDTH_AT_OPEN_MM

# Home pose ("homearm2" pose from home_arm2.py) -- shoulder-to-elbow straight,
# elbow bent ~90 deg.
HOME_ANGLES_DICT = {
    "joint_1": -1.216,
    "joint_2": -0.175,
    "joint_3": 0.553,
    "joint_4": 1.836,
    "joint_5": -0.127,
    "joint_7": 1.021,
}
HOME_ANGLES = np.array([HOME_ANGLES_DICT.get(j, 0.0) for j in JOINT_ORDER], dtype=np.float64)

# find_home.py writes this after you have jogged the arm so the wrist camera
# sees what the demonstrations saw. When it exists it overrides the pose above,
# because a home pose that matches the demos' starting viewpoint matters far
# more to the policy than any gain or clamp setting.
HOME_POSE_FILE = "home_pose.json"


def load_home_angles(path=HOME_POSE_FILE):
    """(angles7, came_from_file). Falls back to HOME_ANGLES if there is no file."""
    if not os.path.exists(path):
        return HOME_ANGLES.copy(), False
    try:
        with open(path) as f:
            data = json.load(f)
        angles = np.array([float(data["angles"][j]) for j in JOINT_ORDER],
                          dtype=np.float64)
        angles[DEAD_JOINT_INDEX] = 0.0
        return angles, True
    except (OSError, KeyError, ValueError, TypeError) as e:
        print(f"[warn] {path} unreadable ({e}); using the built-in home pose.")
        return HOME_ANGLES.copy(), False

# Gains. These are the values the old scripts proved safe on this hardware,
# but they work much better here because commands now arrive at 100 Hz instead
# of in 0.4 s bursts, so tracking error stays small.
DEFAULT_KP = 30.0
DEFAULT_KD = 2.0
DEFAULT_GRIPPER_KP = 8.0
DEFAULT_GRIPPER_KD = 0.4

# Per-joint gain multipliers, applied on top of kp/kd. joint_1 (shoulder) and
# joint_4 (elbow) carry the most gravity load and sag the most at a given kp,
# so if raising --kp globally makes the wrist joints buzz, raise only these
# two here instead. 1.0 = use the global gain unchanged.
# joint_2 (shoulder pitch) and joint_4 (elbow) carry almost all the gravity
# load at the Tune6 home pose (arm reaching forward over the table). At a flat
# kp=50 they sagged ~0.05 / ~0.13 rad below the commanded angle, which shoved
# the demo block into the top-left corner of the wrist frame. Raising just
# these two pulls the settled pose back onto the commanded one; the remaining
# few-mrad are fed forward in home_pose.json (set via jog_server `gohome` +
# nudge + save). Retune with jog_server `kpscale <joint> <mult>` if the arm
# buzzes or the home view drifts.
KP_SCALE = {
    "joint_1": 1.0,
    "joint_2": 1.8,
    "joint_3": 1.0,
    "joint_4": 2.2,
    "joint_5": 1.0,
    "joint_7": 1.0,
}

# 50 Hz, lowered from 100. Seven joints plus the gripper at 100 Hz, on top of
# the library's own feedback polling, was filling the socketcan TX queue --
# "No buffer space available [Error Code 105]". 50 Hz holds just as firmly and
# leaves the bus headroom. If you still see 105, raise the queue:
#     sudo ip link set can1 txqueuelen 1000
DEFAULT_RATE_HZ = 50.0
DEFAULT_JOINT_SPEED = 0.35        # rad/s -- slow and smooth
DEFAULT_GRIPPER_SPEED = 60.0      # mm/s


def mm_to_gripper_rad(width_mm):
    width_mm = float(np.clip(width_mm, GRIPPER_MIN_MM, GRIPPER_MAX_MM))
    return width_mm / GRIPPER_WIDTH_AT_OPEN_MM * GRIPPER_OPEN_RAD


def gripper_rad_to_mm(rad):
    return float(np.clip(rad / GRIPPER_OPEN_RAD * GRIPPER_WIDTH_AT_OPEN_MM,
                         GRIPPER_MIN_MM, GRIPPER_MAX_MM))


def _bring_up_can(iface, log=print):
    """Best-effort `sudo -n` bring-up of a CAN-FD interface (also clears BUS-OFF
    via the implicit down->up). Silent no-op if sudo needs a password."""
    import shutil
    attempts = []
    if shutil.which("openarm-can-up") or os.path.exists("/usr/local/sbin/openarm-can-up"):
        attempts.append(["sudo", "-n", "/usr/local/sbin/openarm-can-up", iface])
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "can_up.sh")
    if os.path.exists(here):
        attempts.append(["sudo", "-n", "bash", here, iface])
    attempts.append(["sudo", "-n", "ip", "link", "set", iface, "down"])
    attempts.append(["sudo", "-n", "ip", "link", "set", iface, "up", "type", "can",
                     "bitrate", "1000000", "dbitrate", "5000000", "fd", "on",
                     "restart-ms", "100"])
    attempts.append(["sudo", "-n", "ip", "link", "set", iface, "txqueuelen", "1000"])
    for cmd in attempts:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if r.returncode == 0 and cmd[3] != "down":
                log(f"  brought {iface} up ({cmd[3] if cmd[3] != 'ip' else 'ip link'})")
        except (OSError, subprocess.SubprocessError):
            pass
        if can_interface_is_up(iface):
            time.sleep(0.4)
            return True
    return False


def can_interface_is_up(iface=CAN_IFACE):
    """True if the socketcan interface exists and is UP."""
    try:
        out = subprocess.run(["ip", "-details", "link", "show", iface],
                             capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return False
    if out.returncode != 0:
        return False
    return "state UP" in out.stdout or "UP," in out.stdout.split("\n")[0]


class ArmDriverError(RuntimeError):
    pass


class ArmDriver:
    """Owns the CAN bus and a 100 Hz command thread. Use as a context manager.

    Typical use:

        with ArmDriver(live=True) as arm:
            arm.go_home(duration=8.0)
            arm.goto(target_angles_7, gripper_mm=80.0)
            ...
        # __exit__ holds position and asks you before releasing
    """

    def __init__(self, channel=CAN_IFACE, live=True, rate_hz=DEFAULT_RATE_HZ,
                 kp=DEFAULT_KP, kd=DEFAULT_KD,
                 gripper_kp=DEFAULT_GRIPPER_KP, gripper_kd=DEFAULT_GRIPPER_KD,
                 max_joint_speed=DEFAULT_JOINT_SPEED,
                 max_gripper_speed=DEFAULT_GRIPPER_SPEED,
                 verbose=True):
        self.channel = channel
        self.live = live
        self.rate_hz = float(rate_hz)
        self.kp = float(kp)
        self.kd = float(kd)
        self.gripper_kp = float(gripper_kp)
        self.gripper_kd = float(gripper_kd)
        self.max_joint_speed = float(max_joint_speed)
        self.max_gripper_speed = float(max_gripper_speed)
        self.verbose = verbose

        self.controller = None
        self.motors = {}                 # name -> DaMiaoMotor

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        # commanded (interpolated) and goal joint positions, 7-vector
        self._cmd = np.zeros(7, dtype=np.float64)
        self._goal = np.zeros(7, dtype=np.float64)
        self._cmd_gripper_mm = GRIPPER_MAX_MM
        self._goal_gripper_mm = GRIPPER_MAX_MM
        self._gain_scale = 1.0
        self._speed = self.max_joint_speed

        self._tick_count = 0
        self._send_errors = 0
        self._last_error = None
        self._started = False

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self):
        if self._started:
            return
        if not self.live:
            # Dry run still runs the command thread, just without sending on
            # CAN. That way the interpolation, speed limiting and blocking
            # behaviour of goto() are exactly what you will get live, so dry-run
            # timing actually predicts live timing instead of finishing instantly.
            self._log("DRY RUN: no CAN bus opened, no motor will move.")
            self._cmd = HOME_ANGLES.copy()
            self._goal = HOME_ANGLES.copy()
            self._stop.clear()
            self._thread = threading.Thread(target=self._command_loop, daemon=True,
                                            name="arm-command-loop-dry")
            self._thread.start()
            self._started = True
            return

        if not can_interface_is_up(self.channel):
            # Auto-heal: interface down, misconfigured, or BUS-OFF (which shows
            # as state DOWN). Try the shared bring-up helper, then a plain
            # `ip link` fallback -- both via `sudo -n` so it is silent when the
            # sudoers rule from setup_can.sh is installed, and simply fails
            # through to the helpful error otherwise.
            self._log(f"{self.channel} is not up -- attempting automatic bring-up ...")
            _bring_up_can(self.channel, log=self._log)

        if not can_interface_is_up(self.channel):
            raise ArmDriverError(
                f"CAN interface {self.channel} is not up, and automatic bring-up "
                f"failed.\n"
                f"  One-time fix so this never happens again (installs a boot + "
                f"hotplug service, no more sudo prompts):\n"
                f"    cd ~/Maaz/ApplRobotics/ML && sudo ./setup_can.sh\n"
                f"  Or just now:\n"
                f"    sudo ./can_up.sh {self.channel}\n"
            )

        self._log(f"Opening {self.channel} ...")
        self.controller = make_controller(self.channel)  # CAN-FD socket -- see make_controller()

        for name, mid, mtype in ARM_JOINTS:
            self.motors[name] = self.controller.add_motor(
                motor_id=mid, feedback_id=mid + 0x10, motor_type=mtype)
        self.motors[GRIPPER_NAME] = self.controller.add_motor(
            motor_id=GRIPPER_ID, feedback_id=GRIPPER_ID + 0x10, motor_type=GRIPPER_TYPE)

        # Probe with ZERO-torque MIT frames (kp=kd=torque=0). This makes every
        # motor answer with a feedback frame -- same thing `damiao scan` does --
        # without applying any torque, so the arm does not twitch.
        #
        # Retry a few times: the CAN-FD motors on the left arm can take a beat
        # to all start replying after the socket opens, and a single 0.6 s poll
        # sometimes catches only the first one or two. Poll in rounds and only
        # fail if a motor is STILL silent after several seconds.
        self._log("Probing motors (zero-torque) for position feedback ...")
        missing = list(self.motors)
        for attempt in range(6):
            for _ in range(30):
                for m in self.motors.values():
                    try:
                        m.send_cmd_mit(0.0, 0.0, 0.0, 0.0, 0.0)
                    except Exception as e:  # noqa: BLE001 - report and keep probing
                        self._last_error = e
                time.sleep(0.02)
            missing = [n for n, m in self.motors.items() if not m.get_states()]
            if not missing:
                break
            self._log(f"  probe attempt {attempt + 1}: still waiting on {missing}")

        if missing:
            self.controller.bus.shutdown()
            raise ArmDriverError(
                "No CAN feedback from: " + ", ".join(missing) +
                f"\n  Check power, CAN wiring and that {self.channel} is the right bus."
            )

        for name, m in self.motors.items():
            st = m.get_states()
            self._log(f"  {name:9s} pos={st['pos']:+.3f} rad  status={st['status']}"
                      f"  t_rotor={st['t_rotor']:.0f}C")

        # The width mapping assumes the jaws are fully open at GRIPPER_OPEN_RAD
        # and closed at 0. A reading well outside that span means the motor's
        # mechanical zero has drifted from the one the calibration was taken at,
        # so every width the policy asks for lands somewhere else on the jaws --
        # and observation.state's gripper channel saturates at the clamp and
        # stops carrying information.
        graw = float(self.motors[GRIPPER_NAME].get_states()["pos"])
        lo, hi = min(GRIPPER_OPEN_RAD, 0.0) - 0.10, max(GRIPPER_OPEN_RAD, 0.0) + 0.10
        if not (lo <= graw <= hi):
            self._log(
                f"  [warn] gripper reads {graw:+.3f} rad, outside the calibrated span "
                f"[{GRIPPER_OPEN_RAD:+.3f}, 0.000].\n"
                f"         Width readings are clipping at {GRIPPER_WIDTH_AT_OPEN_MM:.0f} mm, "
                f"so the policy sees a constant instead of a real width, and commanded\n"
                f"         widths will not land where you expect. Re-zero the gripper or "
                f"redo its width calibration.")

        # Make sure the motors are in MIT control mode and enabled.
        for name, m in self.motors.items():
            try:
                m.ensure_control_mode("MIT")
            except Exception as e:  # noqa: BLE001
                self._log(f"  [warn] {name}: could not verify MIT control mode ({e}); "
                          f"continuing, MIT frames still apply.")
        self.controller.enable_all()
        time.sleep(0.1)

        # Start commanding from exactly where the arm is right now, so the
        # first command produces zero step and therefore zero jerk.
        measured = self.positions()
        self._cmd = measured.copy()
        self._goal = measured.copy()
        gm = self.gripper_mm()
        self._cmd_gripper_mm = gm
        self._goal_gripper_mm = gm

        self._stop.clear()
        self._thread = threading.Thread(target=self._command_loop, daemon=True,
                                        name="arm-command-loop")
        self._thread.start()
        self._started = True
        self._log(f"Command loop running at {self.rate_hz:.0f} Hz "
                  f"(kp={self.kp}, kd={self.kd}). Arm is held, it will not free-fall.")

    def _command_loop(self):
        dt = 1.0 / self.rate_hz
        next_t = time.perf_counter()
        while not self._stop.is_set():
            with self._lock:
                goal = self._goal.copy()
                goal_g = self._goal_gripper_mm
                kp = self.kp * self._gain_scale
                kd = self.kd * self._gain_scale
                gkp = self.gripper_kp * self._gain_scale
                gkd = self.gripper_kd * self._gain_scale
                speed = self._speed
                cmd = self._cmd.copy()
                cmd_g = self._cmd_gripper_mm

            # rate-limited interpolation -> smooth, slow motion
            max_step = speed * dt
            delta = np.clip(goal - cmd, -max_step, max_step)
            cmd = cmd + delta
            cmd[DEAD_JOINT_INDEX] = 0.0

            g_step = self.max_gripper_speed * dt
            cmd_g = cmd_g + float(np.clip(goal_g - cmd_g, -g_step, g_step))

            if self.live:
                for name, mid, _ in ARM_JOINTS:
                    idx = JOINT_ORDER.index(name)
                    scale = KP_SCALE.get(name, 1.0)
                    try:
                        self.motors[name].send_cmd_mit(
                            float(cmd[idx]), 0.0, kp * scale, kd * scale, 0.0)
                    except Exception as e:  # noqa: BLE001
                        self._send_errors += 1
                        self._last_error = e
                        if "105" in str(e):
                            time.sleep(0.002)   # let the TX queue drain
                try:
                    self.motors[GRIPPER_NAME].send_cmd_mit(
                        mm_to_gripper_rad(cmd_g), 0.0, gkp, gkd, 0.0)
                except Exception as e:  # noqa: BLE001
                    self._send_errors += 1
                    self._last_error = e

            with self._lock:
                self._cmd = cmd
                self._cmd_gripper_mm = cmd_g
                self._tick_count += 1

            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()  # we fell behind; resync

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------
    def positions(self):
        """Measured joint angles as a 7-vector in JOINT_ORDER (joint_6 -> 0.0)."""
        if not self.live:
            with self._lock:
                return self._cmd.copy()
        out = np.zeros(7, dtype=np.float64)
        for name, _, _ in ARM_JOINTS:
            st = self.motors[name].get_states()
            if not st:
                raise ArmDriverError(f"no feedback from {name}")
            out[JOINT_ORDER.index(name)] = float(st["pos"])
        out[DEAD_JOINT_INDEX] = 0.0
        return out

    def commanded(self):
        """The joint positions currently being COMMANDED (7-vector).

        Different from positions(): under gravity the arm settles a little
        behind its command, so measured = commanded - sag. Which of the two you
        integrate matters a lot -- see run_pickplace.py's --fk-from.
        """
        with self._lock:
            return self._cmd.copy()

    def commanded_gripper_mm(self):
        with self._lock:
            return self._cmd_gripper_mm

    def gripper_mm(self):
        if not self.live:
            with self._lock:
                return self._cmd_gripper_mm
        st = self.motors[GRIPPER_NAME].get_states()
        if not st:
            return self._cmd_gripper_mm
        return gripper_rad_to_mm(float(st["pos"]))

    def status(self):
        """dict name -> status string, for fault checking."""
        if not self.live:
            return {}
        return {n: (m.get_states() or {}).get("status", "?") for n, m in self.motors.items()}

    def check_faults(self):
        """Returns a list of (name, status) for any motor not ENABLED."""
        bad = []
        for name, st in self.status().items():
            if st not in ("ENABLED",):
                bad.append((name, st))
        return bad

    # ------------------------------------------------------------------
    # motion
    # ------------------------------------------------------------------
    def set_goal(self, angles7, gripper_mm=None, speed=None):
        """Non-blocking: update the target the command thread walks toward."""
        angles7 = np.asarray(angles7, dtype=np.float64).reshape(7).copy()
        angles7[DEAD_JOINT_INDEX] = 0.0
        with self._lock:
            self._goal = angles7
            if gripper_mm is not None:
                self._goal_gripper_mm = float(np.clip(gripper_mm, GRIPPER_MIN_MM, GRIPPER_MAX_MM))
            self._speed = float(speed) if speed is not None else self.max_joint_speed

    def goto(self, angles7, gripper_mm=None, speed=None, duration=None,
             wait=True, settle=0.3, extra_timeout=4.0):
        """Move to a joint target smoothly.

        speed    -- rad/s cap for this move (default: driver's max_joint_speed)
        duration -- if given, speed is chosen so the move takes about this long
        wait     -- block until the commanded trajectory has arrived + settle
        """
        angles7 = np.asarray(angles7, dtype=np.float64).reshape(7).copy()
        angles7[DEAD_JOINT_INDEX] = 0.0

        with self._lock:
            cmd_now = self._cmd.copy()
        travel = float(np.max(np.abs(angles7 - cmd_now)))

        if duration is not None and duration > 0:
            speed = max(travel / duration, 1e-4)
        if speed is None:
            speed = self.max_joint_speed

        self.set_goal(angles7, gripper_mm=gripper_mm, speed=speed)

        if not wait:
            return

        timeout = travel / max(speed, 1e-6) + extra_timeout
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                err = float(np.max(np.abs(self._goal - self._cmd)))
                gerr = abs(self._goal_gripper_mm - self._cmd_gripper_mm)
            if err < 1e-3 and gerr < 0.5:
                break
            time.sleep(0.01)
        time.sleep(settle)

    def go_home(self, duration=8.0, gripper_mm=GRIPPER_MAX_MM, angles=None):
        """Ramp to the home pose over `duration` seconds, then hold.

        `angles` lets the caller pass a home pose already clamped into the URDF
        joint limits, so the arm and the IK solver agree on where home is.
        """
        home = HOME_ANGLES if angles is None else np.asarray(angles, dtype=np.float64)
        self._log(f"Going to home pose over {duration:.0f}s ...")
        self.goto(home, gripper_mm=gripper_mm, duration=duration)
        if self.live:
            err = np.abs(self.positions() - home)
            err[DEAD_JOINT_INDEX] = 0.0
            self._log("At home. Per-joint tracking error (rad): " +
                      np.array2string(np.round(err, 3)))
            if err.max() > 0.2:
                self._log(f"  [WARN] worst joint is {err.max():.2f} rad ({np.rad2deg(err.max()):.0f} deg) "
                          f"off home -- the arm did NOT get back.\n"
                          f"         Something blocked it, or CAN commands were being dropped "
                          f"(send_errors={self._send_errors}).\n"
                          f"         Do not start another run until it is physically back at home.")
        else:
            self._log("At home (dry run).")

    def set_gripper(self, width_mm, wait=True):
        with self._lock:
            goal = self._goal.copy()
        self.goto(goal, gripper_mm=width_mm, wait=wait)

    def hold_here(self):
        """Freeze: make the goal equal to the currently commanded position.
        Safe to call from a signal handler / except block."""
        with self._lock:
            self._goal = self._cmd.copy()
            self._goal_gripper_mm = self._cmd_gripper_mm

    # ------------------------------------------------------------------
    # shutdown
    # ------------------------------------------------------------------
    def release(self, ramp_sec=4.0):
        """Gradually drop kp/kd to zero over ramp_sec, THEN disable the motors.

        The arm will sag as the gains fade -- support it. This is deliberately
        gradual so it settles instead of dropping.
        """
        if not self.live or not self._started:
            self._stop.set()
            return
        self._log(f"Releasing gains over {ramp_sec:.0f}s -- SUPPORT THE ARM NOW.")
        self.hold_here()
        steps = max(1, int(ramp_sec * 20))
        for i in range(steps + 1):
            with self._lock:
                self._gain_scale = 1.0 - (i / steps)
            time.sleep(ramp_sec / steps)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for attempt in range(5):
            try:
                self.controller.disable_all()
                break
            except Exception as e:  # noqa: BLE001
                self._log(f"  [warn] disable attempt {attempt+1}/5 failed ({e}), retrying")
                time.sleep(0.2)
        try:
            self.controller.bus.shutdown()
        except Exception as e:  # noqa: BLE001
            self._log(f"  [warn] bus shutdown: {e}")
        self._started = False
        self._log("Motors disabled, CAN bus closed.")

    def close(self, prompt=True, ramp_sec=4.0):
        """Keep holding until the operator says it is safe to let go."""
        if not self._started:
            return
        self.hold_here()
        if prompt and self.live and sys.stdin is not None and sys.stdin.isatty():
            self._log("")
            self._log("Arm is HELD at its current pose and will stay held.")
            self._log("Take hold of the arm (or make sure it is resting), then press Enter")
            self._log("to fade the gains out. Nothing moves until you do.")
            try:
                input("  >>> press Enter to release: ")
            except (EOFError, KeyboardInterrupt):
                self._log("\n  (no input) -- releasing anyway, gently.")
        self.release(ramp_sec=ramp_sec)

    # ------------------------------------------------------------------
    def _log(self, msg):
        if self.verbose:
            print(msg, flush=True)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        # Never let an exception path leave the arm uncommanded.
        try:
            self.hold_here()
        finally:
            self.close(prompt=True)
        return False
