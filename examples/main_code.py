"""
main_code.py — SYSEN 5920 Team 3
MAIN COMPETITION SUPERVISOR — the one program that runs on the rover
during the competition.

STATES AND LIGHTS
  MENU     blinking PINK    rover parked, waiting for a mode pick
  MANUAL   solid GREEN      joysticks drive; D-pad runs assist moves
  ASSIST   solid CYAN       a D-pad auto move is executing (hands off)
  (disconnected: blinking ORANGE — motors stopped, drops back to MENU)

CONTROLS
  START (Button 9)  -> from ANYWHERE: kill all motion, back to MENU.
                       Checked inside every loop, including mid-assist,
                       so it acts as the all-stop.
  In MENU:
    A (Button 0)    -> MANUAL traversal mode
    B (Button 1)    -> LINE FOLLOWER (imported from line_follow_auto.py;
                       place the robot straddling the line FIRST — the
                       B press is the start)
    Challenges live in their own files: any module with a run(sv)
    function can be wired to a MENU button via self.challenge("name").
    Upload the challenge .py files to the robot alongside this one.
  In MANUAL:
    Left stick Y    -> throttle        Right stick X -> steering
    D-pad UP        -> assist: drive forward 100 cm, watching the
                       rangefinder and stopping short of any obstacle
    D-pad LEFT      -> assist: rotate 90 deg left
    D-pad RIGHT     -> assist: rotate 90 deg right
    D-pad DOWN      -> assist: rotate 180 deg
  Button indices are for a standard gamepad through PestoLink
  (START=9, A=0, D-pad U/D/L/R = 12/13/14/15) — VERIFY in the
  PestoLink gamepad tester and adjust the constants if different.

CRASH SAFETY
  Every mode runs inside a catch-everything wrapper. Any Python
  exception: motors stop, the full traceback goes to main_log.txt,
  and the rover returns to the blinking-pink MENU instead of dying.
  The supervisor loop itself never exits.

LOGGING
  Same black-box scheme as sumo_auto v3.6+: main_log.txt on flash,
  boot self-test (RED LED at boot = log file broken / flash full),
  auto-rotation at 300 KB, battery thresholds scaled for the 4xAA pack.

To auto-run at power-up, save this file on the robot as main.py.
Positive turn() / positive yaw = LEFT (CCW). Negative = RIGHT (CW).
"""

from XRPLib.defaults import *
from pestolink import PestoLinkAgent
from machine import Pin, ADC
import time
import math
import sys

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # BLE name shown in PestoLink (8 chars max)

# Controller mapping (verify in the PestoLink gamepad tester)
BTN_A = 0
BTN_B = 1
BTN_START = 9               # all-stop / menu
DPAD_UP = 12
DPAD_DOWN = 13
DPAD_LEFT = 14
DPAD_RIGHT = 15
THROTTLE_AXIS = 1           # left stick vertical
STEER_AXIS = 2              # right stick horizontal
INVERT_THROTTLE = True      # stick-forward reads negative in PestoLink

# Manual driving feel
MANUAL_MAX_EFFORT = 0.8
MANUAL_STEER_SCALE = 0.7
MANUAL_EXPO = True          # squared stick curve: fine control near center

# D-pad assist moves
ASSIST_FWD_CM = 100.0       # D-pad UP drive distance
ASSIST_FWD_EFFORT = 0.45
OBSTACLE_STOP_CM = 15.0     # stop early if something is closer than this
ASSIST_TURN_EFFORT = 0.55
TURN_CHUNK_DEG = 30         # turns run in chunks so START stays responsive
TURN_TOL_DEG = 4.0
FWD_KP = 0.02               # IMU heading-hold gain on the assist drive
FWD_CORR_MAX = 0.15
SLOW_CHECK_S = 0.12         # rangefinder poll period during assist drive
                            # (line-fast loops matter less here, but the
                            # ultrasonic still must not starve the loop)

# Lights (r, g, b)
PINK = (255, 40, 120)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
ORANGE = (255, 120, 0)
RED = (255, 0, 0)
BLINK_S = 0.35              # menu blink half-period

# Battery (4xAA pack)
BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS

# Logging
LOG_TO_FILE = True
LOG_PATH = "main_log.txt"

# ------------------------------- LOGGING ---------------------------------

_BOOT_MS = time.ticks_ms()
_LOG_BROKEN = {"reported": False}

def battery_voltage():
    try:
        return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)
    except Exception:
        return -1.0

def log(msg):
    t = time.ticks_diff(time.ticks_ms(), _BOOT_MS) / 1000
    line = "[%8.2fs] %s" % (t, msg)
    print(line)
    if LOG_TO_FILE:
        try:
            f = open(LOG_PATH, "a")
            f.write(line + "\n")
            f.close()
        except Exception as e:
            if not _LOG_BROKEN["reported"]:
                _LOG_BROKEN["reported"] = True
                print("*** LOG WRITE FAILED: %r — flash full/corrupt? ***"
                      % e)

def log_exception(e):
    log("!!! CRASH — traceback follows")
    try:
        sys.print_exception(e)
        if LOG_TO_FILE:
            f = open(LOG_PATH, "a")
            f.write("TRACEBACK:\n")
            sys.print_exception(e, f)
            f.close()
    except Exception:
        pass

def log_selftest():
    import os
    try:
        try:
            if os.stat(LOG_PATH)[6] > 300 * 1024:
                os.remove(LOG_PATH)
                print("log: rotated oversized %s" % LOG_PATH)
        except OSError:
            pass
        f = open(LOG_PATH, "a")
        f.write("")
        f.close()
        try:
            st = os.statvfs("/")
            log("log self-test OK, %d KB flash free"
                % (st[0] * st[3] // 1024))
        except Exception:
            log("log self-test OK")
    except Exception as e:
        print("*** LOG SELF-TEST FAILED: %r — delete %s in the IDE "
              "file browser ***" % (e, LOG_PATH))
        try:
            board.set_rgb_led(*RED)
        except Exception:
            pass
        time.sleep(3)

# ------------------------------ MAIN CLASS -------------------------------

class MenuAbort(Exception):
    """Raised anywhere when START is pressed: unwinds to the MENU."""
    pass

class MainCode:
    """Competition supervisor: state machine with an always-available
    all-stop (START), crash containment, and pluggable challenge slots."""

    def __init__(self):
        self.pesto = PestoLinkAgent(ROBOT_NAME)
        self._edge = {}              # button rising-edge memory
        self._blink_ms = 0
        self._blink_on = False

    # ---------------- light + controller helpers ----------------

    def set_light(self, rgb):
        try:
            board.set_rgb_led(*rgb)
        except Exception:
            pass                     # XRP Beta has no RGB LED

    def blink(self, rgb):
        """Non-blocking blink; call from inside a polling loop."""
        now = time.ticks_ms()
        if time.ticks_diff(now, self._blink_ms) >= BLINK_S * 1000:
            self._blink_ms = now
            self._blink_on = not self._blink_on
            self.set_light(rgb if self._blink_on else (0, 0, 0))

    def pressed(self, btn):
        """Rising-edge detect: True once per physical press."""
        now = self.pesto.get_button(btn) if self.pesto.is_connected() \
            else False
        was = self._edge.get(btn, False)
        self._edge[btn] = now
        return now and not was

    def check_abort(self):
        """Call inside EVERY motion loop. START = all-stop -> MENU."""
        if self.pesto.is_connected() and self.pesto.get_button(BTN_START):
            drivetrain.stop()
            raise MenuAbort()

    def wait_release(self, btn):
        while self.pesto.is_connected() and self.pesto.get_button(btn):
            time.sleep(0.02)

    # ---------------- assist moves (D-pad) ----------------

    def normalize(self, a):
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a

    def assist_forward(self):
        """Drive ASSIST_FWD_CM straight (IMU heading hold), stopping
        early if the rangefinder sees an obstacle. START aborts."""
        self.set_light(CYAN)
        log("assist: forward %.0fcm" % ASSIST_FWD_CM)
        hold_yaw = imu.get_yaw()
        start_l = drivetrain.get_left_encoder_position()
        start_r = drivetrain.get_right_encoder_position()
        last_range = 0
        try:
            while True:
                self.check_abort()
                traveled = ((drivetrain.get_left_encoder_position()
                             - start_l) +
                            (drivetrain.get_right_encoder_position()
                             - start_r)) / 2
                if traveled >= ASSIST_FWD_CM:
                    log("assist: forward complete (%.1fcm)" % traveled)
                    break
                now = time.ticks_ms()
                if time.ticks_diff(now, last_range) >= SLOW_CHECK_S * 1000:
                    last_range = now
                    d = self.front_distance()
                    if d < OBSTACLE_STOP_CM:
                        log("assist: obstacle at %.1fcm after %.1fcm — "
                            "stopping" % (d, traveled))
                        break
                err = self.normalize(hold_yaw - imu.get_yaw())
                corr = max(-FWD_CORR_MAX,
                           min(FWD_CORR_MAX, FWD_KP * err))
                drivetrain.set_effort(ASSIST_FWD_EFFORT - corr,
                                      ASSIST_FWD_EFFORT + corr)
                time.sleep(0.01)
        finally:
            drivetrain.stop()

    def assist_turn(self, degrees):
        """Relative turn in TURN_CHUNK_DEG chunks so START stays live,
        then verify against the IMU and finish any shortfall."""
        self.set_light(CYAN)
        log("assist: turn %+d" % degrees)
        target = imu.get_yaw() + degrees
        try:
            remaining = self.normalize(target - imu.get_yaw())
            while abs(remaining) > TURN_TOL_DEG:
                self.check_abort()
                chunk = max(-TURN_CHUNK_DEG,
                            min(TURN_CHUNK_DEG, remaining))
                drivetrain.turn(chunk, ASSIST_TURN_EFFORT, timeout=2)
                new_remaining = self.normalize(target - imu.get_yaw())
                if abs(new_remaining - remaining) < 2.0:
                    # chunk produced almost no rotation — one loud retry,
                    # then give up rather than grind forever
                    drivetrain.turn(chunk, 0.9, timeout=2)
                    if abs(self.normalize(target - imu.get_yaw())
                           - new_remaining) < 2.0:
                        log("assist: turn stalled %.0f deg short"
                            % new_remaining)
                        break
                remaining = self.normalize(target - imu.get_yaw())
            log("assist: turn done, yaw=%+.1f" % imu.get_yaw())
        finally:
            drivetrain.stop()

    def front_distance(self):
        reads = []
        for _ in range(3):
            reads.append(rangefinder.distance())
            time.sleep(0.01)
        reads.sort()
        return reads[1]

    # ---------------- modes ----------------

    def menu_mode(self):
        """Blinking pink. Motors parked. Waits for a mode pick."""
        log("MENU (batt=%.2fV)" % battery_voltage())
        drivetrain.stop()
        self.wait_release(BTN_START)
        while True:
            if not self.pesto.is_connected():
                self.blink(ORANGE)
                time.sleep(0.02)
                continue
            self.blink(PINK)
            self.pesto.telemetryPrint("MENU", "FF2878")
            if self.pressed(BTN_A):
                self.wait_release(BTN_A)
                return self.manual_mode
            if self.pressed(BTN_B):
                self.wait_release(BTN_B)
                return self.challenge("line_follow_auto")
            # ---- more challenge slots: same pattern ----
            # if self.pressed(BTN_X):
            #     return self.challenge("sumo_auto")     # after refactor
            time.sleep(0.02)

    def manual_mode(self):
        """Joystick arcade drive + D-pad assist moves. START -> MENU."""
        log("MANUAL mode")
        self.set_light(GREEN)
        while True:
            if not self.pesto.is_connected():
                drivetrain.stop()
                log("controller lost in MANUAL — back to MENU")
                return self.menu_mode
            self.check_abort()

            if self.pressed(DPAD_UP):
                self.assist_forward()
                self.set_light(GREEN)
            elif self.pressed(DPAD_LEFT):
                self.assist_turn(90)
                self.set_light(GREEN)
            elif self.pressed(DPAD_RIGHT):
                self.assist_turn(-90)
                self.set_light(GREEN)
            elif self.pressed(DPAD_DOWN):
                self.assist_turn(180)
                self.set_light(GREEN)
            else:
                throttle = self.pesto.get_axis(THROTTLE_AXIS)
                if INVERT_THROTTLE:
                    throttle = -throttle
                steer = self.pesto.get_axis(STEER_AXIS)
                if MANUAL_EXPO:
                    throttle = throttle * abs(throttle)
                    steer = steer * abs(steer)
                steer *= MANUAL_STEER_SCALE
                left = (throttle + steer) * MANUAL_MAX_EFFORT
                right = (throttle - steer) * MANUAL_MAX_EFFORT
                m = max(1.0, abs(left), abs(right))
                drivetrain.set_effort(left / m, right / m)

            self.pesto.telemetryPrintBatteryVoltage(battery_voltage())
            time.sleep(0.01)

    # ---------------- challenge loader ----------------

    def challenge(self, module_name):
        """Load a challenge from its OWN FILE and run it. The module
        must be on the robot's flash alongside this file and expose
        run(sv) — see line_follow_auto.py for the pattern:
          * do NOT create a PestoLinkAgent (the supervisor owns BLE)
          * do NOT wait for a start button (the menu press IS the start)
          * call sv.check_abort() inside every loop so START stays the
            all-stop; the supervisor catches MenuAbort and crashes.
        The module is re-imported fresh on every launch, so its state
        resets and a re-uploaded file takes effect without a reboot."""
        def _mode():
            log("CHALLENGE: %s" % module_name)
            self.set_light(CYAN)
            if module_name in sys.modules:
                del sys.modules[module_name]     # force a fresh import
            mod = __import__(module_name)
            mod.run(self)
            log("CHALLENGE %s finished" % module_name)
            return self.menu_mode
        return _mode

    # ---------------- supervisor loop ----------------

    def run(self):
        """Never exits. Any crash: log it, stop, back to MENU."""
        board.led_on()               # solid board LED = program alive
        mode = self.menu_mode
        while True:
            try:
                mode = mode() or self.menu_mode
            except MenuAbort:
                log("START pressed — all stop, back to MENU")
                drivetrain.stop()
                mode = self.menu_mode
            except Exception as e:
                drivetrain.stop()
                log_exception(e)
                self.set_light(RED)
                time.sleep(1.0)      # brief red so the crash is visible
                mode = self.menu_mode

# --------------------------------- BOOT ----------------------------------

log_selftest()
log("")
log("======== BOOT: main_code v1.1 ======== batt=%.2fV" % battery_voltage())
if battery_voltage() < LOW_BATT_V:
    log("*** WARNING: battery LOW for a %d-cell pack (<%.1fV) ***"
        % (BATT_CELLS, LOW_BATT_V))

MainCode().run()
