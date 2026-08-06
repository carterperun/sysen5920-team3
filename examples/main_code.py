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
  ALL-STOP (v1.4: four redundant triggers — an emergency stop must
  never depend on one button index or a healthy radio):
    * START or SELECT (buttons 9/8) from anywhere
    * HOLD the onboard USER button ~1s (pure hardware — works with
      Bluetooth completely dead)
    * during challenges/assists: controller disconnecting, or the
      PestoLink app freezing (connected but silent) for ~1.5s
  All roads lead back to the blinking-pink MENU with motors stopped,
  and every all-stop logs WHICH trigger fired.
  BUTTON SPY: while in the MENU, every controller button press logs
  its index to main_log.txt — press START there once and read the log
  to confirm what index YOUR pad reports.
  In MENU:
    A (Button 0)    -> MANUAL traversal mode
    B (Button 1)    -> LINE FOLLOWER (from line_track.py — the simple
                       no-calibration tracker; place the robot
                       straddling the line FIRST — the B press is the
                       start, START ends it)
    X (Button 2)    -> SUMO (from sumo_auto.py; place the robot at the
                       RING CENTER first — the X press is the start)
    Y (Button 3)    -> MAZE (from maze_solver.py; place the robot in
                       the start cell FACING NORTH first — right-wall
                       follower to the goal cell)
    D-pad UP (12)   -> AUTO SMASH TOWER (from tower_smash_auto.py;
                       point the robot AT THE TOWER first — it ranges
                       the tower, charges through, then does three
                       90-degree clearing sweeps)
    Challenges live in their own files: any module with a run(sv)
    function can be wired to a MENU button via self.challenge("name").
    Upload challenge .py files alongside this one.
  In MANUAL:
    Left stick Y    -> throttle        Right stick X -> steering
    B (Button 1)    -> TURBO toggle: raises the speed cap (0.8 -> 1.0)
                       and the light flashes red fast until pressed
                       again. Resets to OFF on entering manual mode.
                       (B only launches the line follower from the
                       MENU, so there's no conflict.)
    X (Button 2)    -> REVERSE-DRIVE toggle: light flashes WHITE and
                       throttle inverts (stick-forward backs up).
                       v1.12: steering sides NOT inverted (per driver
                       feedback). Press X again for normal; resets on
                       mode entry. D-pad assists stay nose-relative.
    LEFT TRIGGER    -> hold: jog paddle FORWARD (v1.12). ~1.5s of
                       holding covers the full travel. Works while
                       driving, including the D-pad-UP auto drive.
    RIGHT TRIGGER   -> hold: jog paddle BACK, same rate.
                       SERVO SAFETY: angle clamped to 10..190 (never
                       commanded past mechanical range), and power is
                       cut 1s after release — settled angle is logged.
    D-pad UP        -> assist: drive forward 100 cm, watching the
                       rangefinder and stopping short of any obstacle.
                       TURBO also raises this speed (0.45 -> 0.6).
    D-pad LEFT      -> assist: rotate 90 deg left
    D-pad RIGHT     -> assist: rotate 90 deg right
    D-pad DOWN      -> assist: rotate 180 deg
  Button indices for this pad (CONFIRMED via button spy + servo test):
  START=9, A=0, B=1, X=2, Y=3, LT=6, RT=7, D-pad 12/13/14/15.

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
BTN_X = 2
BTN_Y = 3
BTN_LT = 6                  # left trigger — paddle STRIKE in MANUAL
BTN_RT = 7                  # right trigger — paddle to MIDDLE in MANUAL
                            # (triggers CONFIRMED as 6/7 by the 8/6
                            # servo-test session logs)
                            # TURBO toggle in MANUAL = B (BTN_B); B only
                            # launches the line follower from the MENU,
                            # so the two roles never collide
BTN_START = 9               # all-stop / menu
ABORT_BUTTONS = (8, 9)      # ANY of these = all-stop (8=Select/Back,
                            # 9=Start — some pads map Start to 8, so the
                            # emergency stop listens to both)

# All-stop redundancy (see check_abort): the emergency stop must never
# depend on one button index or a healthy radio.
USER_ABORT_HOLD_S = 0.8     # holding the onboard USER button this long =
                            # all-stop (works even if BLE is dead)
DISCONNECT_ABORT_S = 1.5    # controller gone this long mid-run = all-stop
STALE_ABORT_S = 1.5         # connected but NO packets arriving for this
                            # long during a challenge/assist = all-stop
                            # (catches a frozen/backgrounded PestoLink app
                            # that BLE still counts as "connected")
DPAD_UP = 12
DPAD_DOWN = 13
DPAD_LEFT = 14
DPAD_RIGHT = 15
THROTTLE_AXIS = 1           # left stick vertical
STEER_AXIS = 2              # right stick horizontal
INVERT_THROTTLE = True      # stick-forward reads negative in PestoLink

# Manual driving feel
MANUAL_MAX_EFFORT = 0.8     # normal speed cap
TURBO_MAX_EFFORT = 1.0      # cap while TURBO is toggled on (left trigger)
TURBO_STEER_SCALE = 0.55    # steering tamed a bit more at turbo speed
TURBO_BLINK_S = 0.1         # fast red flash period while turbo is on
MANUAL_STEER_SCALE = 0.7
MANUAL_EXPO = True          # squared stick curve: fine control near center

# D-pad assist moves
# v1.11: MIN_EFFORT stiction floor — the logged 11-second 90-degree
# turns were the XRPLib turn PID tapering to 0.1 effort near the
# target, below what this drivetrain can move at. Assist turns now use
# a custom PID that never outputs below 0.4, and the forward drive
# never commands a wheel below 0.4.
MIN_EFFORT = 0.4
# v1.14: WIGGLE TURNS — field friction stalls pure in-place turns, so
# turns now superimpose a small alternating fore/aft bias on the
# counter-rotation. Each wheel keeps breaking static friction instead
# of loading up against it; the biases cancel over a full cycle so the
# robot still turns in place (with a slight shimmy).
WIGGLE_BIAS = 0.18          # fore/aft bias added to BOTH wheels
WIGGLE_PERIOD_S = 0.22      # bias flips direction this often
LEFT_TURN_BOOST = 1.2       # v1.15: this robot turns LEFT weaker than
                            # right (observed on the field — one motor
                            # is weaker in that rotation direction), so
                            # CCW/left turns get 20% extra effort.
                            # Tune: raise if left still lags, set 1.0
                            # if the drivetrain is ever rebalanced.
ASSIST_FWD_CM = 100.0       # D-pad UP drive distance
ASSIST_FWD_EFFORT = 0.45
ASSIST_FWD_EFFORT_TURBO = 0.6   # auto-forward speed while TURBO is on
OBSTACLE_STOP_CM = 15.0     # stop early if something is closer than this
ASSIST_TURN_EFFORT = 0.75  # v1.16: raised from 0.55 — the logs showed
                            # first-pass turns timing out 40 deg short
                            # while EVERY 0.8-effort retry finished in
                            # under a second. This field needs ~0.75.
TURN_CHUNK_DEG = 30         # turns run in chunks so START stays responsive
TURN_TOL_DEG = 4.0
FWD_KP = 0.02               # IMU heading-hold gain on the assist drive
FWD_CORR_MAX = 0.15
SLOW_CHECK_S = 0.12         # rangefinder poll period during assist drive
                            # (line-fast loops matter less here, but the
                            # ultrasonic still must not starve the loop)

# Paddle servo (servo_one — works in MANUAL and during the D-pad-UP
# auto-forward). v1.12: cock-and-strike REPLACED with hold-to-jog for
# testing: hold LEFT trigger to sweep FORWARD, RIGHT trigger to sweep
# BACK, at a rate where ~1.5s of holding covers the full travel.
# SERVO SAFETY LOCKS (burn-out protection):
#   * angle hard-clamped to the 10..190 window — the servo can never be
#     commanded past its mechanical range, which is what makes it stall
#     at near-full current and cook itself
#   * once the paddle reaches an endpoint, further holding commands
#     nothing new (it just stays at the clamp, logged once)
#   * PADDLE_IDLE_FREE_S after both triggers are released, servo power
#     is CUT (servo_one.free()) — zero holding current while idle, and
#     the settled angle is logged for hard-coding later
PADDLE_BACK_DEG = 10        # rear travel limit (safety clamp)
PADDLE_FORWARD_DEG = 190    # forward travel limit (safety clamp)
PADDLE_MID_DEG = 90         # position on entering manual mode
PADDLE_TRAVEL_S = 1.5       # hold time to jog across the FULL travel
PADDLE_RATE_DPS = abs(PADDLE_FORWARD_DEG - PADDLE_BACK_DEG) \
    / PADDLE_TRAVEL_S       # = 120 deg/s jog rate
PADDLE_IDLE_FREE_S = 1.0    # power-cut delay after triggers released

# Lights (r, g, b)
PINK = (255, 40, 120)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
ORANGE = (255, 120, 0)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLINK_S = 0.35              # menu blink half-period
REVERSE_BLINK_S = 0.2       # white flash period while reverse-drive is on

# Battery (4xAA pack)
BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS

# Logging
LOG_TO_FILE = True
LOG_PATH = "LOG.TXT"           # unified log — every program appends here

# ------------------------------- LOGGING ---------------------------------

_BOOT_MS = time.ticks_ms()
_LOG_BROKEN = {"reported": False}

def battery_voltage():
    try:
        return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)
    except Exception:
        return -1.0

def log(msg):
    t = time.ticks_ms() / 1000.0    # seconds since POWER-ON —
    #     the same clock in every program, so LOG.TXT reads as
    #     one continuous session timeline
    line = "[%9.2fs][MAIN ] %s" % (t, msg)
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
        self.in_challenge = False    # staleness abort armed only here
        self._user_hold_ms = None    # USER-button hold tracking
        self.turbo = False
        self.reverse = False
        self._paddle_angle = float(PADDLE_MID_DEG)
        self._paddle_powered = False
        self._paddle_at_limit = False
        self._paddle_idle_ms = None
        self._paddle_lt_prev = False
        self._paddle_rt_prev = False
        self._paddle_last_ms = time.ticks_ms()
        self._last_conn_ms = time.ticks_ms()
        self._last_packet_ms = time.ticks_ms()
        # Timestamp every incoming controller packet by wrapping the
        # PestoLink receive hook — this is how check_abort can tell a
        # LIVE controller from a frozen app that BLE still calls
        # "connected" (buttons frozen at their last state, so a START
        # press would never arrive).
        _orig_on_write = self.pesto.on_write
        def _stamped_on_write(value):
            self._last_packet_ms = time.ticks_ms()
            _orig_on_write(value)
        self.pesto.on_write = _stamped_on_write

    # ---------------- light + controller helpers ----------------

    def set_light(self, rgb):
        try:
            board.set_rgb_led(*rgb)
        except Exception:
            pass                     # XRP Beta has no RGB LED

    def blink(self, rgb, period=BLINK_S):
        """Non-blocking blink; call from inside a polling loop."""
        now = time.ticks_ms()
        if time.ticks_diff(now, self._blink_ms) >= period * 1000:
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

    def _all_stop(self, why):
        drivetrain.stop()
        log("ALL-STOP: %s" % why)
        raise MenuAbort()

    def check_abort(self):
        """Call inside EVERY motion loop. Four INDEPENDENT all-stop
        triggers, so an emergency stop never hinges on one button index
        or a healthy radio:
          1. START or SELECT on the controller (buttons 8/9)
          2. onboard USER button held USER_ABORT_HOLD_S — pure hardware,
             works even if Bluetooth is completely dead
          3. controller disconnected mid-run for DISCONNECT_ABORT_S
          4. during challenges/assists: controller 'connected' but no
             packets for STALE_ABORT_S (frozen/backgrounded app — the
             state where a START press physically cannot reach us)"""
        now = time.ticks_ms()

        if self.pesto.is_connected():
            self._last_conn_ms = now
            for b in ABORT_BUTTONS:
                if self.pesto.get_button(b):
                    self._all_stop("controller button %d" % b)
            if self.in_challenge and \
                    time.ticks_diff(now, self._last_packet_ms) \
                    > STALE_ABORT_S * 1000:
                self._all_stop("controller stale — connected but no "
                               "packets for %.1fs" % STALE_ABORT_S)
        else:
            if self.in_challenge and \
                    time.ticks_diff(now, self._last_conn_ms) \
                    > DISCONNECT_ABORT_S * 1000:
                self._all_stop("controller disconnected mid-run")

        # hardware backup: USER button held (short taps are ignored so
        # challenge calibration taps don't trigger it)
        if board.is_button_pressed():
            if self._user_hold_ms is None:
                self._user_hold_ms = now
            elif time.ticks_diff(now, self._user_hold_ms) \
                    > USER_ABORT_HOLD_S * 1000:
                self._user_hold_ms = None
                self._all_stop("USER button held")
        else:
            self._user_hold_ms = None

    def wait_release(self, btn):
        while self.pesto.is_connected() and self.pesto.get_button(btn):
            time.sleep(0.02)

    def wait_release_aborts(self):
        """Wait until no abort trigger is held (all ABORT_BUTTONS up and
        USER button up) so a long press can't re-trigger instantly."""
        while True:
            held = board.is_button_pressed()
            if self.pesto.is_connected():
                for b in ABORT_BUTTONS:
                    held = held or self.pesto.get_button(b)
            if not held:
                return
            time.sleep(0.02)

    # ---------------- paddle servo (shared) ----------------

    def poll_paddle(self):
        """Hold-to-jog paddle (v1.12). Call from any driving loop
        (manual AND the auto-forward assist). Hold LEFT trigger: jog
        FORWARD. Hold RIGHT trigger: jog BACK. ~1.5s of holding covers
        the full travel. Safety locks: angle clamped to the
        BACK..FORWARD window (no past-range stall), nothing re-commanded
        while pinned at an endpoint, and servo power cut after
        PADDLE_IDLE_FREE_S idle so it never sits at holding current."""
        now = time.ticks_ms()
        lt = self.pesto.is_connected() and self.pesto.get_button(BTN_LT)
        rt = self.pesto.is_connected() and self.pesto.get_button(BTN_RT)

        # dt since last poll, capped so a slow loop pass can't jump far
        dt = time.ticks_diff(now, self._paddle_last_ms) / 1000
        self._paddle_last_ms = now
        if dt > 0.05:
            dt = 0.05

        if lt != self._paddle_lt_prev or rt != self._paddle_rt_prev:
            if lt and not self._paddle_lt_prev:
                log("paddle: jog FORWARD (from %.0f)" % self._paddle_angle)
            if rt and not self._paddle_rt_prev:
                log("paddle: jog BACK (from %.0f)" % self._paddle_angle)
            self._paddle_lt_prev, self._paddle_rt_prev = lt, rt

        if lt or rt:
            self._paddle_idle_ms = None
            direction = 0
            if lt and not rt:
                direction = 1            # toward PADDLE_FORWARD_DEG
            elif rt and not lt:
                direction = -1           # toward PADDLE_BACK_DEG
            # both held -> hold position (no movement)
            new = self._paddle_angle + direction * PADDLE_RATE_DPS * dt
            lo = min(PADDLE_BACK_DEG, PADDLE_FORWARD_DEG)
            hi = max(PADDLE_BACK_DEG, PADDLE_FORWARD_DEG)
            clamped = max(lo, min(hi, new))
            if clamped != self._paddle_angle:
                self._paddle_angle = clamped
                servo_one.set_angle(clamped)
                self._paddle_powered = True
                self._paddle_at_limit = False
            elif direction != 0 and not self._paddle_at_limit:
                # pinned at an endpoint while the trigger is still held:
                # command nothing further (safety) and say so once
                self._paddle_at_limit = True
                log("paddle: at travel limit (%.0f deg) — holding, "
                    "not pushing past it" % self._paddle_angle)
        else:
            self._paddle_at_limit = False
            if self._paddle_powered:
                if self._paddle_idle_ms is None:
                    self._paddle_idle_ms = now
                elif time.ticks_diff(now, self._paddle_idle_ms) \
                        >= PADDLE_IDLE_FREE_S * 1000:
                    servo_one.free()     # zero holding current
                    self._paddle_powered = False
                    self._paddle_idle_ms = None
                    log("paddle: settled at %.0f deg, power cut"
                        % self._paddle_angle)

    # ---------------- assist moves (D-pad) ----------------

    def wiggle_turn(self, degrees, effort, timeout_s=None):
        """In-place turn with the anti-friction WIGGLE (v1.14): the
        wheels counter-rotate at `effort` while BOTH get a small
        fore/aft bias that flips sign every WIGGLE_PERIOD_S. The bias
        keeps each wheel moving through its static-friction dead zone;
        it cancels over a full cycle, so the robot rotates in place
        with a slight shimmy instead of loading up and stalling.
        IMU-verified, abort checked every pass. Returns True if the
        target heading was reached."""
        target = imu.get_yaw() + degrees
        if timeout_s is None:
            timeout_s = 1.5 + abs(degrees) / 90.0 * 2.0
        t0 = time.ticks_ms()
        phase_ms = t0
        bias = WIGGLE_BIAS
        try:
            while True:
                self.check_abort()
                err = self.normalize(target - imu.get_yaw())
                if abs(err) <= TURN_TOL_DEG:
                    return True
                now = time.ticks_ms()
                if time.ticks_diff(now, t0) > timeout_s * 1000:
                    log("wiggle turn: timeout, %.0f deg short" % err)
                    return False
                if time.ticks_diff(now, phase_ms) >= WIGGLE_PERIOD_S * 1000:
                    phase_ms = now
                    bias = -bias
                # positive err = target is CCW/left: left wheel back,
                # right wheel forward; the shared bias rides on both.
                # Left turns get LEFT_TURN_BOOST (weak-side motor).
                mag = effort * (LEFT_TURN_BOOST if err > 0 else 1.0)
                mag = min(0.95, mag)
                eff = mag if err > 0 else -mag
                l = -eff + bias
                r = eff + bias
                # per-wheel stiction floor (v1.15): the bias half-cycle
                # used to dip the weak-side wheel below 0.4, where a
                # marginal motor stalls — clamp magnitude, keep sign
                if 0 < abs(l) < MIN_EFFORT:
                    l = MIN_EFFORT if l > 0 else -MIN_EFFORT
                if 0 < abs(r) < MIN_EFFORT:
                    r = MIN_EFFORT if r > 0 else -MIN_EFFORT
                drivetrain.set_effort(l, r)
                time.sleep(0.01)
        finally:
            drivetrain.stop()

    def normalize(self, a):
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a

    def assist_forward(self):
        """Drive ASSIST_FWD_CM straight (IMU heading hold), stopping
        early if the rangefinder sees an obstacle. START aborts.
        The paddle triggers stay LIVE during the drive (poll_paddle),
        and TURBO raises the drive effort (0.45 -> 0.6)."""
        self.set_light(CYAN)
        effort = ASSIST_FWD_EFFORT_TURBO if self.turbo \
            else ASSIST_FWD_EFFORT
        log("assist: forward %.0fcm at %.2f%s"
            % (ASSIST_FWD_CM, effort, " (turbo)" if self.turbo else ""))
        self.in_challenge = True         # arms disconnect/stale all-stop
        hold_yaw = imu.get_yaw()
        start_l = drivetrain.get_left_encoder_position()
        start_r = drivetrain.get_right_encoder_position()
        last_range = 0
        try:
            while True:
                self.check_abort()
                self.poll_paddle()       # strike stays available mid-drive
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
                # stiction floor on the slow wheel (v1.11)
                drivetrain.set_effort(max(MIN_EFFORT, effort - corr),
                                      max(MIN_EFFORT, effort + corr))
                time.sleep(0.01)
        finally:
            drivetrain.stop()
            self.in_challenge = False

    def assist_turn(self, degrees):
        """Relative turn using the anti-friction wiggle (v1.14). One
        boosted retry if the first pass times out short."""
        self.set_light(CYAN)
        log("assist: turn %+d (wiggle)" % degrees)
        self.in_challenge = True         # arms disconnect/stale all-stop
        target = imu.get_yaw() + degrees
        try:
            if not self.wiggle_turn(degrees, ASSIST_TURN_EFFORT):
                remaining = self.normalize(target - imu.get_yaw())
                log("assist: retrying %.0f deg at higher effort"
                    % remaining)
                if not self.wiggle_turn(remaining, 0.8):
                    log("assist: turn stalled %.0f deg short"
                        % self.normalize(target - imu.get_yaw()))
            log("assist: turn done, yaw=%+.1f" % imu.get_yaw())
        finally:
            drivetrain.stop()
            self.in_challenge = False

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
        self.in_challenge = False
        self._user_hold_ms = None
        self.wait_release_aborts()
        while True:
            if not self.pesto.is_connected():
                self.blink(ORANGE)
                time.sleep(0.02)
                continue
            self.blink(PINK)
            self.pesto.telemetryPrint("MENU", "FF2878")
            # One edge-scan per loop pass (pressed() consumes edges, so
            # scan once and dispatch from the result). Every press in
            # the menu is logged with its index — the BUTTON SPY: use it
            # to verify what YOUR pad calls START/triggers/D-pad.
            hits = [b for b in range(16) if self.pressed(b)]
            for b in hits:
                log("menu: controller button %d pressed" % b)
            if BTN_A in hits:
                self.wait_release(BTN_A)
                return self.manual_mode
            if BTN_B in hits:
                self.wait_release(BTN_B)
                return self.challenge("line_track")
            if BTN_X in hits:
                self.wait_release(BTN_X)
                return self.challenge("sumo_auto")
            if BTN_Y in hits:              # v1.18: MAZE solver
                self.wait_release(BTN_Y)
                return self.challenge("maze_solver")
            if DPAD_UP in hits:            # v1.17: AUTO SMASH TOWER
                self.wait_release(DPAD_UP)
                return self.challenge("tower_smash_auto")
            # ---- more challenge slots: same pattern ----
            time.sleep(0.02)

    def manual_mode(self):
        """Joystick arcade drive + D-pad assist moves + paddle servo.
        START -> MENU.
          LEFT BUMPER: TURBO toggle (cap -> TURBO_MAX_EFFORT, fast red
            flash until toggled off; always starts OFF on mode entry).
          LEFT TRIGGER: paddle STRIKE — servo backs all the way up,
            then sweeps at full speed to the furthest-forward position.
            Non-blocking: driving stays live during the cock-back.
          RIGHT TRIGGER: paddle to the MIDDLE (neutral carry).
        Assist moves run at their own fixed speeds regardless of turbo."""
        log("MANUAL mode (batt=%.2fV)" % battery_voltage())
        self.turbo = False
        self.reverse = False             # always start nose-forward
        self._paddle_angle = float(PADDLE_MID_DEG)
        self._paddle_powered = True
        self._paddle_idle_ms = None
        self._paddle_last_ms = time.ticks_ms()
        servo_one.set_angle(PADDLE_MID_DEG)   # known position on entry
        # (the idle power-cut will free it 1s later if untouched)
        self.set_light(GREEN)
        last_hb = time.ticks_ms()
        while True:
            if not self.pesto.is_connected():
                drivetrain.stop()
                self.turbo = False
                log("controller lost in MANUAL — back to MENU")
                return self.menu_mode
            self.check_abort()

            if self.pressed(BTN_B):
                self.turbo = not self.turbo
                log("TURBO %s" % ("ON" if self.turbo else "OFF"))
                if not self.turbo and not self.reverse:
                    self.set_light(GREEN)

            if self.pressed(BTN_X):
                self.reverse = not self.reverse
                log("REVERSE %s" % ("ON" if self.reverse else "OFF"))
                if not self.reverse and not self.turbo:
                    self.set_light(GREEN)

            self.poll_paddle()           # LT strike / RT center

            # heartbeat every 30s: proves logging is alive mid-session
            # and tracks battery sag between paddle strikes
            if time.ticks_diff(time.ticks_ms(), last_hb) >= 30000:
                last_hb = time.ticks_ms()
                log("manual hb: batt=%.2fV turbo=%s"
                    % (battery_voltage(), self.turbo))

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
                if self.reverse:
                    # drive-backwards mode: stick-forward backs up.
                    # v1.12: steer is NOT inverted anymore — driver
                    # feedback said the flipped turn sides felt wrong,
                    # so turning works the same as normal driving.
                    throttle = -throttle
                if self.turbo:
                    steer *= TURBO_STEER_SCALE
                    cap = TURBO_MAX_EFFORT
                else:
                    steer *= MANUAL_STEER_SCALE
                    cap = MANUAL_MAX_EFFORT
                left = (throttle + steer) * cap
                right = (throttle - steer) * cap
                m = max(1.0, abs(left), abs(right))
                drivetrain.set_effort(left / m, right / m)

            if self.reverse:
                self.blink(WHITE, REVERSE_BLINK_S)   # flashing white
                self.pesto.telemetryPrint(
                    "REV+TRB" if self.turbo else "REVERSE", "FFFFFF")
            elif self.turbo:
                self.blink(RED, TURBO_BLINK_S)   # fast red flash = turbo
                self.pesto.telemetryPrint("TURBO", "FF0000")
            else:
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

            # ---- PRE-FLIGHT (v1.10): inspect the file BEFORE importing.
            # A stale challenge file whose loop runs at import time
            # hijacks the whole program — no abort, START dead, power
            # cycle required (the 'can't cancel line follow' bug, three
            # times now). Importing is the point of no return, so the
            # source is checked for a run() entry point first, and a
            # stale file is REFUSED with a loud log instead of run.
            fname = module_name + ".py"
            src = None
            try:
                with open(fname) as f:
                    src = f.read()
            except OSError:
                try:
                    with open("/" + fname) as f:
                        src = f.read()
                except OSError:
                    log("*** %s NOT FOUND on the robot's flash — "
                        "upload it, then press the button again ***"
                        % fname)
                    self.set_light(RED)
                    time.sleep(1.5)
                    return self.menu_mode
            if src is not None and "def run(" not in src:
                log("*** %s on the robot is STALE — it has no run() "
                    "entry point, so importing it would take over the "
                    "program and kill the START button. Upload the "
                    "current version from the repo. REFUSING to run. ***"
                    % fname)
                self.set_light(RED)
                time.sleep(1.5)
                return self.menu_mode

            self.set_light(CYAN)
            self.in_challenge = True     # arms disconnect/stale all-stop
            try:
                if module_name in sys.modules:
                    del sys.modules[module_name]  # force a fresh import
                mod = __import__(module_name)
                mod.run(self)
                log("CHALLENGE %s finished" % module_name)
            finally:
                self.in_challenge = False
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
log("======== BOOT: main_code v1.18 ======== batt=%.2fV" % battery_voltage())
if battery_voltage() < LOW_BATT_V:
    log("*** WARNING: battery LOW for a %d-cell pack (<%.1fV) ***"
        % (BATT_CELLS, LOW_BATT_V))

MainCode().run()
