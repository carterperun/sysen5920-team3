"""
tower_smash_auto.py — SYSEN 5920 Team 3
XRP Proving Ground: "AUTO SMASH TOWER" — v2.0 (ram + clearing sweeps)

v2.0 REWRITE (8/6, per driver request): the standoff-hammer plan is
retired — this is now a basic RAM script:

  1. Robot is manually pointed at the tower. On launch it RANGES the
     tower with the ultrasonic (median of several pings).
  2. Drives that measured distance + SMASH_EXTRA_CM (20cm) at HIGH
     SPEED — straight through the blocks, IMU heading hold.
  3. Backs up 10cm — that puts it roughly where the tower stood.
  4. Clearing sweeps, three times over: spin 90 degrees, drive 20cm
     forward, back up 20cm. Any bricks still standing near the tower
     spot get knocked over or pushed clear.

Launch: from the main_code MENU press D-PAD UP — that press IS the
start. START (or any all-stop) aborts at any moment; every loop here
polls the supervisor abort. Standalone: run the file directly, then
press Button 0 ("A") or the USER button.

Logs to tower_log.txt on flash (same black-box scheme as sumo/line).
Positive turn = LEFT (CCW), same convention as the other scripts.
"""

from XRPLib.defaults import *
import time
import sys

# Supervisor hook: main_code.py calls run(sv); sv.check_abort raises
# MenuAbort on any all-stop. Standalone leaves it a no-op.
_HOOKS = {"abort": lambda: None}

def _abort():
    _HOOKS["abort"]()

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # standalone only
START_BUTTON = 0            # standalone only ("A")

# The ram
SMASH_EXTRA_CM = 20.0       # drive the measured distance PLUS this
SMASH_EFFORT = 0.9          # high speed — this is a smash
SMASH_MAX_CM = 120.0        # sanity cap: refuse a longer charge even if
                            # the echo says so (bad ping = bad plan)
SMASH_MIN_CM = 5.0          # echo closer than this = something's wrong
RANGE_MAX_CM = 100.0        # echo farther than this (or 65535 timeout)
                            # = no tower seen -> abort with red LED
BACK_TO_TOWER_CM = 10.0     # reverse after the ram = tower spot

# The clearing sweeps
SWEEP_COUNT = 3
SWEEP_TURN_DEG = 90         # LEFT 90 each time (270 total)
SWEEP_FWD_CM = 20.0
SWEEP_EFFORT = 0.7
BACKUP_EFFORT = 0.7         # reverse needs more than forward (proven)

# Motion plumbing (matches the tuned values in sumo/main_code)
TURN_EFFORT = 0.75
MIN_EFFORT = 0.4            # stiction floor — no wheel weaker than this
WIGGLE_BIAS = 0.18
WIGGLE_PERIOD_S = 0.22
WIGGLE_TOL_DEG = 3.0
LEFT_TURN_BOOST = 1.2       # this robot turns LEFT weaker than right
FWD_KP = 0.02               # IMU heading-hold gain on straights
FWD_CORR_MAX = 0.15

MAX_RUNTIME_S = 90

BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS

# Logging (black-box scheme shared with sumo/line)
LOG_TO_FILE = True
LOG_PATH = "tower_log.txt"
HEARTBEAT_S = 0.5

# ------------------------------- LOGGING ---------------------------------

_BOOT_MS = time.ticks_ms()
_LOG_BROKEN = {"reported": False}

def battery_voltage():
    try:
        from machine import Pin, ADC
        return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)
    except Exception:
        return -1.0

def log(msg, console=True):
    t = time.ticks_diff(time.ticks_ms(), _BOOT_MS) / 1000
    line = "[%8.2fs] %s" % (t, msg)
    if console:
        print(line)
    if LOG_TO_FILE:
        try:
            f = open(LOG_PATH, "a")
            f.write(line + "\n")
            f.close()
        except Exception as e:
            if not _LOG_BROKEN["reported"]:
                _LOG_BROKEN["reported"] = True
                print("*** TOWER LOG WRITE FAILED: %r ***" % e)

def _rotate_log():
    try:
        import os
        if os.stat(LOG_PATH)[6] > 200 * 1024:
            os.remove(LOG_PATH)
            print("tower log rotated")
    except Exception:
        pass

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

# ------------------------------- HELPERS ---------------------------------

def set_status(r, g, b):
    try:
        board.set_rgb_led(r, g, b)
    except Exception:
        pass

def front_distance():
    """Median of 5 pings (the ram plan hangs on this one number, so it
    gets two extra samples over the usual 3). 65535 = nothing seen."""
    reads = []
    for _ in range(5):
        reads.append(rangefinder.distance())
        time.sleep(0.015)
    reads.sort()
    return reads[2]

def normalize(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def wiggle_turn(degrees, effort=TURN_EFFORT, timeout_s=None):
    """Relative in-place turn with the anti-friction wiggle (same
    scheme as sumo v4.5+). True = reached."""
    target = imu.get_yaw() + degrees
    if timeout_s is None:
        timeout_s = 1.5 + abs(degrees) / 90.0 * 2.0
    t0 = time.ticks_ms()
    phase_ms = t0
    bias = WIGGLE_BIAS
    try:
        while True:
            _abort()
            err = normalize(target - imu.get_yaw())
            if abs(err) <= WIGGLE_TOL_DEG:
                return True
            now = time.ticks_ms()
            if time.ticks_diff(now, t0) > timeout_s * 1000:
                log("turn: timeout, %.0f deg short" % err)
                return False
            if time.ticks_diff(now, phase_ms) >= WIGGLE_PERIOD_S * 1000:
                phase_ms = now
                bias = -bias
            mag = min(0.95, effort * (LEFT_TURN_BOOST if err > 0 else 1.0))
            eff = mag if err > 0 else -mag
            l = -eff + bias
            r = eff + bias
            if 0 < abs(l) < MIN_EFFORT:
                l = MIN_EFFORT if l > 0 else -MIN_EFFORT
            if 0 < abs(r) < MIN_EFFORT:
                r = MIN_EFFORT if r > 0 else -MIN_EFFORT
            drivetrain.set_effort(l, r)
            time.sleep(0.01)
    finally:
        drivetrain.stop()

def verified_turn(degrees):
    """Turn and retry once at higher effort if it stalled short."""
    target = imu.get_yaw() + degrees
    wiggle_turn(degrees)
    err = normalize(target - imu.get_yaw())
    if abs(err) > WIGGLE_TOL_DEG + 2:
        log("turn: %.0f deg short — retrying at 0.9" % err)
        wiggle_turn(err, 0.9, timeout_s=3)

def _traveled(start_l, start_r):
    dl = drivetrain.get_left_encoder_position() - start_l
    dr = drivetrain.get_right_encoder_position() - start_r
    return (dl + dr) / 2

def drive_straight(dist_cm, effort, tag, timeout_s=None):
    """Signed straight drive with IMU heading hold, encoder target,
    stiction floor, heartbeat logging and abort polling. Returns the
    actual distance driven (signed)."""
    if timeout_s is None:
        timeout_s = 2.0 + abs(dist_cm) / 20.0
    sign = 1 if dist_cm >= 0 else -1
    eff = abs(effort) * sign
    hold_yaw = imu.get_yaw()
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    t0 = time.ticks_ms()
    last_beat = t0
    try:
        while True:
            _abort()
            trav = _traveled(start_l, start_r)
            if abs(trav) >= abs(dist_cm):
                break
            now = time.ticks_ms()
            if time.ticks_diff(now, t0) > timeout_s * 1000:
                log("%s: TIMEOUT at %.1f of %.1fcm (stall?)"
                    % (tag, trav, dist_cm))
                break
            err = normalize(hold_yaw - imu.get_yaw())
            corr = max(-FWD_CORR_MAX, min(FWD_CORR_MAX, FWD_KP * err))
            l = eff - corr * sign
            r = eff + corr * sign
            if 0 < abs(l) < MIN_EFFORT:
                l = MIN_EFFORT * sign
            if 0 < abs(r) < MIN_EFFORT:
                r = MIN_EFFORT * sign
            drivetrain.set_effort(l, r)
            if time.ticks_diff(now, last_beat) >= HEARTBEAT_S * 1000:
                last_beat = now
                log("%s: hb trav=%.1f/%.1fcm yaw=%+.1f batt=%.2fV"
                    % (tag, trav, dist_cm, imu.get_yaw(),
                       battery_voltage()), console=False)
            time.sleep(0.01)
    finally:
        drivetrain.stop()
    actual = _traveled(start_l, start_r)
    if abs(actual - dist_cm) > 5:
        log("%s: commanded %.1fcm, drove %.1fcm" % (tag, dist_cm, actual))
    return actual

# -------------------------------- THE RUN --------------------------------

def smash_tower():
    """Range, ram, back off, three 90-degree clearing sweeps."""
    d = front_distance()
    log("range: tower echo = %.1fcm" % d)
    if d < SMASH_MIN_CM or d > RANGE_MAX_CM:
        log("*** NO TOWER: echo %.1fcm outside %.0f-%.0fcm. Point the "
            "robot at the tower and relaunch. ***"
            % (d, SMASH_MIN_CM, RANGE_MAX_CM))
        set_status(255, 0, 0)
        time.sleep(1.5)
        return False

    charge = min(d + SMASH_EXTRA_CM, SMASH_MAX_CM)
    log("SMASH: charging %.1fcm (echo %.1f + %.0f extra) at %.2f effort"
        % (charge, d, SMASH_EXTRA_CM, SMASH_EFFORT))
    drive_straight(charge, SMASH_EFFORT, "smash")

    log("back up %.0fcm to the tower spot" % BACK_TO_TOWER_CM)
    drive_straight(-BACK_TO_TOWER_CM, BACKUP_EFFORT, "back")

    for i in range(SWEEP_COUNT):
        _abort()
        log("clear %d/%d: LEFT %d deg, out %.0fcm and back"
            % (i + 1, SWEEP_COUNT, SWEEP_TURN_DEG, SWEEP_FWD_CM))
        verified_turn(SWEEP_TURN_DEG)
        out = drive_straight(SWEEP_FWD_CM, SWEEP_EFFORT, "clear%d" % (i + 1))
        drive_straight(-out, BACKUP_EFFORT, "clear%d-back" % (i + 1))
    return True

def run(sv=None):
    """Supervisor entry point: main_code MENU -> D-PAD UP. Point the
    robot at the tower BEFORE pressing — the press is the launch."""
    if sv is not None:
        _HOOKS["abort"] = sv.check_abort
    _rotate_log()
    log("===== TOWER SMASH v2.0 launch: batt=%.2fV yaw=%+.1f"
        % (battery_voltage(), imu.get_yaw()))
    log("config: extra=%.0fcm smash=%.2f sweeps=%dx%ddeg/%.0fcm "
        "backup=%.2f" % (SMASH_EXTRA_CM, SMASH_EFFORT, SWEEP_COUNT,
                         SWEEP_TURN_DEG, SWEEP_FWD_CM, BACKUP_EFFORT))
    if battery_voltage() < LOW_BATT_V:
        log("*** WARNING: battery LOW at launch (%.2fV) ***"
            % battery_voltage())
    try:
        drivetrain.stop()
        try:
            ok = smash_tower()
        except Exception as e:
            if type(e).__name__ != "MenuAbort":
                log_exception(e)
            raise
        finally:
            drivetrain.stop()
            set_status(0, 255, 0)
            log("DONE: batt=%.2fV" % battery_voltage())
        return ok
    finally:
        _HOOKS["abort"] = lambda: None

# ------------------------- STANDALONE OPERATION --------------------------

def _standalone():
    from pestolink import PestoLinkAgent
    pestolink = PestoLinkAgent(ROBOT_NAME)
    log("=========== BOOT: tower_smash_auto v2.0 (standalone) batt=%.2fV"
        % battery_voltage())
    board.led_on()
    set_status(255, 120, 0)
    drivetrain.stop()
    while True:
        if pestolink.is_connected() and pestolink.get_button(START_BUTTON):
            break
        if board.is_button_pressed():
            break
        time.sleep(0.02)
    while pestolink.get_button(START_BUTTON) or board.is_button_pressed():
        time.sleep(0.02)
    set_status(0, 255, 255)
    time.sleep(0.5)
    try:
        run()
    finally:
        board.led_blink(4)

if __name__ == "__main__":
    _standalone()
