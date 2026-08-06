"""
line_follow_auto.py — SYSEN 5920 Team 3
XRP Proving Ground: "Line Follower" challenge (autonomous attempt) — v2

v2: now DUAL-MODE — importable by main_code.py (the competition
supervisor) or runnable standalone.
  * From the supervisor: MENU -> B launches this. No second
    PestoLinkAgent is created (the supervisor owns the radio), launch is
    immediate (the B press IS the start), and the supervisor's START
    button aborts at any point — every loop here calls check_abort().
  * Standalone (run the file directly): same as v1 — own PestoLink,
    Button 1 or USER button to launch.

Scoring context (Appendix A):
  Autonomous: 20 pts per 1/4 completed (max 80)
  RULES: straddle the line; a wheel crossing the line ends the attempt.
  Customer note (8/4): start from the "bottom end" of the path.

Calibration:
  Stored in linecal.txt and reused automatically. To (re)calibrate —
  works in both modes, no controller needed:
    hold the USER button while the program starts this challenge, OR
    have no linecal.txt yet. Then: both sensors over BARE FLOOR, press
    USER; both sensors over TAPE, press USER. White flashes confirm.
  Polarity doesn't matter (light tape on dark floor is fine — the
  arena floor reads ~0.7 and the tape ~0.05, and the normalization
  handles the inverted span automatically).

Controller (standalone mode only): Button 1 or USER button launches.
Positive turn/steer = LEFT (CCW). Negative = RIGHT (CW).
"""

from XRPLib.defaults import *
import time

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # standalone only (8 chars max)
START_BUTTON = 1            # standalone only

BASE_EFFORT = 0.30          # cruise effort. Raise only after clean runs.
KP = 0.55                   # proportional steering gain
KD = 0.12                   # derivative gain — damps wiggle
STEER_LIMIT = 0.25          # max steering correction
LOOP_S = 0.01               # control loop period

ON_LINE_FRAC = 0.45         # normalized reading above this = on tape
LOST_FRAC = 0.25            # BOTH below this = line lost

RECOVER_EFFORT = 0.22       # slow in-place pivot effort
RECOVER_MAX_S = 1.2         # per-side pivot time cap
END_LOST_S = 2.5            # recovery failing this long = end of line
MAX_RUNTIME_S = 240

CAL_FILE = "linecal.txt"

# ------------------------------- HELPERS ---------------------------------

def _noop():
    pass

def set_status(r, g, b):
    try:
        board.set_rgb_led(r, g, b)
    except Exception:
        pass

def flash_white(n=2):
    for _ in range(n):
        set_status(255, 255, 255)
        time.sleep(0.12)
        set_status(0, 0, 0)
        time.sleep(0.12)

def read_raw():
    l = r = 0.0
    for _ in range(4):
        l += reflectance.get_left()
        r += reflectance.get_right()
        time.sleep(0.002)
    return l / 4, r / 4

def wait_press_release(abort=_noop):
    while not board.is_button_pressed():
        abort()
        time.sleep(0.02)
    while board.is_button_pressed():
        abort()
        time.sleep(0.02)

# ----------------------------- CALIBRATION -------------------------------

CAL = {"floor": 0.1, "tape": 0.9}

def save_cal():
    try:
        with open(CAL_FILE, "w") as f:
            f.write("%f,%f" % (CAL["floor"], CAL["tape"]))
    except Exception:
        pass

def load_cal():
    try:
        with open(CAL_FILE) as f:
            floor_s, tape_s = f.read().split(",")
        CAL["floor"], CAL["tape"] = float(floor_s), float(tape_s)
        return True
    except Exception:
        return False

def calibrate(abort=_noop):
    board.led_blink(1)
    print("CAL 1/2: both sensors over BARE FLOOR, press USER")
    wait_press_release(abort)
    l, r = read_raw()
    CAL["floor"] = (l + r) / 2
    flash_white()
    print("  floor = %.3f" % CAL["floor"])

    print("CAL 2/2: both sensors over the TAPE, press USER")
    wait_press_release(abort)
    l, r = read_raw()
    CAL["tape"] = (l + r) / 2
    flash_white()
    print("  tape  = %.3f" % CAL["tape"])
    board.led_off()

    if abs(CAL["tape"] - CAL["floor"]) < 0.08:
        print("WARNING: floor/tape contrast tiny. Re-doing calibration.")
        set_status(255, 0, 0)
        time.sleep(1.0)
        return calibrate(abort)
    save_cal()

def norm(raw):
    span = CAL["tape"] - CAL["floor"]
    x = (raw - CAL["floor"]) / span
    return 0.0 if x < 0 else (1.0 if x > 1 else x)

# ------------------------------ FOLLOWING --------------------------------

def follow(abort=_noop):
    """PD straddle-follow until the line ends. abort() is called every
    pass — the supervisor raises MenuAbort from it for the all-stop."""
    last_error = 0.0
    last_seen_side = 0
    lost_since = None
    start_ms = time.ticks_ms()
    drivetrain.reset_encoder_position()

    while True:
        abort()
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000
        if elapsed > MAX_RUNTIME_S:
            print("Time cap reached")
            return

        nl = norm(reflectance.get_left())
        nr = norm(reflectance.get_right())

        if nl < LOST_FRAC and nr < LOST_FRAC:
            drivetrain.stop()
            if lost_since is None:
                lost_since = time.ticks_ms()
            if not reacquire(last_seen_side, abort):
                lost_ms = time.ticks_diff(time.ticks_ms(), lost_since)
                if lost_ms > END_LOST_S * 1000:
                    dist = (drivetrain.get_left_encoder_position() +
                            drivetrain.get_right_encoder_position()) / 2
                    print("Line gone %.1fs — ending. ~%.0f cm covered "
                          "in %.0fs" % (END_LOST_S, dist, elapsed))
                    return
            else:
                lost_since = None
                last_error = 0.0
            continue

        lost_since = None
        if nl > ON_LINE_FRAC and nr < LOST_FRAC:
            last_seen_side = -1
        elif nr > ON_LINE_FRAC and nl < LOST_FRAC:
            last_seen_side = 1

        error = nr - nl
        steer = KP * error + KD * (error - last_error) / LOOP_S / 100
        last_error = error
        if steer > STEER_LIMIT:
            steer = STEER_LIMIT
        elif steer < -STEER_LIMIT:
            steer = -STEER_LIMIT

        drivetrain.set_effort(BASE_EFFORT + steer, BASE_EFFORT - steer)
        time.sleep(LOOP_S)

def reacquire(side, abort=_noop):
    """Wheels-stopped pivots toward `side`, then the other way."""
    order = (side, -side) if side != 0 else (1, -1)
    for direction in order:
        if direction == 0:
            continue
        t0 = time.ticks_ms()
        drivetrain.set_effort(RECOVER_EFFORT * direction,
                              -RECOVER_EFFORT * direction)
        while time.ticks_diff(time.ticks_ms(), t0) < RECOVER_MAX_S * 1000:
            abort()
            if (norm(reflectance.get_left()) > ON_LINE_FRAC or
                    norm(reflectance.get_right()) > ON_LINE_FRAC):
                drivetrain.stop()
                return True
            time.sleep(0.005)
        drivetrain.set_effort(-RECOVER_EFFORT * direction,
                              RECOVER_EFFORT * direction)
        time.sleep(time.ticks_diff(time.ticks_ms(), t0) / 1000)
        drivetrain.stop()
    return False

# ------------------------------ ENTRY POINTS -----------------------------

def run(sv=None):
    """Supervisor entry point: main_code.py imports this module and calls
    run(self). Launch is immediate — place the robot straddling the line
    BEFORE pressing the menu button. sv.check_abort() (START button) can
    end the run at any moment; the supervisor catches MenuAbort and any
    crash, so this function just does the work."""
    abort = sv.check_abort if sv else _noop
    drivetrain.stop()

    # Recalibrate if USER is held at launch, or no calibration exists.
    if board.is_button_pressed() or not load_cal():
        calibrate(abort)
    else:
        print("Using stored cal: floor=%.3f tape=%.3f"
              % (CAL["floor"], CAL["tape"]))

    set_status(0, 255, 255)          # cyan = run in progress
    time.sleep(0.3)
    try:
        follow(abort)
    finally:
        drivetrain.stop()
        set_status(0, 255, 0)

# ------------------------- STANDALONE OPERATION --------------------------

def _standalone():
    from pestolink import PestoLinkAgent
    pestolink = PestoLinkAgent(ROBOT_NAME)
    board.led_on()
    drivetrain.stop()

    if board.is_button_pressed() or not load_cal():
        while board.is_button_pressed():
            time.sleep(0.02)
        calibrate()
    else:
        print("Using stored cal: floor=%.3f tape=%.3f"
              % (CAL["floor"], CAL["tape"]))

    set_status(255, 120, 0)          # orange = place robot, connect ctrl
    print("Place robot straddling the line. Button 1 (or USER) launches.")
    connected = False
    while True:
        if pestolink.is_connected():
            if not connected:
                connected = True
                set_status(0, 0, 255)
            if pestolink.get_button(START_BUTTON):
                break
        else:
            if connected:
                connected = False
                set_status(255, 120, 0)
        if board.is_button_pressed():
            break
        time.sleep(0.02)
    while pestolink.get_button(START_BUTTON) or board.is_button_pressed():
        time.sleep(0.02)

    set_status(0, 255, 255)
    time.sleep(0.5)
    try:
        follow()
    finally:
        drivetrain.stop()
        set_status(0, 255, 0)
        board.led_blink(4)

if __name__ == "__main__":
    _standalone()
