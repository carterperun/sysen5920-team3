"""
maze_solver.py — SYSEN 5920 Team 3
XRP Proving Ground: MAZE (right-wall follower) — v1.0

Based on the team's draft solver, reworked for the field problems seen
in testing ("not turning consistently 90 degrees" / "turns too wide"):

  * ABSOLUTE-HEADING TURNS: the four cardinal directions are locked to
    absolute IMU yaw targets captured at launch (north = launch yaw).
    Every turn drives to ITS CARDINAL TARGET, not "90 from wherever I
    ended up" — so turn error can never accumulate across the run. Each
    turn is VERIFIED against the IMU and re-commanded with escalating
    effort until it's within tolerance (same scheme that fixed the
    sumo turns).
  * WIGGLE TURNS: the draft's smooth counter-rotation is exactly what
    stalls/arcs on this floor — field friction loads up one wheel, it
    stops, and the other wheel drags the robot around a WIDE arc (the
    clearance problem). The proven anti-friction wiggle (alternating
    fore/aft bias, per-wheel stiction floor, LEFT_TURN_BOOST) turns in
    place with a slight shimmy instead.
    (Also: the draft called drivetrain.set_zero_effort_behavior(),
    which doesn't exist in our XRPLib — it would have crashed with
    AttributeError on the first turn.)
  * HEADING-HELD CELL DRIVES: each 30.48cm cell drive holds the cell's
    cardinal yaw with the IMU, so a slightly-off turn gets corrected
    DURING the drive instead of walking the robot into a wall.
  * BLACK-BOX LOG (maze_log.txt on flash, same scheme as sumo/line):
    every turn logs commanded vs ACHIEVED yaw, every cell drive logs
    commanded vs encoder distance, every wall check logs the ping and
    the decision, plus battery and a crash traceback if anything dies.

Supervisor use: main_code MENU -> Y imports this and calls run(sv).
Place the robot in the START cell facing "north" (the far goal wall)
BEFORE pressing Y — the press is the launch, START aborts anywhere.
Standalone: run the file directly, Y (button 3) or USER launches.

Positive yaw = LEFT (CCW). Headings: 0=N, 1=E, 2=S, 3=W (clockwise),
so the absolute yaw target for heading h is launch_yaw - 90*h.
"""

from XRPLib.defaults import *
import time
import sys

_HOOKS = {"abort": lambda: None}

def _abort():
    _HOOKS["abort"]()

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # standalone only
START_BUTTON = 3            # standalone only. Y is button 3 on OUR pad
                            # (confirmed by the button-spy logs, 8/6) —
                            # the draft's "Y = 2" is the X button here.

# Maze geometry. Coordinates are (column, row).
GRID_WIDTH = 4
GRID_HEIGHT = 6
START = (0, 0)
GOAL = (0, 5)
START_HEADING = 0           # 0 = north (robot placed facing the goal end)

# Motion and sensor tuning.
CELL_DISTANCE_CM = 30.48
DRIVE_EFFORT = 0.5          # cell drives (the draft's 0.45 is below the
                            # stiction floor once the PID tapers)
MIN_EFFORT = 0.4            # per-wheel stiction floor (proven value)
WALL_DISTANCE_CM = 20.0
SETTLE_TIME_S = 0.20
SENSOR_SAMPLES = 5
MOTION_TIMEOUT_S = 5.0

# Trim knobs if testing shows systematic over/under travel.
CELL_DISTANCE_SCALE = 1.0
TURN_ANGLE_SCALE = 1.0      # applied to the 90-deg cardinal spacing

# Turns — the proven wiggle scheme (sumo v4.5+ / main_code v1.14+).
TURN_EFFORT = 0.75
TURN_TOL_DEG = 4.0
TURN_RETRIES = 3
WIGGLE_BIAS = 0.18
WIGGLE_PERIOD_S = 0.22
WIGGLE_TOL_DEG = 3.0
LEFT_TURN_BOOST = 1.2       # this robot turns LEFT (CCW) weaker

# Heading hold during cell drives
FWD_KP = 0.02
FWD_CORR_MAX = 0.15

MAX_RUNTIME_S = 240

BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS

LOG_TO_FILE = True
LOG_PATH = "maze_log.txt"
HEARTBEAT_S = 0.5

DIRECTION_NAMES = ("north", "east", "south", "west")
DIRECTIONS = (
    (0, 1),    # north
    (1, 0),    # east
    (0, -1),   # south
    (-1, 0),   # west
)

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
                print("*** MAZE LOG WRITE FAILED: %r ***" % e)

def _rotate_log():
    try:
        import os
        if os.stat(LOG_PATH)[6] > 200 * 1024:
            os.remove(LOG_PATH)
            print("maze log rotated")
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

def normalize(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def average_distance():
    """Averaged ultrasonic reading. 65535 timeouts are dropped; if ALL
    samples time out, returns 999 (= 'open', nothing in range)."""
    readings = []
    for _ in range(SENSOR_SAMPLES):
        _abort()
        d = rangefinder.distance()
        if 0 < d < 400:
            readings.append(d)
        time.sleep(0.03)
    if not readings:
        return 999.0
    return sum(readings) / len(readings)

# ------------------------------- TURNING ---------------------------------
# Cardinal yaw targets are locked at launch: CARDINAL["yaw0"] is "north".
# heading h target = yaw0 - 90*h (positive yaw = CCW = left; headings
# advance clockwise). Turning always drives to the absolute target, so
# per-turn error CANNOT accumulate into the next turn.

CARDINAL = {"yaw0": 0.0}

def heading_yaw(heading):
    return CARDINAL["yaw0"] - 90.0 * TURN_ANGLE_SCALE * heading

def wiggle_turn(degrees, effort=TURN_EFFORT, timeout_s=None):
    """Relative in-place turn with the anti-friction wiggle. The
    alternating fore/aft bias keeps both wheels breaking static
    friction so the robot spins IN PLACE (a slight shimmy) instead of
    arcing wide around one stalled wheel. True = reached."""
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
                log("turn: wiggle timeout, %.0f deg short" % err)
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

def turn_to_heading_idx(heading, why):
    """Turn to a cardinal direction's ABSOLUTE yaw target, verify with
    the IMU, retry with escalating effort. Logs commanded vs achieved."""
    target = heading_yaw(heading)
    before = imu.get_yaw()
    for attempt in range(TURN_RETRIES):
        _abort()
        delta = normalize(target - imu.get_yaw())
        if abs(delta) <= TURN_TOL_DEG:
            break
        boosted = min(0.9, TURN_EFFORT + 0.15 * attempt)
        wiggle_turn(delta, boosted, timeout_s=3)
    achieved = normalize(imu.get_yaw() - before)
    err = normalize(target - imu.get_yaw())
    log("turn(%s -> %s): yaw %+.1f -> %+.1f (moved %+.1f, err %+.1f%s)"
        % (why, DIRECTION_NAMES[heading], before, imu.get_yaw(),
           achieved, err, "" if abs(err) <= TURN_TOL_DEG
           else " — STILL SHORT, battery/friction?"))
    time.sleep(SETTLE_TIME_S)
    return abs(err) <= TURN_TOL_DEG

# ------------------------------- DRIVING ---------------------------------

def _traveled(start_l, start_r):
    dl = drivetrain.get_left_encoder_position() - start_l
    dr = drivetrain.get_right_encoder_position() - start_r
    return (dl + dr) / 2

def drive_cell(heading):
    """Drive one cell forward holding the cardinal yaw. Logs commanded
    vs encoder distance. Returns the actual distance driven."""
    dist = CELL_DISTANCE_CM * CELL_DISTANCE_SCALE
    hold = heading_yaw(heading)
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    t0 = time.ticks_ms()
    last_beat = t0
    try:
        while True:
            _abort()
            trav = _traveled(start_l, start_r)
            if trav >= dist:
                break
            now = time.ticks_ms()
            if time.ticks_diff(now, t0) > MOTION_TIMEOUT_S * 1000:
                log("drive: TIMEOUT at %.1f of %.1fcm (stall?)"
                    % (trav, dist))
                break
            err = normalize(hold - imu.get_yaw())
            corr = max(-FWD_CORR_MAX, min(FWD_CORR_MAX, FWD_KP * err))
            l = DRIVE_EFFORT - corr
            r = DRIVE_EFFORT + corr
            if 0 < l < MIN_EFFORT:
                l = MIN_EFFORT
            if 0 < r < MIN_EFFORT:
                r = MIN_EFFORT
            drivetrain.set_effort(l, r)
            if time.ticks_diff(now, last_beat) >= HEARTBEAT_S * 1000:
                last_beat = now
                log("drive: hb trav=%.1f/%.1fcm yaw=%+.1f batt=%.2fV"
                    % (trav, dist, imu.get_yaw(), battery_voltage()),
                    console=False)
            time.sleep(0.01)
    finally:
        drivetrain.stop()
    actual = _traveled(start_l, start_r)
    log("drive %s: %.1fcm of %.1fcm, end yaw %+.1f (target %+.1f)"
        % (DIRECTION_NAMES[heading], actual, dist, imu.get_yaw(),
           heading_yaw(heading)))
    time.sleep(SETTLE_TIME_S)
    return actual

# ------------------------------- THE MAZE --------------------------------

def adjacent_cell(position, heading):
    dx, dy = DIRECTIONS[heading]
    return position[0] + dx, position[1] + dy

def is_inside_maze(position):
    x, y = position
    return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT

def path_is_open(position, heading):
    """Grid boundary first, then the physical wall sensor."""
    next_position = adjacent_cell(position, heading)
    if not is_inside_maze(next_position):
        log("check %s: grid boundary (%s is outside)"
            % (DIRECTION_NAMES[heading], next_position))
        return False
    d = average_distance()
    open_ = d > WALL_DISTANCE_CM
    log("check %s: ping %.1fcm -> %s"
        % (DIRECTION_NAMES[heading], d, "OPEN" if open_ else "WALL"))
    return open_

def move_one_cell(position, heading):
    actual = drive_cell(heading)
    if actual < CELL_DISTANCE_CM * CELL_DISTANCE_SCALE * 0.6:
        log("*** cell drive badly short (%.1fcm) — dead-reckoning is "
            "now suspect ***" % actual)
    return adjacent_cell(position, heading)

def solve_maze():
    """Right-wall follower: try right, then straight, then left, then
    back. Every state change is logged."""
    position = START
    heading = START_HEADING
    CARDINAL["yaw0"] = imu.get_yaw()     # placed facing NORTH = launch yaw
    log("maze: start %s facing %s (yaw0 %+.1f) grid %dx%d goal %s"
        % (position, DIRECTION_NAMES[heading], CARDINAL["yaw0"],
           GRID_WIDTH, GRID_HEIGHT, GOAL))
    start_ms = time.ticks_ms()
    steps = 0
    time.sleep(0.5)
    while position != GOAL:
        _abort()
        if time.ticks_diff(time.ticks_ms(), start_ms) \
                > MAX_RUNTIME_S * 1000:
            log("maze: time cap %.0fs reached at %s" %
                (MAX_RUNTIME_S, position))
            return False
        # 1. right-hand path
        h = (heading + 1) % 4
        turn_to_heading_idx(h, "try right")
        heading = h
        if path_is_open(position, heading):
            position = move_one_cell(position, heading)
        else:
            # 2. back to straight ahead
            h = (heading - 1) % 4
            turn_to_heading_idx(h, "back to fwd")
            heading = h
            if path_is_open(position, heading):
                position = move_one_cell(position, heading)
            else:
                # 3. left-hand path
                h = (heading - 1) % 4
                turn_to_heading_idx(h, "try left")
                heading = h
                if path_is_open(position, heading):
                    position = move_one_cell(position, heading)
                else:
                    # 4. dead end — turn to face backward
                    h = (heading - 1) % 4
                    turn_to_heading_idx(h, "dead end")
                    heading = h
                    if not path_is_open(position, heading):
                        log("*** BOXED IN at %s — no open path in any "
                            "direction ***" % (position,))
                        set_status(255, 0, 0)
                        return False
                    position = move_one_cell(position, heading)
        steps += 1
        log("now at %s facing %s (step %d, batt=%.2fV)"
            % (position, DIRECTION_NAMES[heading], steps,
               battery_voltage()))
    log("maze: GOAL reached at %s in %d steps, %.0fs"
        % (position, steps,
           time.ticks_diff(time.ticks_ms(), start_ms) / 1000))
    return True

# ------------------------------ ENTRY POINTS -----------------------------

def run(sv=None):
    """Supervisor entry point: main_code MENU -> Y. Place the robot in
    the START cell facing north FIRST — the Y press is the launch."""
    if sv is not None:
        _HOOKS["abort"] = sv.check_abort
    _rotate_log()
    log("===== MAZE v1.0 launch: batt=%.2fV yaw=%+.1f"
        % (battery_voltage(), imu.get_yaw()))
    log("config: cell=%.1fcm drive=%.2f turn=%.2f tol=%.0fdeg "
        "wall<%.0fcm scale=%.2f/%.2f"
        % (CELL_DISTANCE_CM, DRIVE_EFFORT, TURN_EFFORT, TURN_TOL_DEG,
           WALL_DISTANCE_CM, CELL_DISTANCE_SCALE, TURN_ANGLE_SCALE))
    if battery_voltage() < LOW_BATT_V:
        log("*** WARNING: battery LOW at launch (%.2fV) ***"
            % battery_voltage())
    try:
        drivetrain.stop()
        try:
            ok = solve_maze()
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
    log("=========== BOOT: maze_solver v1.0 (standalone) batt=%.2fV"
        % battery_voltage())
    board.led_on()
    set_status(255, 120, 0)
    drivetrain.stop()
    log("waiting for button %d (Y) or USER..." % START_BUTTON)
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
