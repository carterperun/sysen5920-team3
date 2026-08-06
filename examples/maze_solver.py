"""
maze_solver.py — SYSEN 5920 Team 3
XRP Proving Ground: MAZE (right-wall follower) — v1.2

v1.2 (8/6 night logs — "turns not strictly 90, scanning misaligned"):
the turns ARE IMU-locked to absolute cardinal targets and the logs
show them landing within 3 degrees WHEN THEY MOVE — the failures were
turns that couldn't rotate at all ("wiggle timeout, 89 deg short" =
moved ~1 degree: the robot was pinned against a wall, where the
pressed ultrasonic also reads 999 = "open"). Per driver request:
  * TURN-UNSTICK: if a turn attempt produces almost no rotation, back
    up ~5cm and retry the same IMU-verified turn (up to 4 attempts,
    escalating effort). The retry re-aims at the CARDINAL target, so
    alignment is restored, not accumulated.
  * A blocked cell drive now backs up its driven distance PLUS 5cm —
    backing up only the 0.4cm it managed left it still pinned.
  * Cell drives give up after 2.5s of zero progress instead of
    grinding the full 5s timeout.

v1.1 (8/6 field logs — "runs into a wall completely, then can't turn"):
  * TURN CLEARANCE: before scanning, if the front ping reads under
    TURN_CLEARANCE_CM (2cm) the robot backs up ~6cm first so it has
    room to rotate instead of grinding on the wall.
  * BLOCKED-DIRECTION MEMORY: the logged run drove "north" 0.3cm into
    a wall the ultrasonic called 999cm/OPEN (pressed against a wall,
    every ping times out — reads as 'nothing there'), counted the cell
    as traversed anyway, and corrupted the map. Now a cell drive that
    covers under 70% of a cell does NOT advance the map: the robot
    backs up to the cell center and remembers that (cell, direction)
    as blocked so the wall-follower routes around it. A drive that
    covers 70%+ counts as arrived (a stall 1cm short is arrival, not
    a wall).
  * STALL BOOST in cell drives: no progress for 1s raises effort a
    notch (battery sag was freezing drives mid-cell at 4.1V).
  * STATUS LIGHT (per driver request): YELLOW = scanning/turning at a
    cell, GREEN = driving into a NEW cell, RED = driving into a cell
    it has already visited (backtracking).

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
  * BLACK-BOX LOG (unified LOG.TXT on flash, shared by all programs):
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

# Wall recovery (v1.1)
TURN_CLEARANCE_CM = 2.0     # front ping under this before scanning ->
                            # back up first so the turn has room
CLEAR_BACKUP_CM = 6.0       # how far to back away from a wall
REVERSE_EFFORT = 0.7        # reverse needs more effort (proven)
CELL_OK_FRAC = 0.7          # a drive covering >= this fraction of a
                            # cell counts as ARRIVED; less = blocked,
                            # back to center, remember the wall
DRIVE_STALL_S = 1.0         # no progress this long -> boost effort
DRIVE_STALL_ADD = 0.15
DRIVE_STALL_MAX = 0.8

# Turns — the proven wiggle scheme (sumo v4.5+ / main_code v1.14+).
TURN_EFFORT = 0.75
TURN_EFFORT_LOW = 0.45      # finishing effort to limit overshoot
TURN_SLOW_ZONE_DEG = 35.0   # start slowing this far from the target
TURN_TOL_DEG = 2.0
TURN_RETRIES = 4            # v1.2: one more attempt (unstick eats one)
TURN_STUCK_DEG = 8.0        # attempt moved less than this with lots
                            # left to go = physically pinned -> back up
TURN_UNSTICK_CM = 5.0       # how far to back away before retrying
WIGGLE_BIAS = 0.18
WIGGLE_PERIOD_S = 0.22
WIGGLE_TOL_DEG = 2.0
LEFT_TURN_BOOST = 1.2       # this robot turns LEFT (CCW) weaker

# Heading hold during cell drives
FWD_KP = 0.02
FWD_CORR_MAX = 0.15

MAX_RUNTIME_S = 240

BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS

LOG_TO_FILE = True
LOG_PATH = "LOG.TXT"           # unified log — every program appends here
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
    t = time.ticks_ms() / 1000.0    # seconds since POWER-ON —
    #     the same clock in every program, so LOG.TXT reads as
    #     one continuous session timeline
    line = "[%9.2fs][MAZE ] %s" % (t, msg)
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
        if os.stat(LOG_PATH)[6] > 300 * 1024:
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
    samples time out, returns 999 (= 'open', nothing in range).
    CAUTION (v1.1): a sensor PRESSED against a wall also times out on
    every ping and reads 999 — that's why the blocked-direction memory
    exists; don't trust 'open' when a drive just stalled."""
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

def min_distance():
    """Minimum of 3 quick pings — for the clearance check, where the
    CLOSEST believable echo is what matters."""
    best = 999.0
    for _ in range(3):
        _abort()
        d = rangefinder.distance()
        if 0 < d < 400 and d < best:
            best = d
        time.sleep(0.02)
    return best

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
            commanded_effort = (TURN_EFFORT_LOW
                                if abs(err) <= TURN_SLOW_ZONE_DEG
                                else effort)
            mag = min(0.95, commanded_effort
                      * (LEFT_TURN_BOOST if err > 0 else 1.0))
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
    the IMU, retry with escalating effort. v1.2: an attempt that barely
    rotates means the robot is pinned on a wall — back up ~5cm and
    retry the same cardinal target (alignment is re-aimed, never
    accumulated). Logs commanded vs achieved."""
    target = heading_yaw(heading)
    before = imu.get_yaw()
    for attempt in range(TURN_RETRIES):
        _abort()
        delta = normalize(target - imu.get_yaw())
        if abs(delta) <= TURN_TOL_DEG:
            break
        boosted = min(0.9, TURN_EFFORT + 0.15 * attempt)
        yaw_a = imu.get_yaw()
        wiggle_turn(delta, boosted, timeout_s=3)
        moved = abs(imu.get_yaw() - yaw_a)
        if moved < TURN_STUCK_DEG and abs(delta) > TURN_TOL_DEG * 3:
            # pinned against a wall — the pressed ultrasonic reads 999
            # here, so the ping can't warn us; the dead turn is the tell
            log("turn: PINNED (moved %.0f of %.0f deg) — backing up "
                "%.0fcm and retrying" % (moved, abs(delta),
                                         TURN_UNSTICK_CM))
            drive_back(TURN_UNSTICK_CM, "turn-unstick")
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
    prog_ms = t0
    prog_cm = 0.0
    boosted = False
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
            # v1.1 stall boost: battery sag froze drives mid-cell
            if trav - prog_cm >= 0.5:
                prog_cm = trav
                prog_ms = now
                boosted = False
            stalled_ms = time.ticks_diff(now, prog_ms)
            if stalled_ms > 2500:
                # v1.2: 2.5s of zero progress = wall; stop grinding
                log("drive: BLOCKED (no progress %.1fs at %.1fcm)"
                    % (stalled_ms / 1000, trav))
                break
            eff = DRIVE_EFFORT
            if stalled_ms > DRIVE_STALL_S * 1000:
                eff = min(DRIVE_STALL_MAX, DRIVE_EFFORT + DRIVE_STALL_ADD)
                if not boosted:
                    boosted = True
                    log("drive: no progress at %.1fcm — boosting to %.2f"
                        % (trav, eff))
            err = normalize(hold - imu.get_yaw())
            corr = max(-FWD_CORR_MAX, min(FWD_CORR_MAX, FWD_KP * err))
            l = eff - corr
            r = eff + corr
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

def drive_back(dist_cm, why):
    """Straight reverse with encoder target and heading hold — used to
    get clear of walls. Returns actual distance backed (positive)."""
    if dist_cm < 0.5:
        return 0.0
    hold = imu.get_yaw()
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    t0 = time.ticks_ms()
    try:
        while True:
            _abort()
            trav = -_traveled(start_l, start_r)
            if trav >= dist_cm:
                break
            if time.ticks_diff(time.ticks_ms(), t0) > 2500:
                log("back(%s): stalled at %.1f of %.1fcm"
                    % (why, trav, dist_cm))
                break
            err = normalize(hold - imu.get_yaw())
            corr = max(-FWD_CORR_MAX, min(FWD_CORR_MAX, FWD_KP * err))
            drivetrain.set_effort(-REVERSE_EFFORT + corr,
                                  -REVERSE_EFFORT - corr)
            time.sleep(0.01)
    finally:
        drivetrain.stop()
    actual = -_traveled(start_l, start_r)
    log("back(%s): %.1fcm of %.1fcm" % (why, actual, dist_cm))
    time.sleep(SETTLE_TIME_S)
    return actual

def ensure_turn_clearance():
    """v1.1: if the nose is basically touching a wall, back up so the
    upcoming turns have room instead of grinding on it."""
    d = min_distance()
    if d < TURN_CLEARANCE_CM:
        log("clearance: ping %.1fcm < %.1f — backing up %.0fcm to turn"
            % (d, TURN_CLEARANCE_CM, CLEAR_BACKUP_CM))
        drive_back(CLEAR_BACKUP_CM, "clearance")

# ------------------------------- THE MAZE --------------------------------

# v1.1 run-state: cells already visited (for the status light) and
# (cell, direction) pairs that physically failed (virtual walls).
VISITED = set()
BLOCKED = set()

def adjacent_cell(position, heading):
    dx, dy = DIRECTIONS[heading]
    return position[0] + dx, position[1] + dy

def is_inside_maze(position):
    x, y = position
    return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT

def path_is_open(position, heading):
    """Blocked memory first, then grid boundary, then the wall sensor."""
    if (position, heading) in BLOCKED:
        log("check %s: remembered as BLOCKED (drive failed there before)"
            % DIRECTION_NAMES[heading])
        return False
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
    """Drive one cell. Returns the NEW position, or the SAME position
    if the drive was physically blocked (map not advanced, direction
    remembered as a wall). Light: GREEN into a new cell, RED into a
    cell we've already visited (v1.1 driver request)."""
    target = adjacent_cell(position, heading)
    if target in VISITED:
        set_status(255, 0, 0)            # RED: backtracking
    else:
        set_status(0, 255, 0)            # GREEN: new ground
    actual = drive_cell(heading)
    need = CELL_DISTANCE_CM * CELL_DISTANCE_SCALE
    if actual < need * CELL_OK_FRAC:
        log("*** drive BLOCKED at %.1f of %.1fcm — backing off, "
            "remembering %s from %s as a wall ***"
            % (actual, need, DIRECTION_NAMES[heading], position))
        BLOCKED.add((position, heading))
        # v1.2: back up the driven distance PLUS 5cm — backing only the
        # 0.4cm it managed left the nose still pinned on the wall and
        # the next turn couldn't rotate at all
        drive_back(actual + TURN_UNSTICK_CM, "unblock")
        return position                  # map NOT advanced
    return target

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
    VISITED.clear()
    BLOCKED.clear()
    VISITED.add(position)
    while position != GOAL:
        _abort()
        if time.ticks_diff(time.ticks_ms(), start_ms) \
                > MAX_RUNTIME_S * 1000:
            log("maze: time cap %.0fs reached at %s" %
                (MAX_RUNTIME_S, position))
            return False
        set_status(255, 200, 0)          # YELLOW: scanning at a cell
        ensure_turn_clearance()          # v1.1: room to rotate first
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
        VISITED.add(position)
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
    log("===== MAZE v1.2 launch: batt=%.2fV yaw=%+.1f"
        % (battery_voltage(), imu.get_yaw()))
    log("config: cell=%.1fcm drive=%.2f turn=%.2f tol=%.0fdeg "
        "wall<%.0fcm scale=%.2f/%.2f clear<%.0fcm cellok=%.0f%%"
        % (CELL_DISTANCE_CM, DRIVE_EFFORT, TURN_EFFORT, TURN_TOL_DEG,
           WALL_DISTANCE_CM, CELL_DISTANCE_SCALE, TURN_ANGLE_SCALE,
           TURN_CLEARANCE_CM, CELL_OK_FRAC * 100))
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
    log("=========== BOOT: maze_solver v1.2 (standalone) batt=%.2fV"
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
