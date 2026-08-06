"""
maze_solver.py — SYSEN 5920 Team 3
XRP Proving Ground: MAZE (right-wall follower) — v1.23

v1.23 (driver's field survey): GRID FRAME CORRECTED. The rover starts
in the bottom-left cell with THREE cells on its LEFT and FIVE ahead —
the code had the lateral cells on its RIGHT (START (0,0), maze to the
east). That mirror meant every WEST check in the start column was
skipped as EDGE-OF-MAZE when it physically held three open cells, and
every EAST check pinged the outer boundary wall. START is now (3, 0),
GOAL (3, 5): east = the outer edge, west = into the maze.

v1.22 (driver rule): RIGHT SIDE IS ALWAYS CHECKED FIRST. The goal-
biased revisit ordering (3rd+ visit to a cell) was re-sorting ALL
four directions by distance-to-goal, so a turned-around robot could
scan and take another direction without ever checking its right —
goal bias now reorders only fwd/left/back, with right pinned first.
Also: directions checked but not pinged now show in the sensor log —
"WEST EDGE-OF-MAZE" (outer boundary, nothing to ping) and "NORTH
KNOWN-BLOCKED" (remembered from a failed drive) — because a silently
skipped check looked like a missed one in the BLE view.

v1.21 (driver request): SENSOR LOG over Bluetooth. Every wall scan's
verdict — direction, distance in cm, WALL or OPEN — is pushed to the
PestoLink telemetry box (green = OPEN, red = WALL) and printed to the
serial terminal as a [SCAN] line. EVERYTHING ELSE now writes to
LOG.TXT only, so the live view shows nothing but the sensor verdicts.

v1.20: STOP BEFORE LOGGING + STALLED DRIVES NEVER COUNT AS ARRIVAL.
The 8/6 log caught a wedged drive lunging ~20cm DURING the "BLOCKED"
flash write (log latency was ~1.1s that run and the motors were still
boosted at 0.65 while the message was written): heartbeats swore
19.7cm, the summary said 39.5cm, and the corrupted encoder total was
scored as a clean arrival — every cell after that was fiction. All
terminal branches in drive_cell now STOP THE MOTORS FIRST and log
after. Belt-and-braces: any drive that ends by stall/timeout (rather
than by reaching distance or a confirmed wall) is scored BLOCKED no
matter what the encoders claim — grinding wheels lie. Blocked drives
also back up the full driven distance (cap 20cm, was 8) so the robot
returns to the cell it scanned from instead of loitering mid-corridor
where the next turn wedges.

v1.16: WALL-REFERENCED CELL DRIVES. Commanded 28 "went too far";
commanded 30.5 "went too far" identically, while the encoders swear
both were driven exactly — so the encoder SCALE is suspect (wheels
larger than the library's configured diameter under-report distance).
When any wall is visible inside the destination cell during a drive,
the drive now ends at the 6cm standoff from THAT WALL (two agreeing
pings required) — the maze itself terminates the move, encoder scale
irrelevant. Open corridors still use encoders: calibrate with the
D-pad-UP 100cm assist + tape measure, then set CELL_DISTANCE_SCALE =
100 / measured_cm.

v1.15 (driver: "shifting its location when it turns / distances are
off"): CELL-LOCAL POSE COMPENSATION. All the scan-phase shuffling
(align backups, unpress backups, turn-unstick reverse-arcs) moves the
robot away from where the cell scan started; a fixed 12" drive then
lands short or long. Every motion now records its encoder displacement
at the current yaw, and the cell drive distance is adjusted so the
robot ends EXACTLY one cell from the scan-start point — equivalent to
backing up to the scan-start first, but in a single motion with no
extra wall exposure.

v1.14 (driver): cell drive = EXACTLY 12 inches (30.48cm) before the
next wall scan, with an end-of-drive taper (last 6cm at crawl effort)
so the stop lands on the mark instead of coasting past it. The
wall-ahead standoff stop and the 70%-arrival rule are unchanged.

v1.13 (driver rule): a reading of 999+ that SURVIVES the 4cm escape
backup is scored as a WALL, not open — persistent no-echo means
something is pressed against or defeating the sensor.

v1.12: PRESSED-WALL DISAMBIGUATION. Every ran-into-the-wall event in
the 8/8 log follows the same signature: "ping 999 -> OPEN" then
"BLOCKED at 0.3cm" — the HC-SR04 is blind under ~3cm, so a wall the
nose is touching reads as NOTHING THERE. Any all-pings-timed-out
result now triggers a 4cm backup and re-read before being believed:
a pressed wall becomes visible (scored WALL, then aligned to the 6cm
turning standoff), true open costs only 4cm. This is the "keep
clearance from the walls" rule — enforced by making close walls
measurable again instead of trusting a blind reading.

v1.11: COAST TRIM — the "turns more than 90" mystery solved by the 8/8
log: every turn VERIFIED within 2 deg at motor-stop, then the yaw
jumped 8-19 deg in the next second — the robot was verifying mid-spin
and coasting past the target on momentum (snappier now on a healthy
battery). Turns now wait for rotation to physically STOP before every
verification, so the retry loop trims the coast and the robot settles
ON the cardinal, not past it.

v1.10: IMU DRIFT GUARD. The 8/7 night log shows yaw drifting ~25 deg/s
while the robot was PARKED (gyro bias calibrated at a power-on that
happened mid-handling, likely during the sensor repair) — every
absolute heading was garbage, turns went to wrong physical directions,
and the robot drove into walls it had "checked". Launch now measures
parked drift, attempts one imu recalibration, and FAILS LOUD (red 4x)
if the gyro is still bad — with a log message saying to power-cycle
the robot while it sits still.

v1.9: the constant-distance readings were a HARDWARE fault on the
rangefinder (now fixed) — all the blind-sensor/self-echo detection
code built on that assumption is REMOVED. The sensor is trusted
again. Kept: fail-loud (red 4x on any autonomous failure), three-
point reverse-arc turns, wall alignment, goal-biased revisits.

v1.8 — TURNING RADIUS: turns wedge because the chassis corners sweep
~11cm about the axle while sitting 6cm from a wall; a straight backup
only buys nose clearance, not swing room. Now:
  * WEDGED TURNS recover with a REVERSE-ARC (three-point-turn style):
    reverse while steering toward the target — the rotation center
    moves behind the robot, the nose swings back-and-away from the
    wall, and rotation progresses during the escape.
  * Near-180 (dead-end) turns are done as a deliberate three-point
    turn: half the rotation, reverse-arc, finish verified.

v1.7: FAIL LOUD — any autonomous failure (boxed in, time cap)
flashes RED 4x and returns to the menu. Goal-biased direction
ordering from the 3rd visit to a cell. (v1.7's self-echo detection
was removed in v1.9 — the root cause was a sensor hardware fault.)

v1.6 (8/7 night logs — "didn't drive toward the open space"): the v1.4
paddle stow angle of 10 deg parked the paddle IN FRONT OF THE
RANGEFINDER — every wall check in every direction read a constant
10.6cm (the paddle), so every open corridor was scored WALL and the
solver declared BOXED IN. Stow is now 90 deg (confirmed clear), and
three near-identical short pings in different directions now trigger a
loud "rangefinder is staring at the paddle" log warning.

v1.5 (8/7 logs — "drives too far before scanning" / "backs up too much
and hits the wall behind"): sized for the REAL cells, which measure
11-13 INCHES (28-33cm):
  * cell drive command 30.5 -> 28cm (short end of the range; the
    standoff stop ends the drive at a wall, and the wall alignment
    absorbs undershoot at the next faced wall). Replaces the
    field-edited 0.9 scale.
  * All recovery moves shrunk for small cells: standoff 8 -> 6cm,
    align cap 10 -> 6cm, turn-unstick 5 -> 3cm, clearance backup
    6 -> 4cm, unblock backup capped at 8cm. A backup in a small cell
    was reaching the wall BEHIND and wedging the tail.

v1.4 (8/7 early-AM logs) — the wall-stick + bearing findings:
  * BACKUP SIGN FIX: the heading hold in drive_back steered the WRONG
    way in reverse — every backup rotated the robot ~20 deg (visible
    in the logs as yaw +103 -> +83 across one 10cm backup). That
    rotation is where the lost bearing came from. Fixed; tower had the
    same bug in its reverse legs.
  * WALL ALIGNMENT: every wall the robot FACES is now a position
    reference — at each cell stop and on every scan check that sees a
    wall closer than the 8cm standoff, it backs to the standoff. The
    maze itself re-centers the robot on both axes, which restores turn
    clearance AND absorbs encoder drift.
  * STANDOFF STOP: cell drives ping ahead ~6Hz and stop at the 8cm
    standoff instead of ramming the far wall (the 70%-arrival rule
    still counts the cell).
  * WEDGED-TURN RECOVERY widened: ANY timed-out turn attempt short of
    target backs up 5cm and retries (the old rule only fired when the
    turn barely moved; the logs show a turn jamming 34 deg short).
  * PADDLE STOW at launch: manual mode left the paddle drooped (71
    deg) before maze runs — a snag the ultrasonic cannot see. Prime
    suspect for drives blocking at 2-5cm after a CLEAR ping.
  * Heading hold on cell drives doubled (KP 0.02 -> 0.04, clamp 0.2).

v1.3 (8/6 night logs — "turning still shooting past 90"): SLOW-ZONE
TURNS. Full effort only while >25 deg from the target; inside that,
effort drops to ~60% so momentum cannot carry the robot past the mark
between two 10ms IMU checks. Tolerance tightened 4 -> 2.5 deg to match
the promised +-2 deg placement squareness, and every turn still
re-verifies against its ABSOLUTE cardinal IMU target (launch yaw =
grid north) with up to 4 escalating retries — so a residual error is
trimmed out, never accumulated.

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
import math
import sys

# WHEEL GEOMETRY FIX (8/8): this robot runs ~3.0-3.1 inch (7.75cm)
# wheels; XRPLib's default drivetrain assumes the stock 6.0cm wheels,
# so every encoder distance was under-reported ~29percent — the
# "drives too far" mystery, the 21.5cm "ring radius" (really ~28cm),
# all of it. Correct the library's wheel diameter so every distance
# in every program is real centimeters.
WHEEL_DIAM_CM = 7.75
try:
    if hasattr(drivetrain, "wheel_diam"):
        drivetrain.wheel_diam = WHEEL_DIAM_CM
        _WHEEL_FIX = "wheel_diam corrected to %.2fcm" % WHEEL_DIAM_CM
    else:
        _WHEEL_FIX = ("*** drivetrain has no wheel_diam attribute — "
                      "XRPLib version differs, distances still ~29%% "
                      "over! Report this. ***")
except Exception as _e:
    _WHEEL_FIX = "*** wheel fix FAILED: %r ***" % _e


_HOOKS = {"abort": lambda: None, "pesto": None}

def _abort():
    _HOOKS["abort"]()

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # standalone only
START_BUTTON = 3            # standalone only. Y is button 3 on OUR pad
                            # (confirmed by the button-spy logs, 8/6) —
                            # the draft's "Y = 2" is the X button here.

# Maze geometry. Coordinates are (column, row).
# v1.23 (driver's field survey): the rover starts in the BOTTOM-LEFT
# cell of the maze with THREE cells on its LEFT side and FIVE cells
# ahead. Internally "north" = the rover's forward at launch and
# "east" (+x) = the rover's RIGHT — so the start column is x=3 (the
# right/east edge of the grid) and the three lateral cells are x=2,
# 1, 0 to the WEST. The old START=(0,0) had this mirrored: WEST was
# skipped as EDGE-OF-MAZE when it physically held three open cells.
GRID_WIDTH = 4
GRID_HEIGHT = 6
START = (3, 0)
GOAL = (3, 5)
START_HEADING = 0           # 0 = north (robot placed facing the goal end)

# Motion and sensor tuning.
CELL_DISTANCE_CM = 30.48   # v1.14 (driver): EXACTLY 12 inches per
                            # cell before the next wall scan. The
                            # standoff stop still ends the drive early
                            # if a wall is closer, and the end-taper
                            # below stops it crisply on the mark.
DRIVE_EFFORT = 0.5          # cell drives (the draft's 0.45 is below the
                            # stiction floor once the PID tapers)
MIN_EFFORT = 0.4            # per-wheel stiction floor (proven value)
WALL_DISTANCE_CM = 12.7     # v1.18 (driver rule): WALL only if within
                            # 5 INCHES of the sensor. With corrected
                            # odometry a true adjacent wall reads
                            # <=13cm from the standoff; 20 was catching
                            # the NEXT cell's wall and looping the
                            # solver on phantom blocks. A missed wall
                            # is safe: the drive stops at the 6cm
                            # standoff, covers <70%, and is marked
                            # blocked by the drive itself.
SETTLE_TIME_S = 0.20
SENSOR_SAMPLES = 5
MOTION_TIMEOUT_S = 5.0

# Trim knobs if testing shows systematic over/under travel.
CELL_DISTANCE_SCALE = 1.0   # v1.5: back to 1.0 — the 11" base cell
                            # replaces the field-edited 0.9 scale
TURN_ANGLE_SCALE = 1.0      # applied to the 90-deg cardinal spacing

# Wall recovery (v1.1)
TURN_CLEARANCE_CM = 2.0     # front ping under this before scanning ->
                            # back up first so the turn has room
CLEAR_BACKUP_CM = 4.0       # v1.5: 6 -> 4 (small cells: big backups
                            # hit the wall BEHIND)
REVERSE_EFFORT = 0.7        # reverse needs more effort (proven)

# WALL ALIGNMENT (v1.4): the maze itself is the position reference.
# Whenever the robot is FACING a wall (start of a cell stop, or any
# scan check that sees one), it backs away to TARGET_FRONT_CM. Every
# such correction re-centers the robot along that axis, which (a)
# guarantees swing room for the next turn and (b) cancels the encoder
# drift that was walking it into walls.
ALIGN_WALL_CM = 13.0        # a ping under this = wall close enough to
                            # use as a reference
TARGET_FRONT_CM = 6.0       # v1.5: 8 -> 6 — desired nose-to-wall
                            # standoff; 8 was over-backing in the
                            # 28-33cm cells and jamming the tail
ALIGN_MAX_BACK_CM = 6.0     # v1.5: 10 -> 6 per correction
UNPRESS_BACK_CM = 4.0       # v1.12: all pings timing out can mean the
                            # nose is PRESSED on a wall (HC-SR04 is
                            # blind under ~3cm) — back this far and
                            # re-check before believing "open"
PADDLE_STOW_DEG = 90        # v1.6: STOW AT MID — the 8/7 logs show
                            # that at 10 deg the paddle sits IN FRONT
                            # OF THE RANGEFINDER: every wall check in
                            # every direction read exactly 10.6cm and
                            # the solver declared BOXED IN without
                            # driving anywhere. 90 deg is confirmed
                            # clear (tower ranged 71.6cm fine at 90).
CELL_OK_FRAC = 0.7          # a drive covering >= this fraction of a
                            # cell counts as ARRIVED; less = blocked,
                            # back to center, remember the wall
DRIVE_STALL_S = 1.0         # no progress this long -> boost effort
DRIVE_STALL_ADD = 0.15
DRIVE_STALL_MAX = 0.8

# Turns — the proven wiggle scheme (sumo v4.5+ / main_code v1.14+).
# v1.3 (driver: "still shooting past 90"): a SLOW ZONE — full effort
# only while far from the target; inside TURN_SLOW_ZONE_DEG the effort
# drops so momentum can't carry the robot past the mark between two
# 10ms checks. Tolerances tightened to match the promised +-2 degree
# placement accuracy.
TURN_EFFORT = 0.75
TURN_SLOW_ZONE_DEG = 25.0   # within this of the target -> slow down
TURN_SLOW_FACTOR = 0.6      # slow-zone effort multiplier
TURN_TOL_DEG = 2.5          # v1.3: 4.0 -> 2.5 (cells are square to
                            # +-2 deg; turns should be too)
TURN_RETRIES = 4            # v1.2: one more attempt (unstick eats one)
TURN_STUCK_DEG = 8.0        # attempt moved less than this with lots
                            # left to go = physically pinned -> back up
TURN_UNSTICK_CM = 3.0       # v1.5: 5 -> 3 (5 was reaching the wall
                            # behind in the small cells)
WIGGLE_BIAS = 0.18
WIGGLE_PERIOD_S = 0.22
WIGGLE_TOL_DEG = 2.0        # v1.3: 3.0 -> 2.0
LEFT_TURN_BOOST = 1.2       # this robot turns LEFT (CCW) weaker

# Heading hold during cell drives — v1.4: doubled. The logged drives
# carried a persistent 5-10 deg error that 0.02 was too weak to pull
# out; that sideways creep is what walked the robot into walls.
FWD_KP = 0.04
FWD_CORR_MAX = 0.20

MAX_RUNTIME_S = 240

# IMU drift guard (v1.10): the gyro bias is calibrated at POWER-ON —
# if the robot was being handled when it booted, yaw drifts constantly
# (the 8/7 log shows ~25 deg/s while PARKED) and every absolute
# heading is garbage. Launch now measures the parked drift, tries one
# recalibration, and fails loud rather than driving on a spinning
# compass.
DRIFT_CHECK_S = 0.8
DRIFT_MAX_DPS = 3.0

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

def log(msg, console=False):
    # v1.21 (driver request): console default OFF — the terminal and
    # the BLE telemetry show ONLY the [SCAN] sensor lines (scan_log
    # below); the full story still lands in LOG.TXT
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

def _scan_emit(short, color):
    """Push one SENSOR LOG line to serial + BLE telemetry + LOG.TXT."""
    print("[SCAN] " + short)
    p = _HOOKS.get("pesto")
    if p is not None:
        try:
            if p.is_connected():
                p.telemetryPrint(short, color)
        except Exception:
            pass                 # telemetry must never kill a scan
    log("SCAN %s" % short)

def scan_log(direction, dist_cm, is_open):
    """v1.21 (driver request): the SENSOR LOG — the ONLY live output.
    Pushed to the PestoLink telemetry box over Bluetooth (green =
    OPEN, red = WALL), printed to the serial terminal, and written to
    LOG.TXT so the unified timeline stays complete."""
    _scan_emit("%s %.1fcm %s" % (direction.upper(), dist_cm,
                                 "OPEN" if is_open else "WALL"),
               "00FF00" if is_open else "FF0000")

def scan_skip(direction, reason):
    """v1.22: a direction that was CHECKED but not PINGED (maze edge,
    or remembered-blocked) still shows in the sensor log — an absent
    line looked like a missed check (the 8/6 screenshot: WEST at
    column 0 is the outer edge, so it was skipped silently)."""
    _scan_emit("%s %s" % (direction.upper(), reason), "FFC800")

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
            mag = min(0.95, effort * (LEFT_TURN_BOOST if err > 0 else 1.0))
            # v1.3 SLOW ZONE: close to the target, drop the effort so
            # momentum can't fling the robot past 90 between checks
            if abs(err) <= TURN_SLOW_ZONE_DEG:
                mag = max(MIN_EFFORT + 0.03, mag * TURN_SLOW_FACTOR)
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

# v1.15: CELL-LOCAL POSE. All the scan-phase shuffling (align backups,
# unpress backups, turn-unstick arcs) shifts the robot away from where
# the cell scan started — then a fixed 12" drive lands in the wrong
# spot ("the distances are off"). Every motion primitive now records
# its encoder-measured displacement at the current yaw; the cell drive
# is then adjusted so it ends one cell from the SCAN-START point, not
# one cell from wherever the shuffling left the robot.
CPOSE = {"x": 0.0, "y": 0.0}
SCAN_START = {"x": 0.0, "y": 0.0}

def _cpose_move(dist_cm):
    rad = math.radians(imu.get_yaw())
    CPOSE["x"] += dist_cm * math.cos(rad)
    CPOSE["y"] += dist_cm * math.sin(rad)

def reverse_arc(toward_ccw, dist_cm=4.0):
    """v1.8 three-point-turn segment: reverse while steering toward the
    target rotation. Backing moves the rotation center behind the
    robot, so the nose swings BACK AND AWAY from the wall it was
    grinding on — clearance and rotation in the same move. toward_ccw
    True = gain CCW (left) while backing."""
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    # rotation follows (right - left): CCW while reversing needs the
    # LEFT wheel faster-negative
    if toward_ccw:
        l_eff, r_eff = -0.75, -0.45
    else:
        l_eff, r_eff = -0.45, -0.75
    t0 = time.ticks_ms()
    try:
        while True:
            _abort()
            if -_traveled(start_l, start_r) >= dist_cm:
                break
            if time.ticks_diff(time.ticks_ms(), t0) > 1500:
                log("arc: stalled (wall behind?) — continuing")
                break
            drivetrain.set_effort(l_eff, r_eff)
            time.sleep(0.01)
    finally:
        drivetrain.stop()
    _cpose_move(_traveled(start_l, start_r))     # v1.15 (negative move)
    time.sleep(SETTLE_TIME_S)

def wait_rotation_stop(timeout_s=1.2):
    """v1.11: block until the yaw stops changing. The 8/8 log shows
    turns COASTING 8-19 deg after the motors cut (verification passed
    at motor-stop, the robot kept spinning on momentum) — that coast
    is the physical over-rotation the driver sees. Returns the total
    coast in degrees."""
    y_start = imu.get_yaw()
    last = y_start
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_s * 1000:
        _abort()
        time.sleep(0.08)
        y = imu.get_yaw()
        if abs(y - last) < 0.4:          # <5 deg/s = stopped
            break
        last = y
    return imu.get_yaw() - y_start

def turn_to_heading_idx(heading, why):
    """Turn to a cardinal direction's ABSOLUTE yaw target, verify with
    the IMU, retry with escalating effort. v1.2: an attempt that barely
    rotates means the robot is pinned on a wall — back up ~5cm and
    retry the same cardinal target (alignment is re-aimed, never
    accumulated). Logs commanded vs achieved."""
    target = heading_yaw(heading)
    before = imu.get_yaw()
    first = normalize(target - before)
    if abs(first) > 135:
        # v1.8: a near-180 in a tight cell needs the most swing room —
        # do it as a deliberate three-point turn: half the rotation,
        # reverse-arc, then finish via the normal verified loop.
        wiggle_turn(first * 0.5, TURN_EFFORT, timeout_s=3)
        rem = normalize(target - imu.get_yaw())
        reverse_arc(rem > 0, 3.0)
    for attempt in range(TURN_RETRIES):
        _abort()
        # v1.11: never evaluate while still spinning — wait for the
        # coast to finish, then measure. The retry loop trims whatever
        # the coast added, so the PHYSICAL heading converges on the
        # cardinal target instead of ending 8-19 deg past it.
        coast = wait_rotation_stop()
        if abs(coast) > 3:
            log("turn: coasted %+.0f deg after motor stop — trimming"
                % coast, console=False)
        delta = normalize(target - imu.get_yaw())
        if abs(delta) <= TURN_TOL_DEG:
            break
        boosted = min(0.9, TURN_EFFORT + 0.15 * attempt)
        yaw_a = imu.get_yaw()
        ok = wiggle_turn(delta, boosted, timeout_s=3)
        moved = abs(imu.get_yaw() - yaw_a)
        if not ok and \
                abs(normalize(target - imu.get_yaw())) > TURN_TOL_DEG:
            # v1.8: wedged mid-rotation — the chassis corner is riding
            # a wall. A straight backup only buys nose clearance; a
            # REVERSE-ARC toward the target (three-point-turn style)
            # swings the nose back-and-away AND gains rotation at the
            # same time.
            remaining = normalize(target - imu.get_yaw())
            log("turn: WEDGED (moved %.0f of %.0f deg) — reverse-arc "
                "%s and retrying" % (moved, abs(delta),
                                     "CCW" if remaining > 0 else "CW"))
            reverse_arc(remaining > 0, TURN_UNSTICK_CM)
    wait_rotation_stop()                 # v1.11: final reading at rest
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

LAST_WALL_STOP = {"v": False}   # v1.19: True when the last cell drive
                                # ended ON A CONFIRMED WALL (standoff
                                # stop or verified truncation) rather
                                # than by stall/timeout
LAST_STALL = {"v": False}       # v1.20: True when the last cell drive
                                # ended by no-progress stall or motion
                                # timeout — its encoder total is NOT
                                # trustworthy (wedge-pop lunges, wheel
                                # grind) and must never score arrival

def drive_cell(heading, dist_cm=None):
    """Drive one cell forward holding the cardinal yaw. Logs commanded
    vs encoder distance. Returns the actual distance driven."""
    dist = dist_cm if dist_cm is not None \
        else CELL_DISTANCE_CM * CELL_DISTANCE_SCALE
    hold = heading_yaw(heading)
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    t0 = time.ticks_ms()
    last_beat = t0
    prog_ms = t0
    prog_cm = 0.0
    boosted = False
    last_ping = t0
    wall_hits = 0
    ref_last = -99.0
    orig_dist = dist
    truncated = False
    resumed = False
    LAST_WALL_STOP["v"] = False
    LAST_STALL["v"] = False
    try:
        while True:
            _abort()
            trav = _traveled(start_l, start_r)
            if trav >= dist:
                # v1.18: a truncated (wall-referenced) end gets ONE
                # verification before it can cost the map a cell — a
                # glancing side-wall echo in the 8/8 log truncated a
                # good drive at 13.7cm and the 70% rule then marked an
                # OPEN corridor as permanently blocked (the loop).
                if truncated and not resumed and orig_dist - trav > 5.0:
                    drivetrain.stop()
                    time.sleep(0.25)
                    vd = average_distance()
                    if vd > WALL_DISTANCE_CM:
                        log("drive: truncation wall NOT confirmed "
                            "(re-read %.1fcm) — phantom echo, resuming "
                            "to %.1fcm" % (vd, orig_dist))
                        dist = orig_dist
                        truncated = False
                        resumed = True
                        ref_last = -99.0
                        wall_hits = 0
                        continue
                    log("drive: wall confirmed at %.1fcm — stopping"
                        % vd)
                    LAST_WALL_STOP["v"] = True
                break
            now = time.ticks_ms()
            if time.ticks_diff(now, t0) > MOTION_TIMEOUT_S * 1000:
                drivetrain.stop()            # v1.20: stop BEFORE logging
                LAST_STALL["v"] = True
                log("drive: TIMEOUT at %.1f of %.1fcm (stall?)"
                    % (trav, dist))
                break
            # v1.4: watch for the wall ahead (~6Hz single pings) and
            # STOP at the standoff distance instead of ramming it —
            # the 70%-arrival rule still counts the cell as reached
            if time.ticks_diff(now, last_ping) >= 150:
                last_ping = now
                pd = rangefinder.distance()
                if 0 < pd < TARGET_FRONT_CM:
                    wall_hits += 1
                    if wall_hits >= 2:
                        drivetrain.stop()    # v1.20: stop BEFORE logging
                        LAST_WALL_STOP["v"] = True
                        log("drive: wall ahead at %.1fcm after %.1fcm "
                            "— stopping at standoff" % (pd, trav))
                        break
                else:
                    wall_hits = 0
                # v1.16: WALL-REFERENCED CELL END — if a wall is
                # visible inside the destination cell, end the drive
                # at the 6cm standoff from THAT WALL instead of at the
                # encoder count (immune to wheel-size/encoder scale
                # error, the prime suspect for "still goes too far").
                # Needs two consecutive agreeing pings so one noise
                # spike can't truncate the drive.
                if 3.0 < pd < 20.0:
                    # v1.18: window 45 -> 20cm — long-range referencing
                    # was a crutch for the (now fixed) encoder scale
                    # error, and glancing side-wall echoes at 15-25cm
                    # were truncating good drives
                    if abs(pd - ref_last) < 3.0:
                        cand = trav + pd - TARGET_FRONT_CM
                        if cand < dist - 1.0:
                            dist = max(trav, cand)
                            truncated = True
                            log("drive: wall ahead at %.1fcm — cell "
                                "end re-referenced to %.1fcm"
                                % (pd, dist))
                    ref_last = pd
                else:
                    ref_last = -99.0
            # v1.1 stall boost: battery sag froze drives mid-cell
            if trav - prog_cm >= 0.5:
                prog_cm = trav
                prog_ms = now
                boosted = False
            stalled_ms = time.ticks_diff(now, prog_ms)
            if stalled_ms > 2500:
                # v1.2: 2.5s of zero progress = wall; stop grinding
                drivetrain.stop()            # v1.20: stop BEFORE logging
                LAST_STALL["v"] = True
                log("drive: BLOCKED (no progress %.1fs at %.1fcm)"
                    % (stalled_ms / 1000, trav))
                break
            eff = DRIVE_EFFORT
            # v1.14: crawl the last few cm so the stop lands ON the
            # 12-inch mark instead of coasting past it
            if dist - trav < 6.0:
                eff = MIN_EFFORT
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
    _cpose_move(actual)                          # v1.15
    log("drive %s: %.1fcm of %.1fcm, end yaw %+.1f (target %+.1f)"
        % (DIRECTION_NAMES[heading], actual, dist, imu.get_yaw(),
           heading_yaw(heading)))
    time.sleep(SETTLE_TIME_S)
    return actual

def flash_red4():
    """v1.7 (driver request): four red flashes = the maze failed
    autonomously; robot is parked and returning to the menu."""
    for _ in range(4):
        set_status(255, 0, 0)
        time.sleep(0.18)
        set_status(0, 0, 0)
        time.sleep(0.18)

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
            # v1.4 SIGN FIX: rotation always follows (right - left)
            # velocity, so the correction is l=-corr / r=+corr in BOTH
            # directions. The old reversed signs actively steered the
            # WRONG way while backing — the logs show every backup
            # rotating the robot ~20 deg off its heading.
            drivetrain.set_effort(-REVERSE_EFFORT - corr,
                                  -REVERSE_EFFORT + corr)
            time.sleep(0.01)
    finally:
        drivetrain.stop()
    actual = -_traveled(start_l, start_r)
    _cpose_move(-actual)                         # v1.15
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

def align_off_front_wall(where):
    """v1.4: if a wall is close ahead, back away to TARGET_FRONT_CM —
    the wall becomes a position reference. Called at every cell stop
    and on every scan check that sees a wall, so the maze structure
    continuously re-centers the robot on both axes."""
    d = min_distance()
    if d < TARGET_FRONT_CM and d >= 0.0:
        back = min(TARGET_FRONT_CM - d, ALIGN_MAX_BACK_CM)
        if back > 1.0:
            log("align(%s): wall at %.1fcm — backing %.1fcm to %.0fcm"
                % (where, d, back, TARGET_FRONT_CM))
            drive_back(back, "align")
            return True
    return False

# ------------------------------- THE MAZE --------------------------------

# v1.1 run-state: cells already visited (for the status light) and
# (cell, direction) pairs that physically failed (virtual walls).
VISITED = set()
BLOCKED = set()
ENTRY = {}                  # v1.7: times each cell has started a scan —
                            # revisits switch to goal-biased ordering so
                            # an open area can't trap the right-wall
                            # rule in an endless clockwise loop

def adjacent_cell(position, heading):
    dx, dy = DIRECTIONS[heading]
    return position[0] + dx, position[1] + dy

def is_inside_maze(position):
    x, y = position
    return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT

def resolve_ping(where):
    """v1.12: a 999 (all pings timed out) is ambiguous — genuinely
    open, or nose PRESSED on a wall (the sensor is blind under ~3cm;
    every ran-into-the-wall event in the 8/8 log follows a 999-OPEN).
    Disambiguate by backing UNPRESS_BACK_CM and re-reading: a pressed
    wall becomes visible at ~4-7cm, true open stays far."""
    d = average_distance()
    if d < 990:
        return d
    log("%s: all pings timed out — may be pressed on a wall; backing "
        "%.0fcm to re-check" % (where, UNPRESS_BACK_CM))
    drive_back(UNPRESS_BACK_CM, "unpress")
    d = average_distance()
    if d >= 990:
        # v1.13 (driver rule): STILL no echo after backing away =
        # assume something is pressed against / defeating the sensor.
        # Score it as a WALL rather than driving into the unknown.
        log("%s: STILL no echo after backing — treating as WALL"
            % where)
        return 0.0
    log("%s: re-check reads %.1fcm" % (where, d))
    return d

def path_is_open(position, heading):
    """Blocked memory first, then grid boundary, then the wall sensor."""
    if (position, heading) in BLOCKED:
        log("check %s: remembered as BLOCKED (drive failed there before)"
            % DIRECTION_NAMES[heading])
        scan_skip(DIRECTION_NAMES[heading], "KNOWN-BLOCKED")  # v1.22
        return False
    next_position = adjacent_cell(position, heading)
    if not is_inside_maze(next_position):
        log("check %s: grid boundary (%s is outside)"
            % (DIRECTION_NAMES[heading], next_position))
        scan_skip(DIRECTION_NAMES[heading], "EDGE-OF-MAZE")   # v1.22
        return False
    d = resolve_ping("check " + DIRECTION_NAMES[heading])
    open_ = d > WALL_DISTANCE_CM
    scan_log(DIRECTION_NAMES[heading], d, open_)     # v1.21: BLE-visible
    if not open_:
        # v1.4: a wall we're facing is a free position reference — if
        # it's too close, back off to the target standoff. Over a scan
        # cycle this re-centers the robot on BOTH axes.
        align_off_front_wall("check " + DIRECTION_NAMES[heading])
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
    # v1.15: drive so the cell ends one cell from where the SCAN
    # started — cancels the shuffling (aligns/unpress/unstick backups)
    # that shifted the robot during the scan
    hy = math.radians(heading_yaw(heading))
    ux, uy = math.cos(hy), math.sin(hy)
    along = (CPOSE["x"] - SCAN_START["x"]) * ux + \
            (CPOSE["y"] - SCAN_START["y"]) * uy
    need = CELL_DISTANCE_CM * CELL_DISTANCE_SCALE - along
    need = max(10.0, min(40.0, need))
    if abs(along) > 1.0:
        log("scan drift: %+.1fcm along the drive axis — cell drive "
            "adjusted to %.1fcm" % (along, need))
    actual = drive_cell(heading, need)
    if not LAST_STALL["v"] and actual < need * CELL_OK_FRAC \
            and LAST_WALL_STOP["v"] and actual >= need * 0.4:
        # v1.19 (driver rule): only SCANS declare walls. A drive that
        # ended parked at a CONFIRMED wall's standoff, having covered
        # 40%+ of a cell, has crossed into the next cell — count the
        # arrival; the next scan will see that wall at ~5cm and score
        # it under the 5-inch rule. (The 8/8 loop: a 9-inch-away wall
        # ended the drive at 54% and the old rule walled off an OPEN
        # corridor.)
        log("drive ended on a confirmed wall at %.1fcm (%.0f%% of a "
            "cell) — counting arrival at the wall standoff"
            % (actual, 100.0 * actual / need))
        return target
    if LAST_STALL["v"] or actual < need * CELL_OK_FRAC:
        # v1.20: a stall/timeout-ended drive is scored BLOCKED no
        # matter what the encoders total — grinding/wedged wheels lie
        # (8/6 log: heartbeats 19.7cm, summary 39.5cm, scored as a
        # clean arrival, map corrupted from there on)
        why = "STALLED" if LAST_STALL["v"] else "BLOCKED"
        log("*** drive %s at %.1f of %.1fcm — backing off, "
            "remembering %s from %s as blocked ***"
            % (why, actual, need, DIRECTION_NAMES[heading], position))
        BLOCKED.add((position, heading))
        # v1.20: back up the FULL driven distance (cap 20cm) so the
        # robot returns to the cell it scanned from — the old 8cm cap
        # left it loitering mid-corridor, where the next turn wedged
        drive_back(min(actual + TURN_UNSTICK_CM, 20.0), "unblock")
        return position                  # map NOT advanced
    return target

def imu_drift_dps():
    """Yaw change per second while the robot is PARKED."""
    y0 = imu.get_yaw()
    time.sleep(DRIFT_CHECK_S)
    return (imu.get_yaw() - y0) / DRIFT_CHECK_S

def imu_ok_or_fail():
    """v1.10: verify the gyro is stable; one recalibration attempt,
    then fail loud. True = safe to trust absolute headings."""
    rate = imu_drift_dps()
    if abs(rate) <= DRIFT_MAX_DPS:
        return True
    log("*** IMU DRIFTING %+.1f deg/s while parked — gyro bias is bad "
        "(robot was moving at power-on?). Recalibrating... ***" % rate)
    try:
        imu.calibrate(1)
    except Exception as e:
        log("imu.calibrate unavailable (%r)" % e)
    time.sleep(0.3)
    rate = imu_drift_dps()
    if abs(rate) <= DRIFT_MAX_DPS:
        log("IMU recalibrated — drift now %+.1f deg/s" % rate)
        return True
    log("*** IMU STILL drifting %+.1f deg/s. POWER-CYCLE the robot "
        "while it sits STILL, then relaunch. ***" % rate)
    return False

def solve_maze():
    """Right-wall follower: try right, then straight, then left, then
    back. Every state change is logged."""
    position = START
    heading = START_HEADING
    if not imu_ok_or_fail():
        return False                      # run() flashes red 4x
    CARDINAL["yaw0"] = imu.get_yaw()     # placed facing NORTH = launch yaw
    log("maze: start %s facing %s (yaw0 %+.1f) grid %dx%d goal %s"
        % (position, DIRECTION_NAMES[heading], CARDINAL["yaw0"],
           GRID_WIDTH, GRID_HEIGHT, GOAL))
    start_ms = time.ticks_ms()
    steps = 0
    time.sleep(0.5)
    VISITED.clear()
    BLOCKED.clear()
    ENTRY.clear()
    VISITED.add(position)
    while position != GOAL:
        _abort()
        if time.ticks_diff(time.ticks_ms(), start_ms) \
                > MAX_RUNTIME_S * 1000:
            log("maze: time cap %.0fs reached at %s" %
                (MAX_RUNTIME_S, position))
            return False
        set_status(255, 200, 0)          # YELLOW: scanning at a cell
        SCAN_START["x"] = CPOSE["x"]     # v1.15: remember where this
        SCAN_START["y"] = CPOSE["y"]     # cell's scan began
        # v1.12: cell-stop clearance check that also handles the
        # pressed-on-wall case (999 -> back 4cm and re-read), then
        # aligns to the 6cm turning standoff
        d0 = resolve_ping("cell stop")
        if d0 < TARGET_FRONT_CM:
            back = min(TARGET_FRONT_CM - d0, ALIGN_MAX_BACK_CM)
            if back > 1.0:
                log("align(cell stop): wall at %.1fcm — backing %.1fcm "
                    "to %.0fcm" % (d0, back, TARGET_FRONT_CM))
                drive_back(back, "align")
        ENTRY[position] = ENTRY.get(position, 0) + 1
        # Direction preference: right-wall rule normally; from the 3rd
        # visit to the same cell, order by progress toward the GOAL so
        # an open area can't trap the right-hand rule in a loop.
        order = [(heading + 1) % 4, heading,
                 (heading - 1) % 4, (heading + 2) % 4]
        names = ["try right", "go fwd", "try left", "dead end"]
        if ENTRY[position] >= 3:
            # v1.22 (driver rule): the RIGHT side is ALWAYS checked
            # first, even on revisits — goal bias only reorders the
            # remaining three (fwd/left/back). The old full reorder
            # was why a turned-around robot sometimes never scanned
            # its right side: goal distance outranked the wall rule.
            gx, gy = GOAL
            rest = sorted(zip(order[1:], names[1:]), key=lambda hn: abs(
                gx - adjacent_cell(position, hn[0])[0]) + abs(
                gy - adjacent_cell(position, hn[0])[1]))
            order = order[:1] + [h for h, _ in rest]
            names = names[:1] + [n for _, n in rest]
            log("revisit x%d of %s — right first, rest goal-biased"
                % (ENTRY[position], (position,)))
        moved = False
        for h, why in zip(order, names):
            turn_to_heading_idx(h, why)
            heading = h
            if path_is_open(position, heading):
                position = move_one_cell(position, heading)
                moved = True
                break
        if not moved:
            log("*** BOXED IN at %s — no open path in any "
                "direction ***" % (position,))
            set_status(255, 0, 0)
            return False
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
        _HOOKS["pesto"] = getattr(sv, "pesto", None)   # v1.21: BLE scan log
    _rotate_log()
    log("wheels: %s" % _WHEEL_FIX)
    log("===== MAZE v1.23 launch: batt=%.2fV yaw=%+.1f"
        % (battery_voltage(), imu.get_yaw()))
    log("config: cell=%.1fcm drive=%.2f turn=%.2f tol=%.1fdeg "
        "wall<%.0fcm scale=%.2f/%.2f clear<%.0fcm cellok=%.0f%% "
        "standoff=%.0fcm"
        % (CELL_DISTANCE_CM, DRIVE_EFFORT, TURN_EFFORT, TURN_TOL_DEG,
           WALL_DISTANCE_CM, CELL_DISTANCE_SCALE, TURN_ANGLE_SCALE,
           TURN_CLEARANCE_CM, CELL_OK_FRAC * 100, TARGET_FRONT_CM))
    if battery_voltage() < LOW_BATT_V:
        log("*** WARNING: battery LOW at launch (%.2fV) ***"
            % battery_voltage())
    # v1.4: STOW THE PADDLE fully back before entering the maze. The
    # logs show drives blocking at 2-5cm right after a CLEAR ping —
    # something below the ultrasonic beam was snagging, and a paddle
    # left drooped by manual mode (settled at 71/65/10 deg in the same
    # session) is the prime suspect.
    try:
        servo_one.set_angle(PADDLE_STOW_DEG)
        time.sleep(0.4)
        servo_one.free()
        log("paddle stowed at %d deg" % PADDLE_STOW_DEG)
    except Exception:
        log("paddle stow skipped (no servo?)")
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
            log("DONE: batt=%.2fV" % battery_voltage())
        if ok:
            set_status(0, 255, 0)
        else:
            log("maze: FAILED — flashing red 4x, back to menu")
            flash_red4()
        return ok
    finally:
        _HOOKS["abort"] = lambda: None

# ------------------------- STANDALONE OPERATION --------------------------

def _standalone():
    from pestolink import PestoLinkAgent
    pestolink = PestoLinkAgent(ROBOT_NAME)
    _HOOKS["pesto"] = pestolink                        # v1.21: BLE scan log
    log("=========== BOOT: maze_solver v1.16 (standalone) batt=%.2fV"
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
