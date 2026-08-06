"""
sumo_auto.py — SYSEN 5920 Team 3
XRP Proving Ground: "Sumo" challenge (autonomous attempt) — v4.22

v4.22: turns verify only after rotation physically stops (momentum
was coasting 8-19 deg past the target after the motors cut — see
maze v1.11). Keeps the survey 180 and go_home bearings honest.

v4.21: IMU DRIFT GUARD at launch (parked drift >3 deg/s -> one
recalibration attempt -> red abort with "power-cycle while STILL").
A bad power-on gyro bias makes the pose/fence map garbage.

v4.20: the constant-distance echoes were a rangefinder HARDWARE
fault (now fixed) — the v4.17 self-echo rejection built on that
assumption is REMOVED; the sensor is trusted again.

v4.19: scan margin widened (+3cm past the surveyed line, was -1) —
blocks sitting ON the ring line were projecting just outside the
circle and being ignored; runs ended "ring clear" with blocks still
in. The fence and back-off-only rules still keep the robot inside.

v4.18: trip threshold from tapecal.txt (MENU + LEFT TRIGGER
calibration) when present — half the measured floor/tape contrast.

v4.17: (self-echo rejection — removed in v4.20; the root cause was a
sensor hardware fault, since repaired.)

v4.16 (8/7 night logs): PADDLE STOW ON at 90 deg — after a power-on
the unpowered paddle drooped into the rangefinder's beam and every
scan saw a phantom "block" at a constant 10.6cm; the robot chased it
cycle after cycle into the ring line. (The v4.3-era stow suspicion was
right — the ANGLE was the problem, not the stow.)

v4.15 (8/7 logs — "speeding past the line again"): push effort 0.7 ->
0.55 and survey legs at 0.55 (a leg CROSSED the ring at 0.65 without a
trip); LINE_DELTA 0.05 -> 0.04 (tape dips to 0.921-0.925 were missed
by a hair); and a bug fix — an implausible survey chord now rejects
the whole survey instead of rebasing Home onto a bogus midpoint (one
run put Home 22cm off center that way).

v4.14 (same night, second log pass): SCAN TARGET FILTER — the scan
locked onto an object OUTSIDE the circle (26.6cm echo) and chased it
over the line. Every echo (coarse scan AND fine aim) is now projected
from the live pose and rejected if the target point lies outside the
surveyed ring.

v4.13 (8/6 night logs — "kept crossing out of the ring"): the BLUE
PAINTER'S TAPE only reads ~0.07 below the floor (0.88-0.90 vs 0.96),
so the old LINE_DELTA=0.20 could never trip — the robot literally
could not see the ring boundary. LINE_DELTA is now 0.05 (debounce 3).
The finish sweep is DISABLED per driver request: on any line trip the
robot only backs up and turns away — it never drives forward past the
line again. Approach effort 0.55 -> 0.65 (still limping at 4.5V sag).

v4.12 (8/6 field logs): stall watch in every watched drive (no progress
1.2s -> effort boost; 3.5s -> give up and regroup — the logged approach
sat frozen at 2.4cm for 10+ seconds), one boosted retry when a backup
stalls below half its distance, and a loud diagnostic when the survey
never sees ANY reflectance change (the logged leg 1 drove 48cm with the
sensors frozen at 0.965 — that's a sensor/setup problem, not tuning).

v4.11: SURVEY + GEO-FENCE — the robot keeps leaving the circle, so now
it MEASURES the circle first and then refuses to drive out of it:
  * SURVEY (at launch): drive straight to the ring line, back up 15cm,
    turn 180, drive straight to the line on the far side. The two trip
    points span a diameter — their midpoint IS the ring center. "Home"
    (POSE origin) is rebased onto that surveyed center, and the radius
    is measured on the way. Every go_home() now returns to the real
    center, not to wherever the robot happened to be placed.
  * GEO-FENCE: every push, sweep touch and BACKUP is capped by the
    distance to the surveyed circle edge (chord math from the live
    POSE). The logged ring exits came from the sweep: a backup stalled
    at -1.5cm of -10, but the sweep cap still assumed the full 10cm of
    retreat — so the next angled push drove 17cm from a spot right at
    the line. Sweep caps now use the ACTUAL encoder-measured backup
    distance AND the fence, whichever is smaller.
  * A push stopped by the fence at the ring edge counts as a line touch
    (the block is over the line even if a block on the tape kept the
    sensors from tripping), so the finish sweep still runs.

v4.0: DUAL-MODE. Importable by main_code.py (MENU -> X launches it; the
supervisor owns PestoLink and its START button aborts mid-run via the
_HOOKS["abort"] callback polled in every loop) or runnable standalone
exactly as before (own PestoLink, Button 0 to launch).

v3.3: motor efforts doubled (approach 0.35->0.7, push 0.28->0.56,
turn 0.4->0.8) now that fresh batteries are assumed; heading-correction
clamp scaled to match and PUSH_OVER trimmed for the extra coast.

v3.2 — fixes from the first logged run:
  * CRASH FIX: math.hypot() does not exist in MicroPython — go_home()
    died with AttributeError the first time it ran. Replaced with
    sqrt(x^2 + y^2).
  * LINE DETECTION REWORKED: the log showed the FLOOR reading
    L=0.733 / R=0.620, over the old 0.7 trip threshold — the robot
    declared "tape!" while parked on plain floor, which is why cycle 1
    aborted instantly. Fixed thresholds can't work on a floor that dark,
    so the robot now samples its own floor for a second right after
    launch (it starts at ring center — guaranteed off the tape) and
    trips only when a sensor moves AWAY from that baseline by
    LINE_DELTA, debounced over 2 consecutive reads. Works for light or
    dark tape with no manual calibration.
  * LOW-BATTERY WARNING (v3.7: rescaled for the 4xAA pack — 5.4V
    resting is normal there; warnings now trip below 4.6V) —
    at 5.4V turns stall, pushes are weak, and brownout resets (the
    LED-goes-dark symptom) are expected. The log now shouts when
    voltage is low at boot and at launch. Put fresh batteries in
    before blaming the code.

v3.1: adds a BLACK-BOX LOG so a failed run can tell us what happened.
Every run appends to sumo_log.txt on the XRP's flash:
  * a BOOT marker each time the program starts (a reset mid-run shows up
    as a new BOOT marker with no DONE before it — that means the board
    lost power/browned out, not a code bug)
  * battery voltage at boot, at launch, and through every cycle (sagging
    volts right before the log stops = brownout; swap batteries)
  * every phase transition with sensor numbers (scan hits, fine-aim
    result, approach/push outcomes, line-trip reflectance values, slip
    events, homing distance)
  * a heartbeat during drives (~2/s): distance, line sensors, yaw
  * the full Python traceback if the code crashes — the finally/except
    write it into the log before the program dies.
To retrieve: connect with the XRP IDE (USB), open the filesystem
browser, open sumo_log.txt, copy everything out. Delete the file
occasionally so it doesn't grow forever.

WHY A RUN DIES EARLY — the two usual suspects, and how the log tells
them apart:
  1. BROWNOUT/RESET: motors + BLE sag a weak battery pack, the board
     resets, the LED goes dark (the LED is only ON because the program
     latched it — a reset clears it). Log shows: entries stop mid-phase,
     no traceback, no DONE; often falling "batt=" values first.
     Fix: fresh batteries, motor power switch fully on.
  2. CRASH: a Python exception ends the program. Log shows: TRACEBACK
     lines at the end. Send me those lines and I can fix the bug.
Also: if you run this from the IDE over the BLUETOOTH connection, the
PestoLink BLE radio takes over and the IDE link drops — which kills the
running program a moment in. Upload over USB CABLE and run it there, or
save it as main.py and run it from a power cycle.

v3 recap — NO PLOW, the sensor face is the pusher:
  * coarse rotate-until-echo, then FINE-AIM to center on the block
    (no side wings means off-center contact loses the block sideways)
  * drive to CONTACT (rangefinder <= CONTACT_CM), push straight with
    IMU heading hold until the reflectance sensors catch the ring tape
  * slip detection: consecutive clean open-floor readings mid-push mean
    the block escaped — stop, go home, rescan
  * short PUSH_OVER shove so the block fully CROSSES the line
    (customer: touching the line is still "in"), back up, dead-reckon
    home, rotate for the next block. One press of Button 0 ("A") runs
    the whole thing.

Positive turn() / positive yaw = LEFT (CCW). Negative = RIGHT (CW).
"""

from XRPLib.defaults import *
from XRPLib.pid import PID

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

from machine import Pin, ADC
import time
import math
import sys

# Supervisor hook (v4): main_code.py sets _HOOKS["abort"] to its
# check_abort so the controller's START button can stop this challenge
# at any point. Standalone runs leave it as a no-op.
_HOOKS = {"abort": lambda: None}

def _abort():
    _HOOKS["abort"]()

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # BLE name shown in PestoLink (8 chars max)
START_BUTTON = 0            # "A" on most gamepads (verify in the tester)

# Ring geometry — CONFIRMED by tape measure 8/6: ring radius is 30cm.
# v4.11: this is now the FALLBACK — the launch survey measures the real
# radius and center; RING_RADIUS_CM is used only if the survey fails.
RING_RADIUS_CM = 30.0       # ring radius (measured); caps pushes/homing

# SURVEY (v4.11): measure the ring before playing.
SURVEY = True               # False = old behavior (assume start = center)
SURVEY_EFFORT = 0.55        # v4.15: survey legs slower than approach —
                            # one logged leg 2 CROSSED the ring line at
                            # 0.65 without ever seeing it
SURVEY_BACK_CM = 15.0       # back away from the first line trip before
                            # the 180 turn (customer spec)
SENSOR_AHEAD_CM = 5.0       # reflectance sensors sit ~this far ahead of
                            # the wheel axle; line trips happen this far
                            # early, so the measured radius adds it back
FENCE_MARGIN_CM = 4.0       # geo-fence keeps the AXLE this far inside
                            # the surveyed line — wheels are on the axle,
                            # so this is the wheel margin too. NOTE: the
                            # survey chord runs through the START spot,
                            # so sideways placement error survives into
                            # the center estimate — place the robot
                            # within a few cm of center and this margin
                            # absorbs the rest
SCAN_MAX_CM = 28.0          # accept scan echoes closer than this. Keep
                            # it inside the ring radius so blocks already
                            # out (and anything beyond the ring) are
                            # never targeted.
CONTACT_CM = 5.0            # reading at/below this = block is on the nose
PUSH_OVER_CM = 0.0          # v4.7: the straight shove is retired — the
                            # FINISH SWEEP (angled touches, see below)
                            # now gets the block fully out instead
RETURN_CM = 12.0            # back away from the line after each push
                            # (in a ~30cm ring, 12cm is already nearly
                            # halfway home)

# Block-slip detection during the push (see make_slip_checker)
BLOCK_LOST_MIN_CM = 15.0
BLOCK_LOST_MAX_CM = 150.0
LOST_CONFIRM = 3
SLIP_CHECK = True

# Reflectance line detection (0 = white .. 1 = black).
# The floor baseline is AUTO-SAMPLED at launch (robot starts at ring
# center, guaranteed off the tape). A sensor trips when it moves away
# from its own floor baseline by more than LINE_DELTA.
LINE_DELTA = 0.04           # v4.15: the tape dips only to 0.92-0.93 in
                            # places (logged trips at 0.904-0.914 but
                            # MISSES at 0.921-0.925 with the 0.05
                            # delta) — 0.04 catches the faint spots;
                            # the 3-read debounce guards the noise
                            # v4.13: THE BLUE PAINTER'S TAPE ONLY READS
                            # ~0.07 BELOW THE FLOOR (0.88-0.90 vs 0.96
                            # in the 8/6 night logs) — the old 0.20 was
                            # physically impossible to trip, which is
                            # exactly why the robot kept driving out of
                            # the ring. 0.05 trips on the real tape.
LINE_POLARITY = "lighter"   # tape reads LOWER than the floor baseline
LINE_DEBOUNCE = 3           # v4.13: 3 consecutive reads (~30ms) — the
                            # tighter delta needs one more sample of
                            # noise immunity; tape is 3.65cm wide, so
                            # even at push speed it's seen for >100ms

# Motion — v3.4: doubled efforts (v3.3) outran the line detector, so
# efforts are now original + 0.1. ALSO fixed the real problem: the slip
# checker's ultrasonic reads (~45ms each) were throttling the whole
# drive loop, so the tape was only checked every ~65ms — at speed, the
# tape could pass between checks. Slow sensors now run on their own
# slower schedule; the line check runs every ~10ms pass.
# v4.2: 0.4 is the working effort AND the hard floor. The logged runs
# showed moves stalling because the XRPLib PIDs taper effort toward the
# target (turns bottom out at 0.1, straights at 0.3) — below this
# drivetrain's stiction. Custom PIDs below keep min_output at 0.4.
# v4.4: REVERTED to the v4.2 motion numbers after the v4.3 bump failed
# its test — back to the values from the successful 8/6 run. The only
# keeper from v4.3 is a slightly stronger backup (the logged backups
# stalled at -2cm of -12), and the turn-PID tolerance fix (see
# _turn_pid) which addresses the real turning problem.
APPROACH_EFFORT = 0.65      # v4.13: 0.55 still limped at 4.5V sag (the
                            # survey leg crawled 5cm in 15 seconds)
PUSH_EFFORT = 0.55          # v4.15: 0.7 -> 0.55 (driver: "speeding
                            # past the line") — every scored push in
                            # the 8/7 log was saved by the geo-fence,
                            # not the sensors; slower = more samples
                            # per cm of tape and less overshoot
TURN_EFFORT = 0.75          # v4.8: raised from 0.5 — manual-mode logs
                            # showed 0.55-effort turns timing out while
                            # 0.8-effort retries finished instantly
BACKUP_EFFORT = 0.7         # v4.10: reverse STILL stalled at 0.55
                            # (drove -1.5cm of -12 in the logs)
MIN_EFFORT = 0.4            # no wheel / no PID output ever weaker than this

# FINISH SWEEP (v4.7): on the main push's line trip, instead of one
# deep shove, the block is pushed fully out with three angled touches
# (sensors trip ahead of the wheels, so every touch stops with the
# wheels inside the ring):
#   1. back up SWEEP_BACK_CM      2. turn LEFT 45, push to the line
#   3. back up again              4. turn RIGHT 90 (net 45 right of the
#      original heading), push to the line once more
# v4.13: FINISH SWEEP DISABLED (driver request: after seeing the line,
# ONLY back up and turn — never drive forward past it again). The
# angled follow-up touches drive back toward the line, which with the
# faint blue tape risked missed trips; the straight push to the line
# already carries the block onto/over it.
FINISH_SWEEP = False
SWEEP_BACK_CM = 10.0        # backup before each angled touch
SWEEP_ANGLE_DEG = 45        # first sweep angle (left), then 90 right
SWEEP_OVERDRIVE_CM = 2.0    # sensors may pass the nominal line position
                            # by this much per touch (absorbs geometry
                            # slop; tape is 3.7cm wide)
# v4.9: sweep drive caps are now DERIVED from the original straight-on
# line trip, so the wheels cannot cross even if the sensors never
# re-trip (e.g. the pushed block is sitting on the tape):
#   * after backing B=10cm along the approach normal, the sensors are
#     10cm inside the line. Driving at 45 deg closes the gap at
#     cos(45)=0.707 per cm, so the line is at 10/0.707 = 14.1cm;
#     cap1 = (10+2)/0.707 = 17.0cm  (sensors at most 2cm past the line;
#     wheels ~6cm behind them would need 22.6cm to cross).
#   * after touch 1, backing 10cm along the 45-deg heading retreats
#     only 10*0.707 = 7.1cm perpendicular; cap2 = 10 + 2/0.707 = 12.8cm
#     (wheels would need 18.5cm to cross). Margin holds on both legs.

# Paddle stow (v4.4: OFF by default — introduced in v4.3, and if the
# stow angle left the paddle in front of the rangefinder it could
# explain the failed run; enable deliberately after a bench check).
PADDLE_STOW = True          # v4.16: ON — the 8/7 night scans read a
                            # constant 10.6cm in every direction (the
                            # unpowered paddle drooped into the
                            # rangefinder's beam after power-on) and
                            # the robot chased that phantom every
                            # cycle. 90 deg is confirmed clear.
PADDLE_STOW_DEG = 90

# WIGGLE TURNS (v4.5): field friction stalls pure in-place turns, so
# every turn superimposes a small alternating fore/aft bias on the
# counter-rotation — each wheel keeps breaking static friction, the
# biases cancel over a cycle, and the robot turns in place with a
# slight shimmy instead of loading up and stalling.
WIGGLE_BIAS = 0.18
WIGGLE_PERIOD_S = 0.22
WIGGLE_TOL_DEG = 3.0
LEFT_TURN_BOOST = 1.2       # this robot turns LEFT (CCW) weaker than
                            # right — matches main_code v1.15

def wiggle_turn(degrees, effort, timeout_s=None):
    """Relative in-place turn with the anti-friction wiggle. IMU-
    verified; polls the supervisor abort every pass. True = reached."""
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
                log("wiggle turn: timeout, %.0f deg short" % err)
                return False
            if time.ticks_diff(now, phase_ms) >= WIGGLE_PERIOD_S * 1000:
                phase_ms = now
                bias = -bias
            # left/CCW turns get LEFT_TURN_BOOST (weak-side motor), and
            # each wheel's magnitude is floored at MIN_EFFORT so the
            # bias half-cycle can't dip a wheel into the stall zone
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

def _straight_pid(max_eff):
    """Fresh straight-drive PID per call, same stiction floor."""
    return PID(kp=0.1, ki=0.04, kd=0.04,
               min_output=MIN_EFFORT, max_output=max_eff,
               max_integral=10, tolerance=0.25, tolerance_count=3)
PUSH_KP = 0.02              # IMU heading-hold gain during pushes
PUSH_CORR_MAX = 0.15        # heading-correction clamp (scaled with effort)
SLOW_CHECK_S = 0.15         # period for ultrasonic-based checks (slip /
                            # contact) inside drive loops — the LINE check
                            # runs every pass and is never throttled
SCAN_STEP_DEG = 12
FULL_SWEEP_STEPS = 30       # 30 x 12 = 360
FINE_STEP_DEG = 5
FINE_SPAN_STEPS = 5

MAX_RUNTIME_S = 180
MAX_PUSHES = 8

# Battery — v3.7: the XRP runs a 4xAA pack, so the old ">7V or you're
# dying" warnings were calibrated for a pack this robot doesn't have.
# 4-cell reality: fresh alkaline ~6.3V, full NiMH ~5.4V, so a resting
# 5.4V is HEALTHY. Warn only below ~1.15V/cell resting.
BATT_CELLS = 4
LOW_BATT_V = 1.15 * BATT_CELLS      # 4.6V: genuinely low for 4 cells
SAG_BATT_V = 1.05 * BATT_CELLS      # 4.2V under load: brownout territory

# Logging
LOG_TO_FILE = True
LOG_PATH = "LOG.TXT"           # unified log — every program appends here
HEARTBEAT_S = 0.5           # heartbeat period inside drive loops

def cal_derived_delta():
    """v4.18: buffered trip delta from tapecal.txt (written by the
    supervisor's calibration mode — MENU + LEFT TRIGGER), or None.
    Half the measured floor-to-tape contrast = lighting buffer."""
    try:
        with open("tapecal.txt") as f:
            fl, fr, tl, tr = [float(x) for x in f.read().split(",")]
        c = (fl + fr) / 2 - (tl + tr) / 2
        if c <= 0.02:
            return None
        return min(0.15, max(0.025, c * 0.5))
    except Exception:
        return None

# ------------------------------- LOGGING ---------------------------------

_BOOT_MS = time.ticks_ms()

def battery_voltage():
    try:
        return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)
    except Exception:
        return -1.0

_LOG_BROKEN = {"reported": False}

def log(msg, console=True):
    """Timestamped line to flash (always) and console (unless
    console=False). v4.10: heartbeats are file-only — an attached BLE
    console blocks ~1s per print, which slowed the control loop to ~1Hz
    and let the robot cross the ring tape between line checks (the
    drive-out-of-the-ring failure)."""
    t = time.ticks_ms() / 1000.0    # seconds since POWER-ON —
    #     the same clock in every program, so LOG.TXT reads as
    #     one continuous session timeline
    line = "[%9.2fs][SUMO ] %s" % (t, msg)
    if console:
        print(line)
    if LOG_TO_FILE:
        try:
            f = open(LOG_PATH, "a")
            f.write(line + "\n")
            f.close()               # open/close per line = always flushed
        except Exception as e:
            if not _LOG_BROKEN["reported"]:
                _LOG_BROKEN["reported"] = True
                print("*** LOG FILE WRITE FAILED: %r — flash may be full "
                      "or corrupted. Run continues, file logging off. ***"
                      % e)

def log_selftest():
    """At boot: prove the log file is writable and report free flash.
    If writing fails, the RGB LED goes RED for 3 seconds — if you see
    red at power-up, the filesystem is full or corrupted and no log
    will be produced. Rotates an oversized log automatically."""
    import os
    try:
        try:
            if os.stat(LOG_PATH)[6] > 300 * 1024:   # size > 300KB
                os.remove(LOG_PATH)                  # rotate: start fresh
                print("log: rotated oversized %s" % LOG_PATH)
        except OSError:
            pass                                     # no file yet — fine
        f = open(LOG_PATH, "a")
        f.write("")                                  # write test
        f.close()
        try:
            st = os.statvfs("/")
            free_kb = st[0] * st[3] // 1024
            log("log self-test OK, %d KB flash free" % free_kb)
            if free_kb < 64:
                log("*** WARNING: flash nearly full ***")
        except Exception:
            log("log self-test OK")
        return True
    except Exception as e:
        print("*** LOG SELF-TEST FAILED: %r ***" % e)
        print("*** Flash full or corrupted. Delete sumo_log.txt in the "
              "IDE file browser; if that fails the filesystem may need "
              "a MicroPython reinstall. ***")
        set_status(255, 0, 0)                        # RED = logging broken
        time.sleep(3)
        return False

def log_exception(e):
    log("!!! CRASH — traceback follows")
    try:
        sys.print_exception(e)      # to console
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
        pass  # XRP Beta has no RGB LED

def front_distance():
    """Median of 3 pings; timeouts (65535) safely read as 'nothing there'."""
    reads = []
    for _ in range(3):
        reads.append(rangefinder.distance())
        time.sleep(0.015)
    reads.sort()
    return reads[1]

def line_values():
    return reflectance.get_left(), reflectance.get_right()

# Per-sensor floor baseline, sampled at launch while parked at ring center.
# "lo" = lowest reading seen anywhere in the run (v4.12 diagnostics).
FLOOR = {"l": 0.5, "r": 0.5, "hits": 0, "lo": 1.0}

def capture_floor_baseline():
    l = r = 0.0
    n = 12
    for _ in range(n):
        vl, vr = line_values()
        l += vl
        r += vr
        time.sleep(0.02)
    FLOOR["l"], FLOOR["r"] = l / n, r / n
    log("floor baseline: L=%.3f R=%.3f (delta=%.2f, polarity=%s)"
        % (FLOOR["l"], FLOOR["r"], LINE_DELTA, LINE_POLARITY))

def _deviates(value, baseline):
    d = value - baseline
    if LINE_POLARITY == "darker":
        return d > LINE_DELTA
    if LINE_POLARITY == "lighter":
        return -d > LINE_DELTA
    return abs(d) > LINE_DELTA          # "any"

def line_detected():
    """Tape = a sensor deviating from its own floor baseline, debounced
    over LINE_DEBOUNCE consecutive calls so one noisy read can't trip."""
    l, r = line_values()
    lo = l if l < r else r
    if lo < FLOOR["lo"]:
        FLOOR["lo"] = lo         # v4.12: low-water mark for diagnostics
    if _deviates(l, FLOOR["l"]) or _deviates(r, FLOOR["r"]):
        FLOOR["hits"] += 1
    else:
        FLOOR["hits"] = 0
    return FLOOR["hits"] >= LINE_DEBOUNCE

def normalize(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

TURN_TOL_DEG = 4.0
TURN_RETRIES = 3

def wait_rotation_stop(timeout_s=1.2):
    """v4.22: wait for the yaw to stop changing before verifying a
    turn — verifying mid-coast let turns end 8-19 deg past target
    (see maze v1.11)."""
    last = imu.get_yaw()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_s * 1000:
        _abort()
        time.sleep(0.08)
        y = imu.get_yaw()
        if abs(y - last) < 0.4:
            break
        last = y

def turn_to_heading(target_yaw, effort):
    """Rotate to an absolute IMU yaw AND VERIFY IT GOT THERE. The logged
    runs showed stalled turns silently timing out, after which go_home()
    drove its whole 'home' leg in the wrong direction and the pose map
    fell apart. Now each turn is checked against the IMU and re-commanded
    with escalating effort until it's within TURN_TOL_DEG."""
    for attempt in range(TURN_RETRIES):
        _abort()
        wait_rotation_stop()             # v4.22: measure at rest
        delta = normalize(target_yaw - imu.get_yaw())
        if abs(delta) <= TURN_TOL_DEG:
            return True
        boosted = min(0.9, effort + 0.15 * attempt)
        wiggle_turn(delta, boosted, timeout_s=3)
    final_err = normalize(target_yaw - imu.get_yaw())
    if abs(final_err) > TURN_TOL_DEG:
        log("turn: STALLED %.0f deg short of target (battery?)" % final_err)
        return False
    return True

def traveled_since(start_l, start_r):
    dl = drivetrain.get_left_encoder_position() - start_l
    dr = drivetrain.get_right_encoder_position() - start_r
    return (dl + dr) / 2

POSE = {"x": 0.0, "y": 0.0}

# Ring model (v4.11): center is ALWAYS POSE (0,0). Before the survey
# that's just "where the robot started"; after the survey the POSE is
# rebased so (0,0) is the measured center of the circle.
RING = {"r": RING_RADIUS_CM, "surveyed": False}

def record_move(dist_cm):
    rad = math.radians(imu.get_yaw())
    POSE["x"] += dist_cm * math.cos(rad)
    POSE["y"] += dist_cm * math.sin(rad)

def dist_from_center():
    return math.sqrt(POSE["x"] * POSE["x"] + POSE["y"] * POSE["y"])

def fence_remaining(heading_deg=None):
    """GEO-FENCE (v4.11): how far the robot can travel along a heading
    before the axle reaches the ring line minus FENCE_MARGIN_CM. Chord
    math on the surveyed circle: solve |P + t*u| = r_safe for t."""
    r_safe = RING["r"] - FENCE_MARGIN_CM
    if heading_deg is None:
        heading_deg = imu.get_yaw()
    rad = math.radians(heading_deg)
    ux, uy = math.cos(rad), math.sin(rad)
    px, py = POSE["x"], POSE["y"]
    p_u = px * ux + py * uy
    disc = p_u * p_u - (px * px + py * py - r_safe * r_safe)
    if disc <= 0:
        return 0.0          # already at/outside the safe circle
    return max(0.0, -p_u + math.sqrt(disc))

def straight_tracked(dist_cm, effort, timeout=4):
    """Straight move that records what the ENCODERS say actually
    happened, not what was commanded — a stalled/timed-out move used to
    poison the pose map with distance the robot never drove.
    v4.11: RETURNS the actual signed distance so callers (the sweep
    caps) can plan from reality, not from the command."""
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    drivetrain.straight(dist_cm, effort, timeout=timeout,
                        main_controller=_straight_pid(abs(effort)))
    actual = traveled_since(start_l, start_r)
    record_move(actual)
    if abs(actual - dist_cm) > 5:
        log("straight: commanded %.1fcm, drove %.1fcm (stall?)"
            % (dist_cm, actual))
    return actual

def bounded_backup(dist_cm, effort=BACKUP_EFFORT, timeout=3, tag="backup"):
    """Reverse, but never over the ring line: the requested distance is
    clamped to the fence along the REVERSE heading first. Returns the
    actual distance backed (positive cm)."""
    allow = fence_remaining(imu.get_yaw() + 180.0)
    d = min(dist_cm, allow)
    if d < dist_cm - 0.5:
        log("%s: GEO-FENCED %.1f -> %.1fcm (ring line behind)"
            % (tag, dist_cm, d))
    if d < 0.5:
        return 0.0
    backed = -straight_tracked(-d, effort, timeout=timeout)
    # v4.12: reverse stalls are chronic (logged -1.5cm of -12 even at
    # 0.70) — one louder retry for the remainder before accepting it
    if backed < d * 0.5:
        boost = min(0.9, effort + 0.15)
        log("%s: stalled at %.1f of %.1fcm — retrying remainder at %.2f"
            % (tag, backed, d, boost))
        backed += -straight_tracked(-(d - backed), boost, timeout=timeout)
    return backed

# Stall handling inside watched drives (v4.12): the logged approaches
# froze at 2.4cm for 10+ seconds under battery sag while the loop
# happily commanded 0.55 forever. Now: no progress for STALL_BOOST_S
# raises the effort a notch; still no progress by STALL_GIVEUP_S ends
# the drive with outcome "stall" so the caller can regroup.
STALL_MIN_CM = 0.5
STALL_BOOST_S = 1.2
STALL_BOOST_ADD = 0.15
STALL_EFFORT_MAX = 0.85
STALL_GIVEUP_S = 3.5

def drive_watching(effort, stop_check, max_cm, hold_yaw=None, tag="drive"):
    """Drive forward until stop_check() returns a reason, the tape shows
    up ('line'), max_cm is hit ('cap'), or progress dies ('stall').
    IMU heading hold keeps pushes straight. Logs a heartbeat ~2/s so the
    log shows what the sensors saw right up to the moment anything
    stopped."""
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    last_beat = time.ticks_ms()
    last_slow = 0
    result = "cap"
    FLOOR["hits"] = 0        # a stale debounce count from the previous
                             # trip must not instantly re-trip this drive
    prog_ms = time.ticks_ms()
    prog_cm = 0.0
    boosted = False
    while True:
        _abort()                 # START stays live during every drive
        # LINE CHECK EVERY PASS — reflectance reads are microseconds, so
        # this loop spins ~every 10ms and cannot step over the tape.
        if line_detected():
            l, r = line_values()
            log("%s: LINE trip L=%.3f R=%.3f after %.1fcm"
                % (tag, l, r, traveled_since(start_l, start_r)))
            result = "line"
            break
        # SLOW CHECKS (ultrasonic ~45ms) only every SLOW_CHECK_S — they
        # used to run every pass and starve the line check at speed.
        reason = None
        if stop_check:
            now = time.ticks_ms()
            if time.ticks_diff(now, last_slow) >= SLOW_CHECK_S * 1000:
                last_slow = now
                reason = stop_check()
        if reason:
            log("%s: stop '%s' after %.1fcm"
                % (tag, reason, traveled_since(start_l, start_r)))
            result = reason
            break
        trav = traveled_since(start_l, start_r)
        if trav > max_cm:
            log("%s: cap %.1fcm reached" % (tag, max_cm))
            break
        # ---- stall watch (v4.12) ----
        now = time.ticks_ms()
        if trav - prog_cm >= STALL_MIN_CM:
            prog_cm = trav
            prog_ms = now
            if boosted:
                boosted = False
                log("%s: moving again — boost off" % tag, console=False)
        stalled_ms = time.ticks_diff(now, prog_ms)
        if stalled_ms > STALL_GIVEUP_S * 1000:
            log("%s: STALLED at %.1fcm for %.1fs (batt=%.2fV) — "
                "giving up this drive"
                % (tag, trav, stalled_ms / 1000, battery_voltage()))
            result = "stall"
            break
        eff = effort
        if stalled_ms > STALL_BOOST_S * 1000:
            eff = min(STALL_EFFORT_MAX, effort + STALL_BOOST_ADD)
            if not boosted:
                boosted = True
                log("%s: no progress %.1fs at %.1fcm — boosting %.2f "
                    "-> %.2f" % (tag, stalled_ms / 1000, trav,
                                 effort, eff))
        if hold_yaw is not None:
            err = normalize(hold_yaw - imu.get_yaw())
            corr = max(-PUSH_CORR_MAX, min(PUSH_CORR_MAX, PUSH_KP * err))
            # stiction floor: heading corrections speed the fast wheel
            # up rather than slowing the slow wheel into the dead zone
            drivetrain.set_effort(max(MIN_EFFORT, eff - corr),
                                  max(MIN_EFFORT, eff + corr))
        else:
            drivetrain.set_effort(eff, eff)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_beat) >= HEARTBEAT_S * 1000:
            last_beat = now
            l, r = line_values()
            # NOTE: no front_distance() here — a 45ms ultrasonic read in
            # the heartbeat would blind the line check for ~2cm of travel
            log("%s: hb L=%.3f R=%.3f yaw=%+.1f trav=%.1fcm batt=%.2fV"
                % (tag, l, r, imu.get_yaw(),
                   traveled_since(start_l, start_r), battery_voltage()),
                console=False)
        time.sleep(0.01)
    drivetrain.stop()
    record_move(traveled_since(start_l, start_r))
    return result

def go_home():
    # NOTE: math.hypot() does not exist in MicroPython — use sqrt.
    d = math.sqrt(POSE["x"] * POSE["x"] + POSE["y"] * POSE["y"])
    if d < 8:
        log("home: already near center (%.1fcm off)" % d)
        return
    bearing = math.degrees(math.atan2(-POSE["y"], -POSE["x"]))
    log("home: %.1fcm back to center, bearing %+.1f" % (d, bearing))
    turn_to_heading(bearing, TURN_EFFORT)
    straight_tracked(d, APPROACH_EFFORT, timeout=6)

# ------------------------------- SURVEY ----------------------------------

def survey_ring():
    """v4.11 launch survey: drive straight to the line, back up
    SURVEY_BACK_CM, turn 180, drive straight to the line on the far
    side. The two trip points span a diameter; their midpoint is the
    ring CENTER and half their separation (+ sensor offset) is the
    RADIUS. POSE is rebased so Home (0,0) = that center, then the robot
    drives home. Returns True on success; on any failure it logs, keeps
    the tape-measured fallback geometry, and goes home."""
    log("survey: leg 1 — driving straight to the line")
    yaw0 = imu.get_yaw()
    out = drive_watching(SURVEY_EFFORT, None,
                         max_cm=RING_RADIUS_CM * 1.6,
                         hold_yaw=yaw0, tag="survey1")
    if out != "line":
        log("survey: no line on leg 1 (%s) — using tape-measured "
            "geometry instead" % out)
        # v4.12 diagnostic: the 8/6 logs show leg 1 driving 48cm with
        # the reflectance frozen at 0.965-0.966 the whole way. If the
        # lowest reading of the leg never left the floor baseline, the
        # sensors never saw ANY tape — that's a setup problem, not a
        # tuning problem.
        base_lo = FLOOR["l"] if FLOOR["l"] < FLOOR["r"] else FLOOR["r"]
        if FLOOR["lo"] > base_lo - 0.1:
            log("*** reflectance never changed the whole leg (lowest "
                "%.3f vs floor %.3f). Either the robot is NOT inside "
                "the ring, or the line sensors aren't seeing the floor "
                "(height should be ~3-5mm, nothing blocking them). ***"
                % (FLOOR["lo"], base_lo))
        go_home()
        return False
    ax, ay = POSE["x"], POSE["y"]
    straight_tracked(-SURVEY_BACK_CM, BACKUP_EFFORT, timeout=4)
    if not turn_to_heading(yaw0 + 180.0, TURN_EFFORT):
        log("survey: 180 turn failed — using tape-measured geometry")
        go_home()
        return False
    log("survey: leg 2 — driving to the far side")
    out = drive_watching(SURVEY_EFFORT, None,
                         max_cm=RING_RADIUS_CM * 2.0 * 1.3,
                         hold_yaw=imu.get_yaw(), tag="survey2")
    if out != "line":
        log("survey: no line on leg 2 (%s) — using tape-measured "
            "geometry" % out)
        go_home()
        return False
    bx, by = POSE["x"], POSE["y"]
    cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
    dx, dy = ax - bx, ay - by
    r = math.sqrt(dx * dx + dy * dy) / 2.0 + SENSOR_AHEAD_CM
    if not (0.6 * RING_RADIUS_CM) < r < (1.5 * RING_RADIUS_CM):
        # v4.15 BUG FIX: an implausible chord used to keep the fallback
        # RADIUS but still rebase Home onto the bogus midpoint — the
        # 8/7 log shows a 48.9cm "radius" moving Home 22cm off the real
        # center. Now the whole survey is rejected instead.
        log("survey: measured radius %.1fcm implausible (expected ~%.0f)"
            " — REJECTING the survey, keeping start-point geometry"
            % (r, RING_RADIUS_CM))
        go_home()
        return False
    # Rebase the pose map: the surveyed center becomes (0,0) = Home.
    POSE["x"] -= cx
    POSE["y"] -= cy
    RING["r"] = r
    RING["surveyed"] = True
    log("survey: HOME set — center was (%.1f, %.1f) from the start "
        "spot, radius=%.1fcm" % (cx, cy, r))
    bounded_backup(SWEEP_BACK_CM, tag="survey back")
    go_home()
    return True

# ------------------------------ BEHAVIORS --------------------------------

def echo_inside_ring(d):
    """v4.14: is the point the rangefinder is echoing off actually
    INSIDE the ring? The 8/6 night log shows the scan locking onto an
    object at 26.6cm that was sitting outside the circle — the robot
    then chased it across the line. The echo point is projected from
    the live POSE along the current heading and checked against the
    ring model; anything outside is not a target."""
    rad = math.radians(imu.get_yaw())
    ex = POSE["x"] + (d + SENSOR_AHEAD_CM) * math.cos(rad)
    ey = POSE["y"] + (d + SENSOR_AHEAD_CM) * math.sin(rad)
    # v4.19: margin +3 (was -1) — blocks sitting ON the line were
    # projecting just outside and being ignored (8/7 log: 13-16cm
    # echoes skipped, run ended with blocks still in the ring). The
    # fence and back-off-only rules still keep the robot inside.
    return math.sqrt(ex * ex + ey * ey) <= RING["r"] + 3.0

def rotate_until_block():
    """Coarse: rotate in place, stop at the FIRST echo within SCAN_MAX_CM.

    v3.6: MOTOR-ALIVE CHECK. A frozen 'cyan light, nothing happening'
    run turned out to be turns silently timing out (dead pack / motor
    switch off). Now, if the first scan turn produces almost no yaw
    change, one louder retry is made — and if the robot STILL hasn't
    rotated, the run aborts with a red LED and a loud log line instead
    of pretending to scan for a minute."""
    for step in range(FULL_SWEEP_STEPS):
        _abort()                 # START stays live during the scan
        d = front_distance()
        if d < SCAN_MAX_CM:
            if not echo_inside_ring(d):
                log("scan: echo at %.1fcm (step %d, yaw %+.1f) is "
                    "OUTSIDE the ring — ignoring it"
                    % (d, step, imu.get_yaw()))
            else:
                log("scan: hit at step %d, d=%.1fcm yaw=%+.1f"
                    % (step, d, imu.get_yaw()))
                return d
        yaw_before = imu.get_yaw()
        wiggle_turn(SCAN_STEP_DEG, TURN_EFFORT, timeout_s=2)
        # v4.8: motor-alive threshold loosened 3.0 -> 1.5 deg. A SLOW
        # turn (friction) was reading as "dead motors" and aborting the
        # whole run 4 seconds in — only near-zero rotation after a
        # full-effort retry should count as dead.
        if step == 0 and abs(imu.get_yaw() - yaw_before) < 1.5:
            log("scan: first turn didn't move (yaw %+.1f -> %+.1f) — "
                "retrying at full effort"
                % (yaw_before, imu.get_yaw()))
            wiggle_turn(SCAN_STEP_DEG, 0.9, timeout_s=3)
            if abs(imu.get_yaw() - yaw_before) < 1.5:
                log("*** MOTORS NOT MOVING. Check the motor power "
                    "switch and battery pack. Aborting run. ***")
                set_status(255, 0, 0)        # red = motors dead
                return None
        if step > 0 and step % 8 == 0:
            log("scan: step %d, d=%.1fcm yaw=%+.1f batt=%.2fV"
                % (step, d, imu.get_yaw(), battery_voltage()))
    d = front_distance()
    if d < SCAN_MAX_CM and echo_inside_ring(d):
        log("scan: hit on final look, d=%.1fcm" % d)
        return d
    log("scan: full sweep, nothing (in-ring) inside %.0fcm" % SCAN_MAX_CM)
    return None

def fine_aim(coarse_d):
    """Center the nose on the block: sweep a small window, settle on the
    heading with the smallest distance.

    SANITY CHECK (v3.5): in a logged run the fine sweep 'centered' on a
    reading WORSE than the coarse hit (9.9cm -> 14.0cm) — it had lost
    the block entirely and the approach then drove past the block into
    the ring line. If the sweep can't at least roughly reproduce the
    coarse distance, go back to the exact heading where the coarse scan
    saw the block instead of trusting the sweep."""
    coarse_yaw = imu.get_yaw()
    wiggle_turn(-FINE_STEP_DEG, TURN_EFFORT, timeout_s=1.5)
    best_yaw = imu.get_yaw()
    best_d = front_distance()
    for _ in range(FINE_SPAN_STEPS - 1):
        _abort()
        wiggle_turn(FINE_STEP_DEG, TURN_EFFORT, timeout_s=1.5)
        d = front_distance()
        if d < best_d:
            best_d = d
            best_yaw = imu.get_yaw()
    if best_d > coarse_d + 5.0:
        log("aim: sweep lost the block (best %.1fcm vs coarse %.1fcm) — "
            "reverting to coarse heading" % (best_d, coarse_d))
        turn_to_heading(coarse_yaw, TURN_EFFORT)
        return coarse_d
    turn_to_heading(best_yaw, TURN_EFFORT)
    log("aim: centered d=%.1fcm yaw=%+.1f" % (best_d, best_yaw))
    return best_d

def make_slip_checker():
    """'slip' after LOST_CONFIRM consecutive clean open-floor medians.
    Pressed-block readings (tiny values or 65535 timeouts) don't count,
    so sensor weirdness at contact can't false-trigger."""
    state = {"opens": 0}
    def check():
        if not SLIP_CHECK:
            return None
        d = front_distance()
        if BLOCK_LOST_MIN_CM < d < BLOCK_LOST_MAX_CM:
            state["opens"] += 1
            if state["opens"] >= LOST_CONFIRM:
                return "slip"
        else:
            state["opens"] = 0
        return None
    return check

def finish_sweep():
    """v4.7: after the main push trips the line, push the block FULLY
    out with two angled follow-up touches. Every touch ends on a line
    trip (sensors sit ahead of the wheels, so the wheels stay inside
    the ring), and every leg polls the supervisor abort.

    v4.11: THE RING-EXIT FIX. The old caps assumed the 10cm backups
    actually happened — the logs show them stalling at -1.5cm, after
    which a 17cm angled push from a spot right at the line drove the
    robot out of the circle. Each cap is now computed from the ACTUAL
    encoder-measured backup distance AND clamped by the geo-fence to
    the surveyed circle, whichever is smaller. A touch with no room
    is skipped outright."""
    c = math.cos(math.radians(SWEEP_ANGLE_DEG))

    # touch 1 happened (the main push). Back off, angle LEFT, touch.
    b1 = bounded_backup(SWEEP_BACK_CM, tag="sweep back1")
    wiggle_turn(SWEEP_ANGLE_DEG, TURN_EFFORT)
    cap1 = min((b1 + SWEEP_OVERDRIVE_CM) / c, fence_remaining())
    if cap1 < 1.0:
        log("sweep: no room for the LEFT touch (backed %.1fcm, fence "
            "%.1fcm) — skipping it" % (b1, fence_remaining()))
    else:
        log("sweep: LEFT %d deg, push to line (cap %.1fcm, backed "
            "%.1fcm)" % (SWEEP_ANGLE_DEG, cap1, b1))
        out = drive_watching(PUSH_EFFORT, None, max_cm=cap1,
                             hold_yaw=imu.get_yaw(), tag="sweepL")
        if out != "line":
            log("sweep: left touch ended on the cap (no sensor trip — "
                "block may be covering the tape); continuing safely")
    # back off, swing RIGHT 90 (net 45 right of the original heading),
    # touch the line once more from that side.
    b2 = bounded_backup(SWEEP_BACK_CM, tag="sweep back2")
    wiggle_turn(-2 * SWEEP_ANGLE_DEG, TURN_EFFORT)
    cap2 = min(b2 + SWEEP_OVERDRIVE_CM / c, fence_remaining())
    if cap2 < 1.0:
        log("sweep: no room for the RIGHT touch (backed %.1fcm, fence "
            "%.1fcm) — skipping it" % (b2, fence_remaining()))
    else:
        log("sweep: RIGHT %d deg, final push (cap %.1fcm, backed "
            "%.1fcm)" % (SWEEP_ANGLE_DEG, cap2, b2))
        out = drive_watching(PUSH_EFFORT, None, max_cm=cap2,
                             hold_yaw=imu.get_yaw(), tag="sweepR")
        if out != "line":
            log("sweep: right touch ended on the cap")

def push_out_one_block(cycle):
    """Find a block, drive to contact, push it over the tape, return to
    center. True = keep looping, False = ring is clear."""
    log("--- cycle %d --- batt=%.2fV pose=(%.1f, %.1f) center-dist=%.1f"
        % (cycle, battery_voltage(), POSE["x"], POSE["y"],
           dist_from_center()))
    dist = rotate_until_block()
    if dist is None:
        return False
    dist = fine_aim(dist)
    if not echo_inside_ring(dist):
        # v4.14: the fine aim can swing onto an out-of-ring object too
        log("aim: target at %.1fcm lies OUTSIDE the ring — not chasing "
            "it; rescanning" % dist)
        go_home()
        return True

    def near():
        return "contact" if front_distance() <= CONTACT_CM else None
    # v4.11: the approach is fenced too — an echo can never pull the
    # robot past the surveyed line (contact or the tape stop it first
    # in a normal run; the fence is the backstop).
    outcome = drive_watching(APPROACH_EFFORT, near,
                             max_cm=min(dist + 15.0, fence_remaining()),
                             hold_yaw=imu.get_yaw(), tag="approach")
    if outcome == "line":
        log("approach: tape before contact — echo was outside the ring")
        bounded_backup(RETURN_CM, tag="return")
        go_home()
        return True
    if outcome == "cap":
        log("approach: no contact where the echo was — rescanning")
        go_home()
        return True
    if outcome == "stall":
        log("approach: drive died even boosted — regrouping")
        go_home()
        return True

    push_yaw = imu.get_yaw()
    log("push: contact made, pushing along yaw %+.1f (fence %.1fcm)"
        % (push_yaw, fence_remaining(push_yaw)))
    outcome = drive_watching(PUSH_EFFORT, make_slip_checker(),
                             max_cm=min(RING["r"] * 1.2,
                                        fence_remaining(push_yaw)),
                             hold_yaw=push_yaw, tag="push")
    if outcome == "slip":
        log("push: block slipped off — regrouping")
        go_home()
        return True
    if outcome == "stall" and \
            dist_from_center() < RING["r"] - SENSOR_AHEAD_CM - 2.0:
        # stalled mid-ring (immovable pile / wedged block) — regroup.
        # A stall right AT the edge falls through to the boundary check
        # below and still scores.
        log("push: stalled mid-ring — regrouping")
        bounded_backup(RETURN_CM, tag="return")
        go_home()
        return True

    if outcome in ("cap", "stall") and \
            dist_from_center() >= RING["r"] - SENSOR_AHEAD_CM - 2.0:
        # Fence-stopped right at the ring edge: the block is over the
        # line even though the sensors never tripped (a block sitting
        # ON the tape blocks the view of it). Score it.
        log("push: geo fence stopped at the ring edge — counting as a "
            "line touch")
        outcome = "line"

    if outcome == "line":
        if FINISH_SWEEP:
            finish_sweep()           # angled touches push the block out
            log("cycle: SCORED — push + 3-touch sweep complete")
        else:
            # v4.13: line seen -> ONLY back up and turn away. Never
            # drive forward past the line.
            log("cycle: SCORED — block pushed to the line; backing off")

    bounded_backup(RETURN_CM, tag="return")
    go_home()
    return True

# --------------------------------- MAIN ----------------------------------

def wait_for_start(pestolink):
    """Idle (motors parked) until Button 0 / "A". USER button = backup."""
    connected = False
    while True:
        if pestolink.is_connected():
            if not connected:
                connected = True
                set_status(0, 0, 255)
                log("controller connected — armed")
            if pestolink.get_button(START_BUTTON):
                log("launch: Button %d" % START_BUTTON)
                break
        else:
            if connected:
                connected = False
                set_status(255, 120, 0)
                log("controller disconnected")
        if board.is_button_pressed():
            log("launch: USER button")
            break
        time.sleep(0.02)
    while pestolink.get_button(START_BUTTON) or board.is_button_pressed():
        time.sleep(0.02)

def run(sv=None):
    """Supervisor entry point (v4): main_code.py imports this module and
    calls run(self). Place the robot at RING CENTER before pressing the
    menu button — that press IS the launch. sv.check_abort() (START) can
    end the run at any moment; the supervisor catches MenuAbort and any
    crash, so this function just does the work. Crashes are logged here
    to sumo_log.txt too (and re-raised for the supervisor)."""
    global LINE_DELTA
    d = cal_derived_delta()
    if d is not None:
        LINE_DELTA = d
    if sv is not None:
        _HOOKS["abort"] = sv.check_abort
        # Supervisor launches skip the standalone boot path, so do the
        # log health check + a launch marker here — every run must leave
        # a reviewable trail in sumo_log.txt no matter how it started.
        log_selftest()
        log("===== SUMO launch (from supervisor menu) =====")
        log("wheels: %s" % _WHEEL_FIX)
    if d is not None:
        log("calibration: tapecal.txt -> line delta %.3f" % d)
    try:
        drivetrain.stop()
        if PADDLE_STOW:
            # Raise the paddle to a known angle so it can't drag on the
            # floor (the 8/6 run's backups all stalled — a low paddle
            # raking backward is the prime suspect), then cut power.
            try:
                servo_one.set_angle(PADDLE_STOW_DEG)
                time.sleep(0.4)
                servo_one.free()
                log("paddle stowed at %d deg" % PADDLE_STOW_DEG)
            except Exception:
                log("paddle stow skipped (no servo?)")
        # v4.21 IMU drift guard: pose, fence and every turn hang off
        # the gyro — a bad power-on bias (~25 deg/s parked drift in the
        # 8/7 log) makes the fence map garbage and the robot WILL exit
        # the ring. Fail loud instead.
        _y0 = imu.get_yaw()
        time.sleep(0.8)
        _rate = (imu.get_yaw() - _y0) / 0.8
        if abs(_rate) > 3.0:
            log("*** IMU DRIFTING %+.1f deg/s while parked — "
                "recalibrating... ***" % _rate)
            try:
                imu.calibrate(1)
            except Exception as e:
                log("imu.calibrate unavailable (%r)" % e)
            time.sleep(0.3)
            _y0 = imu.get_yaw()
            time.sleep(0.8)
            _rate = (imu.get_yaw() - _y0) / 0.8
            if abs(_rate) > 3.0:
                log("*** IMU STILL drifting %+.1f deg/s. POWER-CYCLE "
                    "the robot while it sits STILL, then relaunch. "
                    "Aborting run. ***" % _rate)
                set_status(255, 0, 0)
                time.sleep(1.5)
                return
            log("IMU recalibrated — drift now %+.1f deg/s" % _rate)
        POSE["x"] = POSE["y"] = 0.0
        RING["r"] = RING_RADIUS_CM
        RING["surveyed"] = False
        FLOOR["hits"] = 0
        FLOOR["lo"] = 1.0

        pushes = 0
        start = time.ticks_ms()
        try:
            log("run start: batt=%.2fV yaw=%+.1f"
                % (battery_voltage(), imu.get_yaw()))
            log("config v4.22: appr=%.2f push=%.2f turn=%.2f backup=%.2f "
                "ring=%.0fcm scan<%.0fcm contact<%.0fcm linedelta=%.2f "
                "deb=%d sweep=%s survey=%s fence=%.0fcm"
                % (APPROACH_EFFORT, PUSH_EFFORT, TURN_EFFORT,
                   BACKUP_EFFORT, RING_RADIUS_CM, SCAN_MAX_CM,
                   CONTACT_CM, LINE_DELTA, LINE_DEBOUNCE,
                   FINISH_SWEEP, SURVEY, FENCE_MARGIN_CM))
            if battery_voltage() < LOW_BATT_V:
                log("*** WARNING: battery LOW at launch (%.2fV, %d-cell) "
                    "***" % (battery_voltage(), BATT_CELLS))
            capture_floor_baseline()     # start spot = off the tape

            if SURVEY:
                # v4.11: measure the real circle and set Home at its
                # center before hunting blocks. Place the robot roughly
                # mid-ring facing any direction; it finds the rest.
                survey_ring()

            while pushes < MAX_PUSHES:
                _abort()
                if time.ticks_diff(time.ticks_ms(), start) \
                        > MAX_RUNTIME_S * 1000:
                    log("time cap %.0fs reached" % MAX_RUNTIME_S)
                    break
                if not push_out_one_block(pushes + 1):
                    break
                pushes += 1
        except Exception as e:
            if type(e).__name__ != "MenuAbort":   # aborts aren't crashes
                log_exception(e)
            raise
        finally:
            drivetrain.stop()
            set_status(0, 255, 0)
            log("DONE: %d push cycles, batt=%.2fV"
                % (pushes, battery_voltage()))
    finally:
        _HOOKS["abort"] = lambda: None            # detach the hook

# ------------------------- STANDALONE OPERATION --------------------------

def _standalone():
    from pestolink import PestoLinkAgent
    log_selftest()                   # RED LED at boot = log file broken
    log("")
    log("=========== BOOT: sumo_auto v4.22 (standalone) ====== batt=%.2fV"
        % battery_voltage())
    if battery_voltage() < LOW_BATT_V:
        log("*** WARNING: battery LOW for a %d-cell pack (<%.1fV). "
            "Motors will stall and the board may brown out (LED goes "
            "dark). Swap in fresh/charged cells. ***"
            % (BATT_CELLS, LOW_BATT_V))

    pestolink = PestoLinkAgent(ROBOT_NAME)
    board.led_on()
    set_status(255, 120, 0)
    drivetrain.stop()
    wait_for_start(pestolink)
    set_status(0, 255, 255)
    time.sleep(0.5)
    try:
        run()
    finally:
        board.led_blink(4)

if __name__ == "__main__":
    _standalone()

# ------------------------------ TEST NOTES --------------------------------
# Reading sumo_log.txt after a bad run:
#  * Log stops mid-phase, NO traceback, NO "DONE", and the next lines are
#    a fresh BOOT -> the board reset (brownout). Watch the batt= values
#    in the heartbeats just before it died; anything sagging toward ~6V
#    under load means the pack is done. Fresh batteries.
#  * "TRACEBACK:" at the end -> software crash. Send me the log.
#  * "DONE" printed but the robot quit earlier than expected -> read the
#    scan/approach/push lines to see which check fired (scan found
#    nothing? approach capped? slip?) — those are tuning constants.
# Other checks:
#  * Run over USB, not the IDE's Bluetooth link — PestoLink takes the BLE
#    radio and drops the IDE connection, which kills the program.
#  * The scan "wiggle" (short 12-degree steps with pauses) is normal.
#  * Hold a block against the sensor face and check what sensor_log.py
#    reads pressed; if it's inside 15-150cm, tighten BLOCK_LOST_MIN/MAX
#    or set SLIP_CHECK = False.
#  * Line detection is now relative to the floor baseline sampled at
#    launch. Check the "floor baseline" log line, then slide the robot
#    onto the ring tape and confirm the reflectance values differ from
#    that baseline by more than LINE_DELTA (0.20). If the tape/floor
#    contrast is smaller than that, lower LINE_DELTA; once you know
#    whether the tape reads lighter or darker, set LINE_POLARITY to
#    lock out false trips in the other direction.
#  * Sensor face must meet the 2" block squarely — never push on the
#    PCB edge or pins; a flat Lego face flush in front is fine if it
#    stays below the beam.
