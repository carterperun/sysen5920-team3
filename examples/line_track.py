"""
line_track.py — SYSEN 5920 Team 3
The SIMPLE line follower (team original, supervisor-compatible) — v4.

Raw reflectance readings and a P-controller — no calibration. Works on
the competition surface: tape LIGHT (~0.05) on DARK floor (~0.7).

v4.5 (8/6 night logs — "never detects the line"): THE BLUE PAINTER'S
TAPE BARELY REGISTERS. Sumo's heartbeats show it reading ~0.88-0.90
against a ~0.96 black floor — a contrast of ~0.07, while this follower
demanded a drop below a fixed 0.45. It could NEVER trip. v4.5 goes
fully floor-relative:
  * START ON THE BLACK FLOOR (this is now the expected launch): the
    robot samples its own floor for a moment, then creeps forward up
    to 40cm; tape = a sensor dropping TAPE_DELTA (0.05) below its own
    floor baseline. Works for faint blue tape AND high-contrast tape.
  * Steering is normalized by the MEASURED floor-to-tape contrast
    (learned at first tape contact), so the P-gain has the same
    authority on faint tape as it had on bright tape.
  * Lost = both sensors back within LOST_DELTA (0.03) of the floor
    baseline; the wide ambiguous band in between keeps following.

v4.4 (from the 8/6 field logs — "didn't move at start, went back to
menu"):
  * DRIVE-ON START: the placement guard was refusing launches ("NOT ON
    THE LINE at start") — including runs where the left sensor read
    0.456, a hair over the 0.45 threshold, with the robot basically on
    the tape. Refusing was the wrong move. Now, if the robot isn't
    clearly on the tape at launch, it CREEPS FORWARD up to 25cm hunting
    for it (per the team's original plan: start on the floor, drive
    onto the line) and only gives up if the creep finds nothing.
  * NOTE from the same logs: every failed run printed "base=0.42" —
    that's v4.2 STILL ON THE ROBOT. v4.3+ prints its version in the
    start line. Check for "LINE start v4.4" after uploading!

v4.3 (from the 8/6 late logs):
  * FALSE END-OF-LINE FIX: two runs declared "END" at 17-20cm total.
    The logs show readings hovering 0.45-0.55 — right ON the old
    FLOOR_MIN=0.45 knife edge — so the detector flickered lost/found
    three times within a few cm of the start and the 3-strikes end
    rule fired. Two fixes: (1) HYSTERESIS — "lost" now needs BOTH
    sensors clearly on floor (>0.58), while "found" still means <0.45;
    the ambiguous band no longer counts as lost. (2) The end rule is
    ignored before END_MIN_TRAVEL_CM (30cm) — early flicker just
    triggers a normal backup-refind.
  * SLOW SPIN-UP FIX: a LAUNCH KICK drives the first 0.35s at 0.6
    effort to break stiction (0.42 from rest just hummed), the stall
    boost now fires at 0.4s instead of 0.7s, and cruise is nudged
    0.42 -> 0.48 (the v4.2 steer clamp is what tamed the old
    erratic-at-0.45 behavior, so cruise can afford it).

v4 (from the 8/6 evening test — "erratic turns lose the line"):
  * SLOWER CRUISE: base effort 0.45 -> 0.35, per-wheel floor relaxed to
    0.3 to allow it. Gentler speed = gentler corrections = fewer
    blow-through-the-corner moments.
  * ADAPTIVE STALL BOOST: if forward progress stops for 0.7s, effort
    rises to 0.5 until the robot has been moving again for ~0.5s, then
    drops back to cruise. Slow when tracking, strong when stuck.
  * BACKUP-TO-REFIND: a fully lost line no longer means "keep driving
    forward blind" — the robot STOPS and reverses straight until a
    sensor sees tape again, then resumes following.
  * END-OF-LINE now = three lost-line events clustered within a few cm
    of each other (the robot keeps refinding tape behind the same spot
    — that's the line's end, not a miss). Then: the 15cm finish drive.
  * The 1.5s hard-stuck backup (reverse 5cm ON the line + charge) and
    the anti-friction wiggle are unchanged from v3.

Supervisor use: main_code.py MENU -> B imports this, calls run(sv);
the B press is the start, START aborts anywhere (all loops poll it).
Standalone: running the file directly starts following immediately.
"""

from XRPLib.defaults import *
import time

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


# ------------------------------- LOGGING ---------------------------------
# v4.1: line_track gets its own BLACK-BOX LOG (line_log.txt on flash),
# same scheme as sumo — timestamped, flushed per line, auto-rotated.
# Retrieve via the IDE file browser. Console print stays too.

LOG_TO_FILE = True
LOG_PATH = "LOG.TXT"           # unified log — every program appends here
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
    line = "[%9.2fs][LINE ] %s" % (t, msg)
    if console:
        # v4.2: heartbeats skip the console — an attached BLE console
        # blocks ~1s per print, which starved the control loop and blinded
        # the line detector. File logging keeps the full record.
        print(line)
    if LOG_TO_FILE:
        try:
            f = open(LOG_PATH, "a")
            f.write(line + "\n")
            f.close()
        except Exception as e:
            if not _LOG_BROKEN["reported"]:
                _LOG_BROKEN["reported"] = True
                print("*** LINE LOG WRITE FAILED: %r ***" % e)

def _rotate_log():
    try:
        import os
        if os.stat(LOG_PATH)[6] > 300 * 1024:
            os.remove(LOG_PATH)
            print("line log rotated")
    except Exception:
        pass

COUNTS = {"boosts": 0, "backups": 0, "refinds": 0}

def cal_derived_delta():
    """v4.6: buffered trip delta from tapecal.txt (supervisor MENU +
    LEFT TRIGGER calibration), or None. Half the measured contrast."""
    try:
        with open("tapecal.txt") as f:
            fl, fr, tl, tr = [float(x) for x in f.read().split(",")]
        c = (fl + fr) / 2 - (tl + tr) / 2
        if c <= 0.02:
            return None
        return min(0.15, max(0.025, c * 0.5))
    except Exception:
        return None

# ----------------------------- CONFIGURATION -----------------------------

BASE_EFFORT = 0.48          # v4.3: nudged up again — 0.42 was still
                            # crawling; the steer clamp (not the slow
                            # cruise) was the real erratic-turn fix
BOOST_EFFORT = 0.6          # stall boost: cruise rises to this when
                            # progress stops (see STALL_BOOST_S)
LAUNCH_KICK_S = 0.35        # v4.3: drive at least this hard for the
LAUNCH_KICK_EFFORT = 0.6    # first moments — breaks stiction from rest
                            # instead of humming at cruise effort
MIN_EFFORT = 0.3            # per-wheel floor, relaxed 0.4 -> 0.3 so the
                            # slower cruise is actually commandable —
                            # stalls are now handled by the boost tier
                            # instead of a high floor
KP = 0.6                    # steering gain (team-tuned)
STEER_LIMIT = 0.35          # v4.2: clamp the correction — saturated
                            # steering (0.5+) reversed the inside wheel
                            # and fishtailed the robot off the line
STEER_SIGN = -1             # flipped for this build (8/6 test)
LOOP_S = 0.01

# v4.5: FLOOR-RELATIVE detection (fixed thresholds can't see the blue
# painter's tape — contrast is only ~0.07 on this floor).
TAPE_DELTA = 0.05           # sensor this far BELOW its floor baseline
                            # = on tape
LOST_DELTA = 0.03           # BOTH sensors within this of the baseline
                            # = clearly back on bare floor (lost)
BASE = {"l": 0.93, "r": 0.93, "span": 0.5, "tape_lo": 1.0}
                            # l/r: floor baselines sampled at launch;
                            # span: floor-to-tape contrast (learned at
                            # first tape contact) used to normalize
                            # steering; tape_lo: darkest tape seen

# Anti-friction wiggle on hard corrections (unchanged)
WIGGLE = True
WIGGLE_STEER_MIN = 0.25
WIGGLE_BIAS = 0.08          # v4.2: calmed (0.12 fed the snaking)
WIGGLE_PERIOD_S = 0.2

MAX_RUNTIME_S = 240

# Adaptive stall boost (v4)
STALL_BOOST_S = 0.4         # v4.3: 0.7 -> 0.4, boost sooner when stuck
MOVING_AGAIN_S = 0.5        # progress within this window -> back to cruise
STUCK_MIN_CM = 1.0          # "progress" = at least this much forward

# Hard-stuck bump recovery (v3, unchanged): fires if boosting alone
# hasn't produced progress by 1.5s
STUCK_WINDOW_S = 1.5
BACKUP_CM = 5.0
BACKUP_EFFORT = 0.6         # v4.2: reverse needs more than forward
BACKUP_TIMEOUT_S = 2.5
CHARGE_BOOST = 0.15
CHARGE_CM = 10.0

# Lost-line refind (v4)
LOST_DEBOUNCE_S = 0.15      # both-on-floor this long = really lost
REFIND_BACK_CM = 15.0       # reverse up to this far hunting for tape
REFIND_EFFORT = 0.65        # v4.2: 0.4 stalled on this drivetrain
REFIND_TIMEOUT_S = 2.5
END_ZONE_CM = 8.0           # 3 lost events within this span = line END
END_MIN_TRAVEL_CM = 30.0    # v4.3: the end rule is OFF before this much
                            # travel — sensor flicker near the start was
                            # faking "END" 2-4cm in

# Drive-on start (v4.4/v4.5): START ON THE BLACK FLOOR, aimed at the
# line — the robot creeps forward and latches on when it sees the tape
FIND_FWD_CM = 40.0          # v4.5: 25 -> 40
FIND_EFFORT = 0.5
FIND_TIMEOUT_S = 4.0

# End-of-line finish (v3, unchanged)
FINISH_FWD_CM = 15.0
FINISH_EFFORT = 0.45
FINISH_TIMEOUT_S = 3.0

LOG_PERIOD_MS = 1000

# ------------------------------- HELPERS ---------------------------------

def _avg_pos():
    return (drivetrain.get_left_encoder_position() +
            drivetrain.get_right_encoder_position()) / 2

def _floor_eff(e):
    if 0 < e < MIN_EFFORT:
        return MIN_EFFORT
    if -MIN_EFFORT < e < 0:
        return -MIN_EFFORT
    return e

def _capture_floor():
    """Sample the bare-floor baseline. The robot MUST start on the
    black floor (not the tape) — that's the launch procedure."""
    l = r = 0.0
    n = 10
    for _ in range(n):
        l += reflectance.get_left()
        r += reflectance.get_right()
        time.sleep(0.02)
    BASE["l"], BASE["r"] = l / n, r / n
    BASE["span"] = 0.5
    BASE["tape_lo"] = 1.0

def _on_tape(vl, vr):
    """Either sensor clearly below its own floor baseline."""
    return (vl < BASE["l"] - TAPE_DELTA) or (vr < BASE["r"] - TAPE_DELTA)

def _both_floor(vl, vr):
    """BOTH sensors back at bare-floor level = line lost."""
    return (vl > BASE["l"] - LOST_DELTA) and (vr > BASE["r"] - LOST_DELTA)

def _note_tape(vl, vr):
    """Learn the real floor-to-tape contrast from the darkest tape
    reading seen — steering authority scales off this."""
    lo = vl if vl < vr else vr
    if lo < BASE["tape_lo"]:
        BASE["tape_lo"] = lo
        span = (BASE["l"] + BASE["r"]) / 2 - lo
        if span < 0.05:
            span = 0.05
        if abs(span - BASE["span"]) > 0.02:
            BASE["span"] = span
            log("line_track: contrast learned — tape %.3f vs floor "
                "%.3f, span %.3f" % (lo, (BASE["l"] + BASE["r"]) / 2,
                                     span), console=False)

def _steer_from_sensors():
    left = reflectance.get_left()
    right = reflectance.get_right()
    # v4.5: normalize by the measured contrast so faint blue tape gets
    # the same steering authority as bright tape
    err = STEER_SIGN * (right - left) / BASE["span"]
    return err * KP, left, right

def _backup_on_line(abort):
    """Hard-stuck: reverse BACKUP_CM while still steering on the line."""
    start = _avg_pos()
    t0 = time.ticks_ms()
    COUNTS["backups"] += 1
    log("line_track: STUCK — backing up %.0fcm (batt=%.2fV)"
        % (BACKUP_CM, battery_voltage()))
    while True:
        abort()
        if start - _avg_pos() >= BACKUP_CM:
            break
        if time.ticks_diff(time.ticks_ms(), t0) > BACKUP_TIMEOUT_S * 1000:
            log("line_track: backup stalled too — pressing on")
            break
        steer, _, _ = _steer_from_sensors()
        drivetrain.set_effort(_floor_eff(-BACKUP_EFFORT - steer),
                              _floor_eff(-BACKUP_EFFORT + steer))
        time.sleep(LOOP_S)
    drivetrain.stop()

def _refind_backwards(abort):
    """Line fully lost: reverse straight until a sensor sees tape.
    True = tape refound; False = gave up (distance/time cap)."""
    start = _avg_pos()
    t0 = time.ticks_ms()
    COUNTS["refinds"] += 1
    log("line_track: line LOST at %.0fcm — backing up to refind "
        "(L=%.3f R=%.3f)" % (_avg_pos(), reflectance.get_left(),
                             reflectance.get_right()))
    try:
        while True:
            abort()
            left = reflectance.get_left()
            right = reflectance.get_right()
            if _on_tape(left, right):
                log("line_track: refound tape after %.1fcm back"
                      % (start - _avg_pos()))
                return True
            if start - _avg_pos() >= REFIND_BACK_CM:
                log("line_track: no tape within %.0fcm back — giving up"
                      % REFIND_BACK_CM)
                return False
            if time.ticks_diff(time.ticks_ms(), t0) \
                    > REFIND_TIMEOUT_S * 1000:
                log("line_track: refind backup stalled — giving up")
                return False
            drivetrain.set_effort(-REFIND_EFFORT, -REFIND_EFFORT)
            time.sleep(LOOP_S)
    finally:
        drivetrain.stop()

def _drive_onto_line(abort):
    """v4.5: standard launch — start on the black floor, creep straight
    forward until a sensor drops below the floor baseline (= the blue
    tape). True = on the line now."""
    start = _avg_pos()
    t0 = time.ticks_ms()
    log("line_track: creeping forward up to %.0fcm to find the tape "
        "(floor base L=%.3f R=%.3f, trip delta %.2f)"
        % (FIND_FWD_CM, BASE["l"], BASE["r"], TAPE_DELTA))
    try:
        while True:
            abort()
            vl = reflectance.get_left()
            vr = reflectance.get_right()
            if _on_tape(vl, vr):
                _note_tape(vl, vr)
                log("line_track: found the tape after %.1fcm "
                    "(L=%.3f R=%.3f)" % (_avg_pos() - start, vl, vr))
                return True
            if _avg_pos() - start >= FIND_FWD_CM:
                log("*** no tape within %.0fcm ahead — check placement "
                    "and relaunch ***" % FIND_FWD_CM)
                return False
            if time.ticks_diff(time.ticks_ms(), t0) \
                    > FIND_TIMEOUT_S * 1000:
                log("*** creep-to-line stalled/timed out — check "
                    "placement and relaunch ***")
                return False
            drivetrain.set_effort(FIND_EFFORT, FIND_EFFORT)
            time.sleep(LOOP_S)
    finally:
        drivetrain.stop()

def _finish_forward(abort, from_pos):
    """End of line: drive to FINISH_FWD_CM past `from_pos`, then stop."""
    t0 = time.ticks_ms()
    log("line_track: END of line — finishing %.0fcm past last tape"
          % FINISH_FWD_CM)
    while True:
        abort()
        if _avg_pos() - from_pos >= FINISH_FWD_CM:
            break
        if time.ticks_diff(time.ticks_ms(), t0) > FINISH_TIMEOUT_S * 1000:
            log("line_track: finish drive timed out")
            break
        drivetrain.set_effort(FINISH_EFFORT, FINISH_EFFORT)
        time.sleep(LOOP_S)
    drivetrain.stop()
    log("line_track: DONE (~%.0f cm total)" % _avg_pos())

# ------------------------------ FOLLOWER ---------------------------------

def run(sv=None):
    """Entry point for the main_code supervisor (or standalone)."""
    abort = sv.check_abort if sv else (lambda: None)
    global TAPE_DELTA, LOST_DELTA
    _cal = cal_derived_delta()
    if _cal is not None:
        TAPE_DELTA = _cal
        LOST_DELTA = max(0.02, _cal * 0.6)
    _rotate_log()
    COUNTS["boosts"] = COUNTS["backups"] = COUNTS["refinds"] = 0
    # v4.5: STANDARD LAUNCH = robot parked on the BLACK FLOOR aimed at
    # the line. Sample the floor, then creep forward onto the tape.
    _capture_floor()
    if _cal is not None:
        log("calibration: tapecal.txt -> tape delta %.3f, lost delta "
            "%.3f" % (TAPE_DELTA, LOST_DELTA))
    log("wheels: %s" % _WHEEL_FIX)
    log("=== LINE start v4.7: base=%.2f boost=%.2f kick=%.2f kp=%.2f "
        "sign=%+d tapedelta=%.2f lostdelta=%.2f endmin=%.0fcm "
        "batt=%.2fV floor L=%.3f R=%.3f"
        % (BASE_EFFORT, BOOST_EFFORT, LAUNCH_KICK_EFFORT, KP,
           STEER_SIGN, TAPE_DELTA, LOST_DELTA, END_MIN_TRAVEL_CM,
           battery_voltage(), BASE["l"], BASE["r"]))
    if not _on_tape(reflectance.get_left(), reflectance.get_right()):
        if not _drive_onto_line(abort):
            return
    lost_since = None
    lost_pos = 0.0
    lost_events = []             # positions where the line was lost
    last_log = 0
    start_ms = time.ticks_ms()
    wiggle_ms = start_ms
    wiggle_bias = WIGGLE_BIAS
    drivetrain.reset_encoder_position()

    prog_ms = start_ms           # last time we saw >=1cm of progress
    prog_pos = 0.0
    charge_until = None
    boosted = False

    try:
        while True:
            abort()
            now = time.ticks_ms()
            if time.ticks_diff(now, start_ms) > MAX_RUNTIME_S * 1000:
                log("line_track: time cap reached")
                return

            steer, left, right = _steer_from_sensors()
            if steer > STEER_LIMIT:
                steer = STEER_LIMIT
            elif steer < -STEER_LIMIT:
                steer = -STEER_LIMIT
            pos = _avg_pos()

            if _on_tape(left, right):
                _note_tape(left, right)      # keep learning contrast

            # ---------- lost line: refind backwards / detect the end ----
            # v4.5: lost = BOTH sensors back at the floor baseline; the
            # wide ambiguous band in between keeps following
            if _both_floor(left, right):
                if lost_since is None:
                    lost_since = now
                    lost_pos = pos           # where tape was last seen
                elif time.ticks_diff(now, lost_since) \
                        > LOST_DEBOUNCE_S * 1000:
                    drivetrain.stop()
                    nearby = [p for p in lost_events
                              if abs(lost_pos - p) <= END_ZONE_CM]
                    if len(nearby) >= 2 and lost_pos >= END_MIN_TRAVEL_CM:
                        # third loss at the same spot, far enough in:
                        # that IS the end (v4.3: early flicker no
                        # longer qualifies)
                        _finish_forward(abort, lost_pos)
                        return
                    lost_events.append(lost_pos)
                    if not _refind_backwards(abort):
                        log("line_track: stopping (line not refound)")
                        return
                    lost_since = None
                    prog_ms = time.ticks_ms()
                    prog_pos = _avg_pos()
                    continue
            else:
                lost_since = None

            # ---------- progress tracking / adaptive speed ----------
            if pos - prog_pos >= STUCK_MIN_CM:
                prog_ms = now
                prog_pos = pos
            stalled_ms = time.ticks_diff(now, prog_ms)
            if stalled_ms >= STUCK_WINDOW_S * 1000:
                # boosting didn't free it — the bigger hammer
                _backup_on_line(abort)
                charge_until = _avg_pos() + CHARGE_CM
                prog_ms = time.ticks_ms()
                prog_pos = _avg_pos()
                lost_since = None
                continue
            if not boosted and stalled_ms >= STALL_BOOST_S * 1000:
                boosted = True
                COUNTS["boosts"] += 1
                log("line_track: stalled at %.0fcm — boosting to %.2f "
                    "(batt=%.2fV)" % (pos, BOOST_EFFORT,
                                      battery_voltage()))
            elif boosted and stalled_ms <= MOVING_AGAIN_S * 1000:
                boosted = False
                log("line_track: moving — back to cruise %.2f"
                      % BASE_EFFORT)

            base = BOOST_EFFORT if boosted else BASE_EFFORT
            # v4.3 launch kick: break stiction from rest instead of
            # humming at cruise effort for the first second
            if time.ticks_diff(now, start_ms) < LAUNCH_KICK_S * 1000 \
                    and base < LAUNCH_KICK_EFFORT:
                base = LAUNCH_KICK_EFFORT
            if charge_until is not None:
                if pos < charge_until:
                    base += CHARGE_BOOST
                else:
                    charge_until = None

            l_eff = base - steer
            r_eff = base + steer

            if WIGGLE and abs(steer) > WIGGLE_STEER_MIN:
                if time.ticks_diff(now, wiggle_ms) \
                        >= WIGGLE_PERIOD_S * 1000:
                    wiggle_ms = now
                    wiggle_bias = -wiggle_bias
                l_eff += wiggle_bias
                r_eff += wiggle_bias

            drivetrain.set_effort(_floor_eff(l_eff), _floor_eff(r_eff))

            if time.ticks_diff(now, last_log) >= LOG_PERIOD_MS:
                last_log = now
                log("hb L=%.3f R=%.3f steer=%+.3f pos=%.0fcm "
                    "batt=%.2fV%s%s"
                    % (left, right, steer, pos, battery_voltage(),
                       " BOOST" if boosted else "",
                       " CHARGE" if charge_until is not None else ""),
                    console=False)

            time.sleep(LOOP_S)
    finally:
        drivetrain.stop()
        log("=== LINE summary: %.0fcm in %.1fs — boosts=%d backups=%d "
            "refinds=%d batt=%.2fV"
            % (_avg_pos(),
               time.ticks_diff(time.ticks_ms(), start_ms) / 1000,
               COUNTS["boosts"], COUNTS["backups"], COUNTS["refinds"],
               battery_voltage()))

if __name__ == "__main__":
    run()
