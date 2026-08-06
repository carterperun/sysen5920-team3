"""
line_track.py — SYSEN 5920 Team 3
The SIMPLE line follower (team original, supervisor-compatible) — v4.

Raw reflectance readings and a P-controller — no calibration. Works on
the competition surface: tape LIGHT (~0.05) on DARK floor (~0.7).

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

# ------------------------------- LOGGING ---------------------------------
# v4.1: line_track gets its own BLACK-BOX LOG (line_log.txt on flash),
# same scheme as sumo — timestamped, flushed per line, auto-rotated.
# Retrieve via the IDE file browser. Console print stays too.

LOG_TO_FILE = True
LOG_PATH = "line_log.txt"
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
        if os.stat(LOG_PATH)[6] > 200 * 1024:
            os.remove(LOG_PATH)
            print("line log rotated")
    except Exception:
        pass

COUNTS = {"boosts": 0, "backups": 0, "refinds": 0}

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

FLOOR_MIN = 0.45            # reading BELOW this = sensor clearly on tape
LOST_MIN = 0.58             # v4.3 hysteresis: "lost" only when BOTH
                            # sensors read above this — the 0.45-0.58
                            # band (the flickery readings in the logs)
                            # counts as neither lost nor found

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

def _steer_from_sensors():
    left = reflectance.get_left()
    right = reflectance.get_right()
    return STEER_SIGN * (right - left) * KP, left, right

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
            if left < FLOOR_MIN or right < FLOOR_MIN:
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
    _rotate_log()
    COUNTS["boosts"] = COUNTS["backups"] = COUNTS["refinds"] = 0
    log("=== LINE start v4.3: base=%.2f boost=%.2f kick=%.2f kp=%.2f "
        "sign=%+d lost>%.2f endmin=%.0fcm batt=%.2fV L=%.3f R=%.3f"
        % (BASE_EFFORT, BOOST_EFFORT, LAUNCH_KICK_EFFORT, KP,
           STEER_SIGN, LOST_MIN, END_MIN_TRAVEL_CM, battery_voltage(),
           reflectance.get_left(), reflectance.get_right()))
    if (reflectance.get_left() > FLOOR_MIN and
            reflectance.get_right() > FLOOR_MIN):
        log("*** NOT ON THE LINE at start (both sensors read floor). "
            "Place the sensors over the tape and relaunch. ***")
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

            # ---------- lost line: refind backwards / detect the end ----
            # v4.3: LOST_MIN (not FLOOR_MIN) — hysteresis keeps the
            # ambiguous 0.45-0.58 band from flickering into "lost"
            if left > LOST_MIN and right > LOST_MIN:
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
