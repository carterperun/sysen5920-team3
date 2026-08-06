"""
sumo_auto.py — SYSEN 5920 Team 3
XRP Proving Ground: "Sumo" challenge (autonomous attempt) — v3.7

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
from pestolink import PestoLinkAgent
from machine import Pin, ADC
import time
import math
import sys

# ----------------------------- CONFIGURATION -----------------------------

ROBOT_NAME = "T3amThr3"     # BLE name shown in PestoLink (8 chars max)
START_BUTTON = 0            # "A" on most gamepads (verify in the tester)

# Ring geometry — v3.5: resized from the logged runs. The successful
# cycle-2 push hit the ring line ~25cm from center, so this ring is
# small (~30cm radius, not 60). MEASURE the real ring and update.
RING_RADIUS_CM = 30.0       # ring radius; caps pushes and homing
SCAN_MAX_CM = 28.0          # accept scan echoes closer than this. Keep
                            # it inside the ring radius so blocks already
                            # out (and anything beyond the ring) are
                            # never targeted.
CONTACT_CM = 5.0            # reading at/below this = block is on the nose
PUSH_OVER_CM = 3.0          # extra shove so the block fully crosses
                            # (some coast past the trip point is normal;
                            # wheels must stay on our side of the tape)
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
LINE_DELTA = 0.20           # deviation from floor baseline = tape
LINE_POLARITY = "lighter"   # LOCKED from the logs: floor reads ~0.7,
                            # tape trips read ~0.05 — the ring tape is
                            # much LIGHTER than the floor. Locking the
                            # polarity stops dark spots/shadows on the
                            # floor from ever tripping the detector.
LINE_DEBOUNCE = 2           # consecutive reads needed to trip (~20ms)

# Motion — v3.4: doubled efforts (v3.3) outran the line detector, so
# efforts are now original + 0.1. ALSO fixed the real problem: the slip
# checker's ultrasonic reads (~45ms each) were throttling the whole
# drive loop, so the tape was only checked every ~65ms — at speed, the
# tape could pass between checks. Slow sensors now run on their own
# slower schedule; the line check runs every ~10ms pass.
APPROACH_EFFORT = 0.45
PUSH_EFFORT = 0.38
TURN_EFFORT = 0.5
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
LOG_PATH = "sumo_log.txt"
HEARTBEAT_S = 0.5           # heartbeat period inside drive loops

# ------------------------------- LOGGING ---------------------------------

_BOOT_MS = time.ticks_ms()

def battery_voltage():
    try:
        return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)
    except Exception:
        return -1.0

_LOG_BROKEN = {"reported": False}

def log(msg):
    """Timestamped line to console AND flash file, flushed immediately so
    the last lines survive a crash or power loss. v3.6: a file-write
    failure is now REPORTED (console + the boot self-test LED) instead of
    silently swallowed — a brownout mid-write can corrupt the filesystem
    and previously that just made the log go quiet."""
    t = time.ticks_diff(time.ticks_ms(), _BOOT_MS) / 1000
    line = "[%8.2fs] %s" % (t, msg)
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
FLOOR = {"l": 0.5, "r": 0.5, "hits": 0}

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

def turn_to_heading(target_yaw, effort):
    """Rotate to an absolute IMU yaw AND VERIFY IT GOT THERE. The logged
    runs showed stalled turns silently timing out, after which go_home()
    drove its whole 'home' leg in the wrong direction and the pose map
    fell apart. Now each turn is checked against the IMU and re-commanded
    with escalating effort until it's within TURN_TOL_DEG."""
    for attempt in range(TURN_RETRIES):
        delta = normalize(target_yaw - imu.get_yaw())
        if abs(delta) <= TURN_TOL_DEG:
            return True
        boosted = min(0.9, effort + 0.15 * attempt)
        drivetrain.turn(delta, boosted, timeout=3)
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

def record_move(dist_cm):
    rad = math.radians(imu.get_yaw())
    POSE["x"] += dist_cm * math.cos(rad)
    POSE["y"] += dist_cm * math.sin(rad)

def straight_tracked(dist_cm, effort, timeout=4):
    """Straight move that records what the ENCODERS say actually
    happened, not what was commanded — a stalled/timed-out move used to
    poison the pose map with distance the robot never drove."""
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    drivetrain.straight(dist_cm, effort, timeout=timeout)
    actual = traveled_since(start_l, start_r)
    record_move(actual)
    if abs(actual - dist_cm) > 5:
        log("straight: commanded %.1fcm, drove %.1fcm (stall?)"
            % (dist_cm, actual))

def drive_watching(effort, stop_check, max_cm, hold_yaw=None, tag="drive"):
    """Drive forward until stop_check() returns a reason, the tape shows
    up ('line'), or max_cm is hit ('cap'). IMU heading hold keeps pushes
    straight. Logs a heartbeat ~2/s so the log shows what the sensors saw
    right up to the moment anything stopped."""
    start_l = drivetrain.get_left_encoder_position()
    start_r = drivetrain.get_right_encoder_position()
    last_beat = time.ticks_ms()
    last_slow = 0
    result = "cap"
    while True:
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
        if traveled_since(start_l, start_r) > max_cm:
            log("%s: cap %.1fcm reached" % (tag, max_cm))
            break
        if hold_yaw is not None:
            err = normalize(hold_yaw - imu.get_yaw())
            corr = max(-PUSH_CORR_MAX, min(PUSH_CORR_MAX, PUSH_KP * err))
            drivetrain.set_effort(effort - corr, effort + corr)
        else:
            drivetrain.set_effort(effort, effort)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_beat) >= HEARTBEAT_S * 1000:
            last_beat = now
            l, r = line_values()
            # NOTE: no front_distance() here — a 45ms ultrasonic read in
            # the heartbeat would blind the line check for ~2cm of travel
            log("%s: hb L=%.3f R=%.3f yaw=%+.1f trav=%.1fcm batt=%.2fV"
                % (tag, l, r, imu.get_yaw(),
                   traveled_since(start_l, start_r), battery_voltage()))
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

# ------------------------------ BEHAVIORS --------------------------------

def rotate_until_block():
    """Coarse: rotate in place, stop at the FIRST echo within SCAN_MAX_CM.

    v3.6: MOTOR-ALIVE CHECK. A frozen 'cyan light, nothing happening'
    run turned out to be turns silently timing out (dead pack / motor
    switch off). Now, if the first scan turn produces almost no yaw
    change, one louder retry is made — and if the robot STILL hasn't
    rotated, the run aborts with a red LED and a loud log line instead
    of pretending to scan for a minute."""
    for step in range(FULL_SWEEP_STEPS):
        d = front_distance()
        if d < SCAN_MAX_CM:
            log("scan: hit at step %d, d=%.1fcm yaw=%+.1f"
                % (step, d, imu.get_yaw()))
            return d
        yaw_before = imu.get_yaw()
        drivetrain.turn(SCAN_STEP_DEG, TURN_EFFORT, timeout=2)
        if step == 0 and abs(imu.get_yaw() - yaw_before) < 3.0:
            log("scan: first turn didn't move (yaw %+.1f -> %+.1f) — "
                "retrying at full effort"
                % (yaw_before, imu.get_yaw()))
            drivetrain.turn(SCAN_STEP_DEG, 0.9, timeout=2)
            if abs(imu.get_yaw() - yaw_before) < 3.0:
                log("*** MOTORS NOT MOVING. Check the motor power "
                    "switch and battery pack. Aborting run. ***")
                set_status(255, 0, 0)        # red = motors dead
                return None
        if step > 0 and step % 8 == 0:
            log("scan: step %d, d=%.1fcm yaw=%+.1f batt=%.2fV"
                % (step, d, imu.get_yaw(), battery_voltage()))
    d = front_distance()
    if d < SCAN_MAX_CM:
        log("scan: hit on final look, d=%.1fcm" % d)
        return d
    log("scan: full sweep, nothing inside %.0fcm" % SCAN_MAX_CM)
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
    drivetrain.turn(-FINE_STEP_DEG, TURN_EFFORT, timeout=2)
    best_yaw = imu.get_yaw()
    best_d = front_distance()
    for _ in range(FINE_SPAN_STEPS - 1):
        drivetrain.turn(FINE_STEP_DEG, TURN_EFFORT, timeout=2)
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

def push_out_one_block(cycle):
    """Find a block, drive to contact, push it over the tape, return to
    center. True = keep looping, False = ring is clear."""
    log("--- cycle %d --- batt=%.2fV pose=(%.1f, %.1f)"
        % (cycle, battery_voltage(), POSE["x"], POSE["y"]))
    dist = rotate_until_block()
    if dist is None:
        return False
    dist = fine_aim(dist)

    def near():
        return "contact" if front_distance() <= CONTACT_CM else None
    outcome = drive_watching(APPROACH_EFFORT, near, max_cm=dist + 15.0,
                             hold_yaw=imu.get_yaw(), tag="approach")
    if outcome == "line":
        log("approach: tape before contact — echo was outside the ring")
        straight_tracked(-RETURN_CM, APPROACH_EFFORT, timeout=3)
        go_home()
        return True
    if outcome == "cap":
        log("approach: no contact where the echo was — rescanning")
        go_home()
        return True

    push_yaw = imu.get_yaw()
    log("push: contact made, pushing along yaw %+.1f" % push_yaw)
    outcome = drive_watching(PUSH_EFFORT, make_slip_checker(),
                             max_cm=RING_RADIUS_CM * 1.5,
                             hold_yaw=push_yaw, tag="push")
    if outcome == "slip":
        log("push: block slipped off — regrouping")
        go_home()
        return True

    if outcome == "line" and PUSH_OVER_CM > 0:
        straight_tracked(PUSH_OVER_CM, PUSH_EFFORT, timeout=2)
        log("push: block shoved %.1fcm past the line" % PUSH_OVER_CM)

    straight_tracked(-RETURN_CM, APPROACH_EFFORT, timeout=3)
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

def main():
    log_selftest()                   # RED LED at boot = log file broken
    log("")
    log("=========== BOOT: sumo_auto v3.7 =========== batt=%.2fV"
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

    POSE["x"] = POSE["y"] = 0.0

    pushes = 0
    start = time.ticks_ms()
    try:
        # (inside the try since v3.6 so a crash anywhere gets logged)
        log("run start: batt=%.2fV yaw=%+.1f" % (battery_voltage(),
                                                 imu.get_yaw()))
        if battery_voltage() < LOW_BATT_V:
            log("*** WARNING: battery LOW at launch (%.2fV, %d-cell) ***"
                % (battery_voltage(), BATT_CELLS))
        capture_floor_baseline()     # robot is at ring center = off tape

        while pushes < MAX_PUSHES:
            if time.ticks_diff(time.ticks_ms(), start) > MAX_RUNTIME_S * 1000:
                log("time cap %.0fs reached" % MAX_RUNTIME_S)
                break
            if not push_out_one_block(pushes + 1):
                break
            pushes += 1
    except Exception as e:
        log_exception(e)
        raise
    finally:
        drivetrain.stop()
        set_status(0, 255, 0)
        board.led_blink(4)
        log("DONE: %d push cycles, batt=%.2fV" % (pushes,
                                                  battery_voltage()))

main()

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
