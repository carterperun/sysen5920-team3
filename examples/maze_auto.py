"""
maze_auto.py — SYSEN 5920 Team 3
XRP Proving Ground: "The Maze" challenge (autonomous attempt)

Scoring context (Appendix A):
  Autonomous: 10 pts/square (max 180)   Semi-auto: 5 pts/square (max 90)
  Manual: 1.5 pts/square (max 27)       Bonus: up to +25 for finishing
  NOTE: per the RFP, a purely hard-coded route scores at the SEMI-AUTONOMOUS
  level. Sensor-driven navigation (MODE = "wall_follow") is our shot at the
  full autonomous multiplier.

Usage:
  1. Set MODE below ("wall_follow" or "waypoints").
  2. Measure the maze and set CELL_CM (grid square size) — and, if using
     waypoint mode, fill in ROUTE from the base-station survey.
  3. Upload & run. The onboard LED turns SOLID ON to show the program is
     alive, and the robot stays parked — place it pointing at the maze
     entrance, ball loaded. It does not need to start at the maze itself:
     after launch it drives forward in 50 cm hops until it sees a wall
     within 100 cm, and only then starts the maze algorithm.
  4. Connect the controller via PestoLink (https://pestol.ink). RGB status:
     orange = waiting for controller, blue = connected & armed.
  5. Press Button 1 on the controller to begin the autonomous run
     (the USER button on the XRP also works as a backup trigger).
     Per the RFP, starting via a gamepad button still counts as fully
     autonomous — as long as nobody touches the robot afterward.

Robot start: front of chassis flush with the entrance cell edge.
Positive turn() = LEFT (CCW). Negative = RIGHT (CW).
Drive efforts are capped low so the ball stays in its cradle.
"""

from XRPLib.defaults import *
from pestolink import PestoLinkAgent
import time

# ----------------------------- CONFIGURATION -----------------------------

MODE = "wall_follow"        # "wall_follow" (autonomous) | "waypoints" (semi-auto)

ROBOT_NAME = "T3amThr3"     # BLE name shown in PestoLink (8 chars max)
START_BUTTON = 1            # controller button that launches the run.
                            # PestoLink button indices are 0-based — if your
                            # controller's "Button 1" doesn't fire, try 0
                            # (verify in the PestoLink gamepad tester).

CELL_CM = 33.0              # maze grid square size (measured: ~13 in squares)
WALL_NEAR_CM = 18.0         # front reading closer than this = wall ahead
OPEN_CM = CELL_CM * 0.8     # front reading beyond this = passage is open

# Approach phase (runs before the maze algorithm): from the placement spot,
# drive forward in fixed increments until a wall shows up in front, THEN
# start solving. Lets you place the robot short of the maze itself.
APPROACH_STEP_CM = 50.0     # advance this much per hop while seeking the maze
WALL_SEEK_CM = 100.0        # a front reading closer than this = maze found
MAX_APPROACH_STEPS = 6      # safety cap (6 x 50 cm spans the whole arena)
DRIVE_EFFORT = 0.4          # gentle so we don't sling the ball out
TURN_EFFORT = 0.4
TURN_TOL_DEG = 3.0          # a turn "counts" once the IMU is within this
TURN_RETRIES = 3            # re-command an unfinished turn up to this many
                            # times, raising effort each retry (beats floor
                            # friction that stalls the stock turn PID)
MAX_RUNTIME_S = 150         # give up before eating the whole 12-min budget
MAX_CELLS = 40              # wall-follower step cap (also caps loops)

# Hard-coded route for MODE = "waypoints" — fill in after measuring maze.
# ("straight", cm) drives forward; ("turn", deg) rotates in place (+ = left).
ROUTE = [
    ("straight", CELL_CM * 2),
    ("turn", -90),
    ("straight", CELL_CM),
    ("turn", 90),
    ("straight", CELL_CM * 2),
    # ... extend to the exit ...
]

# ------------------------------- HELPERS ---------------------------------

def front_distance():
    """Median of 3 pings to reject the rangefinder's occasional glitches.
    Timeouts read as 65535, which safely counts as 'open'."""
    reads = []
    for _ in range(3):
        reads.append(rangefinder.distance())
        time.sleep(0.015)
    reads.sort()
    return reads[1]

def front_open():
    return front_distance() > OPEN_CM

def advance_cell():
    """Drive one grid cell. If we stall (clipped a wall), back off a touch
    so the next turn doesn't scrub against it. Returns True on clean move."""
    ok = drivetrain.straight(CELL_CM, DRIVE_EFFORT, timeout=4)
    if not ok:
        drivetrain.straight(-4, DRIVE_EFFORT, timeout=2)
    return ok

def normalize(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# The robot's intended heading, snapped to the maze grid (0/90/180/270
# relative to launch). All turns aim at THIS absolute target rather than
# "90 from wherever I stopped", so an undershot turn is corrected by the
# next command instead of compounding until the robot hits a wall.
GRID = {"target": 0.0}

def _settle_on_target():
    """Command the turn, then verify against the IMU and re-command with
    escalating effort until we're within TURN_TOL_DEG of the grid heading.
    The stock turn PID can stall short on high-friction floors and exit
    silently on timeout — this loop catches and finishes those turns."""
    for attempt in range(TURN_RETRIES):
        error = normalize(GRID["target"] - imu.get_yaw())
        if abs(error) < TURN_TOL_DEG:
            return True
        effort = min(0.9, TURN_EFFORT + 0.15 * attempt)
        drivetrain.turn(error, effort, timeout=4)
    return abs(normalize(GRID["target"] - imu.get_yaw())) < TURN_TOL_DEG

def turn_left():
    GRID["target"] = normalize(GRID["target"] + 90)
    _settle_on_target()

def turn_right():
    GRID["target"] = normalize(GRID["target"] - 90)
    _settle_on_target()

def set_status(r, g, b):
    try:
        board.set_rgb_led(r, g, b)
    except Exception:
        pass  # XRP Beta has no RGB LED

# --------------------------- AUTONOMOUS MODE -----------------------------

def approach_maze():
    """Drive forward in APPROACH_STEP_CM hops until a wall is seen within
    WALL_SEEK_CM, then hand off to the maze algorithm. Capped so we can't
    drive across the whole arena chasing a wall that isn't there."""
    steps = 0
    while steps < MAX_APPROACH_STEPS:
        if front_distance() < WALL_SEEK_CM:
            print("Maze found after", steps, "approach hops")
            return True
        drivetrain.straight(APPROACH_STEP_CM, DRIVE_EFFORT, timeout=5)
        steps += 1
    print("No wall found in", steps, "hops - starting maze algorithm anyway")
    return False

def wall_follow():
    """Left-hand-rule maze solve with a single front rangefinder.

    At each cell we physically scan by rotating: prefer LEFT, then
    STRAIGHT, then RIGHT, then dead-end turnaround. Fully sensor-driven —
    no pre-knowledge of the maze — so it qualifies as autonomous.
    """
    cells = 0
    GRID["target"] = imu.get_yaw()   # current facing = first grid heading
    start = time.ticks_ms()
    while cells < MAX_CELLS:
        if time.ticks_diff(time.ticks_ms(), start) > MAX_RUNTIME_S * 1000:
            break

        # 1) Check LEFT first (left-hand rule)
        turn_left()
        if front_open():
            if advance_cell():
                cells += 1
                continue
        # 2) LEFT blocked -> face forward again and check STRAIGHT
        turn_right()
        if front_open():
            if advance_cell():
                cells += 1
                continue
        # 3) STRAIGHT blocked -> check RIGHT
        turn_right()
        if front_open():
            if advance_cell():
                cells += 1
                continue
        # 4) Dead end -> finish the about-face and go back
        turn_right()
        if advance_cell():
            cells += 1
    return cells

# ---------------------------- WAYPOINT MODE ------------------------------

def run_waypoints():
    """Dead-reckoned route from the ROUTE table (semi-autonomous score)."""
    for step in ROUTE:
        kind, value = step
        if kind == "straight":
            drivetrain.straight(value, DRIVE_EFFORT, timeout=6)
        elif kind == "turn":
            drivetrain.turn(value, TURN_EFFORT, timeout=4)

# --------------------------------- MAIN ----------------------------------

def wait_for_start(pestolink):
    """Idle (motors parked) until Button 1 on the controller is pressed.
    The USER button on the XRP board works as a backup trigger.
    RGB shows orange until a controller connects, blue once armed."""
    connected = False
    while True:
        if pestolink.is_connected():
            if not connected:
                connected = True
                set_status(0, 0, 255)        # blue = controller connected, armed
            if pestolink.get_button(START_BUTTON):
                break
        else:
            if connected:
                connected = False
                set_status(255, 120, 0)      # orange = waiting for controller
        if board.is_button_pressed():        # backup: USER button on the board
            break
        time.sleep(0.02)

    # wait for release so a held button can't disturb the run start
    while pestolink.get_button(START_BUTTON) or board.is_button_pressed():
        time.sleep(0.02)

def main():
    pestolink = PestoLinkAgent(ROBOT_NAME)

    board.led_on()                   # solid LED = program is running on the robot
    set_status(255, 120, 0)          # orange = waiting for controller
    drivetrain.stop()                # stay parked while the robot is placed

    wait_for_start(pestolink)        # blocks until Button 1 (or USER button)

    set_status(0, 255, 255)          # cyan = run in progress, hands off!
    time.sleep(0.5)                  # small buffer to step clear of the robot

    try:
        if MODE == "wall_follow":
            approach_maze()              # close the gap to the maze first
            cells = wall_follow()
            print("Wall-follower advanced", cells, "cells")
        else:
            run_waypoints()
    finally:
        drivetrain.stop()
        set_status(0, 255, 0)        # green = done
        board.led_blink(4)           # fast blink = attempt finished

main()
